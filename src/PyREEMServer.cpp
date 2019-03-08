/*
 *  PyREEMServer.cpp
 *  PyREEMServer
 *
 *  Created by Xun Wang on 09/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#include <sys/types.h>
#include <sys/stat.h>

#include "PyREEMServer.h"
#include "PythonServer.h"
#include "REEMProxyManager.h"
#include "AppConfigManager.h"
#include "PyREEMModule.h"
#include "VideoObject.h"
#include "AudioObject.h"
#include "AudioFeedbackStream.h"
#include "VideoToWebBridge.h"

PYRIDE_LOGGING_DECLARE( "pyride_reem.log" );

namespace pyride {
static const float kHFOV_Wide = 90.0;
static const float kHFOV_Narrow = 55.0;

PyREEMServer::PyREEMServer() :
  isRunning_( true )
{
  hcNodeHandle_ = new NodeHandle();
}

PyREEMServer::~PyREEMServer()
{
  delete hcNodeHandle_;
}

bool PyREEMServer::init()
{
  PYRIDE_LOGGING_INIT;

  // setup private parameter image_transport to compressed
  NodeHandle priNh( "~" );
  std::string filename;
  std::string scriptdir;
  bool useOptionNodes = false;
  bool useMoveIt = false;
  
  priNh.param<bool>( "use_optional_nodes", useOptionNodes, false );
  priNh.param<bool>( "use_moveit", useMoveIt, false );
  priNh.param<std::string>( "config_file", filename, "pyrideconfig.xml" );
  priNh.param<std::string>( "script_dir", scriptdir, "scripts" );
  AppConfigManager::instance()->loadConfigFromFile( filename.c_str() );

  if (!initVideoDevices()) {
    ERROR_MSG( "Unable to initialise any active robot camera.\n" );
  }

  if (!initAudioDevices()) {
    ERROR_MSG( "Unable to initialise any active robot audio device.\n" );
  }

  REEMProxyManager::instance()->initWithNodeHandle( hcNodeHandle_, useOptionNodes, useMoveIt );
  ServerDataProcessor::instance()->init( activeVideoDevices_, activeAudioDevices_ );
  ServerDataProcessor::instance()->addCommandHandler( this );
  ServerDataProcessor::instance()->setClientID( AppConfigManager::instance()->clientID() );
  ServerDataProcessor::instance()->setDefaultRobotInfo( REEM, AppConfigManager::instance()->startPosition(),
      (RobotCapability)(MOBILITY|VIDEO_FEEBACK|AUDIO_FEEBACK|MANIPULATION) );
  
  PythonServer::instance()->init( AppConfigManager::instance()->enablePythonConsole(),
      PyREEMModule::instance(), scriptdir.c_str(), "/home/pal/local" );
  ServerDataProcessor::instance()->discoverConsoles();
  VideoToWebBridge::instance()->setPyModuleExtension( PyREEMModule::instance() );
  AudioFeedbackStream::instance()->initWithNode( hcNodeHandle_, PyREEMModule::instance() );

  ros::SubscribeOptions sopts = ros::SubscribeOptions::create<pyride_common_msgs::NodeStatus>( "pyride/node_status",
        10, boost::bind( &PyREEMServer::nodeStatusCB, this, _1 ), ros::VoidPtr(), &nodeStatusQueue_ );

  nodeStatusSub_ = hcNodeHandle_->subscribe( sopts );

  nodeStatusThread_ = new ros::AsyncSpinner( 1, &nodeStatusQueue_ );
  nodeStatusThread_->start();

  return true;
}

void PyREEMServer::stopProcess()
{
  isRunning_ = false;
}

void PyREEMServer::continueProcessing()
{
  ros::Rate publish_rate( kPublishFreq );

  while (isRunning_) {
    ros::spinOnce();

    REEMProxyManager::instance()->publishCommands();

    publish_rate.sleep();
  }
}

void PyREEMServer::fini()
{
  INFO_MSG( "Disconnect to the controllers gracefully...\n" );

  this->notifySystemShutdown();

  // finalise controllers
  finiVideoDevices();
  finiAudioDevices();

  nodeStatusSub_.shutdown();

  nodeStatusThread_->stop();
  delete nodeStatusThread_;
  nodeStatusThread_ = NULL;

  REEMProxyManager::instance()->fini();
  AppConfigManager::instance()->fini();
  ServerDataProcessor::instance()->fini();
}

bool PyREEMServer::executeRemoteCommand( PyRideExtendedCommand command, int & retVal,
                                           const unsigned char * optionalData,
                                           const int optionalDataLength )
{
  // implement the routine to handle commands defined in PyRideExtendedCommand
  // in PyRideCommon.h
  // for example:
  bool status = true;
  retVal = 0;
  switch (command) {
    case SPEAK:
    {
      float volume = *((float *)optionalData);
      //DEBUG_MSG( "received volume %f\n", volume );
      char * text = (char *)optionalData + sizeof( float );
      REEMProxyManager::instance()->sayWithVolume( std::string( text, optionalDataLength - sizeof( float ) ), volume );
    }
      break;
    case HEAD_MOVE_TO: // position
    {
      float yaw = *((float *)optionalData);
      float pitch = *((float *)optionalData+1);
      
      yaw = kHFOV_Wide * yaw * kDegreeToRAD;
      pitch = kHFOV_Wide * pitch * kDegreeToRAD;

      status = REEMProxyManager::instance()->moveHeadTo( yaw, pitch, true );
    }
      break;
    case UPDATE_HEAD_POSE: // velocity vector
    {
      float newHeadYaw = *((float *)optionalData);
      float newHeadPitch = *((float *)optionalData+1);
      REEMProxyManager::instance()->updateHeadPos( newHeadYaw, newHeadPitch );
    }
      break;
    case BODY_MOVE_TO: // position
    {
      unsigned char * dataPtr = (unsigned char *)optionalData;
      RobotPose newPose;
      memcpy( &newPose, dataPtr, sizeof( RobotPose ) );
      dataPtr += sizeof( RobotPose );
      float bestTime = *((float *)dataPtr);
      status = REEMProxyManager::instance()->moveBodyTo( newPose, bestTime );
    }
      break;
    case UPDATE_BODY_POSE: // velocity vector
    {
      unsigned char * dataPtr = (unsigned char *)optionalData;
      RobotPose newPose;
      memcpy( &newPose, dataPtr, sizeof( RobotPose ) );
      REEMProxyManager::instance()->updateBodyPose( newPose );
    }
      break;
    case UPDATE_AUDIO_SETTINGS:
    {
      unsigned char * dataPtr = (unsigned char *)optionalData;
      int volume;
      memcpy( &volume, dataPtr, sizeof( int ) );
      REEMProxyManager::instance()->setAudioVolume( volume );
    }
      break;
    case VIDEO_FEEDBACK:
    {
      bool ison = (bool)optionalData[0];
      if (ison) { // only the client with the exclusive control can do video feedback
        VideoToWebBridge::instance()->start();
        retVal = 1;
      }
      else {
        VideoToWebBridge::instance()->stop();
        retVal = 0;
      }
    }
      break;
    case AUDIO_FEEDBACK:
    {
      bool ison = (bool)optionalData[0];
      if (ison) { // only the client with the exclusive control can do video feedback
        AudioFeedbackStream::instance()->addClient();
        retVal = 1;
      }
      else {
        AudioFeedbackStream::instance()->removeClient();
        retVal = 0;
      }
    }
      break;
    default:
      status = false;
      break;
  }
  return status;
}

void PyREEMServer::cancelCurrentOperation()
{
  REEMProxyManager::instance()->sayWithVolume( "Emergency Stop!" );
}

// helper function
bool PyREEMServer::initVideoDevices()
{
  const DeviceInfoList * devlist = AppConfigManager::instance()->deviceInfoList();
  for (size_t i = 0; i < devlist->size(); i++) {
    DeviceInfo * deviceInfo = devlist->at( i );
    VideoObject * activeVideoObj = new VideoObject( *deviceInfo );
    if (activeVideoObj->initDevice()) {
      activeVideoDevices_.push_back( activeVideoObj );
    }
    else {
      ERROR_MSG( "Failed to activate a video device.\n" );
      delete activeVideoObj;
    }
  }
  return (activeVideoDevices_.size() != 0);
}

void PyREEMServer::finiVideoDevices()
{
  VideoDevice * videoObj = NULL;
  
  for (size_t i = 0; i < activeVideoDevices_.size(); i++) {
    videoObj = activeVideoDevices_.at( i );
    videoObj->finiDevice();
    delete videoObj;
  }
  activeVideoDevices_.clear();
}

bool PyREEMServer::initAudioDevices()
{
  AudioDevice * activeAudioObj = new AudioObject();
  if (activeAudioObj->initDevice()) {
    activeAudioDevices_.push_back( activeAudioObj );
  }
  else {
    ERROR_MSG( "Failed to activate an audio device.\n" );
    delete activeAudioObj;
  }

  return (activeVideoDevices_.size() != 0);
}

void PyREEMServer::finiAudioDevices()
{
  AudioDevice * audioObj = NULL;

  for (size_t i = 0; i < activeAudioDevices_.size(); i++) {
    audioObj = activeAudioDevices_.at( i );
    audioObj->finiDevice();
    delete audioObj;
  }
  activeAudioDevices_.clear();
}

void PyREEMServer::notifySystemShutdown()
{
  INFO_MSG( "PyREEMServer is shutting down..\n" );
  
  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyREEMModule::instance()->invokeCallback( "onSystemShutdown", arg );
  
  PyGILState_Release( gstate );
}

/*! \typedef onNodeStatusUpdate( data )
 *  \memberof PyREEM.
 *  \brief Callback function invoked when an external ROS node dispatch status update message
 *   to PyRIDE on pyride_reem/node_status topic.
 *  \param dictionary data. node status message in the format of {'node', 'timestamp', 'priority', 'message' }.
 *  \return None.
 */
void PyREEMServer::nodeStatusCB( const pyride_common_msgs::NodeStatusConstPtr & msg )
{
  if (msg->for_console) { // reformat the string using colon separated format and pass directly to console
    stringstream ss;
    ss << msg->header.stamp << msg->node_id << ":" << msg->priority << ":" << msg->status_text;
    ServerDataProcessor::instance()->updateOperationalStatus( CUSTOM_STATE,
                                                                ss.str().c_str(),
                                                                (int)ss.str().length() );
    return;
  }

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * retObj = PyDict_New();

  PyObject * elemObj = PyString_FromString( msg->node_id.c_str() );
  PyDict_SetItemString( retObj, "node", elemObj );
  Py_DECREF( elemObj );

  elemObj = PyFloat_FromDouble( (double)msg->header.stamp.sec + (double)msg->header.stamp.nsec / 1E9 );
  PyDict_SetItemString( retObj, "timestamp", elemObj );
  Py_DECREF( elemObj );

  elemObj = PyInt_FromLong( msg->priority );
  PyDict_SetItemString( retObj, "priority", elemObj );
  Py_DECREF( elemObj );

  elemObj = PyString_FromString( msg->status_text.c_str() );
  PyDict_SetItemString( retObj, "message", elemObj );
  Py_DECREF( elemObj );

  PyObject * arg = Py_BuildValue( "(O)", retObj );

  PyREEMModule::instance()->invokeCallback( "onNodeStatusUpdate", arg );
  
  Py_DECREF( arg );
  Py_DECREF( retObj );

  PyGILState_Release( gstate );    
}

} // namespace pyride
