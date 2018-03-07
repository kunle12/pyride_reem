/*
 *  REEMProxyManager.cpp
 *  PyREEMServer
 *
 *  Created by Xun Wang on 24/05/2016.
 *  Copyright 2016 Galaxy Network. All rights reserved.
 *
 */
#include "REEMProxyManager.h"
#include "PyREEMModule.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pal_device_msgs/TimedColourEffect.h>
#include <pal_device_msgs/TimedFadeEffect.h>
#include <pal_device_msgs/CancelEffect.h>
#include <pal_device_msgs/LedGroup.h>
#include <pal_detection_msgs/Recognizer.h>
#include <pal_detection_msgs/SetDatabase.h>
#include <pal_detection_msgs/StartEnrollment.h>
#include <pal_detection_msgs/StopEnrollment.h>
#include <pal_navigation_msgs/GetMapConfiguration.h>
#include <pal_navigation_msgs/POI.h>
#include <pal_web_msgs/WebGoTo.h>
#include <pyride_common_msgs/NodeMessage.h>

namespace pyride {

static const float kMaxWalkSpeed = 0.8;
static const float kYawRate = 0.7;  // ~ 45 degree
static const float kHeadYawRate = 1.5;
static const float kHeadPitchRate = 1.5;
static const float kMotionCommandGapTolerance = 1.2 / (float)kMotionCommandFreq;
static const double kDT = 1.0/double( kPublishFreq );
static const double kHorizon = 5.0 * kDT;

static const double kMaxHeadTilt = 0.78;
static const double kMinHeadTilt = -0.26;
static const double kMaxHeadPan = 1.3;
static const double kMaxTorsoTilt = 0.61;
static const double kMinTorsoTilt = -0.26;
static const double kMaxTorsoPan = 1.3;

static const char *kREEMTFFrameList[] = { "map", "odom", "base_footprint", "base_link",
  "torso_1_link", "torso_2_link", "head_sonar_16_link", "head_1_link", "head_2_link",
  "back_camera_link", "back_camera_optical_frame", "arm_left_1_link", "arm_left_2_link"
  "arm_left_3_link", "arm_left_4_link","arm_left_5_link", "arm_left_6_link", "arm_left_7_link",
  "arm_right_1_link", "arm_right_2_link", "arm_right_3_link", "arm_right_4_link",
  "arm_right_5_link","arm_right_6_link","arm_right_7_link",
  "arm_right_tool_link", "arm_left_tool_link", "hand_right_grasping_frame",
  "hand_left_grasping_frame", "stereo_optical_frame", "stereo_link",
  "stereo_gazebo_left_camera_optical_frame", "stereo_gazebo_right_camera_optical_frame",
  "base_torso_laser_link", "base_laser_link",
  NULL };

static const int kREEMTFFrameListSize = sizeof( kREEMTFFrameList ) / sizeof( kREEMTFFrameList[0] );

// helper function
inline double REEMProxyManager::clamp( double val, double max )
{
  if (val > max) {
    return max;
  }
  else if (val < -max) {
    return -max;
  }
  else {
    return val;
  }
}

inline double REEMProxyManager::max( double val1, double val2 )
{
  return (val1 >= val2 ? val1 : val2);
}
          
REEMProxyManager * REEMProxyManager::s_pREEMProxyManager = NULL;

REEMProxyManager::REEMProxyManager() :
  rawBaseScanSub_( NULL ),
  rawTiltScanSub_( NULL ),
  baseScanSub_( NULL ),
  tiltScanSub_( NULL ),
  baseScanNotifier_( NULL ),
  tiltScanNotifier_( NULL ),
  torsoSonarSub_( NULL ),
  faceDetectSub_( NULL ),
  legDetectSub_( NULL ),
  htObjStatusSub_( NULL ),
  htObjUpdateSub_( NULL ),
  bodyCtrlWithOdmetry_( false ),
  bodyCtrlWithNavigation_( false ),
  torsoCtrl_( false ),
  headCtrlWithTrajActionClient_( false ),
  headCtrlWithActionClient_( false ),
  defaultMotionCtrl_( false ),
  lHandCtrl_( false ),
  rHandCtrl_( false ),
  lArmCtrl_( false ),
  rArmCtrl_( false ),
  speechCtrl_( false ),
  palFaceDatabaseInit_( false ),
  audioVolume_( 0 ),
  powerVoltage_( -1 ),
  lArmActionTimeout_( 20 ),
  rArmActionTimeout_( 20 ),
  lHandActionTimeout_( 20 ),
  rHandActionTimeout_( 20 ),
  bodyActionTimeout_( 100 ),
  legDetectDistance_( 1.0 ),
  soundClient_( NULL ),
  headClient_( NULL ),
  torsoClient_( NULL ),
  phClient_( NULL ),
  lhandClient_( NULL ),
  rhandClient_( NULL ),
  rarmGroup_( NULL ),
  larmGroup_( NULL ),
  mlacClient_( NULL ),
  mracClient_( NULL ),
  moveBaseClient_( NULL ),
  gotoPOIClient_( NULL ),
  playMotionClient_( NULL ),
  playAudioClient_( NULL ),
  recordAudioClient_( NULL ),
  faceEnrolmentClient_( NULL ),
  batChargingState_( UNKNOWN ),
  batCapacity_( 100.0 ),
  lowPowerThreshold_( 0 ), // # no active power monitoring
  batTimeRemain_( Duration( 1.0 ) )
{
}

REEMProxyManager::~REEMProxyManager()
{
}
  
REEMProxyManager * REEMProxyManager::instance()
{
  if (!s_pREEMProxyManager) {
    s_pREEMProxyManager = new REEMProxyManager();
  }
  return s_pREEMProxyManager;
}

void REEMProxyManager::initWithNodeHandle( NodeHandle * nodeHandle, bool useOptionNodes, bool useMoveIt )
{
  mCtrlNode_ = nodeHandle;

  mPub_ = mCtrlNode_->advertise<geometry_msgs::Twist>( "cmd_vel", 5 );
  pPub_ = mCtrlNode_->advertise<geometry_msgs::PoseWithCovarianceStamped>( "initialpose", 1 );
  hPub_ = mCtrlNode_->advertise<trajectory_msgs::JointTrajectory>( "head_vel", 1 );
  wPub_ = mCtrlNode_->advertise<pal_web_msgs::WebGoTo>( "web", 1 );
  aPub_ = mCtrlNode_->advertise<std_msgs::Int8>( "audio_file_player/set_volume", 1 );
  cPub_ = mCtrlNode_->advertise<pal_control_msgs::ActuatorCurrentLimit>( "current_limit", 1 );
  bPub_ = mCtrlNode_->advertise<pyride_common_msgs::NodeMessage>( "pyride/node_message", 5 );

  powerSub_ = mCtrlNode_->subscribe( "diagnostics_agg", 1, &REEMProxyManager::powerStateDataCB, this );
  volumeSub_ = mCtrlNode_->subscribe( "audio_file_player/get_volume", 1, &REEMProxyManager::audioVolumeDataCB, this );

  this->initMotorStiffnessValue();
  ros::SubscribeOptions sopts = ros::SubscribeOptions::create<sensor_msgs::JointState>( "joint_states",
        1, boost::bind( &REEMProxyManager::jointStateDataCB, this, _1 ), ros::VoidPtr(), &jointDataQueue_ );

  jointSub_ = mCtrlNode_->subscribe( sopts );

  jointDataThread_ = new ros::AsyncSpinner( 1, &jointDataQueue_ );
  jointDataThread_->start();

  ros::SubscribeOptions sopts2 = ros::SubscribeOptions::create<sb04_power_board::PowerBoard>( "power_board",
        1, boost::bind( &REEMProxyManager::voltageStateDataCB, this, _1 ), ros::VoidPtr(), &powerBoardDataQueue_ );

  voltageSub_ = mCtrlNode_->subscribe( sopts2 );

  powerBoardDataThread_ = new ros::AsyncSpinner( 1, &powerBoardDataQueue_ );
  powerBoardDataThread_->start();

  mCmd_.linear.x = mCmd_.linear.y = mCmd_.angular.z = 0;
  headPitchRate_ = headYawRate_ = 0.0;
  targetYaw_ = targetPitch_ = 0.0;

  int trials = 0;
  phClient_ = new PointHeadClient( "head_controller/point_head_action", true );
  while (!phClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the point head action server to come up." );
    trials++;
  }
  if (!phClient_->isServerConnected()) {
    ROS_INFO( "Point head action server is down." );
    delete phClient_;
    phClient_ = NULL;
  }

  trials = 0;
  headClient_ = new FollowTrajectoryClient( "head_controller/follow_joint_trajectory", true );

  while (!headClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the head action server to come up." );
    trials++;
  }
  if (!headClient_->isServerConnected()) {
    ROS_INFO( "Head action server is down." );
    delete headClient_;
    headClient_ = NULL;
  }

  trials = 0;
  soundClient_ = new TTSClient( "/tts", true );
  while (!soundClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the TTS action server to come up." );
    trials++;
  }
  if (!soundClient_->isServerConnected()) {
    ROS_INFO( "TTS server is down." );
    delete soundClient_;
    soundClient_ = NULL;
  }

  trials = 0;
  torsoClient_ = new FollowTrajectoryClient( "torso_controller/follow_joint_trajectory", true );
  
  while (!torsoClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the torso action server to come up." );
    trials++;
  }
  if (!torsoClient_->isServerConnected()) {
    ROS_INFO( "Torso action server is down." );
    delete torsoClient_;
    torsoClient_ = NULL;
  }

  trials = 0;
  lhandClient_ = new FollowTrajectoryClient( "left_hand_controller/follow_joint_trajectory", true );
  while (!lhandClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for left hand action server to come up." );
    trials++;
  }
  if (!lhandClient_->isServerConnected()) {
    ROS_INFO( "Left hand action server is down." );
    delete lhandClient_;
    lhandClient_ = NULL;
  }
  
  trials = 0;
  rhandClient_ = new FollowTrajectoryClient( "right_hand_controller/follow_joint_trajectory", true );
  while (!rhandClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for right hand action server to come up." );
    trials++;
  }
  if (!rhandClient_->isServerConnected()) {
    ROS_INFO( "Right hand action server is down." );
    delete rhandClient_;
    rhandClient_ = NULL;
  }

  trials = 0;
  mlacClient_ = new FollowTrajectoryClient( "left_arm_controller/follow_joint_trajectory", true );
  while (!mlacClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for move left arm by joint action server to come up." );
    trials++;
  }
  if (!mlacClient_->isServerConnected()) {
    ROS_INFO( "Move left arm by joint action server is down." );
    delete mlacClient_;
    mlacClient_ = NULL;
  }

  trials = 0;
  mracClient_ = new FollowTrajectoryClient( "right_arm_controller/follow_joint_trajectory", true );
  while (!mracClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for move right arm by joint action server to come up." );
    trials++;
  }
  if (!mracClient_->isServerConnected()) {
    ROS_INFO( "Move right arm by joint action server is down." );
    delete mracClient_;
    mracClient_ = NULL;
  }

  trials = 0;
  playMotionClient_ = new PlayMotionClient( "play_motion", true );

  while (!playMotionClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the play motion action server to come up." );
    trials++;
  }
  if (!playMotionClient_->isServerConnected()) {
    ROS_INFO( "Play motion action server is down." );
    delete playMotionClient_;
    playMotionClient_ = NULL;
  }

  trials = 0;
  gotoPOIClient_ = new GotoPOIClient( "poi_navigation_server/go_to_poi", true );
  while (!gotoPOIClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for POI navigation action server to come up." );
    trials++;
  }
  if (!gotoPOIClient_->isServerConnected()) {
    ROS_INFO( "POI navigation action server is down." );
    delete gotoPOIClient_;
    gotoPOIClient_ = NULL;
  }

  trials = 0;
  moveBaseClient_ = new MoveBaseClient( "move_base", true );
  while (!moveBaseClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for move base server to come up." );
    trials++;
  }
  if (!moveBaseClient_->isServerConnected()) {
    ROS_INFO( "Move base action server is down." );
    delete moveBaseClient_;
    moveBaseClient_ = NULL;
  }

  trials = 0;
  playAudioClient_ = new PlayAudioClient( "/audio_file_player", true );

  while (!playAudioClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the audio player action server to come up." );
    trials++;
  }
  if (!playAudioClient_->isServerConnected()) {
    ROS_INFO( "Audio player action server is down." );
    delete playAudioClient_;
    playAudioClient_ = NULL;
  }

  trials = 0;
  recordAudioClient_ = new RecordAudioClient( "/audio_stream/record_audio", true );

  while (!recordAudioClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the audio record action server to come up." );
    trials++;
  }
  if (!recordAudioClient_->isServerConnected()) {
    ROS_INFO( "Audio record action server is down." );
    delete recordAudioClient_;
    recordAudioClient_ = NULL;
  }

  trials = 0;
  faceEnrolmentClient_ = new ObjectEnrolmentClient( "/face_server/face_enrolment", true );

  while (!faceEnrolmentClient_->waitForServer( ros::Duration( 5.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the face server action server to come up." );
    trials++;
  }
  if (!faceEnrolmentClient_->isServerConnected()) {
    ROS_INFO( "face server action server is down." );
    delete faceEnrolmentClient_;
    faceEnrolmentClient_ = NULL;
  }

  if (useMoveIt) {
    ROS_INFO( "Loading MoveIt service..." );
    try {
      rarmGroup_ = new moveit::planning_interface::MoveGroup( "right_arm",
          boost::shared_ptr<tf::Transformer>(), ros::Duration( 5, 0 ) );
      larmGroup_ = new moveit::planning_interface::MoveGroup( "left_arm",
        boost::shared_ptr<tf::Transformer>(), ros::Duration( 5, 0 ) );
    }
    catch (...) {
      try {
        delete rarmGroup_;
        rarmGroup_ = NULL;
        delete larmGroup_;
        larmGroup_ = NULL;
      }
      catch (...) {}
      ROS_WARN( "Moveit server is down." );
    }
    if (rarmGroup_) {
      colObjPub_ = mCtrlNode_->advertise<moveit_msgs::CollisionObject>( "collision_object", 2 );
      ROS_INFO("MoveIt planning reference frame: %s", rarmGroup_->getPlanningFrame().c_str());
      ROS_INFO("Right arm end effector %s, left arm end effector %s.",
          rarmGroup_->getEndEffectorLink().c_str(), larmGroup_->getEndEffectorLink().c_str() );
    }
  }

  ledColourClient_ = mCtrlNode_->serviceClient<pal_device_msgs::TimedColourEffect>( "/ledManager/TimedColourEffect" );

  if (!ledColourClient_.exists()) {
    ROS_INFO( "No led colour effect service is available." );
  }

  ledPulseClient_ = mCtrlNode_->serviceClient<pal_device_msgs::TimedFadeEffect>( "/ledManager/TimedFadeEffect" );

  if (!ledPulseClient_.exists()) {
    ROS_INFO( "No led colour fade effect service is available." );
  }

  cancelLedClient_ = mCtrlNode_->serviceClient<pal_device_msgs::CancelEffect>( "/ledManager/CancelEffect" );

  if (!cancelLedClient_.exists()) {
    ROS_INFO( "No led colour effect cancelling service is available." );
  }

  ServiceClient palFaceDatabaseClient = mCtrlNode_->serviceClient<pal_detection_msgs::SetDatabase>( "/pal_face/set_database" );

  if (palFaceDatabaseClient.exists()) {
    pal_detection_msgs::SetDatabase srvMsg;
    srvMsg.request.databaseName = "face_data";
    srvMsg.request.purgeAll = false;
    palFaceDatabaseInit_ = palFaceDatabaseClient.call( srvMsg );
    //ROS_INFO( "Pal face database is set to 'face data'" );
  }

  palFaceEnablerClient_ = mCtrlNode_->serviceClient<pal_detection_msgs::Recognizer>( "/pal_face/recognizer" );

  if (!palFaceEnablerClient_.exists()) {
    ROS_INFO( "No Pal face recognizer service is available." );
  }

  palFaceEnrolStartClient_ = mCtrlNode_->serviceClient<pal_detection_msgs::StartEnrollment>( "/pal_face/start_enrollment" );

  if (!palFaceEnrolStartClient_.exists()) {
    ROS_INFO( "No Pal face start enrollment service is available." );
  }

  palFaceEnrolStopClient_ = mCtrlNode_->serviceClient<pal_detection_msgs::StopEnrollment>( "/pal_face/stop_enrollment" );

  if (!palFaceEnrolStopClient_.exists()) {
    ROS_INFO( "No Pal face stop enrollment service is available." );
  }

  mapConfigClient_ = mCtrlNode_->serviceClient<pal_navigation_msgs::GetMapConfiguration>( "/getMapConfiguration" );

  if (!mapConfigClient_.exists()) {
    ROS_INFO( "No Pal map configuration service is available." );
  }

doneInit:
  this->getHeadPos( reqHeadYaw_, reqHeadPitch_ );
  this->getTorsoPos( reqTorsoYaw_, reqTorsoPitch_ );

  ROS_INFO( "REEM PyRIDE is fully initialised." );
}

void REEMProxyManager::fini()
{
  if (rarmGroup_) {
    rarmGroup_->stop();
    rarmGroup_->clearPoseTargets();
    delete rarmGroup_;
    rarmGroup_ = NULL;
  }
  if (larmGroup_) {
    larmGroup_->stop();
    larmGroup_->clearPoseTargets();
    delete larmGroup_;
    larmGroup_ = NULL;
  }
  if (phClient_) {
    delete phClient_;
    phClient_ = NULL;
  }
  if (headClient_) {
    delete headClient_;
    headClient_ = NULL;
  }
  if (lhandClient_) {
    delete lhandClient_;
    lhandClient_ = NULL;
  }
  if (rhandClient_) {
    delete rhandClient_;
    rhandClient_ = NULL;
  }
  if (mlacClient_) {
    delete mlacClient_;
    mlacClient_ = NULL;
  }
  if (mracClient_) {
    delete mracClient_;
    mracClient_ = NULL;
  }
  if (soundClient_) {
    delete soundClient_;
    soundClient_ = NULL;
  }
  if (moveBaseClient_) {
    delete moveBaseClient_;
    moveBaseClient_ = NULL;
  }
  if (gotoPOIClient_) {
    delete gotoPOIClient_;
    gotoPOIClient_ = NULL;
  }
  if (playMotionClient_) {
    delete playMotionClient_;
    playMotionClient_ = NULL;
  }
  if (playAudioClient_) {
    delete playAudioClient_;
    playAudioClient_ = NULL;
  }
  if (recordAudioClient_) {
    delete recordAudioClient_;
    recordAudioClient_ = NULL;
  }
  if (faceEnrolmentClient_) {
    delete faceEnrolmentClient_;
    faceEnrolmentClient_ = NULL;
  }
  jointSub_.shutdown();
  powerSub_.shutdown();
  volumeSub_.shutdown();

  deregisterForHumanData();
  deregisterForBaseScanData();
  deregisterForTiltScanData();
  deregisterForPalFaceData();
  deregisterForLegData();
  deregisterForSonarData();

  powerBoardDataThread_->stop();
  delete powerBoardDataThread_;
  powerBoardDataThread_ = NULL;

  jointDataThread_->stop();
  delete jointDataThread_;
  jointDataThread_ = NULL;
}

/** @name Action Callback Functions
 *
 */
/**@{*/
/*! \typedef onHeadActionSuccess()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.moveHeadTo or PyREEM.moveHeadWithJointTrajectory or PyREEM.pointHeadTo method call is successful.
 *  \return None.
 */
/*! \typedef onHeadActionFailed()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.moveHeadTo or PyREEM.moveHeadWithJointTrajectory or PyREEM.pointHeadTo method call is failed.
 *  \return None.
 */
void REEMProxyManager::doneHeadAction( const actionlib::SimpleClientGoalState & state,
            const PointHeadResultConstPtr & result )
{
  headCtrlWithActionClient_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onHeadActionSuccess", NULL );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onHeadActionFailed", NULL );
  }
  PyGILState_Release( gstate );
  
  ROS_INFO("Head action finished in state [%s]", state.toString().c_str());
}

void REEMProxyManager::doneHeadTrajAction( const actionlib::SimpleClientGoalState & state,
            const FollowJointTrajectoryResultConstPtr & result )
{
  this->getHeadPos( reqHeadYaw_, reqHeadPitch_ );

  headCtrlWithTrajActionClient_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onHeadActionSuccess", NULL );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onHeadActionFailed", NULL );
  }
  PyGILState_Release( gstate );

  ROS_INFO("Head traj action finished in state [%s]", state.toString().c_str());
}

/*! \typedef onMoveArmActionSuccess(is_left_arm)
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.moveArmWithJointPos or PyREEM.moveArmWithJointTrajectory method call is successful.
 *  \param bool is_left_arm. True = left arm; False = right arm.
 *  \return None.
 */
/*! \typedef onMoveArmActionFailed(is_left_arm)
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.moveArmWithJointPos or PyREEM.moveArmWithJointTrajectory method call is failed.
 *  \param bool is_left_arm. True = left arm; False = right arm.
 *  \return None.
 */
void REEMProxyManager::doneMoveLArmAction( const actionlib::SimpleClientGoalState & state,
                                        const FollowJointTrajectoryResultConstPtr & result)
{
  lArmCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * arg = Py_BuildValue( "(O)", Py_True );
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onMoveArmActionSuccess", arg );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onMoveArmActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );

  ROS_INFO("move arm action finished in state [%s]", state.toString().c_str());
}

void REEMProxyManager::doneMoveRArmAction( const actionlib::SimpleClientGoalState & state,
                                         const FollowJointTrajectoryResultConstPtr & result)
{
  rArmCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(O)", Py_False );

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onMoveArmActionSuccess", arg );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onMoveArmActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO("move arm action finished in state [%s]", state.toString().c_str());
}

/*! \typedef onMoveTorsoSuccess()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.moveTorso method call is successful.
 *  \return None.
 */
/*! \typedef onMoveTorsoFailed()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.moveTorso method call is failed.
 *  \return None.
 */
void REEMProxyManager::doneTorsoAction( const actionlib::SimpleClientGoalState & state,
                                      const FollowJointTrajectoryResultConstPtr & result )
{
  torsoCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onMoveTorsoSuccess", NULL );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onMoveTorsoFailed", NULL );
  }
  
  PyGILState_Release( gstate );
  
  ROS_INFO( "Torso action finished in state [%s]", state.toString().c_str());
}

/*! \typedef onNavigateBodySuccess()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.navigateBodyTo method call is successful.
 *  \return None.
 */
/*! \typedef onNavigateBodyFailed()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.navigateBodyTo method call is failed.
 *  \return None.
 */
void REEMProxyManager::doneNavgiateBodyAction( const actionlib::SimpleClientGoalState & state,
                                             const MoveBaseResultConstPtr & result )
{
  bodyCtrlWithNavigation_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onNavigateBodySuccess", NULL );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onNavigateBodyFailed", NULL );
  }
  
  PyGILState_Release( gstate );
  
  ROS_INFO("nagivate body finished in state [%s]", state.toString().c_str());
}

/*! \typedef onGotoPOISuccess( poi_name )
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.gotoPOI method call is successful.
 *  \return None.
 */
/*! \typedef onGotoPOIFailed( poi_name )
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.gotPOI method call is failed.
 *  \return None.
 */
void REEMProxyManager::doneGotoPOIAction( const actionlib::SimpleClientGoalState & state,
                                             const GoToPOIResultConstPtr & result )
{
  std::string poi = targetPOIName_;

  targetPOIName_ = "";

  bodyCtrlWithNavigation_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * arg = Py_BuildValue( "(s)", poi.c_str() );

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onGotoPOISuccess", arg );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onGotoPOIFailed", arg );
  }
  Py_DECREF( arg );

  PyGILState_Release( gstate );

  ROS_INFO("go to POI finished in state [%s]", state.toString().c_str());
}

void REEMProxyManager::moveLArmActionFeedback( const FollowJointTrajectoryFeedbackConstPtr & feedback )
{
  ROS_INFO( "Left arm trajectory move action feedback." );

  /*
  if (feedback->time_to_completion.toSec() > lArmActionTimeout_) {
    ROS_INFO( "Left arm trajectory move action will exceed %f seconds, force cancellation.", lArmActionTimeout_);
    mlacClient_->cancelGoal();
    lArmCtrl_ = false;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    PyObject * arg = Py_BuildValue( "(O)", Py_True );

    PyREEMModule::instance()->invokeCallback( "onMoveArmActionFailed", arg );

    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
   */
}

void REEMProxyManager::moveRArmActionFeedback( const FollowJointTrajectoryFeedbackConstPtr & feedback )
{
  ROS_INFO( "Right arm trajectory move action feedback." );
  /*
  if (feedback->time_to_completion.toSec() > rArmActionTimeout_) {
    ROS_INFO( "Right arm trajectory move action will exceed %f seconds, force cancellation.", rArmActionTimeout_);
    mracClient_->cancelGoal();
    rArmCtrl_ = false;
    
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
    
    PyObject * arg = Py_BuildValue( "(O)", Py_False );
    
    PyREEMModule::instance()->invokeCallback( "onMoveArmActionFailed", arg );
    
    Py_DECREF( arg );
    
    PyGILState_Release( gstate );
  }
   */
}

void REEMProxyManager::moveLHandActionFeedback( const FollowJointTrajectoryFeedbackConstPtr & feedback )
{
  ROS_INFO( "Left hand trajectory move action feedback." );

  /*
  if (feedback->time_to_completion.toSec() > lArmActionTimeout_) {
    ROS_INFO( "Left arm trajectory move action will exceed %f seconds, force cancellation.", lArmActionTimeout_);
    mlacClient_->cancelGoal();
    lArmCtrl_ = false;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    PyObject * arg = Py_BuildValue( "(O)", Py_True );

    PyREEMModule::instance()->invokeCallback( "onMoveArmActionFailed", arg );

    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
   */
}

void REEMProxyManager::moveRHandActionFeedback( const FollowJointTrajectoryFeedbackConstPtr & feedback )
{
  ROS_INFO( "Right hand trajectory move action feedback." );
  /*
  if (feedback->time_to_completion.toSec() > rArmActionTimeout_) {
    ROS_INFO( "Right arm trajectory move action will exceed %f seconds, force cancellation.", rArmActionTimeout_);
    mracClient_->cancelGoal();
    rArmCtrl_ = false;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    PyObject * arg = Py_BuildValue( "(O)", Py_False );

    PyREEMModule::instance()->invokeCallback( "onMoveArmActionFailed", arg );

    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
   */
}

/*! \typedef onHandActionSuccess(is_left_hand)
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.openHand, PyREEM.closeHand and PyREEM.setHandPosition method call is successful.
 *  \param bool is_left_hand. True means left hand; False means right hand.
 *  \return None.
 */
/*! \typedef onHandActionFailed(is_left_hand)
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.openHand, PyREEM.closeHand and PyREEM.setHandPosition method call is failed.
 *  \param bool is_left_hand. True means left hand; False means right hand.
 *  \return None.
 */
void REEMProxyManager::doneLHandAction( const actionlib::SimpleClientGoalState & state,
                                         const FollowJointTrajectoryResultConstPtr & result )
{
  lHandCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * arg = Py_BuildValue( "(O)", Py_True );

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onHandActionSuccess", arg );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onHandActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO( "Left hand action finished in state [%s]", state.toString().c_str());
}

void REEMProxyManager::doneRHandAction( const actionlib::SimpleClientGoalState & state,
                                         const FollowJointTrajectoryResultConstPtr & result )
{
  rHandCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(O)", Py_False );
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onHandActionSuccess", arg );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onHandActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );

  ROS_INFO( "Right hand action finished in state [%s]", state.toString().c_str());
}

/*! \typedef onPlayMotionSuccess()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.playDefaultMotion method call is successful.
 *  \return None.
 */
/*! \typedef onPlayMotionFailed()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.playDefaultMotion method call is failed.
 *  \return None.
 */
void REEMProxyManager::donePlayMotionAction( const actionlib::SimpleClientGoalState & state,
                            const PlayMotionResultConstPtr & result )
{
  defaultMotionCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onPlayMotionSuccess", NULL );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onPlayMotionFailed", NULL );
  }

  PyGILState_Release( gstate );

  ROS_INFO( "On play default motion finished in state [%s]", state.toString().c_str());
}

/*! \typedef onPlayAudioSuccess()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.playAudioFile method call is successful.
 *  \return None.
 */
/*! \typedef onPlayAudioFailed(reason)
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.playAudioFile method call is failed.
 *  \param str reason. The reason for failed audio play.
 *  \return None.
 */
void REEMProxyManager::donePlayAudioAction( const actionlib::SimpleClientGoalState & state,
                          const AudioFilePlayResultConstPtr & result )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onPlayAudioSuccess", NULL );
  }
  else {
    PyObject * arg = Py_BuildValue( "(s)", result->reason.c_str() );

    PyREEMModule::instance()->invokeCallback( "onPlayAudioFailed", NULL );

    Py_DECREF( arg );
  }

  PyGILState_Release( gstate );

  ROS_INFO( "On play audio file finished in state [%s]", state.toString().c_str());
}

/*! \typedef onRecordAudioSuccess()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.recordAudioFile method call is successful.
 *  \return None.
 */
/*! \typedef onRecordAudioFailed(reason)
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.recordAudioFile method call is failed.
 *  \param str reason. The reason for failed audio play.
 *  \return None.
 */
void REEMProxyManager::doneRecordAudioAction( const actionlib::SimpleClientGoalState & state,
                          const RecordAudioResultConstPtr & result )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onRecordAudioSuccess", NULL );
  }
  else {
    PyObject * arg = Py_BuildValue( "(s)", result->reason.c_str() );

    PyREEMModule::instance()->invokeCallback( "onRecordAudioFailed", NULL );

    Py_DECREF( arg );
  }

  PyGILState_Release( gstate );

  ROS_INFO( "On record audio file finished in state [%s]", state.toString().c_str());
}

/*! \typedef onFaceEnrolmentSuccess()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.enrolHumanFace method call is successful.
 *  \return None.
 */
/*! \typedef onFaceEnrolmentFailed()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.enrolHumanFace method call is failed.
 *  \return None.
 */

void REEMProxyManager::doneFaceEnrolmentAction( const actionlib::SimpleClientGoalState & state,
                          const ObjectEnrolmentResultConstPtr & result )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onFaceEnrolmentSuccess", NULL );
  }
  else {
    PyObject * arg = Py_BuildValue( "(s)", result->reason.c_str() );

    PyREEMModule::instance()->invokeCallback( "onFaceEnrolmentFailed", NULL );

    Py_DECREF( arg );
  }

  PyGILState_Release( gstate );

  ROS_INFO( "On face enrolment finished in state [%s]", state.toString().c_str());
}

/*! \typedef onSpeakSuccess()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.say method call is successful.
 *  \return None.
 */
/*! \typedef onSpeakFailed()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.say method call is failed.
 *  \return None.
 */
void REEMProxyManager::doneSpeakAction( const actionlib::SimpleClientGoalState & state,
                            const TtsResultConstPtr & result )
{
  speechCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyREEMModule::instance()->invokeCallback( "onSpeakSuccess", NULL );
  }
  else {
    PyREEMModule::instance()->invokeCallback( "onSpeakFailed", NULL );
  }

  PyGILState_Release( gstate );

  ROS_INFO( "On speak finished in state [%s]", state.toString().c_str());
}

bool REEMProxyManager::sayWithVolume( const std::string & text, float volume, bool toBlock )
{
  if (!soundClient_ || speechCtrl_)
    return false;

  pal_interaction_msgs::TtsGoal goal;

  goal.rawtext.text = text;
  goal.rawtext.lang_id = "en_GB";

  speechCtrl_ = true;
  soundClient_->sendGoal( goal,
                        boost::bind( &REEMProxyManager::doneSpeakAction, this, _1, _2 ),
                        TTSClient::SimpleActiveCallback(),
                        TTSClient::SimpleFeedbackCallback() );
  return true;
}

void REEMProxyManager::setAudioVolume( const int vol )
{
  if (vol < 0 || vol > 100)
    return;

  std_msgs::Int8 msg;
  msg.data = vol;
  aPub_.publish( msg );
}

void REEMProxyManager::baseScanDataCB( const sensor_msgs::LaserScanConstPtr & msg )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  boost::recursive_mutex::scoped_lock lock( basescansub_mutex_, boost::try_to_lock );

  if (!lock) {
    PyGILState_Release( gstate );
    return;
  }

  if (baseScanTransformFrame_.length() > 0) { // we transform to point cloud w.r.t to the frame
    sensor_msgs::PointCloud cloud;
    try  {
      tflistener_.waitForTransform( baseScanTransformFrame_, msg->header.frame_id,
                                   msg->header.stamp,
                                   ros::Duration().fromSec( (msg->ranges.size()-1) * msg->time_increment ) );

      lprojector_.transformLaserScanToPointCloud( baseScanTransformFrame_, *msg, cloud, tflistener_ );
    }
    catch (tf::TransformException& e) {
      PyGILState_Release( gstate );
      return;
    }
    
    size_t psize = cloud.points.size();
    
    PyObject * retList = PyList_New( psize );

    for (size_t i = 0; i < psize; ++i) {
      PyList_SetItem( retList, i, Py_BuildValue( "(ddd)", cloud.points[i].x, cloud.points[i].y, cloud.points[i].z ) );
    }
    PyObject * arg = Py_BuildValue( "(O)", retList );

    PyREEMModule::instance()->invokeBaseScanCallback( arg );
    
    Py_DECREF( arg );
    Py_DECREF( retList );
  }
  else {
    size_t rsize = msg->ranges.size();
    size_t isize = msg->intensities.size();
    
    PyObject * rangeData = PyList_New( rsize );
    PyObject * intensityData = PyList_New( isize );
    
    float data = -0.1;
    for (size_t i = 0; i < rsize; ++i) {
      data = msg->ranges[i];
      if (data > msg->range_max || data < msg->range_min) {
        data = -0.1; // invalid data
      }
      PyList_SetItem( rangeData, i, PyFloat_FromDouble( data ) );
    }

    data = 0.0;
    for (size_t i = 0; i < isize; ++i) {
      data = msg->intensities[i];
      PyList_SetItem( intensityData, i, PyFloat_FromDouble( data ) );
    }

    PyObject * arg = Py_BuildValue( "(OO)", rangeData, intensityData );
    
    PyREEMModule::instance()->invokeBaseScanCallback( arg );

    Py_DECREF( arg );
    Py_DECREF( rangeData );
    Py_DECREF( intensityData );
  }
  PyGILState_Release( gstate );
}

void REEMProxyManager::tiltScanDataCB( const sensor_msgs::LaserScanConstPtr & msg )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  boost::recursive_mutex::scoped_lock lock( tiltscansub_mutex_, boost::try_to_lock );

  if (!lock) {
    PyGILState_Release( gstate );
    return;
  }

  if (tiltScanTransformFrame_.length() > 0) { // we transform to point cloud w.r.t to the frame
    sensor_msgs::PointCloud cloud;
    try  {
      tflistener_.waitForTransform( tiltScanTransformFrame_, msg->header.frame_id,
                                   msg->header.stamp,
                                   ros::Duration().fromSec( (msg->ranges.size()-1) * msg->time_increment ) );
      
      lprojector_.transformLaserScanToPointCloud( tiltScanTransformFrame_, *msg, cloud, tflistener_ );
    }
    catch (tf::TransformException& e) {
      PyGILState_Release( gstate );
      return;
    }

    size_t psize = cloud.points.size();
    
    PyObject * retList = PyList_New( psize );
    
    for (size_t i = 0; i < psize; ++i) {
      PyList_SetItem( retList, i, Py_BuildValue( "(ddd)", cloud.points[i].x, cloud.points[i].y, cloud.points[i].z ) );
    }
    PyObject * arg = Py_BuildValue( "(O)", retList );
    
    PyREEMModule::instance()->invokeTiltScanCallback( arg );
    
    Py_DECREF( arg );
    Py_DECREF( retList );
  }
  else {
    size_t rsize = msg->ranges.size();
    size_t isize = msg->intensities.size();
    
    PyObject * rangeData = PyList_New( rsize );
    PyObject * intensityData = PyList_New( isize );
    
    float data = -0.1;
    for (size_t i = 0; i < rsize; ++i) {
      data = msg->ranges[i];
      if (data > msg->range_max || data < msg->range_min) {
        data = -0.1; // invalid data
      }
      PyList_SetItem( rangeData, i, PyFloat_FromDouble( data ) );
    }
    
    data = 0.0;
    for (size_t i = 0; i < isize; ++i) {
      data = msg->intensities[i];
      PyList_SetItem( intensityData, i, PyFloat_FromDouble( data ) );
    }
    
    PyObject * arg = Py_BuildValue( "(OO)", rangeData, intensityData );
    
    PyREEMModule::instance()->invokeTiltScanCallback( arg );
    
    Py_DECREF( arg );
    Py_DECREF( rangeData );
    Py_DECREF( intensityData );
  }
  PyGILState_Release( gstate );
}

void REEMProxyManager::torsoSonarDataCB( const sensor_msgs::RangeConstPtr & msg )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  boost::recursive_mutex::scoped_lock lock( sonarsub_mutex_, boost::try_to_lock );

  if (!lock) {
    PyGILState_Release( gstate );
    return;
  }

  if ((msg->range > msg->max_range) || (msg->range < msg->min_range)) {
    PyGILState_Release( gstate );
    return;
  }

  PyObject * retObj = PyDict_New();
  PyObject * elemObj = PyBool_FromLong( (msg->header.frame_id.compare( "torso_sonar_15_link" ) == 0) ? 1 : 0 );
  PyDict_SetItemString( retObj, "isleft", elemObj );
  Py_DECREF( elemObj );

  elemObj = PyFloat_FromDouble( roundf(msg->range * 100.0) / 100.0 );
  PyDict_SetItemString( retObj, "range", elemObj );
  Py_DECREF( elemObj );

  PyObject * arg = Py_BuildValue( "(O)", retObj );

  PyREEMModule::instance()->invokeTorsoSonarCallback( arg );

  Py_DECREF( arg );
  Py_DECREF( retObj );

  PyGILState_Release( gstate );
}

void REEMProxyManager::htObjStatusCB( const pyride_common_msgs::TrackedObjectStatusChangeConstPtr & msg )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  boost::recursive_mutex::scoped_lock lock( htsub_mutex_, boost::try_to_lock );

  if (!lock) {
    PyGILState_Release( gstate );
    return;
  }

  PyObject * arg = Py_BuildValue( "(iisi)", msg->objtype, msg->trackid,
                                 msg->name.c_str(), msg->status );

  PyREEMModule::instance()->invokeObjectDetectionCallback( arg );

  Py_DECREF( arg );

  PyGILState_Release( gstate );
}

void REEMProxyManager::htObjUpdateCB( const pyride_common_msgs::TrackedObjectUpdateConstPtr & msg )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  boost::recursive_mutex::scoped_lock lock( htsub_mutex_, boost::try_to_lock );

  if (!lock) {
    PyGILState_Release( gstate );
    return;
  }

  size_t rsize = msg->objects.size();

  PyObject * retList = PyList_New( rsize );

  for (size_t i = 0; i < rsize; i++) {
    pyride_common_msgs::TrackedObjectInfo obj = msg->objects[i];

    PyObject * retObj = PyDict_New();
    PyObject * elemObj = PyInt_FromLong( obj.objtype );
    PyDict_SetItemString( retObj, "object_type", elemObj );
    Py_DECREF( elemObj );

    elemObj = PyInt_FromLong( obj.id );
    PyDict_SetItemString( retObj, "track_id", elemObj );
    Py_DECREF( elemObj );

    elemObj = PyTuple_New( 4 );
    PyTuple_SetItem( elemObj, 0, PyFloat_FromDouble( obj.bound.tl_x ) );
    PyTuple_SetItem( elemObj, 1, PyFloat_FromDouble( obj.bound.tl_y ) );
    PyTuple_SetItem( elemObj, 2, PyFloat_FromDouble( obj.bound.width ) );
    PyTuple_SetItem( elemObj, 3, PyFloat_FromDouble( obj.bound.height ) );
    PyDict_SetItemString( retObj, "bound", elemObj );
    Py_DECREF( elemObj );

    elemObj = PyTuple_New( 3 );
    PyTuple_SetItem( elemObj, 0, PyFloat_FromDouble( obj.est_pos.x ) );
    PyTuple_SetItem( elemObj, 1, PyFloat_FromDouble( obj.est_pos.y ) );
    PyTuple_SetItem( elemObj, 2, PyFloat_FromDouble( obj.est_pos.z ) );
    PyDict_SetItemString( retObj, "est_pos", elemObj );
    Py_DECREF( elemObj );

    PyList_SetItem( retList, i, retObj );
  }

  PyObject * arg = Py_BuildValue( "(O)", retList );

  PyREEMModule::instance()->invokeObjectTrackingCallback( arg );

  Py_DECREF( arg );
  Py_DECREF( retList );

  PyGILState_Release( gstate );
}

bool REEMProxyManager::getRobotPose( std::vector<double> & positions, std::vector<double> & orientation, bool in_map )
{
  tf::StampedTransform curTransform;
  
  try {
    tflistener_.waitForTransform( (in_map ? "map" : "odom"), "base_footprint",
                                 ros::Time(0), ros::Duration( 1.0 ) );
    
    
    tflistener_.lookupTransform( (in_map ? "map" : "odom"), "base_footprint",
                                ros::Time(0), curTransform );
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return false;
  }
  
  tf::Vector3 pos = curTransform.getOrigin();
  tf::Quaternion orient = curTransform.getRotation();

  positions[0] = pos.getX();
  positions[1] = pos.getY();
  positions[2] = pos.getZ();
  
  orientation[0] = orient.w();
  orientation[1] = orient.x();
  orientation[2] = orient.y();
  orientation[3] = orient.z();
  
  return true;
}
  
bool REEMProxyManager::getRelativeTF( const char * ref_frame,
                                        const char * child_frame,
                                        std::vector<double> & positions,
                                        std::vector<double> & orientation )
{
  if (!ref_frame || !child_frame) {
    return false;
  }
  tf::StampedTransform curTransform;
  
  try {
    tflistener_.waitForTransform( ref_frame, child_frame,
                                 ros::Time(0), ros::Duration( 1.0 ) );
    
    
    tflistener_.lookupTransform( ref_frame, child_frame,
                                ros::Time(0), curTransform );
  }
  catch (tf::TransformException ex) {
    ROS_ERROR( "%s",ex.what() );
    return false;
  }
  
  tf::Vector3 pos = curTransform.getOrigin();
  tf::Quaternion orient = curTransform.getRotation();
  
  positions[0] = pos.getX();
  positions[1] = pos.getY();
  positions[2] = pos.getZ();
  
  orientation[0] = orient.w();
  orientation[1] = orient.x();
  orientation[2] = orient.y();
  orientation[3] = orient.z();
  
  return true;
}

bool REEMProxyManager::navigateBodyTo( const std::vector<double> & positions, const std::vector<double> & orientation )
{
  if (bodyCtrlWithNavigation_ || bodyCtrlWithOdmetry_ || !moveBaseClient_)
    return false;
  
  if (positions.size() != (size_t)3 || orientation.size() != (size_t)4) {
    return false;
  }

  move_base_msgs::MoveBaseGoal goal;
  
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = positions[0];
  goal.target_pose.pose.position.y = positions[1];
  goal.target_pose.pose.position.z = positions[2];
  goal.target_pose.pose.orientation.w = orientation[0];
  goal.target_pose.pose.orientation.x = orientation[1];
  goal.target_pose.pose.orientation.y = orientation[2];
  goal.target_pose.pose.orientation.z = orientation[3];
  
  moveBaseClient_->sendGoal( goal,
                        boost::bind( &REEMProxyManager::doneNavgiateBodyAction, this, _1, _2 ),
                        MoveBaseClient::SimpleActiveCallback(),
                        MoveBaseClient::SimpleFeedbackCallback() );

  bodyCtrlWithNavigation_ = true;
  return true;
}

bool REEMProxyManager::gotoPOI( const std::string & poi_name )
{
  if (bodyCtrlWithNavigation_ || bodyCtrlWithOdmetry_ || !gotoPOIClient_)
    return false;

  //TODO: check whether the POI exists?
  pal_navigation_msgs::GoToPOIGoal goal;

  goal.poi.data = poi_name;
  targetPOIName_ = poi_name;

  gotoPOIClient_->sendGoal( goal,
                        boost::bind( &REEMProxyManager::doneGotoPOIAction, this, _1, _2 ),
                        GotoPOIClient::SimpleActiveCallback(),
                        GotoPOIClient::SimpleFeedbackCallback() );

  bodyCtrlWithNavigation_ = true;
  return true;
}

bool REEMProxyManager::getHeadPos( double & yaw, double & pitch )
{
  yaw = pitch = 0.0;
  int resCnt = 0;

  boost::mutex::scoped_lock lock( joint_mutex_ );
  for (size_t i = 0; i < curJointNames_.size(); i++) {
    if (curJointNames_.at( i ).compare( "head_1_joint" ) == 0) {
      yaw = curJointPositions_.at( i );
      resCnt++;
    }
    else if (curJointNames_.at( i ).compare( "head_2_joint" ) == 0) {
      pitch = curJointPositions_.at( i );
      resCnt++;
    }
    if (resCnt == 2) {
      return true;
    }
  }
  return false;
}

bool REEMProxyManager::getTorsoPos( double & yaw, double & pitch )
{
  yaw = pitch = 0.0;
  int resCnt = 0;

  boost::mutex::scoped_lock lock( joint_mutex_ );
  for (size_t i = 0; i < curJointNames_.size(); i++) {
    if (curJointNames_.at( i ).compare( "torso_1_joint" ) == 0) {
      yaw = curJointPositions_.at( i );
      resCnt++;
    }
    else if (curJointNames_.at( i ).compare( "torso_2_joint" ) == 0) {
      pitch = curJointPositions_.at( i );
      resCnt++;
    }
    if (resCnt == 2) {
      return true;
    }
  }
  return false;
}

bool REEMProxyManager::getPositionForJoints( std::vector<std::string> & joint_names, std::vector<double> & positions )
{
  positions.clear();
  
  boost::mutex::scoped_lock lock( joint_mutex_ );
  for (size_t j = 0; j < joint_names.size(); j++) {
    for (size_t i = 0; i < curJointNames_.size(); i++) {
      if (curJointNames_.at( i ).compare( joint_names.at( j ) ) == 0) {
        positions.push_back( curJointPositions_.at( i ) );
        break;
      }
    }
  }
  return (joint_names.size() == positions.size());
}

bool REEMProxyManager::getJointPos( const char * joint_name, double & value )
{
  if (joint_name == NULL)
    return false;

  boost::mutex::scoped_lock lock( joint_mutex_ );
  for (size_t i = 0; i < curJointNames_.size(); i++) {
    if (curJointNames_.at( i ).compare( joint_name ) == 0) {
      value = curJointPositions_.at( i );
      return true;
    }
  }
  return false;
}

void REEMProxyManager::registerForBaseScanData()
{
  if (rawBaseScanSub_ || baseScanSub_) {
    ROS_WARN( "Already registered for base laser scan." );
  }
  else {
    rawBaseScanSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "scan", 1, &REEMProxyManager::baseScanDataCB, this ) );
  }
}

void REEMProxyManager::registerForBaseScanData( const std::string & target_frame )
{
  if (rawBaseScanSub_ || baseScanSub_) {
    ROS_WARN( "Already registered for base laser scan." );
  }
  else {
    baseScanTransformFrame_ = target_frame;
    baseScanSub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>( *mCtrlNode_, "scan", 10 );
    baseScanNotifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>( *baseScanSub_, tflistener_, baseScanTransformFrame_, 10 );

    baseScanNotifier_->registerCallback( boost::bind( &REEMProxyManager::baseScanDataCB, this, _1 ) );
    baseScanNotifier_->setTolerance( ros::Duration( 0.01 ) );
  }
}

void REEMProxyManager::deregisterForBaseScanData()
{
  boost::recursive_mutex::scoped_lock lock( basescansub_mutex_ );
  if (rawBaseScanSub_) {
    rawBaseScanSub_->shutdown();
    delete rawBaseScanSub_;
    rawBaseScanSub_ = NULL;
  }

  if (baseScanSub_) {
    baseScanTransformFrame_ = "";
    baseScanSub_->unsubscribe();
    baseScanNotifier_->clear();
    delete baseScanNotifier_;
    delete baseScanSub_;
    baseScanNotifier_ = NULL;
    baseScanSub_ = NULL;
  }
}
  
void REEMProxyManager::registerForTiltScanData()
{
  if (rawTiltScanSub_ || tiltScanSub_) {
    ROS_WARN( "Already registered for tilt laser scan." );
  }
  else {
    rawTiltScanSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "hokuyo/LAS_01", 1, &REEMProxyManager::tiltScanDataCB, this ) );
  }
}

void REEMProxyManager::registerForTiltScanData( const std::string & target_frame )
{
  if (rawTiltScanSub_ || tiltScanSub_) {
    ROS_WARN( "Already registered for tilt laser scan." );
  }
  else {
    tiltScanTransformFrame_ = target_frame;
    tiltScanSub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>( *mCtrlNode_, "hokuyo/LAS_01", 10 );
    tiltScanNotifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>( *tiltScanSub_, tflistener_, tiltScanTransformFrame_, 10 );
    
    tiltScanNotifier_->registerCallback( boost::bind( &REEMProxyManager::tiltScanDataCB, this, _1 ) );
    tiltScanNotifier_->setTolerance( ros::Duration( 0.01 ) );
  }
}
  
void REEMProxyManager::deregisterForTiltScanData()
{
  boost::recursive_mutex::scoped_lock lock( tiltscansub_mutex_ );
  if (rawTiltScanSub_) {
    rawTiltScanSub_->shutdown();
    delete rawTiltScanSub_;
    rawTiltScanSub_ = NULL;
  }
  
  if (tiltScanSub_) {
    tiltScanTransformFrame_ = "";
    tiltScanSub_->unsubscribe();
    tiltScanNotifier_->clear();
    delete tiltScanNotifier_;
    delete tiltScanSub_;
    tiltScanNotifier_ = NULL;
    tiltScanSub_ = NULL;
  }
}

void REEMProxyManager::registerForPalFaceData()
{
  if (faceDetectSub_) {
    ROS_WARN( "Already registered for face detection scan." );
  }
  else {
    faceDetectSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "pal_face/faces", 1, &REEMProxyManager::palFaceDataCB, this ) );
  }
}

void REEMProxyManager::deregisterForPalFaceData()
{
  if (faceDetectSub_) {
    boost::recursive_mutex::scoped_lock lock( palfacesub_mutex_ );

    faceDetectSub_->shutdown();
    delete faceDetectSub_;
    faceDetectSub_ = NULL;
  }
}

void REEMProxyManager::registerForLegData( const float distance )
{
  if (legDetectSub_) {
    ROS_WARN( "Already registered for leg detection." );
  }
  else {
    legDetectDistance_ = distance * distance;
    legDetectSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "leg_tracker_measurements", 1, &REEMProxyManager::legDataCB, this ) );
  }
}

void REEMProxyManager::deregisterForLegData()
{
  if (legDetectSub_) {
    boost::recursive_mutex::scoped_lock lock( legsub_mutex_ );
    legDetectSub_->shutdown();
    delete legDetectSub_;
    legDetectSub_ = NULL;
  }
}

void REEMProxyManager::registerForSonarData()
{
  if (torsoSonarSub_) {
    ROS_WARN( "Already registered for torso sonar data." );
  }
  else {
    torsoSonarSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "sonar_torso", 1, &REEMProxyManager::torsoSonarDataCB, this ) );
  }
}

void REEMProxyManager::deregisterForSonarData()
{
  if (torsoSonarSub_) {
    boost::recursive_mutex::scoped_lock lock( sonarsub_mutex_ );

    torsoSonarSub_->shutdown();
    delete torsoSonarSub_;
    torsoSonarSub_ = NULL;
  }
}

void REEMProxyManager::registerForHumanData( bool tracking_data )
{
  if (htObjStatusSub_) {
    ROS_WARN( "Already registered for human detection data." );
  }
  else {
    htObjStatusSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "/people_dtr/object_status", 1, &REEMProxyManager::htObjStatusCB, this ) );
    if (tracking_data) {
      htObjUpdateSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "/people_dtr/object_update", 1, &REEMProxyManager::htObjUpdateCB, this ) );
    }
  }
}

void REEMProxyManager::deregisterForHumanData()
{
  if (htObjStatusSub_) {
    boost::recursive_mutex::scoped_lock lock( htsub_mutex_ );
    htObjStatusSub_->shutdown();
    delete htObjStatusSub_;
    htObjStatusSub_ = NULL;
  }
  if (htObjUpdateSub_) {
    boost::recursive_mutex::scoped_lock stop_lock( htsub_mutex_ );
    htObjUpdateSub_->shutdown();
    delete htObjUpdateSub_;
    htObjUpdateSub_ = NULL;
  }
}

int REEMProxyManager::setEarLED( const REEMLedColour colour, const int side )
{
  if (!ledColourClient_.exists()) {
    return -1;
  }
  pal_device_msgs::TimedColourEffect srvMsg;

  srvMsg.request.leds.ledMask = side;
  srvMsg.request.effectDuration = ros::Duration( 0.0 );
  srvMsg.request.priority = 96; //fixed priority
  srvMsg.request.color = this->colour2RGB( colour );
  if (ledColourClient_.call( srvMsg )) {
    return srvMsg.response.effectId;
  }
  return -1;
}

int REEMProxyManager::pulseEarLED( const REEMLedColour colour1, const REEMLedColour colour2, const int side, const float period )
{
  if (!ledPulseClient_.exists()) {
    return -1;
  }
  pal_device_msgs::TimedFadeEffect srvMsg;

  srvMsg.request.leds.ledMask = side;
  srvMsg.request.colorChangeDuration = ros::Duration( period );
  srvMsg.request.priority = 120; //fixed priority
  srvMsg.request.reverseFade = true;
  srvMsg.request.firstColor = this->colour2RGB( colour1 );
  srvMsg.request.secondColor = this->colour2RGB( colour2 );

  if (ledPulseClient_.call( srvMsg )) {
    return srvMsg.response.effectId;
  }
  return -1;
}

bool REEMProxyManager::cancelEarLED( const int effectID )
{
  if (!cancelLedClient_.exists() || effectID <= 0) {
    return false;
  }
  pal_device_msgs::CancelEffect srvMsg;

  srvMsg.request.effectId = effectID;

  return cancelLedClient_.call( srvMsg );
}

bool REEMProxyManager::moveHeadTo( double yaw, double pitch, bool relative, float time_to_reach )
{
  if (headCtrlWithActionClient_ || headCtrlWithTrajActionClient_ || defaultMotionCtrl_ || !headClient_)
    return false;
  
  double newYaw, newPitch;
  this->getHeadPos( reqHeadYaw_, reqHeadPitch_ );
  if (relative) {
    newYaw = clamp( reqHeadYaw_ + yaw, kMaxHeadPan );
    newPitch = reqHeadPitch_ + pitch;
  }
  else {
    newYaw = clamp( yaw, kMaxHeadPan );
    newPitch = pitch;
  }
  
  if (newPitch < kMinHeadTilt) {
    newPitch = kMinHeadTilt;
  }
  else if (newPitch > kMaxHeadTilt) {
    newPitch = kMaxHeadTilt;
  }

  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back( "head_1_joint" );
  goal.trajectory.joint_names.push_back( "head_2_joint" );

  goal.trajectory.points.resize( 1 );

  goal.trajectory.points[0].positions.resize( 2 );
    // Velocities
  goal.trajectory.points[0].velocities.resize( 2 );

  goal.trajectory.points[0].positions[0] = newYaw;
  goal.trajectory.points[0].velocities[0] = 0.0;
  goal.trajectory.points[0].positions[1] = newPitch;
  goal.trajectory.points[0].velocities[1] = 0.0;
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration( time_to_reach );

  headCtrlWithTrajActionClient_ = true;

  headClient_->sendGoal( goal,
                          boost::bind( &REEMProxyManager::doneHeadTrajAction, this, _1, _2 ),
                          FollowTrajectoryClient::SimpleActiveCallback(),
                          FollowTrajectoryClient::SimpleFeedbackCallback() );

  return true;
}

bool REEMProxyManager::moveHeadWithJointTrajectory( std::vector< std::vector<double> > & trajectory,
                                   std::vector<float> & times_to_reach )
{
  if (headCtrlWithActionClient_ || headCtrlWithTrajActionClient_ || defaultMotionCtrl_ || !headClient_)
    return false;

  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back( "head_1_joint" );
  goal.trajectory.joint_names.push_back( "head_2_joint" );

  goal.trajectory.points.resize( trajectory.size() );

  float time_to_reach_for_pt = 0.0;

  for (size_t jp = 0; jp < trajectory.size(); ++jp) {
    goal.trajectory.points[jp].positions.resize( 2 );
    // Velocities
    goal.trajectory.points[jp].velocities.resize( 2 );

    for (size_t j = 0; j < 2; ++j) {
      goal.trajectory.points[jp].positions[j] = trajectory[jp][j];
      goal.trajectory.points[jp].velocities[j] = 0.0;
    }
    time_to_reach_for_pt += times_to_reach[jp];
    goal.trajectory.points[jp].time_from_start = ros::Duration( time_to_reach_for_pt );
  }

  headCtrlWithTrajActionClient_ = true;

  headClient_->sendGoal( goal,
                          boost::bind( &REEMProxyManager::doneHeadTrajAction, this, _1, _2 ),
                          FollowTrajectoryClient::SimpleActiveCallback(),
                          FollowTrajectoryClient::SimpleFeedbackCallback() );

  return true;
}

bool REEMProxyManager::pointHeadTo( const std::string & frame, float x, float y, float z )
{
  if (headCtrlWithActionClient_ || headCtrlWithTrajActionClient_ || defaultMotionCtrl_ || !phClient_)
    return false;

  tf::StampedTransform transform;

  control_msgs::PointHeadGoal goal;
  
  //the target point, expressed in the requested frame
  geometry_msgs::PointStamped point;

  point.header.frame_id = frame;
  point.point.x = x;
  point.point.y = y;
  point.point.z = z;
  
  goal.target = point;
  
  //we are pointing the high-def camera frame
  //(pointing_axis defaults to X-axis)
  goal.pointing_frame = "stereo_link";
 
  goal.pointing_axis.x = 1;
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 0;

  //take at least 0.5 seconds to get there
  goal.min_duration = ros::Duration(0.5);
  
  //and go no faster than 1 rad/s
  goal.max_velocity = 1.0;

  headCtrlWithActionClient_ = true;

  // Need boost::bind to pass in the 'this' pointer
  phClient_->sendGoal( goal,
                      boost::bind( &REEMProxyManager::doneHeadAction, this, _1, _2 ),
                      PointHeadClient::SimpleActiveCallback(),
                      PointHeadClient::SimpleFeedbackCallback() );

  return true;
}

bool REEMProxyManager::moveArmWithJointPos( bool isLeftArm, std::vector<double> & positions, float time_to_reach )
{
  if (defaultMotionCtrl_) {
    ROS_WARN( "Default motion is executing." );
    return false;
  }

  if (positions.size() != 7) {
    return false;
  }
  
  control_msgs::FollowJointTrajectoryGoal goal;
  
  // First, the joint names, which apply to all waypoints
  if (isLeftArm) {
    if (!mlacClient_) {
      return false;
    }
    if (lArmCtrl_) {
      ROS_WARN( "Left arm is in motion." );
      return false;
    }
    goal.trajectory.joint_names.push_back( "arm_left_1_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_2_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_3_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_4_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_5_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_6_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_7_joint" );
    lArmCtrl_ = true;
  }
  else {
    if (!mracClient_) {
      return false;
    }
    if (rArmCtrl_) {
      ROS_WARN( "Right arm is in motion." );
      return false;
    }
    goal.trajectory.joint_names.push_back( "arm_right_1_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_2_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_3_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_4_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_5_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_6_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_7_joint" );
    rArmCtrl_ = true;
  }

  goal.trajectory.points.resize( 1 );
  
  // First trajectory point
  // Positions
  
  goal.trajectory.points[0].positions.resize( 7 );
    // Velocities
  goal.trajectory.points[0].velocities.resize( 7 );

  for (size_t j = 0; j < 7; ++j) {
    goal.trajectory.points[0].positions[j] = positions[j];
    goal.trajectory.points[0].velocities[j] = 0.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration( time_to_reach );

  if (isLeftArm) {
    lArmActionTimeout_ = time_to_reach * 1.05; // give additional 5% allowance
    mlacClient_->sendGoal( goal,
                          boost::bind( &REEMProxyManager::doneMoveLArmAction, this, _1, _2 ),
                          FollowTrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &REEMProxyManager::moveLArmActionFeedback, this, _1 ) );
  }
  else {
    rArmActionTimeout_ = time_to_reach * 1.05; // give additional 5% allowance
    mracClient_->sendGoal( goal,
                          boost::bind( &REEMProxyManager::doneMoveRArmAction, this, _1, _2 ),
                          FollowTrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &REEMProxyManager::moveRArmActionFeedback, this, _1 ) );
  }
  return true;
}

bool REEMProxyManager::moveArmWithJointTrajectory( bool isLeftArm, std::vector< std::vector<double> > & trajectory,
                                                  std::vector<float> & times_to_reach )
{
  if (defaultMotionCtrl_) {
    ROS_WARN( "Default motion is executing." );
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  
  // First, the joint names, which apply to all waypoints
  if (isLeftArm) {
    if (!mlacClient_) {
      return false;
    }
    if (lArmCtrl_) {
      ROS_WARN( "Left arm is in motion." );
      return false;
    }
    goal.trajectory.joint_names.push_back( "arm_left_1_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_2_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_3_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_4_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_5_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_6_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_7_joint" );
    lArmCtrl_ = true;
  }
  else {
    if (!mracClient_) {
      return false;
    }
    if (rArmCtrl_) {
      ROS_WARN( "Right arm is in motion." );
      return false;
    }
    goal.trajectory.joint_names.push_back( "arm_right_1_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_2_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_3_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_4_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_5_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_6_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_7_joint" );
    rArmCtrl_ = true;
  }
  
  goal.trajectory.points.resize( trajectory.size() );
  
  float time_to_reach_for_pt = 0.0;
  
  for (size_t jp = 0; jp < trajectory.size(); ++jp) {
    goal.trajectory.points[jp].positions.resize( 7 );
    // Velocities
    goal.trajectory.points[jp].velocities.resize( 7 );
    
    for (size_t j = 0; j < 7; ++j) {
      goal.trajectory.points[jp].positions[j] = trajectory[jp][j];
      goal.trajectory.points[jp].velocities[j] = 0.0;
    }
    time_to_reach_for_pt += times_to_reach[jp];
    goal.trajectory.points[jp].time_from_start = ros::Duration( time_to_reach_for_pt );
  }
  
  if (isLeftArm) {
    lArmActionTimeout_ = time_to_reach_for_pt * 1.05; // give additional 5% allowance
    mlacClient_->sendGoal( goal,
                          boost::bind( &REEMProxyManager::doneMoveLArmAction, this, _1, _2 ),
                          FollowTrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &REEMProxyManager::moveLArmActionFeedback, this, _1 ) );
  }
  else {
    rArmActionTimeout_ = time_to_reach_for_pt * 1.05; // give additional 5% allowance
    mracClient_->sendGoal( goal,
                          boost::bind( &REEMProxyManager::doneMoveRArmAction, this, _1, _2 ),
                          FollowTrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &REEMProxyManager::moveRArmActionFeedback, this, _1 ) );
  }
  return true;
}

bool REEMProxyManager::moveArmWithJointTrajectoryAndSpeed( bool isLeftArm,
                                        std::vector< std::vector<double> > & trajectory,
                                        std::vector< std::vector<double> > & joint_velocities,
                                        std::vector<float> & times_to_reach )
{
  if (defaultMotionCtrl_) {
    ROS_WARN( "Default motion is executing." );
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  
  // First, the joint names, which apply to all waypoints
  if (isLeftArm) {
    if (!mlacClient_) {
      return false;
    }
    if (lArmCtrl_) {
      ROS_WARN( "Left arm is in motion." );
      return false;
    }
    goal.trajectory.joint_names.push_back( "arm_left_1_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_2_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_3_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_4_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_5_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_6_joint" );
    goal.trajectory.joint_names.push_back( "arm_left_7_joint" );
    lArmCtrl_ = true;
  }
  else {
    if (!mracClient_) {
      return false;
    }
    if (rArmCtrl_) {
      ROS_WARN( "Right arm is in motion." );
      return false;
    }
    goal.trajectory.joint_names.push_back( "arm_right_1_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_2_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_3_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_4_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_5_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_6_joint" );
    goal.trajectory.joint_names.push_back( "arm_right_7_joint" );
    rArmCtrl_ = true;
  }
  
  goal.trajectory.points.resize( trajectory.size() );

  float time_to_reach_for_pt = 0.0;

  for (size_t jp = 0; jp < trajectory.size(); ++jp) {
    goal.trajectory.points[jp].positions.resize( 7 );
    // Velocities
    goal.trajectory.points[jp].velocities.resize( 7 );
    
    for (size_t j = 0; j < 7; ++j) {
      goal.trajectory.points[jp].positions[j] = trajectory[jp][j];
      goal.trajectory.points[jp].velocities[j] = joint_velocities[jp][j];
    }
    time_to_reach_for_pt += times_to_reach[jp];
    goal.trajectory.points[jp].time_from_start = ros::Duration( time_to_reach_for_pt );
  }
  
  if (isLeftArm) {
    lArmActionTimeout_ = time_to_reach_for_pt * 1.05; // give additional 5% allowance
    mlacClient_->sendGoal( goal,
                          boost::bind( &REEMProxyManager::doneMoveLArmAction, this, _1, _2 ),
                          FollowTrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &REEMProxyManager::moveLArmActionFeedback, this, _1 ) );
  }
  else {
    rArmActionTimeout_ = time_to_reach_for_pt * 1.05; // give additional 5% allowance
    mracClient_->sendGoal( goal,
                          boost::bind( &REEMProxyManager::doneMoveRArmAction, this, _1, _2 ),
                          FollowTrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &REEMProxyManager::moveRArmActionFeedback, this, _1 ) );
  }
  return true;
}

bool REEMProxyManager::moveArmWithGoalPose( bool isLeftArm, std::vector<double> & position,
                                          std::vector<double> & orientation, float time_to_reach )
{
  if (defaultMotionCtrl_) {
    ROS_WARN( "Default motion is executing." );
    return false;
  }

  if (!rarmGroup_ || !larmGroup_)
    return false;

  if (position.size() != 3 || orientation.size() != 4) {
    return false;
  }

  geometry_msgs::Pose targetPose;
  targetPose.position.x = position[0];
  targetPose.position.y = position[1];
  targetPose.position.z = position[2];

  targetPose.orientation.w = orientation[0];
  targetPose.orientation.x = orientation[1];
  targetPose.orientation.y = orientation[2];
  targetPose.orientation.z = orientation[3];

  moveit::planning_interface::MoveGroup::Plan movePlan;

  bool success = false;
  if (isLeftArm) {
    if (lArmCtrl_) {
      ROS_WARN( "Left arm is in motion." );
      return false;
    }
    larmGroup_->setPlanningTime( 5.0 );
    larmGroup_->allowReplanning( true );
    larmGroup_->setPoseTarget( targetPose );
    success = larmGroup_->plan( movePlan );
  }
  else  {
    if (rArmCtrl_) {
      ROS_WARN( "Right arm is in motion." );
      return false;
    }
    rarmGroup_->setPlanningTime( 5.0 );
    rarmGroup_->allowReplanning( true );
    rarmGroup_->setPoseTarget( targetPose );
    success = rarmGroup_->plan( movePlan );
  }

  if (!success) {
    ROS_ERROR( "Unable to generate successful motion plan for %s arm",
        isLeftArm ? "left" : "right" );
    return false;
  }

  success = false;

  if (isLeftArm) {
    //lArmCtrl_ = true; //no callback is bad!
    success = larmGroup_->asyncExecute( movePlan );
  }
  else {
    //rArmCtrl_ = true;
    success = rarmGroup_->asyncExecute( movePlan );
  }

  if (!success) {
    ROS_ERROR( "Unable to start executing move plan for %s arm",
        isLeftArm ? "left" : "right" );
    return false;
  }
  return true;
}

void REEMProxyManager::cancelArmMovement( bool isLeftArm )
{
  if (isLeftArm) {
    if (!lArmCtrl_ || !mlacClient_) {
      return;
    }
    if (mlacClient_->getState() == actionlib::SimpleClientGoalState::ACTIVE ||
        mlacClient_->getState() == actionlib::SimpleClientGoalState::PENDING)
    {
      mlacClient_->cancelGoal();
    }
    lArmCtrl_ = false;
  }
  else {
    if (!rArmCtrl_ || !mracClient_) {
      return;
    }
    if (mracClient_->getState() == actionlib::SimpleClientGoalState::ACTIVE ||
        mracClient_->getState() == actionlib::SimpleClientGoalState::PENDING)
    {
      mracClient_->cancelGoal();
    }
    rArmCtrl_ = false;
  }
}

void REEMProxyManager::cancelBodyMovement()
{
  if (!bodyCtrlWithOdmetry_)
    return;

  mCmd_.linear.x = mCmd_.linear.y = mCmd_.angular.z = 0.0;
  bodyCtrlWithOdmetry_ = false;
}

void REEMProxyManager::cancelBodyNavigation()
{
  if (!moveBaseClient_)
    return;

  if (bodyCtrlWithNavigation_ && moveBaseClient_->getState() == actionlib::SimpleClientGoalState::ACTIVE) {
    moveBaseClient_->cancelGoal();
    bodyCtrlWithNavigation_ = false;
  }
}

void REEMProxyManager::cancelGotoPOI()
{
  if (!gotoPOIClient_)
    return;

  if (bodyCtrlWithNavigation_ && gotoPOIClient_->getState() == actionlib::SimpleClientGoalState::ACTIVE) {
    gotoPOIClient_->cancelGoal();
    targetPOIName_= "";
    bodyCtrlWithNavigation_ = false;
  }
}

void REEMProxyManager::cancelAudioPlay()
{
  if (!playAudioClient_)
    return;

  if (playAudioClient_->getState() == actionlib::SimpleClientGoalState::ACTIVE) {
    playAudioClient_->cancelGoal();
  }
}

void REEMProxyManager::cancelAudioRecording()
{
  if (!recordAudioClient_)
    return;

  if (recordAudioClient_->getState() == actionlib::SimpleClientGoalState::ACTIVE) {
    recordAudioClient_->cancelGoal();
  }
}

void REEMProxyManager::cancelDefaultMotion()
{
  if (!playMotionClient_)
    return;

  if (playMotionClient_->getState() == actionlib::SimpleClientGoalState::ACTIVE) {
    playMotionClient_->cancelGoal();
    defaultMotionCtrl_ = false;
  }
}

bool REEMProxyManager::playDefaultMotion( const std::string & motion_name )
{
  if (headCtrlWithActionClient_ || headCtrlWithTrajActionClient_ ||
      rArmCtrl_ || lArmCtrl_ || torsoCtrl_ || defaultMotionCtrl_ || !playMotionClient_)
  {
    return false;
  }
  
  play_motion_msgs::PlayMotionGoal goal;

  goal.motion_name = motion_name;
  goal.skip_planning = false;
  goal.priority = 20;

  defaultMotionCtrl_ = true;

  playMotionClient_->sendGoal( goal,
                      boost::bind( &REEMProxyManager::donePlayMotionAction, this, _1, _2 ),
                      PlayMotionClient::SimpleActiveCallback(),
                      PlayMotionClient::SimpleFeedbackCallback() );
  return true;
}

bool REEMProxyManager::playAudioFile( const std::string & audio_name )
{
  if (!playAudioClient_)
    return false;

  audio_file_player::AudioFilePlayGoal goal;

  goal.filepath = audio_name;

  playAudioClient_->sendGoal( goal,
                      boost::bind( &REEMProxyManager::donePlayAudioAction, this, _1, _2 ),
                      PlayAudioClient::SimpleActiveCallback(),
                      PlayAudioClient::SimpleFeedbackCallback() );
  return true;
}

bool REEMProxyManager::recordAudioFile( const std::string & audio_name, const float period )
{
  if (!recordAudioClient_)
    return false;

  audio_stream::RecordAudioGoal goal;

  goal.format = "wave";
  goal.period = period;
  goal.filename = audio_name;

  recordAudioClient_->sendGoal( goal,
                      boost::bind( &REEMProxyManager::doneRecordAudioAction, this, _1, _2 ),
                      RecordAudioClient::SimpleActiveCallback(),
                      RecordAudioClient::SimpleFeedbackCallback() );
  return true;
}

bool REEMProxyManager::enrolHumanFace( const std::string & face_name, const int required_samples )
{
  if (!faceEnrolmentClient_)
    return false;

  if (face_name.length() == 0 || required_samples <= 0) // really just check for negative sample.
    return false;

  pyride_common_msgs::ObjectEnrolmentGoal goal;

  goal.name = face_name;
  goal.instances = required_samples;
  goal.timeout = required_samples * 1.5;

  faceEnrolmentClient_->sendGoal( goal,
                      boost::bind( &REEMProxyManager::doneFaceEnrolmentAction, this, _1, _2 ),
                      ObjectEnrolmentClient::SimpleActiveCallback(),
                      ObjectEnrolmentClient::SimpleFeedbackCallback() );
  return true;
}

bool REEMProxyManager::palFaceStartEnrollment( const std::string & name )
{
  if (!palFaceEnrolStartClient_.exists() || !palFaceDatabaseInit_)
    return false;

  pal_detection_msgs::StartEnrollment srvMsg;
  srvMsg.request.name = name;

  return (palFaceEnrolStartClient_.call( srvMsg ) && srvMsg.response.result);
}

bool REEMProxyManager::palFaceStopEnrollment()
{
  if (!palFaceEnrolStopClient_.exists() || !palFaceDatabaseInit_)
    return false;

  pal_detection_msgs::StopEnrollment srvMsg;

  if (palFaceEnrolStopClient_.call( srvMsg )) {
    ROS_INFO( "Pal face enrollment total %d", srvMsg.response.numFacesEnrolled );
    return srvMsg.response.enrollment_ok;
  }
  return false;
}

bool REEMProxyManager::getCurrentMapPOIs( std::vector<std::string> & poi_names, std::vector<RobotPose> & poi_pts )
{
  if (!mapConfigClient_.exists())
    return false;

  poi_names.clear();
  poi_pts.clear();
  pal_navigation_msgs::GetMapConfiguration srvMsg;

  if (mapConfigClient_.call( srvMsg )) {
    const pal_navigation_msgs::POI & pois = srvMsg.response.map_config.pois;
    size_t lsize = pois.ids.size();
    poi_names.resize( lsize );
    poi_pts.resize( lsize );
    for (size_t i = 0; i < lsize; ++i) {
      poi_names[i] = pois.ids[i].data;
      RobotPose pt = {pois.points.points[i].x, pois.points.points[i].y, pois.points.points[i].z};
      poi_pts[i] = pt;
    }
    return true;
  }
  else {
    return false;
  }
}

void REEMProxyManager::enablePalFaceDetection( bool enable, float confidence )
{
  if (!palFaceEnablerClient_.exists())
    return;

  pal_detection_msgs::Recognizer srvMsg;
  srvMsg.request.enabled = enable;
  srvMsg.request.minConfidence = confidence;
  palFaceEnablerClient_.call( srvMsg );
}

bool REEMProxyManager::setHandPosition( bool isLeftHand, std::vector<double> & positions, float time_to_reach )
{
  if (defaultMotionCtrl_) {
    ROS_WARN( "Default motion is executing." );
    return false;
  }

  if (positions.size() != 3) {
    return false;
  }

  if (isLeftHand) {
     if (!lhandClient_ || lHandCtrl_) {
       return false;
     }
     else {
       control_msgs::FollowJointTrajectoryGoal goal;
       //goal.trajectory.joint_names.push_back( "hand_left_index_1_joint" );
       //goal.trajectory.joint_names.push_back( "hand_left_index_2_joint" );
       //goal.trajectory.joint_names.push_back( "hand_left_index_3_joint" );
       goal.trajectory.joint_names.push_back( "hand_left_index_joint" );
       //goal.trajectory.joint_names.push_back( "hand_left_middle_1_joint" );
       //goal.trajectory.joint_names.push_back( "hand_left_middle_2_joint" );
       //goal.trajectory.joint_names.push_back( "hand_left_middle_3_joint" );
       goal.trajectory.joint_names.push_back( "hand_left_middle_joint" );
       goal.trajectory.joint_names.push_back( "hand_left_thumb_joint" );
       goal.trajectory.points.resize( 1 );

       goal.trajectory.points[0].positions.resize( 3 );
         // Velocities
       goal.trajectory.points[0].velocities.resize( 3 );

       for (size_t j = 0; j < 3; ++j) {
         goal.trajectory.points[0].positions[j] = positions[j];
         goal.trajectory.points[0].velocities[j] = 0.0;
       }
       // To be reached 2 seconds after starting along the trajectory
       goal.trajectory.points[0].time_from_start = ros::Duration( time_to_reach );
       lHandCtrl_ = true;
       lHandActionTimeout_ = time_to_reach * 1.05; // give additional 5% allowance
       lhandClient_->sendGoal( goal,
                             boost::bind( &REEMProxyManager::doneLHandAction, this, _1, _2 ),
                             FollowTrajectoryClient::SimpleActiveCallback(),
                             boost::bind( &REEMProxyManager::moveLHandActionFeedback, this, _1 ) );
     }
  }
  else {
     if (!rhandClient_ || rHandCtrl_) {
       return false;
     }
     else {
       control_msgs::FollowJointTrajectoryGoal goal;

       //goal.trajectory.joint_names.push_back( "hand_right_index_1_joint" );
       //goal.trajectory.joint_names.push_back( "hand_right_index_2_joint" );
       //goal.trajectory.joint_names.push_back( "hand_right_index_3_joint" );
       goal.trajectory.joint_names.push_back( "hand_right_index_joint" );
       //goal.trajectory.joint_names.push_back( "hand_right_middle_1_joint" );
       //goal.trajectory.joint_names.push_back( "hand_right_middle_2_joint" );
       //goal.trajectory.joint_names.push_back( "hand_right_middle_3_joint" );
       goal.trajectory.joint_names.push_back( "hand_right_middle_joint" );
       goal.trajectory.joint_names.push_back( "hand_right_thumb_joint" );
       goal.trajectory.points.resize( 1 );

       goal.trajectory.points[0].positions.resize( 3 );
         // Velocities
       goal.trajectory.points[0].velocities.resize( 3 );

       for (size_t j = 0; j < 3; ++j) {
         goal.trajectory.points[0].positions[j] = positions[j];
         goal.trajectory.points[0].velocities[j] = 0.0;
       }
       // To be reached 2 seconds after starting along the trajectory
       goal.trajectory.points[0].time_from_start = ros::Duration( time_to_reach );
       rHandCtrl_ = true;
       rHandActionTimeout_ = time_to_reach * 1.05; // give additional 5% allowance
       rhandClient_->sendGoal( goal,
                             boost::bind( &REEMProxyManager::doneRHandAction, this, _1, _2 ),
                             FollowTrajectoryClient::SimpleActiveCallback(),
                             boost::bind( &REEMProxyManager::moveRHandActionFeedback, this, _1 ) );
     }
  }
  return true;
}

void REEMProxyManager::updateHeadPos( float yaw, float pitch )
{
  if (headCtrlWithActionClient_ || headCtrlWithTrajActionClient_ || defaultMotionCtrl_)
    return;
  
  headYawRate_ = clamp( yaw, kHeadYawRate );
  headPitchRate_ = clamp( pitch, kHeadPitchRate );
  
  cmdTimeStamp_ = ros::Time::now();
}

bool REEMProxyManager::moveBodyTo( const RobotPose & pose, const float bestTime )
{
  if (bodyCtrlWithOdmetry_ || bodyCtrlWithNavigation_)
    return false;
  
  tflistener_.waitForTransform( "base_footprint", "odom",
                               ros::Time(0), ros::Duration( 1.0 ) );
  
  //we will record transforms here
  tflistener_.lookupTransform( "base_footprint", "odom",
                              ros::Time(0), startTransform_ );
  
  poseTrans_ = pose;
  while (poseTrans_.theta <= -M_PI) poseTrans_.theta += 2 * M_PI;
  while (poseTrans_.theta > M_PI) poseTrans_.theta -= 2 * M_PI;
  
  
  mCmd_.linear.x = clamp( poseTrans_.x / bestTime, kMaxWalkSpeed );
  mCmd_.linear.y = clamp( poseTrans_.y / bestTime, kMaxWalkSpeed );
  mCmd_.angular.z = clamp( poseTrans_.theta / bestTime, kYawRate );
  
  // calculate estimated finished time.
  double reqTime = 0.0;
  if (mCmd_.linear.x != 0.0)
    reqTime = poseTrans_.x / mCmd_.linear.x;
  if (mCmd_.linear.y != 0.0)
    reqTime = max( reqTime, poseTrans_.y / mCmd_.linear.y );
  if (mCmd_.angular.z != 0.0)
    reqTime = max( reqTime, poseTrans_.theta / mCmd_.angular.z );
  
  bcwoTimeToComplete_ = ros::Time::now() + ros::Duration( reqTime * 1.1 ); // give additional 10% allowance
  
   // for slow running simulator that we increase the speed
  mCmd_.linear.x *= 2.5;
  mCmd_.linear.y *= 2.5;
  mCmd_.angular.z *= 2.5;
  
  bodyCtrlWithOdmetry_ = true;
  return true;
}

void REEMProxyManager::updateBodyPose( const RobotPose & speed, bool localupdate )
{
  if (localupdate) {
    geometry_msgs::Twist mCmd;
    
    mCmd.linear.x = clamp( speed.x, kMaxWalkSpeed );
    mCmd.linear.y = clamp( speed.y, kMaxWalkSpeed );
    mCmd.angular.z = clamp( speed.theta, kYawRate );
    
    ROS_INFO( "REEM Body moving speed update." );
    
    mPub_.publish( mCmd );  // publish once
  }
  else {
    if (bodyCtrlWithOdmetry_)
      return;

    mCmd_.linear.x = clamp( speed.x * 0.3, kMaxWalkSpeed );
    //mCmd_.linear.y = clamp( speed.y, kMaxWalkSpeed );
    if (speed.y != 0.0) { // make y the priority for turning.
      mCmd_.angular.z = clamp( speed.y * 0.3, kYawRate );
    }
    else if (speed.theta != 0.0) {
      mCmd_.angular.z = clamp( speed.theta * 0.3, kYawRate );
    }
    
    cmdTimeStamp_ = ros::Time::now();
  }
}

bool REEMProxyManager::moveTorsoTo( double yaw, double pitch, bool relative, float time_to_reach )
{
  if (torsoCtrl_ || defaultMotionCtrl_ || !torsoClient_)
    return false;

  double newYaw, newPitch;
  this->getTorsoPos( reqTorsoYaw_, reqTorsoPitch_ );

  if (relative) {
    newYaw = clamp( reqTorsoYaw_ + yaw, kMaxTorsoPan );
    newPitch = reqTorsoPitch_ + pitch;
  }
  else {
    newYaw = clamp( yaw, kMaxTorsoPan );
    newPitch = pitch;
  }

  if (newPitch < kMinTorsoTilt) {
    newPitch = kMinTorsoTilt;
  }
  else if (newPitch > kMaxTorsoTilt) {
    newPitch = kMaxTorsoTilt;
  }

  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back( "torso_1_joint" );
  goal.trajectory.joint_names.push_back( "torso_2_joint" );

  goal.trajectory.points.resize( 1 );

  goal.trajectory.points[0].positions.resize( 2 );
	// Velocities
  goal.trajectory.points[0].velocities.resize( 2 );

  goal.trajectory.points[0].positions[0] = newYaw;
  goal.trajectory.points[0].velocities[0] = 0.0;
  goal.trajectory.points[0].positions[1] = newPitch;
  goal.trajectory.points[0].velocities[1] = 0.0;
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration( time_to_reach );

  torsoCtrl_ = true;

  torsoClient_->sendGoal( goal,
						  boost::bind( &REEMProxyManager::doneTorsoAction, this, _1, _2 ),
						  FollowTrajectoryClient::SimpleActiveCallback(),
						  FollowTrajectoryClient::SimpleFeedbackCallback() );

  return true;
}

bool REEMProxyManager::moveTorsoWithJointTrajectory( std::vector< std::vector<double> > & trajectory,
                                   std::vector<float> & times_to_reach )
{
  if (torsoCtrl_ || defaultMotionCtrl_ || !torsoClient_)
    return false;

  control_msgs::FollowJointTrajectoryGoal goal;

  goal.trajectory.joint_names.push_back( "torso_1_joint" );
  goal.trajectory.joint_names.push_back( "torso_2_joint" );

  goal.trajectory.points.resize( trajectory.size() );

  float time_to_reach_for_pt = 0.0;

  for (size_t jp = 0; jp < trajectory.size(); ++jp) {
    goal.trajectory.points[jp].positions.resize( 2 );
    // Velocities
    goal.trajectory.points[jp].velocities.resize( 2 );

    for (size_t j = 0; j < 2; ++j) {
      goal.trajectory.points[jp].positions[j] = trajectory[jp][j];
      goal.trajectory.points[jp].velocities[j] = 0.0;
    }
    time_to_reach_for_pt += times_to_reach[jp];
    goal.trajectory.points[jp].time_from_start = ros::Duration( time_to_reach_for_pt );
  }

  torsoCtrl_ = true;

  torsoClient_->sendGoal( goal,
              boost::bind( &REEMProxyManager::doneTorsoAction, this, _1, _2 ),
              FollowTrajectoryClient::SimpleActiveCallback(),
              FollowTrajectoryClient::SimpleFeedbackCallback() );

  return true;
}

void REEMProxyManager::jointStateDataCB( const sensor_msgs::JointStateConstPtr & msg )
{
  boost::mutex::scoped_lock lock( joint_mutex_, boost::try_to_lock );

  if (lock) {
    curJointNames_ = msg->name;
    curJointPositions_ = msg->position;
  }
}

/* \typedef onPowerPluggedChange(is_plugged_in)
 *  \memberof PyREEM.
 *  \brief Callback function when REEM power status changes.
 *  \param bool is_plugged_in. True if the robot is plugged in main power.
 *  \return None.
 *  \note Require low power threshold to be greater than zero.
 */
/*! \typedef onBatteryChargeChange(battery_status)
 *  \memberof PyREEM.
 *  \brief Callback function when REEM battery status changes.
 *  \param tuple battery_status. A tuple of (battery percentage,is_charging,is_battery_below_threshold).
 *  \return None.
 *  \note Require low power threshold to be greater than zero. charging status
 *  is not available on REEM, hence is_charging always return false.
 */
void REEMProxyManager::powerStateDataCB( const diagnostic_msgs::DiagnosticArrayConstPtr & msg )
{
  for (size_t i = 0; i < msg->status.size(); i++) {
    if (msg->status[i].name.compare( "/Hardware/Battery" ) != 0)
      continue;

    const diagnostic_msgs::DiagnosticStatus & batStatus = msg->status[i];
    for (size_t j = 0; j < batStatus.values.size(); j++) {
      if (batStatus.values[j].key.compare( "Battery Level" ) == 0) {
        boost::mutex::scoped_lock lock( bat_mutex_ );

        float batpercent = strtof( batStatus.values[j].value.c_str(), NULL );
        //batTimeRemain_ = msg->time_remaining;

        if (lowPowerThreshold_ > 0) {
          if (fabs(batpercent - batCapacity_) >= 1.0) {
            PyGILState_STATE gstate;
            gstate = PyGILState_Ensure();

            PyObject * arg = Py_BuildValue( "(fOO)", batpercent, batChargingState_ == CHARGING ? Py_True : Py_False,
                                  (batpercent < (float)lowPowerThreshold_ ? Py_True : Py_False) );

            PyREEMModule::instance()->invokeCallback( "onBatteryChargeChange", arg );
            Py_DECREF( arg );

            PyGILState_Release( gstate );
          }
        }
        batCapacity_ = batpercent;
      }
    }
  }
}

void REEMProxyManager::voltageStateDataCB( const sb04_power_board::PowerBoardConstPtr & msg )
{
  int curvol = msg->voltage;
  if (powerVoltage_ == -1) { // initialisation
    powerVoltage_ = curvol;
    return;
  }

  int voldiff = curvol - powerVoltage_;
  powerVoltage_ = curvol;

  if (voldiff > 300) { // sudden increase of voltage means we have put on main power
    {
      boost::mutex::scoped_lock lock( voltage_mutex_ );
      batChargingState_ = CHARGING;
    }
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    PyObject * arg = Py_BuildValue( "(O)", Py_True );

    PyREEMModule::instance()->invokeCallback( "onPowerPluggedChange", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
  else if (voldiff < -300) { // sudden decrease of voltage means we have unplugged in robot.
    {
      boost::mutex::scoped_lock lock( voltage_mutex_ );
      batChargingState_ = NOTCHARGING;
    }
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    PyObject * arg = Py_BuildValue( "(O)", Py_False );

    PyREEMModule::instance()->invokeCallback( "onPowerPluggedChange", arg );
    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
}

void REEMProxyManager::audioVolumeDataCB( const std_msgs::Int8ConstPtr & msg )
{
  audioVolume_ = msg->data;
}

void REEMProxyManager::palFaceDataCB( const pal_detection_msgs::FaceDetectionsConstPtr & msg )
{
  boost::recursive_mutex::scoped_lock lock( palfacesub_mutex_, boost::try_to_lock );

  if (!lock)
    return;

  size_t rsize = msg->faces.size();

  if (rsize == 0) {
    return;
  }

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * retList = PyList_New( rsize );

  for (size_t i = 0; i < rsize; i++) {
    const pal_detection_msgs::FaceDetection & face = msg->faces[i];

    PyObject * retObj = PyDict_New();
    PyObject * elemObj = PyString_FromString( face.name.c_str() );
    PyDict_SetItemString( retObj, "name", elemObj );
    Py_DECREF( elemObj );

    elemObj = PyFloat_FromDouble( face.confidence );
    PyDict_SetItemString( retObj, "confidence", elemObj );
    Py_DECREF( elemObj );

    elemObj = PyString_FromString( face.expression.c_str() );
    PyDict_SetItemString( retObj, "expression", elemObj );
    Py_DECREF( elemObj );

    elemObj = PyFloat_FromDouble( face.expression_confidence );
    PyDict_SetItemString( retObj, "express_confidence", elemObj );
    Py_DECREF( elemObj );

    elemObj = PyTuple_New( 4 );
    PyTuple_SetItem( elemObj, 0, PyInt_FromLong( face.x ) );
    PyTuple_SetItem( elemObj, 1, PyInt_FromLong( face.y ) );
    PyTuple_SetItem( elemObj, 2, PyInt_FromLong( face.width ) );
    PyTuple_SetItem( elemObj, 3, PyInt_FromLong( face.height ) );
    PyDict_SetItemString( retObj, "bound", elemObj );
    Py_DECREF( elemObj );

    PyList_SetItem( retList, i, retObj );
  }

  PyObject * arg = Py_BuildValue( "(O)", retList );

  PyREEMModule::instance()->invokePalFaceCallback( arg );

  Py_DECREF( arg );
  Py_DECREF( retList );

  PyGILState_Release( gstate );
}

void REEMProxyManager::legDataCB( const people_msgs::PositionMeasurementArrayConstPtr & msg )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  boost::recursive_mutex::scoped_lock lock( legsub_mutex_, boost::try_to_lock );

  if (!lock) {
    PyGILState_Release( gstate );
    return;
  }

  size_t rsize = msg->people.size();

  int count = 0;
  for (size_t i = 0; i < rsize; i++) {
    const people_msgs::PositionMeasurement & person = msg->people[i];

    if ((person.pos.x * person.pos.x + person.pos.y * person.pos.y) > legDetectDistance_) {
      // filter out person outside of specified distance
      continue;
    }
    count++;
  }

  PyObject * retList = PyList_New( count );
  count = 0;

  for (size_t i = 0; i < rsize; i++) {
    const people_msgs::PositionMeasurement & person = msg->people[i];

    if ((person.pos.x * person.pos.x + person.pos.y * person.pos.y) > legDetectDistance_) {
      // filter out person outside of specified distance
      continue;
    }
    PyObject * retObj = PyDict_New();
    PyObject * elemObj = PyString_FromString( person.object_id.c_str() );
    PyDict_SetItemString( retObj, "id", elemObj );
    Py_DECREF( elemObj );

    elemObj = PyFloat_FromDouble( person.reliability );
    PyDict_SetItemString( retObj, "confidence", elemObj );
    Py_DECREF( elemObj );

    elemObj = PyTuple_New( 2 );
    PyTuple_SetItem( elemObj, 0, PyFloat_FromDouble( person.pos.x ) );
    PyTuple_SetItem( elemObj, 1, PyFloat_FromDouble( person.pos.y ) );
    PyDict_SetItemString( retObj, "position", elemObj );
    Py_DECREF( elemObj );

    PyList_SetItem( retList, count++, retObj );
  }

  PyObject * arg = Py_BuildValue( "(O)", retList );

  PyREEMModule::instance()->invokeLegDetectCallback( arg );

  Py_DECREF( arg );
  Py_DECREF( retList );

  PyGILState_Release( gstate );
}

void REEMProxyManager::setLowPowerThreshold( int percent )
{
  if (percent >= 0 && percent < 100) {
    lowPowerThreshold_ = percent;
  }
}

void REEMProxyManager::getBatteryStatus( float & percentage, REEMChargingState & charging, float & timeremain )
{
  {
    boost::mutex::scoped_lock lock( bat_mutex_ );
    percentage = floorf(batCapacity_ * 100.0) / 100.0;
    timeremain = (float)batTimeRemain_.toSec();
  }
  {
    boost::mutex::scoped_lock lock( voltage_mutex_ );
    charging = batChargingState_;
  }
}

void REEMProxyManager::setTorsoStiffness( const float stiffness )
{
  if (stiffness < 0.0 || stiffness > 1.0)
    return;

  stiffCmd_.current_limits[22] = stiffness;
  stiffCmd_.current_limits[23] = stiffness;

  cPub_.publish( stiffCmd_ );
}

void REEMProxyManager::setArmStiffness( bool isLeftArm, const float stiffness )
{
  // TODO: to be implemented
  if (stiffness < 0.0 || stiffness > 1.0)
    return;

  int offset = 0;
  if (!isLeftArm) {
    offset = 7;
  }
  if (stiffCmd_.current_limits[offset] < 0.3 && stiffness > 0.5) { //we are turning on the stiffness
    std::vector<std::string> joints( 7 );
    std::vector<double> positions;

    if (isLeftArm) {
      joints[0] = "arm_left_1_joint";
      joints[1] = "arm_left_2_joint";
      joints[2] = "arm_left_3_joint";
      joints[3] = "arm_left_4_joint";
      joints[4] = "arm_left_5_joint";
      joints[5] = "arm_left_6_joint";
      joints[6] = "arm_left_7_joint";
    }
    else {
      joints[0] = "arm_right_1_joint";
      joints[1] = "arm_right_2_joint";
      joints[2] = "arm_right_3_joint";
      joints[3] = "arm_right_4_joint";
      joints[4] = "arm_right_5_joint";
      joints[5] = "arm_right_6_joint";
      joints[6] = "arm_right_7_joint";
    }
    this->getPositionForJoints( joints, positions );
    // set to this position
    this->moveArmWithJointPos( isLeftArm, positions, 0.5 );
    float mystiffness = stiffCmd_.current_limits[offset];
    float step = (stiffness - mystiffness) / 10.0;
    for (int j = 0; j < 10; j++) {
      for (int i = 0; i < 7; i++) {
        stiffCmd_.current_limits[offset+i] = mystiffness + step*j;
      }
      cPub_.publish( stiffCmd_ );
      //printf( "mystiffness %.3f\n", mystiffness + step*j );
      sleep( 0.1 );
    }
  }
  for (int i = 0; i < 7; i++) {
    stiffCmd_.current_limits[offset+i] = stiffness;
  }
  cPub_.publish( stiffCmd_ );
}

/*! \typedef onMoveBodySuccess()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.moveBodyTo method call is successful.
 *  \return None.
 */
/*! \typedef onMoveBodyFailed()
 *  \memberof PyREEM.
 *  \brief Callback function when PyREEM.moveBodyTo method call is failed.
 *  \return None.
 */
void REEMProxyManager::publishCommands()
{
  if (bodyCtrlWithOdmetry_) {
    mPub_.publish( mCmd_ );
    
    tf::StampedTransform curTransform;
    
    try {
      tflistener_.lookupTransform( "base_footprint", "odom",
                                ros::Time(0), curTransform );
    }
    catch (tf::TransformException ex) {
      ROS_ERROR( "%s",ex.what() );
      return;
    }
    // check translation
    tf::Transform relTransform = startTransform_.inverse() * curTransform;
    
    bool distReached = (relTransform.getOrigin().length2() >= (poseTrans_.x * poseTrans_.x + poseTrans_.y * poseTrans_.y));
    
    if (distReached) {
      mCmd_.linear.x = 0.0; mCmd_.linear.y = 0.0;
    }
    
    tf::Vector3 desired_turn_axis( 0, 0, 1 );

    // -ve theta is clockwise
    if (poseTrans_.theta > 0)
      desired_turn_axis = -desired_turn_axis;

    tf::Vector3 actual_turn_axis = relTransform.getRotation().getAxis();
    
    double angle_turned = relTransform.getRotation().getAngle();
    bool rotationReached = (poseTrans_.theta == 0.0);

    if (fabs(angle_turned) > 1.0e-2) {
      if (actual_turn_axis.dot( desired_turn_axis ) < 0)
        angle_turned = 2 * M_PI - angle_turned;

      rotationReached = (fabs(angle_turned) >= fabs(poseTrans_.theta));

    }
    
    if (rotationReached) {
      mCmd_.angular.z = 0.0;
    }

    bodyCtrlWithOdmetry_ = !(distReached && rotationReached); // set to false when we reach target. shouldn't have race conditions.
    if (!bodyCtrlWithOdmetry_) {
      PyGILState_STATE gstate;
      gstate = PyGILState_Ensure();
      
      PyREEMModule::instance()->invokeCallback( "onMoveBodySuccess", NULL );
      
      PyGILState_Release( gstate );
      
      ROS_INFO( "Move body action finished." );
    }
    else if (this->isBodyControlWithOdometryTimeExpired()) {
      bodyCtrlWithOdmetry_ = false;
      PyGILState_STATE gstate;
      gstate = PyGILState_Ensure();
      
      PyREEMModule::instance()->invokeCallback( "onMoveBodyFailed", NULL );
      
      PyGILState_Release( gstate );
      
      ROS_INFO( "Move body action failed." );
    }
  }
  else if (mCmd_.linear.x || mCmd_.linear.y || mCmd_.angular.z) {
    // check if we have recent command update from the client
    // if not, assume the worse and do not publish cached commmand
    if ((ros::Time::now() - cmdTimeStamp_).toSec() < kMotionCommandGapTolerance) {
      mPub_.publish( mCmd_ );
    }
    else {
      bool fire = false;
      if (fabs(mCmd_.linear.x) > 0.05) {
        mCmd_.linear.x *= 0.5;
        fire = true;
      }
      else {
        mCmd_.linear.x = 0.0;
      }
      if (fabs(mCmd_.linear.y) > 0.05) {
        mCmd_.linear.y *= 0.5;
        fire = true;
      }
      else {
        mCmd_.linear.y = 0.0;
      }      
      if (fabs(mCmd_.angular.z) > 0.05) {
        mCmd_.angular.z *= 0.5;
        fire = true;
      }
      else {
        mCmd_.angular.z = 0.0;
      }
      if (fire) {
        mPub_.publish( mCmd_ );
      }
      else {
        mCmd_.linear.x = mCmd_.linear.y = mCmd_.angular.z = 0.0;
      }
    }
  }
  
  if (headCtrlWithActionClient_ || headCtrlWithTrajActionClient_) {
    return;
  }
  else if (headYawRate_ != 0.0 || headPitchRate_ != 0.0) {
    if ((ros::Time::now() - cmdTimeStamp_).toSec() < kMotionCommandGapTolerance) {
      trajectory_msgs::JointTrajectory traj;
      traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
      traj.joint_names.push_back( "head_1_joint" );
      traj.joint_names.push_back( "head_2_joint" );
      traj.points.resize(1);
      traj.points[0].positions.push_back( reqHeadYaw_ + headYawRate_ * kHorizon );
      traj.points[0].velocities.push_back( headYawRate_ );
      traj.points[0].positions.push_back( reqHeadPitch_ + headPitchRate_ * kHorizon );
      traj.points[0].velocities.push_back( headPitchRate_ );
      traj.points[0].time_from_start = ros::Duration( kHorizon );

      hPub_.publish( traj );

      // Updates the current positions
      reqHeadYaw_ += headYawRate_ * kDT;
      reqHeadYaw_ = max(min(reqHeadYaw_, kMaxHeadPan), -kMaxHeadPan);
      reqHeadPitch_ += headPitchRate_ * kDT;
      reqHeadPitch_ = max(min(reqHeadPitch_, kMaxHeadTilt), kMinHeadTilt);      
    }
  }
}

bool REEMProxyManager::isBodyControlWithOdometryTimeExpired()
{
  return (ros::Time::now() >= bcwoTimeToComplete_);
}

void REEMProxyManager::getTFFrameList( std::vector<std::string> & list )
{
  list.clear();
  for (int i = 0; i < kREEMTFFrameListSize - 1 ; ++i) {
    list.push_back( kREEMTFFrameList[i] );
  }
}
  
bool REEMProxyManager::isTFFrameSupported( const char * frame_name )
{
  if (!frame_name || strlen( frame_name ) == 0 || strlen( frame_name ) > 40) { //rudimentary check
    return false;
  }
  int i = 0;
  while (kREEMTFFrameList[i]) {
    if (strncmp( frame_name, kREEMTFFrameList[i], strlen( kREEMTFFrameList[i]) ) == 0) {
      return true;
    }
    i++;
  }
  return false;
}

bool REEMProxyManager::addSolidObject( const std::string & name, std::vector<double> & volume,
    std::vector<double> & position, std::vector<double> & orientation )
{
  if (!rarmGroup_ || !larmGroup_) {
    ROS_ERROR( "addSolidObject: MoveIt is not in use." );
    return false;
  }

  if (findSolidObjectInScene( name )) {
    ROS_ERROR( "addSolidObject: Solid object %s already exists in the scene.", name.c_str() );
    return false;
  }

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_footprint";

  co.id = name;
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize( shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value );
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = volume[0];
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = volume[1];
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = volume[2];

  co.primitive_poses.resize( 1 );
  co.primitive_poses[0].position.x = position[0];
  co.primitive_poses[0].position.y = position[1];
  co.primitive_poses[0].position.z = position[2];
  co.primitive_poses[0].orientation.w = orientation[0];
  co.primitive_poses[0].orientation.x = orientation[1];
  co.primitive_poses[0].orientation.y = orientation[2];
  co.primitive_poses[0].orientation.z = orientation[3];

  colObjPub_.publish( co );
  return true;
}

void REEMProxyManager::removeSolidObject( const std::string & name )
{
  if (!rarmGroup_ || !larmGroup_) {
    ROS_ERROR( "removeSolidObject: MoveIt is not in use." );
    return;
  }

  if (!findSolidObjectInScene( name )) {
    ROS_ERROR( "removeSolidObject: Solid object %s does not exist in the scene.", name.c_str() );
    return;
  }

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_footprint";

  co.id = name;
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  colObjPub_.publish(co);
}

void REEMProxyManager::listSolidObjects( std::vector<std::string> & list )
{
  list.clear();
  size_t ssize = solidObjectsInScene_.size();
  for (int i = 0; i < ssize ; ++i) {
    list.push_back( solidObjectsInScene_[i] );
  }
}

bool REEMProxyManager::pickupObject( const std::string & name, const std::string & place, std::vector<double> & grasp_pose,
    bool isLeftArm, double approach_dist )
{
  if (!rarmGroup_ || !larmGroup_) {
    ROS_ERROR( "pickupObject: MoveIt is not in use." );
    return false;
  }

  if (!findSolidObjectInScene( name ) || !findSolidObjectInScene( place )) {
    ROS_ERROR( "pickupObject: Solid object %s and/or %s do not exist in the scene.",
        name.c_str(), place.c_str() );
    return false;
  }

  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;

  p.header.frame_id = "base_footprint";
  p.pose.position.x = grasp_pose[0];
  p.pose.position.y = grasp_pose[1];
  p.pose.position.z = grasp_pose[2];

  p.pose.orientation.w = grasp_pose[3];
  p.pose.orientation.x = grasp_pose[4];
  p.pose.orientation.y = grasp_pose[5];
  p.pose.orientation.z = grasp_pose[6];

  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.min_distance = approach_dist / 2.0;
  g.pre_grasp_approach.desired_distance = approach_dist;
  g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = approach_dist / 2.0;
  g.post_grasp_retreat.desired_distance = approach_dist;
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(1);
  g.pre_grasp_posture.points[0].positions[0] = 1;
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(1);
  g.grasp_posture.points[0].positions[0] = 0;

  g.pre_grasp_approach.direction.vector.x = 1.0;
  g.pre_grasp_approach.min_distance = approach_dist / 2.0;
  g.pre_grasp_approach.desired_distance = approach_dist;

  if (isLeftArm) {
    g.pre_grasp_approach.direction.header.frame_id = "l_wrist_roll_link";
    g.pre_grasp_posture.joint_names.resize( 1, "l_hand_joint" );
    g.grasp_posture.joint_names.resize( 1, "l_hand_joint" );
  }
  else {
    g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
    g.pre_grasp_posture.joint_names.resize( 1, "r_hand_joint" );
    g.grasp_posture.joint_names.resize( 1, "r_hand_joint" );
  }

  grasps.push_back( g );

  bool success = false;

  if (isLeftArm) {
    larmGroup_->setPlanningTime( 20.0 ); //TODO make it more flexible
    larmGroup_->setSupportSurfaceName( place );
    success = larmGroup_->pick( name, grasps );
  }
  else {
    rarmGroup_->setPlanningTime( 20.0 ); //TODO make it more flexible
    rarmGroup_->setSupportSurfaceName( place );
    success = rarmGroup_->pick( name, grasps );
  }
  return success;
}

bool REEMProxyManager::placeObject( const std::string & name, const std::string & place, std::vector<double> & place_pose,
    bool isLeftArm, double approach_dist )
{
  if (!rarmGroup_ || !larmGroup_) {
    ROS_ERROR( "placeObject: MoveIt is not in use." );
    return false;
  }

  if (!findSolidObjectInScene( name ) || !findSolidObjectInScene( place )) {
    ROS_ERROR( "placeObject: Solid object %s and/or %s do not exist in the scene.",
        name.c_str(), place.c_str() );
    return false;
  }

  std::vector<moveit_msgs::PlaceLocation> location;
  geometry_msgs::PoseStamped p;

  p.header.frame_id = "base_footprint";
  p.pose.position.x = place_pose[0];
  p.pose.position.y = place_pose[1];
  p.pose.position.z = place_pose[2];

  p.pose.orientation.w = place_pose[3];
  p.pose.orientation.x = place_pose[4];
  p.pose.orientation.y = place_pose[5];
  p.pose.orientation.z = place_pose[6];

  moveit_msgs::PlaceLocation g;
  g.place_pose = p;

  g.pre_place_approach.direction.vector.z = -1.0;
  g.post_place_retreat.direction.vector.x = -1.0;
  g.post_place_retreat.direction.header.frame_id = "base_footprint";
  g.pre_place_approach.min_distance = approach_dist / 2.0;
  g.pre_place_approach.desired_distance = approach_dist;
  g.post_place_retreat.min_distance = approach_dist / 2.0;
  g.post_place_retreat.desired_distance = approach_dist;

  g.post_place_posture.points.resize(1);
  g.post_place_posture.points[0].positions.resize(1);
  g.post_place_posture.points[0].positions[0] = 1;

  if (isLeftArm) {
    g.pre_place_approach.direction.header.frame_id = "l_wrist_roll_link";
    g.post_place_posture.joint_names.resize( 1, "l_hand_joint" );
  }
  else {
    g.pre_place_approach.direction.header.frame_id = "r_wrist_roll_link";
    g.post_place_posture.joint_names.resize( 1, "r_hand_joint" );
  }
  location.push_back( g );

  // add path constraints
  moveit_msgs::Constraints constr;
  constr.orientation_constraints.resize( 1 );
  moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
  ocm.header.frame_id = p.header.frame_id;
  ocm.orientation.x = 0.0;
  ocm.orientation.y = 0.0;
  ocm.orientation.z = 0.0;
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;

  bool success = false;

  if (isLeftArm) {
    ocm.link_name = "l_wrist_roll_link";
    larmGroup_->setPlanningTime( 20.0 ); //TODO make it more flexible
    larmGroup_->setSupportSurfaceName( place );
    larmGroup_->setPathConstraints( constr );

    larmGroup_->setPlannerId( "RRTConnectkConfigDefault" );
    success = larmGroup_->place( name, location );
  }
  else {
    ocm.link_name = "r_wrist_roll_link";
    rarmGroup_->setPlanningTime( 20.0 ); //TODO make it more flexible
    rarmGroup_->setSupportSurfaceName( place );
    rarmGroup_->setPathConstraints( constr );

    rarmGroup_->setPlannerId( "RRTConnectkConfigDefault" );
    success = rarmGroup_->place( name, location );
  }
  return success;
}

bool REEMProxyManager::findSolidObjectInScene( const std::string & name )
{
  bool found = false;
  size_t ssize = solidObjectsInScene_.size();
  for (size_t i = 0; i < ssize; ++i) {
    found = (name.compare( solidObjectsInScene_[i] ) == 0);
    if (found)
      break;
  }
  return found;
}

void REEMProxyManager::directToWeb( const std::string & uri )
{
  pal_web_msgs::WebGoTo msg;
  msg.type = 2;
  msg.value = uri;
  wPub_.publish( msg );
}

void REEMProxyManager::setRobotPoseInMap( const std::vector<double> & positions,
        const std::vector<double> & orientation )
{
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  msg.pose.pose.position.x = positions[0];
  msg.pose.pose.position.y = positions[1];
  msg.pose.pose.position.z = positions[2];
  msg.pose.pose.orientation.w = orientation[0];
  msg.pose.pose.orientation.x = orientation[1];
  msg.pose.pose.orientation.y = orientation[2];
  msg.pose.pose.orientation.z = orientation[3];
  // hardcoded covariance
  boost::array<double, 36> dta = {{0.25, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.25, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0,
          0.06853891945200942}};
  msg.pose.covariance = dta;
  pPub_.publish( msg );
}

void REEMProxyManager::sendNodeMessage( const std::string & node, const std::string & command, const int priority )
{
  pyride_common_msgs::NodeMessage msg;
  msg.header.stamp = ros::Time::now();
  msg.node_id = node;
  msg.priority = priority;
  msg.command = command;

  bPub_.publish( msg );
}

std_msgs::ColorRGBA REEMProxyManager::colour2RGB( const REEMLedColour colour )
{
  std_msgs::ColorRGBA rgba;
  rgba.a = 1.0;
  switch (colour) {
    case BLANK:
      rgba.r = rgba.g = rgba.b = 0.0;
      break;
    case WHITE:
      rgba.r = rgba.g = rgba.b = 1.0;
      break;
    case RED:
      rgba.r = 1.0;
      rgba.g = rgba.b = 0.0;
      break;
    case BLUE:
      rgba.r = rgba.g = 0.0;
      rgba.b = 1.0;
      break;
    case GREEN:
      rgba.r = rgba.b = 0.0;
      rgba.g = 1.0;
      break;
    case YELLOW:
      rgba.r = rgba.g = 1.0;
      rgba.b = 0.0;
      break;
    case PINK:
      rgba.r = 1.0; rgba.g = 192.0/255.0;
      rgba.b = 203.0/255.0;
      break;
    default:
      break;
  }
  return rgba;
}

void REEMProxyManager::initMotorStiffnessValue()
{
  static std::string acnames [] = { "arm_left_1_motor", "arm_left_2_motor", "arm_left_3_motor",
      "arm_left_4_motor", "arm_left_5_motor", "arm_left_6_motor", "arm_left_7_motor",
      "arm_right_1_motor", "arm_right_2_motor", "arm_right_3_motor", "arm_right_4_motor",
      "arm_right_5_motor", "arm_right_6_motor", "arm_right_7_motor", "hand_left_thumb_motor",
      "hand_left_index_motor", "hand_left_middle_motor", "hand_right_thumb_motor",
      "hand_right_index_motor", "hand_right_middle_motor", "head_1_motor", "head_2_motor",
      "torso_1_motor", "torso_2_motor"
  };
  stiffCmd_.actuator_names = std::vector<std::string> (acnames , acnames + 24);
  size_t asize = stiffCmd_.actuator_names.size();
  stiffCmd_.current_limits.resize( asize );
  for (size_t i = 0; i < asize; i++) {
    stiffCmd_.current_limits[i] = 1.0;
  }
}
/**@}*/
} // namespace pyride
