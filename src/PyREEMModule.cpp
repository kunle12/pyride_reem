/*
 *  PyREEMModule.cpp
 *  
 */

#include <pthread.h>
#include <string>
#include "PyREEMModule.h"
#include "REEMProxyManager.h"

namespace pyride {

#define PYRIDE_ROBOT_MODEL  "REEM"

PyDoc_STRVAR( PyREEM_doc, \
             "PyREEM is the main Python extension module of PyRIDE on REEM/ROS system." );

/*! \class PyREEM
 *  \brief PyREEM is the main Python extension module of PyRIDE on REEM/ROS system.
 *
 *  PyREEM module consists of a set of callable Python methods specifically related to
 *  REEM low level functionalities and a set of callback functions that should be implemented
 *  by REEM programmers.
 */
PyREEMModule * PyREEMModule::s_pyREEMModule = NULL;

static const char *kLeftArmKWlist[] = { "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint",
    "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint", "time_to_reach", NULL };
static const char *kRightArmKWlist[] = { "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint",
    "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint", "time_to_reach", NULL };

static const char *kHeadKWlist[] = { "head_1_joint", "head_2_joint", NULL };

static const char *kTorsoKWlist[] = { "torso_1_joint", "torso_2_joint", NULL };

static const char *kLeftHandKWlist[] = { "hand_left_index_joint", "hand_left_middle_joint", "hand_left_thumb_joint" };
static const char *kRightHandKWlist[] = { "hand_right_index_joint", "hand_right_middle_joint", "hand_right_thumb_joint" };
;
static const char *kPoseKWlist[] = { "position", "orientation", NULL };
static const char *kPickAndPlaceKWlist[] = { "name", "place", "grasp_position", "grasp_orientation", "use_left_arm", "distance_from", NULL };
static const char *kObjectKWlist[] = { "name", "volume", "position", "orientation", NULL };
static const char *kArmPoseKWlist[] = { "position", "orientation", "use_left_arm", "time_to_reach", NULL };

static bool colourStr2ID( const char * colourStr, REEMLedColour & colourID )
{
  if (!colourStr)
    return false;

  if (!strcasecmp( colourStr, "red" )) {
    colourID = RED;
  }
  else if (!strcasecmp( colourStr, "green" )) {
    colourID = GREEN;
  }
  else if (!strcasecmp( colourStr, "blue" )) {
    colourID = BLUE;
  }
  else if (!strcasecmp( colourStr, "white" )) {
    colourID = WHITE;
  }
  else if (!strcasecmp( colourStr, "yellow" )) {
    colourID = YELLOW;
  }
  else if (!strcasecmp( colourStr, "pink" )) {
    colourID = PINK;
  }
  else if (!strcasecmp( colourStr, "blank" )) {
    colourID = BLANK;
  }
  else {
    return false;
  }
  return true;
}

static PyObject * PyModule_write( PyObject *self, PyObject * args )
{
  char * msg;
  std::string outputMsg;

  if (!PyArg_ParseTuple( args, "s", &msg )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  // Next send it to all (active) clients.
  while (*msg) {
    if (*msg == '\n')
      outputMsg += "\r\n";
    else
      outputMsg += *msg;
    msg++;
  }
  
  PyREEMModule::instance()->write( outputMsg.c_str() );
  
  Py_RETURN_NONE;
}

/** @name Miscellaneous Functions
 *
 */
/**@{*/
/*! \fn startDataRecording( data_type, output_file )
 *  \brief Start to record various sensor data into a ROS bag file.
 *  \memberof PyREEM
 *  \param int data_type. Types of sensor data to be recorded. Different sensor datatype can be concate together with '|'.
 *  \param str output_file. Optional output file name path.
 *  \return None.
 *  \note Check constants.py for sensor data types.
 *  \note Default output file name path is in the format of /removable/recordings/%Y%m%d_%H%M_%sensordatatype_data.bag.
 */

/*! \fn stopDataRecording()
 *  \brief Stop recording REEM sensor data.
 *  \memberof PyREEM
 *  \return None.
 */
static PyObject * PyModule_SetTeamMemberID( PyObject *self, PyObject * args )
{
  int teamID, teamColour;
  if (!PyArg_ParseTuple( args, "ii", &teamID, &teamColour )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (teamID < 0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.setTeamMemberID: invalid "
                 "team member ID %d!", teamID );
    
    return NULL;
  }
  if (teamColour < BlueTeam || teamColour > PinkTeam) {
    PyErr_Format( PyExc_ValueError, "PyREEM.setTeamMemberID: invalid "
                 "team colour %d! Valid format: %d = Pink; %d = Blue", teamColour, PinkTeam, BlueTeam );

    return NULL;
  }

  ServerDataProcessor::instance()->setTeamMemberID( teamID, (TeamColour)teamColour );
  PyREEMModule::instance()->clientID( ServerDataProcessor::instance()->clientID() );

  Py_RETURN_NONE;
}

static PyObject * PyModule_sendTeamMessage( PyObject *self, PyObject * args )
{
  char * dataStr = NULL;
  
  if (!PyArg_ParseTuple( args, "s", &dataStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  PyREEMModule::instance()->sendTeamMessage( dataStr );
  Py_RETURN_NONE;
}

static PyObject * PyModule_REEMSayWithVolume( PyObject * self, PyObject * args )
{
  float volume = 0.0;
  char * dataStr = NULL;
  PyObject * toBlockObj = NULL;
  
  if (!PyArg_ParseTuple( args, "s|fO", &dataStr, &volume, &toBlockObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (toBlockObj && !PyBool_Check( toBlockObj )) {
    PyErr_Format( PyExc_ValueError, "PyREEM.say: third parameter must be a boolean!" );
    return NULL;
  }
  if (volume < 0.0 || volume > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.say: invalid voice volume!" );
    return NULL;
  }
  if (dataStr) {
    REEMProxyManager::instance()->sayWithVolume( string( dataStr ), volume,
                                               (toBlockObj && PyObject_IsTrue( toBlockObj )) );
  }
  Py_RETURN_NONE;
}

/*! \fn getBatteryStatus()
 *  \memberof PyREEM
 *  \brief Return the current robot battery status.
 *  \return tuple(battery percentage, is_plugged_in, estimated remaining battery time).
 *  \note REEM does not supply charging status and battery remaining estimate, hence
 *  the last two values are default dummy values.
 */
static PyObject * PyModule_REEMGetBatteryStatus( PyObject * self )
{
  float batpercent = 0;
  bool isplugged = false;
  float time_remain = 0.0;
  
  REEMProxyManager::instance()->getBatteryStatus( batpercent, isplugged, time_remain );
  
  return Py_BuildValue( "(fsf)", batpercent, isplugged ? "plugged in" :
                       "unplugged", time_remain );
}

/*! \fn getLowPowerThreshold()
 *  \memberof PyREEM
 *  \brief Return the current low power threshold
 *  \return Battery percentage integer.
 */
static PyObject * PyModule_REEMGetLowPowerThreshold( PyObject * self )
{
  return Py_BuildValue( "i", REEMProxyManager::instance()->getLowPowerThreshold() );
}

/*! \fn setLowPowerThreshold(battery_percentage)
 *  \memberof PyREEM
 *  \brief Set the low power threshold.
 *
 *  When the threshold is greater than zero
 *  this level, callback PyREEM.onBatteryChargeChange or PyREEM.onPowerPluggedChange
 *  will be invoked when battery status changes.
 *  \param int battery_percentage. Must be non-negative.
 *  \return None.
 */
/**@}*/
static PyObject * PyModule_REEMSetLowPowerThreshold( PyObject * self, PyObject * args )
{
  int low_threshold = 0;
  
  if (!PyArg_ParseTuple( args, "i", &low_threshold )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (low_threshold < 0 || low_threshold >= 100) {
    PyErr_Format( PyExc_ValueError, "PyREEM.setLowPowerThreshold: input must be an integer within [0..100)!" );
    return NULL;
  }
  REEMProxyManager::instance()->setLowPowerThreshold( low_threshold );
  Py_RETURN_NONE;
}

/*! \fn listTFFrames()
 *  \memberof PyREEM
 *  \brief Return a list of supported REEM TF frame names.
 *  \return list(frame names).
 */
static PyObject * PyModule_REEMListTFFrames( PyObject * self )
{
  std::vector<std::string> framelist;
  
  REEMProxyManager::instance()->getTFFrameList( framelist );
  int fsize = (int)framelist.size();
  PyObject * retObj = PyList_New( fsize );
  for (int i = 0; i < fsize; ++i) {
    PyList_SetItem( retObj, i, PyString_FromString( framelist[i].c_str() ) );
  }
  return retObj;
}

/*! \fn isSupportedTFFrame()
 *  \memberof PyREEM
 *  \brief Check whether the input TF frame is supported in the current system.
 *  \param string frame. Name of the TF frame.
 *  \return bool. True == supported; False == not supported.
 */
static PyObject * PyModule_REEMCheckTFFrame( PyObject * self, PyObject * args )
{
  char * framename = NULL;

  if (!PyArg_ParseTuple( args, "s", &framename )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (REEMProxyManager::instance()->isTFFrameSupported( framename ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn useMoveIt()
 *  \memberof PyREEM
 *  \brief Return whether MoveIt is in use.
 *  \return list(frame names).
 */
static PyObject * PyModule_REEMUseMoveIt( PyObject * self )
{
  if (REEMProxyManager::instance()->useMoveIt())
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveTorsoTo(pan, pitch, relative, time_to_reach)
 *  \memberof PyREEM
 *  \brief Move the REEM torso to a specific yaw and pitch position.
 *  \param float pan. Torso pan, must be in radian.
 *  \param float pitch. Torso pitch must be in radian.
 *  \param bool relative. True == relative angle values; False == absolute angle values. Optional, default is False.
 *  \param float time_to_reach. Time to reach the target position. Optional, default = 2.0 (seconds).
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_REEMMoveTorsoTo( PyObject * self, PyObject * args )
{
  double yaw = 0.0;
  double pitch = 0.0;
  double time_to_reach = 2.0;
  PyObject * boolObj = NULL;
  bool isRelative = false;
  
  if (!PyArg_ParseTuple( args, "dd|Od", &yaw, &pitch, &boolObj, &time_to_reach )) {
	// PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (boolObj) {
    if (PyBool_Check( boolObj )) {
      isRelative = PyObject_IsTrue( boolObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyREEM.moveTorsoTo: relative input parameter must be a boolean." );
      return NULL;
    }
  }
  if (time_to_reach < 0.5) {
    PyErr_Format( PyExc_ValueError, "PyREEM.moveTorsoTo: time to reach input parameter must greater than 0.5." );
    return NULL;
  }
  if (REEMProxyManager::instance()->moveTorsoTo( yaw, pitch, isRelative, time_to_reach ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveTorsoWithJointTrajectory(joint_trajectory)
 *  \memberof PyREEM
 *  \brief Move REEM torso to a sequence of waypoints, i.e. joint trajectory.
 *  \param list joint_trajectory. A list of waypoints that contain joint position dictionaries of {'torso_1_joint','torso_2_joint'}.
 *  \return None.
 *  \warning no joint value validation
 */
static PyObject * PyModule_REEMMoveTorsoWithJointTraj( PyObject * self, PyObject * args )
{
  PyObject * trajObj = NULL;

  if (!PyArg_ParseTuple( args, "O", &trajObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  int listSize = 0;

  if (!PyList_Check( trajObj ) || (listSize = PyList_Size( trajObj )) == 0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.moveTorsoWithJointTrajectory: input parameter must be a non empty list of dictionary!" );
    return NULL;
  }

  PyObject * jointPos = NULL;
  PyObject * jval = NULL;

  std::vector< std::vector<double> > trajectory;
  std::vector<float> times_to_reach( listSize, 2.0 ); // default to 2 seconds;

  for (int i = 0; i < listSize; ++i) {
    jointPos = PyList_GetItem( trajObj, i );
    if (!PyDict_Check( jointPos ) || PyDict_Size( jointPos ) < 2) {
      PyErr_Format( PyExc_ValueError, "PyREEM.moveTorsoWithJointTrajectory: input list item %d "
                   "must be a dictionary containing all 2 joint entries for REEM torso!", i );
      return NULL;
    }

    std::vector<double> head_joint_pos( 2, 0.0 );

    for (int k = 0; k < 2; k++) {
      jval = PyDict_GetItemString( jointPos,  kTorsoKWlist[k] );
      if (!jval) {
        PyErr_Format( PyExc_ValueError, "PyREEM.moveTorsoWithJointTrajectory: input list item %d has"
                     " missing %s joint value!", i, kTorsoKWlist[k] );
        return NULL;
      }
      if (!PyFloat_Check( jval )) {
        PyErr_Format( PyExc_ValueError, "PyREEM.moveTorsoWithJointTrajectory: input list item %d has"
                     " invalid %s joint values!", i, kTorsoKWlist[k] );
        return NULL;
      }
      head_joint_pos[k] = PyFloat_AsDouble( jval );
    }
    trajectory.push_back( head_joint_pos );
    jval = PyDict_GetItemString( jointPos, "time_to_reach" );
    if (jval && PyFloat_Check( jval )) {
      times_to_reach[i] = (float)PyFloat_AsDouble( jval );
    }
  }

  REEMProxyManager::instance()->moveTorsoWithJointTrajectory( trajectory, times_to_reach );
  Py_RETURN_NONE;
}

/*! \fn moveBodyTo(x,y,theta,best_time)
 *  \memberof PyREEM
 *  \brief Move the REEM body to a pose at (x,y,theta).
 *  \param float x. X coordinate w.r.t. the current pose.
 *  \param float y. Y coordinate w.r.t. the current pose.
 *  \param float theta. Angular position w.r.t. the current pose.
 *  \param float best_time. Optional, ask the robot try its best to reach the input pose in this timeframe.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_REEMMoveBodyTo( PyObject * self, PyObject * args )
{
  float xcoord = 0.0;
  float ycoord = 0.0;
  float theta = 0.0;
  float bestTime = 5.0; //seconds
  
  if (!PyArg_ParseTuple( args, "fff|f", &xcoord, &ycoord, &theta, &bestTime )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  RobotPose pose;
  pose.x = xcoord;
  pose.y = ycoord;
  pose.theta = theta;

  if (REEMProxyManager::instance()->moveBodyTo( pose, bestTime ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveBodyWithSpeed(x,y,theta)
 *  \memberof PyREEM
 *  \brief Set the REEM body moving speed in the form of (x,y,theta).
 *  \param float x. Speed(m/s) in X axis direction.
 *  \param float y. Speed(m/s) in Y axis direction.
 *  \param float theta. Angular turning rate (rad/s) w.r.t Z axis.
 *  \return None.
 *  \note Speed is restricted to maximum of 1 m/s in X/Y direction and 0.7rad/s in turning rate.
 *  \warning This is a low level control function and no safty check is provided.
 *  Programmers must ensure REEM robot movement is controlled safely.
 */
static PyObject * PyModule_REEMMoveBodyWithSpeed( PyObject * self, PyObject * args )
{
  float xcoord = 0.0;
  float ycoord = 0.0;
  float theta = 0.0;
  
  if (!PyArg_ParseTuple( args, "fff", &xcoord, &ycoord, &theta )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  RobotPose pose;
  pose.x = xcoord;
  pose.y = ycoord;
  pose.theta = theta;
  
  REEMProxyManager::instance()->updateBodyPose( pose, true );
  Py_RETURN_NONE;
}

static PyObject * PyModule_REEMUpdateHeadPos( PyObject * self, PyObject * args )
{
  double yaw = 0.0;
  double pitch = 0.0;

  if (!PyArg_ParseTuple( args, "dd", &yaw, &pitch )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  REEMProxyManager::instance()->updateHeadPos( yaw, pitch );
  Py_RETURN_NONE;
}

/*! \fn moveHeadTo(pan, pitch, relative, time_to_reach)
 *  \memberof PyREEM
 *  \brief Move the REEM head to a specific yaw and pitch position.
 *  \param float pan. Head pan, must be in radian.
 *  \param float pitch. Head pitch, must be in radian.
 *  \param bool relative. True == relative angle values; False == absolute angle values. Optional, default is False.
 *  \param float time_to_reach. Time to reach the target position. Optional, default = 0.5 (seconds).
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_REEMMoveHeadTo( PyObject * self, PyObject * args )
{
  double yaw = 0.0;
  double pitch = 0.0;
  double time_to_reach = 0.5;
  PyObject * boolObj = NULL;
  bool isRelative = false;
  
  if (!PyArg_ParseTuple( args, "dd|Od", &yaw, &pitch, &boolObj, &time_to_reach )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (boolObj) {
    if (PyBool_Check( boolObj )) {
      isRelative = PyObject_IsTrue( boolObj );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyREEM.moveHeadTo: last input parameter must be a boolean." );
      return NULL;
    }
  }

  if (time_to_reach < 0.2) {
    PyErr_Format( PyExc_ValueError, "PyREEM.moveHeadTo: time to reach must be >= 0.2." );
    return NULL;
  }

  if (REEMProxyManager::instance()->moveHeadTo( yaw, pitch, isRelative, time_to_reach ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveHeadWithJointTrajectory(joint_trajectory)
 *  \memberof PyREEM
 *  \brief Move REEM head to a sequence of waypoints, i.e. joint trajectory.
 *  \param list joint_trajectory. A list of waypoints that contain joint position dictionaries of {'head_1_joint','head_2_joint'}.
 *  \return None.
 *  \warning no joint value validation
 */
static PyObject * PyModule_REEMMoveHeadWithJointTraj( PyObject * self, PyObject * args )
{
  PyObject * trajObj = NULL;

  if (!PyArg_ParseTuple( args, "O", &trajObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  int listSize = 0;

  if (!PyList_Check( trajObj ) || (listSize = PyList_Size( trajObj )) == 0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.moveHeadWithJointTrajectory: input parameter must be a non empty list of dictionary!" );
    return NULL;
  }

  PyObject * jointPos = NULL;
  PyObject * jval = NULL;

  std::vector< std::vector<double> > trajectory;
  std::vector<float> times_to_reach( listSize, 2.0 ); // default to 2 seconds;

  for (int i = 0; i < listSize; ++i) {
    jointPos = PyList_GetItem( trajObj, i );
    if (!PyDict_Check( jointPos ) || PyDict_Size( jointPos ) < 2) {
      PyErr_Format( PyExc_ValueError, "PyREEM.moveHeadWithJointTrajectory: input list item %d "
                   "must be a dictionary containing all 2 joint entries for REEM head!", i );
      return NULL;
    }

    std::vector<double> head_joint_pos( 2, 0.0 );

    for (int k = 0; k < 2; k++) {
      jval = PyDict_GetItemString( jointPos,  kHeadKWlist[k] );
      if (!jval) {
        PyErr_Format( PyExc_ValueError, "PyREEM.moveHeadWithJointTrajectory: input list item %d has"
                     " missing %s joint value!", i, kHeadKWlist[k] );
        return NULL;
      }
      if (!PyFloat_Check( jval )) {
        PyErr_Format( PyExc_ValueError, "PyREEM.moveHeadWithJointTrajectory: input list item %d has"
                     " invalid %s joint values!", i, kHeadKWlist[k] );
        return NULL;
      }
      head_joint_pos[k] = PyFloat_AsDouble( jval );
    }
    trajectory.push_back( head_joint_pos );
    jval = PyDict_GetItemString( jointPos, "time_to_reach" );
    if (jval && PyFloat_Check( jval )) {
      times_to_reach[i] = (float)PyFloat_AsDouble( jval );
    }
  }

  REEMProxyManager::instance()->moveHeadWithJointTrajectory( trajectory, times_to_reach );
  Py_RETURN_NONE;
}

/*! \fn pointHeadTo(reference_frame, x, y, z)
 *  \memberof PyREEM
 *  \brief Point the REEM head towards a specific coordinates in a reference frame with respect to 'stereo_link'.
 *  \param str reference_frame. Text label for the requested reference frame (TF frame name).
 *  \param float x. X coordinate.
 *  \param float y. Y coordinate.
 *  \param float z. Z coordinate.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_REEMPointHeadTo( PyObject * self, PyObject * args )
{
  float xcoord = 0.0;
  float ycoord = 0.0;
  float zcoord = 0.0;
  char * reqframe = NULL;

  if (!PyArg_ParseTuple( args, "sfff", &reqframe, &xcoord, &ycoord, &zcoord )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (!REEMProxyManager::instance()->isTFFrameSupported( reqframe )) {
    PyErr_Format( PyExc_ValueError, "PyREEM.pointHeadTo: requested reference frame is not supported." );
    return NULL;
  }

  if (REEMProxyManager::instance()->pointHeadTo( reqframe, xcoord, ycoord, zcoord ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn getHeadPos()
 *  \memberof PyREEM
 *  \brief Get the current robot head yaw and pitch in radian.
 *  \return tuple(head_yaw, head_pitch)
 */
static PyObject * PyModule_REEMGetHeadPos( PyObject * self )
{
  double yaw = 0.0;
  double pitch = 0.0;

  if (REEMProxyManager::instance()->getHeadPos( yaw, pitch )) {
    return Py_BuildValue( "(dd)", yaw, pitch );
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyREEM.getHeadPos: unable to get head position!" );
    return NULL;
  }
}

/*! \fn getJointPos(joint_name)
 *  \memberof PyREEM
 *  \brief Get the current (angle) position of a joint.
 *  \param str joint_name. Name of a joint.
 *  \return float position. Must be in radian.
 */
static PyObject * PyModule_REEMGetJointPos( PyObject * self, PyObject * args )
{
  double value = 0.0;
  char * joint_name = NULL;

  if (!PyArg_ParseTuple( args, "s", &joint_name )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (REEMProxyManager::instance()->getJointPos( joint_name, value )) {
    return Py_BuildValue( "d", value );
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyREEM.getJointPos: unable to get current position for joint %s!", joint_name );
    return NULL;
  }
}

/*! \fn getPositionForJoints(joint_names)
 *  \memberof PyREEM
 *  \brief Get the current (angle) position of a joint.
 *  \param list joint_names. A list of joint names.
 *  \return list(joint_positions).
 */
static PyObject * PyModule_REEMGetPositionForJoints( PyObject * self, PyObject * args )
{
  PyObject * jnames = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &jnames )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (!PyList_Check( jnames ) || PyList_Size( jnames ) <= 0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.getPositionForJoints: must provide a list of joints." );
    return NULL;
  }

  int listSize = PyList_Size( jnames );
  
  std::vector<std::string> joints;
  std::vector<double> positions;
  
  PyObject * ckObj = NULL;
  
  for (int i = 0; i < listSize; i++) {
    ckObj = PyList_GetItem( jnames, i );
    if (PyString_Check( ckObj )) {
      joints.push_back( string( PyString_AsString( ckObj ) ));
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyREEM.getPositionForJoints: joint name at index %d is not a string.", i );
      return NULL;
    }
  }

  if (REEMProxyManager::instance()->getPositionForJoints( joints, positions )) {
    PyObject * retObj = PyList_New( listSize );
    for (int i = 0; i < listSize; i++) {
      PyList_SetItem( retObj, i, PyFloat_FromDouble( positions.at( i ) ) );
    }
    return retObj;
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyREEM.getPositionForJoints: unable to get joint position." );
    return NULL;
  }
}

/*! \fn getArmJointPositions(left_arm)
 *  \memberof PyREEM
 *  \brief Get the current joint positions of one of the REEM arm.
 *  \param bool left_arm. True for left arm; False for right arm.
 *  \return dictionary(arm_joint_positions).
 *  \note Returned dictionary use joint names as keys.
 */
static PyObject * PyModule_REEMGetArmJointPositions( PyObject * self, PyObject * args )
{
  PyObject * armsel = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &armsel )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (!PyBool_Check( armsel )) {
    PyErr_Format( PyExc_ValueError, "PyREEM.getArmJointPositions: input parameter must be a boolean!" );
    return NULL;
  }


  std::vector<std::string> joints( 7 );
  std::vector<double> positions;

  if (PyObject_IsTrue( armsel )) {
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

  if (REEMProxyManager::instance()->getPositionForJoints( joints, positions )) {
    PyObject * retObj = PyDict_New();
    for (int i = 0; i < 7; i++) {
      PyObject * numObj = PyFloat_FromDouble( positions.at( i ) );
      PyDict_SetItemString( retObj, joints.at( i ).c_str(), numObj );
      Py_DECREF( numObj );
    }
    return retObj;
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyREEM.getArmJointPositions: unable to get arm joint positions." );
    return NULL;
  }
}

/*! \fn getHandJointPositions(left_hand)
 *  \memberof PyREEM
 *  \brief Get the current joint positions of one of the REEM hand.
 *  \param bool left_hand. True for left hand; False for right hand.
 *  \return dictionary(hand_joint_positions).
 *  \note Returned dictionary use joint names as keys.
 */
static PyObject * PyModule_REEMGetHandJointPositions( PyObject * self, PyObject * args )
{
  PyObject * handsel = NULL;

  if (!PyArg_ParseTuple( args, "O", &handsel )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (!PyBool_Check( handsel )) {
    PyErr_Format( PyExc_ValueError, "PyREEM.getHandJointPositions: input parameter must be a boolean!" );
    return NULL;
  }

  std::vector<std::string> joints( 9 );
  std::vector<double> positions;

  if (PyObject_IsTrue( handsel )) {
    joints[0] = "hand_left_index_1_joint";
    joints[1] = "hand_left_index_2_joint";
    joints[2] = "hand_left_index_3_joint";
    joints[3] = "hand_left_index_joint";
    joints[4] = "hand_left_middle_1_joint";
    joints[5] = "hand_left_middle_2_joint";
    joints[6] = "hand_left_middle_3_joint";
    joints[7] = "hand_left_middle_joint";
    joints[8] = "hand_left_thumb_joint";
  }
  else {
    joints[0] = "hand_right_index_1_joint";
    joints[1] = "hand_right_index_2_joint";
    joints[2] = "hand_right_index_3_joint";
    joints[3] = "hand_right_index_joint";
    joints[4] = "hand_right_middle_1_joint";
    joints[5] = "hand_right_middle_2_joint";
    joints[6] = "hand_right_middle_3_joint";
    joints[7] = "hand_right_middle_joint";
    joints[8] = "hand_right_thumb_joint";
  }

  if (REEMProxyManager::instance()->getPositionForJoints( joints, positions )) {
    PyObject * retObj = PyDict_New();
    for (int i = 0; i < 9; i++) {
      PyObject * numObj = PyFloat_FromDouble( positions.at( i ) );
      PyDict_SetItemString( retObj, joints.at( i ).c_str(), numObj );
      Py_DECREF( numObj );
    }
    return retObj;
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyREEM.getHandJointPositions: unable to get hand joint positions." );
    return NULL;
  }
}

/*! \fn getRobotPose()
 *  \memberof PyREEM
 *  \brief Get the current REEM body pose.
 *  \return tuple(position,orientation).
 *  \note Orientation is in quaternion form (w,x,y,z).
 */
static PyObject * PyModule_REEMGetRobotPose( PyObject * self )
{
  std::vector<double> positions(3, 0.0);
  std::vector<double> orientation(4, 0.0);
  
  if (REEMProxyManager::instance()->getRobotPose( positions, orientation )) {
    PyObject * retObj = PyDict_New();
    PyObject * posObj = PyTuple_New( 3 );
    PyObject * orientObj = PyTuple_New( 4 );
    PyTuple_SetItem( posObj, 0, PyFloat_FromDouble( positions[0] ) );
    PyTuple_SetItem( posObj, 1, PyFloat_FromDouble( positions[1] ) );
    PyTuple_SetItem( posObj, 2, PyFloat_FromDouble( positions[2] ) );
    PyTuple_SetItem( orientObj, 0, PyFloat_FromDouble( orientation[0] ) );
    PyTuple_SetItem( orientObj, 1, PyFloat_FromDouble( orientation[1] ) );
    PyTuple_SetItem( orientObj, 2, PyFloat_FromDouble( orientation[2] ) );
    PyTuple_SetItem( orientObj, 3, PyFloat_FromDouble( orientation[3] ) );

    PyDict_SetItemString( retObj, "position", posObj );
    PyDict_SetItemString( retObj, "orientation", orientObj );
    Py_DECREF( posObj );
    Py_DECREF( orientObj );
    return retObj;
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyREEM.getRobotPose: unable to get robot position!" );
    return NULL;
  }
}

/*! \fn getRelativeTF(reference_frame, frame)
 *  \memberof PyREEM
 *  \brief Get the relative position and orientation of an input frame w.r.t to a reference frame.
 *  \param str reference_frame. Label of reference TF frame.
 *  \param str frame. Label of the target frame.
 *  \return tuple(position,orientation).
 *  \note Orientation is in quaternion form (w,x,y,z).
 */
static PyObject * PyModule_REEMGetRelativeTF( PyObject * self, PyObject * args )
{
  char * frame1 = NULL;
  char * frame2 = NULL;
  
  if (!PyArg_ParseTuple( args, "ss", &frame1, &frame2 )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  std::vector<double> positions(3, 0.0);
  std::vector<double> orientation(4, 0.0);
  
  if (REEMProxyManager::instance()->getRelativeTF( frame1, frame2, positions, orientation )) {
    PyObject * retObj = PyDict_New();
    PyObject * posObj = PyTuple_New( 3 );
    PyObject * orientObj = PyTuple_New( 4 );
    PyTuple_SetItem( posObj, 0, PyFloat_FromDouble( positions[0] ) );
    PyTuple_SetItem( posObj, 1, PyFloat_FromDouble( positions[1] ) );
    PyTuple_SetItem( posObj, 2, PyFloat_FromDouble( positions[2] ) );
    PyTuple_SetItem( orientObj, 0, PyFloat_FromDouble( orientation[0] ) );
    PyTuple_SetItem( orientObj, 1, PyFloat_FromDouble( orientation[1] ) );
    PyTuple_SetItem( orientObj, 2, PyFloat_FromDouble( orientation[2] ) );
    PyTuple_SetItem( orientObj, 3, PyFloat_FromDouble( orientation[3] ) );
    
    PyDict_SetItemString( retObj, "position", posObj );
    PyDict_SetItemString( retObj, "orientation", orientObj );
    Py_DECREF( posObj );
    Py_DECREF( orientObj );
    return retObj;
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyREEM.getRelativeTF: unable to get relative "
                 "transformation of frame '%s' w.r.t '%s'!",
                 frame2, frame1 );
    return NULL;
  }
}

/*! \fn navigateBodyTo(target_position, target_orientation)
 *  \memberof PyREEM
 *  \brief Navigate REEM body to a specified pose.
 *  \param tuple target_position. Position in the form of (x,y,z).
 *  \param tuple target_orientation. Orientation in quaternion form (w,x,y,z).
 *  \return None.
 *  \note Must have REEM navigation stack running prior the start of PyRIDE.
 */
static PyObject * PyModule_REEMNavigateBodyTo( PyObject * self, PyObject * args, PyObject * keywds )
{
  PyObject * posObj = NULL;
  PyObject * orientObj = NULL;
  
  if (!PyArg_ParseTupleAndKeywords( args, keywds, "OO", (char**)kPoseKWlist, &posObj, &orientObj ) ||
      !PyTuple_Check( posObj ) || !PyTuple_Check( orientObj ))
  {
    PyErr_Format( PyExc_ValueError, "PyREEM.navigateBodyTo: input parameter must be a dictionary with position and orientation tuples." );
    return NULL;
  }
  
  if (PyTuple_Size( posObj ) != (Py_ssize_t)3 ||
      PyTuple_Size( orientObj ) != (Py_ssize_t)4)
  {
    PyErr_Format( PyExc_ValueError,
                 "PyREEM.navigateBodyTo: position must be a tuple of 3 and orientation must be a tuple of 4." );
    return NULL;
  }
  
  std::vector<double> position(3, 0.0);
  std::vector<double> orientation(4, 0.0);

  PyObject * tmpObj = NULL;

  for (int i = 0; i < 3; i++) {
    tmpObj = PyTuple_GetItem( posObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyREEM.navigateBodyTo: position tuple must have float numbers." );
      return NULL;
    }
    position[i] = PyFloat_AsDouble( tmpObj );
  }

  for (int i = 0; i < 4; i++) {
    tmpObj = PyTuple_GetItem( orientObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyREEM.navigateBodyTo: orientation tuple must have float numbers." );
      return NULL;
    }
    orientation[i] = PyFloat_AsDouble( tmpObj );
  }

  REEMProxyManager::instance()->navigateBodyTo( position, orientation );
  Py_RETURN_NONE;
}

/*! \fn moveArmTo(position,orientation,use_left_arm)
 *  \memberof PyREEM
 *  \brief Move a REEM arm to a specified pose.
 *  \param tuple position. Target position in (x,y,z) w.r.t to odometry_combined reference frame.
 *  \param tuple orientation. Orientation in quaternion form (w,x,y,z).
 *  \param bool use_left_arm. True to move the left arm; False to use the right arm.
 *  \return None.
 *  \note Must have a working inverse kinematic engine i.e. either MoveIt! or S-REEM.
 */
/*! \fn getArmPose(left_arm)
 *  \memberof PyREEM
 *  \brief Get the current arm pose in task space with respect to base_foot_print TF frame.
 *  \param bool left_arm. True == left arm; False == right arm.
 *  \return A dictionary of position and orientation.
 *  \note Must have a working inverse kinematic engine i.e. either MoveIt! or S-REEM.
 */
static PyObject * PyModule_REEMMoveArmPoseTo( PyObject * self, PyObject * args, PyObject * keywds )
{
  PyObject * posObj = NULL;
  PyObject * orientObj = NULL;
  PyObject * armselObj = NULL;
  double time_to_reach = 5.0;
  
  if (!REEMProxyManager::instance()->useMoveIt()) {
    PyErr_Format( PyExc_RuntimeError, "MoveIt is not in use, "
        "this method must not be used." );
    return NULL;
  }

  if (!PyArg_ParseTupleAndKeywords( args, keywds, "OOO|d", (char**)kArmPoseKWlist, &posObj, &orientObj, &armselObj, &time_to_reach ) ||
      !PyTuple_Check( posObj ) || !PyTuple_Check( orientObj ) || !PyBool_Check( armselObj ))
  {
    PyErr_Format( PyExc_ValueError, "PyREEM.moveArmPoseTo: input parameter must be a dictionary with position, orientation tuples and use_left_arm boolean flag." );
    return NULL;
  }
  
  if (PyTuple_Size( posObj ) != (Py_ssize_t)3 ||
      PyTuple_Size( orientObj ) != (Py_ssize_t)4)
  {
    PyErr_Format( PyExc_ValueError,
                 "PyREEM.moveArmPoseTo: position must be a tuple of 3 and orientation must be a tuple of 4." );
    return NULL;
  }

  std::vector<double> position(3, 0.0);
  std::vector<double> orientation(4, 0.0);
  
  PyObject * tmpObj = NULL;
  
  for (int i = 0; i < 3; i++) {
    tmpObj = PyTuple_GetItem( posObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyREEM.moveArmPoseTo: position tuple must have float numbers." );
      return NULL;
    }
    position[i] = PyFloat_AsDouble( tmpObj );
  }
  
  for (int i = 0; i < 4; i++) {
    tmpObj = PyTuple_GetItem( orientObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyREEM.moveArmPoseTo: orientation tuple must have float numbers." );
      return NULL;
    }
    orientation[i] = PyFloat_AsDouble( tmpObj );
  }
  
  if (REEMProxyManager::instance()->moveArmWithGoalPose( PyObject_IsTrue( armselObj ),
        position, orientation, time_to_reach ))
  {
    Py_RETURN_TRUE;
  }

  Py_RETURN_FALSE;
}

/*! \fn moveArmWithJointPos(joint_position, time_to_reach)
 *  \memberof PyREEM
 *  \brief Move a REEM arm to the specified joint position within a time frame.
 *  \param dict joint_position. A dictionary of arm joint positions in radian.
 *  The dictionary must the same structure as the return of PyREEM.getArmJointPositions.
 *  \param float time_to_reach. Timeframe for reaching the pose.
 *  \return None.
 */
static PyObject * PyModule_REEMMoveArmWithJointPos( PyObject * self, PyObject * args, PyObject * keywds )
{
  double s_p_j, s_l_j, u_a_r_j, e_f_j, f_r_j, w_f_j, w_r_j;
  double time_to_reach = 2.0;
  
  bool isLeftArm = false;
  
  if (PyArg_ParseTupleAndKeywords( args, keywds, "ddddddd|d", (char**)kLeftArmKWlist,
                       &s_p_j, &s_l_j, &u_a_r_j, &e_f_j, &f_r_j,
                       &w_f_j, &w_r_j, &time_to_reach ))
  {
    isLeftArm = true;
  }
  else {
    PyErr_Clear();
    if (!PyArg_ParseTupleAndKeywords( args, keywds, "ddddddd|d", (char**)kRightArmKWlist,
                                    &s_p_j, &s_l_j, &u_a_r_j, &e_f_j, &f_r_j,
                                    &w_f_j, &w_r_j, &time_to_reach ))
    {
      // PyArg_ParseTuple will set the error status.
      return NULL;
    }
  }

  std::vector<double> positions( 7, 0.0 );
  positions[0] = s_p_j;
  positions[1] = s_l_j;
  positions[2] = u_a_r_j;
  positions[3] = e_f_j;
  positions[4] = f_r_j;
  positions[5] = w_f_j;
  positions[6] = w_r_j;

  REEMProxyManager::instance()->moveArmWithJointPos( isLeftArm, positions, time_to_reach );
  Py_RETURN_NONE;
}

/*! \fn moveArmWithJointTrajectory(joint_trajectory)
 *  \memberof PyREEM
 *  \brief Move a REEM arm to a sequence of waypoints, i.e. joint trajectory.
 *  \param list joint_trajectory. A list of waypoints that contain joint position dictionaries with the same structure
 *  of the PyREEM.moveArmWithJointPos.
 *  \return None.
 */
static PyObject * PyModule_REEMMoveArmWithJointTraj( PyObject * self, PyObject * args )
{
  PyObject * trajObj = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &trajObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  int listSize = 0;
  
  if (!PyList_Check( trajObj ) || (listSize = PyList_Size( trajObj )) == 0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectory: input parameter must be a non empty list of dictionary!" );
    return NULL;
  }

  PyObject * jointPos = NULL;
  PyObject * jval = NULL;
  int armsel = 0; // 1 for left and 2 for right
  
  std::vector< std::vector<double> > trajectory;
  std::vector<float> times_to_reach( listSize, 2.0 ); // default to 2 seconds;

  for (int i = 0; i < listSize; ++i) {
    jointPos = PyList_GetItem( trajObj, i );
    if (!PyDict_Check( jointPos ) || PyDict_Size( jointPos ) < 7) {
      PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectory: input list item %d "
                   "must be a dictionary containing all 7 joint entries for a REEM arm!", i );
      return NULL;
    }
    if (!armsel) { // check first object to determine whether we have either left or right arm joint data
      PyObject * key = PyString_FromString( kLeftArmKWlist[0] );
      if (PyDict_Contains( jointPos, key )) {
        armsel = 1;
      }
      else {
        Py_DECREF( key );
        key = PyString_FromString( kRightArmKWlist[0] );
        if (PyDict_Contains( jointPos, key )) {
          armsel = 2;
        }
      }
      Py_DECREF( key );
      if (!armsel) {
        PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectory: input list item %d contains"
                     " values not related to REEM arms!", i );
        return NULL;        
      }
    }

    std::vector<double> arm_joint_pos( 7, 0.0 );

    for (int k = 0; k < 7; k++) {
      jval = PyDict_GetItemString( jointPos, (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
      if (!jval) {
        PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectory: input list item %d has"
                     " missing %s joint value!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      if (!PyFloat_Check( jval )) {
        PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectory: input list item %d has"
                     " invalid %s joint values!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      arm_joint_pos[k] = PyFloat_AsDouble( jval );
    }
    trajectory.push_back( arm_joint_pos );
    jval = PyDict_GetItemString( jointPos, "time_to_reach" );
    if (jval && PyFloat_Check( jval )) {
      times_to_reach[i] = (float)PyFloat_AsDouble( jval );
    }
  }

  REEMProxyManager::instance()->moveArmWithJointTrajectory( (armsel == 1), trajectory, times_to_reach );
  Py_RETURN_NONE;
}

/*! \fn moveArmWithJointTrajectoryAndSpeed(joint_trajectory)
 *  \memberof PyREEM
 *  \brief Move a REEM arm to a sequence of waypoints, i.e. joint trajectory, coupled with associated joint velocities.
 *  \param list joint_trajectory. A list of waypoints with the joint data dictionaries structure
 *  of { "joint_name" : { "position" : value, "velocity" : value }, ... }. Each list item, i.e. a waypoint must also
 *  have a time to reach value that is consistent with the joint velocities at the adjacent waypoints.
 *  \return None.
 */
static PyObject * PyModule_REEMMoveArmWithJointTrajAndSpeed( PyObject * self, PyObject * args )
{
  PyObject * trajObj = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &trajObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  int listSize = 0;
  
  if (!PyList_Check( trajObj ) || (listSize = PyList_Size( trajObj )) == 0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectoryAndSpeed: input parameter must be a non empty list of dictionary!" );
    return NULL;
  }
  
  PyObject * jointPos = NULL;
  PyObject * jval = NULL;
  int armsel = 0; // 1 for left and 2 for right
  
  std::vector< std::vector<double> > trajectory;
  std::vector< std::vector<double> > joint_velocities;
  std::vector<float> times_to_reach( listSize, 2.0 ); // default to 2 seconds;

  for (int i = 0; i < listSize; ++i) {
    jointPos = PyList_GetItem( trajObj, i );
    if (!PyDict_Check( jointPos ) || PyDict_Size( jointPos ) < 7) {
      PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectoryAndSpeed: input list item %d "
                   "must be a dictionary containing all 7 joint entries for a REEM arm!", i );
      return NULL;
    }
    if (!armsel) { // check first object to determine whether we have either left or right arm joint data
      PyObject * key = PyString_FromString( kLeftArmKWlist[0] );
      if (PyDict_Contains( jointPos, key )) {
        armsel = 1;
      }
      else {
        Py_DECREF( key );
        key = PyString_FromString( kRightArmKWlist[0] );
        if (PyDict_Contains( jointPos, key )) {
          armsel = 2;
        }
      }
      Py_DECREF( key );
      if (!armsel) {
        PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectoryAndSpeed: input list item %d contains"
                     " values not related to REEM arms!", i );
        return NULL;
      }
    }
    
    std::vector<double> arm_joint_pos( 7, 0.0 );
    std::vector<double> arm_joint_vel( 7, 0.0 );
    
    for (int k = 0; k < 7; k++) {
      jval = PyDict_GetItemString( jointPos, (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
      if (!jval) {
        PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectoryAndSpeed: input list item %d has"
                     " missing %s joint value!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      if (!PyDict_Check( jval )){
        PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectoryAndSpeed: input list item %d contains "
                     " non-dictionary data for joint %s!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      PyObject * pos_val = PyDict_GetItemString( jval, "position" );
      PyObject * vel_val = PyDict_GetItemString( jval, "velocity" );
      if (!(pos_val && vel_val && PyFloat_Check( pos_val ) && PyFloat_Check( vel_val ))) {
        PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectoryAndSpeed: input list item %d contains invalid"
                     " dictionary data for joint %s!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      arm_joint_pos[k] = PyFloat_AsDouble( pos_val );
      arm_joint_vel[k] = PyFloat_AsDouble( vel_val );
    }
    trajectory.push_back( arm_joint_pos );
    joint_velocities.push_back( arm_joint_vel );
    
    jval = PyDict_GetItemString( jointPos, "time_to_reach" );
    if (jval && PyFloat_Check( jval )) {
      times_to_reach[i] = (float)PyFloat_AsDouble( jval );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyREEM.moveArmWithJointTrajectoryAndSpeed: input list item %d contains invalid"
                   " time to reach data!", i );
    }
  }
  
  REEMProxyManager::instance()->moveArmWithJointTrajectoryAndSpeed( (armsel == 1), trajectory, joint_velocities, times_to_reach );
  Py_RETURN_NONE;
}

/*! \fn cancelMoveArmAction(is_left_arm)
 *  \memberof PyREEM
 *  \brief Cancel current arm movement action invoked by PyREEM.moveArmWithJointPos or
 *  PyREEM.moveArmWithJointTrajectory method call.
 *  \param bool is_left_arm. True cancels the left arm movement; False cancels the right arm movement.
 *  \return None.
 *  \todo This function has not been fully tested on REEM and may be buggy.
 */
static PyObject * PyModule_REEMCancelMoveArmAction( PyObject * self, PyObject * args )
{
  PyObject * armsel = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &armsel )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (!PyBool_Check( armsel )) {
    PyErr_Format( PyExc_ValueError, "PyREEM.cancelMoveArmAction: input parameter must be a boolean!" );
    return NULL;
  }

  REEMProxyManager::instance()->cancelArmMovement( PyObject_IsTrue( armsel ) );
  Py_RETURN_NONE;
}

/*! \fn cancelMoveBodyAction()
 *  \memberof PyREEM
 *  \brief Cancel current body movement action invoked by PyREEM.moveBodyTo or PyREEM.moveBodyWithSpeed method call.
 *  \return None.
 *  \todo This function has not been fully tested on REEM and may be buggy.
 */
static PyObject * PyModule_REEMCancelMoveBodyAction( PyObject * self )
{
  REEMProxyManager::instance()->cancelBodyMovement();
  Py_RETURN_NONE;
}

/*! \fn openHand(which_hand)
 *  \memberof PyREEM
 *  \brief Opens one or both REEM grippers.
 *  \param int which_hand. 1 = left hand, 2 = right hand and 3 = both hand.
 *  \return bool. True == valid command; False == invalid command.
 *  \todo This function has not been fully tested on REEM and may be buggy.
 */
static PyObject * PyModule_REEMOpenHand( PyObject * self, PyObject * args )
{
  int mode = 0;
  if (!PyArg_ParseTuple( args, "i", &mode )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  std::vector<double> hand_pos( 3, 0.0 );
  switch (mode) {
    case 1:
      if (REEMProxyManager::instance()->setHandPosition( true, hand_pos ))
        Py_RETURN_TRUE;
      break;
    case 2:
      if (REEMProxyManager::instance()->setHandPosition( false, hand_pos ))
        Py_RETURN_TRUE;
      break;
    case 3:
      if (REEMProxyManager::instance()->setHandPosition( true, hand_pos ) ||
          REEMProxyManager::instance()->setHandPosition( false, hand_pos ))
      {
        Py_RETURN_TRUE;
      }
      break;
    default:
      PyErr_Format( PyExc_ValueError, "PyREEM.openHand: invalid hand number! 1 = left hand, 2 = right hand and 3 = both hand." );
      return NULL;
  }
  Py_RETURN_FALSE;
}

/*! \fn closeHand(which_hand)
 *  \memberof PyREEM
 *  \brief Closes one or both REEM hand.
 *  \param int which_hand. 1 = left hand, 2 = right hand and 3 = both hands.
 *  \return bool. True == valid command; False == invalid command.
 *  \todo This function has not been fully tested on REEM and may be buggy.
 */
static PyObject * PyModule_REEMCloseHand( PyObject * self, PyObject * args )
{
  int mode = 0;
  if (!PyArg_ParseTuple( args, "i", &mode )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  std::vector<double> hand_pos( 3, 0.0 );
  // TODO: set position
  switch (mode) {
    case 1:
      if (REEMProxyManager::instance()->setHandPosition( true, hand_pos ))
        Py_RETURN_TRUE;
      break;
    case 2:
      if (REEMProxyManager::instance()->setHandPosition( false, hand_pos ))
        Py_RETURN_TRUE;
      break;
    case 3:
      if (REEMProxyManager::instance()->setHandPosition( true, hand_pos ) ||
          REEMProxyManager::instance()->setHandPosition( false, hand_pos ))
      {
        Py_RETURN_TRUE;
      }
      break;
    default:
      PyErr_Format( PyExc_ValueError, "PyREEM.closeHand: invalid hand number! 1 = left hand, 2 = right hand and 3 = both hand." );
      return NULL;
  }
  Py_RETURN_FALSE;
}

/*! \fn setHandPosition(hand_joint_positions)
 *  \memberof PyREEM
 *  \brief Move a REEM hand to the specified joint position within a time frame.
 *  \param dict joint_position. A dictionary of hand joint positions in radian.
 *  The dictionary must the same structure as the return of PyREEM.getHandJointPositions.
 *  \param float time_to_reach. Time frame for reaching the pose.
 *  \todo This function has not been fully tested on REEM and may be buggy.
 */
static PyObject * PyModule_REEMSetHandPosition( PyObject * self, PyObject * args, PyObject * keywds  )
{
  int mode = 0;
  PyObject * posObj = NULL;

  double h_i_j, h_m_j, h_t_j;
  double time_to_reach = 2.0;
  
  bool isLeftArm = false;
  
  if (PyArg_ParseTupleAndKeywords( args, keywds, "ddd|d", (char**)kLeftHandKWlist,
                                     &h_i_j, &h_m_j, &h_t_j, &time_to_reach ))
  {
    isLeftArm = true;
  }
  else {
    PyErr_Clear();
    if (!PyArg_ParseTupleAndKeywords( args, keywds, "ddd|d", (char**)kRightHandKWlist,
                                        &h_i_j, &h_m_j, &h_t_j, &time_to_reach ))
    {
      // PyArg_ParseTuple will set the error status.
      return NULL;
    }
  }
  std::vector<double> hand_pos( 3, 0.0 );
  hand_pos[0] = h_i_j;
  hand_pos[1] = h_m_j;
  hand_pos[2] = h_t_j;

  if (REEMProxyManager::instance()->setHandPosition( isLeftArm, hand_pos ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn registerBaseScanCallback( callback_function, target_frame )
 *  \memberof PyREEM
 *  \brief Register a callback function for receiving base laser scan data.
 *  None object can be used to stop receiving the scan data.
 *  If target frame is provided, the 3D position (x,y,z) w.r.t the target 
 *  frame will be returned. Otherwise, raw laser scan data is returned.
 *  \param callback function that takes a list of raw laser range data or 3D position data as the input.
 *  \param string target_frame. Optional, the name of the target frame
 *  \return None
 */
static PyObject * PyModule_REEMRegisterBaseScanData( PyObject * self, PyObject * args )
{
  PyObject * callbackFn = NULL;
  char * target_frame = NULL;
  
  if (!PyArg_ParseTuple( args, "O|s", &callbackFn, &target_frame )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (callbackFn == Py_None) {
    PyREEMModule::instance()->setBaseScanCallback( NULL );
    REEMProxyManager::instance()->deregisterForBaseScanData();
    Py_RETURN_NONE;
  }
  
  if (!PyCallable_Check( callbackFn )) {
    PyErr_Format( PyExc_ValueError, "First input parameter is not a callable object" );
    return NULL;
  }

  PyREEMModule::instance()->setBaseScanCallback( callbackFn );
  
  if (target_frame) {
    if (!REEMProxyManager::instance()->isTFFrameSupported( target_frame )) {
      PyErr_Format( PyExc_ValueError, "Input target frame is not supported!" );
      return NULL;
    }
    REEMProxyManager::instance()->registerForBaseScanData( target_frame );
  }
  else {
    REEMProxyManager::instance()->registerForBaseScanData();
  }
  Py_RETURN_NONE;
}

/*! \fn registerTiltScanCallback( callback_function, target_frame )
 *  \memberof PyREEM
 *  \brief Register a callback function for receiving tilt laser scan data.
 *  None object can be used to stop receiving the scan data.
 *  If target frame is provided, the 3D position (x,y,z) w.r.t the target
 *  frame will be returned. Otherwise, raw laser scan data is returned.
 *  \param callback function that takes a list of raw laser range data or 3D position data as the input.
 *  \param string target_frame. Optional, the name of the target frame
 *  \return None
 */
static PyObject * PyModule_REEMRegisterTiltScanData( PyObject * self, PyObject * args )
{
  PyObject * callbackFn = NULL;
  char * target_frame = NULL;
  
  if (!PyArg_ParseTuple( args, "O|s", &callbackFn, &target_frame )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (callbackFn == Py_None) {
    PyREEMModule::instance()->setTiltScanCallback( NULL );
    REEMProxyManager::instance()->deregisterForTiltScanData();
    Py_RETURN_NONE;
  }
  
  if (!PyCallable_Check( callbackFn )) {
    PyErr_Format( PyExc_ValueError, "First input parameter is not a callable object" );
    return NULL;
  }
  
  PyREEMModule::instance()->setTiltScanCallback( callbackFn );
  
  if (target_frame) {
    if (!REEMProxyManager::instance()->isTFFrameSupported( target_frame )) {
      PyErr_Format( PyExc_ValueError, "Input target frame is not supported!" );
      return NULL;
    }
    REEMProxyManager::instance()->registerForTiltScanData( target_frame );
  }
  else {
    REEMProxyManager::instance()->registerForTiltScanData();
  }
  Py_RETURN_NONE;
}

/*! \fn registerPalFaceCallback( callback_function )
 *  \memberof PyREEM
 *  \brief Register a callback function for receiving face data.
 *  None object can be used to stop receiving the face data.
 *  \param callback function that takes a list of detected faces (data).
 *  \return None
 */
static PyObject * PyModule_REEMRegisterPalFaceData( PyObject * self, PyObject * args )
{
  PyObject * callbackFn = NULL;
  double confidence = 220.0;

  if (!PyArg_ParseTuple( args, "O|d", &callbackFn, &confidence )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (confidence < 0.0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.registerPalFaceCallback: confidence must be > 0.0." );
    return NULL;
  }

  if (callbackFn == Py_None) {
    PyREEMModule::instance()->setPalFaceCallback( NULL );
    REEMProxyManager::instance()->deregisterForPalFaceData();
    REEMProxyManager::instance()->enablePalFaceDetection( false, 0.0 );
    Py_RETURN_NONE;
  }

  if (!PyCallable_Check( callbackFn )) {
    PyErr_Format( PyExc_ValueError, "PyREEM.registerPalFaceCallback:: first input parameter is not a callable object" );
    return NULL;
  }

  PyREEMModule::instance()->setPalFaceCallback( callbackFn );
  REEMProxyManager::instance()->registerForPalFaceData();
  REEMProxyManager::instance()->enablePalFaceDetection( true, confidence );
  Py_RETURN_NONE;
}

/*! \fn addSolidObject(name,volume,position,orientation)
 *  \memberof PyREEM
 *  \brief Add a solid object to the current collision scene.
 *  \param string name. Name of the solid object.
 *  \param tuple volume. The volume of the object in (width,height,depth).
 *  \param tuple position. Position of the object in (x,y,z).
 *  \param tuple orientation. Orientation of the object in quaternion form (w,x,y,z).
 *  \return bool. True == success; False == otherwise.
 *  \note Require MoveIt! to be running prior the start of PyRIDE.
 */
static PyObject * PyModule_REEMAddSolidObject( PyObject * self, PyObject * args, PyObject * keywds )
{
  char * objName = NULL;
  PyObject * volObj = NULL;
  PyObject * posObj = NULL;
  PyObject * orientObj = NULL;

  if (!REEMProxyManager::instance()->useMoveIt()) {
    PyErr_Format( PyExc_RuntimeError, "MoveIt is not in use, "
        "this method must not be used." );
    return NULL;
  }

  if (!PyArg_ParseTupleAndKeywords( args, keywds, "sOOO", (char**)kObjectKWlist, &objName, &volObj, &posObj, &orientObj ) ||
      !PyTuple_Check( posObj ) || !PyTuple_Check( orientObj ) || !PyTuple_Check( volObj ))
  {
    PyErr_Format( PyExc_ValueError, "PyREEM.addSolidObject: input parameter must contains volume, position, orientation tuples." );
    return NULL;
  }

  if (PyTuple_Size( posObj ) != (Py_ssize_t)3 ||
      PyTuple_Size( volObj ) != (Py_ssize_t)3 ||
      PyTuple_Size( orientObj ) != (Py_ssize_t)4)
  {
    PyErr_Format( PyExc_ValueError,
                 "PyREEM.addSolidObject: volume and position must be tuples of 3 and orientation must be a tuple of 4." );
    return NULL;
  }

  std::vector<double> volume(3, 0.0);
  std::vector<double> position(3, 0.0);
  std::vector<double> orientation(4, 0.0);

  PyObject * tmpObj = NULL;

  for (int i = 0; i < 3; i++) {
    tmpObj = PyTuple_GetItem( volObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyREEM.addSolidObject: volume tuple must have float numbers." );
      return NULL;
    }
    volume[i] = PyFloat_AsDouble( tmpObj );

    tmpObj = PyTuple_GetItem( posObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyREEM.addSolidObject: position tuple must have float numbers." );
      return NULL;
    }
    position[i] = PyFloat_AsDouble( tmpObj );
  }

  for (int i = 0; i < 4; i++) {
    tmpObj = PyTuple_GetItem( orientObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyREEM.addSolidObject: orientation tuple must have float numbers." );
      return NULL;
    }
    orientation[i] = PyFloat_AsDouble( tmpObj );
  }

  if (REEMProxyManager::instance()->addSolidObject( objName, volume, position, orientation )) {
    Py_RETURN_TRUE;
  }
  else {
    Py_RETURN_FALSE;
  }
}

/*! \fn delSolidObject()
 *  \memberof PyREEM
 *  \brief Delete an existing solid object from the current collision scene.
 *  \param string name. Name of the solid object.
 *  \return None.
 *  \note Require MoveIt! to be running prior the start of PyRIDE.
 */
static PyObject * PyModule_REEMDelSolidObject( PyObject * self, PyObject * args )
{
  char * objName = NULL;

  if (!REEMProxyManager::instance()->useMoveIt()) {
    PyErr_Format( PyExc_RuntimeError, "MoveIt is not in use, "
        "this method must not be used." );
    return NULL;
  }

  if (!PyArg_ParseTuple( args, "s", &objName )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  REEMProxyManager::instance()->removeSolidObject( objName );
  Py_RETURN_NONE;
}

/*! \fn listSolidObjects()
 *  \memberof PyREEM
 *  \brief Return a list of known solid objects in the collision scene.
 *  \return list(name of solid objects).
 *  \note Require MoveIt! to be running prior the start of PyRIDE.
 */
static PyObject * PyModule_REEMListSolidObjects( PyObject * self )
{
  std::vector<std::string> objlist;

  if (!REEMProxyManager::instance()->useMoveIt()) {
    PyErr_Format( PyExc_RuntimeError, "MoveIt is not in use, "
        "this method must not be used." );
    return NULL;
  }

  REEMProxyManager::instance()->listSolidObjects( objlist );
  int fsize = (int)objlist.size();
  PyObject * retObj = PyList_New( fsize );
  for (int i = 0; i < fsize; ++i) {
    PyList_SetItem( retObj, i, PyString_FromString( objlist[i].c_str() ) );
  }
  return retObj;
}

/*! \fn pickUpObject(name,place,grasp_position,grasp_orientation,use_left_arm,distance_from)
 *  \memberof PyREEM
 *  \brief Pickup a solid object from a location (another solid object) use a specific grasp pose and approaching distance.
 *  \param string name. Name of the solid object.
 *  \param string name. Name of the object from where the target object is picked up from.
 *  \param tuple grasp_position. Grasp position (x,y,z).
 *  \param tuple grasp_orientation. Grasp orientation in quaternion form (w,x,y,z).
 *  \param bool use_left_arm. True == use the left arm; False = use the right arm.
 *  \param float distance_from. Approaching distance from the object.
 *  \return bool. True == success; False == otherwise.
 *  \warning Not fully tested!
 *  \note Require MoveIt! to be running prior the start of PyRIDE.
 */
/*! \fn placeObject(name,place,place_position,place_orientation,use_left_arm,distance_from)
 *  \memberof PyREEM
 *  \brief Place a solid object to a location (another solid object) use a specific place pose and retreating distance.
 *  \param string name. Name of the solid object.
 *  \param string name. Name of the object from where the target object is placed to.
 *  \param tuple place_position. Place position (x,y,z).
 *  \param tuple place_orientation. Place orientation in quaternion form (w,x,y,z).
 *  \param bool use_left_arm. True == use the left arm; False = use the right arm.
 *  \param float distance_from. Retreating distance from the object.
 *  \return bool. True == success; False == otherwise.
 *  \warning Not fully tested!
 *  \note Require MoveIt! to be running prior the start of PyRIDE.
 */

static PyObject * PyModule_REEMPickUpAndPlaceObject( bool to_place, PyObject * self, PyObject * args, PyObject * keywds )
{
  char * objName = NULL;
  char * placeName = NULL;
  PyObject * armselObj = NULL;
  PyObject * posObj = NULL;
  PyObject * orientObj = NULL;
  double distance = 5.0;

  if (!REEMProxyManager::instance()->useMoveIt()) {
    PyErr_Format( PyExc_RuntimeError, "MoveIt is not in use, "
        "this method must not be used." );
    return NULL;
  }

  if (!PyArg_ParseTupleAndKeywords( args, keywds, "ssOOOf", (char**)kPickAndPlaceKWlist, &objName, &placeName, &posObj, &orientObj, &armselObj, &distance ) ||
      !PyTuple_Check( posObj ) || !PyTuple_Check( orientObj ) || !PyBool_Check( armselObj ))
  {
    PyErr_Format( PyExc_ValueError, "PyREEM.%s: input parameter must be a dictionary with position, "
        "orientation tuples and and use_left_arm boolean flag.",
        to_place ? "placeObject" : "pickUpObject" );
    return NULL;
  }

  if (PyTuple_Size( posObj ) != (Py_ssize_t)3 ||
      PyTuple_Size( orientObj ) != (Py_ssize_t)4)
  {
    PyErr_Format( PyExc_ValueError,
                 "PyREEM.%s: position must be tuples of 3 and orientation must be a tuple of 4.",
                 to_place ? "placeObject" : "pickUpObject" );
    return NULL;
  }

  std::vector<double> pose(7, 0.0);

  PyObject * tmpObj = NULL;

  for (int i = 0; i < 3; i++) {
    tmpObj = PyTuple_GetItem( posObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyREEM.%s: position tuple must have float numbers.",
                   to_place ? "placeObject" : "pickUpObject" );

      return NULL;
    }
    pose[i] = PyFloat_AsDouble( tmpObj );
  }

  for (int i = 0; i < 4; i++) {
    tmpObj = PyTuple_GetItem( orientObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyREEM.%s: orientation tuple must have float numbers.",
                   to_place ? "placeObject" : "pickUpObject" );
      return NULL;
    }
    pose[3+i] = PyFloat_AsDouble( tmpObj );
  }

  if (to_place) {
    if (REEMProxyManager::instance()->placeObject( objName, placeName, pose, PyObject_IsTrue( armselObj ), distance )) {
      Py_RETURN_TRUE;
    }
    else {
      Py_RETURN_FALSE;
    }
  }
  else {
    if (REEMProxyManager::instance()->pickupObject( objName, placeName, pose, PyObject_IsTrue( armselObj ), distance )) {
      Py_RETURN_TRUE;
    }
    else {
      Py_RETURN_FALSE;
    }
  }
  Py_RETURN_FALSE;
}

static PyObject * PyModule_REEMPickUpObject( PyObject * self, PyObject * args, PyObject * keywds )
{
  return PyModule_REEMPickUpAndPlaceObject( false, self, args, keywds );
}

static PyObject * PyModule_REEMPlaceObject( PyObject * self, PyObject * args, PyObject * keywds )
{
  return PyModule_REEMPickUpAndPlaceObject( true, self, args, keywds );
}

/*! \fn setTorsoStiffness(stiffness)
 *  \memberof PyREEM
 *  \brief Set the stiffness of the REEM torso.
 *  \param float stiffness. Must be between [0.0,1.0].
 *  \return None.
 */
static PyObject * PyModule_REEMSetTorsoStiffness( PyObject * self, PyObject * args )
{
  float stiff = 0.0;

  if (!PyArg_ParseTuple( args, "f", &stiff )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (stiff < 0.0 || stiff > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.setTorsoStiffness: the stiffness input must be within the range of [0.0, 1.0]!" );
    return NULL;
  }

  REEMProxyManager::instance()->setTorsoStiffness( stiff );
  Py_RETURN_NONE;
}

/*! \fn setArmStiffness(left_arm,stiffness)
 *  \memberof PyREEM
 *  \brief Set the stiffness of a REEM' arm.
 *  \param bool left_arm. True for left arm; False for right arm.
 *  \param float stiffness. Must be between [0.0,1.0].
 *  \return None.
 *  \note when an arm stiffness changes from 0 to 1.0, the arm snap back to
 *  the position where the stiffness was previously turned off.
 */
static PyObject * PyModule_REEMSetArmStiffness( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  float stiff = 0.0;

  if (!PyArg_ParseTuple( args, "Of", &isYesObj, &stiff )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (!PyBool_Check( isYesObj )) {
    PyErr_Format( PyExc_ValueError, "PyREEM.setArmStiffness: the first parameter must be a boolean!" );
    return NULL;
  }
  if (stiff < 0.0 || stiff > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.setArmStiffness: the stiffness input must be within the range of [0.0, 1.0]!" );
    return NULL;
  }

  REEMProxyManager::instance()->setArmStiffness( PyObject_IsTrue( isYesObj ), stiff );
  Py_RETURN_NONE;
}

/** @name Miscellaneous Functions
 *
 */
/**@{*/
/*! \fn setEarLED(colour, side)
 *  \memberof PyREEM
 *  \brief Set REEM's ear LED to a colour.
 *  \param str colour. Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'.
 *  \param int side. 1 = left side, 2 = right sight and3 = both, default = 3.
 *  \return int effectId. The ID number of the effect.
 */
static PyObject * PyModule_REEMSetEarLED( PyObject * self, PyObject * args )
{
  char * colourStr = NULL;
  REEMLedColour colourID;
  int headSide = 3;

  if (!PyArg_ParseTuple( args, "s|i", &colourStr, &headSide )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (!colourStr2ID( colourStr, colourID )) {
    PyErr_Format( PyExc_ValueError, "PyREEM.setEarLED: invalid input colour. "
                 "Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'." );
    return NULL;
  }

  if (headSide > 3 || headSide <= 0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.setEarLED: invalid side of head: 1 = left, 2 = right, 3 = both." );
    return NULL;
  }

  return PyLong_FromLong(  REEMProxyManager::instance()->setEarLED( colourID, headSide ));
}

/*! \fn pulseEarLED(colour_one, colour_two, side, period)
 *  \memberof PyREEM
 *  \brief Periodically switch REEM's ear LED between the two input colours.
 *  \param str colour_one. Colour label one.
 *  \param str colour_two. Colour label two.
 *  \param int side. 1 = left, 2 = right and3 = both, default = 3.
 *  \param int period. Time (in seconds) before switching LED colour.
 *  \return int effectId. The ID number of the effect.
 *  \note Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'.
 */
static PyObject * PyModule_REEMPulseEarLED( PyObject * self, PyObject * args )
{
  char * colourStr1 = NULL;
  char * colourStr2 = NULL;

  float period = 0.5;
  int headSide = 3;

  REEMLedColour colourID1, colourID2;

  if (!PyArg_ParseTuple( args, "ss|if", &colourStr1, &colourStr2, &headSide, &period )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (!colourStr2ID( colourStr1, colourID1 ) || !colourStr2ID( colourStr2, colourID2 )) {
    PyErr_Format( PyExc_ValueError, "PyREEM.pluseEarLED: invalid input colour(s)."
                 "Colour must be 'red','green', 'blue', 'white', 'blank', 'yellow' or 'pink'." );
    return NULL;
  }

  if (headSide > 3 || headSide <= 0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.pluseEarLED: invalid side of head: 1 = left, 2 = right, 3 = both." );
    return NULL;
  }
  if (period <= 0.0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.pluseEarLED: invalid pulse period." );
    return NULL;
  }

  return PyLong_FromLong( REEMProxyManager::instance()->pulseEarLED( colourID1, colourID2, headSide, period ) );
}

/** @name Miscellaneous Functions
 *
 */
/**@{*/
/*! \fn cancelEarLEDEffect(effect_id)
 *  \memberof PyREEM
 *  \brief Set REEM's ear LED to a colour.
 *  \param int effect_id. ID of the effect to be cancelled.
 *  \return bool True = successful, otherwise False.
 */
static PyObject * PyModule_REEMCancelEarLEDEffect( PyObject * self, PyObject * args )
{
  int effectId = 0;

  if (!PyArg_ParseTuple( args, "i", &effectId )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (effectId <= 0) {
    PyErr_Format( PyExc_ValueError, "PyREEM.cancelEarLEDEffect: invalid effect ID number." );
    return NULL;
  }

  if (REEMProxyManager::instance()->cancelEarLED( effectId ))
    Py_RETURN_TRUE;

  Py_RETURN_FALSE;
}

/*! \fn startPalFaceEnrollment(name)
 *  \memberof PyREEM
 *  \brief Start register face with a name
 *  \param str name. Name of the user to be registered.
 *  \return True = face enrollment started successfully, otherwise, False.
 */
static PyObject * PyModule_REEMStartPalFaceEnrollment( PyObject * self, PyObject * args )
{
  char * nameStr = NULL;

  if (!PyArg_ParseTuple( args, "s", &nameStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (REEMProxyManager::instance()->palFaceStartEnrollment( nameStr )) {
    Py_RETURN_TRUE;
  }

  Py_RETURN_FALSE;
}

/*! \fn stopPalFaceEnrollment(name)
 *  \memberof PyREEM
 *  \brief Start register face with a name
 *  \param str name. Name of the user to be registered.
 *  \return True = face enrollment is successfully, otherwise, False.
 */
static PyObject * PyModule_REEMStopPalFaceEnrollment( PyObject * self )
{
  if (REEMProxyManager::instance()->palFaceStopEnrollment()) {
    Py_RETURN_TRUE;
  }

  Py_RETURN_FALSE;
}

/*! \fn directToWeb(uri)
 *  \memberof PyREEM
 *  \brief direct REEM touch screen to a specified web site.
 *  \param str uri. URI for the web site.
 *  \return None.
 */
static PyObject * PyModule_REEMDirectToWeb( PyObject * self, PyObject * args )
{
  char * uriStr = NULL;

  if (!PyArg_ParseTuple( args, "s", &uriStr ) || strlen(uriStr) == 0) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  REEMProxyManager::instance()->directToWeb( uriStr );
    Py_RETURN_NONE;
}

/**@}*/
#define INCLUDE_COMMON_PYMODULE_MEHTODS
#include "../pyridecore/PyModulePyCommon.cpp"

static PyMethodDef PyModule_methods[] = {
  { "write", (PyCFunction)PyModule_write, METH_VARARGS,
    "standard output for UTS REEM Python console." },
  { "setTeamMemberID", (PyCFunction)PyModule_SetTeamMemberID, METH_VARARGS,
    "Set REEM team member ID and team colour." },
  { "sendTeamMessage", (PyCFunction)PyModule_sendTeamMessage, METH_VARARGS,
    "Send a message to the rest team members." },
  { "say", (PyCFunction)PyModule_REEMSayWithVolume, METH_VARARGS,
    "Let REEM speak with an optional volume." },
  { "pointHeadTo", (PyCFunction)PyModule_REEMPointHeadTo, METH_VARARGS,
    "Point REEM head to a new 3D position in wide stereo camera frame." },
  { "getHeadPos", (PyCFunction)PyModule_REEMGetHeadPos, METH_NOARGS,
    "Get REEM's head position." },
  { "getJointPos", (PyCFunction)PyModule_REEMGetJointPos, METH_VARARGS,
    "Get a joint's current position." },
  { "getPositionForJoints", (PyCFunction)PyModule_REEMGetPositionForJoints, METH_VARARGS,
    "Get positions for a list of joints." },
  { "getArmJointPositions", (PyCFunction)PyModule_REEMGetArmJointPositions, METH_VARARGS,
    "Get joint positions of REEM arms." },
  { "getHandJointPositions", (PyCFunction)PyModule_REEMGetHandJointPositions, METH_VARARGS,
    "Get joint positions of REEM hand." },
  { "getRobotPose", (PyCFunction)PyModule_REEMGetRobotPose, METH_NOARGS,
    "Get the current REEM pose." },
  { "getRelativeTF", (PyCFunction)PyModule_REEMGetRelativeTF, METH_VARARGS,
    "Get the relative TF between two frames with the first frame as the reference frame." },
  { "updateHeadPos", (PyCFunction)PyModule_REEMUpdateHeadPos, METH_VARARGS,
    "Update REEM head position with a specific velocity in radian/s." },
  { "moveHeadTo", (PyCFunction)PyModule_REEMMoveHeadTo, METH_VARARGS,
    "Move REEM head to a new position (in degree)." },
  { "moveHeadWithJointTrajectory", (PyCFunction)PyModule_REEMMoveHeadWithJointTraj, METH_VARARGS,
    "Move REEM's head in a specific joint trajectory (a list of joint positions)." },
  { "moveBodyTo", (PyCFunction)PyModule_REEMMoveBodyTo, METH_VARARGS,
    "Move REEM base to a new pose." },
  { "moveBodyWithSpeed", (PyCFunction)PyModule_REEMMoveBodyWithSpeed, METH_VARARGS,
    "Set REEM base moving speed." },
  { "navigateBodyTo", (PyCFunction)PyModule_REEMNavigateBodyTo, METH_VARARGS|METH_KEYWORDS,
    "Navigate REEM base to a new pose." },
  { "cancelMoveBodyAction", (PyCFunction)PyModule_REEMCancelMoveBodyAction, METH_NOARGS,
    "Cancel the active move body actions." },
  { "moveTorsoTo", (PyCFunction)PyModule_REEMMoveTorsoTo, METH_VARARGS,
    "Move REEM torso joints." },
  { "moveTorsoWithJointTrajectory", (PyCFunction)PyModule_REEMMoveTorsoWithJointTraj, METH_VARARGS,
    "Move REEM's torso in a specific joint trajectory (a list of joint positions)." },
  { "moveArmPoseTo", (PyCFunction)PyModule_REEMMoveArmPoseTo, METH_VARARGS|METH_KEYWORDS,
    "Move one of REEM arms end point pose to a coordinate wrt torso." },
  { "moveArmWithJointPos", (PyCFunction)PyModule_REEMMoveArmWithJointPos, METH_VARARGS|METH_KEYWORDS,
    "Move one of REEM arms with specific joint positions." },
  { "moveArmWithJointTrajectory", (PyCFunction)PyModule_REEMMoveArmWithJointTraj, METH_VARARGS,
    "Move one of REEM arms in a specific joint trajectory (a list of joint positions)." },
  { "moveArmWithJointTrajectoryAndSpeed", (PyCFunction)PyModule_REEMMoveArmWithJointTrajAndSpeed, METH_VARARGS,
    "Move one of REEM arms in a specific joint trajectory with joint velocity (a list of joint positions with associated velocity)." },
  { "cancelMoveArmAction", (PyCFunction)PyModule_REEMCancelMoveArmAction, METH_VARARGS,
    "Cancel the active move arm actions." },
  { "openHand", (PyCFunction)PyModule_REEMOpenHand, METH_VARARGS,
    "Open one or both REEM grippers." },
  { "closeHand", (PyCFunction)PyModule_REEMCloseHand, METH_VARARGS,
    "Close one or both REEM grippers." },
  { "setHandPosition", (PyCFunction)PyModule_REEMSetHandPosition, METH_VARARGS|METH_KEYWORDS,
    "Set specific position on one or both REEM grippers." },
  { "getBatteryStatus", (PyCFunction)PyModule_REEMGetBatteryStatus, METH_NOARGS,
    "Get the current battery status." },
  { "getLowPowerThreshold", (PyCFunction)PyModule_REEMGetLowPowerThreshold, METH_NOARGS,
    "Get the low power warning threshold." },
  { "setLowPowerThreshold", (PyCFunction)PyModule_REEMSetLowPowerThreshold, METH_VARARGS,
    "Set the low power warning threshold." },
  { "setEarLED", (PyCFunction)PyModule_REEMSetEarLED, METH_VARARGS,
      "Set the colour of the ear LEDs on REEM." },
  { "pulseEarLED", (PyCFunction)PyModule_REEMPulseEarLED, METH_VARARGS,
      "Pulse the ear LED of REEM between two colours." },
  { "cancelEarLEDEffect", (PyCFunction)PyModule_REEMCancelEarLEDEffect, METH_VARARGS,
      "Cancel an ear LED effect ." },
  { "setTorsoStiffness", (PyCFunction)PyModule_REEMSetTorsoStiffness, METH_VARARGS,
    "Set the stiffness of the REEM's torso. " },
  { "setArmStiffness", (PyCFunction)PyModule_REEMSetArmStiffness, METH_VARARGS,
    "Set the stiffness of the one of REEM's arms. " },
  { "listTFFrames", (PyCFunction)PyModule_REEMListTFFrames, METH_NOARGS,
    "List supported REEM TF frames." },
  { "isSupportedTFFrame", (PyCFunction)PyModule_REEMCheckTFFrame, METH_VARARGS,
    "Check whether the input TF frames is supported." },
  { "useMoveIt", (PyCFunction)PyModule_REEMUseMoveIt, METH_NOARGS,
    "Check whether MoveIt is in use." },
  { "addSolidObject", (PyCFunction)PyModule_REEMAddSolidObject, METH_VARARGS|METH_KEYWORDS,
    "Add a solid object into the collision scene." },
  { "delSolidObject", (PyCFunction)PyModule_REEMDelSolidObject, METH_VARARGS,
    "Remove an existing solid object from the collision scene." },
  { "listSolidObjects", (PyCFunction)PyModule_REEMListSolidObjects, METH_NOARGS,
    "List all known solid objects in the collision scene." },
  { "pickUpObject", (PyCFunction)PyModule_REEMPickUpObject, METH_VARARGS|METH_KEYWORDS,
    "Pick a known object from a known place." },
  { "placeObject", (PyCFunction)PyModule_REEMPlaceObject, METH_VARARGS|METH_KEYWORDS,
    "Place a known object to a known place." },
  { "registerBaseScanCallback", (PyCFunction)PyModule_REEMRegisterBaseScanData, METH_VARARGS,
    "Register (or deregister) a callback function to get base laser scan data. If target frame is not given, raw data is returned." },
  { "registerTiltScanCallback", (PyCFunction)PyModule_REEMRegisterTiltScanData, METH_VARARGS,
    "Register (or deregister) a callback function to get tilt laser scan data. If target frame is not given, raw data is returned." },
  { "registerPalFaceCallback", (PyCFunction)PyModule_REEMRegisterPalFaceData, METH_VARARGS,
    "Register (or deregister) a callback function to get PAL built-in face detector." },
  { "startPalFaceEnrollment", (PyCFunction)PyModule_REEMStartPalFaceEnrollment, METH_VARARGS,
    "Start PAL built-in face enrollment." },
  { "stopPalFaceEnrollment", (PyCFunction)PyModule_REEMStopPalFaceEnrollment, METH_NOARGS,
    "Stop PAL built-in face enrollment." },
  { "directToWeb", (PyCFunction)PyModule_REEMDirectToWeb, METH_VARARGS,
    "Direct REEM chest screen to a URI." },
#define DEFINE_COMMON_PYMODULE_METHODS
#include "../pyridecore/PyModulePyCommon.cpp"
  { NULL, NULL, 0, NULL }           /* sentinel */
};

PyREEMModule::PyREEMModule() : PyModuleExtension( "PyREEM" )
{
  baseScanCB_ = tiltScanCB_ = palFaceCB_ = NULL;
}

PyREEMModule::~PyREEMModule()
{
  if (baseScanCB_) {
    Py_DECREF( baseScanCB_ );
    baseScanCB_ = NULL;
  }
  if (tiltScanCB_) {
    Py_DECREF( tiltScanCB_ );
    tiltScanCB_ = NULL;
  }
  if (palFaceCB_){
    Py_DECREF( palFaceCB_ );
    palFaceCB_ = NULL;
  }
}

PyObject * PyREEMModule::createPyModule()
{
  return Py_InitModule3( "PyREEM", PyModule_methods, PyREEM_doc );
}

PyREEMModule * PyREEMModule::instance()
{
  if (!s_pyREEMModule)
    s_pyREEMModule = new PyREEMModule();
    
  return s_pyREEMModule;
}
  
void PyREEMModule::invokeBaseScanCallback( PyObject * arg )
{
  this->invokeCallbackHandler( baseScanCB_, arg );
}

void PyREEMModule::invokeTiltScanCallback( PyObject * arg )
{
  this->invokeCallbackHandler( tiltScanCB_, arg );
}

void PyREEMModule::invokePalFaceCallback( PyObject * arg )
{
  this->invokeCallbackHandler( palFaceCB_, arg );
}

void PyREEMModule::setBaseScanCallback( PyObject * obj )
{
  this->swapCallbackHandler( baseScanCB_, obj );
}

void PyREEMModule::setTiltScanCallback( PyObject * obj )
{
  this->swapCallbackHandler( tiltScanCB_, obj );
}

void PyREEMModule::setPalFaceCallback( PyObject * obj )
{
  this->swapCallbackHandler( palFaceCB_, obj );
}
} // namespace pyride
