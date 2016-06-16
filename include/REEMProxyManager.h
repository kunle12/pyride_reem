/*
 *  REEMProxyManager.h
 *  PyREEMServer
 *
 *  Created by Xun Wang on 24/05/2016.
 *  Copyright 2016 Galaxy Network. All rights reserved.
 *
 */

#ifndef REEM_PROXY_MANAGER_H
#define REEM_PROXY_MANAGER_H

#include <string>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <control_msgs/PointHeadAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/JointTrajectoryAction.h>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <pal_interaction_msgs/TtsAction.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ColorRGBA.h>

#include <pal_detection_msgs/FaceDetections.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>

#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/message_filter.h>
#include <tf/transform_listener.h>


// moveit interface
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <shape_tools/solid_primitive_dims.h>

#ifdef WITH_REEMHT
#include <pr2ht/TrackedObjectStatusChange.h>
#include <pr2ht/TrackedObjectUpdate.h>
#endif

#ifdef WITH_RHYTH_DMP
#include <rhyth_dmp/OutputTrajData.h>
#endif

#include "PyRideCommon.h"

using namespace std;
using namespace ros;
using namespace control_msgs;
using namespace pal_interaction_msgs;
using namespace move_base_msgs;

namespace pyride {

typedef actionlib::SimpleActionClient<pal_interaction_msgs::TtsAction> TTSClient;
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> TrajectoryClient;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FollowTrajectoryClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef enum {
  WHITE = 0,
  BLANK,
  RED,
  BLUE,
  GREEN,
  YELLOW,
  PINK
} REEMLedColour;

class REEMProxyManager
{
public:
  static REEMProxyManager * instance();
  ~REEMProxyManager();

  void initWithNodeHandle( NodeHandle * nodeHandle, bool useOptionNodes = false, bool useMoveIt = false );
    
  void sayWithVolume( const std::string & text, float volume  = 0.0, bool toBlock = false );
  void setAudioVolume( const float vol );

  bool getPositionForJoints( std::vector<std::string> & joint_names,
                            std::vector<double> & positions );
  bool getJointPos( const char * joint_name, double & value );

  bool getHeadPos( double & yaw, double & pitch );
  bool getTorsoPos( double & yaw, double & pitch );

  bool getRobotPose( std::vector<double> & positions,
                    std::vector<double> & orientation );
  bool getRelativeTF( const char * frame1,
                         const char * frame2,
                         std::vector<double> & positions,
                         std::vector<double> & orientation );

  bool moveHeadTo( double yaw, double pitch, bool relative = false, float time_to_reach = 0.5 );
  bool pointHeadTo( const std::string & frame, float x, float y, float z );
  void updateHeadPos( float yaw, float pitch );
  
  bool moveArmWithGoalPose( bool isLeftArm, std::vector<double> & position,
                           std::vector<double> & orientation, float time_to_reach = 10.0 );
  void moveArmWithJointPos( bool isLeftArm, std::vector<double> & positions,
                           float time_to_reach = 5.0 );
  void moveArmWithJointTrajectory( bool isLeftArm,
                                   std::vector< std::vector<double> > & trajectory,
                                   std::vector<float> & times_to_reach );
  void moveArmWithJointTrajectoryAndSpeed( bool isLeftArm,
                                  std::vector< std::vector<double> > & trajectory,
                                  std::vector< std::vector<double> > & joint_velocities,
                                  std::vector<float> & times_to_reach );

  void cancelArmMovement( bool isLeftArm );
  
  void setHeadStiffness( const float stiffness );
  void setArmStiffness( bool isLeftArm, const float stiffness );

  bool addSolidObject( const std::string & name, std::vector<double> & volume,
      std::vector<double> & position, std::vector<double> & orientation );

  void removeSolidObject( const std::string & name );

  void listSolidObjects( std::vector<std::string> & list );

  bool pickupObject( const std::string & name, const std::string & place, std::vector<double> & grasp_pose,
      bool isLeftArm = false, double approach_dist = 0.4 );

  bool placeObject( const std::string & name, const std::string & place, std::vector<double> & place_pose,
      bool isLeftArm = false, double approach_dist = 0.4 );

  bool setHandPosition( bool isLeftHand, std::vector<double> & positions, float time_to_reach );

  bool moveBodyTo( const RobotPose & pose, const float bestTime );

  bool moveTorsoTo( double yaw, double pitch, bool relative = false );

  void updateBodyPose( const RobotPose & pose, bool localupdate = false );
  
  bool navigateBodyTo( const std::vector<double> & positions,
                      const std::vector<double> & orientation );
  
#ifdef WITH_REEMHT
  bool enableHumanDetection( bool enable, bool enableTrackingNotif = false );
#endif
  
#ifdef WITH_RHYTH_DMP
  void subscribeRawTrajInput( bool enable );
#endif

  void cancelBodyMovement();

  void publishCommands();
  
  void getBatteryStatus( float & percentage, bool & isplugged, float & timeremain );

  int getLowPowerThreshold() { return lowPowerThreshold_; }
  void setLowPowerThreshold( int percent );
  
  void registerForPalFaceData();
  void deregisterForPalFaceData();

  void registerForBaseScanData();
  void registerForTiltScanData();
  void registerForBaseScanData( const std::string & target_frame );
  void registerForTiltScanData( const std::string & target_frame );
  void deregisterForBaseScanData();
  void deregisterForTiltScanData();
  
  int setEarLED( const REEMLedColour colour, const int side = 3 );
  int pulseEarLED( const REEMLedColour colour1, const REEMLedColour colour2,
      const int side = 3, const float period = 1.0 );
  bool cancelEarLED( const int effectID );

  bool palFaceStartEnrollment( const std::string & name );
  bool palFaceStopEnrollment();
  void enablePalFaceDetection( bool enable, float confidence = 0.4 );

  void getTFFrameList( std::vector<std::string> & list );
  bool isTFFrameSupported( const char * frame_name );

  bool useMoveIt() const { return (rarmGroup_ != NULL && larmGroup_ != NULL); }
  void fini();

private:
  NodeHandle * mCtrlNode_;
  Publisher mPub_;
  Publisher hPub_;
  Publisher colObjPub_;
  Subscriber jointSub_;
  Subscriber powerSub_;

  Subscriber * rawBaseScanSub_;
  Subscriber * rawTiltScanSub_;

  Subscriber * faceDetectSub_;

  AsyncSpinner * jointDataThread_;
  CallbackQueue jointDataQueue_;

  ServiceClient ledColourClient_;
  ServiceClient ledPulseClient_;
  ServiceClient cancelLedClient_;

  ServiceClient palFaceEnablerClient_;
  ServiceClient palFaceEnrolStartClient_;
  ServiceClient palFaceEnrolStopClient_;

#ifdef WITH_REEMHT
  Subscriber * htObjStatusSub_;
  Subscriber * htObjUpdateSub_;

  ServiceClient htClient_;
#endif

#ifdef WITH_RHYTH_DMP
  Subscriber * dmpTrajDataSub_;
  // use separate thread and message queue to cope with possible high resolution inputs
  AsyncSpinner * dmpTrajThread_;
  CallbackQueue dmpTrajQueue_;

  ServiceClient dmpClient_;
#endif

  message_filters::Subscriber<sensor_msgs::LaserScan> * baseScanSub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> * tiltScanSub_;

  tf::MessageFilter<sensor_msgs::LaserScan> * baseScanNotifier_;
  tf::MessageFilter<sensor_msgs::LaserScan> * tiltScanNotifier_;
  
  TTSClient * soundClient_;
  
  laser_geometry::LaserProjection lprojector_;
  
  boost::mutex bat_mutex_;
  boost::mutex joint_mutex_;
  
  bool bodyCtrlWithOdmetry_;
  bool bodyCtrlWithNavigation_;
  bool torsoCtrl_;
  bool headCtrlWithTrajActionClient_;
  bool headCtrlWithActionClient_;
  bool lHandCtrl_;
  bool rHandCtrl_;
  bool lArmCtrl_;
  bool rArmCtrl_;
  
  float lArmActionTimeout_;
  float rArmActionTimeout_;
  float lHandActionTimeout_;
  float rHandActionTimeout_;
  float bodyActionTimeout_;

  std::string baseScanTransformFrame_;
  std::string tiltScanTransformFrame_;
  
  std::vector<std::string> curJointNames_;
  std::vector<double> curJointPositions_;
  
  tf::TransformListener tflistener_;
  tf::StampedTransform startTransform_;
  
  FollowTrajectoryClient * headClient_;
  FollowTrajectoryClient * torsoClient_;
  FollowTrajectoryClient * lhandClient_;
  FollowTrajectoryClient * rhandClient_;
  FollowTrajectoryClient * mlacClient_;
  FollowTrajectoryClient * mracClient_;

  PointHeadClient * phClient_;
  MoveBaseClient * moveBaseClient_;

  moveit::planning_interface::MoveGroup * rarmGroup_;
  moveit::planning_interface::MoveGroup * larmGroup_;

  std::vector<std::string> solidObjectsInScene_;

  //moveit::planning_interface::PlanningSceneInterface planningSceneInf_;
  
  RobotPose poseTrans_;

  ros::Time cmdTimeStamp_;
  ros::Time bcwoTimeToComplete_; // time to complete bodyCtrlWithOdmetry_

  geometry_msgs::Twist mCmd_;
  double headYawRate_, headPitchRate_;
  double reqHeadYaw_, reqHeadPitch_;
  double reqTorsoYaw_, reqTorsoPitch_;
  double targetYaw_, targetPitch_;
  
  // power state
  bool isCharging_;
  float batCapacity_;
  int lowPowerThreshold_;
  Duration batTimeRemain_;

  static REEMProxyManager * s_pREEMProxyManager;

  REEMProxyManager();
  double clamp( double val, double max );
  double max( double val1, double val2 );
  bool isBodyControlWithOdometryTimeExpired();
  bool isHeadControlWithOdometryTimeExpired();
  
  void doneHeadAction( const actionlib::SimpleClientGoalState & state,
                      const PointHeadResultConstPtr & result );
  void doneHeadTrajAction( const actionlib::SimpleClientGoalState & state,
                       const FollowJointTrajectoryResultConstPtr & result );

  void doneTorsoAction( const actionlib::SimpleClientGoalState & state,
                       const FollowJointTrajectoryResultConstPtr & result );

  void doneMoveLArmAction( const actionlib::SimpleClientGoalState & state,
                          const FollowJointTrajectoryResultConstPtr & result );
  void doneMoveRArmAction( const actionlib::SimpleClientGoalState & state,
                          const FollowJointTrajectoryResultConstPtr & result );
  
  void doneLHandAction( const actionlib::SimpleClientGoalState & state,
                          const FollowJointTrajectoryResultConstPtr & result );
  void doneRHandAction( const actionlib::SimpleClientGoalState & state,
                          const FollowJointTrajectoryResultConstPtr & result );

  void doneSpeakAction( const actionlib::SimpleClientGoalState & state,
                              const TtsResultConstPtr & result );

  void doneNavgiateBodyAction( const actionlib::SimpleClientGoalState & state,
                              const MoveBaseResultConstPtr & result );
  
  void moveLHandActionFeedback( const FollowJointTrajectoryFeedbackConstPtr & feedback );
  void moveRHandActionFeedback( const FollowJointTrajectoryFeedbackConstPtr & feedback );

  void moveLArmActionFeedback( const FollowJointTrajectoryFeedbackConstPtr & feedback );
  void moveRArmActionFeedback( const FollowJointTrajectoryFeedbackConstPtr & feedback );

  void jointStateDataCB( const sensor_msgs::JointStateConstPtr & msg );
  void powerStateDataCB( const diagnostic_msgs::DiagnosticArrayConstPtr & msg );
  void baseScanDataCB( const sensor_msgs::LaserScanConstPtr & msg );
  void tiltScanDataCB( const sensor_msgs::LaserScanConstPtr & msg );
  void palFaceDataCB( const pal_detection_msgs::FaceDetectionsConstPtr & msg );

  bool findSolidObjectInScene( const std::string & name );
  std_msgs::ColorRGBA colour2RGB( const REEMLedColour colour );

#ifdef WITH_REEMHT
  void htObjStatusCB( const pr2ht::TrackedObjectStatusChangeConstPtr & msg );
  void htObjUpdateCB( const pr2ht::TrackedObjectUpdateConstPtr & msg );
#endif

#ifdef WITH_RHYTH_DMP
  void trajectoryDataInputCB( const rhyth_dmp::OutputTrajDataConstPtr & msg );
#endif

};
}; // namespace pyride

#endif // REEM_PROXY_MANAGER_H
