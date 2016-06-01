/*
 *  PyREEMServer.h
 *  PyREEMServer
 *
 *  Created by Xun Wang on 24/05/2016.
 *  Copyright 2016 Galaxy Network. All rights reserved.
 *
 */

#ifndef PyREEMServer_h_DEFINED
#define PyREEMServer_h_DEFINED

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>
#include <netdb.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <pyride_common_msgs/NodeStatus.h>

#include "ServerDataProcessor.h"

namespace pyride {

using namespace ros;

class PyREEMServer : public PyRideExtendedCommandHandler {
public:
  PyREEMServer();
  virtual ~PyREEMServer();
  bool init();
  void fini();
  
  void continueProcessing();
  
  void nodeStatusCB( const pyride_common_msgs::NodeStatusConstPtr & msg );

  void stopProcess();

private:
  bool isRunning_;
  NodeHandle * hcNodeHandle_;
  Subscriber nodeStatusSub_;

  VideoDeviceList activeVideoDevices_;
  AudioDeviceList activeAudioDevices_;

  bool initVideoDevices();
  void finiVideoDevices();
  
  void notifySystemShutdown();

  bool executeRemoteCommand( PyRideExtendedCommand command,
                            const unsigned char * optionalData = NULL,
                            const int optionalDataLength = 0 );

  void cancelCurrentOperation();
  
};
}; // namespace pyride
#endif // PyREEMServer_h_DEFINED
