/*
 *  main.cpp
 *  PyREEMServer
 *
 *  Created by Xun Wang on 09/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#include <ros/ros.h>
#include <signal.h>

#include "PyREEMServer.h"

using namespace pyride;
static PyREEMServer * s_server = NULL;

void stopProcess( int sig )
{
  if (s_server)
    s_server->stopProcess();
}

int main( int argc, char * argv[] )
{
  ros::init( argc, argv, "pyride_reem" );

  s_server = new PyREEMServer();

  signal( SIGINT, ::stopProcess );
  
  s_server->init();

  s_server->continueProcessing();
  
  s_server->fini();
  
  delete s_server;

  ros::shutdown();
  return 0;
}
