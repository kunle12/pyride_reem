/*
 *  AudioObject.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 09/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef AUDIOOJECT_H
#define AUDIOOJECT_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <ccrtp/rtp.h>
#include <alsa/asoundlib.h>

#include "DeviceController.h"

using namespace std;
using namespace ost;

namespace pyride {

class AudioObject : public AudioDevice {
public:
  AudioObject();
  
  bool initDevice();
  void finiDevice();

private:
  struct mmapBufferInfo {
    unsigned char * start;
    size_t length;
  };

  snd_pcm_t * audioDevice_;

  boost::thread * streaming_data_thread_;

  bool setDefaultAudioParameters();
  int runtimeErrorRecovery( int err );
  //void getUDPSourcePorts();
  
  bool initWorkerThread();
  void finiWorkerThread();
  
  void doAudioStreaming();
};
} // namespace pyride
#endif // AUDIOOJECT_H
