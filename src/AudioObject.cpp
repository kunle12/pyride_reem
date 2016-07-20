/*
 *  AudioObject.cpp
 *  PyRIDE
 *
 *  Created by Xun Wang on 9/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>

#include "PyRideCommon.h"
#include "AudioObject.h"

namespace pyride {

AudioObject::AudioObject() :
  AudioDevice( 1 )
{
  streaming_data_thread_ = NULL;
}

bool AudioObject::initDevice()
{
  if (isInitialised_)
    return isInitialised_;

  int err = 0;
  const char * device = "plughw:0,0"; // TODO. hardcoded for now
  err = snd_pcm_open( &audioDevice_, device, SND_PCM_STREAM_CAPTURE, 0 );
  if (err < 0) {
    ERROR_MSG( "Unable to open audio capturing device %s (%s).\n", device,
              snd_strerror( err ) );
    return false;
  }

  if (!this->setDefaultAudioParameters()) {
    ERROR_MSG( "Unable to set default audio setting to device %s.\n", device );
    return false;
  }
  if (snd_pcm_start( audioDevice_ ) < 0) {
    ERROR_MSG( "Unable to start the audio device %s.\n", device );
    return false;
  }

  packetStamp_ = 0;
  clientNo_ = 0;
  isInitialised_ = true;

  INFO_MSG( "Robot audio is successfully initialised.\n" );

  return true;
}

bool AudioObject::initWorkerThread()
{
  if (!isInitialised_)
    return false;

  if (streaming_data_thread_)
    return true;

  streaming_data_thread_ = new boost::thread( &AudioObject::doAudioStreaming, this );

  return true;
}

void AudioObject::finiWorkerThread()
{
  if (streaming_data_thread_) {
    streaming_data_thread_->join();
    delete streaming_data_thread_;
    streaming_data_thread_ = NULL;
  }
}
  
void AudioObject::doAudioStreaming()
{
  snd_pcm_sframes_t frames;

  unsigned char * audioBuffers = new unsigned char[PYRIDE_AUDIO_FRAME_SIZE*8];

  while (isStreaming_) {
    frames = snd_pcm_readi( audioDevice_, (void *)audioBuffers, (PYRIDE_AUDIO_FRAME_SIZE*2) );
    if (frames < 0) {
      int err =  runtimeErrorRecovery( frames );
      if (err < 0) {
        ERROR_MSG( "Unable to recover from runtime error, exit loop!\n" );
        isStreaming_ = false;
      }
      continue;
    }
    //printf( "nof of frames %d\n", frames );
    this->processAndSendAudioData( (short*)audioBuffers, frames );
  }
  delete [] audioBuffers;
}

bool AudioObject::setDefaultAudioParameters()
{
  int err = 0;

  snd_pcm_hw_params_t * audioHWParams = NULL;
  snd_pcm_sw_params_t * audioSWParams = NULL;

  err = snd_pcm_hw_params_malloc( &audioHWParams );
  if (err < 0) {
    ERROR_MSG( "Unable to allocate audio parameter (%s).\n", snd_strerror( err ) );
    return false;
  }
  err = snd_pcm_hw_params_any( audioDevice_, audioHWParams );
  if (err < 0) {
    ERROR_MSG( "Unable to initialize audio parameter structure (%s).\n", snd_strerror( err ) );
    snd_pcm_hw_params_free( audioHWParams );
    return false;
  }

  err = snd_pcm_hw_params_set_access( audioDevice_, audioHWParams, SND_PCM_ACCESS_RW_INTERLEAVED );
  if (err < 0) {
    ERROR_MSG( "Unable to set audio interleaved MMAP access (%s).\n", snd_strerror( err ) );
    snd_pcm_hw_params_free( audioHWParams );
    return false;
  }

  err = snd_pcm_hw_params_set_format( audioDevice_, audioHWParams, (snd_pcm_format_t)SND_PCM_FORMAT_S16_LE );
  if (err < 0) {
    ERROR_MSG( "Unable to set audio sample format (%s).\n", snd_strerror( err ) );
    snd_pcm_hw_params_free( audioHWParams );
    return false;
  }

  err = snd_pcm_hw_params_set_rate_near( audioDevice_, audioHWParams, (unsigned int*)&(aSettings_.sampling), 0 );
  if (err < 0) {
    ERROR_MSG( "Unable to set audio sample rate (%s).\n", snd_strerror( err ) );
    snd_pcm_hw_params_free( audioHWParams );
    return false;
  }

  err = snd_pcm_hw_params_set_channels( audioDevice_, audioHWParams, aSettings_.channels );
  if (err < 0) {
    ERROR_MSG( "Unable to set audio channel count (%s).\n", snd_strerror( err ) );
    snd_pcm_hw_params_free( audioHWParams );
    return false;
  }

  err = snd_pcm_hw_params( audioDevice_, audioHWParams );
  if (err < 0) {
    ERROR_MSG( "Unable to set audio parameters (%s).\n", snd_strerror( err ) );
    snd_pcm_hw_params_free( audioHWParams );
    return false;
  }
  snd_pcm_hw_params_free( audioHWParams );

  err = snd_pcm_sw_params_malloc( &audioSWParams );
  if (err < 0) {
    ERROR_MSG( "Unable to allocate software parameters structure (%s).\n",
              snd_strerror( err ) );
    return false;
  }
  err = snd_pcm_sw_params_current( audioDevice_, audioSWParams );
  if (err < 0) {
    ERROR_MSG( "Unable to initialize software parameters structure (%s).\n",
              snd_strerror( err ) );
    snd_pcm_sw_params_free( audioSWParams );
    return false;
  }
  err = snd_pcm_sw_params_set_avail_min( audioDevice_, audioSWParams, PYRIDE_AUDIO_FRAME_SIZE );
  if (err < 0) {
    ERROR_MSG( "Unable to set minimum available count (%s).\n",
              snd_strerror( err ) );
    snd_pcm_sw_params_free( audioSWParams );
    return false;
  }
  err = snd_pcm_sw_params_set_start_threshold( audioDevice_, audioSWParams, 0U );
  if (err < 0) {
    ERROR_MSG( "Unable to set start mode (%s).\n", snd_strerror( err ) );
    snd_pcm_sw_params_free( audioSWParams );
    return false;
  }
  err = snd_pcm_sw_params_set_silence_threshold( audioDevice_, audioSWParams, 0U );
  if (err < 0) {
    ERROR_MSG( "Unable to set set silence threshold (%s).\n", snd_strerror( err ) );
    snd_pcm_sw_params_free( audioSWParams );
    return false;
  }
  snd_pcm_uframes_t boundary;
  if (snd_pcm_sw_params_get_boundary( audioSWParams, &boundary ) == 0) {
    err = snd_pcm_sw_params_set_silence_size( audioDevice_, audioSWParams, boundary );
    if (err < 0) {
      ERROR_MSG( "Unable to set set silence buffer size (%s).\n", snd_strerror( err ) );
      snd_pcm_sw_params_free( audioSWParams );
      return false;
    }
  }
  err = snd_pcm_sw_params( audioDevice_, audioSWParams );
  if (err < 0) {
    ERROR_MSG( "Unable to set audio software parameters (%s).\n", snd_strerror( err ) );
    snd_pcm_sw_params_free( audioSWParams );
    return false;
  }

  snd_pcm_sw_params_free( audioSWParams );

  err = snd_pcm_prepare( audioDevice_ );
  if (err < 0) {
    ERROR_MSG( "Unable to prepare audio interface for use (%s).\n", snd_strerror( err ) );
    return false;
  }
  return true;
}

int AudioObject::runtimeErrorRecovery( int err )
{
  if (err == -EPIPE) {    /* over-run */
    err = snd_pcm_prepare( audioDevice_ );
    if (err < 0) {
      ERROR_MSG( "AudioObject::runtimeErrorRecovery: Unable to recover "
                "from overrun, prepare failed: %s\n", snd_strerror( err ) );
    }
    return 0;
  }
  else if (err == -ESTRPIPE) {
    while ((err = snd_pcm_resume( audioDevice_ )) == -EAGAIN) sleep( 1 );       /* wait until the suspend flag is released */
    if (err < 0) {
      err = snd_pcm_prepare( audioDevice_ );
      if (err < 0) {
        ERROR_MSG( "AudioObject::runtimeErrorRecovery: Unable to recover "
                  "from suspend, prepare failed: %s\n", snd_strerror( err ) );
      }
    }
    return 0;
  }
  return err;
}

void AudioObject::finiDevice()
{
  if (!isInitialised_)
    return;

  if (isStreaming_) {
    isStreaming_ = false;
  }
  
  packetStamp_ = 0;

  isInitialised_ = false;
}
} // namespace pyride
