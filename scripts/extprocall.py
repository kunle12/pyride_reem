import os
import stat
import signal
import subprocess
import time
import PyREEM
import constants

RECORDED_DATA_DIR = '/home/pal/recordings'

class ProcConduit:
  def __init__( self ):
    self.recording = None

    if not os.path.exists( RECORDED_DATA_DIR ) or not os.path.isdir( RECORDED_DATA_DIR ):
      print 'Create data recording directory', RECORDED_DATA_DIR
      try:
        os.makedirs( RECORDED_DATA_DIR )
        os.chmod( RECORDED_DATA_DIR, stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP | stat.S_IWGRP )
        os.chown( RECORDED_DATA_DIR, -1, 100 )
      except:
        print 'Unable to create data recording directory', RECORDED_DATA_DIR
    self.setCallbacks()

  def spawnProc( self, cmd ):
    # The os.setsid() is passed in the argument preexec_fn so
    # it's run after the fork() and before  exec() to run the shell.
    pro = subprocess.Popen( cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid )
    return pro

  def killProc( self, proc ):
    if not proc or not isinstance( proc, subprocess.Popen ):
      print 'Input is not a process object'
      return

    os.killpg( proc.pid, signal.SIGINT )  # Send the signal to all the process groups

  def enableAlive( self, ison ):
    if ison:
      subprocess.call( 'headManagerStart.sh > /dev/null 2>&1', shell=True )
      subprocess.call( 'rosrun dynamic_reconfigure dynparam set alive_engine disabled False', shell=True )
    else:
      subprocess.call( 'headManagerStop.sh > /dev/null 2>&1', shell=True )
      subprocess.call( 'rosrun dynamic_reconfigure dynparam set alive_engine disabled True', shell=True )

  def startDataRecording( self, mode, filename = "" ):
    cmd = 'rosbag record -b 1024 '
    str = ''
    if mode & constants.REC_CAM:
      #cmd = cmd + '-e "/(.*)_stereo/(left|right)/image_rect_color" '
      cmd = cmd + '-e "/stereo/left/image" ' # record only one camera data
      str = '_cam'
    #if mode & constants.REC_KINECT:
      #cmd = cmd + '"/camera/rgb/image_rect_color" "/camera/depth_registered/image_rect" '
      #str = '_kinect'
    if mode & constants.REC_SCAN:
      cmd = cmd + '-e "/scan$" -e "/hokuyo/LAS_01" '
      str = str + '_laser'
    if mode & constants.REC_IMU: #temp disable for now.
      cmd = cmd + '"/mobile_base_controller/odom" '
      str = str + '_imu'
    if mode & constants.REC_JOINTS:
      cmd = cmd + '"/joint_states" '
      str = str + '_joint'
    if mode & constants.REC_TF:
      cmd = cmd + '"/tf" '
      str = str + '_tf'

    if filename == "":
      cmd = cmd + '--duration=1m --split -O %s/%s%s_data.bag' % \
        (RECORDED_DATA_DIR, time.strftime( "%Y%m%d_%H%M", time.localtime()), str)
    else:
      cmd = cmd + '--duration=1m --split -O %s/%s.bag' % \
        (RECORDED_DATA_DIR, filename)

    if self.recording:
      self.killProc( self.recording )

    self.recording = self.spawnProc( cmd )
    PyREEM.say( "Start data recording!" )

  def stopDataRecording( self ):
    if self.recording:
      self.killProc( self.recording )
      self.recording = None
      PyREEM.say( "Stopped data recording!" )

  def setCallbacks( self ):
    PyREEM.startDataRecording = self.startDataRecording
    PyREEM.stopDataRecording = self.stopDataRecording
    PyREEM.setAliveEngineOn = self.enableAlive

  def fini( self ):
    self.stopDataRecording()
