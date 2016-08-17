import PyREEM
import math
import constants
import tinstate
import messenger
import tininfo
import tinmind
import extprocall
import iksresolver
import ipkspawner

from timers import timermanager

myMessenger = None
extProcCall = None
iksResolver = None
ipkSpawner = None
msgTryTimer = -1

def userLogon( name ):
  PyREEM.say( '%s has logged on.' % name )
  tinstate.updateStatus( constants.USER_PRESENT, False )

def userLogoff( name ):
  PyREEM.say( '%s has logged off.' % name )
  tinstate.updateStatus( constants.USER_PRESENT, len(PyREEM.listCurrentUsers()) == 0)

def remoteCommandActions( cmd, arg ):
  pass

def nodeStatusUpdate( data ):
  global ipkSpawner
  if data['node'] == 'jupyter':
    ipkSpawner.spawnkernel( data['message'] )
  #process messages from external ROS nodes here.
  #data['node'] data['priority'] data['message'] data['timestamp']

def timerLapsedActions( id ):
  timermanager.onTimerLapsed( id )

def timerActions( id ):
  global myMessenger, msgTryTimer

  if msgTryTimer == id and myMessenger.checkin():
    PyREEM.removeTimer( msgTryTimer )
    msgTryTimer = -1
  else:
    timermanager.onTimerCall( id )

def powerPlugChangeActions( isplugged ):
  global myMessenger

  text = ""
  if isplugged:
    text = "I'm on main power."
  else:
    text = "I'm on battery power."

  PyREEM.say( text )

  if myMessenger:
    myMessenger.updatestatus( text )

def batteryChargeChangeActions( batpc, isplugged, time_remain ):
  global myMessenger

  if batpc < 20 and not isplugged:
    PyREEM.say( "I'm low on battery, please put me back on main power." )

    if myMessenger:
      myMessenger.updatestatus( "I have only %d percent battery power left!" % batpc )

def systemShutdownActions():
  global myMessenger
  global extProcCall

  PyREEM.say( 'I am going off line. Goodbye.' )

  myMessenger.checkout()
  extProcCall.fini()

def main():
  global myMessenger, msgTryTimer
  global extProcCall, iksResolver
  global ipkSpawner

  extProcCall = extprocall.ProcConduit()
  iksResolver = iksresolver.IKSResolver()
  ipkSpawner = ipkspawner.IPKSpawner()

  PyREEM.onUserLogOn = userLogon
  PyREEM.onUserLogOff = userLogoff
  PyREEM.onTimer = timerActions
  PyREEM.onTimerLapsed = timerLapsedActions
  PyREEM.onRemoteCommand = remoteCommandActions
  PyREEM.onSystemShutdown = systemShutdownActions
  PyREEM.onPowerPluggedChange = powerPlugChangeActions
  PyREEM.onBatteryChargeChange = batteryChargeChangeActions
  PyREEM.onNodeStatusUpdate = nodeStatusUpdate

  myMessenger = messenger.Messenger()
  if not myMessenger.checkin():
    msgTryTimer = PyREEM.addTimer( 10*60, -1, 10*60 )

  PyREEM.say( constants.INTRO_TEXT )
  PyREEM.setLowPowerThreshold( 20 )
