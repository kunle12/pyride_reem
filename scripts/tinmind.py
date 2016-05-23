import PyREEM
import re
import tininfo

def respond( question ):
  q = question.strip().lower()

  if 'ip' in q or 'addr' in q:
    return "My IP Address is %s." % PyREEM.getMyIPAddress()
  elif 'tuck' in q and 'arm' in q:
    PyREEM.tuckBothArms()
    return "I'm tucking my arms."
  elif 'battery' in q and ('how much' in q or 'status' in q):
    (batpc, isplug, timeremain) = PyREEM.getBatteryStatus()
    if isplug != 'unplugged':
      return "I'm currently %s with %d percent battery power and finish charging in approximately %d minutes." % (isplug, batpc, timeremain % 60 )
    else:
      return "I'm currently %s with %d percent battery power and running out battery in approximately %d minutes." % (isplug, batpc, timeremain % 60 )

  return "I don't understand your request."

