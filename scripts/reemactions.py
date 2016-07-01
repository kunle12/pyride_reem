import PyREEM
import math

class Actionlet( object ):
  def __init__( self, name ):
    self.name = name
    self.action_finishes = True

  def play( self ):
    pass

  def onFinishingAction( self, type ):
    self.action_finishes = True

  def isActionCompleted( self ):
    return self.action_finishes

  def reset( self ):
    self.action_finishes = True

class PointHeadToAction( Actionlet ):
  def __init__( self ):
    super(PointHeadToAction, self).__init__( "PointHeadToAction" )
    self.reset()
    self.target_x = None
    self.target_y = None
    self.target_z = None

  def setTargetPosition( self, x, y, z ):
    self.target_x = x
    self.target_y = y
    self.target_z = z

  def play( self ):
    if not self.target_x:
      print "No position set for target"
      self.action_finishes = True
      return

    if not self.action_finishes:
      print "still playing action '%s'" % self.name
      return

    self.action_finishes = False
    PyREEM.pointHeadTo( "odom", self.target_x, self.target_y, self.target_z )

class PrimitiveHeadAction( Actionlet ):
  def __init__( self, name, headaction ):
    super(PrimitiveHeadAction, self).__init__( name )
    self.head_action = headaction

  def play( self ):
    if not self.action_finishes:
      print "still playing action '%s'" % self.name
      return

    if self.head_action:
      self.action_finishes = False
      if isinstance( self.head_action, list ):
        PyREEM.moveArmWithJointTrajectory( self.head_action )
      else:
        PyREEM.moveArmWithJointPos( **(self.head_action) )

class PrimitiveTorsoAction( Actionlet ):
  def __init__( self, name, torsoaction ):
    super(PrimitiveTorsoAction, self).__init__( name )
    self.torso_action = torsoaction

  def play( self ):
    if not self.action_finishes:
      print "still playing action '%s'" % self.name
      return

    if self.torso_action:
      self.action_finishes = False
      if isinstance( self.torso_action, list ):
        PyREEM.moveArmWithJointTrajectory( self.torso_action )
      else:
        PyREEM.moveArmWithJointPos( **(self.torso_action) )

class PrimitiveArmAction( Actionlet ):
  def __init__( self, name, armaction ):
    super(PrimitiveArmAction, self).__init__( name )
    self.arm_action = armaction

  def play( self ):
    if not self.action_finishes:
      print "still playing action '%s'" % self.name
      return

    if self.arm_action:
      self.action_finishes = False
      if isinstance( self.arm_action, list ):
        PyREEM.moveArmWithJointTrajectory( self.arm_action )
      else:
        PyREEM.moveArmWithJointPos( **(self.arm_action) )

class PrimitiveHandAction( Actionlet ):
  def __init__( self, name, handaction ):
    super(PrimitiveHandAction, self).__init__( name )
    self.hand_action = handaction

  def play( self ):
    if not self.action_finishes:
      print "still playing action '%s'" % self.name
      return

    if self.hand_action:
      self.action_finishes = False
      PyREEM.setHandPostion( **(self.hand_action) )

class PrimitiveSpeakAction( Actionlet ):
  def __init__( self, name, text ):
    super(PrimitiveSpeakAction, self).__init__( name )
    self.text = text

  def play( self ):
    if not self.action_finishes:
      print "still playing action '%s'" % self.name
      return

    if self.text:
      self.action_finishes = False
      PyREEM.say( text )

class ActionPlayerDelegate( object ):
  def onActionTrackCompleted( self, name ):
     pass

class ActionPlayer( object ):
  def __init__( self, delegate = None ):
    self.actiontracks = {}
    self.playingtrack = None

    #save existing callback functions, if any.
    self.existingArmActionHandler = PyREEM.onMoveArmActionSuccess
    self.existingHeadActionHandler = PyREEM.onHeadActionSuccess
    self.existingHandActionHandler = PyREEM.onHandActionSuccess
    self.existingTorsoActionHandler = PyREEM.onMoveTorsoSuccess
    self.existingBodyActionHandler = PyREEM.onMoveBodySuccess
    self.existingSpeakActionHandler = PyREEM.onSpeakSuccess
    self.existingNavigateActionHandler = PyREEM.onNavigateBodySuccess

    PyREEM.onMoveArmActionSuccess = self.onArmActionComplete
    PyREEM.onHeadActionSuccess = self.onHeadActionComplete
    PyREEM.onHandActionSuccess = self.onHandActionComplete
    PyREEM.onMoveTorsoSuccess = self.onTorsoActionComplete
    PyREEM.onSpeakSuccess = self.onSpeakActionComplete
    PyREEM.onMoveBodySuccess = self.onMoveBodyComplete
    PyREEM.onNavigateBodySuccess = self.onNavigateBodyComplete

    if delegate and isinstance( delegate, ActionPlayerDelegate ):
      self.delegate = delegate
    else:
      self.delegate = None

  def setDelegate( self, delegate ):
    if isinstance( delegate, ActionPlayerDelegate ):
      self.delegate = delegate

  def createActionTrack( self, name ):
    if name in self.actiontracks:
      print "addActionTrack: track named '%s' has already been created" % name
      return False

  def appendActionToTrack( self, name, newaction, delay = 0 ):
    if not isinstance( newaction, Actionlet ):
      print "the second input is not an Actionlet"
      return False

    if not isinstance( delay, float ) or delay < 0:
      print "the third input is not a non negative float number"
      return False

    if not name in self.actiontracks:
      print "track named '%s' does not exihandactionst." % name
      return False

    (track, idx) = self.actiontracks[name]
    track.append( (newaction, delay) )
    return True

  def listTrackName( self ):
    return self.actiontracks.keys()

  def isTrackCompleted( self, name ):
    if not name in self.actiontracks:
      print "track named '%s' does not exist." % name
      return False

    (track, idx) = self.actiontracks[name]
    return (len(track) == idx)

  def isPlayingTrack( self ):
    return (self.playingtrack != None)

  def playTrack( self, name ):
    if not name in self.actiontracks:
      print "track named '%s' does not exist." % name
      return False

    if self.playingtrack == name:
      print "track '%s' is already playing." % name
      return True

    self.playingtrack = name
    (track, idx) = self.actiontracks[name]

    (action, delay) = track[idx] #ignore the first delay
    action.play()
    return True

  def resetTrack( self, name ):
    if not name in self.actiontracks:
      print "track named '%s' does not exist." % name
      return False

    if self.playingtrack == name:
      print "stopping track '%s'." % name
      self.playingtrack = None

    (track, idx) = self.actiontracks[name]
    self.actiontracks[name] = (track, 0)
    return True

  def deleteActionTrack( self, name ):
    if not name in self.actiontracks:
      print "track named '%s' does not exist." % name
      return False

    del self.actiontracks[name]
    return True

  def removeActionFromTrack( self, name, idx ):
    if not name in self.actiontracks:
      print "track named '%s' does not exist." % name
      return False

    (track, ind) = self.actiontracks[name]
    if idx >= len(track):
      print "invalid Actionlet index %d for action track %s" % (idx, name)
      return False

    del track[idx]
    print "removing Actionlet at index %d on action track %s" % (idx, name)
    return True

  def clearActionTracks( self ):
    self.actiontracks = {}

  def swapActionTracks( self, new_tracks ):
    old_tracks = self.actiontracks.copy()
    self.actiontracks = new_tracks
    return old_tracks

  def onSpeakActionComplete( self ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    (action, delay) = track[idx]
    action.onFinishingAction( "speak" )

    self.updateTrackStatus( action, track, idx )

  def onArmActionComplete( self, isleft ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    (action, delay) = track[idx]
    action.onFinishingAction( "arm_{}".format( "left" if isleft else "right"))

    self.updateTrackStatus( action, track, idx )

  def onHandActionComplete( self, isleft ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    (action, delay) = track[idx]
    action.onFinishingAction( "hand_{}".format( "left" if isleft else "right"))

    self.updateTrackStatus( action, track, idx )

  def onNavigateBodyComplete( self ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    (action, delay) = track[idx]
    action.onFinishingAction( "navigate" )

    self.updateTrackStatus( action, track, idx )

  def onMoveBodyComplete( self ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    (action, delay) = track[idx]
    action.onFinishingAction( "move" )

    self.updateTrackStatus( action, track, idx )

  def onHeadActionComplete( self ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    (action, delay) = track[idx]
    action.onFinishingAction( "head" )

    self.updateTrackStatus( action, track, idx )

  def onTorsoActionComplete( self ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    (action, delay) = track[idx]
    action.onFinishingAction( "torso" )

    self.updateTrackStatus( action, track, idx )

  def updateTrackStatus( self, action, track, idx ):
    if action.isActionCompleted():
      idx = idx + 1
      if idx >= len(track):  #track completed
        self.actiontracks[self.playingtrack] = (track, len(track))
        if self.delegate:
          self.delegate.onActionTrackCompleted( self.playingtrack )
        self.playingtrack = None
      else:
        (action, delay) = track[idx]
        if delay > 0:
            time.sleep( delay )
        action.play()
        self.actiontracks[self.playingtrack] = (track, idx)

  def fini( self ):
    self.clearActionTracks()
    PyREEM.onMoveArmActionSuccess = self.existingArmActionHandler
    PyREEM.onHeadActionSuccess = self.existingHeadActionHandler
    PyREEM.onHandActionSuccess = self.existingHandActionHandler
    PyREEM.onMoveTorsoSuccess = self.existingTorsoActionHandler
    PyREEM.onSpeakSuccess = self.existingSpeakActionHandler
    PyREEM.onMoveBodySuccess = self.existingBodyActionHandler
    PyREEM.onNavigateBodySuccess = self.existingNavigateActionHandler
