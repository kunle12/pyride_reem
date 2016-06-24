import PyREEM
import math

class Actionlet( object ):
  def __init__( self, name ):
    self.name = name

  def play( self ):
    pass

  def finishsHeadAction( self ):
    pass

  def finishsNavigateBodyAction( self ):
    pass

  def finishsMoveBodyAction( self ):
    pass

  def finishsGripperAction( self ):
    pass

  def finishsArmAction( self, is_left ):
    pass

  def isActionCompleted( self ):
    return True

  def reset( self ):
    pass

class PointHeadToAction( Actionlet ):
  def __init__( self ):
    super(PointHeadToAction, self).__init__( "PointHeadToAction" )
    self.action_finishes = True
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

    PyREEM.pointHeadTo( "odom", self.target_x, self.target_y, self.target_z )

  def finishsHeadAction( self ):
    self.action_finishes = True

  def isActionCompleted( self ):
    return self.action_finishes

  def reset( self ):
    self.action_finishes = True

class PrimitiveArmAction( Actionlet ):
  def __init__( self, name, armaction, handaction, isleft ):
    super(PrimitiveArmAction, self).__init__( name )
    self.arm_action = armaction
    self.hand_action = handaction
    self.is_left = isleft
    self.reset()

  def play( self ):
    if not self.isarm_finished or not self.ishandper_finished:
      print "still playing action '%s'" % self.name
      return

    if self.arm_action:
      if isinstance( self.arm_action, list ):
        PyREEM.moveArmWithJointTrajectory( self.arm_action )
      else:
        PyREEM.moveArmWithJointPos( **(self.arm_action) )
      self.isarm_finished = False
    if self.hand_action != None:
      if self.is_left:
        PyREEM.setGripperPosition( 1, self.hand_action )
      else:
        PyREEM.setGripperPosition( 2, self.hand_action )

      self.ishandper_finished = False

  def finishsGripperAction( self ):
    self.ishandper_finished = True

  def finishsArmAction( self ):
    self.isarm_finished = True

  def isActionCompleted( self ):
    return (self.isarm_finished and self.ishandper_finished)

  def reset( self ):
    self.isarm_finished = True
    self.ishandper_finished = True

class ActionPlayerDelegate( object ):
  def onActionTrackCompleted( self, name ):
     pass

class ActionPlayer( object ):
  def __init__( self, delegate = None ):
    self.actiontracks = {}
    self.playingtrack = None
    if delegate and isinstance( delegate, ActionPlayerDelegate ):
      self.delegate = delegate
    else:
      self.delegate = None

  def setDelegate( self, delegate ):
    if isinstance( delegate, ActionPlayerDelegate ):
      self.delegate = delegate

  def addActionTrack( self, name, track ):
    if name in self.actiontracks:
      print "addActionTrack: track named '%s' is already added" % name
      return False

    if not isinstance( track, list ) or len( track ) == 0:
      print "addActionTrack: track must be a non-empty list."
      return False

    for ac in track:
      if not isinstance( ac, Actionlet ):
        print "addActionTrack: track contains no Actionlet."
        return False

    self.actiontracks[name] = (track, 0)
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

    track[idx].play()
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

  def appendActionToTrack( self, name, newaction ):
    if not isinstance( newaction, Actionlet ):
      print "the second input is not an Actionlet"
      return False

    if not name in self.actiontracks:
      print "track named '%s' does not exist." % name
      return False

    (track, idx) = self.actiontracks[name]
    track.append( newaction )
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

  def onArmActionComplete( self, isleft ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    action = track[idx]
    action.finishsArmAction()

    self.updateTrackStatus( action, track, idx )

  def onGripperActionComplete( self, isleft ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    action = track[idx]
    action.finishsGripperAction()

    self.updateTrackStatus( action, track, idx )

  def onNavigateBodyComplete( self ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    action = track[idx]
    action.finishsNavigateBodyAction()

    self.updateTrackStatus( action, track, idx )

  def onMoveBodyComplete( self ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    action = track[idx]
    action.finishsMoveBodyAction()

    self.updateTrackStatus( action, track, idx )

  def onHeadActionComplete( self ):
    if self.playingtrack == None:
      return

    (track, idx) = self.actiontracks[self.playingtrack]
    action = track[idx]
    action.finishsHeadAction()

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
        track[idx].play()
        self.actiontracks[self.playingtrack] = (track, idx)
