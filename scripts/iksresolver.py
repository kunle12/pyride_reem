import os
import sys
import math
import PyREEM

MagiksREEMPath = 'Magiks/magiks/projects/s_pr2'

class IKSError( Exception ):
  pass

class IKSResolver( object ):
  def __init__( self ):
    self.spr2_obj = None
    self.iks_in_use = 0
    self.np = None
    self.pint = None
    self.geometry = None
    self.resolveIKS()
    self.useSREEM()

  def getArmPose( self, left_arm ):
    if self.iks_in_use == 2:
      self.spr2_obj.sync_object()
      pos = None
      orient = None
      if left_arm:
        pos = tuple(self.spr2_obj.larm_end_position())
        orient = self.geometry.Orientation_3D( self.spr2_obj.larm_end_orientation(), representation = 'matrix' )
      else:
        pos = tuple(self.spr2_obj.rarm_end_position())
        orient = self.geometry.Orientation_3D( self.spr2_obj.rarm_end_orientation(), representation = 'matrix' )
      pose = {'position': pos, 'orientation': tuple(orient.quaternion())}
      return pose
    else:
      if left_arm:
        return PyREEM.getRelativeTF( '/base_footprint', '/hand_left_grasping_frame' )
      else:
        return PyREEM.getRelativeTF( '/base_footprint', '/hand_right_grasping_frame' )

  def moveArmInTrajectory( self, traj, time = 10.0, left_arm = False, relative = False ):
    if self.iks_in_use != 2:
      raise IKSError( 'This function is only available with S-REEM.' )

    if time <= 1.0:
      raise IKSError( 'Invalid execution time.' )

    if not isinstance( traj, list ) or len( traj ) == 0:
      raise IKSError( 'Input trajectory must be a non-empty list of pose (dictionary)' )

    pos_traj = self.traj.Trajectory_Polynomial()
    orient_traj = self.traj.Orientation_Trajectory_Polynomial()

    for idx, pose in enumerate(traj):
      if not isinstance( pose, dict ) or not pose.has_key( 'position' ) or not isinstance(pose['position'], tuple) or len(pose['position']) != 3:
        print 'invalid pose position at {0}'.format( idx )
      else:
        pos_traj.add_point(phi = float(idx), pos = self.np.array(pose['position']))
      # optional
      if pose.has_key( 'orientation' ) and isinstance(pose['orientation'], tuple) and len(pose['orientation']) == 4:
        orient_traj.add_point(phi = float(idx), ori = self.geometry.Orientation_3D( pose['orientation'], representation = 'quaternion' ) )

    #pos_traj.consistent_velocities()

    if left_arm:
      jt = self.spr2_obj.larm.project_to_js( pos_traj, orient_traj, relative = relative )
      pos_traj.consistent_velocities()
      jt.consistent_velocities()
      self.spr2_obj.run_config_trajectory(jt, is_left_arm = True, duration = time)
    else:
      jt = self.spr2_obj.rarm.project_to_js( pos_traj, orient_traj, relative = relative )
      jt.consistent_velocities()
      self.spr2_obj.run_config_trajectory(jt, is_left_arm = False, duration = time)

  def moveArmWithSREEM( self, **kwargs ):
    if not self.spr2_obj:
      return False

    if not kwargs.has_key( 'position' ) or not kwargs.has_key( 'orientation' ) or not kwargs.has_key( 'use_left_arm'):
      print 'Invalid input argument'
      return False

    self.spr2_obj.arm_speed = 0.1

    if kwargs.has_key( 'wait' ):
      wait = kwargs['wait']
    else:
      wait = True

    if kwargs['use_left_arm']:
      self.spr2_obj.larm_reference = True
    else:
      self.spr2_obj.larm_reference = False

    self.spr2_obj.sync_object()
    arm_orient = self.geometry.Orientation_3D( kwargs['orientation'], representation = 'quaternion' )
    self.spr2_obj.set_target( self.np.array(kwargs['position']), arm_orient.matrix() )
    return self.spr2_obj.arm_target(wait = wait)

  def dummyMoveArmTo( self, wait = True, **kwargs ):
    raise IKSError( 'NO IKS solver is available to PyRIDE' )

  def resetMotionCallbacks( self ):
    if self.iks_in_use == 2:
      self.pint.set_callback_functions()

  def resolveIKS( self ):
    PyREEM.moveArmTo = self.dummyMoveArmTo
    PyREEM.getArmPose = self.getArmPose
    PyREEM.moveArmInTrajectory = self.moveArmInTrajectory
    iksPath = os.path.join( sys.path[0], MagiksREEMPath )
    if os.path.exists( iksPath ):
      sys.path.append('/usr/local/lib/python2.7/dist-packages/')
      sys.path.append('/usr/lib/python2.7/dist-packages/')
      sys.path.append('/usr/lib/pymodules/python2.7')
      sys.path.append(iksPath)
      try:
        import initialize
        from magiks.specific_geometries.pr2 import pyride_synchronizer as pys
        import numpy as np
        from math_tools.geometry import geometry
        from math_tools.geometry import trajectory as traj

        self.np = np
        self.pint = pys.pint
        self.geometry = geometry
        self.spr2_obj = pys.PyRide_REEM()
        self.traj = traj
      except:
        print 'unable to load S-REEM/Magiks engine'
        self.spr2_obj = None

  def useSREEM( self ):
    if self.spr2_obj:
      PyREEM.moveArmTo = self.moveArmWithSREEM
      self.iks_in_use = 2
      print 'PyRIDE is using S-REEM for REEM'

  def useMoveIt( self ):
    if PyREEM.useMoveIt():
      PyREEM.moveArmTo = PyREEM.moveArmPoseTo
      self.iks_in_use = 1
      print 'PyRIDE is using MoveIt! for REEM'

  def iksInUse( self ):
    if self.iks_in_use == 1:
      return 'MoveIt!'
    elif self.iks_in_use == 2:
      return 'S-REEM'
    else:
      return 'None'
