#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
REFERENCE_FRAME = 'world'
from std_msgs.msg import String
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest, ExecuteKnownTrajectoryResponse

from std_msgs.msg import String
from moveit_msgs.msg._RobotTrajectory import RobotTrajectory

def join_trajectories(traj1, traj2):
    """Given two RobotTrajectory join their joints and steps
    and return a RobotTrajectory that fuses them"""
    joined_traj = copy.deepcopy(traj1)
    # add all the names
    joined_traj.joint_trajectory.joint_names.extend(traj2.joint_trajectory.joint_names)
    print "traj1 size:"
    print len(traj1.joint_trajectory.points)
    
    print "traj2 size:"
    print len(traj2.joint_trajectory.points)
    
    if len(traj1.joint_trajectory.points) != len(traj2.joint_trajectory.points):
        rospy.logerr("Trajectories of different size, TODO, MERGE THEM MORE SMARTLY")
    
    # Add all the positions
    idx = 0
    for point in traj2.joint_trajectory.points:
        print "joined_traj.joint_trajectory.points[idx]"
        print joined_traj.joint_trajectory.points[idx]
        new_positions = []
        new_positions.extend(joined_traj.joint_trajectory.points[idx].positions)
        new_positions.extend(point.positions)
        joined_traj.joint_trajectory.points[idx].positions = new_positions
        idx += 1
        
    
    return joined_traj










def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)
  box_id ='box'
  table_id ='table'
  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  ## Instantiate a MoveGroupCommander object.  This object is an interface
  ## to one group of joints.  In this case the group is the joints in the left
  ## arm.  This interface can be used to plan and execute motions on the left
  ## arm.
  group = moveit_commander.MoveGroupCommander("arm_left")
  group_r = moveit_commander.MoveGroupCommander("arm_right")
  #group.set_pose_reference_frame('torso_link_b1')

  ## We create this DisplayTrajectory publisher which is used below to publish
  ## trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)

  ## Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
  print "============ Waiting for RVIZ..."
  rospy.sleep(2)
  print "============ Starting tutorial "

  ## Getting Basic Information
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^
  ##
  ## We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group.get_planning_frame()

  ## We can also print the name of the end-effector link for this group
  print "============ Reference frame: %s" % group.get_end_effector_link()

  ## We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  ## Sometimes for debugging it is useful to print the entire state of the
  ## robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"


  ## Planning to a Pose goal
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the 
  ## end-effector
  print "============ Generating plan 1"


  box_size = [0.37, 0.46, 0.33]
 # Add a table top and two boxes to the scene
  box_pose = PoseStamped()
  box_pose.header.frame_id = REFERENCE_FRAME
  box_pose.pose.position.x = 1
  box_pose.pose.position.y = 0
  box_pose.pose.position.z = 1
  box_pose.pose.orientation.w = 1.0
  #scene.add_box(box_id, box_pose, box_size)


 # add table
  table_size = [1.6, 1.6, 1.7]
 # Add a table top and two boxes to the scene
  table_pose = PoseStamped()
  table_pose.header.frame_id = REFERENCE_FRAME
  table_pose.pose.position.x = 1.2
  table_pose.pose.position.y = 0
  table_pose.pose.position.z = 0
  table_pose.pose.orientation.w = 1.0
  scene.add_box(table_id, table_pose, table_size)
 
 
  rospy.sleep(5)

  pose_target = geometry_msgs.msg.Pose()
  
  pose_target.position.x = 1
  pose_target.position.y = -0.23
  pose_target.position.z = 1.004
  
  pose_target.orientation.x = 0.449
  pose_target.orientation.y = 0.459
  pose_target.orientation.z = 0.533
  pose_target.orientation.w = 0.550






  group.set_pose_target(pose_target)

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
  plan1 = group.plan()
  pose_target_r = geometry_msgs.msg.Pose()
  
  pose_target_r.position.x = 1
  pose_target_r.position.y = 0.23
  pose_target_r.position.z = 1.004
  
  pose_target_r.orientation.x = 0.449
  pose_target_r.orientation.y = -0.459
  pose_target_r.orientation.z = -0.533
  pose_target_r.orientation.w = 0.550

  group_r.set_pose_target(pose_target_r)

  

  

  group.go()
  rospy.sleep(5)
  group_r.go()


  group.clear_pose_targets()
  rospy.sleep(1)
  group_r.clear_pose_targets()

  #rospy.sleep(10)
  print "============ Generating plan 2"
  rospy.sleep(2)

  waypoints = []
  rospy.sleep(1)
  waypoints.append(group.get_current_pose().pose)
  
  wpose = geometry_msgs.msg.Pose()
  
  wpose.position.x = waypoints[0].position.x 
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z+0.2
  wpose.orientation.x = 0.449
  wpose.orientation.y = 0.459
  wpose.orientation.z = 0.533
  wpose.orientation.w = 0.550
  waypoints.append(copy.deepcopy(wpose))
  (plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold 



  waypoints_r= []
  rospy.sleep(1)
  waypoints_r.append(group_r.get_current_pose().pose)
  wpose_r = geometry_msgs.msg.Pose()
  
  wpose_r.position.x = waypoints_r[0].position.x 
  wpose_r.position.y = waypoints_r[0].position.y
  wpose_r.position.z = waypoints_r[0].position.z+0.2
  
  wpose_r.orientation.x = 0.449
  wpose_r.orientation.y = -0.459
  wpose_r.orientation.z = -0.533
  wpose_r.orientation.w = 0.550
  #plan_2=group.plan()
  # start with the current pose
  
  waypoints_r.append(copy.deepcopy(wpose_r))
  # first orient gripper and move forward (+x)
  

  # second move down
  #wpose.position.z += 0.10
  #wpose_r.position.z += 0.10
  #waypoints.append(copy.deepcopy(wpose))
  #waypoints_r.append(copy.deepcopy(wpose_r))

  #group.go(wait=True)
  #group_r.go(wait=True)
 
  (plan4, fraction) = group_r.compute_cartesian_path(
                             waypoints_r,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
  #group.go()
  #group_r.go()
  #plan3=group.plan()
 # plan4=group_r.plan()


  rospy.sleep(1)
    
  finalplan = join_trajectories(plan3, plan4)
  print "final plan looks like: " + str(finalplan)

  ekp = rospy.ServiceProxy('/execute_kinematic_path', ExecuteKnownTrajectory)
  ekp.wait_for_service()
     
  print "!!!! Gonna send BOTH ARMS goal to execute_kinematic_path"
  #rospy.sleep(3)
  ektr = ExecuteKnownTrajectoryRequest()
  #ektr_ = ExecuteKnownTrajectoryRequest()
  
  ektr.trajectory = finalplan
  ektr.wait_for_execution = True
  print "Sending call both arms"
  ekp.call(ektr)
  #ekp.call(ektr_)
  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## First, we will define the collision object message
  #collision_object = moveit_msgs.msg.CollisionObject(box_id)
 # current_scene.addCollisionObject(collision_object)

  collision_object_pub = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject)
  box_collision_object_msg = moveit_msgs.msg.CollisionObject()
  box_collision_object_msg.operation = box_collision_object_msg.MOVE

  box_collision_object_msg.id = 'box'
  box_collision_object_msg.mesh_poses = [box_pose]
  box_collision_object_msg.header.stamp = rospy.Time.now()
  box_collision_object_msg.header.frame_id = REFERENCE_FRAME
  
  #collision_object_pub.publish(box_collision_object_msg)

  table_collision_object_msg = moveit_msgs.msg.CollisionObject()
  table_collision_object_msg.id = 'table'
  table_collision_object_msg.mesh_poses = [table_pose]
  table_collision_object_msg.header.stamp = rospy.Time.now()
  table_collision_object_msg.header.frame_id = REFERENCE_FRAME
  
  collision_object_pub.publish(table_collision_object_msg)
  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
