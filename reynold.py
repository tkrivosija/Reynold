#!/usr/bin/env python3


import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import random


# definding parameters for boid area
RADIUS = 2
ANGLE = 180
MAX_VEL = 0.3

# coefficients of Reynold_rule
k_separation = 0.5
k_alignment = 0.5
k_cohesion = 2

# twist_publishers
publishers = {}

# robots positoins
robot_positions = {}

#robot x-axis positions
robot_xpositions = {}

# robots velocities
robot_velocity = {}

# three reynold's rule

def reynold_function(robot_number):

    # initialize
    separation = (0,0)
    alignment = (0,0)
    cohesion = (0,0)
    separation_sum = (0,0)
    alignment_sum = (0,0)
    cohesion_sum = (0,0)
    neighbour_count = 0

    # get position of observed robot
    boid_poes = robot_positions[robot_number]
    boid_vel = robot_velocity[robot_number]
    
    for key, position in robot_positions.items():
          if key != robot_number:
              distance = ((position.x - boid_poes.x) ** 2 + (position.y - boid_poes.y) ** 2) ** 0.5
 
              if distance < RADIUS:
                  relative_pose = (position.x - boid_poes.x, position.y - boid_poes.y)
                  relative_angle = math.atan2(relative_pose[1], relative_pose[0])
                  angle_degrees = math.degrees(relative_angle % (2 * math.pi))
 
                  if 0 <= angle_degrees <= ANGLE / 2 or 360 - ANGLE / 2 <= angle_degrees <= 360:
                      # SEPARATION
                      dx = boid_poes.x - position.x
                      dy = boid_poes.y - position.y
                      separation_sum = (separation_sum[0] + dx/(pow(dx,2)+0.000000001), separation_sum[1] + dy/(pow(dy,2)+0.000000001))
 
                      # ALIGNMENT
                      alignment_sum = (alignment_sum[0] + robot_velocity[key].x - boid_vel.x, alignment_sum[1] + robot_velocity[key].y - boid_vel.y)
 
                      # COHESION
                      cohesion_sum = (cohesion_sum[0] + position.x - boid_poes.x, cohesion_sum[1] + position.y - boid_poes.y)
                      neighbour_count += 1

    if neighbour_count > 0:
        separation = (separation_sum[0] / neighbour_count, separation_sum[1] / neighbour_count)
        alignment = (alignment_sum[0] / neighbour_count, alignment_sum[1] / neighbour_count)
        cohesion = (cohesion_sum[0] / neighbour_count, cohesion_sum[1] / neighbour_count)

    # init publisher
    if not (robot_number in publishers.keys()):
          init_publisher(robot_number)

    # create message for new velocity
    if boid_poes.x == max(robot_xpositions.values()): # this is leader
        vel_msg = Twist()
        vel_msg.linear.x = MAX_VEL
        vel_msg.linear.y = 0
    else:
        vel_msg = Twist()
        vel_msg.linear.x = separation[0] * k_separation + alignment[0] * k_alignment + cohesion[0] * k_cohesion
        vel_msg.linear.y = separation[1] * k_separation + alignment[1] * k_alignment + cohesion[1] * k_cohesion
    publishers[robot_number].publish(vel_msg)



def init_publisher(robot_number):
    pub = rospy.Publisher('/robot_{}/cmd_vel'.format(robot_number), Twist, queue_size = 10)
    publishers[robot_number] = pub
    

# get information about location of boids
def callback(msg, robot_number):
    robot_positions[robot_number] = msg.pose.pose.position
    robot_xpositions[robot_number] = msg.pose.pose.position.x
    robot_velocity[robot_number] = msg.twist.twist.linear
    reynold_function(robot_number)


if __name__ == '__main__':   
    rospy.init_node('reynold_rule')

    # get location of boid
    num_of_robots = rospy.get_param("/num_of_robots")
    [rospy.Subscriber("/robot_{}/odom".format(i), Odometry, callback, callback_args = i, queue_size = 10) for i in range(num_of_robots)]

    rospy.spin()

