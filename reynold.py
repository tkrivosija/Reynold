#!/usr/bin/env python3


import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import random


# definding parameters for boid area
RADIUS = 3
ANGLE = 180

# coefficients of Reynold_rule
k_separation = 0.8
k_alignment = 0.4
k_cohesion = 0.5

# twist_publishers
publishers = {}

# robots positoins
robot_positions = {}

# robots velocities
robot_velocity = {}

# three reynold's rule

def reynold_function(robot_number):

    # initialize
    separation = (0,0)
    alignment = (0,0)
    cohesion = (0,0)
    neigbour_count = 0

    # get position of observed robot
    boid_poes = robot_positions[robot_number]
    
    # check if robots is in range
    for key, position in robot_positions.items():   # position -> position of other robots
        if key != robot_number:
            distance = ((position.x - boid_poes.x) ** 2 + (position.y - boid_poes.y) ** 2) ** 0.5
            # check if it is in neigbour radius
            if distance < RADIUS:
                # check if it is in angle view
                relative_pose = (position.x - boid_poes.x,position.y - boid_poes.y)
                relative_angle = math.atan2(relative_pose[1], relative_pose[0])
                if math.degrees(relative_angle%(2*math.pi))<=ANGLE/2 or math.degrees(relative_angle%(2*math.pi))>=360-(ANGLE/2):
                    # SEPARATION
                    dx = boid_poes.x - position.x
                    dy = boid_poes.y - position.y
                    diff = (dx ** 2 + dy ** 2) ** 0.5
                    separation = (dx / diff, dy / diff)

                    # ALIGNMENT
                    alignment += (robot_velocity[key].x, robot_velocity[key].y)

                    # COHESION
                    cohesion += (robot_positions[key].x, robot_positions[key].y)
                    neigbour_count += 1

    if neigbour_count > 0:
        alignment = (alignment[0] / neigbour_count, alignment[1] / neigbour_count)
        cohesion = (cohesion[0] - boid_poes.x, cohesion[1] - boid_poes.y)
        len_cohesion = (cohesion[0] ** 2 + cohesion[1] ** 2) ** 0.5
        cohesion = (cohesion[0] / len_cohesion, cohesion[1] / len_cohesion) #ovdje se nekad desi 0 u nazivniku
    
    # init publisher
    if not (robot_number in publishers.keys()):
        init_publisher(robot_number)
    
    # create message for new velocity
    vel_msg = Twist()
    vel_msg.linear.x = (separation[0] * k_separation + alignment[0] * k_alignment + cohesion[0] * k_cohesion) + random.random() * 0.01 # you can remove random
    vel_msg.linear.y = (separation[1] * k_separation + alignment[1] * k_alignment + cohesion[1] * k_cohesion) + random.random() * 0.01
    publishers[robot_number].publish(vel_msg)


def init_publisher(robot_number):
    pub = rospy.Publisher('/robot_{}/cmd_vel'.format(robot_number), Twist, queue_size = 10)
    publishers[robot_number] = pub
    

# get information about location of boids
def callback(msg, robot_number):
    robot_positions[robot_number] = msg.pose.pose.position
    robot_velocity[robot_number] = msg.twist.twist.linear
    
    reynold_function(robot_number)


if __name__ == '__main__':
    rospy.init_node('reynold_rule')

    # get location of boid
    num_of_robots = rospy.get_param("/num_of_robots")
    [rospy.Subscriber("/robot_{}/odom".format(i), Odometry, callback, callback_args = i, queue_size = 10) for i in range(num_of_robots)]

    rospy.spin()

