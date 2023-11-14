#!/usr/bin/env python3
import time
import rospy
import subprocess
import yaml
from dynamic_reconfigure.server import Server
from reynold.cfg import dynamicVariablesConfig
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import random
import math


class projectManager:

    def __init__(self):
        self.dynamicVariables = None  # Initialize the variable as a class attribute
        rospy.init_node('reynoldMain')

        srv = Server(dynamicVariablesConfig, self.reconfigure_callback)
        self.rate = rospy.Rate(1)

        self.robotPublishers = {}
        self.robotSubscribers = {}
        self.robotPosition = {}
        self.robotVelocity = {}
        self.robot_xpositions = {}

        
        
    def botOdomUpdate(self, data, robot_id):
        #rospy.loginfo("Received Odometry from Robot %d: %s", data)
        #rospy.loginfo("Robot%d,  =  %s",robot_id, data)
        if(robot_id < self.botNumber):
            self.robotPosition[robot_id] = data.pose.pose.position
            self.robotVelocity[robot_id] = data.twist.twist.linear
            self.robot_xpositions[robot_id] = data.pose.pose.position.x

        #vel_msg = self.reynold_function(robot_id)
        if(self.dynamicVariables.globalEnable):
            vel_msg = self.reynold_function2(robot_id)
            self.robotPublishers[robot_id].publish(vel_msg)
    
    def restartRobotPublishersAndSubscribers(self):
        #kill previous subscribers and publishers...
        for subscriber in self.robotSubscribers.values():
            subscriber.unregister()
        time.sleep(0.2)
        [pub.unregister() for pub in self.robotPublishers.values()]
        #rospy.loginfo(self.robotPublishers)

        time.sleep(0.2)
        #[rospy.Subscriber("/robot_{}/odom".format(robot_number), Odometry, self.botOdomUpdate, callback_args = robot_number, queue_size = 10) for robot_number in range(self.dynamicVariables.botNumber)]
        

        self.robotPublishers.clear()
        self.robotPublishers.clear()
        self.robotPublishers = {robot_number: rospy.Publisher('/robot_{}/cmd_vel'.format(robot_number), Twist, queue_size=10) for robot_number in range(self.dynamicVariables.botNumber)}
        self.robotSubscribers = {robot_number: rospy.Subscriber("/robot_{}/odom".format(robot_number), Odometry, self.botOdomUpdate, callback_args=robot_number, queue_size=10) for robot_number in range(self.dynamicVariables.botNumber)}

    def reboot(self):

        rospy.loginfo(self.dynamicVariables.map_name)
        rospy.loginfo(self.dynamicVariables.botNumber)
        rospy.loginfo(self.dynamicVariables.distribution)
        rospy.loginfo(self.dynamicVariables.circle_center_x)
        rospy.loginfo(self.dynamicVariables.circle_center_y)
        rospy.loginfo(self.dynamicVariables.circle_radius)
        rospy.loginfo(self.dynamicVariables.line_center_x)
        rospy.loginfo(self.dynamicVariables.line_center_y)
        rospy.loginfo(self.dynamicVariables.line_separation)
        rospy.loginfo(self.dynamicVariables.line_direction)

        # Load the YAML file
        with open('/home/developer/catkin_ws/src/sphero_simulation/sphero_stage/launch/launch_params.yaml', 'r') as file:
            config = yaml.safe_load(file)


        # Modify the parameters as needed
        config['map_name'] = self.dynamicVariables.map_name
        config['num_of_robots'] = self.dynamicVariables.botNumber
        config['distribution'] = self.dynamicVariables.distribution

        # Modify the 'distribution_list' based on your requirements
        config['distribution_list'] = {
            'circle': {
                'center_x': self.dynamicVariables.circle_center_x,
                'center_y': self.dynamicVariables.circle_center_y,
                'radius': self.dynamicVariables.circle_radius
            },
            'line': {
                'center_x': self.dynamicVariables.line_center_x,
                'center_y': self.dynamicVariables.line_center_y,
                'separation': self.dynamicVariables.line_separation,
                'direction': self.dynamicVariables.line_direction
            }
        }

        # Save the modified configuration back to the file
        with open('/home/developer/catkin_ws/src/sphero_simulation/sphero_stage/launch/launch_params.yaml', 'w') as file:
            yaml.dump(config, file)

        node_name = 'sphero_stage'
        package = 'sphero_stage'
        executable = 'start.py'





        self.start_node(node_name, package, executable)


        self.botNumber = self.dynamicVariables.botNumber
        self.restartRobotPublishersAndSubscribers()
        
        #rospy.Publisher('/robot_{}/cmd_vel'.format(robot_number), Twist, queue_size = 10)
        #self.rebootRobotPublishersAndSubscribers(self.dynamicVariables.botNumber)
        #for robot_id in range (0, self.dynamicVariables.botNumber):
        #    rospy.loginfo("In the loop")
        #    rospy.Subscriber("/robot_{}/odom".format(robot_id), Odometry, self.botOdomUpdate, robot_id)

    def simpleRandom(self, robot_number):
        vel_msg = Twist()
        amplitude = 1
        #vel_msg.linear.x =  random.random() * amplitude - 0.5*amplitude# you can remove random
        vel_msg.linear.x =  1
        vel_msg.linear.y =  1
        return vel_msg
    
    def reynold_function2(self, robot_number):
        # initialize
        separation = (0,0)
        alignment = (0,0)
        cohesion = (0,0)
        separation_sum = (0,0)
        alignment_sum = (0,0)
        cohesion_sum = (0,0)
        neighbour_count = 0

        # get position of observed robot
        boid_poes = self.robotPosition[robot_number]
        boid_vel = self.robotVelocity[robot_number]
        
        for key, position in self.robotPosition.items():
            if key != robot_number:
                distance = ((position.x - boid_poes.x) ** 2 + (position.y - boid_poes.y) ** 2) ** 0.5
    
                if distance < self.dynamicVariables.BOID_RADIUS:
                    relative_pose = (position.x - boid_poes.x, position.y - boid_poes.y)
                    relative_angle = math.atan2(relative_pose[1], relative_pose[0])
                    angle_degrees = math.degrees(relative_angle % (2 * math.pi))
    
                    if 0 <= angle_degrees <= self.dynamicVariables.BOID_ANGLE / 2 or 360 - self.dynamicVariables.BOID_ANGLE / 2 <= angle_degrees <= 360:
                        # SEPARATION
                        dx = boid_poes.x - position.x
                        dy = boid_poes.y - position.y
                        separation_sum = (separation_sum[0] + dx/(pow(dx,2)+0.000000001), separation_sum[1] + dy/(pow(dy,2)+0.000000001))
    
                        # ALIGNMENT
                        alignment_sum = (alignment_sum[0] + self.robotVelocity[key].x - boid_vel.x, alignment_sum[1] + self.robotVelocity[key].y - boid_vel.y)
    
                        # COHESION
                        cohesion_sum = (cohesion_sum[0] + position.x - boid_poes.x, cohesion_sum[1] + position.y - boid_poes.y)
                        neighbour_count += 1

        if neighbour_count > 0:
            separation = (separation_sum[0] / neighbour_count, separation_sum[1] / neighbour_count)
            alignment = (alignment_sum[0] / neighbour_count, alignment_sum[1] / neighbour_count)
            cohesion = (cohesion_sum[0] / neighbour_count, cohesion_sum[1] / neighbour_count)


        vel_msg = Twist()
        # create message for new velocity
        if boid_poes.x == max(self.robot_xpositions.values()): # this is leader
            vel_msg.linear.x = random.uniform(-0.5, 0.5) * self.dynamicVariables.BOID_MAX_VEL
            vel_msg.linear.y = random.uniform(-0.5, 0.5) * self.dynamicVariables.BOID_MAX_VEL
        else:
            vel_msg.linear.x = separation[0] * self.dynamicVariables.separation + alignment[0] * self.dynamicVariables.alignment + cohesion[0] * self.dynamicVariables.cohesion
            vel_msg.linear.y = separation[1] * self.dynamicVariables.separation + alignment[1] * self.dynamicVariables.alignment + cohesion[1] * self.dynamicVariables.cohesion
        return vel_msg
        
    def reynold_function(self, robot_number):
        separation = (0,0)
        alignment = (0,0)
        cohesion = (0,0)
        neigbour_count = 0

        # get position of observed robot
        #rospy.loginfo(self.robotPosition)
        boid_poes = self.robotPosition[robot_number]
        
        # check if robots is in range
        for key, position in self.robotPosition.items():   # position -> position of other robots
            if key != robot_number:
                distance = ((position.x - boid_poes.x) ** 2 + (position.y - boid_poes.y) ** 2) ** 0.5
                # check if it is in neigbour radius
                if distance < self.dynamicVariables.BOID_RADIUS:
                    # check if it is in angle view
                    relative_pose = (position.x - boid_poes.x,position.y - boid_poes.y)
                    relative_angle = math.atan2(relative_pose[1], relative_pose[0])
                    if math.degrees(relative_angle%(2*math.pi))<=self.dynamicVariables.BOID_ANGLE/2 or math.degrees(relative_angle%(2*math.pi))>=360-(self.dynamicVariables.BOID_ANGLE/2):
                        # SEPARATION
                        dx = boid_poes.x - position.x
                        dy = boid_poes.y - position.y
                        diff = (dx ** 2 + dy ** 2) ** 0.5
                        separation = (dx / diff, dy / diff)

                        # ALIGNMENT
                        alignment += (self.robotVelocity[key].x, self.robotVelocity[key].y)

                        # COHESION
                        cohesion += (self.robotPosition[key].x, self.robotPosition[key].y)
                        neigbour_count += 1

        if neigbour_count > 0:
            alignment = (alignment[0] / neigbour_count, alignment[1] / neigbour_count)
            cohesion = (cohesion[0] - boid_poes.x, cohesion[1] - boid_poes.y)
            len_cohesion = (cohesion[0] ** 2 + cohesion[1] ** 2) ** 0.5
            cohesion = (cohesion[0] / len_cohesion, cohesion[1] / len_cohesion) #ovdje se nekad desi 0 u nazivniku
        
        
        # create message for new velocity
        vel_msg = Twist()
        vel_msg.linear.x = (separation[0] * self.dynamicVariables.separation + alignment[0] * self.dynamicVariables.alignment + cohesion[0] * self.dynamicVariables.cohesion) + random.random() * 0.01 # you can remove random
        vel_msg.linear.y = (separation[1] * self.dynamicVariables.separation + alignment[1] * self.dynamicVariables.alignment + cohesion[1] * self.dynamicVariables.cohesion) + random.random() * 0.01
        return vel_msg
        
        
        
    def kill_node(self, node_name):
        try:
            # Use subprocess to kill the ROS node
            subprocess.check_call(['rosnode', 'kill', node_name])
            rospy.loginfo(f"Killed node: {node_name}")
        except subprocess.CalledProcessError:
            rospy.logerr(f"Failed to kill node: {node_name}")

    def start_node(self, node_name, package, executable):
        try:
            # Use subprocess to start the ROS node
            subprocess.Popen(['rosrun', package, executable], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            rospy.loginfo(f"Started node: {node_name}")
        except Exception as e:
            rospy.logerr(f"Failed to start node: {node_name} - {str(e)}")


    def reconfigure_callback(self, config, level):

        
        if(self.dynamicVariables and not self.dynamicVariables.reboot_params and config.reboot_params):
            self.reboot()
        self.dynamicVariables = config

        
        return config

    def run(self):
        while not rospy.is_shutdown():
            #ukoliko su se učitale dinamičke varijable i dopušten je globalni enabl vrti glavnu petlju...
            
            #for robot_id in range (0, self.botNumber):
            #    if(self.dynamicVariables is not None and self.dynamicVariables.globalEnable):
            #        vel_msg = self.reynold_function(robot_id)
            #        self.robotPublishers[robot_id].publish(vel_msg)
            #    else:
            #        break
                
            
            time.sleep(0.1)
        rospy.spin()

if __name__ == '__main__':
    node = projectManager()
    node.run()

