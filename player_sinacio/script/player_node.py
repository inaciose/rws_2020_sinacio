#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Transform, Quaternion
from rws2020_msgs.msg import MakeAPlay

import math
import random
from random import randrange
import numpy

class Player:
    def __init__(self, player_name):
        self.player_name = player_name
        #self.animal = 'dog'

        self.max_vel = 0
        self.max_angle = math.pi/30

        rospy.Subscriber("make_a_play", MakeAPlay, self.makeAPlayCallback)

        self.br = tf.TransformBroadcaster()
        self.transform = Transform()

        self.transform.translation.x = randrange(8)
        self.transform.translation.y = randrange(8)

        Initial_R = 8 * random.random()
        Initial_Theta = 2 * math.pi * random.random()
        Initial_X = Initial_R * math.cos(Initial_Theta)
        Initial_Y = Initial_R * math.sin(Initial_Theta)
        Initial_Rotation = 2 * math.pi * random.random()
        self.transform.translation.x = Initial_X
        self.transform.translation.y = Initial_Y
        self.transform.rotation= tf.transformations.quaternion_from_euler(0, 0, Initial_Rotation)        

        rospy.loginfo("I am the player" + self.player_name)
        #rospy.loginfo("I am the animal" + self.animal)

        red_team = rospy.get_param('/red_team')
        green_team = rospy.get_param('/green_team')
        blue_team = rospy.get_param('/blue_team')

        print("red_team = " + str(red_team))
        print("green_team = " + str(green_team))
        print("blue_team = " + str(blue_team))

        if self.player_name in red_team:
            self.my_team, self.prey_team, self.hunter_team = 'red', 'green', 'blue'
            self.my_players, self.preys, self.hunters = red_team, green_team, blue_team

        elif self.player_name in green_team:
            self.my_team, self.prey_team, self.hunter_team = 'green', 'blue', 'red'
            self.my_players, self.preys, self.hunters = green_team, blue_team, red_team

        elif self.player_name in blue_team:
            self.my_team, self.prey_team, self.hunter_team = 'blue', 'green', 'red'
            self.my_players, self.preys, self.hunters = blue_team, green_team, red_team

        #if self.player_name in red_team:
        #    self.my_team = 'red'
        #    self.prey_team = 'green'
        #    self.hunter_team = 'blue'
        #elif self.player_name in green_team:
        #    self.my_team = 'green'
        #    self.prey_team = 'blue'
        #    self.hunter_team = 'red'
        #elif self.player_name in blue_team:
        #    self.my_team = 'blue'
        #    self.prey_team = 'red'
        #    self.hunter_team = 'green'

        else:
            rospy.logerr("My name is not in any team list")
            exit(0)

        rospy.loginfo("I am " + self.player_name + " and im in this team " + self.my_team + " hunting " + self.prey_team + " run away from " + self.hunter_team)

    def makeAPlayCallback(self, data):
        print("receive message make a play " + str(data.dog)) 
  
        #self.max_vel = data.dog
        #self.max_angle = math.pi/30

        # set delta angle
        #angle = random.uniform(0, self.max_angle)
        #print(self.max_angle)
        #print(angle)

        #vel = self.max_vel

        self.max_vel = data.dog
        self.max_angle = math.pi / 30
        print('Received message make a play ... my max velocity is ' + str(self.max_vel))

        # Make a play
        vel = self.max_vel  # full throttle
        angle = self.max_angle

        self.move(self.transform, vel/10, angle)
    
    def move(self, transform_now, vel, angle):

        if angle > self.max_angle:
            angle = self.max_angle
        elif angle < -self.max_angle:
            angle = -self.max_angle

        if vel > self.max_vel:
            vel = self.max_vel

        T1 = transform_now

        T2 = Transform()
        T2.rotation = tf.transformations.quaternion_from_euler(0, 0, angle)
        T2.translation.x = vel
        matrix_trans = tf.transformations.translation_matrix((T2.translation.x,
                                                              T2.translation.y,
                                                              T2.translation.z))

        matrix_rot = tf.transformations.quaternion_matrix((T2.rotation[0],
                                                           T2.rotation[1],
                                                           T2.rotation[2],
                                                           T2.rotation[3]))
        matrixT2 = numpy.matmul(matrix_trans, matrix_rot)

        matrix_trans = tf.transformations.translation_matrix((T1.translation.x,
                                                              T1.translation.y,
                                                              T1.translation.z))

        matrix_rot = tf.transformations.quaternion_matrix((T1.rotation.x,
                                                           T1.rotation.y,
                                                           T1.rotation.z,
                                                           T1.rotation.w))
        matrixT1 = numpy.matmul(matrix_trans, matrix_rot)

        matrix_new_transform = numpy.matmul(matrixT2, matrixT1)

        quat = tf.transformations.quaternion_from_matrix(matrix_new_transform)
        trans = tf.transformations.translation_from_matrix(matrix_new_transform)

        self.transform.rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        self.transform.translation.x = trans[0]
        self.transform.translation.y = trans[1]
        self.transform.translation.z = trans[2]

        self.br.sendTransform(trans, quat, rospy.Time.now(),
                              self.player_name, "world")

    #def move(self, transform_now, vel, angle):
    #    Tdeslocamento = self.transform
    #    Tdeslocamento.rotation = tf.transformations.quaternion_from_euler(0,0, angle)
    #    Tdeslocamento.translation.x = vel


def callback(msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
    rospy.loginfo("recieved a message containing string " + msg.data)

#def makeAPlayCallback(data):
#    print("receive message make a play " + str(data.dog)) 

def main():
    print("hello world node")
    rospy.init_node('sinacio', anonymous=False)
    player = Player("sinacio")

    rospy.Subscriber("chatter", String, callback)
    #rospy.Subscriber("make_a_play", MakeAPlay, makeAPlayCallback)

    rospy.spin()

if __name__ == "__main__":
    main()