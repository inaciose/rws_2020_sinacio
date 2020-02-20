#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class Player:
    def __init__(self, player_name):
        self.player_name = player_name
        rospy.loginfo("I am " + self.player_name)

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
        
def callback(msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)
    print("recieved a message containing string " + msg.data)


def main():
    print("hello world node")
    rospy.init_node('sinacio', anonymous=False)
    player = Player("sinacio")

    rospy.Subscriber("chatter", String, callback)

    rospy.spin()

if __name__ == "__main__":
    main()