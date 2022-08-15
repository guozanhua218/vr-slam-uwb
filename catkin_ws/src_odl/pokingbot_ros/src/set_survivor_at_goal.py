#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Float64
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import SetModelConfigurationRequest, SetModelConfiguration, SetModelState, GetModelState, GetPhysicsProperties, SetPhysicsProperties, SetPhysicsPropertiesRequest, GetJointProperties

class subvrandpub(object):
 
    def __init__(self):
        self.twist = Twist()
        self.twist.linear.x=0
        self.twist.linear.y=0
        self.twist.linear.z=0
        self.twist.angular.x=0
        self.twist.angular.y=0
        self.twist.angular.z=0
        self.reset_model = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_model1 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_model2 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.reset_model3 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.reset_model4 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.reset_model5 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.reset_model6 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.reset_model7 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.reset_model8 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.reset_model9 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.reset_model10 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # self.reset_model11 = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.sub_goal1 = rospy.Subscriber("/survivor1", PoseStamped, self.callback_survivor1)
        self.sub_goal2 = rospy.Subscriber("/survivor2", PoseStamped, self.callback_survivor2)
        self.sub_goal3 = rospy.Subscriber("/survivor3", PoseStamped, self.callback_survivor3)
        self.sub_goal4 = rospy.Subscriber("/survivor4", PoseStamped, self.callback_survivor4)
        # self.sub_obj1 = rospy.Subscriber("/backpack", PoseStamped, self.callback_backpack)
        # self.sub_obj2 = rospy.Subscriber("/backpack0", PoseStamped, self.callback_backpack0)
        # self.sub_obj3 = rospy.Subscriber("/backpack1", PoseStamped, self.callback_backpack1)
        # self.sub_obj4 = rospy.Subscriber("/gas", PoseStamped, self.callback_gas)
        # self.sub_obj5 = rospy.Subscriber("/gas0", PoseStamped, self.callback_gas0)
        # self.sub_obj6 = rospy.Subscriber("/phone", PoseStamped, self.callback_phone)
        # self.sub_obj7 = rospy.Subscriber("/vent", PoseStamped, self.callback_vent)
        # self.sub_obj8 = rospy.Subscriber("/vent0", PoseStamped, self.callback_vent0)
 
    def callback_survivor1(self, data):
        try: 
            state_msg = ModelState()
            state_msg.model_name = 'Survivor Male'
            state_msg.reference_frame = "world"
            state_msg.pose = data.pose
            state_msg.twist = self.twist
            state_msg.pose.position.z += 0.9
            state_msg.pose.orientation.x = -0.77119666338
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 0.636597156525
            self.reset_model(state_msg)
        except(rospy.ServiceException) as e: print(e)
    
    def callback_survivor2(self, data):
        try: 
            state_msg = ModelState()
            state_msg.model_name = 'Survivor Male_0'
            state_msg.reference_frame = "world"
            state_msg.pose = data.pose
            state_msg.twist = self.twist
            state_msg.pose.position.z += 0.9
            state_msg.pose.orientation.x = -0.77119666338
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 0.636597156525
            self.reset_model1(state_msg)
        except(rospy.ServiceException) as e: print(e)

    def callback_survivor3(self, data):
        try: 
            state_msg = ModelState()
            state_msg.model_name = 'Survivor Male_1'
            state_msg.reference_frame = "world"
            state_msg.pose = data.pose
            state_msg.twist = self.twist
            state_msg.pose.position.z += 0.9
            state_msg.pose.orientation.x = -0.77119666338
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 0.636597156525
            self.reset_model2(state_msg)
        except(rospy.ServiceException) as e: print(e)

    def callback_survivor4(self, data):
        try: 
            state_msg = ModelState()
            state_msg.model_name = 'Survivor Male_2'
            state_msg.reference_frame = "world"
            state_msg.pose = data.pose
            state_msg.twist = self.twist
            state_msg.pose.position.z += 0.9
            state_msg.pose.orientation.x = -0.77119666338
            state_msg.pose.orientation.y = 0
            state_msg.pose.orientation.z = 0
            state_msg.pose.orientation.w = 0.636597156525
            self.reset_model3(state_msg)
        except(rospy.ServiceException) as e: print(e)

    # def callback_backpack(self, data):
    #     try: 
    #         state_msg = ModelState()
    #         state_msg.model_name = 'Backpack'
    #         state_msg.reference_frame = "world"
    #         state_msg.pose = data.pose
    #         state_msg.twist = self.twist
    #         state_msg.pose.position.z += 0
    #         state_msg.pose.orientation.x = -0.77119666338
    #         state_msg.pose.orientation.y = 0
    #         state_msg.pose.orientation.z = 0
    #         state_msg.pose.orientation.w = 0.636597156525
    #         self.reset_model4(state_msg)
    #     except(rospy.ServiceException) as e: print(e)

    # def callback_backpack0(self, data):
    #     try: 
    #         state_msg = ModelState()
    #         state_msg.model_name = 'Backpack_0'
    #         state_msg.reference_frame = "world"
    #         state_msg.pose = data.pose
    #         state_msg.twist = self.twist
    #         state_msg.pose.position.z += 0
    #         state_msg.pose.orientation.x = -0.77119666338
    #         state_msg.pose.orientation.y = 0
    #         state_msg.pose.orientation.z = 0
    #         state_msg.pose.orientation.w = 0.636597156525
    #         self.reset_model5(state_msg)
    #     except(rospy.ServiceException) as e: print(e)

    # def callback_backpack1(self, data):
    #     try: 
    #         state_msg = ModelState()
    #         state_msg.model_name = 'Backpack_1'
    #         state_msg.reference_frame = "world"
    #         state_msg.pose = data.pose
    #         state_msg.twist = self.twist
    #         # state_msg.pose.position.z += 0
    #         state_msg.pose.orientation.x = -0.77119666338
    #         state_msg.pose.orientation.y = 0
    #         state_msg.pose.orientation.z = 0
    #         state_msg.pose.orientation.w = 0.636597156525
    #         self.reset_model6(state_msg)
    #     except(rospy.ServiceException) as e: print(e)

    # def callback_gas(self, data):
    #     try: 
    #         state_msg = ModelState()
    #         state_msg.model_name = 'Extinguisher'
    #         state_msg.reference_frame = "world"
    #         state_msg.pose = data.pose
    #         state_msg.twist = self.twist
    #         # state_msg.pose.position.z += 0
    #         state_msg.pose.orientation.x = -0.77119666338
    #         state_msg.pose.orientation.y = 0
    #         state_msg.pose.orientation.z = 0
    #         state_msg.pose.orientation.w = 0.636597156525
    #         self.reset_model7(state_msg)
    #     except(rospy.ServiceException) as e: print(e)

    # def callback_gas0(self, data):
    #     try: 
    #         state_msg = ModelState()
    #         state_msg.model_name = 'Extinguisher_0'
    #         state_msg.reference_frame = "world"
    #         state_msg.pose = data.pose
    #         state_msg.twist = self.twist
    #         # state_msg.pose.position.z += 0
    #         state_msg.pose.orientation.x = -0.77119666338
    #         state_msg.pose.orientation.y = 0
    #         state_msg.pose.orientation.z = 0
    #         state_msg.pose.orientation.w = 0.636597156525
    #         self.reset_model8(state_msg)
    #     except(rospy.ServiceException) as e: print(e)

    # def callback_phone(self, data):
    #     try: 
    #         state_msg = ModelState()
    #         state_msg.model_name = 'Phone'
    #         state_msg.reference_frame = "world"
    #         state_msg.pose = data.pose
    #         state_msg.twist = self.twist
    #         # state_msg.pose.position.z += 0
    #         state_msg.pose.orientation.x = -0.77119666338
    #         state_msg.pose.orientation.y = 0
    #         state_msg.pose.orientation.z = 0
    #         state_msg.pose.orientation.w = 0.636597156525
    #         self.reset_model9(state_msg)
    #     except(rospy.ServiceException) as e: print(e)

    # def callback_vent(self, data):
    #     try: 
    #         state_msg = ModelState()
    #         state_msg.model_name = 'vent'
    #         state_msg.reference_frame = "world"
    #         state_msg.pose = data.pose
    #         state_msg.twist = self.twist
    #         # state_msg.pose.position.z += 0
    #         state_msg.pose.orientation.x = -0.77119666338
    #         state_msg.pose.orientation.y = 0
    #         state_msg.pose.orientation.z = 0
    #         state_msg.pose.orientation.w = 0.636597156525
    #         self.reset_model10(state_msg)
    #     except(rospy.ServiceException) as e: print(e)

    # def callback_vent0(self, data):
    #     try: 
    #         state_msg = ModelState()
    #         state_msg.model_name = 'vent_0'
    #         state_msg.reference_frame = "world"
    #         state_msg.pose = data.pose
    #         state_msg.twist = self.twist
    #         # state_msg.pose.position.z += 0
    #         state_msg.pose.orientation.x = -0.77119666338
    #         state_msg.pose.orientation.y = 0
    #         state_msg.pose.orientation.z = 0
    #         state_msg.pose.orientation.w = 0.636597156525
    #         self.reset_model11(state_msg)
    #     except(rospy.ServiceException) as e: print(e)

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    SUB = subvrandpub()
    rospy.spin() #run forever