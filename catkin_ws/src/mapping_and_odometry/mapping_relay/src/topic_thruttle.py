#! /usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import MarkerArray
import tf
from tf import TransformListener,TransformerROS
from tf import LookupException, ConnectivityException, ExtrapolationException


class Thruttle():
    def __init__(self):
        self.artifacts = MarkerArray()
        self.poses = PoseArray()
        self.poses.header.frame_id = "global"
        self.count = 0
        self.listener = TransformListener()

        self.pub_poses = rospy.Publisher('poses_thruttle', PoseArray, queue_size=1)
        self.pub_marker = rospy.Publisher('marker_arti_thruttle', MarkerArray, queue_size=1)
        self.sub_arti_marker = rospy.Subscriber("marker_arti_pub",MarkerArray,self.cb_marker,queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(5),self.send_to_server)

    def send_to_server(self,event):
        trans, rot = None, None
        try:
            (trans, rot) = self.listener.lookupTransform(
                'global', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        if trans != None and rot != None:
            self.poses.header.stamp = rospy.Time.now()
            self.poses.header.seq = self.count
            self.count += 1
            pose = Pose()
            pose.position.x = trans[0]
            pose.position.y = trans[1]
            pose.position.z = trans[2]
            pose.orientation.x = rot[0]
            pose.orientation.y = rot[1]
            pose.orientation.z = rot[2]
            pose.orientation.w = rot[3]
            self.poses.poses.append(pose)

        self.pub_poses.publish(self.poses)

        if self.artifacts is not None:
            self.pub_marker.publish(self.artifacts)


    def cb_marker(self,msg):
        self.artifacts = msg



if __name__ == "__main__":
    rospy.init_node("thruttle")
    thruttle = Thruttle()
    rospy.spin()
