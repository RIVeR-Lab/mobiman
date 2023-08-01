#!/usr/bin/python3
import rospy
import tf
from tf2_msgs.msg import TFMessage


def publish_transform(msg):
    known_transform = tf.broadcaster.TransformBroadcaster()
    known_transform.sendTransform(translation=(0, 0, 0), rotation=(0, 0, 0, 1), time=rospy.Time.now(), child='/base_link', parent='/jackal_jaco_body/base_link')


if __name__ == '__main__':
    rospy.init_node("jackal_jaco_transform_publisher")
    sub = rospy.Subscriber('/tf', TFMessage, callback=publish_transform)
    rospy.spin()