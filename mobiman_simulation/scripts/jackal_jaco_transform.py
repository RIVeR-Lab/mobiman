#!/usr/bin/python3
# imports
import rospy
import tf
from tf2_msgs.msg import TFMessage


if __name__ == '__main__':
    rospy.init_node("jackal_jaco_transform_publisher")
    # rate set to 300 for avoiding conflicts
    rate = rospy.Rate(100)
    trans = None
    rot = None
    # transform listener and broadcaster
    transform = tf.TransformListener()
    known_transform = tf.broadcaster.TransformBroadcaster()
    while not rospy.is_shutdown():
        # Try publishing transform
        try:
            transform.waitForTransform('/world_1', '/base_link_1', rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = transform.lookupTransform('/world_1', '/base_link_1', rospy.Time(0))
            trans[2] = 0
            known_transform.sendTransform(translation=trans, rotation=rot, time=rospy.Time.now(), child='/base_link', parent='/world')
        # If transform not found publish last known transform
        except Exception as e:
            if trans != None and rot != None:
                known_transform.sendTransform(translation=trans, rotation=rot, time=rospy.Time.now(), child='/base_link', parent='/world')
            print(e, "Still, published though!")
        # For debugging if needed
        else:
            pass
        rate.sleep()