#!/usr/bin/python3
# imports
import rospy
import tf
from tf2_msgs.msg import TFMessage


if __name__ == '__main__':
    rospy.init_node("jackal_jaco_transform_publisher")
    # rate set to 300 for avoiding conflicts
    # rate = rospy.Rate(10000)
    trans = None
    rot = None
    # transform listener and broadcaster
    transform = tf.TransformListener()
    known_transform = tf.broadcaster.TransformBroadcaster()
    print("[+] Node Started")
    while not rospy.is_shutdown():
        # Try publishing transform
        try:
            transform.waitForTransform('/world', '/jackalJaco_0/odom', rospy.Time(0), rospy.Duration(0.01))
            (trans, rot) = transform.lookupTransform('/world', '/jackalJaco_0/odom', rospy.Time(0))
            trans[2] = 0.184/2.9
            known_transform.sendTransform(translation=trans, rotation=(0,0,0,1), time=rospy.Time.now(), child='/odom', parent='/world')
            known_transform.sendTransform(translation=(0,0,0), rotation=(0,0,0,1), time=rospy.Time.now(), child='/base_link', parent='/odom')
        # If transform not found publish last known transform
        except Exception as e:
            if trans != None and rot != None:
                # known_transform.sendTransform(translation=trans, rotation=rot, time=rospy.Time.now(), child='/base_link', parent='/world')
                known_transform.sendTransform(translation=trans, rotation=(0,0,0,1), time=rospy.Time.now(), child='/odom', parent='/world')
                known_transform.sendTransform(translation=(0,0,0), rotation=(0,0,0,1), time=rospy.Time.now(), child='/base_link', parent='/odom')
            print(e, "Still, published though!")
        # For debugging if needed
        else:
            pass
        # rate.sleep()