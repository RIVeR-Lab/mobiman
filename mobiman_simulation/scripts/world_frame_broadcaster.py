#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.01)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "cam2_color_optical_frame"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "world"
            t.transform.translation.x = 0.6247646970731584
            t.transform.translation.y = 0.5875674991736752
            t.transform.translation.z = 2.4035590620478753

            t.transform.rotation.x = 0.9283736675128444
            t.transform.rotation.y = 0.11063793792865799
            t.transform.rotation.z = -0.24237512714706008
            t.transform.rotation.w = 0.2591059202335822

            tfm = tf2_msgs.msg.TFMessage([t])
            print("Publishing a static world transform with respect to cam2")
            rospy.loginfo(tfm)
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()