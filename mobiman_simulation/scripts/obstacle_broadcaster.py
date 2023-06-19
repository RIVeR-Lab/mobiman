#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
import math

if __name__ == '__main__':
    rospy.init_node('obstacle_broadcaster')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer) # to listen to tag1 transformations

    broadcaster = tf2_ros.TransformBroadcaster()            # To broadcast the the obstacle transformations from the tag1 transformations
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = "World"
    t.child_frame_id = "Obstacle1"

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans_tag1_camera = tfBuffer.lookup_transform('World', 'tag_1', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        rospy.loginfo(trans_tag1_camera)
        # t.header.stamp = trans_tag1_camera.header.stamp
        # t.transform.translation.x = trans_tag1_camera.transform.translation.x
        # t.transform.translation.y = trans_tag1_camera.transform.translation.y
        # t.transform.translation.z = trans_tag1_camera.transform.translation.z
        # t.transform.rotation.x = trans_tag1_camera.transform.rotation.x
        # t.transform.rotation.y = trans_tag1_camera.transform.rotation.y
        # t.transform.rotation.z = trans_tag1_camera.transform.rotation.z
        # t.transform.rotation.w = trans_tag1_camera.transform.rotation.w
        # rospy.loginfo(t)
        broadcaster.sendTransform(t)
        
        rate.sleep()