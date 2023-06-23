#!/usr/bin/env python  
import rospy
import tf2_ros
import geometry_msgs.msg
import math
import numpy as np

################### TUNABLE PARAMETERS IN THIS CODE #############################

# Angular distance of 5 degrees and 0.05 m seems to work. 
n = 10                        # angular distance from previous transform in degrees
threshold_angular_distance = n*math.pi/180           
threshold_euclidean_distance = 0.1              
averaging_window = 5                              # Number of raw poses to consider in the averaging window  

# Later on replace tag_8 with world frame (tag 0) and object1 with whatever frame we want to localize. 
#################################################################################

# A FUNCTION TO COMPUTE QUATERNION ANGULAR DISTANCE.    

## Source for this expression : https://fgiesen.wordpress.com/2013/01/07/small-note-on-quaternion-distance-metrics/
def quaternion_angular_distance(q1, q2):                          
    dot_product = np.dot(q1, q2)
    angle = 2 * np.arccos(np.abs(dot_product))
    return angle 

if __name__ == '__main__':
    rospy.init_node('obstacle_broadcaster')
    rospy.loginfo("Broadcasting")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer) # to listen to tag1 transformations

    broadcaster = tf2_ros.TransformBroadcaster()            # To broadcast the the obstacle transformations from the tag1 transformations
    filtered_transform = geometry_msgs.msg.TransformStamped()
    old_trans = geometry_msgs.msg.TransformStamped()

    filtered_transform.header.frame_id ="tag_8"
    filtered_transform.child_frame_id = "object1_averaged"

    ## DISTANCES    
    distance_from_previous = 0               # Variable to store distance of raw transformation from the previously published averaged transformation. 
    angular_distance_from_previous = 0       # Variable to store angular distance of quaternions from previously published averaged transformation.

    rate = rospy.Rate(10.0)
    position_list = []      # List to store positions from the previous timestamps.
    quat_list = []          # Quaternion list to store quaternions from previous timestamps 
    
    while not rospy.is_shutdown():
        try:
            trans_tag8_object1 = tfBuffer.lookup_transform('world','object_1',rospy.Time())          # Looking up the raw transform from the tf tree. 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        x_raw = trans_tag8_object1.transform.translation.x
        y_raw = trans_tag8_object1.transform.translation.y
        z_raw = trans_tag8_object1.transform.translation.z
        xrot_raw = trans_tag8_object1.transform.rotation.x
        yrot_raw = trans_tag8_object1.transform.rotation.y
        zrot_raw = trans_tag8_object1.transform.rotation.z
        wrot_raw = trans_tag8_object1.transform.rotation.w
        q_raw = np.array([xrot_raw,yrot_raw,zrot_raw,wrot_raw])
        distance = ((x_raw-old_trans.transform.translation.x)**2 + (y_raw - old_trans.transform.translation.y)**2 + (z_raw - old_trans.transform.translation.z)**2)**0.5
        rospy.loginfo("Distance between raw readings")
        rospy.loginfo(distance)
        ## COMPUTING DISTANCE OF THE NEW TRANSFORM FROM THE PREVIOUS.

        if len(position_list)>0:        ## Try finding the distance only if the list has atleast one element. 
            q_old = np.array([averaged_rotx,averaged_roty,averaged_rotz,averaged_rotw])

            distance_from_previous = ((x_raw-averaged_x)**2 + (y_raw - averaged_y)**2 + (z_raw - averaged_z)**2)**0.5      ## Euclidean Distance computed. 
            angular_distance_from_previous = quaternion_angular_distance(q_old,q_raw)

        # rospy.loginfo(trans_tag8_object1)
        rospy.loginfo("Distance between current raw reading and previous average")
        rospy.loginfo(distance_from_previous)
        if distance_from_previous < threshold_euclidean_distance : #and angular_distance_from_previous < threshold_angular_distance:   # So a sudden jump of greater than 10mm is unacceptable and the transform will be discarded. 
        #     rospy.loginfo("Inside")
            if len(position_list)>=averaging_window:
                position_list.pop(0)
                position_list.append([x_raw,y_raw,z_raw])
                quat_list.pop(0)
                quat_list.append([trans_tag8_object1.transform.rotation.x,trans_tag8_object1.transform.rotation.y,trans_tag8_object1.transform.rotation.z,trans_tag8_object1.transform.rotation.w])

            else : 
                position_list.append([x_raw,y_raw,z_raw])
                quat_list.append([trans_tag8_object1.transform.rotation.x,trans_tag8_object1.transform.rotation.y,trans_tag8_object1.transform.rotation.z,trans_tag8_object1.transform.rotation.w])

        rospy.loginfo(len(position_list))   
        ## AVERAGING THE POSITION FROM THE LIST
        averaged_x = np.mean([position[0] for position in position_list]) 
        averaged_y = np.mean([position[1] for position in position_list]) 
        averaged_z = np.mean([position[2] for position in position_list]) 

        averaged_rotx = np.mean([quat[0] for quat in quat_list]) 
        averaged_roty = np.mean([quat[1] for quat in quat_list]) 
        averaged_rotz = np.mean([quat[2] for quat in quat_list]) 
        averaged_rotw = np.mean([quat[3] for quat in quat_list]) 


       
        filtered_transform.header.stamp = rospy.Time.now()
        filtered_transform.transform.translation.x = averaged_x
        filtered_transform.transform.translation.y = averaged_y
        filtered_transform.transform.translation.z = averaged_z
        filtered_transform.transform.rotation.x = averaged_rotx
        filtered_transform.transform.rotation.y = averaged_roty
        filtered_transform.transform.rotation.z = averaged_rotz
        filtered_transform.transform.rotation.w = averaged_rotw
       
        broadcaster.sendTransform(filtered_transform)
        old_trans = trans_tag8_object1
        rospy.loginfo("Broadcasting")
        rospy.loginfo(filtered_transform)
        rate.sleep()