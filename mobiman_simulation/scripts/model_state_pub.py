import rospy
from gazebo_msgs.msg import ModelStates
import rospkg
import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header, UInt8
from geometry_msgs.msg import PoseStamped, Twist, Pose, Point


transform = None
model_state_pub = None

def timer_transform(timer):
        #print("[" + self.ns + "][igibson_env_jackalJaco::iGibsonEnv::timer_transform] START")
        try:
            model_state_msg = ModelStates()
            for dict in ['conveyor_belt', 'red_cube']:
                # self.br.sendTransform(obj.get_position(), obj.get_orientation(), rospy.Time.now(), f'{self.ns}{dict[0]}', 'world')
                model_state_msg.name.append(dict)
                transform.waitForTransform('/world', dict, rospy.Time(0), rospy.Duration(0.01))
                POSE, ORIENTATION = transform.lookupTransform('/world', dict, rospy.Time(0))
                pose = Pose()
                x,y,z = POSE
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                x,y,z,w = ORIENTATION
                pose.orientation.x = x
                pose.orientation.y = y
                pose.orientation.z = z
                pose.orientation.w = w
                model_state_msg.pose.append(pose)
            model_state_pub.publish(model_state_msg)
        except Exception as e:
            pass

if __name__ == '__main__':
    rospy.init_node("model_state_publisher_node")
    print("[+] MODEL STATE PUBLISHER STARTED")
    transform = tf.TransformListener()
    model_state_pub = rospy.Publisher("model_states", ModelStates, queue_size=10)
    rospy.Timer(rospy.Duration(0.01), timer_transform)
    rate = rospy.Rate(100)
    while True:
        rate.sleep()