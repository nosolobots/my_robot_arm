import roslib
import rospy
from sensor_msgs.msg import JointState

if __name__ == '__main__':
    rospy.init_node('myrobot_joint_state_publisher')
    r = rospy.Rate(1)

    theta = 0.0

    joint_states_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    msg = JointState()
    
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'base_link'
        msg.name = ['base_link__link_01', \
                    'link_01__link_02', \
                    'link_02__link_03', \
                    'link_03__link_04', \
                    'link_04__link_05', \
                    'link_05__gripper']
        msg.position = [theta,.0,.0,.0,.0,.0]
        msg.velocity = []
        msg.effort = []

        joint_states_pub.publish(msg)
        print(msg)

        theta += .1
        r.sleep()
