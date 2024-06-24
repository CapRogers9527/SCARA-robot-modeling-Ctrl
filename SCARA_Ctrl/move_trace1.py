import rospy
import subprocess
import numpy as np
from math import cos, sin, pi, sqrt, atan2, acos
from sensor_msgs.msg import JointState
from time import sleep

# Initialize ROS node
rospy.init_node('scara_trajectory_control', anonymous=True)
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rate = rospy.Rate(25)  # 25 Hz

# SCARA Robot Parameters
l1 = 0.19874
l2 = 0.13299
lengri = 0.05
lentri = np.sqrt(3)*lengri/2
trate = 1 # must be integer


def kill_rosnode(node_name):
    try:
        subprocess.run(['rosnode', 'kill', node_name], check=True)
        print(f"Successfully killed ROS node: {node_name}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to kill ROS node: {node_name}")
        print(f"Error: {str(e)}")

def inverse_kinematics(x, y, z):
    # Compute theta1 and theta2 using provided forward kinematics for a SCARA robot
    
    r = np.sqrt(x**2 + y**2)
    theta_2 = np.arccos((r**2 - l1**2 - l2**2) / (2 * l1 * l2))
    theta_1 = np.arctan2(y, x) - np.arctan2(l2*np.sin(theta_2), l1 + l2*np.cos(theta_2))

    # Adjust theta_2 for the mechanism specifics, if necessary
    theta_3 = z - 0.08321  # Dummy calculation for z
    return theta_1, theta_2, theta_3


def publish_joint_states(theta1, theta2, theta3, theta4=0):
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['j1', 'j2', 'j3', 'j4']
    joint_state.position = [theta1, theta2, theta3, theta4]
    pub.publish(joint_state)
    rate.sleep()

def generate_line_points(start, end, steps):
    return [np.linspace(start[i], end[i], steps) for i in range(len(start))]

def main_trajectory():
    while not rospy.is_shutdown():
        # Define the trajectory points and movement characteristics
        points = [
            (0.331, 0, 0.08321), (0.33, 0, 0),  # Segment 1 & 2: z movement at x=0.33173, y=0
            (0.331, 0, 0.08321), (0.1, 0.2, 0.08321),  # Segment 3: move to x=0.1, y=0.2
            (0.1, 0.2, 0), (0.1, 0.2, 0.08321),  # Segment 4 & 5: z movement at x=0.1, y=0.2
            (0.1, -0.2, 0.08321),  # Segment 6: move to x=0.1, y=-0.2
            (0.1, -0.2, 0), (0.15, -0.2, 0), # Segment 7: z movement at x=0.1, y=-0.2
            # Segment 8: Circle at z=0, center (0.1, -0.2), radius 0.05
            (0.15-lengri/2,-0.2-lentri,0), (0.15-3*lengri/2,-0.2-lentri,0), (0.15-2*lengri,-0.2,0), (0.15-3*lengri/2,-0.2+lentri,0), (0.15-lengri/2,-0.2+lentri,0), (0.15, -0.2, 0), (0.1, -0.2, 0),
            (0.1, -0.2, 0.08321),  # Segment 9: return to z=0.08321
            (0.33173, 0, 0.08321)  # Segment 10: return to start
        ]

        # Loop through each segment
        for i in range(len(points) - 1):
            if isinstance(points[i], tuple):  # Simple line segment
                steps = 20*trate if i in [0, 1, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15] else 100*trate  # Faster segments
                xs, ys, zs = generate_line_points(points[i], points[i+1], steps)
                for x, y, z in zip(xs, ys, zs):
                    theta1, theta2, theta3 = inverse_kinematics(x, y, z)
                    publish_joint_states(theta1, theta2, theta3)
                    sleep(0.01)  # Delay for visual effect
            else:  # Circle segment
                for (x, y, z) in points[i]:
                    theta1, theta2, theta3 = inverse_kinematics(x, y, z)
                    publish_joint_states(theta1, theta2, theta3)
                    sleep(0.01)  # Delay for visual effect

        sleep(1)  # Wait for a second before repeating the trajectory

if __name__ == '__main__':
    try:
        kill_rosnode('/joint_state_publisher_gui')
        main_trajectory()
    except rospy.ROSInterruptException:
        pass
