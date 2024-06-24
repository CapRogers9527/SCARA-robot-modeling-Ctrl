import rospy
import subprocess
import numpy as np
from math import sqrt, atan2, acos, sin, cos
from sensor_msgs.msg import JointState
from threading import Thread
import sys
import select
import termios
import tty

# Initialize ROS node
rospy.init_node('scara_continuous_trajectory', anonymous=True)
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rate = rospy.Rate(25)  # 25 Hz

# SCARA Robot Parameters
l1 = 0.19874
l2 = 0.13299

def kill_rosnode(node_name):
    try:
        subprocess.run(['rosnode', 'kill', node_name], check=True)
        print(f"Successfully killed ROS node: {node_name}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to kill ROS node: {node_name}")
        print(f"Error: {str(e)}")

def inverse_kinematics(x, y, z):
    r = sqrt(x**2 + y**2)
    theta_2 = acos((r**2 - l1**2 - l2**2) / (2 * l1 * l2))
    theta_1 = atan2(y, x) - atan2(l2*sin(theta_2), l1 + l2*cos(theta_2))
    theta_3 = z - 0.08321
    return theta_1, theta_2, theta_3

def publish_joint_states(theta1, theta2, theta3, theta4=0):
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['j1', 'j2', 'j3', 'j4']
    joint_state.position = [theta1, theta2, theta3, theta4]
    pub.publish(joint_state)
    rate.sleep()

def five_polynomial(start, end, t, T):
    a0 = start
    a1 = 0
    a2 = 0
    a3 = 10 * (end - start) / T**3
    a4 = -15 * (end - start) / T**4
    a5 = 6 * (end - start) / T**5
    return a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5

def trajectory_motion():
    kill_rosnode('/joint_state_publisher_gui')
    T = 5  # total time of the motion
    start_pos = np.array([0.1, 0.1, 0])
    end_pos = np.array([0.3, 0.1, 0])
    steps = 100
    dt = T / steps

    while not rospy.is_shutdown() and not exit_program:
        for direction in [1, -1]:  # 1 for forward, -1 for reverse
            for i in range(steps + 1):
                t = i * dt
                if direction == 1:
                    x = five_polynomial(start_pos[0], end_pos[0], t, T)
                    y = five_polynomial(start_pos[1], end_pos[1], t, T)
                    z = five_polynomial(start_pos[2], end_pos[2], t, T)
                else:
                    x = five_polynomial(end_pos[0], start_pos[0], t, T)
                    y = five_polynomial(end_pos[1], start_pos[1], t, T)
                    z = five_polynomial(end_pos[2], start_pos[2], t, T)
                theta1, theta2, theta3 = inverse_kinematics(x, y, z)
                publish_joint_states(theta1, theta2, theta3, 0)
                if exit_program:
                    break
            if exit_program:
                break

exit_program = False

def check_for_exit():
    global exit_program
    while True:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = input()
            if line == 'q':
                exit_program = True
                break

if __name__ == '__main__':
    try:
        exit_thread = Thread(target=check_for_exit)
        exit_thread.start()
        trajectory_motion()
    except rospy.ROSInterruptException:
        pass
    finally:
        exit_program = True
        exit_thread.join()
