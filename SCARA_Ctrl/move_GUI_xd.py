import tkinter as tk
from tkinter import messagebox
import numpy as np
import rospy
import subprocess
from sensor_msgs.msg import JointState

# Initialization
rospy.init_node('scara_gui_control', anonymous=True)
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rate = rospy.Rate(25)


def kill_rosnode(node_name):
    try:
        subprocess.run(['rosnode', 'kill', node_name], check=True)
        print(f"Successfully killed ROS node: {node_name}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to kill ROS node: {node_name}")
        print(f"Error: {str(e)}")

def calculate_joint_angles(x, y, z, alpha):
    # Compute theta1 and theta2 using provided forward kinematics for a SCARA robot
    
    r = np.sqrt(x**2 + y**2)
    l1 = 198.74/1000  # length of the first arm segment
    l2 = 132.99/1000  # length of the second arm segment
    theta_2 = np.arccos((r**2 - l1**2 - l2**2) / (2 * l1 * l2))
    theta_1 = np.arctan2(y, x) - np.arctan2(l2*np.sin(theta_2), l1 + l2*np.cos(theta_2))

    # Adjust theta_2 for the mechanism specifics, if necessary
    theta_3 = z - 0.08321  # Dummy calculation for z
    theta_4 = alpha - (theta_1 + theta_2)  # Based on your formula
    return theta_1, theta_2, theta_3, theta_4

def calculate_and_publish():
    try:
        x = float(entry_x.get())
        y = float(entry_y.get())
        z = float(entry_z.get())
        alpha = float(entry_alpha.get())
        x = x/1000
        y = y/1000
        z = z/1000

        if (x**2 + y**2 > (0.19874 + 0.13299)**2) or (z > 0.177) or (z < -0.14):
            messagebox.showerror("Error", "The specified position is out of reach. Please enter a valid position.")
            return  # 终止函数运行
        theta_1, theta_2, theta_3, theta_4 = calculate_joint_angles(x, y, z, alpha)
        
        # Simulate the robot's motion by publishing interpolated joint states
        steps = 200
        current_angles = np.zeros(4)  # Assume start at all zeros
        target_angles = np.array([theta_1, theta_2, theta_3, theta_4])
        interpolated_angles = np.linspace(current_angles, target_angles, num=steps)

        for i in range(steps):
            joint_state = JointState()
            joint_state.name = ['j1', 'j2', 'j3', 'j4']
            joint_state.position = interpolated_angles[i]
            joint_state.header.stamp = rospy.Time.now()
            pub.publish(joint_state)
            rate.sleep()
        
        messagebox.showinfo("Success", "Trajectory completed successfully!")
    except Exception as e:
        messagebox.showerror("Error", str(e))

root = tk.Tk()
root.title("SCARA Robot Controller")
kill_rosnode('/joint_state_publisher_gui')

# Setup GUI entries
labels = ["X", "Y", "Z", "Alpha (overall rotation)"]
entries = []
for idx, label in enumerate(labels):
    tk.Label(root, text=label).grid(row=idx, column=0)
    entry = tk.Entry(root)
    entry.grid(row=idx, column=1)
    entries.append(entry)

entry_x, entry_y, entry_z, entry_alpha = entries  # Unpack for easier access

button = tk.Button(root, text="Calculate and Move", command=calculate_and_publish)
button.grid(row=5, column=0, columnspan=2)

root.mainloop()
