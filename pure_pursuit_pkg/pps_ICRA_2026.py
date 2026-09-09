#!/usr/bin/env python3

import rclpy 
import numpy as np 
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String , Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point	
import math

"""
===============================================================================
Module: pps_ICRA_2026.py
Description:
    Production Pure Pursuit Controller with Fixed Lookahead for Competition Racetracks.
    
    This node implements search-window lookahead point selection along a 
    raceline CSV, Odometry-based state estimation, non-blocking Matplotlib 
    visualizations (comparison between Filtered Odom and Ground Truth IPS), 
    and path trajectory export upon node shutdown.

Inputs:
    - /autodrive/roboracer_1/ips (geometry_msgs/Point): Ground-truth IPS position.
    - /autodrive/roboracer_1/imu (sensor_msgs/Imu): Vehicle orientation (Yaw).
Outputs:
    - /autodrive/roboracer_1/steering_command (std_msgs/Float32): Steering command [-1.0, 1.0].
    - /autodrive/roboracer_1/throttle_command (std_msgs/Float32): Fixed throttle command.
===============================================================================
"""
#-----------------------------------------------------------------------
#-----------------------Global variables-------------------------------- 
#-----------------------------------------------------------------------

## car current position & orientation 
x_postition =  0.8
y_postition =  3.16
postition = np.array([x_postition , y_postition ])

# IPS-based position (used ONLY for comparison / plotting, not for control)
ips_postition = np.array([x_postition , y_postition ])
 
car_yaw = 0.0

# Centerline Path of ICRA 2026 Competition 
path_data = pd.read_csv('/home/autodrive_devkit/src/control/control/raceline_approved.csv')

#-----------------------------------------------------------------------------------
    # ( path_data['x'] + x_postition ) , 
    #  ( -(path_data['y'] + y_postition - 0.74 )) 
goal_list = list(zip(
    path_data['positions_X']  , 
    path_data['positions_y'] ))

goal = np.array(goal_list)
path_len = len(goal)

# pure pursuit parameter 
velocity = 0.13  
look_ahead = 1.5 
wheelbase = 0.3240 
# goal is Nx2 -> [:,0] = x , [:,1] = y

distances = np.sqrt((goal[:,0] - x_postition)**2 + (goal[:,1] - y_postition)**2)
index = np.argmin(distances)  # index of closest point


# count = index + 30 # start index  
count = index   # start index  
search_len = path_len / 5
search_end = min(count + int(search_len), path_len) # to avoid being out of range 


#------------------------------------------------------------- 
#---------------------- PLOTTING SETUP -----------------------
#------------------------------------------------------------- 

# Plotting state counter
plot_counter = 0 

car_trail_x = []
car_trail_y = []


# IPS trail (ground-truth / comparison path)
ips_trail_x = []
ips_trail_y = []

plt.ion() # Enable interactive mode
fig, ax = plt.subplots(figsize=(8, 8))

# Static plot elements
ax.plot(goal[:, 0], goal[:, 1], 'k--', label='CSV Path') 
car_plot, = ax.plot([], [], 'ro', markersize=8, label='Current Pose') 
target_plot, = ax.plot([], [], 'go', markersize=8, label='Lookahead Point') 
trail_plot,  = ax.plot([], [], 'b-', linewidth=1.5, label='Actual Path')   # <-- add this


# --- NEW: IPS plot elements ---
ips_plot, = ax.plot([], [], 'm^', markersize=8, label='IPS Pose')
ips_trail_plot, = ax.plot([], [], 'm-', linewidth=1.2, alpha=0.7, label='IPS Path')

ax.set_title("Pure Pursuit Tracking")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.legend(loc='upper right')
ax.grid(True)
ax.axis('equal') 

# Draw initial empty figure without taking window focus
fig.canvas.draw()
fig.canvas.flush_events()
#-------------------------------------------------------------
#-----------------------call back functions-------------------
#------------------------------------------------------------- 

def pose_callback (ips_msg):
    global postition

    ips_postition[0] = ips_msg.pose.pose.position.y   + x_postition         
    ips_postition[1] = -1*(ips_msg.pose.pose.position.x)+ y_postition 

    # postition[0] = ips_msg.x 
    # postition[1] =  ips_msg.y 

def ips_callback (point_msg):
    """IPS callback - used ONLY for plotting/comparison against odom."""
    global ips_postition
 
    postition[0] = point_msg.x 
    postition[1] = point_msg.y 
 

def yaw_callback (imu_msg) : 

    global car_yaw 

    ## Quaternion from imu data 
    qx = imu_msg.orientation.x
    qy = imu_msg.orientation.y
    qz = imu_msg.orientation.z
    qw = imu_msg.orientation.w
    ## convert to euler to get yaw 
    r = R.from_quat ([qx,qy,qz,qw])
    roll , pitch , car_yaw = r.as_euler('xyz')
    #print(f"Euler angles: roll={roll}, pitch={pitch}, yaw={car_yaw}")

#---------------------------------------------------------------------------
#-------------------------Pure pursuit functions----------------------------
#---------------------------------------------------------------------------

def transformation ( xy_world_arr , point_world_arr , yaw  ) : 

    R_T = np.array([ [np.cos(yaw) , np.sin(yaw)],
                     [-np.sin(yaw), np.cos(yaw)]   ])
    
    point_car_frame = R_T @ (point_world_arr-xy_world_arr)
    return  point_car_frame

def curvature_calc (xy_car_frame) :
    x = xy_car_frame[0]
    y = xy_car_frame[1]
    curvature = (2* y) / (look_ahead*look_ahead)  ## without abs()
    return curvature

def steering_func (wh_base , gamma) :

    steering_angle = np.arctan(wh_base *gamma) 
    
    """Avoid steering more than maximum"""
    # if (steering_angle >=  0.5236):
    #    steering_angle = 1
    # elif (steering_angle <= -0.5236):
    #    steering_angle = -1

    return steering_angle

#---------------------------------------------------------------------------
#-------------------- ROS 2 Timer_function ---------------------------------
#---------------------------------------------------------------------------

def timer_func( node, st_pub , thr_pub ) : 
    global  postition , car_yaw  , count , plot_counter , car_trail_x , car_trail_y
    st = Float32()
    thr = Float32() 
    start = count   
    search_end = min(count + int(search_len), path_len) # to avoid being out of range 

    car_trail_x.append(postition[0])  
    car_trail_y.append(postition[1])  


    ips_trail_x.append(ips_postition[0])
    ips_trail_y.append(ips_postition[1])
 

    node.get_logger().info("Publishing : >_<" )
 

    check_distance = np.sqrt((goal[count:search_end ,0] - postition[0])**2 + (goal[count:search_end ,1] - postition[1])**2)
    nearest_idx = np.where(check_distance >= (look_ahead))[0]   # index of next goal point

    if len(nearest_idx) >  0 :  
        count = start + nearest_idx[0]
    else :
        count += 1  

    if ( count >= path_len  ):  ## start point faraway = no oscullation
        count = 10

    xy_cf = transformation( postition , goal[count] ,car_yaw )
    curve = curvature_calc ( xy_cf )
    steer = steering_func( wheelbase, curve ) / 0.5236
    st.data =  float(steer)

    thr_msg = velocity
    thr.data = thr_msg
    st_pub.publish(st) 
    thr_pub.publish(thr) 
    
    # --- NON-BLOCKING PLOTTING UPDATE (Every 10 cycles = 10 Hz) ---
    plot_counter += 1
    if plot_counter % 10 == 0:
        car_plot.set_data([postition[0]], [postition[1]])
        target_plot.set_data([goal[count, 0]], [goal[count, 1]])
        trail_plot.set_data(car_trail_x, car_trail_y)   # <-- add this

        # --- NEW: update IPS pose + trail ---
        ips_plot.set_data([ips_postition[0]], [ips_postition[1]])
        ips_trail_plot.set_data(ips_trail_x, ips_trail_y)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()


    node.get_logger().info(f" yaw angle  : {round(car_yaw,3)} " )
    node.get_logger().info(f" steeing command value : {round(steer,3)} >_<" )
    node.get_logger().info(f" throttle command value : {thr_msg} >_<" )
    node.get_logger().info(f" Lookahead  : {look_ahead} >_<" )
    node.get_logger().info(f" index   : {count} >_<" )

#------------------------- Main ------------------------
def main (args=None):    
    

    #   Node creation

    rclpy.init(args=args)    
    my_node = rclpy.create_node('control_node')

    #  subscribers

    car_pose = my_node.create_subscription(Odometry , '/odometry/filtered' , pose_callback , 10)
    ips_msg = Odometry()        
    car_pose = my_node.create_subscription(Point , '/autodrive/roboracer_1/ips', ips_callback , 10)
    ips_point_msg = Point() 

    imu_sub = my_node.create_subscription(Imu , '/autodrive/roboracer_1/imu' , yaw_callback , 10)
    imu_msg = Imu()
    steer_pub = my_node.create_publisher( Float32 , "/autodrive/roboracer_1/steering_command" , 10 )  
    st_msg = Float32()
    throttle_pub = my_node.create_publisher( Float32 , "/autodrive/roboracer_1/throttle_command" , 10 )
    th_msg = Float32()


    timer = my_node.create_timer ( 0.01 , lambda:timer_func(my_node,steer_pub,throttle_pub) )   #  100 hz  
    rclpy.spin(my_node)

# --- CLOSE PLOTS ON SHUTDOWN  ---
    plt.close('all')
    np.savetxt('/home/autodrive_devkit/actual_path.csv',
            np.column_stack((car_trail_x, car_trail_y)),
            delimiter=',', header='x,y', comments='')
    
    my_node.destroy_timer(timer)
    my_node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__" :    # the entry of the code     


    main()                  