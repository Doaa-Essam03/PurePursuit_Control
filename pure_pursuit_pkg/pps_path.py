#!/usr/bin/env python3

import rclpy 
import numpy as np 
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String , Float32
from geometry_msgs.msg import Point 
from sensor_msgs.msg import Imu

"""
===============================================================================
Module: pps_path.py
Description:
    ROS 2 Pure Pursuit Controller for generated mathematical paths 
    (e.g., Zig-Zag, S-Curve).testing in explore map 
    
    This node evaluates trajectory tracking across array-based waypoints generated 
    programmatically. It includes sequential waypoint switching based on 
    the lookahead radius.

    - Increments target index when distance to current goal falls within threshold.

Inputs:
    - /autodrive/f1tenth_1/ips (geometry_msgs/Point)
    - /autodrive/f1tenth_1/imu (sensor_msgs/Imu)
Outputs:
    - /autodrive/f1tenth_1/steering_command (std_msgs/Float32)
    - /autodrive/f1tenth_1/throttle_command (std_msgs/Float32)
===============================================================================
"""

#-----------------------Global variables-------------------------------- 
# pure pursuit parameter 
look_ahead = 1.5    # 3 > 2.5 good but cutting some edges ##  1.5 > 1 good # .3 > 1 not good  
wheelbase = 0.3240 


## car current position & orientation 
x_postition =  0.748
y_postition =  3.16
postition = np.array([x_postition , y_postition ])

car_yaw = 0.0

# ----------------------- Generated Paths Selection --------------------------------
## Goal path points created relative to world frame 
# Choose path mode: 'straight', 's_curve', 'circle', 'sharp_turn', 'zig_zag'
PATH_TYPE = 'zig_zag'  

def generate_path(path_type, x_init, y_init):
    """Switch-case generator for trajectory waypoints."""
    
    if path_type == 'straight':
        x = np.linspace(0, 4, 100) + x_init
        y = np.linspace(0, 7, 100) + y_init
        
    elif path_type == 's_curve':
        x = np.linspace(0, 30, 40) + x_init
        y = -5 * np.sin(0.5 * x) + y_init
        
    elif path_type == 'circle':
        theta = np.linspace(0, 2 * np.pi, 100)
        radius = 2
        x = x_init + radius * np.cos(theta)
        y = y_init + radius * np.sin(theta)
        
    elif path_type == 'sharp_turn':
        x = np.concatenate([np.linspace(0, 1.75, 50), np.full(50, 1.75)]) + x_init
        y = np.concatenate([np.full(50, 0), np.linspace(0, 7, 50)]) + y_init
        
    elif path_type == 'zig_zag':
        x = np.linspace(0, 15, 20) + x_init
        y = y_init - 10 * np.sign(np.sin(2 * np.pi * x / 10))
        
    else:
        raise ValueError(f"Unknown path_type: {path_type}")
        
    return x, y

# Generate unified x and y
x, y = generate_path(PATH_TYPE, x_postition, y_postition)
goal = np.column_stack((x, y))

count = 0


# ---------- Live Plotting Setup -------------

car_path_x = []
car_path_y = []

plt.ion()
fig, ax = plt.subplots()

# Red dot for current position
sc, = ax.plot([], [], 'ro')

# Green X for goal point
# Plot all waypoints as green Xs
goal_markers = ax.plot(goal[:, 0], goal[:, 1], 'gx')[0]

# Line to draw full path
path_line, = ax.plot([], [], 'b--', linewidth=1.5)

ax.set_title("Live Car Trajectory")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.grid(True)



#-----------------------call back functions-------------------------------- 
def pose_callback (ips_msg):
    global postition

    postition[0] =  ips_msg.x  
    postition[1] =  ips_msg.y 
    
    
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


#-------------------------Pure pursuit functions--------------------------------

def transformation ( xy_world_arr , point_world_arr , yaw  ) : 

    R_T = np.array([ [np.cos(yaw) , np.sin(yaw)],
                     [-np.sin(yaw), np.cos(yaw)]   ])
    
    point_car_frame = R_T @ (point_world_arr-xy_world_arr)
    return  point_car_frame

def curvature_calc (xy_car_frame) :
    x = xy_car_frame[0]
    y = xy_car_frame[1]
    curvature = (2* y) / (look_ahead)   ## without abs()
    return curvature

def steering_func (wh_base , gamma) :

    steering_angle = np.arctan(wh_base *gamma) 
    
    """Avoid steering more than maximum"""
    if (steering_angle >=  0.5236):
        steering_angle = 0.5236 

    elif (steering_angle <= -0.5236):
        steering_angle = -0.5236

    return steering_angle




#-------------------- ROS 2 Timer_function ------------------------
def timer_func( node, st_pub , thr_pub ) : 
    global  postition , car_yaw  , count
    st = Float32()
    thr = Float32()

    node.get_logger().info("Publishing : >_<" )
    if np.isclose(10.748, postition[0], atol=0.1) and np.isclose(1.580, postition[1], atol=0.1):
        print(f"Goal Reached at x={postition[0]}, y={postition[1]}")

        # Stop motion
        st.data = 0.0
        thr.data = 0.0
        st_pub.publish(st)
        thr_pub.publish(thr)
        return  # stop this timer iteration
        
        

    dist = np.linalg.norm( goal[count]-postition ) 
    if dist <= 1.0 :
        count += 1 


    xy_cf = transformation( postition , goal[count] ,car_yaw )
    curve = curvature_calc ( xy_cf )
    steer = steering_func( wheelbase, curve )
    st.data =  float(steer)

    node.get_logger().info(f" yaw angle  : {round(car_yaw,3)} " )
    node.get_logger().info(f" steeing command value : {round(steer,3)} >_<" )

    thr.data = 0.02
    st_pub.publish(st) 
    thr_pub.publish(thr) 

        
        # --- Live plot update ---
    car_path_x.append(postition[0])
    car_path_y.append(postition[1])

    sc.set_data(postition[0], postition[1])  # red dot
    path_line.set_data(car_path_x, car_path_y)  # trajectory line

    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()


#------------------------- Main ------------------------
def main (args=None):    
    

    #   Node creation

    rclpy.init(args=args)    
    my_node = rclpy.create_node('control_node')

    #  subscribers

    car_pose = my_node.create_subscription(Point , '/autodrive/f1tenth_1/ips' , pose_callback , 10)
    ips_msg = Point()    
    imu_sub = my_node.create_subscription(Imu , '/autodrive/f1tenth_1/imu' , yaw_callback , 10)
    imu_msg = Imu()
    steer_pub = my_node.create_publisher( Float32 , "/autodrive/f1tenth_1/steering_command" , 10 )  
    st_msg = Float32()
    throttle_pub = my_node.create_publisher( Float32 , "/autodrive/f1tenth_1/throttle_command" , 10 )
    th_msg = Float32()


    timer = my_node.create_timer ( 0.1 , lambda:timer_func(my_node,steer_pub,throttle_pub) )   #  10 hz  
    rclpy.spin(my_node)
    my_node.destroy_timer(timer)
    my_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__" :    # the entry of the code     


    main()                  
