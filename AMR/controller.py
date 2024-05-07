import numpy as np
import matplotlib.pyplot as plt 
wind_active = False # Select whether you want to activate wind or not
group_number = 24  # Enter your group number here

error_x_list = []
error_y_list = []
error_a_list = []
dt_list = []

class Update():
    y = 0
    x = 0 
    a = 0
    prev_error_y = 0
    prev_error_x = 0
    prev_error_a = 0

# Implement a controller
def controller(state, target_pos, dt):
    # state format: [position_x (m), position_y (m), velocity_x (m/s), velocity_y (m/s), attitude(radians), angular_velocity (rad/s)]
    # target_pos format: [x (m), y (m)]
    # dt: time step
    # return: action format: (u_1, u_2)
    # u_1 and u_2 are the throttle settings of the left and right motor
    position_x, position_y, veloity_x, velocity_y, attitude, angular_velocity = state
    x, y = target_pos
    desired_attitude = 0
   
    # Gains for the Y
    #  axis 
    Kp_y = 18 
    Ki_y = 0.12 
    Kd_y = 10 
    
    #Gains for the X axis
    Kp_x = 0.295 
    Ki_x = 0.0002 
    Kd_x = 0.87

    # Gains for the Attitude(Roll)
    Kp_a = 9.5    
    Ki_a = 0.035 
    Kd_a = 6     

    # Calculating the error in both axis 
    error_x = position_x - x
    error_y = position_y - y
    error_a = np.arctan2(np.sin(attitude-desired_attitude),np.cos(attitude-desired_attitude))

    #Calculating the control inputs 
    control_input_x = (Kp_x * error_x) + (Ki_x * Update.x ) + (Kd_x * (error_x-Update.prev_error_x))/dt
    control_input_y = (Kp_y * error_y) + (Ki_y * Update.y) + (Kd_y * (error_y-Update.prev_error_y))/dt
    control_input_a = (Kp_a * error_a) + (Ki_a * Update.a) + (Kd_a * (error_a-Update.prev_error_a))/dt        

    # Updating the error values
    Update.y = error_y*dt + Update.y
    Update.prev_error_y = error_y

    Update.x = error_x*dt + Update.x
    Update.prev_error_x = error_x

    Update.a = error_a*dt + Update.a
    Update.prev_error_a = error_a

    time = np.arange(0,15,0.015)


    global error_x_list, error_a_list, error_y_list, dt_list
    error_x_list.append(error_x)
    error_y_list.append(error_y)
    error_a_list.append(error_a)
    dt_list.append(dt)

    if len(error_x_list) == len(time):
        plt.plot(time,error_x_list, label='x')
        #plt.plot(time,error_y_list, label='y')
        #plt.plot(time,error_a_list, label='a')
        plt.xlabel('Time (s)')
        plt.ylabel('Error in x (m)')
        plt.title('Error in X vs Time')
        #plt.ylabel('Error in y (m)')
        #plt.title('Error in Y vs Time')
        #plt.ylabel('Error in Attitude a (radians)')
        #plt.title('Error in Attitude vs Time')
        plt.legend()
        plt.show() 


    u_1 = control_input_y - control_input_x - control_input_a
    u_2 = control_input_y + control_input_x + control_input_a

    # Normalising the Thrusts to remain between 0 and 1 
    u_1 = max(0, min(u_1, 1))
    u_2 = max(0, min(u_2, 1))
    
    action = (u_1,u_2)
    return action
