# Once the CoppeliaSim ChildScripts are working, copy the child script for "Pendulum B" and paste it here.
#python

###### GLOBAL VARIABLES HERE ######
base = None
motor = None
arm = None
pendulum = None
U = None

k = [-9.7502  , -10.0000 ,  -37.7885,  -144.9205] # 17 seconds
#k = [-3.6402, -3.1623, -12.3813, -60.1181] # 15 seconds

theta = alpha = 0
x2_prev = x4_prev = 0

# You can add variables here 
# as required by your implementation.
###################################

def sysCall_init():
    global arm, motor, pendulum, U, k, theta, alpha, pivot_b
    # do some initialization here
    # This function will be executed once when the simulation starts
    
    ####### ADD YOUR CODE HERE ######
    # Hint: Initialize the scene objects which you will require 
    #       Initialize algorithm related variables here
    
    #################################
    motor = sim.getObject("/Elbow_motor_B")
    pendulum = sim.getObject("/Elbow_free_B")
    pivot_b = sim.getObject("/Pivot_B")
    pass

def sysCall_actuation():
    global arm, motor, pendulum, U, k, theta, alpha, x2_prev, x4_prev, pivot_b
    # put your actuation code here
    # This function will be executed at each simulation time step

    ####### ADD YOUR CODE HERE ######
    # Hint: Use the error feedback and apply control algorithm here
    #       Provide the resulting actuation as input to the actuator joint
    
    # Example psuedo code:
    #   x1 = error_state_1; # Error in states w.r.t desired setpoint
    #   x2 = error_state_2;
    #   x3 = error_state_3;
    #   x4 = error_state_4;
    #   k = [gain_1 , gain_1, gain_3, gain_4];      # These gains will be generated by control algorithm. For ex: LQR, PID, etc.
    #   U = -k[1]*x1 +k[2]*x2 -k[3]*x3 +k[4]*x4;    # +/- Sign convention may differ according to implementation
    #   Set_joint_actuation(U);                     # Provide this calculated input to system's actuator

    #################################
    x4 = 0.0 - theta
    x2 = 0.0 - alpha
    
    # Error Rate calculations for x1, x3 (alpha_dot and theta_dot)
    x1 = (x2 - x2_prev)/0.010
    x3 = (x4 - x4_prev)/0.010
    
    U = +k[0]*x1 + k[1]*x2 + k[2]*x3 + k[3]*x4
    
    sim.setJointTargetVelocity(motor, U)
    x2_prev = x2
    x4_prev = x4
    pass

def sysCall_sensing():
    global arm, motor, pendulum, U, k, theta, alpha, pivot_b
    # put your sensing code here
    # This function will be executed at each simulation time step
    
    ####### ADD YOUR CODE HERE ######
    # Hint: Take feedback here & do the error calculation
    
    #################################
    theta = sim.getJointPosition(pendulum)
    alpha = sim.getJointPosition(pivot_b)
    
    pass

def sysCall_cleanup():
    global arm, motor, pendulum, U, pivot_b
    # do some clean-up here
    # This function will be executed when the simulation ends
    
    ####### ADD YOUR CODE HERE ######
    # Any cleanup (if required) to take the scene back to it's original state after simulation
    # It helps in case simulation fails in an unwanted state.
    #################################
    sim.setJointPosition(pendulum, 0)
    sim.setJointPosition(pivot_b, 0)
    pass

# See the user manual or the available code snippets for additional callback functions and details