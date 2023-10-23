#python

yaw_setpoint = 0

def sysCall_init():
    # do some initialization here
    pass

def sysCall_actuation():
    # put your actuation code here
    pass

def sysCall_sensing():
    global yaw_setpoint
    # put your sensing code here
    yaw_setpoint = sim.getFloatSignal("yaw_setpoint")
    print(yaw_setpoint)
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details
