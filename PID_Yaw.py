'''
This is to demonstrate, when the rocket is moving vertically upwards and there is some Yaw angle.
Lets assume that the rocket is supposed to go vertically up. If there is any yaw angle to the rocket,
that means that the course needs to be corrected.
'''

altitude = 5

ascentFlag = 1
descentFlag = 0
landedFlag = 0

yaw = 0 # Set point for yaw angle
kp = 0.2 # Proportional gain
ki = 0.3 # Integral gain
kd = 0.4 # Derivative gain

p = 0 # Proportional error
i = 0 # Integral error
d = 0 # Derivative error

t = 0
dt = 0.5

current_angle = 0

control_signal = 0

p_last_calc = 0
while altitude > 0:
    if altitude > 500:
        break
    p_last_calc = p
    p = yaw - current_angle # Error term
    i += p # Integral term
    d = p - p_last_calc

    if(t == 2):
        current_angle = 50


    try:
        control_signal = (kp * p) + (ki * t * i) + ((kd/t)*d)
    except:
        print("EXCEPTION")

    current_angle += control_signal

    print("Control signal : " , control_signal,"Current angle : " ,current_angle, "Setpoint : ", yaw)
    

    altitude += 25
    t += 0.5


