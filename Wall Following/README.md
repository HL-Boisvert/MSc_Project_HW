
# Tuning process
The PID was tuned using the following procedure, from https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
1. Setting the three gains to zero
2. Increasing kp until the response to a disturbance is a steady oscillation
3. Increasing kd until there are no oscillations left
4. Repeating steps 2 and 3 until kd doesn't stop the response from oscillating
5. Increase ki until the response reaches the setpoint within the required number of oscillations.
I ended up with a system that has only 2 oscillations and a very quick response compared to before it was tuned.

# Settings
## WallFollow.py
kp = -70  
kd = -30  
ki = -0,07  

ANGLE_RANGE = 270° (from UST-10LX Specifications)  
DESIRED_DISTANCE_RIGHT = 0,9m  
DESIRED_DISTANCE_LEFT = 1m  
VELOCITY = 5.00m/s (reduced from the max speed of 20 m/s given by the simulator for safety reasons)  
MAX_ACCEL = 12m/s^2  
MAX_DECEL = 8,26m/s^2  
CAR_LENGTH = 0,35m  

theta = 35°  
