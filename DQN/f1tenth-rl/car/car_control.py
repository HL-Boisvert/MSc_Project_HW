import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from rospy.exceptions import ROSException

from threading import Thread
import time
import argparse

try:
    from geometry_msgs.msg import PoseStamped
except ImportError:
    pass

try:
    from car.sensors import Sensors
except ImportError:
    from sensors import Sensors

PUBLISHER_WAIT = 0.005 # Dealy for publishing the Ackermann Messages

# Values for real car

MAX_SPEED_REDUCTION = 4.5
MIN_SPEED_REDUCTION = 5
STEERING_SPEED_REDUCTION = 4.5
BACKWARD_SPEED_REDUCTION = 4.5
LIGHTLY_STEERING_REDUCTION = 2.4
BACKWARD_SECONDS = 1.5

# Values for simulator

MAX_SPEED_REDUCTION_SIM = 1
STEERING_SPEED_REDUCTION_SIM = 1.4
BACKWARD_SPEED_REDUCTION_SIM = 3
LIGHTLY_STEERING_REDUCTION_SIM = 2.4
BACKWARD_SECONDS_SIM = 1.5 # Duration for the car backing up after a collision
USE_RESET_INSTEAD_OF_BACKWARDS_SIM = False # Respawn the car whenever there is a collision



class Drive():
    def __init__(self, sensors, is_simulator=False):
        self.is_simulator = is_simulator
        if not is_simulator:
            topic = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"
            max_steering = 0.34 # Estimation from https://github.com/MichaelBosello/f1tenth-RL/
            self.max_speed_reduction = MAX_SPEED_REDUCTION
            self.steering_speed_reduction = STEERING_SPEED_REDUCTION
            self.backward_speed_reduction = BACKWARD_SPEED_REDUCTION
            self.lightly_steering_reduction = LIGHTLY_STEERING_REDUCTION
            self.backward_seconds = BACKWARD_SECONDS
        else:
            topic = "/drive"
            max_steering = 0.4189 # From simulator parameters in params.yaml
            self.max_speed_reduction = MAX_SPEED_REDUCTION_SIM
            self.steering_speed_reduction = STEERING_SPEED_REDUCTION_SIM
            self.backward_speed_reduction = BACKWARD_SPEED_REDUCTION_SIM
            self.lightly_steering_reduction = LIGHTLY_STEERING_REDUCTION_SIM
            self.backward_seconds = BACKWARD_SECONDS_SIM
            self.reset_publisher = rospy.Publisher("/pose", PoseStamped, queue_size=0)
        self.max_speed = rospy.get_param("max_speed", 5)
        self.max_steering = rospy.get_param("max_steering", max_steering)
        self.drive_publisher = rospy.Publisher(topic, AckermannDriveStamped, queue_size=0)
        self.sensors = sensors
        self.stop()
        process = Thread(target=self.drive_command_runner)
        process.daemon = True
        process.start()
        print("max_speed: ", self.max_speed, ", max_steering: ", self.max_steering)


# Commands used in CarEnv
    def forward(self):
        self.send_drive_command(self.max_speed/self.max_speed_reduction, 0)
    
    def backward(self):
        self.send_drive_command(-self.max_speed/self.backward_speed_reduction, 0)
    
    def right(self):
        self.send_drive_command(self.max_speed/self.steering_speed_reduction, -self.max_steering)

    def left(self):
        self.send_drive_command(self.max_speed/self.steering_speed_reduction, self.max_steering)

    def slowdown(self):
        speed = self.last_speed/2 if self.last_speed/2 > self.max_speed/MIN_SPEED_REDUCTION else self.max_speed/MIN_SPEED_REDUCTION
        self.send_drive_command(speed, self.last_angle)

    def stop(self):
        self.send_drive_command(0, 0)

    def send_drive_command(self, speed, steering_angle):
        self.last_angle = steering_angle
        self.last_speed = speed
        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = speed
        ack_msg.drive.steering_angle = steering_angle
        self.ack_msg = ack_msg

    def drive_command_runner(self): # Publishing the Ackermann Messages
        while True:
            try:
                self.drive_publisher.publish(self.ack_msg)
            except ROSException as e:
                if str(e) == "publish() to a closed topic":
                    pass
                else:
                    raise e
            time.sleep(PUBLISHER_WAIT)

    def backward_until_obstacle(self): # In the simulator, make the car back-up after hitting a wall
        if USE_RESET_INSTEAD_OF_BACKWARDS_SIM and self.is_simulator:
            self.reset_simulator()
        else:
            self.backward()
            start = time.time()
            while time.time() - start < self.backward_seconds:
                time.sleep(0.01)
            self.stop()
            time.sleep(0.1)


    def reset_simulator(self):
        if self.is_simulator:
            self.reset_publisher.publish(PoseStamped())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulator", action='store_true', help="to set the use of the simulator")
    args = parser.parse_args()

    run_seconds = 0.3
    rospy.init_node('drive_test')
    drive = Drive(args.simulator)