import numpy as np
import os
import random
import time
import rospy

from state import State
from car.car_control import Drive
from car.safety_control import SafetyControl
from car.sensors import Sensors

LIDAR_DISTANCE_WEIGHT = 0.01 # Weight for LiDAR distance reward
VELOCITY_NORMALIZATION = 0.2 # Normalize the speed between 0 and 1 (=1/max_speed)
REWARD_SCALING = 0.09 # Scale the velocity rewards between 0 and REWARD_SCALING (at max velocity the reward is REWARD_SCALING)

class CarEnv:
    
    def __init__(self, args):
        self.history_length = 2
        self.is_simulator = args.simulator
        rospy.init_node('rl_driver')
        self.sensors = Sensors(is_simulator=args.simulator)
        self.control = Drive(self.sensors, is_simulator=args.simulator)
        self.safety_control = SafetyControl(self.control, self.sensors, is_simulator=args.simulator)
        time.sleep(4)

        # available actions
        self.action_set = [0, 1, 2, 3]

        self.game_number = 0
        self.step_number = 0
        self.is_terminal = False # Whether the state is terminal

        self.reset_game()

    def step(self, action): # Return the reward, the new state, and wether it is terminal
        self.is_terminal = False
        self.step_number += 1
        self.episode_step_number += 1

        if self.safety_control.emergency_brake: # When emergency braking is necessary
            if self.is_simulator:
                self.safety_control.disable_safety()
                time.sleep(0.3)
                self.control.backward_until_obstacle()
                self.safety_control.enable_safety()
                self.safety_control.unlock_brake()
                time.sleep(0.3)
            else:
                self.safety_control.disable_safety()
                time.sleep(0.6)
                self.control.backward_until_obstacle()
                time.sleep(0.4)
                self.safety_control.enable_safety()
                self.safety_control.unlock_brake()
                self.control.forward()
                time.sleep(0.4)

            reward = -1
            self.is_terminal = True
            self.game_score += reward
            return reward, self.state, self.is_terminal

        reward = 0  # Reward function definition
        """if action == 0:
            self.control.forward()
            reward = 0.08
        elif action == 1:
            self.control.right()
            reward = 0.02
        elif action == 2:
            self.control.left()
            reward = 0.02
        else:
            self.control.slowdown()
            reward = 0"""

        self.state = self.state.state_by_adding_data(self._get_car_state())

        reward = self.sensors.get_car_linear_velocity() * VELOCITY_NORMALIZATION * REWARD_SCALING + min(list(self.sensors.get_lidar_ranges())) * LIDAR_DISTANCE_WEIGHT

        self.game_score += reward
        return reward, self.state, self.is_terminal

    def reset_game(self):
        self.control.stop()

        if self.is_terminal:
            self.game_number += 1
            self.is_terminal = False
        self.state = State().state_by_adding_data(self._get_car_state())
        self.game_score = 0
        self.episode_step_number = 0
        self.car_stop_count = 0

    def _get_car_state(self):
        current_data = list(self.sensors.get_lidar_ranges())
        current_data.append(self.sensors.get_car_linear_velocity())
        return current_data


    def get_state_size(self):
        return len(self.state.get_data()[0])

    def get_num_actions(self):
        return len(self.action_set)

    def get_state(self):
        return self.state
    
    def get_game_number(self):
        return self.game_number
    
    def get_episode_step_number(self):
        return self.episode_step_number
    
    def get_step_number(self):
        return self.step_number
    
    def get_game_score(self):
        return self.game_score

    def is_game_over(self):
        return self.is_terminal
