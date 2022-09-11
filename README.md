# MSc_Project_HW
# Evaluating the Performance of Diﬀerent Reinforcement Learning Methods for Autonomous Racing

[[Dissertation](https://github.com/HL-Boisvert/MSc_Project_HW/blob/main/Dissertation/Evaluating the Performance of Different Reinforcement Learning Methods for Autonomous Racing.pdf)]
[[Videos of the evaluation of the DQN controllers on the physical car](https://bit.ly/3whXsn5)]

<img src="Images/car.jpg" alt="car top" width="450"/>

- The simulator is from https://github.com/f1tenth/f1tenth_simulator  
- The Wall Following controller is based on the code from the official F1Tenth lab n°3 at https://github.com/f1tenth/f1tenth_labs/tree/main/lab3  
- The DQN controller is based on https://github.com/MichaelBosello/f1tenth-RL  
- The code is designed to run on [f1tenth cars](https://f1tenth.org/), both on the real car and the simulator.  

The controllers implement several parameters both for the car (maximum speed, steering angle, acceleration...) and for the training process (learning rate, epsilon starting value and decay, size of the replay buffer...), to make the code as flexible as possible.  
Logging in Tensorboard was already implemented in https://github.com/MichaelBosello/, and logging to csv format was added.

## Introduction
_Abstract_ &mdash; The increasing popularity of Reinforcement Learning over the last decade has transformed the AI field. One area that has benefited from this so-called deep learning revolution is autonomous driving and more specifically autonomous racing. Many Reinforcement Learning based controllers have been implemented with varying levels of success, to such an extent that it has become a challenge to find the method with the right compromise between performance and complexity of implementation. Furthermore, most implementations of Reinforcement Learning controllers for autonomous racing are trained in a simulated environment and assume that the Sim2Real gap won't have too much of an impact on performance: this is something that can have a considerable impact on the choice of a specific method for real-life applications.  
This project will have a double aim. Firstly, it will be to compare the performance of different autonomous racing controllers and their ability to generalise to new data to help users choose one that corresponds to their needs. Secondly, it will be to compare the Sim2Real gaps to give users an idea of performance in real-life situations; this will be achieved by using the F1Tenth platform. This research report details my review of the relevant literature and establishes research questions, hypotheses, objectives, and a well-defined experimental protocol.

## Experiments

Several experiments have been performed (more details are available in the dissertation) to answer all experimental requirements:  

- In-simulator comparison of DQN to Wall Following: compares the performance of DQN to Wall Following on training track using both CNNs and NNs, with LiDAR data as input. The performance is assessed by three different metrics: speed, safety and smoothness; three different reward functions were tested.  
- In-simulator comparison of DQN to assess whether DRL generalises well to new tracks: compares the performance of DQN controllers trained on track A and tested on track B to DQN controllers tested and trained on track B.  
- Sim2real experiment to assess the Sim2Real gap: the DQN controllers are trained in the simulator and evaluated on the physical F1tenth car. Using the same metrics as before, the DQN controllers are compared to Wall Following and manual driving.  

## Installation

1. Install ROS [Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) or [Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)  
2. Install required dependencies:  
`$ sudo apt install rospkg catkin_pkg`  
`$ sudo apt install ros-noetic-ackermann-msgs` or `ros-melodic-ackermann-msgs`  
`$ sudo apt install tensorflow==2.0.0`  
When on a machine with ARM architecture, a [pre-built Tensorflow binary](https://github.com/PINTO0309/Tensorflow-bin#usage) has to be installed instead.  


## DQN training  
DQN/rl_car_driver.py  

`epsilon-decay = 0.99994`  

DQN/car_control.py  

`MAX_SPEED_REDUCTION_SIM = 1`  
`STEERING_SPEED_REDUCTION_SIM = 1.4`  
`BACKWARD_SPEED_REDUCTION_SIM = 4.5`  
`BACKWARD_SECONDS = 1.5`  


## Testing parameters for DQN
gpu-time = 0.0001  
train-epoch-steps = 0  
max-step-limit = 1000000  
save-model-freq = 100000  

## Run in simulator
1. Launch the F1Tenth simulator:  
`$ cd catkin_ws`  
`$ source devel/setup.bash`  
`$ roslaunch f1tenth_simulator simulator.launch`  
2. Launch the controller:  
`$ python3 rl_car_driver.py --simulator`  

### Simulator options:
+ The guide of the simulator is found in the readme *simulator/src/f1tenth_simulator/README/md*  

+ Many parameters can be changed in *simulator/src/f1tenth_simulator/params.yaml*  

+ The race track can be changed by editing *simulator/src/f1tenth_simulator/launch/simulator.launch*  

+ The simulation can be sped up by adding <param name="/use_sim_time" value="true"/> to *simulator/src/f1tenth_simulator/launch/simulator.launch* and changing the sim_speed_multiplier parameter in Clock.py

## Run on the physical car

`roslaunch racecar teleop.launch`  

## Load a model
 the argument --model can be used to load a trained model:  
`python3 rl_car_driver.py --model=./CNNRW3/models`
