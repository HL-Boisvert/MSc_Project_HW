#!/usr/bin/env python3
#
from threading import Thread
import sys
import numpy as np
import os
import random
import time
import argparse
import datetime
import tensorflow as tf

import replay
import dqn
from car_env import CarEnv
from state import State

#################################
# parameters
#################################

parser = argparse.ArgumentParser()
# real car or simulator
parser.add_argument("--simulator", action='store_true', help="to set the use of the simulator")
# agent parameters
parser.add_argument("--learning-rate", type=float, default=0.00042, help="learning rate of the NN")
parser.add_argument("--gamma", type=float, default=0.98, help="""gamma [0, 1] is the discount factor. It determines the importance of future rewards.
                                A factor of 0 will make the agent consider only immediate reward, a factor approaching 1 will make it strive for a long-term high reward""")
parser.add_argument("--epsilon", type=float, default=1, help="]0, 1]for epsilon greedy train")
parser.add_argument("--epsilon-decay", type=float, default=0.99994, help="]0, 1] every step epsilon = epsilon * decay, in order to decrease constantly")
parser.add_argument("--epsilon-min", type=float, default=0.1, help="epsilon with decay doesn't fall below epsilon min")
parser.add_argument("--target-model-update-freq", type=int, default=500, help="how often (in steps) to update the target model")
parser.add_argument("--model", help="load a pre-trained model")
# lidar pre-processing
parser.add_argument("--reduce-lidar-data", type=int, default=27, help="lidar data are grouped by taking the min of [reduce-lidar-data] elements")
# train parameters
parser.add_argument("--train-epoch-steps", type=int, default=3500, help="how many steps (1 step = [history-length] frames) to run during a training epoch")
parser.add_argument("--eval-epoch-steps", type=int, default=500, help="how many steps (1 step = [history-length] frames) to run during an eval epoch")
parser.add_argument("--max-step-limit", type=int, default=2000, help="maximum steps that can be done in one episode")
parser.add_argument("--replay-capacity", type=int, default=100000, help="how many states to store for future training")
parser.add_argument("--save-model-freq", type=int, default=10000, help="save the model every X steps")
args = parser.parse_args()

print('Arguments: ', (args))


#################################
# setup
#################################

base_output_dir = 'run-out-' + time.strftime("%Y-%m-%d-%H-%M-%S")
os.makedirs(base_output_dir)
f = open('data.csv','w')

tensorboard_dir = base_output_dir + "/tensorboard/"
os.makedirs(tensorboard_dir)
summary_writer = tf.summary.create_file_writer(tensorboard_dir)
with summary_writer.as_default():
    tf.summary.text('params', str(args), step=0)

State.setup(args)

environment = CarEnv(args)
replay_memory = replay.ReplayMemory(base_output_dir, args)
dqn = dqn.DeepQNetwork(environment.get_num_actions(), environment.get_state_size(),
                        replay_memory, base_output_dir, tensorboard_dir, args)

train_epsilon = args.epsilon # Before the run_epoch function to avoid resetting epsilon between epochs
start_time = datetime.datetime.now()
train_episodes = 0
eval_episodes = 0
episode_train_reward_list = []
episode_eval_reward_list = []
first_train = True

#################################
# stop handler
#################################

stop = False
pause = False

def stop_handler():
  global stop
  while not stop:
    user_input = input()
    if user_input == 'q':
      print("Stopping...")
      stop = True

process = Thread(target=stop_handler)
process.daemon = True
process.start()

#################################
# training cycle definition
#################################

def run_epoch(min_epoch_steps, eval_with_epsilon=None):
    global train_epsilon
    global train_episodes
    global eval_episodes
    global episode_train_reward_list
    global episode_eval_reward_list
    global first_train
    is_training = True if eval_with_epsilon is None else False
    step_start = environment.get_step_number()
    start_game_number = environment.get_game_number()
    epoch_total_score = 0
    stuck_count = 0
    time_list = []

    while environment.get_step_number() - step_start < min_epoch_steps and not stop:
        state_reward = 0
        state = environment.get_state()
        
        episode_losses = []
        save_net = False
        while not environment.is_game_over() and not stop and environment.get_episode_step_number() < args.max_step_limit:
            f.write('\n' + str(datetime.datetime.now()) + ',' + str(environment.sensors.get_car_linear_velocity())) # Exporting data to csv
            # epsilon selection and update
            if is_training:
                epsilon = train_epsilon
                if train_epsilon > args.epsilon_min:
                    train_epsilon = train_epsilon * args.epsilon_decay
                    if train_epsilon < args.epsilon_min:
                        train_epsilon = args.epsilon_min
            else:
                epsilon = eval_with_epsilon

            # action selection following epsilon-greedy policy
            if random.random() < epsilon:
                action = random.randrange(environment.get_num_actions())
            else:
                action = dqn.inference(state.get_data())

            # We need to wait the evolution of the environment, but we don't want to waste GPU time
            # We can use a training sweep (which requires some time) instead of using a sleep
            old_state = state
            for i in range(0, 6): # upper bound is history_length * (repeat_action + 1)

                if environment.get_step_number() % args.save_model_freq == 0:
                    save_net = True

                # Make the move
                reward, state, is_terminal = environment.step(action)


                # train
                if is_training and old_state is not None:
                    if environment.get_step_number() > 500: # Wait 500 steps before training to store samples in the replay memory
                        if first_train:
                            environment.control.stop()
                            print("LOADING TENSORFLOW MODEL, PLEASE WAIT...")
                            first_train = False
                        batch = replay_memory.draw_batch(32) # Use a 32 samples batch size
                        loss = dqn.train(batch, environment.get_step_number())
                        episode_losses.append(loss)
                        time.sleep(0.0003)
                    else:
                        time.sleep(0.0003) # In seconds, based on the speed of an Apple M1 GPU: it is needed to have the same states and rewards as
                                           # training takes time and the environment evolves independently
                else:
                    time.sleep(0.0003) # In seconds, based on the speed of an Apple M1 GPU
                
                if is_terminal:
                    break

            # Record experience in replay memory
            if is_training and old_state is not None:
                replay_memory.add_sample(replay.Sample(old_state, action, reward, state, is_terminal))

            if is_terminal:
                state = None

            if args.simulator:
                if reward == -1:
                    stuck_count = stuck_count + 1
                else:
                    stuck_count = 0
                if stuck_count > 2:
                    print("Car stuck, resetting simulator position...")
                    environment.control.reset_simulator()
                    stuck_count = 0

        if save_net:
            dqn.save_network()



        #################################
        # logging
        #################################

        episode_time = datetime.datetime.now() - start_time

        if is_training: # Training episode
            train_episodes += 1
            episode_train_reward_list.insert(0, environment.get_game_score())
            if len(episode_train_reward_list) > 100:
                episode_train_reward_list = episode_train_reward_list[:-1]
            avg_rewards = np.mean(episode_train_reward_list)

            episode_avg_loss = 0
            if episode_losses:
                episode_avg_loss = np.mean(episode_losses)

            log = ('Episode %d ended with score: %.2f (%s elapsed) (step: %d). Avg score: %.2f Avg loss: %.5f' %
                (environment.get_game_number(), environment.get_game_score(), str(episode_time),
                environment.get_step_number(), avg_rewards, episode_avg_loss))
            print(log)
            print("   epsilon " + str(train_epsilon))
            with summary_writer.as_default():
                tf.summary.scalar('train episode reward', environment.get_game_score(), step=train_episodes)
                tf.summary.scalar('train avg reward(100)', avg_rewards, step=train_episodes)
                tf.summary.scalar('average loss', episode_avg_loss, step=train_episodes)
                tf.summary.scalar('epsilon', train_epsilon, step=train_episodes)
                tf.summary.scalar('steps', environment.get_step_number(), step=train_episodes)
        else: # Evaluating episode
            eval_episodes += 1
            episode_eval_reward_list.insert(0, environment.get_game_score())
            if len(episode_eval_reward_list) > 100:
                episode_eval_reward_list = episode_eval_reward_list[:-1]
            avg_rewards = np.mean(episode_eval_reward_list)

            log = ('Eval %d ended with score: %.2f (%s elapsed) (step: %d). Avg score: %.2f' %
                (environment.get_game_number(), environment.get_game_score(), str(episode_time),
                environment.get_step_number(), avg_rewards))
            print(log)
            with summary_writer.as_default():
                tf.summary.scalar('eval episode reward', environment.get_game_score(), step=eval_episodes)
                tf.summary.scalar('eval avg reward(100)', avg_rewards, step=eval_episodes)

        epoch_total_score += environment.get_game_score()
        environment.reset_game()

        while pause and not stop:
            time.sleep(1)


    if environment.get_game_number() - start_game_number == 0:
        return 0
    return epoch_total_score / (environment.get_game_number() - start_game_number)

#################################
# main
#################################

while not stop:
    avg_score = run_epoch(args.train_epoch_steps) # train
    print('Average epoch training score: %d' % (avg_score))
    avg_score = run_epoch(args.eval_epoch_steps, eval_with_epsilon=0) # eval
    print('Average epoch eval score: %d' % (avg_score))
