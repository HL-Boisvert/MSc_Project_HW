import math
import numpy as np
import os
import tensorflow as tf
from tensorflow.keras import layers, initializers, losses, optimizers

class DeepQNetwork:
    def __init__(self, num_actions, state_size, replay_buffer, base_dir, tensorboard_dir, args):
        
        self.num_actions = num_actions
        self.state_size = state_size
        self.replay_buffer = replay_buffer
        self.history_length = 2

        self.learning_rate = args.learning_rate
        self.gamma = args.gamma
        self.target_model_update_freq = args.target_model_update_freq

        self.checkpoint_dir = base_dir + '/models/' # Directory to store the model

        if not os.path.isdir(self.checkpoint_dir):
            os.makedirs(self.checkpoint_dir)


        self.behavior_net = self.__build_q_net()
        self.target_net = self.__build_q_net()

        model_as_string = []
        self.target_net.summary(print_fn=lambda x : model_as_string.append(x))
        "\n".join(model_as_string)

        summary_writer = tf.summary.create_file_writer(tensorboard_dir) # Display a description of the model
        with summary_writer.as_default():
            tf.summary.text('model', model_as_string, step=0)

        if args.model is not None:
            self.target_net.load_weights(args.model)
            self.behavior_net.set_weights(self.target_net.get_weights())

    def __build_q_net(self):
        return self.__build_cnn1D_plus_velocity()

    def __build_dense(self):
        inputs = tf.keras.Input(shape=(self.state_size, self.history_length))
        x = layers.Dense(128, activation='relu', kernel_initializer=initializers.VarianceScaling(scale=2.))(inputs)
        x = layers.Dense(128, activation='relu', kernel_initializer=initializers.VarianceScaling(scale=2.))(x)
        x = layers.Flatten()(x)
        predictions = layers.Dense(self.num_actions, activation='linear', kernel_initializer=initializers.VarianceScaling(scale=2.))(x)
        model = tf.keras.Model(inputs=inputs, outputs=predictions)
        model.compile(optimizer=optimizers.Adam(self.learning_rate),
                            loss=losses.Huber()) #loss to be removed. It is needed in the bugged version installed on Jetson
        model.summary()
        return model

    def __build_cnn1D_plus_velocity(self):
        inputs = tf.keras.Input(shape=(self.state_size, self.history_length), name="lidar")
        input_acceleration = tf.keras.Input(shape=((self.history_length)), name="acc")
        x = layers.Conv1D(filters=16, kernel_size=4, strides=2, activation='relu', kernel_initializer=initializers.VarianceScaling(scale=2.))(inputs)
        x = layers.Conv1D(filters=32, kernel_size=2, strides=1, activation='relu', kernel_initializer=initializers.VarianceScaling(scale=2.))(x)
        x = layers.Flatten()(x)
        x = layers.concatenate([x, input_acceleration])
        x = layers.Dense(64, activation='relu', kernel_initializer=initializers.VarianceScaling(scale=2.))(x)
        predictions = layers.Dense(self.num_actions, activation='linear', kernel_initializer=initializers.VarianceScaling(scale=2.))(x)
        model = tf.keras.Model(inputs=[inputs, input_acceleration], outputs=predictions)
        model.compile(optimizer=optimizers.Adam(self.learning_rate),
                            loss=losses.Huber()) # loss to be removed. It is needed in the bugged version installed on Jetson
        model.summary()
        return model

    def inference(self, state):
        state[0] = state[0].reshape((-1, self.state_size, self.history_length))
        state[1] = state[1].reshape((-1, self.history_length))
        return np.asarray(self.behavior_predict(state)).argmax(axis=1)

    def train(self, batch, step_number):
        # Get states, actions, rewards and next states stored in memory
        old_states_lidar = np.asarray([sample.old_state.get_data()[0] for sample in batch])
        old_states_acc = np.asarray([sample.old_state.get_data()[1] for sample in batch])
        new_states_lidar = np.asarray([sample.new_state.get_data()[0] for sample in batch])
        new_states_acc = np.asarray([sample.new_state.get_data()[1] for sample in batch])
        actions = np.asarray([sample.action for sample in batch])
        rewards = np.asarray([sample.reward for sample in batch])
        is_terminal = np.asarray([sample.terminal for sample in batch]) # equals 1 if the state is terminal, else equals 0

        predicted = self.target_predict({'lidar': new_states_lidar, 'acc': new_states_acc}) # Returns the Q-value for each possible action
        q_new_state = np.max(predicted, axis=1) # returns the action with the highest score (Q-value)
        target_q = rewards + (self.gamma*q_new_state * (1-is_terminal))
        one_hot_actions = tf.keras.utils.to_categorical(actions, self.num_actions) # using tf.one_hot causes strange errors according to Bosello

        loss = self.gradient_train({'lidar': old_states_lidar, 'acc': old_states_acc}, target_q, one_hot_actions) # Perform gradient training on the target network

        if step_number % self.target_model_update_freq == 0: # Reset every so often the weights of the behaviour network to those of the target network
            self.behavior_net.set_weights(self.target_net.get_weights())

        return loss

    @tf.function
    def target_predict(self, state):
        return self.target_net(state)

    @tf.function
    def behavior_predict(self, state):
        return self.behavior_net(state)

    @tf.function
    def gradient_train(self, old_states, target_q, one_hot_actions): # Perform gradient training on the target network
        with tf.GradientTape() as tape:
            q_values = self.target_net(old_states)
            current_q = tf.reduce_sum(tf.multiply(q_values, one_hot_actions), axis=1)
            loss = losses.Huber()(target_q, current_q)

        variables = self.target_net.trainable_variables
        gradients = tape.gradient(loss, variables)
        self.target_net.optimizer.apply_gradients(zip(gradients, variables))

        return loss


    def save_network(self):
        print("saving..")
        self.target_net.save_weights(self.checkpoint_dir)
        self.replay_buffer.save()
        print("saved")
