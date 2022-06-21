import bisect
import math
import random
import os
import pickle

class Sample:
    def __init__(self, old_state, action, reward, new_state, terminal):
        self.old_state = old_state
        self.action = action
        self.reward = reward
        self.new_state = new_state
        self.terminal = terminal
        self.weight = 1
        self.cumulative_weight = 1

    def is_interesting(self):
        return self.terminal or self.reward != 0

    def __cmp__(self, obj):
        return self.cumulative_weight - obj.cumulative_weight


class ReplayMemory:
    def __init__(self, base_output_dir, args):
        self.save_buffer_dir = base_output_dir + "/models/" # Store the .dat file in the models sub-directory
        if not os.path.isdir(self.save_buffer_dir):
            os.makedirs(self.save_buffer_dir)
        self.file = "replay_buffer.dat"
        self.samples = []
        self.max_samples = args.replay_capacity # Maximum number of samples in the buffer
        self.num_interesting_samples = 0
        self.batches_drawn = 0

        if args.model is not None:
            self.load(args.model + self.file)

    def num_samples(self):
        return len(self.samples)

    def add_sample(self, sample):
        self.samples.append(sample)
        self._truncate_list_if_necessary()

    def draw_batch(self, batch_size):
        if batch_size > len(self.samples):
            raise IndexError('Too few samples (%d) to draw a batch of %d' % (len(self.samples), batch_size))
        
        return random.sample(self.samples, batch_size) # Pick the samples randomly from the ReplayMemory

    def save(self):
        with open(self.save_buffer_dir + self.file, "wb") as f:
            pickle.dump(self.samples, f)

    def load(self, file):
        with open(file, "rb") as f:
            self.samples = pickle.load(f)


    def _truncate_list_if_necessary(self):
        if len(self.samples) > self.max_samples * 1.05:
            # Truncate the list
            self.samples = self.samples[(len(self.samples) - self.max_samples):]
