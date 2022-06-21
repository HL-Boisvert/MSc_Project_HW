import numpy as np
import math
import time

try:
    import cv2
except ImportError:
    pass
try:
    import blosc
except ImportError:
    pass

class State:

    @staticmethod
    def setup(args):
        State.history_length = 2
        State.reduce_by = args.reduce_lidar_data
        State.cut_by = 10 # Remove the 10 minimum and 10 maximum outliers for the LiDAR data
        State.max_distance_norm = 20 # Divide by 20 to normalize LiDAR distances (considering 20 is the max distance that can be returned)

    def state_by_adding_data(self, data):
        data = self.process_data(data)

        new_state = State()
        if hasattr(self, 'data'):
            new_state.data = self.data[:State.history_length -1]
            new_state.data.insert(0, data)
        else:
            new_state.data = []
            for i in range(State.history_length):
                new_state.data.append(data)
        return new_state
    
    def get_data(self):
        state = self.data


        lidar_state = [state[0][0], state[1][0]]
        acc_state = [state[0][1], state[1][1]]
        return [np.asarray(lidar_state).reshape((len(lidar_state[0]), State.history_length)), np.asarray(acc_state)]

    def process_data(self, data):
        lidar_data, acceleration_value = data[:-1], data[-1]
        data = lidar_data

        #we have the same max value for sampling errors and max range exceeding
        #thus, we compute the reduction on the values under the max range and then we sobstitute the max value to the empty sets that resulted in 0
        data_avg = []
        for i in range(0, len(data), State.reduce_by):
            filtered = list(filter(lambda x:  x <= State.max_distance_norm, data[i:i + State.reduce_by]))
            if len(filtered) == 0:
                data_avg.append(State.max_distance_norm)
            else:
                data_avg.append(sum(filtered)/len(filtered))
        data = data_avg

        data = data[State.cut_by:-State.cut_by]
        if State.max_distance_norm > 1:
            data = [x / State.max_distance_norm for x in data]

        return (data, acceleration_value)
