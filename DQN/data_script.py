import csv
import time
import datetime
import os
import numpy as np



class Output:
	def __init__(self):
		self.filename = ''
		self.start_time = datetime.datetime.now()
		self.initialised = False

	def initialise(self):
		self.filename = 'output_' + str(time.strftime("%m-%d-%H-%M-%S")) + '.csv'
		self.start_time = datetime.datetime.now()
		with open(self.filename, 'w',newline = '') as f:
			try:
				writer = csv.writer(f)
				writer.writerow(['Time', 'Velocity', 'Minimum LiDAR Range'])
				self.initialised = True
			except csv.Error as e:
				sys.exit('file {}, line {}: {}'.format(self.filename, writer.line_num, e))


	def append(self, lidar_ranges, velocity):
		min_range = min(np.array(lidar_ranges))
		relative_time = (datetime.datetime.now() - self.start_time).total_seconds()
		data = [str('{:.2f}'.format(relative_time)), str('{:.2f}'.format(velocity)),str('{:.2f}'.format(min_range))]
		if self.initialised == False:
			self.initialise()
		with open(self.filename, 'a', newline = '') as f:
			try:
				writer = csv.writer(f)
				writer.writerow(data)
			except csv.Error as e:
				sys.exit('file {}, line {}: {}'.format(self.filename, writer.line_num, e))


