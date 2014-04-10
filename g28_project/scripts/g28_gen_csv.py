#!/usr/bin/python3.3
import subprocess
import re
import csv
import random
import numpy as help
iterations = 100
reruns = 10
randomreruns = 5
#subprocess.call(['./mybins/cs296_28_exe', '15'])
with open('../data/g28_lab09.csv' , 'w') as csvfile_main:
	writer = csv.writer(csvfile_main,delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
	for i in range(iterations):
		for j in range(reruns):
			a = subprocess.check_output(['../mybins/cs296_28_exe', str(i+1)])
#			print(i,j)
#			b = re.findall(r'\d+\.\d+', str(a),flags=0)
			b = re.findall(r'[\d\.]+', str(a),flags=0)
			b = [float(i) for i in b]
#			print(b)
			b = [int(i+1) , int(j+1)] + b[1:len(b)]
			
			writer.writerow(b)


with open('../data/g28_lab09_random.csv' , 'w') as csvfile_random:
	writer= csv.writer(csvfile_random,delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
	for i in range(iterations):
		sequence_random = random.sample(range(reruns),randomreruns)
		data = help.loadtxt('../data/g28_lab09.csv', delimiter=',')
		for j in sequence_random:
			row = data[10*i + j]
			writer.writerow(row)
