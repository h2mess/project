#!/usr/bin/python3.3
###################################This is for loop time i.e., this is the first plot ################################

iterations = 100
reruns = 10
import re
f = open('../data/g28_lab09.csv')

s = f.read()

looptime= re.findall('^.*,([0-9.]*)$', s, re.MULTILINE)
steptime = re.findall('^[^,]*,[^,]*,([0-9.]*).*$', s, re.MULTILINE)
#print(looptime)
iteration = []
#print(len(looptime))
averagedLooptime = []
averagedSteptime = []
for x in range(iterations):
	iteration.append(x+1)
	summl = 0
	summs = 0
	ppointer = 1
	while ppointer <= reruns:
#		print(x*reruns + ppointer)
		if float(looptime[x*reruns + ppointer-1]) < 30000:
			summl = summl + float(looptime[x*reruns + ppointer - 1])
		if float(steptime[x*reruns + ppointer - 1]) < 30000:
			summs = summs + float(steptime[x*reruns + ppointer -1])
		ppointer+=1
	summl = summl/reruns
	summs = summs/reruns
	averagedLooptime.append(summl)
	averagedSteptime.append(summs)
import matplotlib.pyplot as plt
#plt.plot(iteration, averagedLooptime,'bo')
maxi = max(averagedLooptime)
mini = min(averagedLooptime)

fig = plt.figure()
fig.suptitle('g28_plot01')
plt.xlabel('iteration number')
plt.ylabel('time in ms')
plt.plot(iteration, [mini]*len(iteration),label = 'minimum')
#plt.Text(len(iteration),mini,'Me','r',fontsize=12)
plt.plot(iteration, [maxi]*len(iteration),label = 'maximum')
plt.plot(iteration, averagedLooptime, linewidth=1.5, linestyle = '-',label='average loop time') ##legend, markers, labels, title must be added here
ax = plt.subplot(111)
ax.bar(iteration,averagedSteptime,color='g',label='average step time')
plt.legend(loc=9,bbox_to_anchor=(0.5,1.05),ncol=3)
fig.savefig('../plots/g28_plot01.png')
plt.close()



################################# Here ends the first plot #############################################


##!/usr/bin/python3.3
#########################################Here starts the second question plots are named as individual, sum .pngs######################
iterations = 100
reruns = 10


import re
f = open('../data/g28_lab09.csv')

s = f.read()

steptime= re.findall('^[^,]*,[^,]*,([0-9.]*).*$', s, re.MULTILINE)
collisiontime = re.findall('^[^,]*,[^,]*,[^,]*,([0-9.]*).*$', s, re.MULTILINE)
velocitytime = re.findall('^[^,]*,[^,]*,[^,]*,[^,]*,([0-9.]*).*$', s, re.MULTILINE)
positiontime = re.findall('^[^,]*,[^,]*,[^,]*,[^,]*,[^,]*,([0-9.]*).*$', s, re.MULTILINE)
iteration = []
#print(len(steptime))
averagedSteptime = []
averagedCollisiontime = []
averagedVelocitytime = []
averagedPositiontime = []
for x in range(iterations):
	iteration.append(x+1)
	summs = 0
	summc = 0
	summv = 0
	summp = 0
	ppointer = 1
	while ppointer <= reruns:
		#		print(x*reruns + ppointer)
		if float(steptime[x*reruns + ppointer-1]) < 30000:
			summs = summs + float(steptime[x*reruns + ppointer - 1])
		if float(collisiontime[x*reruns + ppointer-1]) < 30000:
			summc = summc + float(collisiontime[x*reruns + ppointer - 1])
		if float(velocitytime[x*reruns + ppointer-1]) < 30000:
			summv = summv + float(velocitytime[x*reruns + ppointer - 1])
		if float(positiontime[x*reruns + ppointer-1]) < 30000:
			summp = summp + float(positiontime[x*reruns + ppointer - 1])
		ppointer+=1
	summs = summs/reruns
	summc = summc/reruns
	summv = summv/reruns
	summp = summp/reruns
	averagedSteptime.append(summs)
	averagedCollisiontime.append(summc)
	averagedVelocitytime.append(summv)
	averagedPositiontime.append(summp)
#import matplotlib.pyplot as plt
fig = plt.figure()
fig.suptitle('g28_plot02')
plt.xlabel('iteration number')
plt.ylabel('average time in ms')
plt.plot(iteration, averagedSteptime,linewidth=1.5, linestyle = '-', label = 'avg Step time')
plt.plot(iteration, averagedCollisiontime,linewidth=1.5, linestyle = '-', label = 'avg collision time')
plt.plot(iteration, averagedVelocitytime,linewidth=1.5, linestyle = '-', label = 'avg velocity time')
plt.plot(iteration, averagedPositiontime,linewidth=1.5, linestyle = '-', label = 'avg position time')
#plt.legend()
#plt.savefig('individual.png')
#plt.close()
nextArray = []
for x in range(iterations):
	nextArray.append(averagedCollisiontime[x]+averagedVelocitytime[x]+averagedPositiontime[x])
#plt.xlabel('iteration number')
#plt.ylabel('average time')
plt.plot(iteration, nextArray, linewidth = 1.5, linestyle = '-', label = 'sum of times')
plt.legend()
plt.savefig('../plots/g28_plot02.png')
plt.close()


#######################################Here ends the second question ##############################################

##!/usr/bin/python3.3
#########################################Here starts the third 3rd question  and name of file is std.png ###################################################
iterations = 100
reruns = 10
import re
f = open('../data/g28_lab09.csv')

s = f.read()

steptime= re.findall('^[^,]*,[^,]*,([0-9.]*).*$', s, re.MULTILINE)
iteration = []
#print(len(looptime))
averagedSteptime = []
standardDeviation = []
for x in range(iterations):
	iteration.append(x+1)
	summ = 0
	ppointer = 1
	while ppointer <= reruns:
		#		print(x*reruns + ppointer)
		if float(steptime[x*reruns + ppointer-1]) < 30000:
			summ = summ + float(steptime[x*reruns + ppointer - 1])
		ppointer+=1
	summ = summ/reruns
	averagedSteptime.append(summ)
for x in range(iterations):
	squaresum = 0
	ppointer = 1
	while ppointer <= reruns:
		if float(steptime[x*reruns + ppointer -1]) < 30000:
			squaresum = squaresum + (float(steptime[x*reruns + ppointer - 1]) - averagedSteptime[x])**2
		ppointer+=1
	squaresum = squaresum/reruns
	squaresum = squaresum**(0.5)
	standardDeviation.append(squaresum)
#import matplotlib.pyplot as plt
fig = plt.figure()
fig.suptitle('g28_plot03')
plt.xlabel('iteration number')
plt.ylabel('Average step time in ms')
plt.errorbar(iteration, averagedSteptime, yerr = standardDeviation, ecolor = 'red', linewidth=1.5, linestyle = '-', label = 'step time')
plt.legend()
plt.savefig('../plots/g28_plot03.png')
#plt.show()
plt.close()
############################################I think the 3rd question is also over. But, not sure ###########################################

##!/usr/bin/python3.3
############################################Start of 4th question. ################################################
import matplotlib.pyplot as plt
rollno = 73  ##this has to be changed to 73


iterations = 100
reruns = 10
import re
f = open('../data/g28_lab09.csv')

s = f.read()
steptime= re.findall('^[^,]*,[^,]*,([0-9.]*).*$', s, re.MULTILINE)
iteration = []
rollnoSteptime = []

for x in range(reruns):
	rollnoSteptime.append(float(steptime[(rollno - 1)*reruns + x]))
#print(len(rollnoSteptime))
xlimits = []
mini, maxi = min(rollnoSteptime), max(rollnoSteptime)
frequencies = [0,0,0,0,0,0,0,0,0,0]
width = (maxi - mini)/10
for x in range(10):
	xlimits.append(mini + x*width + width/2)

for x in range(reruns):
#	print(width)
#	print(rollnoSteptime[x]-mini)
#	print(int((rollnoSteptime[x]-mini)//width))
	if int(((rollnoSteptime[x]-mini)//width)) != 10:
		frequencies[int((rollnoSteptime[x]-mini)//width)]+=1
	else:
		frequencies[9]+=1
s = 0
for x in range(10): 
	s += frequencies[x]
	frequencies[x] = s
fig = plt.figure()
fig.suptitle('g28_plot04')
plt.plot(xlimits, frequencies, linewidth = 1.5, label = 'cumulative frequency')
plt.xlabel('step time in ms')
plt.ylabel('frequencies')


#print(rollnoSteptime)
#import matplotlib.pyplot as plt
plt.hist(rollnoSteptime, cumulative = False, label= 'frequencies')
#plt.hist(rollnoSteptime, cumulative = True, alpha = 0.5, label= 'frequencies')
plt.legend(loc=7)
plt.savefig('../plots/g28_plot04.png')
plt.close()
######################################################## Here ends the 4th question #####################################


##!/usr/bin/python3.3
############################################# Here starts the 5th question ##############################################




iterations = 100
reruns = 5
allreruns = 10
import re
f = open('../data/g28_lab09_random.csv')
g = open('../data/g28_lab09.csv')
s = f.read()
alls = g.read()
steptime= re.findall('^[^,]*,[^,]*,([0-9.]*),.*$', s, re.MULTILINE)
allsteptime= re.findall('^[^,]*,[^,]*,([0-9.]*),.*$', alls, re.MULTILINE)
iteration = []
#alliteration = []
#print(len(steptime))
averagedSteptime = []
allaveragedSteptime = []
for x in range(iterations):
	iteration.append(x+1)
	summ = 0
	allsumm = 0
	ppointer = 1
	while ppointer <= reruns:
		#		print(x*reruns + ppointer)
#		print(x*reruns + ppointer - 1)
#		print(x, reruns, ppointer)
		if float(steptime[x*reruns + ppointer-1]) < 30000:
			summ = summ + float(steptime[x*reruns + ppointer - 1])
		ppointer+=1
	ppointer = 1
	while ppointer <= allreruns:
		#		print(x*reruns + ppointer)
		if float(allsteptime[x*allreruns + ppointer-1]) < 30000:
			allsumm = allsumm + float(allsteptime[x*allreruns + ppointer - 1])
		ppointer+=1
	summ = summ/reruns
	allsumm = allsumm/allreruns
	averagedSteptime.append(summ)
	allaveragedSteptime.append(allsumm)	
#import matplotlib.pyplot as plt
#plt.plot(iteration, averagedSteptime,'bo')
#plt.plot(iteration, averagedSteptime, linewidth=0, linestyle = '-', marker = 'o', markerfacecolor = 'blue') ##legend, markers, labels, title must be added here
fig = plt.figure()
fig.suptitle('g28_plot05')

import numpy
fit = numpy.polyfit(iteration, averagedSteptime, 1)
fit_fn = numpy.poly1d(fit)
git = numpy.polyfit(iteration, allaveragedSteptime, 1)
git_fn = numpy.poly1d(git)
plt.xlabel('iteration number')
plt.ylabel('step time in ms')

plt.plot(iteration, averagedSteptime, iteration,fit_fn(iteration), label='Average step time with random points')
plt.plot(iteration, allaveragedSteptime, iteration,git_fn(iteration), label='Average step time')
plt.legend()
plt.savefig('../plots/g28_plot05.png')
plt.close()


