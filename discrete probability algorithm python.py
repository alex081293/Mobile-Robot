def prob1(theoreticalDistance, startingDistance, oldProbofStartingDistance, sensorValue):
		x = prob2(theoreticalDistance, sensorValue, 1)
		y = prob2(theoreticalDistance, startingDistance)
		z = oldProbofStartingDistance
		return x * y * z


def prob2(first, second, isSensor = 0):
	index = 0
	x = second - first
	if (isSensor):
		if(x <= -1):
			index = 0
		elif (x >= -1 and x < -.5): 
			index = 1
		elif (x >= -.5 and x < -.25):
			index = 2
		elif (x >= -.25 and x <= -.05):
			index = 3
		elif (x >= -.05 and x <= 0):
			index = 4		
		elif (x > 0):
			index = 5
		return sesnorProbBuckets[index]
	else:
		if(x <= 4.95):
			index = 0
		elif (x > 4.95 and x <= 4.982):
			index = 1
		elif (x > 4.982 and x <= 4.97): 
			index = 2
		elif (x > 4.97 and x <= 5.03): 
			index = 3
		elif (x > 5.03):
			index = 4	
				
		return distanceProbBuckets[index]


distanceBuckets = [4.925, 4.4966, 4.976, 5.00, 5.045]
distanceProbBuckets = [.137931, .03448276, .2758621, .4827586, .6896551]

sesnorProbBuckets = [.03125, .265625, .03125, .4296785, .140625, .101565]

oldValues = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
newStartingValues = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
newStartingValues2 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
newProbValues2 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

total = 0

for i in range(5):
	newStartingValues[i] = theoreticalDistance = 30 - distanceBuckets[i]
	oldValues[i] = prob1(theoreticalDistance, 30, 1, 25)
	total+=oldValues[i]

n = 1/total

calcDistance = 0

total = 0
for i in range(5): 
	oldValues[i] = n*oldValues[i]
	calcDistance += oldValues[i]*(30-distanceBuckets[i])
	total += oldValues[i]

print calcDistance
# second run

startingValues = newStartingValues;
oldProbValues = oldValues

total = 0
index = -1
for i in range(5):
	probablitiy = oldProbValues[i]
	for j in range(5):
		index += 1
		theoreticalDistance = startingValues[i] - distanceBuckets[j]
		newStartingValues2[index] = theoreticalDistance
		oldValues[index] = prob1(theoreticalDistance, startingValues[i], probablitiy, 20)
		total += oldValues[index]

n = 1 / total

index = -1
calcDistance = 0
for i in range(5):
	for j in range(5):
		index += 1
		oldValues[index] = oldValues[index] * n			
		newProbValues2[index] = oldValues[index]
		calcDistance += oldValues[index]*(startingValues[i]-distanceBuckets[j])

print calcDistance

# sets up data for next iteration
for i in range(25):
	for j in range(25):
		if newProbValues2[i] > newProbValues2[j]:
			temp = newProbValues2[j]
			newProbValues2[j] = newProbValues2[i]
			newProbValues2[i] = temp;

			temp = newStartingValues2[j]
			newStartingValues2[j] = newStartingValues2[i]
			newStartingValues2[i] = temp;

total = 0
for i in range(10):
	total += newProbValues2[i]
leftOvers = (1 - total)/10

total = 0
for i in range(10):
	newProbValues2[i] += leftOvers
	total += newProbValues2[i]

for i in range(10):
	oldProbValues[i] = newProbValues2[i]
	startingValues[i] = newStartingValues2[i]

# round 3
for k in range(2):
	if(k == 0):
		sensorValue = 15
	if(k == 1):
		sensorValue = 10

	for i in range(50):
		for j in range(50):
			if newProbValues2[i] > newProbValues2[j]:
				temp = newProbValues2[j]
				newProbValues2[j] = newProbValues2[i]
				newProbValues2[i] = temp;

				temp = newStartingValues2[j]
				newStartingValues2[j] = newStartingValues2[i]
				newStartingValues2[i] = temp;

	total = 0
	for i in range(10):
		total += newProbValues2[i]
	leftOvers = (1 - total)/10

	total = 0
	for i in range(10):
		newProbValues2[i] += leftOvers
		total += newProbValues2[i]

	for i in range(10):
		oldProbValues[i] = newProbValues2[i]
		startingValues[i] = newStartingValues2[i]



	total = 0
	index = -1
	for i in range(10):
		probablitiy = oldProbValues[i]
		for j in range(5):
			index += 1
			theoreticalDistance = startingValues[i] - distanceBuckets[j]
			oldValues[index] = prob1(theoreticalDistance, startingValues[i], probablitiy, sensorValue)
			total += oldValues[index]
			newStartingValues2[index] = theoreticalDistance


	n = 1 / total
	index = -1
	calcDistance = 0
	for i in range(10):
		for j in range(5):
			index += 1
			oldValues[index] = oldValues[index] * n
			newProbValues2[index] = oldValues[index]
			calcDistance += oldValues[index]*(startingValues[i]-distanceBuckets[j])

	print calcDistance
