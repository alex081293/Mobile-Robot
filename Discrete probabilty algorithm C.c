#include <stdio.h>
#include <math.h>

float prob2(float first, float second, int isSensor) {
	int indexValue2 = 0;
	float x = second - first;
	float distanceProbBuckets[5] = {.137931, .03448276, .2758621, .4827586, .6896551};
	float sensorProbBuckets[6] = {.03125, .265625, .03125, .4296785, .140625, .101565};

	if (isSensor) {

		if(x <= -1) indexValue2 = 0;
		else if (x >= -1 && x < -.5) indexValue2 = 1;
		else if (x >= -.5 && x < -.25) indexValue2 = 2;
		else if (x >= -.25 && x <= -.05) indexValue2 = 3;
		else if (x >= -.05 && x <= 0) indexValue2 = 4;		
		else if (x > 0) indexValue2 = 5;	
		return sensorProbBuckets[indexValue2];

	} else {

		if(x <= 4.95) indexValue2 = 0;
		else if (x > 4.95 && x <= 4.982) indexValue2 = 1;
		else if (x > 4.982 && x <= 4.97) indexValue2 = 2;
		else if (x > 4.97 && x <= 5.03) indexValue2 = 3;
		else if (x > 5.03) indexValue2 = 4;		
		
		return distanceProbBuckets[indexValue2];
	}
}


float prob1(float theoreticalDistance, float startingDistance, float oldProbofStartingDistance, int sensorValue) {
	float x = prob2(theoreticalDistance, sensorValue, 1);
	float y = prob2(theoreticalDistance, startingDistance, 0);
	float z = oldProbofStartingDistance;
	return x * y * z; 
}

float translateSensor(int sensor) {
	float distance = 0;
	// at 25
	if(sensor > 0 && sensor < 26) {
		if(sensor < 22) distance = 25.1;
		else if(sensor == 22) distance = 25.05;
		else if(sensor == 23) distance = 25;
		else if(sensor == 24) distance = 24.95;
		else if(sensor > 24) distance = 24.75;
	}
	// at 20
	else if(sensor >= 26 && sensor < 30) {
		if(sensor < 26) distance = 20.1;
		else if(sensor == 27) distance = 20.05;
		else if(sensor == 28) distance = 20;
		else if(sensor == 29) distance = 19.95;
	}
	// at 15
	else if(sensor >= 30 && sensor < 36) {
		if(sensor < 34) distance = 15.1;
		else if(sensor == 34) distance = 15.05;
		else if(sensor == 35) distance = 15;
		else if(sensor == 36) distance = 14.85;
	}
	// at 10
	else if(sensor >= 36) {
		if(sensor < 46) distance = 10.1;
		else if(sensor == 47) distance = 10.55;
		else if(sensor == 48) distance = 10;
		else if(sensor == 49) distance = 10.95;
		else if(sensor == 50) distance = 10.5;
		if(sensor > 50) distance = 10.1;
	}
	return distance;
}

int main() {
	float distanceBuckets[5] = {4.925, 4.4966, 4.976, 5.00, 5.045};
	float oldValues[50] = {0};
	float oldProbValues[50];
	float startingValues[50] ={0};
	float newStartingValues[50] = {0};
	float newStartingValues2[50] = {0};
	float newProbValues2[50] = {0};
	float total = 0;
	int i, j, k;
	float temp;
	int indexValue = 0;
	float n;
	float calcDistance;
	float probablitiy;
	float leftOvers;
	float theoreticalDistance;
	float sensorValue;


	// Place Forward
	sensorValue = translateSensor(25);
	for(i=0; i<5; i++) {
		theoreticalDistance = 30 - distanceBuckets[i];
		newStartingValues[i] = theoreticalDistance;
		oldValues[i] = prob1(theoreticalDistance, 30, 1, sensorValue);
		total+=oldValues[i];
	}
	n = 1/total;

	total = 0;
	for(i=0; i<5; i++) {
		oldValues[i] = n*oldValues[i];
		calcDistance += oldValues[i]*(30-distanceBuckets[i]);
		total += oldValues[i];
	}

	printf("%f\n", calcDistance);

	calcDistance = 0;

	// Place Forward

	for(i=0; i<5; i++) {
		startingValues[i] = newStartingValues[i];
		oldProbValues[i] = oldValues[i];
	}

	total = 0;
	indexValue = -1;
	sensorValue = translateSensor(20);
	for(i=0; i<5; i++) {
		probablitiy = oldProbValues[i];
		for(j=0; j<5; j++) {
			indexValue += 1;
			theoreticalDistance = startingValues[i] - distanceBuckets[j];
			newStartingValues2[indexValue] = theoreticalDistance;
			oldValues[indexValue] = prob1(theoreticalDistance, startingValues[i], probablitiy, sensorValue);
			total += oldValues[indexValue];
		}
	}

	n = 1 / total;
	indexValue = -1;
	calcDistance = 0;
	for(i=0;i<5;i++) {
		for(j=0;j<5;j++) {
			indexValue += 1;
			oldValues[indexValue] = oldValues[indexValue] * n;		
			newProbValues2[indexValue] = oldValues[indexValue];
			calcDistance += oldValues[indexValue]*(startingValues[i]-distanceBuckets[j]);
		}
	}

	printf("%f\n", calcDistance);

	for(i=0;i<25;i++) {
		for(j=0;j<25;j++) {
			if (newProbValues2[i] > newProbValues2[j]) {
				temp = newProbValues2[j];
				newProbValues2[j] = newProbValues2[i];
				newProbValues2[i] = temp;

				temp = newStartingValues2[j];
				newStartingValues2[j] = newStartingValues2[i];
				newStartingValues2[i] = temp;
			}
		}
	}
	total = 0;
	for(i=0; i<10;i++) {
		total += newProbValues2[i];
	}

	leftOvers = (1 - total) / 10;

	total = 0;

	for(i=0; i<10;i++) {
		newProbValues2[i] += leftOvers;
		total += newProbValues2[i];
	}

	for(i=0; i<10;i++) {
		oldProbValues[i] = newProbValues2[i];
		startingValues[i] = newStartingValues2[i];
	}

	for(k=0; k<2; k++) {

		if(k==0) sensorValue = translateSensor(15);
		else sensorValue = translateSensor(10);

		for(i=0;i<50;i++) {
			for(j=0;j<50;j++) {
				if (newProbValues2[i] > newProbValues2[j]) {
					temp = newProbValues2[j];
					newProbValues2[j] = newProbValues2[i];
					newProbValues2[i] = temp;

					temp = newStartingValues2[j];
					newStartingValues2[j] = newStartingValues2[i];
					newStartingValues2[i] = temp;
				}
			}
		}

		total = 0;
		for(i=0; i<10;i++) {
			total += newProbValues2[i];
		}
		leftOvers = (1 - total) / 10;

		total = 0;

		for(i=0; i<10;i++) {
			newProbValues2[i] += leftOvers;
			total += newProbValues2[i];
		}

		for(i=0; i<10;i++) {
			oldProbValues[i] = newProbValues2[i];
			startingValues[i] = newStartingValues2[i];
		}

		total = 0;
		indexValue = -1;
		for(i=0; i<10;i++) {
			probablitiy = oldProbValues[i];
			for(j=0; j<5;j++) {
				indexValue++;
				theoreticalDistance = startingValues[i] - distanceBuckets[j];
				oldValues[indexValue] = prob1(theoreticalDistance, startingValues[i], probablitiy, sensorValue);
				total += oldValues[indexValue];
				newStartingValues2[indexValue] = theoreticalDistance;
			}
		}
		n = 1 / total;
		indexValue = -1;
		calcDistance = 0;
		for(i=0; i<10;i++) {
			for(j=0; j<5;j++) {
				indexValue++;
				oldValues[indexValue] = oldValues[indexValue] * n;
				newProbValues2[indexValue] = oldValues[indexValue];
				calcDistance += oldValues[indexValue]*(startingValues[i]-distanceBuckets[j]);
			}
		}
		printf("%f\n", calcDistance);
	}

	return 0;
}


