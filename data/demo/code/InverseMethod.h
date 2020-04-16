#ifndef INVERSE_H
#define INVERSE_H

extern int SIZE, CAPACITY, DISTANCE;
extern int* service_time; //global variable for service time
extern int I;//for running different data sets loop

void adjustFreq(float *(&freq), int num_module);
void findCumu(float *freq, int num_module, float *(&cutPoint), bool *(flag_m));
int find_module (float rand, float *cutPoint, int num_module,  bool *(flag_m));

#endif