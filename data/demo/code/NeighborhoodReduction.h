#ifndef NR_H
#define NR_H

extern int SIZE, CAPACITY, DISTANCE;
extern float **dist;
extern int* service_time; //global variable for service time
extern bool **NR_FLAG; //global variable for neighborhood reduction
extern bool **NR_FLAG_DEPOT; //global variable for neighborhood reduction consider when inserting between customer and depot
extern const float alphamin; //pi/6 = 30 degree
extern const float alphamax; //pi/3 = 60 degree
extern int I;//for running different data sets loop

void neighborhood_reduction(float *x, float *y, int TOTAL_DEMAND);

#endif