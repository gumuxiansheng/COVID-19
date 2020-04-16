#ifndef GUIDED_SHAKE_H
#define GUIDED_SHAKE_H

extern int SIZE, CAPACITY, DISTANCE;
//extern int* service_time; //global variable for service time
//extern float GLOBAL_BEST_COST;
//extern int GLOBAL_NO_ROUTE;
////extern int GLOBAL_NO_BEST_ROUTE;
//extern int big_index; //to keep track of total routes for big_matrix file
//extern int *GLOBAL_capa; //record capacity of the best solution //the same as total_demand???????????????????????????????????
//extern int **GLOBAL; //to store the best solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//extern int* GLOBAL_SAIZ;
//extern const float alphamin; //pi/3
//extern const float alphamax; //2pi/3
//
////================================ LOCAL_BEST_SOLUTION ======================================================//
//extern float LOCAL_BEST_COST;
//extern int LOCAL_NO_ROUTE;
//extern int *LOCAL_capa; //record capacity of the local best solution 
//extern int **LOCAL; //to store the local best solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//extern int* LOCAL_SAIZ;
////================================= GLOBAL VARIABLE FOR TEMPORARY SOLUTION =====================================//
////for inter-route imrovement
//extern int number, module;
//extern int route_change[3]; //last two routes that have been changed  
////extern struct r_change previousChange; //to be used in reoptimize later
//
extern int* space_available;	//to check if the space available is enough for the new customer to be inserted
extern int* total_demand;
extern float* distance_available;
extern int* saiz;
extern float* route_cost;
extern int **VEHICLE;
extern int no_routes;
//extern int *which_module;
//extern bool **NR_FLAG;
//extern int I;//for running different data sets loop

extern float **route_CGravity; //for guided shake
extern float *custRGravity;
extern float **sorted_custRGravity;

void calculate_centreGravity(float *x, float *y);
void findDist_from_CGravity (int cust, float *x, float *y, float *custRGravity, float **(&sorted_custRGravity));
void onceSelected_shift_up (int route, int index, float **sorted_custRGravity);
int findRoute_fromRand (float **sorted_custRGravity, float rand);

#endif