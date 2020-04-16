#ifndef SORT_SOLUTION_H
#define SORT_SOLUTION_H

extern int SIZE, CAPACITY, DISTANCE;
extern int* demand;
extern int* service_time; //global variable for service time
extern int no_routes;

extern float GLOBAL_BEST_COST;
extern int GLOBAL_NO_ROUTE;
extern int *GLOBAL_capa; //record capacity of the best solution //the same as total_demand???????????????????????????????????
extern int **GLOBAL; //to store the best solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
extern int* GLOBAL_SAIZ;
extern float *GLOBAL_Rcost;

extern float *GLOBAL_distance_cost;
extern float *LOCAL_distance_cost;
extern float *distance_cost;
extern float **GLOBALCumDist;//record cumulative distance
extern float **CumDist;//record cumulative distance
extern float **LOCALCumDist;//record cumulative distance

//================================ LOCAL_BEST_SOLUTION ======================================================//
extern float LOCAL_BEST_COST;
extern int LOCAL_NO_ROUTE;
extern int *LOCAL_capa; //record capacity of the local best solution 
extern int **LOCAL; //to store the local best solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
extern int* LOCAL_SAIZ;
extern float *LOCAL_Rcost;
//================================= GLOBAL VARIABLE FOR TEMPORARY SOLUTION =====================================//
extern int* space_available;	//to check if the space available is enough for the new customer to be inserted
extern int* total_demand;
extern float* distance_available;
extern int* saiz;
extern float* route_cost;
extern float *distance_cost;
extern int I;//for running different data sets loop

void sort_GLOBALsolution (int **(&GLOBAL));
void sort_LOCALsolution (int **(&LOCAL));
void sort_solution (int **(&VEHICLE));

void sort_GLOBALsolutionSMALLEST (int **(&GLOBAL));
void sort_LOCALsolutionSMALLEST (int **(&LOCAL));
void sort_solutionSMALLEST (int **(&VEHICLE));

#endif