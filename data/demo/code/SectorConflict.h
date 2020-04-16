#ifndef SECTOR_CONFLICT_H
#define SECTOR_CONFLICT_H

extern int SIZE, CAPACITY, DISTANCE;
extern float **dist;
extern int *demand;
extern int* service_time; //global variable for service time
extern const float chi; //parameter for capacity constraint
extern const float delta; //parameter for distance constraint

extern float GLOBAL_BEST_COST;
extern int GLOBAL_NO_ROUTE;
extern int big_index; //to keep track of total routes for big_matrix file
extern int *GLOBAL_capa; //record capacity of the best solution //the same as total_demand???????????????????????????????????
extern int **GLOBAL; //to store the best solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
extern int* GLOBAL_SAIZ;
extern float *GLOBAL_Rcost;
//=============================================================================================================//
//extern const float PI;
extern float *theta;
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
extern int I;//for running different data sets loop
extern int **VEHICLE;
extern int no_routes;

float Diversification_conflict_sector(int **(&VEHICLE), int K, float *x, float *y);


#endif
