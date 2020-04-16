#ifndef BEST_IMPROVEMENT_H
#define BEST_IMPROVEMENT_H

extern int SIZE, CAPACITY, DISTANCE;
extern float **dist;
extern int *demand;

extern int* service_time; //global variable for service time
extern float GLOBAL_BEST_COST;
extern int GLOBAL_NO_ROUTE;
extern int big_index; //to keep track of total routes for big_matrix file
extern int *GLOBAL_capa; //record capacity of the best solution //the same as total_demand???????????????????????????????????
extern int **GLOBAL; //to store the best solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
extern int* GLOBAL_SAIZ;
extern float *GLOBAL_Rcost;
extern float *GLOBAL_distance_cost;

extern float **GLOBALCumDist;//record cumulative distance
extern float **CumDist;//record cumulative distance

extern const float alphamin; //pi/3
extern const float alphamax; //2pi/3
extern const float epsilon;
//================================= GLOBAL VARIABLE FOR TEMPORARY SOLUTION =====================================//
//for inter-route imrovement
extern int number, module; //number is corresponding to number in lower diagonal of gain matrix
extern int route_change[2]; //last two routes that have been changed

//extern struct r_change previousChange; //to be used in reoptimize later
extern int **VEHICLE;
extern int* total_demand;
extern int* saiz;
extern float* route_cost;
extern float* distance_cost;
extern int* space_available;	//to check if the space available is enough for the new customer to be inserted
extern float* distance_available;

extern bool **NR_FLAG;
extern bool **NR_FLAG_DEPOT;
struct r_change //to be used in reoptimize later
{
	int r1;
	int r2;
};
extern int I;//for running different data sets loop

void inter_route_improvement();
void inter_route_improvementforLNS();
void best_gain_1_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));
void insert_1_0(float max_gain, float *(&cost_of_removing), float **info_1_0, float **(same_r_info), int **(&VEHICLE), bool same_route);//int **(&VEHICLE) to show that VEHICLE will be updated in this function, actually noneed pass as argument, just for clarity
void reoptimize_1_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));

void best_gain_1_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));
void insert_1_1(float max_gain, float *(&cost_of_removing), float **info_1_0, float **(same_r_info), int **(&VEHICLE), bool same_route);
void reoptimize_1_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));

void best_gain_2_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));
void insert_2_1(float max_gain, float *(&cost_of_removing), float **info_1_0, float **(same_r_info), int **(&VEHICLE), bool same_route);
void reoptimize_2_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));

void best_gain_2_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));
void insert_2_0(float max_gain, float *(&cost_of_removing), float **info_1_0, float **(same_r_info), int **(&VEHICLE), bool same_route);
void reoptimize_2_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));

void best_gain_2_2swap(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));
void insert_2_2swap(float max_gain, float *(&cost_of_removing), float **info_1_0, float **(same_r_info), int **(&VEHICLE), bool same_route);
void reoptimize_2_2swap(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));

void two_optintra(float **(&same_r_info), float **(&gain_matrix_1_0));
void two_optinter(float **(&info_1_0), float **(&gain_matrix_1_0));
void crossTail2(float **(&info_1_0), float **(&gain_matrix_1_0));
void CROSS(float **(&info_1_0), float **(&gain_matrix_1_0));

void reoptimize_two_optintra(float **(&same_r_info), float **(&gain_matrix_1_0));
void reoptimize_two_optinter(float **(&info_1_0), float **(&gain_matrix_1_0));
void reoptimize_crossTail2(float **(&info_1_0), float **(&gain_matrix_1_0));
void reoptimize_CROSS(float **(&info_1_0), float **(&gain_matrix_1_0));

void insert_2optintra(float max_gain, float *(&cost_of_removing), float **(same_r_info), int **(&VEHICLE));
void insert_2optinter(float max_gain, float *(&cost_of_removing), float **info_1_0, int **(&VEHICLE));
void insert_crosstail(float max_gain, float *(&cost_of_removing), float **info_1_0, int **(&VEHICLE));
void insert_cross(float max_gain, float *(&cost_of_removing), float **info_1_0, int **(&VEHICLE));

//void updateCostofRemoving (int f, int fromP, int toP, float *(&cost_of_removing), bool *(&tempFlag));
void updateCostofRemoving2 (int f, float *(&cost_of_removing));
int comprareCost (float *cost_of_removing, int f);

#endif