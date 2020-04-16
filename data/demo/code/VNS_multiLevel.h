#ifndef VNS_MULTILEVEL_H
#define VNS_MULTILEVEL_H

extern const float epsilonn;
extern int SIZE, CAPACITY, DISTANCE;
extern float **dist;
extern int *demand;
extern int* service_time; //global variable for service time
extern float GLOBAL_BEST_COST;
extern int GLOBAL_NO_ROUTE;
//extern int GLOBAL_NO_BEST_ROUTE;
extern int big_index; //to keep track of total routes for big_matrix file
extern int *GLOBAL_capa; //record capacity of the best solution //the same as total_demand???????????????????????????????????
extern int **GLOBAL; //to store the best solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
extern int* GLOBAL_SAIZ;
extern float *GLOBAL_Rcost;

extern const float alphamin; //pi/3
extern const float alphamax; //2pi/3
//================================ LOCAL_BEST_SOLUTION ======================================================//
extern float LOCAL_BEST_COST;
extern int LOCAL_NO_ROUTE;
extern int *LOCAL_capa; //record capacity of the local best solution 
extern int **LOCAL; //to store the local best solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
extern int* LOCAL_SAIZ;
extern float *LOCAL_Rcost;

//extern int **LOCAL_B;//to store the best of LOCAL, best of the best in all neighbourhoods, if it is not too far, eg 1% from the GLOBAL, use it for diversification (added 25Feb2016)
//extern float LOCAL_BCOST;//record the LOCAL_B cost
//extern int LOCAL_Br;//record number of routes for LOCAL_B
//extern float *LOCAL_Bc;
//extern int *LOCAL_Bs;//record the saiz
//extern int *LOCAL_Bq;//record capacity

//================================= GLOBAL VARIABLE FOR TEMPORARY SOLUTION =====================================//
//for inter-route imrovement
extern int number, module;
//extern int route_change[3]; //last two routes that have been changed  
extern int after_shake_route_change[6]; //record the route change in shake to update cost of removing accordingly, noneed to update all routes
extern int after_shake_route_changePtr; //to record how many routes have been changed, this if for multi-shake so that all routes chnged can be recorded

extern int **VEHICLE;
extern int* space_available;	//to check if the space available is enough for the new customer to be inserted
extern int* total_demand;
extern float* distance_available;
extern int* saiz;
extern float* route_cost;
extern int *which_module;
extern std::string getLastLine(std::ifstream& in);
extern bool **NR_FLAG;

extern int I;//for running different data sets loop

extern float** route_CGravity;
extern float **route_CGravity; //for guided shake
extern float *custRGravity;
extern float **sorted_custRGravity;

void VNS_kth_improvement(float *x, float *y);
void insert_1_0_2nd(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute);
void insert_1_1_2nd(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute);
void insert_2_1_2nd(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute);
void insert_2_0_2nd(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute);
void insert_2_2_swap2nd(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute);
void insert_swap1_1(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute);

void insert_2optintra(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE));
void insert_2optinter(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE));
void insert_crosstail(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE));
void insert_cross(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE));

int find_k_1_0(float *cost_of_removing, float **(&kGain), int k, int level);
int find_k_1_1(float *cost_of_removing, float **(&kGain), int k, int level);
int find_k_2_1(float *cost_of_removing, float **(&kGain), int k, int level);
int find_k_2_0(float *cost_of_removing, float **(&kGain), int k, int level);
int find_k_2_2swap(float *cost_of_removing, float **(&kGain), int k, int level);

int find_two_optintra(float **(&kGain), int k, int level);
int find_two_optinter(float **(&kGain), int k, int level);
int find_crossTail2(float **(&kGain), int k, int level);
int find_CROSS(float **(&kGain), int k, int level);

void adaptiveVNS_kth_improvement(float *x, float *y, float *(freq));
int find_swap_1_1(float *cost_of_removing, float **(&kGain), int K, int level);
void initializeEachRfullsearchstatus (int maxLocalSearch);
//void adaptiveVNS_kth_improvementNO(float *x, float *y, float *(freq));//no data structure of full search status

#endif