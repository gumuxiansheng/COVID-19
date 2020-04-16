#ifndef VNS_H
#define VNS_H

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
//extern float *LOCAL_Bdistance_cost;
//extern int *LOCAL_Bs;//record the saiz
//extern int *LOCAL_Bq;//record capacity
//extern float **LOCAL_BCumDist;//record cumulative distance


extern float *GLOBAL_distance_cost;
extern float *LOCAL_distance_cost;
extern float *distance_cost;

extern float **GLOBALCumDist;//record cumulative distance
extern float **CumDist;//record cumulative distance
extern float **LOCALCumDist;//record cumulative distance

//================================= GLOBAL VARIABLE FOR TEMPORARY SOLUTION =====================================//
//for inter-route imrovement
extern int number, module;
extern int route_change[2]; //last two routes that have been changed  


extern int* space_available;	//to check if the space available is enough for the new customer to be inserted
extern int* total_demand;
extern float* distance_available;
extern int* saiz;
extern float* route_cost;
extern int *which_module;
extern std::string getLastLine(std::ifstream& in);
extern bool **NR_FLAG;
extern bool **NR_FLAG_DEPOT;
extern int I;//for running different data sets loop
extern float** route_CGravity;
extern int no_routes;

extern float **route_CGravity; //for guided shake
extern float *custRGravity;
extern float **sorted_custRGravity;

extern bool *RchangedStatus;
//extern bool *CustaffectedStatus;//record customer affected status

struct originalGain
{
	float *Oricost_of_removing; 
	float **Oriinfo_1_0; 
	float **Orisame_r_info; 
	float **Origain_matrix_1_0;
};

struct VNS_routeinfo //to be used in reoptimize later
{
	float t_cost;
	int t_demand;
	float t_distance;
};

//struct Rmodified
//{
//	int RchangedStatus;
//}*Route;

void VNS(float *x, float *y, float *(&freq));
//void VNS(int *(&capa), float *(&cost_of_removing),  float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0), bool **(NR_FLAG));
//void find_capacity(int *(&capa), float **best_routes, int *demand); //find capacity of best solution and display in capa[i]
void find_capacity(); //find capacity of best solution and display in capa[i]
//void VND(float *x, float *y, float **(&vehic_depotCust));
//void miniVND(float **(&vehic_depotCust)); //used by diversification stage in greedy insertion

//========================SHAKING============================//
int shake_1_0(int **(&VEHICLE), float *x, float *y);
int shake_1_1(int **(&VEHICLE), float *x, float *y);
int shake_2_0_twoR(int **(&VEHICLE),  float *x, float *y);
int shake_2_0_threeR(int **(&VEHICLE), float *x, float *y);
int shake_2_1_twoR(int **(&VEHICLE), float *x, float *y);
int shake_2_1_threeR(int **(&VEHICLE),  float *x, float *y);
int shake_2_2(int **(&VEHICLE), float *x, float *y); 
int shakeCROSS(int **(&VEHICLE), float *x, float *y);
int shake_intraReverseSegment(int **(&VEHICLE), float *x, float *y);//added 10June2016
int shake_intraSegmentReshuffle(int **(&VEHICLE));
int shake_intraHeadReshuffle(int **(&VEHICLE), int neighbourhood);
void add_empty_route();
//void insert_one_cust (int route_number, int position, float **(&routes), int *(&capa), int customer);
//void delete_one_cust (int route_number, int position, float **(&routes), int *(&capa));
//void delete_two_cust (int route_number, int position, float **(&routes), int *(&capa));
//void insert_two_cust (int route_number, int position, float **(&routes), int *(&capa), int customer, int customer2);
//void swap_in_oneCust (int route_number, int position, float **(&routes), int *(&capa), int customer);
//void swap2_1_cust (int route_number, int position, float **(&routes), int *(&capa), int customer); //2 cust out, 1 cust in
//void swap1_2_cust (int route_number, int position, float **(&routes), int *(&capa), int customer1, int customer2); //1 cust out, 2 cust in
void calculate_best_gain(float *(&cost_of_removing), float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0), originalGain &oriGain);

void find_all_cost_of_removing (float *(&cost_of_removing));
void reinitializeRchangedStatus ();
void partialupdate_reoptimize_1_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));
void partialupdate_reoptimize_1_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));
void partialupdate_reoptimize_2_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));
void partialupdate_reoptimize_2_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));
void partialupdate_reoptimize_2_2swap(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0));
int shake_reshuffle (int **(&VEHICLE));
int shakeCROSSbasedonNeighbourhood(int **(&VEHICLE), float *x, float *y, int neighbourhood); //shake_cross based on neighbourhood

void partialupdate_reoptimize_two_optintra(float **(&same_r_info), float **(&gain_matrix_1_0));
void partialupdate_reoptimize_two_optinter(float **(&info_1_0), float **(&gain_matrix_1_0));
void partialupdate_reoptimize_crossTail2(float **(&info_1_0), float **(&gain_matrix_1_0));
void partialupdate_reoptimize_CROSS(float **(&info_1_0), float **(&gain_matrix_1_0));

void partialupdate_costremoving(float *(&cost_of_removing));

#endif