#ifndef DIVERSIFICATION_LNS_H
#define DIVERSIFICATION_LNS_H

extern int SIZE, CAPACITY, DISTANCE, SERVICE_T;
extern float **dist;
extern int *demand;
extern int* service_time; //global variable for service time
extern float GLOBAL_BEST_COST;
extern int GLOBAL_NO_ROUTE;
//extern int big_index; //to keep track of total routes for big_matrix file
extern int *GLOBAL_capa; //record capacity of the best solution //the same as total_demand???????????????????????????????????
extern int **GLOBAL; //to store the best solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
extern int* GLOBAL_SAIZ;
extern float *GLOBAL_Rcost;
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
extern bool **NR_FLAG; //global variable for neighborhood reduction
extern bool **DEMAND_FLAG;
//================================= GLOBAL VARIABLE FOR TEMPORARY SOLUTION =====================================//
//for inter-route imrovement
//extern int number, module; //number is corresponding to number in lower diagonal of gain matrix
//extern int route_change[3]; //last two routes that have been changed  
//extern struct r_change previousChange; //to be used in reoptimize later

extern int I;//for running different data sets loop
struct cust2
{
	int custID;
	float costInsert;
	int bestPos;
	float distInsert;
};

float Diversify_LongestArc(int **(&VEHICLE), int Kappa);
float Diversify_LongestArc2(int **(&VEHICLE), int Kappa);
float Diversify_LNS2(int **(&VEHICLE), int Kappa);
float Diversify_LNS3(int **(&VEHICLE), int Kappa);
float deleteHeadTail(int **(&VEHICLE), int Kappa);
float Diversify_Relatedness(int **(&VEHICLE), int Kappa);
float Diversify_RelatednessDemand(int **(&VEHICLE), int Kappa);
float Diversify_BadArcNR(int **(&VEHICLE), int Kappa);
float Diversify_EntireRoute(int **(&VEHICLE), int Kappa, float *x, float *y) ;

void two_opt_singleR(int **(&VEHICLE), int route);
//void or_opt_singleR(int **(&VEHICLE), int route);
void or_opt_singleR2(int **(&VEHICLE), int r);
void relocatesameR(int **(&VEHICLE), int r, int i, int j, int type);

int pertub1_1 (int **(&VEHICLE), int cust,  int &pos, float &bestCost, float &bestDist);
void greedyInsertion(int **(&VEHICLE), int numcustRemoved, int* custRemoved);
void greedyInsertion2(int **(&VEHICLE), int numcustRemoved, int* custRemoved);
bool findbestinsertion(int numcustRemoved, bool *insertStatus, cust2 **cheapInsert, int *custRemoved, int &id, int &bestR, int &bestPos, float &bestCost, float &bestDist);
void ReevaluateAffectedR (int numcustRemoved, bool *insertStatus, cust2 **(&cheapInsert), int *custRemoved);

bool fullPerturb (int **(&VEHICLE), int cust);//insertion is done here!!!!!!!!!!!!!! , fullPerturb will return true is successly done
void bestInsertionwithCapacityExceed (int **(&VEHICLE), int cust);

void regretInsertion(int **(&VEHICLE), int numcustRemoved, int* custRemoved); //added on 25Feb2016
bool findmaxRegret(int numcustRemoved, bool *insertStatus, cust2 **cheapInsert1, cust2 **cheapInsert2, int *custRemoved, int &id, int &bestR, int &bestPos, float &bestCost, float &bestDist);
void ReevaluateAffectedRinsertions (int numcustRemoved, bool *insertStatus, cust2 **(&cheapInsert1), cust2 **(&cheapInsert2), int *custRemoved);

//void insert_one (int route_number, int position, int **(&routes), int customer);
//void delete_one(int route_number, int position, int **(&routes));
//void copytovehicle(int **vehic_depotCust, float **(&vehicle));
//void copytovehic_depotCust(int **(&vehic_depotCust), float **(vehicle));



#endif