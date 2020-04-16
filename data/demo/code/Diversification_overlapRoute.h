#ifndef OVERLAP_ROUTE_H
#define OVERLAP_ROUTE_H

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
extern float* distance_cost;
extern int I;//for running different data sets loop
extern int no_routes;
extern int **VEHICLE;
extern float **CumDist;


struct Point 
{   
    float x, y;  
}; 
bool onSegment(Point p, Point q, Point r);
int orientation(Point p, Point q, Point r);
bool doIntersect(Point p1, Point q1, Point p2, Point q2);
float Diversification_overlapRoute(int **(&VEHICLE), int K, float *x, float *y);
//Diversification_overlapRoute2 will delete the overlap arc and the neighbourhood of the customer that overlap
float Diversification_overlapRoute2(int**(&VEHICLE), int K, float *x, float *y);
void flagpartial_delete (int route, int more_delete, bool *(&flag_delete));

#endif
