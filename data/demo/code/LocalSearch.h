#ifndef LOCALSEARCH_H
#define LOCALSEARCH_H

extern int SIZE, CAPACITY, DISTANCE;
extern float **dist;
extern int *demand;
extern int* service_time; //global variable for service time
extern int* space_available;	//to check if the space available is enough for the new customer to be inserted
extern int* total_demand;
extern float* distance_available;
extern int* saiz;
extern float* route_cost;
extern float* distance_cost;
extern int I;//for running different data sets loop

extern float **CumDist;//record cumulative distance
extern float *gainvec;//so that after insert or delete, noneed to count the cost or distance again, just use this values since they have been calculated

void insert_one_cust (int route_number, int position, int **(&VEHICLE), int customer);
void delete_one_cust (int route_number, int position, int **(&VEHICLE));
void delete_two_cust (int route_number, int position, int **(&VEHICLE));
void insert_two_cust (int route_number, int position, int **(&VEHICLE), int customer, int customer2);
void swap_in_oneCust (int route_number, int position, int **(&VEHICLE), int customer);
void swap2_1_cust (int route_number, int position, int **(&VEHICLE), int customer); //2 cust out, 1 cust in
void swap1_2_cust (int route_number, int position, int **(&VEHICLE), int customer1, int customer2); //1 cust out, 2 cust in
void swap_in_twoCust (int route_number, int position, int **(&VEHICLE), int customer1, int customer2);
void reverseASegment (int route_number, int position, int L1, int **(&VEHICLE));
void cross_insert_fromshaking (int L1, int L2, int r1, int r2, int r1p, int r2p, int **(&VEHICLE), int reverseType);
void insert_1_0_sameRoute (int route, int old_pos, int new_pos, int **(&VEHICLE), int customer);//only used by fullPerturb;

#endif