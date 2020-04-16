// Start - add an empty route - copy vehicle[][], saiz[] from LOCAL_BEST_ROUTE[][] - recalculate best_gain() {vehic_depotCust[][] is copied from vehicle[][], find cost_of_removing[], removing_route[], removing position[]} - 
// module: shake_1_0() {the shake function update vehicle[][] and all global variables} - record route change from shake - copy vehic_depotCust[][] from vehicle[][], update saiz[], update cost_of_removing[], removing_route[], removing position[], distance _available[] for shaked routes
// do while (max_gain>0) {reoptimize_1_0(), ..., find best_gain, if (gain>0) insert_1_0(),... }
#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include "BestImprovement.h"
#include "VNS.h"
#include "LocalSearch.h"
#include "DiversificationDijkstra.h"
#include "DijkstraRefinement.h"
#include "InverseMethod.h"
#include "RecordBestSol.h"
#include "DiversificationLNS.h"
#include "SectorConflict.h"
#include "Guided_shake.h"

#define INFEASIBLE 0.06

using namespace std;

void VNS(int *demand, double *x, double *y, double **dist, double *(&freq)) 
{
	ofstream violation("Infeasible_" + std::to_string( I ) + ".txt");
	ofstream timefile("16.Time_" + std::to_string( I ) + ".txt", ios::app);
	ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
	ofstream localbest_sol("30.LOCAL_BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
	ofstream recordFreq("27.Frequency_of_module_" + std::to_string( I ) + ".txt");
	ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);

	double start_s = clock();// the code you wish to time goes here

	//********************************************* STEP 1: INITIALIZATION *********************************************//
	int max_neigbor = 6; //i have 6 neighbourhoods
	int maxLocalSearch = 8;
	int NbDiv = 0;
	int maxDiv = 2;
	//int maxDiv = max(30, GLOBAL_NO_ROUTE/2);
	int Ndel_min = 1; //min number of iteration for each route //to control the intensity of diversification

	//================ Frequency for 8 LS =====================// record the overall frequency for 8 LS, later in multi-level got 9 LS, the last level, 2-opt no frequency recorded, perform at evry loop
	double *gain = new double[SIZE]; //can be many depends on how many positive gain
	int *modul = new int[SIZE]; //can be many depends on how many positive gain
	for (int i = 0; i < SIZE; i++)//initialize
	{
		freq[i] = 0;
		gain[i] = 0;
		modul[i] = 0;
	}
	//============= Copy Local_BEST from GLOBAL_BEST ===========//
	for (int i = 0; i < SIZE; i++)
	{
		for (int j = 0; j < SIZE; j++)
		{
			LOCAL_BEST_ROUTE[i][j] = -1; //reinitialize
		}
		LOCAL_SAIZ[i] = 0;
		LOCAL_capa[i] = 0;
	}
	
	for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	{
		for (int j = 0; j <= GLOBAL_SAIZ[i]+3; j++)
		{
			LOCAL_BEST_ROUTE[i][j] = GLOBAL_BEST_ROUTE[i][j];
		}
		LOCAL_capa[i] = GLOBAL_capa[i];
		LOCAL_SAIZ[i] = GLOBAL_SAIZ[i];
	}
	LOCAL_BEST_COST = GLOBAL_BEST_COST;
	LOCAL_NO_ROUTE = GLOBAL_NO_ROUTE;
	
	while (NbDiv <= maxDiv) //#############################################################################################################################
	{
	//restartAfterDiver:
		double violateThreshold = INFEASIBLE; //reinitialize to original value after diversification
		int found_GLOBEST_status = 0;//must put before infeasibility check, otherwise found_GLOBEST_status is not recognise after goto violateDiverstart
		localbest_sol << "Start of Best improvement NbDiv= " <<NbDiv <<endl;

		//********************************************* STEP 2: ADD EMPTY ROUTE *********************************************//
		int no_empty_route=0;
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			if (LOCAL_SAIZ[i] == 0)
				no_empty_route++;
		}
		if (no_empty_route == 1)
		{
			goto proceed;
		}
		else if (no_empty_route == 0)
		{
			//************************************add one empty route
			LOCAL_BEST_ROUTE[LOCAL_NO_ROUTE][0] = LOCAL_NO_ROUTE;
			LOCAL_BEST_ROUTE[LOCAL_NO_ROUTE][1] = 0.0; //cost
			LOCAL_BEST_ROUTE[LOCAL_NO_ROUTE][2] = 0; //number of customer
			LOCAL_BEST_ROUTE[LOCAL_NO_ROUTE][3] = -1;
			//saiz[LOCAL_NO_ROUTE] = 0; //added on 9 MAC 2015
			//total_demand[LOCAL_NO_ROUTE] = 0;
			//space_available[LOCAL_NO_ROUTE] = CAPACITY*(1+violateThreshold);
			//distance_available[LOCAL_NO_ROUTE] = DISTANCE*(1+violateThreshold);
			//space_available[LOCAL_NO_ROUTE] = CAPACITY;
			//distance_available[LOCAL_NO_ROUTE] = DISTANCE;
			LOCAL_capa[LOCAL_NO_ROUTE] = 0;
			LOCAL_SAIZ[LOCAL_NO_ROUTE] = 0;
			LOCAL_NO_ROUTE++;
			//************************************end of add one empty route
		}
	
		else if (no_empty_route > 1) //if more than 1 empty route, do not copy the empty route and add one at the end
		{
			int k=0; //for new route index
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				if (LOCAL_SAIZ[i] == 0)
				{
					continue;
				}
				LOCAL_BEST_ROUTE[k][0] = k;
				for (int j = 1; j <= LOCAL_SAIZ[i]+3; j++)
				{
					LOCAL_BEST_ROUTE[k][j] = LOCAL_BEST_ROUTE[i][j];
			
				}

				for (int m = LOCAL_SAIZ[i]+4; m < SIZE; m++)
				{
					LOCAL_BEST_ROUTE[k][m] = -1; //reinitialize the rest
				}
				LOCAL_SAIZ[k] = LOCAL_SAIZ[i];
				LOCAL_capa[k] = LOCAL_capa[i];
				k++;
			}
			LOCAL_NO_ROUTE = k;
			//************************************add one empty route
			LOCAL_BEST_ROUTE[LOCAL_NO_ROUTE][0] = LOCAL_NO_ROUTE;
			LOCAL_BEST_ROUTE[LOCAL_NO_ROUTE][1] = 0.0; //cost
			LOCAL_BEST_ROUTE[LOCAL_NO_ROUTE][2] = 0; //number of customer
			LOCAL_BEST_ROUTE[LOCAL_NO_ROUTE][3] = -1;
			//saiz[LOCAL_NO_ROUTE] = 0; //added on 9 MAC 2015
			//total_demand[LOCAL_NO_ROUTE] = 0;
			//space_available[LOCAL_NO_ROUTE] = CAPACITY*(1+violateThreshold);
			//distance_available[LOCAL_NO_ROUTE] = DISTANCE*(1+violateThreshold);
			//space_available[LOCAL_NO_ROUTE] = CAPACITY;
			//distance_available[LOCAL_NO_ROUTE] = DISTANCE;
			LOCAL_capa[LOCAL_NO_ROUTE] = 0;
			LOCAL_SAIZ[LOCAL_NO_ROUTE] = 0;
			LOCAL_NO_ROUTE++;
			//************************************end of add one empty route
			//reinitialize the remaining route
			for (int i = LOCAL_NO_ROUTE; i < SIZE; i++)
			{
				for (int j = 0; j < SIZE; j++)
				{
					LOCAL_BEST_ROUTE[i][j]=-1;
				}
			}
		}
	

	proceed:
		cout<<"LOCAL_NO_ROUTE= "<< LOCAL_NO_ROUTE<<endl;

		double **vehicle = new double* [SIZE]; //declare more
		double **vehicle_c = new double* [SIZE]; 
		int **vehic_depotCust = new int* [SIZE];

		for (int i = 0; i < SIZE; i++)
		{
			vehicle[i] = new double[SIZE]; //same as best routes, for passing function to 
			vehicle_c[i] = new double[SIZE]; //vehicle_c only contains customers!!!, no -1 , for module selection
			vehic_depotCust[i] = new int[SIZE]; //vehic_depotCust contains only customer with depot at front and back, for reoptimize
		}
		int no_routes = LOCAL_NO_ROUTE;
		bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0
		int num_attributes = 9;

		//++++++++++++++++++++++++++++++++++++++++++++++++++DATA STRUCTURE++++++++++++++++++++++++++++++++++++++++++++++++++++++//
		double *cost_of_removing = new double[SIZE + 1];
		int *removing_route = new int[SIZE];
		int *removing_position = new int[SIZE];

		//========================================2-D matrices=============================================//
		double **gain_matrix_1_0 = new double*[no_routes];
		double **same_r_info = new double*[no_routes]; // to record the best improvement within the same route
		for (int i = 0; i < no_routes; ++i)
		{
			gain_matrix_1_0[i] = new double[no_routes];
			same_r_info[i] = new double[num_attributes];
		}

		int s = (((no_routes*no_routes) - no_routes) / 2) + 1;
		double** info_1_0 = new double*[s];
		for (int i = 0; i < s; ++i)
		{
			info_1_0[i] = new double[num_attributes];
		}
		//========================================Initialize Gain matrix ============================================//
		//diagonal of gain matrix is -1
		for (int i = 0; i < no_routes; i++)
		{
			gain_matrix_1_0[i][i] = -1;
		}

		int i = ((no_routes*no_routes) - no_routes) / 2;
		for (int u = (no_routes - 1); u > 0; u--)
		{
			for (int v = u - 1; v >= 0; v--)
			{
				gain_matrix_1_0[u][v] = i;
				i--;
			}
		}
		int Neighbour_k = 1; //Neighbour_k 1 = (1-0), Neighbour_k 2 = (1-1), Neighbour_k 3 = (2-1)
	
		int total_time = 0;
		double maxCPUtime = 10000/(CLOCKS_PER_SEC);
		cout << "maxCPUtime= " <<maxCPUtime<<endl;
	
		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ k+1 neighborhood starts here @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
	while (Neighbour_k <= max_neigbor)	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	{
		//module://after local search if no improvement, Neighbour_k++ goto module //this is k+1 in the algorithm (move to next neighborhood)
			if (violateThreshold < 0.01) //if smaller than 1%, make it zero
				violateThreshold = 0;
			no_routes = LOCAL_NO_ROUTE;
			for (int i = 0; i < SIZE; i++)
			{
				for (int j = 0; j < SIZE; j++)
				{
					vehicle[i][j] = -1; //reinitialize
				}
			}

			//vehicle copied from BEST_ROUTE to be passed to shake()
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				int j=0; 
				saiz[i] = LOCAL_BEST_ROUTE[i][2];

				do
				{
					vehicle[i][j] = LOCAL_BEST_ROUTE[i][j];
					j++;
				} while (LOCAL_BEST_ROUTE[i][j]!= -1);
				vehicle[i][j] = -1;
				saiz[i] = LOCAL_SAIZ[i] = LOCAL_BEST_ROUTE[i][2];//initially both are the same

			}
	
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				total_demand[i] = 0;//reinitialize
				for (int j = 3; j < LOCAL_SAIZ[i]+3; j++)
				{
					total_demand[i] = total_demand[i] + demand[(int)LOCAL_BEST_ROUTE[i][j]];
				}
				LOCAL_capa[i] = total_demand[i]; //capa[] record the BEST_ROUTE demand
			}
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				route_cost[i] = LOCAL_BEST_ROUTE[i][1];
				saiz[i] = LOCAL_BEST_ROUTE[i][2];
				space_available[LOCAL_NO_ROUTE] = (CAPACITY*(1+violateThreshold)) - (total_demand[i]);
				distance_available[LOCAL_NO_ROUTE] = (DISTANCE*(1+violateThreshold)) - (route_cost[i]);
				//space_available[i] = CAPACITY - (total_demand[i]);
				//distance_available[i] = DISTANCE - (route_cost[i]);
			}
			//========================= To calculate gain_matrix[] and info-matrix[][] ===============================//

			//recalculate_best_gain(dist, demand, cost_of_removing, removing_route, removing_position, info_1_0, same_r_info, vehic_depotCust, gain_matrix_1_0, vehicle);
			//cout<<endl<<"gain_matrix after recalculate_best_gain, beofe shake" <<endl; 
			//for (int i = 0; i < no_routes; i++)
			//{
			//	for (int j = 0; j < no_routes; j++)
			//	{
			//		cout<<gain_matrix_1_0[i][j] << ' ';
			//	}
			//	cout<<endl;
			//}
			//cout<<endl;


			//**************all insertion and deletion starts from 1 to saiz because the vehic_depotCust starts from depot, end with depot**************//
			//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//

			custRGravity = new double[LOCAL_NO_ROUTE];
			sorted_custRGravity = new double*[LOCAL_NO_ROUTE];
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				custRGravity[i] = 0; //initialize
				sorted_custRGravity[i] = new double[5]; //[0]sorted distance, [1]sorted route index, [2]1/distance, [3]d/sum(d), [4]cumulative 
			}
	
		reshake://if no feasible shake, Neighbour_k++ and goto reshake

			for (int i = 0; i < LOCAL_NO_ROUTE; i++)//added 22Apr15
			{
				total_demand[i] = 0;//reinitialize
				for (int j = 0; j <= LOCAL_SAIZ[i]+3; j++)
				{
					vehicle[i][j] = LOCAL_BEST_ROUTE[i][j];
					if((j>=3) && (j<=(LOCAL_SAIZ[i]+2)))
					{
						total_demand[i] = total_demand[i]+demand[(int)vehicle[i][j]];
					}

				}
				saiz[i] = LOCAL_BEST_ROUTE[i][2]; 
				route_cost[i] = LOCAL_BEST_ROUTE[i][1]; 
				space_available[LOCAL_NO_ROUTE] = (CAPACITY*(1+violateThreshold)) - (total_demand[i]);
				distance_available[LOCAL_NO_ROUTE] = (DISTANCE*(1+violateThreshold)) - (route_cost[i]);
				//space_available[i] = CAPACITY - total_demand[i];
				//distance_available[i] = DISTANCE - route_cost[i];
			}

			int shake_status1 = 1; //initialize flag_status=0, 0 is not ok, 1 is ok
			int shake_status2 = 1;
			if (Neighbour_k == 1) //(1-0)
			{
				shake_status1 = shake_1_0(vehicle, dist, demand, x, y); //the one going to shake is always the current best solution
				if (shake_status1 == 0) //if no feasible shake can be performed
				{
					Neighbour_k++;
					goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
				}
			}
		
			else if (Neighbour_k == 2) //1-1
			{	
				shake_status1 = shake_1_1(vehicle, dist, demand, x, y);
				if (shake_status1 == 0)
				{
					Neighbour_k++;
					goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
				}
		
			}
			else if (Neighbour_k == 3) //(2-0) - a pair from route A go to route B
			{			
				shake_status1 = shake_2_0_twoR(vehicle, dist, demand, x, y);
				shake_status2 = shake_1_0(vehicle, dist, demand, x, y);
				if ((shake_status1 == 0) && (shake_status2 == 0))
				{
					Neighbour_k++;
					goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
				}
			}
	
			else if (Neighbour_k == 4) //(2-0) - a pair from route A go to route B and route C
			{			
				shake_status1 = shake_2_0_threeR(vehicle, dist, demand, x, y);
				if (shake_status1 == 0)
				{
					Neighbour_k++;
					goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
				}
			}

			else if (Neighbour_k == 5) //(2-1) - a pair from route A swap with one customer from route B
			{			
				shake_status1 = shake_2_1_twoR(vehicle, dist, demand, x, y);
				if (shake_status1 == 0)
				{
					Neighbour_k++;
					goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
				}
			}

			else if (Neighbour_k == 6) //(2-1) - first pair of cust must from route A to B, another cust from route A can go to route C
			{
				shake_status1 = shake_2_1_threeR(vehicle, dist, demand, x, y);
				shake_status2 = shake_1_0(vehicle, dist, demand, x, y);

				if ((shake_status1 == 0) && (shake_status2 == 0))
				{
					Neighbour_k++;
					goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
					//goto endVNS;
				}
					
			}

			//else if (Neighbour_k == 7) //cannot make_giant_tour here becaus number of route will change (become more or less)
			//{
			//	double cost = make_giant_tour(dist, demand, vehicle, no_routes); //pass in vehicle and no_routes to be updated in Dijkstra, they will be initialize in the Dijkstra function
			//	if (no_routes > LOCAL_NO_ROUTE)

			//	//make_giant_tour has updated saiz[], route_cost[], total_demand[]
			//	//vehicle[][] after giant_tour has no empty route!!!!!!!!! //I have already added in function make_giant_tour()
			//}

			//vehic_depotCust[][] copy from vehicle[][] after shake
			int r1 = route_change[0];
			int r2 = route_change[1];
			int r3 = route_change[2];

			cout<<"r1= " <<r1 << ' ' << "r2= " <<r2 << ' ' << "r3= " <<r3 << endl;
			cout<<"After shake " <<Neighbour_k <<endl;
			//vehic_depotCust copied from vehicle to be passed to reoptimize() later
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				int j=0; 
				saiz[i] = vehicle[i][2];
				vehic_depotCust[i][0] = SIZE; //vehic_depotCust contains only customer with depot at front and back, for reoptimize
				int v=1; //for vehic_depotCust
				do
				{
					if (j>=3)
					{
						vehic_depotCust[i][v] = vehicle[i][j];
						v++;
					}
					j++;
				} while (vehicle[i][j]!= -1);
	
				vehic_depotCust[i][v] = SIZE;
			}


			bool same_route;
			int set = 0 ;
	
			//////=================================================update cost of removing=====================================//////////////////////
			update_all_cost_of_removing (dist, vehic_depotCust, cost_of_removing, no_routes);
			//int r_change =0;
			//if ( (Neighbour_k == 6) || (Neighbour_k == 4) )
			//	r_change = 3; //3 routes changed for 2-1 threeR
			//else if (Neighbour_k == 7) //dijkstra split route
			//{
			//	update_all_cost_of_removing (dist, vehic_depotCust, cost_of_removing, no_routes);
			//	goto skip_partialUpdate;
			//}
			//else
			//	r_change = 2; //other modules have 2 routes change

			//for (int g = 0; g < r_change; g++) //find cost of removing for each customer and put in array
			//{
			//	int f=-1;
			//	if (g == 0)
			//		f = r1;
			//	else if (g==1)
			//		f = r2;
			//	else if (g==2)
			//		f = r3;

			//	distance_available[f] = DISTANCE - (route_cost[f]); //update available distance
			//	int siz = vehicle[f][2];
			//	for (int h = 1; h <= siz; h++) //first customer start from element [1]
			//	{
			//		cost_of_removing[vehic_depotCust[f][h]] = dist[vehic_depotCust[f][h]][vehic_depotCust[f][h - 1]] + dist[vehic_depotCust[f][h]][vehic_depotCust[f][h + 1]] + service_time[vehic_depotCust[f][h]] - dist[vehic_depotCust[f][h - 1]][vehic_depotCust[f][h + 1]];//(old + service_time – new)
			//		removing_route[vehic_depotCust[f][h]] = f;
			//		removing_position[vehic_depotCust[f][h]] = h;
			//	}
			//}
		skip_partialUpdate:

			recalculate_best_gain(dist, demand, cost_of_removing, removing_route, removing_position, info_1_0, same_r_info, vehic_depotCust, gain_matrix_1_0, vehicle);//CHANGED ON 7MAC2015
			double max_gain = INT_MIN;	

			do
			{
				int num_positiveGain=0;
				for (int i = 0; i < SIZE; i++)//reinitialize
				{
					gain[i] = 0;
					modul[i] = 0;
				}


				set++;
		
				reoptimize_1_0(dist, demand, cost_of_removing, info_1_0, same_r_info, vehic_depotCust, gain_matrix_1_0);
				reoptimize_1_1(dist, demand, cost_of_removing, info_1_0, same_r_info, vehic_depotCust, gain_matrix_1_0);
				reoptimize_2_1(dist, demand, cost_of_removing, info_1_0, same_r_info, vehic_depotCust, gain_matrix_1_0);
				reoptimize_2_0(dist, demand, cost_of_removing, info_1_0, same_r_info, vehic_depotCust, gain_matrix_1_0);

				//if((Neighbour_k == 4) || (Neighbour_k == max_neigbor)) //reopt for one more route because module 4 and module six shake 3 routes
				//{
				//	reopt_singleRoute (r3, dist, demand, cost_of_removing, info_1_0, same_r_info, vehic_depotCust, gain_matrix_1_0);
				//}

				//for (int i = 0; i < no_routes; i++)
				//{
				//	for (int j = 0; j < no_routes; j++)
				//	{
				//		cout<<gain_matrix_1_0[i][j] << ' ' ;
				//	}
				//	cout<<endl;
				//}
				int k=0;//for gain[]
				//=============== Record the gain for each module =============================//
				for (int i = 0; i < no_routes; i++)
				{
					for (int m = i ; m < no_routes; m++)//////////////////////////////////////////////////////////////////////////////////////////////
					{
						if (gain_matrix_1_0[i][m] > 0.01)
						{
							gain[k] = gain_matrix_1_0[i][m];
							if (i!=m)
							{
								number = gain_matrix_1_0[m][i];
								modul[k] = ((info_1_0[number][8])*2);//m1->m2, m2->m4, m3->m6, m4->m8
						

							}
							else if (i==m)
							{
								number=i;
								if (same_r_info[number][8] == 1)//1 is (1-0) same route, still the same
									modul[k] = same_r_info[number][8];
								else if (same_r_info[number][8] != 1)
									modul[k] = ((same_r_info[number][8])*2) - 1; //m1->m1, m2->m3, m3->m5, m4->m7
							}
							k++;
						}
					}
				}
				num_positiveGain = k;
				//s = ((no_routes*no_routes) - no_routes) / 2;
				//int m=0;
				//for (int i = 1; i <= s; i++) //info[][] starts from 1 int s = (((no_routes*no_routes) - no_routes) / 2) + 1;
				//{
				//	if(info_1_0[i][1] > 0)//if gain>0
				//	{
				//		m = info_1_0[i][8]; //the module number
				//		if (m==1)
				//			gain[1] = info_1_0[i][1]; //record gain of 1_0 diff route
				//		else if (m==2)
				//			gain[3] = info_1_0[i][1]; //record gain of 1_1 diff route
				//		else if (m==3)
				//			gain[5] = info_1_0[i][1]; //record gain of 2_1 diff route
				//	}
				//}

				//for (int i = 0; i < no_routes; i++)
				//{
				//	if(same_r_info[i][1] > 0)//if gain>0
				//	{
				//		m = same_r_info[i][8]; //the module number
				//		if (m==1)
				//			gain[2] = same_r_info[i][1]; //record gain of 1_0 same route
				//		else if (m==2)
				//			gain[4] = same_r_info[i][1]; //record gain of 1_1 same route
				//		else if (m==3)
				//			gain[6] = same_r_info[i][1]; //record gain of 2_1 same route
				//	}
				//}


				//============================================to find the best gain out of all modules=============================================//
				max_gain = INT_MIN;
				same_route = false; // reinitialize
				for (int i = 0; i < no_routes; i++)
				{
					for (int m = i ; m < no_routes; m++)//////////////////////////////////////////////////////////////////////////////////////////////
					{
						if (gain_matrix_1_0[i][m]-max_gain > 0.01) //gain_matrix_1_0[i][m] > max_gain
						{
							if (i!=m)
							{
								max_gain = gain_matrix_1_0[i][m];
								number = gain_matrix_1_0[m][i];
								module = info_1_0[number][8];
								route_change[0] = info_1_0[number][2]; //route (from)
								route_change[1] = info_1_0[number][3]; //route (to)
								//sPtr->r1 = info_1_0[(int)gain_matrix_1_0[m][i]][2]; //route (from)
								//sPtr->r1 = info_1_0[(int)gain_matrix_1_0[m][i]][3]; //route (to)

								same_route = false;
							}

							else if (i==m) //same route, read from same_r_info[][]
							{
								max_gain = gain_matrix_1_0[i][m];
								number = i;
								module = same_r_info[number][8];
								route_change[0] = same_r_info[number][2]; //route (from)
								route_change[1] = same_r_info[number][3]; //route (to)
								//sPtr->r1 = same_r_info[number][2]; //route (from)
								//sPtr->r1 = same_r_info[number][3]; //route (to)
								same_route = true;
							}
						}
					}
				}
		
				for (int i = 0; i < num_positiveGain; i++) //starts from 1, be careful of heap error, must declare freq[max_neigbor+1]
				{
					int m = modul[i]; //from which module
					freq[m] = freq[m] + (gain[i]/max_gain);
				}
		
				for (int i = 0; i < num_positiveGain; i++)
				{
					recordFreq<<gain[i] << ' '<<' ' <<modul[i]<<endl;
				}


				//=================================insert to corresponding module based on the highest gain=====================================//
				if (max_gain > 0)
				{
					cout<< "max_gain = " <<max_gain <<endl;
					cout<<"========== max gain > 0===========" << ' ' <<"module = " << module<<' '<<"same_route= "<<same_route<<endl;
					if (module == 1)
					{
						insert_1_0(dist, demand, cost_of_removing, removing_route, removing_position, info_1_0, same_r_info, vehic_depotCust, same_route);
					}

					else if (module == 2)
					{
						insert_1_1(dist, demand, cost_of_removing, removing_route, removing_position, info_1_0, same_r_info, vehic_depotCust, same_route);
					}

					else if (module == 3)
					{
						insert_2_1(dist, demand, cost_of_removing, removing_route, removing_position, info_1_0, same_r_info, vehic_depotCust, same_route);
					}
					else if (module == 4)
					{
						insert_2_0(dist, demand, cost_of_removing, removing_route, removing_position, info_1_0, same_r_info, vehic_depotCust, same_route);
					}

				}//end if 


			} while (max_gain > 0.05);
	
		endVNS:

			//======================================================Display route========================================================================
			cout << "==================Routes (in VNS)===================================== " << endl;
			double demd = 0.0, total_cost = 0.0;
			int total_cust = 0;
			bool dist_ok = true;
			for (int g = 0; g < no_routes; g++)
			{
				if (saiz[g] == 0)
				{
					route_cost[g] = 0;
					total_demand[g]=0;
				}
				demd = 0.0;
				cout << g << ' ';
				cout << route_cost[g] << ' ';
				cout << saiz[g] << ' ';
				total_cust = total_cust + saiz[g];
				for (int h = 0; h < saiz[g] + 2; h++)
				{
					cout << vehic_depotCust[g][h] << ' ';
					demd = demd + demand[vehic_depotCust[g][h]];

				}
	
				total_cost = total_cost + route_cost[g];
				cout << setw(3) << "d=" << demd << endl;

			}
			cout << "Total customers = " << total_cust << endl;
			cout << "Total cost = " << total_cost << endl;

			cout<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl;
			cout<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
			//if ((Rptr->t_cost < BEST_COST) && (Rptr->t_demand <= CAPACITY) && (Rptr->t_distance <= DISTANCE) )
			//copy to vehicle[][]
			for (int i = 0; i < SIZE; i++)
			{
				for (int j = 0; j < SIZE; j++)
				{
					vehicle[i][j] = -1; //reinitialize
				}
			}
			for (int i = 0; i < no_routes; i++)
			{
				vehicle[i][0] = i;
				vehicle[i][1] = route_cost[i];
				vehicle[i][2] = saiz[i];
				int k=1; //for vehic_depotCust
				for (int j = 3; j <= saiz[i]+2; j++)
				{
					vehicle[i][j] = vehic_depotCust[i][k];
					k++;
				}
				vehicle[i][saiz[i]+3] = -1;
			}
			//int found_GLOBEST_status = 0;//must put before infeasibility check, otherwise found_GLOBEST_status is not recognise after goto violateDiverstart
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ for infeasibility check @@@@@@@@@@@@@@@@@@@@@@@@//
			for (int i = 0; i < no_routes; i++)
			{
				if ((total_demand[i]>CAPACITY) || (route_cost[i]>DISTANCE))//for infeasibility check
				{
					violation << "In VNS best improvement after module "<< Neighbour_k <<endl;
					for (int j = 0; j <= saiz[i]+3; j++)
					{
						violation << vehicle[i][j]<<' ';
					}
					violation<<endl;
					violateThreshold = violateThreshold/2; //divide by half if the solution after LS still violate constraint

					Neighbour_k++;
					goto violateDiverstart; //if violation, skip all the process of compare LOCAL_BEST and GLOBAL_BEST, check if max diversification is reached
			
				}
			}
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ End of (for infeasibility check) @@@@@@@@@@@@@@@@@@@@@@@@//


			//PROBLEM: THE BEST SOLUTION IS ALREADY RECORDED IN FUNCTION INSERT, THAT'S WHY IT NEVER ENTER HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!
			if (((LOCAL_BEST_COST - total_cost) > 0.05) && (dist_ok == true) )//total_cost < LOCAL_BEST_COST
			//if (total_cost < BEST_COST)
			{
				LOCAL_BEST_COST = total_cost;
				cout << endl << "BEST COST IS " << LOCAL_BEST_COST << endl;
				for (int i = 0; i < SIZE; i++)
				{
					for (int j = 0; j < SIZE; j++)
					{
						LOCAL_BEST_ROUTE[i][j] = -1;  //reinitialize
					}

				}
		
				for (int i = 0; i < no_routes; i++)
				{
					int j=0;
					LOCAL_BEST_ROUTE[i][j] = i;//route index
					j++;
					LOCAL_BEST_ROUTE[i][j] = route_cost[i];//route cost
					j++;
					LOCAL_BEST_ROUTE[i][j] = saiz[i];//route size
					j++;
			
					for (int k = 1; k <= saiz[i]; k++)
					{
						LOCAL_BEST_ROUTE[i][j] = vehic_depotCust[i][k]; //vehic_depotCust start with depot, end with depot
						j++;
					}

					LOCAL_BEST_ROUTE[i][j] = -1; //because n starts from 1, n+1 is route size
					LOCAL_SAIZ[i] = saiz[i]; //copy temp saiz[] to best SAIZ[]
					LOCAL_capa[i] = total_demand[i];
				}
				if (no_routes != LOCAL_NO_ROUTE)
				{
					cout<<"ERROR no_routes != LOCAL_NO_ROUTE in VNS best improvement)"<<endl;
					exit(EXIT_FAILURE);
				}

				//=================================record localbest solution in txt=====================================//
				cout << "========================================================================== " << endl;
				//=================================display final solution=====================================//
				for (int i = 0; i < LOCAL_NO_ROUTE; i++)
				{
					for (int j = 0; j <= LOCAL_SAIZ[i]+3; j++)
					{
						localbest_sol << LOCAL_BEST_ROUTE[i][j] << ' ';
					}
					localbest_sol<<endl;
				}
				localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"in VNS best improvement" <<"after shake Neighbour_k "<<Neighbour_k << endl;	
				//localbest_sol.close();
				//======================= End of record localbest solution ===========================//

				Neighbour_k = 1;
		
			}
			
			double cost = 0.0; //to be passed to dijkstra_refinement
			//dijkstra_refinement always refine the LOCAL_BEST_SOL (incumbent best)
			//vehicle[][] is not important, it will be reinitialize in dijkstra_refinement, and passed out from there
			dijkstra_refinement(dist, demand, cost, vehicle); //already added no_routes same as LOCAL_NO_ROUTE in dijkstra_refinement
			cout<<"cost after dijkstra_refinement= "<<cost<<endl;
			if (LOCAL_BEST_COST - cost > 0.05)//cost < LOCAL_BEST_COST
			{
				//============= Record the best solution ====================//
				LOCAL_BEST_COST = cost;
				cout << endl << "LOCAL_BEST COST IS " << LOCAL_BEST_COST << endl;
				for (int i = 0; i < SIZE; i++)
				{
					for (int j = 0; j < SIZE; j++)
					{
						LOCAL_BEST_ROUTE[i][j] = -1;  //reinitialize
					}

				}

				for (int i = 0; i < LOCAL_NO_ROUTE; i++)
				{
					for (int j = 0; j <= saiz[i]+3; j++)
					{
						LOCAL_BEST_ROUTE[i][j] = vehicle[i][j];
					}
			
					LOCAL_SAIZ[i] = saiz[i]; //copy temp saiz[] to best SAIZ[]
					LOCAL_capa[i] = total_demand[i]; // capa[] record the BEST_ROUTE demand
				}
				//int empty_route = original_route - no_routes; //no_routes is after dijkstra_partition
		

					
				//=================================record localbest solution in txt=====================================//
				cout << "========================================================================== " << endl;

				//=================================display final solution=====================================//
				for (int i = 0; i < LOCAL_NO_ROUTE; i++)
				{
					for (int j = 0; j <= LOCAL_SAIZ[i]+3; j++)
					{
						localbest_sol << LOCAL_BEST_ROUTE[i][j] << ' ';
					}
					localbest_sol <<endl;
				}
				localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
				//localbest_sol.close();
				//======================= End localrecord best solution ===========================//

				Neighbour_k = 1;
			}
		} //end while (Neighbour_k <= max_neighbor) @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

		if ((GLOBAL_BEST_COST - LOCAL_BEST_COST) > 0.05)//LOCAL_BEST_COST < GLOBAL_BEST_COST
		{
			GLOBAL_BEST_COST = LOCAL_BEST_COST;
			cout << endl << "GLOBAL_BEST COST IS " <<GLOBAL_BEST_COST << endl;
			found_GLOBEST_status = 1; //for diversificaton use

			for (int i = 0; i < SIZE; i++)
			{
				for (int j = 0; j < SIZE; j++)
				{
					GLOBAL_BEST_ROUTE[i][j] = -1;  //reinitialize
				}
				GLOBAL_SAIZ[i] = 0;
				GLOBAL_capa[i] = 0;
			}

			int r = 0; //for GLOBAL_NO_ROUTE becuase do not consider empty route
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				if (LOCAL_SAIZ[i] == 0)
					continue;

				GLOBAL_BEST_ROUTE[r][0] = r;
				for (int j = 1; j <= LOCAL_SAIZ[i]+3; j++)
				{
					GLOBAL_BEST_ROUTE[r][j] = LOCAL_BEST_ROUTE[i][j];
					if ((j>=3) && (j < LOCAL_SAIZ[i]))
					{
						GLOBAL_capa[r] = GLOBAL_capa[r] + demand[(int)LOCAL_BEST_ROUTE[i][j]];
					}
				}
				GLOBAL_SAIZ[r] = GLOBAL_BEST_ROUTE[r][2];
				r++;
			}
			GLOBAL_NO_ROUTE = r;

			//=================================record the best solution=====================================//
			cout << "========================================================================== " << endl;

			ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
			//=================================display final solution=====================================//
			for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
			{
				int k = 0;
				do
				{
					best_sol << GLOBAL_BEST_ROUTE[i][k] << ' ';
					k++;

				} while (GLOBAL_BEST_ROUTE[i][k] != -1);

				best_sol << "-1" << endl;

			}

			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
			best_sol << "From VNS best improvement after Neighbour_k " <<Neighbour_k<<  endl;
			//best_sol.close();
		}

	violateDiverstart: //if violation, skip all the process of compare LOCAL_BEST and GLOBAL_BEST, check if max diversification is reached

		NbDiv++;
		double cost = 0;
		//vehicle[][] to be passed in to make_giant_tour() is not important, make_giant_tour always consider LOCAL_BEST_SOL (incumbent best solution)
		if (found_GLOBEST_status == 1)
		{
			cost = make_giant_tour(dist, demand, vehicle, no_routes); //pass in vehicle and no_routes to be updated in Dijkstra, they will be initialize in the Dijkstra function
			Ndel_min = 1;
		}
		else if (found_GLOBEST_status == 0) //if not found global_best
		{
			cout<<"Entering Diversify LNS/conflict sector in VNS best improvement"<<endl;
			int div=(rand() % 2); 
			if (div == 0)
				cost = Diversify_LNS(dist, demand, vehicle, no_routes, Ndel_min);
			else
				cost = Diversification_conflict_sector(dist, demand, vehicle, no_routes, Ndel_min);
			Ndel_min++;
		}
		//AFter diversification, record in LOCAL_BEST_ROUTE, when it enters restartAfterDiver, it will restart with add empty route to LOCAL_BEST_ROUTE again
		for (int i = 0; i < SIZE; i++)
		{
			for (int j = 0; j < SIZE; j++)
			{
				LOCAL_BEST_ROUTE[i][j] = -1;
			}
		}
		int k=0;//for LOCAL_BEST_ROUTE[][] route index
		for (int i = 0; i < no_routes; i++)
		{
			if (saiz[i] == 0)
				continue;
			LOCAL_BEST_ROUTE[k][0] = k;
			for (int j = 1; j <= saiz[i]+3; j++)
			{
				LOCAL_BEST_ROUTE[k][j] = vehicle[i][j];
			}
			LOCAL_SAIZ[k] = saiz[i];
			LOCAL_capa[k] = total_demand[i];
			k++;
		}
		LOCAL_BEST_COST = cost;
		LOCAL_NO_ROUTE = k;

		//if the cost better than GLOBAL_BEST_COST 
		if (GLOBAL_BEST_COST-cost > 0.05)//cost < GLOBAL_BEST_COST
		{
			for (int i = 0; i < SIZE; i++)
			{
				for (int j = 0; j < SIZE; j++)
				{
					GLOBAL_BEST_ROUTE[i][j] = -1;
				}
			}
			int m=0;//for GLOBAL_BEST_ROUTE[][] route index
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i] == 0)
					continue;
				GLOBAL_BEST_ROUTE[m][0] = m;
				best_sol << m<<' ';
				for (int j = 1; j <= saiz[i]+3; j++)
				{
					GLOBAL_BEST_ROUTE[m][j] = vehicle[i][j];
					best_sol << GLOBAL_BEST_ROUTE[m][j] << ' ';
				}
				best_sol <<endl;
				GLOBAL_SAIZ[m] = saiz[i];
				GLOBAL_capa[m] = total_demand[i];
				m++;
			}
			GLOBAL_BEST_COST = cost;
			GLOBAL_NO_ROUTE = m;

			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
			best_sol << "From VNS best improvement after Diversify_LNS or Diversify giant tour or Diversify sector conflict"<<  endl;

		}
		
	}//end of while (NbDiv < maxDiv)//#############################################################################################################################


last:
	adjustFreq(freq, maxLocalSearch); //if probability = 0, add 0.05 so that it will have a small chance to be selected
	recordFreq<<"=========== Frequency of gain in VNS ============"<<endl;
	for (int i = 1; i <= maxLocalSearch; i++) //freq[i] starts from 1
	{
		recordFreq<<"Freq["<<i<<"]= "<<freq[i]<<' '<<' ';
	}
	recordFreq<<endl;
	recordFreq.close();
	timefile << "VNS best improvement "<<maxDiv <<" diversification time: " << (clock() - start_s) / double(CLOCKS_PER_SEC) << " second"<<endl;
	//write_2opt.close();

	for (int i = 0; i < SIZE; i++)
	{
		delete[] vehicle[i];
		delete[] vehicle_c[i];
		delete[] vehic_depotCust[i];
	}
	delete[] vehicle;
	delete[] vehicle_c;
	delete[] vehic_depotCust;
	delete[] gain;
}


//all shake() functions will shake the current best solution, in VNS, if no improvement then go to next shake of current best solution
int shake_1_0(double **(&routes), double **dist, int *demand, double *x, double *y)//routes will copy from LOCAL_BEST_ROUTE in this function, only shake the best route
{
	ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);

	int **vehic_depotCust = new int* [LOCAL_NO_ROUTE];
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		vehic_depotCust[i] = new int[SIZE]; //vehic_depotCust contains only customer with depot at front and back, for reoptimize
	}

	int no_routes = LOCAL_NO_ROUTE;

	route_CGravity = new double*[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		route_CGravity[i] = new double[2]; //2 columns consists of x and y
	}

	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		int j=0; 
		int m=1; //for vehic_depotCust
		//saiz[i] = LOCAL_BEST_ROUTE[i][2];
		saiz[i] = routes[i][2]; //added 22Apr15
		do
		{
			//routes[i][j] = LOCAL_BEST_ROUTE[i][j];
			vehic_depotCust[i][0] = SIZE;
			if (j>=3)
			{
				vehic_depotCust[i][m] = routes[i][j];//added 22Apr15
				//vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
				m++;
			}
			j++;
		//} while (LOCAL_BEST_ROUTE[i][j]!= -1);
		} while (routes[i][j]!= -1);//added 22Apr15
		//routes[i][j] = -1;
		vehic_depotCust[i][saiz[i]+1] = SIZE;

	}

	int rand_r1 = -1, rand_r2 = -1, rand_r1_p = -1, rand_r2_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1;
	//double average_capa_available = 0;
	//double average_demand = 0;
	int saiz_r1 = 0, saiz_r2 = 0;
	int demand_c1, demand_c2;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1;
	double cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0;
	//********************************************************  Neighbour_k 1 (1-0)  ******************************************************************************//
	
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //initialize to 1 initially
	}	
	int no_flag=0;
	m1_regenerate:
		int r2_found =0 ;
		srand ( time(NULL) ); //seed it
		rand_r1 = (rand() % LOCAL_NO_ROUTE); // for deletion cannot delete from empty route, NO_BEST_ROUTE = 3 , [0][1][2], can only delete from [0][1]
		saiz_r1 = (int)routes[rand_r1][2];

		if(saiz_r1 == 0)//have to test this before finding the position to avoid division by zero error
			goto m1_regenerate;

		rand_r1_p = (rand() % saiz_r1)+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE
		cust1_r1 = vehic_depotCust[rand_r1][rand_r1_p];
		before_cust1 = vehic_depotCust[rand_r1][rand_r1_p-1];
		after_cust1 = vehic_depotCust[rand_r1][rand_r1_p+1];
		demand_c1 = demand[cust1_r1];

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from false flag
			goto m1_regenerate;
	
		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		//{
		//	if ((saiz[rand_r1] == 1) && (saiz[i] == 0)) //if original 1 customer, cannot insert into empty route, it will be the same //THIS IS A REAL TRICK!!!!!!!!!!!!!!!!!!!
		//		continue;
		//	if ((i == rand_r1) || (demand_c1 > space_available[i]))//if the same route or exceed capacity, skip
		//		continue;
		//	if (saiz[i] == 0)
		//	{
		//		rand_r2 = i;
		//		rand_r2_p = 1;
		//		r2_found = 1;
		//		goto found_r2_m1;
		//	}
		//	for (int j = 1; j <= saiz[i]+1; j++) //insertion can be made one position more than saiz
		//	{
		//		before_cust2 = vehic_depotCust[i][j-1];
		//		after_cust2 = vehic_depotCust[i][j];//current position
		//		rand_r2 = i;

		//		gain1 = (dist[before_cust1][after_cust1]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][after_cust1] + service_time[cust1_r1]); //new-old 
		//		gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) - (dist[before_cust2][after_cust2]); //new-old

		//		if ((demand_c1 <= space_available[rand_r2]) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
		//		{
		//			rand_r2 = i;
		//			rand_r2_p = j;
		//			r2_found = 1;
		//			goto found_r2_m1;
		//		}
		//		else
		//			continue;

		//	}
		//}
		//================================ End (Change here for guided shake) ================================///
		//++++++++++++++++++++++++++++++++ Guided shake +++++++++++++++++++++++++++++++++++++++++++++++++++++//
		bool *RFlag = new bool[no_routes];
		for (int i = 0; i < no_routes; i++)
		{
			RFlag[i] = true; //initially all routes are true
		}
		calculate_centreGravity(routes, x, y, no_routes); //calculate centroid and put in route_CGravity[i][0] and route_CGravity[i][1]
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
		}
		CGravityfile <<endl;
		
		CGravityfile << "cust1_r1= "<<cust1_r1<<endl;
		
		findDist_from_CGravity (cust1_r1, x, y, dist, custRGravity, sorted_custRGravity, no_routes); //sort dist and route, compute 1/distance, compute d/sum(d), compute cumulative
		for (int i = 0; i < no_routes; i++)//[0]sorted distance, [1]route index, [2]1/distance, [3]d/sum(d), [4]cumulative
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		CGravityfile <<endl;
		

	refindR2:
		int route2 = -1;//initialize
		
		//******************************** find route based on probability ************************************//
		//double randNum = (double) rand() / double(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		for (int i = 0; i < no_routes; i++)
		{
			if (RFlag[(int)sorted_custRGravity[i][1]] == true) //if the route is true
			{
				route2 = sorted_custRGravity[i][1];
				goto goout;
			}
		}
		if (route2 == -1) //if not found route from the list
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			goto m1_regenerate;
		}
	goout:
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		//CGravityfile <<"randNum= "<<randNum<<' '<<' '<<"route2= "<<route2<<endl;
		CGravityfile <<"route2= "<<route2<<endl;
		CGravityfile << "cust is "<<cust1_r1<<' '<<"x[cust]= "<<x[cust1_r1]<<' '<<"y[cust]= "<<y[cust1_r1]<<endl;
		CGravityfile <<endl;

		if ((saiz[rand_r1] == 1) && (saiz[route2] == 0)) //if original 1 customer, cannot insert into empty route, it will be the same //THIS IS A REAL TRICK!!!!!!!!!!!!!!!!!!!
		{
			RFlag[route2] = false; //for findDist_from_CGravity_choose_nearest use
			goto refindR2;
		}
		if ((route2 == rand_r1) || (demand_c1 > space_available[route2]))//if the same route or exceed capacity, 
		{
			RFlag[route2] = false; //for findDist_from_CGravity_choose_nearest use
			goto refindR2;
		}
		if (saiz[route2] == 0)
		{
			rand_r2 = route2;
			rand_r2_p = 1;
			r2_found = 1;
			goto found_r2_m1;
		}
		for (int j = 1; j <= saiz[route2]+1; j++) //insertion can be made one position more than saiz
		{
			before_cust2 = vehic_depotCust[route2][j-1];
			after_cust2 = vehic_depotCust[route2][j];//current position
			rand_r2 = route2;

			gain1 = (dist[before_cust1][after_cust1]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][after_cust1] + service_time[cust1_r1]); //new-old 
			gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) - (dist[before_cust2][after_cust2]); //new-old

			if ((demand_c1 <= space_available[rand_r2]) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
			{
				rand_r2 = route2;
				rand_r2_p = j;
				r2_found = 1;
				goto found_r2_m1;
			}
			else
				continue;

		}	
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//

		if (r2_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			no_flag++;
			if (no_flag >= SIZE)
			{	
				cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER!" <<endl;
				for (int i = 0; i < SIZE; i++)
				{
					cout<<flag_module[i]<<' ';
				}
				return 0;
			}
			goto m1_regenerate;
		}


found_r2_m1:
		route_change[0] = rand_r1; //route (from)
		route_change[1] = rand_r2; //route (to)
		route_change[2] = -1; //reinitialize 

	//******************************************************** END OF Neighbour_k 1 (1-0)  ******************************************************************************//

	// erase the element from route 1

	int customer = cust1_r1; //must put before deletion, error this one can be -1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	rand_r1_p = rand_r1_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	rand_r2_p = rand_r2_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	if (customer < 0)
	{
		cout<< "customer to be deleted cannot be negative!!!!!!" <<endl;
		cout<< "r1= " <<rand_r1 <<endl;
		cout<< "r1_p= " <<rand_r1_p<< endl;
		cout<< "customer = "<< customer <<endl;
	}
	cout<<"===========CUST from shake 1-0=========" <<endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p << "]=" <<customer<< endl; //position starts from 0 here
	cout<< "goto route[" << rand_r2 <<"][" << rand_r2_p << endl;

	insert_one_cust (dist, demand, rand_r2, rand_r2_p, routes, customer);
	delete_one_cust (dist, demand, rand_r1, rand_r1_p, routes);


	double total_cost = 0.0;

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		route_cost[i] = routes[i][1]; //update route_cost (global variable)  
		total_cost += routes[i][1]; 
	}

	//================= update global variable (added on 26Feb2015) ===============================//
	//total_demand[rand_r1] = total_demand[rand_r1] - demand_c1; 
	//total_demand[rand_r2] = total_demand[rand_r2] + demand_c1;
	//saiz[rand_r1] = saiz[rand_r1]-1;
	//saiz[rand_r2] = saiz[rand_r2]+1;
	//distance_available[rand_r1] = DISTANCE - route_cost[rand_r1];
	//distance_available[rand_r2] = DISTANCE - route_cost[rand_r2];
	//space_available[rand_r1] = CAPACITY - total_demand[rand_r1];
	//space_available[rand_r2] = CAPACITY - total_demand[rand_r2];

	cout<<"============Routes after shake in 1-0============"<<endl;
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		for (int k = 0; k <= routes[i][2]+3; k++)
		{
			cout << routes[i][k] << ' ';
		}
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		delete[] vehic_depotCust[i];
	}
	delete[] vehic_depotCust;
	delete[] flag_module;
	return 1; //shake status is ok

}


int shake_1_1(double **(&routes), double **dist, int *demand, double *x, double *y)//routes will copy from LOCAL_BEST_ROUTE in this function, only shake the best route
{
	ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	int **vehic_depotCust = new int* [LOCAL_NO_ROUTE];
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		vehic_depotCust[i] = new int[SIZE]; //vehic_depotCust contains only customer with depot at front and back, for reoptimize
	}

	int no_routes = LOCAL_NO_ROUTE;
	route_CGravity = new double*[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		route_CGravity[i] = new double[2]; //2 columns consists of x and y
	}

	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		int j=0; 
		int m=1; //for vehic_depotCust
		//saiz[i] = LOCAL_BEST_ROUTE[i][2];
		saiz[i] = routes[i][2]; //added 22Apr15
		do
		{
			//routes[i][j] = LOCAL_BEST_ROUTE[i][j];
			vehic_depotCust[i][0] = SIZE;
			if (j>=3)
			{
				vehic_depotCust[i][m] = routes[i][j];//added 22Apr15
				//vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
				m++;
			}
			j++;
		//} while (LOCAL_BEST_ROUTE[i][j]!= -1);
		} while (routes[i][j]!= -1);//added 22Apr15
		//routes[i][j] = -1;
		vehic_depotCust[i][saiz[i]+1] = SIZE;

	}
	//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	//{
	//	int j=0; 
	//	int m=1; //for vehic_depotCust
	//	saiz[i] = LOCAL_BEST_ROUTE[i][2];

	//	do
	//	{
	//		routes[i][j] = LOCAL_BEST_ROUTE[i][j];
	//		vehic_depotCust[i][0] = SIZE;
	//		if (j>=3)
	//		{
	//			vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
	//			//k++;
	//			m++;

	//		}
	//		j++;
	//	} while (LOCAL_BEST_ROUTE[i][j]!= -1);
	//	routes[i][j] = -1;
	//	vehic_depotCust[i][saiz[i]+1] = SIZE;

	//}

	int rand_r1 = -1, rand_r2 = -1,  rand_r1_p = -1, rand_r2_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust1_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0;
	int demand_c1, demand_c2;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1;
	double cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0;

	//********************************************************  Neighbour_k 2 (1-1)  ******************************************************************************//

	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}
	int no_flag=0;
	m2_regenerate:
		int r2_found =0 ;
		srand ( time(NULL) ); //seed it
		rand_r1 = (rand() % LOCAL_NO_ROUTE); // for deletion cannot delete from empty route (last route), NO_BEST_ROUTEs = 3 , [0][1][2], can only delete from [0][1]
		saiz_r1 = (int)routes[rand_r1][2];
		
		if(saiz_r1 == 0)//have to test this before finding the position to avoid division by zero error
			goto m2_regenerate;
		
		rand_r1_p = (rand() % saiz_r1)+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE
		cust1_r1 = vehic_depotCust[rand_r1][rand_r1_p];
		demand_c1 = demand[cust1_r1];

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or from flag false
			goto m2_regenerate;
			
		before_cust1 = vehic_depotCust[rand_r1][rand_r1_p-1];
		after_cust1 = vehic_depotCust[rand_r1][rand_r1_p+1];

		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		//{
		//	if ((i == rand_r1) || (saiz[i] == 0) ) //if the same route, skip or if the saiz = 0 because this is 1-1 cannot from empty route
		//		continue;
		//	
		//	for (int j = 1; j <= saiz[i]; j++) //swap positions are  equal to the saiz
		//	{
		//		cust1_r2 = vehic_depotCust[i][j];
		//		before_cust2 = vehic_depotCust[i][j-1];
		//		after_cust2 = vehic_depotCust[i][j+1];
		//		demand_c2 = demand[cust1_r2];
		//		rand_r2 = i;

		//		gain1 = (dist[before_cust1][cust1_r2] + dist[cust1_r2][after_cust1] + service_time[cust1_r2]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][after_cust1] + service_time[cust1_r1]); //new-old
		//		gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) - (dist[before_cust2][cust1_r2] + dist[cust1_r2][after_cust2] + service_time[cust1_r2]); //new-old

		//		if ((demand_c1 <= (space_available[rand_r2]+demand_c2)) && (demand_c2 <= (space_available[rand_r1]+demand_c1)) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
		//		{
		//			rand_r2 = i;
		//			rand_r2_p = j;
		//			r2_found = 1;
		//			goto found_r2_m2;
		//		}

		//	}
		//}
		//================================ End (Change here for guided shake) ================================///
		//++++++++++++++++++++++++++++++++ Guided shake +++++++++++++++++++++++++++++++++++++++++++++++++++++//
		bool *RFlag = new bool[no_routes];
		for (int i = 0; i < no_routes; i++)
		{
			RFlag[i] = true; //initially all routes are true
		}
		calculate_centreGravity(routes, x, y, no_routes);
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
		}
		CGravityfile <<endl;

		findDist_from_CGravity (cust1_r1, x, y, dist, custRGravity, sorted_custRGravity, no_routes);
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		CGravityfile <<endl;
		
	refindR2:
		int route2 = -1;//initialize
		
		//******************************** find route based on probability ************************************//
		//double randNum = (double) rand() / double(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		for (int i = 0; i < no_routes; i++)
		{
			if (RFlag[(int)sorted_custRGravity[i][1]] == true) //if the route is true
			{
				route2 = sorted_custRGravity[i][1];
				goto goout;
			}
		}
		if (route2 == -1) //if not found route from the list
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			goto m2_regenerate;
		}
	goout:
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		//CGravityfile <<"randNum= "<<randNum<<' '<<' '<<"route2= "<<route2<<endl;
		CGravityfile <<"route2= "<<route2<<endl;
		CGravityfile << "cust is "<<cust1_r1<<' '<<"x[cust]= "<<x[cust1_r1]<<' '<<"y[cust]= "<<y[cust1_r1]<<endl;
		CGravityfile <<endl;
		
		if ((route2 == rand_r1) || (saiz[route2] == 0) ) //if the same route, skip or if the saiz = 0 because this is 1-1 cannot from empty route
		{
			RFlag[route2] = false; //for findDist_from_CGravity_choose_nearest use
			goto refindR2;
		}

		for (int j = 1; j <= saiz[route2]; j++) //swap positions are  equal to the saiz
		{
			cust1_r2 = vehic_depotCust[route2][j];
			before_cust2 = vehic_depotCust[route2][j-1];
			after_cust2 = vehic_depotCust[route2][j+1];
			demand_c2 = demand[cust1_r2];
			rand_r2 = route2;

			gain1 = (dist[before_cust1][cust1_r2] + dist[cust1_r2][after_cust1] + service_time[cust1_r2]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][after_cust1] + service_time[cust1_r1]); //new-old
			gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) - (dist[before_cust2][cust1_r2] + dist[cust1_r2][after_cust2] + service_time[cust1_r2]); //new-old

			if ((demand_c1 <= (space_available[rand_r2]+demand_c2)) && (demand_c2 <= (space_available[rand_r1]+demand_c1)) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
			{
				rand_r2 = route2;
				rand_r2_p = j;
				r2_found = 1;
				goto found_r2_m2;
			}

		}
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//

		if (r2_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			no_flag++;
			if (no_flag >= SIZE)
			{	
				cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE 1-1!" <<endl;
				for (int i = 0; i < SIZE; i++)
				{
					cout<<flag_module[i]<<' ';
				}
				return 0;
			}

			goto m2_regenerate;
		}
		
	found_r2_m2:
		if ( (cust1_r1 == -1) || (cust1_r2 == -1) )
			cout<< "cust1_r1 or cust1_r2 cannot be negative!!" <<endl;

		route_change[0] = rand_r1; //route (from)
		route_change[1] = rand_r2; //route (to)
		route_change[2] = -1; //reinitialize 

	//******************************************************** END OF Neighbour_k 2 (1-1)  ******************************************************************************//
	int customer1 = cust1_r1; 
	int customer2 = cust1_r2;

	if ( (customer1 < 0) || (customer2 < 0) )
	{
		cout<< "customer to be deleted cannot be negative!!!!!!" <<endl;
		cout<< "r1= " <<rand_r1 <<' '<< "r2= " <<rand_r2 <<endl;
		cout<< "r1_p= " <<rand_r1_p<<' ' << "r2_p= " <<rand_r2_p<<endl;
		cout<< "customer1 = "<< customer1 <<endl;
		cout<< "customer2 = "<< customer2 <<endl;
	}
	
	
	rand_r1_p = rand_r1_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	rand_r2_p = rand_r2_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	
	cout<<"===========CUST from shake 1-1=========" <<endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p << "]=" <<customer1<< endl;
	cout<< "route[" << rand_r2 <<"][" << rand_r2_p << "]=" <<customer2<< endl;

	swap_in_oneCust (dist, demand, rand_r1, rand_r1_p, routes, customer2);
	swap_in_oneCust (dist, demand, rand_r2, rand_r2_p, routes, customer1);


	double total_cost = 0.0;

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		route_cost[i] = routes[i][1]; //update route_cost (global variable)  
		total_cost += routes[i][1]; //update total_cost (global variable)  
	}
	//================= update global variable (added on 26Feb2015) ===============================//
	//total_demand[rand_r1] = total_demand[rand_r1] - demand_c1 + demand_c2; 
	//total_demand[rand_r2] = total_demand[rand_r2] - demand_c2 + demand_c1;
	//distance_available[rand_r1] = DISTANCE - route_cost[rand_r1];
	//distance_available[rand_r2] = DISTANCE - route_cost[rand_r2];
	//space_available[rand_r1] = CAPACITY - total_demand[rand_r1];
	//space_available[rand_r2] = CAPACITY - total_demand[rand_r2];

	cout<<"============ Routes after shake in 1-1 ============"<<endl;
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		for (int k = 0; k <= routes[i][2]+3; k++)
		{
			cout << routes[i][k] << ' ';
		}
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		delete[] vehic_depotCust[i];
	}
	delete[] vehic_depotCust;
	delete[] flag_module;
	return 1; //shake status is ok
}


int shake_2_0_twoR(double **(&routes), double **dist, int *demand, double *x, double *y)//routes will copy from BEST_ROUTE in this function, only shake the best route
{
	ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	int **vehic_depotCust = new int* [LOCAL_NO_ROUTE];
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		vehic_depotCust[i] = new int[SIZE]; //vehic_depotCust contains only customer with depot at front and back, for reoptimize
	}

	int no_routes = LOCAL_NO_ROUTE;
	route_CGravity = new double*[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		route_CGravity[i] = new double[2]; //2 columns consists of x and y
	}
	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		int j=0; 
		int m=1; //for vehic_depotCust
		//saiz[i] = LOCAL_BEST_ROUTE[i][2];
		saiz[i] = routes[i][2]; //added 22Apr15
		do
		{
			//routes[i][j] = LOCAL_BEST_ROUTE[i][j];
			vehic_depotCust[i][0] = SIZE;
			if (j>=3)
			{
				vehic_depotCust[i][m] = routes[i][j];//added 22Apr15
				//vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
				m++;
			}
			j++;
		//} while (LOCAL_BEST_ROUTE[i][j]!= -1);
		} while (routes[i][j]!= -1);//added 22Apr15
		//routes[i][j] = -1;
		vehic_depotCust[i][saiz[i]+1] = SIZE;

	}
	//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	//{
	//	int j=0; 
	//	int m=1; //for vehic_depotCust
	//	saiz[i] = LOCAL_BEST_ROUTE[i][2];

	//	do
	//	{
	//		routes[i][j] = LOCAL_BEST_ROUTE[i][j];
	//		vehic_depotCust[i][0] = SIZE;
	//		if (j>=3)
	//		{
	//			vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
	//			m++;

	//		}
	//		j++;
	//	} while (LOCAL_BEST_ROUTE[i][j]!= -1);
	//	routes[i][j] = -1;
	//	vehic_depotCust[i][saiz[i]+1] = SIZE;

	//}

	//**************all insertion and deletion starts from 1 to saiz because the vehic_depotCust starts from depot, end with depot**************//
	//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//

	int rand_r1 = -1, rand_r2 = -1, rand_r3 = -1, rand_r1_p = -1, rand_r2_p = -1, rand_r3_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1, cust1_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0, saiz_r3 = 0;
	int demand_c1, demand_c2, demand_c3;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1;
	double cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0;

	//******************************************************** Neighbour_k 3 (2-0)  ******************************************************************************//
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}	
	int no_flag=0;
	for (int i = 0; i < no_routes; i++)
	{
		if (saiz[i] == 1)
		{
			flag_module[vehic_depotCust[i][1]] = false;
			no_flag++;
		}
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			flag_module[vehic_depotCust[i][saiz[i]]] = false; //flag the last cust in a route false because last customer cannot be selected, this is 2-1 sequential//this is a trick!!!!!!!!!!!
			no_flag++;
		}
		
	}
	m3_regenerate:
		int r2_found=0;
		srand ( time(NULL) ); //seed it
		rand_r1 = (rand() % LOCAL_NO_ROUTE); 
		saiz_r1 = (int)routes[rand_r1][2];
		if((saiz_r1 == 0)||(saiz_r1 == 1))//have to test this before finding the position to avoid division by zero error
			goto m3_regenerate;
		rand_r1_p = (rand() % (saiz_r1-1))+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE; //minus one because two customers will be deleted in sequential
		cust1_r1 = vehic_depotCust[rand_r1][rand_r1_p];
		cust2_r1 = vehic_depotCust[rand_r1][rand_r1_p+1];//customer in sequential		
		demand_c1 = demand[cust1_r1];
		demand_c2 = demand[cust2_r1];

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or route with 1 cust or from flag false //make sure deletion cannot be made from empty route
			goto m3_regenerate;
				
		before_cust1 = vehic_depotCust[rand_r1][rand_r1_p-1];
		after_cust1 = vehic_depotCust[rand_r1][rand_r1_p+2];

		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		//{
		//	if ((saiz[rand_r1] == 2) && (saiz[i]==0)) //if size of original route is 2 and new route is empty route, skip because it will end up the same
		//		continue;

		//	if ((i == rand_r1) || (demand_c1+demand_c2 > space_available[i]))//if the same route or exceed capacity, skip
		//			continue;
		//		if (saiz[i] == 0)
		//		{
		//			rand_r2 = i;
		//			rand_r2_p = 1;
		//			r2_found = 1;
		//			goto found_r2_m3;
		//		}
		//		for (int j = 1; j <= saiz[i]+1; j++) //insertion can be made one position more than saiz
		//		{
		//			//cust1_r2 = vehic_depotCust[i][j];
		//			before_cust2 = vehic_depotCust[i][j-1];
		//			after_cust2 = vehic_depotCust[i][j]; //current position
		//			rand_r2 = i;

		//			gain1 = (dist[before_cust1][after_cust1]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1] + service_time[cust2_r1]); //new-old
		//			gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust2] + service_time[cust1_r1] + service_time[cust2_r1]) - (dist[before_cust2][after_cust2]); //new-old

		//			if ((demand_c1+demand_c2<= space_available[rand_r2]) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
		//			{
		//				rand_r2 = i;
		//				rand_r2_p = j;
		//				r2_found = 1;
		//				goto found_r2_m3;
		//			}
		//			else
		//				continue;

		//		}
		//}
		//================================ End (Change here for guided shake) ================================///
		//++++++++++++++++++++++++++++++++ Guided shake +++++++++++++++++++++++++++++++++++++++++++++++++++++//
		bool *RFlag = new bool[no_routes];
		for (int i = 0; i < no_routes; i++)
		{
			RFlag[i] = true; //initially all routes are true
		}
		calculate_centreGravity(routes, x, y, no_routes);
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
		}
		CGravityfile <<endl;
		

		//find route based on probability
		findDist_from_CGravity (cust1_r1, x, y, dist, custRGravity, sorted_custRGravity, no_routes);
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		CGravityfile <<endl;
		

	refindR2:
		int route2 = -1;//initialize
		
		//******************************** find route based on probability ************************************//
		//double randNum = (double) rand() / double(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		for (int i = 0; i < no_routes; i++)
		{
			if (RFlag[(int)sorted_custRGravity[i][1]] == true) //if the route is true
			{
				route2 = sorted_custRGravity[i][1];
				goto goout;
			}
		}
		if (route2 == -1)//if not found route from the list
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			goto m3_regenerate;
		}
	goout:
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		//CGravityfile <<"randNum= "<<randNum<<' '<<' '<<"route2= "<<route2<<endl;
		CGravityfile <<"route2= "<<route2<<endl;
		CGravityfile << "cust is "<<cust1_r1<<' '<<"x[cust]= "<<x[cust1_r1]<<' '<<"y[cust]= "<<y[cust1_r1]<<endl;
		CGravityfile <<endl;
		
		if ((saiz[rand_r1] == 2) && (saiz[route2]==0)) //if size of original route is 2 and new route is empty route, skip because it will end up the same
		{
			RFlag[route2] = false; //for findDist_from_CGravity_choose_nearest use
			goto refindR2;
		}
		
		if ((route2 == rand_r1) || (demand_c1+demand_c2 > space_available[route2]))//if the same route or exceed capacity, skip
		{
			RFlag[route2] = false; //for findDist_from_CGravity_choose_nearest use
			goto refindR2;
		}
		if (saiz[route2] == 0)
		{
			rand_r2 = route2;
			rand_r2_p = 1;
			r2_found = 1;
			goto found_r2_m3;
		}
		for (int j = 1; j <= saiz[route2]+1; j++) //insertion can be made one position more than saiz
		{
			//cust1_r2 = vehic_depotCust[route2][j];
			before_cust2 = vehic_depotCust[route2][j-1];
			after_cust2 = vehic_depotCust[route2][j]; //current position
			rand_r2 = route2;

			gain1 = (dist[before_cust1][after_cust1]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1] + service_time[cust2_r1]); //new-old
			gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust2] + service_time[cust1_r1] + service_time[cust2_r1]) - (dist[before_cust2][after_cust2]); //new-old

			if ((demand_c1+demand_c2<= space_available[rand_r2]) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
			{
				rand_r2 = route2;
				rand_r2_p = j;
				r2_found = 1;
				goto found_r2_m3;
			}
			else
				continue;

		}
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//

		if (r2_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			no_flag++;
			if (no_flag >= SIZE)
			{	
				cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE2-0 TWO ROUTES!" <<endl;
				for (int i = 0; i < SIZE; i++)
				{
					cout<<flag_module[i]<<' ';
				}
				return 0;
			}
			goto m3_regenerate;
		}
		
	found_r2_m3:
		route_change[0] = rand_r1; //route (from)
		route_change[1] = rand_r2; //route (to)
		route_change[2] = -1; //reinitialize 

	//******************************************************** END OF Neighbour_k 3 (2-0)  ******************************************************************************//

	// erase the element from route 1

	int customer1 = cust1_r1; 
	int customer2 = cust2_r1;
	
	rand_r1_p = rand_r1_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	rand_r2_p = rand_r2_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]

	if ( (customer1 < 0) || (customer2 < 0) )
	{
		cout<< "customer to be deleted cannot be negative!!!!!!" <<endl;
		cout<< "r1= " <<rand_r1 <<' '<< "r2= " <<rand_r2 <<endl;
		cout<< "r1_p= " <<rand_r1_p<<' ' << "r2_p= " <<rand_r2_p<<endl;
		cout<< "customer1 = "<< customer1 <<endl;
		cout<< "customer2 = "<< customer2 <<endl;
	}
	cout<<"===========CUST from shake 2-0 TwoR=========" <<endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p << "]=" <<customer1<< endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p+1 << "]=" <<customer2<< endl;
	cout<< "goto route[" << rand_r2 <<"]["<<rand_r2_p<<"]"<<endl;

	insert_two_cust (dist, demand, rand_r2, rand_r2_p, routes, customer1, customer2);
	delete_two_cust (dist, demand, rand_r1, rand_r1_p, routes);

	double total_cost = 0.0;

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		route_cost[i] = routes[i][1]; //update route_cost (global variable)  
		total_cost += routes[i][1]; //update total_cost (global variable)  
	}

	//================= update global variable (added on 26Feb2015) ===============================//
	//total_demand[rand_r1] = total_demand[rand_r1] - demand_c1 - demand_c2; 
	//total_demand[rand_r2] = total_demand[rand_r2] + demand_c1 + demand_c2;
	//saiz[rand_r1] = saiz[rand_r1] - 2;
	//saiz[rand_r2] = saiz[rand_r2] + 2;
	//distance_available[rand_r1] = DISTANCE - route_cost[rand_r1];
	//distance_available[rand_r2] = DISTANCE - route_cost[rand_r2];
	//space_available[rand_r1] = CAPACITY - total_demand[rand_r1];
	//space_available[rand_r2] = CAPACITY - total_demand[rand_r2];


	cout<<"============Routes after shake in 2-0 two routes============"<<endl;
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		for (int k = 0; k <= routes[i][2]+3; k++)
		{
			cout << routes[i][k] << ' ';
		}
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	//find_capacity(capa, demand);
	//
	//cout<<"capa from shake 2-0 two routes, not yet update routes" <<endl;						
	//for (int i = 0; i < NO_BEST_ROUTE; i++)
	//{
	//	cout<< "capa[i]" << capa[i] <<' ' ;
	//}
	//cout<<endl<<endl;
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		delete[] vehic_depotCust[i];
	}
	delete[] vehic_depotCust;
	delete[] flag_module;
	return 1; //shake status is ok
}

int shake_2_0_threeR(double **(&routes), double **dist, int *demand, double *x, double *y)//routes will copy from BEST_ROUTE in this function, only shake the best route
{
	ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	int **vehic_depotCust = new int* [LOCAL_NO_ROUTE];
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		vehic_depotCust[i] = new int[SIZE]; //vehic_depotCust contains only customer with depot at front and back, for reoptimize
	}

	int no_routes = LOCAL_NO_ROUTE;
	route_CGravity = new double*[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		route_CGravity[i] = new double[2]; //2 columns consists of x and y
	}

	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		int j=0; 
		int m=1; //for vehic_depotCust
		//saiz[i] = LOCAL_BEST_ROUTE[i][2];
		saiz[i] = routes[i][2]; //added 22Apr15
		do
		{
			//routes[i][j] = LOCAL_BEST_ROUTE[i][j];
			vehic_depotCust[i][0] = SIZE;
			if (j>=3)
			{
				vehic_depotCust[i][m] = routes[i][j];//added 22Apr15
				//vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
				m++;
			}
			j++;
		//} while (LOCAL_BEST_ROUTE[i][j]!= -1);
		} while (routes[i][j]!= -1);//added 22Apr15
		//routes[i][j] = -1;
		vehic_depotCust[i][saiz[i]+1] = SIZE;

	}

	//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	//{
	//	int j=0; 
	//	int m=1; //for vehic_depotCust
	//	saiz[i] = LOCAL_BEST_ROUTE[i][2];

	//	do
	//	{
	//		routes[i][j] = LOCAL_BEST_ROUTE[i][j];
	//		vehic_depotCust[i][0] = SIZE;
	//		if (j>=3)
	//		{
	//			vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
	//			m++;

	//		}
	//		j++;
	//	} while (LOCAL_BEST_ROUTE[i][j]!= -1);
	//	routes[i][j] = -1;
	//	vehic_depotCust[i][saiz[i]+1] = SIZE;

	//}

	//**************all insertion and deletion starts from 1 to saiz because the vehic_depotCust starts from depot, end with depot**************//
	//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//

	int rand_r1 = -1, rand_r2 = -1, rand_r3 = -1, rand_r1_p = -1, rand_r2_p = -1, rand_r3_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1, cust1_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0, saiz_r3 = 0;
	int demand_c1, demand_c2, demand_c3;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1, before_cust3=-1, after_cust3=-1;
	double cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0, gain3=0.0;
	
	
	//******************************************************** Neighbour_k 4 (2-0)  ******************************************************************************//
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}	
	int no_flag=0;
	for (int i = 0; i < no_routes; i++)
	{
		if (saiz[i] == 1)  //if saiz=1 
		{
			flag_module[vehic_depotCust[i][1]] = false;
			no_flag++;
		}
		
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			flag_module[vehic_depotCust[i][saiz[i]]] = false; //flag the last cust in a route false because last customer cannot be selected, this is 2-1 sequential//this is a trick!!!!!!!!!!!
			no_flag++;
		}
	
		
	}

	m4_regenerate:
		int r2_found=0;
		srand ( time(NULL) ); //seed it
		rand_r1 = (rand() % LOCAL_NO_ROUTE); 
		saiz_r1 = (int)routes[rand_r1][2];

		if((saiz_r1 == 0) || (saiz_r1 == 1))//have to test this before finding the position to avoid division by zero error
			goto m4_regenerate;
		
		rand_r1_p = (rand() % (saiz_r1-1))+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE; //minus one because two customers will be deleted in sequential
		cust1_r1 = vehic_depotCust[rand_r1][rand_r1_p];
		cust2_r1 = vehic_depotCust[rand_r1][rand_r1_p+1];//customer in sequential		
		demand_c1 = demand[cust1_r1];
		demand_c2 = demand[cust2_r1];

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or from flag false //make sure deletion cannot be made from empty route
			goto m4_regenerate;
				
		before_cust1 = vehic_depotCust[rand_r1][rand_r1_p-1];
		after_cust1 = vehic_depotCust[rand_r1][rand_r1_p+2];
		
		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		//{
		//	if (i == rand_r1) //if the same route, skip or if the saiz = 0 because this is 2-0 cannot from empty route
		//		continue;
		//	
		//	if (saiz[i] == 0)
		//	{
		//		rand_r2 = i;
		//		rand_r2_p = 1;
		//		r2_found = 1;
		//		goto found_r2_m4;
		//	}

		//	for (int j = 1; j <= saiz[i]+1; j++) //insertion can be made one position more than saiz
		//	{
		//		before_cust2 = vehic_depotCust[i][j-1];
		//		after_cust2 = vehic_depotCust[i][j]; //current position
		//		rand_r2 = i;

		//		gain1 = (dist[before_cust1][after_cust1]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1] + service_time[cust2_r1]); //new-old
		//		gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) - (dist[before_cust2][after_cust2]); //new-old

		//		if ((demand_c1 <= space_available[rand_r2]) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
		//		{
		//			rand_r2 = i;
		//			rand_r2_p = j;
		//			r2_found = 1;
		//			goto found_r2_m4;
		//		}

		//	}
		//}
		//================================ End (Change here for guided shake) ================================///
		//++++++++++++++++++++++++++++++++ Guided shake +++++++++++++++++++++++++++++++++++++++++++++++++++++//
		bool *RFlag = new bool[no_routes];
		for (int i = 0; i < no_routes; i++)
		{
			RFlag[i] = true; //initially all routes are true
		}
		calculate_centreGravity(routes, x, y, no_routes);
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
		}
		CGravityfile <<endl;
		

		//find route based on probability
		findDist_from_CGravity (cust1_r1, x, y, dist, custRGravity, sorted_custRGravity, no_routes);
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		CGravityfile <<endl;
		

	refindR2:
		int route2 = -1;//initialize
		
		//******************************** find route based on probability ************************************//
		//double randNum = (double) rand() / double(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		for (int i = 0; i < no_routes; i++)
		{
			if (RFlag[(int)sorted_custRGravity[i][1]] == true) //if the route is true
			{
				route2 = sorted_custRGravity[i][1];
				goto goout;
			}
		}
		if (route2 == -1)//if not found route from the list
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			goto m4_regenerate;
		}
	goout:
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		//CGravityfile <<"randNum= "<<randNum<<' '<<' '<<"route2= "<<route2<<endl;
		CGravityfile <<"route2= "<<route2<<endl;
		CGravityfile << "cust is "<<cust1_r1<<' '<<"x[cust]= "<<x[cust1_r1]<<' '<<"y[cust]= "<<y[cust1_r1]<<endl;
		CGravityfile <<endl;

		if (route2 == rand_r1) //if the same route, skip or if the saiz = 0 because this is 2-0 cannot from empty route
		{
			RFlag[route2] = false; //for findDist_from_CGravity_choose_nearest use
			goto refindR2;
		}
		if (saiz[route2] == 0)
		{
			rand_r2 = route2;
			rand_r2_p = 1;
			r2_found = 1;
			goto found_r2_m4;
		}
		for (int j = 1; j <= saiz[route2]+1; j++) //insertion can be made one position more than saiz
		{
			before_cust2 = vehic_depotCust[route2][j-1];
			after_cust2 = vehic_depotCust[route2][j]; //current position
			rand_r2 = route2;

			gain1 = (dist[before_cust1][after_cust1]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1] + service_time[cust2_r1]); //new-old
			gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) - (dist[before_cust2][after_cust2]); //new-old

			if ((demand_c1 <= space_available[rand_r2]) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
			{
				rand_r2 = route2;
				rand_r2_p = j;
				r2_found = 1;
				goto found_r2_m4;
			}

		}
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//
		
		if (r2_found != 1)
		{
			if (flag_module[cust1_r1] != false)
			{
				flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
				no_flag++;
			}
			goto m4_regenerate;
		}
		
	found_r2_m4:
		//==================find r3=========================//
		int r3_found = 0;
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			if ((i == rand_r1) || (i == rand_r2) ) //if the same route, skip, saiz = 0 is ok because insertion can go to empty route
				continue;
			
			if (saiz[i] == 0)
			{
				rand_r3 = i;
				rand_r3_p = 1;
				r3_found = 1;
				goto found_r3_m4;
			}
			for (int j = 1; j <= saiz[i]+1; j++) //insertion can be made one position more than saiz
			{
				before_cust3 = vehic_depotCust[i][j-1];
				after_cust3 = vehic_depotCust[i][j]; //current position
				rand_r3 = i;

				gain3 = (dist[before_cust3][cust2_r1] + dist[cust2_r1][after_cust3] + service_time[cust2_r1]) - (dist[before_cust3][after_cust3]); //new-old //CHANGED THIS ON 4 MARC 2015!!!!!!!!!!!!!!!CARELESS MISTAKE!!!!!!!!!!!!!!

				if ((demand_c2 <= space_available[rand_r3]) && (gain3 <= distance_available[rand_r3])) //if fulfill both constraints
				{
					rand_r3 = i;
					rand_r3_p = j;
					r3_found = 1;
					goto found_r3_m4;
				}

			}
		}
		
		if (r3_found != 1)
		{
			if (flag_module[cust1_r1] != false)
			{
				flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd //this will redo everything again!!!!!!!!!!!!!!!!!!!!
				no_flag++;
			}
		
			if (no_flag >= SIZE)
			{	
				cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE2_0 THREE ROUTES!" <<endl;
				for (int i = 0; i < SIZE; i++)
				{
					cout<<flag_module[i]<<' ';
				}
				return 0;
			}
			goto m4_regenerate;
		}
			
	found_r3_m4:
			
		if ( (cust1_r1 == -1) || (cust2_r1 == -1))
			cout<< "cust1_r1 or cust1_r2 cannot be negative!!" <<endl;
			

		route_change[0] = rand_r1; //route (from)
		route_change[1] = rand_r2; //route (to)
		route_change[2] = rand_r3; //route (to)
					

	//******************************************************** END OF Neighbour_k 4 (2-0)  ******************************************************************************//
	int customer1 = cust1_r1; 
	int customer2 = cust2_r1; //second customer in route1

	rand_r1_p = rand_r1_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	rand_r2_p = rand_r2_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	rand_r3_p = rand_r3_p-1; // added this on 1MARCH2015
	
	if ( (customer1 < 0) || (customer2 < 0))
	{
		cout<< "customer to be deleted cannot be negative!!!!!!" <<endl;
		cout<< "r1= " <<rand_r1 <<' '<< "r2= " <<rand_r2 <<endl;
		cout<< "r1_p= " <<rand_r1_p<<' ' << "r2_p= " <<rand_r2_p<<endl;
		cout<< "customer1 = "<< customer1 <<endl;
		cout<< "customer2 = "<< customer2 <<endl;
	}
	cout<<"===========CUST from shake 2-0 ThreeR=========" <<endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p << "]=" <<customer1<< endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p+1 << "]=" <<customer2<< endl;
	cout<< "goto route[" << rand_r2 <<"][" << rand_r2_p << "]=" << endl;
	cout<< "goto route[" << rand_r3 <<"][" << rand_r3_p << "]=" << endl;

	delete_two_cust (dist, demand, rand_r1, rand_r1_p, routes); //2 out
	insert_one_cust (dist, demand, rand_r2, rand_r2_p, routes, customer1); //1 in for route2
	insert_one_cust (dist, demand, rand_r3, rand_r3_p, routes, customer2); //1 in for route3

	double total_cost = 0.0;

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		route_cost[i] = routes[i][1]; //update route_cost (global variable)  
		total_cost += routes[i][1]; //update total_cost (global variable) 
	}

	//================= update global variable (added on 26Feb2015) ===============================//
	//total_demand[rand_r1] = total_demand[rand_r1] - demand_c1 - demand_c2; 
	//total_demand[rand_r2] = total_demand[rand_r2] + demand_c1;
	//total_demand[rand_r3] = total_demand[rand_r3] + demand_c2;
	//saiz[rand_r1] = saiz[rand_r1] - 2;
	//saiz[rand_r2] = saiz[rand_r2] + 1;
	//saiz[rand_r3] = saiz[rand_r3] + 1;
	//distance_available[rand_r1] = DISTANCE - route_cost[rand_r1];
	//distance_available[rand_r2] = DISTANCE - route_cost[rand_r2];
	//distance_available[rand_r3] = DISTANCE - route_cost[rand_r3];
	//space_available[rand_r1] = CAPACITY - total_demand[rand_r1];
	//space_available[rand_r2] = CAPACITY - total_demand[rand_r2];
	//space_available[rand_r3] = CAPACITY - total_demand[rand_r3];

	cout<<"============Routes after shake in 2-0 three routes============"<<endl;
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		for (int k = 0; k <= routes[i][2]+3; k++)
		{
			cout << routes[i][k] << ' ';
		}
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	//find_capacity(capa, demand);
	//
	//cout<<"capa from shake 2-1 three routes, not yet update routes" <<endl;						
	//for (int i = 0; i < NO_BEST_ROUTE; i++)
	//{
	//	cout<< "capa[i]" << capa[i] <<' ' ;
	//}
	//cout<<endl<<endl;
	
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		delete[] vehic_depotCust[i];
	}

	delete[] vehic_depotCust;
	delete[] flag_module;
	return 1; //shake status is ok
}

int shake_2_1_twoR(double **(&routes), double **dist, int *demand, double *x, double *y)//routes will copy from BEST_ROUTE in this function, only shake the best route
{
	ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	int **vehic_depotCust = new int* [LOCAL_NO_ROUTE];
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		vehic_depotCust[i] = new int[SIZE]; //vehic_depotCust contains only customer with depot at front and back, for reoptimize
	}

	int no_routes = LOCAL_NO_ROUTE;
	route_CGravity = new double*[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		route_CGravity[i] = new double[2]; //2 columns consists of x and y
	}
	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		int j=0; 
		int m=1; //for vehic_depotCust
		//saiz[i] = LOCAL_BEST_ROUTE[i][2];
		saiz[i] = routes[i][2]; //added 22Apr15
		do
		{
			//routes[i][j] = LOCAL_BEST_ROUTE[i][j];
			vehic_depotCust[i][0] = SIZE;
			if (j>=3)
			{
				vehic_depotCust[i][m] = routes[i][j];//added 22Apr15
				//vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
				m++;
			}
			j++;
		//} while (LOCAL_BEST_ROUTE[i][j]!= -1);
		} while (routes[i][j]!= -1);//added 22Apr15
		//routes[i][j] = -1;
		vehic_depotCust[i][saiz[i]+1] = SIZE;

	}
	
	//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	//{
	//	int j=0; 
	//	int m=1; //for vehic_depotCust
	//	saiz[i] = LOCAL_BEST_ROUTE[i][2];

	//	do
	//	{
	//		routes[i][j] = LOCAL_BEST_ROUTE[i][j];
	//		vehic_depotCust[i][0] = SIZE;
	//		if (j>=3)
	//		{
	//			vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
	//			m++;

	//		}
	//		j++;
	//	} while (LOCAL_BEST_ROUTE[i][j]!= -1);
	//	routes[i][j] = -1;
	//	vehic_depotCust[i][saiz[i]+1] = SIZE;

	//}


	//**************all insertion and deletion starts from 1 to saiz because the vehic_depotCust starts from depot, end with depot**************//
	//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//

	int rand_r1 = -1, rand_r2 = -1, rand_r3 = -1, rand_r1_p = -1, rand_r2_p = -1, rand_r3_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1, cust1_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0, saiz_r3 = 0;
	int demand_c1, demand_c2, demand_c3;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1;
	double cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0;

	//******************************************************** Neighbour_k 5 (2-1)  ******************************************************************************//
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}	
	int no_flag=0;

	for (int i = 0; i < no_routes; i++)
	{
		if (saiz[i] == 1)
		{
			flag_module[vehic_depotCust[i][1]] = false;
			no_flag++;
		}
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			flag_module[vehic_depotCust[i][saiz[i]]] = false; //flag the last cust in a route false because last customer cannot be selected, this is 2-1 sequential//this is a trick!!!!!!!!!!!
			no_flag++;
		}
	}
m5_regenerate:
		if (no_flag >= (0.7*SIZE))
		{
			for (int r = 0; r < no_routes; r++)
			{
				rand_r1 = r;
				saiz_r1 = (int)routes[rand_r1][2];
				if((saiz_r1 == 0) || (saiz_r1 == 1))
					continue;
				
				for (int j = 1; j < saiz[r]; j++)
				{
					if (flag_module[vehic_depotCust[r][j]] == false)
						continue;
					rand_r1_p = j;
					goto getout;
				}
			}
			
		}

		else
		{
			srand ( time(NULL) ); //seed it
			rand_r1 = (rand() % LOCAL_NO_ROUTE); 
			saiz_r1 = (int)routes[rand_r1][2];
			if ((saiz_r1 == 0) || (saiz_r1 == 1))//have to test this before finding the position to avoid division by zero error
				goto m5_regenerate;
			rand_r1_p = (rand() % (saiz_r1-1))+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE; //minus one because two customers will be deleted in sequential
		}
	getout:
		int r2_found=0;
		//srand ( time(NULL) ); //seed it
		//rand_r1 = (rand() % NO_BEST_ROUTE); 
		//saiz_r1 = (int)routes[rand_r1][2];
		//if(saiz_r1 == 0)//have to test this before finding the position to avoid division by zero error
		//	goto m5_regenerate;
		//if(saiz_r1 == 1)//have to flag the customer so that it would not be selected
		//{
		//	flag_module[vehic_depotCust[rand_r1][1]] = false;
		//	goto m5_regenerate;
		//}
		//rand_r1_p = (rand() % (saiz_r1-1))+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE; //minus one because two customers will be deleted in sequential
		
		cust1_r1 = vehic_depotCust[rand_r1][rand_r1_p];
		cust2_r1 = vehic_depotCust[rand_r1][rand_r1_p+1];//customer in sequential		
		demand_c1 = demand[cust1_r1];
		demand_c2 = demand[cust2_r1];

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or route with 1 cust or from flag false //make sure deletion cannot be made from empty route
			goto m5_regenerate;
				
		before_cust1 = vehic_depotCust[rand_r1][rand_r1_p-1];
		after_cust1 = vehic_depotCust[rand_r1][rand_r1_p+2];
		
		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		//{
		//	if ((i == rand_r1) || (saiz[i] == 0) ) //if the same route, skip or if the saiz = 0 because this is 2-1 cannot from empty route
		//		continue;
		//	
		//	for (int j = 1; j <= saiz[i]; j++) //swap positions are  equal to the saiz
		//	{
		//		cust1_r2 = vehic_depotCust[i][j];
		//		before_cust2 = vehic_depotCust[i][j-1];
		//		after_cust2 = vehic_depotCust[i][j+1];
		//		demand_c3 = demand[cust1_r2];
		//		rand_r2 = i;

		//		gain1 = (dist[before_cust1][cust1_r2] + dist[cust1_r2][after_cust1] + service_time[cust1_r2]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1]+ service_time[cust2_r1]); //new-old
		//		gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust2] + service_time[cust1_r1]+ service_time[cust2_r1]) - (dist[before_cust2][cust1_r2] + dist[cust1_r2][after_cust2] + service_time[cust1_r2]); //new-old

		//		if ((demand_c3 <= (space_available[rand_r1]+demand_c1+demand_c2)) && ((demand_c1+demand_c2)<= (space_available[rand_r2]+demand_c3)) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
		//		{
		//			rand_r2 = i;
		//			rand_r2_p = j;
		//			r2_found = 1;
		//			goto found_r2_m5;
		//		}

		//	}
		//}
		//================================ End (Change here for guided shake) ================================///
		//++++++++++++++++++++++++++++++++ Guided shake +++++++++++++++++++++++++++++++++++++++++++++++++++++//
		bool *RFlag = new bool[no_routes];
		for (int i = 0; i < no_routes; i++)
		{
			RFlag[i] = true; //initially all routes are true
		}
		calculate_centreGravity(routes, x, y, no_routes);
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
		}
		CGravityfile <<endl;
		

		//find route based on probability
		findDist_from_CGravity (cust1_r1, x, y, dist, custRGravity, sorted_custRGravity, no_routes);
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		CGravityfile <<endl;

	refindR2:
		int route2 = -1;//initialize
		
		//******************************** find route based on probability ************************************//
		//double randNum = (double) rand() / double(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		for (int i = 0; i < no_routes; i++)
		{
			if (RFlag[(int)sorted_custRGravity[i][1]] == true) //if the route is true
			{
				route2 = sorted_custRGravity[i][1];
				goto goout;
			}
		}
		if (route2 == -1)//if not found route from the list
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			goto m5_regenerate;
		}
	goout:
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		//CGravityfile <<"randNum= "<<randNum<<' '<<' '<<"route2= "<<route2<<endl;
		CGravityfile <<"route2= "<<route2<<endl;
		CGravityfile << "cust is "<<cust1_r1<<' '<<"x[cust]= "<<x[cust1_r1]<<' '<<"y[cust]= "<<y[cust1_r1]<<endl;
		CGravityfile <<endl;


		if ((route2 == rand_r1) || (saiz[route2] == 0) ) //if the same route, skip or if the saiz = 0 because this is 2-1 cannot from empty route
		{
			RFlag[route2] = false; //for findDist_from_CGravity_choose_nearest use
			goto refindR2;
		}
			
		for (int j = 1; j <= saiz[route2]; j++) //swap positions are  equal to the saiz
		{
			cust1_r2 = vehic_depotCust[route2][j];
			before_cust2 = vehic_depotCust[route2][j-1];
			after_cust2 = vehic_depotCust[route2][j+1];
			demand_c3 = demand[cust1_r2];
			rand_r2 = route2;

			gain1 = (dist[before_cust1][cust1_r2] + dist[cust1_r2][after_cust1] + service_time[cust1_r2]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1]+ service_time[cust2_r1]); //new-old
			gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust2] + service_time[cust1_r1]+ service_time[cust2_r1]) - (dist[before_cust2][cust1_r2] + dist[cust1_r2][after_cust2] + service_time[cust1_r2]); //new-old

			if ((demand_c3 <= (space_available[rand_r1]+demand_c1+demand_c2)) && ((demand_c1+demand_c2)<= (space_available[rand_r2]+demand_c3)) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
			{
				rand_r2 = route2;
				rand_r2_p = j;
				r2_found = 1;
				goto found_r2_m5;
			}

		}
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//

		if (r2_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			no_flag++;
			
			if (no_flag >= SIZE)
			{	
				cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE2_0 THREE ROUTES!" <<endl;
				for (int i = 0; i < SIZE; i++)
				{
					cout<<flag_module[i]<<' ';
				}
				return 0;
			}
			goto m5_regenerate;
		}
		
	found_r2_m5:
		route_change[0] = rand_r1; //route (from)
		route_change[1] = rand_r2; //route (to)
		route_change[2] = -1; //reinitialize 

	//******************************************************** END OF Neighbour_k 5 (2-1)  ******************************************************************************//

	// erase the element from route 1

	int customer1 = cust1_r1; 
	int customer2 = cust2_r1;
	int customer3 = cust1_r2; //second customer in route1
	rand_r1_p = rand_r1_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	rand_r2_p = rand_r2_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	
	if ( (customer1 < 0) || (customer2 < 0) )
	{
		cout<< "customer to be deleted cannot be negative!!!!!!" <<endl;
		cout<< "r1= " <<rand_r1 <<' '<< "r2= " <<rand_r2 <<endl;
		cout<< "r1_p= " <<rand_r1_p<<' ' << "r2_p= " <<rand_r2_p<<endl;
		cout<< "customer1 = "<< customer1 <<endl;
		cout<< "customer2 = "<< customer2 <<endl;
	}
	cout<<"===========CUST from shake 2-1 TwoR=========" <<endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p << "]=" <<customer1<< endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p+1 << "]=" <<customer2<< endl;
	cout<< "goto route[" << rand_r2 <<"][" << rand_r2_p << "]=" <<customer3<< endl;

	swap2_1_cust (dist, demand, rand_r1, rand_r1_p, routes, customer3);//2 out, 1 in
	swap1_2_cust (dist, demand, rand_r2, rand_r2_p, routes, customer1, customer2);//1 out, 2 in

	double total_cost = 0.0;

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		route_cost[i] = routes[i][1]; //update route_cost (global variable)  
		total_cost += routes[i][1]; //update total_cost (global variable) 
	}
	//================= update global variable (added on 26Feb2015) ===============================//
	//total_demand[rand_r1] = total_demand[rand_r1] - demand_c1 - demand_c2 + demand_c3; 
	//total_demand[rand_r2] = total_demand[rand_r2] - demand_c3 + demand_c1 + demand_c2;
	//saiz[rand_r1] = saiz[rand_r1] - 2 + 1;
	//saiz[rand_r2] = saiz[rand_r2] - 1 + 2;
	//distance_available[rand_r1] = DISTANCE - route_cost[rand_r1];
	//distance_available[rand_r2] = DISTANCE - route_cost[rand_r2];
	//space_available[rand_r1] = CAPACITY - total_demand[rand_r1];
	//space_available[rand_r2] = CAPACITY - total_demand[rand_r2];


	cout<<"============Routes after shake in 2-1 two routes============"<<endl;
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		for (int k = 0; k <= routes[i][2]+3; k++)
		{
			cout << routes[i][k] << ' ';
		}
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	//find_capacity(capa, demand);
	//
	//cout<<"capa from shake 2-1 two routes, not yet update routes" <<endl;						
	//for (int i = 0; i < NO_BEST_ROUTE; i++)
	//{
	//	cout<< "capa[i]" << capa[i] <<' ' ;
	//}
	//cout<<endl<<endl;
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		delete[] vehic_depotCust[i];
	}

	delete[] vehic_depotCust;
	delete[] flag_module;
	return 1; //shake status is ok
}


int shake_2_1_threeR(double **(&routes), double **dist, int *demand, double *x, double *y)//routes will copy from BEST_ROUTE in this function, only shake the best route
{
	ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	int **vehic_depotCust = new int* [LOCAL_NO_ROUTE];
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		vehic_depotCust[i] = new int[SIZE]; //vehic_depotCust contains only customer with depot at front and back, for reoptimize
	}

	int no_routes = LOCAL_NO_ROUTE;
	route_CGravity = new double*[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		route_CGravity[i] = new double[2]; //2 columns consists of x and y
	}
	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		int j=0; 
		int m=1; //for vehic_depotCust
		//saiz[i] = LOCAL_BEST_ROUTE[i][2];
		saiz[i] = routes[i][2]; //added 22Apr15
		do
		{
			//routes[i][j] = LOCAL_BEST_ROUTE[i][j];
			vehic_depotCust[i][0] = SIZE;
			if (j>=3)
			{
				vehic_depotCust[i][m] = routes[i][j];//added 22Apr15
				//vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
				m++;
			}
			j++;
		//} while (LOCAL_BEST_ROUTE[i][j]!= -1);
		} while (routes[i][j]!= -1);//added 22Apr15
		//routes[i][j] = -1;
		vehic_depotCust[i][saiz[i]+1] = SIZE;

	}

	//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	//{
	//	int j=0; 
	//	int m=1; //for vehic_depotCust
	//	saiz[i] = LOCAL_BEST_ROUTE[i][2];

	//	do
	//	{
	//		routes[i][j] = LOCAL_BEST_ROUTE[i][j];
	//		vehic_depotCust[i][0] = SIZE;
	//		if (j>=3)
	//		{
	//			vehic_depotCust[i][m] = LOCAL_BEST_ROUTE[i][j];
	//			m++;

	//		}
	//		j++;
	//	} while (LOCAL_BEST_ROUTE[i][j]!= -1);
	//	routes[i][j] = -1;
	//	vehic_depotCust[i][saiz[i]+1] = SIZE;

	//}


	//**************all insertion and deletion starts from 1 to saiz because the vehic_depotCust starts from depot, end with depot**************//
	//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//

	int rand_r1 = -1, rand_r2 = -1, rand_r3 = -1, rand_r1_p = -1, rand_r2_p = -1, rand_r3_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1, cust1_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0, saiz_r3 = 0;
	int demand_c1, demand_c2, demand_c3;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1, before_cust3=-1, after_cust3=-1;
	double cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0, gain3=0.0;
	
	
	//******************************************************** Neighbour_k 6 (2-1) threeRoutes  ******************************************************************************//
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}	
	int no_flag=0;
	for (int i = 0; i < no_routes; i++)
	{
		if (saiz[i] == 1)
		{
			flag_module[vehic_depotCust[i][1]] = false;
			no_flag++;
		}
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			flag_module[vehic_depotCust[i][saiz[i]]] = false; //flag the last cust in a route false because last customer cannot be selected, this is 2-1 sequential//this is a trick!!!!!!!!!!!
			no_flag++;
		}
	}
	m6_regenerate:

		int r2_found=0;
		srand ( time(NULL) ); //seed it
		rand_r1 = (rand() % LOCAL_NO_ROUTE); 
		saiz_r1 = (int)routes[rand_r1][2];

		if((saiz_r1 == 0) || (saiz_r1 == 1))//have to test this before finding the position to avoid division by zero error
			goto m6_regenerate;

		rand_r1_p = (rand() % (saiz_r1-1))+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE; //minus one because two customers will be deleted in sequential
		cust1_r1 = vehic_depotCust[rand_r1][rand_r1_p];
		cust2_r1 = vehic_depotCust[rand_r1][rand_r1_p+1];//customer in sequential		
		demand_c1 = demand[cust1_r1];
		demand_c2 = demand[cust2_r1];

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or from flag false //make sure deletion cannot be made from empty route
			goto m6_regenerate;
				
		before_cust1 = vehic_depotCust[rand_r1][rand_r1_p-1];
		after_cust1 = vehic_depotCust[rand_r1][rand_r1_p+2];
		
		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		//{
		//	if ((i == rand_r1) || (saiz[i] == 0) ) //if the same route, skip or if the saiz = 0 because this is 2-1 cannot from empty route
		//		continue;
		//	
		//	for (int j = 1; j <= saiz[i]; j++) //swap positions are  equal to the saiz
		//	{
		//		cust1_r2 = vehic_depotCust[i][j];
		//		before_cust2 = vehic_depotCust[i][j-1];
		//		after_cust2 = vehic_depotCust[i][j+1];
		//		demand_c3 = demand[cust1_r2];
		//		rand_r2 = i;

		//		gain1 = (dist[before_cust1][cust1_r2] + dist[cust1_r2][after_cust1] + service_time[cust1_r2]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1] + service_time[cust2_r1]); //new-old
		//		gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) - (dist[before_cust2][cust1_r2] + dist[cust1_r2][after_cust2] + service_time[cust1_r2]); //new-old

		//		if ((demand_c3 <= (space_available[rand_r1]+demand_c1+demand_c2)) && (demand_c1<= (space_available[rand_r2]+demand_c3)) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
		//		{
		//			rand_r2 = i;
		//			rand_r2_p = j;
		//			r2_found = 1;
		//			goto found_r2_m6;
		//		}

		//	}
		//}
		//================================ End (Change here for guided shake) ================================///
		//++++++++++++++++++++++++++++++++ Guided shake +++++++++++++++++++++++++++++++++++++++++++++++++++++//
		bool *RFlag = new bool[no_routes];
		for (int i = 0; i < no_routes; i++)
		{
			RFlag[i] = true; //initially all routes are true
		}
		calculate_centreGravity(routes, x, y, no_routes);
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
		}
		CGravityfile <<endl;
		

		//find route based on probability
		findDist_from_CGravity (cust1_r1, x, y, dist, custRGravity, sorted_custRGravity, no_routes);
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		CGravityfile <<endl;
		

	refindR2:
		int route2 = -1;//initialize
		
		//******************************** find route based on probability ************************************//
		//double randNum = (double) rand() / double(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		for (int i = 0; i < no_routes; i++)
		{
			if (RFlag[(int)sorted_custRGravity[i][1]] == true) //if the route is true
			{
				route2 = sorted_custRGravity[i][1];
				goto goout;
			}
		}
		if (route2 == -1) //if not found route from the list
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			goto m6_regenerate;
		}
	goout:
		for (int i = 0; i < no_routes; i++)
		{
			CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		}
		//CGravityfile <<"randNum= "<<randNum<<' '<<' '<<"route2= "<<route2<<endl;
		CGravityfile <<"route2= "<<route2<<endl;
		CGravityfile << "cust is "<<cust1_r1<<' '<<"x[cust]= "<<x[cust1_r1]<<' '<<"y[cust]= "<<y[cust1_r1]<<endl;
		CGravityfile <<endl;

		if ((route2 == rand_r1) || (saiz[route2] == 0) ) //if the same route, skip or if the saiz = 0 because this is 2-1 cannot from empty route
		{
			RFlag[route2] = false; //for findDist_from_CGravity_choose_nearest use
			goto refindR2;
		}
			
		for (int j = 1; j <= saiz[route2]; j++) //swap positions are  equal to the saiz
		{
			cust1_r2 = vehic_depotCust[route2][j];
			before_cust2 = vehic_depotCust[route2][j-1];
			after_cust2 = vehic_depotCust[route2][j+1];
			demand_c3 = demand[cust1_r2];
			rand_r2 = route2;

			gain1 = (dist[before_cust1][cust1_r2] + dist[cust1_r2][after_cust1] + service_time[cust1_r2]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1] + service_time[cust2_r1]); //new-old
			gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) - (dist[before_cust2][cust1_r2] + dist[cust1_r2][after_cust2] + service_time[cust1_r2]); //new-old

			if ((demand_c3 <= (space_available[rand_r1]+demand_c1+demand_c2)) && (demand_c1<= (space_available[rand_r2]+demand_c3)) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
			{
				rand_r2 = route2;
				rand_r2_p = j;
				r2_found = 1;
				goto found_r2_m6;
			}

		}
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//

		if (r2_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			no_flag++;
			goto m6_regenerate;
		}
		
	found_r2_m6:
		//==================find r3=========================//
		int r3_found = 0;
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			if ((i == rand_r1) || (i == rand_r2) ) //if the same route, skip, saiz = 0 is ok because insertion can go to empty route
				continue;
			
			if (saiz[i] == 0)
			{
				rand_r3 = i;
				rand_r3_p = 1;
				r3_found = 1;
				goto found_r3_m6;
			}
			for (int j = 1; j <= saiz[i]+1; j++) //insertion can be made one position more than saiz
			{
				before_cust3 = vehic_depotCust[i][j-1];
				after_cust3 = vehic_depotCust[i][j]; //current position
				rand_r3 = i;

				gain3 = (dist[before_cust3][cust2_r1] + dist[cust2_r1][after_cust3] + service_time[cust2_r1]) - (dist[before_cust3][after_cust3]); //new-old

				if ((demand_c2 <= space_available[rand_r3]) && (gain3 <= distance_available[rand_r3])) //if fulfill both constraints
				{
					rand_r3 = i;
					rand_r3_p = j;
					r3_found = 1;
					goto found_r3_m6;
				}

			}
		}

		if (r3_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd //this will redo everything again!!!!!!!!!!!!!!!!!!!!
			no_flag++;
			
			if (no_flag >= SIZE)
			{	
				cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE2_0 THREE ROUTES!" <<endl;
				for (int i = 0; i < SIZE; i++)
				{
					cout<<flag_module[i]<<' ';
				}
				return 0;
			}
			goto m6_regenerate;
		}
			
	found_r3_m6:
			
		if ( (cust1_r1 == -1) || (cust2_r1 == -1) || (cust1_r2 == -1) )
			cout<< "cust1_r1 or cust1_r2 cannot be negative!!" <<endl;
			

		route_change[0] = rand_r1; //route (from)
		route_change[1] = rand_r2; //route (to)
		route_change[2] = rand_r3; //route (to)
					

	//******************************************************** END OF Neighbour_k 6 (2-1) threeRoute  ******************************************************************************//
	int customer1 = cust1_r1; 
	int customer2 = cust2_r1; //second customer in route1
	int customer3 = cust1_r2; 
	rand_r1_p = rand_r1_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	rand_r2_p = rand_r2_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	rand_r3_p = rand_r3_p-1; // because position considered before starts from 1, the format is [SIZE 1 2 3 4 ... SIZE]
	
	if ( (customer1 < 0) || (customer2 < 0) || (customer3 < 0))
	{
		cout<< "customer to be deleted cannot be negative!!!!!!" <<endl;
		cout<< "r1= " <<rand_r1 <<' '<< "r2= " <<rand_r2 <<endl;
		cout<< "r1_p= " <<rand_r1_p<<' ' << "r2_p= " <<rand_r2_p<<endl;
		cout<< "customer1 = "<< customer1 <<endl;
		cout<< "customer2 = "<< customer2 <<endl;
	}
	cout<<"===========CUST from shake 2-1 ThreeR=========" <<endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p << "]=" <<customer1<< endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p+1 << "]=" <<customer2<< endl;
	cout<< "goto route[" << rand_r2 <<"][" << rand_r2_p << "]=" <<customer3<< endl;
	cout<< "route[" << rand_r3 <<"][" << rand_r3_p << "]=" << endl;

	swap2_1_cust (dist, demand, rand_r1, rand_r1_p, routes, customer3);//2 out, 1 in for route1
	swap_in_oneCust (dist, demand, rand_r2, rand_r2_p, routes, customer1); //1 in, 1 out for route2
	insert_one_cust (dist, demand, rand_r3, rand_r3_p, routes, customer2); //1 in for route3

	double total_cost = 0.0;

	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		route_cost[i] = routes[i][1]; //update route_cost (global variable)  
		total_cost += routes[i][1]; //update total_cost (global variable) 
	}
	
	//================= update global variable (added on 26Feb2015) ===============================//
	//total_demand[rand_r1] = total_demand[rand_r1] - demand_c1 - demand_c2 + demand_c3; 
	//total_demand[rand_r2] = total_demand[rand_r2] + demand_c1 - demand_c3;
	//total_demand[rand_r3] = total_demand[rand_r3] + demand_c2;
	//saiz[rand_r1] = saiz[rand_r1] - 2 + 1;
	//saiz[rand_r3] = saiz[rand_r3] + 1;
	//distance_available[rand_r1] = DISTANCE - route_cost[rand_r1];
	//distance_available[rand_r2] = DISTANCE - route_cost[rand_r2];
	//distance_available[rand_r3] = DISTANCE - route_cost[rand_r3];
	//space_available[rand_r1] = CAPACITY - total_demand[rand_r1];
	//space_available[rand_r2] = CAPACITY - total_demand[rand_r2];
	//space_available[rand_r3] = CAPACITY - total_demand[rand_r3];
	cout<<"============Routes after shake in 2-1 three routes============"<<endl;
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		for (int k = 0; k <= routes[i][2]+3; k++)
		{
			cout << routes[i][k] << ' ';
		}
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	//find_capacity(capa, demand);
	
	//cout<<"capa from shake 2-1 three routes, not yet update routes" <<endl;						
	//for (int i = 0; i < NO_BEST_ROUTE; i++)
	//{
	//	cout<< "capa[i]" << capa[i] <<' ' ;
	//}
	//cout<<endl<<endl;
	
	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
	{
		delete[] vehic_depotCust[i];
	}

	delete[] vehic_depotCust;
	delete[] flag_module;
	return 1; //shake status is ok
}


void check_capacity (int *demand, int *(&capa), int r1, int r2, int r3, int c_f_r1, int c2_f_r1, int c_f_r2, int c2_f_r2, int (&status))
{
	int capa1 = 0, capa2 = 0, capa3 = 0;

	if (status == 1) //(1-0)
	{
		capa1 = capa[r1] - demand[c_f_r1];
		capa2 = capa[r2] + demand[c_f_r1];
		if ( (capa1 > CAPACITY) || (capa2 > CAPACITY) )
			status = 0; //not OK

		else
			status = 1; //OK
	}
	 
	else if (status == 2) //(1-1)
	{
		capa1 = capa[r1] - demand[c_f_r1] + demand[c_f_r2];
		capa2 = capa[r2] - demand[c_f_r2] + demand[c_f_r1];
		if ( (capa1 > CAPACITY) || (capa2 > CAPACITY) )
			status = 0;

		else
			status = 1;
	}

	if (status == 3) //(2-1) or 2-1-0
	{
		capa1 = capa[r1] - demand[c_f_r1] - demand[c2_f_r1] + demand[c_f_r2];
		capa2 = capa[r2] - demand[c_f_r2] + demand[c_f_r1];
		capa3 = capa[r3] + demand[c2_f_r1];
		if ( (capa1 > CAPACITY) || (capa2 > CAPACITY) )
			status = 0;

		else
			status = 1;
	}

}


void reopt_singleRoute(int r, double **dist, int *demand, double *cost_of_removing, double **(&info_1_0), double **(&same_r_info),int **(&vehicle), double **(&gain_matrix_1_0))
{
	int no_routes = LOCAL_NO_ROUTE;
	int element, before_ele, after_ele, from_route, to_route, before_pos, after_pos, from_position;

	int element1, element2, before_ele1, before_ele2, after_ele1, after_ele2, before_pos1, before_pos2, after_pos1, after_pos2;

	double cost_with_i, cost_without_i, gain=0.0;
	double cost_with_j, cost_without_j, r1_change, r2_change;

	int element1f;
	int element3, before_ele3, after_ele3;
	
	std::vector<int> vector_r1;
	std::vector<int> vector_r2;
	std::vector<int> vector_r3;

	total_demand[r] = 0.0; //total_demand is global variable //reinitialize total demand  of affected route to zero

	//to calculate total demand for each route
	
	for (int q = 1; q < saiz[r] + 1; q++)  // consider each customer 
	{
		total_demand[r] = total_demand[r] + demand[vehicle[r][q]];
	}
	

	//to calculate space available for each route

	for (int i = 0; i < no_routes; i++)
	{
		space_available[i] = CAPACITY - (total_demand[i]);
		//cout << i << ' ' << ' ' << space_available[i] <<endl;
	}

	//reinitialize the gain for affected routes to be -1
	for (int t = 0; t < no_routes; t++)//////////////////////////////////////////////////////////////////////////////////////////////////////
	{
		if (r<t)
		{
			gain_matrix_1_0[r][t] = INT_MIN;
			for (int u = 1; u < 9; u++)
			{
				info_1_0[(int)gain_matrix_1_0[t][r]][u] = -1;

			}
		}

		else if (t<r)
		{
			gain_matrix_1_0[t][r] = INT_MIN;
			for (int u = 1; u < 9; u++)
			{
				info_1_0[(int)gain_matrix_1_0[r][t]][u] = -1;
			}
		}
		gain_matrix_1_0[r][r] = INT_MIN; //initialize diagonal

	}
	gain_matrix_1_0[r][r] = INT_MIN; //reiniatilize //ONLY DO THIS ONCE!!!!!!!!!!!!!!!!!!!!!!!!!!
	for (int u = 1; u < 9; u++)
	{
		same_r_info[r][u] = -1;
	}
	///////////////////////////////////////////////////////// 1 - 0 ///////////////////////////////////////////////////////////////
	//////////////////================================== SAME ROUTE =========================////////////////////////////////////
	if (saiz[r]==0) //if it is empty route, cutomer from other route may go to empty route
		goto end_of_1_0_first;
	for (int j = 1; j <= saiz[r]; j++) //which one to delete
	{
		double available_dist_r2 = distance_available[r];
		//copy a temporary route
		int b=0;
		int* temp_r = new int[saiz[r]+2];
		for (int a = 0; a < saiz[r] + 2; a++)
		{
			if (a == j)
			{
				continue; //dont copy the element, assume the element to be deleted
			}

			else
			{
				temp_r[b] = vehicle[r][a];
				b++;
			}
		}
		element = vehicle[r][j];
		before_ele = vehicle[r][j - 1];   //initially, unless stated otherwise
		after_ele = vehicle[r][j + 1];
		cost_without_i = cost_of_removing[vehicle[r][j]];//cost of r1 without i
		for (int n = 1; n < saiz[r] + 1; n++) //has 1 customer less because it has been deleted
		{
			
			if (j == n) //originally from this position, so noneed to consider
				continue; 
			else
			{
				before_pos = temp_r[n - 1];
				after_pos = temp_r[n];  //noneed to add 1 because it is insert in between n-1 and n
			}	
			
			if ((NR_FLAG[element][before_pos] == false) || (NR_FLAG[element][after_pos] == false)) //if insertion cannot between cust before and cust after
			{
				continue;
			}				
	
			cost_with_i = dist[before_pos][element] + dist[element][after_pos] + service_time[element]- dist[before_pos][after_pos]; //cost of r2 with i //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						
						
			gain = cost_without_i - cost_with_i;
			//if (abs(gain) > available_dist_r2) //if negative gain, it will not be recorded anyway, so this code can be omitted actually
			//{
			//	continue;
			//}
					
			if (gain-gain_matrix_1_0[r][r] > 0.01)//gain > gain_matrix_1_0[r][r]
			{
				gain_matrix_1_0[r][r] = gain;
				same_r_info[r][0] = r;
				same_r_info[r][1] = gain;
				same_r_info[r][2] = r; //from route
				same_r_info[r][3] = r; //to route, i=m here
				same_r_info[r][4] = j; //from which position
				same_r_info[r][5] = n; //to which position //assume modified route, customer has been deleted
				same_r_info[r][6] = -1; 
				same_r_info[r][7] = -1;
				same_r_info[r][8] = 1; //1 is (1-0)

			}	
		}
		delete temp_r;
	}//end of j


	///////////////// ============================== DIFERENT ROUTES ======================================//////////////////////
	//from 2 routes to every position=======================update row============================= ///////////////////////////////////

	for (int j = 1; j < saiz[r] + 1; j++)  // consider each customer (from)
	{
		element = vehicle[r][j];			//elemet to be inserted into other route
		from_route = r;
		from_position = j;
		//cout << element;

		for (int m = 0; m < no_routes; m++)  //insert (to) which route
		{
			if ((m == r) || (demand[element] > space_available[m])) //if customer already in the route or demand exceed avaialable space in route
			{
				continue;
				//m++;
				//goto mylabel;
			}


			for (int n = 1; n < saiz[m] + 2; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
			{
				double available_dist_r2 = distance_available[m];
				to_route = m;
				before_ele = vehicle[r][j - 1];   //initially, unless stated otherwise
				after_ele = vehicle[r][j + 1];
				before_pos = vehicle[m][n - 1];
				after_pos = vehicle[m][n];  //noneed to add 1 because it is insert in between n-1 and n
				if ((NR_FLAG[element][before_pos] == false) || (NR_FLAG[element][after_pos] == false)) //if insertion cannot between cust before and cust after
				{
					continue;
				}
				cost_without_i = cost_of_removing[vehicle[r][j]];//cost of r1 without i
				cost_with_i = dist[before_pos][element] + dist[element][after_pos] + service_time[element] - dist[before_pos][after_pos]; //cost of r2 with i 

				if (cost_with_i > available_dist_r2)
				{
					//cout<<"2) ele1= " << element1 << "ele1f= "<< element1f <<"ele2= " << element2 <<endl;
					continue;
				}
				gain = cost_without_i - cost_with_i;


				if (r < m)
				{
					if (gain-gain_matrix_1_0[r][m] > 0.01)//gain > gain_matrix_1_0[r][m]
					{
						gain_matrix_1_0[r][m] = gain;
						//info_1_0[gain_matrix_1_0[m][from1]][0] = gain_matrix_1_0[m][from1];
						info_1_0[(int)gain_matrix_1_0[m][r]][1] = gain;
						info_1_0[(int)gain_matrix_1_0[m][r]][2] = r;
						info_1_0[(int)gain_matrix_1_0[m][r]][3] = m;
						info_1_0[(int)gain_matrix_1_0[m][r]][4] = j;
						info_1_0[(int)gain_matrix_1_0[m][r]][5] = n;
						info_1_0[(int)gain_matrix_1_0[m][r]][6] = -1;
						info_1_0[(int)gain_matrix_1_0[m][r]][7] = -1;
						info_1_0[(int)gain_matrix_1_0[m][r]][8] = 1; //1 is (1-0)

					}
				}

				else if (r > m)
				{
					if (gain-gain_matrix_1_0[m][r] > 0.01)//gain > gain_matrix_1_0[m][r]
					{
						gain_matrix_1_0[m][r] = gain;
						//info_1_0[gain_matrix_1_0[m][from1]][0] = gain_matrix_1_0[from1][m];
						info_1_0[(int)gain_matrix_1_0[r][m]][1] = gain;
						info_1_0[(int)gain_matrix_1_0[r][m]][2] = r;
						info_1_0[(int)gain_matrix_1_0[r][m]][3] = m;
						info_1_0[(int)gain_matrix_1_0[r][m]][4] = j;
						info_1_0[(int)gain_matrix_1_0[r][m]][5] = n;
						info_1_0[(int)gain_matrix_1_0[r][m]][6] = -1;
						info_1_0[(int)gain_matrix_1_0[r][m]][7] = -1;
						info_1_0[(int)gain_matrix_1_0[r][m]][8] = 1; //1 is (1-0)
					}
				}
			}//end for n
		}//end for m
	}//end for j
	//cout << i << ' '<< element << ' '<< old_route[i] << ' ' <<old_position[i]<< ' ' << ' '<< new_route[i] << ' ' << new_position[i] << ' '<< ' '<< gain_array[i] << endl;

end_of_1_0_first: //customer from other route may go to empty route
	//######################################update gain matrix################################################//
	//from every position to 2 routes=======================update column============================= ///////////////////////////////////

	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from)
	{
		if (saiz[i]==0) //if it is empty route
			continue;
		for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from)
		{
			element = vehicle[i][j];			//element to be inserted into other route
			from_route = i;
			from_position = j;
			//cout << element;

			double available_dist_r2 = distance_available[r];
			if ((i == r) || (demand[element] > space_available[r])) //if demand exceed avaialable space in route && differnt route, if same route, capacity exceed stil have to find best gain
			{
				continue;
		
			}

			for (int n = 1; n < saiz[r] + 2; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
			{
				to_route = r;
				before_ele = vehicle[i][j - 1];   //initially, unless stated otherwise
				after_ele = vehicle[i][j + 1];
				before_pos = vehicle[r][n - 1];
				after_pos = vehicle[r][n];  //noneed to add 1 because it is insert in between n-1 and n
				if ((NR_FLAG[element][before_pos] == false) || (NR_FLAG[element][after_pos] == false)) //if insertion cannot between cust before and cust after
				{
					continue;
				}
				cost_without_i = cost_of_removing[vehicle[i][j]];//cost of r1 without i
				cost_with_i = dist[before_pos][element] + dist[element][after_pos] + service_time[element]- dist[before_pos][after_pos]; //cost of r2 with i 

				if (cost_with_i > available_dist_r2)
				{
					continue;
				}
				gain = cost_without_i - cost_with_i;

				if (i < r)
				{
					if (gain-gain_matrix_1_0[i][r] > 0.01)//gain > gain_matrix_1_0[i][r]
					{
						gain_matrix_1_0[i][r] = gain;
						//info_1_0[(int)gain_matrix_1_0[i][to1]][0] = gain_matrix_1_0[to1][i];
						info_1_0[(int)gain_matrix_1_0[r][i]][1] = gain;
						info_1_0[(int)gain_matrix_1_0[r][i]][2] = i;
						info_1_0[(int)gain_matrix_1_0[r][i]][3] = r;
						info_1_0[(int)gain_matrix_1_0[r][i]][4] = j;
						info_1_0[(int)gain_matrix_1_0[r][i]][5] = n;
						info_1_0[(int)gain_matrix_1_0[r][i]][6] = -1;
						info_1_0[(int)gain_matrix_1_0[r][i]][7] = -1;
						info_1_0[(int)gain_matrix_1_0[r][i]][8] = 1; //1 is (1-0)

					}
				}
				else if (i > r)
				{
					if (gain-gain_matrix_1_0[r][i] > 0.01)//gain > gain_matrix_1_0[r][i]
					{
						gain_matrix_1_0[r][i] = gain;
						//info_1_0[(int)gain_matrix_1_0[to1][i]][0] = gain_matrix_1_0[i][to1];
						info_1_0[(int)gain_matrix_1_0[i][r]][1] = gain;
						info_1_0[(int)gain_matrix_1_0[i][r]][2] = i;
						info_1_0[(int)gain_matrix_1_0[i][r]][3] = r;
						info_1_0[(int)gain_matrix_1_0[i][r]][4] = j;
						info_1_0[(int)gain_matrix_1_0[i][r]][5] = n;
						info_1_0[(int)gain_matrix_1_0[i][r]][6] = -1;
						info_1_0[(int)gain_matrix_1_0[i][r]][7] = -1;
						info_1_0[(int)gain_matrix_1_0[i][r]][8] = 1;//1 is (1-0)
					}
				}
			}//end for n
		}//end for j
	}

	///////////////////////////////////////////////////////// 1 - 1 ///////////////////////////////////////////////////////////////

	//////////////////////////////////============================= SAME ROUTE ===============================================//////////////////////
	//gain_matrix_1_0[r][r] = INT_MIN; //reiniatilize //CHANGED THIS ON 7MAC2015 to comment
	//for (int u = 1; u < 9; u++)
	//{
	//	same_r_info[r][u] = -1;
	//}

	//copy route in vector
	if (saiz[r] == 0) //finish 1_1 because no customer could be removed from empty route
		goto end_of_1_1;
	else if (saiz[r]==1) //no reshuffle in the same route but customer from other route may go in
		goto end_of_1_1_first;
	for (int i = 0; i < saiz[r] + 2; i++) //first and last is depot
	{
		vector_r3.push_back(vehicle[r][i]);
	}
	
	for (int j = 1; j < saiz[r] ; j++) //from which position
	{
		element1 = vector_r3[j];
		before_ele1 = vector_r3[j-1];
		after_ele1 = vector_r3[j+1];
		for (int k = j+1; k <= saiz[r]; k++)//to which position
		{
			element2 = vector_r3[k];
			before_ele2 = vector_r3[k-1];
			after_ele2 = vector_r3[k+1];
			
			if ((NR_FLAG[element1][before_ele2] == false) || (NR_FLAG[element1][after_ele2] == false) || (NR_FLAG[element2][before_ele1] == false) || (NR_FLAG[element2][after_ele1] == false))
			{
				continue;
			}

			//same route noneed to consider service_time[]	
			if (k==(j+1)) // if consecutive customer, delete 3 arcs and insert 3 arcs
			{
				cost_without_i = dist[before_ele1][element1] + dist[element1][after_ele1] + dist[element2][after_ele2]; //route_cost is global variable //minus 4 arcs 
				cost_with_i = dist[before_ele1][element2] + dist[element2][element1] +dist[element1][after_ele2];
			}
			else  // if non-consecutive customer, delete 4 arcs and insert 4 arcs
			{
				cost_without_i = dist[before_ele1][element1] + dist[element1][after_ele1] + dist[before_ele2][element2] + dist[element2][after_ele2]; //route_cost is global variable //minus 4 arcs 
				cost_with_i = dist[before_ele1][element2] + dist[element2][after_ele1] +dist[before_ele2][element1] +dist[element1][after_ele2];
			}
			gain = cost_without_i - cost_with_i; //gain = old-new
			//if (abs(gain) > distance_available[r])//gain is negative, change to positive using abs//if negative gain, it will not be recorded anyway, so this code can be omitted actually
			//{
			//	continue;
			//}
			if (gain-gain_matrix_1_0[r][r] > 0.01)//gain > gain_matrix_1_0[r][r]
			{
				gain_matrix_1_0[r][r] = gain;
				same_r_info[r][0] = r;
				same_r_info[r][1] = gain;
				same_r_info[r][2] = r; //from route
				same_r_info[r][3] = r; //to route, i=m here
				same_r_info[r][4] = j; //from which position
				same_r_info[r][5] = k; //this is exactly the position to be swapped !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! insertion need to be careful
				same_r_info[r][6] = -1; 
				same_r_info[r][7] = -1;
				same_r_info[r][8] = 2; //2 is (1-1)

			}	
		}		
	}
	vector_r3.clear();
	
end_of_1_1_first:;

	//////////////////////////////////==============================  DIFFERENT ROUTES ==========================================////////////////////


	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//================================================(from) 2 affected routes to other routes (update ROW)================================================================

	for (int j = 1; j < saiz[r] + 1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
	{
		//int space_r1 = space_available[i];
		element1 = vehicle[r][j];			//element to be inserted into other route
		from_route = r;
		from_position = j;
		//cout << element;

		for (int m = 0; m < no_routes; m++)  //consider each and other routes (from R2)
		{
			if ((r == m) || (saiz[m]==0)) //|| (demand[element2] > space_r1)) //if customer already in the route or demand exceed avaialable space in route
			{
				continue;
			}
		
			for (int n = 1; n < saiz[m] + 1; n++) //which customer in r2 to remove
			{
				element2 = vehicle[m][n];			//element to be removed from r2

				if ((NR_FLAG[element1][vehicle[m][n - 1]] == false) || (NR_FLAG[element1][vehicle[m][n + 1]] == false) || (NR_FLAG[element2][vehicle[r][j - 1]] == false) || (NR_FLAG[element2][vehicle[r][j + 1]] == false))
				{
					continue;
				}

				int space_r1 = space_available[r];
				int space_r2 = space_available[m];
				space_r1 = space_r1 + demand[element1];
				space_r2 = space_r2 + demand[element2];	
				if ((demand[element2] > space_r1) || (demand[element1] > space_r2))
					continue;

				//==================================================copy r1 without customer j=================================================================//
				vector_r1.clear();
				for (int a = 0; a < saiz[r] + 2; a++)
				{
					vector_r1.push_back(vehicle[r][a]);
				}
				vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot
								
				//==================================================copy r2 without customer n=================================================================//
				vector_r2.clear();
				for (int a = 0; a < saiz[m] + 2; a++)
				{
					vector_r2.push_back(vehicle[m][a]);

				}
				vector_r2.erase(vector_r2.begin() + n); //first one is depot		
								
				double available_dist_r1 = distance_available[r];
				double available_dist_r2 = distance_available[m];
				to_route = m;
					
				//############################################## To Insert ##########################################################////////////////////
				for (int p = 1; p < vector_r2.size(); p++) //from r1, insert to r2
				{
					for (int q = 1; q < vector_r1.size(); q++) //from r2, insert to r1
					{
						before_ele1 = vehicle[r][j - 1];   //initially, unless stated otherwise
						after_ele1 = vehicle[r][j + 1];

						before_ele2 = vehicle[m][n - 1];   //initially, unless stated otherwise
						after_ele2 = vehicle[m][n + 1];

						before_pos1 = vector_r1[q - 1];
						after_pos1 = vector_r1[q];  //noneed to add 1 because it is insert in between n-1 and n
						before_pos2 = vector_r2[p - 1];
						after_pos2 = vector_r2[p];  //noneed to add 1 because it is insert in between n-1 and n

						cost_without_i = cost_of_removing[vehicle[r][j]];
						cost_without_j = cost_of_removing[vehicle[m][n]];


						cost_with_i = dist[before_pos1][element2] + dist[element2][after_pos1] + service_time[element2] - dist[before_pos1][after_pos1]; //element from r2 to r1 or insert ele2 to r1
						cost_with_j = dist[before_pos2][element1] + dist[element1][after_pos2] + service_time[element1] - dist[before_pos2][after_pos2]; //element from r1 to r2 or insert ele1 to r2

						if ((cost_with_i > available_dist_r1 + cost_without_i) || (cost_with_j > available_dist_r2 + cost_without_j))
							continue;

						gain = (cost_without_i + cost_without_j) - (cost_with_i + cost_with_j);
						//cout << "hhhhhhhhhh " << r1_change << ' ' << r2_change <<endl;

						//myfile << element << ' ' << vehic[m][n] << ' ' << i << ' ' << m << ' ' << j << ' ' << n << ' ' << gain << endl;

						if (r < m)
						{
							if (gain-gain_matrix_1_0[r][m] > 0.01)//gain > gain_matrix_1_0[r][m]
							{

								gain_matrix_1_0[r][m] = gain;

								//info_1_0[number_matrix_1_1[k][m]][0] = gain_matrix_1_1[m][k];
								info_1_0[(int)gain_matrix_1_0[m][r]][1] = gain;
								info_1_0[(int)gain_matrix_1_0[m][r]][2] = r;
								info_1_0[(int)gain_matrix_1_0[m][r]][3] = m;
								info_1_0[(int)gain_matrix_1_0[m][r]][4] = j;
								info_1_0[(int)gain_matrix_1_0[m][r]][5] = p;
								info_1_0[(int)gain_matrix_1_0[m][r]][6] = n;
								info_1_0[(int)gain_matrix_1_0[m][r]][7] = q;
								info_1_0[(int)gain_matrix_1_0[m][r]][8] = 2;//2 is (1-1)


							}//end if 
						}//end if (k < m)
						else if (r > m)
						{
							if (gain-gain_matrix_1_0[m][r] > 0.01)//gain > gain_matrix_1_0[m][r]
							{

								gain_matrix_1_0[m][r] = gain;

								//info_1_0[gain_matrix_1_0[k][m]][0] = gain_matrix_1_0[k][m];
								info_1_0[(int)gain_matrix_1_0[r][m]][1] = gain;
								info_1_0[(int)gain_matrix_1_0[r][m]][2] = r;
								info_1_0[(int)gain_matrix_1_0[r][m]][3] = m;
								info_1_0[(int)gain_matrix_1_0[r][m]][4] = j;
								info_1_0[(int)gain_matrix_1_0[r][m]][5] = p;
								info_1_0[(int)gain_matrix_1_0[r][m]][6] = n;
								info_1_0[(int)gain_matrix_1_0[r][m]][7] = q;
								info_1_0[(int)gain_matrix_1_0[r][m]][8] = 2;//2 is (1-1)
							}//end if 
						}//end if (r > m)
					}//end for q
				}//end for p
			}//end for n
		}//end for m
	}//end for j

	//================================================(To) From other routes to 2 affected routes (update column)================================================================
	//for (int i = 0; i < no_routes; i++)  //consider each and other routes (from R1)
	//{
	//	if (saiz[i]==0)  //if it is empty route
	//		continue;
	//	for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
	//	{
	//		//int space_r1 = space_available[i];
	//		element1 = vehicle[i][j];			//element to be inserted into other route
	//		from_route = i;
	//		from_position = j;
	//		//cout << element;

	//		//==================================================copy r1 without customer j=================================================================//
	//		vector_r1.clear();
	//		for (int a = 0; a < saiz[i] + 2; a++)
	//		{
	//			vector_r1.push_back(vehicle[i][a]);
	//		}
	//		vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot

	//		if (i == r)// || (demand[element2] > space_available[m])) //if customer already in the route or demand exceed avaialable space in route
	//		{
	//			continue;
	//			//m++;
	//			//goto mylabel;
	//		}

	//		else
	//		{

	//			for (int n = 1; n < saiz[r] + 1; n++) //which customer in r2 to remove
	//			{
	//				element2 = vehicle[r][n];			//element to be removed from r2
	//				if ((NR_FLAG[element1][vehicle[r][n - 1]] == false) || (NR_FLAG[element1][vehicle[r][n + 1]] == false) || (NR_FLAG[element2][vehicle[i][j - 1]] == false) || (NR_FLAG[element2][vehicle[i][j + 1]] == false))
	//				{
	//					continue;
	//				}
	//				
	//				int space_r1 = space_available[i]; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//				space_r1 = space_r1 + demand[element1];

	//				if (demand[element2] > space_r1)
	//					continue;

	//				else
	//				{

	//					//############################################## To Insert ##########################################################////////////////////
	//					//int space_r1 = space_available[i];//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//					int space_r2 = space_available[r];
	//					//space_r1 = space_r1 + demand[element1];
	//					space_r2 = space_r2 + demand[element2];

	//					double available_dist_r1 = distance_available[i];
	//					double available_dist_r2 = distance_available[r];

	//					if ((demand[element1] > space_r2) || (demand[element2] > space_r1))
	//					{
	//						goto nextcust_1_1_second;
	//					}

	//					else
	//					{
	//						vector_r2.clear();
	//						for (int a = 0; a < saiz[r] + 2; a++)
	//						{
	//							vector_r2.push_back(vehicle[r][a]);

	//						}

	//						vector_r2.erase(vector_r2.begin() + n); //first one is depot
	//						//space_r2 = space_r2 + demand[element2];
	//						//cout<<" 2. shshshshshsh" <<space_r2;

	//						//############################################## To Insert ##########################################################////////////////////
	//						for (int p = 1; p < vector_r2.size(); p++) //from r1, insert to r2
	//						{
	//							for (int q = 1; q < vector_r1.size(); q++) //from r2, insert to r1
	//							{
	//								/*int space_r1 = space_available[i];
	//								int space_r2 = space_available[m];
	//								space_r1 = space_r1 + demand[element1];
	//								space_r2 = space_r2 + demand[element2];*/

	//								double available_dist_r1 = distance_available[i];
	//								double available_dist_r2 = distance_available[r];

	//								to_route = r;
	//								before_ele1 = vehicle[i][j - 1];   //initially, unless stated otherwise
	//								after_ele1 = vehicle[i][j + 1];

	//								before_ele2 = vehicle[r][n - 1];   //initially, unless stated otherwise
	//								after_ele2 = vehicle[r][n + 1];

	//								before_pos1 = vector_r1[q - 1];
	//								after_pos1 = vector_r1[q];  //noneed to add 1 because it is insert in between n-1 and n
	//								before_pos2 = vector_r2[p - 1];
	//								after_pos2 = vector_r2[p];  //noneed to add 1 because it is insert in between n-1 and n
	//								//cout << "hhhhhhhhhh " << before_ele << ' ' << after_ele << ' ' << before_pos << ' ' << after_pos <<endl;

	//								r1_change = dist[element1][before_ele1] + dist[element1][after_ele1] - dist[before_ele1][after_ele1];
	//								r2_change = dist[element2][before_ele2] + dist[element2][after_ele2] - dist[before_ele2][after_ele2];
	//								cost_without_i = cost_of_removing[vehicle[i][j]]; //remove ele1 from r1
	//								cost_without_j = cost_of_removing[vehicle[r][n]]; //remove ele2 from r2

	//								cost_with_i = dist[before_pos1][element2] + dist[element2][after_pos1] + service_time[element2] - dist[before_pos1][after_pos1]; //element from r2 to r1 or cost of r1 with ele2  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//								cost_with_j = dist[before_pos2][element1] + dist[element1][after_pos2] + service_time[element1] - dist[before_pos2][after_pos2]; //element from r1 to r2 or cost of r2 with ele1

	//								if ((cost_with_i > available_dist_r1 + cost_without_i) || (cost_with_j > available_dist_r2 + cost_without_j))
	//									continue;

	//								gain = (cost_without_i + cost_without_j) - (cost_with_i + cost_with_j);
	//								//cout << "hhhhhhhhhh " << r1_change << ' ' << r2_change <<endl;

	//								//myfile << element << ' ' << vehic[m][n] << ' ' << i << ' ' << m << ' ' << j << ' ' << n << ' ' << gain << endl;

	//								if (i < r)
	//								{
	//									if (gain-gain_matrix_1_0[i][r] > 0.01)//gain > gain_matrix_1_0[i][r]
	//									{
	//										gain_matrix_1_0[i][r] = gain;

	//										//info[gain_matrix_1_0[i][r]][0] = gain_matrix_1_0[i][r];
	//										info_1_0[(int)gain_matrix_1_0[r][i]][1] = gain;
	//										info_1_0[(int)gain_matrix_1_0[r][i]][2] = i;
	//										info_1_0[(int)gain_matrix_1_0[r][i]][3] = r;
	//										info_1_0[(int)gain_matrix_1_0[r][i]][4] = j;
	//										info_1_0[(int)gain_matrix_1_0[r][i]][5] = p;
	//										info_1_0[(int)gain_matrix_1_0[r][i]][6] = n;
	//										info_1_0[(int)gain_matrix_1_0[r][i]][7] = q;
	//										info_1_0[(int)gain_matrix_1_0[r][i]][8] = 2;//2 is (1-1)


	//									}//end if
	//								}//end if (i < r)

	//								else if (i > r)
	//								{
	//									if (gain-gain_matrix_1_0[r][i] > 0.01)//gain > gain_matrix_1_0[r][i]
	//									{
	//										gain_matrix_1_0[r][i] = gain;

	//										//info[gain_matrix_1_1[i][m]][0] = gain_matrix_1_0[m][i];
	//										info_1_0[(int)gain_matrix_1_0[i][r]][1] = gain;
	//										info_1_0[(int)gain_matrix_1_0[i][r]][2] = i;
	//										info_1_0[(int)gain_matrix_1_0[i][r]][3] = r;
	//										info_1_0[(int)gain_matrix_1_0[i][r]][4] = j;
	//										info_1_0[(int)gain_matrix_1_0[i][r]][5] = p;
	//										info_1_0[(int)gain_matrix_1_0[i][r]][6] = n;
	//										info_1_0[(int)gain_matrix_1_0[i][r]][7] = q;
	//										info_1_0[(int)gain_matrix_1_0[i][r]][8] = 2;//2 is (1-1)


	//									}//end if
	//								}//end if (m < i)

	//							}//end for q
	//						}//end for p

	//					}//end else

	//				}//end else
	//			nextcust_1_1_second:;
	//			}//end for n

	//		}//end else
	//		//cout << i << ' '<< element << ' '<< old_route[i] << ' ' <<old_position[i]<< ' ' << ' '<< new_route[i] << ' ' << new_position[i] << ' '<< ' '<< gain_array[i] << endl;
	//	}//end for j
	//}//end for i
end_of_1_1:; //no customer could be removed from empty route, (1-1) cannot be performed

	///////////////////////////////////////////////////////// 2 - 1 //////////////////////////////////////////////////////
	vector_r1.clear();
	vector_r2.clear();


	///////////////////////////////=================================== SAME ROUTE =================================////////////////////////
	//gain_matrix_1_0[r][r] = INT_MIN; //reiniatilize
	//for (int u = 1; u < 9; u++)
	//{
	//	same_r_info[r][u] = -1;
	//}
	if(saiz[r]==0) 
		goto end_of_2_1;
	else if ((saiz[r]==1) || (saiz[r]==2)) //no reshuffle can be made
		goto end_of_2_1_first;
	vector_r3.clear();
	
	//copy route in vector
	for (int i = 0; i < saiz[r] + 2; i++) //first and last is depot
	{
		vector_r3.push_back(vehicle[r][i]);
	}
	
	for (int j = 1; j <= saiz[r]-1 ; j++) //from which position, take in pair, so until saiz-1
	{
		element1 = vector_r3[j];
		element2 = vector_r3[j+1];
		before_ele1 = vector_r3[j-1];
		after_ele2 = vector_r3[j+2];
		for (int k = 1; k <= saiz[r]; k++) //which position to swap with, take one customer
		{
			if ((j==k) || ((j+1) ==k) )
			{
				continue;
			}
			element3 = vector_r3[k];
			before_ele3 = vector_r3[k-1];
			after_ele3 = vector_r3[k+1];

			if ( (NR_FLAG[element1][before_ele3]==false)||(NR_FLAG[element2][after_ele3]==false)||(NR_FLAG[element3][before_ele1]==false)||(NR_FLAG[element3][after_ele2]==false) )
				continue;
			//same route noneed to consider service_time[]	
			if (k > j) //if single customer is after pair customer
			{
					if (after_ele2 == element3) //in consecutive, so minus 3 arcs, for eg: 4 2 3 7 5 1 (swap pair 2 3 with 7) 
					{
						cost_without_i = dist[before_ele1][element1] + dist[element2][after_ele2] + dist[element3][after_ele3]; //route_cost is global variable //minus 4 arcs 
						cost_with_i = dist[before_ele1][element3]  + dist[element3][element1] +dist[element2][after_ele3];
					}
				
					else 
					{
						cost_without_i = dist[before_ele1][element1] + dist[element2][after_ele2] + dist[before_ele3][element3] + dist[element3][after_ele3]; //route_cost is global variable //minus 4 arcs 
						cost_with_i = dist[before_ele1][element3] + dist[element3][after_ele2] +dist[before_ele3][element1] +dist[element2][after_ele3];
					}

			}

			else //if single customer is before pair customer
			{
				if(element3 == before_ele1) //in consecutive, so minus 3 arcs, for eg: 4 2 3 7 5 1 (swap 2 with pair 3 7) 
				{
					cost_without_i = dist[before_ele3][element3] + dist[before_ele1][element1] + dist[element2][after_ele2];
					cost_with_i = dist[before_ele3][element1] + dist[element2][element3] + dist[element3][after_ele2];
				}

				else //all other sequence, so minus 4 arcs? for eg: 4 2 3 7 5 1 (swap 4 with pair 7 5) 
				{
					cost_without_i =  dist[before_ele3][element3] + dist[element3][after_ele3] +dist[before_ele1][element1] +dist[element2][after_ele2];
					cost_with_i = dist[before_ele3][element1] + dist[element2][after_ele3] +dist[before_ele1][element3] +dist[element3][after_ele2];
				}

			}

			gain = cost_without_i - cost_with_i; //gain = old-new
			//if (abs(gain) > distance_available[r])//gain is negative, change to positive using abs//if negative gain, it will not be recorded anyway, so this code can be omitted actually
			//{
			//	continue;
			//}
			if (gain-gain_matrix_1_0[r][r] > 0.01)//gain > gain_matrix_1_0[r][r]
			{
				gain_matrix_1_0[r][r] = gain;
				same_r_info[r][0] = r;
				same_r_info[r][1] = gain;
				same_r_info[r][2] = r; //from route
				same_r_info[r][3] = r; //to route, i=m here
				same_r_info[r][4] = j; //from which position
				same_r_info[r][5] = k; //this is exactly the position to be swapped !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! insertion need to be careful
				same_r_info[r][6] = -1; 
				same_r_info[r][7] = -1;
				same_r_info[r][8] = 3; //3 is (2-1)

			}	
		}
			
	}
	vector_r3.clear();

end_of_2_1_first:

	//////////////////////////////////////// ====================== DIFFERENT ROUTES =========================================////////////////////////////
	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//============================================(From) Affected routes: R1 and R2 to other routes===========================================================
	for (int j = 1; j < saiz[r]; j++)  // consider each customer (from R1), start from 1 because [0] is depot
	{
		//int space_r1 = space_available[i];
		element1 = vehicle[r][j];			//element to be inserted into other route
		element1f = vehicle[r][j + 1];
		from_route = r;
		from_position = j;
		//cout << element;

		for (int m = 0; m < no_routes; m++)  //consider each and other routes (from R2)
		{
			if ((r == m)|| (saiz[m]==0) ||(saiz[m]==1))  //|| (demand[element2] > space_r1)) //if customer already in the route or demand exceed avaialable space in route
			{
				continue;   //to make it faster get out of loop
			}
	
			for (int n = 1; n < saiz[m] + 1; n++) //which customer in r2 to remove
			{
				element2 = vehicle[m][n];			//elemet to be removed from r2
				if ( (NR_FLAG[element1][vehicle[m][n-1]]==false)||(NR_FLAG[element1f][vehicle[m][n+1]]==false)||(NR_FLAG[element2][vehicle[r][j-1]]==false)||(NR_FLAG[element2][vehicle[r][j+2]]==false) )
					continue;

				int space_r1 = space_available[r];
				int space_r2 = space_available[m];
				space_r1 = space_r1 + demand[element1] + demand[element1f];
				space_r2 = space_r2 + demand[element2];

				if ((demand[element1] + demand[element1f] > space_r2) || (demand[element2] > space_r1))
				{
					continue;

				}
		
				//==================================================copy r1 without customer j and j+1=================================================================//
				vector_r1.clear();
				for (int a = 0; a < saiz[r] + 2; a++)
				{
					vector_r1.push_back(vehicle[r][a]);
				}
				vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot
				vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot
							
				//==================================================copy r2 without customer n=================================================================//
				vector_r2.clear();
				for (int a = 0; a < saiz[m] + 2; a++)
				{
					vector_r2.push_back(vehicle[m][a]);

				}
				vector_r2.erase(vector_r2.begin() + n); //first one is depot	
				
				double available_dist_r1 = distance_available[r];
				double available_dist_r2 = distance_available[m];
				to_route = m;
				//############################################## To Insert ##########################################################////////////////////
				for (int p = 1; p < vector_r2.size(); p++) //from r1, insert to r2
				{
					for (int q = 1; q < vector_r1.size(); q++) //from r2, insert to r1
					{
						before_ele1 = vehicle[r][j - 1];   //initially, unless stated otherwise
						after_ele1 = vehicle[r][j + 2];

						before_ele2 = vehicle[m][n - 1];   //initially, unless stated otherwise
						after_ele2 = vehicle[m][n + 1];

						before_pos1 = vector_r1[q - 1];
						after_pos1 = vector_r1[q];  //noneed to add 1 because it is insert in between n-1 and n
						before_pos2 = vector_r2[p - 1];
						after_pos2 = vector_r2[p];  //noneed to add 1 because it is insert in between n-1 and n
	
						cost_without_i = dist[element1][before_ele1] + dist[element1][element1f] + dist[element1f][after_ele1] + service_time[element1] + service_time[element1f] - dist[before_ele1][after_ele1];//cost of r1 without i and i+1
						cost_without_j = cost_of_removing[vehicle[m][n]];//cost of r2 without j

						cost_with_i = dist[before_pos1][element2] + dist[element2][after_pos1] + service_time[element2] - dist[before_pos1][after_pos1]; //cost of r1 with j
						cost_with_j = dist[before_pos2][element1] + dist[element1][element1f] + dist[element1f][after_pos2]  + service_time[element1] + service_time[element1f] - dist[before_pos2][after_pos2]; //cost of r2 with i and i+1

						if ((cost_with_i > available_dist_r1 + cost_without_i) || (cost_with_j > available_dist_r2 + cost_without_j))
						{
							continue;
						}
						gain = (cost_without_i + cost_without_j) - (cost_with_i + cost_with_j);

						if (r < m)
						{
							if (gain-gain_matrix_1_0[r][m] > 0.01)//gain > gain_matrix_1_0[r][m]
							{

								gain_matrix_1_0[r][m] = gain;
								//info[number_matrix_1_0[m][r]][0] = number_matrix_1_0[m][r];
								info_1_0[(int)gain_matrix_1_0[m][r]][1] = gain;
								info_1_0[(int)gain_matrix_1_0[m][r]][2] = r;
								info_1_0[(int)gain_matrix_1_0[m][r]][3] = m;
								info_1_0[(int)gain_matrix_1_0[m][r]][4] = j;
								info_1_0[(int)gain_matrix_1_0[m][r]][5] = p;
								info_1_0[(int)gain_matrix_1_0[m][r]][6] = n;
								info_1_0[(int)gain_matrix_1_0[m][r]][7] = q;
								info_1_0[(int)gain_matrix_1_0[m][r]][8] = 3;//3 is (2-1)

							}//end if
						}

						else if (r > m)
						{
							if (gain-gain_matrix_1_0[m][r] > 0.01)//gain > gain_matrix_1_0[m][r]/////////////////////////////CHANGED on 1 OCT 2014!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
							{

								gain_matrix_1_0[m][r] = gain;
								//info[number_matrix_1_0[k][m]][0] = number_matrix_1_0[m][k];
								info_1_0[(int)gain_matrix_1_0[r][m]][1] = gain;
								info_1_0[(int)gain_matrix_1_0[r][m]][2] = r;
								info_1_0[(int)gain_matrix_1_0[r][m]][3] = m;
								info_1_0[(int)gain_matrix_1_0[r][m]][4] = j;
								info_1_0[(int)gain_matrix_1_0[r][m]][5] = p;
								info_1_0[(int)gain_matrix_1_0[r][m]][6] = n;
								info_1_0[(int)gain_matrix_1_0[r][m]][7] = q;
								info_1_0[(int)gain_matrix_1_0[r][m]][8] = 3;//3 is (2-1)

							}//end if
						}			
					}//end for q
				}//end for p
			}//end for n
		}//end for m
	}//end for j

	//============================================(To) From other routes to Affected routes: R1 and R2 ===========================================================
	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from R1)
	{
		if ((saiz[i]==0)||(saiz[i]==1)) //if it is empty route or saiz=1
			continue;
		for (int j = 1; j < saiz[i]; j++)  // consider each customer (from R1), start from 1 because [0] is depot
		{
			element1 = vehicle[i][j];			//elemet to be inserted into other route
			element1f = vehicle[i][j + 1];
			from_route = i;
			from_position = j;
			
			if (i == r) // || (demand[element2] > space_available[m])) //if customer already in the route or demand exceed avaialable space in route
			{
				continue;

			}
			for (int n = 1; n < saiz[r] + 1; n++) //which customer in r2 to remove
			{
				element2 = vehicle[r][n];			//elemet to be removed from r2
				if ( (NR_FLAG[element1][vehicle[r][n-1]]==false)||(NR_FLAG[element1f][vehicle[r][n+1]]==false)||(NR_FLAG[element2][vehicle[i][j-1]]==false)||(NR_FLAG[element2][vehicle[i][j+2]]==false) )
					continue;

				int space_r1 = space_available[i];//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				int space_r2 = space_available[r];
				space_r1 = space_r1 + demand[element1] + demand[element1f];
				space_r2 = space_r2 + demand[element2];

				if ((demand[element1] + demand[element1f] > space_r2) || (demand[element2] > space_r1))
				{
					continue;
				}
				
				//==================================================copy r1 without customer j and j+1=================================================================//
				vector_r1.clear();
				for (int a = 0; a < saiz[i] + 2; a++)
				{
					vector_r1.push_back(vehicle[i][a]);
				}
				vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot
				vector_r1.erase(vector_r1.begin() + j); //delete next element

				//==================================================copy r2 without customer n=================================================================//	
				vector_r2.clear();
				for (int a = 0; a < saiz[r] + 2; a++)
				{
					vector_r2.push_back(vehicle[r][a]);

				}

				vector_r2.erase(vector_r2.begin() + n); //first one is depot

				double available_dist_r1 = distance_available[i];
				double available_dist_r2 = distance_available[r];
				to_route = r;

				//############################################## To insert ##########################################################////////////////////
				for (int p = 1; p < vector_r2.size(); p++) //from r1, insert to r2
				{
					for (int q = 1; q < vector_r1.size(); q++) //from r2, insert to r1
					{
						
						before_ele1 = vehicle[i][j - 1];   //initially, unless stated otherwise
						after_ele1 = vehicle[i][j + 2];

						before_ele2 = vehicle[r][n - 1];   //initially, unless stated otherwise
						after_ele2 = vehicle[r][n + 1];

						before_pos1 = vector_r1[q - 1];
						after_pos1 = vector_r1[q];  //noneed to add 1 because it is insert in between n-1 and n
						before_pos2 = vector_r2[p - 1];
						after_pos2 = vector_r2[p];  //noneed to add 1 because it is insert in between n-1 and n
						
						//cost_without_i = cost_of_removing[vehic[i][j]] + cost_of_removing[vehic[i][j+1]];
						cost_without_i = dist[element1][before_ele1] + dist[element1][element1f] + dist[element1f][after_ele1] + service_time[element1] + service_time[element1f] - dist[before_ele1][after_ele1]; //cost of r1 without i and i+1
						cost_without_j = cost_of_removing[vehicle[r][n]];//cost of r2 without j

						cost_with_i = dist[before_pos1][element2] + dist[element2][after_pos1] + service_time[element2]- dist[before_pos1][after_pos1]; //cost of r1 with j 
						cost_with_j = dist[before_pos2][element1] + dist[element1][element1f] + dist[element1f][after_pos2] + service_time[element1] + service_time[element1f]- dist[before_pos2][after_pos2]; //cost of r2 with i and i+1

						if ((cost_with_i > available_dist_r1 + cost_without_i) || (cost_with_j > available_dist_r2 + cost_without_j))
						{
							//cout<<"2) ele1= " << element1 << "ele1f= "<< element1f <<"ele2= " << element2 <<endl;
							continue;
						}
						gain = (cost_without_i + cost_without_j) - (cost_with_i + cost_with_j);
						
						if (i < r)
						{
							if (gain-gain_matrix_1_0[i][r] > 0.01)//gain > gain_matrix_1_0[i][r]
							{
								gain_matrix_1_0[i][r] = gain;

								//info[number_matrix_1_0[i][r]][0] = gain_matrix_1_0[r][i];
								info_1_0[(int)gain_matrix_1_0[r][i]][1] = gain;
								info_1_0[(int)gain_matrix_1_0[r][i]][2] = i;
								info_1_0[(int)gain_matrix_1_0[r][i]][3] = r;
								info_1_0[(int)gain_matrix_1_0[r][i]][4] = j;
								info_1_0[(int)gain_matrix_1_0[r][i]][5] = p;
								info_1_0[(int)gain_matrix_1_0[r][i]][6] = n;
								info_1_0[(int)gain_matrix_1_0[r][i]][7] = q;
								info_1_0[(int)gain_matrix_1_0[r][i]][8] = 3; //3 is (2-1)

							}//end if
						}//end if (i < r)
						else if (i > r)
						{
							if (gain-gain_matrix_1_0[r][i] > 0.01)//gain > gain_matrix_1_0[r][i]
							{
								gain_matrix_1_0[r][i] = gain;

								//info[number_matrix_2_1[i][m]][0] = number_matrix_2_1[i][m];
								info_1_0[(int)gain_matrix_1_0[i][r]][1] = gain;
								info_1_0[(int)gain_matrix_1_0[i][r]][2] = i;
								info_1_0[(int)gain_matrix_1_0[i][r]][3] = r;
								info_1_0[(int)gain_matrix_1_0[i][r]][4] = j;
								info_1_0[(int)gain_matrix_1_0[i][r]][5] = p;
								info_1_0[(int)gain_matrix_1_0[i][r]][6] = n;
								info_1_0[(int)gain_matrix_1_0[i][r]][7] = q;
								info_1_0[(int)gain_matrix_1_0[i][r]][8] = 3; //3 is (2-1)
							}//end if
						}//end if (i > m)
					}//end for q
				}//end for p
			}//end for n
		}//end for j
	}//end for i 
end_of_2_1:;

	///////////////////////////////////////////////////////// 2 - 0 //////////////////////////////////////////////////////
	///////////////////////////////=================================== SAME ROUTE =================================////////////////////////

	if(saiz[r]==0) 
		goto end_of_2_0;
	else if ((saiz[r]==1) || (saiz[r]==2)) //no reshuffle can be made
		goto end_of_2_0_first;
	
	for (int j = 1; j < saiz[r]; j++) //which one to delete (always take a pair, so from 1 until SAIZ-1
	{
		double available_dist_r2 = distance_available[r];
		//copy a temporary route
		int b=0;
		int* temp_r = new int[saiz[r]+2];
		for (int a = 0; a < saiz[r] + 2; a++)
		{
			if ((a == j) || (a == j+1))
			{
				continue; //dont copy the element, assume the element to be deleted
			}

			else
			{
				temp_r[b] = vehicle[r][a];
				b++;
			}
		}
		element1 = vehicle[r][j];
		element1f = vehicle[r][j+1];
		before_ele = vehicle[r][j - 1];   //initially, unless stated otherwise
		after_ele = vehicle[r][j + 2];
		
		cost_without_i = dist[element1][before_ele] + dist[element1][element1f] + dist[element1f][after_ele]  + service_time[element1] + service_time[element1f] - dist[before_ele][after_ele]; //cost of r1 without i and i+1
		
		for (int n = 1; n < saiz[r] ; n++) //has 1 customer less because it has been deleted
		{
			if (j == n) //originally from this position, so noneed to consider
				continue; 
			else
			{
				before_pos = temp_r[n - 1];
				after_pos = temp_r[n];  //noneed to add 1 because it is insert in between n-1 and n
			}	
			
			if ( (NR_FLAG[element1][before_pos] == false) || (NR_FLAG[element1f][after_pos] == false) )
					continue;
			cost_with_i = dist[before_pos][element1] + dist[element1][element1f] + dist[element1f][after_pos] + service_time[element1] + service_time[element1f]- dist[before_pos][after_pos]; //cost of r2 with i //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						
			gain = cost_without_i - cost_with_i;
			//if (abs(gain) > available_dist_r2)//if negative gain, it will not be recorded anyway, so this code can be omitted actually
			//{
			//	continue;
			//}
					
			if (gain-gain_matrix_1_0[r][r] > 0.01)//gain > gain_matrix_1_0[r][r]
			{
				gain_matrix_1_0[r][r] = gain;
				same_r_info[r][0] = r;
				same_r_info[r][1] = gain;
				same_r_info[r][2] = r; //from route
				same_r_info[r][3] = r; //to route, i=m here
				same_r_info[r][4] = j; //from which position
				same_r_info[r][5] = n; //to which position //assume modified route, customer has been deleted
				same_r_info[r][6] = -1; 
				same_r_info[r][7] = -1;
				same_r_info[r][8] = 4; //4 is (2-0)

			}	
		}
		delete[] temp_r;
	}//end of j

end_of_2_0_first:
	///////////////////////////////=================================== DIFFERENT ROUTE =================================////////////////////////
		//from 2 routes to every position=======================update row============================= ///////////////////////////////////
	
		for (int j = 1; j < saiz[r] + 1; j++)  // consider each customer (from)
		{
			element1 = vehicle[r][j];			//elemet to be inserted into other route
			element2 = vehicle[r][j+1];
			from_route = r;
			from_position = j;
			//cout << element;

			for (int m = 0; m < no_routes; m++)  //insert (to) which route
			{
				if ((m == r) || (demand[element1]+demand[element2] > space_available[m])) //if customer already in the route or demand exceed avaialable space in route
				{
					continue;
					//m++;
					//goto mylabel;
				}


				for (int n = 1; n <= saiz[m] + 1; n++) //insert (to) which position, from 0 to saiz + 1, insert at 1 means replace element [1] which is the first customer
				{
					double available_dist_r2 = distance_available[m];
					to_route = m;
					before_ele = vehicle[r][j - 1];   //initially, unless stated otherwise
					after_ele = vehicle[r][j + 1];
					before_pos = vehicle[m][n - 1];
					after_pos = vehicle[m][n];  //noneed to add 1 because it is insert in between n-1 and n
					
					if ( (NR_FLAG[element1][before_pos] == false) || (NR_FLAG[element2][after_pos] == false) )
						continue;

					cost_without_i = (dist[element1][before_ele] + dist[element1][element2] + dist[element2][after_ele] + service_time[element1] + service_time[element2]) - (dist[before_ele][after_ele]); //cost of r1 without i and i+1 //old-new
					cost_with_i = dist[before_pos][element1] + dist[element1][element2] + dist[element2][after_pos] + service_time[element1] + service_time[element2] - dist[before_pos][after_pos]; //cost of r2 with i //new-old

					if (cost_with_i > available_dist_r2)
					{
						//cout<<"2) ele1= " << element1 << "ele1f= "<< element1f <<"ele2= " << element2 <<endl;
						continue;
					}
					gain = cost_without_i - cost_with_i;


					if (r < m)
					{
						if (gain-gain_matrix_1_0[r][m] > 0.01)//gain > gain_matrix_1_0[r][m]
						{
							gain_matrix_1_0[r][m] = gain;
							//info_1_0[gain_matrix_1_0[m][from1]][0] = gain_matrix_1_0[m][from1];
							info_1_0[(int)gain_matrix_1_0[m][r]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[m][r]][2] = r;
							info_1_0[(int)gain_matrix_1_0[m][r]][3] = m;
							info_1_0[(int)gain_matrix_1_0[m][r]][4] = j;
							info_1_0[(int)gain_matrix_1_0[m][r]][5] = n;
							info_1_0[(int)gain_matrix_1_0[m][r]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[m][r]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[m][r]][8] = 4; //4 is (2-0)

						}
					}

					else if (r > m)
					{
						if (gain-gain_matrix_1_0[m][r] > 0.01)//gain > gain_matrix_1_0[m][r]
						{
							gain_matrix_1_0[m][r] = gain;
							//info_1_0[gain_matrix_1_0[m][from1]][0] = gain_matrix_1_0[from1][m];
							info_1_0[(int)gain_matrix_1_0[r][m]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[r][m]][2] = r;
							info_1_0[(int)gain_matrix_1_0[r][m]][3] = m;
							info_1_0[(int)gain_matrix_1_0[r][m]][4] = j;
							info_1_0[(int)gain_matrix_1_0[r][m]][5] = n;
							info_1_0[(int)gain_matrix_1_0[r][m]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[r][m]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[r][m]][8] = 4; //4 is (2-0)
						}
					}
				}//end for n
			}//end for m
		}//end for j
		
	//######################################update gain matrix################################################//
	//from every position to 2 routes=======================update column============================= ///////////////////////////////////

	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from)
	{
		if ((saiz[i]==0) || (saiz[i]==1) )//if it is empty route or saiz=1
			continue;
		for (int j = 1; j < saiz[i] ; j++)  // consider each customer (from)
		{
			element1 = vehicle[i][j];			//element to be inserted into other route
			element2 = vehicle[i][j+1];			//element to be inserted into other route
			from_route = i;
			from_position = j;

			double available_dist_r2 = distance_available[r];
			if ((i == r) || (demand[element1] + demand[element2] > space_available[r])) //if demand exceed avaialable space in route && differnt route, if same route, capacity exceed stil have to find best gain
			{
				continue;
			}

			for (int n = 1; n < saiz[r] + 2; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
			{
				to_route = r;
				before_ele = vehicle[i][j - 1];   //initially, unless stated otherwise
				after_ele = vehicle[i][j + 1];
				before_pos = vehicle[r][n - 1];
				after_pos = vehicle[r][n];  //noneed to add 1 because it is insert in between n-1 and n
				if ( (NR_FLAG[element1][before_pos] == false) || (NR_FLAG[element2][after_pos] == false) )
					continue;

				cost_without_i = (dist[element1][before_ele] + dist[element1][element2] + dist[element2][after_ele] + service_time[element1] + service_time[element2]) - (dist[before_ele][after_ele]); //cost of r1 without i and i+1 //old-new
				cost_with_i = dist[before_pos][element1] + dist[element1][element2] + dist[element2][after_pos] + service_time[element1] + service_time[element2] - dist[before_pos][after_pos]; //cost of r2 with i //new-old

				if (cost_with_i > available_dist_r2)
				{
					continue;
				}
				gain = cost_without_i - cost_with_i;

				if (i < r)
				{
					if (gain-gain_matrix_1_0[i][r] > 0.01)//gain > gain_matrix_1_0[i][r]
					{
						gain_matrix_1_0[i][r] = gain;
						//info_1_0[(int)gain_matrix_1_0[i][to1]][0] = gain_matrix_1_0[to1][i];
						info_1_0[(int)gain_matrix_1_0[r][i]][1] = gain;
						info_1_0[(int)gain_matrix_1_0[r][i]][2] = i; //from_route
						info_1_0[(int)gain_matrix_1_0[r][i]][3] = r; //to_route
						info_1_0[(int)gain_matrix_1_0[r][i]][4] = j; //from_position
						info_1_0[(int)gain_matrix_1_0[r][i]][5] = n; //to_position in other route
						info_1_0[(int)gain_matrix_1_0[r][i]][6] = -1;
						info_1_0[(int)gain_matrix_1_0[r][i]][7] = -1;
						info_1_0[(int)gain_matrix_1_0[r][i]][8] = 4; //4 is (2-0)

					}
				}
				else if (i > r)
				{
					if (gain-gain_matrix_1_0[r][i] > 0.01)//gain > gain_matrix_1_0[r][i]
					{
						gain_matrix_1_0[r][i] = gain;
						//info_1_0[(int)gain_matrix_1_0[to1][i]][0] = gain_matrix_1_0[i][to1];
						info_1_0[(int)gain_matrix_1_0[i][r]][1] = gain;
						info_1_0[(int)gain_matrix_1_0[i][r]][2] = i;
						info_1_0[(int)gain_matrix_1_0[i][r]][3] = r;
						info_1_0[(int)gain_matrix_1_0[i][r]][4] = j;
						info_1_0[(int)gain_matrix_1_0[i][r]][5] = n;
						info_1_0[(int)gain_matrix_1_0[i][r]][6] = -1;
						info_1_0[(int)gain_matrix_1_0[i][r]][7] = -1;
						info_1_0[(int)gain_matrix_1_0[i][r]][8] = 4; //4 is (2-0)
					}
				}
			}//end for n	
		}//end for j
	}
end_of_2_0:;

}


void recalculate_best_gain(double **distance, int *demand, double *(&cost_of_removing), int *(&removing_route), int *(&removing_position), double **(&info_1_0), double **(&same_r_info),int **vehicle, double **(&gain_matrix_1_0), double **(temp_sol))
{
	int num_attributes = 9;
	//++++++++++++++++++++++++++++++++++++++++++++++++++DATA STRUCTURE++++++++++++++++++++++++++++++++++++++++++++++++++++++//
	int no_routes = LOCAL_NO_ROUTE;

	cout << "TOTAL ROUTES= " << no_routes << endl;

	//double *cost_of_removing = new double[SIZE + 1];
	//int *removing_route = new int[SIZE];
	//int *removing_position = new int[SIZE];

	//========================================2-D matrices=============================================//
	//int **vehicle = new int*[no_routes];
	//double **gain_matrix_1_0 = new double*[no_routes];
	//double **same_r_info = new double*[no_routes]; // to record the best improvement within the same route
	//for (int i = 0; i < no_routes; ++i)
	//{
	//	vehicle[i] = new int[SIZE];
	//	gain_matrix_1_0[i] = new double[no_routes];
	//	same_r_info[i] = new double[num_attributes];
	//}

	//int s = (((no_routes*no_routes) - no_routes) / 2) + 1;
	//double** info_1_0 = new double*[s];
	//for (int i = 0; i < s; ++i)
	//{
	//	info_1_0[i] = new double[num_attributes];
	//}


	//int* route = new int[SIZE];


	//========================================Initialize Gain matrix ============================================//
	
	for (int i = 0; i < no_routes; i++)
	{
		for (int j = 0; j < no_routes; j++)
		{
			if (i<j)
			{
				gain_matrix_1_0[i][j] = INT_MIN;
			}
			gain_matrix_1_0[i][i] = INT_MIN;
		}
	}
	int i = ((no_routes*no_routes) - no_routes) / 2;
	for (int u = (no_routes - 1); u > 0; u--)
	{
		for (int v = u - 1; v >= 0; v--)
		{
			gain_matrix_1_0[u][v] = i;
			i--;
		}
	}

	//read customer and put in vehicle[i][j] which is in the form or SIZE 2 3 1 0 5 SIZE (start and end with depot with customers in between)
	for (int i = 0; i < no_routes; i++)
	{
		int j=3; //from BEST_ROUTE[i][3] onwards are customers
		int k=1; //for vehicle[i][k]
		vehicle[i][0] = SIZE; //first element in vehicle[0][0] is depot
		route_cost[i] = temp_sol[i][1];
		for (int j = 3; j <= saiz[i]+3; j++)
		{
			vehicle[i][k] = temp_sol[i][j];
			k++;
		}
		vehicle[i][saiz[i]+1] = SIZE;  //last element in vehicle[i][j] is depot
	}

	double T_COST = 0;
	for (int g = 0; g < no_routes; g++)
	{

		T_COST = T_COST + route_cost[g];

	}
	//vehicle should be [0]depot, [1] (cust) , [last]depot
	for (int i = 0; i < no_routes; i++)
	{
		for (int b = 0; b < saiz[i]+2; b++)
		{
			cout << vehicle[i][b] << ' ';
		}
		cout << endl;

	}

	//========================================Cost of removing ============================================//
	//cost of removing for single customer
	for (int f = 0; f < no_routes; f++) //find cost of removing for each customer and put in array
	{
		for (int h = 1; h < saiz[f] + 1; h++) //first customer start from element [1]
		{

			cost_of_removing[vehicle[f][h]] = distance[vehicle[f][h]][vehicle[f][h - 1]] + distance[vehicle[f][h]][vehicle[f][h + 1]] + service_time[vehicle[f][h]] - distance[vehicle[f][h - 1]][vehicle[f][h + 1]];

			removing_route[vehicle[f][h]] = f;
			removing_position[vehicle[f][h]] = h;
		}
	}

	best_gain_1_0(distance, demand, cost_of_removing, info_1_0, same_r_info, vehicle, gain_matrix_1_0);
	best_gain_1_1(distance, demand, cost_of_removing, info_1_0, same_r_info, vehicle, gain_matrix_1_0);
	best_gain_2_1(distance, demand, cost_of_removing, info_1_0, same_r_info, vehicle, gain_matrix_1_0);
	best_gain_2_0(distance, demand, cost_of_removing, info_1_0, same_r_info, vehicle, gain_matrix_1_0);
}


void update_all_cost_of_removing (double **dist, int **vehic_depotCust, double *(&cost_of_removing), int no_routes)
{
	for (int f = 0; f < no_routes; f++)
	{
		if (saiz[f]==0)
			continue;
		for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		{
			cost_of_removing[vehic_depotCust[f][h]] = dist[vehic_depotCust[f][h]][vehic_depotCust[f][h - 1]] + dist[vehic_depotCust[f][h]][vehic_depotCust[f][h + 1]] + service_time[vehic_depotCust[f][h]] - dist[vehic_depotCust[f][h - 1]][vehic_depotCust[f][h + 1]];
			//removing_route[vehic_depotCust[f][h]] = f;
			//removing_position[vehic_depotCust[f][h]] = h;
		}
	}
	

}

void delete_middle_empty_route (int route) //route is the empty route index to delete, shift up the route
{
	for (int i = route; i < LOCAL_NO_ROUTE-1; i++)
	{
		for (int j = 0; j <= LOCAL_SAIZ[i+1]+3; j++) //SAIZ of next route
		{
			//copy next route to current route
			LOCAL_BEST_ROUTE[i][j] = LOCAL_BEST_ROUTE[i+1][j];
			LOCAL_SAIZ[i] =  LOCAL_SAIZ[i+1];
			LOCAL_capa[i] = LOCAL_capa[i+1];

		}	
		//reinitialize next route
		for (int k = 0; k <SIZE; k++)
		{
			LOCAL_BEST_ROUTE[i+1][k]=-1;
		}
		LOCAL_SAIZ[i+1] = 0;
		LOCAL_capa[i+1] = 0;
	}
	LOCAL_NO_ROUTE = LOCAL_NO_ROUTE-1;
}

