// Start - add an empty route - copy vehicle[][], saiz[] from LOCAL[][] - recalculate best_gain() {vehic_depotCust[][] is copied from vehicle[][], find cost_of_removing[]} - 
// module: shake_1_0() {the shake function update vehicle[][] and all global variables} - record route change from shake - cost_of_removingcopy vehic_depotCust[][] from vehicle[][], update saiz[], update cost_of_removing[], distance _available[] for shaked routes
// do while (max_gain>0) {reoptimize_1_0(), ..., find best_gain, if (gain>0) insert_1_0(),... }
//shake use float vehicle[][],
//LS use int vehic_depoCust[][]
#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <algorithm>
#include "BestImprovement.h"
#include "VNS.h"
#include "LocalSearch.h"
//#include "DiversificationDijkstra.h"
//#include "DijkstraRefinement.h"
#include "InverseMethod.h"
//#include "RecordBestSol.h"
#include "DiversificationLNS.h"
#include "SectorConflict.h"
#include "Guided_shake.h"
//#include "2_opt.h"
#include "Diversification_overlapRoute.h"
//#include "VND.h"
#include "Sort_solution.h"

#define INFEASIBLE 0.00
#define INFIN 99999999 

using namespace std;
const float BIGC = 20;

int num_info=5;//one indicate status if gain is available, one store gain in cost, one store gain in distance, special case for cross_insert_fromshaking record r1 and r2 together
	//gainvec[1] = gain1;
	//gainvec[2] = distgain1;
	//gainvec[3] = gain2; //only for  cross_insert_fromshaking
	//gainvec[4] = distgain2; //only for  cross_insert_fromshaking
float *gainvec = new float[num_info];

void VNS(float *x, float *y, float *(&freq)) 
{
	//ofstream gain_matrix("23.GAIN_MATRIX_" + std::to_string( I ) + ".txt");
	//ofstream recordTIME("TIME.txt", ios::app);
	//ofstream violation("Infeasible_" + std::to_string( I ) + ".txt");
	ofstream timefile("16.Time_" + std::to_string( I ) + ".txt", ios::app);
	ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
	ofstream localbest_sol("30.LOCAL_BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
	ofstream recordFreq("27.Frequency_of_module_" + std::to_string( I ) + ".txt");
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	//ofstream EV_diver ("1.Diver_" + std::to_string( I ) + ".csv", ios::out);
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);
	float start_s = clock();// the code you wish to time goes here

	//********************************************* STEP 1: INITIALIZATION *********************************************//
	int max_neigbor = 6; //i have 6 neighbourhoods - for shaking 
	//int max_neigbor = 7; //i have 7 neighbourhoods - for shaking 
	//int maxLocalSearch = 9; //number of local search: 1-0, 1-1, 2-1, 2-0, 2-2, 2-optintra, 2-optinter, crosstail, cross
	int maxLocalSearch = 9; //number of local search: 1-0,  2-0,  2-optintra, 2-optinter, crosstail, cross //ONLY 7 will be performed
	int NbDiv = 0; //number of diversification, initially starts at 0, add one in every diversifcation, 
	int maxDiv = min(3, GLOBAL_NO_ROUTE/2);//max diversification as stopping criterion
	int LB_Ndel_min = max (15.00, (0.15*SIZE));
	int UB_Ndel_min = min (400.00, (0.5*SIZE));
	int Ndel_min = LB_Ndel_min; //min number of iteration for each route for diversification //to control the strength in removal strategies
	int RS = 6; //number of removal strategies
	//================ Frequency for 8 LS =====================// record the overall frequency for LS
	float *gain = new float[SIZE]; //can be many depends on how many positive gain
	int *modul = new int[SIZE]; //declare many
	for (int i = 0; i < SIZE; i++)//initialize
	{
		freq[i] = 0;
		gain[i] = 0;
		modul[i] = 0;
	}
	//EV_diver<<"maxDiv= "<<maxDiv<<endl;
	int *divsequence = new int[RS];//randomize the diversification strategy
	for (int i = 0; i < RS; i++)
	{
		divsequence[i]=i;//put in 0 to 5
	}
	std::srand ( std::time(0));
	std::random_shuffle ( &divsequence[0], &divsequence[RS-1] );//randomize the sequence
	
	//============= Copy Local_BEST from GLOBAL_BEST ===========//

	for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	{

		for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
		{
			LOCAL[i][j] = GLOBAL[i][j];
			LOCALCumDist[i][j] = GLOBALCumDist[i][j];
		}
	
		LOCAL_capa[i] = GLOBAL_capa[i];
		LOCAL_SAIZ[i] = GLOBAL_SAIZ[i];
		LOCAL_Rcost[i] = GLOBAL_Rcost[i];
		LOCAL_distance_cost[i] = GLOBAL_distance_cost[i];
	}
	LOCAL_BEST_COST = GLOBAL_BEST_COST;
	LOCAL_NO_ROUTE = GLOBAL_NO_ROUTE;

	//LOCAL_BCOST = INT_MAX;//declare very big value because we dont want this to record the GLOBAL_BEST, it will record the good solution for diversifiy

	int div=0;
	int noImproveDiv = 0;
	//while (noImproveDiv <= 4)
	while (NbDiv <= maxDiv) //#############################################################################################################################
	{
	//restartAfterDiver:
		int ori_no_routes = no_routes;//for deletion after diversification
		float violateThreshold = INFEASIBLE; //reinitialize to original value after diversification
		int found_GLOBEST_status = 0;
		localbest_sol << "Start of Best improvement NbDiv= " <<NbDiv <<endl;

		//********************************************* STEP 2: DELETE EMPTY ROUTE *********************************************//
		int no_empty_route=0;
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			if (LOCAL_SAIZ[i] == 0)
				no_empty_route++;//calculate how many empty route
		}

		if (no_empty_route > 1) //if more than 1 empty route, do not copy the empty route 
		{
			int k=0; //for new route index
	
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				if (LOCAL_SAIZ[i] == 0)
				{
					continue;
				}
				
				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
				{
					LOCAL[k][j] = LOCAL[i][j];
					LOCALCumDist[k][j] = LOCALCumDist[i][j];
				}

				LOCAL_SAIZ[k] = LOCAL_SAIZ[i];
				LOCAL_capa[k] = LOCAL_capa[i];
				LOCAL_Rcost[k] = LOCAL_Rcost[i];
				LOCAL_distance_cost[k] = LOCAL_distance_cost[i];
				k++;
			}
			LOCAL_NO_ROUTE = k;
			
		}
	

		cout<<"LOCAL_NO_ROUTE= "<< LOCAL_NO_ROUTE<<endl;

		no_routes = LOCAL_NO_ROUTE; //###########################################################################################
		bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0
		int num_attributes = 13;//[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance

		//++++++++++++++++++++++++++++++++++++++++++++++++++DATA STRUCTURE++++++++++++++++++++++++++++++++++++++++++++++++++++++//
		float *cost_of_removing = new float[SIZE + 1];//cost_of_removing[][] only used in LS, not shaking

		originalGain oriGain;
		oriGain.Oricost_of_removing = new float[SIZE + 1];

		//========================================2-D matrices=============================================//
		float **gain_matrix_1_0 = new float*[no_routes]; //gain_matrix is NumR x NumR
		float **same_r_info = new float*[no_routes]; // to record the best improvement within the same route //same_r_info is NumR x NumAttributes

		//declare a copy of gain_matrix, update it whenever local best is found
		//this is because shaking always starts with local best, it is not necessary to recalculate the gain_matrix again, we can just copy from ori_gain_matrix. The gain_matrix is the working matrix which will be updated in shaking and LS, the ori_gain_matrix does not chnge  
		oriGain.Origain_matrix_1_0 = new float*[no_routes];
		oriGain.Orisame_r_info = new float*[no_routes]; 
		for (int i = 0; i < no_routes; ++i)
		{
			gain_matrix_1_0[i] = new float[no_routes];
			same_r_info[i] = new float[num_attributes];

			oriGain.Origain_matrix_1_0[i] = new float[no_routes];
			oriGain.Orisame_r_info[i] = new float[num_attributes];
		}

		int s = ((no_routes*no_routes) - no_routes) / 2;
		float** info_1_0 = new float*[s+1];//info_matrix is s x numAttributes, s starts from 1, so declare s+1 here

		oriGain.Oriinfo_1_0= new float*[s+1];
		for (int i = 1; i <= s; ++i)
		{
			info_1_0[i] = new float[num_attributes];
			oriGain.Oriinfo_1_0[i] = new float[num_attributes];
		}
		//========================================Initialize Gain matrix ============================================//
		//diagonal of gain matrix is -1
		//for (int i = 0; i < no_routes; i++)
		//{
		//	gain_matrix_1_0[i][i] = -1;
		//}

		//int i = ((no_routes*no_routes) - no_routes) / 2;
		//for (int u = (no_routes - 1); u > 0; u--)
		//{
		//	for (int v = u - 1; v >= 0; v--)
		//	{
		//		gain_matrix_1_0[u][v] = i;
		//		i--;
		//	}
		//}

		//================= Record route change status from shaking and best improvement======================//added 15Sept2015
		RchangedStatus = new bool[LOCAL_NO_ROUTE];

		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			RchangedStatus[i]=false;//initialize to 0 for every route, they have not been changed
		}
		//CustaffectedStatus = new bool[SIZE+1];//including depot because sometimes the flag position is [0] but the depot has no cost of removing, so doesnt matter
		//for (int i = 0; i <= SIZE; i++)
		//{	
		//	CustaffectedStatus[i] = false;//initialize to false means not affected	
		//}

		//============= Copy VEHICLE from LOCAL_BEST ===========//
		bool copyalready;

		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			
			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
			{
				VEHICLE[i][j] = LOCAL[i][j];
				CumDist[i][j] = LOCALCumDist[i][j];
			}

			saiz[i] = LOCAL_SAIZ[i]; 
			total_demand[i] = LOCAL_capa[i];
			route_cost[i] = LOCAL_Rcost[i]; 
			distance_cost[i] = LOCAL_distance_cost[i];
			space_available[i] = (CAPACITY*(1+violateThreshold)) - (total_demand[i]);
			distance_available[i] = (DISTANCE*(1+violateThreshold)) - (distance_cost[i]);
			//space_available[i] = CAPACITY - total_demand[i];
			//distance_available[i] = DISTANCE - distance_cost[i];
		}
		no_routes=LOCAL_NO_ROUTE;//added 24 April 2016
		copyalready = true;
		//============= End of Copy VEHICLE from LOCAL_BEST ===========//
		
		//find cost of removing and best gain for the first time
		calculate_best_gain(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0, oriGain);
		for (int f = 0; f < SIZE; f++) //copy from cost_of_removing
		{
			 oriGain.Oricost_of_removing[f] = cost_of_removing[f]; //Oricost_of_removing based on LOCAL
		} 
		int Neighbour_k = 1; //Neighbour_k 1 = (1-0), Neighbour_k 2 = (1-1), Neighbour_k 3 = (2-1)
		
		//==========================================================================//
		int total_time = 0;
		float maxCPUtime = 10000/(CLOCKS_PER_SEC);
		cout << "maxCPUtime= " <<maxCPUtime<<endl;

		bool foundLOCALinCurrNeighbor = false;//Do Dijkstra refinement if found LOCAL BEST in any neighbourhood in this diversification, otherwise, just do refinement for VEHICLE
		
		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ k+1 neighborhood starts here @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
		while (Neighbour_k <= max_neigbor)	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		{
		//module://after local search if no improvement, Neighbour_k++ goto module //this is k+1 in the algorithm (move to next neighborhood)
			if (violateThreshold < 0.01) //if smaller than 1%, make it zero
				violateThreshold = 0;
			//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//
			route_CGravity = new float*[LOCAL_NO_ROUTE];//change to LOCAL_NO_ROUTE on 12Oct2015 //record centre of gravity for each route
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)//change to LOCAL_NO_ROUTE on 12Oct2015
			{
				route_CGravity[i] = new float[2]; //2 columns consists of x and y
			}

			custRGravity = new float[LOCAL_NO_ROUTE]; //record distance from customer to the centre of gravity of each route
			sorted_custRGravity = new float*[LOCAL_NO_ROUTE];
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				custRGravity[i] = 0; //initialize
				sorted_custRGravity[i] = new float[5]; //[0]sorted distance, [1]sorted route index, [2]1/distance, [3]d/sum(d), [4]cumulative 
			}
	
			//VEHICLE copied from LOCAL_BEST_ROUTE to be passed to shake(), need to recopy because when neighbourhoood++, VEHICLE[][] need to recopy from LOCAL_BEST although the first iteration this is not necessary because VEHICLE already copy before
			if (copyalready == false)//if first time, VEHICLE already copy from LOCAL_BEST, noneed to copy, added on 12Oct2015
			{
				no_routes = LOCAL_NO_ROUTE; //########################################################################################################
				
				for (int i = 0; i < LOCAL_NO_ROUTE; i++)
				{
					for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
					{
						VEHICLE[i][j] = LOCAL[i][j];	
						CumDist[i][j] = LOCALCumDist[i][j];
					}
					
					saiz[i] = LOCAL_SAIZ[i]; 
					route_cost[i] = LOCAL_Rcost[i];
					distance_cost[i] = LOCAL_distance_cost[i];
					total_demand[i] = LOCAL_capa[i];
					space_available[i] = (CAPACITY*(1+violateThreshold)) - (total_demand[i]);
					distance_available[i] = (DISTANCE*(1+violateThreshold)) - (distance_cost[i]);
					//space_available[i] = CAPACITY - total_demand[i];
					//distance_available[i] = DISTANCE - distance_cost[i];
				}
			}
			copyalready = false;//reinitialize to false after first time

			calculate_centreGravity(x, y); //MUST PUT THIS ONE HERE TO AVOID CALCULATING EVERYTIME IN EACH SHAKE
	reshake://if no feasible shake, Neighbour_k++ and goto reshake

			int shake_status1 = 1; //initialize flag_status=0, 0 is not ok, 1 is ok
			//int shake_status2 = 1;
			//int shake_status3 = 1;
			//int shake_status4 = 1;
			if (Neighbour_k == 1) //(1-0) First neighbourhood must shake twice otherwise it will back to the same solution as LOCAL BEST because this is best improvement
			{
				
				shake_status1 = shakeCROSSbasedonNeighbourhood(VEHICLE, x, y, Neighbour_k);
				//shake_status1 = shake_intraSegmentReshuffle(VEHICLE);
				//shake_status1 = shake_intraReverseSegment(VEHICLE, x, y);
				
				if (shake_status1 == 0)
				{
					shake_reshuffle(VEHICLE);
					//Neighbour_k++;
					//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
				}
			}

			if (Neighbour_k == 2) //(1-0) First neighbourhood must shake twice otherwise it will back to the same solution as LOCAL BEST because this is best improvement
			{
				//cout<<"Entering Neighbour_k == 1 "<<endl;
				//shake_status1 = shake_1_0(VEHICLE, x, y); //the one going to shake is always the current best solution
				//shake_status1 = shake_1_1(VEHICLE, x, y); //the one going to shake is always the current best solution
				shake_status1 = shakeCROSSbasedonNeighbourhood(VEHICLE, x, y, Neighbour_k);
				//shake_status1 = shakeCROSS(VEHICLE, x, y);
				//shake_status1 = shake_intraSegmentReshuffle(VEHICLE);//pass in the Neighbourhood number to determine how many route head will be reshuffle
		
				if (shake_status1 == 0)
				//if ((shake_status1 == 0) && (shake_status2 == 0) && (shake_status3 == 0) && (shake_status4 == 0)) //if no feasible shake can be performed, if one shake is ok, still proceed
				{
					shake_reshuffle(VEHICLE);
					//Neighbour_k++;
					//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
				}
			}
		
			else if (Neighbour_k == 3) //1-1
			{	
				//cout<<"Entering Neighbour_k == 2 "<<endl;
				
				shake_status1 = shakeCROSSbasedonNeighbourhood(VEHICLE, x, y, Neighbour_k);
				//shake_status1 = shake_2_1_twoR(VEHICLE, x, y);
				//shake_status1 = shake_intraHeadReshuffle(VEHICLE, Neighbour_k);
				//shake_status1 = shake_2_0_threeR(VEHICLE, x, y);
				//shake_status3 = shakeCROSS(VEHICLE, x, y);
				//shake_status4 = shakeCROSS(VEHICLE, x, y);
				if (shake_status1 == 0)
				//if ((shake_status1 == 0) && (shake_status2 == 0) && (shake_status3 == 0) && (shake_status4 == 0))
				{
					shake_reshuffle(VEHICLE);
					//Neighbour_k++;
					//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
				}
		
			}
			else if (Neighbour_k == 4) //(2-0) - a pair from route A go to route B
			{			
				//cout<<"Entering Neighbour_k == 3 "<<endl;
				//shake_status1 = shake_2_0_threeR(VEHICLE, x, y);
				shake_status1 = shakeCROSSbasedonNeighbourhood(VEHICLE, x, y, Neighbour_k);
				//shake_status1 = shake_2_1_threeR(VEHICLE, x, y);
				//shake_status1 = shake_intraHeadReshuffle(VEHICLE, Neighbour_k);//pass in the Neighbourhood number to determine how many route head will be reshuffle
				//shake_status3 = shakeCROSS(VEHICLE, x, y);
				if (shake_status1 == 0) 
				//if ((shake_status1 == 0) && (shake_status2 == 0) && (shake_status3 == 0))
				{
					shake_reshuffle(VEHICLE);
					//Neighbour_k++;
					//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
				}
			}
	
			else if (Neighbour_k == 5) //(2-0) - a pair from route A go to route B and route C
			{			
				//cout<<"Entering Neighbour_k == 4 "<<endl;
				//shake_status1 = shake_2_1_threeR(VEHICLE, x, y);
				shake_status1 = shakeCROSSbasedonNeighbourhood(VEHICLE, x, y, Neighbour_k);
				//shake_status1 = shake_2_2(VEHICLE, x, y);
				//shake_status1 = shake_reshuffle(VEHICLE);
				//shake_status3 = shakeCROSS(VEHICLE, x, y);
				//if ((shake_status1 == 0) && (shake_status2 == 0) && (shake_status3 == 0))
				if (shake_status1 == 0) 
				{
					shake_reshuffle(VEHICLE);
					//Neighbour_k++;
					//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
				}
			}

			else if (Neighbour_k == 6) //(2-1) - a pair from route A swap with one customer from route B
			{			
				//cout<<"Entering Neighbour_k == 5 "<<endl;
				//shake_status1 = shake_2_2(VEHICLE, x, y);
				//shake_status2 = shake_2_0_threeR(VEHICLE, x, y);
				//shake_status1 = shakeCROSS(VEHICLE, x, y);
				shake_status1 = shakeCROSSbasedonNeighbourhood(VEHICLE, x, y, Neighbour_k);
				//shake_status1 = shake_2_0_threeR(VEHICLE, x, y);
				//shake_status1 = shake_reshuffle(VEHICLE);//pass in the Neighbourhood number to determine how many route head will be reshuffle
				if (shake_status1 == 0) 
				//if ((shake_status1 == 0) && (shake_status2 == 0) && (shake_status3 == 0))
				{
					shake_reshuffle(VEHICLE);
					//Neighbour_k++;
					//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
					//goto skip_LS; //if no feasible shake at last  neighborhod, dont perform LS, goto skip_LS, but need to copy vehicDepoCust[][] from vehicle[][] //added 15June2015 //check this if correct
				}
			}

			//else if (Neighbour_k == 6) //(2-1) - first pair of cust must from route A to B, another cust from route A can go to route C
			//{			
			//	//cout<<"Entering Neighbour_k == 6 "<<endl;
			//	shake_status1 = shake_2_1_threeR(VEHICLE, x, y);
			//	//shake_status2 = shake_2_0_threeR(VEHICLE, x, y);
			//	shake_status3 = shakeCROSS(VEHICLE, x, y);

			//	if ((shake_status1 == 0) && (shake_status3 == 0))
			//	//if ((shake_status1 == 0) && (shake_status2 == 0) && (shake_status3 == 0))
			//	{
			//		Neighbour_k++;
			//		goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
			//	}
			//}
			//else if (Neighbour_k == 7) //(2-2) - swap 2 customers /CROSS
			//{
			//	//cout<<"Entering Neighbour_k == 7 "<<endl;
			//	shake_status1 = shake_2_2(VEHICLE, x, y);
			//	shake_status2 = shake_2_1_threeR(VEHICLE, x, y);
			//	shake_status3 = shakeCROSS(VEHICLE, x, y);
			//	if ((shake_status1 == 0) && (shake_status2 == 0) && (shake_status3 == 0))
			//	//if ((shake_status1 == 0) && (shake_status2 == 0))
			//	{
			//		Neighbour_k++;
			//		//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
			//		goto skip_LS; //if no feasible shake at last  neighborhod, dont perform LS, goto skip_LS, but need to copy vehicDepoCust[][] from vehicle[][] //added 15June2015 //check this if correct
			//	}			
			//}

			//for (int r = 0; r < no_routes; r++)
			//{
			//	cout<<"Route changed are: "<< ' ';
			//	if(RchangedStatus[r] == true)
			//	{
			//		cout<<r<<' ';
			//	}	
			//	cout<<endl; 
			//}
			//cout<<"After shake " <<Neighbour_k <<endl;
			
			bool same_route;
			
			//========================= To update cost_of_removing, gain_matrix[] and info-matrix[][] after shake===============================//
			partialupdate_costremoving(cost_of_removing);//update cost_of_removing according to RchangedStatus
			partialupdate_reoptimize_1_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			partialupdate_reoptimize_1_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			partialupdate_reoptimize_2_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			partialupdate_reoptimize_2_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			partialupdate_reoptimize_2_2swap(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			partialupdate_reoptimize_two_optintra(same_r_info, gain_matrix_1_0);
			partialupdate_reoptimize_two_optinter(info_1_0, gain_matrix_1_0);
			partialupdate_reoptimize_crossTail2(info_1_0, gain_matrix_1_0);
			//partialupdate_reoptimize_CROSS(info_1_0, gain_matrix_1_0);

			reinitializeRchangedStatus (); //initialize to 0 for every route


			float max_gain = INT_MIN;	
			do
			{
				int num_positiveGain=0;
				for (int i = 0; i < SIZE; i++)//reinitialize
				{
					gain[i] = 0;
					modul[i] = 0;
				}
	
				//============== To record frequency for Adaptive VNS =========================//
				int k=0;//for gain[] //to count how many positive gain
				//=============== Record the gain for each module =============================//
				for (int i = 0; i < no_routes; i++) 
				{
					for (int m = i ; m < no_routes; m++)//only read the upper diagonal of matrix which record the gain
					{
						if (gain_matrix_1_0[i][m] > epsilon)//if it is positive gain, record the gain to be added to the frequency
						{
							gain[k] = gain_matrix_1_0[i][m];
							if (i!=m) //if different route, the info is stored in info_1_0[][]
							{
								number = gain_matrix_1_0[m][i];//number is the positional index(lower diagonal) in info_1_0[][]
								modul[k] = info_1_0[number][8];//record the modul which is stores in info_1_0[][]
						
							}
							else if (i==m)//if same route, the info is stored in same_r_info[][]
							{
								number = i; //number is the route number 
								modul[k] = same_r_info[number][8];//record the modul which is stores in same_r_info[][]	
							}
							k++;
							
						}
					}
				}
				num_positiveGain = k;
				//gain[0], gain[1], gain[2],..., gain[num_positiveGain]   correspond to   modul[0], modul[1], modul[2] ,..., modul[num_positiveGain] 

				for (int i = 0; i < num_positiveGain; i++) //starts from 1, be careful of heap error, must declare freq[max_neigbor+1]
				{
					int m = modul[i]; //from which module // m record the modul, so m only takes 1, 2, ..., max modul
					freq[m] = freq[m] + (gain[i]/max_gain); //max_gain has no value at the moment!!!!!!!!!!!!! found this error on 1Nov2015!!!!!!!
				}
		
				for (int i = 0; i < num_positiveGain; i++)
				{
					recordFreq<<gain[i] << ' '<<' ' <<modul[i]<<endl;
				}
				//============== End of record frequency for Adaptive VNS =========================//
				//============================================to find the best gain out of all modules=============================================//
				max_gain = INT_MIN;
				same_route = false; // reinitialize
				for (int i = 0; i < no_routes; i++)
				{
					for (int m = i ; m < no_routes; m++)//////////////////////////////////////////////////////////////////////////////////////////////
					{
						if (gain_matrix_1_0[i][m]-max_gain > epsilon) //gain_matrix_1_0[i][m] > max_gain
						{
							if (i!=m)
							{
								max_gain = gain_matrix_1_0[i][m];
								number = gain_matrix_1_0[m][i];
								module = info_1_0[number][8];
	

								same_route = false;
							}

							else if (i==m) //same route, read from same_r_info[][]
							{
								max_gain = gain_matrix_1_0[i][m];
								number = i;
								module = same_r_info[number][8];

	
								same_route = true;
							}
						}
					}
				}

				//=================================insert to corresponding module based on the highest gain=====================================//
				//need this because last iteration when max_gain is negative, the do-while loop will not jumpt out directly, it will execute the rest of the code and stop at the end.
				if (max_gain > epsilon)//if highest gain is positive
				{
					for (int i = 0; i < num_positiveGain; i++) //starts from 1, be careful of heap error, must declare freq[max_neigbor+1]
					{
						int m = modul[i]; //from which module // m record the modul, so m only takes 1, 2, ..., max modul
						freq[m] = freq[m] + (gain[i]/max_gain); //put here on 1Nov2015!!!!!!!
					}
					recordFreq<<"max_gain= "<<max_gain<<endl;
					for (int i = 0; i < num_positiveGain; i++)
					{
						recordFreq<<gain[i] << ' '<<' ' <<modul[i]<<endl;
					}

					if (same_route == false)
					{
						RchangedStatus[(int)info_1_0[number][2]] = true;//route (from)
						RchangedStatus[(int)info_1_0[number][3]] = true;//route (to)
					}

					else
					{	
						RchangedStatus[(int)same_r_info[number][2]] = true;//route (from)
					}
					//route_file<< endl<<"========================" <<endl;
					//route_file<< "max_gain = " <<max_gain <<endl;
					//route_file<< "Routes before insert "<<endl;
					float tempc=0.0;
					for (int i = 0; i < no_routes; i++)
					{
						//route_file<< i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
						//for (int j = 0; j <= saiz[i]+1; j++)
						//{
						//		route_file<< VEHICLE[i][j]<<' ';
						//}
						//route_file<<endl;
						tempc += route_cost[i];

					}
					//route_file<< "Total cost before insert= "<<tempc<<endl;
					//route_file<< "max_gain = " <<max_gain <<endl;
					//route_file<<"module = " << module<<' '<<"same_route= "<<same_route<<endl;
					//if (same_route==true)
					//	route_file<<" r1= "<<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
					//else
					//	route_file<<" r1= "<<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r1w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
					cout<< "max_gain = " <<max_gain <<' '<<"module = " << module<<' '<<"same_route= "<<same_route<<endl;
					cout<<"========== max gain > 0===========" << ' ' <<"module = " << module<<' '<<"same_route= "<<same_route<<endl;
				
					if (module == 1)
					{	
						//if(same_route == false)
						//	cout<< "before insert 1_0, r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r1w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
						//else 
						//	cout<< "before insert 1_0, r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
						insert_1_0(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);
					}

					else if (module == 2)
					{
						//if(same_route == false)
						//	cout<< "before insert 1_1, r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r1w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
						//else 
						//	cout<< "before insert 1_1, r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
						insert_1_1(max_gain, cost_of_removing,  info_1_0, same_r_info, VEHICLE, same_route);
					}

					else if (module == 3)
					{
						//if(same_route == false)
						//	cout<< "before insert 2_1, r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r2w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
						//else 
						//	cout<< "before insert 2_1, r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
						insert_2_1(max_gain,  cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);
					}
					else if (module == 4)
					{
						//if(same_route == false)
						//	cout<< "before insert 2_0, r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r2w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
						//else 
						//	cout<< "before insert 2_0, r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
						insert_2_0(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);
					}
					else if (module == 5)
					{
						//if(same_route == false)
						//	cout<< "before insert 2_2swap, r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r2w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
						//else 
						//	cout<< "before insert 2_2swap, r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
						insert_2_2swap(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);
					}
					else if (module == 6)
					{
						//cout<< "before insert 2optIntra, r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
						insert_2optintra(max_gain, cost_of_removing, same_r_info, VEHICLE);
					}
					else if (module == 7)
					{	
						//cout<< "before 2optInter, r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r2w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
						insert_2optinter(max_gain, cost_of_removing, info_1_0, VEHICLE);
					}
					else if (module == 8)
					{			
						//cout<< "before insert_crosstail, r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r2w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
						insert_crosstail(max_gain, cost_of_removing, info_1_0, VEHICLE);
					}
					else if (module == 9)
					{	
						//cout<< "before insert_cross, r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r2w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
						insert_cross(max_gain, cost_of_removing, info_1_0, VEHICLE);
					}
					//update cost_of_removing, gain_matrix[][] after insert
					//partialupdate_costremoving(cost_of_removing); //all cost of removing are updated in the insert function, make comment on 13Oct2015
					partialupdate_reoptimize_1_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
					partialupdate_reoptimize_1_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
					partialupdate_reoptimize_2_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
					partialupdate_reoptimize_2_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
					partialupdate_reoptimize_2_2swap(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
					partialupdate_reoptimize_two_optintra(same_r_info, gain_matrix_1_0);
					partialupdate_reoptimize_two_optinter(info_1_0, gain_matrix_1_0);
					partialupdate_reoptimize_crossTail2(info_1_0, gain_matrix_1_0);
					//partialupdate_reoptimize_CROSS(info_1_0, gain_matrix_1_0);

					reinitializeRchangedStatus ();//initialize to 0 for every route

				}//end if max_gain is positive

			} while (max_gain > epsilon);
	
		//route_file << "No more positive gain, check solution and go to next neighbourhood"<<endl;
		cout << "No more positive gain, check solution and go to next neighbourhood"<<endl;
		skip_LS://if no feasible shake in last shake, skip LS, must put before display route because we need the value total_cost
		
		//check reverse order
		float reversecost=0;
		for (int i = 0; i < no_routes; i++)
		{
			reversecost = saiz[i]*distance_cost[i] + saiz[i]*SERVICE_T - route_cost[i];

			if (reversecost+epsilon < route_cost[i])//if reverse is better
			{
				cout<<"Reversed order is better"<<endl;
				route_cost[i] = reversecost;
			
				int count = saiz[i]+2;
				int temp;

				for (int t = 0; t < count/2; t++)
				{
					temp = VEHICLE[i][count-t-1];
					VEHICLE[i][count-t-1] = VEHICLE[i][t];
					VEHICLE[i][t] = temp;

				}
				CumDist[i][0] =0 ;
				for (int j = 1; j <= saiz[i]+1; j++)
				{
					CumDist[i][j] = CumDist[i][j-1]+dist[VEHICLE[i][j-1]][VEHICLE[i][j]] + service_time[VEHICLE[i][j]];
				
				}
				//if reversed order is successful, update cost_of_removing because cost of removing will be copied if this solution is LOCAL_BEST
				int before = -1, after = -1;
				for (int h = 1; h <= saiz[i]; h++) //first customer start from element [1]
				{
					before = VEHICLE[i][h - 1];
					after = VEHICLE[i][h + 1];	
					float origain = dist[VEHICLE[i][h]][before] + dist[VEHICLE[i][h]][after] + service_time[VEHICLE[i][h]] - dist[before][after];//old-new
					int remainingcust = saiz[i]-h;
					if (remainingcust < 0)//if negative
						remainingcust= 0;
					cost_of_removing[VEHICLE[i][h]] = CumDist[i][h] + remainingcust*origain;//cannot update from a certain start point because cost_of_removing for the entire route change when a customer is deleted from the route, because number_of_remaining customer changes
				}
				getchar();
			}
		}

		//======================================================Display route========================================================================
			cout << "==================Routes (in VNS)===================================== " << endl;
			float total_cost = 0.0;
			int total_cust = 0;
			for (int g = 0; g < no_routes; g++)
			{
				if (saiz[g] == 0)
				{
					route_cost[g] = 0;//just to avoid very small number here
					distance_cost[g] = 0;//just to avoid very small number here
					total_demand[g]=0;
				}
				total_cust = total_cust + saiz[g];
				cout<<g<<' '<<route_cost[g]<<' '<<saiz[g]<<' '<<total_demand[g]<<' '<<' ';
				for (int h = 0; h <= saiz[g]+1; h++)
				{
					cout << VEHICLE[g][h] << ' ';
				}
	
				total_cost = total_cost + route_cost[g];
				cout << endl;
			}
			cout << "Total customers = " << total_cust << endl;
			cout << "Total cost = " << total_cost << endl;

			cout<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl;
			cout<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
			//getchar();

			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ for infeasibility check @@@@@@@@@@@@@@@@@@@@@@@@//
			//for (int i = 0; i < no_routes; i++)
			//{
			//	if ((total_demand[i]>CAPACITY) || (distance_cost[i] > DISTANCE + epsilon))//for infeasibility check
			//	{
			//		//violation << "In VNS best improvement after module "<< Neighbour_k <<endl;
			//		//violation << i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
			//		//for (int j = 0; j <= saiz[i]+1; j++)
			//		//{
			//		//	violation << VEHICLE[i][j]<<' ';
			//		//}
			//		//violation<<"capacity= "<<total_demand[i]<<' '<<"cost= "<<route_cost[i]<<endl;
			//		//violateThreshold = violateThreshold/2; //divide by half if the solution after LS still violate constraint

			//		Neighbour_k++;
			//		goto skip_check_local;

			//	}
			//}
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ End of (for infeasibility check) @@@@@@@@@@@@@@@@@@@@@@@@//
			
			//Check solution with LOCAL_BEST
			//PROBLEM: THE BEST SOLUTION IS ALREADY RECORDED IN FUNCTION INSERT, THAT'S WHY IT NEVER ENTER HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!
			if (total_cost + epsilon < LOCAL_BEST_COST)//total_cost < LOCAL_BEST_COST
			//if (total_cost < BEST_COST)
			{
				//sort_solution(VEHICLE);
				LOCAL_BEST_COST = total_cost;
				cout << endl << "BEST COST IS " << LOCAL_BEST_COST << endl;
		
				for (int i = 0; i < no_routes; i++)
				{		
					for (int j = 0; j <= saiz[i]+1; j++)
					{
						LOCAL[i][j] = VEHICLE[i][j];
						LOCALCumDist[i][j] = CumDist[i][j];
					}
					LOCAL_SAIZ[i] = saiz[i]; 
					LOCAL_capa[i] = total_demand[i];
					LOCAL_Rcost[i] = route_cost[i];
					LOCAL_distance_cost[i] = distance_cost[i];
				}

				if (no_routes != LOCAL_NO_ROUTE)
				{
					cout<<"ERROR no_routes != LOCAL_NO_ROUTE in VNS best improvement)"<<endl;
					getchar();
				}

				//=================================record localbest solution in txt=====================================//
				cout << "========================================================================== " << endl;
				//=================================display final solution=====================================//
				for (int i = 0; i < LOCAL_NO_ROUTE; i++)
				{
					localbest_sol << i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
					for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
					{
						localbest_sol << LOCAL[i][j] << ' ';
					}
					localbest_sol<<endl;
				}
				localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"in VNS best improvement" <<"after shake Neighbour_k "<<Neighbour_k << endl;	
	
				//======================= End of record localbest solution ===========================//
				
				//gain_matrix<<"FOUND LOCAL BEST, UPDATE ORIGAIN"<<endl;
				//once found local best, copy to oriGain, //if not better than local best solution and neighbourhood havent finished, use the original gain because shaking is based on local best, oriGain is updated evrytime found local best
				//============================= copy to oriGain ================================//
				for (int i = 0; i < SIZE; i++)
				{
					oriGain.Oricost_of_removing[i] = cost_of_removing[i];//if found local best, Oricost_of_removing always hold LOCAL
				}
				////!!!!!!!!!!!!!!!!!!!!!!!!!!!! check cost of removing !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
				//cout<<"1. Found local best, Entering checkcost_of_removing"<<endl;	
				//float *checkcost_of_removing = new float [SIZE];
				//	for (int f = 0; f < no_routes; f++)
				//	{
				//		if (saiz[f]==0)
				//			continue;
				//		int before = -1, after = -1;
				//		for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
				//		{
				//			before = VEHICLE[f][h - 1];
				//			after = VEHICLE[f][h + 1];	
				//			float origain = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];//old-new
				//			int remainingcust = saiz[f]-h;
				//			if (remainingcust < 0)//if negative
				//				remainingcust= 0;
				//			checkcost_of_removing[VEHICLE[f][h]] = CumDist[f][h] + remainingcust*origain;//cannot update from a certain start point because cost_of_removing for the entire route change when a customer is deleted from the route, because number_of_remaining customer changes
				//			if ( (checkcost_of_removing[VEHICLE[f][h]] - cost_of_removing[VEHICLE[f][h]]) > 0.1)
				//			{
				//				cout<<"Diff"<<' '<<VEHICLE[f][h]<<' '<<checkcost_of_removing[VEHICLE[f][h]]<<' '<<cost_of_removing[VEHICLE[f][h]]<<endl;
				//			}
				//		}
				//	}
				//	getchar();
				//	delete[] checkcost_of_removing;
				//	//!!!!!!!!!!!!!!!!!!!!!!!!!!!! check cost of removing !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//


				for (int i = 0; i < no_routes; i++)
				{
					for (int j = 0; j < no_routes; j++)
					{
						oriGain.Origain_matrix_1_0[i][j] = gain_matrix_1_0[i][j];	
					}
					for (int j = 0; j < num_attributes; j++)
					{
						oriGain.Orisame_r_info[i][j] = same_r_info[i][j];
					}
				}
				for (int i = 1; i <= s; i++)
				{
					for (int j = 0; j < num_attributes; j++)
					{
						oriGain.Oriinfo_1_0[i][j] = info_1_0[i][j];
					}
				}
				calculate_centreGravity(x, y);
				foundLOCALinCurrNeighbor = true; //if found LOCAL BEST, use local best, else just use current VEHICLE
				Neighbour_k = 1;

				//if ( (LOCAL_BEST_COST < LOCAL_BCOST) && (LOCAL_BEST_COST != GLOBAL_BEST_COST) )//to record the best of LOCAL for diversification purpose
				//{
				//	//record to LOCAL_B
				//	LOCAL_BCOST = LOCAL_BEST_COST;
				//	int r = 0; //for LOCAL_B because do not consider empty route
				//	for (int i = 0; i < LOCAL_NO_ROUTE; i++)
				//	{
				//		if (LOCAL_SAIZ[i] == 0)
				//			continue;
				//		for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
				//		{
				//			LOCAL_B[r][j] = LOCAL[i][j];
				//			LOCAL_BCumDist[r][j] = LOCALCumDist[i][j];
				//		}
				//		LOCAL_Bs[r] = LOCAL_SAIZ[i];
				//		LOCAL_Bc[r] = LOCAL_Rcost[i];
				//		LOCAL_Bq[r] = LOCAL_capa[i];
				//		LOCAL_Bdistance_cost[r] = LOCAL_distance_cost[i];
				//		r++;
				//	}
				//	LOCAL_Br = r;
				//}
				
			}//end if (total cost < LOCAL_BEST_COST)
			else  //if is is not better than the incumbent
			{
				Neighbour_k++;
				//******************** SKIP DIJKSTRA REFINEMENT *****************************
				//if (Neighbour_k > max_neigbor)
				//{
				//	float cost = 0.0; //to be passed to dijkstra_refinement
				//	//dijkstra_refinement always refine the LOCAL_BEST_SOL (incumbent best)
				//	//vehicle[][] is not important, it will be reinitialize in dijkstra_refinement, and passed out from there
				//	localbest_sol << "Entering dijkstra refinement in VNS best improvement "<<endl;
				//	
				//	if(foundLOCALinCurrNeighbor == true)//if found LOCAL BEST, use local best, else just use current VEHICLE
				//	{
				//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
				//		{
				//			
				//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
				//			{
				//				VEHICLE[i][j] = LOCAL[i][j];
				//				CumDist[i][j] = LOCALCumDist[i][j];
				//			}
				//			
				//			distance_cost[i] = LOCAL_distance_cost[i];
				//			route_cost[i] = LOCAL_Rcost[i];
				//			saiz[i] = LOCAL_SAIZ[i];
				//			total_demand[i] = LOCAL_capa[i];
				//			space_available[i] = CAPACITY - total_demand[i];
				//			distance_available[i] = DISTANCE - distance_cost[i];
				//		}
				//		no_routes = LOCAL_NO_ROUTE;
				//	}
				//	
				//	cout<<"cost after dijkstra_refinement= "<<cost<<endl;
				//	//=================== if better than LOCAL_BEST, compare route  by route to see if the route has changed =====================//
				//	//I want to avoid calculating the bestGain for every route because the solution after Dijkstra Refinement is very similar to the original solution.
				//	//**************************************** If Dijkstra refinement found better Local_best ***************************************//
				//	if (LOCAL_BEST_COST - cost > epsilon)//cost < LOCAL_BEST_COST //is my LOCAL_BEST sorted???
				//	{
				//		cout<< "Found new LOCAL_BEST right after Dijkstra REfinement"<<endl;
				//		calculate_best_gain(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0, oriGain);
				//		//============ if route is different from 
				//		//============= Record the best solution ====================//
				//		LOCAL_BEST_COST = cost;
				//		LOCAL_NO_ROUTE = no_routes;
				//		cout << endl << "LOCAL_BEST COST IS " << LOCAL_BEST_COST << endl;
				//		for (int i = 0; i < no_routes; i++)
				//		{
				//			for (int j = 0; j <= saiz[i]+1; j++)
				//			{
				//				LOCAL[i][j] = VEHICLE[i][j];
				//				LOCALCumDist[i][j] = CumDist[i][j];
				//			}
				//			LOCAL_SAIZ[i] = saiz[i]; //copy temp saiz[] to best SAIZ[]
				//			LOCAL_capa[i] = total_demand[i]; // capa[] record the BEST_ROUTE demand
				//			LOCAL_Rcost[i] = route_cost[i];
				//			LOCAL_distance_cost[i] = distance_cost[i];
				//		}
				//		if ( (LOCAL_BEST_COST < LOCAL_BCOST) && (LOCAL_BEST_COST != GLOBAL_BEST_COST) )//to record the best of LOCAL for diversification purpose
				//		{
				//			//record to LOCAL_B
				//			LOCAL_BCOST = LOCAL_BEST_COST;
				//			int r = 0; //for LOCAL_B becuase do not consider empty route
				//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
				//			{
				//				if (LOCAL_SAIZ[i] == 0)
				//					continue;
				//				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
				//				{
				//					LOCAL_B[r][j] = LOCAL[i][j];
				//					LOCAL_BCumDist[r][j] = LOCALCumDist[i][j];
				//				}
				//				LOCAL_Bs[r] = LOCAL_SAIZ[i];
				//				LOCAL_Bc[r] = LOCAL_Rcost[i];
				//				LOCAL_Bq[r] = LOCAL_capa[i];
				//				LOCAL_Bdistance_cost[r] = LOCAL_distance_cost[i];
				//				r++;
				//			}
				//			LOCAL_Br = r;
				//		}
				//		calculate_centreGravity(x, y);
				//		//int empty_route = original_route - no_routes; //no_routes is after dijkstra_partition
				//		//=================================record localbest solution in txt=====================================//
				//		cout << "========================================================================== " << endl;
				//		//=================================display final solution=====================================//
				//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
				//		{
				//			cout<< i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
				//			localbest_sol << i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
				//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
				//			{
				//				cout<< LOCAL[i][j] << ' ';
				//				localbest_sol << LOCAL[i][j] << ' ';
				//			}
				//			localbest_sol <<endl;
				//			cout<<endl;
				//		}
				//		localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
				//		cout << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
				//		//======================= End localrecord best solution ===========================//
				//		//gain_matrix<<"FOUND LOCAL BEST AFTER DIJKSTRA REFINEMENT, UPDATE ORIGAIN"<<endl;
				//		//============================= copy to oriGain because oriGain always hold the gain for local_best, it is used in shaking later================================//
				//		for (int i = 0; i < SIZE; i++)
				//		{
				//			oriGain.Oricost_of_removing[i] = cost_of_removing[i];
				//		}
				//		for (int i = 0; i < no_routes; i++)
				//		{
				//			for (int j = 0; j < no_routes; j++)
				//			{
				//				oriGain.Origain_matrix_1_0[i][j] = gain_matrix_1_0[i][j];	
				//			}
				//			for (int j = 0; j < num_attributes; j++)
				//			{
				//				oriGain.Orisame_r_info[i][j] = same_r_info[i][j];
				//			}
				//		}
				//		for (int i = 1; i <= s; i++)
				//		{
				//			for (int j = 0; j < num_attributes; j++)
				//			{
				//				oriGain.Oriinfo_1_0[i][j] = info_1_0[i][j];
				//			}
				//		}
				//		Neighbour_k = 1;
				//		foundLOCALinCurrNeighbor = false; //initialize to false when found LOCAL solution after Dijkstra refinement, so it will not go through Dijkstra refinement later
				//		goto skip_check_local; //if Dijkstra refinement produce better result, noneed to copy original gain matrix because we use this solution now
				//	}//end if (cost < LOCAL_BEST_COST) //**************************************** end If Dijkstra refinement found better Local_best ***************************************//
				//}//if (Neighbour_k > max_neigbor), do Dijkstra refinement based on vehicle, not LOCAL_BEST
			
				//if not better than local best solution and neighbourhood havent finished, use the original gain because shaking is based on local best, oriGain is updated evrytime found local best
				//============================= copy from oriGain ================================//
				cout<<"Entering copy from oriGain because shaking is based on local best"<<endl;
				for (int f = 0; f < SIZE; f++) //find cost of removing for each customer and put in array
				{
					cost_of_removing[f] = oriGain.Oricost_of_removing[f];//copy from LOCAL Oricost_of_removing because always shake from local solution
				}
				
				////!!!!!!!!!!!!!!!!!!!!!!!!!!!! check cost of removing !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
				//	cout<<"2. Entering checkcost_of_removing"<<endl;
				//float *checkcost_of_removing = new float [SIZE];
				//	for (int f = 0; f < LOCAL_NO_ROUTE; f++)
				//	{
				//		int before = -1, after = -1;
				//		for (int h = 1; h <= LOCAL_SAIZ[f]; h++) //first customer start from element [1]
				//		{
				//			before = LOCAL[f][h - 1];
				//			after = LOCAL[f][h + 1];	
				//			float origain = dist[LOCAL[f][h]][before] + dist[LOCAL[f][h]][after] + service_time[LOCAL[f][h]] - dist[before][after];//old-new
				//			int remainingcust = LOCAL_SAIZ[f]-h;
				//			if (remainingcust < 0)//if negative
				//				remainingcust= 0;
				//			checkcost_of_removing[LOCAL[f][h]] = LOCALCumDist[f][h] + remainingcust*origain;//cannot update from a certain start point because cost_of_removing for the entire route change when a customer is deleted from the route, because number_of_remaining customer changes
				//			if ( (checkcost_of_removing[LOCAL[f][h]] - cost_of_removing[LOCAL[f][h]]) > 0.1)
				//			{
				//				cout<<"Diff"<<' '<<LOCAL[f][h]<<' '<<checkcost_of_removing[LOCAL[f][h]]<<' '<<cost_of_removing[LOCAL[f][h]]<<endl;
				//			}
				//		}
				//	}
				//	getchar();
				//	delete[] checkcost_of_removing;
				//	//!!!!!!!!!!!!!!!!!!!!!!!!!!!! check cost of removing !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//


				for (int i = 0; i < no_routes; i++)
				{
					for (int j = 0; j < no_routes; j++)
					{
						gain_matrix_1_0[i][j] = oriGain.Origain_matrix_1_0[i][j];	
					}
					for (int j = 0; j < num_attributes; j++)
					{
						same_r_info[i][j] = oriGain.Orisame_r_info[i][j];
					}
				}
				s = ((no_routes*no_routes)-no_routes)/2;
				for (int i = 1; i <= s; i++)
				{
					for (int j = 0; j < num_attributes; j++)
					{
						info_1_0[i][j] = oriGain.Oriinfo_1_0[i][j];
					}
				}

				//gain_matrix << "AFTER COPY FROM ORIGAIN"<<endl<<endl;
				//for (int i = 0; i < no_routes; i++)
				//{
				//	for (int j = 0; j < no_routes; j++)
				//	{
				//		gain_matrix << gain_matrix_1_0[i][j] << ' ';
				//	}
				//	gain_matrix << endl;
				//}
				//gain_matrix<<endl<<endl;
				//for (int i = 1; i <= s; i++)
				//{
				//	for (int j = 0; j < num_attributes; j++)
				//	{
				//		gain_matrix<<info_1_0[i][j]<<' ';
				//	}
				//	gain_matrix<<endl;
				//}
				//gain_matrix<<endl<<endl;
				//for (int i = 0; i < no_routes; i++)
				//{
				//	for (int j = 0; j < num_attributes; j++)
				//	{
				//		gain_matrix<<same_r_info[i][j]<<' ';
				//	}
				//	gain_matrix<<endl;
				//}
				//gain_matrix<<endl<<endl;
				reinitializeRchangedStatus();//initialize to 0 for every route
				//============================= End copy from oriGain ================================//

			}//end else //if is is not better than the incumbent
skip_check_local:;
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)//added 05/06/2016
			{
				delete[] route_CGravity[i]; 
				delete[] sorted_custRGravity[i];
			}
			delete[] route_CGravity;
			delete[] custRGravity;
			delete[] sorted_custRGravity;
		} //end while (Neighbour_k <= max_neighbor) @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	
		
		if ((GLOBAL_BEST_COST - LOCAL_BEST_COST) > epsilon)//LOCAL_BEST_COST < GLOBAL_BEST_COST
		{
			GLOBAL_BEST_COST = LOCAL_BEST_COST;
			cout << endl << "GLOBAL_BEST COST IS " <<GLOBAL_BEST_COST << endl;
			found_GLOBEST_status = 1; //for diversificaton use

			int r = 0; //for GLOBAL_NO_ROUTE becuase do not consider empty route
			srand ( time(NULL) ); //seed it
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				if (LOCAL_SAIZ[i] == 0)
					continue;
				
				
				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
				{
					GLOBAL[r][j] = LOCAL[i][j];
					GLOBALCumDist[r][j] = LOCALCumDist[i][j];
				}
				
				GLOBAL_SAIZ[r] = LOCAL_SAIZ[i];
				GLOBAL_Rcost[r] = LOCAL_Rcost[i];
				GLOBAL_distance_cost[r] = LOCAL_distance_cost[i];
				GLOBAL_capa[r] = LOCAL_capa[i];
				r++;
			}
			GLOBAL_NO_ROUTE = r;
			
			sort_GLOBALsolutionSMALLEST(GLOBAL); 
			//=================================record the best solution=====================================//
			cout << "========================================================================== " << endl;

			ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
			//=================================display final solution=====================================//
			for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
			{
				best_sol << i <<' '<< GLOBAL_Rcost[i] <<' '<< GLOBAL_SAIZ[i] <<' '<< GLOBAL_capa[i]<<' '<<' ';
				cout << i <<' '<< GLOBAL_Rcost[i] <<' '<< GLOBAL_SAIZ[i] <<' '<< GLOBAL_capa[i]<<' '<<' ';
				for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
				{
					best_sol << GLOBAL[i][j] << ' ';
					cout << GLOBAL[i][j] << ' ';
				}
				best_sol <<  endl;
				cout << endl;	
			}

			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
			cout << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
			best_sol << "From VNS best improvement after Neighbour_k " <<Neighbour_k<<' ' <<' '<<"NbDiv= "<<NbDiv<<  endl;
			cout<<"Found Global best"<<endl;
			//getchar();
		}

		//============================== DIVERSIFICATION PHASE STARTS HERE ====================================//
		//copy to vehicle[][] because vehicle[][] not necessarily holds GLOBAL BEST now 
		//if GLOBAL_BEST  is not found difference between LOCAL_B and GLOBAL_BEST is small
		//if ( (LOCAL_BCOST > GLOBAL_BEST_COST) && abs(LOCAL_BCOST-GLOBAL_BEST_COST)<= (0.01*GLOBAL_BEST_COST))//if LOCAL_B is a good solution, diversify LOCAL_B
		//{
		//	for (int i = 0; i < LOCAL_Br; i++)
		//	{	
		//		for (int j = 0; j <= LOCAL_Bs[i]+1; j++)
		//		{
		//			VEHICLE[i][j] = LOCAL_B[i][j];
		//			CumDist[i][j] = LOCAL_BCumDist[i][j];

		//		}
		//		distance_cost[i] = LOCAL_Bdistance_cost[i];
		//		saiz[i] = LOCAL_Bs[i];
		//		route_cost[i] = LOCAL_Bc[i];
		//		total_demand[i] = LOCAL_Bq[i];
		//		space_available[i] = CAPACITY - total_demand[i];
		//		distance_available[i] = DISTANCE - distance_cost[i];
		//	}
		//	no_routes = LOCAL_Br;
		//
		//}
		//else
		//{
		
			for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
			{		
				
				for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
				{
					VEHICLE[i][j] = GLOBAL[i][j];
					CumDist[i][j] = GLOBALCumDist[i][j];
				}
				distance_cost[i] = GLOBAL_distance_cost[i];
				saiz[i] = GLOBAL_SAIZ[i];
				route_cost[i] = GLOBAL_Rcost[i];
				total_demand[i] = GLOBAL_capa[i];
				space_available[i] = CAPACITY - total_demand[i];
				distance_available[i] = DISTANCE - distance_cost[i];
			}
			no_routes = GLOBAL_NO_ROUTE;
		//}

		//************************************************ NO DIVERSIFICATION *************************************************//	

		//route_file<<"ENTERING  DIVERSIFICATION PHASE "<<NbDiv<<endl;
		cout<<"ENTERING  DIVERSIFICATION PHASE "<<NbDiv<<endl;
		for (int i = 0; i < no_routes; i++)
		{
			cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
			for (int j = 0; j <= saiz[i]+1; j++)
			{
				cout<<VEHICLE[i][j]<<' ';
			}
			cout<<endl;
		}
		cout<<"GLOBAL_BEST_COST= "<<GLOBAL_BEST_COST;
		float cost = 0;
		//int div=0;
		//vehicle[][] to be passed in to make_giant_tour() is not important, make_giant_tour always consider LOCAL_BEST_SOL (incumbent best solution)
		//if (found_GLOBEST_status == 1)
		//{
		//	cost = make_giant_tour(VEHICLE); //pass in GLOBAL_BEST_ROUTE< only overwrite when better one found
		//	Ndel_min = LB_Ndel_min;
		//}
		//else if (found_GLOBEST_status == 0) //if not found global_best
		//{
			NbDiv++;
			if (NbDiv > maxDiv) //if this is last diversification, dont perform LNS because it will not go through the LS //added on 12 Oct2015
			{
				cout<<"goto lastdiversify in VNS"<<endl;
				goto lastdiversify;//check if this one ever executed or change to if (NbDiv == maxDiv) 
			}
			cout<<endl<<"Entering Diversify in VNS best improvement: ";
			//div=(rand() % 4); 
			if (div == RS)//execuste LNS one by one, if more than the existing number, go back to the first one [0]
			{
				div=0;
			}
			if (divsequence[div] == 0)
			{
				cout<<"LNS Diversification 3"<<endl;
				cost = Diversify_LNS3(VEHICLE, Ndel_min);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div] <<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			else if (divsequence[div] == 1)
			{
				cout<<"LNS Diversification overlap route2"<<endl;
				cost = Diversification_overlapRoute2(VEHICLE, Ndel_min, x, y);
				//cost = Diversify_LNS3(VEHICLE, Ndel_min);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div] <<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			else if (divsequence[div] == 2)
			{
				//cout<<"Diversify_RelatednessDemand "<<endl;
				//cost = Diversify_RelatednessDemand(VEHICLE, Ndel_min);
				cout<<"Diversify_EntireRoute "<<endl;
				cost = Diversify_EntireRoute(VEHICLE,  Ndel_min, x, y) ;
				//cost = Diversify_LNS3(VEHICLE, Ndel_min);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div] <<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			else if (divsequence[div] == 3)
			{
				cout<<"LNS Diversification LongestArc"<<endl;
				cost = Diversify_LongestArc2(VEHICLE, Ndel_min);
				//cost = Diversify_LNS3(VEHICLE, Ndel_min);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div] <<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			else if (divsequence[div] == 4)
			{
				cout<<"Conflict sector Diversification: "<<endl;
				cost = Diversification_conflict_sector(VEHICLE, Ndel_min, x, y);
				//cost = Diversify_LNS3(VEHICLE, Ndel_min);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div]<<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			else if (divsequence[div] == 5)
			{
				cout<<"Diversify_BadArcNR Diversification: "<<endl;
				cost = Diversify_BadArcNR(VEHICLE, Ndel_min);
				//cost = Diversify_LNS3(VEHICLE, Ndel_min);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div] <<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			int incre = ceil(0.01*(float)SIZE);
			Ndel_min+=incre;
			if (Ndel_min > UB_Ndel_min)
				Ndel_min = UB_Ndel_min; //added this on 2March2016
		//}
		//AFter diversification, record in LOCAL_BEST_ROUTE, when it enters restartAfterDiver, it will restart with add empty route to LOCAL_BEST_ROUTE again
		sort_solutionSMALLEST (VEHICLE);

			//========for checking =============//
			for (int r = 0; r < no_routes; r++)
			{
				float checkR_cost=0;
				int checkR_dmd=0;
				float checkR_dist=0;

				for (int j = 1; j <=saiz[r]; j++)//until th elast customer, last depot no need to calculate 
				{
					checkR_cost += CumDist[r][j];
					checkR_dmd += demand[VEHICLE[r][j]];
					checkR_dist += dist[VEHICLE[r][j-1]][VEHICLE[r][j]] + service_time[VEHICLE[r][j]];
				}	
				checkR_dist += dist[VEHICLE[r][saiz[r]]][VEHICLE[r][saiz[r]+1]];
				if (abs(distance_cost[r]-checkR_dist) > BIGC)
				{
					cout<<r<<"  After diversification, This dist is different from original "<<endl;
					cout<<distance_cost[r]<<' '<<checkR_dist<<' ';
					getchar();
				}
				if (abs(route_cost[r]-checkR_cost) > BIGC)
				{
					cout<<r<<"   After diversification, This cost is different from original "<<endl;
					cout<<route_cost[r]<<' '<<checkR_cost<<' ';
					getchar();
				}
				if (checkR_dmd != total_demand[r])
				{
					cout<<r<<"  After diversification, This demand is different from original "<<endl;
					cout<<checkR_dmd<<' '<<total_demand[r]<<' ';
					getchar();
				}
			}
			//========for checking =============//
			

		int k=0;//for LOCAL[][] route index
		for (int i = 0; i < no_routes; i++)
		{
			if (saiz[i] == 0)
				continue;
			
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					LOCAL[k][j] = VEHICLE[i][j];
					LOCALCumDist[k][j] = CumDist[i][j];
				}
			LOCAL_SAIZ[k] = saiz[i];
			LOCAL_capa[k] = total_demand[i];
			LOCAL_Rcost[k] = route_cost[i];
			LOCAL_distance_cost[k] = distance_cost[i];
			k++;
		}
		LOCAL_BEST_COST = cost;
		LOCAL_NO_ROUTE = k;
		//AFter diversification, reinitialize LOCAL_B so that in new iteration, it will start recording the best of all neighbourhood solutions
		//LOCAL_BCOST = INT_MAX;
		//for (int i = 0; i < LOCAL_Br; i++)
		//{
		//	for (int j = 0; j <= LOCAL_Bs[i]+1; j++)
		//	{
		//		LOCAL_B[i][j]=-1;
		//	}
		//}
		cout<<"AFTER DIVERSIFICATION"<<endl;
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			cout<<i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
			{
				cout<<LOCAL[i][j]<<' ';
			}
			cout<<endl;
		}
		cout<<"LOCAL_BEST_COST ="<<LOCAL_BEST_COST<<endl;
		//getchar();
		//if the cost better than GLOBAL_BEST_COST 
		if (GLOBAL_BEST_COST-cost > epsilon)//cost < GLOBAL_BEST_COST
		{
			noImproveDiv = 0;//reinitialize to 0 if found GLOBAL_BEST
			int m=0;//for GLOBAL[][] route index
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i] == 0)
					continue;
				best_sol << m<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					GLOBAL[m][j] = VEHICLE[i][j];
					GLOBALCumDist[m][j] = CumDist[i][j];
					best_sol << GLOBAL[m][j] << ' ';
				}
				best_sol <<endl;
				GLOBAL_SAIZ[m] = saiz[i];
				GLOBAL_capa[m] = total_demand[i];
				GLOBAL_Rcost[m] = route_cost[i];
				GLOBAL_distance_cost[m] = distance_cost[i];
				m++;
			}
			GLOBAL_BEST_COST = cost;
			GLOBAL_NO_ROUTE = m;
			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
			best_sol << "From VNS best improvement after Diversification type "<<div<<  endl;
			cout << "Found GLOBAL_BEST in VNS best improvement right after Diversification (only basic LNS no other improvement) type "<<div<<  endl;
			//getchar();
		}
		else
			noImproveDiv++;//noImproveDiv++ if not found GLOBAL_BEST
		best_sol << "Type of div = "<<div<<endl;

		//************************************************ NO DIVERSIFICATION *************************************************//	
		
		reinitializeRchangedStatus();//initialize to 0 for every route	
	lastdiversify:;
		
		delete[] flag_module; 
		delete[] cost_of_removing;
		delete[] oriGain.Oricost_of_removing;
		for (int i = 0; i < no_routes; i++)
		{
			delete[] gain_matrix_1_0[i];
			delete[] same_r_info[i];
			delete[] oriGain.Origain_matrix_1_0[i];
			delete[] oriGain.Orisame_r_info[i];
		}
		delete[] gain_matrix_1_0; 
		delete[] same_r_info ; 
		delete[] oriGain.Origain_matrix_1_0;
		delete[] oriGain.Orisame_r_info; 
		for (int i = 1; i <= s; ++i)
		{
			delete[] info_1_0[i];
			delete[] oriGain.Oriinfo_1_0[i];
		}
		delete[] info_1_0;
		delete[] oriGain.Oriinfo_1_0;
		delete[] RchangedStatus;
	}//end of while (NbDiv < maxDiv)//#############################################################################################################################

	adjustFreq(freq, maxLocalSearch); //if probability = 0, add 0.05 so that it will have a small chance to be selected
	recordFreq<<"=========== Frequency of gain in VNS ============"<<endl;
	for (int i = 1; i <= maxLocalSearch; i++) //freq[i] starts from 1
	{
		recordFreq<<"Freq["<<i<<"]= "<<freq[i]<<' '<<' ';
	}
	recordFreq<<endl;
	recordFreq.close();
	float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
	//EV_diver << div <<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;		
	timefile << "VNS best improvement "<<maxDiv <<" diversification time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << " second"<<endl;
	//recordTIME << "VNS best improvement ("<<std::to_string( I )<<") with "<<maxDiv <<" diversification time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << " second"<<endl;
	cout<<"Deleting gain"<<endl;
	delete[] gain;//could be error if zero or more than SIZE?????/
	delete[] modul;
	cout<<"After deleting gain"<<endl;
	delete[] divsequence;

}


//all shake() functions will shake the current best solution, in VNS, if no improvement then go to next shake of current best solution
int shake_1_0(int **(&VEHICLE), float *x, float *y)
{
	if (no_routes != LOCAL_NO_ROUTE)
	{
		cout<<"in shake1_0";
		getchar();
	}
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);

	//route_CGravity = new float*[no_routes];

	//for (int i = 0; i < no_routes; i++)
	//{
	//	route_CGravity[i] = new float[2]; //2 columns consists of x and y
	//}

	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	
	int rand_r1 = -1, rand_r2 = -1, rand_r1_p = -1, rand_r2_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1;
	//float average_capa_available = 0;
	//float average_demand = 0;
	int saiz_r1 = 0, saiz_r2 = 0;
	int demand_c1;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1;
	float cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0;
	//********************************************************  Neighbour_k 1 (1-0)  ******************************************************************************//

	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //initialize to 1 initially
	}	
	int no_flag=0;

	//============= Guided shake, calculate center of gravity ============// put this before m1_regenerate, previously calculate center of gravity everytime generate a customer, OMGGGGGG!!!!!!!! on 29Jun2015
	//calculate_centreGravity(x, y);
	//for (int i = 0; i < no_routes; i++)
	//{
	//	CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
	//}
	//CGravityfile <<endl;
	//============= End of calculate center of gravity ============//
	float internal_start_s = clock();// the code you wish to time goes here
	m1_regenerate:
		if (no_flag >= SIZE)
		{	
			cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE 1_0!" <<endl;
			for (int i = 0; i < SIZE; i++)
			{
				cout<<flag_module[i]<<' ';
			}
			//getchar();
			return 0;
		}
		int r2_found =0 ;
		float timeElapsed = (clock() - internal_start_s) / float(CLOCKS_PER_SEC) ;
		if(timeElapsed > 15)//if more than 15seconds
		{
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i] ==0)
					continue;
				for (int j = 1; j <= saiz[i]; j++)
				{
					if (flag_module[VEHICLE[i][j]] == false)//if it is flagged false, just skipped it
						continue;
					rand_r1 = i;
					saiz_r1 = saiz[i];
					rand_r1_p = j;
					goto getout;
				}
			}
		}
		else
		{
			srand ( time(NULL) ); //seed it
			rand_r1 = (rand() % no_routes); // for deletion cannot delete from empty route, NO_BEST_ROUTE = 3 , [0][1][2], can only delete from [0][1]
			saiz_r1 = saiz[rand_r1]; 

			if(saiz_r1 == 0)//have to test this before finding the position to avoid division by zero error
				goto m1_regenerate;

			rand_r1_p = (rand() % saiz_r1)+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE
		}
		
	getout:
		cust1_r1 = VEHICLE[rand_r1][rand_r1_p];
		
		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from false flag
			goto m1_regenerate;
	
		before_cust1 = VEHICLE[rand_r1][rand_r1_p-1];
		after_cust1 = VEHICLE[rand_r1][rand_r1_p+1];
		demand_c1 = demand[cust1_r1];

		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < no_routes; i++)
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
		//		before_cust2 = VEHICLE[i][j-1];
		//		after_cust2 = VEHICLE[i][j];//current position
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

		//bool *RFlag = new bool[no_routes]; //to flag the routes' center gravivity as traversing the sorted_custRGravity[] from top to bottom
		//for (int i = 0; i < no_routes; i++)
		//{
		//	RFlag[i] = true; //initially all routes are true
		//}
		
		findDist_from_CGravity (cust1_r1, x, y, custRGravity, sorted_custRGravity); //sort dist and route, compute 1/distance, compute d/sum(d), compute cumulative
		//for (int i = 0; i < no_routes; i++)//[0]sorted distance, [1]route index, [2]1/distance, [3]d/sum(d), [4]cumulative
		//{
		//	CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		//}
		//CGravityfile <<endl;

		//******************************** find route based on probability ************************************//
		//float randNum = (float) rand() / float(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		int route2 = -1;//initialize
		//CGravityfile << "cust1_r1= "<<cust1_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p<<endl;
		for (int i = 0; i < no_routes; i++) //only travel half of the nearest route to the customer, the rest just skip because the shaking is not good
		{
			route2 = sorted_custRGravity[i][1];
	
			if ((saiz[rand_r1] == 1) && (saiz[route2] == 0)) //if original 1 customer, cannot insert into empty route, it will be the same //THIS IS A REAL TRICK!!!!!!!!!!!!!!!!!!!
			{
				continue;
			}
			if ((route2 == rand_r1) || (demand_c1 > space_available[route2]))//if the same route or exceed capacity, 
			{
				continue;
			}
			//if (saiz[route2] == 0)//if it is an empty route, just insert
			//{
			//	rand_r2 = route2;
			//	rand_r2_p = 1;
			//	r2_found = 1;
			//	//CGravityfile << "Found r2 in shake 1-0, r2= "<<route2<<" , rand_r2_p= "<<rand_r2_p<<endl;
			//	goto found_r2_m1;
			//}
			for (int j = 1; j <= saiz[route2]+1; j++) //insertion can be made one position more than saiz
			{
				before_cust2 = VEHICLE[route2][j-1];
				after_cust2 = VEHICLE[route2][j];//current position

				gain1 = (dist[before_cust1][after_cust1]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][after_cust1] + service_time[cust1_r1]); //new-old 
				gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) - (dist[before_cust2][after_cust2]); //new-old

				if ((gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[route2])) //if fulfill both constraints
				{
					rand_r2 = route2;
					rand_r2_p = j;
					r2_found = 1;
					//CGravityfile << "Found r2 in shake 1-0, r2= "<<route2<<" , rand_r2_p= "<<rand_r2_p<<endl;
					goto found_r2_m1;
				}
			}
		}
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//
 
		if (r2_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			no_flag++;
			goto m1_regenerate;
		}


found_r2_m1:
		RchangedStatus[rand_r1] = true;//route (from)
		RchangedStatus[rand_r2] = true;//route (to)
		//customer still not inserted at this point (Route havent changed)

		//int delete1=1;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		//int delete2=0;//mean insert 1
		////r1
		//for (int i = -1; i <= delete1; i++)//flag the one before, current, and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r1][rand_r1_p+i]] = true;//flagged customer affected true
		//}
		////r2
		//for (int i = -1; i <= delete2; i++)//flag the one before and current
		//{
		//	CustaffectedStatus[VEHICLE[rand_r2][rand_r2_p+i]] = true;//flagged customer affected true
		//}
	
	//******************************************************** END OF Neighbour_k 1 (1-0)  ******************************************************************************//

	// erase the element from route 1

	int customer = cust1_r1; //must put before deletion, error this one can be -1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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

	gainvec[0] = 1;//status yes
	gainvec[1] = gain1;
	//gainvec[2] = origain1;
	insert_one_cust (rand_r2, rand_r2_p, VEHICLE, customer);
	gainvec[0] = 1;//status yes
	gainvec[1] = gain2;
	//gainvec[2] = origain1;
	delete_one_cust (rand_r1, rand_r1_p, VEHICLE);


	float total_cost = 0.0;

	cout<<"============Routes after shake in 1-0============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		//float r_cost = 0.0;//to check
		//for (int c = 0; c <= saiz[i]; c++)
		//{
		//	r_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		//}
		//if (abs (r_cost-route_cost[i]) > 0.1)
		//{
		//	cout<<endl<<"After shake shake_1_0!!!!!!!!!"<<' ';
		//	cout<<"route_cost["<<i<<"]= "<<route_cost[i]<<' '<<"r_cost= "<<r_cost<<endl;
		//	cout<<rand_r1<<' '<<rand_r2<<' '<<rand_r1_p<<' '<<rand_r2_p<<' '<<gain1<<' '<<gain2<<endl;
		//	getchar();
		//}
		total_cost += route_cost[i]; 
		cout << endl;
	}
	cout<<"total cost = " <<total_cost<<endl;
	if (no_routes != LOCAL_NO_ROUTE)
		getchar();
	
	delete[] flag_module;
	//delete[] RFlag;
	return 1; //shake status is ok

}


int shake_1_1(int **(&VEHICLE), float *x, float *y)//routes will copy from LOCAL_BEST_ROUTE in this function, only shake the best route
{
	if (no_routes != LOCAL_NO_ROUTE)
	{
		cout<<"in shake1_1";
		getchar();
	}
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	
	//route_CGravity = new float*[no_routes];

	//for (int i = 0; i < no_routes; i++)
	//{
	//	route_CGravity[i] = new float[2]; //2 columns consists of x and y
	//}

	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	int rand_r1 = -1, rand_r2 = -1,  rand_r1_p = -1, rand_r2_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust1_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0;
	int demand_c1, demand_c2;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1;
	float cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0;

	//********************************************************  Neighbour_k 2 (1-1)  ******************************************************************************//

	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}
	int no_flag=0;
	
	//============= Guided shake, calculate center of gravity ============//put this before m2_regenerate, previously calculate center of gravity everytime generate a customer, OMGGGGGG!!!!!!!! on 29Jun2015
	//calculate_centreGravity(x, y);
	//for (int i = 0; i < no_routes; i++)
	//{
	//	CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
	//}
	//CGravityfile <<endl;
	//============= End of calculate center of gravity ============//
	float internal_start_s = clock();// the code you wish to time goes here
	m2_regenerate:
		if (no_flag >= SIZE)
		{	
			cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE 1_1!" <<endl;
			for (int i = 0; i < SIZE; i++)
			{
				cout<<flag_module[i]<<' ';
			}
			//getchar();
			return 0;
		}
		int r2_found =0 ;
		float timeElapsed = (clock() - internal_start_s) / float(CLOCKS_PER_SEC) ;
		if(timeElapsed > 15)//if more than 15seconds
		{
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i] ==0)
					continue;
				for (int j = 1; j <= saiz[i]; j++)
				{
					if (flag_module[VEHICLE[i][j]] == false)//if it is flagged false, just skipped it
						continue;
					rand_r1 = i;
					saiz_r1 = saiz[i];
					rand_r1_p = j;
					goto getout;
				}
			}
		}
		else
		{
			srand ( time(NULL) ); //seed it
			rand_r1 = (rand() % no_routes); // for deletion cannot delete from empty route (last route), NO_BEST_ROUTEs = 3 , [0][1][2], can only delete from [0][1]
			saiz_r1 = saiz[rand_r1];
		
			if(saiz_r1 == 0)//have to test this before finding the position to avoid division by zero error
				goto m2_regenerate;
		
			rand_r1_p = (rand() % saiz_r1)+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE
		}
	
	getout:
		cust1_r1 = VEHICLE[rand_r1][rand_r1_p];

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or from flag false
			goto m2_regenerate;
		
		demand_c1 = demand[cust1_r1];
		before_cust1 = VEHICLE[rand_r1][rand_r1_p-1];
		after_cust1 = VEHICLE[rand_r1][rand_r1_p+1];

		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < no_routes; i++)
		//{
		//	if ((i == rand_r1) || (saiz[i] == 0) ) //if the same route, skip or if the saiz = 0 because this is 1-1 cannot from empty route
		//		continue;
		//	
		//	for (int j = 1; j <= saiz[i]; j++) //swap positions are  equal to the saiz
		//	{
		//		cust1_r2 = VEHICLE[i][j];
		//		before_cust2 = VEHICLE[i][j-1];
		//		after_cust2 = VEHICLE[i][j+1];
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
		//bool *RFlag = new bool[no_routes];
		//for (int i = 0; i < no_routes; i++)
		//{
		//	RFlag[i] = true; //initially all routes are true
		//}

		findDist_from_CGravity (cust1_r1, x, y, custRGravity, sorted_custRGravity);
		//for (int i = 0; i < no_routes; i++)
		//{
		//	CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		//}
		//CGravityfile <<endl;
	
		
		//******************************** find route based on probability ************************************//
		//float randNum = (float) rand() / float(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		//CGravityfile << "cust1_r1= "<<cust1_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p<<endl;
		int route2 = -1;//initialize
		for (int i = 0; i < no_routes; i++) //only travel half of the nearest route to the customer, the rest just skip because the shaking is not good
		{
			route2 = sorted_custRGravity[i][1];
			if ((route2 == rand_r1) || (saiz[route2] == 0)) //if the same route, skip or if the saiz = 0 because this is 1-1 cannot from empty route
			{
				continue;
			}

			for (int j = 1; j <= saiz[route2]; j++) //swap positions are  equal to the saiz
			{
				cust1_r2 = VEHICLE[route2][j];
				before_cust2 = VEHICLE[route2][j-1];
				after_cust2 = VEHICLE[route2][j+1];
				demand_c2 = demand[cust1_r2];

				gain1 = (dist[before_cust1][cust1_r2] + dist[cust1_r2][after_cust1] + service_time[cust1_r2]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][after_cust1] + service_time[cust1_r1]); //new-old
				gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) - (dist[before_cust2][cust1_r2] + dist[cust1_r2][after_cust2] + service_time[cust1_r2]); //new-old

				if ((demand_c1 <= (space_available[route2]+demand_c2)) && (demand_c2 <= (space_available[rand_r1]+demand_c1)) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[route2])) //if fulfill both constraints
				{
					rand_r2 = route2;
					rand_r2_p = j;
					r2_found = 1;
					//CGravityfile << "Found r2 in shake 1-1, r2= "<<rand_r2<<", rand_r2_p= "<<rand_r2_p<<", cust1_r2= "<<cust1_r2<<endl;
					goto found_r2_m2;
				}
			}
		}
			
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//

		if (r2_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			no_flag++;

			goto m2_regenerate;
		}
		
	found_r2_m2:
		if ( (cust1_r1 == -1) || (cust1_r2 == -1) )
		{
			cout<< "cust1_r1 or cust1_r2 cannot be negative!!" <<endl;
			getchar();
		}
		
		//reinitializeRchangedStatus ();
		RchangedStatus[rand_r1] = true;//route (from)
		RchangedStatus[rand_r2] = true;//route (to)
		//Route[rand_r1].RchangedStatus = 1;//route (from)
		//Route[rand_r2].RchangedStatus = 1;//route (to)
		//customer still not inserted at this point (Route havent changed)

		//int delete1=1;//if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		////r1
		//for (int i = -1; i <= delete1; i++)//flag the one before, current, and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r1][rand_r1_p+i]] = true;//flagged customer affected true
		//}
		////r2
		//for (int i = -1; i <= delete1; i++)//flag the one before, current, and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r2][rand_r2_p+i]] = true;//flagged customer affected true
		//}


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
	
	cout<<"===========CUST from shake 1-1=========" <<endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p << "]=" <<customer1<< endl;
	cout<< "route[" << rand_r2 <<"][" << rand_r2_p << "]=" <<customer2<< endl;

	swap_in_oneCust (rand_r1, rand_r1_p, VEHICLE, customer2);
	swap_in_oneCust (rand_r2, rand_r2_p, VEHICLE, customer1);


	float total_cost = 0.0;

	cout<<"============ Routes after shake in 1-1 ============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		//float r_cost = 0.0;//to check
		//for (int c = 0; c <= saiz[i]; c++)
		//{
		//	r_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		//}
		//if (abs (r_cost-route_cost[i]) > 0.1)
		//{
		//	cout<<endl<<"After shake_1_1!!!!!!!!!"<<' ';
		//	cout<<"route_cost["<<i<<"]= "<<route_cost[i]<<' '<<"r_cost= "<<r_cost<<endl;
		//	cout<<rand_r1<<' '<<rand_r2<<' '<<rand_r1_p<<' '<<rand_r2_p<<' '<<gain1<<' '<<gain2<<endl;
		//	getchar();
		//}
		total_cost += route_cost[i]; //update total_cost (global variable)  
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	if (no_routes != LOCAL_NO_ROUTE)
		getchar();
	
	delete[] flag_module;
	return 1; //shake status is ok
}


int shake_2_0_twoR(int **(&VEHICLE), float *x, float *y)//routes will copy from BEST_ROUTE in this function, only shake the best route
{
	if (no_routes != LOCAL_NO_ROUTE)
	{
		cout<<"in shake2_0_twoR";
		getchar();
	}
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	
	//route_CGravity = new float*[no_routes];

	//for (int i = 0; i < no_routes; i++)
	//{
	//	route_CGravity[i] = new float[2]; //2 columns consists of x and y
	//}
	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	//**************all insertion and deletion starts from 1 to saiz because the VEHICLE starts from depot, end with depot**************//
	//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//

	int rand_r1 = -1, rand_r2 = -1, rand_r3 = -1, rand_r1_p = -1, rand_r2_p = -1, rand_r3_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1, cust1_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0, saiz_r3 = 0;
	int demand_c1, demand_c2;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1;
	float cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0;

	//******************************************************** Neighbour_k 3 (2-0)  ******************************************************************************//
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}	
	int no_flag=0;
	for (int i = 0; i < no_routes; i++)
	{
		//if (saiz[i] == 1) //made comment on 29June2015, it will be flagged in if(saiz[i] !=0)
		//{
		//	flag_module[vehic_depotCust[i][1]] = false;
		//	no_flag++;
		//}
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			flag_module[VEHICLE[i][saiz[i]]] = false; //flag the last cust in a route false because last customer cannot be selected, this is 2-1 sequential//this is a trick!!!!!!!!!!!
			no_flag++;
		}
		
	}
	//============= Guided shake, calculate center of gravity ============////put this before m3_regenerate, previously calculate center of gravity everytime generate a customer, OMGGGGGG!!!!!!!! on 29Jun2015
	//calculate_centreGravity(x, y);
	//for (int i = 0; i < no_routes; i++)
	//{
	//	CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
	//}
	//CGravityfile <<endl;
	//============= End of calculate center of gravity ============//
	float internal_start_s = clock();// the code you wish to time goes here
	m3_regenerate:
		if (no_flag >= SIZE)
		{	
			cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE2-0 TWO ROUTES!" <<endl;
			for (int i = 0; i < SIZE; i++)
			{
				cout<<flag_module[i]<<' ';
			}
			//getchar();
			return 0;
		}
		int r2_found=0;
		float timeElapsed = (clock() - internal_start_s) / float(CLOCKS_PER_SEC) ;
		if(timeElapsed > 15)//if more than 15seconds
		{
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i] ==0 || saiz[i]==1)
					continue;
				for (int j = 1; j <= saiz[i]-1; j++)
				{
					if (flag_module[VEHICLE[i][j]] == false)//if it is flagged false, just skipped it
						continue;
					rand_r1 = i;
					saiz_r1 = saiz[i];
					rand_r1_p = j;
					goto getout;
				}
			}
		}
		else
		{
			srand ( time(NULL) ); //seed it
			rand_r1 = (rand() % no_routes); 
			saiz_r1 = saiz[rand_r1];
			if((saiz_r1 == 0)||(saiz_r1 == 1))//have to test this before finding the position to avoid division by zero error
				goto m3_regenerate;
			rand_r1_p = (rand() % (saiz_r1-1))+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE; //minus one because two customers will be deleted in sequential
		}
	
	getout:
		cust1_r1 = VEHICLE[rand_r1][rand_r1_p];
		

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or route with 1 cust or from flag false //make sure deletion cannot be made from empty route
			goto m3_regenerate;
			
		cust2_r1 = VEHICLE[rand_r1][rand_r1_p+1];//customer in sequential		
		demand_c1 = demand[cust1_r1];
		demand_c2 = demand[cust2_r1];
		before_cust1 = VEHICLE[rand_r1][rand_r1_p-1];
		after_cust1 = VEHICLE[rand_r1][rand_r1_p+2];

		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < no_routes; i++)
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
		//			//cust1_r2 = VEHICLE[i][j];
		//			before_cust2 = VEHICLE[i][j-1];
		//			after_cust2 = VEHICLE[i][j]; //current position
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
		//bool *RFlag = new bool[no_routes];
		//for (int i = 0; i < no_routes; i++)
		//{
		//	RFlag[i] = true; //initially all routes are true
		//}

		//find route based on probability
		findDist_from_CGravity (cust1_r1, x, y, custRGravity, sorted_custRGravity);
		//for (int i = 0; i < no_routes; i++)
		//{
		//	CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		//}
		//CGravityfile <<endl;
		

		//******************************** find route based on probability ************************************//
		//float randNum = (float) rand() / float(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		//CGravityfile << "cust1_r1= "<<cust1_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p<<endl;
		//CGravityfile << "cust2_r1= "<<cust2_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p+1<<endl;
		int route2 = -1;//initialize
		for (int i = 0; i < no_routes; i++) //only travel half of the nearest route to the customer, the rest just skip because the shaking is not good
		{
			route2 = sorted_custRGravity[i][1];
			if ((saiz[rand_r1] == 2) && (saiz[route2]==0)) //if size of original route is 2 and new route is empty route, skip because it will end up the same
			{
				continue;
			}
		
			if ((route2 == rand_r1) || (demand_c1+demand_c2 > space_available[route2]))//if the same route or exceed capacity, skip
			{
				continue;
			}
			//if (saiz[route2] == 0)
			//{
			//	rand_r2 = route2;
			//	rand_r2_p = 1;
			//	r2_found = 1;
			//	//CGravityfile << "Found r2 in shake 2-0 TwoR, r2= "<<rand_r2<<", rand_r2_p= "<<rand_r2_p<<endl;
			//	goto found_r2_m3;
			//}
			for (int j = 1; j <= saiz[route2]+1; j++) //insertion can be made one position more than saiz
			{
				//cust1_r2 = vehic_depotCust[route2][j];
				before_cust2 = VEHICLE[route2][j-1];
				after_cust2 = VEHICLE[route2][j]; //current position

				gain1 = (dist[before_cust1][after_cust1]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1] + service_time[cust2_r1]); //new-old
				gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust2] + service_time[cust1_r1] + service_time[cust2_r1]) - (dist[before_cust2][after_cust2]); //new-old

				if ((gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[route2])) //if fulfill both constraints
				{
					rand_r2 = route2;
					rand_r2_p = j;
					r2_found = 1;
					//CGravityfile << "Found r2 in shake 2-0 TwoR, r2= "<<rand_r2<<", rand_r2_p= "<<rand_r2_p<<endl;
					goto found_r2_m3;
				}		
			}
		}

		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//

		if (r2_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			no_flag++;
			
			goto m3_regenerate;
		}
		
	found_r2_m3:
		//route_change[0] = rand_r1; //route (from)
		//route_change[1] = rand_r2; //route (to)
		//route_change[2] = -1; //reinitialize 
		//reinitializeRchangedStatus ();
		RchangedStatus[rand_r1] = true;//route (from)
		RchangedStatus[rand_r2] = true;//route (to)
		//Route[rand_r1].RchangedStatus = 1;//route (from)
		//Route[rand_r2].RchangedStatus = 1;//route (to)
		
		//customer still not inserted at this point (Route havent changed)
		//int delete1=2;//if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		//int delete2=0;
		////r1
		//for (int i = -1; i <= delete1; i++)//flag the one before, current, and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r1][rand_r1_p+i]] = true;//flagged customer affected true
		//}
		////r2
		//for (int i = -1; i <= delete2; i++)//flag the one before and current
		//{
		//	CustaffectedStatus[VEHICLE[rand_r2][rand_r2_p+i]] = true;//flagged customer affected true
		//}

	//******************************************************** END OF Neighbour_k 3 (2-0)  ******************************************************************************//

	// erase the element from route 1

	int customer1 = cust1_r1; 
	int customer2 = cust2_r1;
	

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

		gainvec[0] = 1;//status yes
	gainvec[1] = gain1;
	//gainvec[2] = origain1;
	insert_two_cust (rand_r2, rand_r2_p, VEHICLE, customer1, customer2);
		gainvec[0] = 1;//status yes
	gainvec[1] = gain2;
	//gainvec[2] = origain1;
	delete_two_cust (rand_r1, rand_r1_p, VEHICLE);

	float total_cost = 0.0;

	cout<<"============Routes after shake in 2-0 two routes============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		//float r_cost = 0.0;//to check
		//for (int c = 0; c <= saiz[i]; c++)
		//{
		//	r_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		//}
		//if (abs (r_cost-route_cost[i]) > 0.1)
		//{
		//	cout<<endl<<"After shake_2_0_twoR!!!!!!!!!"<<' ';
		//	cout<<"route_cost["<<i<<"]= "<<route_cost[i]<<' '<<"r_cost= "<<r_cost<<endl;
		//	cout<<rand_r1<<' '<<rand_r2<<' '<<rand_r1_p<<' '<<rand_r2_p<<' '<<gain1<<' '<<gain2<<endl;
		//	getchar();
		//}
		total_cost += route_cost[i]; //update total_cost (global variable)  
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	
	if (no_routes != LOCAL_NO_ROUTE)
		getchar();
	
	delete[] flag_module;
	return 1; //shake status is ok
}

//DONE
int shake_2_0_threeR(int **(&VEHICLE), float *x, float *y)//routes will copy from BEST_ROUTE in this function, only shake the best route
{
	if (no_routes != LOCAL_NO_ROUTE)
	{
		cout<<"in shake2_0three";
		getchar();
	}
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	
	//route_CGravity = new float*[no_routes];

	//for (int i = 0; i < no_routes; i++)
	//{
	//	route_CGravity[i] = new float[2]; //2 columns consists of x and y
	//}

	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	
	//**************all insertion and deletion starts from 1 to saiz because the vehic_depotCust starts from depot, end with depot**************//
	//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//

	int rand_r1 = -1, rand_r2 = -1, rand_r3 = -1, rand_r1_p = -1, rand_r2_p = -1, rand_r3_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1, cust1_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0, saiz_r3 = 0;
	int demand_c1, demand_c2;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1, before_cust3=-1, after_cust3=-1;
	float cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0, gain3=0.0;
	float origain1, origain2, origain3;
	
	//******************************************************** Neighbour_k 4 (2-0)  ******************************************************************************//
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}	
	int no_flag=0;
	for (int i = 0; i < no_routes; i++)
	{
		//if (saiz[i] == 1)  //made comment on 29Jun2015, it will be flagged in if (saiz[i] != 0)
		//{
		//	flag_module[vehic_depotCust[i][1]] = false;
		//	no_flag++;
		//}
		
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			flag_module[VEHICLE[i][saiz[i]]] = false; //flag the last cust in a route false because last customer cannot be selected, this is 2-0 sequential//this is a trick!!!!!!!!!!!
			no_flag++;
		}
	
	}
	//============= Guided shake, calculate center of gravity ============////put this before m4_regenerate, previously calculate center of gravity everytime generate a customer, OMGGGGGG!!!!!!!! on 29Jun2015
	//calculate_centreGravity(x, y);
	//for (int i = 0; i < no_routes; i++)
	//{
	//	CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
	//}
	//CGravityfile <<endl;
	//============= End of calculate center of gravity ============//
	
	float internal_start_s = clock();// the code you wish to time goes here
	m4_regenerate:
		if (no_flag >= SIZE)
		{	
			cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE2-0 THREE ROUTES!" <<endl;
			for (int i = 0; i < SIZE; i++)
			{
				cout<<flag_module[i]<<' ';
			}
			//getchar();
			return 0;
		}
		int r2_found=0;
		float timeElapsed = (clock() - internal_start_s) / float(CLOCKS_PER_SEC) ;
		if(timeElapsed > 15)//if more than 15seconds
		{
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i] ==0 || saiz[i]==1)
					continue;
				for (int j = 1; j <= saiz[i]-1; j++)
				{
					if (flag_module[VEHICLE[i][j]] == false)//if it is flagged false, just skipped it
						continue;
					rand_r1 = i;
					saiz_r1 = saiz[i];
					rand_r1_p = j;
					goto getout;
				}
			}
		}
		else
		{
			srand ( time(NULL) ); //seed it
			rand_r1 = (rand() % no_routes); 
			saiz_r1 = saiz[rand_r1];

			if((saiz_r1 == 0) || (saiz_r1 == 1))//have to test this before finding the position to avoid division by zero error
				goto m4_regenerate;
		
			rand_r1_p = (rand() % (saiz_r1-1))+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE; //minus one because two customers will be deleted in sequential
		}
	getout:
		cust1_r1 = VEHICLE[rand_r1][rand_r1_p];
		
		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or from flag false //make sure deletion cannot be made from empty route
			goto m4_regenerate;
		
		cust2_r1 = VEHICLE[rand_r1][rand_r1_p+1];//customer in sequential		
		demand_c1 = demand[cust1_r1];
		demand_c2 = demand[cust2_r1];		
		before_cust1 = VEHICLE[rand_r1][rand_r1_p-1];
		after_cust1 = VEHICLE[rand_r1][rand_r1_p+2];
		
		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < no_routes; i++)
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
		//		before_cust2 = VEHICLE[i][j-1];
		//		after_cust2 = VEHICLE[i][j]; //current position
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

		//bool *RFlag = new bool[no_routes];
		//for (int i = 0; i < no_routes; i++)
		//{
		//	RFlag[i] = true; //initially all routes are true
		//}
		
		//sorted_custRGravity[i][0] = sorted distance from route center of gravity 
		//sorted_custRGravity[i][1] = route index
		//sorted_custRGravity[i][2] = 1/distance
		//sorted_custRGravity[i][3] = [2]/sum_distance
		//sorted_custRGravity[i][4] = cumulative of [3]
		//find route based on probability
		findDist_from_CGravity (cust1_r1, x, y, custRGravity, sorted_custRGravity);
		
		//for (int i = 0; i < no_routes; i++)
		//{
		//	CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		//}
		//CGravityfile <<endl;
		

		//******************************** find route based on probability ************************************//
		//float randNum = (float) rand() / float(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		//CGravityfile << "cust1_r1= "<<cust1_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p<<endl;
		//CGravityfile << "cust2_r1= "<<cust2_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p+1<<endl;
		int route2 = -1;//initialize
		for (int i = 0; i < no_routes; i++) //only travel half of the nearest route to the customer, the rest just skip because the shaking is not good
		{
			route2 = sorted_custRGravity[i][1];
			if ((route2 == rand_r1) || (demand_c1 > space_available[route2])) //if the same route, skip or if the saiz = 0 because this is 2-0 cannot from empty route
			{
				continue;
			}
			//if (saiz[route2] == 0)
			//{
			//	rand_r2 = route2;
			//	rand_r2_p = 1;
			//	r2_found = 1;
			//	//CGravityfile << "Found r2 in shake 2-0 TwoR, r2= "<<rand_r2<<", rand_r2_p= "<<rand_r2_p<<endl;
			//	goto found_r2_m4;
			//}
			for (int j = 1; j <= saiz[route2]+1; j++) //insertion can be made one position more than saiz
			{
				before_cust2 = VEHICLE[route2][j-1];
				after_cust2 = VEHICLE[route2][j]; //current position

				//r1 (delete 2)
				origain1 = (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1] + service_time[cust2_r1]) - (dist[before_cust1][after_cust1]) ; //old-new
				int remainingCust1 = saiz[rand_r1]- (rand_r1_p+1); //saiz – 2ndpos delete 
				float oldCumDist1 = CumDist[rand_r1][rand_r1_p] + CumDist[rand_r1][rand_r1_p+1]; //2 customers originally from r1
				if (remainingCust1<0)
					remainingCust1=0;
				gain1 = (oldCumDist1) + remainingCust1*origain1;


				//r2 (insert 1)
				origain2 = (dist[before_cust2][after_cust2]) - (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) ; //old-new
				int remainingCust2 = saiz[route2]- (j) +1;//saiz – posinsert + 1 
				float newCumDist2 = CumDist[route2][j-1] + dist[before_cust2][cust1_r1] + service_time[cust1_r1];
				
				if (remainingCust2<0)
					remainingCust2=0;
				gain2 = - newCumDist2 + remainingCust2*origain2;

				if ((distance_cost[rand_r1]-origain1 <= DISTANCE+epsilon) && (distance_cost[route2]-origain2 <= DISTANCE+epsilon)) //if fulfill both constraints
				{
					rand_r2 = route2;
					rand_r2_p = j;
					r2_found = 1;
					//CGravityfile << "Found R2 in Shake2_0ThreeR!!!, R2= "<<route2<<" , rand_r2_p= "<<rand_r2_p<<endl;
					goto found_r2_m4;
				}
			}
		}
		
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//
		
		if (r2_found != 1)
		{
			if (flag_module[cust1_r1] != false)//if originally not flagged false, to keep track of no_flag. BUT if it is flagged true, will it enter here????
			{
				flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
				no_flag++;
			}
			goto m4_regenerate;
		}
		
	found_r2_m4:
		//================== find r3 GUIDED SHAKE =========================//
		//bool *R3Flag = new bool[no_routes];
		//for (int i = 0; i < no_routes; i++)
		//{
		//	R3Flag[i] = true; //initially all routes are true
		//}

		findDist_from_CGravity (cust2_r1, x, y, custRGravity, sorted_custRGravity);//find for second customer where to insert too//added on 29June2015
		//for (int i = 0; i < no_routes; i++)
		//{
		//	CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		//}
		//CGravityfile <<endl;

		int r3_found = 0;
		int route3 = -1;//initialize

		for (int i = 0; i < no_routes; i++) //only travel half of the nearest route to the customer, the rest just skip because the shaking is not good
		{
			route3 = sorted_custRGravity[i][1];
			if ((route3 == rand_r1) || (route3 == route2) || (demand_c2 > space_available[route3])) //if the same route, skip 
			{
				continue;
			}
			//if (saiz[route3] == 0)
			//{
			//	rand_r3 = route3;
			//	rand_r3_p = 1;
			//	r3_found = 1;
			//	//CGravityfile << "Found R3 in SHake2_0ThreeR!!!, R3= "<<route3<<" , rand_r3_p= "<<rand_r3_p<<endl;
			//	goto found_r3_m4;
			//}
			for (int j = 1; j <= saiz[route3]+1; j++) //insertion can be made one position more than saiz
			{
				before_cust3 = VEHICLE[route3][j-1];
				after_cust3 = VEHICLE[route3][j]; //current position

				//r3 (insert 1)
				origain3 = (dist[before_cust3][after_cust3]) - (dist[before_cust3][cust2_r1] + dist[cust2_r1][after_cust3] + service_time[cust2_r1]) ; //old-new //CHANGED THIS ON 4 MARC 2015!!!!!!!!!!!!!!!CARELESS MISTAKE!!!!!!!!!!!!!!
				int remainingCust3 = saiz[route3]- (j) +1;//saiz – posinsert + 1 
				float newCumDist3 = CumDist[route3][j-1] + dist[before_cust3][cust2_r1] + service_time[cust2_r1];
				
				if (remainingCust3<0)
					remainingCust3=0;
				gain3 = - newCumDist3 + remainingCust3*origain3;

				if ((demand_c2 <= space_available[route3]) && distance_cost[route3]-origain3 <= DISTANCE+epsilon)//if fulfill r3 constraint
				{
					rand_r3 = route3;
					rand_r3_p = j;
					r3_found = 1;
					//CGravityfile << "Found R3 in SHake2_0ThreeR!!!, R3= "<<route3<<" , rand_r3_p= "<<rand_r3_p<<endl;
					goto found_r3_m4;
				}
			}
		}

		//================= Uncomment here for guided shake route 3 ===============================//
		//for (int i = 0; i < no_routes; i++)
		//{
		//	if ((i == rand_r1) || (i == rand_r2) ) //if the same route, skip, saiz = 0 is ok because insertion can go to empty route
		//		continue;
		//	
		//	if (saiz[i] == 0)
		//	{
		//		rand_r3 = i;
		//		rand_r3_p = 1;
		//		r3_found = 1;
		//		goto found_r3_m4;
		//	}
		//	for (int j = 1; j <= saiz[i]+1; j++) //insertion can be made one position more than saiz
		//	{
		//		before_cust3 = vehic_depotCust[i][j-1];
		//		after_cust3 = vehic_depotCust[i][j]; //current position
		//		gain3 = (dist[before_cust3][cust2_r1] + dist[cust2_r1][after_cust3] + service_time[cust2_r1]) - (dist[before_cust3][after_cust3]); //new-old //CHANGED THIS ON 4 MARC 2015!!!!!!!!!!!!!!!CARELESS MISTAKE!!!!!!!!!!!!!!
		//		if ((demand_c2 <= space_available[i]) && (gain3 <= distance_available[i])) //if fulfill both constraints
		//		{
		//			rand_r3 = i;
		//			rand_r3_p = j;
		//			r3_found = 1;
		//			goto found_r3_m4;
		//		}

		//	}
		//}
		//================= End of uncomment here for guided shake route 3 ===============================//
		if (r3_found != 1)
		{
			if (flag_module[cust1_r1] != false)
			{
				flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd //this will redo everything again!!!!!!!!!!!!!!!!!!!!
				no_flag++;
			}
			goto m4_regenerate;
		}
			
	found_r3_m4:
			
		if ( (cust1_r1 == -1) || (cust2_r1 == -1))
			cout<< "cust1_r1 or cust1_r2 cannot be negative!!" <<endl;			

		//route_change[0] = rand_r1; //route (from)
		//route_change[1] = rand_r2; //route (to)
		//route_change[2] = rand_r3; //route (to)
		//reinitializeRchangedStatus ();
		RchangedStatus[rand_r1] = true;//route (from)
		RchangedStatus[rand_r2] = true;//route (to)
		RchangedStatus[rand_r3] = true;//route (to)
		//Route[rand_r1].RchangedStatus = 1;//route (from)
		//Route[rand_r2].RchangedStatus = 1;//route (to)
		//Route[rand_r3].RchangedStatus = 1;//route (to)
		//customer still not inserted at this point (Route havent changed)
		//int delete1=2;//if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		//int delete2=0;	
		////r1
		//for (int i = -1; i <= delete1; i++)//flag the one before, current, after current and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r1][rand_r1_p+i]] = true;//flagged customer affected true
		//}
		////r2
		//for (int i = -1; i <= delete2; i++)//flag the one before and current
		//{
		//	CustaffectedStatus[VEHICLE[rand_r2][rand_r2_p+i]] = true;//flagged customer affected true
		//}
		////r3
		//for (int i = -1; i <= delete2; i++)//flag the one before and current
		//{
		//	CustaffectedStatus[VEHICLE[rand_r3][rand_r3_p+i]] = true;//flagged customer affected true
		//}			

	//******************************************************** END OF Neighbour_k 4 (2-0)  ******************************************************************************//
	int customer1 = cust1_r1; 
	int customer2 = cust2_r1; //second customer in route1

	
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

	gainvec[0] = 1;//status yes
	gainvec[1] = gain1;
	gainvec[2] = origain1;
	delete_two_cust (rand_r1, rand_r1_p, VEHICLE); //2 out

	gainvec[0] = 1;//status yes
	gainvec[1] = gain2;
	gainvec[2] = origain2;
	insert_one_cust (rand_r2, rand_r2_p, VEHICLE, customer1); //1 in for route2

	gainvec[0] = 1;//status yes
	gainvec[1] = gain3;
	gainvec[2] = origain3;
	insert_one_cust (rand_r3, rand_r3_p, VEHICLE, customer2); //1 in for route3

	float total_cost = 0.0;

	cout<<"============Routes after shake in 2-0 three routes============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		//float r_cost = 0.0;//to check
		//for (int c = 0; c <= saiz[i]; c++)
		//{
		//	r_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		//}
		//if (abs (r_cost-route_cost[i]) > 0.1)
		//{
		//	cout<<endl<<"After shake_2_0_threeR!!!!!!!!!"<<' ';
		//	cout<<"route_cost["<<i<<"]= "<<route_cost[i]<<' '<<"r_cost= "<<r_cost<<endl;
		//	cout<<rand_r1<<' '<<rand_r2<<' '<<rand_r1_p<<' '<<rand_r2_p<<' '<<gain1<<' '<<gain2<<endl;
		//	getchar();
		//}
		total_cost += route_cost[i]; 
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	if (no_routes != LOCAL_NO_ROUTE)
		getchar();
	
	delete[] flag_module;
	//delete[] RFlag;
	//delete[] R3Flag;
	return 1; //shake status is ok
}

//DONE
int shake_2_1_twoR(int **(&VEHICLE), float *x, float *y)//routes will copy from BEST_ROUTE in this function, only shake the best route
{
	if (no_routes != LOCAL_NO_ROUTE)
	{
		cout<<"in shake2_1two";
		getchar();
	}
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	
	//route_CGravity = new float*[no_routes];

	//for (int i = 0; i < no_routes; i++)
	//{
	//	route_CGravity[i] = new float[2]; //2 columns consists of x and y
	//}
	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	//**************all insertion and deletion starts from 1 to saiz because the vehic_depotCust starts from depot, end with depot**************//
	//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//

	int rand_r1 = -1, rand_r2 = -1, rand_r3 = -1, rand_r1_p = -1, rand_r2_p = -1, rand_r3_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1, cust1_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0, saiz_r3 = 0;
	int demand_c1, demand_c2, demand_c3;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1;
	float cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0;
	float origain1, origain2;

	//******************************************************** Neighbour_k 5 (2-1)  ******************************************************************************//
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}	
	int no_flag=0;

	for (int i = 0; i < no_routes; i++)
	{
		//if (saiz[i] == 1)//made comment on 29Jun2015, it will be flagged in if (saiz[i] != 0)
		//{
		//	flag_module[vehic_depotCust[i][1]] = false;
		//	no_flag++;
		//}
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			flag_module[VEHICLE[i][saiz[i]]] = false; //flag the last cust in a route false because last customer cannot be selected, this is 2-1 sequential//this is a trick!!!!!!!!!!!
			no_flag++;
		}
	}

	//============= Guided shake, calculate center of gravity ============////put this before m5_regenerate, previously calculate center of gravity everytime generate a customer, OMGGGGGG!!!!!!!! on 29Jun2015
	//calculate_centreGravity(x, y);
	//for (int i = 0; i < no_routes; i++)
	//{
	//	CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
	//}
	//CGravityfile <<endl;
	//============= End of calculate center of gravity ============//
	float internal_start_s = clock();// the code you wish to time goes here
m5_regenerate:
		if (no_flag >= SIZE)
		{	
			cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE2_1 TWO ROUTES!" <<endl;
			for (int i = 0; i < SIZE; i++)
			{
				cout<<flag_module[i]<<' ';
			}
			return 0;
		}
		int r2_found=0;
		float timeElapsed = (clock() - internal_start_s) / float(CLOCKS_PER_SEC) ;
		if(timeElapsed > 15)//if more than 15seconds
		{
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i] ==0 || saiz[i]==1)
					continue;
				for (int j = 1; j <= saiz[i]-1; j++)
				{
					if (flag_module[VEHICLE[i][j]] == false)//if it is flagged false, just skipped it
						continue;
					rand_r1 = i;
					saiz_r1 = saiz[i];
					rand_r1_p = j;
					goto getout;
				}
			}
		}
		else
		{
			srand ( time(NULL) ); //seed it
			rand_r1 = (rand() % no_routes); 
			saiz_r1 = saiz[rand_r1];
			if ((saiz_r1 == 0) || (saiz_r1 == 1))//have to test this before finding the position to avoid division by zero error
				goto m5_regenerate;
			rand_r1_p = (rand() % (saiz_r1-1))+1; //position is from 1 to saiz-1, eg vehic_depotCust: SIZE 1 2 3 SIZE; //minus one because two customers will be deleted in sequential
		}

	getout:
		cust1_r1 = VEHICLE[rand_r1][rand_r1_p];

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or route with 1 cust or from flag false //make sure deletion cannot be made from empty route
			goto m5_regenerate;
		
		cust2_r1 = VEHICLE[rand_r1][rand_r1_p+1];//customer in sequential		
		demand_c1 = demand[cust1_r1];
		demand_c2 = demand[cust2_r1];
		before_cust1 = VEHICLE[rand_r1][rand_r1_p-1];
		after_cust1 = VEHICLE[rand_r1][rand_r1_p+2];
		
		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < no_routes; i++)
		//{
		//	if ((i == rand_r1) || (saiz[i] == 0) ) //if the same route, skip or if the saiz = 0 because this is 2-1 cannot from empty route
		//		continue;
		//	
		//	for (int j = 1; j <= saiz[i]; j++) //swap positions are  equal to the saiz
		//	{
		//		cust1_r2 = VEHICLE[i][j];
		//		before_cust2 = VEHICLE[i][j-1];
		//		after_cust2 = VEHICLE[i][j+1];
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
		//bool *RFlag = new bool[no_routes];
		//for (int i = 0; i < no_routes; i++)
		//{
		//	RFlag[i] = true; //initially all routes are true
		//}

		//find route based on probability
		findDist_from_CGravity (cust1_r1, x, y, custRGravity, sorted_custRGravity);
		//for (int i = 0; i < no_routes; i++)
		//{
		//	CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		//}
		//CGravityfile <<endl;

		//******************************** find route based on probability ************************************//
		//float randNum = (float) rand() / float(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		//CGravityfile << "cust1_r1= "<<cust1_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p<<endl;
		//CGravityfile << "cust2_r1= "<<cust2_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p+1<<endl;
		int route2 = -1;//initialize

		for (int i = 0; i < no_routes; i++) //only travel half of the nearest route to the customer, the rest just skip because the shaking is not good
		{
			route2 = sorted_custRGravity[i][1];
			if ((route2 == rand_r1) || (saiz[route2] == 0) ) //if the same route, skip or if the saiz = 0 because this is 2-1 cannot from empty route
			{
				continue;
			}
			
			for (int j = 1; j <= saiz[route2]; j++) //swap positions are  equal to the saiz
			{
				cust1_r2 = VEHICLE[route2][j];
				before_cust2 = VEHICLE[route2][j-1];
				after_cust2 = VEHICLE[route2][j+1];
				demand_c3 = demand[cust1_r2];

				//r1 (delete 2, accept 1)
				origain1 = (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1]+ service_time[cust2_r1]) - (dist[before_cust1][cust1_r2] + dist[cust1_r2][after_cust1] + service_time[cust1_r2]) ; //old-new
				int remainingCust1 = saiz[rand_r1]- (rand_r1_p+1);//saiz-2ndPosdelete
				float oldCumDist1 = CumDist[rand_r1][rand_r1_p] + CumDist[rand_r1][rand_r1_p+1]; //2 customers originally from r1
				float newCumDist1 = CumDist[rand_r1][rand_r1_p-1] + dist[before_cust1][cust1_r2] + service_time[cust1_r2];
				if (remainingCust1<0)
					remainingCust1=0;
				gain1 = (oldCumDist1 - newCumDist1) + remainingCust1*origain1;

				//r2 (accept2, delete1)
				origain2 = (dist[before_cust2][cust1_r2] + dist[cust1_r2][after_cust2] + service_time[cust1_r2]) - (dist[before_cust2][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust2] + service_time[cust1_r1]+ service_time[cust2_r1]) ; //old-new
				int remainingCust2 = saiz[route2]- (j);//saiz-Posdelete
				float oldCumDist2 = CumDist[route2][j];//1 customers originally from r2
				float newCumDist2 = CumDist[route2][j-1] + dist[before_cust2][cust1_r1] + service_time[cust1_r1];
				float newCumDist2b = newCumDist2 + dist[cust1_r1][cust2_r1] + service_time[cust2_r1];
				if (remainingCust2<0)
					remainingCust2=0;
				gain2 = (oldCumDist2 - newCumDist2 - newCumDist2b) + remainingCust2*origain2;

				if ((demand_c3 <= (space_available[rand_r1]+demand_c1+demand_c2)) && ((demand_c1+demand_c2)<= (space_available[route2]+demand_c3)) && (distance_cost[rand_r1]-origain1 <= DISTANCE+epsilon) && (distance_cost[route2]-origain2 <= DISTANCE+epsilon)) //if fulfill both constraints
				{
					rand_r2 = route2;
					rand_r2_p = j;
					r2_found = 1;
					//CGravityfile << "Found R2 in SHake2_1TwoR!!!, R2= "<<route2<<" , rand_r2_p= "<<rand_r2_p<<", cust1_r2= "<<cust1_r2<<endl;
					goto found_r2_m5;
				}
			}
		}
		
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//

		if (r2_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			no_flag++;

			goto m5_regenerate;
		}
		
	found_r2_m5:
		//route_change[0] = rand_r1; //route (from)
		//route_change[1] = rand_r2; //route (to)
		//route_change[2] = -1; //reinitialize 
		//reinitializeRchangedStatus ();
		RchangedStatus[rand_r1] = true;//route (from)
		RchangedStatus[rand_r2] = true;//route (to)
		//Route[rand_r1].RchangedStatus = 1;//route (from)
		//Route[rand_r2].RchangedStatus = 1;//route (to)
		//int delete1=2;//if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		//int delete2=1;
		////r1
		//for (int i = -1; i <= delete1; i++)//flag the one before, current 1 and2, and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r1][rand_r1_p+i]] = true;//flagged customer affected true
		//}
		////r2
		//for (int i = -1; i <= delete2; i++)//flag the one before, current and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r2][rand_r2_p+i]] = true;//flagged customer affected true
		//}
		

	//******************************************************** END OF Neighbour_k 5 (2-1)  ******************************************************************************//

	// erase the element from route 1

	int customer1 = cust1_r1; 
	int customer2 = cust2_r1;
	int customer3 = cust1_r2; //second customer in route1

	
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

	gainvec[0] = 1;//status yes
	gainvec[1] = gain1;
	gainvec[2] = origain1;
	swap2_1_cust (rand_r1, rand_r1_p, VEHICLE, customer3);//2 out, 1 in

	gainvec[0] = 1;//status yes
	gainvec[1] = gain2;
	gainvec[2] = origain2;
	swap1_2_cust (rand_r2, rand_r2_p, VEHICLE, customer1, customer2);//1 out, 2 in

	float total_cost = 0.0;
	cout<<"============Routes after shake in 2-1 two routes============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		//float r_cost = 0.0;//to check
		//for (int c = 0; c <= saiz[i]; c++)
		//{
		//	r_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		//}
		//if (abs (r_cost-route_cost[i]) > 0.1)
		//{
		//	cout<<endl<<"After shake_2_1_twoR!!!!!!!!!"<<' ';
		//	cout<<"route_cost["<<i<<"]= "<<route_cost[i]<<' '<<"r_cost= "<<r_cost<<endl;
		//	cout<<rand_r1<<' '<<rand_r2<<' '<<rand_r1_p<<' '<<rand_r2_p<<' '<<gain1<<' '<<gain2<<endl;
		//	getchar();
		//}
		total_cost += route_cost[i]; 
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;

	delete[] flag_module;
	//delete[] RFlag;
	return 1; //shake status is ok
}

//DONE
int shake_2_1_threeR(int **(&VEHICLE), float *x, float *y)//routes will copy from BEST_ROUTE in this function, only shake the best route
{
	if (no_routes != LOCAL_NO_ROUTE)
	{
		cout<<"in shake2_1thr";
		getchar();
	}
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	
	//route_CGravity = new float*[no_routes];

	//for (int i = 0; i < no_routes; i++)
	//{
	//	route_CGravity[i] = new float[2]; //2 columns consists of x and y
	//}
	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	//**************all insertion and deletion starts from 1 to saiz because the vehic_depotCust starts from depot, end with depot**************//
	//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//

	int rand_r1 = -1, rand_r2 = -1, rand_r3 = -1, rand_r1_p = -1, rand_r2_p = -1, rand_r3_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1, cust1_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0, saiz_r3 = 0;
	int demand_c1, demand_c2, demand_c3;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1, before_cust3=-1, after_cust3=-1;
	float cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0, gain3=0.0;
	float origain1, origain2, origain3;
	
	//******************************************************** Neighbour_k 6 (2-1) threeRoutes  ******************************************************************************//
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}	
	int no_flag=0;
	for (int i = 0; i < no_routes; i++)
	{
		//if (saiz[i] == 1)//made comment on 29Jun2015, it will be flagged in if (saiz[i] != 0)
		//{
		//	flag_module[vehic_depotCust[i][1]] = false;
		//	no_flag++;
		//}
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			flag_module[VEHICLE[i][saiz[i]]] = false; //flag the last cust in a route false because last customer cannot be selected, this is 2-1 sequential//this is a trick!!!!!!!!!!!
			no_flag++;
		}
	}
	//============= Guided shake, calculate center of gravity ============////put this before m6_regenerate, previously calculate center of gravity everytime generate a customer, OMGGGGGG!!!!!!!! on 29Jun2015
	//calculate_centreGravity(x, y);
	//for (int i = 0; i < no_routes; i++)
	//{
	//	CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
	//}
	//CGravityfile <<endl;
	//============= End of calculate center of gravity ============//
	float internal_start_s = clock();// the code you wish to time goes here
	m6_regenerate:
		if (no_flag >= SIZE)
		{	
			cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE2_1 THREE ROUTES!" <<endl;
			for (int i = 0; i < SIZE; i++)
			{
				cout<<flag_module[i]<<' ';
			}
			//getchar();
			return 0;
		}
		int r2_found=0;
		float timeElapsed = (clock() - internal_start_s) / float(CLOCKS_PER_SEC) ;
		if(timeElapsed > 15)//if more than 15seconds
		{
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i] ==0 || saiz[i]==1)
					continue;
				for (int j = 1; j <= saiz[i]-1; j++)
				{
					if (flag_module[VEHICLE[i][j]] == false)//if it is flagged false, just skipped it
						continue;
					rand_r1 = i;
					saiz_r1 = saiz[i];
					rand_r1_p = j;
					goto getout;
				}
			}
		}
		else
		{
			srand ( time(NULL) ); //seed it
			rand_r1 = (rand() % no_routes); 
			saiz_r1 = saiz[rand_r1];

			if((saiz_r1 == 0) || (saiz_r1 == 1))//have to test this before finding the position to avoid division by zero error
				goto m6_regenerate;

			rand_r1_p = (rand() % (saiz_r1-1))+1; //position is from 1 to saiz, eg vehic_depotCust: SIZE 1 2 3 SIZE; //minus one because two customers will be deleted in sequential
		}
	getout:
		cust1_r1 = VEHICLE[rand_r1][rand_r1_p];

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or from flag false //make sure deletion cannot be made from empty route
			goto m6_regenerate;
		
		cust2_r1 = VEHICLE[rand_r1][rand_r1_p+1];//customer in sequential		
		demand_c1 = demand[cust1_r1];
		demand_c2 = demand[cust2_r1];		
		before_cust1 = VEHICLE[rand_r1][rand_r1_p-1];
		after_cust1 = VEHICLE[rand_r1][rand_r1_p+2];
		
		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < no_routes; i++)
		//{
		//	if ((i == rand_r1) || (saiz[i] == 0) ) //if the same route, skip or if the saiz = 0 because this is 2-1 cannot from empty route
		//		continue;
		//	
		//	for (int j = 1; j <= saiz[i]; j++) //swap positions are  equal to the saiz
		//	{
		//		cust1_r2 = VEHICLE[i][j];
		//		before_cust2 = VEHICLE[i][j-1];
		//		after_cust2 = VEHICLE[i][j+1];
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
		//bool *RFlag = new bool[no_routes];
		//for (int i = 0; i < no_routes; i++)
		//{
		//	RFlag[i] = true; //initially all routes are true
		//}

		//find route based on probability
		findDist_from_CGravity (cust1_r1, x, y, custRGravity, sorted_custRGravity);
		//for (int i = 0; i < no_routes; i++)
		//{
		//	CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		//}
		//CGravityfile <<endl;
		
		
		//******************************** find route based on probability ************************************//
		//float randNum = (float) rand() / float(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		//CGravityfile << "cust1_r1= "<<cust1_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p<<endl;
		//CGravityfile << "cust2_r1= "<<cust2_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p+1<<endl;
		int route2 = -1;//initialize
		for (int i = 0; i < no_routes; i++) //only travel half of the nearest route to the customer, the rest just skip because the shaking is not good
		{
			route2 = sorted_custRGravity[i][1];
			if ((route2 == rand_r1) || (saiz[route2] == 0) ) //if the same route, skip or if the saiz = 0 because this is 2-1 cannot from empty route
			{
				continue;
			}
			
			for (int j = 1; j <= saiz[route2]; j++) //swap positions are  equal to the saiz
			{
				cust1_r2 = VEHICLE[route2][j];
				before_cust2 = VEHICLE[route2][j-1];
				after_cust2 = VEHICLE[route2][j+1];
				demand_c3 = demand[cust1_r2];

				//r1 (delete 2, accept 1)
				origain1 = (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1]+ service_time[cust2_r1]) - (dist[before_cust1][cust1_r2] + dist[cust1_r2][after_cust1] + service_time[cust1_r2]) ; //old-new
				int remainingCust1 = saiz[rand_r1]- (rand_r1_p+1);//saiz-2ndPosdelete
				float oldCumDist1 = CumDist[rand_r1][rand_r1_p] + CumDist[rand_r1][rand_r1_p+1]; //2 customers originally from r1
				float newCumDist1 = CumDist[rand_r1][rand_r1_p-1] + dist[before_cust1][cust1_r2] + service_time[cust1_r2];
				if (remainingCust1<0)
					remainingCust1=0;
				gain1 = (oldCumDist1 - newCumDist1) + remainingCust1*origain1;

				//r2 (accept1, delete1)
				origain2 = (dist[before_cust2][cust1_r2] + dist[cust1_r2][after_cust2] + service_time[cust1_r2]) - (dist[before_cust2][cust1_r1] + dist[cust1_r1][after_cust2] + service_time[cust1_r1]) ; //old-new
				int remainingCust2 = saiz[route2]- (j);//saiz-Posdelete
				float oldCumDist2 = CumDist[route2][j];//1 customers originally from r2
				float newCumDist2 = CumDist[route2][j-1] + dist[before_cust2][cust1_r1] + service_time[cust1_r1];
				if (remainingCust2<0)
					remainingCust2=0;
				gain2 = (oldCumDist2 - newCumDist2) + remainingCust2*origain2;


				if ((demand_c3 <= (space_available[rand_r1]+demand_c1+demand_c2)) && (demand_c1<= (space_available[route2]+demand_c3)) && (distance_cost[rand_r1]-origain1 <= DISTANCE+epsilon) && (distance_cost[route2]-origain2 <= DISTANCE+epsilon)) //if fulfill both constraints
				{
					rand_r2 = route2;
					rand_r2_p = j;
					r2_found = 1;
					//CGravityfile << "Found R2 in SHake2_1ThreeR!!!, R2= "<<route2<<" , rand_r2_p= "<<rand_r2_p<<", cust1_r2= "<<cust1_r2<<endl;
					goto found_r2_m6;
				}
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

		//================== find r3 GUIDED SHAKE =========================//
		//bool *R3Flag = new bool[no_routes];
		//for (int i = 0; i < no_routes; i++)
		//{
		//	R3Flag[i] = true; //initially all routes are true
		//}

		findDist_from_CGravity (cust2_r1, x, y, custRGravity, sorted_custRGravity);//find for second customer where to insert too//added on 29June2015
		//for (int i = 0; i < no_routes; i++)
		//{
		//	CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		//}
		//CGravityfile <<endl;

		int r3_found = 0;
		int route3 = -1;//initialize

		for (int i = 0; i < no_routes; i++) //only travel half of the nearest route to the customer, the rest just skip because the shaking is not good
		{
			route3 = sorted_custRGravity[i][1];
			if ((route3 == rand_r1) || (route3 == route2) || (demand_c2 > space_available[route3]) ) //if the same route, skip 
			{
				continue;
			}
			//if (saiz[route3] == 0)
			//{
			//	rand_r3 = route3;
			//	rand_r3_p = 1;
			//	r3_found = 1;
			//	//CGravityfile << "Found R3 in SHake2_1ThreeR!!!, R3= "<<route3<<" , rand_r3_p= "<<rand_r3_p<<endl;
			//	goto found_r3_m6;
			//}
			for (int j = 1; j <= saiz[route3]+1; j++) //insertion can be made one position more than saiz
			{
				before_cust3 = VEHICLE[route3][j-1];
				after_cust3 = VEHICLE[route3][j]; //current position

				//r3 (insert 1)
				origain3 = (dist[before_cust3][after_cust3]) - (dist[before_cust3][cust2_r1] + dist[cust2_r1][after_cust3] + service_time[cust2_r1]) ; //new-old //CHANGED THIS ON 4 MARC 2015!!!!!!!!!!!!!!!CARELESS MISTAKE!!!!!!!!!!!!!!
				int remainingCust3 = saiz[route3]- (j) +1;//saiz – posinsert + 1 
				float newCumDist3 = CumDist[route3][j-1] + dist[before_cust3][cust2_r1] + service_time[cust2_r1];
				if (remainingCust3<0)
					remainingCust3=0;
				gain3 = - newCumDist3 + remainingCust3*origain3;


				if ((demand_c2 <= space_available[route3]) && (distance_cost[route3]-origain3 <= DISTANCE+epsilon)) //if fulfill both constraints
				{
					rand_r3 = route3;
					rand_r3_p = j;
					r3_found = 1;
					//CGravityfile << "Found R3 in SHake2_1ThreeR!!!, R3= "<<route3<<" , rand_r3_p= "<<rand_r3_p<<endl;
					goto found_r3_m6;
				}
			}
		}

		//==================Uncomment here for find r3 no guided shake=========================//
		//int r3_found = 0;
		//for (int i = 0; i < no_routes; i++)
		//{
		//	if ((i == rand_r1) || (i == rand_r2) ) //if the same route, skip, saiz = 0 is ok because insertion can go to empty route
		//		continue;
		//	
		//	if (saiz[i] == 0)
		//	{
		//		rand_r3 = i;
		//		rand_r3_p = 1;
		//		r3_found = 1;
		//		goto found_r3_m6;
		//	}
		//	for (int j = 1; j <= saiz[i]+1; j++) //insertion can be made one position more than saiz
		//	{
		//		before_cust3 = VEHICLE[i][j-1];
		//		after_cust3 = VEHICLE[i][j]; //current position
		//		rand_r3 = i;

		//		gain3 = (dist[before_cust3][cust2_r1] + dist[cust2_r1][after_cust3] + service_time[cust2_r1]) - (dist[before_cust3][after_cust3]); //new-old

		//		if ((demand_c2 <= space_available[i]) && (gain3 <= distance_available[i])) //if fulfill both constraints
		//		{
		//			rand_r3 = i;
		//			rand_r3_p = j;
		//			r3_found = 1;
		//			goto found_r3_m6;
		//		}

		//	}
		//}
		//================== End of Uncomment here for find r3 no guided shake=========================//
		if (r3_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd //this will redo everything again!!!!!!!!!!!!!!!!!!!!
			no_flag++;	
			goto m6_regenerate;
		}
			
	found_r3_m6:
			
		if ( (cust1_r1 == -1) || (cust2_r1 == -1) || (cust1_r2 == -1) )
			cout<< "cust1_r1 or cust1_r2 cannot be negative!!" <<endl;
			

		//route_change[0] = rand_r1; //route (from)
		//route_change[1] = rand_r2; //route (to)
		//route_change[2] = rand_r3; //route (to)
		//reinitializeRchangedStatus ();
		RchangedStatus[rand_r1] = true;//route (from)
		RchangedStatus[rand_r2] = true;//route (to)
		RchangedStatus[rand_r3] = true;//route (to)
		//Route[rand_r1].RchangedStatus = 1;//route (from)
		//Route[rand_r2].RchangedStatus = 1;//route (to)
		//Route[rand_r3].RchangedStatus = 1;//route (to)
		//int delete1=2;//if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		//int delete2=1;
		//int delete3=0;
		////r1
		//for (int i = -1; i <= delete1; i++)//flag the one before, current 1 and 2, and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r1][rand_r1_p+i]] = true;//flagged customer affected true
		//}
		////r2
		//for (int i = -1; i <= delete2; i++)//flag the one before, current, and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r2][rand_r2_p+i]] = true;//flagged customer affected true
		//}
		////r3
		//for (int i = -1; i <= delete3; i++)//flag the one before and current
		//{
		//	CustaffectedStatus[VEHICLE[rand_r3][rand_r3_p+i]] = true;//flagged customer affected true
		//}	


	//******************************************************** END OF Neighbour_k 6 (2-1) threeRoute  ******************************************************************************//
	int customer1 = cust1_r1; 
	int customer2 = cust2_r1; //second customer in route1
	int customer3 = cust1_r2; 

	
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

	gainvec[0] = 1;//status yes
	gainvec[1] = gain1;
	gainvec[2] = origain1;
	swap2_1_cust (rand_r1, rand_r1_p, VEHICLE, customer3);//2 out, 1 in for route1

	gainvec[0] = 1;//status yes
	gainvec[1] = gain2;
	gainvec[2] = origain2;
	swap_in_oneCust (rand_r2, rand_r2_p, VEHICLE, customer1); //1 in, 1 out for route2

	gainvec[0] = 1;//status yes
	gainvec[1] = gain3;
	gainvec[2] = origain3;
	insert_one_cust (rand_r3, rand_r3_p, VEHICLE, customer2); //1 in for route3

	float total_cost = 0.0;
	cout<<"============Routes after shake in 2-1 three routes============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		//float r_cost = 0.0;//to check
		//for (int c = 0; c <= saiz[i]; c++)
		//{
		//	r_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		//}
		//if (abs (r_cost-route_cost[i]) > 0.1)
		//{
		//	cout<<endl<<"After shake shake_2_1_threeR!!!!!!!!!"<<' ';
		//	cout<<"route_cost["<<i<<"]= "<<route_cost[i]<<' '<<"r_cost= "<<r_cost<<endl;
		//	cout<<rand_r1<<' '<<rand_r2<<' '<<rand_r1_p<<' '<<rand_r2_p<<' '<<gain1<<' '<<gain2<<endl;
		//	getchar();
		//}
		total_cost += route_cost[i];
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	
	delete[] flag_module;
	//delete[] RFlag;
	//delete[] R3Flag;
	return 1; //shake status is ok
}

//DONE
int shake_2_2(int **(&VEHICLE), float *x, float *y)//added 29June2015
{
	if (no_routes != LOCAL_NO_ROUTE)
	{
		cout<<"in shake2_2";
		getchar();
	}
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	
	//route_CGravity = new float*[no_routes];

	//for (int i = 0; i < no_routes; i++)
	//{
	//	route_CGravity[i] = new float[2]; //2 columns consists of x and y
	//}

	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0

	int rand_r1 = -1, rand_r2 = -1,  rand_r1_p = -1, rand_r2_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete for 1-1 and 2-1)
	int cust1_r1 = -1, cust2_r1 = -1, cust1_r2 = -1, cust2_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0;
	int demand_c1, demand_c2, demand_c3, demand_c4;
	int before_cust1=-1, after_cust1=-1, before_cust2=-1, after_cust2=-1;
	float cost_without_i=0.0, cost_with_i=0.0, cost_without_i2=0.0, cost_with_i2=0.0, gain1=0.0, gain2=0.0;
	float origain1, origain2;
	//********************************************************  Neighbour_k 7 (2-2)  ******************************************************************************//

	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}
	int no_flag=0;
	for (int i = 0; i < no_routes; i++)
	{
		//if (saiz[i] == 1)//made comment on 29Jun2015, it will be flagged in if (saiz[i] != 0)
		//{
		//	flag_module[vehic_depotCust[i][1]] = false;
		//	no_flag++;
		//}
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			flag_module[VEHICLE[i][saiz[i]]] = false; //flag the last cust in a route false because last customer cannot be selected, this is 2-2 sequential//this is a trick!!!!!!!!!!!
			no_flag++;
		}
	}
	//============= Guided shake, calculate center of gravity ============//put this before m2_regenerate, previously calculate center of gravity everytime generate a customer, OMGGGGGG!!!!!!!! on 29Jun2015
	//calculate_centreGravity(x, y);
	//for (int i = 0; i < no_routes; i++)
	//{
	//	CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
	//}
	//CGravityfile <<endl;
	//============= End of calculate center of gravity ============//
	float internal_start_s = clock();// the code you wish to time goes here
	m7_regenerate:
		if (no_flag >= SIZE)
		{	
			cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE 2_2!" <<endl;
			for (int i = 0; i < SIZE; i++)
			{
				cout<<flag_module[i]<<' ';
			}
			//getchar();
			return 0;
		}
		int r2_found =0 ;
		float timeElapsed = (clock() - internal_start_s) / float(CLOCKS_PER_SEC) ;
		if(timeElapsed > 15)//if more than 15seconds
		{
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i] ==0 || saiz[i]==1)
					continue;
				for (int j = 1; j <= saiz[i]-1; j++)
				{
					if (flag_module[VEHICLE[i][j]] == false)//if it is flagged false, just skipped it
						continue;
					rand_r1 = i;
					saiz_r1 = saiz[i];
					rand_r1_p = j;
					goto getout;
				}
			}
		}
		else
		{
			srand ( time(NULL) ); //seed it
			rand_r1 = (rand() % no_routes); // for deletion cannot delete from empty route (last route), NO_BEST_ROUTEs = 3 , [0][1][2], can only delete from [0][1]
			saiz_r1 = saiz[rand_r1];
		
			if((saiz_r1 == 0) || (saiz_r1 == 1))//have to test this before finding the position to avoid division by zero error
				goto m7_regenerate;

			rand_r1_p = (rand() % (saiz_r1-1))+1; //position is from 1 to saiz-1, eg vehic_depotCust: SIZE 1 2 3 SIZE
		}

	getout:
		cust1_r1 = VEHICLE[rand_r1][rand_r1_p];

		if(flag_module[cust1_r1] == false) //make sure deletion cannot be made from empty route or from flag false
			goto m7_regenerate;
		
		cust2_r1 = VEHICLE[rand_r1][rand_r1_p+1];
		demand_c1 = demand[cust1_r1];
		demand_c2 = demand[cust2_r1];
		before_cust1 = VEHICLE[rand_r1][rand_r1_p-1];
		after_cust1 = VEHICLE[rand_r1][rand_r1_p+2];

		//================================ Change here for guided shake ================================///
		//for (int i = 0; i < no_routes; i++)
		//{
		//	if ((i == rand_r1) || (saiz[i] == 0) ) //if the same route, skip or if the saiz = 0 because this is 1-1 cannot from empty route
		//		continue;
		//	
		//	for (int j = 1; j <= saiz[i]-1; j++) //swap positions are  equal to the saiz-1
		//	{
		//		cust1_r2 = VEHICLE[i][j];
		//		cust2_r2 = VEHICLE[i][j+1];
		//		before_cust2 = VEHICLE[i][j-1];
		//		after_cust2 = VEHICLE[i][j+2];
		//		demand_c3 = demand[cust1_r2];
		//		demand_c4 = demand[cust2_r2];
		//		rand_r2 = i;

		//		gain1 = (dist[before_cust1][cust1_r2] + dist[cust1_r2][cust2_r2] + dist[cust2_r2][after_cust1] + service_time[cust1_r2] + service_time[cust2_r2]) - (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1] + service_time[cust2_r1]); //new-old
		//		gain2 = (dist[before_cust2][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust2] + service_time[cust1_r1] + service_time[cust2_r1]) - (dist[before_cust2][cust1_r2] + dist[cust1_r2][cust2_r2] + dist[cust2_r2][after_cust2] + service_time[cust1_r2] + service_time[cust2_r2]); //new-old

		//		if ((demand_c1+demand_c2 <= (space_available[rand_r2]+demand_c3+demand_c4)) && (demand_c3+demand_c4 <= (space_available[rand_r1]+demand_c1+demand_c2)) && (gain1 <= distance_available[rand_r1]) && (gain2 <= distance_available[rand_r2])) //if fulfill both constraints
		//		{
		//			rand_r2 = i;
		//			rand_r2_p = j;
		//			r2_found = 1;
		//			goto found_r2_m7;
		//		}

		//	}
		//}
		//================================ End (Change here for guided shake) ================================///
		//++++++++++++++++++++++++++++++++ Guided shake +++++++++++++++++++++++++++++++++++++++++++++++++++++//
		//bool *RFlag = new bool[no_routes];
		//for (int i = 0; i < no_routes; i++)
		//{
		//	RFlag[i] = true; //initially all routes are true
		//}

		findDist_from_CGravity (cust1_r1, x, y, custRGravity, sorted_custRGravity);
		//for (int i = 0; i < no_routes; i++)
		//{
		//	CGravityfile << sorted_custRGravity[i][0]<< ' '<<' '<< sorted_custRGravity[i][1]<< ' '<<' '<< sorted_custRGravity[i][2]<< ' '<<' '<< sorted_custRGravity[i][3]<< ' '<<' '<< sorted_custRGravity[i][4]<<endl;
		//}
		//CGravityfile <<endl;
	
		
		//******************************** find route based on probability ************************************//
		//float randNum = (float) rand() / float(RAND_MAX); //generate a number between 0 and 1
		//route2 = findRoute_fromRand (sorted_custRGravity, no_routes, randNum);//based on rand, find the route number and return, call onceSelected_shift_up ()
		
		//******************************** find route based on nearest route in descending order ************************************//
		//CGravityfile << "cust1_r1= "<<cust1_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p<<endl;
		//CGravityfile << "cust2_r1= "<<cust2_r1<<" from route "<<rand_r1<<" position "<<rand_r1_p+1<<endl;
		int route2 = -1;//initialize
		for (int i = 0; i < no_routes; i++) //only travel half of the nearest route to the customer, the rest just skip because the shaking is not good
		{
			route2 = sorted_custRGravity[i][1];

			if ((saiz[route2] == 2) && (saiz[rand_r1] == 2)) //if both same size 2, after swap also the same, so skip
			{
				continue;
			}

			if ((route2 == rand_r1) || (saiz[route2] == 0)) //if the same route, skip or if the saiz = 0 because this is 2-2 cannot from empty route
			{
				continue;
			}

			for (int j = 1; j <= saiz[route2]-1; j++) //swap positions are  equal to the saiz-1
			{
				cust1_r2 = VEHICLE[route2][j];
				cust2_r2 = VEHICLE[route2][j+1];
				before_cust2 = VEHICLE[route2][j-1];
				after_cust2 = VEHICLE[route2][j+2];
				demand_c3 = demand[cust1_r2];
				demand_c4 = demand[cust2_r2];
				
				//r1 (accept2, delete2)
				origain1 = (dist[before_cust1][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust1] + service_time[cust1_r1] + service_time[cust2_r1]) - (dist[before_cust1][cust1_r2] + dist[cust1_r2][cust2_r2] + dist[cust2_r2][after_cust1] + service_time[cust1_r2] + service_time[cust2_r2]) ; //old -new
				int remainingCust1 = saiz[rand_r1]- (rand_r1_p+1);//saiz – 2ndpos delete
				float oldCumDist1 = CumDist[rand_r1][rand_r1_p] + CumDist[rand_r1][rand_r1_p+1]; //2 customers originally from r1
				float newCumDist1 = CumDist[rand_r1][rand_r1_p-1] + dist[before_cust1][cust1_r2] + service_time[cust1_r2];
				float newCumDist1b = newCumDist1 + dist[cust1_r2][cust2_r2] + service_time[cust2_r2];
				if (remainingCust1<0)
					remainingCust1=0;
				gain1= (oldCumDist1 - newCumDist1 - newCumDist1b) + remainingCust1*origain1;

				//r2 (accept2, delete2)
				origain2 = (dist[before_cust2][cust1_r2] + dist[cust1_r2][cust2_r2] + dist[cust2_r2][after_cust2] + service_time[cust1_r2] + service_time[cust2_r2]) - (dist[before_cust2][cust1_r1] + dist[cust1_r1][cust2_r1] + dist[cust2_r1][after_cust2] + service_time[cust1_r1] + service_time[cust2_r1]) ; //old- new
				int remainingCust2 = saiz[route2]- (j+1);//saiz – 2ndpos delete
				float oldCumDist2 = CumDist[route2][j] + CumDist[route2][j+1];//2 customers originally from r2
				float newCumDist2 = CumDist[route2][j-1] + dist[before_cust2][cust1_r1] + service_time[cust1_r1];
				float newCumDist2b = newCumDist2 + dist[cust1_r1][cust2_r1] + service_time[cust2_r1];
				if (remainingCust2<0)
					remainingCust2=0;
				gain2 = (oldCumDist2 - newCumDist2 - newCumDist2b) + remainingCust2*origain2;

				
				if ((demand_c1+demand_c2 <= (space_available[route2]+demand_c3+demand_c4)) && (demand_c3+demand_c4 <= (space_available[rand_r1]+demand_c1+demand_c2)) && (distance_cost[rand_r1]-origain1 <= DISTANCE+epsilon) && (distance_cost[route2]-origain2 <= DISTANCE+epsilon)) //if fulfill both constraints
				{
					rand_r2 = route2;
					rand_r2_p = j;
					r2_found = 1;
					//CGravityfile << "Found r2 in shake 2-2, r2= "<<rand_r2<<", rand_r2_p= "<<rand_r2_p<<", cust1_r2= "<<cust1_r2<<", cust2_r2= "<<cust2_r2<<endl;
					goto found_r2_m7;
				}
			}
		}
			
		//++++++++++++++++++++++++++++++++ End (Guided shake) +++++++++++++++++++++++++++++++++++++++++++++++++++++//

		if (r2_found != 1)
		{
			flag_module[cust1_r1] = false; //flag it to false so that it would not be reselecetd
			no_flag++;

			goto m7_regenerate;
		}
		
	found_r2_m7:
		if ( (cust1_r1 == -1) || (cust1_r2 == -1) )
			cout<< "cust1_r1 or cust1_r2 cannot be negative!!" <<endl;
		
		//reinitializeRchangedStatus ();
		RchangedStatus[rand_r1] = true;//route (from)
		RchangedStatus[rand_r2] = true;//route (to)
		//Route[rand_r1].RchangedStatus = 1;//route (from)
		//Route[rand_r2].RchangedStatus = 1;//route (to)
		//int delete1=2;//if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		////r1
		//for (int i = -1; i <= delete1; i++)//flag the one before, current 1and2, and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r1][rand_r1_p+i]] = true;//flagged customer affected true
		//}
		////r2
		//for (int i = -1; i <= delete1; i++)//flag the one before, current 1and2, and one after
		//{
		//	CustaffectedStatus[VEHICLE[rand_r2][rand_r2_p+i]] = true;//flagged customer affected true
		//}
			


	//******************************************************** END OF Neighbour_k 7 (2-2)  ******************************************************************************//
	int customer1 = cust1_r1; 
	int customer2 = cust2_r1;
	int customer3 = cust1_r2; 
	int customer4 = cust2_r2;

	if ( (customer1 < 0) || (customer2 < 0) || (customer3 < 0) || (customer4 < 0))
	{
		cout<< "customer to be deleted cannot be negative!!!!!!" <<endl;
		cout<< "r1= " <<rand_r1 <<' '<< "r2= " <<rand_r2 <<endl;
		cout<< "r1_p= " <<rand_r1_p<<' ' << "r2_p= " <<rand_r2_p<<endl;
		cout<< "customer1 = "<< customer1 << " customer2 = "<< customer2<<endl;
		cout<< "customer3 = "<< customer3 << " customer4 = "<< customer4<<endl;
	}
	
	
	cout<<"===========CUST from shake 2-2=========" <<endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p << "]=" <<customer1<<" route[" << rand_r1 <<"][" << rand_r1_p+1 << "]=" <<customer2<< endl;
	cout<< "route[" << rand_r2 <<"][" << rand_r2_p << "]=" <<customer3<<" route[" << rand_r2 <<"][" << rand_r2_p+1 << "]=" <<customer4<< endl;

	gainvec[0] = 1;//status yes
	gainvec[1] = gain1;
	gainvec[2] = origain1;
	swap_in_twoCust (rand_r1, rand_r1_p, VEHICLE, customer3, customer4);

	gainvec[0] = 1;//status yes
	gainvec[1] = gain2;
	gainvec[2] = origain2;
	swap_in_twoCust (rand_r2, rand_r2_p, VEHICLE, customer1, customer2);


	float total_cost = 0.0;

	cout<<"============ Routes after shake in 2-2 ============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		//float r_cost = 0.0;//to check
		//for (int c = 0; c <= saiz[i]; c++)
		//{
		//	r_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		//}
		//if (abs (r_cost-route_cost[i]) > 0.1)
		//{
		//	cout<<endl<<"After shake_2_2!!!!!!!!!"<<' ';
		//	cout<<"route_cost["<<i<<"]= "<<route_cost[i]<<' '<<"r_cost= "<<r_cost<<endl;
		//	cout<<rand_r1<<' '<<rand_r2<<' '<<rand_r1_p<<' '<<rand_r2_p<<' '<<gain1<<' '<<gain2<<endl;
		//	getchar();
		//}
		total_cost += route_cost[i]; 
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	
	delete[] flag_module;
	return 1; //shake status is ok
}


int shake_reshuffle (int **(&VEHICLE))
{
	srand ( time(NULL) ); //seed it
	int r = (rand() % no_routes); // for deletion cannot delete from empty route (last route), NO_BEST_ROUTEs = 3 , [0][1][2], can only delete from [0][1]
	
	
	std::vector<int> v1; 

	for (int i = 1; i <= saiz[r]; i++)
	{
		v1.push_back(VEHICLE[r][i]);
	}
	srand ( time(NULL) ); //seed it
	std::random_shuffle ( v1.begin(), v1.end() );

	int k=0;//for v1
	CumDist[r][0]=0;
	route_cost[r]=0;
	distance_cost[r]=0;
	for (int i = 1; i <= saiz[r]; i++)
	{
		VEHICLE[r][i] = v1[k];
		CumDist[r][i] = CumDist[r][i-1] + dist[VEHICLE[r][i-1]][VEHICLE[r][i]] + service_time[VEHICLE[r][i]];
		route_cost[r] += CumDist[r][i];
		distance_cost[r] += dist[VEHICLE[r][i-1]][VEHICLE[r][i]] + service_time[VEHICLE[r][i]];
		k++;
	}
	CumDist[r][saiz[r]+1] = CumDist[r][saiz[r]] + dist[VEHICLE[r][saiz[r]]][VEHICLE[r][saiz[r]+1]];//actually dont need this one
	distance_cost[r] += dist[VEHICLE[r][saiz[r]]][SIZE];
	distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp soluti

	RchangedStatus[r] = true;//route (from)
		

	float total_cost=0;
	cout<<"============ Routes after shake reshuffle ============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		
		total_cost += route_cost[i]; 
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	v1.clear();
	v1.shrink_to_fit();
	return 1; //shake status is ok
}

int shake_intraSegmentReshuffle(int **(&VEHICLE))//added 12June2016
{
intraSegment_regenerate:

	srand ( time(NULL) ); //seed it
	int r = (rand() % no_routes); // for deletion cannot delete from empty route (last route), NO_BEST_ROUTEs = 3 , [0][1][2], can only delete from [0][1]
	int saiz_r1 = saiz[r];
	int averageCustperroute = (float)SIZE/(float)no_routes;
	int maxL = ceil((float)averageCustperroute*0.6);
	int minL = ceil((float)averageCustperroute*0.4);

	srand ( time(NULL) ); //seed it
	int L1 = (rand() % (maxL-minL+1))+minL;//length of segment

	if(saiz_r1 < L1)
		goto intraSegment_regenerate;

	int rand_r1_p = (rand() % (saiz_r1-L1+1))+1; //position is from 1 to saiz_r1-L1+1, eg vehic_depotCust: SIZE 1 2 3 SIZE


	std::vector<int> v1; 

	for (int i = rand_r1_p; i <= rand_r1_p+L1-1; i++)
	{
		v1.push_back(VEHICLE[r][i]);
	}
	srand ( time(NULL) ); //seed it
	std::random_shuffle ( v1.begin(), v1.end() );

	int k=0;//for v1

	for (int i = rand_r1_p; i <= rand_r1_p+L1-1; i++)
	{
		VEHICLE[r][i] = v1[k];
		k++;
	}
	
	route_cost[r]=0;
	distance_cost[r]=0;

	for (int i = 1; i <= rand_r1_p-1; i++)//this part is not changed, think of a formula to compute this part
	{
		route_cost[r] += CumDist[r][i];
		distance_cost[r] += dist[VEHICLE[r][i-1]][VEHICLE[r][i]] + service_time[VEHICLE[r][i]];
	}

	for (int i = rand_r1_p; i <= saiz[r]; i++)
	{
		CumDist[r][i] = CumDist[r][i-1] + dist[VEHICLE[r][i-1]][VEHICLE[r][i]] + service_time[VEHICLE[r][i]];
		route_cost[r] += CumDist[r][i];
		distance_cost[r] += dist[VEHICLE[r][i-1]][VEHICLE[r][i]] + service_time[VEHICLE[r][i]];
	}
	CumDist[r][saiz[r]+1] = CumDist[r][saiz[r]] + dist[VEHICLE[r][saiz[r]]][VEHICLE[r][saiz[r]+1]];//actually dont need this one
	distance_cost[r] += dist[VEHICLE[r][saiz[r]]][SIZE];
	distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp soluti

	RchangedStatus[r] = true;//route (from)
		

	float total_cost=0;
	cout<<"============ Routes after shake reshuffle segment============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		
		total_cost += route_cost[i]; 
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	v1.clear();
	v1.shrink_to_fit();
	return 1; //shake status is ok
}

int shake_intraHeadReshuffle(int **(&VEHICLE), int neighbourhood)//added 16June2016, based on neighbourhood, reshuffle more route
{
	int Rreshuffled = neighbourhood;//number of routes gonna be reshuffled based on neighbourhood 
	if (no_routes < neighbourhood)
		Rreshuffled = no_routes;
	bool *routealreadychosen = new bool[no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		routealreadychosen[i] = false;
	}
	srand ( time(NULL) ); //seed it
	int averageCustperroute = (float)SIZE/(float)no_routes;
	int maxL = ceil((float)averageCustperroute*0.5);
	int minL = ceil((float)averageCustperroute*0.3);
	int L1 = (rand() % (maxL-minL+1))+minL;//length of segment
	for (int R = 0; R < Rreshuffled; R++)
	{
		regenerateHeadshake:
		int r = (rand() % no_routes); // for deletion cannot delete from empty route (last route), NO_BEST_ROUTEs = 3 , [0][1][2], can only delete from [0][1]
		int saiz_r1 = saiz[r];

		if(saiz_r1 < L1 || routealreadychosen[r] == true)
			goto regenerateHeadshake;

		//int rand_r1_p = (rand() % (saiz_r1-L1+1))+1; //position is from 1 to saiz_r1-L1+1, eg vehic_depotCust: SIZE 1 2 3 SIZE

		std::vector<int> v1; 

		for (int i = 1; i <= L1; i++)
		{
			v1.push_back(VEHICLE[r][i]);
		}
		srand ( time(NULL) ); //seed it
		std::random_shuffle ( v1.begin(), v1.end() );

		int k=0;//for v1

		for (int i = 1; i <= L1; i++)
		{
			VEHICLE[r][i] = v1[k];
			k++;
		}
	
		route_cost[r]=0;
		distance_cost[r]=0;

		//think of a formula to compute this part
		for (int i = 1; i <= saiz[r]; i++)
		{
			CumDist[r][i] = CumDist[r][i-1] + dist[VEHICLE[r][i-1]][VEHICLE[r][i]] + service_time[VEHICLE[r][i]];
			route_cost[r] += CumDist[r][i];
			distance_cost[r] += dist[VEHICLE[r][i-1]][VEHICLE[r][i]] + service_time[VEHICLE[r][i]];
		}
		CumDist[r][saiz[r]+1] = CumDist[r][saiz[r]] + dist[VEHICLE[r][saiz[r]]][VEHICLE[r][saiz[r]+1]];//actually dont need this one
		distance_cost[r] += dist[VEHICLE[r][saiz[r]]][SIZE];
		distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp soluti

		RchangedStatus[r] = true;
		routealreadychosen[r] = true;//for the use in this function, flagged the route true so it wont be selected again

		float total_cost=0;
		cout<<"============ Routes after shake reshuffle Head============"<<endl;
		for (int i = 0; i < no_routes; i++)
		{
			cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
			for (int k = 0; k <= saiz[i]+1; k++)
			{
				cout << VEHICLE[i][k] << ' ';
			}
		
			total_cost += route_cost[i]; 
			cout << endl;

		}
		cout<<"total cost = " <<total_cost<<endl;
		v1.clear();
		v1.shrink_to_fit();
	}//end of R
	delete[] routealreadychosen;
	return 1; //shake status is ok
}

int shake_intraReverseSegment(int **(&VEHICLE), float *x, float *y)//added 10June2016
{
	if (no_routes != LOCAL_NO_ROUTE)
	{
		cout<<"in shake_intraReverseSegment";
		getchar();
	}

	int rand_r1=-1, rand_r1_p=-1, saiz_r1;
	//********************************************************  Neighbour_k 0 (shake_intraReverseSegment)  ******************************************************************************//

m7_regenerate:
	srand ( time(NULL) ); //seed it
	rand_r1 = (rand() % no_routes); // for deletion cannot delete from empty route (last route), NO_BEST_ROUTEs = 3 , [0][1][2], can only delete from [0][1]
	saiz_r1 = saiz[rand_r1];
	
	int maxL = 7;
	int minL = 3;
	srand ( time(NULL) ); //seed it
	int L1 = (rand() % (maxL-minL+1))+minL;//length of segment

	if(saiz_r1 < L1)
		goto m7_regenerate;

	rand_r1_p = (rand() % (saiz_r1-L1+1))+1; //position is from 1 to saiz_r1-L1+1, eg vehic_depotCust: SIZE 1 2 3 SIZE
	
	int prevC = VEHICLE[rand_r1][rand_r1_p-1];
	int afterC= VEHICLE[rand_r1][rand_r1_p+L1];
	int cStart = VEHICLE[rand_r1][rand_r1_p];//start of customer
	int cEnd = VEHICLE[rand_r1][rand_r1_p+L1-1];//end of customer

	//=================== CONSIDER REVERSE ORDER ===========================// 
	float origain = dist[prevC][cStart] + dist[afterC][cEnd] - (dist[prevC][cEnd] + dist[afterC][cStart]); //only compare two ends cost, the intermediate all the same //old-new
	float *tempCumDist = new float[SIZE];
	float oldCumDist=0, newCumDist=0;
	tempCumDist[0]=CumDist[rand_r1][rand_r1_p-1];
	int u=1;//for tempCumDist
	for (int t = rand_r1_p+L1-1; t >= rand_r1_p; t--)//recalculate CumDist in reverse order
	{
		oldCumDist += CumDist[rand_r1][t];
		tempCumDist[u] = tempCumDist[u-1] + dist[prevC][VEHICLE[rand_r1][t]] + service_time[VEHICLE[rand_r1][t]];
		newCumDist += tempCumDist[u];
		prevC = VEHICLE[rand_r1][t];
		u++;
	}

	int remainingCust = saiz[rand_r1] - (rand_r1_p+L1-1);
	if (remainingCust <0)
		remainingCust =0;

	float gain = (oldCumDist - newCumDist) + remainingCust*origain;
				
	delete[] tempCumDist;


	RchangedStatus[rand_r1] = true;
	


	//******************************************************** END OF Neighbour_k 0 (shake_intraReverseSegment)  ******************************************************************************//

	if ( (prevC < 0) || (afterC < 0) || (cStart < 0) || (cEnd < 0))
	{
		cout<< "customer to be deleted cannot be negative!!!!!!" <<endl;
		cout<< "r1= " <<rand_r1 << endl;
		cout<< "r1_p= " <<rand_r1_p<<endl;
		cout<< "before_cust= "<< prevC << " firstEle = "<< cStart<<endl;
		cout<< "lastEle = "<< cEnd << " after_cust = "<< afterC<<endl;
	}
	
	
	cout<<"===========CUST from shake_intraReverseSegment=========" <<endl;
	cout<<"Length of segment = "<<L1<<endl;
	cout<< "route[" << rand_r1 <<"][" << rand_r1_p << "]=" <<cStart<<" route[" << rand_r1 <<"][" << rand_r1_p+L1-1 << "]=" <<cEnd<< endl;
	

	gainvec[0] = 1;//status yes
	gainvec[1] = gain;
	gainvec[2] = origain;

	reverseASegment (rand_r1, rand_r1_p, L1, VEHICLE);


	float total_cost = 0.0;

	cout<<"============ Routes after shake in shake_intraReverseSegment ============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		//float r_cost = 0.0;//to check
		//for (int c = 0; c <= saiz[i]; c++)
		//{
		//	r_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		//}
		//if (abs (r_cost-route_cost[i]) > 0.1)
		//{
		//	cout<<endl<<"After shake_2_2!!!!!!!!!"<<' ';
		//	cout<<"route_cost["<<i<<"]= "<<route_cost[i]<<' '<<"r_cost= "<<r_cost<<endl;
		//	cout<<rand_r1<<' '<<rand_r2<<' '<<rand_r1_p<<' '<<rand_r2_p<<' '<<gain1<<' '<<gain2<<endl;
		//	getchar();
		//}
		total_cost += route_cost[i]; 
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	
	return 1; //shake status is ok
}

int shakeCROSSbasedonNeighbourhood(int **(&VEHICLE), float *x, float *y, int neighbourhood) //shake_cross based on neighbourhood
{

	if (no_routes != LOCAL_NO_ROUTE)
	{
		cout<<"in shake_CROSS";
		getchar();
	}
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	//======================variable to be passed to function cross_insert
	int r1=-1, r2 = -1, r1_p = -1, r2_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete) to be pass to function cross_insert
	int reverseType = 0;//initially zero means no reverse for both segments
	float gain1, gain2=0.0;
	float distgain1=0, distgain2=0;
	//=================================//
	int rand_r1=-1, rand_r1_p = -1; //internal function use
	int cust_r1 = -1, cust_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0;


	//route_CGravity = new float*[no_routes];

	//for (int i = 0; i < no_routes; i++)
	//{
	//	route_CGravity[i] = new float[2]; //2 columns consists of x and y
	//}
	
	
	int averageCustperroute = (float)SIZE/(float)no_routes;
	float minPercentage = (float)neighbourhood/10 - 0.05;
	float maxPercentage = (float)neighbourhood/10 + 0.05;
	
	int minL = max(3, (int)ceil((float)averageCustperroute*minPercentage));
	int maxL = max(5, (int)ceil((float)averageCustperroute*maxPercentage));
	
	srand ( time(NULL) ); //seed it
	int L1 = (rand() % (maxL-minL+1))+minL;//length of segment1
	int L2 = (rand() % (maxL-minL+1))+minL;//length of segment2
	//int L1 = 3;//length of segment1
	//int L2 = 2;//length of segment2
	//cout<<"L1= "<<L1<<"L2= "<<L2<<endl;
	bool r1found = false;
	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}
	int no_flag=0;
	for (int i = 0; i < no_routes; i++)
	{
		//if (saiz[i] == 1)//made comment on 29Jun2015, it will be flagged in if (saiz[i] != 0)
		//{
		//	flag_module[vehic_depotCust[i][1]] = false;
		//	no_flag++;
		//}
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			
			for (int j = saiz[i]; j >= saiz[i]-L1+2 && j>0; j--)//flag the last L1-1 of customer because this can never be selected
			{
				flag_module[VEHICLE[i][j]] = false; //flag the last few cust in a route false because last customers cannot be selected//this is a trick!!!!!!!!!!!
				no_flag++;
			}
		}
	}
		//============= Guided shake, calculate center of gravity ============//put this before m2_regenerate, previously calculate center of gravity everytime generate a customer, OMGGGGGG!!!!!!!! on 29Jun2015
	//calculate_centreGravity(x, y);
	//for (int i = 0; i < no_routes; i++)
	//{
	//	CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
	//}
	//CGravityfile <<endl;
	//============= End of calculate center of gravity ============//
	

	float internal_start_s = clock();// the code you wish to time goes here
	
	while ((r1found == false) && (no_flag < SIZE)) //while r1 not found
	{		
		float timeElapsed = (clock() - internal_start_s) / float(CLOCKS_PER_SEC) ;
		if(timeElapsed < 15)//if less than 15seconds, select route by random
		{
		
		m8_regenerate:

			srand ( time(NULL) ); //seed it
			rand_r1 = (rand() % no_routes); // for deletion cannot delete from empty route (last route), NO_BEST_ROUTEs = 3 , [0][1][2], can only delete from [0][1]
			saiz_r1 = saiz[rand_r1];
		
			if(saiz[rand_r1]<L1)//have to test this before finding the position to avoid division by zero error
				goto m8_regenerate;

			rand_r1_p = (rand() % (saiz[rand_r1]-L1+1))+1; //position is from 1 to saiz-L1+1, +1 again at last because modulus produce from 0
			
			if (flag_module[VEHICLE[rand_r1][rand_r1_p]] == false)
				goto m8_regenerate;

			flag_module[VEHICLE[rand_r1][rand_r1_p]] = false;//flag this customer false as all possible r2 and position are evaluated
			no_flag++;
			std::vector<int> segment1; 
			segment1.clear();
			
			int segment1Start = VEHICLE[rand_r1][rand_r1_p];
			int segment1End = VEHICLE[rand_r1][rand_r1_p+L1-1];
			int segment1Size=L1;
			//copy segment1
			int r1demandDelete=0;
			//cout<<"00. This is ok"<<endl;
			for (int k = rand_r1_p; k <= rand_r1_p+L1-1; k++)
			{
				segment1.push_back(VEHICLE[rand_r1][k]);//push in segment element
				//segment1Reverse.push_back(VEHICLE[rand_r1][k]);//push in segment element
				r1demandDelete = r1demandDelete + demand[VEHICLE[rand_r1][k]];

			}
			//cout<<"01. This is ok"<<endl;
			//find second route
			for (int m = 0; m < no_routes; m++)//from route 0 to last route
			{
				if ((saiz[m] < L2 ) || (rand_r1==m))
					continue;
				for (int n = 1; n <= saiz[m]-L2+1; n++)//for the length of second route
				{
					std::vector<int> segment2; 
					gain1 = 0, gain2=0;//reinitialize
					distgain1 = 0, distgain2=0;//reinitialize
					segment2.clear();
					//segment2Reverse.clear();
					int segment2Start = VEHICLE[m][n];
					int segment2End = VEHICLE[m][n+L2-1];
					int segment2Size=L2;
					
					//copy segment2
					int r2demandDelete=0;
					for (int k = n; k <= n+L2-1; k++)
					{
						segment2.push_back(VEHICLE[m][k]);//push in segment element
						//segment2Reverse.push_back(VEHICLE[m][k]);//push in segment element
						r2demandDelete = r2demandDelete + demand[VEHICLE[m][k]];
					}
					//cout<<"011. This is ok"<<endl;

					int r1demand = total_demand[rand_r1] - r1demandDelete + r2demandDelete;
					int r2demand = total_demand[m] - r2demandDelete + r1demandDelete;

					//check capacity constraint
					if (r1demand> CAPACITY || r2demand> CAPACITY)
						continue;
					
					//if capacity ok, reverse the order, consider 4 types, R1&segmemt2, R1&segment2Reverse, R2&segment1, R2&segment1Reverse


					int seg1before = VEHICLE[rand_r1][rand_r1_p-1];
					int seg1after = VEHICLE[rand_r1][rand_r1_p+L1];
					
					int seg2before = VEHICLE[m][n-1];
					int seg2after = VEHICLE[m][n+L2];
					

					//calculate the dist delete segment1 from route i and dist insert segment2 to route m
					float oriCumDist1=0;
					float *tempCumDist2=new float[SIZE];//inserted to r2
					float *tempCumDist2Rev=new float[SIZE];//inserted to r2 in reverse order
					tempCumDist2[0] = CumDist[m][n-1];
					int t=1;//for tempCumDist2[]

					float deletedist1=0.0, insertdist2=0.0;
					int oribefore = seg1before;//this is original cust before j //for check condition
					int prevC = seg2before;
					
					float newCumDist2=0, newCumDist2Rev=0;

					for (int k = rand_r1_p; k <= rand_r1_p+L1-1; k++)
					{
						deletedist1 += dist[oribefore][VEHICLE[rand_r1][k]] + service_time[VEHICLE[rand_r1][k]];
						insertdist2 += dist[prevC][VEHICLE[rand_r1][k]] + service_time[VEHICLE[rand_r1][k]];

						oriCumDist1 += CumDist[rand_r1][k];
						tempCumDist2[t] = tempCumDist2[t-1]+dist[prevC][VEHICLE[rand_r1][k]] + service_time[VEHICLE[rand_r1][k]];
						newCumDist2 += tempCumDist2[t];

						oribefore = VEHICLE[rand_r1][k];
						prevC = VEHICLE[rand_r1][k];
						t++;
					
					}
					deletedist1 += dist[VEHICLE[rand_r1][rand_r1_p+L1-1]][seg1after];//last one in segment to the element after segment
					insertdist2 += dist[VEHICLE[rand_r1][rand_r1_p+L1-1]][seg2after];//last one in segment to the next element in segment2

					int remainingCust1 = saiz[rand_r1] - (rand_r1_p+L1-1);//saiz - lastpos
					if (remainingCust1 < 0)
						remainingCust1=0;
					

					//calculate the cost delete segment2 from route m, and inserting segment2 to route i
					float oriCumDist2=0;
					float *tempCumDist1=new float[SIZE];//inserted to r1
					float *tempCumDist1Rev=new float[SIZE];//inserted to r1 in reverse order

					//find cost_of_removing2 and cost_of_inserting1
					float newCumDist1=0, newCumDist1Rev=0;
					tempCumDist1[0] = CumDist[rand_r1][rand_r1_p-1];
					int w=1;//for tempCumDist1[]
					prevC = seg1before;
					oribefore = seg2before;//this is original cust before j //for check condition
					float deletedist2=0, insertdist1=0;//for check condition;//for check condition
					for (int k = n; k <= n+L2-1; k++)
					{
						deletedist2 += dist[oribefore][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];//for check condition
						insertdist1 += dist[prevC][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];//for check condition

						oriCumDist2 += CumDist[m][k];
						tempCumDist1[w] = tempCumDist1[w-1]+dist[prevC][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];
						newCumDist1 += tempCumDist1[w];

						prevC=VEHICLE[m][k];
						oribefore=VEHICLE[m][k];
						w++;

					}
					deletedist2 += dist[VEHICLE[m][n+L2-1]][seg2after];//last one in segment to the element after segment
					insertdist1 += dist[VEHICLE[m][n+L2-1]][seg1after];//last one in segment to the next element in segment1
					
					int remainingCust2 = saiz[m] - (n+L2-1);//saiz - lastpos
					if (remainingCust2 < 0)
						remainingCust2=0;

					
					//=============================== reverse the order (added this on 7JUly2015
					
					//find newCumDist2Rev
					tempCumDist2Rev[0] = CumDist[m][n-1];
		
					t=1;//for tempCumDist2Rev[]
					float insertdist2Rev=0.0;//for check condition;//r2 with segment2
					prevC = seg2before;
					
					//cout<<"1. This is ok"<<endl;
					for (int k = segment1Size-1; k >= 0; k--)
					{
						//segment1Reverse.push_back(segment1[k]);//push in reverse order, so noneed to use reverse function

						insertdist2Rev += dist[prevC][segment1[k]] + service_time[segment1[k]];
						tempCumDist2Rev[t] = tempCumDist2Rev[t-1]+dist[prevC][segment1[k]] + service_time[segment1[k]];
						newCumDist2Rev += tempCumDist2Rev[t];
						prevC=segment1[k];
						t++;
					}
					insertdist2Rev += dist[segment1[0]][seg2after]; //for check condition
					//cout<<"2. This is ok"<<endl;
					//find newCumDist1Rev

					tempCumDist1Rev[0] = CumDist[rand_r1][rand_r1_p-1];
					w=1;//for tempCumDist1Rev[]
					prevC = seg1before;
					float insertdist1Rev=0;//for check condition;//r1 with tail1
					//cout<<"3. This is ok"<<endl;
					for (int k = segment2Size-1; k >= 0; k--)
					{
						//segment2Reverse.push_back(segment2[k]);//push in reverse order, so noneed to use reverse function
						insertdist1Rev += dist[prevC][segment2[k]] + service_time[segment2[k]];
						tempCumDist1Rev[w] = tempCumDist1Rev[w-1]+dist[prevC][segment2[k]] + service_time[segment2[k]];
						newCumDist1Rev += tempCumDist1Rev[w];
						prevC=segment2[k];
						w++;
					}
					insertdist1Rev += dist[segment2[0]][seg1after];//last arc //for check condition
					//cout<<"4. This is ok"<<endl;
					//=========================================================//

					reverseType = 0;//initialize to zero which means no reverse

					bool seg1Reverse = false, seg2Reverse = false;
					if (newCumDist1Rev < newCumDist1)
					{
						newCumDist1 = newCumDist1Rev;
						insertdist1 = insertdist1Rev;//for check condition
						seg2Reverse = true; //if segment2 reverse, reverseType = 2
						reverseType = 2;
						
					}

					if (newCumDist2Rev < newCumDist2)
					{
						newCumDist2 = newCumDist2Rev;
						insertdist2 = insertdist2Rev;//for check condition
						seg1Reverse = true;//if segment1 reverse, reverseType = 1
						reverseType = 1;
						//tail1ptr = &tail1Rev;
					}

					if (seg1Reverse == true && seg2Reverse == true)//if both segment are reverse, type3
						reverseType = 3;

					delete[] tempCumDist1;
					delete[] tempCumDist1Rev;
					delete[] tempCumDist2;
					delete[] tempCumDist2Rev;
					segment2.clear();
					segment2.shrink_to_fit();

					if ((insertdist1-deletedist1 > DISTANCE+epsilon) || (insertdist2-deletedist2 > DISTANCE+epsilon))
						continue;
			
					gain1 = (oriCumDist1 - newCumDist1) + remainingCust1*(deletedist1-insertdist1);
					gain2 = (oriCumDist2 - newCumDist2) + remainingCust2*(deletedist2-insertdist2);

					distgain1 = (deletedist1-insertdist1);
					distgain2 = (deletedist2-insertdist2);



					
					r1found = true;	
					r1 = rand_r1;
					r1_p = rand_r1_p;
					r2 = m;
					r2_p = n;
					goto foundshake;
				}//end of n
			}//end of m	
			segment1.clear();
			segment1.shrink_to_fit();
		}//end if less than 15seconds, select route by random
		//======================================= if more than 15 seconds ================================//
		else //if more than 15seconds, select route one by one
		{
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i]< L1)//not enough to be selected, this including empty route
					continue;
				
				for (int j = 1; j <= saiz[i]-L1+1; j++) //starting from the first point, consider a segment of size L1
				{
					if (flag_module[VEHICLE[i][j]] == false)//if it has been considered, just skip it
						continue;
					flag_module[VEHICLE[i][j]] = false;//flag this customer false as all possible r2 and position are evaluated
					no_flag++;
					std::vector<int> segment1; 
					segment1.clear();
					//segment1Reverse.clear();
					int segment1Start = VEHICLE[i][j];
					int segment1End = VEHICLE[i][j+L1-1];
					int segment1Size=L1;
					//copy segment1
					int r1demandDelete=0;
					for (int k = j; k <= j+L1-1; k++)
					{
						segment1.push_back(VEHICLE[i][k]);//push in segment element
						//segment1Reverse.push_back(VEHICLE[i][k]);//push in segment element
						r1demandDelete = r1demandDelete + demand[VEHICLE[i][k]];
					}

					//find second route
					for (int m = 0; m < no_routes; m++)//from route 0 to last route
					{
						if ((saiz[m] < L2 ) || (i==m))
							continue;
						for (int n = 1; n <= saiz[m]-L2+1; n++)//for the length of second route
						{
							gain1 = 0, gain2=0;//reinitialize
					distgain1 = 0, distgain2=0;//reinitialize
					std::vector<int> segment2; 
					segment2.clear();
					//segment2Reverse.clear();
					int segment2Start = VEHICLE[m][n];
					int segment2End = VEHICLE[m][n+L2-1];
					int segment2Size=L2;
					
					//copy segment2
					int r2demandDelete=0;
					for (int k = n; k <= n+L2-1; k++)
					{
						segment2.push_back(VEHICLE[m][k]);//push in segment element
						//segment2Reverse.push_back(VEHICLE[m][k]);//push in segment element
						r2demandDelete = r2demandDelete + demand[VEHICLE[m][k]];
					}
					//cout<<"011. This is ok"<<endl;

					int r1demand = total_demand[i] - r1demandDelete + r2demandDelete;
					int r2demand = total_demand[m] - r2demandDelete + r1demandDelete;

					//check capacity constraint
					if (r1demand> CAPACITY || r2demand> CAPACITY)
						continue;
					
					//if capacity ok, reverse the order, consider 4 types, R1&segmemt2, R1&segment2Reverse, R2&segment1, R2&segment1Reverse


					int seg1before = VEHICLE[i][j-1];
					int seg1after = VEHICLE[i][j+L1];
					
					int seg2before = VEHICLE[m][n-1];
					int seg2after = VEHICLE[m][n+L2];
					

					//calculate the dist delete segment1 from route i and dist insert segment2 to route m
					float oriCumDist1=0;
					float *tempCumDist2=new float[SIZE];//inserted to r2
					float *tempCumDist2Rev=new float[SIZE];//inserted to r2 in reverse order
					tempCumDist2[0] = CumDist[m][n-1];
					int t=1;//for tempCumDist2[]

					float deletedist1=0.0, insertdist2=0.0;
					int oribefore = seg1before;//this is original cust before j //for check condition
					int prevC = seg2before;
					
					float newCumDist2=0, newCumDist2Rev=0;

					for (int k = j; k <= j+L1-1; k++)
					{
						deletedist1 += dist[oribefore][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];
						insertdist2 += dist[prevC][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];

						oriCumDist1 += CumDist[i][k];
						tempCumDist2[t] = tempCumDist2[t-1]+dist[prevC][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];
						newCumDist2 += tempCumDist2[t];

						oribefore = VEHICLE[i][k];
						prevC = VEHICLE[i][k];
						t++;
					
					}
					deletedist1 += dist[VEHICLE[i][j+L1-1]][seg1after];//last one in segment to the element after segment
					insertdist2 += dist[VEHICLE[i][j+L1-1]][seg2after];//last one in segment to the next element in segment2

					int remainingCust1 = saiz[i] - (j+L1-1);//saiz - lastpos
					if (remainingCust1 < 0)
						remainingCust1=0;
					

					//calculate the cost delete segment2 from route m, and inserting segment2 to route i
					float oriCumDist2=0;
					float *tempCumDist1=new float[SIZE];//inserted to r1
					float *tempCumDist1Rev=new float[SIZE];//inserted to r1 in reverse order

					//find cost_of_removing2 and cost_of_inserting1
					float newCumDist1=0, newCumDist1Rev=0;
					tempCumDist1[0] = CumDist[i][j-1];
					int w=1;//for tempCumDist1[]
					prevC = seg1before;
					oribefore = seg2before;//this is original cust before j //for check condition
					float deletedist2=0, insertdist1=0;//for check condition;//for check condition
					for (int k = n; k <= n+L2-1; k++)
					{
						deletedist2 += dist[oribefore][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];//for check condition
						insertdist1 += dist[prevC][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];//for check condition

						oriCumDist2 += CumDist[m][k];
						tempCumDist1[w] = tempCumDist1[w-1]+dist[prevC][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];
						newCumDist1 += tempCumDist1[w];

						prevC=VEHICLE[m][k];
						oribefore=VEHICLE[m][k];
						w++;

					}
					deletedist2 += dist[VEHICLE[m][n+L2-1]][seg2after];//last one in segment to the element after segment
					insertdist1 += dist[VEHICLE[m][n+L2-1]][seg1after];//last one in segment to the next element in segment1
					
					int remainingCust2 = saiz[m] - (n+L2-1);//saiz - lastpos
					if (remainingCust2 < 0)
						remainingCust2=0;

					
					//=============================== reverse the order (added this on 7JUly2015
					
					//find newCumDist2Rev
					tempCumDist2Rev[0] = CumDist[m][n-1];
		
					t=1;//for tempCumDist2Rev[]
					float insertdist2Rev=0.0;//for check condition;//r2 with segment2
					prevC = seg2before;
					
					//cout<<"1. This is ok"<<endl;
					for (int k = segment1Size-1; k >= 0; k--)
					{
						//segment1Reverse.push_back(segment1[k]);//push in reverse order, so noneed to use reverse function

						insertdist2Rev += dist[prevC][segment1[k]] + service_time[segment1[k]];
						tempCumDist2Rev[t] = tempCumDist2Rev[t-1]+dist[prevC][segment1[k]] + service_time[segment1[k]];
						newCumDist2Rev += tempCumDist2Rev[t];
						prevC=segment1[k];
						t++;
					}
						insertdist2Rev += dist[segment1[0]][seg2after]; //for check condition
						//cout<<"2. This is ok"<<endl;
						//find newCumDist1Rev
						tempCumDist1Rev[0] = CumDist[i][j-1];
						w=1;//for tempCumDist1Rev[]
						prevC = seg1before;
						float insertdist1Rev=0;//for check condition;//r1 with tail1
						//cout<<"3. This is ok"<<endl;
						for (int k = segment2Size-1; k >= 0; k--)
						{
							//segment2Reverse.push_back(segment2[k]);//push in reverse order, so noneed to use reverse function
							insertdist1Rev += dist[prevC][segment2[k]] + service_time[segment2[k]];
							tempCumDist1Rev[w] = tempCumDist1Rev[w-1]+dist[prevC][segment2[k]] + service_time[segment2[k]];
							newCumDist1Rev += tempCumDist1Rev[w];
							prevC=segment2[k];
							w++;
						}
						insertdist1Rev += dist[segment2[0]][seg1after];//last arc //for check condition
						//cout<<"4. This is ok"<<endl;
						//=========================================================//

						reverseType = 0;//initialize to zero which means no reverse

						bool seg1Reverse = false, seg2Reverse = false;
						if (newCumDist1Rev < newCumDist1)
						{
							newCumDist1 = newCumDist1Rev;
							insertdist1 = insertdist1Rev;//for check condition
							seg2Reverse = true; //if segment2 reverse, reverseType = 2
							reverseType = 2;
						
						}

						if (newCumDist2Rev < newCumDist2)
						{
							newCumDist2 = newCumDist2Rev;
							insertdist2 = insertdist2Rev;//for check condition
							seg1Reverse = true;//if segment1 reverse, reverseType = 1
							reverseType = 1;
							//tail1ptr = &tail1Rev;
						}

						if (seg1Reverse == true && seg2Reverse == true)//if both segment are reverse, type3
							reverseType = 3;

						delete[] tempCumDist1;
						delete[] tempCumDist1Rev;
						delete[] tempCumDist2;
						delete[] tempCumDist2Rev;
						segment2.clear();
						segment2.shrink_to_fit();

						if ((insertdist1-deletedist1 > DISTANCE+epsilon) || (insertdist2-deletedist2 > DISTANCE+epsilon))
							continue;
			
						gain1 = (oriCumDist1 - newCumDist1) + remainingCust1*(deletedist1-insertdist1);
						gain2 = (oriCumDist2 - newCumDist2) + remainingCust2*(deletedist2-insertdist2);

						distgain1 = (deletedist1-insertdist1);
						distgain2 = (deletedist2-insertdist2);

					

						

							r1found = true;	
							r1 = i;
							r1_p = j;
							r2 = m;
							r2_p = n;
							goto foundshake;
						}//end of n
					}//end of m
					segment1.clear();
					segment1.shrink_to_fit();
				}//end of for j
			}
		}//end if more than 15seconds, select route one by one
	}//end while (r1found == false)  //while r1 not found
	if (no_flag >= SIZE)
	{	
		cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE cross!" <<endl;
		for (int i = 0; i < SIZE; i++)
		{
			cout<<flag_module[i]<<' ';
		}
		//getchar();
		return 0;
	}
foundshake:;
	RchangedStatus[r1] = true;//route 1
	RchangedStatus[r2] = true;//route 2
	
	//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
	//int delete1=0;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5	
	//for (int q = -1; q <= delete1; q++)//flag the one before and current
	//{
	//	CustaffectedStatus[VEHICLE[r1][r1_p+q]] = true;//flagged customer affected true
	//}
	//for (int q = -1; q <= delete1; q++)//flag the one before and current
	//{
	//	CustaffectedStatus[VEHICLE[r2][r2_p+q]] = true;//flagged customer affected true
	//}
	//int last1=1;
	//for (int q = 0; q <= last1; q++)//flag the current last and one after for the end point of L1
	//{
	//	CustaffectedStatus[VEHICLE[r1][r1_p+L1-1+q]] = true;//flagged customer affected true
	//}
	//for (int q = 0; q <= last1; q++)//flag the current last and one after for the end point of L2
	//{
	//	CustaffectedStatus[VEHICLE[r2][r2_p+L2-1+q]] = true;//flagged customer affected true
	//}

	cout<<"From cross-shaking, r1="<<r1<<' '<<r2<<endl;
	gainvec[0] = 1;//status yes
	gainvec[1] = gain1;
	gainvec[2] = distgain1;
	gainvec[3] = gain2;
	gainvec[4] = distgain2;
	cross_insert_fromshaking (L1, L2, r1, r2, r1_p, r2_p, VEHICLE, reverseType);
	float total_cost = 0.0;

	cout<<"============ Routes after shake in CROSS ============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		//float r_cost = 0.0;//to check
		//for (int c = 0; c <= saiz[i]; c++)
		//{
		//	r_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		//}
		//if (abs (r_cost-route_cost[i]) > 0.1)
		//{
		//	cout<<endl<<"After shake cross!!!!!!!!!"<<' ';
		//	cout<<"route_cost["<<i<<"]= "<<route_cost[i]<<' '<<"r_cost= "<<r_cost<<endl;
		//	cout<<r1<<' '<<r2<<' '<<r1_p<<' '<<r2_p<<' '<<gain<<endl;
		//	getchar();
		//}
		total_cost += route_cost[i]; 
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	
	delete[] flag_module;
	return 1; //shake status is ok
}

//DONE
int shakeCROSS(int **(&VEHICLE), float *x, float *y) 
{

	if (no_routes != LOCAL_NO_ROUTE)
	{
		cout<<"in shake_CROSS";
		getchar();
	}
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	//======================variable to be passed to function cross_insert
	int r1=-1, r2 = -1, r1_p = -1, r2_p = -1; //route1, route2, route1_p (to delete), route2_p(to delete) to be pass to function cross_insert
	int reverseType = 0;//initially zero means no reverse for both segments
	float gain1, gain2=0.0;
	float distgain1=0, distgain2=0;
	//=================================//
	int rand_r1=-1, rand_r1_p = -1; //internal function use
	int cust_r1 = -1, cust_r2 = -1;
	int saiz_r1 = 0, saiz_r2 = 0;


	//route_CGravity = new float*[no_routes];

	//for (int i = 0; i < no_routes; i++)
	//{
	//	route_CGravity[i] = new float[2]; //2 columns consists of x and y
	//}
	int maxL = 5;
	int minL = 3;
	srand ( time(NULL) ); //seed it
	int L1 = (rand() % (maxL-minL+1))+minL;//length of segment1
	int L2 = (rand() % (maxL-minL+1))+minL;//length of segment2
	//int L1 = 3;//length of segment1
	//int L2 = 2;//length of segment2
	//cout<<"L1= "<<L1<<"L2= "<<L2<<endl;
	bool r1found = false;
	bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0
	for (int i = 0; i < SIZE; i++)
	{
		flag_module[i] = true; //reinitialize to 1 
	}
	int no_flag=0;
	for (int i = 0; i < no_routes; i++)
	{
		//if (saiz[i] == 1)//made comment on 29Jun2015, it will be flagged in if (saiz[i] != 0)
		//{
		//	flag_module[vehic_depotCust[i][1]] = false;
		//	no_flag++;
		//}
		if (saiz[i] != 0)  //if not empty route, otherwise no_flag is added one even though for empty route 
		{
			
			for (int j = saiz[i]; j >= saiz[i]-L1+2 && j>0; j--)//flag the last L1-1 of customer because this can never be selected
			{
				flag_module[VEHICLE[i][j]] = false; //flag the last few cust in a route false because last customers cannot be selected//this is a trick!!!!!!!!!!!
				no_flag++;
			}
		}
	}
		//============= Guided shake, calculate center of gravity ============//put this before m2_regenerate, previously calculate center of gravity everytime generate a customer, OMGGGGGG!!!!!!!! on 29Jun2015
	//calculate_centreGravity(x, y);
	//for (int i = 0; i < no_routes; i++)
	//{
	//	CGravityfile << route_CGravity[i][0]<< ' ' <<' '<<route_CGravity[i][1]<<endl;
	//}
	//CGravityfile <<endl;
	//============= End of calculate center of gravity ============//
	

	float internal_start_s = clock();// the code you wish to time goes here
	
	while ((r1found == false) && (no_flag < SIZE)) //while r1 not found
	{		
		float timeElapsed = (clock() - internal_start_s) / float(CLOCKS_PER_SEC) ;
		if(timeElapsed < 15)//if less than 15seconds, select route by random
		{
		
		m8_regenerate:

			srand ( time(NULL) ); //seed it
			rand_r1 = (rand() % no_routes); // for deletion cannot delete from empty route (last route), NO_BEST_ROUTEs = 3 , [0][1][2], can only delete from [0][1]
			saiz_r1 = saiz[rand_r1];
		
			if(saiz[rand_r1]<L1)//have to test this before finding the position to avoid division by zero error
				goto m8_regenerate;

			rand_r1_p = (rand() % (saiz[rand_r1]-L1+1))+1; //position is from 1 to saiz-L1+1, +1 again at last because modulus produce from 0
			
			if (flag_module[VEHICLE[rand_r1][rand_r1_p]] == false)
				goto m8_regenerate;

			flag_module[VEHICLE[rand_r1][rand_r1_p]] = false;//flag this customer false as all possible r2 and position are evaluated
			no_flag++;
			std::vector<int> segment1; 
			segment1.clear();
			
			int segment1Start = VEHICLE[rand_r1][rand_r1_p];
			int segment1End = VEHICLE[rand_r1][rand_r1_p+L1-1];
			int segment1Size=L1;
			//copy segment1
			int r1demandDelete=0;
			//cout<<"00. This is ok"<<endl;
			for (int k = rand_r1_p; k <= rand_r1_p+L1-1; k++)
			{
				segment1.push_back(VEHICLE[rand_r1][k]);//push in segment element
				//segment1Reverse.push_back(VEHICLE[rand_r1][k]);//push in segment element
				r1demandDelete = r1demandDelete + demand[VEHICLE[rand_r1][k]];

			}
			//cout<<"01. This is ok"<<endl;
			//find second route
			for (int m = 0; m < no_routes; m++)//from route 0 to last route
			{
				if ((saiz[m] < L2 ) || (rand_r1==m))
					continue;
				for (int n = 1; n <= saiz[m]-L2+1; n++)//for the length of second route
				{
					std::vector<int> segment2; 
					gain1 = 0, gain2=0;//reinitialize
					distgain1 = 0, distgain2=0;//reinitialize
					segment2.clear();
					//segment2Reverse.clear();
					int segment2Start = VEHICLE[m][n];
					int segment2End = VEHICLE[m][n+L2-1];
					int segment2Size=L2;
					
					//copy segment2
					int r2demandDelete=0;
					for (int k = n; k <= n+L2-1; k++)
					{
						segment2.push_back(VEHICLE[m][k]);//push in segment element
						//segment2Reverse.push_back(VEHICLE[m][k]);//push in segment element
						r2demandDelete = r2demandDelete + demand[VEHICLE[m][k]];
					}
					//cout<<"011. This is ok"<<endl;

					int r1demand = total_demand[rand_r1] - r1demandDelete + r2demandDelete;
					int r2demand = total_demand[m] - r2demandDelete + r1demandDelete;

					//check capacity constraint
					if (r1demand> CAPACITY || r2demand> CAPACITY)
						continue;
					
					//if capacity ok, reverse the order, consider 4 types, R1&segmemt2, R1&segment2Reverse, R2&segment1, R2&segment1Reverse


					int seg1before = VEHICLE[rand_r1][rand_r1_p-1];
					int seg1after = VEHICLE[rand_r1][rand_r1_p+L1];
					
					int seg2before = VEHICLE[m][n-1];
					int seg2after = VEHICLE[m][n+L2];
					

					//calculate the dist delete segment1 from route i and dist insert segment2 to route m
					float oriCumDist1=0;
					float *tempCumDist2=new float[SIZE];//inserted to r2
					float *tempCumDist2Rev=new float[SIZE];//inserted to r2 in reverse order
					tempCumDist2[0] = CumDist[m][n-1];
					int t=1;//for tempCumDist2[]

					float deletedist1=0.0, insertdist2=0.0;
					int oribefore = seg1before;//this is original cust before j //for check condition
					int prevC = seg2before;
					
					float newCumDist2=0, newCumDist2Rev=0;

					for (int k = rand_r1_p; k <= rand_r1_p+L1-1; k++)
					{
						deletedist1 += dist[oribefore][VEHICLE[rand_r1][k]] + service_time[VEHICLE[rand_r1][k]];
						insertdist2 += dist[prevC][VEHICLE[rand_r1][k]] + service_time[VEHICLE[rand_r1][k]];

						oriCumDist1 += CumDist[rand_r1][k];
						tempCumDist2[t] = tempCumDist2[t-1]+dist[prevC][VEHICLE[rand_r1][k]] + service_time[VEHICLE[rand_r1][k]];
						newCumDist2 += tempCumDist2[t];

						oribefore = VEHICLE[rand_r1][k];
						prevC = VEHICLE[rand_r1][k];
						t++;
					
					}
					deletedist1 += dist[VEHICLE[rand_r1][rand_r1_p+L1-1]][seg1after];//last one in segment to the element after segment
					insertdist2 += dist[VEHICLE[rand_r1][rand_r1_p+L1-1]][seg2after];//last one in segment to the next element in segment2

					int remainingCust1 = saiz[rand_r1] - (rand_r1_p+L1-1);//saiz - lastpos
					if (remainingCust1 < 0)
						remainingCust1=0;
					

					//calculate the cost delete segment2 from route m, and inserting segment2 to route i
					float oriCumDist2=0;
					float *tempCumDist1=new float[SIZE];//inserted to r1
					float *tempCumDist1Rev=new float[SIZE];//inserted to r1 in reverse order

					//find cost_of_removing2 and cost_of_inserting1
					float newCumDist1=0, newCumDist1Rev=0;
					tempCumDist1[0] = CumDist[rand_r1][rand_r1_p-1];
					int w=1;//for tempCumDist1[]
					prevC = seg1before;
					oribefore = seg2before;//this is original cust before j //for check condition
					float deletedist2=0, insertdist1=0;//for check condition;//for check condition
					for (int k = n; k <= n+L2-1; k++)
					{
						deletedist2 += dist[oribefore][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];//for check condition
						insertdist1 += dist[prevC][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];//for check condition

						oriCumDist2 += CumDist[m][k];
						tempCumDist1[w] = tempCumDist1[w-1]+dist[prevC][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];
						newCumDist1 += tempCumDist1[w];

						prevC=VEHICLE[m][k];
						oribefore=VEHICLE[m][k];
						w++;

					}
					deletedist2 += dist[VEHICLE[m][n+L2-1]][seg2after];//last one in segment to the element after segment
					insertdist1 += dist[VEHICLE[m][n+L2-1]][seg1after];//last one in segment to the next element in segment1
					
					int remainingCust2 = saiz[m] - (n+L2-1);//saiz - lastpos
					if (remainingCust2 < 0)
						remainingCust2=0;

					
					//=============================== reverse the order (added this on 7JUly2015
					
					//find newCumDist2Rev
					tempCumDist2Rev[0] = CumDist[m][n-1];
		
					t=1;//for tempCumDist2Rev[]
					float insertdist2Rev=0.0;//for check condition;//r2 with segment2
					prevC = seg2before;
					
					//cout<<"1. This is ok"<<endl;
					for (int k = segment1Size-1; k >= 0; k--)
					{
						//segment1Reverse.push_back(segment1[k]);//push in reverse order, so noneed to use reverse function

						insertdist2Rev += dist[prevC][segment1[k]] + service_time[segment1[k]];
						tempCumDist2Rev[t] = tempCumDist2Rev[t-1]+dist[prevC][segment1[k]] + service_time[segment1[k]];
						newCumDist2Rev += tempCumDist2Rev[t];
						prevC=segment1[k];
						t++;
					}
					insertdist2Rev += dist[segment1[0]][seg2after]; //for check condition
					//cout<<"2. This is ok"<<endl;
					//find newCumDist1Rev

					tempCumDist1Rev[0] = CumDist[rand_r1][rand_r1_p-1];
					w=1;//for tempCumDist1Rev[]
					prevC = seg1before;
					float insertdist1Rev=0;//for check condition;//r1 with tail1
					//cout<<"3. This is ok"<<endl;
					for (int k = segment2Size-1; k >= 0; k--)
					{
						//segment2Reverse.push_back(segment2[k]);//push in reverse order, so noneed to use reverse function
						insertdist1Rev += dist[prevC][segment2[k]] + service_time[segment2[k]];
						tempCumDist1Rev[w] = tempCumDist1Rev[w-1]+dist[prevC][segment2[k]] + service_time[segment2[k]];
						newCumDist1Rev += tempCumDist1Rev[w];
						prevC=segment2[k];
						w++;
					}
					insertdist1Rev += dist[segment2[0]][seg1after];//last arc //for check condition
					//cout<<"4. This is ok"<<endl;
					//=========================================================//

					reverseType = 0;//initialize to zero which means no reverse

					bool seg1Reverse = false, seg2Reverse = false;
					if (newCumDist1Rev < newCumDist1)
					{
						newCumDist1 = newCumDist1Rev;
						insertdist1 = insertdist1Rev;//for check condition
						seg2Reverse = true; //if segment2 reverse, reverseType = 2
						reverseType = 2;
						
					}

					if (newCumDist2Rev < newCumDist2)
					{
						newCumDist2 = newCumDist2Rev;
						insertdist2 = insertdist2Rev;//for check condition
						seg1Reverse = true;//if segment1 reverse, reverseType = 1
						reverseType = 1;
						//tail1ptr = &tail1Rev;
					}

					if (seg1Reverse == true && seg2Reverse == true)//if both segment are reverse, type3
						reverseType = 3;

					delete[] tempCumDist1;
					delete[] tempCumDist1Rev;
					delete[] tempCumDist2;
					delete[] tempCumDist2Rev;
					segment2.clear();
					segment2.shrink_to_fit();

					if ((insertdist1-deletedist1 > DISTANCE+epsilon) || (insertdist2-deletedist2 > DISTANCE+epsilon))
						continue;
			
					gain1 = (oriCumDist1 - newCumDist1) + remainingCust1*(deletedist1-insertdist1);
					gain2 = (oriCumDist2 - newCumDist2) + remainingCust2*(deletedist2-insertdist2);

					distgain1 = (deletedist1-insertdist1);
					distgain2 = (deletedist2-insertdist2);



					
					r1found = true;	
					r1 = rand_r1;
					r1_p = rand_r1_p;
					r2 = m;
					r2_p = n;
					goto foundshake;
				}//end of n
			}//end of m	
			segment1.clear();
			segment1.shrink_to_fit();
		}//end if less than 15seconds, select route by random
		//======================================= if more than 15 seconds ================================//
		else //if more than 15seconds, select route one by one
		{
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i]< L1)//not enough to be selected, this including empty route
					continue;
				
				for (int j = 1; j <= saiz[i]-L1+1; j++) //starting from the first point, consider a segment of size L1
				{
					if (flag_module[VEHICLE[i][j]] == false)//if it has been considered, just skip it
						continue;
					flag_module[VEHICLE[i][j]] = false;//flag this customer false as all possible r2 and position are evaluated
					no_flag++;
					std::vector<int> segment1; 
					segment1.clear();
					//segment1Reverse.clear();
					int segment1Start = VEHICLE[i][j];
					int segment1End = VEHICLE[i][j+L1-1];
					int segment1Size=L1;
					//copy segment1
					int r1demandDelete=0;
					for (int k = j; k <= j+L1-1; k++)
					{
						segment1.push_back(VEHICLE[i][k]);//push in segment element
						//segment1Reverse.push_back(VEHICLE[i][k]);//push in segment element
						r1demandDelete = r1demandDelete + demand[VEHICLE[i][k]];
					}

					//find second route
					for (int m = 0; m < no_routes; m++)//from route 0 to last route
					{
						if ((saiz[m] < L2 ) || (i==m))
							continue;
						for (int n = 1; n <= saiz[m]-L2+1; n++)//for the length of second route
						{
							gain1 = 0, gain2=0;//reinitialize
					distgain1 = 0, distgain2=0;//reinitialize
					std::vector<int> segment2; 
					segment2.clear();
					//segment2Reverse.clear();
					int segment2Start = VEHICLE[m][n];
					int segment2End = VEHICLE[m][n+L2-1];
					int segment2Size=L2;
					
					//copy segment2
					int r2demandDelete=0;
					for (int k = n; k <= n+L2-1; k++)
					{
						segment2.push_back(VEHICLE[m][k]);//push in segment element
						//segment2Reverse.push_back(VEHICLE[m][k]);//push in segment element
						r2demandDelete = r2demandDelete + demand[VEHICLE[m][k]];
					}
					//cout<<"011. This is ok"<<endl;

					int r1demand = total_demand[i] - r1demandDelete + r2demandDelete;
					int r2demand = total_demand[m] - r2demandDelete + r1demandDelete;

					//check capacity constraint
					if (r1demand> CAPACITY || r2demand> CAPACITY)
						continue;
					
					//if capacity ok, reverse the order, consider 4 types, R1&segmemt2, R1&segment2Reverse, R2&segment1, R2&segment1Reverse


					int seg1before = VEHICLE[i][j-1];
					int seg1after = VEHICLE[i][j+L1];
					
					int seg2before = VEHICLE[m][n-1];
					int seg2after = VEHICLE[m][n+L2];
					

					//calculate the dist delete segment1 from route i and dist insert segment2 to route m
					float oriCumDist1=0;
					float *tempCumDist2=new float[SIZE];//inserted to r2
					float *tempCumDist2Rev=new float[SIZE];//inserted to r2 in reverse order
					tempCumDist2[0] = CumDist[m][n-1];
					int t=1;//for tempCumDist2[]

					float deletedist1=0.0, insertdist2=0.0;
					int oribefore = seg1before;//this is original cust before j //for check condition
					int prevC = seg2before;
					
					float newCumDist2=0, newCumDist2Rev=0;

					for (int k = j; k <= j+L1-1; k++)
					{
						deletedist1 += dist[oribefore][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];
						insertdist2 += dist[prevC][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];

						oriCumDist1 += CumDist[i][k];
						tempCumDist2[t] = tempCumDist2[t-1]+dist[prevC][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];
						newCumDist2 += tempCumDist2[t];

						oribefore = VEHICLE[i][k];
						prevC = VEHICLE[i][k];
						t++;
					
					}
					deletedist1 += dist[VEHICLE[i][j+L1-1]][seg1after];//last one in segment to the element after segment
					insertdist2 += dist[VEHICLE[i][j+L1-1]][seg2after];//last one in segment to the next element in segment2

					int remainingCust1 = saiz[i] - (j+L1-1);//saiz - lastpos
					if (remainingCust1 < 0)
						remainingCust1=0;
					

					//calculate the cost delete segment2 from route m, and inserting segment2 to route i
					float oriCumDist2=0;
					float *tempCumDist1=new float[SIZE];//inserted to r1
					float *tempCumDist1Rev=new float[SIZE];//inserted to r1 in reverse order

					//find cost_of_removing2 and cost_of_inserting1
					float newCumDist1=0, newCumDist1Rev=0;
					tempCumDist1[0] = CumDist[i][j-1];
					int w=1;//for tempCumDist1[]
					prevC = seg1before;
					oribefore = seg2before;//this is original cust before j //for check condition
					float deletedist2=0, insertdist1=0;//for check condition;//for check condition
					for (int k = n; k <= n+L2-1; k++)
					{
						deletedist2 += dist[oribefore][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];//for check condition
						insertdist1 += dist[prevC][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];//for check condition

						oriCumDist2 += CumDist[m][k];
						tempCumDist1[w] = tempCumDist1[w-1]+dist[prevC][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];
						newCumDist1 += tempCumDist1[w];

						prevC=VEHICLE[m][k];
						oribefore=VEHICLE[m][k];
						w++;

					}
					deletedist2 += dist[VEHICLE[m][n+L2-1]][seg2after];//last one in segment to the element after segment
					insertdist1 += dist[VEHICLE[m][n+L2-1]][seg1after];//last one in segment to the next element in segment1
					
					int remainingCust2 = saiz[m] - (n+L2-1);//saiz - lastpos
					if (remainingCust2 < 0)
						remainingCust2=0;

					
					//=============================== reverse the order (added this on 7JUly2015
					
					//find newCumDist2Rev
					tempCumDist2Rev[0] = CumDist[m][n-1];
		
					t=1;//for tempCumDist2Rev[]
					float insertdist2Rev=0.0;//for check condition;//r2 with segment2
					prevC = seg2before;
					
					//cout<<"1. This is ok"<<endl;
					for (int k = segment1Size-1; k >= 0; k--)
					{
						//segment1Reverse.push_back(segment1[k]);//push in reverse order, so noneed to use reverse function

						insertdist2Rev += dist[prevC][segment1[k]] + service_time[segment1[k]];
						tempCumDist2Rev[t] = tempCumDist2Rev[t-1]+dist[prevC][segment1[k]] + service_time[segment1[k]];
						newCumDist2Rev += tempCumDist2Rev[t];
						prevC=segment1[k];
						t++;
					}
						insertdist2Rev += dist[segment1[0]][seg2after]; //for check condition
						//cout<<"2. This is ok"<<endl;
						//find newCumDist1Rev
						tempCumDist1Rev[0] = CumDist[i][j-1];
						w=1;//for tempCumDist1Rev[]
						prevC = seg1before;
						float insertdist1Rev=0;//for check condition;//r1 with tail1
						//cout<<"3. This is ok"<<endl;
						for (int k = segment2Size-1; k >= 0; k--)
						{
							//segment2Reverse.push_back(segment2[k]);//push in reverse order, so noneed to use reverse function
							insertdist1Rev += dist[prevC][segment2[k]] + service_time[segment2[k]];
							tempCumDist1Rev[w] = tempCumDist1Rev[w-1]+dist[prevC][segment2[k]] + service_time[segment2[k]];
							newCumDist1Rev += tempCumDist1Rev[w];
							prevC=segment2[k];
							w++;
						}
						insertdist1Rev += dist[segment2[0]][seg1after];//last arc //for check condition
						//cout<<"4. This is ok"<<endl;
						//=========================================================//

						reverseType = 0;//initialize to zero which means no reverse

						bool seg1Reverse = false, seg2Reverse = false;
						if (newCumDist1Rev < newCumDist1)
						{
							newCumDist1 = newCumDist1Rev;
							insertdist1 = insertdist1Rev;//for check condition
							seg2Reverse = true; //if segment2 reverse, reverseType = 2
							reverseType = 2;
						
						}

						if (newCumDist2Rev < newCumDist2)
						{
							newCumDist2 = newCumDist2Rev;
							insertdist2 = insertdist2Rev;//for check condition
							seg1Reverse = true;//if segment1 reverse, reverseType = 1
							reverseType = 1;
							//tail1ptr = &tail1Rev;
						}

						if (seg1Reverse == true && seg2Reverse == true)//if both segment are reverse, type3
							reverseType = 3;

						delete[] tempCumDist1;
						delete[] tempCumDist1Rev;
						delete[] tempCumDist2;
						delete[] tempCumDist2Rev;
						segment2.clear();
						segment2.shrink_to_fit();

						if ((insertdist1-deletedist1 > DISTANCE+epsilon) || (insertdist2-deletedist2 > DISTANCE+epsilon))
							continue;
			
						gain1 = (oriCumDist1 - newCumDist1) + remainingCust1*(deletedist1-insertdist1);
						gain2 = (oriCumDist2 - newCumDist2) + remainingCust2*(deletedist2-insertdist2);

						distgain1 = (deletedist1-insertdist1);
						distgain2 = (deletedist2-insertdist2);

					

						

							r1found = true;	
							r1 = i;
							r1_p = j;
							r2 = m;
							r2_p = n;
							goto foundshake;
						}//end of n
					}//end of m
					segment1.clear();
					segment1.shrink_to_fit();
				}//end of for j
			}
		}//end if more than 15seconds, select route one by one
	}//end while (r1found == false)  //while r1 not found
	if (no_flag >= SIZE)
	{	
		cout<<"ALL CUSTOEMRS ARE FLAGGED AND STILL NOT FOUND CUSTOMER in SHAKE cross!" <<endl;
		for (int i = 0; i < SIZE; i++)
		{
			cout<<flag_module[i]<<' ';
		}
		//getchar();
		return 0;
	}
foundshake:;
	RchangedStatus[r1] = true;//route 1
	RchangedStatus[r2] = true;//route 2
	
	//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
	//int delete1=0;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5	
	//for (int q = -1; q <= delete1; q++)//flag the one before and current
	//{
	//	CustaffectedStatus[VEHICLE[r1][r1_p+q]] = true;//flagged customer affected true
	//}
	//for (int q = -1; q <= delete1; q++)//flag the one before and current
	//{
	//	CustaffectedStatus[VEHICLE[r2][r2_p+q]] = true;//flagged customer affected true
	//}
	//int last1=1;
	//for (int q = 0; q <= last1; q++)//flag the current last and one after for the end point of L1
	//{
	//	CustaffectedStatus[VEHICLE[r1][r1_p+L1-1+q]] = true;//flagged customer affected true
	//}
	//for (int q = 0; q <= last1; q++)//flag the current last and one after for the end point of L2
	//{
	//	CustaffectedStatus[VEHICLE[r2][r2_p+L2-1+q]] = true;//flagged customer affected true
	//}

	cout<<"From cross-shaking, r1="<<r1<<' '<<r2<<endl;
	gainvec[0] = 1;//status yes
	gainvec[1] = gain1;
	gainvec[2] = distgain1;
	gainvec[3] = gain2;
	gainvec[4] = distgain2;
	cross_insert_fromshaking (L1, L2, r1, r2, r1_p, r2_p, VEHICLE, reverseType);
	float total_cost = 0.0;

	cout<<"============ Routes after shake in CROSS ============"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int k = 0; k <= saiz[i]+1; k++)
		{
			cout << VEHICLE[i][k] << ' ';
		}
		//float r_cost = 0.0;//to check
		//for (int c = 0; c <= saiz[i]; c++)
		//{
		//	r_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		//}
		//if (abs (r_cost-route_cost[i]) > 0.1)
		//{
		//	cout<<endl<<"After shake cross!!!!!!!!!"<<' ';
		//	cout<<"route_cost["<<i<<"]= "<<route_cost[i]<<' '<<"r_cost= "<<r_cost<<endl;
		//	cout<<r1<<' '<<r2<<' '<<r1_p<<' '<<r2_p<<' '<<gain<<endl;
		//	getchar();
		//}
		total_cost += route_cost[i]; 
		cout << endl;

	}
	cout<<"total cost = " <<total_cost<<endl;
	
	delete[] flag_module;
	return 1; //shake status is ok
}

void check_capacity (int *(&capa), int r1, int r2, int r3, int c_f_r1, int c2_f_r1, int c_f_r2, int c2_f_r2, int (&status))
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


void calculate_best_gain(float *(&cost_of_removing), float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0), originalGain &oriGain)
{

	int num_attributes = 13;//[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
	//++++++++++++++++++++++++++++++++++++++++++++++++++DATA STRUCTURE++++++++++++++++++++++++++++++++++++++++++++++++++++++//

	cout << "TOTAL ROUTES= " << no_routes << endl;

	//========================================Initialize Gain matrix ============================================//
	int i = ((no_routes*no_routes) - no_routes) / 2;
	for (int u = (no_routes - 1); u > 0; u--)
	{
		for (int v = u - 1; v >= 0; v--)
		{
			gain_matrix_1_0[u][v] = i;
			info_1_0[(int)gain_matrix_1_0[u][v]][0] = i;
			i--;
		}
	}

	for (int i = 0; i < no_routes; i++)
	{
		for (int j = i; j < no_routes; j++)
		{
			if (i<j)
			{
				gain_matrix_1_0[i][j] = INT_MIN;
				info_1_0[(int)gain_matrix_1_0[j][i]][1] = INT_MIN;
			}
				
		}
		gain_matrix_1_0[i][i] = INT_MIN;
		same_r_info[i][0] = i;
		same_r_info[i][1] = INT_MIN;
		for (int j = 2; j < num_attributes; j++)
		{
			same_r_info[i][j] = -1;
		}
	}

	int s = ((no_routes*no_routes) - no_routes) / 2;
	for (int j = 1; j <= s; j++)
	{
		for (int k = 2; k < num_attributes; k++)
		{
			info_1_0[j][k] = -1;
		}
	}

	//vehicle should be [0]depot, [1] (cust) , [last]depot
	for (int i = 0; i < no_routes; i++)
	{
		for (int b = 0; b <= saiz[i]+1; b++)
		{
			cout << VEHICLE[i][b] << ' ';
		}
		cout << endl;

	}

	
	//========================================Cost of removing ============================================//
	//cost of removing for single customer
	for (int f = 0; f < no_routes; f++) //find cost of removing for each customer and put in array
	{
		int before = -1, after = -1;
		for (int h = 1; h <= saiz[f]; h++) 
		{
			//if (tempFlag[VEHICLE[f][h]]==false)//if false, noneed to find , true need to find
			//	continue;
		
			before = VEHICLE[f][h - 1];
			after = VEHICLE[f][h + 1];	
			float origain = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];//old-new
			int remainingcust = saiz[f]-h;
			if (remainingcust < 0)//if negative
				remainingcust= 0;
			cost_of_removing[VEHICLE[f][h]] = CumDist[f][h] + remainingcust*origain;//cannot update from a certain start point because cost_of_removing for the entire route change when a customer is deleted from the route, because number_of_remaining customer changes
		
		}
	}

	best_gain_1_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	best_gain_1_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	best_gain_2_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	best_gain_2_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	best_gain_2_2swap(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	two_optintra(same_r_info, gain_matrix_1_0);
	two_optinter(info_1_0, gain_matrix_1_0);
	crossTail2(info_1_0, gain_matrix_1_0);
	//CROSS(info_1_0, gain_matrix_1_0);

	//============================= copy to oriGain ================================//
	for (int i = 0; i < SIZE; i++)
	{
		oriGain.Oricost_of_removing[i] = cost_of_removing[i];

	}

	for (int i = 0; i < no_routes; i++)
	{
		for (int j = 0; j < no_routes; j++)
		{
			oriGain.Origain_matrix_1_0[i][j] = gain_matrix_1_0[i][j];	
		}

		for (int j = 0; j < num_attributes; j++)
		{
			oriGain.Orisame_r_info[i][j] = same_r_info[i][j];
		}

	}

	for (int i = 1; i <= s; i++)
	{
		for (int j = 0; j < num_attributes; j++)
		{
			oriGain.Oriinfo_1_0[i][j] = info_1_0[i][j];
		}	
	}
}


void find_all_cost_of_removing (float *(&cost_of_removing))
{
	for (int f = 0; f < no_routes; f++)
	{
		if (saiz[f]==0)
			continue;
		int before = -1, after = -1;
		for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		{
			before = VEHICLE[f][h - 1];
			after = VEHICLE[f][h + 1];	
			float origain = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];//old-new
			int remainingcust = saiz[f]-h;
			if (remainingcust < 0)//if negative
				remainingcust= 0;
			cost_of_removing[VEHICLE[f][h]] = CumDist[f][h] + remainingcust*origain;//cannot update from a certain start point because cost_of_removing for the entire route change when a customer is deleted from the route, because number_of_remaining customer changes
		
		}
	}
}


void reinitializeRchangedStatus ()
{
	for (int i = 0; i < no_routes; i++)
	{
		RchangedStatus[i] = false;//initialize to 0 for every route, they have not been changed
		//Route[i].RchangedStatus=0;//initialize to 0 for every route, they have not been changed
	}
	
	//for (int i = 0; i <= SIZE; i++)
	//{
	//	CustaffectedStatus[i] = false; //initialize to 0	//including depot because sometimes the flag position is [0] but the depot has no cost of removing, so doesnt matter
	//}
}

//DONE
void partialupdate_costremoving(float *(&cost_of_removing))//update cost of removing based on Rchanged status and position in the route 
{
	
	for (int f = 0; f < no_routes; f++)
	{
		if ((saiz[f]==0) || (RchangedStatus[f] == false))
			continue;
		int before = -1, after = -1;
		for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		{
			//if (CustaffectedStatus[VEHICLE[f][h]] == false)//if false, they have not been affected, so noneed to find cost of removing
			//	continue;
			before = VEHICLE[f][h - 1];
			after = VEHICLE[f][h + 1];	
			float origain = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];//old-new
			int remainingcust = saiz[f]-h;
			if (remainingcust < 0)//if negative
				remainingcust= 0;
			cost_of_removing[VEHICLE[f][h]] = CumDist[f][h] + remainingcust*origain;//cannot update from a certain start point because cost_of_removing for the entire route change when a customer is deleted from the route, because number_of_remaining customer changes
		
		}
	}

}

//DONE
void partialupdate_reoptimize_1_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{
	int num_attributes = 13;//[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
	int element=-1, before_ele, after_ele, before_pos, after_pos;

	float gain;
	int from1, to1;
	float cost_without_i, cost_with_i;
	
	//REMEMBER, GAIN_MATRIX AND SAME_INFO RECORDED BASED ON ROUTES, info_matrix RECORDED BASED ON POSITIONAL INDEX!!!!!!!!!!!!!!!!!
	//reinitialize the gain for affected routes to be INT_MIN
	for (int s = 0; s < no_routes; s++)
	{
		if (RchangedStatus[s] == false)
			continue;
		
		for (int t = 0; t < no_routes; t++)//////////////////////////////////////////////////////////////////////////////////////////////////////
		{
			if (s<t)
			{
				gain_matrix_1_0[s][t] = INT_MIN;
				info_1_0[(int)gain_matrix_1_0[t][s]][1] = INT_MIN;

			}

			else if (t<s)
			{
				gain_matrix_1_0[t][s] = INT_MIN;
				info_1_0[(int)gain_matrix_1_0[s][t]][1] = INT_MIN;
			}
		}
		gain_matrix_1_0[s][s] = INT_MIN; //initialize diagonal for intra-route gain	
		same_r_info[s][1] = INT_MIN;
		for (int u = 2; u < num_attributes; u++)
		{
			same_r_info[s][u] = -1;
		}

	}

	int s = ((no_routes*no_routes)-no_routes)/2;
	for (int i = 1; i < s; i++)
	{
		if ( info_1_0[i][1] == INT_MIN)
		{	
			for (int j = 2; j < num_attributes; j++)
			{
				info_1_0[i][j] = -1;
			}
		}
	}
	

	int i = -1; //initilize for affected route
	//////////////////================================== SAME ROUTE =========================////////////////////////////////////
	for (int r = 0; r < no_routes; r++) 
	{
		if (RchangedStatus[r] == false)
			continue;
		else
			i = r;

		//if(r==0)
		//	i=r1;
		//else
		//	i=r2;
		//if ((saiz[i]==0) || (saiz[i]==1))//if it is empty route or saiz=1
		if (saiz[i]<=2)
			continue;
		
		for (int j = 1; j <= saiz[i]; j++) //which one to delete
		{
			float available_dist_r2 = distance_available[i];
			//copy a temporary route
			int b=0;
			int* temp_r = new int[saiz[i]+2];
			for (int a = 0; a < saiz[i] + 2; a++)
			{
				if (a == j)
				{
					continue; //dont copy the element, assume the element to be deleted
				}

				else
				{
					temp_r[b] = VEHICLE[i][a];
					b++;
				}
			}
			element = VEHICLE[i][j];
			before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
			after_ele = VEHICLE[i][j + 1];
			float origain = dist[element][before_ele] + dist[element][after_ele] + service_time[element] - dist[before_ele][after_ele];//old-new positive is good
				
			//to find cost_with_i which is besed on the modified route
			//find a temporaryCumDCust
			float *tempCumDist= new float[SIZE];
			tempCumDist[0] = 0;
			for (int t = 1; t < saiz[i]; t++)//have 1 size less because 1 customer is deleted
			{
				if (t<j)
					tempCumDist[t]= CumDist[i][t];//before the delete pos, the CumDist is the same
				else if(t>=j)
					tempCumDist[t]=CumDist[i][t+1]-origain;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
					
			}
			
			for (int n = 1; n < saiz[i] + 1; n++) //has 1 customer less because it has been deleted
			{
				if (j == n) //originally from this position, so noneed to consider
					continue; 
				else
				{
					before_pos = temp_r[n - 1];
					after_pos = temp_r[n];  //noneed to add 1 because it is insert in between n-1 and n

				}	
				

				if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
				{
					if ( NR_FLAG_DEPOT[element][before_pos] == false && NR_FLAG_DEPOT[element][after_pos] == false ) //use NR_FLAG_DEPOT
						continue;
				}

				else //if not between customer and depot
				{	
					if ( NR_FLAG[element][before_pos] == false && NR_FLAG[element][after_pos] == false ) //if insertion cannot between cust before and cust after
						continue;
				}	
				
				cost_without_i = cost_of_removing[VEHICLE[i][j]];//cost of r1 without i
				//float origain = dist[element][before_ele] + dist[element][after_ele] + service_time[element] - dist[before_ele][after_ele];//old-new positive is good
				//
				////to find cost_with_i which is besed on the modified route
				////find a temporaryCumDCust
				//float *tempCumDist= new float[SIZE];
				//tempCumDist[0] = 0;
				//for (int t = 1; t < saiz[i]; t++)//have 1 size less because 1 customer is deleted
				//{
				//	if (t<j)
				//		tempCumDist[t]= CumDist[i][t];//before the delete pos, the CumDist is the same
				//	else if(t>=j)
				//		tempCumDist[t]=CumDist[i][t+1]-origain;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
				//	
				//}
				
				float oriloss = dist[before_pos][element] + dist[element][after_pos] + service_time[element] - dist[before_pos][after_pos]; //(new+service_time – old)
				
				if (oriloss - origain > available_dist_r2 + epsilon)
				{
					continue;
				}
				float CumAtEle = tempCumDist[n-1] + dist[before_pos][element] + service_time[element]; 
				int remainingCustafterEle = saiz[i] - n;
				if (remainingCustafterEle < 0)//if negative
							remainingCustafterEle = 0;
				cost_with_i = CumAtEle + remainingCustafterEle*oriloss; 

				gain = cost_without_i - cost_with_i;
				if (gain > INFIN)
					{
						cout<<"Gain is too big, sth wrong"<<endl;
						getchar();
					}
				
				if (gain-gain_matrix_1_0[i][i] > epsilon)//gain > gain_matrix_1_0[i][i]
				{
					gain_matrix_1_0[i][i] = gain;
					same_r_info[i][0] = i;
					same_r_info[i][1] = gain;
					same_r_info[i][2] = i; //from route
					same_r_info[i][3] = i; //to route, i=m here
					same_r_info[i][4] = j; //from which position
					same_r_info[i][5] = n; //to which position //assume modified route, customer has been deleted
					same_r_info[i][6] = -1; 
					same_r_info[i][7] = -1;
					same_r_info[i][8] = 1; //1 is (1-0)
					same_r_info[i][10] = gain; //gain1 //this is intra-route, gain1  = gain
					same_r_info[i][11] = origain - oriloss; //gain1 in distance

				}	
			}
			delete[] tempCumDist;
			delete[] temp_r;
		}//end of j

	}

	///////////////// ============================== DIFERENT ROUTES ======================================//////////////////////
	//######################################update gain matrix################################################//
	//from 2 routes to every position=======================update row============================= ///////////////////////////////////
	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from)
	{
		if (RchangedStatus[i] == false)
			continue;
		else
			from1 = i; 
		
		//if (i == 0)
		//	from1 = r1;

		//else
		//	from1 = r2;
		
		if (saiz[from1]==0)//if it is empty route
			continue;
		for (int j = 1; j < saiz[from1] + 1; j++)  // consider each customer (from)
		{
			element = VEHICLE[from1][j];			//elemet to be inserted into other route
	
			//cout << element;

			for (int m = 0; m < no_routes; m++)  //insert (to) which route
			{
				if ((m == from1) || (demand[element] > space_available[m])) //if customer already in the route or demand exceed avaialable space in route
				{
					continue;
				}
				float available_dist_r2 = distance_available[m];

				for (int n = 1; n < saiz[m] + 2; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
				{
					before_ele = VEHICLE[from1][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[from1][j + 1];
					before_pos = VEHICLE[m][n - 1];
					after_pos = VEHICLE[m][n];  //noneed to add 1 because it is insert in between n-1 and n
					
					if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
					{
						if ( NR_FLAG_DEPOT[element][before_pos] == false && NR_FLAG_DEPOT[element][after_pos] == false ) //use NR_FLAG_DEPOT
							continue;
					}

					else
					{
						if ( NR_FLAG[element][before_pos] == false && NR_FLAG[element][after_pos] == false ) //if insertion cannot between cust before and cust after
							continue;
					}
					float origain = dist[element][before_ele] + dist[element][after_ele] + service_time[element] - dist[before_ele][after_ele];
					cost_without_i = cost_of_removing[VEHICLE[from1][j]];//cost of r1 without i

					float oriloss = dist[before_pos][element] + dist[element][after_pos] + service_time[element] - dist[before_pos][after_pos]; //cost of r2 with i //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //(new+service_time – old)
					float CumAtEle = CumDist[m][n-1] + dist[before_pos][element] + service_time[element]; 

					int remainingCustafterEle = saiz[m] - n +1; //different from sameroute
					if (remainingCustafterEle < 0)//if negative
							remainingCustafterEle = 0;
					cost_with_i = CumAtEle + remainingCustafterEle*oriloss; 
					
					
					if (oriloss - available_dist_r2 > epsilon) //only consider r2
					{
						continue;
					}

					gain = cost_without_i - cost_with_i;
					if (gain > INFIN)
					{
						cout<<"Gain is too big, sth wrong"<<endl;
						getchar();
					}

					if (from1 < m)
					{
						if (gain-gain_matrix_1_0[from1][m] > epsilon)//gain > gain_matrix_1_0[from1][m]
						{
							gain_matrix_1_0[from1][m] = gain;
							//info_1_0[gain_matrix_1_0[m][from1]][0] = gain_matrix_1_0[m][from1];
							info_1_0[(int)gain_matrix_1_0[m][from1]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[m][from1]][2] = from1;
							info_1_0[(int)gain_matrix_1_0[m][from1]][3] = m;
							info_1_0[(int)gain_matrix_1_0[m][from1]][4] = j;
							info_1_0[(int)gain_matrix_1_0[m][from1]][5] = n;
							info_1_0[(int)gain_matrix_1_0[m][from1]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[m][from1]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[m][from1]][8] = 1; //1 is (1-0)
							info_1_0[(int)gain_matrix_1_0[m][from1]][10] = cost_without_i; //gain1
							info_1_0[(int)gain_matrix_1_0[m][from1]][11] = origain;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[m][from1]][12] = -oriloss;//gain2 in distance
						}
					}

					else if (from1 > m)
					{
						if (gain-gain_matrix_1_0[m][from1] > epsilon)//gain > gain_matrix_1_0[m][from1]
						{
							gain_matrix_1_0[m][from1] = gain;
							//info_1_0[gain_matrix_1_0[m][from1]][0] = gain_matrix_1_0[from1][m];
							info_1_0[(int)gain_matrix_1_0[from1][m]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[from1][m]][2] = from1;
							info_1_0[(int)gain_matrix_1_0[from1][m]][3] = m;
							info_1_0[(int)gain_matrix_1_0[from1][m]][4] = j;
							info_1_0[(int)gain_matrix_1_0[from1][m]][5] = n;
							info_1_0[(int)gain_matrix_1_0[from1][m]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[from1][m]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[from1][m]][8] = 1; //1 is (1-0)
							info_1_0[(int)gain_matrix_1_0[from1][m]][10] = cost_without_i; //gain1
							info_1_0[(int)gain_matrix_1_0[from1][m]][11] = origain;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[from1][m]][12] = -oriloss;//gain2 in distance

						}
					}
				}//end for n
			}//end for m
		}//end for j
		//cout << i << ' '<< element << ' '<< old_route[i] << ' ' <<old_position[i]<< ' ' << ' '<< new_route[i] << ' ' << new_position[i] << ' '<< ' '<< gain_array[i] << endl;
	}//end for i 

	//######################################update gain matrix################################################//
	//from every position to 2 routes=======================update column============================= ///////////////////////////////////


	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from)
	{
		if (saiz[i]==0) //if it is empty route
			continue;
		for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from)
		{
			element = VEHICLE[i][j];			//element to be inserted into other route
	
			//cout << element;

			for (int m = 0; m < no_routes; m++)  //insert (to) which route
			{
				if (RchangedStatus[m] == false)
					continue;
				else
					to1 = m;
				
				//if (m == 0)
				//	to1 = r1;

				//else
				//	to1 = r2;

				float available_dist_r2 = distance_available[to1];
	
				if ((i == to1) || (demand[element] > space_available[to1])) //if demand exceed avaialable space in route || differnt route, if same route, capacity exceed stil have to find best gain
				{
					continue;
				}
				for (int n = 1; n < saiz[to1] + 2; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
				{
					before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[i][j + 1];
					before_pos = VEHICLE[to1][n - 1];
					after_pos = VEHICLE[to1][n];  //noneed to add 1 because it is insert in between n-1 and n
					
					if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
					{
						if ( NR_FLAG_DEPOT[element][before_pos] == false && NR_FLAG_DEPOT[element][after_pos] == false ) //use NR_FLAG_DEPOT
							continue;
					}

					else
					{
						if ( NR_FLAG[element][before_pos] == false && NR_FLAG[element][after_pos] == false ) //if insertion cannot between cust before and cust after
							continue;
					}

					float origain = dist[element][before_ele] + dist[element][after_ele] + service_time[element] - dist[before_ele][after_ele];
					cost_without_i = cost_of_removing[VEHICLE[i][j]];//cost of r1 without i
					float oriloss = dist[before_pos][element] + dist[element][after_pos] + service_time[element] - dist[before_pos][after_pos]; //cost of r2 with i //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //(new+service_time – old)
					
					if (oriloss - available_dist_r2 > epsilon) //only consider r2
					{
						continue;
					}

					float CumAtEle = CumDist[to1][n-1] + dist[before_pos][element] + service_time[element]; 
					int remainingCustafterEle = saiz[to1] - n + 1;//different from sameroute
					if (remainingCustafterEle < 0)//if negative
							remainingCustafterEle = 0;
					cost_with_i = CumAtEle + remainingCustafterEle*oriloss; 

					//cost_with_i = dist[before_pos][element] + dist[element][after_pos] + service_time[element]  - dist[before_pos][after_pos]; //cost of r2 with i  //(new+service_time – old)
					
			
					gain = cost_without_i - cost_with_i;
					if (gain > INFIN)
					{
						cout<<"Gain is too big, sth wrong"<<endl;
						getchar();
					}

					if (i < to1)
					{
						if (gain-gain_matrix_1_0[i][to1] > epsilon)//gain > gain_matrix_1_0[i][to1]
						{
							gain_matrix_1_0[i][to1] = gain;
							//info_1_0[(int)gain_matrix_1_0[i][to1]][0] = gain_matrix_1_0[to1][i];
							info_1_0[(int)gain_matrix_1_0[to1][i]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[to1][i]][2] = i;
							info_1_0[(int)gain_matrix_1_0[to1][i]][3] = to1;
							info_1_0[(int)gain_matrix_1_0[to1][i]][4] = j;
							info_1_0[(int)gain_matrix_1_0[to1][i]][5] = n;
							info_1_0[(int)gain_matrix_1_0[to1][i]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[to1][i]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[to1][i]][8] = 1; //1 is (1-0)
							info_1_0[(int)gain_matrix_1_0[to1][i]][10] = cost_without_i;
							info_1_0[(int)gain_matrix_1_0[to1][i]][11] = origain; //gain1 in cost
							info_1_0[(int)gain_matrix_1_0[to1][i]][12] = -oriloss; //gain2 in cost

						}
					}
					else if (i > to1)
					{
						if (gain-gain_matrix_1_0[to1][i] > epsilon)//gain > gain_matrix_1_0[to1][i]
						{
							gain_matrix_1_0[to1][i] = gain;
							//info_1_0[(int)gain_matrix_1_0[to1][i]][0] = gain_matrix_1_0[i][to1];
							info_1_0[(int)gain_matrix_1_0[i][to1]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[i][to1]][2] = i;
							info_1_0[(int)gain_matrix_1_0[i][to1]][3] = to1;
							info_1_0[(int)gain_matrix_1_0[i][to1]][4] = j;
							info_1_0[(int)gain_matrix_1_0[i][to1]][5] = n;
							info_1_0[(int)gain_matrix_1_0[i][to1]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[i][to1]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[i][to1]][8] = 1;//1 is (1-0)
							info_1_0[(int)gain_matrix_1_0[i][to1]][10] = cost_without_i;
							info_1_0[(int)gain_matrix_1_0[i][to1]][11] = origain; //gain1 in cost
							info_1_0[(int)gain_matrix_1_0[i][to1]][12] = -oriloss; //gain2 in cost
						}
					}
				}//end for n
			}//end for m
		}//end for j
	}

}

//DONE
void partialupdate_reoptimize_1_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{
	int num_attributes = 13;//[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
	
	int element1, element2, before_ele1, before_ele2, after_ele1=-1, after_ele2=-1, before_pos1, before_pos2, after_pos1, after_pos2;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_j, gain;
	
	//NONEED REINITIALIZE, this will delete the previous found gain
	std::vector<int> vector_r1;
	std::vector<int> vector_r2;
	std::vector<int> vector_r3;
	int r=-1;

	//////////////////////////////////============================= SAME ROUTE ===============================================//////////////////////
	for (int I = 0; I < no_routes; I++)
	{
		if (RchangedStatus[I] == false)
			continue;
		else
			r = I;

		float available_dist_r2  = distance_available[r];
		//if (I==0)
		//	r=r1;
		//else
		//	r=r2;
		//if ((saiz[r]==0)||(saiz[r]==1)) //if it is empty route or saiz=1, if only one customer, nothing to move around
		if (saiz[r]<=2)	
			continue;
	
		vector_r3.clear();
		for (int i = 0; i < saiz[r] + 2; i++) //first and last is depot
		{
			vector_r3.push_back(VEHICLE[r][i]);
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

				if ( (before_ele2 == SIZE || after_ele2 == SIZE) && (before_ele1 == SIZE || after_ele1 == SIZE) ) //if mn and ij depot
				{	
					if ( (NR_FLAG_DEPOT[element1][before_ele2] == false && NR_FLAG_DEPOT[element1][after_ele2] == false) || (NR_FLAG_DEPOT[element2][before_ele1] == false && NR_FLAG_DEPOT[element2][after_ele1] == false) )
						continue;
				}
				else if ((before_ele2 == SIZE) || (after_ele2 == SIZE) ) //if mn depot
				{
					if ((NR_FLAG_DEPOT[element1][before_ele2] == false && NR_FLAG_DEPOT[element1][after_ele2] == false) || (NR_FLAG[element2][before_ele1] == false && NR_FLAG[element2][after_ele1] == false) )
						continue;
				}
				else if ((before_ele1 == SIZE) || (after_ele1 == SIZE) ) //if ij depot
				{
					if ((NR_FLAG[element1][before_ele2] == false && NR_FLAG[element1][after_ele2] == false) || (NR_FLAG_DEPOT[element2][before_ele1] == false && NR_FLAG_DEPOT[element2][after_ele1] == false) )
						continue;
				}
				else //if no depot
				{
					if ((NR_FLAG[element1][before_ele2] == false && NR_FLAG[element1][after_ele2] == false) || (NR_FLAG[element2][before_ele1] == false && NR_FLAG[element2][after_ele1] == false))
						continue;
				}
				gain=0;
					
				//in this case, j is always smaller than k
				//int first = j, second=j+1, third=k, fourth=k+1;//first is position of ele1, second is after_ele1, third is ele2, fourth is after_ele2
				float newCumDistFirst = CumDist[r][j-1] + dist[before_ele1][element2] + service_time[element2];
				//float newCumDistSecond = newCumDistFirst + dist[element2][after_ele1] + service_time[after_ele1];
				float Firstincre = CumDist[r][j] - newCumDistFirst ;//old - new CumDist//positive is good, negative is bad
				//float Secondincre = CumDist[r][j+1] - newCumDistSecond;
				gain = Firstincre;
				//to find how many customers in between Second and Third (exclusive)
				
				if (k==j+1)//if consecutive customer and k is not the last customer, if it is last customer, noneed to calculate the remaining
				{
					cost_without_i = dist[before_ele1][element1] + dist[element1][after_ele1] + dist[element2][after_ele2]; //minus 3 arcs 
					cost_with_i = dist[before_ele1][element2] + dist[element2][element1] +dist[element1][after_ele2];
					if (cost_with_i - cost_without_i > available_dist_r2 + epsilon)
					{
						continue;
					}
					float newCumDistSecond = newCumDistFirst + dist[element2][element1] + service_time[element1];
					float Secondincre = CumDist[r][j+1] - newCumDistSecond;
					gain = gain +Secondincre;

					if (k!=saiz[r])//if k is not the last customer, if it is last customer, noneed to calculate the remaining
					{
						float newCumDistThird = (newCumDistSecond) + dist[element1][after_ele2] + service_time[after_ele2];
						float Thirdincre = CumDist[r][j+2] - newCumDistThird;//old - new CumDist//positive is good, negative is bad
						//inbetween calculate number of customers at j+1 until saiz[r] (including j+1)
						int inbetween1 = saiz[r]-(j+1);//if use saiz[r]-(j+1+1), then need to add Thirdincre in gain because Thirdincre is used on third element until the last one
						if (inbetween1>0)
						{
							gain = gain+(Thirdincre*inbetween1);
						}
					}
				}
				else//if not consecutive customer
				{
					
					cost_without_i = dist[before_ele1][element1] + dist[element1][after_ele1] + dist[before_ele2][element2] + dist[element2][after_ele2]; //minus 4 arcs 
					cost_with_i = dist[before_ele1][element2] + dist[element2][after_ele1] +dist[before_ele2][element1] +dist[element1][after_ele2];
					if (cost_with_i - cost_without_i > available_dist_r2 + epsilon)
					{
						continue;
					}
					float newCumDistSecond = newCumDistFirst + dist[element2][after_ele1] + service_time[after_ele1];
					float Secondincre = CumDist[r][j+1] - newCumDistSecond;
					gain = gain + Secondincre;
					//inbetween1 calculate number of customers at j+1 until k-1 (exclude j and exclude k)
					int inbetween1 = (k-1)-(j+1);
					if (inbetween1>0)
					{
						gain = gain+(Secondincre*inbetween1);//in between can be 0 mean ele1 and ele is 1 customer apart, eg: ele1, 5, ele2
					}
					//we dont know the newCumDist at point k-1, so we use the oriCumDist minus the Seconsdincre (minus increment because for increment positive is good, negative is bad)
					float newCumDistThird = (CumDist[r][k-1]-Secondincre) + dist[before_ele2][element1] + service_time[element1];//minus increment, because for increment positive is good, negative is bad
					float Thirdincre = CumDist[r][k] - newCumDistThird;//old - new CumDist//positive is good, negative is bad
					gain = gain+Thirdincre;


					//Fourth may never exists if k is saiz[r]
					if (k<saiz[r])
					{
						float newCumDistFourth = newCumDistThird + dist[element1][after_ele2]  + service_time[after_ele2];;
						float Fourthincre = CumDist[r][k+1] - newCumDistFourth;
						gain = gain+Fourthincre;
						//to find how many customers in between Fourth and end of route
						//inbetween2 calculate number of customers at k+1 until saiz[r] (including k+1)
						int inbetween2 = saiz[r]-(k+1);//if use saiz[r]-(k+1+1), then need to add Fourthincre in gain because Fourthincre is used on fourth element until the last one
						if (inbetween2>0)
						{
							gain = gain+(Fourthincre*inbetween2);
						}
					}
				}
				if (gain > INFIN)
					{
						cout<<"Gain is too big, sth wrong"<<endl;
						getchar();
					}
				if (gain-gain_matrix_1_0[r][r] > epsilon)//gain > gain_matrix_1_0[r][r]
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
					same_r_info[r][10] = gain; //gain1
					same_r_info[r][11] = cost_without_i - cost_with_i; //gain1 in distance
				}	
			}		
		}
		vector_r3.clear();
	}

	//////////////////////////////////==============================  DIFFERENT ROUTES ==========================================////////////////////
	
	int k = -1; //initialize to -1
	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//================================================(from) 2 affected routes to other routes (update ROW)================================================================
	for (int I = 0; I < no_routes; I++)  //consider each and other routes (from R1)
	{
		if (RchangedStatus[I] == false)
			continue;
		else
			k = I;
		//if (I == 0)
		//	k = r1;

		//else
		//	k = r2;

		if (saiz[k]==0) //if it is empty route
			continue;
		for (int j = 1; j < saiz[k] + 1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
		{
			element1 = VEHICLE[k][j];			//element to be inserted into other route
			before_ele1 = VEHICLE[k][j-1];
			after_ele1 = VEHICLE[k][j+1];
			
			float origain1 = dist[element1][before_ele1] + dist[element1][after_ele1] + service_time[element1] - dist[before_ele1][after_ele1];//old-new positive is good //for tempCumDist
			//to find cost_with_i which is besed on the modified route
			//find a temporaryCumDCust
			float *tempCumDist1= new float[SIZE];
			tempCumDist1[0] = 0;
							
			//following is R1
			for (int t = 1; t < saiz[k]; t++)//have 1 size less because 1 customer is deleted
			{
				if (t<j)
					tempCumDist1[t]= CumDist[k][t];//before the delete pos, the CumDist is the same
				else if(t>=j)
					tempCumDist1[t]=CumDist[k][t+1]-origain1;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
							
			}

			for (int m = 0; m < no_routes; m++)  //consider each and other routes (from R2)
			{
				if ((k == m) || (saiz[m]==0)) //|| (demand[element2] > space_r1)) //if customer already in the route or demand exceed avaialable space in route
				{
					continue;
				}
			
				for (int n = 1; n < saiz[m] + 1; n++) //which customer in r2 to remove
				{
					element2 = VEHICLE[m][n];			//element to be removed from r2
					before_ele2 = VEHICLE[m][n-1];
					after_ele2 = VEHICLE[m][n+1];

					int space_r1 = space_available[k];
					int space_r2 = space_available[m];
					space_r1 = space_r1 + demand[element1];
					space_r2 = space_r2 + demand[element2];

					if ((demand[element2] > space_r1) || (demand[element1] > space_r2))
						continue; //go to next element to delete
					
					float origain2 = dist[element2][before_ele2] + dist[element2][after_ele2] + service_time[element2] - dist[before_ele2][after_ele2];//old-new positive is good //for tempCumDist
					
					//to find cost_with_i which is besed on the modified route
					//find a temporaryCumDCust
					float *tempCumDist2= new float[SIZE];
					tempCumDist2[0] = 0;

					//following is R2
					for (int t = 1; t < saiz[m]; t++)//have 1 size less because 1 customer is deleted
					{
						if (t<n)
							tempCumDist2[t]= CumDist[m][t];//before the delete pos, the CumDist is the same
						else if(t>=n)
							tempCumDist2[t]=CumDist[m][t+1]-origain2;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
							
					}

					//==================================================copy r1 without customer j=================================================================//
					vector_r1.clear();
					for (int a = 0; a < saiz[k] + 2; a++)
					{
						if (a == j)
							continue;
						vector_r1.push_back(VEHICLE[k][a]);
					}
					//vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot
							
					//==================================================copy r2 without customer n=================================================================//	
					vector_r2.clear();
					for (int a = 0; a < saiz[m] + 2; a++)
					{
						if (a == n)
							continue;
						vector_r2.push_back(VEHICLE[m][a]);

					}
					//vector_r2.erase(vector_r2.begin() + n); //first one is depot	

					float available_dist_r1 = distance_available[k];
					float available_dist_r2 = distance_available[m];
	

					for (int p = 1; p < vector_r2.size(); p++) //from r1, insert to r2
					{
						before_pos2 = vector_r2[p - 1];
						after_pos2 = vector_r2[p];  //noneed to add 1 because it is insert in between n-1 and n
						if ( before_pos2 == SIZE || after_pos2 == SIZE ) //if mn depot //changed this to here on 25JUN2015!!!!!!!!!!!!!!!!!!!!! OMG!!! before that put after n loop
						{
							if ( NR_FLAG_DEPOT[element1][before_pos2] == false && NR_FLAG_DEPOT[element1][after_pos2] == false )
								continue;
						}
						else //if no depot
						{
							if ( NR_FLAG[element1][before_pos2] == false && NR_FLAG[element1][after_pos2] == false )
								continue;
						}

						for (int q = 1; q < vector_r1.size(); q++) //from r2, insert to r1
						{
							before_pos1 = vector_r1[q - 1];
							after_pos1 = vector_r1[q];  //noneed to add 1 because it is insert in between n-1 and n
							if ( before_pos1 == SIZE || after_pos1 == SIZE ) //if ij depot
							{	
								if ( NR_FLAG_DEPOT[element2][before_pos1] == false && NR_FLAG_DEPOT[element2][after_pos1] == false )
									continue;
							}
							
							else //if no depot
							{
								if ( NR_FLAG[element2][before_pos1] == false && NR_FLAG[element2][after_pos1] == false )
									continue;
							}

							cost_without_i = cost_of_removing[VEHICLE[k][j]];
							//float origain1 = dist[element1][before_ele1] + dist[element1][after_ele1] + service_time[element1] - dist[before_ele1][after_ele1];//old-new positive is good //for tempCumDist

							cost_without_j = cost_of_removing[VEHICLE[m][n]];
							//float origain2 = dist[element2][before_ele2] + dist[element2][after_ele2] + service_time[element2] - dist[before_ele2][after_ele2];//old-new positive is good //for tempCumDist
							

							//to find cost_with_i which is besed on the modified route
							//find a temporaryCumDCust
							//float *tempCumDist1= new float[SIZE];
							//tempCumDist1[0] = 0;
							//float *tempCumDist2= new float[SIZE];
							//tempCumDist2[0] = 0;

							////following is R1
							//for (int t = 1; t < saiz[k]; t++)//have 1 size less because 1 customer is deleted
							//{
							//	if (t<j)
							//		tempCumDist1[t]= CumDist[k][t];//before the delete pos, the CumDist is the same
							//	else if(t>=j)
							//		tempCumDist1[t]=CumDist[k][t+1]-origain1;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
							//
							//}
						
							float oriloss1 = dist[before_pos1][element2] + dist[element2][after_pos1] + service_time[element2] - dist[before_pos1][after_pos1]; //element from r2 to r1 or insert ele2 to r1
							float CumAtEle1 = tempCumDist1[q-1] + dist[before_pos1][element2] + service_time[element2]; 
							int remainingCustafterEle1 = saiz[k] - q;
							if (remainingCustafterEle1 < 0)//if negative
								remainingCustafterEle1 = 0;
							cost_with_i = CumAtEle1 + remainingCustafterEle1*oriloss1; 


							////following is R2
							//for (int t = 1; t < saiz[m]; t++)//have 1 size less because 1 customer is deleted
							//{
							//	if (t<n)
							//		tempCumDist2[t]= CumDist[m][t];//before the delete pos, the CumDist is the same
							//	else if(t>=n)
							//		tempCumDist2[t]=CumDist[m][t+1]-origain2;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
							//
							//}
						
							float oriloss2 = dist[before_pos2][element1] + dist[element1][after_pos2] + service_time[element1]- dist[before_pos2][after_pos2]; //element from r1 to r2 or insert ele1 to r2
							float CumAtEle2 = tempCumDist2[p-1] + dist[before_pos2][element1] + service_time[element1]; 
							int remainingCustafterEle2 = saiz[m] - p;
							if (remainingCustafterEle2 < 0)//if negative
								remainingCustafterEle2 = 0;
							cost_with_j = CumAtEle2 + remainingCustafterEle2*oriloss2; 

							//cost_with_i is R1 //cost_with_j is R2 
							if ((oriloss1 - origain1 > available_dist_r1 + epsilon) || (oriloss2 - origain2 > available_dist_r2 + epsilon))
								continue;

							gain = (cost_without_i + cost_without_j) - (cost_with_i + cost_with_j);

						if (gain > INFIN)
					{
						cout<<"Gain is too big, sth wrong"<<endl;
						getchar();
					}

							if (k < m)
							{
								if (gain-gain_matrix_1_0[k][m] > epsilon)//gain > gain_matrix_1_0[k][m]
								{
									gain_matrix_1_0[k][m] = gain;

									//info_1_0[number_matrix_1_1[k][m]][0] = gain_matrix_1_1[m][k];
									info_1_0[(int)gain_matrix_1_0[m][k]][1] = gain;
									info_1_0[(int)gain_matrix_1_0[m][k]][2] = k;
									info_1_0[(int)gain_matrix_1_0[m][k]][3] = m;
									info_1_0[(int)gain_matrix_1_0[m][k]][4] = j;//r1 without
									info_1_0[(int)gain_matrix_1_0[m][k]][5] = p;//r2 with
									info_1_0[(int)gain_matrix_1_0[m][k]][6] = n;//r2 without
									info_1_0[(int)gain_matrix_1_0[m][k]][7] = q;//r1 with
									info_1_0[(int)gain_matrix_1_0[m][k]][8] = 2; //2 is (1-1)
									info_1_0[(int)gain_matrix_1_0[m][k]][10] = cost_without_i  - cost_with_i; //gain1
									//ofstream TEMP("TO CHECK.txt", ios::app);
									//to check
									//if((j==q) && (p==n))
									//{
									//	float cost_without_i2 = dist[before_pos1][element1] + dist[element1][after_pos1] + service_time[element1] - dist[before_pos1][after_pos1];//to check 
									//	float cost_without_j2 = dist[before_pos2][element2] + dist[element2][after_pos2] + service_time[element2] - dist[before_pos2][after_pos2];//to check 
									//	if ((abs(cost_without_i2-cost_of_removing[VEHICLE[k][j]])>0.1) || (abs(cost_without_j2-cost_of_removing[VEHICLE[m][n]])>0.1))
									//	{
									//		
									//		cout<<"partialupdate_reoptimize_1_1 got problem!!"<<endl;
									//		TEMP<<cost_without_i2<<' '<<cost_of_removing[VEHICLE[k][j]]<<endl;
									//		TEMP<<cost_without_j2<<' '<<cost_of_removing[VEHICLE[m][n]]<<endl;
									//		for (int i = 0; i < no_routes; i++)
									//		{
									//			
									//			TEMP<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
									//			for (int j = 0; j <= saiz[i]+1; j++)
									//			{
									//				TEMP<<VEHICLE[i][j]<<' ';
									//			}
									//			TEMP<<endl;
									//		}
									//		TEMP<<"r1= "<<k<<' '<<"r2= "<<m<<' '<<"r1_without= "<<j<<' '<<"r2 with= "<<p<<' '<<"r2without= "<<n<<' '<<"r1 with= "<<q<<endl;
									//		
									//		//find cost of removing 
									//		
									//		for (int i = 0; i < no_routes; i++)
									//		{
									//			for (int j = 1; j <= saiz[i]; j++)
									//			{
									//				float COF = dist[VEHICLE[i][j-1]][VEHICLE[i][j]] + dist[VEHICLE[i][j]][VEHICLE[i][j+1]] + service_time[VEHICLE[i][j]] - dist[VEHICLE[i][j-1]][VEHICLE[i][j+1]];//to check 
									//				if (abs (COF - cost_of_removing[VEHICLE[i][j]])> 0.2)
									//					cout<<"Calculated COF= "<<COF<< ' '<<' '<<cost_of_removing[VEHICLE[i][j]]<<endl;
									//			}
									info_1_0[(int)gain_matrix_1_0[m][k]][11] = origain1 - oriloss1; //gain1 in distance
									info_1_0[(int)gain_matrix_1_0[m][k]][12] = origain2 - oriloss2; //gain2 in distance
									//		}
									//		getchar();
									//	}
									//	
									//}//end of to check

								}//end if 
							}//end if (k < m)
							else if (k > m)
							{
								if (gain-gain_matrix_1_0[m][k] > epsilon)//gain > gain_matrix_1_0[m][k]
								{

									gain_matrix_1_0[m][k] = gain;

									//info_1_0[gain_matrix_1_0[k][m]][0] = gain_matrix_1_0[k][m];
									info_1_0[(int)gain_matrix_1_0[k][m]][1] = gain;
									info_1_0[(int)gain_matrix_1_0[k][m]][2] = k;
									info_1_0[(int)gain_matrix_1_0[k][m]][3] = m;
									info_1_0[(int)gain_matrix_1_0[k][m]][4] = j;//r1 without
									info_1_0[(int)gain_matrix_1_0[k][m]][5] = p;//r2 with
									info_1_0[(int)gain_matrix_1_0[k][m]][6] = n;//r2 without
									info_1_0[(int)gain_matrix_1_0[k][m]][7] = q;//r1 with
									info_1_0[(int)gain_matrix_1_0[k][m]][8] = 2;//2 is (1-1)
									info_1_0[(int)gain_matrix_1_0[k][m]][10] = cost_without_i  - cost_with_i; //gain1
									//ofstream TEMP("TO CHECK.txt", ios::app);
									//to check
									//if((j==q) && (p==n))
									//{
									//	float cost_without_i2 = dist[before_pos1][element1] + dist[element1][after_pos1] + service_time[element1] - dist[before_pos1][after_pos1];//to check 
									//	float cost_without_j2 = dist[before_pos2][element2] + dist[element2][after_pos2] + service_time[element2] - dist[before_pos2][after_pos2];//to check 
									//	if ((abs(cost_without_i2-cost_of_removing[VEHICLE[k][j]])>0.1) || (abs(cost_without_j2-cost_of_removing[VEHICLE[m][n]])>0.1))
									//	{
									//		cout<<"partialupdate_reoptimize_1_1 got problem!!"<<endl;
									//		TEMP<<cost_without_i2<<' '<<cost_of_removing[VEHICLE[k][j]]<<endl;
									//		TEMP<<cost_without_j2<<' '<<cost_of_removing[VEHICLE[m][n]]<<endl;
									//		for (int i = 0; i < no_routes; i++)
									//		{
									//			for (int j = 0; j <= saiz[i]+1; j++)
									//			{
									//				TEMP<<VEHICLE[i][j]<<' ';
									//			}
									//			TEMP<<endl;
									//		}
									//		TEMP<<"r1= "<<k<<' '<<"r2= "<<m<<' '<<"r1_without= "<<j<<' '<<"r2 with= "<<p<<' '<<"r2without= "<<n<<' '<<"r1 with= "<<q<<endl;
									//		//find cost of removing 
									//		
									//		for (int i = 0; i < no_routes; i++)
									//		{
									//			for (int j = 1; j <= saiz[i]; j++)
									//			{
									//				float COF = dist[VEHICLE[i][j-1]][VEHICLE[i][j]] + dist[VEHICLE[i][j]][VEHICLE[i][j+1]] + service_time[VEHICLE[i][j]] - dist[VEHICLE[i][j-1]][VEHICLE[i][j+1]];//to check 
									//				if (abs (COF - cost_of_removing[VEHICLE[i][j]])> 0.2)
									//					cout<<"Calculated COF= "<<COF<< ' '<<' '<<cost_of_removing[VEHICLE[i][j]]<<endl;
									//			}
									info_1_0[(int)gain_matrix_1_0[k][m]][11] = origain1 - oriloss1; //gain1 in distance
									info_1_0[(int)gain_matrix_1_0[k][m]][12] = origain2 - oriloss2; //gain2 in distance
									//		}
									//		getchar();
									//	}
									//
									//}//end of to check

								}//end if 
							}//end if (k > m)
						}//end for q
					}//end for p							
					delete[]tempCumDist2;
				}//end for n
			}//end for m
			delete[]tempCumDist1;
		}//end for j
	}//end for i 
	vector_r1.clear();
	vector_r1.shrink_to_fit();
	vector_r2.clear();
	vector_r2.shrink_to_fit();
	vector_r3.clear();
	vector_r3.shrink_to_fit();
}

//DONE
void partialupdate_reoptimize_2_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{
	int num_attributes = 13;//[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
	
	int element1, element1f, element2, before_ele1, before_ele2, after_ele1=-1, after_ele2=-1, before_pos1, before_pos2, after_pos1, after_pos2;
	//int element3, before_ele3, after_ele3;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_jRev, cost_with_j, gain;
	int reverseStatus;
	
	//NONEED REINITIALIZE, this will delete the previous found gain
	std::vector<int> vector_r1;
	std::vector<int> vector_r2;
	std::vector<int> vector_r3;
	int r=-1;

	///////////////////////////////=================================== SAME ROUTE =================================////////////////////////
	//for (int I = 0; I < no_routes; I++)
	//{
	//	if (RchangedStatus[I] == false)
	//		continue;
	//	else
	//		r = I;
	//	//if (I==0)
	//	//	r=r1;
	//	//else
	//	//	r=r2;
	//	//if ((saiz[r]==0)||(saiz[r]==1)||(saiz[r]==2)) //if it is empty route, saiz=1, or saiz=2
	//	if (saiz[r]<=2)	
	//		continue;
	//	//gain_matrix_1_0[r][r] = INT_MIN; //reiniatilize //changed this (make comment on 7MAC2015)
	//	//for (int u = 1; u < 9; u++)
	//	//{
	//	//	same_r_info[r][u] = -1;
	//	//}
	//	
	//	
	//	vector_r3.clear();
	//	//copy route in vector
	//	for (int i = 0; i < saiz[r] + 2; i++) //first and last is depot
	//	{
	//		vector_r3.push_back(VEHICLE[r][i]);
	//	}
	//
	//	for (int j = 1; j <= saiz[r]-1 ; j++) //from which position, take in pair, so until saiz-1
	//	{
	//		element1 = vector_r3[j];
	//		element2 = vector_r3[j+1];
	//		before_ele1 = vector_r3[j-1];
	//		after_ele2 = vector_r3[j+2];
	//		for (int k = 1; k <= saiz[r]; k++) //which position to swap with, take one customer
	//		{
	//			if ((j==k) || ((j+1) ==k) )
	//			{
	//				continue;
	//			}
	//			element3 = vector_r3[k];
	//			before_ele3 = vector_r3[k-1];
	//			after_ele3 = vector_r3[k+1];
	//			
	//			if ( (before_ele3 == SIZE || after_ele3 == SIZE) && (before_ele1 == SIZE || after_ele2 == SIZE) ) //if mn and ij depot
	//			{	
	//				if ( (NR_FLAG_DEPOT[element1][before_ele3] == false && NR_FLAG_DEPOT[element2][after_ele3] == false) || (NR_FLAG_DEPOT[element3][before_ele1] == false && NR_FLAG_DEPOT[element3][after_ele2] == false) )//for normal order
	//				{	if (NR_FLAG_DEPOT[element2][before_ele3] == false && NR_FLAG_DEPOT[element1][after_ele3] == false) //for reverse order
	//						continue;
	//				}
	//			}
	//			else if ((before_ele3 == SIZE) || (after_ele3 == SIZE) ) //if mn depot
	//			{
	//				if ( (NR_FLAG_DEPOT[element1][before_ele3] == false && NR_FLAG_DEPOT[element2][after_ele3] == false) || (NR_FLAG[element3][before_ele1] == false && NR_FLAG[element3][after_ele2] == false) )//for normal order
	//				{	if (NR_FLAG_DEPOT[element2][before_ele3] == false && NR_FLAG_DEPOT[element1][after_ele3] == false)//for reverse order
	//						continue;
	//				}
	//			}
	//			else if ((before_ele1 == SIZE) || (after_ele2 == SIZE) ) //if ij depot
	//			{
	//				if ( (NR_FLAG[element1][before_ele3] == false && NR_FLAG[element2][after_ele3] == false) || (NR_FLAG_DEPOT[element3][before_ele1] == false && NR_FLAG_DEPOT[element3][after_ele2] == false) )//for normal order
	//				{	if ( NR_FLAG[element2][before_ele3] == false && NR_FLAG[element1][after_ele3] == false)//for reverse order
	//						continue;
	//				}
	//			}
	//			else //if no depot
	//			{
	//				if ( (NR_FLAG[element1][before_ele3] == false && NR_FLAG[element2][after_ele3] == false) || (NR_FLAG[element3][before_ele1] == false && NR_FLAG[element3][after_ele2] == false) )//for normal order
	//				{	if ( NR_FLAG[element2][before_ele3] == false && NR_FLAG[element1][after_ele3] == false)//for reverse order
	//						continue;
	//				}
	//			}
	//	
	//			//same route, noneed to consider service_time[]
	//			if (k > j) //if single customer is after pair customer
	//			{
	//					if (after_ele2 == element3) //in consecutive, so minus 3 arcs, for eg: 4 2 3 7 5 1 (swap pair 2 3 with 7) 
	//					{
	//						cost_without_i = dist[before_ele1][element1] + dist[element2][after_ele2] + dist[element3][after_ele3]; //route_cost is global variable //minus 4 arcs 
	//						cost_with_i = dist[before_ele1][element3]  + dist[element3][element1] +dist[element2][after_ele3];
	//						cost_with_iReverse = dist[before_ele1][element3]  + dist[element3][element2] +dist[element1][after_ele3];
	//					}
	//			
	//					else 
	//					{
	//						cost_without_i = dist[before_ele1][element1] + dist[element2][after_ele2] + dist[before_ele3][element3] + dist[element3][after_ele3]; //route_cost is global variable //minus 4 arcs 
	//						cost_with_i = dist[before_ele1][element3] + dist[element3][after_ele2] +dist[before_ele3][element1] +dist[element2][after_ele3];
	//						cost_with_iReverse = dist[before_ele1][element3] + dist[element3][after_ele2] +dist[before_ele3][element2] +dist[element1][after_ele3];
	//					}
	//			}
	//			else //if single customer is before pair customer
	//			{
	//				if(element3 == before_ele1) //in consecutive, so minus 3 arcs, for eg: 4 2 3 7 5 1 (swap 2 with pair 3 7) 
	//				{
	//					cost_without_i = dist[before_ele3][element3] + dist[before_ele1][element1] + dist[element2][after_ele2];
	//					cost_with_i = dist[before_ele3][element1] + dist[element2][element3] + dist[element3][after_ele2];
	//					cost_with_iReverse = dist[before_ele3][element2] + dist[element1][element3] + dist[element3][after_ele2];
	//				}
	//				else //all other sequence, so minus 4 arcs， for eg: 4 2 3 7 5 1 (swap 4 with pair 7 5) 
	//				{
	//					cost_without_i =  dist[before_ele3][element3] + dist[element3][after_ele3] +dist[before_ele1][element1] +dist[element2][after_ele2];
	//					cost_with_i = dist[before_ele3][element1] + dist[element2][after_ele3] +dist[before_ele1][element3] +dist[element3][after_ele2];
	//					cost_with_iReverse = dist[before_ele3][element2] + dist[element1][after_ele3] +dist[before_ele1][element3] +dist[element3][after_ele2];
	//				}
	//			}
	//			reverseStatus = 0; //initialize to 0
	//			if (cost_with_iReverse - cost_with_i < -epsilon)
	//			{
	//				cost_with_i = cost_with_iReverse;
	//				reverseStatus = 1;
	//			}
	//			gain = cost_without_i - cost_with_i; //gain = old-new
	//			//if (abs(gain) > distance_available[r])//gain is negative, change to positive using abs//if negative gain, it will  be omitted actually
	//			//{
	//			//	continue;
	//			//}
	//			if (gain-gain_matrix_1_0[r][r] > epsilon)//gain > gain_matrix_1_0[r][r]
	//			{
	//				gain_matrix_1_0[r][r] = gain;
	//				same_r_info[r][0] = r;
	//				same_r_info[r][1] = gain;
	//				same_r_info[r][2] = r; //from route
	//				same_r_info[r][3] = r; //to route, i=m here
	//				same_r_info[r][4] = j; //from which position
	//				same_r_info[r][5] = k; //this is exactly the position to be swapped !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! insertion need to be careful
	//				same_r_info[r][6] = -1; 
	//				same_r_info[r][7] = -1;
	//				same_r_info[r][8] = 3; //3 is (2-1)
	//				same_r_info[r][9] = reverseStatus; 
	//				same_r_info[r][10] = gain; //gain1
	//			}	
	//		}
	//		
	//	}
	//	vector_r3.clear();
	//}

	//////////////////////////////////////// ====================== DIFFERENT ROUTES =========================================////////////////////////////
	int k = -1;
	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//============================================(From) Affected routes: R1 and R2 to other routes (2-1)===========================================================
	for (int I = 0; I < no_routes; I++)  //consider each and other routes (from R1)
	{
		if (RchangedStatus[I] == false)
			continue;
		else
			k = I;

		//if (I == 0)
		//	k = r1;

		//else
		//	k = r2;

		if ((saiz[k]==0) || (saiz[k]==1)) //if it is empty route
			continue;
		for (int j = 1; j < saiz[k]; j++)  // consider each customer (from R1), start from 1 because [0] is depot //customer considered in pair, from 1 until saiz-1
		{
			//int space_r1 = space_available[i];
			element1 = VEHICLE[k][j];			//elemet to be inserted into other route
			element1f = VEHICLE[k][j + 1];
			before_ele1 = VEHICLE[k][j-1];
			after_ele1 = VEHICLE[k][j+2];

			float origain1 = dist[element1][before_ele1] + dist[element1][element1f] + dist[element1f][after_ele1] + service_time[element1]+ service_time[element1f]- dist[before_ele1][after_ele1]; //cost of r1 without i and i+1//(old + service_time – new)
			float *tempCumDist1= new float[SIZE];
			tempCumDist1[0] = 0;
			for (int t = 1; t <= saiz[k]-2; t++)//have 2 size less because 2 customers are deleted
			{
				if (t<j)
					tempCumDist1[t]= CumDist[k][t];//before the delete pos, the CumDist is the same
				else if(t>=j)
					tempCumDist1[t]=CumDist[k][t+2]-origain1;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain		
			}

			for (int m = 0; m < no_routes; m++)  //consider each and other routes (from R2)
			{
				if ((k == m)|| (saiz[m]==0))  //|| (demand[element2] > space_r1)) //if customer already in the route or demand exceed avaialable space in route
				{
					continue;   //to make it faster get out of loop
				}

				for (int n = 1; n < saiz[m] + 1; n++) //which customer in r2 to remove
				{
					element2 = VEHICLE[m][n];			//elemet to be removed from r2
					before_ele2 = VEHICLE[m][n-1];
					after_ele2 = VEHICLE[m][n+1];

					int space_r1 = space_available[k];
					int space_r2 = space_available[m];
					space_r1 = space_r1 + demand[element1] + demand[element1f];
					space_r2 = space_r2 + demand[element2];
					
					if ((demand[element2] > space_r1) || (demand[element1] + demand[element1f] > space_r2))
						continue; //to go to next element2 to delete
					
					float origain2 = dist[element2][before_ele2] + dist[element2][after_ele2] + service_time[element2] - dist[before_ele2][after_ele2];//old-new positive is good
					float *tempCumDist2= new float[SIZE];
					tempCumDist2[0] = 0;
					for (int t = 1; t <= saiz[m]-1; t++)//have 1 size less because 1 customer is deleted
					{
						if (t<n)
							tempCumDist2[t]= CumDist[m][t];//before the delete pos, the CumDist is the same
						else if(t>=n)
							tempCumDist2[t]=CumDist[m][t+1]-origain2;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
					
					}
					//==================================================copy r1 without customer j and j+1=================================================================//
					vector_r1.clear();
					for (int a = 0; a < saiz[k] + 2; a++)
					{
						if ((a == j) || (a == j+1))
							continue;
						vector_r1.push_back(VEHICLE[k][a]);
					}
					//vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot
					//vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot
								
					//==================================================copy r2 without customer n=================================================================//
					vector_r2.clear();
					for (int a = 0; a < saiz[m] + 2; a++)
					{
						if (a == n)
							continue;
						vector_r2.push_back(VEHICLE[m][a]);

					}
					//vector_r2.erase(vector_r2.begin() + n); //first one is depot

					float available_dist_r1 = distance_available[k];
					float available_dist_r2 = distance_available[m];
			
					//############################################## To Insert ##########################################################////////////////////
					for (int p = 1; p < vector_r2.size(); p++) //from r1, insert to r2
					{
						before_pos2 = vector_r2[p - 1];
						after_pos2 = vector_r2[p];  //noneed to add 1 because it is insert in between n-1 and n
						if ( before_pos2 == SIZE || after_pos2 == SIZE ) //if mn depot //changed this to here on 25JUN2015!!!!!!!!!!!!!!!!!!!!! OMG!!! before that put after n loop
						{
							if (NR_FLAG_DEPOT[element1][before_pos2] == false && NR_FLAG_DEPOT[element1f][after_pos2] == false) //original order
							{	if (NR_FLAG_DEPOT[element1f][before_pos2] == false && NR_FLAG_DEPOT[element1][after_pos2] == false)//reverse order
									continue;
							}
						}
						else //if no depot
						{
							if (NR_FLAG[element1][before_pos2] == false && NR_FLAG[element1f][after_pos2] == false)//original order
							{	if (NR_FLAG[element1f][before_pos2] == false && NR_FLAG[element1][after_pos2] == false)//reverse order
									continue;
							}
						}
						for (int q = 1; q < vector_r1.size(); q++) //from r2, insert to r1
						{
							reverseStatus = 0; //initialize to 0
							before_pos1 = vector_r1[q - 1];
							after_pos1 = vector_r1[q];  //noneed to add 1 because it is insert in between n-1 and n
							if ( before_pos1 == SIZE || after_pos1 == SIZE ) //if ij depot
							{	
								if (NR_FLAG_DEPOT[element2][before_pos1] == false && NR_FLAG_DEPOT[element2][after_pos1] == false)
									continue;
							}
							
							else //if no depot
							{
								if (NR_FLAG[element2][before_pos1] == false && NR_FLAG[element2][after_pos1] == false)
									continue;
							}

							//before_ele1 = VEHICLE[k][j - 1];   //initially, unless stated otherwise
							//after_ele1 = VEHICLE[k][j + 2];
							//before_ele2 = VEHICLE[m][n - 1];   //initially, unless stated otherwise
							//after_ele2 = VEHICLE[m][n + 1];

							//R1 (remove 2, insert 1)
							int remainingcust = saiz[k]-(j+1);//always take the second customer j+1
							if (remainingcust < 0)//if negative
								remainingcust = 0;
							float oriCumDist1 = CumDist[k][j];
							float oriCumDist2 = CumDist[k][j+1];
							cost_without_i = oriCumDist1 +  oriCumDist2 + remainingcust*origain1;
							
							float oriloss1 = dist[before_pos1][element2] + dist[element2][after_pos1] + service_time[element2] - dist[before_pos1][after_pos1]; //insert ele2 to r1
							float CumAtEle1 = tempCumDist1[q-1] + dist[before_pos1][element2] + service_time[element2]; 
							int remainingCustafterEle1 = saiz[k] - q -1;//(saiz-pos-1) //minus 1 more because two customers have been deleted 
							if (remainingCustafterEle1 < 0)//if negative
								remainingCustafterEle1 = 0;
							cost_with_i = CumAtEle1 + remainingCustafterEle1*oriloss1; 


							//R2 (remove 1, insert 2)
							cost_without_j = cost_of_removing[VEHICLE[m][n]];//cost of r2 without j
							
							float oriloss2 = (dist[before_pos2][element1] + dist[element1][element1f] + dist[element1f][after_pos2]+ service_time[element1]+ service_time[element1f]) - (dist[before_pos2][after_pos2]); //cost of r2 with i and i+1 //new-old
							float oriloss2Rev = (dist[before_pos2][element1f] + dist[element1f][element1] + dist[element1][after_pos2]+ service_time[element1]+ service_time[element1f]) - (dist[before_pos2][after_pos2]); //cost of r2 with i and i+1 //new-old

							float newCumDist1 = tempCumDist2[p-1] + dist[before_pos2][element1] + service_time[element1]; 
							float newCumDist2 = newCumDist1 + dist[element1][element1f] + service_time[element1f];
							float newCumDist1Rev = tempCumDist2[p-1] + dist[before_pos2][element1f] + service_time[element1f]; 
							float newCumDist2Rev = newCumDist1Rev + dist[element1f][element1] + service_time[element1];

							int remainingCustafterEle = saiz[m] - p;//(saiz - pos)

							if (remainingCustafterEle < 0)//if negative
									remainingCustafterEle = 0;
							cost_with_j = newCumDist1 + newCumDist2 + remainingCustafterEle*oriloss2; 
							cost_with_jRev = newCumDist1Rev + newCumDist2Rev + remainingCustafterEle*oriloss2Rev; 

							if (cost_with_jRev < cost_with_j)//if reverse is better
							{
								cost_with_j = cost_with_jRev;
								oriloss2 = oriloss2Rev;
								reverseStatus = 1;
							}
							//cost_with_i is R1 //cost_with_j is R2 
							if ((oriloss1 - origain1 > available_dist_r1 + epsilon) || (oriloss2 - origain2 > available_dist_r2 + epsilon))
							{
								continue;
							}
							gain = (cost_without_i + cost_without_j) - (cost_with_i + cost_with_j);
							
							if (k < m)
							{
								if (gain-gain_matrix_1_0[k][m] > epsilon)//gain > gain_matrix_1_0[k][m]
								{

									gain_matrix_1_0[k][m] = gain;
									//info[number_matrix_1_0[m][k]][0] = number_matrix_1_0[m][k];
									info_1_0[(int)gain_matrix_1_0[m][k]][1] = gain;
									info_1_0[(int)gain_matrix_1_0[m][k]][2] = k;
									info_1_0[(int)gain_matrix_1_0[m][k]][3] = m;
									info_1_0[(int)gain_matrix_1_0[m][k]][4] = j;
									info_1_0[(int)gain_matrix_1_0[m][k]][5] = p;
									info_1_0[(int)gain_matrix_1_0[m][k]][6] = n;
									info_1_0[(int)gain_matrix_1_0[m][k]][7] = q;
									info_1_0[(int)gain_matrix_1_0[m][k]][8] = 3; //3 is (2-1)
									info_1_0[(int)gain_matrix_1_0[m][k]][9] = reverseStatus; 
									info_1_0[(int)gain_matrix_1_0[m][k]][10] = cost_without_i  - cost_with_i; //gain1
									info_1_0[(int)gain_matrix_1_0[m][k]][11] = origain1  - oriloss1; //gain1 in distance
									info_1_0[(int)gain_matrix_1_0[m][k]][12] = origain2  - oriloss2; //gain2  in distance

								}//end if
							}

							else if (k > m)
							{
								if (gain-gain_matrix_1_0[m][k] > epsilon)//gain > gain_matrix_1_0[m][k]/////////////////////////////CHANGED on 1 OCT 2014!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
								{

									gain_matrix_1_0[m][k] = gain;
									//info[number_matrix_1_0[k][m]][0] = number_matrix_1_0[m][k];
									info_1_0[(int)gain_matrix_1_0[k][m]][1] = gain;
									info_1_0[(int)gain_matrix_1_0[k][m]][2] = k;
									info_1_0[(int)gain_matrix_1_0[k][m]][3] = m;
									info_1_0[(int)gain_matrix_1_0[k][m]][4] = j;
									info_1_0[(int)gain_matrix_1_0[k][m]][5] = p;
									info_1_0[(int)gain_matrix_1_0[k][m]][6] = n;
									info_1_0[(int)gain_matrix_1_0[k][m]][7] = q;
									info_1_0[(int)gain_matrix_1_0[k][m]][8] = 3;//3 is (2-1)
									info_1_0[(int)gain_matrix_1_0[k][m]][9] = reverseStatus; 
									info_1_0[(int)gain_matrix_1_0[k][m]][10] = cost_without_i  - cost_with_i; //gain1
									info_1_0[(int)gain_matrix_1_0[k][m]][11] = origain1  - oriloss1; //gain1 in distance
									info_1_0[(int)gain_matrix_1_0[k][m]][12] = origain2  - oriloss2; //gain2  in distance
								}//end if
							}
						}//end for q
					}//end for p	
					delete[] tempCumDist2;
				}//end for n
			}//end for m
			delete[] tempCumDist1;
		}//end for j
	}//end for i 
	//============================================(To) From other routes to Affected routes: R1 and R2 (1-2) 1 from affected route, 2 from other route ===========================================================
	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from R1)
	{
		//if ((saiz[i]==0) || (saiz[i]==1)) //if it is empty route or saiz =1
		if (saiz[i]<=1)
			continue;
		for (int j = 1; j < saiz[i]; j++)  // consider each customer (from R1), start from 1 because [0] is depot
		{
			//int space_r1 = space_available[i];
			element1 = VEHICLE[i][j];			//elemet to be inserted into other route
			element1f = VEHICLE[i][j + 1];
			before_ele1 = VEHICLE[i][j-1];
			after_ele1 = VEHICLE[i][j+2];
			float origain1 = dist[element1][before_ele1] + dist[element1][element1f] + dist[element1f][after_ele1] + service_time[element1]+ service_time[element1f]- dist[before_ele1][after_ele1]; //cost of r1 without i and i+1//(old + service_time – new)
			float *tempCumDist1= new float[SIZE];
			tempCumDist1[0] = 0;
			for (int t = 1; t <= saiz[i]-2; t++)//have 2 size less because 2 customers are deleted
			{
				if (t<j)
					tempCumDist1[t]= CumDist[i][t];//before the delete pos, the CumDist is the same
				else if(t>=j)
					tempCumDist1[t]=CumDist[i][t+2]-origain1;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
					
			}

			int m=-1;
			for (int I = 0; I < no_routes; I++)  //consider each and other routes (from R2)
			{
				if (RchangedStatus[I] == false)
					continue;
				else
					m = I;
				//int m;
				//if (I == 0)
				//	m = r1;

				//else
				//	m = r2;

				if ((i == m)||(saiz[m]==0)) // || (demand[element2] > space_available[m])) //if customer already in the route or demand exceed avaialable space in route
				{
					continue;
					//m++;
					//goto mylabel;
				}
				for (int n = 1; n < saiz[m] + 1; n++) //which customer in r2 to remove
				{
					element2 = VEHICLE[m][n];			//elemet to be removed from r2
					before_ele2 = VEHICLE[m][n-1];
					after_ele2 = VEHICLE[m][n+1];

					int space_r1 = space_available[i];//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					int space_r2 = space_available[m];
					space_r1 = space_r1 + demand[element1] + demand[element1f];
					space_r2 = space_r2 + demand[element2];

					if ((demand[element1] + demand[element1f] > space_r2) || (demand[element2] > space_r1))
						continue; //go to next elementt2 to delete

					float origain2 = dist[element2][before_ele2] + dist[element2][after_ele2] + service_time[element2] - dist[before_ele2][after_ele2];//old-new positive is good
					float *tempCumDist2= new float[SIZE];
					tempCumDist2[0] = 0;
					for (int t = 1; t <= saiz[m]-1; t++)//have 1 size less because 1 customer is deleted
					{
						if (t<n)
							tempCumDist2[t]= CumDist[m][t];//before the delete pos, the CumDist is the same
						else if(t>=n)
							tempCumDist2[t]=CumDist[m][t+1]-origain2;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
					
					}

					//==================================================copy r1 without customer j and j+1=================================================================//
					vector_r1.clear();
					for (int a = 0; a < saiz[i] + 2; a++)
					{
						if ((a == j) || (a == j+1))
							continue;
						vector_r1.push_back(VEHICLE[i][a]);
					}
					//vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot
					//vector_r1.erase(vector_r1.begin() + j); //delete next element
					
					//==================================================copy r2 without customer n=================================================================//
					vector_r2.clear();
					for (int a = 0; a < saiz[m] + 2; a++)
					{
						if (a == n)
							continue;
						vector_r2.push_back(VEHICLE[m][a]);

					}
					//vector_r2.erase(vector_r2.begin() + n); //first one is depot
					
					float available_dist_r1 = distance_available[i];
					float available_dist_r2 = distance_available[m];


					//############################################## To insert ##########################################################////////////////////
					for (int p = 1; p < vector_r2.size(); p++) //from r1, insert to r2
					{
						before_pos2 = vector_r2[p - 1];
						after_pos2 = vector_r2[p];  //noneed to add 1 because it is insert in between n-1 and n
						if ( before_pos2 == SIZE || after_pos2 == SIZE ) //if mn depot //changed this to here on 25JUN2015!!!!!!!!!!!!!!!!!!!!! OMG!!! before that put after n loop
						{
							if (NR_FLAG_DEPOT[element1][before_pos2] == false && NR_FLAG_DEPOT[element1f][after_pos2] == false)//original order
								if (NR_FLAG_DEPOT[element1f][before_pos2] == false && NR_FLAG_DEPOT[element1][after_pos2] == false)//reverse order
									continue;
						}
						else //if no depot
						{
							if (NR_FLAG[element1][before_pos2] == false && NR_FLAG[element1f][after_pos2] == false)//original order
								if (NR_FLAG[element1f][before_pos2] == false && NR_FLAG[element1][after_pos2] == false)//reverse order
									continue;
						}
						for (int q = 1; q < vector_r1.size(); q++) //from r2, insert to r1
						{
							reverseStatus = 0; //initialize to 0
							before_pos1 = vector_r1[q - 1];
							after_pos1 = vector_r1[q];  //noneed to add 1 because it is insert in between n-1 and n
							if ( before_pos1 == SIZE  || after_pos1 == SIZE ) //if ij depot
							{	
								if (NR_FLAG_DEPOT[element2][before_pos1] == false && NR_FLAG_DEPOT[element2][after_pos1] == false)
									continue;
							}
							
							else //if no depot
							{
								if (NR_FLAG[element2][before_pos1] == false && NR_FLAG[element2][after_pos1] == false)
									continue;
							}
					
							//before_ele1 = VEHICLE[i][j - 1];   //initially, unless stated otherwise
							//after_ele1 = VEHICLE[i][j + 2];
							//before_ele2 = VEHICLE[m][n - 1];   //initially, unless stated otherwise
							//after_ele2 = VEHICLE[m][n + 1];

							//R1 (remove 2, insert 1)
							int remainingcust = saiz[i]-(j+1);//always take the second customer j+1
							if (remainingcust < 0)//if negative
								remainingcust = 0;
							float oriCumDist1 = CumDist[i][j];
							float oriCumDist2 = CumDist[i][j+1];
							cost_without_i = oriCumDist1 +  oriCumDist2 + remainingcust*origain1;
							
							float oriloss1 = dist[before_pos1][element2] + dist[element2][after_pos1] + service_time[element2] - dist[before_pos1][after_pos1]; //insert ele2 to r1
							float CumAtEle1 = tempCumDist1[q-1] + dist[before_pos1][element2] + service_time[element2]; 
							int remainingCustafterEle1 = saiz[i] - q-1;//(saiz-pos-1) //minus 1 more because two customers have been deleted 
							if (remainingCustafterEle1 < 0)//if negative
								remainingCustafterEle1 = 0;
							cost_with_i = CumAtEle1 + remainingCustafterEle1*oriloss1; 


							//R2 (remove 1, insert 2)
							cost_without_j = cost_of_removing[VEHICLE[m][n]];//cost of r2 without j
							
							float oriloss2 = (dist[before_pos2][element1] + dist[element1][element1f] + dist[element1f][after_pos2]+ service_time[element1]+ service_time[element1f]) - (dist[before_pos2][after_pos2]); //cost of r2 with i and i+1 //new-old
							float oriloss2Rev = (dist[before_pos2][element1f] + dist[element1f][element1] + dist[element1][after_pos2]+ service_time[element1]+ service_time[element1f]) - (dist[before_pos2][after_pos2]); //cost of r2 with i and i+1 //new-old

							float newCumDist1 = tempCumDist2[p-1] + dist[before_pos2][element1] + service_time[element1]; 
							float newCumDist2 = newCumDist1 + dist[element1][element1f] + service_time[element1f];
							float newCumDist1Rev = tempCumDist2[p-1] + dist[before_pos2][element1f] + service_time[element1f]; 
							float newCumDist2Rev = newCumDist1Rev + dist[element1f][element1] + service_time[element1];

							int remainingCustafterEle = saiz[m] - p;//(saiz - pos)

							if (remainingCustafterEle < 0)//if negative
									remainingCustafterEle = 0;
							cost_with_j = newCumDist1 + newCumDist2 + remainingCustafterEle*oriloss2; 
							cost_with_jRev = newCumDist1Rev + newCumDist2Rev + remainingCustafterEle*oriloss2Rev; 

							if (cost_with_jRev < cost_with_j)//if reverse is better
							{
								cost_with_j = cost_with_jRev;
								oriloss2 = oriloss2Rev; 
								reverseStatus = 1;
							}
							//cost_with_i is R1 //cost_with_j is R2 
							if ((oriloss1 - origain1 > available_dist_r1 + epsilon) || (oriloss2 - origain2 > available_dist_r2 + epsilon))
							{
								continue;
							}
							gain = (cost_without_i + cost_without_j) - (cost_with_i + cost_with_j);

							if (i < m)
							{
								if (gain-gain_matrix_1_0[i][m] > epsilon)//gain > gain_matrix_1_0[i][m]
								{
									gain_matrix_1_0[i][m] = gain;

									//info[number_matrix_1_0[i][m]][0] = gain_matrix_1_0[m][i];
									info_1_0[(int)gain_matrix_1_0[m][i]][1] = gain;
									info_1_0[(int)gain_matrix_1_0[m][i]][2] = i;
									info_1_0[(int)gain_matrix_1_0[m][i]][3] = m;
									info_1_0[(int)gain_matrix_1_0[m][i]][4] = j;
									info_1_0[(int)gain_matrix_1_0[m][i]][5] = p;
									info_1_0[(int)gain_matrix_1_0[m][i]][6] = n;
									info_1_0[(int)gain_matrix_1_0[m][i]][7] = q;
									info_1_0[(int)gain_matrix_1_0[m][i]][8] = 3; //3 is (2-1)
									info_1_0[(int)gain_matrix_1_0[m][i]][9] = reverseStatus; 
									info_1_0[(int)gain_matrix_1_0[m][i]][10] = cost_without_i - cost_with_i;//gain1
									info_1_0[(int)gain_matrix_1_0[m][i]][11] = origain1 - oriloss1;//gain1 in distance
									info_1_0[(int)gain_matrix_1_0[m][i]][12] = origain2 - oriloss2;//gain2 in distance

								}//end if
							}//end if (i < m)
							else if (i > m)
							{
								if (gain-gain_matrix_1_0[m][i] > epsilon)//gain > gain_matrix_1_0[m][i]
								{
									gain_matrix_1_0[m][i] = gain;

									//info[number_matrix_2_1[i][m]][0] = number_matrix_2_1[i][m];
									info_1_0[(int)gain_matrix_1_0[i][m]][1] = gain;
									info_1_0[(int)gain_matrix_1_0[i][m]][2] = i;
									info_1_0[(int)gain_matrix_1_0[i][m]][3] = m;
									info_1_0[(int)gain_matrix_1_0[i][m]][4] = j;
									info_1_0[(int)gain_matrix_1_0[i][m]][5] = p;
									info_1_0[(int)gain_matrix_1_0[i][m]][6] = n;
									info_1_0[(int)gain_matrix_1_0[i][m]][7] = q;
									info_1_0[(int)gain_matrix_1_0[i][m]][8] = 3; //3 is (2-1)
									info_1_0[(int)gain_matrix_1_0[i][m]][9] = reverseStatus;
									info_1_0[(int)gain_matrix_1_0[i][m]][10] = cost_without_i - cost_with_i;//gain1
									info_1_0[(int)gain_matrix_1_0[i][m]][11] = origain1 - oriloss1;//gain1 in distance
									info_1_0[(int)gain_matrix_1_0[i][m]][12] = origain2 - oriloss2;//gain2 in distance

								}//end if
							}//end if (i > m)
						}//end for q
					}//end for p
					delete[] tempCumDist2;
				}//end for n
			}//end for m
			delete[] tempCumDist1;
		}//end for j
	}//end for i 
	
	vector_r1.clear();
	vector_r1.shrink_to_fit();
	vector_r2.clear();
	vector_r2.shrink_to_fit();
	vector_r3.clear();
	vector_r3.shrink_to_fit();
}

//DONE
void partialupdate_reoptimize_2_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{
	int num_attributes = 13;//[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
	
	int element1=-1, element2=-1, before_ele, after_ele, before_pos, after_pos;

	float gain;

	int from1, to1;
	float cost_without_i, cost_with_i, cost_with_iRev;
	//int r1 = route_change[0]; //route (from)
	//int r2 = route_change[1]; //route (to)
	int reverseStatus;
	
	//NONEED REINITIALIZE, this will delete the previous found gain

	int i = -1; //initilize for affected route
	//////////////////================================== SAME ROUTE =========================////////////////////////////////////
	for (int r = 0; r < no_routes; r++) 
	{
		if (RchangedStatus[r] == false)
			continue;
		else
			i = r;
		//if(r==0)
		//	i=r1;
		//else
		//	i=r2;
		//if ((saiz[i]==0) || (saiz[i]==1) || (saiz[i]==2))//if it is empty route, saiz=1, or saiz=2, nothing to move around
		if (saiz[i]<=2)	
			continue;
		for (int j = 1; j < saiz[i]; j++) //which pair to delete, from 1 to saiz-1
		{
			float available_dist_r2 = distance_available[i];
			//copy a temporary route
			int b=0;
			int* temp_r = new int[saiz[i]+2];
			for (int a = 0; a < saiz[i] + 2; a++)
			{
				if ((a == j)|| (a == j+1))
				{
					continue; //dont copy the elements, assume the elements to be deleted
				}

				else
				{
					temp_r[b] = VEHICLE[i][a];
					b++;
				}
			}
			element1 = VEHICLE[i][j];
			element2 = VEHICLE[i][j+1];
			before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
			after_ele = VEHICLE[i][j + 2];
			float origain = dist[element1][before_ele] + dist[element1][element2] + dist[element2][after_ele] - dist[before_ele][after_ele]; //cost of r1 without i and i+1 // old-new

			float *tempCumDist= new float[SIZE];
			tempCumDist[0] = 0;
			for (int t = 1; t <= saiz[i]-2; t++)//have 2 size less because 2 customers are deleted
			{
				if (t<j)
					tempCumDist[t]= CumDist[i][t];//before the delete pos, the CumDist is the same
				else if(t>=j)
					tempCumDist[t]=CumDist[i][t+2]-origain;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
					
			}
			for (int n = 1; n < saiz[i] ; n++) //has 2 positions less because they have been deleted
			{
				reverseStatus = 0;
				//element1 = VEHICLE[i][j];//must initialize again because elemnt1 and 2 might be reversed just now
				//element2 = VEHICLE[i][j+1];
				if (j == n) //originally from this position, so noneed to consider
					continue; 
				else
				{
					before_pos = temp_r[n - 1];
					after_pos = temp_r[n];  //noneed to add 1 because it is insert in between n-1 and n
				}

				if ((before_pos == SIZE) || (after_pos == SIZE) ) //if the insertion is between customer and depot
				{
					if ( NR_FLAG_DEPOT[element1][before_pos] == false && NR_FLAG_DEPOT[element2][after_pos] == false )//original order
					{	if ( NR_FLAG_DEPOT[element2][before_pos] == false && NR_FLAG_DEPOT[element1][after_pos] == false )//reverse order
							continue;
					}
				}

				else //if not between customer and depot
				{	
					if ( NR_FLAG[element1][before_pos] == false && NR_FLAG[element2][after_pos] == false )//original order
					{	if ( NR_FLAG[element2][before_pos] == false && NR_FLAG[element1][after_pos] == false )//reverse order
							continue;
					}
				}	
				
				//same route, noneed to consider service_time[]
				//First, delete the two elements
				//float origain = dist[element1][before_ele] + dist[element1][element2] + dist[element2][after_ele] - dist[before_ele][after_ele]; //cost of r1 without i and i+1 // old-new
				int remaining_cust = saiz[i] - (j+1);//always minus the position of the second customer j+1
				float oriCumDist1= CumDist[i][j];
				float oriCumDist2= CumDist[i][j+1];
				cost_without_i = oriCumDist1 + oriCumDist2 + remaining_cust*origain; //cost of r1 without i and i+1
				
				//float *tempCumDist= new float[SIZE];
				//tempCumDist[0] = 0;
				//for (int t = 1; t <= saiz[i]-2; t++)//have 2 size less because 2 customers are deleted
				//{
				//	if (t<j)
				//		tempCumDist[t]= CumDist[i][t];//before the delete pos, the CumDist is the same
				//	else if(t>=j)
				//		tempCumDist[t]=CumDist[i][t+2]-origain;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
				//	
				//}

				//Now, insert two elements

				float oriloss =  dist[before_pos][element1] + dist[element1][element2]+dist[element2][after_pos] + service_time[element1]+ service_time[element2] - dist[before_pos][after_pos]; //cost of r2 with i and i+1 //new-old , big not good, small is good
				float orilossRev =  dist[before_pos][element2] + dist[element2][element1]+dist[element1][after_pos] + service_time[element1]+ service_time[element2] - dist[before_pos][after_pos]; //cost of r2 with i+1 and i //new-old
				
				remaining_cust = saiz[i] - (n+1);//always minus the position of the second customer n+1
				if (remaining_cust < 0)//if negative
						remaining_cust = 0;

				float newCumDist1= tempCumDist[n-1] + dist[before_pos][element1] + service_time[element1]; 
				float newCumDist2= newCumDist1 + dist[element1][element2] + service_time[element2]; 
				float newCumDist1Rev= tempCumDist[n-1] + dist[before_pos][element2] + service_time[element2]; 
				float newCumDist2Rev= newCumDist1Rev + dist[element2][element1] + service_time[element1]; 

				cost_with_i = newCumDist1 + newCumDist2 + remaining_cust*oriloss;
				cost_with_iRev = newCumDist1Rev + newCumDist2Rev + remaining_cust*orilossRev;
				
				if(cost_with_iRev < cost_with_i)//if reverse has smaller cost
				{
					cost_with_i = cost_with_iRev;
					oriloss = orilossRev;
					reverseStatus = 1;
				}
				
				
				gain = cost_without_i - cost_with_i;
				if (gain > INFIN)
				{
						cout<<"Gain is too big, sth wrong"<<endl;
						getchar();
				}
				
				
					
				if (gain-gain_matrix_1_0[i][i] > epsilon)//gain > gain_matrix_1_0[i][i]
				{
					gain_matrix_1_0[i][i] = gain;
					same_r_info[i][0] = i;
					same_r_info[i][1] = gain;
					same_r_info[i][2] = i; //from route
					same_r_info[i][3] = i; //to route, i=m here
					same_r_info[i][4] = j; //from which position
					same_r_info[i][5] = n; //to which position //assume modified route, customer has been deleted
					same_r_info[i][6] = -1; 
					same_r_info[i][7] = -1;
					same_r_info[i][8] = 4; //4 is (2-0)
					same_r_info[i][9] = reverseStatus; 
					same_r_info[i][10] = gain; //gain1
					same_r_info[i][11] = origain - oriloss; //gain1 in distance
				}	
			}
			delete[]tempCumDist;
			delete[] temp_r;
		}//end of j

	}

	///////////////// ============================== DIFERENT ROUTES ======================================//////////////////////
	//######################################update gain matrix################################################//
	//from 2 routes to every position=======================update row============================= ///////////////////////////////////
	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from)
	{
		if (RchangedStatus[i] == false)
			continue;
		else
			from1 = i;
		//if (i == 0)
		//	from1 = r1;

		//else
		//	from1 = r2;
		
		//if ((saiz[from1]==0) || (saiz[from1]==1)) //if it is empty route, or saiz=1
		if (saiz[from1]<=1)
			continue;
		for (int j = 1; j < saiz[from1]; j++)  // consider each customer (from)//customer considered in pair, from 1 until saiz-1
		{
			element1 = VEHICLE[from1][j];			//element to be inserted into other route
			element2 = VEHICLE[from1][j+1];

			//cout << element;

			for (int m = 0; m < no_routes; m++)  //insert (to) which route
			{
				if ((m == from1) || (demand[element1]+demand[element2] > space_available[m])) //if customer already in the route or demand exceed avaialable space in route
				{
					continue;
				}

				float available_dist_r2 = distance_available[m];
	
				for (int n = 1; n <= saiz[m] + 1; n++) //insert (to) which position, from 0 to saiz + 1, insert at 1 means replace element [1] which is the first customer
				{
					reverseStatus = 0;
					//element1 = VEHICLE[from1][j];//must initialize again because elemnt1 and 2 might be reversed just now
					//element2 = VEHICLE[from1][j+1];
					before_ele = VEHICLE[from1][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[from1][j + 2];
					before_pos = VEHICLE[m][n - 1];
					after_pos = VEHICLE[m][n];  //noneed to add 1 because it is insert in between n-1 and n

					if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
					{
						if ( NR_FLAG_DEPOT[element1][before_pos] == false && NR_FLAG_DEPOT[element2][after_pos] == false )//original order
						{	if ( NR_FLAG_DEPOT[element2][before_pos] == false && NR_FLAG_DEPOT[element1][after_pos] == false )//reverse order
								continue;
								
						}
					}
					else //if not between customer and depot
					{	
						if ( NR_FLAG[element1][before_pos] == false && NR_FLAG[element2][after_pos] == false )//original order
						{	if ( NR_FLAG[element2][before_pos] == false && NR_FLAG[element1][after_pos] == false )//reverse order
								continue;
								
						}
					}	

					//for r1 delete customer
					float origain = (dist[element1][before_ele] + dist[element1][element2] + dist[element2][after_ele]+ service_time[element1]+ service_time[element2]) - (dist[before_ele][after_ele]); //cost of r1 without i and i+1 //old-new
					int remainingcust = saiz[from1]-(j+1);//always take the second customer j+1
					if (remainingcust < 0)//if negative
							remainingcust = 0;
					float oriCumDist1 = CumDist[from1][j];
					float oriCumDist2 = CumDist[from1][j+1];
					cost_without_i = oriCumDist1 +  oriCumDist2 + remainingcust*origain;
				
					//for r2, insert customer
					float oriloss = (dist[before_pos][element1] + dist[element1][element2] + dist[element2][after_pos]+ service_time[element1]+ service_time[element2]) - (dist[before_pos][after_pos]); //cost of r2 with i and i+1 //new-old
					float orilossRev = (dist[before_pos][element2] + dist[element2][element1] + dist[element1][after_pos]+ service_time[element1]+ service_time[element2]) - (dist[before_pos][after_pos]); //cost of r2 with i and i+1 //new-old

					float newCumDist1 = CumDist[m][n-1] + dist[before_pos][element1] + service_time[element1]; 
					float newCumDist2 = newCumDist1 + dist[element1][element2] + service_time[element2];

					float newCumDist1Rev = CumDist[m][n-1] + dist[before_pos][element2] + service_time[element2]; 
					float newCumDist2Rev = newCumDist1Rev + dist[element2][element1] + service_time[element1];

					int remainingCustafterEle = saiz[m] - n+1;//(saiz - pos+1)
					if (remainingCustafterEle < 0)//if negative
							remainingCustafterEle = 0;
					cost_with_i = newCumDist1 + newCumDist2 + remainingCustafterEle*oriloss; 
					cost_with_iRev = newCumDist1Rev + newCumDist2Rev + remainingCustafterEle*orilossRev; 

					if (cost_with_iRev < cost_with_i)//if reverse is better
					{
						cost_with_i = cost_with_iRev;
						oriloss = orilossRev;
						reverseStatus = 1;
					}

					if (oriloss - available_dist_r2 > epsilon) //only consider r2
					{
						continue;
					}

					gain = cost_without_i - cost_with_i;

					if (gain > INFIN)
					{
						cout<<"Gain is too big, sth wrong"<<endl;
						getchar();
					}

					if (from1 < m)
					{
						if (gain-gain_matrix_1_0[from1][m] > epsilon)//gain > gain_matrix_1_0[from1][m]
						{
							gain_matrix_1_0[from1][m] = gain;
							//info_1_0[gain_matrix_1_0[m][from1]][0] = gain_matrix_1_0[m][from1];
							info_1_0[(int)gain_matrix_1_0[m][from1]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[m][from1]][2] = from1;
							info_1_0[(int)gain_matrix_1_0[m][from1]][3] = m;
							info_1_0[(int)gain_matrix_1_0[m][from1]][4] = j;
							info_1_0[(int)gain_matrix_1_0[m][from1]][5] = n;
							info_1_0[(int)gain_matrix_1_0[m][from1]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[m][from1]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[m][from1]][8] = 4; //4 is (2-0)
							info_1_0[(int)gain_matrix_1_0[m][from1]][9] = reverseStatus;
							info_1_0[(int)gain_matrix_1_0[m][from1]][10] = cost_without_i; //gain1
							info_1_0[(int)gain_matrix_1_0[m][from1]][11] = origain;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[m][from1]][12] = -oriloss;//gain2 in distance
						}
					}

					else if (from1 > m)
					{
						if (gain-gain_matrix_1_0[m][from1] > epsilon)//gain > gain_matrix_1_0[m][from1]
						{
							gain_matrix_1_0[m][from1] = gain;
							//info_1_0[gain_matrix_1_0[m][from1]][0] = gain_matrix_1_0[from1][m];
							info_1_0[(int)gain_matrix_1_0[from1][m]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[from1][m]][2] = from1;
							info_1_0[(int)gain_matrix_1_0[from1][m]][3] = m;
							info_1_0[(int)gain_matrix_1_0[from1][m]][4] = j;
							info_1_0[(int)gain_matrix_1_0[from1][m]][5] = n;
							info_1_0[(int)gain_matrix_1_0[from1][m]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[from1][m]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[from1][m]][8] = 4; //4 is (2-0)
							info_1_0[(int)gain_matrix_1_0[from1][m]][9] = reverseStatus;
							info_1_0[(int)gain_matrix_1_0[from1][m]][10] = cost_without_i; //gain1
							info_1_0[(int)gain_matrix_1_0[from1][m]][11] = origain;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[from1][m]][12] = -oriloss;//gain2 in distance
						}
					}
				}//end for n
			}//end for m
		}//end for j
		//cout << i << ' '<< element << ' '<< old_route[i] << ' ' <<old_position[i]<< ' ' << ' '<< new_route[i] << ' ' << new_position[i] << ' '<< ' '<< gain_array[i] << endl;
	}//end for i 

	//######################################update gain matrix################################################//
	//from every position to 2 routes=======================update column============================= ///////////////////////////////////


	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from)
	{
		//if ((saiz[i]==0) || (saiz[i]==1) )//if it is empty route or saiz=1
		if (saiz[i]<=1)
			continue;
		for (int j = 1; j < saiz[i] ; j++)  // consider each customer (from) //customer considered in pair, from 1 until saiz-1
		{
			element1 = VEHICLE[i][j];			//element to be inserted into other route
			element2 = VEHICLE[i][j+1];			//element to be inserted into other route

			//cout << element;

			for (int m = 0; m < no_routes; m++)  //insert (to) which route
			{
				if (RchangedStatus[m] == false)
					continue;
				else
					to1 = m;

				//if (m == 0)
				//	to1 = r1;

				//else
				//	to1 = r2;

				float available_dist_r2 = distance_available[to1];
				if ((i == to1) || (demand[element1] + demand[element2] > space_available[to1])) //if demand exceed avaialable space in route && differnt route, if same route, capacity exceed stil have to find best gain
				{
					continue;
	
				}

				for (int n = 1; n < saiz[to1] + 2; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
				{
					reverseStatus = 0;
					//element1 = VEHICLE[i][j];//must initialize again because elemnt1 and 1f might be reversed just now
					//element2 = VEHICLE[i][j+1];
					before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[i][j + 2];
					before_pos = VEHICLE[to1][n - 1];
					after_pos = VEHICLE[to1][n];  //noneed to add 1 because it is insert in between n-1 and n

					if ( before_pos == SIZE || after_pos == SIZE  ) //if the insertion is between customer and depot
					{
						if ( NR_FLAG_DEPOT[element1][before_pos] == false && NR_FLAG_DEPOT[element2][after_pos] == false )//original order
						{	if ( NR_FLAG_DEPOT[element2][before_pos] == false && NR_FLAG_DEPOT[element1][after_pos] == false )//reverse order
								continue;
						}
					}
					else //if not between customer and depot
					{	
						if ( NR_FLAG[element1][before_pos] == false && NR_FLAG[element2][after_pos] == false )//original order
						{	if ( NR_FLAG[element2][before_pos] == false && NR_FLAG[element1][after_pos] == false )//reverse order
								continue;
						}
					}	
	
					//for r1 delete customer
					float origain = (dist[element1][before_ele] + dist[element1][element2] + dist[element2][after_ele]+ service_time[element1]+ service_time[element2]) - (dist[before_ele][after_ele]); //cost of r1 without i and i+1 //old-new
					int remainingcust = saiz[i]-(j+1);//always take the second customer j+1
					if (remainingcust < 0)//if negative
							remainingcust = 0;
					float oriCumDist1 = CumDist[i][j];
					float oriCumDist2 = CumDist[i][j+1];
					cost_without_i = oriCumDist1 +  oriCumDist2 + remainingcust*origain;
				
					//for r2, insert customer
					float oriloss = (dist[before_pos][element1] + dist[element1][element2] + dist[element2][after_pos]+ service_time[element1]+ service_time[element2]) - (dist[before_pos][after_pos]); //cost of r2 with i and i+1 //new-old
					float orilossRev = (dist[before_pos][element2] + dist[element2][element1] + dist[element1][after_pos]+ service_time[element1]+ service_time[element2]) - (dist[before_pos][after_pos]); //cost of r2 with i and i+1 //new-old

					float newCumDist1 = CumDist[to1][n-1] + dist[before_pos][element1] + service_time[element1]; 
					float newCumDist2 = newCumDist1 + dist[element1][element2] + service_time[element2];

					float newCumDist1Rev = CumDist[to1][n-1] + dist[before_pos][element2] + service_time[element2]; 
					float newCumDist2Rev = newCumDist1Rev + dist[element2][element1] + service_time[element1];

					int remainingCustafterEle = saiz[to1] - n+1;//(saiz - pos+1)
					if (remainingCustafterEle < 0)//if negative
							remainingCustafterEle = 0;

					cost_with_i = newCumDist1 + newCumDist2 + remainingCustafterEle*oriloss; 
					cost_with_iRev = newCumDist1Rev + newCumDist2Rev + remainingCustafterEle*orilossRev; 


					if (cost_with_iRev < cost_with_i)//if reverse is better
					{
						cost_with_i = cost_with_iRev;	
						oriloss = orilossRev;
						reverseStatus = 1;
					}
					

					if (oriloss - available_dist_r2 > epsilon) //only consider r2
					{
						continue;
					}

					gain = cost_without_i - cost_with_i;
					if (gain > INFIN)
					{
						cout<<"Gain is too big, sth wrong"<<endl;
						getchar();
					}


					if (i < to1)
					{
						if (gain-gain_matrix_1_0[i][to1] > epsilon)//gain > gain_matrix_1_0[i][to1]
						{
							gain_matrix_1_0[i][to1] = gain;
							//info_1_0[(int)gain_matrix_1_0[i][to1]][0] = gain_matrix_1_0[to1][i];
							info_1_0[(int)gain_matrix_1_0[to1][i]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[to1][i]][2] = i; //from_route
							info_1_0[(int)gain_matrix_1_0[to1][i]][3] = to1; //to_route
							info_1_0[(int)gain_matrix_1_0[to1][i]][4] = j; //from_position
							info_1_0[(int)gain_matrix_1_0[to1][i]][5] = n; //to_position in other route
							info_1_0[(int)gain_matrix_1_0[to1][i]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[to1][i]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[to1][i]][8] = 4; //4 is (2-0)
							info_1_0[(int)gain_matrix_1_0[to1][i]][9] = reverseStatus;
							info_1_0[(int)gain_matrix_1_0[to1][i]][10] = cost_without_i;
							info_1_0[(int)gain_matrix_1_0[to1][i]][11] = origain;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[to1][i]][12] = -oriloss;//gain2 in distance
						}
					}
					else if (i > to1)
					{
						if (gain-gain_matrix_1_0[to1][i] > epsilon)//gain > gain_matrix_1_0[to1][i]
						{
							gain_matrix_1_0[to1][i] = gain;
							//info_1_0[(int)gain_matrix_1_0[to1][i]][0] = gain_matrix_1_0[i][to1];
							info_1_0[(int)gain_matrix_1_0[i][to1]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[i][to1]][2] = i;
							info_1_0[(int)gain_matrix_1_0[i][to1]][3] = to1;
							info_1_0[(int)gain_matrix_1_0[i][to1]][4] = j;
							info_1_0[(int)gain_matrix_1_0[i][to1]][5] = n;
							info_1_0[(int)gain_matrix_1_0[i][to1]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[i][to1]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[i][to1]][8] = 4; //4 is (2-0)
							info_1_0[(int)gain_matrix_1_0[i][to1]][9] = reverseStatus;
							info_1_0[(int)gain_matrix_1_0[i][to1]][10] = cost_without_i;
							info_1_0[(int)gain_matrix_1_0[i][to1]][11] = origain;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[i][to1]][12] = -oriloss;//gain2 in distance
						}
					}
				}//end for n
			}//end for m
		}//end for j
	}
}

//DONE
void partialupdate_reoptimize_2_2swap(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{
	int num_attributes = 13;//[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
	
	int element1, element1f, element2, element2f, before_ele1, before_ele2, after_ele1=-1, after_ele2=-1;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_j, cost_with_i_Reverse, cost_with_j_Reverse, cost_with_both_Reverse, gain, dist_gain;
	float dist_without_i, dist_without_j, dist_with_i, dist_with_j, dist_with_i_Reverse, dist_with_j_Reverse, dist_with_both_Reverse, gain_with_i_Reverse, gain_with_j_Reverse, gain_with_both_Reverse;
	int r=-1;
	int temp = -1;
	int reverseType = 0;//initialize zero means no reverse for both
	//////////////////////////////////============================= SAME ROUTE ===============================================//////////////////////
	for (int I = 0; I < no_routes; I++)
	{
		if (RchangedStatus[I] == false)
			continue;
		else
			r = I;
	
		//if (I==0)
		//	r=r1;
		//else
		//	r=r2;
		//if ((saiz[r]==0)||(saiz[r]==1)) //if it is empty route or saiz=1, if only one customer, nothing to move around

		if (saiz[r]<=3)
			continue;

		for (int j = 1; j <= saiz[r]-2 ; j++) //from which position
		{
			element1 = VEHICLE[r][j];
			element1f = VEHICLE[r][j+1];
			before_ele1 = VEHICLE[r][j-1];
			after_ele1 = VEHICLE[r][j+2];
			for (int k = j+2; k <= saiz[r]-1; k++)
			{
				element2 = VEHICLE[r][k];
				element2f = VEHICLE[r][k+1];
				before_ele2 = VEHICLE[r][k-1];
				after_ele2 = VEHICLE[r][k+2];

				if ( (before_ele2 == SIZE || after_ele2 == SIZE) && (before_ele1 == SIZE || after_ele1 == SIZE) ) //if mn and ij depot
				{	
					if ( (NR_FLAG_DEPOT[element1][before_ele2] == false && NR_FLAG_DEPOT[element1f][after_ele2] == false) && (NR_FLAG_DEPOT[element1f][before_ele2] == false && NR_FLAG_DEPOT[element1][after_ele2] == false) )//original and reverse sequence of ij cannot
					{	if ( (NR_FLAG_DEPOT[element2][before_ele1] == false && NR_FLAG_DEPOT[element2f][after_ele1] == false) && (NR_FLAG_DEPOT[element2f][before_ele1] == false && NR_FLAG_DEPOT[element2][after_ele1] == false) )//original and reverse sequence of mn cannot
							continue;
					}
				}
				else if ( before_ele2 == SIZE || after_ele2 == SIZE ) //if mn depot
				{
					if ( (NR_FLAG_DEPOT[element1][before_ele2] == false && NR_FLAG_DEPOT[element1f][after_ele2] == false) && (NR_FLAG_DEPOT[element1f][before_ele2] == false && NR_FLAG_DEPOT[element1][after_ele2] == false) )//original and reverse sequence of ij cannot
					{	if ( (NR_FLAG[element2][before_ele1] == false && NR_FLAG[element2f][after_ele1] == false) && (NR_FLAG[element2f][before_ele1] == false && NR_FLAG[element2][after_ele1] == false) )//original and reverse sequence of mn cannot
							continue;
					}
				}
				else if ( before_ele1 == SIZE || after_ele1 == SIZE ) //if ij depot
				{
					if ( (NR_FLAG[element1][before_ele2] == false && NR_FLAG[element1f][after_ele2] == false) && (NR_FLAG[element1f][before_ele2] == false && NR_FLAG[element1][after_ele2] == false) )//original and reverse sequence of ij cannot
					{	if ( (NR_FLAG_DEPOT[element2][before_ele1] == false && NR_FLAG_DEPOT[element2f][after_ele1] == false) && (NR_FLAG_DEPOT[element2f][before_ele1] == false && NR_FLAG_DEPOT[element2][after_ele1] == false) )//original and reverse sequence of mn cannot
							continue;
					}
				}
				else //if no depot
				{
					if ( (NR_FLAG[element1][before_ele2] == false && NR_FLAG[element1f][after_ele2] == false) && (NR_FLAG[element1f][before_ele2] == false && NR_FLAG[element1][after_ele2] == false) )//original and reverse sequence of ij cannot
					{	if ( (NR_FLAG[element2][before_ele1] == false && NR_FLAG[element2][after_ele1] == false) && (NR_FLAG[element2f][before_ele1] == false && NR_FLAG[element2][after_ele1] == false) ) //original and reverse sequence of mn cannot //if insertion cannot between cust before and cust after
							continue;
					}
				}
				//same route, noneed to consider service_time[]
				//D-1-2-3-4-5-6-7-D
				int n = saiz[r];
				int a = j-1;
				int b = j;
				int c = j+1;
				int d = k-1;
				int e = k;
				int f = k+1;
				
				//same route, noneed to consider service_time[]
				//D-1-2-3-4-5-6-7-D
				if (k==(j+2)) // if consecutive customer, delete 3 arcs and insert 3 arcs (j+1 == k-1)
				{
					//eg: D-1-2-3-4-5-6-7-D consider 1,2 and 3,4
					dist_without_i = dist[before_ele1][element1] + dist[element1f][after_ele1] + dist[element2f][after_ele2]; //route_cost is global variable //minus 3 arcs 
					dist_with_i = dist[before_ele1][element2] + dist[element2f][element1] +dist[element1f][after_ele2];//D-3-4-1-2-5-6-7-D
					//in R1, ele2 and ele2f reversed
					dist_with_i_Reverse = dist[before_ele1][element2f] + dist[element2][element1] +dist[element1f][after_ele2];//D-4-3-1-2-5-6-7-D
					//in R2, ele1 and ele1f reversed
					dist_with_j_Reverse = dist[before_ele1][element2] + dist[element2f][element1f] +dist[element1][after_ele2];//D-3-4-2-1-5-6-7-D
					dist_with_both_Reverse = dist[before_ele1][element2f] + dist[element2][element1f] +dist[element1][after_ele2];//D-4-3-2-1-5-6-7-D


					cost_without_i = (n-a)*dist[before_ele1][element1] + (n-b)*dist[element1][element1f] + (n-c)*dist[element1f][after_ele1] + (n-e)*dist[element2][element2f] + (n-f)*dist[element2f][after_ele2]; //minus 5 arcs 
					cost_with_i = (n-a)*dist[before_ele1][element2] + (n-b)*dist[element2][element2f] + (n-c)*dist[element2f][element1] + (n-e)*dist[element1][element1f] + (n-f)*dist[element1f][after_ele2];//D-3-4-1-2-5-6-7-D
					//in R1, ele2 and ele2f reversed
					cost_with_i_Reverse = (n-a)*dist[before_ele1][element2f] + (n-b)*dist[element2f][element2] + (n-c)*dist[element2][element1] + (n-e)*dist[element1][element1f] + (n-f)*dist[element1f][after_ele2];//D-4-3-1-2-5-6-7-D
					//in R2, ele1 and ele1f reversed
					cost_with_j_Reverse = (n-a)*dist[before_ele1][element2] + (n-b)*dist[element2][element2f] + (n-c)*dist[element2f][element1f] + (n-e)*dist[element1f][element1] + (n-f)*dist[element1][after_ele2];//D-3-4-2-1-5-6-7-D
					cost_with_both_Reverse = (n-a)*dist[before_ele1][element2f] + (n-b)*dist[element2f][element2] + (n-c)*dist[element2][element1f] + (n-e)*dist[element1f][element1] + (n-f)*dist[element1][after_ele2];//D-4-3-2-1-5-6-7-D

					gain = cost_without_i - cost_with_i;
					gain_with_i_Reverse = cost_without_i - cost_with_i_Reverse;
					gain_with_j_Reverse = cost_without_i - cost_with_j_Reverse;
					gain_with_both_Reverse = cost_without_i - cost_with_both_Reverse;
				}
				else  // if non-consecutive customer, delete 4 arcs and insert 4 arcs
				{
					//eg: D-1-2-3-4-5-6-7-D consider 1,2 and 4,5
					dist_without_i = dist[before_ele1][element1] + dist[element1f][after_ele1] + dist[before_ele2][element2] + dist[element2f][after_ele2]; //minus 4 arcs 
					dist_with_i = dist[before_ele1][element2] + dist[element2f][after_ele1] +dist[before_ele2][element1] +dist[element1f][after_ele2]; //D-4-5-3-1-2-6-7-D 
					//in R1, ele2 and ele2f reversed
					dist_with_i_Reverse = dist[before_ele1][element2f] + dist[element2][after_ele1] +dist[before_ele2][element1] +dist[element1f][after_ele2];//D-5-4-3-1-2-6-7-D 
					//in R2, ele1 and ele1f reversed
					dist_with_j_Reverse = dist[before_ele1][element2] + dist[element2f][after_ele1] +dist[before_ele2][element1f] +dist[element1][after_ele2];//D-4-5-3-2-1-6-7-D 
					dist_with_both_Reverse = dist[before_ele1][element2f] + dist[element2][after_ele1] +dist[before_ele2][element1f] +dist[element1][after_ele2];//D-5-4-3-2-1-6-7-D 

					cost_without_i = (n-a)*dist[before_ele1][element1] + (n-b)*dist[element1][element1f] + (n-c)*dist[element1f][after_ele1] + (n-d)*dist[before_ele2][element2] + (n-e)*dist[element2][element2f] + (n-f)*dist[element2f][after_ele2]; //minus 6 arcs 
					cost_with_i = (n-a)*dist[before_ele1][element2] + (n-b)*dist[element2][element2f] + (n-c)*dist[element2f][after_ele1] + (n-d)*dist[before_ele2][element1] + (n-e)*dist[element1][element1f] + (n-f)*dist[element1f][after_ele2]; //D-4-5-3-1-2-6-7-D 
					cost_with_i_Reverse = (n-a)*dist[before_ele1][element2f] + (n-b)*dist[element2f][element2] + (n-c)*dist[element2][after_ele1] + (n-d)*dist[before_ele2][element1] + (n-e)*dist[element1][element1f] + (n-f)*dist[element1f][after_ele2]; //D-5-4-3-1-2-6-7-D 
					cost_with_j_Reverse = (n-a)*dist[before_ele1][element2] + (n-b)*dist[element2][element2f] + (n-c)*dist[element2f][after_ele1] + (n-d)*dist[before_ele2][element1f] + (n-e)*dist[element1f][element1] + (n-f)*dist[element1][after_ele2]; //D-4-5-3-2-1-6-7-D 
					cost_with_both_Reverse = (n-a)*dist[before_ele1][element2f] + (n-b)*dist[element2f][element2] + (n-c)*dist[element2][after_ele1] + (n-d)*dist[before_ele2][element1f] + (n-e)*dist[element1f][element1] + (n-f)*dist[element1][after_ele2]; //D-5-4-3-2-1-6-7-D 
				
				}
				reverseType = 0;//initialize to zero which means no reverse
				if (cost_with_i_Reverse - cost_with_i < -epsilon)//cost_with_i_Reverse < cost_with_i
				{
					cost_with_i = cost_with_i_Reverse;
					dist_with_i = dist_with_i_Reverse;
					reverseType = 2;//in R1, ele2 and ele2f reversed
				}
				if (cost_with_j_Reverse - cost_with_i < -epsilon)//cost_with_i_Reverse2 < cost_with_i
				{
					cost_with_i = cost_with_j_Reverse;
					dist_with_i = dist_with_j_Reverse;
					reverseType = 1;//in R2, ele1 and ele1f reversed
				}
				if (cost_with_both_Reverse - cost_with_i < -epsilon)//cost_with_bothReverse < cost_with_i
				{
					cost_with_i = cost_with_both_Reverse;
					dist_with_i = dist_with_both_Reverse;
					reverseType = 3;
				}
				gain = cost_without_i - cost_with_i; //gain = old-new
				dist_gain = dist_without_i - dist_with_i;

				//if (abs(gain) > distance_available[r])//if negative gain, it will not be recorded anyway, so this code can be omitted actually
				//{
				//	continue;
				//}

				if (gain-gain_matrix_1_0[r][r] > epsilon)//gain > gain_matrix_1_0[r][r]
				{
					gain_matrix_1_0[r][r] = gain;
					same_r_info[r][0] = r;
					same_r_info[r][1] = gain;
					same_r_info[r][2] = r; //from route
					same_r_info[r][3] = r; //to route, i=m here
					same_r_info[r][4] = j; //from which position
					same_r_info[r][5] = k; //this is the original position to be swapped, not modified route !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! insertion need to be careful
					same_r_info[r][6] = k; 
					same_r_info[r][7] = j;
					same_r_info[r][8] = 5; //2 is (2-2)
					same_r_info[r][9] = reverseType;
					same_r_info[r][10] = gain; //gain1
					same_r_info[r][11] = dist_gain; //gain1 in distance
				}	
			}//end of k		
		}	//end of j	
	}//end of I

	//////////////////////////////////==============================  DIFFERENT ROUTES ==========================================////////////////////
	int i = -1; //initialize to -1
	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//================================================(from) 2 affected routes to other routes (update ROW)================================================================
	for (int I = 0; I < no_routes; I++)  //consider each and other routes (from R1)
	{
		if (RchangedStatus[I] == false)
			continue;
		else
			i = I;

		if (saiz[i] <= 1) //if it is empty route or only one customer
			continue;
		for (int j = 1; j <= saiz[i] - 1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
		{	
			element1 = VEHICLE[i][j];			//elemet to be inserted into other route
			element1f = VEHICLE[i][j+1];			//elemet to be inserted into other route
			before_ele1 = VEHICLE[i][j-1];
			after_ele1 = VEHICLE[i][j+2];
	

			for (int m = i+1; m < no_routes; m++)  //insert (to) which route (to R2)//changed to m=i+1 on 6April2015
			{
				if ((i == m) || (saiz[m] <= 1))// || (demand[element2] > space_available[m])) //if customer already in the route or demand exceed avaialable space in route
				{
					continue;

				}
				for (int n = 1; n <= saiz[m] - 1; n++) //which customer in r2 to remove
				{
				
					element2 = VEHICLE[m][n];			//element to be removed from r2
					element2f = VEHICLE[m][n+1];			//element to be removed from r2	
					before_ele2 = VEHICLE[m][n-1];
					after_ele2 = VEHICLE[m][n+2];
					int space_r1 = space_available[i];
					int space_r2 = space_available[m];
					space_r1 = space_r1 + demand[element1] + demand[element1f];
					space_r2 = space_r2 + demand[element2] + demand[element2f];

					if ((demand[element1] + demand[element1f] > space_r2) || (demand[element2] + demand[element2f] > space_r1))
						continue;//go to next element to delete

					float available_dist_r1 = distance_available[i];
					float available_dist_r2 = distance_available[m];
			

					
					if ( (before_ele2 == SIZE || after_ele2 == SIZE) && (before_ele1 == SIZE || after_ele1 == SIZE) ) //if mn and ij depot
					{	
						if ( (NR_FLAG_DEPOT[element1][before_ele2] == false && NR_FLAG_DEPOT[element1f][after_ele2] == false) && (NR_FLAG_DEPOT[element1f][before_ele2] == false && NR_FLAG_DEPOT[element1][after_ele2] == false) )//original and reverse sequence of ij cannot
						{	if ( (NR_FLAG_DEPOT[element2][before_ele1] == false && NR_FLAG_DEPOT[element2f][after_ele1] == false) && (NR_FLAG_DEPOT[element2f][before_ele1] == false && NR_FLAG_DEPOT[element2][after_ele1] == false) )//original and reverse sequence of mn cannot
								continue;
						}
					}
					else if ( before_ele2 == SIZE || after_ele2 == SIZE ) //if mn depot
					{
						if ( (NR_FLAG_DEPOT[element1][before_ele2] == false && NR_FLAG_DEPOT[element1f][after_ele2] == false) && (NR_FLAG_DEPOT[element1f][before_ele2] == false && NR_FLAG_DEPOT[element1][after_ele2] == false) )//original and reverse sequence of ij cannot
						{	if ( (NR_FLAG[element2][before_ele1] == false && NR_FLAG[element2f][after_ele1] == false) && (NR_FLAG[element2f][before_ele1] == false && NR_FLAG[element2][after_ele1] == false) )//original and reverse sequence of mn cannot
								continue;
						}
					}
					else if ( before_ele1 == SIZE || after_ele1 == SIZE ) //if ij depot
					{
						if ( (NR_FLAG[element1][before_ele2] == false && NR_FLAG[element1f][after_ele2] == false) && (NR_FLAG[element1f][before_ele2] == false && NR_FLAG[element1][after_ele2] == false) )//original and reverse sequence of ij cannot
						{	if ( (NR_FLAG_DEPOT[element2][before_ele1] == false && NR_FLAG_DEPOT[element2f][after_ele1] == false) && (NR_FLAG_DEPOT[element2f][before_ele1] == false && NR_FLAG_DEPOT[element2][after_ele1] == false) )//original and reverse sequence of mn cannot
								continue;
						}
					}
					else //if no depot
					{
						if ( (NR_FLAG[element1][before_ele2] == false && NR_FLAG[element1f][after_ele2] == false) && (NR_FLAG[element1f][before_ele2] == false && NR_FLAG[element1][after_ele2] == false) )//original and reverse sequence of ij cannot
						{	if ( (NR_FLAG[element2][before_ele1] == false && NR_FLAG[element2][after_ele1] == false) && (NR_FLAG[element2f][before_ele1] == false && NR_FLAG[element2][after_ele1] == false) ) //original and reverse sequence of mn cannot //if insertion cannot between cust before and cust after
								continue;
						}
					}
					int N1 = saiz[i];
					int N2 = saiz[m];
					int a = j-1;
					int b = j;
					int c = j+1;
					int d = n-1;
					int e = n;
					int f = n+1;

					cost_without_i = (N1-a)*dist[before_ele1][element1] + (N1-b)*dist[element1][element1f] + (N1-c)*dist[element1f][after_ele1]; //minus 3 arcs 
					cost_without_j = (N2-d)*dist[before_ele2][element2] + (N2-e)*dist[element2][element2f] + (N2-f)*dist[element2f][after_ele2];

					cost_with_i = (N1-a)*dist[before_ele1][element2] + (N1-b)*dist[element2][element2f] + (N1-c)*dist[element2f][after_ele1];
					cost_with_j = (N2-d)*dist[before_ele2][element1] + (N2-e)*dist[element1][element1f] + (N2-f)*dist[element1f][after_ele2];
					//in R1, ele2 and ele2f reversed
					cost_with_i_Reverse = (N1-a)*dist[before_ele1][element2f] + (N1-b)*dist[element2f][element2] + (N1-c)*dist[element2][after_ele1];
					//in R2, ele1 and ele1f reversed
					cost_with_j_Reverse =  (N2-d)*dist[before_ele2][element1f] + (N2-e)*dist[element1f][element1] + (N2-f)*dist[element1][after_ele2];//D-3-4-2-1-5-6-7-D
					
					
					dist_without_i = dist[element1][before_ele1] + dist[element1][element1f] + dist[element1f][after_ele1]; //cost of r1 without i and i+1 //old
					dist_without_j = dist[element2][before_ele2] + dist[element2][element2f] + dist[element2f][after_ele2]; //cost of r2 without i and i+1 //old

					
					dist_with_i = dist[before_ele1][element2] + dist[element2][element2f] + dist[element2f][after_ele1]; //cost of r1 with i and i+1 //new
					dist_with_j = dist[before_ele2][element1] + dist[element1][element1f] + dist[element1f][after_ele2]; //cost of r2 with i and i+1 //new
					//in R1, ele2 and ele2f reversed
					dist_with_i_Reverse = dist[before_ele1][element2f] + dist[element2f][element2] + dist[element2][after_ele1]; //cost of r1 with i+1 and i //new-old
					//in R2, ele1 and ele1f reversed
					dist_with_j_Reverse = dist[before_ele2][element1f] + dist[element1f][element1] + dist[element1][after_ele2]; //cost of r2 with i+1 and i //new
					

					reverseType = 0;//initialize to zero which means no reverse
					//reverseType=1 (no reverse), 1(i reverse), 2(j reverse), 3(both reverse)
					if ( (cost_with_i_Reverse - cost_with_i < -epsilon) && (cost_with_j_Reverse - cost_with_j < -epsilon) )//cost_with_i_Reverse < cost_with_i //both reverse
					{
						cost_with_i = cost_with_i_Reverse;
						cost_with_j = cost_with_j_Reverse;
						dist_with_i = dist_with_i_Reverse;
						dist_with_j = dist_with_j_Reverse;
						reverseType = 3;
					}
					else//if both reverse not true, test if only one reverse is true
					{
						if (cost_with_i_Reverse - cost_with_i < -epsilon)//cost_with_i_Reverse < cost_with_i
						{
							cost_with_i = cost_with_i_Reverse;
							dist_with_i = dist_with_i_Reverse;
							reverseType = 2;//in R1, ele2 and ele2f reversed
						}
						if (cost_with_j_Reverse - cost_with_j < -epsilon)//cost_with_i_Reverse < cost_with_i
						{
							cost_with_j = cost_with_j_Reverse;
							dist_with_j = dist_with_j_Reverse;
							reverseType = 1; //in R2, ele1 and ele1f reversed
						}
					}
					//cost_with_i is R1 //cost_with_j is R2 //changed on 22Nov15
					if ((dist_with_i > available_dist_r1 + dist_without_i) || (dist_with_j > available_dist_r2 + dist_without_j))
						continue; //go to next position to insert, element deleted is still the same

					gain = (cost_without_i + cost_without_j) - (cost_with_i + cost_with_j);

					if (i < m)
					{
						if (gain-gain_matrix_1_0[i][m] > epsilon)//gain > gain_matrix_1_0[i][m]
						{
							gain_matrix_1_0[i][m] = gain;

							//info_1_0[gain_matrix_1_0[m][i]][0] = gain_matrix_1_0[m][i];
							info_1_0[(int)gain_matrix_1_0[m][i]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[m][i]][2] = i;
							info_1_0[(int)gain_matrix_1_0[m][i]][3] = m;
							info_1_0[(int)gain_matrix_1_0[m][i]][4] = j;
							info_1_0[(int)gain_matrix_1_0[m][i]][5] = n;
							info_1_0[(int)gain_matrix_1_0[m][i]][6] = n;
							info_1_0[(int)gain_matrix_1_0[m][i]][7] = j;
							info_1_0[(int)gain_matrix_1_0[m][i]][8] = 5;//5 is (2-2) //this is swap 2-2
							info_1_0[(int)gain_matrix_1_0[m][i]][9] = reverseType;
							info_1_0[(int)gain_matrix_1_0[m][i]][10] = cost_without_i - cost_with_i;//gain1
							info_1_0[(int)gain_matrix_1_0[m][i]][11] = dist_without_i - dist_with_i;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[m][i]][12] = dist_without_j - dist_with_j;//gain2 in distance
						}//end if
					}

					else if (i > m)
					{
						if (gain-gain_matrix_1_0[m][i] > epsilon)//gain > gain_matrix_1_0[m][i]
						{
							gain_matrix_1_0[m][i] = gain;////////////////////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! (CHANGED ON 1 OCT 2014!!!!!!!!!!!!!!!!!!)

							//info_1_0[number_matrix_1_0[i][m]][0] = gain_matrix_1_0[i][m];
							info_1_0[(int)gain_matrix_1_0[i][m]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[i][m]][2] = i;
							info_1_0[(int)gain_matrix_1_0[i][m]][3] = m;
							info_1_0[(int)gain_matrix_1_0[i][m]][4] = j;
							info_1_0[(int)gain_matrix_1_0[i][m]][5] = n;
							info_1_0[(int)gain_matrix_1_0[i][m]][6] = n;
							info_1_0[(int)gain_matrix_1_0[i][m]][7] = j;
							info_1_0[(int)gain_matrix_1_0[i][m]][8] = 5;//5 is (2-2)
							info_1_0[(int)gain_matrix_1_0[i][m]][9] = reverseType;
							info_1_0[(int)gain_matrix_1_0[i][m]][10] = cost_without_i - cost_with_i;//gain1
							info_1_0[(int)gain_matrix_1_0[i][m]][11] = dist_without_i - dist_with_i;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[i][m]][12] = dist_without_j - dist_with_j;//gain2 in distance
						}//end if
					}//end else if
				}//end for n
			}//end for m
		}//end for j
	}//end for I 
}



//DONE
void partialupdate_reoptimize_two_optintra(float **(&same_r_info), float **(&gain_matrix_1_0)) 
{
	for (int i = 0; i < no_routes; i++)  //from route 0 to last route
	{
		if (RchangedStatus[i] == false)//if route has not been changed previously, just skip it
			continue;

		if (saiz[i] <= 2) //changed 28Jun2015, only route size > 2 can do 2-opt, if size=2, front and back is depot, so it is the same
			continue;
			
		for (int m = 1; m <= saiz[i]-1; m++)
		{
			for (int n = m + 1; n <= saiz[i]; n++)
			{
				int prevC = VEHICLE[i][m-1];
				int afterC= VEHICLE[i][n+1];
				int cStart = VEHICLE[i][m];//start of customer
				int cEnd = VEHICLE[i][n];//end of customer
				if (prevC == SIZE || afterC ==SIZE) //added 2March2016
				{
					if (NR_FLAG_DEPOT[prevC][cEnd] == false && NR_FLAG_DEPOT[afterC][cStart] == false) //prev and n, next and m because ereverse order, the segment is from m to n
						continue;
				}
				else
					if (NR_FLAG[prevC][cEnd] == false && NR_FLAG[afterC][cStart] == false)
						continue;
				//=================== CONSIDER REVERSE ORDER ===========================// 
				float origain = dist[prevC][cStart] + dist[afterC][cEnd] - (dist[prevC][cEnd] + dist[afterC][cStart]); //only compare two ends cost, the intermediate all the same //old-new

				float *tempCumDist = new float[SIZE];
				
				float oldCumDist=0, newCumDist=0;
				tempCumDist[0]=CumDist[i][m-1];
				int u=1;//for tempCumDist
				
				for (int t = n; t >= m; t--)//recalculate CumDist in reverse order
				{
					oldCumDist += CumDist[i][t];
					tempCumDist[u] = tempCumDist[u-1] + dist[prevC][VEHICLE[i][t]] + service_time[VEHICLE[i][t]];
					newCumDist += tempCumDist[u];
					prevC = VEHICLE[i][t];
					u++;
				}

				int remainingCust = saiz[i] - n;
				if (remainingCust <0)
					remainingCust =0;

				float gain = (oldCumDist - newCumDist) + remainingCust*origain;
				
				delete[] tempCumDist;
			
				if (gain-gain_matrix_1_0[i][i] > epsilon)//gain > gain_matrix_1_0[r][r]
				{
					gain_matrix_1_0[i][i] = gain;
					same_r_info[i][0] = i;
					same_r_info[i][1] = gain;
					same_r_info[i][2] = i; //from route
					same_r_info[i][3] = i; //to route, i=m here
					same_r_info[i][4] = m; //beginning of subroute
					same_r_info[i][5] = n; //end of subroute
					same_r_info[i][6] = -1; 
					same_r_info[i][7] = -1;
					same_r_info[i][8] = 6; //6 is (2optintra)
					//same_r_info[i][9] = reverseStatus; //no reverse status
					same_r_info[i][10] = gain; //gain1
					same_r_info[i][11] = origain; //gain1 in distance
				}			
			}//end of n
		}//end of m
	}//end of i
}
//DONE
void partialupdate_reoptimize_two_optinter(float **(&info_1_0), float **(&gain_matrix_1_0)) //return number of route change for updating cost_of_removing in function call
{
	std::vector<int> myvector2; //add depot at beginning and end, this is for swapping

	for (int i = 0; i < no_routes; i++)  //from route 0 to last route
	{
		if (saiz[i] == 0) 
			continue;
		
		for (int m = 0; m < no_routes; m++)//from route 0 to last route
		{
			if ((saiz[m] == 0) || (i==m))
				continue;
			
			if (RchangedStatus[i] == false && RchangedStatus[m] == false)//if both routes have not been changed previosuly, skip it
				continue;

			for (int j = 1; j <= saiz[i]; j++) //for the length of first route //j++ means delete less in r1 because delete from j to saiz
			{
				//nextR2:
				int startEle = VEHICLE[i][j];
				int ele1before = VEHICLE[i][j-1];

				for (int n = 1; n <= saiz[m]; n++)//for the length of second route //n++ means delete more in r2 because delete from 1 to n
				{
					myvector2.clear();
					int endEle = VEHICLE[m][n];
					int ele2after = VEHICLE[m][n+1];
					
					if (j==1 && n==saiz[m]) //the one before j is DEPOT, after n is depot
					{
						if (NR_FLAG_DEPOT[startEle][VEHICLE[m][n+1]] == false && NR_FLAG_DEPOT[endEle][VEHICLE[i][j-1]] == false)//segment1 original sequence
						{		
							continue;	
						}
					}
					else if (j==1 && n!=saiz[m])//the one before j is DEPOT
					{
						if (NR_FLAG[startEle][VEHICLE[m][n+1]] == false && NR_FLAG_DEPOT[endEle][VEHICLE[i][j-1]] == false)//segment1 original sequence
						{		
							continue;	
						}
					}
					else if (j!=1 && n==saiz[m])//the one after n is depot
					{
						if (NR_FLAG_DEPOT[startEle][VEHICLE[m][n+1]] == false && NR_FLAG[endEle][VEHICLE[i][j-1]] == false)//segment1 original sequence
						{		
							continue;	
						}
					}
					else
					{
						if (NR_FLAG[startEle][VEHICLE[m][n+1]] == false && NR_FLAG[endEle][VEHICLE[i][j-1]] == false)//segment1 original sequence
						{		
							continue;	
						}
					}
					if (j==1 && n==saiz[m])//if both routes considered are full route, just skip, added on 8Oct2015
						continue;

					//delete from R1 (from position j until last one)
					int r1Size=0;
					int r1demandDelete=0;
					float oriCumDist1=0;
					
					float dist_without1=0;//for check condition
					int before1 = ele1before;
					for (int k = j; k <= saiz[i]; k++)
					{
						dist_without1 += dist[before1][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];//for check condition
						before1 = VEHICLE[i][k];//for check condition

						myvector2.push_back(VEHICLE[i][k]); //first part of myvector2 is from the first route from j to end
						r1demandDelete = r1demandDelete + demand[VEHICLE[i][k]];//consider they are to be deleted from r1
						oriCumDist1 += CumDist[i][k];//original CumDist of part1
						r1Size++;//number of customer consider in r1
					}
					dist_without1 += dist[VEHICLE[i][saiz[i]]][SIZE];//last arc from last ele to depot, noneed add service time becaus eit is an arc //for check condition

					float cost_without1 = oriCumDist1;//remainingCust1 is zero, so noneed multiply the remaining customer //remaining cust sure zero for route1 because delete until the last one

					//delete from R2
					//push in data from second route, m, starting from the first one until n
					int r2Size=0;
					int r2demandDelete=0;

					float oricost2=0;//original r2 cost//need for origain2 (old-new)
					float oriCumDist2=0;

					float dist_without2=0;//for check condition
					int before2 = SIZE;//before lement is depot
					for (int k = 1; k <= n; k++)
					{
						myvector2.push_back(VEHICLE[m][k]);//second part of myvector2 is from the second route from 1 to n
						oricost2 += dist[before2][VEHICLE[m][k]]+ service_time[VEHICLE[m][k]];
						before2 = VEHICLE[m][k];//for check condition

						oriCumDist2 += CumDist[m][k];//original CumDist of part2
						r2demandDelete = r2demandDelete + demand[VEHICLE[m][k]];//consider they are to be deleted from r2
						r2Size++;//number of customer considered in r2
					}
					oricost2 += dist[VEHICLE[m][n]][ele2after];//last arc from last ele to next, noneed add service time becaus eit is an arc
					dist_without2 = oricost2; //for check condition

					//for CCVRP, reverse order is not the same
					//if (r1Size+r2Size == saiz[i]+saiz[m])//added this one on 12Nov2015, if consider both full route, it remains the same for both routes, only reverse the order of the routes, element in the routes will stay but reverse order
					//	continue;
					//if r2 too full, delete more from r2
					if (r1demandDelete > space_available[m] + r2demandDelete) //added on 12Nov2015 //means too big for r2
					{
						continue;//if exceed capacity of r2, need to delete more in r2, so n++ meaning taking from 1 to n
					}
					//if r1 too full, goto next r2
					if (r2demandDelete > space_available[i] + r1demandDelete) //added on 12Nov2015 
					{
						goto next_r2; //if exceed capacity of r1, goto next r2, dont consider more customer in r2, this will exceed more
					}

					//calculate cost_without2
					float origain2 = oricost2 - dist[SIZE][ele2after]; //old-new
					int remainingCust2 = saiz[m] - n;
					float cost_without2 = oriCumDist2 + remainingCust2*origain2;


					//reverse the order
					std::reverse(myvector2.begin(),myvector2.end());    // 9 8 7 6 5 4 3 2 1
					
					//======UNNECESSARY ======= just for checking
					//calculate new demand, delete the previous one and add the reverse customers
					int r1demand = total_demand[i] - r1demandDelete;
					//add in customer from second route where number of customer follows number of customer deleted in r2
					for (int k = 0; k < r2Size; k++)//change the size follow other route, change to r2Size!!!!! r1 follows r2Size
					{
						r1demand = r1demand + demand[myvector2[k]];
					}
					int r2demand = total_demand[m] - r2demandDelete;
					//r2 follows r1Size
					for (int k = r2Size; k < (r1Size+r2Size); k++)//change the size follow other route, change to r2Size!!!!!!!
					{
						r2demand = r2demand + demand[myvector2[k]];
					}
					if (r1demand < 0 || r2demand < 0)
					{
						cout<< "Demand in 2-optInter2 is wrong!!"<<endl;
						cout<<r1demand<<' '<<r2demand<<' '<<"r1 is "<<i<<" r2 is "<<m<<" r1Size= "<<r1Size<<" r2Size= "<<r2Size<<endl;
						getchar();
					}
					//check capacity constraint
					if (r1demand> CAPACITY || r2demand> CAPACITY)
						continue;
					//======UNNECESSARY ======= just for checking
					


					
					//insert to r1
					float newCumDist1=0, newCumDist2=0;
					float *tempCumDist1 = new float[SIZE]; //inserted to r1
					float *tempCumDist2 = new float[SIZE]; //inserted to r2

					tempCumDist1[0] = CumDist[i][j-1]; 
					int u=1;//for tempCumDist1
					float oriloss2=0.0;
					int prevC = ele1before;//the first one is 
					float dist_with1=0;//for check condition
					for (int k = 0; k <= r2Size-1; k++)//change to r2Size!!!!!!! r1 follows r2Size
					{
						dist_with1 += dist[prevC][myvector2[k]] + service_time[myvector2[k]];//inserted to r1
						tempCumDist1[u] = tempCumDist1[u-1] + dist[prevC][myvector2[k]] + service_time[myvector2[k]];//inserted to r1
						newCumDist1 += tempCumDist1[u];//total CumDist inserted to r1
						prevC = myvector2[k];
						u++;
					}
					dist_with1 += dist[myvector2[r2Size-1]][SIZE];//last ele to depot //for check condition
					float cost_with1 = newCumDist1;//remainingCust1 is zero

					//insert to r2
					float newcost2=0;//to calculate oriloss2
					tempCumDist2[0] = 0; 
					int v=1;//for tempCumDist2
					prevC = SIZE;//first one is depot
					float dist_with2=0;//for check condition
					for (int k = r2Size; k <= (r1Size+r2Size)-1; k++)//r2 follows r1Size, r2 is in the second part of myvector
					{
						newcost2 = newcost2 + dist[prevC][myvector2[k]] + service_time[myvector2[k]];
						tempCumDist2[v] = tempCumDist2[v-1] + dist[prevC][myvector2[k]] + service_time[myvector2[k]];
						newCumDist2 += tempCumDist2[v];
						prevC = myvector2[k];
						v++;
					}
					newcost2 = newcost2 + dist[myvector2[r1Size+r2Size-1]][ele2after];//last one to next ele
					oriloss2 = newcost2 - dist[SIZE][ele2after];  //new -old
					float cost_with2 = newCumDist2 + remainingCust2*oriloss2;
					dist_with2 = newcost2;//for check condition

					float gain = (cost_without1+cost_without2) - (cost_with1+cost_with2);//
					
					delete[] tempCumDist1;
					delete[] tempCumDist2;
					//if r2 too full, delete more from r2
					if (dist_with2 - dist_without2 > DISTANCE+epsilon) //if distance constraint not ok, skip
						continue;

					//if r1 too full
					if (dist_with1 - dist_without1 > DISTANCE+epsilon) 
						goto next_r2;

					
	
					//cost_with_i is R1 //cost_with_j is R2 
					if ( (dist_with1 - dist_without1 > DISTANCE+epsilon) || (dist_with2 - dist_without2 > DISTANCE+epsilon)) //if distance constraint not ok, skip
						continue;
					
					
					
					
					if (i<m)
					{
						if (gain- gain_matrix_1_0[i][m] > epsilon)//gain > gain_matrix_1_0[i][m]
						{
							gain_matrix_1_0[i][m] = gain;

							//info[number_matrix_1_0[i][m]][0] = number_matrix_1_0[m][i];
							info_1_0[(int)gain_matrix_1_0[m][i]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[m][i]][2] = i;
							info_1_0[(int)gain_matrix_1_0[m][i]][3] = m;
							info_1_0[(int)gain_matrix_1_0[m][i]][4] = j; //the element considered in r1 (begin of suboute)
							info_1_0[(int)gain_matrix_1_0[m][i]][5] = n; //the element considered in r2 (end of suboute)
							info_1_0[(int)gain_matrix_1_0[m][i]][6] = -1; //from position in r2
							info_1_0[(int)gain_matrix_1_0[m][i]][7] = -1; //to position in r1
							info_1_0[(int)gain_matrix_1_0[m][i]][8] = 7; //7 is (2optinter)
							//info_1_0[(int)gain_matrix_1_0[m][i]][9] = reverseStatus; //no need reverse order
							info_1_0[(int)gain_matrix_1_0[m][i]][10] = cost_without1 - cost_with1;//gain1
							info_1_0[(int)gain_matrix_1_0[m][i]][11] = dist_without1 - dist_with1;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[m][i]][12] = dist_without2 - dist_with2;//gain2 in distance
						}//end if
					}//end if (i<m)

					else if (i>m)
					{
						if (gain-gain_matrix_1_0[m][i] > epsilon)//gain > gain_matrix_1_0[m][i]
						{
							gain_matrix_1_0[m][i] = gain;

							//info[number_matrix_1_0[i][m]][0] = number_matrix_1_0[m][i];
							info_1_0[(int)gain_matrix_1_0[i][m]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[i][m]][2] = i;
							info_1_0[(int)gain_matrix_1_0[i][m]][3] = m;
							info_1_0[(int)gain_matrix_1_0[i][m]][4] = j; //the element considered in r1 (begin of suboute)
							info_1_0[(int)gain_matrix_1_0[i][m]][5] = n; //the element considered in r2 (end of suboute)
							info_1_0[(int)gain_matrix_1_0[i][m]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[i][m]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[i][m]][8] = 7; //7 is (2optinter)
							//info_1_0[(int)gain_matrix_1_0[i][m]][9] = reverseStatus; //no need reverse order
							info_1_0[(int)gain_matrix_1_0[i][m]][10] = cost_without1 - cost_with1;//gain1
							info_1_0[(int)gain_matrix_1_0[i][m]][11] = dist_without1 - dist_with1;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[i][m]][12] = dist_without2 - dist_with2;//gain2 in distance
						}//end if
					}//end if (i>m)					
				}//end of n
			}//end of j
			next_r2:;//if demand deleted in r2 exceeded demand in r1, dont delete anymore in r2 because this cannot be put in r1 //added 12Nov2015
		}//end of for m
	}//end of i
		
	myvector2.clear();
	myvector2.shrink_to_fit();
	
}
//DONE
void partialupdate_reoptimize_crossTail2(float **(&info_1_0), float **(&gain_matrix_1_0)) //return number of route change for updating cost_of_removing in function call
{
	std::vector<int> tail1; 
	std::vector<int> tail2; 
	std::vector<int> tail1Rev; 
	std::vector<int> tail2Rev; 

	for (int i = 0; i < no_routes-1; i++)  //from route 0 to last route-1 because consider 2 routes at a time
	{
		if (saiz[i] == 0) 
			continue;
		
		for (int m = i+1; m < no_routes; m++)//from route i+1 to last route //the start of tail2
		{
			if ((saiz[m] == 0) || (i==m))
				continue;
			
			if (RchangedStatus[i] == false && RchangedStatus[m] == false)//if both routes have not been changed previosuly, skip it
				continue;

			for (int j = 1; j <= saiz[i]; j++) //the start of tail1 //j++ means delete less in r1
			{
				int startTail1 = VEHICLE[i][j];
				int endTail1 = VEHICLE[i][saiz[i]];
				//tail1.clear();
				//tail1Rev.clear();

				for (int n = 1; n <= saiz[m]; n++)//the start of tail2 //n++ means delete less in r2
				{
					if ((j==1) && (n==1))//skip if both start at 1, there is no head for both
						continue;
					//if ((j==saiz[i]+1) && (n==saiz[m]+1))//skip if both take full route, there is no tail for both
					//	continue;
					tail1.clear();
					tail1Rev.clear();
					tail2.clear();
					tail2Rev.clear();
					int startTail2 = VEHICLE[m][n];
					int endTail2 = VEHICLE[m][saiz[m]];
					
					if (n==1) //the one before n is depot
					{
						if (NR_FLAG_DEPOT[startTail1][VEHICLE[m][n-1]] == false)//Tail1 original sequence //changed on 28Oct15 because last element already next to depot originally
						//if (NR_FLAG_DEPOT[startTail1][VEHICLE[m][n-1]] == false && NR_FLAG_DEPOT[endTail1][SIZE] == false)//Tail1 original sequence
						{
							if (NR_FLAG_DEPOT[startTail1][SIZE] == false && NR_FLAG_DEPOT[endTail1][VEHICLE[m][n-1]] == false) //Tail1 reverse sequence , both use NR_FLAG_DEPOT because n-1 is depot
							{
								continue; 
							}
						}
					}
					else
					{
						if (NR_FLAG[startTail1][VEHICLE[m][n-1]] == false)//Tail1 original sequence //changed on 28Oct15 because last element already next to depot originally
						//if (NR_FLAG[startTail1][VEHICLE[m][n-1]] == false && NR_FLAG[endTail1][SIZE] == false)//Tail1 original sequence
						{
							if (NR_FLAG_DEPOT[startTail1][SIZE] == false && NR_FLAG[endTail1][VEHICLE[m][n-1]] == false) //Tail1 reverse sequence //the one next to depot use NR_FLAG_DEPOT 
							{
								continue;
							}
						}
					}

					if (j==1) //the one before j is depot
					{
						if (NR_FLAG_DEPOT[startTail2][VEHICLE[i][j-1]] == false)//Tail2 original sequence //changed on 28Oct15 because last element already next to depot originally
						//if (NR_FLAG_DEPOT[startTail2][VEHICLE[i][j-1]] == false && NR_FLAG_DEPOT[endTail2][SIZE] == false)//Tail2 original sequence
						{
							if (NR_FLAG_DEPOT[startTail2][SIZE] == false && NR_FLAG_DEPOT[endTail2][VEHICLE[i][j-1]] == false) //Tail2 reverse sequence both use NR_FLAG_DEPOT because j-1 is depot
							{
								continue;
							}
						}
					}
					else
					{
						if (NR_FLAG[startTail2][VEHICLE[i][j-1]] == false)//Tail2 original sequence //changed on 28Oct15 because last element already next to depot originally
						//if (NR_FLAG[startTail2][VEHICLE[i][j-1]] == false && NR_FLAG[endTail2][SIZE] == false)//Tail2 original sequence
						{
							if (NR_FLAG_DEPOT[startTail2][SIZE] == false && NR_FLAG[endTail2][VEHICLE[i][j-1]] == false) //Tail2 reverse sequence //the one next to depot use NR_FLAG_DEPOT 
							{
								continue;
							}
						}
					}
					//push in data from first route, i, starting from position j until last one
					int tail1Size=0;
					int r1Taildemand=0;
					//float r1Tailcost = 0.0, r1TailRevcost=0.0;

					float oriCumDist1=0;
					float *tempCumDist2=new float[SIZE];//inserted to r2
					float *tempCumDist2Rev=new float[SIZE];//inserted to r2 in reverse order
					tempCumDist2[0] = CumDist[m][n-1];
					int t=1;//for tempCumDist2[]
					int prevC = VEHICLE[m][n-1];
					int oribefore = VEHICLE[i][j-1];//this is original cust before j //for check condition
					float dist_without1=0, dist_with2=0;//for check condition
					//find cost_of_removing1 and cost_of_inserting2
					float newCumDist2=0, newCumDist2Rev=0;
					for (int k = j; k <= saiz[i]; k++)//push in start from first element of tail until end excluding depot, add in depot later
					{
						tail1.push_back(VEHICLE[i][k]);
						r1Taildemand = r1Taildemand + demand[VEHICLE[i][k]]; //demand of depot=0, so no worry
						tail1Size++;//record the size of tail1 excluding depot 
						//r1Tailcost = r1Tailcost + dist[VEHICLE[i][k]][VEHICLE[i][k+1]] + service_time[VEHICLE[i][k]];//the whole cost starting from j to depot

						dist_without1 += dist[oribefore][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];//for check condition
						dist_with2 += dist[prevC][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];

						oriCumDist1 += CumDist[i][k];
						tempCumDist2[t] = tempCumDist2[t-1]+dist[prevC][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];
						newCumDist2 += tempCumDist2[t];

						prevC=VEHICLE[i][k];
						oribefore=VEHICLE[i][k];
						t++;
					}
					
					dist_without1 += dist[VEHICLE[i][saiz[i]]][SIZE];//last arc from last ele to next, noneed add service time becaus eit is an arc //for check condition
					dist_with2 += dist[VEHICLE[i][saiz[i]]][SIZE];

					float cost_of_removing1 = oriCumDist1;
					

					//push in data from second route, m, starting from the first one until n
					int tail2Size=0;
					int r2Taildemand=0;
					float r2Tailcost = 0.0, r2TailRevcost=0.0;
					
					float oriCumDist2=0;
					float *tempCumDist1=new float[SIZE];//inserted to r1
					float *tempCumDist1Rev=new float[SIZE];//inserted to r1 in reverse order

					//find cost_of_removing2 and cost_of_inserting1
					float newCumDist1=0, newCumDist1Rev=0;
					tempCumDist1[0] = CumDist[i][j-1];
					int w=1;//for tempCumDist1[]
					prevC = VEHICLE[i][j-1];
					oribefore = VEHICLE[m][n-1];//this is original cust before j //for check condition
					float dist_without2=0, dist_with1=0;//for check condition;//for check condition
					for (int k = n; k <= saiz[m]; k++)
					{
						tail2.push_back(VEHICLE[m][k]);
						r2Taildemand = r2Taildemand + demand[VEHICLE[m][k]];
						tail2Size++;//record the size of tail2 exluding depot 
						r2Tailcost = r2Tailcost + dist[VEHICLE[m][k]][VEHICLE[m][k+1]] + service_time[VEHICLE[m][k]];//the whole cost starting from n to depot
						
						dist_without2 += dist[oribefore][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];//for check condition
						dist_with1 += dist[prevC][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];//for check condition

						oriCumDist2 += CumDist[m][k];
						tempCumDist1[w] = tempCumDist1[w-1]+dist[prevC][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];
						newCumDist1 += tempCumDist1[w];

						prevC=VEHICLE[m][k];
						oribefore=VEHICLE[m][k];
						w++;
					}
					dist_without2 += dist[VEHICLE[m][saiz[m]]][SIZE];
					dist_with1 += dist[VEHICLE[m][saiz[m]]][SIZE];//last arc from last ele to next, noneed add service time becaus eit is an arc //for check condition

					float cost_of_removing2 = oriCumDist2;
					

					int r1demand = total_demand[i] - r1Taildemand + r2Taildemand;
					int r2demand = total_demand[m] - r2Taildemand + r1Taildemand;


					//check capacity constraint //changed on 12Nov2015
					if (r1demand> CAPACITY)//if exceed r1demand, delete less in r2
					{
						delete[] tempCumDist1;
						delete[] tempCumDist1Rev;
						delete[] tempCumDist2;
						delete[] tempCumDist2Rev;
						continue;
					}
					if (r2demand> CAPACITY)//if exceed r2demand, delete less in r1
					{
						delete[] tempCumDist1;
						delete[] tempCumDist1Rev;
						delete[] tempCumDist2;
						delete[] tempCumDist2Rev;
						goto increasej;
					}



					//find newCumDist2Rev
					tempCumDist2Rev[0] = CumDist[m][n-1];
					prevC = VEHICLE[m][n-1];
					t=1;//for tempCumDist2Rev[]
					float dist_with2Rev=0;//for check condition;//r2 with tail2
					for (int k = tail1Size-1; k >= 0; k--)
					{
						tail1Rev.push_back(tail1[k]);//push in reverse order, so noneed to use reverse function
						dist_with2Rev += dist[prevC][tail1[k]] + service_time[tail1[k]];
						tempCumDist2Rev[t] = tempCumDist2Rev[t-1]+dist[prevC][tail1[k]] + service_time[tail1[k]];
						newCumDist2Rev += tempCumDist2Rev[t];
						prevC=tail1[k];
						t++;
					}
					dist_with2Rev += dist[tail1[0]][SIZE]; //for check condition

					//find newCumDist1Rev
					tempCumDist1Rev[0] = CumDist[i][j-1];
					w=1;//for tempCumDist1Rev[]
					prevC = VEHICLE[i][j-1];
					
					float dist_with1Rev=0;//for check condition;//r2 with tail2
					for (int k = tail2Size-1; k >= 0; k--)
					{
						tail2Rev.push_back(tail2[k]);//push in reverse order, so noneed to use reverse function
						dist_with1Rev += dist[prevC][tail2[k]] + service_time[tail2[k]];
						tempCumDist1Rev[w] = tempCumDist1Rev[w-1]+dist[prevC][tail2[k]] + service_time[tail2[k]];
						newCumDist1Rev += tempCumDist1Rev[w];
						prevC=tail2[k];
						w++;
					}
					dist_with1Rev += dist[tail2[0]][SIZE];//last arc //for check condition

					int reverseType = 0;//initialize to zero which means no reverse

					bool tail1Reverse = false, tail2Reverse = false;
					if (newCumDist1Rev < newCumDist1)
					{
						newCumDist1 = newCumDist1Rev;
						dist_with1 = dist_with1Rev;//for check condition
						tail2Reverse = true; //if tail2 is reverse, type2
						reverseType = 2;
						
					}

					if (newCumDist2Rev < newCumDist2)
					{
						newCumDist2 = newCumDist2Rev;
						dist_with2 = dist_with2Rev;//for check condition
						tail1Reverse = true;//if tail1 is reverse, type1
						reverseType = 1;
						//tail1ptr = &tail1Rev;
					}

					if (tail1Reverse == true && tail2Reverse == true)//if both tail are reverse, type3
						reverseType = 3;


					float cost_of_inserting1 = newCumDist1;
					float cost_of_inserting2 = newCumDist2;
					
					delete[] tempCumDist1;
					delete[] tempCumDist1Rev;
					delete[] tempCumDist2;
					delete[] tempCumDist2Rev;
				
					if (dist_with1-dist_without1 > DISTANCE+epsilon)//if exceed r1dist, delete less in r2
						continue;
					if (dist_with2-dist_without2 > DISTANCE+epsilon)//if exceed r2dist, delete less in r1
						//continue;
						goto increasej;


					float gain = (cost_of_removing1 + cost_of_removing2) - (cost_of_inserting1 + cost_of_inserting2);

					

					//i is always less than m because m=i+1
					if (gain- gain_matrix_1_0[i][m] > epsilon)//gain > gain_matrix_1_0[i][m] 
					{
						gain_matrix_1_0[i][m] = gain;

						//info[number_matrix_1_0[i][m]][0] = number_matrix_1_0[m][i];
						info_1_0[(int)gain_matrix_1_0[m][i]][1] = gain;
						info_1_0[(int)gain_matrix_1_0[m][i]][2] = i;
						info_1_0[(int)gain_matrix_1_0[m][i]][3] = m;
						info_1_0[(int)gain_matrix_1_0[m][i]][4] = j; //the element considered in r1 (begin of tail1)
						info_1_0[(int)gain_matrix_1_0[m][i]][5] = n; //the element considered in r2 (begin of tail2)
						info_1_0[(int)gain_matrix_1_0[m][i]][6] = -1; //from position in r2
						info_1_0[(int)gain_matrix_1_0[m][i]][7] = -1; //to position in r1
						info_1_0[(int)gain_matrix_1_0[m][i]][8] = 8; //8 is (CROSStail)
						info_1_0[(int)gain_matrix_1_0[m][i]][9] = reverseType; 
						info_1_0[(int)gain_matrix_1_0[m][i]][10] = cost_of_removing1 - cost_of_inserting1;//gain1
						info_1_0[(int)gain_matrix_1_0[m][i]][11] = dist_without1 - dist_with1;//gain1 in distance
						info_1_0[(int)gain_matrix_1_0[m][i]][12] = dist_without2 - dist_with2;//gain2 in distance

					}//end if better gain found
				}//end of n
				increasej:;
			}//end of j

		}//end of for m
	}//end of i

	tail1.clear();
	tail1.shrink_to_fit();
	tail2.clear();
	tail2.shrink_to_fit();
	tail1Rev.clear();
	tail1Rev.shrink_to_fit();
	tail2Rev.clear();
	tail2Rev.shrink_to_fit();
}

void partialupdate_reoptimize_CROSS(float **(&info_1_0), float **(&gain_matrix_1_0)) //return number of route change for updating cost_of_removing in function call
{
	//ofstream TEMP("TO CHECK.txt");
	std::vector<int> segment1;
	std::vector<int> segment2; 
	std::vector<int> segment1Reverse;
	std::vector<int> segment2Reverse; 

	//int maxL = 4;
	//int minL = 2;
	//srand ( time(NULL) ); //seed it
	//int L1 = (rand() % (maxL-minL+1))+minL;//length of segment1
	//int L2 = (rand() % (maxL-minL+1))+minL;//length of segment2
	int L1 = 3;//length of segment1
	int L2 = 2;//length of segment2

	for (int i = 0; i < no_routes; i++)  //from route 0 to last route
	{
		if (saiz[i] == 0) 
			continue;

		for (int m = 0; m < no_routes; m++)//from route 0 to last route
		{
			if ((saiz[m] == 0) || (i==m))
				continue;

			if (RchangedStatus[i] == false && RchangedStatus[m] == false)//if both routes have not been changed previosuly, skip it
				continue;

			for (int j = 1; j <= saiz[i]-L1+1; j++) //for the length of first route
			{
				//segment1.clear();
				//segment1Reverse.clear();
				int segment1Start = VEHICLE[i][j];
				int segment1End = VEHICLE[i][j+L1-1];
				int segment1Size=L1;

				for (int n = 1; n <= saiz[m]-L2+1; n++)//for the length of second route
				{
					segment1.clear();
					segment1Reverse.clear();
					segment2.clear();
					segment2Reverse.clear();
					int segment2Start = VEHICLE[m][n];
					int segment2End = VEHICLE[m][n+L2-1];
					int segment2Size=L2;
					
					if (n==1 && n == saiz[m]-L2+1)//second route both ends are depot
					{
						if (NR_FLAG_DEPOT[segment1Start][VEHICLE[m][n-1]] == false && NR_FLAG_DEPOT[segment1End][VEHICLE[m][n+L2]] == false)//segment1 original sequence
						{
							if (NR_FLAG_DEPOT[segment1Start][VEHICLE[m][n+L2]] == false && NR_FLAG_DEPOT[segment1End][VEHICLE[m][n-1]] == false) //segment1 reverse sequence
							{
								continue;
							}
						}
					}
					else if (n==1)//the one before n is depot
					{
						if (NR_FLAG_DEPOT[segment1Start][VEHICLE[m][n-1]] == false && NR_FLAG[segment1End][VEHICLE[m][n+L2]] == false)//segment1 original sequence
						{
							if (NR_FLAG[segment1Start][VEHICLE[m][n+L2]] == false && NR_FLAG_DEPOT[segment1End][VEHICLE[m][n-1]] == false) //segment1 reverse sequence
							{
								continue;
							}
						}
					}
					else if (n == saiz[m]-L2+1)//the one after n is depot
					{
						if (NR_FLAG[segment1Start][VEHICLE[m][n-1]] == false && NR_FLAG_DEPOT[segment1End][VEHICLE[m][n+L2]] == false)//segment1 original sequence
						{
							if (NR_FLAG_DEPOT[segment1Start][VEHICLE[m][n+L2]] == false && NR_FLAG[segment1End][VEHICLE[m][n-1]] == false) //segment1 reverse sequence
							{
								continue;
							}
						}
					}
					else//none end of n is depot
					{
						if (NR_FLAG[segment1Start][VEHICLE[m][n-1]] == false && NR_FLAG[segment1End][VEHICLE[m][n+L2]] == false)//segment1 original sequence
						{
							if (NR_FLAG[segment1Start][VEHICLE[m][n+L2]] == false && NR_FLAG[segment1End][VEHICLE[m][n-1]] == false) //segment1 reverse sequence
							{
								continue;
							}
						}
					}


					if (j==1 && j==saiz[i]-L1+1)//first route both ends are depot
					{
						if (NR_FLAG_DEPOT[segment2Start][VEHICLE[i][j-1]] == false && NR_FLAG_DEPOT[segment2End][VEHICLE[i][j+L1]] == false)//segment2 original sequence
						{
							if (NR_FLAG_DEPOT[segment2Start][VEHICLE[i][j+L1]] == false && NR_FLAG_DEPOT[segment2End][VEHICLE[i][j-1]] == false) //segment2 reverse sequence
							{
								continue;
							}
						}
					}
					else if (j==1)//the one before j is depot
					{
						if (NR_FLAG_DEPOT[segment2Start][VEHICLE[i][j-1]] == false && NR_FLAG[segment2End][VEHICLE[i][j+L1]] == false)//segment2 original sequence
						{
							if (NR_FLAG[segment2Start][VEHICLE[i][j+L1]] == false && NR_FLAG_DEPOT[segment2End][VEHICLE[i][j-1]] == false) //segment2 reverse sequence
							{
								continue;
							}
						}
					}
					else if (j == saiz[i]-L1+1)//the one after n is depot
					{
						if (NR_FLAG[segment2Start][VEHICLE[i][j-1]] == false && NR_FLAG_DEPOT[segment2End][VEHICLE[i][j+L1]] == false)//segment2 original sequence
						{
							if (NR_FLAG_DEPOT[segment2Start][VEHICLE[i][j+L1]] == false && NR_FLAG[segment2End][VEHICLE[i][j-1]] == false) //segment2 reverse sequence
							{
								continue;
							}
						}
					}
					else//none end of j is depot
					{
						if (NR_FLAG[segment2Start][VEHICLE[i][j-1]] == false && NR_FLAG[segment2End][VEHICLE[i][j+L1]] == false)//segment2 original sequence
						{
							if (NR_FLAG[segment2Start][VEHICLE[i][j+L1]] == false && NR_FLAG[segment2End][VEHICLE[i][j-1]] == false) //segment2 reverse sequence
							{
								continue;
							}
						}

					}
					//copy segment1
					int r1demandDelete=0;
					for (int k = j; k <= j+L1-1; k++)
					{
						segment1.push_back(VEHICLE[i][k]);//push in segment element
						//segment1Reverse.push_back(VEHICLE[i][k]);//push in segment element
						r1demandDelete = r1demandDelete + demand[VEHICLE[i][k]];
					}

					//copy segment2
					int r2demandDelete=0;
					for (int k = n; k <= n+L2-1; k++)
					{
						segment2.push_back(VEHICLE[m][k]);//push in segment element
						//segment2Reverse.push_back(VEHICLE[m][k]);//push in segment element
						r2demandDelete = r2demandDelete + demand[VEHICLE[m][k]];
					}
					int r1demand = total_demand[i] - r1demandDelete;

					for (int k = 0; k < segment2Size; k++)
					{
						r1demand = r1demand + demand[segment2[k]];
					}

					int r2demand = total_demand[m] - r2demandDelete;

					for (int k = 0; k < segment1Size; k++)
					{
						r2demand = r2demand + demand[segment1[k]];
					}
					//check capacity constraint
					if (r1demand> CAPACITY || r2demand> CAPACITY)
						continue;
					
					//if capacity ok, reverse the order, consider 4 types, R1&segmemt2, R1&segment2Reverse, R2&segment1, R2&segment1Reverse
					//=============================== reverse the order (added this on 7JUly2015
					for (int k = segment1.size()-1; k >= 0; k--)
					{
						segment1Reverse.push_back(segment1[k]);//push in reverse order, so noneed to use reverse function
					}
					for (int k = segment2.size()-1; k >= 0; k--)
					{
						segment2Reverse.push_back(segment2[k]);//push in reverse order, so noneed to use reverse function
					}
					//std::reverse(segment1Reverse.begin(),segment1Reverse.end());    // 9 8 7 6 5 4 3 2 1
					//std::reverse(segment2Reverse.begin(),segment2Reverse.end());    // 9 8 7 6 5 4 3 2 1
					//=========================================================//

					int seg1before = VEHICLE[i][j-1];
					int seg1after = VEHICLE[i][j+L1];
					
					int seg2before = VEHICLE[m][n-1];
					int seg2after = VEHICLE[m][n+L2];
					

					//calculate the cost delete segment1 from route i
					float deletecost1=0.0, deletecost2=0.0;
					deletecost1 = dist[seg1before][segment1[0]];//previous one and the start of segment
					for (int k = 0; k < segment1Size-1; k++)
					{
						deletecost1 = deletecost1 + dist[segment1[k]][segment1[k+1]] + service_time[segment1[k]];
					}
					deletecost1 = deletecost1 + service_time[segment1[segment1Size-1]] + dist[segment1[segment1Size-1]][seg1after];//last one in segment to the element after segment

					//calculate the cost delete segment2 from route m
					deletecost2 = dist[seg2before][segment2[0]];//previous one and the start of segment
					for (int k = 0; k < segment2Size-1; k++)
					{
						deletecost2 = deletecost2 + dist[segment2[k]][segment2[k+1]] + service_time[segment2[k]];
					}
					deletecost2 = deletecost2 + service_time[segment2[segment2Size-1]] + dist[segment2[segment2Size-1]][seg2after];//last one in segment to the element after segment
					
					//calculate the new distance
					bool seg1Reverse = false; //record the status of segment1 reverse, so we know which one is better
					bool seg2Reverse = false; //record the status of segment2 reverse, so we know which one is better
					//=================insert segment2 and segment2Reverse to R1===================//
					float insertcost1=0.0, insertcost2=0.0, insertcost1Rev=0.0, insertcost2Rev=0.0;
					insertcost1 = dist[seg1before][segment2[0]];//previous one and segment2
					insertcost1Rev = dist[seg1before][segment2Reverse[0]];//previous one and segment2
					for (int k = 0; k < segment2Size-1; k++)
					{
						insertcost1 = insertcost1 + dist[segment2[k]][segment2[k+1]] + service_time[segment2[k]];
						insertcost1Rev = insertcost1Rev + dist[segment2Reverse[k]][segment2Reverse[k+1]] + service_time[segment2Reverse[k]];
					}
					insertcost1 = insertcost1 + service_time[segment2[segment2Size-1]] + dist[segment2[segment2Size-1]][seg1after];//last one to 
					insertcost1Rev = insertcost1Rev + service_time[segment2Reverse[segment2Size-1]] + dist[segment2Reverse[segment2Size-1]][seg1after];//last one 

					//=================insert segment1 and segment1Reverse to R2===================//
					insertcost2 = dist[seg2before][segment1[0]];//previous one and segment1
					insertcost2Rev = dist[seg2before][segment1Reverse[0]];//previous one and segment1Reverse
					for (int k = 0; k < segment1Size-1; k++)
					{
						insertcost2 = insertcost2 + dist[segment1[k]][segment1[k+1]] + service_time[segment1[k]];
						insertcost2Rev = insertcost2Rev + dist[segment1Reverse[k]][segment1Reverse[k+1]] + service_time[segment1Reverse[k]];
					}
					insertcost2 = insertcost2 + service_time[segment1[segment1Size-1]] + dist[segment1[segment1Size-1]][seg2after];
					insertcost2Rev = insertcost2Rev + service_time[segment1Reverse[segment1Size-1]] + dist[segment1Reverse[segment1Size-1]][seg2after];
					
					int reverseType = 0;//initialize zero means no reverse for both
					vector<int> *seg1ptr = &segment1; //by default pointing to segment1
					vector<int> *seg2ptr = &segment2;//by default pointing to segment2
					if (insertcost1Rev < insertcost1) //no else, by default it is taking insertcost1
					{
						insertcost1 = insertcost1Rev;
						seg2Reverse = true;
						reverseType = 2;//if segment2 reverse, reverseType = 2
						seg2ptr = &segment2Reverse;
					}

					if (insertcost2Rev < insertcost2)
					{
						insertcost2 = insertcost2Rev;
						seg1Reverse = true;
						reverseType = 1;//if segment1 reverse, reverseType = 1
						seg1ptr = &segment1Reverse;
					}
					if (seg1Reverse == true && seg2Reverse == true)//if both segment reverse, reverseType = 3
						reverseType = 3;

					float old_cost1 = route_cost[i];
					float old_cost2 = route_cost[m];
					float new_cost1 = route_cost[i] - deletecost1 + insertcost1;
					float new_cost2 = route_cost[m] - deletecost2 + insertcost2;
					
					if ( (new_cost1 > DISTANCE+epsilon) || (new_cost2 > DISTANCE+epsilon)) //if constraint not ok, skip
						continue;

					float gain = (old_cost1+old_cost2) - (new_cost1+new_cost2);

					if (i<m)
					{
						if (gain- gain_matrix_1_0[i][m] > epsilon)//gain > gain_matrix_1_0[i][m]
						{
		
							gain_matrix_1_0[i][m] = gain;

							//info[number_matrix_1_0[i][m]][0] = number_matrix_1_0[m][i];
							info_1_0[(int)gain_matrix_1_0[m][i]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[m][i]][2] = i;
							info_1_0[(int)gain_matrix_1_0[m][i]][3] = m;
							info_1_0[(int)gain_matrix_1_0[m][i]][4] = j; //the start of in segment 1 
							info_1_0[(int)gain_matrix_1_0[m][i]][5] = n; //the start of segment2 
							info_1_0[(int)gain_matrix_1_0[m][i]][6] = -1; //from position in r2
							info_1_0[(int)gain_matrix_1_0[m][i]][7] = -1; //to position in r1
							info_1_0[(int)gain_matrix_1_0[m][i]][8] = 9; //9 is (CROSS)
							info_1_0[(int)gain_matrix_1_0[m][i]][9] = reverseType; 
							info_1_0[(int)gain_matrix_1_0[m][i]][10] = old_cost1 - new_cost1;//gain1

						}//end if
					}//end if (i<m)

					else if (i>m)
					{
						if (gain-gain_matrix_1_0[m][i] > epsilon)//gain > gain_matrix_1_0[m][i]
						{
			
							gain_matrix_1_0[m][i] = gain;

							//info[number_matrix_1_0[i][m]][0] = number_matrix_1_0[m][i];
							info_1_0[(int)gain_matrix_1_0[i][m]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[i][m]][2] = i;
							info_1_0[(int)gain_matrix_1_0[i][m]][3] = m;
							info_1_0[(int)gain_matrix_1_0[i][m]][4] = j; //the element considered in r1 (begin of suboute)
							info_1_0[(int)gain_matrix_1_0[i][m]][5] = n; //the element considered in r2 (end of suboute)
							info_1_0[(int)gain_matrix_1_0[i][m]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[i][m]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[i][m]][8] = 9; //9 is (CROSS)
							info_1_0[(int)gain_matrix_1_0[i][m]][9] = reverseType; 
							info_1_0[(int)gain_matrix_1_0[i][m]][10] = old_cost1 - new_cost1;//gain1

						}//end if
					}//end if (i>m)					
				}//end of n
			}//end of j
		}//end of for m
	}//end of i
	segment1.clear();
	segment1.shrink_to_fit();
	segment2.clear();
	segment2.shrink_to_fit();
	segment1Reverse.clear();
	segment1Reverse.shrink_to_fit();
	segment2Reverse.clear();
	segment2Reverse.shrink_to_fit();
}



