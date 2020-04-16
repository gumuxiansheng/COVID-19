#include <time.h>
//#include <windows.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <algorithm>    // std::sort
#include "VNS_multiLevel.h"
#include "VNS.h"
#include "LocalSearch.h"
//#include "DiversificationDijkstra.h"
//#include "DijkstraRefinement.h"
//#include "RecordBestSol.h"
//#include "2_opt.h"
#include "InverseMethod.h"
#include "DiversificationLNS.h"
#include "SectorConflict.h"
#include "Diversification_overlapRoute.h"
#include "Sort_solution.h"
//#include "VND.h"
#include "Guided_shake.h"
#include "BestImprovement.h"
//float infeasible_threshold = 0.06; //initially allow 6% violation
#define INFEASIBLE 0.00

#define INFIN 99999999 
using namespace std;

//added 10Sept2015
//need to use three level operand[i].eachRfullsearchstatus[j][k] because i use kth improvement here. if found improvement between [2][3], flag [2][3], next time need to find [2][3] again but for [2][0],[2][1],...,[2][r] noneed
struct fullsearch 
{	
	int	**eachRfullsearchstatus; //recod the each route full search status with respect to each operator, once found improvement, all operator for this route is declared to zero
}*operand; //globalinfo; //globalinfo is used to record the status of which route has been changed, it record in globalinfo.Rchange[0], [1] 

bool ***FullSearch;
//struct LNSremoval
//{
//	float probability;
//	float cutpoints;
//
//};

//void VNS_kth_improvement(float *x, float *y) 
//{
//	ofstream recordTIME("TIME.txt", ios::app);
//	ofstream violation("Infeasible_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream timefile("16.Time_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream localbest_sol("30.LOCAL_BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream VNS_multilevel("35.VNSMultiLevel_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream EV_nolearning ("1.Learning_" + std::to_string( I ) + ".txt", ios::out);
//	int Total_LSmove=0;//to evaluate effect of learning
//	float start_s = clock();// the code you wish to time goes here
//
//	
//	//******************************************** STEP 1: INITIALIZATION ********************************************//
//	//============================================ all predefined variables =================================================//
//	int max_neigbor = 5; //7 neighbourhoods
//	//int max_level = 9; //9 levels of local search improvement 
//	//int maxLocalSearch = max_level-1; //excluding last LS:2-opt for freq use
//	int maxLocalSearch = 9;//9 levels of local search improvement //ONLY 6 will be performed
//	int NbDiv = 0;
//	int maxDiv = min(6, GLOBAL_NO_ROUTE/2);
//	int LB_Ndel_min = max (5.00, (0.25*SIZE));
//	int UB_Ndel_min = min (400.00, (0.5*SIZE));
//	int Ndel_min = LB_Ndel_min; //min number of iteration for each route for diversification
//	int RS = 6; //number of removal strategies
//	//============================================= all predefined variables ================================================//
//
//	int TOTAL_DEMAND=0;
//	for (int i = 0; i < SIZE; i++)
//	{
//		TOTAL_DEMAND = TOTAL_DEMAND+demand[i];
//	}//just to chack the overall demand is correct at the end of VNS multi-level
//	
//	//============= Copy Local_BEST from GLOBAL_BEST ===========//
//	for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
//	{
//		for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
//		{
//			LOCAL[i][j] = GLOBAL[i][j];
//		}
//		LOCAL_Rcost[i] = GLOBAL_Rcost[i];
//		LOCAL_capa[i] = GLOBAL_capa[i];
//		LOCAL_SAIZ[i] = GLOBAL_SAIZ[i];
//	}
//	LOCAL_BEST_COST = GLOBAL_BEST_COST;
//	LOCAL_NO_ROUTE = GLOBAL_NO_ROUTE;
//
//	int div=0;
//	while (NbDiv <= maxDiv) //#############################################################################################################################
//	{
//		float violateThreshold = INFEASIBLE; //reinitialize to original value after diversification
//		int found_GLOBEST_status = 0;//must put before infeasibility check, otherwise found_GLOBEST_status is not recognise after goto violateDiverstart
//		localbest_sol << "Multi-level NbDiv= " <<NbDiv<<endl;
//		
//		int no_empty_route=0;
//		cout<<"LOCAL_NO_ROUTE after restartAfterDiver= "<<LOCAL_NO_ROUTE<<endl;
//		VNS_multilevel<<"LOCAL_NO_ROUTE after restartAfterDiver= "<<LOCAL_NO_ROUTE<<endl;
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			cout<<i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
//			VNS_multilevel << i <<' '<< LOCAL_Rcost[i] <<' '<< LOCAL_SAIZ[i] <<' '<< LOCAL_capa[i]<<' '<<' ';
//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//			{
//				VNS_multilevel<<LOCAL[i][j]<< ' ';
//				cout<<LOCAL[i][j]<< ' ';
//			}
//			cout<<endl;
//			VNS_multilevel<<endl;
//		}
//		cout<<"LOCAL_BEST_COST= "<<LOCAL_BEST_COST<<endl;
//		VNS_multilevel<<"LOCAL_BEST_COST= "<<LOCAL_BEST_COST<<endl;
//
//		//******************************************** STEP 2: ADD EMPTY ROUTE ********************************************//
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			if (LOCAL_SAIZ[i] == 0)
//				no_empty_route++; //to count how many empty route
//		}
//		if (no_empty_route == 1)
//		{
//			goto skipaddemptyR; //if empty route=1, skip add empty route
//		}
//		else if (no_empty_route == 0)
//		{
//			//************************************add one empty route
//			LOCAL[LOCAL_NO_ROUTE][0] = SIZE; //depot
//			LOCAL[LOCAL_NO_ROUTE][1] = SIZE; //depot
//			LOCAL_capa[LOCAL_NO_ROUTE] = 0;
//			LOCAL_SAIZ[LOCAL_NO_ROUTE] = 0;
//			LOCAL_Rcost[LOCAL_NO_ROUTE] = 0;
//			LOCAL_NO_ROUTE++;
//			//************************************end of add one empty route
//		}
//		else if (no_empty_route > 1) //if more than 1 empty route, do not copy the empty route and add one at the end
//		{
//			int k=0; //for new route index
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			{
//				if (LOCAL_SAIZ[i] == 0)
//				{
//					continue;
//				}
//				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//				{
//					LOCAL[k][j] = LOCAL[i][j];
//				}
//				LOCAL_SAIZ[k] = LOCAL_SAIZ[i];
//				LOCAL_capa[k] = LOCAL_capa[i];
//				LOCAL_Rcost[k] = LOCAL_Rcost[i];
//				k++;
//			}
//			LOCAL_NO_ROUTE = k;
//
//			//************************************add one empty route
//			LOCAL[LOCAL_NO_ROUTE][0] = SIZE ;//depot
//			LOCAL[LOCAL_NO_ROUTE][1] = SIZE; //depot
//			LOCAL_capa[LOCAL_NO_ROUTE] = 0;
//			LOCAL_SAIZ[LOCAL_NO_ROUTE] = 0;
//			LOCAL_Rcost[LOCAL_NO_ROUTE] = 0;
//			LOCAL_NO_ROUTE++;
//			//************************************end of add one empty route
//		}
//
//		cout<<"LOCAL_NO_ROUTE before skipaddemptyR= "<<LOCAL_NO_ROUTE<<endl;
//		VNS_multilevel<<"LOCAL_NO_ROUTE before skipaddemptyR= "<<LOCAL_NO_ROUTE<<endl;
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			cout << i <<' '<< LOCAL_Rcost[i] <<' '<< LOCAL_SAIZ[i] <<' '<< LOCAL_capa[i]<<' '<<' ';
//			VNS_multilevel << i <<' '<< LOCAL_Rcost[i] <<' '<< LOCAL_SAIZ[i] <<' '<< LOCAL_capa[i]<<' '<<' ';
//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//			{
//				cout<<LOCAL[i][j]<< ' ';
//				VNS_multilevel<<LOCAL[i][j]<< ' ';
//			}
//			cout<<endl;
//			VNS_multilevel<<endl;
//		}
//	skipaddemptyR: //if empty route=1, skip add empty route
//		cout<<"LOCAL_NO_ROUTE= "<< LOCAL_NO_ROUTE<<endl;
//		VNS_multilevel<<"LOCAL_NO_ROUTE= "<< LOCAL_NO_ROUTE<<endl;
//
//		no_routes = LOCAL_NO_ROUTE;
//
//		float *cost_of_removing = new float[SIZE + 1];//cost_of_removing[][] only used in LS, not shaking
//		float *Oricost_of_removing = new float[SIZE + 1];
//
//		//================= Record route change status from shaking and best improvement======================//added 15Sept2015
//		//Route = new Rmodified[LOCAL_NO_ROUTE];
//		RchangedStatus = new bool[LOCAL_NO_ROUTE];
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			//Route[i].RchangedStatus=0;//initialize to 0 for every route, they have not been changed
//			RchangedStatus[i]=false;//initialize to 0 for every route, they have not been changed
//		}
//		//CustaffectedStatus = new bool [SIZE+1];
//		//for (int i = 0; i <= SIZE; i++)
//		//{
//		//	CustaffectedStatus[i] = false;//initialize to false initially
//		//}
//		
//		//============= Copy VEHICLE from LOCAL_BEST ===========// guided shake needs to use VEHICLE
//		bool copyalready;
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)//added 22Apr15
//		{
//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//			{
//				VEHICLE[i][j] = LOCAL[i][j];
//				CumDist[i][j] = LOCALCumDist[i][j];
//			}
//			saiz[i] = LOCAL_SAIZ[i]; 
//			route_cost[i] = LOCAL_Rcost[i];
//			distance_cost[i] = LOCAL_distance_cost[i];
//			total_demand[i] = LOCAL_capa[i];
//			space_available[i] = (CAPACITY*(1+violateThreshold)) - (total_demand[i]);
//			distance_available[i] = (DISTANCE*(1+violateThreshold)) - (distance_cost[i]);
//			//space_available[i] = CAPACITY - total_demand[i];
//			//distance_available[i] = DISTANCE - distance_cost[i];
//		}
//		copyalready = true;
//		//============= End of Copy VEHICLE from LOCAL_BEST ===========//
//		
//		find_all_cost_of_removing (cost_of_removing);//cost_of_removing[][] only used in LS, not shaking, this is the first time calculating cost_of_removing
//		for (int f = 0; f < SIZE; f++) //copy from cost_of_removing
//		{
//			 Oricost_of_removing[f] = cost_of_removing[f];
//		}
//
//		int Neighbour_k = 1; //Neighbour_k 1 = (1-0), Neighbour_k 2 = (1-1), Neighbour_k 3 = (2-1), Neighbour_k 4 = (2-0)
//
//		int total_time = 0;
//		float maxCPUtime = 10000/(CLOCKS_PER_SEC);
//		cout << "maxCPUtime= " <<maxCPUtime<<endl;
//		bool foundLOCALinCurrNeighbor = false;//Do Dijkstra refinement if found LOCAL BEST in any neighbourhood in this diversification, otherwise, just do refinement for VEHICLEE
//		//###################################### STEP 3(a) Shaking ######################################//
//		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ k+1 neighborhood starts here @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
//		while (Neighbour_k <= max_neigbor)	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//		{	
//			if (violateThreshold < 0.01) //if smaller than 1%, make it zero
//				violateThreshold = 0;
//
//			//**************all insertion and deletion starts from		1 to saiz because the VEHICLE starts from depot, end with depot**************//
//			//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//
//			route_CGravity = new float*[LOCAL_NO_ROUTE];
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			{
//				route_CGravity[i] = new float[2]; //2 columns consists of x and y
//			}
//			custRGravity = new float[LOCAL_NO_ROUTE];
//			sorted_custRGravity = new float*[LOCAL_NO_ROUTE];
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			{
//				custRGravity[i] = 0; //initialize
//				sorted_custRGravity[i] = new float[5]; //[0]sorted distance, [1]sorted route index, [2]1/distance, [3]d/sum(d), [4]cumulative 
//			}
//
//		//reshake: //if no feasible shake, Neighbour_k++ and goto reshake //put comment on 14Oct2015, put reshake after copy
//			//for (int i = 0; i < 5; i++) //reinitialize after_shake_route_change[6]
//			//{
//			//	after_shake_route_change[i] = -1;
//			//}
//			//after_shake_route_changePtr = 0; //reinitialize pointer to 0, to keep track how many routes have been changed
//
//			//VEHICLE copied from LOCAL_BEST_ROUTE to be passed to shake(), need to recopy because when neighbourhoood++, VEHICLE[][] need to recopy from LOCAL_BEST although the first iteration this is not necessary because VEHICLE already copy before
//			if (copyalready == false)//if first time, VEHICLE already copy from LOCAL_BEST, noneed to copy, added on 12Oct2015
//			{
//				no_routes = LOCAL_NO_ROUTE;
//				for (int i = 0; i < LOCAL_NO_ROUTE; i++)//added 22Apr15
//				{
//					for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//					{
//						VEHICLE[i][j] = LOCAL[i][j];
//						CumDist[i][j] = LOCALCumDist[i][j];
//					}
//					saiz[i] = LOCAL_SAIZ[i]; 
//					route_cost[i] = LOCAL_Rcost[i];
//					distance_cost[i] = LOCAL_distance_cost[i];
//					total_demand[i] = LOCAL_capa[i];
//					space_available[i] = (CAPACITY*(1+violateThreshold)) - (total_demand[i]);
//					distance_available[i] = (DISTANCE*(1+violateThreshold)) - (distance_cost[i]);
//					//space_available[i] = CAPACITY - total_demand[i];
//					//distance_available[i] = DISTANCE - distance_cost[i];
//				}	
//			}
//			copyalready = false;//reinitialize to false after first time
//			
//			calculate_centreGravity(x, y); //MUST PUT THIS ONE HERE TO AVOID CALCULATING EVERYTIME IN EACH SHAKE		
//		reshake: //if no feasible shake, Neighbour_k++ and goto reshake //put here on 14 Oct 2015
//
//			int shake_status1 = 1;
//			int shake_status2 = 1;
//			if (Neighbour_k == 1) //(1-0)
//			{
//				cout<<"Entering Neighbour_k == 1 "<<endl;
//				shake_status1 = shake_1_0(VEHICLE, x, y); //the one going to shake is always the current best solution
//				shake_status2 = shake_1_0(VEHICLE, x, y);
//				if ((shake_status1 == 0) && (shake_status2 == 0))
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//			else if (Neighbour_k == 2) //1-1
//			{	
//				cout<<"Entering Neighbour_k == 2 "<<endl;
//				shake_status1 = shake_1_1(VEHICLE, x, y);
//		
//				if (shake_status1 == 0) 
//				{
//					Neighbour_k++; 
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//			else if (Neighbour_k == 3) //(2-0) - a pair from route A go to route B
//			{			
//				cout<<"Entering Neighbour_k == 3 "<<endl;
//				shake_status1 = shake_2_0_twoR(VEHICLE, x, y);
//				shake_status2 = shake_1_0(VEHICLE, x, y);
//				if ((shake_status1 == 0) && (shake_status2 == 0))
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//			else if (Neighbour_k == 4) //(2-0) - a pair from route A go to route B and route C
//			{			
//				cout<<"Entering Neighbour_k == 4 "<<endl;
//				shake_status1 = shake_2_0_threeR(VEHICLE, x, y);
//
//				if (shake_status1 == 0) 
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//			else if (Neighbour_k == 5) //(2-1) - a pair from route A swap with one customer from route B
//			{			
//				cout<<"Entering Neighbour_k == 5 "<<endl;
//				shake_status1 = shake_2_1_twoR(VEHICLE,  x, y);
//				shake_status2 = shake_1_0(VEHICLE, x, y);
//				if ((shake_status1 == 0) && (shake_status2 == 0))
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//			else if (Neighbour_k == 6) //(2-1) - first pair of cust must from route A to B, another cust from route A can go to route C
//			{			
//				cout<<"Entering Neighbour_k == 6 "<<endl;
//				shake_status1 = shake_2_1_threeR(VEHICLE, x, y);
//				shake_status2 = shake_1_1(VEHICLE, x, y);
//
//				if ((shake_status1 == 0) && (shake_status2 == 0))
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//			else if (Neighbour_k == 7) //(2-2) 
//			{
//				cout<<"Entering Neighbour_k == 7 "<<endl;
//				shake_status1 = shake_2_2(VEHICLE, x, y);
//
//				if (shake_status1 == 0) 
//				{
//					Neighbour_k++; 
//					//goto check_sol; //if no feasible shake at last neighborhod, dont perform LS, goto check_sol
//										//VEHICLE copied from vehicle to be passed to reoptimize() later //made comment on 15June2015
//					goto check_LOCAL_BEST; //if no feasible shake at last  neighborhod, dont perform LS, goto check_LOCAL_BEST, but need to copy vehicDepoCust[][] from vehicle[][] //added 15June2015 //check this if correct
//				}			
//			}
//			for (int r = 0; r < no_routes; r++)
//			{
//				cout<<"Route changed are: "<< ' ';
//				if (RchangedStatus[r] == true)
//				//if (Route[r].RchangedStatus == 1)
//				{
//					cout<<r<<' ';
//				}		
//				cout<<endl; 
//			}
//			cout<<"After shake " <<Neighbour_k <<endl;
//			cout<<endl<<"================================"<<endl;
//			VNS_multilevel<<endl<<"================================"<<endl;
//			cout<<"VEHICLE after shake " <<endl;
//			VNS_multilevel<<"VEHICLE after shake " <<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				VNS_multilevel<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//					VNS_multilevel<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//				VNS_multilevel<<endl;
//			}
//	
//			float afterShake_cost = 0.0; //to record the current cost so that later after kth improvement can check if the solution got improved
//			for (int i = 0; i < no_routes; i++)
//			{
//				afterShake_cost = afterShake_cost + route_cost[i];
//			}
//			cout<<"afterShake_cost= "<<afterShake_cost<<endl;
//			VNS_multilevel<<"afterShake_cost= "<<afterShake_cost<<endl;
//
//			//========================= To update cost_of_removing, gain_matrix[] and info-matrix[][] after shake===============================//
//			partialupdate_costremoving(cost_of_removing);//update according to RchangedStatus
//			reinitializeRchangedStatus (); //initialize to 0 for every route
//			//int r1 = route_change[0];
//			//int r2 = route_change[1];
//			//int r3 = route_change[2];	
//
//			//cout<<"r1= " <<r1 << ' ' << "r2= " <<r2 << ' ' << "r3= " <<r3 << endl;
//			//cout<<"After shake " <<Neighbour_k <<endl;
//	
//			//for (int r = 0; r < after_shake_route_changePtr; r++)
//			//{
//			//	cout<<"r" <<r<<" = "<< after_shake_route_change[r] <<' '; 
//			//}
//			//cout<<"After shake " <<Neighbour_k <<endl;
//
//
//			////=================================================find cost of removing for the first time=====================================//////////////////////
//			//for (int r = 0; r < after_shake_route_changePtr; r++)
//			//{
//			//	int f = after_shake_route_change[r];	//this store the route number
//			//	if (saiz[f]==0)
//			//		continue;
//			//	for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
//			//	{
//			//		cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
//			//	}
//			//}
//			
//			//find_all_cost_of_removing (cost_of_removing);//cost_of_removing[][] only used in LS, not shaking, this is the first time calculating cost_of_removing
//	
//	
//			/////////////// ================================ MULTI_LEVEL VNS ==================================================== ////////////////////////////
//			int Kth = 5; //kth improvement, find the best value in K gain !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
//			int level = 1; //initialize local search to 1
//			bool sameroute = false; //false if different route, true if same route
//			int no_attribute = 13; //[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
//			
//			//=========== declare data structure ======================//
//			operand = new fullsearch[maxLocalSearch+1]; //there are (maxLocalSearch) of operand that contains info in struct , maxLocalSearch start from 1
//			for (int i = 1; i <= maxLocalSearch; i++)
//			{
//				operand[i].eachRfullsearchstatus = new int* [no_routes];
//			}
//			
//			for (int i = 1; i <= maxLocalSearch; i++)
//			{
//				for (int j = 0; j < no_routes; j++)
//				{
//					operand[i].eachRfullsearchstatus[j] = new int [no_routes];
//				}
//			}
//			initializeEachRfullsearchstatus (maxLocalSearch);
//
//			//=========== end of declare data structure ======================//
//
//			//string *LS = new string[maxLocalSearch+1]; //to record the LS that found improvement, just to print out purpose to see which LS is effective //record level start from 1 until (maxLevel+1)
//			//for (int i = 0; i <= maxLocalSearch; i++)
//			//{
//			//	LS[i] = "No";//initialize all not found improvement
//			//}
//			float **kGain = new float* [Kth]; //record information for kth level gain, 
//			for (int i = 0; i < Kth; i++)
//			{
//				kGain[i] = new float [no_attribute];
//			}
//
//
//		while (level <= maxLocalSearch)
//		{
//			//redo_LS://1) if no improvement found in each level of LS, 2) if cost is better than aftershake cost set level=1, 3) if LS level has not finished all level level++ and goto redo_LS
//			if (level == 1) //the first level is 1-0 
//			{
//				//operand[level].levelID = level;
//				int number = find_k_1_0(cost_of_removing, kGain, Kth, level);
//				Total_LSmove++;
//				if (Total_LSmove %500 == 0)
//				{
//					float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
//					EV_nolearning << Total_LSmove<<' '<<' '<<runtime<<' '<<' '<<GLOBAL_BEST_COST;
//				}
//				cout<<"in level "<<level<<": find_k_1_0, best number= "<<number << endl;
//				VNS_multilevel<<"in level 2: find_k_1_0, best number= "<<number << endl;
//
//				if (number == -1)//if no improvement
//				{
//					level++;
//					continue;
//				}
//								
//				if (kGain[number][2] == kGain[number][3])
//					sameroute = true; //same route
//				else
//					sameroute = false; //different route
//
//				cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP= "<<kGain[number][4]<<" ToP= "<<kGain[number][5]<< endl;
//				VNS_multilevel<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP= "<<kGain[number][4]<<" ToP= "<<kGain[number][5]<< endl;
//				//route_file<<"in level 2: find_k_1_0, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
//				for (int i = 1; i <= maxLocalSearch; i++)
//				{		
//					int j = kGain[number][2]; //Route1
//					int k = kGain[number][3]; //Route2
//					for (int m = 0; m < no_routes; m++)
//					{
//						operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
//						operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
//					}
//					if (sameroute == false)
//					{
//						for (int m = 0; m < no_routes; m++)
//						{	
//							operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
//							operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
//						}
//					}
//				}
//				
//				//LS[level] = "Yes";//record the level that found improvement
//				//cost_of_removing is updated in insert()
//				insert_1_0_2nd(cost_of_removing, Kth, number, kGain, VEHICLE, sameroute);
//				cout<<endl<<endl;
//				VNS_multilevel<<endl<<endl;
//				cout<<"========== VEHICLE after insert in level 2 find_k_1_0==================="<<endl;
//				VNS_multilevel<<"========== VEHICLE after insert in level 2 find_k_1_0==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					VNS_multilevel<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//						VNS_multilevel<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//					VNS_multilevel<<endl;
//				}
//				goto check_sol;//after each LS goto check_sol
//			}
//
//			else if (level == 2)
//			{
//				//operand[level].levelID = level;
//				int number = find_k_1_1(cost_of_removing, kGain, Kth, level);
//				Total_LSmove++;
//				if (Total_LSmove %500 == 0)
//				{
//					float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
//					EV_nolearning << Total_LSmove<<' '<<' '<<runtime<<' '<<' '<<GLOBAL_BEST_COST;
//				}
//				cout<<"in level "<<level<<": find_k_1_1, best number= "<<number <<endl;
//				VNS_multilevel<<"in level "<<level<<": find_k_1_1, best number= "<<number <<endl;
//
//				if (number == -1)//if no improvement
//				{	
//					level++;
//					continue;
//				}
//
//				if (kGain[number][2] == kGain[number][3])
//					sameroute = true; //same route
//				else
//					sameroute = false; //different route
//
//				cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<< " FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<<endl;
//				VNS_multilevel<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<< " FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<<endl;
//				//route_file<<"in level 4: find_k_1_1, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				
//				//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
//				for (int i = 1; i <= maxLocalSearch; i++)
//				{		
//					int j = kGain[number][2]; //Route1
//					int k = kGain[number][3]; //Route2
//					for (int m = 0; m < no_routes; m++)
//					{
//						operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
//						operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
//					}
//					if (sameroute == false)
//					{
//						for (int m = 0; m < no_routes; m++)
//						{	
//							operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
//							operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
//						}
//					}
//				}
//				//LS[level] = "Yes";//record the level that found improvement
//				//cost_of_removing is updated in insert()
//				insert_1_1_2nd(cost_of_removing, Kth, number, kGain, VEHICLE, sameroute);
//				cout<<endl<<endl;
//				VNS_multilevel<<endl<<endl;
//				cout<<"========== VEHICLE after insert in level 4: find_k_1_1==================="<<endl;
//				VNS_multilevel<<"========== VEHICLE after insert in level 4: find_k_1_1==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					VNS_multilevel<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//						VNS_multilevel<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//					VNS_multilevel<<endl;
//				}
//				goto check_sol;//after each LS goto check_sol
//				
//			}
//			else if (level == 3)
//			{
//				//operand[level].levelID = level;
//				int number = find_k_2_1(cost_of_removing, kGain, Kth, level);
//				Total_LSmove++;
//				if (Total_LSmove %500 == 0)
//				{
//					float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
//					EV_nolearning << Total_LSmove<<' '<<' '<<runtime<<' '<<' '<<GLOBAL_BEST_COST;
//				}
//				cout<<"in level "<<level<<": find_k_2_1, best number= "<<number <<endl;
//				VNS_multilevel<<"in level "<<level<<": find_k_2_1, best number= "<<number <<endl;
//
//				if (number == -1)//if no improvement
//				{
//					level++;
//					continue;
//				}
//								
//				if (kGain[number][2] == kGain[number][3])
//					sameroute = true; //same route
//				else
//					sameroute = false; //different route
//
//				cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<< " FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<<endl;
//				VNS_multilevel<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<< " FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<<endl;
//				//route_file<<"in level 6: find_k_2_1, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				
//				//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
//				for (int i = 1; i <= maxLocalSearch; i++)
//				{		
//					int j = kGain[number][2]; //Route1
//					int k = kGain[number][3]; //Route2
//					for (int m = 0; m < no_routes; m++)
//					{
//						operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
//						operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
//					}
//					if (sameroute == false)
//					{
//						for (int m = 0; m < no_routes; m++)
//						{	
//							operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
//							operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
//						}
//					}
//				}
//				//LS[level] = "Yes";//record the level that found improvement
//				//cost_of_removing is updated in insert()
//				insert_2_1_2nd(cost_of_removing, Kth, number, kGain, VEHICLE, sameroute);
//				cout<<endl<<endl;
//				VNS_multilevel<<endl<<endl;
//				cout<<"========== VEHICLE after insert in level 6: find_k_2_1==================="<<endl;
//				VNS_multilevel<<"========== VEHICLE after insert in level 6: find_k_2_1==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					VNS_multilevel<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//						VNS_multilevel<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//					VNS_multilevel<<endl;
//				}
//				goto check_sol;//after each LS goto check_sol
//			}
//
//			else if (level == 4)
//			{
//				//operand[level].levelID = level;
//				int number = find_k_2_0(cost_of_removing, kGain, Kth, level);
//				Total_LSmove++;
//				if (Total_LSmove %500 == 0)
//				{
//					float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
//					EV_nolearning << Total_LSmove<<' '<<' '<<runtime<<' '<<' '<<GLOBAL_BEST_COST;
//				}
//				cout<<"in level "<<level<<": find_k_2_0, best number= "<<number <<endl;
//				VNS_multilevel<<"in level "<<level<<": find_k_2_0, best number= "<<number <<endl;
//
//				if (number == -1)//if no can be improvement
//				{
//					level++;
//					continue;
//				}
//								
//				if (kGain[number][2] == kGain[number][3])
//					sameroute = true; //same route
//				else
//					sameroute = false; //different route
//
//				cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<< " FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<<endl;
//				VNS_multilevel<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<< " FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<<endl;
//				//route_file<<"in level 8: find_k_2_0, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				
//				//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
//				for (int i = 1; i <= maxLocalSearch; i++)
//				{		
//					int j = kGain[number][2]; //Route1
//					int k = kGain[number][3]; //Route2
//					for (int m = 0; m < no_routes; m++)
//					{
//						operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
//						operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
//					}
//					if (sameroute == false)
//					{
//						for (int m = 0; m < no_routes; m++)
//						{	
//							operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
//							operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
//						}
//					}
//				}
//
//				//LS[level] = "Yes";//record the level that found improvement
//				//cost_of_removing is updated in insert()
//				insert_2_0_2nd(cost_of_removing, Kth, number, kGain, VEHICLE, sameroute);
//				cout<<endl<<endl;
//				VNS_multilevel<<endl<<endl;
//				VNS_multilevel<<"========== VEHICLE after insert in level 8: find_k_2_0==================="<<endl;
//				cout<<"========== VEHICLE after insert in level 8: find_k_2_0==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					VNS_multilevel<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//						VNS_multilevel<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//					VNS_multilevel<<endl;
//				}
//				goto check_sol; //after each LS goto check_sol
//			}
//			
//			else if (level == 5)
//			{
//				//operand[level].levelID = level;
//				int number = find_k_2_2swap(cost_of_removing, kGain, Kth, level);
//				Total_LSmove++;
//				if (Total_LSmove %500 == 0)
//				{
//					float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
//					EV_nolearning << Total_LSmove<<' '<<' '<<runtime<<' '<<' '<<GLOBAL_BEST_COST;
//				}
//				cout<<"in level "<<level<<": find_k_2_2swap, best number= "<<number << endl;
//				VNS_multilevel<<"in level "<<level<<": find_k_2_2swap, best number= "<<number << endl;
//
//				if (number == -1)//if no improvement found
//				{
//					level++;
//					continue;
//				}
//								
//				if (kGain[number][2] == kGain[number][3])
//					sameroute = true; //same route
//				else
//					sameroute = false; //different route
//
//				cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//				VNS_multilevel<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//				//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
//				for (int i = 1; i <= maxLocalSearch; i++)
//				{		
//					int j = kGain[number][2]; //Route1
//					int k = kGain[number][3]; //Route2
//					for (int m = 0; m < no_routes; m++)
//					{
//						operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
//						operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
//					}
//					if (sameroute == false)
//					{
//						for (int m = 0; m < no_routes; m++)
//						{	
//							operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
//							operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
//						}
//					}
//				}
//
//				//cost_of_removing is updated in insert()
//				insert_2_2_swap2nd(cost_of_removing, Kth, number, kGain, VEHICLE, sameroute);
//				cout<<endl<<endl;
//				VNS_multilevel<<endl<<endl;
//				cout<<"========== vehicle after insert in level "<<level<<": find_k_2_2swap==================="<<endl;
//				VNS_multilevel<<"========== vehicle after insert in level "<<level<<": find_k_2_2swap========="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					VNS_multilevel<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//						VNS_multilevel<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//					VNS_multilevel<<endl;
//				}
//				goto check_sol;//after each LS , compare with aftershakecost
//			}
//			else if (level == 6)
//			{
//				//operand[level].levelID = level;
//				int number = find_two_optintra(kGain, Kth, level);
//				Total_LSmove++;
//				if (Total_LSmove %500 == 0)
//				{
//					float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
//					EV_nolearning << Total_LSmove<<' '<<' '<<runtime<<' '<<' '<<GLOBAL_BEST_COST;
//				}
//				cout<<"in level "<<level<<": find_two_optintra, best number= "<<number << endl;
//				VNS_multilevel<<"in level "<<level<<": find_two_optintra, best number= "<<number << endl;
//
//				if (number == -1)//if no improvement found
//				{
//					level++;
//					continue;
//				}
//								
//				sameroute = true; //same route
//
//				cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//				VNS_multilevel<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//				//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
//				for (int i = 1; i <= maxLocalSearch; i++)
//				{		
//					int j = kGain[number][2]; //Route1
//					
//					for (int m = 0; m < no_routes; m++)
//					{
//						operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
//						operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
//					}
//					
//				}
//
//				//cost_of_removing is updated in insert()
//				insert_2optintra(cost_of_removing, Kth, number, kGain, VEHICLE);
//				cout<<endl<<endl;
//				VNS_multilevel<<endl<<endl;
//				cout<<"========== vehicle after insert in level "<<level<<": find_two_optintra==================="<<endl;
//				VNS_multilevel<<"========== vehicle after insert in level "<<level<<": find_two_optintra========="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					VNS_multilevel<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//						VNS_multilevel<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//					VNS_multilevel<<endl;
//				}
//				goto check_sol;//after each LS , compare with aftershakecost
//			}
//			else if (level == 7)
//			{
//				//operand[level].levelID = level;
//				int number = find_two_optinter(kGain, Kth, level);
//				Total_LSmove++;
//				if (Total_LSmove %500 == 0)
//				{
//					float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
//					EV_nolearning << Total_LSmove<<' '<<' '<<runtime<<' '<<' '<<GLOBAL_BEST_COST;
//				}
//				cout<<"in level "<<level<<": find_two_optinter, best number= "<<number << endl;
//				VNS_multilevel<<"in level "<<level<<": find_two_optinter, best number= "<<number << endl;
//
//				if (number == -1)//if no improvement found
//				{
//					level++;
//					continue;
//				}
//								
//				sameroute = false; //diff route
//
//				cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//				VNS_multilevel<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//				//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
//				for (int i = 1; i <= maxLocalSearch; i++)
//				{		
//					int j = kGain[number][2]; //Route1
//					int k = kGain[number][3]; //Route2
//					for (int m = 0; m < no_routes; m++)
//					{
//						operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
//						operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
//						operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
//						operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
//					}
//				}
//
//				//cost_of_removing is updated in insert()
//				insert_2optinter(cost_of_removing, Kth, number, kGain, VEHICLE);
//				cout<<endl<<endl;
//				VNS_multilevel<<endl<<endl;
//				cout<<"========== vehicle after insert in level "<<level<<": find_two_optinter==================="<<endl;
//				VNS_multilevel<<"========== vehicle after insert in level "<<level<<": find_two_optinter========="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					VNS_multilevel<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//						VNS_multilevel<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//					VNS_multilevel<<endl;
//				}
//				goto check_sol;//after each LS , compare with aftershakecost
//			}
//			else if (level == 8)
//			{
//				//operand[level].levelID = level;
//				int number = find_crossTail2(kGain, Kth, level);
//				Total_LSmove++;
//				if (Total_LSmove %500 == 0)
//				{
//					float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
//					EV_nolearning << Total_LSmove<<' '<<' '<<runtime<<' '<<' '<<GLOBAL_BEST_COST;
//				}
//				cout<<"in level "<<level<<": find_crossTail2, best number= "<<number << endl;
//				VNS_multilevel<<"in level "<<level<<": find_crossTail2, best number= "<<number << endl;
//
//				if (number == -1)//if no improvement found
//				{
//					level++;
//					continue;
//				}
//								
//				sameroute = false; //diff route
//
//				cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//				VNS_multilevel<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//				//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
//				for (int i = 1; i <= maxLocalSearch; i++)
//				{		
//					int j = kGain[number][2]; //Route1
//					int k = kGain[number][3]; //Route2
//					for (int m = 0; m < no_routes; m++)
//					{
//						operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
//						operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
//						operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
//						operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
//					}
//				}
//
//				//cost_of_removing is updated in insert()
//				insert_crosstail(cost_of_removing, Kth, number, kGain, VEHICLE);
//				cout<<endl<<endl;
//				VNS_multilevel<<endl<<endl;
//				cout<<"========== vehicle after insert in level "<<level<<": find_crossTail2==================="<<endl;
//				VNS_multilevel<<"========== vehicle after insert in level "<<level<<": find_crossTail2========="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					VNS_multilevel<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//						VNS_multilevel<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//					VNS_multilevel<<endl;
//				}
//				goto check_sol;//after each LS , compare with aftershakecost
//			}
//			else if (level == 9)
//			{
//				//operand[level].levelID = level;
//				int number = find_CROSS(kGain, Kth, level);
//				Total_LSmove++;
//				if (Total_LSmove %500 == 0)
//				{
//					float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
//					EV_nolearning << Total_LSmove<<' '<<' '<<runtime<<' '<<' '<<GLOBAL_BEST_COST;
//				}
//				cout<<"in level "<<level<<": find_CROSS, best number= "<<number << endl;
//				VNS_multilevel<<"in level "<<level<<": find_CROSS, best number= "<<number << endl;
//
//				if (number == -1)//if no improvement found
//				{
//					level++;
//					continue;
//				}
//								
//				sameroute = false; //diff route
//
//				cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//				VNS_multilevel<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//				//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
//				for (int i = 1; i <= maxLocalSearch; i++)
//				{		
//					int j = kGain[number][2]; //Route1
//					int k = kGain[number][3]; //Route2
//					for (int m = 0; m < no_routes; m++)
//					{
//						operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
//						operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
//						operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
//						operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
//					}
//				}
//
//				//cost_of_removing is updated in insert()
//				insert_cross(cost_of_removing, Kth, number, kGain, VEHICLE);
//				cout<<endl<<endl;
//				VNS_multilevel<<endl<<endl;
//				cout<<"========== vehicle after insert in level "<<level<<": find_CROSS==================="<<endl;
//				VNS_multilevel<<"========== vehicle after insert in level "<<level<<": find_CROSS========="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					VNS_multilevel<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//						VNS_multilevel<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//					VNS_multilevel<<endl;
//				}
//				goto check_sol;//after each LS , compare with aftershakecost
//			}
//			
//		check_sol: //after each LS or if no feasible shake at last module(neighborhod), goto check_sol, compare with aftershakecost
//
//			float t_cost = 0; //reinitialize temporary total cost
//			//========== check if temporary cost is less than BEST COST ============//
//			for (int i = 0; i < no_routes; i++)
//			{
//				if (saiz[i] == 0)
//				{
//					route_cost[i] =0;//if it is empty route
//					total_demand[i]=0;
//				}
//				t_cost = t_cost + route_cost[i];
//			}
//
//			//======================================================Display route=======================================================================//
//			cout << "==================Routes (in VNS kth improvement) after LS Level " << level <<" ===================================== " << endl;
//			VNS_multilevel<< "==================Routes (in VNS kth improvement) after LS Level " << level <<" ===================================== " << endl;
//			//route_file << "==================Routes (in VNS kth improvement) after LS Level " << level <<" ===================================== " << endl;
//			float total_cost = 0.0;
//			int total_cust = 0;
//			int sum_demand=0;
//			for (int g = 0; g < no_routes; g++)
//			{
//				if (saiz[g] == 0)
//				{
//					route_cost[g] = 0;
//					total_demand[g]=0;
//					distance_cost[g] = 0;
//				}
//				cout << g << ' '<< route_cost[g] << ' '<< saiz[g] << ' '<<total_demand[g]<<' '<<' ';
//				VNS_multilevel<< g << ' '<< route_cost[g] << ' '<< saiz[g] << ' '<<total_demand[g]<<' '<<' ';
//
//				for (int h = 0; h <= saiz[g]+1; h++)
//				{
//					cout << VEHICLE[g][h] << ' ';
//					VNS_multilevel << VEHICLE[g][h] << ' ';
//				}
//				cout<<endl;
//				VNS_multilevel << endl;
//
//				total_cust = total_cust+saiz[g];
//				total_cost = total_cost + route_cost[g];
//				sum_demand = sum_demand+total_demand[g];
//			}
//			cout << "Total customers = " << total_cust << endl<< "Total cost = " << total_cost << endl<< "Sum_demand = " << sum_demand <<endl<< "TOTAL_DEMAMD = " << TOTAL_DEMAND<<endl;
//			VNS_multilevel<< "Total customers = " << total_cust << endl<< "Total cost = " << total_cost << endl<< "Sum_demand = " << sum_demand <<endl<< "TOTAL_DEMAMD = " << TOTAL_DEMAND<<endl;
//			//route_file<< "Total customers = " << total_cust << endl<< "Total cost = " << total_cost << endl<< "Sum_demand = " << sum_demand <<endl<< "TOTAL_DEMAMD = " << TOTAL_DEMAND<<endl;
//			//route_file<< "After shake cost = " <<afterShake_cost <<endl;
//			//route_file<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
//
//			if ((afterShake_cost - t_cost) > epsilonn)//t_cost < afterShake_cost
//			{
//				//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ for infeasibility check @@@@@@@@@@@@@@@@@@@@@@@@//
//				for (int i = 0; i < no_routes; i++)
//				{
//					if ((total_demand[i]>CAPACITY) || (distance_cost[i] - DISTANCE > epsilon))//for infeasibility check
//					{
//						violation << "In VNS multi-level after module "<< Neighbour_k <<endl;
//						violation<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//						for (int j = 0; j <= saiz[i]+1; j++)
//						{
//							violation << VEHICLE[i][j]<<' ';
//						}
//						violation<<endl;
//						violateThreshold = violateThreshold/2; //divide by half if the solution after LS still violate constraint
//						//if (Neighbour_k < max_neigbor)
//						//{
//							Neighbour_k++; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//							goto skip_check_local;
//					
//						//}
//						//else //if maximum neighbourhood is reached
//						//	goto violateDiverstart; //if violation, skip all the process of compare LOCAL_BEST and GLOBAL_BEST, check if max diversification is reached
//					}
//				}
//				//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ End of (for infeasibility check) @@@@@@@@@@@@@@@@@@@@@@@@//
//				//try checking fesibility here next time!!!!!!!!!!!!!!!! to avoid  time wasted if route is infeasible after so many LS
//				//if((abs(t_cost-LOCAL_BEST_COST) <= 0.005) || (abs(t_cost-GLOBAL_BEST_COST) <= 0.005)) //added on 3April2015 to avoid recalculate if it is the same as LOCAl or GLOBAL BEST //for early escape
//				//	goto check_LOCAL_BEST;
//				afterShake_cost = t_cost;
//				level = 1;
//				//goto redo_LS; //go to LS level 1 if it is better than aftershake cost
//			}
//	
//			else //if not better than after shake cost
//			{
//				level = level+1;
//			}
//		}//end while (level <= maxLocalSearch)
//
//	check_LOCAL_BEST://if all LS are done, goto check_LOCAL_BEST //compare with LOCAL_BEST_COST
//			//======================================================Display route========================================================================
//			//route_file<< "==================Routes (in VNS kth improvement) after all LS before check LOCAL BEST===================================== " << endl;
//			cout << "==================Routes (in VNS kth improvement) after all LS before check LOCAL BEST===================================== " << endl;
//			cout<< "current Neighbour_k= "<< Neighbour_k<<endl;
//			float total_cost = 0.0;
//			int total_cust = 0;
//			for (int g = 0; g < no_routes; g++)
//			{
//				if (saiz[g] == 0)
//				{
//					route_cost[g] = 0;
//					distance_cost[g] = 0;
//				}
//				cout << g << ' '<< route_cost[g] << ' '<< saiz[g] << ' '<<total_demand[g]<<' '<<' ';
//				//route_file<< g << ' '<< route_cost[g] << ' '<< saiz[g] << ' ';
//				total_cust = total_cust + saiz[g];
//				for (int h = 0; h <= saiz[g]+1; h++)
//				{
//					cout << VEHICLE[g][h] << ' ';
//				}
//				cout << endl;
//				total_cost = total_cost + route_cost[g];
//			}
//			cout << "Total customers = " << total_cust << endl << "Total cost = " << total_cost << endl;
//			cout<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
//			//route_file<<"Total customers = " << total_cust << endl << "Total cost = " << total_cost << endl;
//			//route_file<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
//
//
//			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ for infeasibility check @@@@@@@@@@@@@@@@@@@@@@@@//
//			for (int i = 0; i < no_routes; i++)
//			{
//				if ((total_demand[i]>CAPACITY) || (distance_cost[i] - DISTANCE > epsilon))//for infeasibility check
//				{
//					cout << "Infeasibility check In VNS multi-level after module "<< Neighbour_k <<endl;
//					violation << "In VNS multi-level after module "<< Neighbour_k <<endl;
//					violation << i << ' '<< route_cost[i] << ' '<< saiz[i] << ' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						violation << VEHICLE[i][j]<<' ';
//					}
//					violation<<endl;
//					violateThreshold = violateThreshold/2; //divide by half if the solution after LS still violate constraint
//					//if (Neighbour_k < max_neigbor)
//					//{
//						Neighbour_k++; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//						goto skip_check_local;
//					
//					//}
//					//else //if maximum neighbourhood is reached
//					//	goto violateDiverstart; //if violation, skip all the process of compare LOCAL_BEST and GLOBAL_BEST, check if max diversification is reached
//				}
//			}
//			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ End of (for infeasibility check) @@@@@@@@@@@@@@@@@@@@@@@@//
//
//			if ((LOCAL_BEST_COST - total_cost) > epsilonn )//t_cost < LOCAL_BEST_COST
//			{
//				//int num_skip_r = 0;
//				//int r=0;//for route index because empty route has been deleted
//				//sort_solution(VEHICLE); //sort solution but cost_of_removing based on unsorted solution
//				LOCAL_BEST_COST = total_cost;
//				
//				for (int i = 0; i < no_routes; i++)
//				{
//					LOCAL_Rcost[i] = route_cost[i];//route cost
//					LOCAL_SAIZ[i] = saiz[i];//route size
//					LOCAL_capa[i] = total_demand[i];
//			
//					for (int k = 0; k <= saiz[i]+1; k++)
//					{
//						LOCAL[i][k] = VEHICLE[i][k]; //VEHICLE start with depot, end with depot
//					}
//				}
//				if (no_routes != LOCAL_NO_ROUTE)
//				{
//					cout<<"ERROR no_routes != LOCAL_NO_ROUTE in VNS multi-level)"<<endl;
//					getchar();
//				}
//				
//				//=================================record localbest solution in txt=====================================//
//				cout << "========================================================================== " << endl;
//				VNS_multilevel<< "========================================================================== " << endl;
//				//ofstream localbest_sol("30.LOCAL_BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
//				//=================================display final solution=====================================//
//				for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//				{
//					localbest_sol << i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
//					for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//					{
//						localbest_sol << LOCAL[i][j] << ' ';
//						VNS_multilevel<< LOCAL[i][j] << ' ';
//					}
//					localbest_sol<<endl;
//					VNS_multilevel<<endl;
//				}
//				localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"in VNS multi-level after shake Neighbour_k "<<Neighbour_k <<"level " <<level<< endl;	
//				VNS_multilevel<< "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"in VNS multi-level after shake Neighbour_k "<<Neighbour_k <<"level " <<level<< endl;	
//				//for (int l = 1; l <= maxLocalSearch; l++)
//				//{
//				//	localbest_sol << "LS["<<l<<"]= "<<LS[l]<<' '<< ' ';
//				//}
//				//localbest_sol <<endl;
//	
//				//======================= End of record localbest solution ===========================//
//				
//				//once found local best, copy to Oricost_of_removing, //if not better than local best solution and neighbourhood havent finished, use the Oricost_of_removing because shaking is based on local best, oriGain is updated evrytime found local best
//				//============================= copy to Oricost_of_removing ================================//
//				for (int i = 0; i < SIZE; i++)
//				{
//					Oricost_of_removing[i] = cost_of_removing[i];
//				}
//				calculate_centreGravity(x, y);
//				Neighbour_k = 1;	
//				foundLOCALinCurrNeighbor = true;
//			}//end if (total cost < LOCAL_BEST_COST)
//			else  //if is is not better than the incumbent
//			{
//				if (Neighbour_k == max_neigbor) //current neighbourhood is the last neighbourhood, perform dijkstra refinement
//				{
//					float cost = 0.0; //to be passed to dijkstra_refinement
//					//dijkstra_refinement always refine the LOCAL_BEST_SOL (incumbent best)
//					//vehicle[][] is not important, it will be reinitialize in dijkstra_refinement, and passed out from there
//					cout<<"Entering dijkstra_refinement in kth VNS"<<endl;	
//
//					if(foundLOCALinCurrNeighbor == true)//if found LOCAL BEST, use local best, else just use current VEHICLE
//					{
//						for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//						{
//							for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//							{
//								VEHICLE[i][j] = LOCAL[i][j];
//								CumDist[i][j] = LOCALCumDist[i][j];
//							}
//							route_cost[i] = LOCAL_Rcost[i];
//							distance_cost[i] = LOCAL_distance_cost[i];
//							saiz[i] = LOCAL_SAIZ[i];
//							total_demand[i] = LOCAL_capa[i];
//							space_available[i] = CAPACITY - total_demand[i];
//							distance_available[i] = DISTANCE - distance_cost[i];
//						}
//						no_routes = LOCAL_NO_ROUTE;
//					}
//
//					//sort_solution(VEHICLE);//sort first so that empty route will be at the end, dijkstra refinement does not consider empty route
//					//sort_solutionSMALLEST(VEHICLE);
//					////=============== copyto tempvec to compare whether route have changed or not after Dijkstra refinement ================//
//					//float *tempRcost = new float[no_routes];
//					//int *tempRcapa = new int[no_routes];
//					//int *tempRsize = new int[no_routes];
//					//int **tempvec = new int* [no_routes];
//					//for (int j = 0; j < no_routes; j++)
//					//{
//					//	tempvec[j] = new int [SIZE];
//					//}
//					//
//					//for (int i = 0; i < no_routes; i++)
//					//{
//					//	tempRcost[i] = route_cost[i];
//					//	tempRcapa[i] = total_demand[i];
//					//	tempRsize[i] = saiz[i];
//					//	for (int j = 0; j <= saiz[i]+1; j++)
//					//	{
//					//		tempvec[i][j] = VEHICLE[i][j]; //copy to tempvec[][] and both tempvec[][] and vehicle[][] are sorted
//					//	}
//					//}//copyto tempvec to compare whether route have changed or not after Dijkstra refinement
//
//					dijkstra_refinement(cost, VEHICLE); 
//
//					cout<<"cost after dijkstra_refinement= "<<cost<<endl;
//					//=================== if better than LOCAL_BEST, compare route  by route to see if the route has changed =====================//
//					//I want to avoid calculating the cost_of_removing for every route because the solution after Dijkstra Refinement is very similar to the original solution.
//					//**************************************** If Dijkstra refinement found better Local_best ***************************************//
//					if ((LOCAL_BEST_COST-cost) > epsilonn)//cost < LOCAL_BEST_COST
//					{
//						cout<< "Found new LOCAL_BEST right after Dijkstra REfinement"<<endl;
//						//getchar();
//						//sort_solution(VEHICLE);
//						//sort_solutionSMALLEST(VEHICLE);
//						//reinitializeRchangedStatus ();//initialize to 0 for every route
//
//						//******************* Begin of check each route has been modified or not after Dijkstra refinement *********************//
//						//for (int i = 0; i < no_routes; i++)
//						//{
//						//	if (abs(tempRcost[i] - route_cost[i]) > 0.001) //if route_cost is different
//						//	{
//						//		RchangedStatus[i] = true;
//						//		//Route[i].RchangedStatus = 1;
//						//	}
//						//	else //if cost is the same, compare size
//						//	{
//						//		if(tempRsize[i] != saiz[i]) //if saiz different, flag route
//						//		{
//						//			RchangedStatus[i] = true;
//						//			//Route[i].RchangedStatus = 1;
//						//			goto nex;//noneed to check total demand and order
//						//		}
//						//		if (tempRcapa[i] != total_demand[i])//if total demand different, flag route
//						//		{
//						//			RchangedStatus[i] = true;
//						//			//Route[i].RchangedStatus = 1;
//						//			goto nex; //noneed to check order
//						//		}
//						//		
//						//		bool reverse = false;
//						//		//check normal order
//						//		for (int w = 1; w <= saiz[i]; w++)
//						//		{
//						//			if(VEHICLE[i][w] == LOCAL[i][w])
//						//			{
//						//				reverse = false;
//						//				continue;
//						//			}
//						//			else
//						//				goto checkreverse;
//						//		}//end check normal order
//						//		
//						//		if (reverse == false)
//						//			goto nex;//confirm route no change
//						//		checkreverse:
//						//		//checkReverseSingleR(); //route maybe reverse because consider connecting endpoint in dijkstra refinement, if it is reverse, need to rewrite in original order because information recorded based on original order
//						//		int u = saiz[i];
//						//		for (int w = 1; w <= saiz[i]; w++)
//						//		{
//						//			if(VEHICLE[i][w] == LOCAL[i][u])
//						//			{
//						//				reverse = true;
//						//				continue;
//						//			}
//						//			else
//						//			{
//						//				RchangedStatus[i] = true;
//						//				//Route[i].RchangedStatus = 1;
//						//				goto nex;
//						//			}
//						//			u--;		
//						//		}//end check reverse
//						//		if(reverse == true) //if confirm it is reverse, copy the original route
//						//		{
//						//			for (int q = 1; q <= saiz[i]; q++)
//						//			{
//						//				VEHICLE[i][q] = tempvec[i][q];
//						//			}
//						//		}
//						//	}//end //if cost is the same, compare size
//						//	nex:;
//						//} 
//						////=========== update cost_of_removing, best_gain[][] if Dijkstra refinement found local_best
//						//partialupdate_costremoving(cost_of_removing); //based on RchangedStatus
//						//reinitializeRchangedStatus ();//initialize to 0 for every route
//						//******************* ENd of check each route has been modified or not after Dijkstra refinement *********************//
//						
//						//find all cost_of_removing and reinitialize the route because routes have been sorted and changed after Dijkstra refinement
//						find_all_cost_of_removing (cost_of_removing);//after Dijkstra, all route and position might have changed
//						reinitializeRchangedStatus ();//initialize to 0 for every route
//						initializeEachRfullsearchstatus (maxLocalSearch);
//
//						//============= Record the best solution ====================//
//						LOCAL_BEST_COST = cost;
//						cout << endl << "LOCAL_BEST COST IS " << LOCAL_BEST_COST << endl;
//						
//						for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//						{
//							for (int j = 0; j <= saiz[i]+1; j++)
//							{
//								LOCAL[i][j] = VEHICLE[i][j];
//							}
//			
//							LOCAL_SAIZ[i] = saiz[i]; //copy temp saiz[] to best SAIZ[]
//							LOCAL_capa[i] = total_demand[i]; // capa[] record the BEST_ROUTE demand
//							LOCAL_Rcost[i] = route_cost[i];
//						}
//						calculate_centreGravity(x, y);
//
//						//=================================record localbest solution in txt=====================================//
//						cout << "========================================================================== " << endl;
//						//ofstream localbest_sol("30.LOCAL_BEST_SOLUTION.txt", ios::app);
//						//=================================display final solution=====================================//
//						for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//						{
//							cout<< i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
//							localbest_sol << i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
//							for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//							{
//								cout<< LOCAL[i][j] << ' ';
//								localbest_sol << LOCAL[i][j] << ' ';
//							}
//							cout<<endl;
//							localbest_sol<<endl;
//						}
//						localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
//						cout << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
//
//						localbest_sol<<"FOUND LOCAL BEST AFTER DIJKSTRA REFINEMENT, UPDATE ORIGAIN"<<endl;
//						//============================= copy to oriGain because oriGain always hold the gain for local_best, it is used in shaking later================================//
//						for (int i = 0; i < SIZE; i++)
//						{
//							Oricost_of_removing[i] = cost_of_removing[i];
//						}
//						//localbest_sol.close();
//						//======================= End localrecord best solution ===========================//
//						Neighbour_k = 1;
//						foundLOCALinCurrNeighbor = false; //initialize to false when found LOCAL solution after Dijkstra refinement, so it will not go through Dijkstra refinement later
//						//delete[] tempRcost;
//						//for (int i = 0; i < no_routes; i++)
//						//{
//						//	delete[] tempvec[i];
//						//}
//						//delete[] tempvec;
//						//delete[] tempRcapa;
//						//delete[] tempRsize;
//						goto skip_check_local; //if Dijkstra refinement produce better result, noneed to copy original gain matrix because we use this solution now
//					}//end if DIjkstra found (cost < LOCAL_BEST_COST)	
//				}//end if (Neighbour_k == max_neigbor) //current neighbourhood is the last neighbourhood
//				Neighbour_k++;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //put at the back on 18June2015, previously before    if (Neighbour_k > max_neigbor)  
//				//if not better than local best solution and neighbourhood havent finished, use the original gain because shaking is based on local best, oriGain is updated evrytime found local best
//				//============================= copy from oriGain ================================//
//				cout<<"Entering copy from Oricost_of_removing because shaking is based on local best"<<endl;
//				for (int f = 0; f < SIZE; f++) //find cost of removing for each customer and put in array
//				{
//					cost_of_removing[f] = Oricost_of_removing[f];
//				}
//			}//if is is not better than the incumbent
//			
//			skip_check_local:
//			for (int i = 0; i < Kth; i++)
//			{
//				delete[] kGain[i];
//			}delete[] kGain;
//			//delete[] LS; //this is just to record which LS produce improvement for print out purpose
//
//		}//end while (Neighbour_k <= max_neigbor)	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@	
//
//		//violateDiverstart: //if violation, skip the process of compare LOCAL_BEST , check if max diversification is reached
//		if ((GLOBAL_BEST_COST-LOCAL_BEST_COST) > epsilonn)//LOCAL_BEST_COST < GLOBAL_BEST_COST
//		{
//			GLOBAL_BEST_COST = LOCAL_BEST_COST;
//			cout << endl << "GLOBAL_BEST COST IS " <<GLOBAL_BEST_COST << endl;
//			found_GLOBEST_status = 1; //for diversificaton use
//			
//			//record the best solution in GLOBAL_BEST_ROUTE in sorted order
//			int r = 0; //for GLOBAL_NO_ROUTE becuase do not consider empty route
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			{
//				if (LOCAL_SAIZ[i] == 0)
//					continue;
//				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//				{
//					GLOBAL[r][j] = LOCAL[i][j];
//				}
//				GLOBAL_SAIZ[r] = LOCAL_SAIZ[i];
//				GLOBAL_Rcost[r] = LOCAL_Rcost[i];
//				GLOBAL_capa[r] = LOCAL_capa[i];
//				r++;
//			}
//			GLOBAL_NO_ROUTE = r;
//
//			//sort_GLOBALsolution (GLOBAL); //sort here so that cost_of removing will not be affected
//			sort_GLOBALsolutionSMALLEST (GLOBAL); //sort here so that cost_of removing will not be affected
//			//=================================record the best solution=====================================//
//			cout << "========================================================================== " << endl;
//
//			//ofstream best_sol("7.BEST_SOLUTION.txt", ios::app);
//			//=================================display final solution=====================================//
//			best_sol <<endl<<"From VNS multi level after Neighbour_k " <<Neighbour_k<<' ' <<' '<<"NbDiv= "<<NbDiv<<  endl;
//			for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
//			{
//				best_sol << i <<' '<< GLOBAL_Rcost[i] <<' '<< GLOBAL_SAIZ[i] <<' '<< GLOBAL_capa[i]<<' '<<' ';
//				cout << i <<' '<< GLOBAL_Rcost[i] <<' '<< GLOBAL_SAIZ[i] <<' '<< GLOBAL_capa[i]<<' '<<' ';
//				for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
//				{
//					best_sol << GLOBAL[i][j] << ' ';
//					cout << GLOBAL[i][j] << ' ';
//				}
//				best_sol <<  endl;
//				cout << endl;	
//			}
//			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
//			cout << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
//			cout<<"Found GLOBAL_BEST"<<endl;
//			//getchar();
//		}
//		VNS_multilevel<<"ENTERING  DIVERSIFICATION PHASE"<<endl;
//		
//		//MUST COPY TO VEHICLE before diversification because VEHICLE is not necessarily GLOBAL bEST now
//		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
//		{
//			for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
//			{
//				VEHICLE[i][j] = GLOBAL[i][j];
//				CumDist[i][j] = GLOBALCumDist[i][j];
//			}
//			saiz[i] = GLOBAL_SAIZ[i];
//			route_cost[i] = GLOBAL_Rcost[i];
//			distance_cost[i] = GLOBAL_distance_cost[i];
//			total_demand[i] = GLOBAL_capa[i];
//			space_available [i] = CAPACITY - total_demand[i];
//			distance_available[i] = DISTANCE - distance_cost[i];
//		}
//		no_routes = GLOBAL_NO_ROUTE;
//		cout<<"ENTERING  DIVERSIFICATION PHASE"<<endl;
//		for (int i = 0; i < no_routes; i++)
//		{
//			cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//			for (int j = 0; j <= saiz[i]+1; j++)
//			{
//				cout<<VEHICLE[i][j]<<' ';
//			}
//			cout<<endl;
//		}
//		cout<<"GLOBAL_BEST_COST= "<<GLOBAL_BEST_COST<<endl;
//		NbDiv++;
//		if (NbDiv > maxDiv) //if this is last diversification, dont perform LNS because it will not go through the LS //added on 12 Oct2015
//		{
//			goto lastdiversify;//check if this one ever executed or change to if (NbDiv == maxDiv) 
//		}
//		float cost = 0;
//		//int div=0;
//		if (found_GLOBEST_status == 1)
//		{
//			cost = make_giant_tour(VEHICLE); //pass in vehicle and no_routes to be updated in Dijkstra
//			Ndel_min = LB_Ndel_min; //back to initial value
//		}
//
//		else if (found_GLOBEST_status == 0) //if not found global_best
//		{
//			cout<<"Entering Diversify LNS/conflict sector in VNS kth improvement: ";
//			//div=(rand() % 4); 
//			if (div == RS)
//				div=0;
//			if (div == 0)
//			{
//				cout<<"LNS Diversification LongestArc"<<endl;
//				cost = Diversify_LongestArc2(VEHICLE, Ndel_min);
//				div++;
//			}
//			else if (div == 1)
//			{
//				cout<<"LNS Diversification overlap route"<<endl;
//				//Diversification_overlapRoute(float **(&vehicle), int (&no_routes), int K, float *x, float *y)
//				cost = Diversification_overlapRoute2(VEHICLE, Ndel_min, x, y);
//				div++;
//			}
//			else if (div == 2)
//			{
//				cout<<"LNS Diversification 2 "<<endl;
//				//cost = Diversify_LNS2(VEHICLE, Ndel_min);
//				cost = Diversify_LNS3(VEHICLE, Ndel_min);
//				div++;
//			}
//			else if (div == 3)
//			{
//				cout<<"Conflict sector Diversification: "<<endl;
//				cost = Diversification_conflict_sector(VEHICLE, Ndel_min, x, y);
//				div++;
//			}
//			int incre = ceil(0.05*(float)SIZE);
//			Ndel_min+=incre;
//			if (Ndel_min > UB_Ndel_min)
//				Ndel_min = UB_Ndel_min; //added this on 2March2016
//		}
//		
//		//AFter diversification, record in LOCAL_BEST_ROUTE, when it enters restartAfterDiver, it will restart with add empty route to LOCAL_BEST_ROUTE again
//		//sort_solution (VEHICLE);
//		sort_solutionSMALLEST (VEHICLE);
//		int k=0;//for LOCAL[][] route index
//		for (int i = 0; i < no_routes; i++)
//		{
//			if (saiz[i] == 0)
//				continue;
//			for (int j = 0; j <= saiz[i]+1; j++)
//			{
//				LOCAL[k][j] = VEHICLE[i][j];
//			}
//			LOCAL_SAIZ[k] = saiz[i];
//			LOCAL_capa[k] = total_demand[i];
//			LOCAL_Rcost[k] = route_cost[i];
//			k++;
//		}
//		LOCAL_BEST_COST = cost;
//		LOCAL_NO_ROUTE = k;
//
//		cout<<"AFTER DIVERSIFICATION"<<endl;
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			cout<<i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//			{
//				cout<<LOCAL[i][j]<<' ';
//			}
//			cout<<endl;
//		}
//		cout<<"LOCAL_BEST_COST ="<<LOCAL_BEST_COST<<endl;
//
//		//if the cost better than GLOBAL_BEST_COST 
//		if (GLOBAL_BEST_COST-cost > epsilonn)//cost < GLOBAL_BEST_COST
//		{
//			int m=0;//for GLOBAL[][] route index
//			for (int i = 0; i < no_routes; i++)
//			{
//				if (saiz[i] == 0)
//					continue;
//				best_sol << m<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';//GLOBAL_Rcost not yet recorded, so use route_cost
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					GLOBAL[m][j] = VEHICLE[i][j];
//					best_sol << GLOBAL[m][j] << ' ';
//				}
//				best_sol <<endl;
//				GLOBAL_SAIZ[m] = saiz[i];
//				GLOBAL_capa[m] = total_demand[i];
//				GLOBAL_Rcost[m] = route_cost[i];
//				m++;
//			}
//			GLOBAL_BEST_COST = cost;
//			GLOBAL_NO_ROUTE = m;
//
//			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
//			best_sol << "From VNS multi-level after Diversification type "<< div <<  endl;
//			cout << "Found GLOBAL_BEST in VNS multi-level right after Diversification (only basic LNS no other improvement) type "<<div<<  endl;
//			//getchar();
//
//		}
//		best_sol << "Type of div = "<<div<<endl;
//		
//		reinitializeRchangedStatus();//initialize to 0 for every route	
//		lastdiversify:;
//		delete[] cost_of_removing;
//		delete[] Oricost_of_removing;
//		delete[] RchangedStatus;
//		//delete[] CustaffectedStatus;
//			
//	}//end while (NbDiv <= maxDiv) //#############################################################################################################################
//	float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
//	EV_nolearning << Total_LSmove<<' '<<' '<<runtime<<' '<<' '<<GLOBAL_BEST_COST; //record for the very last one
//
//	timefile << "VNS multi-level "<<maxDiv <<" diversification time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << " second"<<endl;
//	recordTIME << "VNS  multi-level ("<<std::to_string( I )<<") with "<<maxDiv <<" diversification time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << " second"<<endl;
//	best_sol.close();
//	localbest_sol.close();
//}
//======================== Without data stucture in multi-level, check on every route even though full search have been done before ============// before 10Sept2015
//void VNS_kth_improvement(int *demand, float *x, float *y, float **dist) 
//{
//	ofstream recordTIME("TIME.txt", ios::app);
//	ofstream violation("Infeasible_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream timefile("16.Time_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream localbest_sol("30.LOCAL_BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
//	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);
//
//	float start_s = clock();// the code you wish to time goes here
//
//	//******************************************** STEP 1: INITIALIZATION ********************************************//
//	//============================================ all predefined variables =================================================//
//	int max_neigbor = 7; //7 neighbourhoods
//	int max_level = 9; //9 levels of LS improvement 
//	int maxLocalSearch = 8; //excluding last LS:2-opt for freq use
//	int NbDiv = 0;
//	int maxDiv = max(5, GLOBAL_NO_ROUTE/2);
//	int LB_Ndel_min = max (5.00, (0.10*SIZE));
//	float siz=SIZE;
//	int UB_Ndel_min = min (400.00, (0.4*SIZE));
//	int Ndel_min = LB_Ndel_min; //min number of iteration for each route for diversification
//
//	//============================================= all predefined variables ================================================//
//
//	int TOTAL_DEMAND=0;
//	for (int i = 0; i < SIZE; i++)
//	{
//		TOTAL_DEMAND = TOTAL_DEMAND+demand[i];
//	}//just to chack the overall demand is correct at the end of VNS multi-level
//	
//	//============= Copy Local_BEST from GLOBAL_BEST ===========//
//	for (int i = 0; i < SIZE; i++)
//	{
//		for (int j = 0; j < SIZE; j++)
//		{
//			LOCAL[i][j] = -1; //reinitialize
//		}
//		LOCAL_SAIZ[i] = 0;
//		LOCAL_capa[i] = 0;
//	}
//	
//	for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
//	{
//		for (int j = 0; j <= GLOBAL_SAIZ[i]+3; j++)
//		{
//			LOCAL[i][j] = GLOBAL[i][j];
//		}
//		LOCAL_capa[i] = GLOBAL_capa[i];
//		LOCAL_SAIZ[i] = GLOBAL_SAIZ[i];
//	}
//	LOCAL_BEST_COST = GLOBAL_BEST_COST;
//	LOCAL_NO_ROUTE = GLOBAL_NO_ROUTE;
//	
//	while (NbDiv <= maxDiv) //#############################################################################################################################
//	{
//		float violateThreshold = INFEASIBLE; //reinitialize to original value after diversification
//		int found_GLOBEST_status = 0;//must put before infeasibility check, otherwise found_GLOBEST_status is not recognise after goto violateDiverstart
//		localbest_sol << "Multi-level NbDiv= " <<NbDiv<<endl;
//		int no_empty_route=0;
//		cout<<"LOCAL_NO_ROUTE after restartAfterDiver= "<<LOCAL_NO_ROUTE<<endl;
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			for (int j = 0; j <= LOCAL_SAIZ[i]+3; j++)
//			{
//				cout<<LOCAL[i][j]<< ' ';
//			}
//			cout<<endl;
//		}
//
//		//******************************************** STEP 2: ADD EMPTY ROUTE ********************************************//
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			if (LOCAL_SAIZ[i] == 0)
//				no_empty_route++;
//		}
//		if (no_empty_route == 1)
//		{
//			goto skipaddemptyR; //if empty route=1, skip add empty route
//		}
//		else if (no_empty_route == 0)
//		{
//			//************************************add one empty route
//			LOCAL[LOCAL_NO_ROUTE][0] = LOCAL_NO_ROUTE;
//			LOCAL[LOCAL_NO_ROUTE][1] = 0.0; //cost
//			LOCAL[LOCAL_NO_ROUTE][2] = 0; //number of customer
//			LOCAL[LOCAL_NO_ROUTE][3] = -1;
//			LOCAL_capa[LOCAL_NO_ROUTE] = 0;
//			LOCAL_SAIZ[LOCAL_NO_ROUTE] = 0;
//			LOCAL_Rcost[LOCAL_NO_ROUTE] = 0;
//			LOCAL_distance_cost[LOCAL_NO_ROUTE] = 0;
//			LOCAL_NO_ROUTE++;
//			//************************************end of add one empty route
//		}
//		else if (no_empty_route > 1) //if more than 1 empty route, do not copy the empty route and add one at the end
//		{
//			int k=0; //for new route index
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			{
//				if (LOCAL_SAIZ[i] == 0)
//				{
//					continue;
//				}
//				LOCAL[k][0] = k;
//				for (int j = 1; j <= LOCAL_SAIZ[i]+1; j++)
//				{
//					LOCAL[k][j] = LOCAL[i][j];
//			
//				}
//				for (int m = LOCAL_SAIZ[i]+4; m < SIZE; m++)
//				{
//					LOCAL[k][m] = -1; //reinitialize the rest
//				}
//				LOCAL_SAIZ[k] = LOCAL_SAIZ[i];
//				LOCAL_capa[k] = LOCAL_capa[i];
//				k++;
//			}
//			LOCAL_NO_ROUTE = k;
//			//************************************add one empty route
//			LOCAL[LOCAL_NO_ROUTE][0] = LOCAL_NO_ROUTE;
//			LOCAL[LOCAL_NO_ROUTE][1] = 0.0; //cost
//			LOCAL[LOCAL_NO_ROUTE][2] = 0; //number of customer
//			LOCAL[LOCAL_NO_ROUTE][3] = -1;
//			LOCAL_Rcost[LOCAL_NO_ROUTE] = 0;
//			LOCAL_distance_cost[LOCAL_NO_ROUTE] = 0;
//			LOCAL_capa[LOCAL_NO_ROUTE] = 0;
//			LOCAL_SAIZ[LOCAL_NO_ROUTE] = 0;
//			LOCAL_NO_ROUTE++;
//			//************************************end of add one empty route
//			//reinitialize the remaining route
//			for (int i = LOCAL_NO_ROUTE; i < SIZE; i++)
//			{
//				for (int j = 0; j < SIZE; j++)
//				{
//					LOCAL[i][j]=-1;
//				}
//			}
//		}
//
//		cout<<"LOCAL_NO_ROUTE before skipaddemptyR= "<<LOCAL_NO_ROUTE<<endl;
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//			{
//				cout<<LOCAL[i][j]<< ' ';
//			}
//			cout<<endl;
//		}
//	skipaddemptyR: //if empty route=1, skip add empty route
//		cout<<"LOCAL_NO_ROUTE= "<< LOCAL_NO_ROUTE<<endl;
//		
//		int Neighbour_k = 1; //Neighbour_k 1 = (1-0), Neighbour_k 2 = (1-1), Neighbour_k 3 = (2-1), Neighbour_k 4 = (2-0)
//
//		float **vehicle = new float* [SIZE]; //declare more
//		int **vehic_depotCust = new int* [SIZE];
//		for (int i = 0; i < SIZE; i++)
//		{
//			vehicle[i] = new float[SIZE]; //same as best routes, for passing function to shake()
//			vehic_depotCust[i] = new int[SIZE]; //vehic_depotCust contains only customer with depot at front and back, for reoptimize
//	
//		}
//	
//		int no_routes = LOCAL_NO_ROUTE;
//		//bool *flag_module = new bool [SIZE]; //if no place for insertion, put 0
//		float *cost_of_removing = new float[SIZE + 1];//cost_of_removing[][] only used in LS, not shaking!!!!!!!!!!!!!!!!!!!!!!
//		int total_time = 0;
//		float maxCPUtime = 10000/(CLOCKS_PER_SEC);
//		cout << "maxCPUtime= " <<maxCPUtime<<endl;
//	
//	//###################################### STEP 3(a) Shaking ######################################//
//	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ k+1 neighborhood starts here @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
//		while (Neighbour_k <= max_neigbor)	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//		{	
//			if (violateThreshold < 0.01) //if smaller than 1%, make it zero
//				violateThreshold = 0;
//
//			no_routes = LOCAL_NO_ROUTE;
//			//for (int i = 0; i < SIZE; i++)
//			//{
//			//	for (int j = 0; j < SIZE; j++)
//			//	{
//			//		vehicle[i][j] = -1; //reinitialize
//			//		vehic_depotCust[i][j] = -1;
//			//	}
//			//}
//			////vehicle copied from BEST_ROUTE to be passed to shake()
//			//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			//{
//			//	int j=0; 
//			//	int k=1; // for vehic_depotCust[i][k]
//
//			//	route_cost[i] = LOCAL_Rcost[i];
//			//	vehic_depotCust[i][0] = SIZE;
//
//			//	for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//			//	{
//			//		vehicle[i][j] = LOCAL[i][j];
//			//	}
//			//	saiz[i] = LOCAL_SAIZ[i] ;//initially both are the same
//			//	vehic_depotCust[i][saiz[i]+1] = SIZE;
//			//}
//
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			{
//				LOCAL_capa[i] = 0;//reinitialize
//				for (int j = 3; j < LOCAL_SAIZ[i]+1; j++)
//				{
//					LOCAL_capa[i] = LOCAL_capa[i] + demand[(int)LOCAL[i][j]]; //LOCAL_capa[] record the LOCAL_BEST_ROUTE demand
//				}
//			}
//			//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			//{
//			//	route_cost[i] = LOCAL_Rcost[i];
//			//	saiz[i] = LOCAL_SAIZ[i];
//			//	space_available[i] = (CAPACITY*(1+violateThreshold)) - (total_demand[i]);
//			//	distance_available[i] = (DISTANCE*(1+violateThreshold)) - (distance_cost[i]);
//			//	//space_available[i] = CAPACITY - (total_demand[i]);
//			//	//distance_available[i] = DISTANCE - (distance_cost[i]);
//			//}
//
//
//	
//			//cout<<"//=============== Go to next neighbourhood =============//"<<endl;
//			//cout<<"Next module is "<<Neighbour_k<<endl;
//			//cout<<"no_routes= "<<no_routes<<endl;
//			//for (int i = 0; i < no_routes; i++)
//			//{
//			//	for (int j = 0; j <= saiz[i]+1; j++)
//			//	{
//			//		cout<<vehicle[i][j]<<' ';
//			//	}
//			//	if (vehicle[i][0] == -1)
//			//		exit (EXIT_FAILURE);
//			//	cout<<"d= "<<total_demand[i]<<endl;
//			//}
//
//			//**************all insertion and deletion starts from		1 to saiz because the vehic_depotCust starts from depot, end with depot**************//
//			//*************in the insert 1_0 ,insert 1_1, insert 2_1 it is not the case, need to check ***********************//
//			custRGravity = new float[LOCAL_NO_ROUTE];
//			sorted_custRGravity = new float*[LOCAL_NO_ROUTE];
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			{
//				custRGravity[i] = 0; //initialize
//				sorted_custRGravity[i] = new float[5]; //[0]sorted distance, [1]sorted route index, [2]1/distance, [3]d/sum(d), [4]cumulative 
//			}
//
//		reshake: //if no feasible shake, Neighbour_k++ and goto reshake
//			for (int i = 0; i < 5; i++) //reinitialize after_shake_route_change[6]
//			{
//				after_shake_route_change[i] = -1;
//			}
//			after_shake_route_changePtr = 0; //reinitialize pointer to 0, to keep track how many routes have been changed
//
//			//vehicle copied from LOCAL_BEST_ROUTE to be passed to shake()
//			for (int i = 0; i < SIZE; i++)
//			{
//				for (int j = 0; j < SIZE; j++)
//				{
//					vehicle[i][j] = -1; //reinitialize
//				}
//			}
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)//added 22Apr15
//			{
//				total_demand[i] = 0;//reinitialize
//				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//				{
//					vehicle[i][j] = LOCAL[i][j];
//					if((j>=3) && (j<=(LOCAL_SAIZ[i]+2)))
//					{
//						total_demand[i] = total_demand[i]+demand[(int)vehicle[i][j]];
//					}
//
//				}
//				saiz[i] = LOCAL_SAIZ[i]; 
//				route_cost[i] = LOCAL_RCost[i];
//				space_available[i] = (CAPACITY*(1+violateThreshold)) - (total_demand[i]);
//				distance_available[i] = (DISTANCE*(1+violateThreshold)) - (distance_cost[i]);
//				//space_available[i] = CAPACITY - total_demand[i];
//				//distance_available[i] = DISTANCE - distance_cost[i];
//
//			}
//
//			int shake_status1 = 1;
//			int shake_status2 = 1;
//			if (Neighbour_k == 1) //(1-0)
//			{
//				shake_status1 = shake_1_0(vehicle, x, y); //the one going to shake is always the current best solution
//				shake_status2 = shake_1_0(vehicle, x, y);
//				if ((shake_status1 == 0) && (shake_status2 == 0))
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//		
//			else if (Neighbour_k == 2) //1-1
//			{	
//				shake_status1 = shake_1_1(vehicle,  x, y);
//		
//				if (shake_status1 == 0) 
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//			else if (Neighbour_k == 3) //(2-0) - a pair from route A go to route B
//			{			
//				shake_status1 = shake_2_0_twoR(vehicle,  x, y);
//				shake_status2 = shake_1_0(vehicle, x, y);
//				if ((shake_status1 == 0) && (shake_status2 == 0))
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//	
//			else if (Neighbour_k == 4) //(2-0) - a pair from route A go to route B and route C
//			{			
//				shake_status1 = shake_2_0_threeR(vehicle,  x, y);
//
//				if (shake_status1 == 0) 
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//
//			else if (Neighbour_k == 5) //(2-1) - a pair from route A swap with one customer from route B
//			{			
//				shake_status1 = shake_2_1_twoR(vehicle, x, y);
//				shake_status2 = shake_1_0(vehicle, x, y);
//				if ((shake_status1 == 0) && (shake_status2 == 0))
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//			else if (Neighbour_k == 6) //(2-1) - first pair of cust must from route A to B, another cust from route A can go to route C
//			{			
//				shake_status1 = shake_2_1_threeR(vehicle, x, y);
//				shake_status2 = shake_1_1(vehicle, x, y);
//
//				if ((shake_status1 == 0) && (shake_status2 == 0))
//				{
//					Neighbour_k++;
//					goto reshake; //if no feasible shake, Neighbour_k++ and goto reshake
//				}
//			}
//			else if (Neighbour_k == 7) //(2-2) 
//			{
//				shake_status1 = shake_2_2(vehicle,  x, y);
//
//				if (shake_status1 == 0) 
//				{
//					Neighbour_k++; 
//					//goto check_sol; //if no feasible shake at last neighborhod, dont perform LS, goto check_sol
//										//vehic_depotCust copied from vehicle to be passed to reoptimize() later //made comment on 15June2015
//					for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//					{
//						int j=0; 
//						saiz[i] = vehicle[i][2];
//						vehic_depotCust[i][0] = SIZE; //vehic_depotCust contains only customer with depot at front and back, for reoptimize
//						int v=1; //for vehic_depotCust
//						do
//						{
//							if (j>=3)
//							{
//								vehic_depotCust[i][v] = vehicle[i][j];
//								v++;
//							}
//							j++;
//						} while (vehicle[i][j]!= -1);
//	
//						vehic_depotCust[i][v] = SIZE;
//					}
//					goto check_LOCAL_BEST; //if no feasible shake at last  neighborhod, dont perform LS, goto check_LOCAL_BEST, but need to copy vehicDepoCust[][] from vehicle[][] //added 15June2015 //check this if correct
//				}
//					
//			}
//
//			cout<<endl<<"================================"<<endl;
//			cout<<"vehicle[][] after shake " <<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<vehicle[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			cout<<endl;
//
//
//			float afterShake_cost = 0.0; //to record the current cost so that later after kth improvement can check if the solution got improved
//			for (int i = 0; i < no_routes; i++)
//			{
//				afterShake_cost = afterShake_cost + route_cost[i];
//			}
//			//vehic_depotCust[][] copy from vehicle[][] after shake
//			int r1 = route_change[0];
//			int r2 = route_change[1];
//			int r3 = route_change[2];	
//
//			cout<<"r1= " <<r1 << ' ' << "r2= " <<r2 << ' ' << "r3= " <<r3 << endl;
//			cout<<"After shake " <<Neighbour_k <<endl;
//			//vehic_depotCust copied from vehicle to be passed to reoptimize() later
//			//=========== copy vehicle[][] to vehic_depotCust[][] after shake ============================//
//			for (int i = 0; i < SIZE; i++)
//			{
//				for (int j = 0; j < SIZE; j++)
//				{
//					vehic_depotCust[i][j] = -1;//reinitialize
//				}
//			}
//
//			for (int i = 0; i < no_routes; i++)
//			{
//				vehic_depotCust[i][0] = SIZE;
//				int k=1; //for vehic_depotCust[i][k]
//				for (int j = 3; j <= saiz[i]+2; j++)
//				{
//					vehic_depotCust[i][k] = vehicle[i][j] ;
//					k++;
//				}
//				vehic_depotCust[i][k] = SIZE;
//			}
//	
//			for (int r = 0; r < after_shake_route_changePtr; r++)
//			{
//				cout<<"r" <<r<<" = "<< after_shake_route_change[r] <<' '; 
//			}
//			cout<<"After shake " <<Neighbour_k <<endl;
//
//
//			////=================================================update cost of removing=====================================//////////////////////
//			//for (int r = 0; r < after_shake_route_changePtr; r++)
//			//{
//			//	int f = after_shake_route_change[r];	//this store the route number
//			//	if (saiz[f]==0)
//			//		continue;
//			//	for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
//			//	{
//			//		cost_of_removing[vehic_depotCust[f][h]] = dist[vehic_depotCust[f][h]][vehic_depotCust[f][h - 1]] + dist[vehic_depotCust[f][h]][vehic_depotCust[f][h + 1]] + service_time[vehic_depotCust[f][h]] - dist[vehic_depotCust[f][h - 1]][vehic_depotCust[f][h + 1]];
//			//	}
//			//}
//			find_all_cost_of_removing (vehic_depotCust, cost_of_removing, no_routes);//cost_of_removing[][] only used in LS, not shaking
//	
//	
//			/////////////// ================================ MULTI_LEVEL VNS ==================================================== ////////////////////////////
//			int k = 5; //kth improvement, find the best value in K gain !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//			int level = 1; //initialize local search to 1
//			bool sameroute = false; //false if different route, true if same route
//			int no_attribute = 13; //[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
//			
//			string *LS = new string[max_level+1]; //to record the LS that found improvement, just to print out purpose to see which LS is effective //record level start from 1 until (maxLevel+1)
//			for (int i = 0; i <= max_level; i++)
//			{
//				LS[i] = "No";//initialize all not found improvement
//			}
//			float **kGain = new float* [k]; //record information for kth level gain, 
//			for (int i = 0; i < k; i++)
//			{
//				kGain[i] = new float [no_attribute];
//			}
//
//			for (int i = 0; i < no_routes; i++) //update this so that LS wil only consider feasible move //added on 16June2015 //should add this to VNS? noneed, because it is updated in best_gain
//			{
//				space_available[i] = CAPACITY - total_demand[i];
//				distance_available[i] = DISTANCE - distance_cost[i];
//			}
//		while (level <= max_level)
//		{
//			//redo_LS://1) if no improvement found in each level of LS, 2) if cost is better than aftershake cost set level=1, 3) if LS level has not finished all level level++ and goto redo_LS
//			if (level == 1) //the first level is 1-0 same route
//			{
//				int number = find_k_1_0_sameR( cost_of_removing, kGain, vehic_depotCust, k);
//				cout<<"in level 1: find_k_1_0_sameR, best number= "<<number << endl;
//				
//				sameroute = true;//same route
//
//				if (number == -1)//if no improvement found in each level of LS
//				{
//					//LS[level] = "No";//record the level not found improvement
//					level++;
//					continue;
//					//goto redo_LS; //if no improvement found in each level of LS
//				}
//				//route_file<<"in level 1: find_k_1_0_sameR, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				LS[level] = "Yes";//record the level that found improvement
//				insert_1_0(cost_of_removing, k, number, kGain, vehic_depotCust, sameroute);
//				cout<<endl<<endl;
//				cout<<"========== vehic_depotCust after insert in level 1 find_k_1_0_sameR ==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<vehic_depotCust[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//				goto check_sol;//after each LS goto check_sol
//			}
//
//			else if (level == 2)
//			{
//				int number = find_k_1_0(cost_of_removing, kGain, vehic_depotCust, k);
//				cout<<"in level 2: find_k_1_0, best number= "<<number<<endl;
//		
//				sameroute = false; //different route
//				if (number == -1)//if no improvement
//				{
//					//LS[level] = "No";//record the level not found improvement
//					level++;
//					continue;
//					//goto redo_LS;//if no improvement found in each level of LS
//				}
//				//route_file<<"in level 2: find_k_1_0, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				LS[level] = "Yes";//record the level that found improvement
//				insert_1_0(cost_of_removing, k, number, kGain, vehic_depotCust, sameroute);
//				cout<<endl<<endl;
//				cout<<"========== vehic_depotCust after insert in level 2 find_k_1_0==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<vehic_depotCust[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//				goto check_sol;//after each LS goto check_sol
//			}
//
//			else if (level == 3)
//			{
//				int number = find_k_1_1_sameR(cost_of_removing, kGain, vehic_depotCust, k);
//				cout<<"in level 3: find_k_1_1_sameR, best number= "<<number <<endl;
//				
//				sameroute = true; //same route
//				if (number == -1)//if no improvement
//				{
//					//LS[level] = "No";//record the level not found improvement
//					level++;
//					continue;
//					//goto redo_LS;//if no improvement found in each level of LS
//				}
//				//route_file<<"in level 3: find_k_1_1_sameR, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				LS[level] = "Yes";//record the level that found improvement
//				insert_1_1( cost_of_removing, k, number, kGain, vehic_depotCust, sameroute);
//				cout<<endl<<endl;
//				cout<<"========== vehic_depotCust after insert in level 3: find_k_1_1_sameR==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<vehic_depotCust[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//				goto check_sol;//after each LS goto check_sol
//			}
//
//			else if (level == 4)
//			{
//				int number = find_k_1_1( cost_of_removing, kGain, vehic_depotCust, k);
//				cout<<"in level 4: find_k_1_1, best number= "<<number <<endl;
//				
//				sameroute = false; //different route
//				if (number == -1)//if no improvement
//				{
//					//LS[level] = "No";//record the level not found improvement
//					level++;
//					continue;
//					//goto redo_LS;//if no improvement found in each level of LS
//				}
//				//route_file<<"in level 4: find_k_1_1, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				LS[level] = "Yes";//record the level that found improvement
//				insert_1_1( cost_of_removing, k, number, kGain, vehic_depotCust, sameroute);
//				cout<<endl<<endl;
//				cout<<"========== vehic_depotCust after insert in level 4: find_k_1_1==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<vehic_depotCust[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//				goto check_sol;//after each LS goto check_sol
//			}
//
//			else if (level == 5)
//			{
//				int number = find_k_2_1_sameR(cost_of_removing, kGain, vehic_depotCust, k);
//				cout<<"in level 5: find_k_2_1_sameR, best number= "<<number <<endl;
//				
//				sameroute = true; //same route
//				if (number == -1)//if no improvement
//				{
//					//LS[level] = "No";//record the level not found improvement
//					level++;
//					continue;
//					//goto redo_LS;//if no improvement found in each level of LS
//				}
//				//route_file<<"in level 5: find_k_2_1_sameR, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				LS[level] = "Yes";//record the level that found improvement
//				insert_2_1(cost_of_removing, k, number, kGain, vehic_depotCust, sameroute);
//				cout<<endl<<endl;
//				cout<<"========== vehic_depotCust after insert in level 5: find_k_2_1_sameR==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<vehic_depotCust[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//				goto check_sol;//after each LS goto check_sol
//			}
//
//			else if (level == 6)
//			{
//				int number = find_k_2_1(cost_of_removing, kGain, vehic_depotCust, k);
//				cout<<"in level 6: find_k_2_1, best number= "<<number <<endl;
//				
//				sameroute = false; //different route
//				if (number == -1)//if no improvement
//				{
//					//LS[level] = "No";//record the level not found improvement
//					level++;
//					continue;
//					//goto redo_LS;//if no improvement found in each level of LS
//				}
//				//route_file<<"in level 6: find_k_2_1, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				LS[level] = "Yes";//record the level that found improvement
//				insert_2_1(cost_of_removing, k, number, kGain, vehic_depotCust, sameroute);
//				cout<<endl<<endl;
//				cout<<"========== vehic_depotCust after insert in level 6: find_k_2_1==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<vehic_depotCust[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//				goto check_sol;//after each LS goto check_sol
//			}
//
//			else if (level == 7)
//			{
//				int number = find_k_2_0_sameR(cost_of_removing, kGain, vehic_depotCust, k);
//				cout<<"in level 7: find_k_2_0_sameR, best number= "<<number <<endl;
//				
//				sameroute = true; //same route
//				if (number == -1)//if no improvement can be found
//				{
//					//LS[level] = "No";//record the level not found improvement
//					level++;
//					continue;
//					//goto redo_LS;//if no improvement found in each level of LS
//				}
//				//route_file<<"in level 7: find_k_2_0_sameR, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				LS[level] = "Yes";//record the level that found improvement
//				insert_2_0(cost_of_removing, k, number, kGain, vehic_depotCust, sameroute);
//				cout<<endl<<endl;
//				cout<<"========== vehic_depotCust after insert in level 5: find_k_2_0_sameR==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<vehic_depotCust[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//				goto check_sol;//after each LS goto check_sol
//			}
//		
//			else if (level == 8)
//			{
//				int number = find_k_2_0(cost_of_removing, kGain, vehic_depotCust, k);
//				cout<<"in level 8: find_k_2_0, best number= "<<number <<endl;
//				
//				sameroute = false; //different route
//				if (number == -1)//if no can be improvement
//				{
//					//LS[level] = "No";//record the level not found improvement
//					level++;
//					continue;
//					//goto redo_LS;//if no improvement found in each level of LS
//				}
//				//route_file<<"in level 8: find_k_2_0, best number= "<<number <<", maxgain= "<<kGain[number][1]<< endl;
//				LS[level] = "Yes";//record the level that found improvement
//				insert_2_0(cost_of_removing, k, number, kGain, vehic_depotCust, sameroute);
//				cout<<endl<<endl;
//				cout<<"========== vehic_depotCust after insert in level 8: find_k_2_0==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<vehic_depotCust[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//				goto check_sol; //after each LS goto check_sol
//			}
//			else if (level == 9)
//			{
//				int *routeChange2opt = new int[no_routes];
//				int num_routeChange = 0;
//				cout<<"Before entering 2-opt"<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' ' <<saiz[i]<<' ';
//					for (int j = 1; j <= saiz[i]; j++)
//					{
//						cout<<vehic_depotCust[i][j]<< ' ';
//					}
//					cout<<"-1"<<endl;
//				}
//				num_routeChange = two_opt(vehic_depotCust, routeChange2opt);
//				//LS[level] = "No";//record the level not found improvement
//				//update cost of removing
//				cout<<"Out of 2-opt"<<endl;
//				if (num_routeChange != 0)
//				{
//					LS[level] = "Yes";//record the level that found improvement
//					for (int i = 0; i < num_routeChange; i++)
//					{
//						int r = routeChange2opt[i];
//						for (int h = 1; h <= saiz[r]; h++) //first customer start from element [1]
//						{
//							cost_of_removing[vehic_depotCust[r][h]] = dist[vehic_depotCust[r][h]][vehic_depotCust[r][h - 1]] + dist[vehic_depotCust[r][h]][vehic_depotCust[r][h + 1]]  + service_time[vehic_depotCust[r][h]] - dist[vehic_depotCust[r][h - 1]][vehic_depotCust[r][h + 1]];
//						}
//					}
//			
//				}
//				cout<<"========== vehic_depotCust after 2-opt in level 9: 2-opt intra route==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<vehic_depotCust[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//				delete[] routeChange2opt;
//				goto check_sol; //after each LS goto check_sol
//			}
//		check_sol: //after each LS or if no feasible shake at last module(neighborhod), goto check_sol, compare with aftershakecost
//
//			float t_cost = 0; //reinitialize temporary total cost
//			//========== check if temporary cost is less than BEST COST ============//
//			for (int i = 0; i < no_routes; i++)
//			{
//				if (saiz[i] == 0)
//				{
//					route_cost[i] =0;//if it is empty route
//					total_demand[i]=0;
//				}
//				t_cost = t_cost + route_cost[i];
//			}
//
//			////=========== copy vehic_depotCust[][] to vehicle[][] ============================//
//			//for (int i = 0; i < SIZE; i++)
//			//{
//			//	for (int j = 0; j < SIZE; j++)
//			//	{
//			//		vehicle[i][j] = -1;//reinitialize
//			//	}
//			//}
//
//			//for (int i = 0; i < no_routes; i++)
//			//{
//			//	vehicle[i][0] = i; //route_index
//			//	vehicle[i][1] = route_cost[i];
//			//	vehicle[i][2] = saiz[i];
//			//	int k=3;//for vehicle[i][k]
//			//	for (int j = 1; j <= saiz[i]; j++)
//			//	{
//			//		vehicle[i][k] = vehic_depotCust[i][j];
//			//		k++;
//			//	}
//			//	vehicle[i][k]=-1;
//			//}
//			//======================================================Display route=======================================================================//
//			cout << "==================Routes (in VNS kth improvement) after LS Level " << level <<" ===================================== " << endl;
//			//route_file << "==================Routes (in VNS kth improvement) after LS Level " << level <<" ===================================== " << endl;
//			float total_cost = 0.0;
//			int total_cust = 0;
//			int sum_demand=0;
//			for (int g = 0; g < no_routes; g++)
//			{
//				if (saiz[g] == 0)
//				{
//					route_cost[g] = 0;
//					total_demand[g]=0;
//					distance_cost[g] = 0;
//				}
//				cout << g << ' '<< route_cost[g] << ' '<< saiz[g] << ' ';
//				//route_file << g << ' '<< route_cost[g] << ' '<< saiz[g] << ' ';
//				for (int h = 1; h <= saiz[g]; h++)
//				{
//					cout << vehic_depotCust[g][h] << ' ';	
//					//route_file << vehic_depotCust[g][h] << ' ';	
//				}
//				total_cust = total_cust+saiz[g];
//				total_cost = total_cost + route_cost[g];
//				sum_demand = sum_demand+total_demand[g];
//				cout << "-1"<< ' ' << "d=" << total_demand[g] << endl;
//				//route_file << "-1"<< ' ' << "d=" << total_demand[g] <<endl;
//			}
//			cout << "Total customers = " << total_cust << endl<< "Total cost = " << total_cost << endl<< "Sum_demand = " << sum_demand <<endl<< "TOTAL_DEMAMD = " << TOTAL_DEMAND<<endl;
//			//route_file<< "Total customers = " << total_cust << endl<< "Total cost = " << total_cost << endl<< "Sum_demand = " << sum_demand <<endl<< "TOTAL_DEMAMD = " << TOTAL_DEMAND<<endl;
//			//route_file<< "After shake cost = " <<afterShake_cost <<endl;
//			//route_file<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
//
//			if ((afterShake_cost - t_cost) > 0.05)//t_cost < afterShake_cost
//			{
//				//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ for infeasibility check @@@@@@@@@@@@@@@@@@@@@@@@//
//				for (int i = 0; i < no_routes; i++)
//				{
//					if ((total_demand[i]>CAPACITY) || (distance_cost[i] - DISTANCE > 0.05))//for infeasibility check
//					{
//						violation << "In VNS multi-level after module "<< Neighbour_k <<endl;
//						for (int j = 0; j <= saiz[i]+1; j++)
//						{
//							violation << vehicle[i][j]<<' ';
//						}
//						violation<<endl;
//						violateThreshold = violateThreshold/2; //divide by half if the solution after LS still violate constraint
//						//if (Neighbour_k < max_neigbor)
//						//{
//							Neighbour_k++; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//							goto skip_check_local;
//					
//						//}
//						//else //if maximum neighbourhood is reached
//						//	goto violateDiverstart; //if violation, skip all the process of compare LOCAL_BEST and GLOBAL_BEST, check if max diversification is reached
//					}
//				}
//				//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ End of (for infeasibility check) @@@@@@@@@@@@@@@@@@@@@@@@//
//				//try checking fesibility here next time!!!!!!!!!!!!!!!! to avoid  time wasted if route is infeasible after so many LS
//				//if((abs(t_cost-LOCAL_BEST_COST) <= 0.005) || (abs(t_cost-GLOBAL_BEST_COST) <= 0.005)) //added on 3April2015 to avoid recalculate if it is the same as LOCAl or GLOBAL BEST //for early escape
//				//	goto check_LOCAL_BEST;
//				afterShake_cost = t_cost;
//				level = 1;
//				//goto redo_LS; //go to LS level 1 if it is better than aftershake cost
//			}
//	
//			else
//			{
//				level = level+1;
//				//if (level > max_level) //when level =10
//				//	goto check_BEST; //if all LS are done, goto check_BEST
//				//goto redo_LS; //goto next level of LS if it has not finished all level
//			}
//		}//end while (level <= max_level)
//
//	check_LOCAL_BEST://if all LS are done, goto check_LOCAL_BEST //compare with LOCAL_BEST_COST
//			//======================================================Display route========================================================================
//			//route_file<< "==================Routes (in VNS kth improvement) after all LS before check LOCAL BEST===================================== " << endl;
//			cout << "==================Routes (in VNS kth improvement) after all LS before check LOCAL BEST===================================== " << endl;
//			cout<< "current Neighbour_k= "<< Neighbour_k<<endl;
//			float demd = 0.0, total_cost = 0.0;
//			int total_cust = 0;
//			for (int g = 0; g < no_routes; g++)
//			{
//				if (saiz[g] == 0)
//				{
//					route_cost[g] = 0;
//					total_demand[g]=0;
//					distance_cost[g] = 0;
//				}
//				demd = 0.0;
//				cout << g << ' '<< route_cost[g] << ' '<< saiz[g] << ' ';
//				//route_file<< g << ' '<< route_cost[g] << ' '<< saiz[g] << ' ';
//				total_cust = total_cust + saiz[g];
//				for (int h = 1; h <= saiz[g]; h++)
//				{
//					cout << vehic_depotCust[g][h] << ' ';
//					//route_file << vehic_depotCust[g][h] << ' ';
//					demd = demd + demand[vehic_depotCust[g][h]];
//				}
//				total_cost = total_cost + route_cost[g];
//				cout << "-1"<<setw(3) << "d=" << demd << endl;
//				//route_file<< setw(3) << "d=" << demd << endl;
//			}
//			cout << "Total customers = " << total_cust << endl << "Total cost = " << total_cost << endl;
//			cout<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
//			//route_file<<"Total customers = " << total_cust << endl << "Total cost = " << total_cost << endl;
//			//route_file<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
//
//			for (int i = 0; i < SIZE; i++)
//			{
//				for (int j = 0; j < SIZE; j++)
//				{
//					vehicle[i][j] = -1; //reinitialize
//				}
//			}
//			for (int i = 0; i < no_routes; i++)
//			{
//				vehicle[i][0] = i;
//				vehicle[i][1] = route_cost[i];
//				vehicle[i][2] = saiz[i];
//				int k=1; //for vehic_depotCust
//				for (int j = 3; j <= saiz[i]+2; j++)
//				{
//					vehicle[i][j] = vehic_depotCust[i][k];
//					k++;
//				}
//				vehicle[i][saiz[i]+3] = -1;
//			}
//
//			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ for infeasibility check @@@@@@@@@@@@@@@@@@@@@@@@//
//			for (int i = 0; i < no_routes; i++)
//			{
//				if ((total_demand[i]>CAPACITY) || (distance_cost[i] - DISTANCE > 0.05))//for infeasibility check
//				{
//					cout << "Infeasibility check In VNS multi-level after module "<< Neighbour_k <<endl;
//					violation << "In VNS multi-level after module "<< Neighbour_k <<endl;
//					for (int j = 0; j <= saiz[i]+3; j++)
//					{
//						violation << vehicle[i][j]<<' ';
//					}
//					violation<<endl;
//					violateThreshold = violateThreshold/2; //divide by half if the solution after LS still violate constraint
//					//if (Neighbour_k < max_neigbor)
//					//{
//						Neighbour_k++; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//						goto skip_check_local;
//					
//					//}
//					//else //if maximum neighbourhood is reached
//					//	goto violateDiverstart; //if violation, skip all the process of compare LOCAL_BEST and GLOBAL_BEST, check if max diversification is reached
//				}
//			}
//			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ End of (for infeasibility check) @@@@@@@@@@@@@@@@@@@@@@@@//
//
//			if ((LOCAL_BEST_COST - total_cost) > 0.05 )//t_cost < LOCAL_BEST_COST
//			{
//				//reinitialize vehicle[][] so that it can be passed to shaking afterwards
//				//============= Record the best solution ====================//
//				LOCAL_BEST_COST = total_cost;
//				cout << endl << "LOCAL_BEST COST IS " << LOCAL_BEST_COST << endl;
//				for (int i = 0; i < SIZE; i++)
//				{
//					for (int j = 0; j < SIZE; j++)
//					{
//						LOCAL[i][j] = -1;  //reinitialize
//						vehicle[i][j] = -1;  //reinitialize
//					}
//				}
//				//int num_skip_r = 0;
//				//int r=0;//for route index because empty route has been deleted
//				for (int i = 0; i < no_routes; i++)
//				{
//					int j=0;
//					LOCAL[i][j] = i;//route index
//					j++;
//					LOCAL[i][j] = vehicle[i][j] = route_cost[i];//route cost
//					j++;
//					LOCAL[i][j] = vehicle[i][j] = saiz[i];//route size
//					j++;
//			
//					for (int k = 1; k <= saiz[i]; k++)
//					{
//						LOCAL[i][j] = vehicle[i][j] = vehic_depotCust[i][k]; //vehic_depotCust start with depot, end with depot
//						j++;
//					}
//	
//					LOCAL[i][j] = vehicle[i][j] = -1; //because n starts from 1, n+1 is route size
//					LOCAL_SAIZ[i] = saiz[i]; //copy temp saiz[] to best SAIZ[]
//					LOCAL_capa[i] = total_demand[i]; // capa[] record the BEST_ROUTE demand
//					//r++;
//				}
//				if (no_routes != LOCAL_NO_ROUTE)
//				{
//					cout<<"ERROR no_routes != LOCAL_NO_ROUTE in VNS multi-level)"<<endl;
//					exit(EXIT_FAILURE);
//				}
//
//
//				//=================================record localbest solution in txt=====================================//
//				cout << "========================================================================== " << endl;
//				//ofstream localbest_sol("30.LOCAL_BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
//				//=================================display final solution=====================================//
//				for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//				{
//					for (int j = 0; j <= LOCAL_SAIZ[i]+3; j++)
//					{
//						localbest_sol << LOCAL[i][j] << ' ';
//					}
//					localbest_sol<<endl;
//				}
//				localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"in VNS multi-level after shake Neighbour_k "<<Neighbour_k <<"level " <<level<< endl;	
//		
//				for (int l = 1; l <= max_level; l++)
//				{
//					localbest_sol << "LS["<<l<<"]= "<<LS[l]<<' '<< ' ';
//				}
//				localbest_sol <<endl;
//				//localbest_sol.close();
//				//======================= End of record localbest solution ===========================//
//
//				Neighbour_k = 1;	
//			}
//			else  //if is is not better than the incumbent
//			{
//				
//				if (Neighbour_k == max_neigbor) //current neighbourhood is the last neighbourhood, perform dijkstra refinement
//				{
//					float cost = 0.0; //to be passed to dijkstra_refinement
//					//dijkstra_refinement always refine the LOCAL_BEST_SOL (incumbent best)
//					//vehicle[][] is not important, it will be reinitialize in dijkstra_refinement, and passed out from there
//					cout<<"Entering dijkstra_refinement in kth VNS"<<endl;
//					dijkstra_refinement(cost, vehicle); 
//					cout<<"cost after dijkstra_refinement= "<<cost<<endl;
//					if ((LOCAL_BEST_COST-cost) > 0.05)//cost < LOCAL_BEST_COST
//					{
//						//============= Record the best solution ====================//
//						LOCAL_BEST_COST = cost;
//						cout << endl << "LOCAL_BEST COST IS " << LOCAL_BEST_COST << endl;
//						for (int i = 0; i < SIZE; i++)
//						{
//							for (int j = 0; j < SIZE; j++)
//							{
//								LOCAL[i][j] = -1;  //reinitialize
//							}
//
//						}
//						sort_LOCALsolution (vehicle, LOCAL_NO_ROUTE, demand);
//						//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//						//{
//						//	for (int j = 0; j <= saiz[i]+3; j++)
//						//	{
//						//		LOCAL[i][j] = vehicle[i][j];
//						//	}
//			
//						//	LOCAL_SAIZ[i] = saiz[i]; //copy temp saiz[] to best SAIZ[]
//						//	LOCAL_capa[i] = total_demand[i]; // capa[] record the BEST_ROUTE demand
//						//}
//
//						//=================================record localbest solution in txt=====================================//
//						cout << "========================================================================== " << endl;
//						//ofstream localbest_sol("30.LOCAL_BEST_SOLUTION.txt", ios::app);
//						//=================================display final solution=====================================//
//						for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//						{
//							for (int j = 0; j <= LOCAL_SAIZ[i]+3; j++)
//							{
//								localbest_sol << LOCAL[i][j] << ' ';
//							}
//							localbest_sol<<endl;
//						}
//						localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
//						//localbest_sol.close();
//						//======================= End localrecord best solution ===========================//
//						Neighbour_k = 1;	
//					}//end if (cost < LOCAL_BEST_COST)	
//				}//end if (Neighbour_k == max_neigbor) //current neighbourhood is the last neighbourhood
//				Neighbour_k++;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //put at the back on 18June2015, previously before    if (Neighbour_k > max_neigbor)  
//			}
//		skip_check_local:
//			for (int i = 0; i < k; i++)
//			{
//				delete[] kGain[i];
//			}delete[] kGain;
//			delete[] LS; //this is just to record which LS produce improvement for print out purpose
//		}//end while (Neighbour_k <= max_neigbor)	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@	
//
//		//violateDiverstart: //if violation, skip the process of compare LOCAL_BEST , check if max diversification is reached
//		if ((GLOBAL_BEST_COST-LOCAL_BEST_COST) > 0.05)//LOCAL_BEST_COST < GLOBAL_BEST_COST
//		{
//			GLOBAL_BEST_COST = LOCAL_BEST_COST;
//			cout << endl << "GLOBAL_BEST COST IS " <<GLOBAL_BEST_COST << endl;
//			found_GLOBEST_status = 1; //for diversificaton use
//
//			for (int i = 0; i < SIZE; i++)
//			{
//				for (int j = 0; j < SIZE; j++)
//				{
//					GLOBAL[i][j] = -1;  //reinitialize
//				}
//				GLOBAL_SAIZ[i] = 0;
//				GLOBAL_capa[i] = 0;
//			}
//			
//			//record the best solution in GLOBAL_BEST_ROUTE in sorted order
//			sort_GLOBALsolution (LOCAL_BEST_ROUTE, LOCAL_NO_ROUTE, demand);
//			//int r = 0; //for GLOBAL_NO_ROUTE becuase do not consider empty route
//			//for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			//{
//			//	if (LOCAL_SAIZ[i] == 0)
//			//		continue;
//			//	GLOBAL[r][0] = r;
//			//	for (int j = 1; j <= LOCAL_SAIZ[i]+3; j++)
//			//	{
//			//		GLOBAL[r][j] = LOCAL[i][j];
//			//		if ((j>=3) && (j < LOCAL_SAIZ[i]+3))
//			//		{
//			//			GLOBAL_capa[r] = GLOBAL_capa[r] + demand[(int)LOCAL[i][j]];
//			//		}
//			//	}
//			//	GLOBAL_SAIZ[r] = GLOBAL[r][2];
//			//	r++;
//			//}
//			//GLOBAL_NO_ROUTE = r;
//
//			//=================================record the best solution=====================================//
//			cout << "========================================================================== " << endl;
//
//			//ofstream best_sol("7.BEST_SOLUTION.txt", ios::app);
//			//=================================display final solution=====================================//
//			for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
//			{
//				int k = 0;
//				do
//				{
//					best_sol << GLOBAL[i][k] << ' ';
//					k++;
//
//				} while (GLOBAL[i][k] != -1);
//
//				best_sol << "-1" << endl;
//
//			}
//
//			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
//			best_sol << "From VNS multi level after Neighbour_k " <<Neighbour_k<<' ' <<' '<<"NbDiv= "<<NbDiv<<  endl;
//			//best_sol.close();
//		}
//
//	
//		NbDiv++;
//		float cost = 0;
//		int div=0;
//		if (found_GLOBEST_status == 1)
//		{
//			cost = make_giant_tour( vehicle, no_routes); //pass in vehicle and no_routes to be updated in Dijkstra, they will be initialize in the Dijkstra function
//			Ndel_min = LB_Ndel_min; //back to initial value
//		}
//		else if (found_GLOBEST_status == 0) //if not found global_best
//		{
//			cout<<"Entering Diversify LNS/conflict sector in VNS kth improvement: ";
//			div=(rand() % 4); 
//			if (div == 0)
//			{
//				cout<<"LNS Diversification "<<endl;
//				cost = Diversify_LongestArc(vehicle, no_routes, Ndel_min, x, y);
//			}
//			else if (div == 1)
//			{
//				cout<<"LNS Diversification overlap route"<<endl;
//				//Diversification_overlapRoute(float **(&vehicle), int (&no_routes), int K, float *x, float *y)
//				cost = Diversification_overlapRoute2(vehicle, no_routes, Ndel_min, x, y);
//			}
//			else if (div == 2)
//			{
//				cout<<"LNS Diversification 2 "<<endl;
//				cost = Diversify_LNS2(vehicle, no_routes, Ndel_min, x, y);
//				cost = Diversify_LNS3(VEHICLE, Ndel_min);
//			}
//			else if (div == 3)
//			{
//				cout<<"Conflict sector Diversification: "<<endl;
//				cost = Diversification_conflict_sector(vehicle, no_routes, Ndel_min, x, y);
//			}
//			int incre = ceil(0.05*SIZE);
//			Ndel_min+=incre;
//			if (Ndel_min > UB_Ndel_min)
//				Ndel_min = UB_Ndel_min; //added this on 2March2016
//		}
//		
//		//AFter diversification, record in LOCAL_BEST_ROUTE, when it enters restartAfterDiver, it will restart with add empty route to LOCAL_BEST_ROUTE again
//		for (int i = 0; i < SIZE; i++)
//		{
//			for (int j = 0; j < SIZE; j++)
//			{
//				LOCAL[i][j] = -1;
//			}
//		}
//		int k=0;//for LOCAL[][] route index
//		for (int i = 0; i < no_routes; i++)
//		{
//			if (saiz[i] == 0)
//				continue;
//			LOCAL[k][0] = k;
//			for (int j = 1; j <= saiz[i]+3; j++)
//			{
//				LOCAL[k][j] = vehicle[i][j];
//			}
//			LOCAL_SAIZ[k] = saiz[i];
//			LOCAL_capa[k] = total_demand[i];
//			k++;
//		}
//		LOCAL_BEST_COST = cost;
//		LOCAL_NO_ROUTE = k;
//		//if the cost better than GLOBAL_BEST_COST 
//		if (GLOBAL_BEST_COST-cost > 0.05)//cost < GLOBAL_BEST_COST
//		{
//			for (int i = 0; i < SIZE; i++)
//			{
//				for (int j = 0; j < SIZE; j++)
//				{
//					GLOBAL[i][j] = -1;
//				}
//			}
//			int m=0;//for GLOBAL[][] route index
//			for (int i = 0; i < no_routes; i++)
//			{
//				if (saiz[i] == 0)
//					continue;
//				GLOBAL[m][0] = m;
//				best_sol << m<<' ';
//				for (int j = 1; j <= saiz[i]+3; j++)
//				{
//					GLOBAL[m][j] = vehicle[i][j];
//					best_sol << GLOBAL[m][j] << ' ';
//				}
//				best_sol <<endl;
//				GLOBAL_SAIZ[m] = saiz[i];
//				GLOBAL_capa[m] = total_demand[i];
//				m++;
//			}
//			GLOBAL_BEST_COST = cost;
//			GLOBAL_NO_ROUTE = m;
//
//			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
//			best_sol << "From VNS multi-level after Diversification type "<< div <<  endl;
//
//		}
//		best_sol << "Type of div = "<<div<<endl;
//		for (int i = 0; i < SIZE; i++)
//		{
//			delete[] vehicle[i];
//			delete[] vehic_depotCust[i];
//		}
//		delete[] vehicle;
//		delete[] vehic_depotCust;
//		delete[] cost_of_removing;
//		delete[] Oricost_of_removing;
//
//	}//end while (NbDiv <= maxDiv) //#############################################################################################################################
//	
//	timefile << "VNS multi-level "<<maxDiv <<" diversification time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << " second"<<endl;
//	recordTIME << "VNS  multi-level ("<<std::to_string( I )<<") with "<<maxDiv <<" diversification time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << " second"<<endl;
//	best_sol.close();
//	localbest_sol.close();
//
//
//}


//insert_1_0 (same route based on modified route, diff beased on nothing
//DONE
void insert_1_0_2nd(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute) //from is fromroute 
{
	//int no_routes = LOCAL_NO_ROUTE;
	float total_cost = 0.0;

	int old_pos = -1;
	int new_pos = -1;
	int ele = -1;
	int from = -1;
	int to = -1;
	float gain = kGain[number][1];
	bool *tempFlag = new bool[SIZE+1];
	for (int i = 0; i <= SIZE; i++)
	{
		tempFlag[i] = false;//false noneed to find, true need to find
	}
	//float r2_cost=0.0;
	//based on modified
	if (sameroute == true)//if it is from the same route
	{
		int route = (int)kGain[number][2];
		old_pos = kGain[number][4];
		new_pos = kGain[number][5];  //this position is based on modified route!!!!!!!!, assumed one has been deleted
		ele = VEHICLE[route][old_pos];
		from = route;
		to = route;
		//gain = kGain[number][1];

		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=1;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		int specialdelete=0;//mean insert 1 
		//r1
		for (int i = -1; i <= delete1; i++)//flag the one before, current, and one after
		{
			tempFlag[VEHICLE[from][old_pos+i]] = true;//flagged customer affected true
		}

		if (ele == SIZE || ele < 0)
			getchar();

		int* temp_r = new int[saiz[route]+2];

		//copy route without ele
		int b=0;
		for (int a = 0; a < saiz[route] + 2; a++)
		{
			if (a == old_pos)
			{
				continue; //dont copy the element, assume the element to be deleted
			}

			else
			{
				temp_r[b] = VEHICLE[route][a];
				b++;
			}
		}
		//r2 //r2 position is based on modified route, so flag the customer after one customer has been deleted
		for (int i = -1; i <= specialdelete; i++)//flag the one before and current
		{
			tempFlag[temp_r[new_pos+i]] = true;//flagged customer affected true
		}

		int i = saiz[route]-1+1; //1 has been deleted, so saiz-1. +1 so that copy the last element shift right
		///changed from new_pos+1 to new_pos on 11 Feb 2015
		while ( (i >= new_pos) && (i>=1) )
		{
			temp_r[i+1] = temp_r[i];
			i--;
		}
	
		temp_r[i+1] = ele;

		//std::vector<int> myvector1;
		//std::vector<int> myvector2;
		//
		//for (int i = 0; i<saiz[route] + 2; i++) myvector1.push_back(VEHICLE[route][i]);

		//myvector1.erase(myvector1.begin() + old_pos); //+1 because element [0] is depot

		//for (int j = 0; j<saiz[route] + 2; j++) myvector2.push_back(VEHICLE[route][j]);

		//myvector2.insert(myvector2.begin() + new_pos, ele);


		for (int k = 0; k < saiz[route]+2; k++)
		{
			VEHICLE[route][k] = temp_r[k];  //copy to VEHICLE matrix
		}
		distance_cost[route] = distance_cost[route] - kGain[number][11];
		//r2_cost = 0.0;//reinitialize //to check
		//for (int c = 0; c < saiz[route] + 1; c++)
		//{
		//	r2_cost += dist[VEHICLE[route][c]][VEHICLE[route][c + 1]] + service_time[VEHICLE[route][c]];
		//}
		//route_cost[route] = r1_cost; 
		route_cost[route] = route_cost[route] - gain; //////////////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//if (abs(route_cost[route]-r2_cost)> 0.1)
		//{
		//	cout<<"3.In insert 1-0"<<endl;
		//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
		//	cout<< route_cost[route]<<' '<<r2_cost<<endl;
		//	getchar();
		//}
		distance_available[route] = DISTANCE - distance_cost[route];
		//noneed to update total_demand[] as it is still the same
		//update CumDist starting from the affecting pos until last cust
		int start = old_pos;//the start of update point, always hold old or new hold whichever comes first
		if (new_pos < old_pos)
			start = new_pos;
		for (int j = start; j <=saiz[route]; j++)
		{
			CumDist[route][j] = CumDist[route][j-1]+dist[VEHICLE[route][j-1]][VEHICLE[route][j]] + service_time[VEHICLE[route][j]];
		}
		delete[] temp_r;
		
	}

	else if (sameroute == false) //if it is from different route
	{
		old_pos = kGain[number][4];
		new_pos = kGain[number][5];
		ele = VEHICLE[(int)kGain[number][2]][old_pos];
		from = (int)kGain[number][2];
		to = (int)kGain[number][3];
		//gain = kGain[number][1];


		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=1;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		int delete2=0;//mean insert 1
		//r1 //if different route, put together, if same route, put this seperately
		for (int i = -1; i <= delete1; i++)//flag the one before, current, and one after
		{
			tempFlag[VEHICLE[from][old_pos+i]] = true;//flagged customer affected true
		}

		//r2
		for (int i = -1; i <= delete2; i++)//flag the one before and current
		{
			tempFlag[VEHICLE[to][new_pos+i]] = true;//flagged customer affected true
		}

		if (ele == SIZE || ele < 0)
			getchar();

		//r1 (delete 1)
		for (int i = old_pos; i<=saiz[from]; i++)
		{
			VEHICLE[from][i] = VEHICLE[from][i+1];
		}
		VEHICLE[from][saiz[from]+1] = -1;//optional
		saiz[from] = saiz[from] - 1;
		distance_cost[from] = distance_cost[from] - kGain[number][11];
		route_cost[from] = route_cost[from] - kGain[number][10]; ////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		total_demand[from] = total_demand[from] - demand[ele];
		distance_available[from] = DISTANCE - distance_cost[from];
		space_available[from] = CAPACITY - total_demand[from];
		//update CumDist starting from the affecting pos until last cust
		int start = old_pos;//the start of update point
		for (int j = start; j <=saiz[from]; j++)
		{
			CumDist[from][j] = CumDist[from][j-1]+dist[VEHICLE[from][j-1]][VEHICLE[from][j]] + service_time[VEHICLE[from][j]];
		}

		//r2 (insert 1)
		int j = saiz[to]+1; //start copy from the ending depot
		while ( (j >= (new_pos)) && (j>0) )
		{
			VEHICLE[to][j+1] = VEHICLE[to][j];
			j--;
		}
		VEHICLE[to][j+1] = ele;

		saiz[to] = saiz[to] + 1; //size of new route is plus one
		distance_cost[to] = distance_cost[to] - kGain[number][12];
		route_cost[to] = route_cost[to] - (gain - kGain[number][10]); ////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		total_demand[to] = total_demand[to] + demand[ele];
		distance_available[to] = DISTANCE - distance_cost[to];
		space_available[to] = CAPACITY - total_demand[to];
		//update CumDist starting from the affecting pos until last cust
		int start2 = new_pos;//the start of update point
		for (int j = start2; j <=saiz[to]; j++)
		{
			CumDist[to][j] = CumDist[to][j-1]+dist[VEHICLE[to][j-1]][VEHICLE[to][j]] + service_time[VEHICLE[to][j]];
		}

		
	}
	//////////////////////// POSITION OF INSERTION IS FOR MODIFIED ROUTE !!!!!!!!!!!!!!! ////////////////////////////


	ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	route_file << ' ' << endl;
	//======================================================Display route========================================================================
	route_file << "==================From insert (1-0)===================================== " << endl;
	route_file << "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g]==0)
		{
			route_cost[g]=0;
		}
		
		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';

		for (int h = 0; h < saiz[g] + 2; h++)
		{
			route_file << VEHICLE[g][h] << ' ';
		}
		route_file <<endl;
		total_cost = total_cost + route_cost[g];
		if (route_cost[g] < 0)
		{
			cout<< "in insert 1_0, route_cost["<<g<<"] is negative!"<<endl;
			getchar();
		}

	}
	route_file << "Total cost = " << total_cost << endl;
	route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	route_file << "========================================================================== " << endl;

	//////=================================================update cost of removing=====================================//////////////////////
	
	if (from != to) //if diferent routes
	{
		for (int g = 0; g < 2; g++) //find cost of removing for each customer and put in array
		{
			int f;

			if (g == 0)
			{
				f = from;
			}
			else
			{
				f = to;
			}
			
			
	
		updateCostofRemoving2 (f, cost_of_removing);


			
			//for (int h = 1; h < saiz[f] + 1; h++) //first customer start from element [1]
			//{
			//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]]  + service_time[VEHICLE[f][h]]- dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
			//}
			cout<<"Insert 1-0 diff R"<<endl;
			//int status=comprareCost (cost_of_removing, f);//to check 
			//if (status == 1)
			//	cout<<old_pos<<' '<<' '<<new_pos<<endl;
		}
		
	}

	else // if the same route
	{
		int f = from;
		
	
		updateCostofRemoving2 (f, cost_of_removing);


		//for (int h = 1; h < saiz[f] + 1; h++) //first customer start from element [1]
		//{
		//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]]  + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
		//}
		cout<<"Insert 1-0 same R"<<endl;
		//int status=comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<old_pos<<' '<<' '<<new_pos<<endl;
	}
	delete[] tempFlag;
}

//insert_1_1 (same route based on non-modified route, diff based on modified route
//DONE
void insert_1_1_2nd(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute)
{
	float total_cost = 0.0;
	int r1 = -1;
	int r2 = -1;
	float gain = kGain[number][1];
	bool *tempFlag = new bool[SIZE+1];
	for (int i = 0; i <= SIZE; i++)
	{
		tempFlag[i] = true;//false noneed to find, true need to find
	}
	//float r2_cost = 0.0;
	//based on non-modified route
	if(sameroute == true) //if it is the same route, the position is to swap
	{
		int route = (int)kGain[number][2];
		int old_pos = kGain[number][4];
		int new_pos = kGain[number][5];
		int ele1 = VEHICLE[route][old_pos];
		int ele2 = VEHICLE[route][new_pos];
		r1 = route;
		r2 = route;
		//gain = kGain[number][1];

		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=1;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5

		//r1 
		for (int i = -1; i <= delete1; i++)//flag the one before, current, and one after
		{
			tempFlag[VEHICLE[route][old_pos+i]] = true;//flagged customer affected true
		}

		//r2
		for (int i = -1; i <= delete1; i++)//flag the one before, current, and one after
		{
			tempFlag[VEHICLE[route][new_pos+i]] = true;//flagged customer affected true
		}


		if (ele1 == SIZE || ele1 < 0 || ele2 == SIZE || ele2 < 0 )
			getchar();

		//copy route to swap ele1 and ele2
		for (int a = 0; a < saiz[route] + 2; a++)
		{
			if (a == old_pos)
			{
				VEHICLE[route][a] = ele2;
			}

			else if (a == new_pos)
			{
				VEHICLE[route][a] = ele1;
				
			}
		}
		distance_cost[route] = distance_cost[route] - kGain[number][11];
		//r2_cost = 0.0; //reinitialize //to check
		//for (int c = 0; c < saiz[route] + 1; c++)
		//{
		//	r2_cost += dist[VEHICLE[route][c]][VEHICLE[route][c + 1]] + service_time[VEHICLE[route][c]];
		//}
		route_cost[route] = route_cost[route] - gain;
		//if (abs(route_cost[route]-r2_cost)> 0.1)
		//{
		//	cout<<"1.In insert 1-1"<<endl;
		//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
		//	cout<< route_cost[route]<<' '<<r2_cost<<endl;
		//	getchar();
		//}
		distance_available[route] = DISTANCE - distance_cost[route];
		//update CumDist starting from the affecting pos until last cust
		int start = old_pos;//the start of update point, always hold old or new hold whichever comes first
		if (new_pos < old_pos)
			start = new_pos;
		for (int j = start; j <=saiz[route]; j++)
		{
			CumDist[route][j] = CumDist[route][j-1]+dist[VEHICLE[route][j-1]][VEHICLE[route][j]] + service_time[VEHICLE[route][j]];
		}
		
	}

	else if (sameroute == false) //if from different route
	{
		//##############################################################################################################################################
		//======================================================Route 1 ===================================================================================
		int r1_without = kGain[number][4];
		int r2_with = kGain[number][5];//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		int r2_without = kGain[number][6];
		int r1_with = kGain[number][7];//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		r1 = kGain[number][2];
		r2 = kGain[number][3];

		int ele2=-1, ele1=-1;

		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=1;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		
		//r1 
		for (int i = -1; i <= delete1; i++)//flag the one before, current, and one after
		{
			tempFlag[VEHICLE[r1][r1_without+i]] = true;//flagged customer affected true
		}

		//r2
		for (int i = -1; i <= delete1; i++)//flag the one before, current, and one after
		{
			tempFlag[VEHICLE[r2][r2_without+i]] = true;//flagged customer affected true
		}

		std::vector<int> myvector1;
		std::vector<int> myvector2;
		for (int i = 0; i<saiz[r1] + 2; i++)
		{
			if (i==r1_without)
			{
				ele2 = VEHICLE[r1][i];
				continue;
			}
			myvector1.push_back(VEHICLE[r1][i]);
		}
		//int ele2 = myvector1[r1_without];
		// erase the element from route 1
		//myvector1.erase(myvector1.begin() + r1_without);


		for (int j = 0; j<saiz[r2] + 2; j++)
		{
			if (j==r2_without)
			{
				ele1 = VEHICLE[r2][j];
				continue;
			}
			myvector2.push_back(VEHICLE[r2][j]);
		}

		int insert1=0;//means insert 1 (in modified route)
		//r1 
		for (int i = -1; i <= insert1; i++)//flag the one before, and current
		{
			tempFlag[myvector1[r1_with+i]] = true;//flagged customer affected true
		}

		//r2
		for (int i = -1; i <= insert1; i++)//flag the one before, and current
		{
			tempFlag[myvector2[r2_with+i]] = true;//flagged customer affected true
		}

		if (ele1 == SIZE || ele1 < 0 || ele2 == SIZE || ele2 < 0 )
			getchar();
		//int ele1 = myvector2[r2_without];
		//myvector2.erase(myvector2.begin() + r2_without);

		myvector1.insert(myvector1.begin() + r1_with, ele1);
		myvector2.insert(myvector2.begin() + r2_with, ele2);

		for (int i = 0; i <= saiz[r1]+1; i++)
		{
			VEHICLE[r1][i] = -1; //reinitialize, otherwise the old customers are still there, problem when deletion, customer become less than previous
		}

		for (int k = 0; k < myvector1.size(); k++)
		{
			VEHICLE[r1][k] = myvector1[k];  //copy to VEHICLE matrix
		}
		float new_sum_cost = route_cost[r1] + route_cost[r2] - gain;
		distance_cost[r1] = distance_cost[r1] - kGain[number][11];
		//route_cost[r1]=0.0, route_cost[r2]=0.0;	//reinitialize
		//for (int c = 0; c < saiz[r1] + 1; c++)
		//{
		//	route_cost[r1] += dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]] + service_time[VEHICLE[r1][c]];
		//}
		route_cost[r1] = route_cost[r1] - kGain[number][10];////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		total_demand[r1] = total_demand[r1] - demand[ele2] + demand[ele1];
		distance_available[r1] = DISTANCE - distance_cost[r1];
		space_available[r1] = CAPACITY - total_demand[r1];

		for (int i = 0; i <= saiz[r2]+1; i++)
		{
			VEHICLE[r2][i] = -1; //reinitialize, otherwise the old customers are still there, problem when deletion, customer become less than previous
		}

		for (int p = 0; p < myvector2.size(); p++)
		{
			VEHICLE[r2][p] = myvector2[p];
		}
		distance_cost[r2] = distance_cost[r2] - kGain[number][12];
		//r2_cost=0.0; //to check
		//for (int c = 0; c < saiz[r2] + 1; c++)
		//{
		//	r2_cost += dist[VEHICLE[r2][c]][VEHICLE[r2][c + 1]] + service_time[VEHICLE[r2][c]];
		//}

		route_cost[r2] = new_sum_cost - route_cost[r1] ;
		//if (abs(route_cost[r2]-r2_cost)> 0.1)
		//{
		//	cout<<"2.In insert 1-1"<<endl;
		//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
		//	cout<< route_cost[r2]<<' '<<r2_cost<<endl;
		//	getchar();
		//}
		total_demand[r2] = total_demand[r2] - demand[ele1] + demand[ele2];
		distance_available[r2] = DISTANCE - distance_cost[r2];
		space_available[r2] = CAPACITY - total_demand[r2];
		//update CumDist starting from the affecting pos until last cust
		int start = r1_without;//the start of update point, always hold old or new hold whichever comes first
		if (r1_with < r1_without)
			start = r1_with;
		for (int j = start; j <=saiz[r1]; j++)
		{
			CumDist[r1][j] = CumDist[r1][j-1]+dist[VEHICLE[r1][j-1]][VEHICLE[r1][j]] + service_time[VEHICLE[r1][j]];
		}
		//update CumDist starting from the affecting pos until last cust
		int start2 = r2_without;//the start of update point, always hold old or new hold whichever comes first
		if (r2_with < r2_without)
			start2 = r2_with;
		for (int j = start2; j <=saiz[r2]; j++)
		{
			CumDist[r2][j] = CumDist[r2][j-1]+dist[VEHICLE[r2][j-1]][VEHICLE[r2][j]] + service_time[VEHICLE[r2][j]];
		}
	}//end of else not the same route

	ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	route_file << ' ' << endl;
	//======================================================Display route========================================================================
	route_file << "==================From insert (1-1)===================================== " << endl;
	route_file << "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g]==0)
		{
			route_cost[g]=0;
			total_demand[g]=0;
		}

		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		for (int h = 0; h < saiz[g] + 2; h++)
		{
			route_file << VEHICLE[g][h] << ' ';	
		}
		route_file << endl;	

		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "in insert 1_1,  route_cost["<<g<<"] is negative!"<<endl;
			getchar();
		}

	}
	route_file << "Total cost = " << total_cost << endl;
	route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	route_file << "========================================================================== " << endl;
	
	//////=================================================update cost of removing=====================================//////////////////////
	if (r1 != r2) //if diferent routes
	{
		for (int g = 0; g < 2; g++) //find cost of removing for each customer and put in array
		{
			int f;

			if (g == 0)
			{
				f = r1;
			}
			else
			{
				f = r2;
			}
	
	
			updateCostofRemoving2 (f, cost_of_removing);
	
			cout<<"Insert 1-1 diff R"<<endl;
			//int status	= comprareCost (cost_of_removing, f);//to check 
			//if (status == 1)
			//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][5]<<' '<<' '<<kGain[number][6]<<' '<<' '<<kGain[number][7]<<endl;
		}
	}
	
	else // if the same route
	{
		int f = r1;

	
		updateCostofRemoving2 (f, cost_of_removing);

		cout<<"Insert 1-1 same R"<<endl;
		//int status	= comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][5]<<endl;
	}
	delete[] tempFlag;
}

void insert_swap1_1(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute)
{
	float total_cost = 0.0;
	int r1 = -1;
	int r2 = -1;
	float gain = kGain[number][1];
	//========================  ALL CUSTOMER POSITION IN NON_MODIFIED ROUTE ===================//
	if(sameroute == true) //if it is the same route, the position is to swap
	{
		int route = kGain[number][2];
		int old_pos = kGain[number][4];
		int new_pos = kGain[number][5];
		int ele1 = VEHICLE[route][old_pos];
		int ele2 = VEHICLE[route][new_pos];
		r1 = route;
		r2 = route;
		//gain = kGain[number][1];
		if (ele1 == SIZE || ele1 < 0 || ele2 == SIZE || ele2 < 0 )
			getchar();
		
		VEHICLE[route][old_pos] = ele2;
		VEHICLE[route][new_pos] = ele1;
		distance_cost[route] = distance_cost[route] - kGain[number][11];
		//route_cost[route] = 0.0 //reinitialize
		//for (int c = 0; c < saiz[route] + 1; c++)
		//{
		//	route_cost[route] += dist[VEHICLE[route][c]][VEHICLE[route][c + 1]] + service_time[VEHICLE[route][c]];
		//}
		route_cost[route] = route_cost[route] - gain;
		distance_available[route] = DISTANCE - distance_cost[route];
	}

	else if (sameroute == false) //if from different route
	{
		//##############################################################################################################################################
		//======================================================Route 1 ===================================================================================
		
		int r1_without = kGain[number][4]; //from r1_p
		int r1_with = kGain[number][5];//to_r2_p //to which position (swap position)=route is not modified
		int r2_without = kGain[number][6];//from_r2_p
		int r2_with = kGain[number][7];//to_r1_p
		r1 = kGain[number][2];
		r2 = kGain[number][3];
		//[4]=[7], [5]=[6]
	
		int ele1 = VEHICLE[r1][r1_without];
		int ele2 = VEHICLE[r2][r2_without];
		if (ele1 == SIZE || ele1 < 0 || ele2 == SIZE || ele2 < 0 )
			getchar();
		VEHICLE[r1][r1_without] = ele2;
		VEHICLE[r2][r2_without] = ele1;
		
		float new_sum_cost = route_cost[r1] + route_cost[r2] - gain;
		distance_cost[r1] = distance_cost[r1] - kGain[number][11];
		//route_cost[r1]=0.0, route_cost[r2]=0.0;	//reinitialize
		//for (int c = 0; c < saiz[r1] + 1; c++)
		//{
		//	route_cost[r1] += dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]] + service_time[VEHICLE[r1][c]];
		//}
		route_cost[r1] = route_cost[r1] - kGain[number][10];////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		total_demand[r1] = total_demand[r1] - demand[ele1] + demand[ele2];
		distance_available[r1] = DISTANCE - distance_cost[r1];
		space_available[r1] = CAPACITY - total_demand[r1];
		
		distance_cost[r2] = distance_cost[r2] - kGain[number][12];
		//for (int c = 0; c < saiz[r2] + 1; c++)
		//{
		//	route_cost[r2] += dist[VEHICLE[r2][c]][VEHICLE[r2][c + 1]] + service_time[VEHICLE[r2][c]];
		//}
		route_cost[r2] = new_sum_cost - route_cost[r1];
		total_demand[r2] = total_demand[r2] - demand[ele2] + demand[ele1];
		distance_available[r2] = DISTANCE - distance_cost[r2];
		space_available[r2] = CAPACITY - total_demand[r2];

	}//end of else not the same route

	ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	route_file << ' ' << endl;
	//======================================================Display route========================================================================
	
	route_file << "==================From insert (1-1)swap===================================== " << endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g]==0)
		{
			route_cost[g]=0;
		}
		
		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		for (int h = 0; h < saiz[g] + 2; h++)
		{
		
			route_file << VEHICLE[g][h] << ' ';
		
		}
		route_file <<endl;
		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "in insert swap 1_1,  route_cost["<<g<<"] is negative!"<<endl;
			getchar();
		}
	}
	route_file << "Total cost = " << total_cost << endl;
	route_file << "========================================================================== " << endl;
	route_file << "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	//////=================================================update cost of removing=====================================//////////////////////
	if (r1 != r2) //if diferent routes
	{
		for (int g = 0; g < 2; g++) //find cost of removing for each customer and put in array
		{
			int f;
			if (g == 0)
				f = r1;
			else
				f = r2;

			for (int h = 1; h < saiz[f] + 1; h++) //first customer start from element [1]
			{
				cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]]- dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
			}

		}
	}
	
	else // if the same route
	{
		int f = r1;
		for (int h = 1; h < saiz[f] + 1; h++) //first customer start from element [1]
		{
			cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
		}
		
	}

}

//insert_2_1 (same route based on non-modified route, diff based on modified route
//DONE
void insert_2_1_2nd(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute)
{
	//##############################################################################################################################################
	//======================================================Route 1 ===================================================================================
	//int no_routes = LOCAL_NO_ROUTE;
	float total_cost = 0.0;
	int r1=-1, r2=-1;
	float gain=kGain[number][1];
	bool *tempFlag = new bool[SIZE+1];
	for (int i = 0; i <= SIZE; i++)
	{
		tempFlag[i] = true;//false noneed to find, true need to find
	}
	int reverseStatus;
	//float r2_cost=0.0;
	
	if(sameroute == true) //if it is the same route, the position is to swap
	{
		int route = (int)kGain[number][2];
		int pos1 = kGain[number][4];
		int pos2 = kGain[number][4]+1;
		int pos3 = kGain[number][5];
		int ele1 = VEHICLE[route][pos1];
		int ele2 = VEHICLE[route][pos2];
		int ele3 = VEHICLE[route][pos3];
		if (ele1 == SIZE || ele1 < 0 || ele2 == SIZE || ele2 < 0  ||ele3 == SIZE || ele3 < 0 )
		{
			cout<<ele1<<' '<<ele2<<' '<<ele3<<endl;
			getchar();
		}
		r1 = route;
		r2 = route;
		//gain = kGain[number][1];

		//non modified route
		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=2;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		int delete2=1;
		//r1 
		for (int i = -1; i <= delete1; i++)//flag the one before, current1&2, and one after
		{
			tempFlag[VEHICLE[route][pos1+i]] = true;//flagged customer affected true
		}

		//r2
		for (int i = -1; i <= delete2; i++)//flag the one before, current, and one after
		{
			tempFlag[VEHICLE[route][pos3+i]] = true;//flagged customer affected true
		}

		
		reverseStatus = kGain[number][9];
		if (reverseStatus == 1)
		{
			int temp = ele1;
			ele1 = ele2;
			ele2 = temp;
		}

		//copy route to swap ele1, ele2 with ele3
		int *temp_r3 = new int[saiz[route] + 2];
		//for (int i = 0; i < saiz[route] + 2; i++)
		//{
		//	vector_r3[i] = VEHICLE[route][i];
		//}
		int a =0; //for vector_r3
		for (int i = 0; i < saiz[route] + 2; i++)
		{
			if (i==pos1)
			{
				temp_r3[a]= ele3;
				a++;
	
			}

			else if (i==pos2)
			{
				continue;
			}

			else if(i==pos3)
			{
				temp_r3[a] = ele1;
				a++;
				temp_r3[a] = ele2;
				a++;
			}

			else
			{
				temp_r3[a] = VEHICLE[route][i];
				a++;
			}

		}
		//copy to VEHICLE
		for (int i = 0; i < saiz[route] + 2; i++)
		{
			VEHICLE[route][i] = temp_r3[i];
		}
		distance_cost[route] = distance_cost[route] - kGain[number][11];
		//r2_cost = 0.0; //reinitialize //to check
		//for (int c = 0; c < saiz[route] + 1; c++)
		//{
		//	r2_cost += dist[VEHICLE[route][c]][VEHICLE[route][c + 1]] + service_time[VEHICLE[route][c]];
		//}
		//route_cost[route] = r1_cost;
		route_cost[route] = route_cost[route] - gain;//////////////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//if (abs(route_cost[route]-r2_cost)> 0.1)
		//{
		//	cout<<"1.In insert 2-1"<<endl;
		//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
		//	cout<< route_cost[route]<<' '<<r2_cost<<endl;
		//	getchar();
		//}
		distance_available[route] = DISTANCE - distance_cost[route];
		//update CumDist starting from the affecting pos until last cust
		int start = pos1;//the start of update point, always hold old or new hold whichever comes first
		if (pos3 < pos1)
			start = pos3;
		for (int j = start; j <=saiz[route]; j++)
		{
			CumDist[route][j] = CumDist[route][j-1]+dist[VEHICLE[route][j-1]][VEHICLE[route][j]] + service_time[VEHICLE[route][j]];
		}
		delete[] temp_r3;
	}//end of if same route

	else if (sameroute == false) //if different routes
	{

		int r1_without = kGain[number][4];
		int r1f_without = (kGain[number][4]) + 1; //the next element
		int r1_with = kGain[number][7];
		int r2_without = kGain[number][6];
		int r2_with = kGain[number][5];
		r1 = kGain[number][2];
		r2 = kGain[number][3];
		reverseStatus = kGain[number][9];

		//based on modified route
		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=2;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		int delete2=1;
		//r1 
		for (int i = -1; i <= delete1; i++)//flag the one before, current1&2, and one after
		{
			tempFlag[VEHICLE[r1][r1_without+i]] = true;//flagged customer affected true
		}

		//r2
		for (int i = -1; i <= delete2; i++)//flag the one before, current, and one after
		{
			tempFlag[VEHICLE[r2][r2_without+i]] = true;//flagged customer affected true
		}

		std::vector<int> myvector1;
		std::vector<int> myvector2;
		int ele2=-1, ele3=-1, ele1=-1;
		// set some values (from 1 to 10)
		for (int i = 0; i<saiz[r1] + 2; i++)
		{
			if (i==r1_without)
			{
				ele2 = VEHICLE[r1][i];
				continue;
			}

			if (i==r1f_without)
			{
				ele3 = VEHICLE[r1][i];
				continue;
			}
			myvector1.push_back(VEHICLE[r1][i]);
		}
		
		if (reverseStatus == 1)
		{
			int temp = ele2;
			ele2 = ele3;
			ele3 = temp;
		}
		//int ele2 = myvector1[r1_without];
		//int ele3 = myvector1[r1f_without];

		// erase the element from route 1
		//myvector1.erase(myvector1.begin() + r1_without);
		//myvector1.erase(myvector1.begin() + r1_without);

		for (int j = 0; j<saiz[r2] + 2; j++)
		{
			if (j==r2_without)
			{
				ele1 = VEHICLE[r2][j];
				continue;
			}
			myvector2.push_back(VEHICLE[r2][j]);
		}
		//based on modified route
		int insert1=0;
		//r1 
		for (int i = -1; i <= insert1; i++)//flag the one before, and current
		{
			tempFlag[myvector1[r1_with+i]] = true;//flagged customer affected true
		}

		//r2
		for (int i = -1; i <= insert1; i++)//flag the one before, and current
		{
			tempFlag[myvector2[r2_with+i]] = true;//flagged customer affected true
		}

		if (ele1 == SIZE || ele1 < 0 || ele2 == SIZE || ele2 < 0  ||ele3 == SIZE || ele3 < 0 )
		{
			cout<<ele1<<' '<<ele2<<' '<<ele3<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					cout<<VEHICLE[i][j]<<' ';
				}
				cout<<endl;
			}
			getchar();
		}

		myvector1.insert(myvector1.begin() + r1_with, ele1);//CHANGED ON 26FEB2015!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!#######################################################################
		myvector2.insert(myvector2.begin() + r2_with, ele3);
		myvector2.insert(myvector2.begin() + r2_with, ele2);

		//myvector1.insert(myvector1.begin() + r2_with, ele1);
		//myvector2.insert(myvector2.begin() + r1_with, ele3);
		//myvector2.insert(myvector2.begin() + r1_with, ele2);

		for (int i = 0; i <= saiz[r1]+1; i++)
		{
			VEHICLE[r1][i] = -1; //reinitialize, otherwise the old customers are still there, problem when deletion, customer become less than previous
		}
		for (int i = 0; i <= saiz[r2]+1; i++)
		{
			VEHICLE[r2][i] = -1; //reinitialize, otherwise the old customers are still there, problem when deletion, customer become less than previous
		}
		saiz[r1] = myvector1.size() - 2;
		saiz[r2] = myvector2.size() - 2;

		
		for (int k = 0; k < myvector1.size(); k++)
		{
			VEHICLE[r1][k] = myvector1[k];  //copy to VEHICLE matrix
		}

		float new_sum_cost = route_cost[r1] + route_cost[r2] - gain;
		distance_cost[r1] = distance_cost[r1] - kGain[number][11];
		//route_cost[r1]=0.0, route_cost[r2]=0.0;	//reinitialize
		//for (int c = 0; c < saiz[r1] + 1; c++)
		//{
		//	route_cost[r1] += dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]] + service_time[VEHICLE[r1][c]];
		//}
		route_cost[r1] = route_cost[r1] - kGain[number][10];////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		total_demand[r1] = total_demand[r1] - demand[ele2] - demand[ele3] + demand[ele1];
		distance_available[r1] = DISTANCE - distance_cost[r1];
		space_available[r1] = CAPACITY - total_demand[r1];

		for (int p = 0; p < myvector2.size(); p++)
		{
			VEHICLE[r2][p] = myvector2[p];
		}
		//siz[r2] = siz[r2] - 1; //size of new route is plus one
		distance_cost[r2] = distance_cost[r2] - kGain[number][12];
		//r2_cost=0.0; //to check
		//for (int c = 0; c < saiz[r2] + 1; c++)
		//{
		//	r2_cost = r2_cost + dist[VEHICLE[r2][c]][VEHICLE[r2][c + 1]] + service_time[VEHICLE[r2][c]];
		//}
		//route_cost[r2] = r2_cost;
		route_cost[r2] = new_sum_cost - route_cost[r1];
		//if (abs(route_cost[r2]-r2_cost)> 0.1)
		//{
		//	cout<<"2.In insert 2-1"<<endl;
		//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
		//	cout<< route_cost[r2]<<' '<<r2_cost<<endl;
		//	getchar();
		//}
		total_demand[r2] = total_demand[r2] - demand[ele1] + demand[ele2] + demand[ele3];
		distance_available[r2] = DISTANCE - distance_cost[r2];
		space_available[r2] = CAPACITY - total_demand[r2];
		//update CumDist starting from the affecting pos until last cust
		int start = r1_without;//the start of update point, always hold old or new hold whichever comes first
		if (r1_with < r1_without)
			start = r1_with;
		for (int j = start; j <=saiz[r1]; j++)
		{
			CumDist[r1][j] = CumDist[r1][j-1]+dist[VEHICLE[r1][j-1]][VEHICLE[r1][j]] + service_time[VEHICLE[r1][j]];
		}
		//update CumDist starting from the affecting pos until last cust
		int start2 = r2_without;//the start of update point, always hold old or new hold whichever comes first
		if (r2_with < r2_without)
			start2 = r2_with;
		for (int j = start2; j <=saiz[r2]; j++)
		{
			CumDist[r2][j] = CumDist[r2][j-1]+dist[VEHICLE[r2][j-1]][VEHICLE[r2][j]] + service_time[VEHICLE[r2][j]];
		}
	}//end of else (different route)


	ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	route_file << ' ' << endl;
	//======================================================Display route========================================================================
	route_file << "==================From insert (2-1)============================== " << endl;
	route_file << "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g]==0)
		{
			route_cost[g]=0;
			
		}
		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		for (int h = 0; h < saiz[g] + 2; h++)
		{
			route_file << VEHICLE[g][h] << ' ';	
		}
		route_file <<endl;
		total_cost = total_cost + route_cost[g];
		if (route_cost[g] < 0)
		{
			cout<< "in insert 2_1,  route_cost["<<g<<"] is negative!"<<endl;
			getchar();
		}
		
	}
	
	route_file << "Total cost = " << total_cost << endl;
	route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	route_file << "========================================================================== " << endl;
	////=================================================update cost of removing=====================================//////////////////////
	if (r1 != r2) //if diferent routes
	{
		for (int g = 0; g < 2; g++) //find cost of removing for each customer and put in array
		{
			int f;
	
			if (g == 0)
			{
				f = r1;
			}
			else
			{
				f = r2;				
			}

	
			updateCostofRemoving2 (f, cost_of_removing);

			cout<<"Insert 2-1 diff R"<<endl;
			//int status=comprareCost (cost_of_removing, f);//to check 
			//if (status == 1)
			//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][5]<<' '<<' '<<kGain[number][6]<<' '<<' '<<kGain[number][7]<<endl;
		}
	}
	
	else // if the same route
	{
		int f = r1;

	
		updateCostofRemoving2 (f, cost_of_removing);

		//for (int h = 1; h < saiz[f] + 1; h++) //first customer start from element [1]
		//{
		//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
		//}
		cout<<"Insert 2-1 same R"<<endl;
		//int status=comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][5]<<endl;
	}
	delete[] tempFlag;
}


//insert_2_0 (same route based on modified route, diff based on nothing
//DONE
void insert_2_0_2nd(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute) //from is fromroute 
{
	float total_cost = 0.0;
	//float r2_cost = 0.0;
	int old_pos = -1;
	int new_pos = -1;
	int ele1 = -1, ele2 = -1;
	int from = -1;
	int to = -1;
	float gain = kGain[number][1];
	bool *tempFlag = new bool[SIZE+1];
	for (int i = 0; i <= SIZE; i++)
	{
		tempFlag[i] = true;//false noneed to find, true need to find
	}
	int reverseStatus;


	if (sameroute == true)//if it is from the same route //////////////////////// POSITION OF INSERTION IS FOR MODIFIED ROUTE !!!!!!!!!!!!!!! ////////////////////////////
	{
		int route = kGain[number][2];
		old_pos = kGain[number][4];
		new_pos = kGain[number][5];  //this position is based on modified route!!!!!!!!, assumed one has been deleted
		ele1 = VEHICLE[route][old_pos];
		ele2 = VEHICLE[route][old_pos+1];
		from = route;
		to = route;

		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=2;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
	
		//r1 
		for (int i = -1; i <= delete1; i++)//flag the one before, current1&2, and one after
		{
			tempFlag[VEHICLE[route][old_pos+i]] = true;//flagged customer affected true
		}

		//gain = kGain[number][1];
		reverseStatus = kGain[number][9];
		if (reverseStatus == 1)
		{
			int temp = ele1;
			ele1 = ele2;
			ele2 = temp;
		}
		if (ele1 == SIZE || ele1 < 0 || ele2 == SIZE || ele2 < 0 )
			getchar();
		int* temp_r = new int[saiz[route]+2];

		//copy route without ele
		int b=0;
		for (int a = 0; a < saiz[route] + 2; a++)
		{
			if ((a == old_pos) || (a == old_pos+1))
			{
				continue; //dont copy the element, assume the element to be deleted
			}

			else
			{
				temp_r[b] = VEHICLE[route][a];
				b++;
			}
		}

		int insert1=0;
		for (int i = -1; i <= insert1; i++)//flag the one before, and current
		{
			tempFlag[temp_r[new_pos+i]] = true;//flagged customer affected true
		}

		int i = saiz[route]-2+1; //2 has been deleted, so saiz-2. +1 so that copy the last element shift right
		///changed from new_pos+1 to new_pos on 11 Feb 2015
		while ( (i >= new_pos) && (i>=1) )
		{
			temp_r[i+2] = temp_r[i];
			i--;
		}
		i++;//changed this on 7MAC2015
		temp_r[i+1] = ele2;
		temp_r[i] = ele1;


		for (int k = 0; k < saiz[route]+2; k++)
		{
			VEHICLE[route][k] = temp_r[k];  //copy to VEHICLE matrix
		}
		distance_cost[route] = distance_cost[route] - kGain[number][11];
		//r2_cost = 0.0;//reinitialize //to check
		//for (int c = 0; c < saiz[route] + 1; c++)
		//{
		//	r2_cost = r2_cost + dist[VEHICLE[route][c]][VEHICLE[route][c + 1]] + service_time[VEHICLE[route][c]];
		//}
		//route_cost[route] = r1_cost; 
		route_cost[route] = route_cost[route] - gain;
		//if (abs(route_cost[route]-r2_cost)> 0.1)
		//{
		//	cout<<"2.In insert 2-0"<<endl;
		//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
		//	cout<< route_cost[route]<<' '<<r2_cost<<endl;
		//	getchar();
		//}
		distance_available[route] = DISTANCE - distance_cost[route];
		//noneed to update total_demand[] as it is still the same
		//update CumDist starting from the affecting pos until last cust
		int start = old_pos;//the start of update point, always hold old or new hold whichever comes first
		if (new_pos < old_pos)
			start = new_pos;
		for (int j = start; j <=saiz[route]; j++)
		{
			CumDist[route][j] = CumDist[route][j-1]+dist[VEHICLE[route][j-1]][VEHICLE[route][j]] + service_time[VEHICLE[route][j]];
		}
		delete[] temp_r;
		
	}
	//based on nothing
	else if (sameroute == false) //if it is from different route
	{
		old_pos = kGain[number][4];
		new_pos = kGain[number][5];
		ele1 = VEHICLE[(int)kGain[number][2]][old_pos];
		ele2 = VEHICLE[(int)kGain[number][2]][old_pos+1];
		if (ele1 == SIZE || ele1 < 0 || ele2 == SIZE || ele2 < 0 )
			getchar();
		from = kGain[number][2];
		to = kGain[number][3];

		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=2;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		int insert1=0;
		//r1 
		for (int i = -1; i <= delete1; i++)//flag the one before, current1&2, and one after
		{
			tempFlag[VEHICLE[from][old_pos+i]] = true;//flagged customer affected true
		}

		//r2
		for (int i = -1; i <= insert1; i++)//flag the one before and current
		{
			tempFlag[VEHICLE[to][new_pos+i]] = true;//flagged customer affected true
		}

		//gain = kGain[number][1];
		reverseStatus = kGain[number][9];
		if (reverseStatus == 1)
		{
			int temp = ele1;
			ele1 = ele2;
			ele2 = temp;
		}

		//r1 (delete 2)	
		for (int i = old_pos; i<=saiz[from]-1; i++)// cust 0-4, can delete 5 customers
		{
			VEHICLE[from][i] = VEHICLE[from][i+2];
		}
		VEHICLE[from][saiz[from]] = -1;//optional
		VEHICLE[from][saiz[from]] = -1;//optional
		saiz[from] = saiz[from] - 2;
		float new_sum_cost = route_cost[from] + route_cost[to] - gain;
		distance_cost[from] = distance_cost[from] - kGain[number][11];
		//route_cost[from]=0.0, route_cost[to]=0.0;	//reinitialize
		//for (int c = 0; c < saiz[from] + 1; c++)
		//{
		//	route_cost[from] += dist[VEHICLE[from][c]][VEHICLE[from][c + 1]]+ service_time[VEHICLE[from][c]];
		//}
		route_cost[from] = route_cost[from] - kGain[number][10];////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		total_demand[from] = total_demand[from] - demand[ele1] - demand[ele2];
		distance_available[from] = DISTANCE - distance_cost[from];
		space_available[from] = CAPACITY - total_demand[from];
		//update CumDist starting from the affecting pos until last cust
		int start = old_pos;//the start of update point
		for (int j = start; j <=saiz[from]; j++)
		{
			CumDist[from][j] = CumDist[from][j-1]+dist[VEHICLE[from][j-1]][VEHICLE[from][j]] + service_time[VEHICLE[from][j]];
		}

		//r2 (insert 2)	
		int j = saiz[to]+1; 

		while ( j >= new_pos && j>0 )
		{
			VEHICLE[to][j+2] = VEHICLE[to][j];
			j--;
		}	
		VEHICLE[to][new_pos] = ele1;
		VEHICLE[to][new_pos+1] = ele2;
		saiz[to] = saiz[to] + 2; //size of new route is plus 2
		distance_cost[to] = distance_cost[to] - kGain[number][12];
		route_cost[to] = new_sum_cost - route_cost[from];
		total_demand[to] = total_demand[to] + demand[ele1] + demand[ele2];
		distance_available[to] = DISTANCE - distance_cost[to];
		space_available[to] = CAPACITY - total_demand[to];
		//update CumDist starting from the affecting pos until last cust
		int start2 = new_pos;//the start of update point
		for (int j = start2; j <=saiz[to]; j++)
		{
			CumDist[to][j] = CumDist[to][j-1]+dist[VEHICLE[to][j-1]][VEHICLE[to][j]] + service_time[VEHICLE[to][j]];
		}
		

	}

	ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	route_file << ' ' << endl;
	//======================================================Display route========================================================================
	route_file << "==================From insert (2-0)===================================== " << endl;
	route_file << "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g]==0)
		{
			route_cost[g]=0;	
		}
		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		for (int h = 0; h < saiz[g] + 2; h++)
		{
			route_file << VEHICLE[g][h] << ' ';
		}
		route_file <<endl;
		total_cost = total_cost + route_cost[g];
		if (route_cost[g] < 0)
		{
			cout<< "in insert 2_0,  route_cost["<<g<<"] is negative!"<<endl;
			getchar();
		}
	}
	route_file << "Total cost = " << total_cost << endl;
	route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	route_file << "========================================================================== " << endl;
	//////=================================================update cost of removing=====================================//////////////////////
	if (from != to) //if diferent routes
	{
		for (int g = 0; g < 2; g++) //find cost of removing for each customer and put in array
		{
			int f;
	
			if (g == 0)
			{
				f = from;
			}
			else
			{
				f = to;		
			}

	
			updateCostofRemoving2 (f, cost_of_removing);
	
			cout<<"Insert 2-0 diff R"<<endl;
			//int status=comprareCost (cost_of_removing, f);//to check 
			//if (status == 1)
			//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][5]<<endl;
		}
	}

	else // if the same route
	{
		int f = from;
		
	
		updateCostofRemoving2 (f, cost_of_removing);
	

		
		//for (int h = 1; h < saiz[f] + 1; h++) //first customer start from element [1]
		//{
		//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
		//}
		cout<<"Insert 2-0 same R"<<endl;
		//int status=comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][5]<<endl;
	}
	delete[] tempFlag;
}

//insert_2_2_swap (same route based on non-modified route, diff based on non-modified route
//DONE
void insert_2_2_swap2nd(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE), bool sameroute)
{
	float total_cost = 0.0;
	int r1 = -1;
	int r2 = -1;
	float gain = kGain[number][1];
	bool *tempFlag = new bool[SIZE+1];
	for (int i = 0; i <= SIZE; i++)
	{
		tempFlag[i] = true;//false noneed to find, true need to find
	}
	int reverseStatus =0;
	//float r2_cost=0.0;

	/////////// ================  ALL CUSTOMER POSITION IN NON MODIFIED ROUTE!!!!!!!!!!!!!!!!!! =====================///////////////
	int pos1=-1, pos2=-1, pos3=-1, pos4=-1, ele1=-1, ele2=-1, ele3=-1, ele4=-1;
	//gain = kGain[number][1];
	pos1 = kGain[number][4]; //kGain[number][4] = kGain[number][7] because this is swap
	pos2 = pos1+1;
	pos3 = kGain[number][6];//kGain[number][5] = kGain[number][6] because this is swap
	pos4 = pos3+1;
	reverseStatus = kGain[number][9];

	if(sameroute == true) //if it is the same route, the position is to swap
	{
		r1 = r2 = kGain[number][2];
	}
	else //if different route
	{
		r1 = kGain[number][2];//from r1
		r2 = kGain[number][3];//to r2
	}
	ele1 = VEHICLE[r1][pos1];
	ele2 = VEHICLE[r1][pos2];
	ele3 = VEHICLE[r2][pos3];
	ele4 = VEHICLE[r2][pos4];

	if (reverseStatus == 1)//ele1 and ele2 reverse
	{
		int temp = ele1;
		ele1 = ele2;
		ele2 = temp;
	}
	else if (reverseStatus == 2)//ele3 and ele4 reverse
	{
		int temp = ele3;
		ele3 = ele4;
		ele4 = temp;
	}
	else if (reverseStatus == 3)///ele1 and ele2 reverse, ele3 and ele4 reverse
	{
		int temp = ele1;
		ele1 = ele2;
		ele2 = temp;
			
		temp = ele3;
		ele3 = ele4;
		ele4 = temp;
	}
		
	if (sameroute == true) //if it is the same route, swap for one route
	{
		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=2;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		
		for (int i = -1; i <= delete1; i++)//flag the one before, current1&2, and one after
		{
			tempFlag[VEHICLE[r1][pos1+i]] = true;//flagged customer affected true
		}

		for (int i = -1; i <= delete1; i++)//flag the one before, current1&2, and one after
		{
			tempFlag[VEHICLE[r1][pos3+i]] = true;//flagged customer affected true
		}

		VEHICLE[r1][pos1]= ele3;
		VEHICLE[r1][pos2]= ele4;
		VEHICLE[r1][pos3]= ele1;
		VEHICLE[r1][pos4]= ele2;
		distance_cost[r1] = distance_cost[r1] - kGain[number][11];
		//r2_cost=0.0; //to check
		//for (int c = 0; c < saiz[r1] + 1; c++)
		//{
		//	r2_cost = r2_cost + dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]]  + service_time[VEHICLE[r1][c]];
		//}
		//route_cost[r1] = r1_cost;
		route_cost[r1] = route_cost[r1] - gain; //////////////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		//if (abs(route_cost[r1]-r2_cost)> 0.1)
		//{
		//	cout<<"1.In insert 2-2swap"<<endl;
		//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
		//	cout<< route_cost[r1]<<' '<<r2_cost<<endl;
		//	getchar();
		//}
		distance_available[r1] = DISTANCE - distance_cost[r1];
		//update CumDist starting from the affecting pos until last cust
		int start = pos1;//the start of update point, always hold old or new hold whichever comes first
		if (pos3 < pos1)
			start = pos3;
		for (int j = start; j <=saiz[r1]; j++)
		{
			CumDist[r1][j] = CumDist[r1][j-1]+dist[VEHICLE[r1][j-1]][VEHICLE[r1][j]] + service_time[VEHICLE[r1][j]];
		}
	}
		
	else //different route
	{
		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=2;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		
		for (int i = -1; i <= delete1; i++)//flag the one before, current1&2, and one after
		{
			tempFlag[VEHICLE[r1][pos1+i]] = true;//flagged customer affected true
		}

		for (int i = -1; i <= delete1; i++)//flag the one before, current1&2, and one after
		{
			tempFlag[VEHICLE[r2][pos3+i]] = true;//flagged customer affected true
		}

		VEHICLE[r1][pos1]= ele3;
		VEHICLE[r1][pos2]= ele4;
		VEHICLE[r2][pos3]= ele1;
		VEHICLE[r2][pos4]= ele2;

		float new_sum_cost = route_cost[r1] + route_cost[r2] - gain;
		distance_cost[r1] = distance_cost[r1] - kGain[number][11];
		//route_cost[r1]=0.0, route_cost[r2]=0.0;	//reinitialize
		//for (int c = 0; c < saiz[r1] + 1; c++)
		//{
		//	route_cost[r1] += dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]]+ service_time[VEHICLE[r1][c]];
		//}
		route_cost[r1] = route_cost[r1] - kGain[number][10];////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		total_demand[r1] = total_demand[r1] - demand[ele1] - demand[ele2] + demand[ele3] + demand[ele4];
		distance_available[r1] = DISTANCE - distance_cost[r1];
		space_available[r1] = CAPACITY - total_demand[r1];
		distance_cost[r2] = distance_cost[r2] - kGain[number][12];
		route_cost[r2] = new_sum_cost - route_cost[r1];
		//update CumDist starting from the affecting pos until last cust
		int start = pos1;//the start of update point, always hold old or new hold whichever comes first
		for (int j = start; j <=saiz[r1]; j++)
		{
			CumDist[r1][j] = CumDist[r1][j-1]+dist[VEHICLE[r1][j-1]][VEHICLE[r1][j]] + service_time[VEHICLE[r1][j]];
		}

		total_demand[r2] = total_demand[r2] - demand[ele3] - demand[ele4] + demand[ele1] + demand[ele2];
		distance_available[r2] = DISTANCE - distance_cost[r2];
		space_available[r2] = CAPACITY - total_demand[r2];
		//update CumDist starting from the affecting pos until last cust
		int start2 = pos3;//the start of update point, always hold old or new hold whichever comes first
		for (int j = start2; j <=saiz[r2]; j++)
		{
			CumDist[r2][j] = CumDist[r2][j-1]+dist[VEHICLE[r2][j-1]][VEHICLE[r2][j]] + service_time[VEHICLE[r2][j]];
		}
	}//end of else (different route)

	ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);
	
	route_file << ' ' << endl;
	//======================================================Display route========================================================================
	
	route_file << "==================From insert (2-2swap)===================================== " << endl;
	route_file << "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g]==0)
		{
			route_cost[g]=0;
		}

		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		for (int h = 0; h < saiz[g] + 2; h++)
		{
			route_file << VEHICLE[g][h] << ' ';	
		}
		route_file <<endl;

		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0 || distance_cost[g] > DISTANCE+epsilon)
		{
			cout<< "in insert 2_2swap,  route_cost["<<g<<"] is negative or violated!"<<endl;
			getchar();
		}
		
	}
	route_file << "Total cost = " << total_cost << endl;
	route_file << "========================================================================== " << endl;

	//////=================================================update cost of removing=====================================//////////////////////
	if (r1 != r2) //if diferent routes
	{
		for (int g = 0; g < 2; g++) //find cost of removing for each customer and put in array
		{
			int f;
		
			if (g == 0)
			{
				f = r1;
			}
			else
			{
				f = r2;
			}
			
	
			updateCostofRemoving2 (f, cost_of_removing);
		
			
			//for (int h = 1; h < saiz[f] + 1; h++) //first customer start from element [1]
			//{
			//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]]- dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
			//}
			cout<<"Insert 2-2 diff R"<<endl;//to check 
			//int status=comprareCost (cost_of_removing, f);//to check 
			//if (status == 1)
			//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][6]<<endl;
		}
	}
	
	else // if the same route
	{
		int f = r1;

		
	
		updateCostofRemoving2 (f, cost_of_removing);
	

		//for (int h = 1; h < saiz[f] + 1; h++) //first customer start from element [1]
		//{
		//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
		//}	
		cout<<"Insert 2-2 same R"<<endl;
		//int status=comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][5]<<endl;
	}
	delete[] tempFlag;
}

//DONE
void insert_2optintra(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE))
{
	bool *tempFlag = new bool[SIZE+1];
	for (int i = 0; i <= SIZE; i++)
	{
		tempFlag[i] = true;//false noneed to find, true need to find
	}
	std::vector<int> subvector; //for reverse order
	int num_c_in_subroute=0;
	//float ori_cost=0.0, temp_cost=0.0;

	int i = kGain[number][2];//route number
	int m = kGain[number][4]; //eleStart
	int n = kGain[number][5]; //eleEnd
	float gain = kGain[number][1];//gain

	//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
	int delete1=0;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		
	for (int q = -1; q <= delete1; q++)//flag the one before and current
	{
		tempFlag[VEHICLE[i][m+q]] = true;//flagged customer affected true
	}

	int delete2=1;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		
	for (int q = 0; q <= delete2; q++)//flag the current and after
	{
		tempFlag[VEHICLE[i][n+q]] = true;//flagged customer affected true
	}
	//copy the subroute in subvector
	for (int z = n; z >= m; z--)
	{
		subvector.push_back(VEHICLE[i][z]);
		num_c_in_subroute++;	
	}
	//ori_cost = dist[VEHICLE[i][m-1]][subvector[0]] + dist[VEHICLE[i][n+1]][subvector[num_c_in_subroute-1]]; //only compare two ends cost, the intermediate all the same

	//std::reverse(subvector.begin(),subvector.end());    // 9 8 7 6 5 4 3 2 1
	
	//calculate temp cost, only change at 2 ends
	//temp_cost = dist[VEHICLE[i][m-1]][subvector[0]] + dist[VEHICLE[i][n+1]][subvector[num_c_in_subroute-1]];//only compare two ends cost, the intermediate all the same
					

	int s=0;//for subvector
	for (unsigned z = m; z <= n; z++)
	{
		VEHICLE[i][z] = subvector[s];  //copy to VEHICLE 
		s++;
	}
	distance_cost[i] = distance_cost[i] - kGain[number][11];
	//float r_cost=route_cost[i] ; //to check
	//r_cost = r_cost - (ori_cost-temp_cost);  //overwrite the cost //check rounding error	
	route_cost[i] = route_cost[i] - gain;//if gain is correctly found, this one should be correct!!!
	//if (abs(route_cost[i]-r_cost)> 0.1)
	//{
	//	cout<<"1.In insert 2optintra"<<endl;
	//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	//	cout<< route_cost[i]<<' '<<r_cost<<endl;
	//	getchar();
	//}

	//CUmDIst must be updated before update cost_ofremoving because cost_pf removing need to use Cumdist!!!!!!!!! before this put after update cost_ofremoving//changed this on 07/06/2016
	int start = m;//the start of update point
	for (int j = start; j <=saiz[i]; j++)
	{
		CumDist[i][j] = CumDist[i][j-1]+dist[VEHICLE[i][j-1]][VEHICLE[i][j]] + service_time[VEHICLE[i][j]];
	}

	ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	route_file << ' ' << endl;
	//======================================================Display route========================================================================
	route_file << "==================From insert 2optintra============================== " << endl;
	route_file << "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	float total_cost=0;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		for (int h = 0; h <= saiz[g] + 1; h++)
		{
			route_file << VEHICLE[g][h] << ' ';
		}
		route_file<<endl;
		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
		
	}
	route_file << "Total cost = " << total_cost << endl;
	route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	route_file << "========================================================================== " << endl;
	
	
	
	//update cost_of_removing for one single route
	updateCostofRemoving2 (i, cost_of_removing);
	


	cout<<"Insert 2-opt intra"<<endl;
	//int status=comprareCost (cost_of_removing, i);//to check 
	//if (status == 1)
	//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][5]<<endl;
	//update CumDist starting from the affecting pos until last cust

	delete[] tempFlag;
}

//DONE
void insert_2optinter(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE))
{
	ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);	
	//======================================================Display route========================================================================
	route_file << "==================From insert 2optinter (BEFORE!!!!!!!!)============================== " << endl;
	
	
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		for (int h = 0; h <= saiz[g] + 1; h++)
		{
			route_file << VEHICLE[g][h] << ' ';
		}
		route_file<<endl;
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
		
	}

	route_file << "========================================================================== " << endl;
	
	bool *tempFlag = new bool[SIZE+1];
	for (int i = 0; i <= SIZE; i++)
	{
		tempFlag[i] = true;//false noneed to find, true need to find
	}
	std::vector<int> myvector1;
	std::vector<int> myvector2; 
	int i = kGain[number][2];//r1
	int m = kGain[number][3];//r2
	int j = kGain[number][4]; //eleStart
	int n = kGain[number][5]; //eleEnd
	float gain = kGain[number][1];//gain

	//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
	int delete1=0;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		
	for (int q = -1; q <= delete1; q++)//flag the one before and current
	{
		tempFlag[VEHICLE[i][j+q]] = true;//flagged customer affected true
	}
	tempFlag[VEHICLE[i][saiz[i]]] = true;//flagged the last one in r1 as true

	int delete2=1;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
	for (int q = 0; q <= delete2; q++)//flag the one current and after
	{
		tempFlag[VEHICLE[m][n+q]] = true;//flagged customer affected true
	}
	tempFlag[VEHICLE[m][1]] = true;//flagged the first one in r2 as true

	int r1Size=0 ;//number of customer consider in r1
	int r2Size=0 ;//number of customer considered in r2
	int r1demandDelete=0;
	int r2demandDelete=0;
								
	for (int z = saiz[i]; z >= j; z--)//copy segment from route1 in reverse order
	{
		myvector1.push_back(VEHICLE[i][z]); //route1 is from j to end
		r1demandDelete = r1demandDelete + demand[VEHICLE[i][z]];//consider they are to be deleted from r1
		r1Size++;//number of customer consider in r1
	}

	for (int z = n; z >= 1; z--)//copy segment from route2 in reverse order
	{
		myvector2.push_back(VEHICLE[m][z]); //route2 from 1 to n
		r2demandDelete = r2demandDelete + demand[VEHICLE[m][z]];//consider they are to be deleted from r2
		r2Size++;//number of customer consider in r2
	}
	
	int w = j; //start of route1
	for (int z = 0; z < myvector2.size(); z++)//insert segment2 to route1
	{
		VEHICLE[i][w] = myvector2[z];
		w++;
	}

	std::vector<int> tempR2; //to hold second part of route2 from [n+1] to saiz[m] because route 2 is to replace the front part until j, but the size maybe more than original because it is based on how many deleted from r1
	for (int e = n+1; e <= saiz[m]; e++)//push the rest starting from [n+1] until the end depot, saiz[] havent updated yet
	{
		tempR2.push_back(VEHICLE[m][e]);
	}

	saiz[i] = saiz[i] - r1Size + r2Size; //update size because the size has changed
	saiz[m] = saiz[m] - r2Size + r1Size;
	
	w = 1; //start of route2
	for (int z = 0; z < myvector1.size(); z++)//insert segment1 to route2
	{
		VEHICLE[m][w] = myvector1[z];
		w++;
	}
	//copy the remaining part of route2 (unchanged part)
	int t=0;//for tempR2
	for (int t = 0; t < tempR2.size(); t++)//copy the unchanged part to route2
	{
		VEHICLE[m][w] = tempR2[t];//w continue from previous one
		w++;
	}
	//put the depot as the last element
	VEHICLE[i][saiz[i]+1] = SIZE;
	VEHICLE[m][saiz[m]+1] = SIZE;//already put the depot hen copy from tempR2

	float new_sumcost = route_cost[i] + route_cost[m] - gain;

	distance_cost[i] = distance_cost[i] - kGain[number][11];
	//for (int t = 0; t <= saiz[i]+1; t++)
	//{
	//	cout<<VEHICLE[i][t]<<' ';
	//}
	//cout<<endl;
	//for (int t = 0; t <= saiz[m]+1; t++)
	//{
	//	cout<<VEHICLE[m][t]<<' ';
	//}
	//route_cost[i] = route_cost[m] = 0.0;//reinitialize
	//for (int c = 0; c <= saiz[i]; c++)
	//{
	//	route_cost[i] += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
	//}
	route_cost[i] = route_cost[i]-kGain[number][10];

	distance_cost[m] = distance_cost[m] - kGain[number][12];
	//float r2_cost=0.0;//to check
	//for (int c = 0; c <= saiz[m]; c++)
	//{
	//	r2_cost += dist[VEHICLE[m][c]][VEHICLE[m][c + 1]] + service_time[VEHICLE[m][c]];
	//}
	route_cost[m] = new_sumcost - route_cost[i]; //avoid calculate the second route cost, just use gain
	//if (abs(route_cost[m]-r2_cost)> 0.1)
	//{
	//	cout<<"1.In insert 2-optinter"<<endl;
	//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	//	cout<< route_cost[m]<<' '<<r2_cost<<endl;
	//	getchar();
	//}
	total_demand[i] = total_demand[i] - r1demandDelete + r2demandDelete; //check if this is correct
	total_demand[m] = total_demand[m] - r2demandDelete + r1demandDelete; //check if this is correct
	space_available[i] = CAPACITY - total_demand[i];
	space_available[m] = CAPACITY - total_demand[m];
	distance_available[i] = DISTANCE - distance_cost[i];
	distance_available[m] = DISTANCE - distance_cost[m];
	//update CumDist starting from the affecting pos until last cust
	int start = j;//the start of update point
	for (int q = start; q <=saiz[i]; q++)
	{
		CumDist[i][q] = CumDist[i][q-1]+dist[VEHICLE[i][q-1]][VEHICLE[i][q]] + service_time[VEHICLE[i][q]];
	}

	//update CumDist starting from the affecting pos until last cust
	int start2 = 1;//the start of update point
	for (int q = start2; q <=saiz[m]; q++)
	{
		CumDist[m][q] = CumDist[m][q-1]+dist[VEHICLE[m][q-1]][VEHICLE[m][q]] + service_time[VEHICLE[m][q]];
	}
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	route_file << ' ' << endl;
	//======================================================Display route========================================================================
	route_file << "==================From insert 2optinter============================== " << endl;
	route_file << "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	float total_cost=0;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		for (int h = 0; h <= saiz[g] + 1; h++)
		{
			route_file << VEHICLE[g][h] << ' ';
		}
		route_file<<endl;
		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
		
	}
	route_file << "Total cost = " << total_cost << endl;
	route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	route_file << "========================================================================== " << endl;

	//////=================================================update cost of removing=====================================//////////////////////
	for (int g = 0; g < 2; g++) //find cost of removing for each customer and put in array
	{
		int f;
	
		if (g == 0)
		{
			f = i; //r1
		}
		else
		{
			f = m;//r2		
		}

	
		updateCostofRemoving2 (f, cost_of_removing);

		cout<<"Insert 2-opt inter"<<endl;
		//int status=comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<i<<' '<<' '<<m<<' '<<' '<<j<<' '<<' '<<n<<' '<<' '<<saiz[i]<<' '<<' '<<saiz[m]<<endl;
	}
	delete[] tempFlag;
}

//DONE
void insert_crosstail(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE))
{
	bool *tempFlag = new bool[SIZE+1];
	for (int i = 0; i <= SIZE; i++)
	{
		tempFlag[i] = true;//false noneed to find, true need to find
	}
	std::vector<int> myvector1;
	std::vector<int> myvector2; 
	int i = kGain[number][2];//r1
	int m = kGain[number][3];//r2
	int j = kGain[number][4]; //eleStart in r1
	int n = kGain[number][5]; //eleStart in r2
	float gain = kGain[number][1];//gain
	int reverseType = kGain[number][9];//reverseType


	//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
	int delete1=0;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		
	for (int q = -1; q <= delete1; q++)//flag the one before and current
	{
		tempFlag[VEHICLE[i][j+q]] = true;//flagged customer affected true
	}
	tempFlag[VEHICLE[i][saiz[i]]] = true;//flagged the last one in r1 as true

	for (int q = -1; q <= delete1; q++)//flag the one before and current
	{
		tempFlag[VEHICLE[m][n+q]] = true;//flagged customer affected true
	}
	tempFlag[VEHICLE[m][saiz[m]]] = true;//flagged the last one in r2 as true

	int tail1Size=0 ;//number of customer consider in r1
	int tail2Size=0 ;//number of customer considered in r2
	int r1demandDelete=0;
	int r2demandDelete=0;
	
	for (int z = j; z <= saiz[i]; z++)//copy segment from route1 
	{
		myvector1.push_back(VEHICLE[i][z]); //route1 is from j to end
		r1demandDelete = r1demandDelete + demand[VEHICLE[i][z]];//consider they are to be deleted from r1
		tail1Size++;//number of customer consider in r1
	}
	
	for (int z = n; z <= saiz[m]; z++)//copy segment from route2 in reverse order
	{
		myvector2.push_back(VEHICLE[m][z]); //route2 from 1 to n
		r2demandDelete = r2demandDelete + demand[VEHICLE[m][z]];//consider they are to be deleted from r2
		tail2Size++;//number of customer consider in r2
	}
	
	if (reverseType == 3)//both reverse
	{
		std::reverse(myvector1.begin(),myvector1.end());    // 9 8 7 6 5 4 3 2 1
		std::reverse(myvector2.begin(),myvector2.end());    // 9 8 7 6 5 4 3 2 1
	}
	else if (reverseType == 1)
		std::reverse(myvector1.begin(),myvector1.end());    //TRICK!!!!! //tail1 reverse
	else if (reverseType == 2)
		std::reverse(myvector2.begin(),myvector2.end());    //TRICK!!!!!!//tail2 reverse
	
	int w = j; //start of route1
	for (int z = 0; z < myvector2.size(); z++)//insert segment2 to route1
	{
		VEHICLE[i][w] = myvector2[z];
		w++;
	}

	w = n; //start of route2
	for (int z = 0; z < myvector1.size(); z++)//insert segment1 to route2
	{
		VEHICLE[m][w] = myvector1[z];
		w++;
	}

	saiz[i] = saiz[i] - tail1Size + tail2Size; //update size because the size has changed
	saiz[m] = saiz[m] - tail2Size + tail1Size;
	
	//put the depot as the last element
	VEHICLE[i][saiz[i]+1] = SIZE;
	VEHICLE[m][saiz[m]+1] = SIZE;

	float new_sumcost = route_cost[i] + route_cost[m] - gain;
	distance_cost[i] = distance_cost[i] - kGain[number][11];
	//route_cost[i] = route_cost[m] = 0.0;//reinitialize
	//for (int c = 0; c <= saiz[i]; c++)
	//{
	//	route_cost[i] += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
	//}
	route_cost[i] = route_cost[i]-kGain[number][10];

	distance_cost[m] = distance_cost[m] - kGain[number][12];
	//float r2_cost=0.0;//to check
	//for (int c = 0; c <= saiz[m]; c++)
	//{
	//	r2_cost += dist[VEHICLE[m][c]][VEHICLE[m][c + 1]] + service_time[VEHICLE[m][c]];
	//}
	route_cost[m] = new_sumcost - route_cost[i]; //avoid calculate the second route cost, just use gain
	//if (abs(route_cost[m]-r2_cost)> 0.1)
	//{
	//	cout<<"1.In insert crosstail"<<endl;
	//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	//	cout<< route_cost[m]<<' '<<r2_cost<<endl;
	//	getchar();
	//}
	total_demand[i] = total_demand[i] - r1demandDelete + r2demandDelete; //check if this is correct
	total_demand[m] = total_demand[m] - r2demandDelete + r1demandDelete; //check if this is correct
	space_available[i] = CAPACITY - total_demand[i];
	space_available[m] = CAPACITY - total_demand[m];
	distance_available[i] = DISTANCE - distance_cost[i];
	distance_available[m] = DISTANCE - distance_cost[m];
	//update CumDist starting from the affecting pos until last cust
	int start = j;//the start of update point
	for (int q = start; q <=saiz[i]; q++)
	{
		CumDist[i][q] = CumDist[i][q-1]+dist[VEHICLE[i][q-1]][VEHICLE[i][q]] + service_time[VEHICLE[i][q]];
	}
	//update CumDist starting from the affecting pos until last cust
	int start2 = n;//the start of update point
	for (int q = start2; q <=saiz[m]; q++)
	{
		CumDist[m][q] = CumDist[m][q-1]+dist[VEHICLE[m][q-1]][VEHICLE[m][q]] + service_time[VEHICLE[m][q]];
	}

	ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	route_file << ' ' << endl;
	//======================================================Display route========================================================================
	route_file << "==================From insert crosstail============================== " << endl;
	route_file << "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	float total_cost=0;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		for (int h = 0; h <= saiz[g] + 1; h++)
		{
			route_file << VEHICLE[g][h] << ' ';
		}
		route_file<<endl;
		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
		
	}
	route_file << "Total cost = " << total_cost << endl;
	route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	route_file << "========================================================================== " << endl;

	//////=================================================update cost of removing=====================================//////////////////////
	for (int g = 0; g < 2; g++) //find cost of removing for each customer and put in array
	{
		int f;
	
		if (g == 0)
		{
			f = i; //r1
		}
		else
		{
			f = m;//r2
		}

	
		updateCostofRemoving2 (f, cost_of_removing);
		
		cout<<"Insert crosstail"<<endl;
		//int status = comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][5]<<endl;
	}
	delete[] tempFlag;
}

void insert_cross(float *(&cost_of_removing), int k, int number, float **kGain, int **(&VEHICLE))
{
	bool *tempFlag = new bool[SIZE+1];
	for (int i = 0; i <= SIZE; i++)
	{
		tempFlag[i] = true;//false noneed to find, true need to find
	}
	std::vector<int> myvector1;
	std::vector<int> myvector2; 
	int i = kGain[number][2];//r1
	int m = kGain[number][3];//r2
	int j = kGain[number][4]; //eleStart in r1
	int n = kGain[number][5]; //eleStart in r2
	float gain = kGain[number][1];//gain
	int reverseType = kGain[number][9];//reverseType
	int L1 = 3;//length of segment1
	int L2 = 2;//length of segment2

	//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
	int delete1=0;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		
	for (int q = -1; q <= delete1; q++)//flag the one before and current
	{
		tempFlag[VEHICLE[i][j+q]] = true;//flagged customer affected true
	}
	for (int q = -1; q <= delete1; q++)//flag the one before and current
	{
		tempFlag[VEHICLE[m][n+q]] = true;//flagged customer affected true
	}
	int last1=1;
	for (int q = 0; q <= last1; q++)//flag the current last and one after for the end point of L1
	{
		tempFlag[VEHICLE[i][j+L1-1+q]] = true;//flagged customer affected true
	}

	for (int q = 0; q <= last1; q++)//flag the current last and one after for the end point of L2
	{
		tempFlag[VEHICLE[m][n+L2-1+q]] = true;//flagged customer affected true
	}

	
	int seg1Size=0 ;//number of customer consider in r1
	int seg2Size=0 ;//number of customer considered in r2
	int r1demandDelete=0;
	int r2demandDelete=0;
	
	for (int z = j; z <= j+L1-1; z++)//copy segment from route1 
	{
		myvector1.push_back(VEHICLE[i][z]); //route1 is from j to end
		r1demandDelete = r1demandDelete + demand[VEHICLE[i][z]];//consider they are to be deleted from r1
		seg1Size++;//number of customer consider in r1
	}
	
	for (int z = n; z <= n+L2-1; z++)//copy segment from route2 
	{
		myvector2.push_back(VEHICLE[m][z]); //route2 from 1 to n
		r2demandDelete = r2demandDelete + demand[VEHICLE[m][z]];//consider they are to be deleted from r2
		seg2Size++;//number of customer consider in r2
	}
	
	if (reverseType == 3)//both reverse
	{
		std::reverse(myvector1.begin(),myvector1.end());    // 9 8 7 6 5 4 3 2 1
		std::reverse(myvector2.begin(),myvector2.end());    // 9 8 7 6 5 4 3 2 1
	}
	else if (reverseType == 1)
		std::reverse(myvector1.begin(),myvector1.end());    // 9 8 7 6 5 4 3 2 1
	else if (reverseType == 2)
		std::reverse(myvector2.begin(),myvector2.end());    // 9 8 7 6 5 4 3 2 1

	if (L1 == L2) //if they are the same length, just replace, no changes of saiz[]
	{
		//copy segment2 to route1
		int v=0;//for segment2[]
		for (int k = j; k < L2+j; k++)
		{
			VEHICLE[i][k] = myvector2[v];
			v++;
		}
								

		//copy segment1 to route2
		v=0;//for segment1[]
		for (int k = n; k < L1+n; k++)
		{
			VEHICLE[m][k] = myvector1[v];
			v++;
		}

	}

	else//if L1 and L2 not same length
	{
		std::vector<int> temp1;
		int partialSize1=0;
		//copy the partial route after segment1 including the last one depot
		for (int k = j+L1; k <= saiz[i]+1; k++)
		{
			temp1.push_back(VEHICLE[i][k]);
			partialSize1++;
		}
		//copy segment2 to route1
		int v=0;//for segment2[]
		for (int k = j; k < L2+j; k++)//from j to the length of segment2
		{
			VEHICLE[i][k] = myvector2[v];
			v++;
		}
		//copy the partial route after segment 1 to VEHICLE
		v=0;//for temp[]
		for (int k = L2+j; k < partialSize1+L2+j; k++)
		{
			VEHICLE[i][k] = temp1[v];
			v++;
		}
		
		std::vector<int> temp2;
		int partialSize2=0;
		//copy the partial route after segment2 including the last one depot
		for (int k = n+L2; k <= saiz[m]+1; k++)
		{
			temp2.push_back(VEHICLE[m][k]);
			partialSize2++;
		}
		//copy segment1 to route2
		v=0;//for segment1[]
		for (int k = n; k < L1+n; k++)
		{
			VEHICLE[m][k] = myvector1[v];
			v++;
		}
		//copy the partial route after segment 1 to VEHICLE
		v=0;
		for (int k = L1+n; k <  partialSize2+L1+n; k++)
		{
			VEHICLE[m][k] = temp2[v];
			v++;
		}
		
		//update saiz[]
		saiz[i] = saiz[i] - L1 + L2;
		saiz[m] = saiz[m] - L2 + L1;
								
		temp1.clear();
		temp2.clear();
	}
	
	float new_sumcost = route_cost[i] + route_cost[m] - gain;
	distance_cost[i] = distance_cost[i] - kGain[number][11];
	//route_cost[i] = route_cost[m] = 0.0;//reinitialize
	//for (int c = 0; c <= saiz[i]; c++)
	//{
	//	route_cost[i] += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
	//}
	route_cost[i] = route_cost[i]-kGain[number][10];

	distance_cost[m] = distance_cost[m] - kGain[number][12];
	//float r2_cost=0.0;//to check
	//for (int c = 0; c <= saiz[m]; c++)
	//{
	//	r2_cost += dist[VEHICLE[m][c]][VEHICLE[m][c + 1]] + service_time[VEHICLE[m][c]];
	//}
	route_cost[m] = new_sumcost - route_cost[i]; //avoid calculate the second route cost, just use gain
	//if (abs(route_cost[m]-r2_cost)> 0.1)
	//{
	//	cout<<"1.In insert cross"<<endl;
	//	cout<< "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	//	cout<< route_cost[m]<<' '<<r2_cost<<endl;
	//	getchar();
	//}
	total_demand[i] = total_demand[i] - r1demandDelete + r2demandDelete; //check if this is correct
	total_demand[m] = total_demand[m] - r2demandDelete + r1demandDelete; //check if this is correct
	space_available[i] = CAPACITY - total_demand[i];
	space_available[m] = CAPACITY - total_demand[m];
	distance_available[i] = DISTANCE - distance_cost[i];
	distance_available[m] = DISTANCE - distance_cost[m];
	//update CumDist starting from the affecting pos until last cust
	int start = j;//the start of update point
	for (int q = start; q <=saiz[i]; q++)
	{
		CumDist[i][q] = CumDist[i][q-1]+dist[VEHICLE[i][q-1]][VEHICLE[i][q]] + service_time[VEHICLE[i][q]];
	}
	//update CumDist starting from the affecting pos until last cust
	int start2 = n;//the start of update point
	for (int q = start2; q <=saiz[m]; q++)
	{
		CumDist[m][q] = CumDist[m][q-1]+dist[VEHICLE[m][q-1]][VEHICLE[m][q]] + service_time[VEHICLE[m][q]];
	}

	ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	route_file << ' ' << endl;
	//======================================================Display route========================================================================
	route_file << "==================From insert cross============================== " << endl;
	route_file << "gain= " <<gain<< " r1= " <<kGain[number][2]<<" r2= " <<kGain[number][3]<<" r1_w/o= "<<kGain[number][4]<<" r2w= "<<kGain[number][5]<<" || r2w/o= "<<kGain[number][6]<<" r1w= "<<kGain[number][7]<<" reverse status= "<<kGain[number][9]<<endl;
	float total_cost=0;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		for (int h = 0; h <= saiz[g] + 1; h++)
		{
			route_file << VEHICLE[g][h] << ' ';
		}
		route_file<<endl;
		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
		
	}
	route_file << "Total cost = " << total_cost << endl;
	route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	route_file << "========================================================================== " << endl;

	//////=================================================update cost of removing=====================================//////////////////////
	for (int g = 0; g < 2; g++) //find cost of removing for each customer and put in array
	{
		int f;
	
		if (g == 0)
		{
			f = i; //r1
			
		}
		else
		{
			f = m;//r2

		}

	
		updateCostofRemoving2 (f, cost_of_removing);
	
		cout<<"Insert cross"<<endl;
		//int status = comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<kGain[number][4]<<' '<<' '<<kGain[number][5]<<endl;
	}
	delete[] tempFlag;
}

//DONE
int find_k_1_0(float *cost_of_removing, float **(&kGain), int K, int level) //return which one is the best out of k
{
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::app);
	
	int no_of_attribute = 13;
	for (int i = 0; i < K; i++)
	{
		for (int j = 0; j < no_of_attribute; j++)
		{
			kGain[i][j] = -1; //reinitialize
		}
	}
	int element=-1, before_ele, after_ele, before_pos, after_pos;
	float gain = 0.0;
	float cost_without_i, cost_with_i;
	//int no_routes = LOCAL_NO_ROUTE;	
	
	////// ======================== Find and Record the kth best in kGain matrix =====================//
	int set = 0;
	
	//route_file<<"Level "<<level;

	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from)
	{
		if (saiz[i]==0) //if it is empty route
			continue;

		/////////////================================== SAME ROUTE =========================////////////////////////////
		if (operand[level].eachRfullsearchstatus[i][i] == 1 || saiz[i]<=1)//if saiz=2, front and back is depot, so D 1 2 D = D 2 1 D can be considered here for CCVRP
			goto endsameroute;
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
			for (int t = 1; t <= saiz[i]-1; t++)//have 1 size less because 1 customer is deleted
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

				//just checking of cost_of_removing//
				//int remainingcust = saiz[i]-j;
				//if (remainingcust < 0)//if negative
				//	remainingcust= 0;
				//float checkcost_of_removing = CumDist[i][j] + remainingcust*origain;
				//if (abs (checkcost_of_removing-cost_without_i) > 0.5)
				//{
				//	
				//	cout<<"checking of cost of removing is wrong in find1_0 same route for customer "<<VEHICLE[i][j]<<endl;
				//	cout<<checkcost_of_removing<< ' '<<cost_without_i<<endl;
				//	getchar();
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

				
				
					
				if (gain > epsilon)//gain > 0
				{
					kGain[set][0] = set;
					kGain[set][1] = gain;
					kGain[set][2] = i; //from r1,
					kGain[set][3] = i; //to_r2
					kGain[set][4] = j; //from r1_p
					kGain[set][5] = n; //to_r2_p //to which position //assume modified route, customer has been deleted
					kGain[set][6] = -1; //from_r2_p
					kGain[set][7] = -1; //to_r1_p
					kGain[set][8] = 1; //1 is (1-0)
					kGain[set][10] = gain;//gain1
					kGain[set][11] = origain - oriloss; //gain1 in distance
					set++;
					operand[level].eachRfullsearchstatus[i][i] = 0 ;
			
				}
				if (set == K) //set starts from 0 => set=3 means 0, 1, 2 sets //if K positive are filled 
				{
					delete[] tempCumDist;
					delete[] temp_r;
					goto find_best_k1;
				}
			}//end of n
			delete[] tempCumDist;
			delete[] temp_r;
		}//end of j
	endsameroute:

		for (int m = 0; m < no_routes; m++)  //insert (to) which route
		//for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from)
		{
			if (i == m)  //if same route	
				continue;

			int foundimprovementstatus=0;//to indicate improvement found or not for this route, to flag the operand[1].eachRfullsearchstatus[i] at the end
			if (operand[level].eachRfullsearchstatus[i][m] == 1 )
			{
				//route_file<<" Omit ["<<i<<"]["<<m<<"]"<<' '<<' '<<' '<<' ';
				continue;
			}

			//====================================== different route =========================================//
			for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from)
			//for (int m = 0; m < no_routes; m++)  //insert (to) which route
			{
				element = VEHICLE[i][j];			//elemet to be inserted into other route
	
				if (demand[element] > space_available[m])  //if demand exceed avaialable space in route and different route, if same route, still need to find the best even the capacity is violated
				{
					continue;
				}

				for (int n = 1; n <= saiz[m] + 1; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
				{
					float available_dist_r2 = distance_available[m];
		
					before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[i][j + 1];
					before_pos = VEHICLE[m][n - 1];
					after_pos = VEHICLE[m][n];  //noneed to add 1 because it is insert in between n-1 and n
					
					if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
					{
						if ( (NR_FLAG_DEPOT[element][before_pos] == false && NR_FLAG_DEPOT[element][after_pos] == false) ) //use NR_FLAG_DEPOT
							continue;
					}

					else //if not between customer and depot
					{	
						if ( (NR_FLAG[element][before_pos] == false && NR_FLAG[element][after_pos] == false) ) //if insertion cannot between cust before and cust after
							continue;
					}	
					
					float origain = dist[element][before_ele] + dist[element][after_ele] + service_time[element] - dist[before_ele][after_ele];
					cost_without_i = cost_of_removing[VEHICLE[i][j]];//cost of r1 without i

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

					if (gain > epsilonn) //only record positive gain in kGain
					{
						foundimprovementstatus = 1;//once found an improvement, flag 1 
						kGain[set][0] = set;
						kGain[set][1] = gain;
						kGain[set][2] = i; //from r1,
						kGain[set][3] = m; //to_r2
						kGain[set][4] = j; //from r1_p
						kGain[set][5] = n; //to_r2_p //to which position //assume modified route, customer has been deleted
						kGain[set][6] = -1; //from_r2_p
						kGain[set][7] = -1; //to_r1_p
						kGain[set][8] = 1; //1 is (1-0)
						kGain[set][10] = cost_without_i;//gain1
						kGain[set][11] = origain;//gain1 in distance
						kGain[set][12] = -oriloss;//gain2 in distance
						set++;
						operand[level].eachRfullsearchstatus[i][m] = 0 ;
					}	

					if (set == K) //set starts from 0 => set=3 means 0, 1, 2 sets //if K positive are filled 
					{
						goto find_best_k1;
					}
				}//end for n
				//if (foundimprovementstatus == 0)//if never found improvement in this route, flag 0
				//{
				//	operand[level].eachRfullsearchstatus[i][m] = 1;
				//}
			}//end for j
			if (foundimprovementstatus == 0)//if never found improvement in this route, flag 0
			{
				operand[level].eachRfullsearchstatus[i][m] = 1;
			}
		}//end for m
	}//end for i 

find_best_k1:
	//cout<<"======= kGain in find_k_1_0() =========="<<endl;
	//for (int i = 0; i < K; i++)
	//{
	//	for (int j = 0; j < 9; j++)
	//	{
	//		cout<< kGain[i][j] << ' ' << ' ';
	//	}
	//	cout<<endl;
	//}
	int bestK=-1; //initialize
	float maxGain = INT_MIN;
	for (int i = 0; i < K; i++)
	{
		if ((kGain[i][1] > maxGain) && (kGain[i][1] >= epsilonn))
		{
			maxGain = kGain[i][1];
			bestK = i;
		}
	}
	//if (bestK != -1)
	//{
	//	operand[level].foundImprove = 1;
	//	globalinfo.Rchanged[0] = kGain[bestK][2];
	//	globalinfo.Rchanged[1] = kGain[bestK][3];
	//	//fulls << "In, find_k_1_0, level= "<< level<<' '<<"operand[level].foundImprove = 1"<<endl;
	//	//fulls <<"globalinfo.Rchanged[0]= "<<globalinfo.Rchanged[0]<<' '<<"globalinfo.Rchanged[1]= "<<globalinfo.Rchanged[1]<<endl<<endl;
	//}
	
	return bestK;
}

//DONE
int find_k_1_1(float *cost_of_removing, float **(&kGain), int K, int level) //K is kth level of improvement
{
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::app);
	
	int no_of_attribute = 13;
	for (int i = 0; i < K; i++)
	{
		for (int j = 0; j < no_of_attribute; j++)
		{
			kGain[i][j] = -1; //reinitialize
		}
	}
	//int no_routes = LOCAL_NO_ROUTE;
	int element1, element2, before_pos1, before_pos2, after_pos1, after_pos2;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_j, gain;
	int before_ele1, before_ele2, after_ele1, after_ele2;
	std::vector<int> vector_r1;
	std::vector<int> vector_r2;
	std::vector<int> vector_r3;

	////// ======================== Find and Record the kth best in kGain matrix =====================//
	int set = 0;
	//route_file<<"Level "<<level;
	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	for (int i = 0; i < no_routes-1; i++)  //consider each and other routes (from R1)//changed to no_routes-1 on 6April2015
	{
		if (saiz[i]==0) //if it is empty route
			continue;
		float available_dist_r2 = distance_available[i];
		//============================= same route ========================================//
		if (operand[level].eachRfullsearchstatus[i][i] == 1 || saiz[i] <= 1)//if it is empty route or saiz=1, nothing to move around
			goto endsameroute;
		
		//copy route in vector
		vector_r3.clear();
		for (int j = 0; j < saiz[i] + 2; j++) //first and last is depot
		{
			vector_r3.push_back(VEHICLE[i][j]);
		}
	
		for (int j = 1; j <= saiz[i]-1 ; j++) //from which position
		{
			element1 = vector_r3[j];
			before_ele1 = vector_r3[j-1];
			after_ele1 = vector_r3[j+1];
			for (int k = j+1; k <= saiz[i]; k++)
			{
				element2 = vector_r3[k];
				before_ele2 = vector_r3[k-1];
				after_ele2 = vector_r3[k+1];

				if ( (before_ele2 == SIZE || after_ele2 == SIZE) && (before_ele1 == SIZE || after_ele1 == SIZE) ) //if mn and ij depot
				{	
					if ( (NR_FLAG_DEPOT[element1][before_ele2] == false && NR_FLAG_DEPOT[element1][after_ele2] == false) || (NR_FLAG_DEPOT[element2][before_ele1] == false && NR_FLAG_DEPOT[element2][after_ele1] == false) )
						continue;
				}
				else if ( before_ele2 == SIZE || after_ele2 == SIZE ) //if mn depot
				{
					if ( (NR_FLAG_DEPOT[element1][before_ele2] == false && NR_FLAG_DEPOT[element1][after_ele2] == false) || (NR_FLAG[element2][before_ele1] == false && NR_FLAG[element2][after_ele1] == false) )
						continue;
				}
				else if ( before_ele1 == SIZE || after_ele1 == SIZE ) //if ij depot
				{
					if ( (NR_FLAG[element1][before_ele2] == false && NR_FLAG[element1][after_ele2] == false) || (NR_FLAG_DEPOT[element2][before_ele1] == false && NR_FLAG_DEPOT[element2][after_ele1] == false) )
						continue;
				}
				else //if no depot
				{
					if ((NR_FLAG[element1][before_ele2] == false && NR_FLAG[element1][after_ele2] == false) || (NR_FLAG[element2][before_ele1] == false && NR_FLAG[element2][after_ele1] == false) ) //if insertion cannot between cust before and cust after
						continue;
				}

				gain=0;
					
				//in this case, j is always smaller than k
				//int first = j, second=j+1, third=k, fourth=k+1;//first is position of ele1, second is after_ele1, third is ele2, fourth is after_ele2
				float newCumDistFirst = CumDist[i][j-1] + dist[before_ele1][element2] + service_time[element2];
				//float newCumDistSecond = newCumDistFirst + dist[element2][after_ele1] + service_time[after_ele1];
				float Firstincre = CumDist[i][j] - newCumDistFirst ;//old - new CumDist//positive is good, negative is bad
				//float Secondincre = CumDist[i][j+1] - newCumDistSecond;
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
					float Secondincre = CumDist[i][j+1] - newCumDistSecond;
					gain = gain +Secondincre;

					if (k!=saiz[i])//if k is not the last customer, if it is last customer, noneed to calculate the remaining
					{
						float newCumDistThird = (newCumDistSecond) + dist[element1][after_ele2] + service_time[after_ele2];
						float Thirdincre = CumDist[i][j+2] - newCumDistThird;//old - new CumDist//positive is good, negative is bad
						//inbetween calculate number of customers at j+1 until saiz[i] (including j+1)
						int inbetween1 = saiz[i]-(j+1);//if use saiz[i]-(j+1+1), then need to add Thirdincre in gain because Thirdincre is used on third element until the last one
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
					float Secondincre = CumDist[i][j+1] - newCumDistSecond;
					gain = gain + Secondincre;
					//inbetween1 calculate number of customers at j+1 until k-1 (exclude j and exclude k)
					int inbetween1 = (k-1)-(j+1);
					if (inbetween1>0)
					{
						gain = gain+(Secondincre*inbetween1);//in between can be 0 mean ele1 and ele is 1 customer apart, eg: ele1, 5, ele2
					}
					//we dont know the newCumDist at point k-1, so we use the oriCumDist minus the Seconsdincre (minus increment because for increment positive is good, negative is bad)
					float newCumDistThird = (CumDist[i][k-1]-Secondincre) + dist[before_ele2][element1] + service_time[element1];//minus increment, because for increment positive is good, negative is bad
					float Thirdincre = CumDist[i][k] - newCumDistThird;//old - new CumDist//positive is good, negative is bad
					gain = gain+Thirdincre;


					//Fourth may never exists if k is saiz[i]
					if (k<saiz[i])
					{
						float newCumDistFourth = newCumDistThird + dist[element1][after_ele2]  + service_time[after_ele2];;
						float Fourthincre = CumDist[i][k+1] - newCumDistFourth;
						gain = gain+Fourthincre;
						//to find how many customers in between Fourth and end of route
						//inbetween2 calculate number of customers at k+1 until saiz[i] (including k+1)
						int inbetween2 = saiz[i]-(k+1);//if use saiz[i]-(k+1+1), then need to add Fourthincre in gain because Fourthincre is used on fourth element until the last one
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

				//if (abs(gain) > distance_available[i])//if negative gain, it will not be recorded anyway, so this code can be omitted actually
				//{
				//	continue;
				//}

				if (gain > epsilonn)
				{
					kGain[set][0] = set;
					kGain[set][1] = gain;
					kGain[set][2] = i; //from r1,
					kGain[set][3] = i; //to_r2
					kGain[set][4] = j; //from r1_p
					kGain[set][5] = k; //to_r2_p //to which position (swap position)=route is not modified
					kGain[set][6] = k; //from_r2_p
					kGain[set][7] = j; //to_r1_p
					kGain[set][8] = 2; //2 is (1-1)
					kGain[set][10] = gain;//gain1
					kGain[set][11] = cost_without_i - cost_with_i; //gain1 in distance
					set++;
					operand[level].eachRfullsearchstatus[i][i] = 0;

				}	
				if (set == K)
					goto find_best_k2;
			}//end k
		}//end j
		vector_r3.clear();

	endsameroute:
		for (int m = i+1; m < no_routes; m++)  //insert (to) which route (to R2) //changed m=i+1 on 6April2015
		//for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
		{
			if ((i == m) || (saiz[m]==0))//same route already consider
			{
				continue;
			}
			int foundimprovementstatus=0;//to indicate improvement found or not for this route, to flag the operand[1].eachRfullsearchstatus[i] at the end
			if (operand[level].eachRfullsearchstatus[i][m] == 1 )
			{
				//route_file<<" Omit ["<<i<<"]["<<m<<"]"<<' '<<' '<<' '<<' ';
				continue;
			}
			
			//======================================== different route ============================================//
			for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
			//for (int m = i+1; m < no_routes; m++)  //insert (to) which route (to R2) //changed m=i+1 on 6April2015
			{
			
				element1 = VEHICLE[i][j];			//elemet to be inserted into other route
				before_ele1 = VEHICLE[i][j-1];
				after_ele1 = VEHICLE[i][j+1];
			
				float origain1 = dist[element1][before_ele1] + dist[element1][after_ele1] + service_time[element1] - dist[before_ele1][after_ele1];//old-new positive is good
				//to find cost_with_i which is besed on the modified route
				//find a temporaryCumDCust
				float *tempCumDist1= new float[SIZE];
				tempCumDist1[0] = 0;
							
				//following is R1
				for (int t = 1; t < saiz[i]; t++)//have 1 size less because 1 customer is deleted
				{
					if (t<j)
						tempCumDist1[t]= CumDist[i][t];//before the delete pos, the CumDist is the same
					else if(t>=j)
						tempCumDist1[t]=CumDist[i][t+1]-origain1;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
							
				}
				for (int n = 1; n <= saiz[m]; n++) //which customer in r2 to remove
				{
					element2 = VEHICLE[m][n];			//element to be removed from r2
					before_ele2 = VEHICLE[m][n-1];
					after_ele2 = VEHICLE[m][n+1];
					int space_r1 = space_available[i];
					int space_r2 = space_available[m];
					space_r1 = space_r1 + demand[element1];
					space_r2 = space_r2 + demand[element2];

					if ((demand[element1] > space_r2) || (demand[element2] > space_r1))
						continue; //remove the next element2
					
					float origain2 = dist[element2][before_ele2] + dist[element2][after_ele2] + service_time[element2] - dist[before_ele2][after_ele2];//old-new positive is good	
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
					for (int a = 0; a < saiz[i] + 2; a++)
					{
						if (a==j)
							continue;
						vector_r1.push_back(VEHICLE[i][a]);
					}
					//vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot

					//==================================================copy r2 without customer n=================================================================//
					vector_r2.clear();
					for (int a = 0; a < saiz[m] + 2; a++)
					{
						if (a==n)
							continue;
						vector_r2.push_back(VEHICLE[m][a]);
					}
					//vector_r2.erase(vector_r2.begin() + n); //first one is depot
											
					float available_dist_r1 = distance_available[i];
					float available_dist_r2 = distance_available[m];
	

					//############################################## To Insert ##########################################################////////////////////
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

							cost_without_i = cost_of_removing[VEHICLE[i][j]];
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
							//for (int t = 1; t < saiz[i]; t++)//have 1 size less because 1 customer is deleted
							//{
							//	if (t<j)
							//		tempCumDist1[t]= CumDist[i][t];//before the delete pos, the CumDist is the same
							//	else if(t>=j)
							//		tempCumDist1[t]=CumDist[i][t+1]-origain1;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
							//
							//}
						
							float oriloss1 = dist[before_pos1][element2] + dist[element2][after_pos1] + service_time[element2] - dist[before_pos1][after_pos1]; //element from r2 to r1 or insert ele2 to r1//old -new
							float CumAtEle1 = tempCumDist1[q-1] + dist[before_pos1][element2] + service_time[element2]; 
							int remainingCustafterEle1 = saiz[i] - q;
							if (remainingCustafterEle1 < 0)//if negative
								remainingCustafterEle1 = 0;
							cost_with_i = CumAtEle1 + remainingCustafterEle1*oriloss1; 


							//following is R2
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


							if (gain > epsilonn)
							{
								foundimprovementstatus = 1;
								kGain[set][0] = set;
								kGain[set][1] = gain;
								kGain[set][2] = i; //from r1,
								kGain[set][3] = m; //to_r2
								kGain[set][4] = j; //from r1_p
								kGain[set][5] = p; //to_r2_p //to which position (swap position)=route is not modified
								kGain[set][6] = n; //from_r2_p
								kGain[set][7] = q; //to_r1_p
								kGain[set][8] = 2; //2 is (1-1)
								kGain[set][10] = cost_without_i- cost_with_i;//gain1
								kGain[set][11] = origain1 - oriloss1; //gain1 in distance
								kGain[set][12] = origain2 - oriloss2; //gain2 in distance
								set++;
								operand[level].eachRfullsearchstatus[i][m] = 0;
								//to check
								//if((j==q) && (p==n))
								//{
								//	float cost_without_i2 = dist[before_pos1][element1] + dist[element1][after_pos1] + service_time[element1] - dist[before_pos1][after_pos1];//to check 
								//	float cost_without_j2 = dist[before_pos2][element2] + dist[element2][after_pos2] + service_time[element2] - dist[before_pos2][after_pos2];//to check 
								//	if ((abs(cost_without_i2-cost_of_removing[VEHICLE[i][j]])>0.1) || (abs(cost_without_j2-cost_of_removing[VEHICLE[m][n]])>0.1))
								//	{
								//		cout<<"find_1_1 got problem!!"<<endl;
								//		getchar();
								//	}
								//	
								//}
							}//end if
							if (set == K)
							{
								delete[]tempCumDist2;
								delete[]tempCumDist1;
								goto find_best_k2;
							}
							
						}//end for q
					}//end for p
					
					delete[]tempCumDist2;
				}//end for n
				delete[]tempCumDist1;
			}//end for j
			if(foundimprovementstatus == 0)
				operand[level].eachRfullsearchstatus[i][m] = 1;
		}//end for m
	}//end for i 

find_best_k2:;
	//cout<<"======= kGain in find_k_1_1() =========="<<endl;
	//for (int i = 0; i < K; i++)
	//{
	//	for (int j = 0; j < 9; j++)
	//	{
	//		cout<< kGain[i][j] << ' ' << ' ';
	//	}
	//	cout<<endl;
	//}
	int bestK = -1; //initialize
	float maxGain = INT_MIN;
	for (int i = 0; i < K; i++)
	{
		if ((kGain[i][1] > maxGain) && (kGain[i][1] >= epsilonn))
		{
			maxGain = kGain[i][1];
			bestK = i;
		}

	}
	//if (bestK != -1)
	//{
	//	operand[level].foundImprove = 1;
	//	globalinfo.Rchanged[0] = kGain[bestK][2];
	//	globalinfo.Rchanged[1] = kGain[bestK][3];
	//	//fulls << "In, find_k_1_1, level= "<< level<<' '<<"operand[level].foundImprove = 1"<<endl;
	//	//fulls <<"globalinfo.Rchanged[0]= "<<globalinfo.Rchanged[0]<<' '<<"globalinfo.Rchanged[1]= "<<globalinfo.Rchanged[1]<<endl<<endl;
	//}
	
	return bestK;
}

//No same route, different route only
//DONE
int find_k_2_1(float *cost_of_removing, float **(&kGain), int K, int level)
{
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::app);
	
	int no_of_attribute = 13;
	for (int i = 0; i < K; i++)
	{
		for (int j = 0; j < no_of_attribute; j++)
		{
			kGain[i][j] = -1; //reinitialize
		}
	}
	//int no_routes = LOCAL_NO_ROUTE;
	int element1, element1f, element2, before_ele1, before_ele2, after_ele1, after_ele2, before_pos1, before_pos2, after_pos1, after_pos2;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_j, cost_with_jRev, gain;
	int element3, before_ele3, after_ele3;
	int reverseStatus;
	std::vector<int> vector_r1;
	std::vector<int> vector_r2;
	std::vector<int> vector_r3;
	
	////// ======================== Find and Record the kth best in kGain matrix =====================//
	int set = 0;
	//fulls << "In, find_k_2_1, level= "<< level<<endl;
	//fulls <<"globalinfo.Rchanged[0]= "<<globalinfo.Rchanged[0]<<' '<<"globalinfo.Rchanged[1]= "<<globalinfo.Rchanged[1]<<endl<<endl;
	
	//##########################################calculate GAIN when remove 2 and insert 1 simultaneously######################################################//
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	//route_file<<"Level "<<level;
	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from R1)
	{
		if ((saiz[i]==0) || (saiz[i]==1)) //if it is empty route or saiz=1
			continue;
	//	//================================= same route ======================================//	
	//	if (operand[level].eachRfullsearchstatus[i][i] == 1 || saiz[i]<=2)  //if it is empty route or saiz=2, nothing to move around
	//		goto endsameroute;
	//	
	//	//copy route in vector
	//	vector_r3.clear();
	//	for (int j = 0; j < saiz[i] + 2; j++) //first and last is depot
	//	{
	//		vector_r3.push_back(VEHICLE[i][j]);
	//	}
	//
	//	for (int j = 1; j <= saiz[i]-1 ; j++) //from which position, take in pair, so until saiz-1
	//	{
	//		element1 = vector_r3[j];
	//		element2 = vector_r3[j+1];
	//		before_ele1 = vector_r3[j-1];
	//		after_ele2 = vector_r3[j+2];
	//		for (int k = 1; k <= saiz[i]; k++) //which position to swap with, take one customer
	//		{
	//			if ((j==k) || ((j+1) ==k) )
	//			{
	//				continue;
	//			}
	//			element3 = vector_r3[k];
	//			before_ele3 = vector_r3[k-1];
	//			after_ele3 = vector_r3[k+1];
	//			if ( (before_ele3 == SIZE) || (after_ele3 == SIZE) && (before_ele1 == SIZE || after_ele2 == SIZE) ) //if mn and ij depot
	//			{	
	//				if ( (NR_FLAG_DEPOT[element1][before_ele3] == false && NR_FLAG_DEPOT[element2][after_ele3] == false) || (NR_FLAG_DEPOT[element3][before_ele1] == false && NR_FLAG_DEPOT[element3][after_ele2] == false) )//original order
	//				{	if ( NR_FLAG_DEPOT[element2][before_ele3] == false && NR_FLAG_DEPOT[element1][after_ele3] == false)//reverse order
	//						continue;
	//				}
	//			}
	//			else if ((before_ele3 == SIZE) || (after_ele3 == SIZE) ) //if mn depot
	//			{
	//				if ( (NR_FLAG_DEPOT[element1][before_ele3] == false && NR_FLAG_DEPOT[element2][after_ele3] == false) || (NR_FLAG[element3][before_ele1] == false && NR_FLAG[element3][after_ele2] == false) )//original order
	//				{	if ( NR_FLAG_DEPOT[element2][before_ele3] == false && NR_FLAG_DEPOT[element1][after_ele3] == false)//reverse order
	//						continue;
	//				}
	//			}
	//			else if ((before_ele1 == SIZE) || (after_ele2 == SIZE) ) //if ij depot
	//			{
	//				if ( (NR_FLAG[element1][before_ele3] == false && NR_FLAG[element2][after_ele3] == false) || (NR_FLAG_DEPOT[element3][before_ele1] == false && NR_FLAG_DEPOT[element3][after_ele2] == false) )//original order
	//				{	if ( NR_FLAG[element2][before_ele3] == false && NR_FLAG[element1][after_ele3] == false)//reverse order
	//						continue;
	//				}
	//			}
	//			else //if no depot
	//			{
	//				if ( (NR_FLAG[element1][before_ele3] == false && NR_FLAG[element2][after_ele3] == false) || (NR_FLAG[element3][before_ele1] == false && NR_FLAG[element3][after_ele2] == false) )//original order
	//				{	if ( NR_FLAG[element2][before_ele3] == false && NR_FLAG[element1][after_ele3] == false)//reverse order
	//						continue;
	//				}
	//			}
	//
	//			//same route, noneed to consider service_time[]
	//			if (k > j) //if single customer is after pair customer
	//			{
	//				if (after_ele2 == element3) //in consecutive, so minus 3 arcs, for eg: 4 2 3 7 5 1 (swap pair 2 3 with 7) 
	//				{
	//					cost_without_i = dist[before_ele1][element1] + dist[element2][after_ele2] + dist[element3][after_ele3]; //route_cost is global variable //minus 4 arcs 
	//					cost_with_i = dist[before_ele1][element3]  + dist[element3][element1] +dist[element2][after_ele3];
	//					cost_with_iReverse = dist[before_ele1][element3]  + dist[element3][element2] +dist[element1][after_ele3];
	//				}
	//			
	//				else 
	//				{
	//					cost_without_i = dist[before_ele1][element1] + dist[element2][after_ele2] + dist[before_ele3][element3] + dist[element3][after_ele3]; //route_cost is global variable //minus 4 arcs 
	//					cost_with_i = dist[before_ele1][element3] + dist[element3][after_ele2] +dist[before_ele3][element1] +dist[element2][after_ele3];
	//					cost_with_iReverse = dist[before_ele1][element3] + dist[element3][after_ele2] +dist[before_ele3][element2] +dist[element1][after_ele3];
	//				}
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
	//			reverseStatus = 0;//initialize to 0
	//			if (cost_with_iReverse - cost_with_i < -epsilonn)
	//			{
	//				cost_with_i = cost_with_iReverse;
	//				reverseStatus = 1;
	//			}
	//			gain = cost_without_i - cost_with_i; //gain = old-new
	//			//if (abs(gain) > distance_available[r])//gain is negative, change to positive using abs
	//			//{
	//			//	continue;
	//			//}
	//			if (gain > epsilonn)
	//			{
	//				kGain[set][0] = set;
	//				kGain[set][1] = gain;
	//				kGain[set][2] = i; //from r1,
	//				kGain[set][3] = i; //to_r2
	//				kGain[set][4] = j; //from r1_p
	//				kGain[set][5] = k; //to_r2_p //to which position (swap position)=route is not modified
	//				kGain[set][6] = k; //from_r2_p
	//				kGain[set][7] = j; //to_r1_p
	//				kGain[set][8] = 3; //3 is (2-1)
	//				kGain[set][9] = reverseStatus; 
	//				kGain[set][10] = gain;//gain1
	//				set++;
	//				operand[level].eachRfullsearchstatus[i][i] = 0;
	//				
	//			}	
	//			if (set == K)
	//				goto find_best_k3;
	//		}
	//	}//end j
	//	vector_r3.clear();
	//	
	//endsameroute:

		for (int m = 0; m < no_routes; m++)  //consider each and other routes (from R2)
		//for (int j = 1; j < saiz[i]; j++)  // consider each customer (from R1), start from 1 because [0] is depot
		{	
			if ((i == m) || (saiz[m]==0)) //same route already consider
			{
				continue;
			}
			int foundimprovementstatus=0;//to indicate improvement found or not for this route, to flag the operand[1].eachRfullsearchstatus[i] at the end
			if (operand[level].eachRfullsearchstatus[i][m] == 1 )
			{
				//route_file<<" Omit ["<<i<<"]["<<m<<"]"<<' '<<' '<<' '<<' ';
				continue;
			}
			
			//========================================== different route =======================================//

			for (int j = 1; j < saiz[i]; j++)  // consider each customer (from R1), start from 1 because [0] is depot
			//for (int m = 0; m < no_routes; m++)  //consider each and other routes (from R2)
			{
				element1 = VEHICLE[i][j];			//element to be inserted into other route
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

				for (int n = 1; n < saiz[m] + 1; n++) //which customer in r2 to remove
				{	
					element2 = VEHICLE[m][n];			//element to be removed from r2
					before_ele2 = VEHICLE[m][n-1];
					after_ele2 = VEHICLE[m][n+1];

					int space_r1 = space_available[i];
					int space_r2 = space_available[m];
					space_r1 = space_r1 + demand[element1] + demand[element1f];
					space_r2 = space_r2 + demand[element2];
					
					if ((demand[element1] + demand[element1f] > space_r2) || (demand[element2] > space_r1))
					{
						continue;
					}
					
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
						if ((a==j) || (a==j+1))
							continue;
						vector_r1.push_back(VEHICLE[i][a]);
					}
					//vector_r1.erase(vector_r1.begin() + j); //+1 because element [0] is depot
					//vector_r1.erase(vector_r1.begin() + j); //delete next element
					
					//==================================================copy r2 without customer n=================================================================//
					vector_r2.clear();
					for (int a = 0; a < saiz[m] + 2; a++)
					{
						if (a==n)
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
						if ((before_pos2 == SIZE) || (after_pos2 == SIZE) ) //if mn depot //changed this to here on 25JUN2015!!!!!!!!!!!!!!!!!!!!! OMG!!! before that put after n loop
						{
							if ( NR_FLAG_DEPOT[element1][before_pos2] == false && NR_FLAG_DEPOT[element1f][after_pos2] == false )//original order
							{	if ( NR_FLAG_DEPOT[element1f][before_pos2] == false && NR_FLAG_DEPOT[element1][after_pos2] == false )//reverse order
									continue;
							}
						}
						else //if no depot
						{
							if ( NR_FLAG[element1][before_pos2] == false && NR_FLAG[element1f][after_pos2] == false )//original order
							{	if ( NR_FLAG[element1f][before_pos2] == false && NR_FLAG[element1][after_pos2] == false )//reverse order
									continue;
							}
						}

						for (int q = 1; q < vector_r1.size(); q++) //from r2, insert to r1
						{
							reverseStatus = 0; //initialize 
							before_pos1 = vector_r1[q - 1];
							after_pos1 = vector_r1[q];  //noneed to add 1 because it is insert in between n-1 and n
							if ((before_pos1 == SIZE) || (after_pos1 == SIZE)) //if ij depot
							{	
								if ( NR_FLAG_DEPOT[element2][before_pos1] == false && NR_FLAG_DEPOT[element2][after_pos1] == false )
									continue;
							}
							
							else //if no depot
							{
								if ( NR_FLAG[element2][before_pos1] == false && NR_FLAG[element2][after_pos1] == false )
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
							int remainingCustafterEle1 = saiz[i] - q - 1;//(saiz-pos-1) //minus 1 more because two customers have been deleted 
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
							
							if (gain > epsilonn)
							{
								foundimprovementstatus = 1;
								kGain[set][0] = set;
								kGain[set][1] = gain;
								kGain[set][2] = i; //from r1,
								kGain[set][3] = m; //to_r2
								kGain[set][4] = j; //from r1_p
								kGain[set][5] = p; //to_r2_p //to which position (swap position)=route is not modified
								kGain[set][6] = n; //from_r2_p
								kGain[set][7] = q; //to_r1_p
								kGain[set][8] = 3; //3 is (2-1)
								kGain[set][9] = reverseStatus;
								kGain[set][10] = cost_without_i- cost_with_i;//gain1
								kGain[set][11] = origain1 - oriloss1;//gain1 in distance
								kGain[set][12] = origain2 - oriloss2;//gain2 in distance
								set++;
								operand[level].eachRfullsearchstatus[i][m] = 0;
							}//end if

							if (set == K)
							{
								delete[] tempCumDist2;
								delete[] tempCumDist1;
								goto find_best_k3;
							}
						}//end for q
					}//end for p
					delete[] tempCumDist2;
				}//end for n
				delete[] tempCumDist1;
			}//end for j
			if(foundimprovementstatus==0)
				operand[level].eachRfullsearchstatus[i][m] = 1;
		}//end for m
	}//end for i 

find_best_k3:
	//cout<<"======= kGain in find_k_2_1() =========="<<endl;
	//for (int i = 0; i < K; i++)
	//{
	//	for (int j = 0; j < 9; j++)
	//	{
	//		cout<< kGain[i][j] << ' ' << ' ';
	//	}
	//	cout<<endl;
	//}
	int bestK = -1; //initialize
	float maxGain = INT_MIN;
	for (int i = 0; i < K; i++)
	{
		if ((kGain[i][1] > maxGain)&& (kGain[i][1] >= epsilonn))
		{
			maxGain = kGain[i][1];
			bestK = i;
		}
	}

	//if (bestK != -1)
	//{
	//	operand[level].foundImprove = 1;
	//	globalinfo.Rchanged[0] = kGain[bestK][2];
	//	globalinfo.Rchanged[1] = kGain[bestK][3];
	//	//fulls << "In, find_k_2_1, level= "<< level<<' '<<"operand[level].foundImprove = 1"<<endl;
	//	//fulls <<"globalinfo.Rchanged[0]= "<<globalinfo.Rchanged[0]<<' '<<"globalinfo.Rchanged[1]= "<<globalinfo.Rchanged[1]<<endl<<endl;
	//}
	
	return bestK;
}

//DONE
int find_k_2_0(float *cost_of_removing, float **(&kGain), int K, int level) //return which one is the best out of k
{
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::app);
	
	int no_of_attribute = 13;
	for (int i = 0; i < K; i++)
	{
		for (int j = 0; j < no_of_attribute; j++)
		{
			kGain[i][j] = -1; //reinitialize
		}
	}
	int element1=-1, element2=-1, before_ele, after_ele, before_pos, after_pos;
	float gain = 0.0;
	float cost_without_i, cost_with_i, cost_with_iRev;
	int reverseStatus;

	//int no_routes = LOCAL_NO_ROUTE;	
	
	////// ======================== Find and Record the kth best in kGain matrix =====================//
	int set = 0;
	
	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from)
	{
		if ((saiz[i]==0)||(saiz[i]==1)) //if it is empty route or saiz=1
			continue;
		
		//======================================== same route ======================================//
		if (operand[level].eachRfullsearchstatus[i][i] == 1 || saiz[i] <= 2)  //if it is empty route or saiz=1 or saiz=2, nothing to move around
			goto endofsameroute;
	
		for (int j = 1; j <= saiz[i]-1; j++) //which one to delete, customer deleted in sequential //customer considered in pair, from 1 until saiz-1
		{
			float available_dist_r2 = distance_available[i];
			//copy a temporary route
			int b=0;

			int* temp_r = new int[saiz[i]+2];
			for (int a = 0; a < saiz[i] + 2; a++)
			{
				if ((a == j)||(a == j+1))
				{
					continue; //dont copy the element, assume the element to be deleted
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

			float origain = dist[element1][before_ele] + dist[element1][element2] + dist[element2][after_ele] + service_time[element1] + service_time[element2] - dist[before_ele][after_ele]; //cost of r1 without i and i+1 // old-new
			float *tempCumDist= new float[SIZE];
			tempCumDist[0] = 0;
			for (int t = 1; t <= saiz[i]-2; t++)//have 2 size less because 2 customers are deleted
			{
				if (t<j)
					tempCumDist[t]= CumDist[i][t];//before the delete pos, the CumDist is the same
				else if(t>=j)
					tempCumDist[t]=CumDist[i][t+2]-origain;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
					
			}
			//this is just to calculate the cost, not really to insert, the insertion is done is function insert()
			for (int n = 1; n < saiz[i]; n++) //has 2 customer less because it has been deleted
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

				float oriloss =  dist[before_pos][element1] + dist[element1][element2]+dist[element2][after_pos] + service_time[element1] + service_time[element2] - dist[before_pos][after_pos]; //cost of r2 with i and i+1 //new-old , big not good, small is good
				float orilossRev =  dist[before_pos][element2] + dist[element2][element1]+dist[element1][after_pos] + service_time[element1] + service_time[element2] - dist[before_pos][after_pos]; //cost of r2 with i+1 and i //new-old
				
				float newCumDist1= tempCumDist[n-1] + dist[before_pos][element1] + service_time[element1]; 
				float newCumDist2= newCumDist1 + dist[element1][element2] + service_time[element2]; 
				
				float newCumDist1Rev= tempCumDist[n-1] + dist[before_pos][element2] + service_time[element2]; 
				float newCumDist2Rev= newCumDist1Rev + dist[element2][element1] + service_time[element1]; 
				
				remaining_cust = saiz[i] - (n+1);//always minus the position of the second customer n+1
				
				cost_with_i = newCumDist1 + newCumDist2 + remaining_cust*oriloss;
				cost_with_iRev = newCumDist1Rev + newCumDist2Rev + remaining_cust*orilossRev;

				if(cost_with_iRev < cost_with_i )//if reverse has smaller cost
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
				
				if (oriloss - origain > available_dist_r2 + epsilon)
				{
					continue;
				}

	
				if (gain > epsilonn)
				{
					kGain[set][0] = set;
					kGain[set][1] = gain;
					kGain[set][2] = i; //from r1,
					kGain[set][3] = i; //to_r2
					kGain[set][4] = j; //from r1_p
					kGain[set][5] = n; //to_r2_p //to which position //assume modified route, customer has been deleted
					kGain[set][6] = -1; //from_r2_p
					kGain[set][7] = -1; //to_r1_p
					kGain[set][8] = 4; //4 is (2-0)
					kGain[set][9] = reverseStatus; 
					kGain[set][10] = gain;//gain1
					kGain[set][11] = origain - oriloss; //gain1 in distance
					set++;
					operand[level].eachRfullsearchstatus[i][i] = 0;
	
				}	
						
				if (set == K) //set starts from 0 => set=3 means 0, 1, 2 sets
				{
					delete[]tempCumDist;	
					delete[] temp_r;	
					goto find_best_k4;
				}
			}
			delete[]tempCumDist;	
			delete[] temp_r;	
		}//end of j

endofsameroute:

		for (int m = 0; m < no_routes; m++)  //insert (to) which route
		//for (int j = 1; j < saiz[i]; j++)  // consider each customer (from) //consider 2 customer in sequence //customer considered in pair, from 1 until saiz-1
		{
			if (i == m) //if same route, already consider
			{
				continue;
			}
			int foundimprovementstatus=0;//to indicate improvement found or not for this route, to flag the operand[1].eachRfullsearchstatus[i] at the end
			if (operand[level].eachRfullsearchstatus[i][m] == 1 )
			{
				//route_file<<" Omit ["<<i<<"]["<<m<<"]"<<' '<<' '<<' '<<' ';
				continue;
			}
			
			//===================================== different route =====================================//
			for (int j = 1; j < saiz[i]; j++)  // consider each customer (from) //consider 2 customer in sequence //customer considered in pair, from 1 until saiz-1
			//for (int m = 0; m < no_routes; m++)  //insert (to) which route
			{
				element1 = VEHICLE[i][j];			//elemet to be inserted into other route
				element2 = VEHICLE[i][j+1];

				
				if (demand[element1]+demand[element2] > space_available[m]) //if demand exceed avaialable space in route and different route, if same route, still need to find the best even the capacity is violated
				{
					continue;
				}

				for (int n = 1; n <= saiz[m] + 1; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
				{
					
					reverseStatus = 0;
					//element1 = VEHICLE[i][j];//must initialize again because elemnt1 and 1f might be reversed just now
					//element2 = VEHICLE[i][j+1];

					before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[i][j + 2];
					before_pos = VEHICLE[m][n - 1];
					after_pos = VEHICLE[m][n];  //noneed to add 1 because it is insert in between n-1 and n
					
					if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
					{
						if ( (NR_FLAG_DEPOT[element1][before_pos] == false && NR_FLAG_DEPOT[element2][after_pos] == false) )
						{	if ( (NR_FLAG_DEPOT[element2][before_pos] == false && NR_FLAG_DEPOT[element1][after_pos] == false) )
								continue;
						}
					}

					else //if not between customer and depot
					{	
						if ( (NR_FLAG[element1][before_pos] == false && NR_FLAG[element2][after_pos] == false) )
						{	if ( (NR_FLAG[element2][before_pos] == false && NR_FLAG[element1][after_pos] == false) )
								continue;
						}
					}


					float available_dist_r2 = distance_available[m];
					before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[i][j + 2];

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

					if (gain > epsilonn)
					{
						foundimprovementstatus = 1;
						kGain[set][0] = set;
						kGain[set][1] = gain;
						kGain[set][2] = i; //from r1,
						kGain[set][3] = m; //to_r2
						kGain[set][4] = j; //from r1_p
						kGain[set][5] = n; //to_r2_p //to which position //assume modified route, customer has been deleted
						kGain[set][6] = -1; //from_r2_p
						kGain[set][7] = -1; //to_r1_p
						kGain[set][8] = 4; //4 is (2-0)
						kGain[set][9] = reverseStatus;
						kGain[set][10] = cost_without_i;//gain1
						kGain[set][11] = origain;//gain1 in distance
						kGain[set][12] = -oriloss;//gain2 in distance
						set++;
						operand[level].eachRfullsearchstatus[i][m] = 0;
					}	

					if (set == K) //set starts from 0 => set=3 means 0, 1, 2 sets
					{
						goto find_best_k4;
					}	
				}//end for n
			}//end for j
			if(foundimprovementstatus == 0)
				operand[level].eachRfullsearchstatus[i][m] = 1 ;
		}//end for m
	}//end for i 

find_best_k4:
	//cout<<"======= kGain in find_k_2_0() =========="<<endl;
	//for (int i = 0; i < K; i++)
	//{
	//	for (int j = 0; j < 9; j++)
	//	{
	//		cout<< kGain[i][j] << ' ' << ' ';
	//	}
	//	cout<<endl;
	//}
	int bestK=-1; //initialize
	float maxGain = INT_MIN;
	for (int i = 0; i < K; i++)
	{
		if ((kGain[i][1] > maxGain) && (kGain[i][1] >= epsilonn))
		{
			maxGain = kGain[i][1];
			bestK = i;
		}
	}
	//if (bestK != -1)
	//{
	//	operand[level].foundImprove = 1;
	//	globalinfo.Rchanged[0] = kGain[bestK][2];
	//	globalinfo.Rchanged[1] = kGain[bestK][3];
	//	//fulls << "In, find_k_2_0, level= "<< level<<' '<<"operand[level].foundImprove = 1"<<endl;
	//	//fulls <<"globalinfo.Rchanged[0]= "<<globalinfo.Rchanged[0]<<' '<<"globalinfo.Rchanged[1]= "<<globalinfo.Rchanged[1]<<endl<<endl;
	//}
	
	return bestK;
}

int find_swap_1_1(float *cost_of_removing, float **(&kGain), int K, int level) //K is kth level of improvement
{
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::app);
	
	int no_of_attribute = 13;
	for (int i = 0; i < K; i++)
	{
		for (int j = 0; j < no_of_attribute; j++)
		{
			kGain[i][j] = -1; //reinitialize
		}
	}
	//int no_routes = LOCAL_NO_ROUTE;
	int element1, element2, before_pos1, before_pos2, after_pos1, after_pos2;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_j, gain;

	//std::vector<int> vector_r1;
	//std::vector<int> vector_r2;

	////// ======================== Find and Record the kth best in kGain matrix =====================//
	int set = 0;
	
	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//route_file<<"Level "<<level;
	for (int i = 0; i < no_routes-1; i++)  //consider each and other routes (from R1)//changed to no_routes-1 on 6April2015
	{
		if (saiz[i]==0) //if it is empty route
			continue;
		
		for (int m = i+1; m < no_routes; m++)  //insert (to) which route (to R2) //changed m=i+1 on 6April2015
		//for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
		{
			if ((i == m) || (saiz[m]==0)) //if customer already in the route or demand exceed avaialable space in route
			{
				continue;
			}
			int foundimprovementstatus=0;//to indicate improvement found or not for this route, to flag the operand[1].eachRfullsearchstatus[i] at the end
			if (operand[level].eachRfullsearchstatus[i][m] == 1 )
			{
				continue;
			}

			for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
			//for (int m = i+1; m < no_routes; m++)  //insert (to) which route (to R2) //changed m=i+1 on 6April2015
			{
				element1 = VEHICLE[i][j];			//element to be inserted into other route
				before_pos1 = VEHICLE[i][j-1];
				after_pos1 = VEHICLE[i][j+1];



				for (int n = 1; n < saiz[m] + 1; n++) //which customer in r2 to remove
				{
					element2 = VEHICLE[m][n];			//element to be removed from r2
					before_pos2 = VEHICLE[m][n-1];
					after_pos2 = VEHICLE[m][n+1];
					
					if ( (before_pos2 == SIZE || after_pos2 == SIZE) && (before_pos1 == SIZE || after_pos1 == SIZE) ) //if mn and ij depot
					{	
						if ( (NR_FLAG_DEPOT[element1][before_pos2] == false && NR_FLAG_DEPOT[element1][after_pos2] == false) || (NR_FLAG_DEPOT[element2][before_pos1] == false && NR_FLAG_DEPOT[element2][after_pos1] == false) )
							continue;
					}
					else if ((before_pos2 == SIZE) || (after_pos2 == SIZE) ) //if mn depot
					{
						if ((NR_FLAG_DEPOT[element1][before_pos2] == false && NR_FLAG_DEPOT[element1][after_pos2] == false) || (NR_FLAG[element2][before_pos1] == false && NR_FLAG[element2][after_pos1] == false) )
							continue;
					}
					else if ((before_pos1 == SIZE) || (after_pos1 == SIZE) ) //if ij depot
					{
						if ((NR_FLAG[element1][before_pos2] == false && NR_FLAG[element1][after_pos2] == false) || (NR_FLAG_DEPOT[element2][before_pos1] == false && NR_FLAG_DEPOT[element2][after_pos1] == false) )
							continue;
					}
					else //if no depot
					{
						if ((NR_FLAG[element1][before_pos2] == false && NR_FLAG[element1][after_pos2] == false) || (NR_FLAG[element2][before_pos1] == false && NR_FLAG[element2][after_pos1] == false))
							continue;
					}

					//if (NR_FLAG_DEPOT[element1][element2] == false) //or if (NR_FLAG_DEPOT[element1][element2] == false)
					//	continue;

					int space_r1 = space_available[i];
					int space_r2 = space_available[m];
					space_r1 = space_r1 + demand[element1];
					space_r2 = space_r2 + demand[element2];

					if ((demand[element1] > space_r2) || (demand[element2] > space_r1))
						continue; //remove the next element2

					//==================================================swap element 1 with element 2 =================================================================//
					//vector_r1.clear();
					//for (int a = 0; a < saiz[i] + 2; a++)
					//{
					//	if (a==j)
					//		vector_r1.push_back(element2);
					//	vector_r1.push_back(VEHICLE[i][a]);
					//}
			
					//vector_r2.clear();
					//for (int a = 0; a < saiz[m] + 2; a++)
					//{
					//	if (a==n)
					//		vector_r2.push_back(element1);
					//	vector_r2.push_back(VEHICLE[m][a]);
					//}
											
					float available_dist_r1 = distance_available[i];
					float available_dist_r2 = distance_available[m];
	

					cost_without_i = cost_of_removing[VEHICLE[i][j]];
					cost_without_j = cost_of_removing[VEHICLE[m][n]];

					cost_with_i = dist[before_pos1][element2] + dist[element2][after_pos1] + service_time[element2]- dist[before_pos1][after_pos1]; //element from r2 to r1
					cost_with_j = dist[before_pos2][element1] + dist[element1][after_pos2] + service_time[element1]- dist[before_pos2][after_pos2]; //element from r1 to r2

					//cost_with_i is R1 //cost_with_j is R2 
					if ((cost_with_i > available_dist_r1 + cost_without_i) || (cost_with_j > available_dist_r2 + cost_without_j))
						continue;

					gain = (cost_without_i + cost_without_j) - (cost_with_i + cost_with_j);
							
					if (gain > epsilonn)
					{
						foundimprovementstatus = 1;
						kGain[set][0] = set;
						kGain[set][1] = gain;
						kGain[set][2] = i; //from r1,
						kGain[set][3] = m; //to_r2
						kGain[set][4] = j; //from r1_p
						kGain[set][5] = n; //to_r2_p //to which position (swap position)=route is not modified
						kGain[set][6] = n; //from_r2_p
						kGain[set][7] = j; //to_r1_p
						kGain[set][8] = 2; //2 is (1-1)
						kGain[set][10] = cost_without_i- cost_with_i;//gain1

						set++;
						operand[level].eachRfullsearchstatus[i][m] = 0;

					}//end if
					if (set == K)
						goto find_best_k2;
				}//end for n
			}//end for j
			if(foundimprovementstatus == 0)
				operand[level].eachRfullsearchstatus[i][m] = 1 ;
		}//end for m
	}//end for i 

find_best_k2:;
	
	int bestK = -1; //initialize
	float maxGain = INT_MIN;
	for (int i = 0; i < K; i++)
	{
		if ((kGain[i][1] > maxGain) && (kGain[i][1] >= epsilonn))
		{
			maxGain = kGain[i][1];
			bestK = i;
		}

	}
	//if (bestK != -1)
	//{
	//	operand[level].foundImprove = 1;
	//	globalinfo.Rchanged[0] = kGain[bestK][2];
	//	globalinfo.Rchanged[1] = kGain[bestK][3];
	//	//fulls << "In, find_swap_1_1, level= "<< level<<' '<<"operand[level].foundImprove = 1"<<endl;
	//	//fulls <<"globalinfo.Rchanged[0]= "<<globalinfo.Rchanged[0]<<' '<<"globalinfo.Rchanged[1]= "<<globalinfo.Rchanged[1]<<endl<<endl;
	//}
	
	return bestK;
}

//DONE
int find_k_2_2swap(float *cost_of_removing, float **(&kGain), int K, int level) //K is kth level of improvement
{
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::app);
	
	int no_of_attribute = 13;
	for (int i = 0; i < K; i++)
	{
		for (int j = 0; j < no_of_attribute; j++)
		{
			kGain[i][j] = -1; //reinitialize
		}
	}
	//int no_routes = LOCAL_NO_ROUTE;
	int element1, element2, element1f, element2f, before_ele1, before_ele2, after_ele1, after_ele2;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_j, cost_with_i_Reverse, cost_with_j_Reverse, cost_with_both_Reverse, gain, dist_gain;
	float dist_without_i, dist_without_j, dist_with_i, dist_with_j, dist_with_i_Reverse, dist_with_j_Reverse, dist_with_both_Reverse, gain_with_i_Reverse, gain_with_j_Reverse, gain_with_both_Reverse;
	int reverseType = 0;
	
	////// ======================== Find and Record the kth best in kGain matrix =====================//
	int set = 0;
	
	for (int i = 0; i < no_routes-1; i++)  //consider each and other routes (from R1)//changed to no_routes-1 on 6April2015
	{
		if (saiz[i] <= 1) //if it has 1 customer or less
			continue;
		//====================================== same route ==================================//
		
		if (operand[level].eachRfullsearchstatus[i][i] == 1 || saiz[i]<=3)
			goto endofsameroute;

		for (int j = 1; j <= saiz[i]-2 ; j++) //from which position
		{
			element1 = VEHICLE[i][j];
			element1f = VEHICLE[i][j+1];
			before_ele1 = VEHICLE[i][j-1];
			after_ele1 = VEHICLE[i][j+2];
			for (int k = j+2; k <= saiz[i]-1; k++)
			{
				element2 = VEHICLE[i][k];
				element2f = VEHICLE[i][k+1];
				before_ele2 = VEHICLE[i][k-1];
				after_ele2 = VEHICLE[i][k+2];

				
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
				int n = saiz[i];
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
					
				if (gain > epsilonn)
				{
					kGain[set][0] = set;
					kGain[set][1] = gain;
					kGain[set][2] = i; //from r1,
					kGain[set][3] = i; //to_r2
					kGain[set][4] = j; //from r1_p
					kGain[set][5] = k; //to_r2_p //to which position //assume modified route, customer has been deleted
					kGain[set][6] = k; //from_r2_p
					kGain[set][7] = j; //to_r1_p
					kGain[set][8] = 5; //5 is (2-2swap)
					kGain[set][9] = reverseType; 
					kGain[set][10] = gain;//gain1
					kGain[set][11] = dist_gain; //gain1 in distance
					set++;
					operand[level].eachRfullsearchstatus[i][i] = 0;
							
				}	

				if (set == K) //set starts from 0 => set=3 means 0, 1, 2 sets
				{
					goto find_best_k5;
				}
			}	
		}//end of j
				
endofsameroute:

		for (int m = i+1; m < no_routes; m++)  //insert (to) which route (to R2) //changed m=i+1 on 6April2015
		//for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
		{
			if ((i == m) || (saiz[m] <= 1)) //if same route, already consider
			{
				continue;
			}
			int foundimprovementstatus=0;//to indicate improvement found or not for this route, to flag the operand[1].eachRfullsearchstatus[i] at the end
			if (operand[level].eachRfullsearchstatus[i][m] == 1 )
			{
				//route_file<<" Omit ["<<i<<"]["<<m<<"]"<<' '<<' '<<' '<<' ';
				continue;
			}
			

			//======================================= different route ============================//
			for (int j = 1; j <= saiz[i]-1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
			//for (int m = i+1; m < no_routes; m++)  //insert (to) which route (to R2) //changed m=i+1 on 6April2015
			{
				element1 = VEHICLE[i][j];			//element to be inserted into other route
				element1f = VEHICLE[i][j+1];			//element to be inserted into other route
				before_ele1 = VEHICLE[i][j-1];
				after_ele1 = VEHICLE[i][j+2];



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
							
					if (gain > epsilonn)
					{
						foundimprovementstatus = 1;
						kGain[set][0] = set;
						kGain[set][1] = gain;
						kGain[set][2] = i; //from r1,
						kGain[set][3] = m; //to_r2
						kGain[set][4] = j; //from r1_p
						kGain[set][5] = n; //to_r2_p //to which position (swap position)=route is not modified
						kGain[set][6] = n; //from_r2_p
						kGain[set][7] = j; //to_r1_p
						kGain[set][8] = 5; //5 is (2-2swap)
						kGain[set][9] = reverseType;
						kGain[set][10] = cost_without_i- cost_with_i;//gain1
						kGain[set][11] = dist_without_i - dist_with_i;//gain1 in distance
						kGain[set][12] = dist_without_j - dist_with_j;//gain2 in distance
						set++;
						operand[level].eachRfullsearchstatus[i][m] = 0;

					}//end if
					if (set == K)
						goto find_best_k5;
				}//end for n
			}//end for j
			if(foundimprovementstatus == 0)
				operand[level].eachRfullsearchstatus[i][m] = 1 ;
		}//end for m
	}//end for i 

find_best_k5:;
	
	int bestK = -1; //initialize
	float maxGain = INT_MIN;
	for (int i = 0; i < K; i++)
	{
		if ((kGain[i][1] > maxGain) && (kGain[i][1] >= epsilonn))
		{
			maxGain = kGain[i][1];
			bestK = i;
		}

	}

	return bestK;
}




//DONE
int find_two_optintra(float **(&kGain), int K, int level) 
{
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::app);
	
	int no_of_attribute = 13;
	for (int i = 0; i < K; i++)
	{
		for (int j = 0; j < no_of_attribute; j++)
		{
			kGain[i][j] = -1; //reinitialize
		}
	}

	////// ======================== Find and Record the kth best in kGain matrix =====================//
	int set = 0;

	//bool *flag_routeChange = new bool[no_routes];//to calculate how many route have been changed at the end, return to original function how many route have been changed
	//for (int i = 0; i < no_routes; i++)
	//{
	//	flag_routeChange[i] = false; //initially, all routes have not been changed, flag false
	//}
	
	for (int i = 0; i < no_routes; i++)  //from route 0 to last route
	{
		float new_cost = 0.0;
		
		if (operand[level].eachRfullsearchstatus[i][i] == 1 || saiz[i] <= 2) //changed 28Jun2015, only route size > 2 can do 2-opt, if size=2, front and back is depot, so it is the same
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
			
				if (gain > epsilonn)
				{
					kGain[set][0] = set;
					kGain[set][1] = gain;
					kGain[set][2] = i; //r1,
					kGain[set][3] = i; //r2
					kGain[set][4] = m; //beginning of subroute
					kGain[set][5] = n; //end of subroute
					kGain[set][6] = -1; 
					kGain[set][7] = -1; 
					kGain[set][8] = 6; //6 is (2optintra)
					//kGain[set][9] = reverseStatus; //no reverse status
					kGain[set][10] = gain;//gain1
					kGain[set][11] = origain; //gain1 in distance
					set++;
					operand[level].eachRfullsearchstatus[i][i] = 0;
	
				}	
						
				if (set == K) //set starts from 0 => set=3 means 0, 1, 2 sets
				{
					goto find_best_k6;
				}

			}			
		}
	}

find_best_k6:
	int bestK=-1; //initialize
	float maxGain = INT_MIN;
	for (int i = 0; i < K; i++)
	{
		if ((kGain[i][1] > maxGain) && (kGain[i][1] >= epsilonn))
		{
			maxGain = kGain[i][1];
			bestK = i;
		}
	}
	return bestK;	
}
//DONE
int find_two_optinter(float **(&kGain), int K, int level) //return number of route change for updating cost_of_removing in function call
{
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::app);

	int no_of_attribute = 13;
	for (int i = 0; i < K; i++)
	{
		for (int j = 0; j < no_of_attribute; j++)
		{
			kGain[i][j] = -1; //reinitialize
		}
	}
	int reverseStatus;
	
	////// ======================== Find and Record the kth best in kGain matrix =====================//
	int set = 0;

	std::vector<int> myvector2; 

	for (int i = 0; i < no_routes; i++)  //from route 0 to last route
	{
		if (saiz[i] == 0) 
			continue;
		
		for (int m = 0; m < no_routes; m++)//from route 0 to last route
		{
			if ((saiz[m] == 0) || (i==m))
				continue;
			
			int foundimprovementstatus=0;//to indicate improvement found or not for this route, to flag the operand[1].eachRfullsearchstatus[i] at the end
			if (operand[level].eachRfullsearchstatus[i][m] == 1 )
			{
				//route_file<<" Omit ["<<i<<"]["<<m<<"]"<<' '<<' '<<' '<<' ';
				continue;
			}

			for (int j = 1; j <= saiz[i]; j++) //for the length of first route //j++ means delete less in r1 because delete from j to saiz
			{
				//nextR2:
				int startEle = VEHICLE[i][j];
				int ele1before = VEHICLE[i][j-1];

				for (int n = 1; n <= saiz[m]; n++)//for the length of second route //n++ means delete more in r2 because delete from 1 to n
				{
					reverseStatus = 0;//initialize to 0
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
					int before2 = SIZE;//before element is depot
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

					//if r1 too full, goto next r2, r2 starts from delete less
					if (r2demandDelete > space_available[i] + r1demandDelete) //added on 12Nov2015 
					{
						continue;
						//goto next_r2; //if exceed capacity of r1, goto next r2, dont consider more customer in r2, this will exceed more
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
	

					if (gain > epsilonn)
					{
						foundimprovementstatus = 1;
						kGain[set][0] = set;
						kGain[set][1] = gain;
						kGain[set][2] = i; 
						kGain[set][3] = m; 
						kGain[set][4] = j; //the element considered in r1 (begin of suboute)
						kGain[set][5] = n; //the element considered in r2 (end of suboute)
						kGain[set][6] = -1; 
						kGain[set][7] = -1; 
						kGain[set][8] = 7; //7 is (2optinter)
						//kGain[set][9] = reverseStatus; //no need reverse order
						kGain[set][10] = cost_without1 - cost_with1;//gain1
						kGain[set][11] = dist_without1 - dist_with1;//gain1 in distance
						kGain[set][12] = dist_without2 - dist_with2;//gain2 in distance
						set++;
						operand[level].eachRfullsearchstatus[i][m] = 0;
					}	

					if (set == K) //set starts from 0 => set=3 means 0, 1, 2 sets
					{
						goto find_best_k7;
					}					
				}//end of n
			}//end of j
			next_r2:;//if demand deleted in r2 exceeded demand in r1, dont delete anymore in r2 because this cannot be put in r1 //added 12Nov2015
			if(foundimprovementstatus == 0)
				operand[level].eachRfullsearchstatus[i][m] = 1 ;
		}//end of for m
	}//end of i

find_best_k7:

	int bestK=-1; //initialize
	float maxGain = INT_MIN;
	for (int i = 0; i < K; i++)
	{
		if ((kGain[i][1] > maxGain) && (kGain[i][1] >= epsilonn))
		{
			maxGain = kGain[i][1];
			bestK = i;
		}
	}
	return bestK;
}

//crossTail2 consider reverse order of tail
//Route 1: Take from j until the last
//Route 2: Take from n until the last
//must consider i=0,...,no_routes-1, m=i+1,...no_routes
//DONE
int find_crossTail2(float **(&kGain), int K, int level) //return number of route change for updating cost_of_removing in function call
{
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::app);
	int no_of_attribute = 13;
	for (int i = 0; i < K; i++)
	{
		for (int j = 0; j < no_of_attribute; j++)
		{
			kGain[i][j] = -1; //reinitialize
		}
	}
	int reverseStatus;
	
	////// ======================== Find and Record the kth best in kGain matrix =====================//
	int set = 0;

	std::vector<int> tail1; 
	std::vector<int> tail2; 
	//std::vector<int> tail1Rev; 
	//std::vector<int> tail2Rev; 

	for (int i = 0; i < no_routes-1; i++)  //from route 0 to last route-1 because consider 2 routes at a time
	{
		if (saiz[i] == 0) 
			continue;
		
		for (int m = i+1; m < no_routes; m++)//from route i+1 to last route //the start of tail2
		{
			if ((saiz[m] == 0) || (i==m))
				continue;

			int foundimprovementstatus=0;//to indicate improvement found or not for this route, to flag the operand[1].eachRfullsearchstatus[i] at the end
			if (operand[level].eachRfullsearchstatus[i][m] == 1 )
			{
				//route_file<<" Omit ["<<i<<"]["<<m<<"]"<<' '<<' '<<' '<<' ';
				continue;
			}

			for (int j = 1; j <= saiz[i]; j++) //the start of tail1 //j++ means delete less in r1
			{
				int startTail1 = VEHICLE[i][j];
				int endTail1 = VEHICLE[i][saiz[i]];
				//tail1.clear();
				

				for (int n = 1; n <= saiz[m]; n++)//the start of tail2 //n++ means delete less in r2
				{
					reverseStatus = 0;//initialize to 0
					if ((j==1) && (n==1))//skip if both start at 1, there is no head for both
						continue;
					//if ((j==saiz[i]+1) && (n==saiz[m]+1))//skip if both take full route, there is no tail for both
					//	continue;
					tail1.clear();
					tail2.clear();

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
						//tail1Rev.push_back(tail1[k]);//push in reverse order, so noneed to use reverse function
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
						//tail2Rev.push_back(tail2[k]);//push in reverse order, so noneed to use reverse function
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
					if (gain > epsilonn)
					{
						foundimprovementstatus = 1;
						kGain[set][0] = set;
						kGain[set][1] = gain;
						kGain[set][2] = i; 
						kGain[set][3] = m; 
						kGain[set][4] = j; //the element considered in r1 (begin of tail1)
						kGain[set][5] = n; //the element considered in r2 (end of tail2)
						kGain[set][6] = -1; 
						kGain[set][7] = -1; 
						kGain[set][8] = 8; //8 is (crosstail)
						kGain[set][9] = reverseType; 
						kGain[set][10] = cost_of_removing1 - cost_of_inserting1;//gain1
						kGain[set][11] = dist_without1 - dist_with1;//gain1 in distance
						kGain[set][12] = dist_without2 - dist_with2;//gain2 in distance
						set++;
						operand[level].eachRfullsearchstatus[i][m] = 0;
					}	

					if (set == K) //set starts from 0 => set=3 means 0, 1, 2 sets
					{
						goto find_best_k8;
					}					
				}//end of n
				increasej:;
			}//end of j
			if(foundimprovementstatus == 0)
				operand[level].eachRfullsearchstatus[i][m] = 1 ;
		}//end of for m
	}//end of i

	find_best_k8:

	int bestK=-1; //initialize
	float maxGain = INT_MIN;
	for (int i = 0; i < K; i++)
	{
		if ((kGain[i][1] > maxGain) && (kGain[i][1] >= epsilonn))
		{
			maxGain = kGain[i][1];
			bestK = i;
		}
	}
	return bestK;
}

int find_CROSS(float **(&kGain), int K, int level) //return number of route change for updating cost_of_removing in function call
{
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::app);

	int no_of_attribute = 13;
	for (int i = 0; i < K; i++)
	{
		for (int j = 0; j < no_of_attribute; j++)
		{
			kGain[i][j] = -1; //reinitialize
		}
	}
	int reverseStatus;
	
	////// ======================== Find and Record the kth best in kGain matrix =====================//
	int set = 0;

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

			int foundimprovementstatus=0;//to indicate improvement found or not for this route, to flag the operand[1].eachRfullsearchstatus[i] at the end
			if (operand[level].eachRfullsearchstatus[i][m] == 1 )
			{
				continue;
			}

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
					reverseStatus = 0;//initialize to 0
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

					if (gain > epsilonn)
					{
						foundimprovementstatus = 1;
						kGain[set][0] = set;
						kGain[set][1] = gain;
						kGain[set][2] = i; 
						kGain[set][3] = m; 
						kGain[set][4] = j; //the start of segmengt1
						kGain[set][5] = n; //the start of segmengt2 
						kGain[set][6] = -1; 
						kGain[set][7] = -1; 
						kGain[set][8] = 9; //9 is (cross)
						kGain[set][9] = reverseType; 
						kGain[set][10] = old_cost1 - new_cost1;//gain1
						set++;
						operand[level].eachRfullsearchstatus[i][m] = 0;
					}	

					if (set == K) //set starts from 0 => set=3 means 0, 1, 2 sets
					{
						goto find_best_k9;
					}						
				}//end of n
			}//end of j
			if(foundimprovementstatus == 0)
				operand[level].eachRfullsearchstatus[i][m] = 1 ;
		}//end of for m
	}//end of i

find_best_k9:
	int bestK=-1; //initialize
	float maxGain = INT_MIN;
	for (int i = 0; i < K; i++)
	{
		if ((kGain[i][1] > maxGain) && (kGain[i][1] >= epsilonn))
		{
			maxGain = kGain[i][1];
			bestK = i;
		}
	}
	return bestK;
}


void adaptiveVNS_kth_improvement(float *x, float *y, float *(freq)) 
{
	//ofstream recordTIME("TIME.txt", ios::app);
	//ofstream violation("Infeasible_" + std::to_string( I ) + ".txt", ios::app);
	ofstream timefile("16.Time_" + std::to_string( I ) + ".txt", ios::app);
	ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
	ofstream localbest_sol("30.LOCAL_BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
	//ofstream recordFreq("27.Frequency_of_module_" + std::to_string( I ) + ".txt", ios::app);
	//ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);
	
	//ofstream EV_learning ("1.Learning_" + std::to_string( I ) + ".csv");
	//ofstream EV_diver ("1.Diver_" + std::to_string( I ) + ".csv", ios::app);
	int Total_LSmove=0;//to evaluate effect of learning


	//route_file<<"============================ START OF ADAPTVE VNS =================================="<<endl;
	float start_s = clock();// the code you wish to time goes here

	//******************************************** STEP 1: INITIALIZATION ********************************************//
	//============================================ all predefined variables =================================================//
	int max_neigbor = 6; //6 modules of shake
	//int max_level = 6; //6 levels of local search 
	//int maxLocalSearch = max_level-1; //excluding last LS:2-opt for freq use
	int maxLocalSearch = 9;//9 levels of local search //ONLY 7 will be performed
	int NbDiv = 0;
	int maxDiv = min(3, GLOBAL_NO_ROUTE/2);
	int LB_Ndel_min = max (15.00, (0.15*SIZE));
	//float siz=SIZE;
	int UB_Ndel_min = min (400.00, (0.5*SIZE));
	int Ndel_min = LB_Ndel_min; //min number of iteration for each route for diversification
	int RS = 6; //number of removal strategies
	//int divsequence[5];//randomize the diversification strategy
	int *divsequence = new int[RS];//randomize the diversification strategy
	for (int i = 0; i < RS; i++)
	{
		divsequence[i]=i;//put in 0 to 5
	}
	std::srand ( std::time(0) );
	std::random_shuffle ( &divsequence[0], &divsequence[RS-1] );//randomize the sequence
	
	//for (int i = 0; i < RS; i++)
	//{
	//	cout<<divsequence[i]<<' ';//put in 0 to 4
	//}
	
	//EV_diver<<"maxDiv= "<<maxDiv<<endl;
	//LNSremoval *strategy = new LNSremoval[RS];
	//strategy[0].cutpoints = 1.00/RS;//first cutpoint is 0.25
	//for (int i = 0; i < RS; i++)
	//{ 
	//	strategy[i].probability = 1.00/RS; //initially all equal probability
	//	if (i!=0) //first cut point already determined
	//		strategy[i].cutpoints += strategy[i-1].cutpoints;
	//}

	//============================================= all predefined variables ================================================//

	int TOTAL_DEMAND=0;
	for (int i = 0; i < SIZE; i++)
	{
		TOTAL_DEMAND = TOTAL_DEMAND+demand[i];
	}//just to chack the overall demand is correct at the end of VNS multi-level

	
	
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
	//while (Total_LSmove<=20000)
	while (NbDiv <= maxDiv) //#############################################################################################################################
	{
		float violateThreshold = INFEASIBLE; //reinitialize to original value after diversification
		int found_GLOBEST_status = 0;//must put before infeasibility check, otherwise found_GLOBEST_status is not recognise after goto violateDiverstart
		//route_file<<"Adaptive NbDiv= "<<NbDiv<<endl;
		
		int no_empty_route=0;

		cout<<"LOCAL_NO_ROUTE after restartAfterDiver= "<<LOCAL_NO_ROUTE<<endl;
		//route_file<<"LOCAL_NO_ROUTE after restartAfterDiver= "<<LOCAL_NO_ROUTE<<endl;
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			cout<<i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
			//route_file << i <<' '<< LOCAL_Rcost[i] <<' '<< LOCAL_SAIZ[i] <<' '<< LOCAL_capa[i]<<' '<<' ';
			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
			{
				//route_file<<LOCAL[i][j]<< ' ';
				cout<<LOCAL[i][j]<< ' ';
			}
			cout<<endl;
			//route_file<<endl;
		}
		cout<<"LOCAL_BEST_COST= "<<LOCAL_BEST_COST<<endl;
		//route_file<<"LOCAL_BEST_COST= "<<LOCAL_BEST_COST<<endl;

		//******************************************** STEP 2: DELETE EMPTY ROUTE ********************************************//
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			if (LOCAL_SAIZ[i] == 0)
				no_empty_route++;
		}
		
		if (no_empty_route > 1) //if more than 1 empty route, do not copy the empty route and add one at the end
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
		cout<<"LOCAL_NO_ROUTE after delete empty route= "<<LOCAL_NO_ROUTE<<endl;
		//route_file<<"LOCAL_NO_ROUTE after delete empty route= "<<LOCAL_NO_ROUTE<<endl;
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			cout << i <<' '<< LOCAL_Rcost[i] <<' '<< LOCAL_SAIZ[i] <<' '<< LOCAL_capa[i]<<' '<<' ';
			//route_file << i <<' '<< LOCAL_Rcost[i] <<' '<< LOCAL_SAIZ[i] <<' '<< LOCAL_capa[i]<<' '<<' ';
			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
			{
				cout<<LOCAL[i][j]<< ' ';
				//route_file<<LOCAL[i][j]<< ' ';
			}
			cout<<endl;
			//route_file<<endl;
		}

		cout<<"LOCAL_NO_ROUTE= "<< LOCAL_NO_ROUTE<<endl;
		//route_file<<"LOCAL_NO_ROUTE= "<< LOCAL_NO_ROUTE<<endl;
		
		no_routes = LOCAL_NO_ROUTE;
		float *cost_of_removing = new float[SIZE + 1];//cost_of_removing[][] only used in LS, not shaking
		float *Oricost_of_removing = new float[SIZE + 1];

		//================= Record route change status from shaking and best improvement======================//added 15Sept2015
		//Route = new Rmodified[LOCAL_NO_ROUTE];
		RchangedStatus = new bool[LOCAL_NO_ROUTE];
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			//Route[i].RchangedStatus=0;//initialize to 0 for every route, they have not been changed
			RchangedStatus[i]=false;//initialize to 0 for every route, they have not been changed
		}
		//CustaffectedStatus = new bool[SIZE+1];
		//for (int i = 0; i <= SIZE; i++)
		//{
		//	CustaffectedStatus[i] = false;//initialize to 0 for every route, they have not been changed
		//}
		
		//============= Copy VEHICLE from LOCAL_BEST ===========// guided shake needs to use VEHICLE 
		bool copyalready; //to avoid copy vehicle from local_best again in while loop for the first time
		
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)//added 22Apr15
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
		no_routes=LOCAL_NO_ROUTE;//added 24 April 2016
		copyalready = true; 
		//============= End of Copy VEHICLE from LOCAL_BEST ===========//
		
		find_all_cost_of_removing (cost_of_removing);//cost_of_removing[][] only used in LS, not shaking, this is the first time calculating cost_of_removing
		for (int f = 0; f < SIZE; f++) //copy from cost_of_removing
		{
			 Oricost_of_removing[f] = cost_of_removing[f];
		}
		int Neighbour_k = 1; //Neighbour_k 1 = (1-0), Neighbour_k 2 = (1-1), Neighbour_k 3 = (2-1), Neighbour_k 4 = (2-0)
	
		int total_time = 0;
		float maxCPUtime = 10000/(CLOCKS_PER_SEC);
		cout << "maxCPUtime= " <<maxCPUtime<<endl;
	
	//###################################### STEP 3(a) Shaking ######################################//
	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ k+1 neighborhood starts here @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
	bool foundLOCALinCurrNeighbor = false;//Do Dijkstra refinement if found LOCAL BEST in any neighbourhood in this diversification, otherwise, just do refinement for VEHICLE
	

	while (Neighbour_k <= max_neigbor)	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	{	
		//module: //1) after local search if no improvement, Neighbour_k++ goto module //this is k+1 in the algorithm (move to next neighborhood)
		if (violateThreshold < 0.01) //if smaller than 1%, make it zero
			violateThreshold = 0;

		route_CGravity = new float*[LOCAL_NO_ROUTE];
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			route_CGravity[i] = new float[2]; //2 columns consists of x and y
		}
		custRGravity = new float[LOCAL_NO_ROUTE];
		sorted_custRGravity = new float*[LOCAL_NO_ROUTE];
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			custRGravity[i] = 0; //initialize
			sorted_custRGravity[i] = new float[5]; //[0]sorted distance, [1]sorted route index, [2]1/distance, [3]d/sum(d), [4]cumulative 
		}

	//reshake: //if no feasible shake, Neighbour_k++ and goto reshake //put comment on 14Oct2015, put reshake after copy
		//for (int i = 0; i < 5; i++) //reinitialize after_shake_route_change[6]
		//{
		//	after_shake_route_change[i] = -1;
		//}
		//after_shake_route_changePtr = 0; //reinitialize pointer to 0, to keep track how many routes have been changed

		//VEHICLE copied from LOCAL_BEST_ROUTE to be passed to shake(), need to recopy because when neighbourhoood++, VEHICLE[][] need to recopy from LOCAL_BEST although the first iteration this is not necessary because VEHICLE already copy before	
		if (copyalready == false)//if first time, VEHICLE already copy from LOCAL_BEST, noneed to copy, added on 12Oct2015
		{
			no_routes = LOCAL_NO_ROUTE;
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)//added 22Apr15 
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

	reshake: //if no feasible shake, Neighbour_k++ and goto reshake
		//for (int i = 0; i < 5; i++) //reinitialize after_shake_route_change[6]
		//{
		//	after_shake_route_change[i] = -1;
		//}
		//after_shake_route_changePtr = 0; //reinitialize pointer to 0, to keep track how many routes have been changed
		
		int shake_status1=1;
		bool deletKGain=true;//to indicate delete kGain, otherwise there will be run time error because it has not been declared yet
		//int shake_status2=1;
		//int shake_status3=1;

		if (Neighbour_k == 1) //(1-0)
		{
			//cout<<"Entering Neighbour_k == 1"<<endl;
			//route_file<<"Entering Neighbour_k == 1"<<endl;
			shake_status1 = shake_2_1_twoR(VEHICLE, x, y); //the one going to shake is always the current best solution
			//shake_status2 = shake_1_0(VEHICLE, x, y); //the one going to shake is always the current best solution
			//shake_status3 = shakeCROSS(VEHICLE, x, y);
			//if ((shake_status1 == 0) && (shake_status2 == 0))
			//if ((shake_status1 == 0) && (shake_status2 == 0) && (shake_status3 == 0))
			shake_status1 = shake_intraSegmentReshuffle(VEHICLE);
			if (shake_status1 == 0) 
			{
				shake_reshuffle(VEHICLE);
				//Neighbour_k++;
				//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
			}
		}	
		if (Neighbour_k == 2) //(1-0)
		{
			
			//shake_status1 = shake_intraReverseSegment(VEHICLE, x, y); //the one going to shake is always the current best solution
			shake_status1 = shake_intraSegmentReshuffle(VEHICLE);
			shake_status1 = shake_intraHeadReshuffle(VEHICLE, Neighbour_k);
			if (shake_status1 == 0) 
			{
				shake_reshuffle(VEHICLE);
				//Neighbour_k++;
				//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
			}
		}	
		else if (Neighbour_k == 3) //1-1
		{	
			//cout<<"Entering Neighbour_k == 2"<<endl;
			//route_file<<"Entering Neighbour_k == 2"<<endl;
			shake_status1 = shake_2_0_threeR(VEHICLE, x, y);
			shake_status1 = shake_intraHeadReshuffle(VEHICLE, Neighbour_k);
			//shake_status2 = shakeCROSS(VEHICLE, x, y);
			//shake_status3 = shakeCROSS(VEHICLE, x, y);
			//if (shake_status1 == 0)
			if (shake_status1 == 0) 
			{
				shake_reshuffle(VEHICLE);
				//Neighbour_k++;
				//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
			}
		}
		else if (Neighbour_k == 4) //(2-0) - a pair from route A go to route B
		{			
			//cout<<"Entering Neighbour_k == 3"<<endl;
			//route_file<<"Entering Neighbour_k == 3"<<endl;
			shake_status1 = shake_2_1_threeR(VEHICLE, x, y);
			shake_status1 = shake_intraSegmentReshuffle(VEHICLE);
			//shake_status2 = shake_1_1(VEHICLE, x, y);
			//shake_status3 = shakeCROSS(VEHICLE, x, y);
			//if ((shake_status1 == 0) && (shake_status2 == 0))
			if (shake_status1 == 0) 
			{
				shake_reshuffle(VEHICLE);
				//Neighbour_k++;
				//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
			}
		}
		else if (Neighbour_k == 5) //(2-0) - a pair from route A go to route B and route C
		{			
			//cout<<"Entering Neighbour_k == 4"<<endl;
			//route_file<<"Entering Neighbour_k == 4"<<endl;
			shake_status1 = shake_2_2(VEHICLE, x, y);
			shake_status1 = shake_reshuffle(VEHICLE);
			//shake_status2 = shakeCROSS(VEHICLE, x, y);
			//shake_status3 = shakeCROSS(VEHICLE, x, y);
			//if (shake_status1 == 0)
			if (shake_status1 == 0) 
			{
				shake_reshuffle(VEHICLE);
				//Neighbour_k++;
				//goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
			}
		}
		else if (Neighbour_k == 6) //(2-1) - a pair from route A swap with one customer from route B
		{			
			//cout<<"Entering Neighbour_k == 5"<<endl;
			//route_file<<"Entering Neighbour_k == 5"<<endl;
			//shake_status1 = shake_2_2(VEHICLE, x, y);
			//shake_status2 = shake_1_0(VEHICLE, x, y);
			shake_status1 = shakeCROSS(VEHICLE, x, y);
			shake_status1 = shake_reshuffle(VEHICLE);
			//if ((shake_status1 == 0) && (shake_status2 == 0))
			if (shake_status1 == 0)  
			{
				shake_reshuffle(VEHICLE);
				//Neighbour_k++;
				//deletKGain=false;
				//goto check_LOCAL_BEST; //if no feasible shake at last  neighborhod, dont perform LS, goto check_LOCAL_BEST, but need to copy vehicDepoCust[][] from vehicle[][] //added 15June2015 //check this if correct
				//!!!!!!!!!!!!!because of this, kGain is not declared yet, so evrytime try to delete it will have run time error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!//
			}
		}

		//else if (Neighbour_k == 6) //(2-1) - first pair of cust must from route A to B, another cust from route A can go to route C
		//{			
		//	//cout<<"Entering Neighbour_k == 6"<<endl;
		//	//route_file<<"Entering Neighbour_k == 6"<<endl;
		//	shake_status1 = shake_2_1_threeR(VEHICLE, x, y);
		//	shake_status2 = shake_1_0(VEHICLE, x, y);
		//	shake_status3 = shakeCROSS(VEHICLE, x, y);
		//	//if ((shake_status1 == 0) && (shake_status2 == 0))
		//	if ((shake_status1 == 0) && (shake_status2 == 0) && (shake_status3 == 0))
		//	{
		//		Neighbour_k++;
		//		goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
		//	}
		//}
		//else if (Neighbour_k == 7) //(2-2) 
		//{
		//	//cout<<"Entering Neighbour_k == 7"<<endl;
		//	//route_file<<"Entering Neighbour_k == 7"<<endl;
		//	shake_status1 = shake_2_2(VEHICLE, x, y);
		//	shake_status3 = shakeCROSS(VEHICLE, x, y);
		//	//if (shake_status1 == 0)
		//	if ((shake_status1 == 0) && (shake_status3 == 0))
		//	{
		//		Neighbour_k++;
		//		//goto check_sol; //if no feasible shake at last neighborhod, dont perform LS, goto check_sol
		//								//VEHICLE copied from vehicle to be passed to reoptimize() later //made comment on 15June2015
		//		goto check_LOCAL_BEST; //if no feasible shake at last  neighborhod, dont perform LS, goto check_LOCAL_BEST, but need to copy vehicDepoCust[][] from vehicle[][] //added 15June2015 //check this if correct
		//	}
		//			
		//}
		//route_file<<"Route changed are: "<< ' ';
		for (int r = 0; r < no_routes; r++)
		{
			cout<<"Route changed are: "<< ' ';
			if (RchangedStatus[r] == true)
			{
				cout<<r<<' ';
				//route_file<<r<<' ';
			}		
			cout<<endl; 
		}
		cout<<"After shake " <<Neighbour_k <<endl;
		//route_file<<"After shake " <<Neighbour_k <<endl;
		cout<<endl<<"================================"<<endl;
		//route_file<<endl<<"================================"<<endl;
		cout<<"VEHICLE after shake " <<endl;
		//route_file<<"VEHICLE after shake " <<endl;
		for (int i = 0; i < no_routes; i++)
		{
			cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
			//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
			for (int j = 0; j <= saiz[i]+1; j++)
			{
				cout<<VEHICLE[i][j]<<' ';
				//route_file<<VEHICLE[i][j]<<' ';
			}
			cout<<endl;
			//route_file<<endl;
		}
		
		float afterShake_cost = 0.0; //to record the current cost so that later after kth improvement can check if the solution got improved
		for (int i = 0; i < no_routes; i++)
		{
			afterShake_cost = afterShake_cost + route_cost[i];
		}
		cout<<"afterShake_cost= "<<afterShake_cost<<endl;
		//route_file<<"afterShake_cost= "<<afterShake_cost<<endl;

		//========================= To update cost_of_removing, gain_matrix[] and info-matrix[][] after shake===============================//
		partialupdate_costremoving(cost_of_removing);
		reinitializeRchangedStatus (); //initialize to 0 for every route

		//=========== declare data structure ======================//
		operand = new fullsearch[maxLocalSearch+1]; //there are (maxLocalSearch) of operand that contains info in struct , maxLocalSearch start from 1
		for (int i = 1; i <= maxLocalSearch; i++)
		{
			operand[i].eachRfullsearchstatus = new int* [no_routes];
		}
			
		for (int i = 1; i <= maxLocalSearch; i++)
		{
			for (int j = 0; j < no_routes; j++)
			{
				operand[i].eachRfullsearchstatus[j] = new int [no_routes];
			}
		}
		initializeEachRfullsearchstatus (maxLocalSearch);

		/////////////// ================================ INITIALISATION FOR LS ==================================================== ////////////////////////////
		int minLS = 3;
		int maxLS = 5;
		int Kth = 3; //kth improvement, find the best value from K gain!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		int level = 1; //initialize local search to 1
		bool sameroute = false; //false if different route, true if same route
		int no_attribute = 13; //[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
	
		float **kGain = new float* [Kth];
		for (int i = 0; i < Kth; i++)
		{
			kGain[i] = new float [no_attribute];
		}

		//Adaptive goes here, generate randNum for module, then generate the module and sequence
		//adjustFreq(freq, maxLocalSearch); //if probability = 0, add 0.05 so that it will have a small chance to be selected
		//cout<<"=========== Frequency of gain in Multi-level ============"<<endl;
		//recordFreq<<"============ Frequency of gain IN MULTI_LEVEL ==============="<<endl;
		//for (int i = 1; i <= maxLocalSearch; i++) //freq[i] starts from 1
		//{
		//	//cout<<"Freq["<<i<<"]= "<<freq[i]<<' '<<' ';
		//	recordFreq<<"Freq["<<i<<"]= "<<freq[i]<<' '<<' ';
		//}
		//recordFreq<<endl;

		float *cutPoint = new float[maxLocalSearch+1]; //from 0 to 1 inclusive // 10 modules, 11 cutpoint
		int rand_numLS=0; //number of LS
		bool *flag_m = new bool[maxLocalSearch+1]; // starts from 1 //flag which module have been asisgned
		for (int i = 0; i <= maxLocalSearch; i++)//flag_m[0] has no meaning because modul starts from 1
		{
			flag_m[i]=false; //initially all false
		}
		//int *modu = new int[rand_numLS+2]; //modul[] starts from 1;//+1 to add last module 2-opt, compulsory LS //cannot declare here because rand_numLS is unknown!! cause heap error when delete!!!!!!!!!!!!
		/////////////// ================================ End of INITIALISATION FOR LS ==================================================== ///

		//determine how many module
		srand ( time(NULL) ); //seed it
	
		rand_numLS = (rand() % (maxLS-minLS+1))+minLS; //generate number of LS between minLS and maxLS
		//rand_numLS = (rand() % maxLocalSearch)+1; //generate number of LS 
		int *modu = new int[rand_numLS+1]; //modul[] starts from 1;
		//cout<<"number of LS = " <<rand_numLS<<endl;
		
		//recordFreq<<"number of LS = " <<rand_numLS<<endl;
		float rand_m; //generate uniform number
		int m=0; //m is the module returned from find_module()
		//generate the module based on cumulative probability, always select the remaining module based on flag
		flag_m[9] =  true;//make sure these three are not selected @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/////
		srand ( time(NULL) ); //seed it
		for (int i = 1; i <= rand_numLS; i++)
		{
			refind:
			findCumu(freq, maxLocalSearch, cutPoint, flag_m);
			
			rand_m = (float) rand() / float(RAND_MAX); //generate a number between 0 and 1
			//cout<<"rand_m= "<<rand_m << endl;
			//recordFreq<<"rand_m= "<<rand_m << endl;
			m = find_module (rand_m, cutPoint, maxLocalSearch, flag_m); //m is the module returned 
			if (m==0 || m==9 ) //make sure these three are not selected @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/////
				goto refind;
			modu[i] = m;
			flag_m[m]=true; //flag it to true so that it would not be selected again
			m=0;//initialize m =0;
		}

		//for (int i = 1; i <= rand_numLS; i++) 
		//{
			//cout<<"modu["<<i<<"]= "<<modu[i]<<' '<<' ';
			//recordFreq<<"modu["<<i<<"]= "<<modu[i]<<' '<<' ';
			//route_file <<"modu["<<i<<"]= "<<modu[i]<<' '<<' ';
			
		//}
		//sort the modu
		int temp;
		for (int i = 1; i < rand_numLS; i++) 
		{
			for (int j = i+1; j <= rand_numLS; j++)
			{
				if (modu[j] < modu[i])
				{
					temp = modu[i];
					modu[i] = modu[j];
					modu[j] = temp;
				}
			}

		}
		//recordFreq<<endl;
		//route_file <<endl;
		//for (int i = 1; i <= rand_numLS; i++) 
		//{
		//	recordFreq<<"modu["<<i<<"]= "<<modu[i]<<' '<<' ';
		//	route_file <<"modu["<<i<<"]= "<<modu[i]<<' '<<' ';
		//}
		//cout<<endl;
		//recordFreq<<endl;

		int I=1;//to keep track of modu[I] - which local search to be performed

	while (I <= rand_numLS)
	{
		//assign level
	//redo_LS: //1) if no improvement found in each level of LS, 2) if cost is better than aftershake cost set level=1, 3) if LS level has not finished all level level++ and goto redo_LS
		level= modu[I];
		//cout<<"level= "<<level<<endl;
		if (level == 1)
		{
			//operand[level].levelID = level;
			int number = find_k_1_0(cost_of_removing, kGain, Kth, level);
			Total_LSmove++;
			if (Total_LSmove %500 == 0)
			{
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_learning << Total_LSmove<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
			}
			cout<<"in level "<<level<<": find_k_1_0, best number= "<<number << endl;
			//route_file<<"in level "<<level<<": find_k_1_0, best number= "<<number << endl;
			
			if (number == -1)//if no improvement
			{
				I++;
				continue;
			}

			if (kGain[number][2] == kGain[number][3])
				sameroute = true; //same route
			else
				sameroute = false; //different route

			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP= "<<kGain[number][4]<<" ToP= "<<kGain[number][5]<< endl;
			//route_file<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP= "<<kGain[number][4]<<" ToP= "<<kGain[number][5]<< endl;
			//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
			for (int i = 1; i <= maxLocalSearch; i++)
			{		
				int j = kGain[number][2]; //Route1
				int k = kGain[number][3]; //Route2
				for (int m = 0; m < no_routes; m++)
				{
					operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
					operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
				}
				if (sameroute == false)
				{
					for (int m = 0; m < no_routes; m++)
					{	
						operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
						operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
					}
				}
			}
			//cost_of_removing is updated in insert()
			insert_1_0_2nd(cost_of_removing, Kth, number, kGain, VEHICLE, sameroute);
			cout<<endl<<endl;
			cout<<"========== vehicle after insert in level 1 find_k_1_0==================="<<endl;
			//route_file<<"========== vehicle after insert in level 1 find_k_1_0==================="<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					cout<<VEHICLE[i][j]<<' ';
					//route_file<<VEHICLE[i][j]<<' ';
				}
				cout<<endl;
				//route_file<<endl;
			}
			goto check_sol;//after each LS , compare with aftershakecost
		}

		else if (level == 2)
		{
			
			int number = find_k_1_1(cost_of_removing, kGain, Kth, level);//in every position
			Total_LSmove++;
			if (Total_LSmove %500 == 0)
			{
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_learning << Total_LSmove<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
			}
			cout<<"in level "<<level<<": find_k_1_1, best number= "<<number << endl;
			//route_file<<"in level "<<level<<": find_k_1_1, best number= "<<number << endl;
			
			if (number == -1)//if no improvement
			{	
				I++;
				continue;
			}

			if (kGain[number][2] == kGain[number][3])
				sameroute = true; //same route
			else
				sameroute = false; //different route

			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
			//route_file<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
			//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
			for (int i = 1; i <= maxLocalSearch; i++)
			{		
				int j = kGain[number][2]; //Route1
				int k = kGain[number][3]; //Route2
				for (int m = 0; m < no_routes; m++)
				{
					operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
					operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
				}
				if (sameroute == false)
				{
					for (int m = 0; m < no_routes; m++)
					{	
						operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
						operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
					}
				}
			}
			//cost_of_removing is updated in insert()
			insert_1_1_2nd(cost_of_removing, Kth, number, kGain, VEHICLE, sameroute);
			cout<<endl<<endl;
			cout<<"========== vehicle after insert in level 4: find_k_1_1==================="<<endl;
			//route_file<<"========== vehicle after insert in level 4: find_k_1_1==================="<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					cout<<VEHICLE[i][j]<<' ';
					//route_file<<VEHICLE[i][j]<<' ';
				}
				cout<<endl;
				//route_file<<endl;
			}
			goto check_sol;//after each LS , compare with aftershakecost
		}

		else if (level == 3)
		{
			//operand[level].levelID = level;
			int number = find_k_2_1(cost_of_removing, kGain, Kth, level);
			Total_LSmove++;
			if (Total_LSmove %500 == 0)
			{
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_learning << Total_LSmove<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
			}
			cout<<"in level "<<level<<": find_k_2_1, best number= "<<number << endl;
			//route_file<<"in level "<<level<<": find_k_2_1, best number= "<<number << endl;

			if (number == -1)//if no improvement
			{
				I++;
				continue;
			}

			if (kGain[number][2] == kGain[number][3])
				sameroute = true; //same route
			else
				sameroute = false; //different route

			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
			//route_file<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
			//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
			for (int i = 1; i <= maxLocalSearch; i++)
			{		
				int j = kGain[number][2]; //Route1
				int k = kGain[number][3]; //Route2
				for (int m = 0; m < no_routes; m++)
				{
					operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
					operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
				}
				if (sameroute == false)
				{
					for (int m = 0; m < no_routes; m++)
					{	
						operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
						operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
					}
				}
			}
			//cost_of_removing is updated in insert()
			insert_2_1_2nd(cost_of_removing, Kth, number, kGain, VEHICLE, sameroute);
			cout<<endl<<endl;
			cout<<"========== vehicle after insert in level 6: find_k_2_1==================="<<endl;
			//route_file<<"========== vehicle after insert in level 6: find_k_2_1==================="<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					cout<<VEHICLE[i][j]<<' ';
					//route_file<<VEHICLE[i][j]<<' ';
				}
				cout<<endl;
				//route_file<<endl;
			}
			goto check_sol;//after each LS , compare with aftershakecost
		}

		else if (level == 4)
		{
			//operand[level].levelID = level;
			int number = find_k_2_0(cost_of_removing, kGain, Kth, level);
			Total_LSmove++;
			if (Total_LSmove %500 == 0)
			{
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_learning << Total_LSmove<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
			}
			cout<<"in level "<<level<<": find_k_2_0, best number= "<<number << endl;
			//route_file<<"in level "<<level<<": find_k_2_0, best number= "<<number << endl;

			if (number == -1)//if no improvement found
			{
				I++;
				continue;
			}	

			if (kGain[number][2] == kGain[number][3])
				sameroute = true; //same route
			else
				sameroute = false; //different route

			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
			//route_file<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
			//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
			for (int i = 1; i <= maxLocalSearch; i++)
			{		
				int j = kGain[number][2]; //Route1
				int k = kGain[number][3]; //Route2
				for (int m = 0; m < no_routes; m++)
				{
					operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
					operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
				}
				if (sameroute == false)
				{
					for (int m = 0; m < no_routes; m++)
					{	
						operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
						operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
					}
				}
			}
			//cost_of_removing is updated in insert()
			insert_2_0_2nd(cost_of_removing, Kth, number, kGain, VEHICLE, sameroute);
			cout<<endl<<endl;
			cout<<"========== vehicle after insert in level 8: find_k_2_0==================="<<endl;
			//route_file<<"========== vehicle after insert in level 8: find_k_2_0==================="<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					cout<<VEHICLE[i][j]<<' ';
					//route_file<<VEHICLE[i][j]<<' ';
				}
				cout<<endl;
				//route_file<<endl;
			}
			goto check_sol;//after each LS , compare with aftershakecost
		}

		else if (level == 5)
		{
			int number = find_k_2_2swap(cost_of_removing, kGain, Kth, level);
			Total_LSmove++;
			if (Total_LSmove %500 == 0)
			{
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_learning << Total_LSmove<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
			}
			cout<<"in level "<<level<<": find_k_2_2swap, best number= "<<number << endl;
			//route_file<<"in level "<<level<<": find_k_2_2swap, best number= "<<number << endl;

			if (number == -1)//if no improvement found
			{	
				I++;
				continue;
			}
						
			if (kGain[number][2] == kGain[number][3])
				sameroute = true; //same route
			else
				sameroute = false; //different route

			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
			//route_file<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
			//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
			for (int i = 1; i <= maxLocalSearch; i++)
			{		
				int j = kGain[number][2]; //Route1
				int k = kGain[number][3]; //Route2
				for (int m = 0; m < no_routes; m++)
				{
					operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
					operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
				}
				if (sameroute == false)
				{
					for (int m = 0; m < no_routes; m++)
					{	
						operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
						operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
					}
				}
			}

			//cost_of_removing is updated in insert()
			insert_2_2_swap2nd(cost_of_removing, Kth, number, kGain, VEHICLE, sameroute);
			cout<<endl<<endl;
			//route_file<<endl<<endl;
			cout<<"========== vehicle after insert in level "<<level<<": find_k_2_2swap==================="<<endl;
			//route_file<<"========== vehicle after insert in level "<<level<<": find_k_2_2swap==================="<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					cout<<VEHICLE[i][j]<<' ';
					//route_file<<VEHICLE[i][j]<<' ';
				}
				cout<<endl;
				//route_file<<endl;
			}
			goto check_sol;//after each LS , compare with aftershakecost
		}
		else if (level == 6)
		{
			//operand[level].levelID = level;
			int number = find_two_optintra(kGain, Kth, level);
			Total_LSmove++;
			if (Total_LSmove %500 == 0)
			{
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_learning << Total_LSmove<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
			}
			cout<<"in level "<<level<<": find_two_optintra, best number= "<<number << endl;
			//route_file<<"in level "<<level<<": find_two_optintra, best number= "<<number << endl;

			if (number == -1)//if no improvement found
			{
				I++;
				continue;
			}
								
			sameroute = true; //same route

			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" revese_status= "<<kGain[number][9]<< endl;
			//route_file<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" revese_status= "<<kGain[number][9]<< endl;
			//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
			for (int i = 1; i <= maxLocalSearch; i++)
			{		
				int j = kGain[number][2]; //Route1
	
				for (int m = 0; m < no_routes; m++)
				{
					operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
					operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
				}
			}

			//cost_of_removing is updated in insert()
			insert_2optintra(cost_of_removing, Kth, number, kGain, VEHICLE);
			cout<<endl<<endl;
			//route_file<<endl<<endl;
			cout<<"========== vehicle after insert in level "<<level<<": find_two_optintra==================="<<endl;
			//route_file<<"========== vehicle after insert in level "<<level<<": find_two_optintra==================="<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					cout<<VEHICLE[i][j]<<' ';
					//route_file<<VEHICLE[i][j]<<' ';
				}
				cout<<endl;
				//route_file<<endl;
			}
			goto check_sol;//after each LS , compare with aftershakecost
		}
		else if (level == 7)
		{
			//operand[level].levelID = level;
			int number = find_two_optinter(kGain, Kth, level);
			Total_LSmove++;
			if (Total_LSmove %500 == 0)
			{
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_learning << Total_LSmove<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
			}
			cout<<"in level "<<level<<": find_two_optinter, best number= "<<number << endl;
			//route_file<<"in level "<<level<<": find_two_optinter, best number= "<<number << endl;

			if (number == -1)//if no improvement found
			{
				//operand[level].fullsearchstatus = 1;
				I++;
				continue;
				//goto redo_LS;//if no improvement found in each level of LS
			}
								
			sameroute = false; //diff route

			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" revese_status= "<<kGain[number][9]<< endl;
			//route_file<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" revese_status= "<<kGain[number][9]<< endl;
			//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
			for (int i = 1; i <= maxLocalSearch; i++)
			{		
				int j = kGain[number][2]; //Route1
				int k = kGain[number][3]; //Route2
				for (int m = 0; m < no_routes; m++)
				{
					operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
					operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
					operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
					operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
				}
			}

			//cost_of_removing is updated in insert()
			insert_2optinter(cost_of_removing, Kth, number, kGain, VEHICLE);
			cout<<endl<<endl;
			//route_file<<endl<<endl;
			cout<<"========== vehicle after insert in level "<<level<<": find_two_optinter==================="<<endl;
			//route_file<<"========== vehicle after insert in level "<<level<<": find_two_optinter==================="<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					cout<<VEHICLE[i][j]<<' ';
					//route_file<<VEHICLE[i][j]<<' ';
				}
				cout<<endl;
				//route_file<<endl;
			}
			goto check_sol;//after each LS , compare with aftershakecost
		}
		else if (level == 8)
		{
			//operand[level].levelID = level;
			int number = find_crossTail2(kGain, Kth, level);
			Total_LSmove++;
			if (Total_LSmove %500 == 0)
			{
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_learning << Total_LSmove<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
			}
			cout<<"in level "<<level<<": find_crossTail2, best number= "<<number << endl;
			//route_file<<"in level "<<level<<": find_crossTail2, best number= "<<number << endl;

			if (number == -1)//if no improvement found
			{
				//operand[level].fullsearchstatus = 1;
				I++;
				continue;
				//goto redo_LS;//if no improvement found in each level of LS
			}
								
			sameroute = false; //diff route

			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" revese_status= "<<kGain[number][9]<< endl;
			//route_file<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" revese_status= "<<kGain[number][9]<< endl;
			//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
			for (int i = 1; i <= maxLocalSearch; i++)
			{		
				int j = kGain[number][2]; //Route1
				int k = kGain[number][3]; //Route2
				for (int m = 0; m < no_routes; m++)
				{
					operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
					operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
					operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
					operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
				}
			}

			//cost_of_removing is updated in insert()
			insert_crosstail(cost_of_removing, Kth, number, kGain, VEHICLE);
			cout<<endl<<endl;
			//route_file<<endl<<endl;
			cout<<"========== vehicle after insert in level "<<level<<": find_crossTail2==================="<<endl;
			//route_file<<"========== vehicle after insert in level "<<level<<": find_crossTail2==================="<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					cout<<VEHICLE[i][j]<<' ';
					//route_file<<VEHICLE[i][j]<<' ';
				}
				cout<<endl;
				//route_file<<endl;
			}
			goto check_sol;//after each LS , compare with aftershakecost
		}
		else if (level == 9)
		{
			//operand[level].levelID = level;
			int number = find_CROSS(kGain, Kth, level);
			Total_LSmove++;
			if (Total_LSmove %500 == 0)
			{
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_learning << Total_LSmove<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
			}
			cout<<"in level "<<level<<": find_CROSS, best number= "<<number << endl;
			//route_file<<"in level "<<level<<": find_CROSS, best number= "<<number << endl;

			if (number == -1)//if no improvement found
			{
				//operand[level].fullsearchstatus = 1;
				I++;
				continue;
				//goto redo_LS;//if no improvement found in each level of LS
			}
								
			sameroute = false; //diff route

			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" revese_status= "<<kGain[number][9]<< endl;
			//route_file<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" revese_status= "<<kGain[number][9]<< endl;
			//if found improvement, flag the route false, because route has been changed, all operators need to perform on this route
			for (int i = 1; i <= maxLocalSearch; i++)
			{		
				int j = kGain[number][2]; //Route1
				int k = kGain[number][3]; //Route2
				for (int m = 0; m < no_routes; m++)
				{
					operand[i].eachRfullsearchstatus[j][m] = 0;	//R1 to all other routes
					operand[i].eachRfullsearchstatus[m][j] = 0;	//all other routes to R1 
					operand[i].eachRfullsearchstatus[k][m] = 0;	//R2 to all other routes
					operand[i].eachRfullsearchstatus[m][k] = 0;	//all other routes to R2 
				}
			}

			//cost_of_removing is updated in insert()
			insert_cross(cost_of_removing, Kth, number, kGain, VEHICLE);
			cout<<endl<<endl;
			//route_file<<endl<<endl;
			cout<<"========== vehicle after insert in level "<<level<<": find_CROSS==================="<<endl;
			//route_file<<"========== vehicle after insert in level "<<level<<": find_CROSS==================="<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					cout<<VEHICLE[i][j]<<' ';
					//route_file<<VEHICLE[i][j]<<' ';
				}
				cout<<endl;
				//route_file<<endl;
			}
			goto check_sol;//after each LS , compare with aftershakecost
		}

	check_sol://after each LS or if no feasible shake at last module(neighborhod), goto check_sol, compare with aftershakecost

		//======================================================Display route=======================================================================//

		cout << "================= Routes (in VNS Adaptive) after LS Level " << level <<" ===================================== " << endl;
		//route_file << "==================Routes (in VNS Adaptive) after LS Level " << level <<" ===================================== " << endl;
		float total_cost = 0.0;
		int total_cust = 0;
		int sum_demand=0;
		for (int g = 0; g < no_routes; g++)
		{
			if (saiz[g] == 0)
			{
				route_cost[g] = 0;
				distance_cost[g] = 0;
			}
			cout<<g<<' '<<route_cost[g]<<' '<<saiz[g]<<' '<<total_demand[g]<<' '<<' ';
			//route_file<<g<<' '<<route_cost[g]<<' '<<saiz[g]<<' '<<total_demand[g]<<' '<<' ';
			for (int h = 0; h <= saiz[g]+1; h++)
			{
				cout << VEHICLE[g][h] << ' ';		
				//route_file << VEHICLE[g][h] << ' ';		
			}
			cout << endl;
			//route_file << endl;

			total_cust = total_cust+saiz[g];
			total_cost = total_cost + route_cost[g];
			sum_demand = sum_demand+total_demand[g];
			
			if (total_demand[g] < 0)
				getchar();
		}
		cout << "Total customers = " << total_cust << endl<< "Total cost = " << total_cost << endl<< "Sum_demand = " << sum_demand <<endl<< "TOTAL_DEMAMD = " << TOTAL_DEMAND<<endl;
		//route_file << "Total customers = " << total_cust << endl<< "Total cost = " << total_cost << endl<< "Sum_demand = " << sum_demand <<endl<< "TOTAL_DEMAMD = " << TOTAL_DEMAND<<endl;
		//route_file<< "After shake cost = " <<afterShake_cost <<endl;
		//route_file<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
	
		if ((afterShake_cost - total_cost) > epsilonn)//t_cost < afterShake_cost
		{
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ for infeasibility check @@@@@@@@@@@@@@@@@@@@@@@@//
			//for (int i = 0; i < no_routes; i++)
			//{
			//	if ((total_demand[i]>CAPACITY) || (distance_cost[i] - DISTANCE > epsilon))//for infeasibility check
			//	{
			//		violation << "In VNS multi-level after module "<< Neighbour_k <<endl;
			//		violation << i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
			//		for (int j = 0; j <= saiz[i]+1; j++)
			//		{
			//			violation << VEHICLE[i][j]<<' ';
			//		}
			//		violation<<endl;
			//		violateThreshold = violateThreshold/2; //divide by half if the solution after LS still violate constraint
			//		Neighbour_k++; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			//		goto skip_check_local;
			//	}
			//}
			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ End of (for infeasibility check) @@@@@@@@@@@@@@@@@@@@@@@@//
			//try checking fesibility here next time!!!!!!!!!!!!!!!! to avoid  time wasted if route is infeasible after so many LS
			//if((abs(t_cost-LOCAL_BEST_COST) <= 0.05) || (abs(t_cost-GLOBAL_BEST_COST) <= 0.05)) //added on 3April2015 to avoid recalculate if it is the same as LOCAl or GLOBAL BEST
			//	goto check_local_best; //compare with LOCAL_BEST_COST after all LS are finished
			afterShake_cost = total_cost;
			I=1;
			
		}
	
		else
		{
			I = I+1;
			
		}
	}//end while (I <= rand_numLS)

	check_LOCAL_BEST://if all LS are done, goto check_LOCAL_BEST //compare with LOCAL_BEST_COST
		//======================================================Display route========================================================================

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
				//getchar();
			}
		}

		cout << "==================Routes (in VNS Adaptive) after all LS before check LOCAL BEST===================================== " << endl;
		cout << "Current Neighbour_k is "<<Neighbour_k<<endl;
		//route_file << "==================Routes (in VNS Adaptive) after all LS before check LOCAL BEST========================= " << endl;
		//route_file << "Current Neighbour_k is "<<Neighbour_k<<endl;

		float total_cost = 0.0;
		int total_cust = 0;
		for (int g = 0; g < no_routes; g++)
		{
			if (saiz[g] == 0)
			{
				route_cost[g] = 0;//to avoid very small value
				distance_cost[g] = 0;
				distance_cost[g] = 0;
			}
		
			total_cust = total_cust + saiz[g];
			cout<<g<<' '<<route_cost[g]<<' '<<saiz[g]<<' '<<total_demand[g]<<' '<<' ';
			//route_file<<g<<' '<<route_cost[g]<<' '<<saiz[g]<<' '<<total_demand[g]<<' '<<' ';
			for (int h = 0; h <= saiz[g]+1; h++)
			{
				cout << VEHICLE[g][h] << ' ';
				//route_file << VEHICLE[g][h] << ' ';
			}
			cout << endl;
			//route_file << endl;
			total_cost = total_cost + route_cost[g];
		
		}
		cout << "Total customers = " << total_cust << endl << "Total cost = " << total_cost << endl;
		cout <<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
		//route_file << "Total customers = " << total_cust << endl << "Total cost = " << total_cost << endl;
		//route_file <<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
		//getchar();

	
		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ for infeasibility check @@@@@@@@@@@@@@@@@@@@@@@@//
		
		//for (int i = 0; i < no_routes; i++)
		//{
		//	if ((total_demand[i]>CAPACITY) || (distance_cost[i] - DISTANCE > epsilon))//for infeasibility check
		//	{
		//		cout << "Infeasibility check In VNS adaptive in Neighbour "<< Neighbour_k <<endl;
		//		violation << "In VNS adaptive after module "<< Neighbour_k <<endl;
		//		violation << i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		//		for (int j = 0; j <= saiz[i]+1; j++)
		//		{
		//			violation << VEHICLE[i][j]<<' ';
		//		}
		//		violation<<endl;
		//		violateThreshold = violateThreshold/2; //divide by half if the solution after LS still violate constraint
		//		//if (Neighbour_k != max_neigbor)
		//		//{
		//			Neighbour_k++;
		//			goto skip_check_local;
		//	}
		//}
		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ End of (for infeasibility check) @@@@@@@@@@@@@@@@@@@@@@@@//

		if ((LOCAL_BEST_COST - total_cost) > epsilonn )//t_cost < LOCAL_BEST_COST
		{
			//reinitialize vehicle[][] so that it can be passed to shaking afterwards
			//============= Record the best solution ====================//
			//sort_solution(VEHICLE); //dont sort here, cost_of_removing will be affected
			LOCAL_BEST_COST = total_cost;
			
			for (int i = 0; i < no_routes; i++)
			{
				LOCAL_SAIZ[i] = saiz[i];
				LOCAL_capa[i] = total_demand[i];
				LOCAL_Rcost[i] = route_cost[i];
				LOCAL_distance_cost[i] = distance_cost[i];
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					LOCAL[i][j] = VEHICLE[i][j];	
					LOCALCumDist[i][j] = CumDist[i][j];
				}
			}
			
			if (no_routes != LOCAL_NO_ROUTE)
			{
				cout<<"ERROR no_routes != LOCAL_NO_ROUTE in Adaptive VNS)"<<endl;
				getchar();
			}
			//LOCAL_NO_ROUTE = no_routes;//number of routes is the same as LOCAL_NO_ROUTE, noneed to initialize

			//=================================record localbest solution in txt=====================================//
			cout << "========================================================================== " << endl;
			//route_file << "========================================================================== " << endl;
			//=================================display final solution=====================================//
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				localbest_sol << i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
				//route_file << i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
				{
					localbest_sol << LOCAL[i][j] << ' ';
					//route_file << LOCAL[i][j] << ' ';
				}
				localbest_sol<<endl;
				//route_file<<endl;
			}
			localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"in VNS multi-level adaptive after shake Neighbour_k "<<Neighbour_k ;
			//route_file << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"in VNS multi-level adaptive after shake Neighbour_k "<<Neighbour_k <<"level " <<level<< endl;	
			if (deletKGain==true)
			{
				localbest_sol << " level " <<level<< endl;	
				for (int i = 1; i <= rand_numLS; i++)
				{
					localbest_sol <<"modu["<<i<<"]= "<<modu[i]<<' '<<' ';
					//route_file <<"modu["<<i<<"]= "<<modu[i]<<' '<<' ';
				}
			}
			
			localbest_sol <<endl;
			//route_file <<endl;
			//once found local best, copy to Oricost_of_removing, //if not better than local best solution and neighbourhood havent finished, use the Oricost_of_removing because shaking is based on local best, oriGain is updated evrytime found local best
			//============================= copy to Oricost_of_removing ================================//
			for (int i = 0; i < SIZE; i++)
			{
				Oricost_of_removing[i] = cost_of_removing[i];
			}
			calculate_centreGravity(x, y);
			Neighbour_k = 1;
			foundLOCALinCurrNeighbor = true;//Do Dijkstra refinement if found LOCAL BEST in any neighbourhood in this diversification, otherwise, just do refinement for VEHICLE
			
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
			//******************** SKIP DIJKSTRA REFINEMENT *****************************

			//if (Neighbour_k == max_neigbor) //current neighbourhood is the last neighbourhood, go to dijkstra refinement
			//{
			//	float cost = 0.0; //to be passed to dijkstra_refinement
			//	//dijkstra_refinement refine the LOCAL_BEST (incumbent best) if it found new LOCAL_BEST in current neighbourhood
			//	cout<<"Entering Dijkstra refinement in Adaptive VNS"<<endl;
			//	route_file<<"Entering Dijkstra refinement in Adaptive VNS"<<endl;
			//
			//	if(foundLOCALinCurrNeighbor == true)//if found LOCAL BEST, use local best, else just use current VEHICLE
			//	{
			//		srand ( time(NULL) ); //seed it
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
			//	dijkstra_refinement(cost, VEHICLE); 
			//	cout<<"cost after dijkstra_refinement= "<<cost<<endl;
			//	route_file<<"cost after dijkstra_refinement= "<<cost<<endl;
			//	if ((LOCAL_BEST_COST-cost) > epsilonn)//cost < LOCAL_BEST_COST //changed t_cost to cost on 1April2015
			//	{
			//		cout<< "Found new LOCAL_BEST right after Dijkstra REfinement"<<endl;
			//		route_file<< "Found new LOCAL_BEST right after Dijkstra REfinement"<<endl;
			//		//find all cost_of_removing and reinitialize the route because routes have been sorted and changed after Dijkstra refinement
			//		find_all_cost_of_removing (cost_of_removing);//after Dijkstra, routes and position migh have changed
			//		reinitializeRchangedStatus ();//initialize to 0 for every route
			//		initializeEachRfullsearchstatus (maxLocalSearch);
			//		//============= Record the best solution ====================//
			//		LOCAL_BEST_COST = cost;
			//		cout << endl << "LOCAL_BEST COST IS " << LOCAL_BEST_COST << endl;
			//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			//		{
			//			for (int j = 0; j <= saiz[i]+1; j++)
			//			{
			//				LOCAL[i][j] = VEHICLE[i][j];
			//				LOCALCumDist[i][j] = CumDist[i][j];
			//			}
			//
			//			LOCAL_SAIZ[i] = saiz[i]; //copy temp saiz[] to best SAIZ[]
			//			LOCAL_capa[i] = total_demand[i]; // capa[] record the BEST_ROUTE demand
			//			LOCAL_Rcost[i] = route_cost[i];
			//			LOCAL_distance_cost[i] = distance_cost[i];
			//		}
			//		calculate_centreGravity(x, y);
			//		
			//		if ( (LOCAL_BEST_COST < LOCAL_BCOST) && (LOCAL_BEST_COST != GLOBAL_BEST_COST) )//to record the best of LOCAL for diversification purpose
			//		{
			//			//record to LOCAL_B
			//			LOCAL_BCOST = LOCAL_BEST_COST;
			//			int r = 0; //for LOCAL_B because do not consider empty route
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
			//		//=================================record localbest solution in txt=====================================//
			//		cout << "========================================================================== " << endl;
			//		route_file << "========================================================================== " << endl;
			//		//ofstream localbest_sol("30.LOCAL_BEST_SOLUTION.txt", ios::app);
			//		//=================================display final solution=====================================//
			//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			//		{
			//			cout<< i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
			//			localbest_sol << i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
			//			route_file << i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
			//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
			//			{
			//				cout<< LOCAL[i][j] << ' ';
			//				localbest_sol << LOCAL[i][j] << ' ';
			//				route_file<< LOCAL[i][j] << ' ';
			//			}
			//			cout<<endl;
			//			localbest_sol<<endl;
			//			route_file<<endl;
			//		}
			//		cout << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
			//		localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
			//		route_file << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
			//		localbest_sol<<"FOUND LOCAL BEST AFTER DIJKSTRA REFINEMENT, UPDATE ORIGAIN"<<endl;
			//		//getchar();
			//		//============================= copy to oriGain because oriGain always hold the gain for local_best, it is used in shaking later================================//
			//		for (int i = 0; i < SIZE; i++)
			//		{
			//			Oricost_of_removing[i] = cost_of_removing[i];
			//		}
			//		//localbest_sol.close();
			//		//======================= End localrecord best solution ===========================//
			//		Neighbour_k = 1;
			//		foundLOCALinCurrNeighbor = false; //initialize to false when found LOCAL solution after Dijkstra refinement, so it will not go through Dijkstra refinement later
			//		//delete[] tempRcost;
			//		//for (int i = 0; i < no_routes; i++)
			//		//{
			//		//	delete[] tempvec[i];
			//		//}
			//		//delete[] tempvec;
			//		//delete[] tempRcapa;
			//		//delete[] tempRsize;
			//		goto skip_check_local; //if Dijkstra refinement produce better result, noneed to copy original gain matrix because we use this solution now		
			//	}//end if (cost < LOCAL_BEST_COST)			
			//}//end if (Neighbour_k == max_neigbor) //current neighbourhood is the last neighbourhood
			Neighbour_k++;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //put at the back on 18June2015, previously before    if (Neighbour_k > max_neigbor)  
			//if not better than local best solution and neighbourhood havent finished, use the original gain because shaking is based on local best, oriGain is updated evrytime found local best
			//============================= copy from oriGain ================================//
			cout<<"Entering copy from Oricost_of_removing because shaking is based on local best"<<endl;


			for (int f = 0; f < SIZE; f++) //find cost of removing for each customer and put in array
			{
				cost_of_removing[f] = Oricost_of_removing[f];		
			}
			
			
		}//if is is not better than the incumbent
	
//skip_check_local:;
		
		if (deletKGain==true)//only delete if it has been declared yet, if use goto check_LOCAL_BEST, no declaration for kGian, therefore error when delete
		{
			for (int i = 0; i < Kth; i++)
			{
				delete[] kGain[i];
			}delete[] kGain;
			delete[] cutPoint; //from 0 to 1 inclusive // 8 modules, 9 cutpoint
			delete[] flag_m; // starts from 1
			delete[] modu; //modul[] starts from 1;//+1 to add last module 2-opt, compulsory LS
		}
		
		for (int i = 1; i <= maxLocalSearch; i++)
		{
			for (int j = 0; j < no_routes; j++)
			{
				delete[] operand[i].eachRfullsearchstatus[j];
			}
			delete[] operand[i].eachRfullsearchstatus;
		}
		delete[] operand;
		
		

skip_check_local:;
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)//added 05/06/2016
		{
			delete[] route_CGravity[i]; 
			delete[] sorted_custRGravity[i];
		}
		delete[] route_CGravity;
		delete[] custRGravity;
		delete[] sorted_custRGravity;
		
	}//end while (Neighbour_k <= max_neigbor) //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@	

	//violateDiverstart: //if violation, skip all the process of compare LOCAL_BEST, check if max diversification is reached
		if ((GLOBAL_BEST_COST-LOCAL_BEST_COST) > epsilonn)//LOCAL_BEST_COST < GLOBAL_BEST_COST
		{
			GLOBAL_BEST_COST = LOCAL_BEST_COST;
			cout << endl << "GLOBAL_BEST COST IS " <<GLOBAL_BEST_COST << endl;
			//route_file << endl << "GLOBAL_BEST COST IS " <<GLOBAL_BEST_COST << endl;
			found_GLOBEST_status = 1; //for diversification use
			srand ( time(NULL) ); //seed it
			//record the best solution in GLOBAL_BEST_ROUTE in sorted order
			int r=0;
			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
			{
				if (LOCAL_SAIZ[i] == 0)
					continue;
				
				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
				{
					GLOBAL[r][j] = LOCAL[i][j];
					GLOBALCumDist[r][j] = LOCALCumDist[i][j];
				}
				
				GLOBAL_Rcost[r] = LOCAL_Rcost[i];
				GLOBAL_distance_cost[r] = LOCAL_distance_cost[i];
				GLOBAL_SAIZ[r] = LOCAL_SAIZ[i];
				GLOBAL_capa[r] = LOCAL_capa[i];
				r++;
			}
			GLOBAL_NO_ROUTE = r;

			sort_GLOBALsolutionSMALLEST(GLOBAL); 
			//=================================record the best solution=====================================//
			cout << "========================================================================== " << endl;

			//=================================display final solution=====================================//
			best_sol <<endl<<"From VNS multi level after Neighbour_k " <<Neighbour_k<<' ' <<' '<<"NbDiv= "<<NbDiv<<  endl;
			//route_file <<endl<<"From VNS multi level after Neighbour_k " <<Neighbour_k<<' ' <<' '<<"NbDiv= "<<NbDiv<<  endl;
			for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
			{
				best_sol << i<<' '<<GLOBAL_Rcost[i]<<' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<<' ';
				cout << i <<' '<< GLOBAL_Rcost[i] <<' '<< GLOBAL_SAIZ[i] <<' '<< GLOBAL_capa[i]<<' '<<' ';
				//route_file << i<<' '<<GLOBAL_Rcost[i]<<' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<<' ';
				for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
				{
					best_sol << GLOBAL[i][j] << ' ';
					cout << GLOBAL[i][j] << ' ';
					//route_file << GLOBAL[i][j] << ' ';
				}
				best_sol << endl;
				cout << endl;	
				//route_file << endl;
			}
			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
			best_sol << "From VNS adaptive after Neighbour_k " <<Neighbour_k<<' ' <<' '<<"NbDiv= "<<NbDiv<<  endl;
			cout << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
			cout<<"Found GLOBAL_BEST"<<endl;
			//route_file << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
			//route_file<< "From VNS adaptive after Neighbour_k " <<Neighbour_k<<' ' <<' '<<"NbDiv= "<<NbDiv<<  endl;
			//getchar();
		}

		//MUST COPY TO VEHICLE before diversification because VEHICLE is not necessarily GLOBAL bEST now// added 30Sept2015
		//if ( (LOCAL_BCOST > GLOBAL_BEST_COST) && abs(LOCAL_BCOST-GLOBAL_BEST_COST)<= (0.01*GLOBAL_BEST_COST))//if LOCAL_B is a good solution, diversify LOCAL_B
		//{
		//	srand ( time(NULL) ); //seed it
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
				route_cost[i] = GLOBAL_Rcost[i];
				saiz[i] = GLOBAL_SAIZ[i];
				total_demand[i] = GLOBAL_capa[i];
				space_available [i] = CAPACITY - total_demand[i];
				distance_available[i] = DISTANCE - distance_cost[i];
			}
			no_routes = GLOBAL_NO_ROUTE;
		//}
		//************************************************ NO DIVERSIFICATION *************************************************//	

		cout<<"ENTERING  DIVERSIFICATION PHASE"<<endl;
		//route_file<<"ENTERING  DIVERSIFICATION PHASE"<<endl;
		for (int i = 0; i < no_routes; i++)
		{
			cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
			//route_file<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
			for (int j = 0; j <= saiz[i]+1; j++)
			{
				cout<<VEHICLE[i][j]<<' ';
				//route_file<<VEHICLE[i][j]<<' ';
			}
			cout<<endl;
			//route_file<<endl;
		}
		cout<<"GLOBAL_BEST_COST= "<<GLOBAL_BEST_COST<<endl;
		//route_file<<"GLOBAL_BEST_COST= "<<GLOBAL_BEST_COST<<endl;
		
		float cost=0;
		//vehicle[][] to be passed in to make_giant_tour() is not important, make_giant_tour always consider LOCAL_BEST_SOL (incumbent best solution)
		
		//if (found_GLOBEST_status == 1)
		//{
		//	cost = make_giant_tour(VEHICLE); //pass in vehicle and no_routes to be updated in Dijkstra, they will be initialize in the Dijkstra function
		//	Ndel_min = LB_Ndel_min;//back to initial value
		//}
		//else if (found_GLOBEST_status == 0) //if not found global_best
		//{
			NbDiv++;
			if (NbDiv > maxDiv) //if this is last diversification, dont perform LNS because it will not go through the LS //added on 12 Oct2015
			{
				cout<<"goto lastdiversify in VNSmultilevel"<<endl;
				goto lastdiversify;//check if this one ever executed or change to if (NbDiv == maxDiv) 
			}
			//if (Total_LSmove >= 20000) //if this is last diversification, dont perform LNS because it will not go through the LS //added on 12 Oct2015
			//{
			//	goto lastdiversify;//check if this one ever executed or change to if (NbDiv == maxDiv) 
			//}
			cout<<"Entering Diversify LNS/conflict sector in VNS kth improvement: ";
			//route_file<<"Entering Diversify LNS/conflict sector in VNS kth improvement: ";
			
			if(div == RS)
				div=0;
			//div=(rand() % 4); 
			if (divsequence[div] == 0)
			{
				
				cout<<"LNS Diversification 3"<<endl;
				//route_file<<"LNS Diversification 3"<<endl;
				//cost = Diversify_LNS2(VEHICLE, Ndel_min);
				cost = Diversify_LNS3(VEHICLE, Ndel_min);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div] <<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			
			else if (divsequence[div] == 1)
			{
				cout<<"LNS Diversification overlap route2"<<endl;
				//route_file<<"LNS Diversification overlap route2"<<endl;
				//Diversification_overlapRoute( float **(&vehicle), int (&no_routes), int K, float *x, float *y)
				cost = Diversification_overlapRoute2(VEHICLE,  Ndel_min, x, y);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div] <<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			else if (divsequence[div] == 2)
			{
				cout<<"Diversify_RelatednessDemand Diversification "<<endl;
				//route_file<<"Diversify_Relatedness Diversification "<<endl;
				cost = Diversify_RelatednessDemand(VEHICLE, Ndel_min);
				//cost = Diversification_conflict_sector(VEHICLE, Ndel_min, x, y);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div] <<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			else if (divsequence[div] == 3)
			{
				
				cout<<"LNS Diversification LongestArc"<<endl;
				//route_file<<"LNS Diversification LongestArc"<<endl;
				cost = Diversify_LongestArc2(VEHICLE, Ndel_min);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div] <<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			else if (divsequence[div] == 4)
			{
				cout<<"Conflict sector Diversification: "<<endl;
				//route_file<<"Conflict sector Diversification: "<<endl;
				cost = Diversification_conflict_sector(VEHICLE, Ndel_min, x, y);
				//cost = Diversify_Relatedness(VEHICLE, Ndel_min);
				float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
				//EV_diver << divsequence[div] <<","<<Ndel_min<<","<<cost<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;
				div++;
			}
			
			else if (divsequence[div] == 5)
			{
				//cout<<"Diversify_BadArcNR Diversification: "<<endl;
				//cost = Diversify_BadArcNR(VEHICLE, Ndel_min);
				cout<<"Diversify_EntireRoute "<<endl;
				cost = Diversify_EntireRoute(VEHICLE,  Ndel_min, x, y) ;
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
		//sort_solution (VEHICLE);
		sort_solutionSMALLEST (VEHICLE);
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
		//route_file<<"AFTER DIVERSIFICATION"<<endl;
		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
		{
			cout<<i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
			//route_file<<i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
			{
				cout<<LOCAL[i][j]<<' ';
				//route_file<<LOCAL[i][j]<<' ';
			}
			cout<<endl;
			//route_file<<endl;
		}
		cout<<"LOCAL_BEST_COST ="<<LOCAL_BEST_COST<<endl;
		//route_file<<"LOCAL_BEST_COST ="<<LOCAL_BEST_COST<<endl;
		//if the cost better than GLOBAL_BEST_COST 
		if (GLOBAL_BEST_COST-cost > epsilonn)//cost < GLOBAL_BEST_COST
		{
			int m=0;//for GLOBAL[][] route index
			for (int i = 0; i < no_routes; i++)
			{
				if (saiz[i] == 0)
					continue;
				best_sol << m<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				//route_file << m<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					GLOBAL[m][j] = VEHICLE[i][j];
					GLOBALCumDist[m][j] = CumDist[i][j];
					best_sol << GLOBAL[m][j] << ' ';
					//route_file<< GLOBAL[m][j] << ' ';
				}
				best_sol <<endl;
				//route_file <<endl;
				GLOBAL_SAIZ[m] = saiz[i];
				GLOBAL_capa[m] = total_demand[i];
				GLOBAL_Rcost[m] = route_cost[i];
				GLOBAL_distance_cost[m] = distance_cost[i];
				m++;
				
			}
			GLOBAL_BEST_COST = cost;
			GLOBAL_NO_ROUTE = m;
			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
			best_sol << "From VNS adaptive after Diversification type "<<div<<  endl;
			cout << "Found GLOBAL_BEST in VNS multi-level right after Diversification (only basic LNS no other improvement) type "<<div<<  endl;
			//route_file << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
			//route_file << "Found GLOBAL_BEST in VNS multi-level right after Diversification (only basic LNS no other improvement) type "<<div<<  endl;
			//getchar();
		}//end cost < GLOBAL_BEST_COST
		best_sol << "Type of div = "<<div<<endl;
		
		//************************************************ NO DIVERSIFICATION *************************************************//	
		//NbDiv++;//################################DELETE THIS WHEN ADED DIVERSIFICATION########################################

		reinitializeRchangedStatus();//initialize to 0 for every route	
		lastdiversify:;
		delete[] cost_of_removing;
		delete[] Oricost_of_removing;
		delete[] RchangedStatus;
		//delete[] CustaffectedStatus;
	}//end while (NbDiv <= maxDiv) //#############################################################################################################################

	
	float runtime=(clock() - start_s) / float(CLOCKS_PER_SEC);
	//EV_learning << Total_LSmove<<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;; //record for the very last one
	//EV_diver << div <<","<<runtime<<","<<GLOBAL_BEST_COST<<endl;		

	timefile << "VNS adaptive "<<maxDiv <<" diversification time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << " second"<<endl;
	//recordTIME << "VNS  adaptive ("<<std::to_string( I )<<") with "<<maxDiv <<" diversification time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << " second"<<endl;
	//recordFreq.close();
	best_sol.close();
	localbest_sol.close();
}

void initializeEachRfullsearchstatus (int maxLocalSearch)
{
	for (int i = 1; i <= maxLocalSearch; i++)
	{
		for (int j = 0; j < no_routes; j++)
		{
			for (int k = 0; k < no_routes; k++)
			{
				operand[i].eachRfullsearchstatus[j][k] = 0;
			}
		}
	}
}

//No data structure of fullsearchStatus
//void adaptiveVNS_kth_improvementNO(float *x, float *y, float *(freq)) 
//{
//	ofstream recordTIME("TIME.txt", ios::app);
//	ofstream violation("Infeasible_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream timefile("16.Time_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream localbest_sol("30.LOCAL_BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream recordFreq("27.Frequency_of_module_" + std::to_string( I ) + ".txt", ios::app);
//	ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::app);
//
//	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);
//
//	float start_s = clock();// the code you wish to time goes here
//
//	//******************************************** STEP 1: INITIALIZATION ********************************************//
//	//============================================ all predefined variables =================================================//
//	int max_neigbor = 7; //7 modules of shake
//	int max_level = 11; //11 levels of LS improvement 
//	int maxLocalSearch = max_level-1; //excluding last LS:2-opt for freq use
//	int NbDiv = 0;
//	int maxDiv = min(3, GLOBAL_NO_ROUTE/2);
//	int LB_Ndel_min = max (5.00, (0.10*SIZE));
//	float siz=SIZE;
//	int UB_Ndel_min = min (400.00, (0.4*SIZE));
//	int Ndel_min = LB_Ndel_min; //min number of iteration for each route for diversification
//	int RS = 5; //number of removal strategies
//	//LNSremoval *strategy = new LNSremoval[RS];
//	//strategy[0].cutpoints = 1.00/RS;//first cutpoint is 0.25
//	//for (int i = 0; i < RS; i++)
//	//{ 
//	//	strategy[i].probability = 1.00/RS; //initially all equal probability
//	//	if (i!=0) //first cut point already determined
//	//		strategy[i].cutpoints += strategy[i-1].cutpoints;
//	//}
//
//	//============================================= all predefined variables ================================================//
//
//	int TOTAL_DEMAND=0;
//	for (int i = 0; i < SIZE; i++)
//	{
//		TOTAL_DEMAND = TOTAL_DEMAND+demand[i];
//	}//just to chack the overall demand is correct at the end of VNS multi-level
//
//	//============= Copy Local_BEST from GLOBAL_BEST ===========//
//	for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
//	{
//		for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
//		{
//			LOCAL[i][j] = GLOBAL[i][j];
//		}
//		LOCAL_capa[i] = GLOBAL_capa[i];
//		LOCAL_SAIZ[i] = GLOBAL_SAIZ[i];
//		LOCAL_Rcost[i] = GLOBAL_Rcost[i];
//	}
//	LOCAL_BEST_COST = GLOBAL_BEST_COST;
//	LOCAL_NO_ROUTE = GLOBAL_NO_ROUTE;
//
//	int div=0;
//	while (NbDiv <= maxDiv) //#############################################################################################################################
//	{
//		float violateThreshold = INFEASIBLE; //reinitialize to original value after diversification
//		int found_GLOBEST_status = 0;//must put before infeasibility check, otherwise found_GLOBEST_status is not recognise after goto violateDiverstart
//		localbest_sol<<"Adaptive NbDiv= "<<NbDiv<<endl;
//		
//		int no_empty_route=0;
//
//		cout<<"LOCAL_NO_ROUTE after restartAfterDiver= "<<LOCAL_NO_ROUTE<<endl;
//		localbest_sol<<"LOCAL_NO_ROUTE after restartAfterDiver= "<<LOCAL_NO_ROUTE<<endl;
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			cout<<i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
//			localbest_sol << i <<' '<< LOCAL_Rcost[i] <<' '<< LOCAL_SAIZ[i] <<' '<< LOCAL_capa[i]<<' '<<' ';
//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//			{
//				localbest_sol<<LOCAL[i][j]<< ' ';
//				cout<<LOCAL[i][j]<< ' ';
//			}
//			cout<<endl;
//			localbest_sol<<endl;
//		}
//		cout<<"LOCAL_BEST_COST= "<<LOCAL_BEST_COST<<endl;
//		localbest_sol<<"LOCAL_BEST_COST= "<<LOCAL_BEST_COST<<endl;
//
//		//******************************************** STEP 2: ADD EMPTY ROUTE ********************************************//
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			if (LOCAL_SAIZ[i] == 0)
//				no_empty_route++;
//		}
//		if (no_empty_route == 1)
//		{
//			goto skipaddemptyR;
//		}
//		else if (no_empty_route == 0)
//		{
//			//************************************add one empty route
//			LOCAL[LOCAL_NO_ROUTE][0] = SIZE; //depot
//			LOCAL[LOCAL_NO_ROUTE][1] = SIZE; //depot
//			LOCAL_capa[LOCAL_NO_ROUTE] = 0;
//			LOCAL_SAIZ[LOCAL_NO_ROUTE] = 0;
//			LOCAL_Rcost[LOCAL_NO_ROUTE] = 0;
//			LOCAL_NO_ROUTE++;
//			//************************************end of add one empty route
//		}
//		else if (no_empty_route > 1) //if more than 1 empty route, do not copy the empty route and add one at the end
//		{
//			int k=0; //for new route index
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			{
//				if (LOCAL_SAIZ[i] == 0)
//				{
//					continue;
//				}
//				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//				{
//					LOCAL[k][j] = LOCAL[i][j];
//				}
//				LOCAL_SAIZ[k] = LOCAL_SAIZ[i];
//				LOCAL_capa[k] = LOCAL_capa[i];
//				LOCAL_Rcost[k] = LOCAL_Rcost[i];
//				k++;
//			}
//			LOCAL_NO_ROUTE = k;
//			//************************************add one empty route
//			LOCAL[LOCAL_NO_ROUTE][0] = SIZE; //depot
//			LOCAL[LOCAL_NO_ROUTE][1] = SIZE; //depot
//			LOCAL_capa[LOCAL_NO_ROUTE] = 0;
//			LOCAL_SAIZ[LOCAL_NO_ROUTE] = 0;
//			LOCAL_Rcost[LOCAL_NO_ROUTE] = 0;
//			LOCAL_NO_ROUTE++;
//			//************************************end of add one empty route
//		}
//		cout<<"LOCAL_NO_ROUTE before skipaddemptyR= "<<LOCAL_NO_ROUTE<<endl;
//		localbest_sol<<"LOCAL_NO_ROUTE before skipaddemptyR= "<<LOCAL_NO_ROUTE<<endl;
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			cout << i <<' '<< LOCAL_Rcost[i] <<' '<< LOCAL_SAIZ[i] <<' '<< LOCAL_capa[i]<<' '<<' ';
//			localbest_sol << i <<' '<< LOCAL_Rcost[i] <<' '<< LOCAL_SAIZ[i] <<' '<< LOCAL_capa[i]<<' '<<' ';
//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//			{
//				cout<<LOCAL[i][j]<< ' ';
//				localbest_sol<<LOCAL[i][j]<< ' ';
//			}
//			cout<<endl;
//			localbest_sol<<endl;
//		}
//	skipaddemptyR:
//		cout<<"LOCAL_NO_ROUTE= "<< LOCAL_NO_ROUTE<<endl;
//		localbest_sol<<"LOCAL_NO_ROUTE= "<< LOCAL_NO_ROUTE<<endl;
//		
//		no_routes = LOCAL_NO_ROUTE;
//		float *cost_of_removing = new float[SIZE + 1];//cost_of_removing[][] only used in LS, not shaking
//		float *Oricost_of_removing = new float[SIZE + 1];
//
//		//================= Record route change status from shaking and best improvement======================//added 15Sept2015
//		//Route = new Rmodified[LOCAL_NO_ROUTE];
//		RchangedStatus = new bool[LOCAL_NO_ROUTE];
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			//Route[i].RchangedStatus=0;//initialize to 0 for every route, they have not been changed
//			RchangedStatus[i]=false;//initialize to 0 for every route, they have not been changed
//		}
//
//		//============= Copy VEHICLE from LOCAL_BEST ===========// guided shake needs to use VEHICLE 
//		bool copyalready; //to avoid copy vehicle from local_best again in while loop for the first time
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)//added 22Apr15
//		{
//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//			{
//				VEHICLE[i][j] = LOCAL[i][j];
//			}
//			saiz[i] = LOCAL_SAIZ[i]; 
//			route_cost[i] = LOCAL_Rcost[i];
//			total_demand[i] = LOCAL_capa[i];
//			space_available[i] = (CAPACITY*(1+violateThreshold)) - (total_demand[i]);
//			distance_available[i] = (DISTANCE*(1+violateThreshold)) - (distance_cost[i]);
//			//space_available[i] = CAPACITY - total_demand[i];
//			//distance_available[i] = DISTANCE - distance_cost[i];
//		}
//		copyalready = true; 
//		//============= End of Copy VEHICLE from LOCAL_BEST ===========//
//		
//		find_all_cost_of_removing (cost_of_removing);//cost_of_removing[][] only used in LS, not shaking, this is the first time calculating cost_of_removing
//
//		int Neighbour_k = 1; //Neighbour_k 1 = (1-0), Neighbour_k 2 = (1-1), Neighbour_k 3 = (2-1), Neighbour_k 4 = (2-0)
//	
//		int total_time = 0;
//		float maxCPUtime = 10000/(CLOCKS_PER_SEC);
//		cout << "maxCPUtime= " <<maxCPUtime<<endl;
//		bool foundLOCALinCurrNeighbor = false;//Do Dijkstra refinement if found LOCAL BEST in any neighbourhood in this diversification, otherwise, just do refinement for VEHICLE
//	//###################################### STEP 3(a) Shaking ######################################//
//	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ k+1 neighborhood starts here @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@//
//	while (Neighbour_k <= max_neigbor)	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//	{	
//		//module: //1) after local search if no improvement, Neighbour_k++ goto module //this is k+1 in the algorithm (move to next neighborhood)
//		if (violateThreshold < 0.01) //if smaller than 1%, make it zero
//			violateThreshold = 0;
//
//		route_CGravity = new float*[no_routes];
//		for (int i = 0; i < no_routes; i++)
//		{
//			route_CGravity[i] = new float[2]; //2 columns consists of x and y
//		}
//		custRGravity = new float[LOCAL_NO_ROUTE];
//		sorted_custRGravity = new float*[LOCAL_NO_ROUTE];
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			custRGravity[i] = 0; //initialize
//			sorted_custRGravity[i] = new float[5]; //[0]sorted distance, [1]sorted route index, [2]1/distance, [3]d/sum(d), [4]cumulative 
//		}
//
//	//reshake: //if no feasible shake, Neighbour_k++ and goto reshake //put comment on 14Oct2015, put reshake after copy
//		//for (int i = 0; i < 5; i++) //reinitialize after_shake_route_change[6]
//		//{
//		//	after_shake_route_change[i] = -1;
//		//}
//		//after_shake_route_changePtr = 0; //reinitialize pointer to 0, to keep track how many routes have been changed
//
//		//VEHICLE copied from LOCAL_BEST_ROUTE to be passed to shake(), need to recopy because when neighbourhoood++, VEHICLE[][] need to recopy from LOCAL_BEST although the first iteration this is not necessary because VEHICLE already copy before	
//		if (copyalready == false)//if first time, VEHICLE already copy from LOCAL_BEST, noneed to copy, added on 12Oct2015
//		{
//			no_routes = LOCAL_NO_ROUTE;
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)//added 22Apr15 
//			{
//				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//				{
//					VEHICLE[i][j] = LOCAL[i][j];	
//				}
//				saiz[i] = LOCAL_SAIZ[i]; 
//				route_cost[i] = LOCAL_Rcost[i]; 
//				total_demand[i] = LOCAL_capa[i];
//				space_available[i] = (CAPACITY*(1+violateThreshold)) - (total_demand[i]);
//				distance_available[i] = (DISTANCE*(1+violateThreshold)) - (distance_cost[i]);
//				//space_available[i] = CAPACITY - total_demand[i];
//				//distance_available[i] = DISTANCE - distance_cost[i];
//			}
//		}
//		copyalready = false;//reinitialize to false after first time
//
//		calculate_centreGravity(x, y); //MUST PUT THIS ONE HERE TO AVOID CALCULATING EVERYTIME IN EACH SHAKE
//
//	reshake: //if no feasible shake, Neighbour_k++ and goto reshake
//		//for (int i = 0; i < 5; i++) //reinitialize after_shake_route_change[6]
//		//{
//		//	after_shake_route_change[i] = -1;
//		//}
//		//after_shake_route_changePtr = 0; //reinitialize pointer to 0, to keep track how many routes have been changed
//
//		int shake_status1=1;
//		int shake_status2=1;
//		if (Neighbour_k == 1) //(1-0)
//		{
//			cout<<"Entering Neighbour_k == 1"<<endl;
//			shake_status1 = shake_1_0(VEHICLE, x, y); //the one going to shake is always the current best solution
//			shake_status2 = shake_1_0(VEHICLE, x, y); //the one going to shake is always the current best solution
//			if ((shake_status1 == 0) && (shake_status2 == 0))
//			{
//				Neighbour_k++;
//				goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
//			}
//		}		
//		else if (Neighbour_k == 2) //1-1
//		{	
//			cout<<"Entering Neighbour_k == 2"<<endl;
//			shake_status1 = shake_1_1(VEHICLE, x, y);
//
//			if (shake_status1 == 0)
//			{
//				Neighbour_k++;
//				goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
//			}
//		}
//		else if (Neighbour_k == 3) //(2-0) - a pair from route A go to route B
//		{			
//			cout<<"Entering Neighbour_k == 3"<<endl;
//			shake_status1 = shake_2_0_twoR(VEHICLE, x, y);
//			shake_status2 = shake_1_1(VEHICLE, x, y);
//			if ((shake_status1 == 0) && (shake_status2 == 0))
//			{
//				Neighbour_k++;
//				goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
//			}
//		}
//		else if (Neighbour_k == 4) //(2-0) - a pair from route A go to route B and route C
//		{			
//			cout<<"Entering Neighbour_k == 4"<<endl;
//			shake_status1 = shake_2_0_threeR(VEHICLE, x, y);
//
//			if (shake_status1 == 0)
//			{
//				Neighbour_k++;
//				goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
//			}
//		}
//		else if (Neighbour_k == 5) //(2-1) - a pair from route A swap with one customer from route B
//		{			
//			cout<<"Entering Neighbour_k == 5"<<endl;
//			shake_status1 = shake_2_1_twoR(VEHICLE, x, y);
//			shake_status2 = shake_1_0(VEHICLE, x, y);
//			if ((shake_status1 == 0) && (shake_status2 == 0))
//			{
//				Neighbour_k++;
//				goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
//			}
//		}
//
//		else if (Neighbour_k == 6) //(2-1) - first pair of cust must from route A to B, another cust from route A can go to route C
//		{			
//			cout<<"Entering Neighbour_k == 6"<<endl;
//			shake_status1 = shake_2_1_threeR(VEHICLE, x, y);
//			shake_status2 = shake_1_0(VEHICLE, x, y);
//			if ((shake_status1 == 0) && (shake_status2 == 0))
//			{
//				Neighbour_k++;
//				goto reshake;//if no feasible shake, Neighbour_k++ and goto reshake
//			}
//		}
//		else if (Neighbour_k == 7) //(2-2) 
//		{
//			cout<<"Entering Neighbour_k == 7"<<endl;
//			shake_status1 = shake_2_2(VEHICLE, x, y);
//		
//			if (shake_status1 == 0)
//			{
//				Neighbour_k++;
//				//goto check_sol; //if no feasible shake at last neighborhod, dont perform LS, goto check_sol
//										//VEHICLE copied from vehicle to be passed to reoptimize() later //made comment on 15June2015
//				goto check_LOCAL_BEST; //if no feasible shake at last  neighborhod, dont perform LS, goto check_LOCAL_BEST, but need to copy vehicDepoCust[][] from vehicle[][] //added 15June2015 //check this if correct
//			}
//					
//		}
//		for (int r = 0; r < no_routes; r++)
//		{
//			cout<<"Route changed are: "<< ' ';
//			if (RchangedStatus[r] == true)
//			//if (Route[r].RchangedStatus == 1)
//			{
//				cout<<r<<' ';
//			}		
//			cout<<endl; 
//		}
//		cout<<"After shake " <<Neighbour_k <<endl;
//		cout<<endl<<"================================"<<endl;
//		localbest_sol<<endl<<"================================"<<endl;
//		cout<<"VEHICLE after shake " <<endl;
//		localbest_sol<<"VEHICLE after shake " <<endl;
//		for (int i = 0; i < no_routes; i++)
//		{
//			cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//			localbest_sol<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//			for (int j = 0; j <= saiz[i]+1; j++)
//			{
//				cout<<VEHICLE[i][j]<<' ';
//				localbest_sol<<VEHICLE[i][j]<<' ';
//			}
//			cout<<endl;
//			localbest_sol<<endl;
//		}
//		float afterShake_cost = 0.0; //to record the current cost so that later after kth improvement can check if the solution got improved
//		for (int i = 0; i < no_routes; i++)
//		{
//			afterShake_cost = afterShake_cost + route_cost[i];
//		}
//		cout<<"afterShake_cost= "<<afterShake_cost<<endl;
//		localbest_sol<<"afterShake_cost= "<<afterShake_cost<<endl;
//
//		//========================= To update cost_of_removing, gain_matrix[] and info-matrix[][] after shake===============================//
//		partialupdate_costremoving(cost_of_removing);
//		reinitializeRchangedStatus (); //initialize to 0 for every route
//
//		//VEHICLE[][] copy from vehicle[][] after shake
//		//int r1 = route_change[0];
//		//int r2 = route_change[1];
//		//int r3 = route_change[2];
//		//
//		//cout<<"r1= " <<r1 << ' ' << "r2= " <<r2 << ' ' << "r3= " <<r3 << endl;
//		//cout<<"After shake " <<Neighbour_k <<endl;
//		//
//		//for (int r = 0; r < after_shake_route_changePtr; r++)
//		//{
//		//	cout<<"r" <<r<<" = "<< after_shake_route_change[r] <<' '; 
//		//}
//		//cout<<"After shake " <<Neighbour_k <<endl;
//
//		////=================================================update cost of removing=====================================//////////////////////
//		//for (int r = 0; r < after_shake_route_changePtr; r++)
//		//{
//		//	int f = after_shake_route_change[r];	//this store the route number
//		//	if (saiz[f]==0)
//		//		continue;
//		//	for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
//		//	{
//		//		cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
//		//	}
//		//}
//		//find_all_cost_of_removing (cost_of_removing);//cost_of_removing[][] only used in LS, not shaking
//	
//		//=========== declare data structure ======================//
//		operand = new fullsearch[max_level+1]; //there are (max_level) of operand that contains info in struct , max_level start from 1
//		for (int i = 1; i <= max_level; i++)
//		{
//			operand[i].eachRfullsearchstatus = new int* [no_routes];
//		}
//			
//		for (int i = 1; i <= max_level; i++)
//		{
//			for (int j = 0; j < no_routes; j++)
//			{
//				operand[i].eachRfullsearchstatus[j] = new int [no_routes];
//			}
//		}
//		initializeEachRfullsearchstatus (max_level);
//
//		/////////////// ================================ INITIALISATION FOR LS ==================================================== ////////////////////////////
//		int minLS = 2;
//		int maxLS = 4;
//		int k = 5; //kth improvement, find the best value from K gain!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//		int level = 1; //initialize local search to 1
//		bool sameroute = false; //false if different route, true if same route
//		int no_attribute = 13; //[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12][11]gain2 in distance
//	
//		float **kGain = new float* [k];
//		for (int i = 0; i < k; i++)
//		{
//			kGain[i] = new float [no_attribute];
//		}
//
//		//Adaptive goes here, generate randNum for module, then generate the module and sequence
//		//adjustFreq(freq, maxLocalSearch); //if probability = 0, add 0.05 so that it will have a small chance to be selected
//		cout<<"=========== Frequency of gain in Multi-level ============"<<endl;
//		recordFreq<<"============ Frequency of gain IN MULTI_LEVEL ==============="<<endl;
//		for (int i = 1; i <= maxLocalSearch; i++) //freq[i] starts from 1
//		{
//			cout<<"Freq["<<i<<"]= "<<freq[i]<<' '<<' ';
//			recordFreq<<"Freq["<<i<<"]= "<<freq[i]<<' '<<' ';
//		}
//		recordFreq<<endl;
//
//		float *cutPoint = new float[maxLocalSearch+1]; //from 0 to 1 inclusive // 8 modules, 9 cutpoint
//		int rand_numLS=0; //number of LS
//		bool *flag_m = new bool[maxLocalSearch+1]; // starts from 1
//		for (int i = 0; i <= maxLocalSearch; i++)//flag_m[0] has no meaning because modul starts from 1
//		{
//			flag_m[i]=false; //initially all false
//		}
//		//int *modu = new int[rand_numLS+2]; //modul[] starts from 1;//+1 to add last module 2-opt, compulsory LS //cannot declare here because rand_numLS is unknown!! cause heap error when delete!!!!!!!!!!!!
//		/////////////// ================================ End of INITIALISATION FOR LS ==================================================== ///
//
//		//determine how many module
//		srand ( time(NULL) ); //seed it
//	
//		rand_numLS = (rand() % (maxLS-minLS+1))+minLS; //generate number of LS between minLS and maxLS
//		//rand_numLS = (rand() % maxLocalSearch)+1; //generate number of LS 
//		int *modu = new int[rand_numLS+2]; //modul[] starts from 1;//+1 to add last module 2-opt, compulsory LS
//		cout<<"number of LS = " <<rand_numLS<<endl;
//		recordFreq<<"number of LS = " <<rand_numLS<<endl;
//		float rand_m; //generate uniform number
//		int m; //m is the module returned from find_module()
//		//generate the module based on cumulative probability, always select the remaining module based on flag
//		for (int i = 1; i <= rand_numLS; i++)
//		{
//			findCumu(freq, maxLocalSearch, cutPoint, flag_m);
//			//srand ( time(NULL) ); //seed it
//			rand_m = (float) rand() / float(RAND_MAX); //generate a number between 0 and 1
//			cout<<"rand_m= "<<rand_m << endl;
//			recordFreq<<"rand_m= "<<rand_m << endl;
//			m = find_module (rand_m, cutPoint, maxLocalSearch, flag_m); //m is the module returned 
//			modu[i] = m;
//			flag_m[m]=true; //flag it to true so that it would not be selected again
//		}
//		modu[rand_numLS+1] = max_level; //2-opt is compulsory modul
//		rand_numLS = rand_numLS+1; //add one to accomodate 2-opt as the last LS
//		for (int i = 1; i <= rand_numLS; i++) //rand_numLS has added 1 now!!!!!!!!!!!!
//		{
//			cout<<"modu["<<i<<"]= "<<modu[i]<<' '<<' ';
//			recordFreq<<"modu["<<i<<"]= "<<modu[i]<<' '<<' ';
//			 
//		}
//		cout<<endl;
//		recordFreq<<endl;
//
//		int I=1;//to keep track of modu[I] - which local search to be performed
//
//	while (I <= rand_numLS)
//	{
//		//assign level
//	//redo_LS: //1) if no improvement found in each level of LS, 2) if cost is better than aftershake cost set level=1, 3) if LS level has not finished all level level++ and goto redo_LS
//		level= modu[I];
//		cout<<"level= "<<level<<endl;
//		if (level == 1) //the first level is 1-0 same route
//		{
//			//operand[level].levelID = level;
//			int number = find_k_1_0_sameR(cost_of_removing, kGain, k, level);
//			cout<<"in level "<<level<<": find_k_1_0_sameR, best number= "<<number << endl;
//			
//			sameroute = true;//same route
//			
//			if (number == -1)//if no improvement
//			{
//				//operand[level].fullsearchstatus = 1;
//				I++;
//				continue;
//				//goto redo_LS;//if no improvement found in each level of LS
//			}
//			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP= "<<kGain[number][4]<<" ToP= "<<kGain[number][5]<< endl;
//			
//			initializeEachRfullsearchstatus (max_level);
//
//			//cost_of_removing is updated in insert()
//			insert_1_0_2nd(cost_of_removing, k, number, kGain, VEHICLE, sameroute);
//			cout<<endl<<endl;
//			cout<<"========== vehicle after insert in level 1 find_k_1_0_sameR ==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//
//		else if (level == 2)
//		{
//			//operand[level].levelID = level;
//			int number = find_k_1_0(cost_of_removing, kGain, k, level);
//			cout<<"in level "<<level<<": find_k_1_0, best number= "<<number << endl;
//			sameroute = false; //different route
//			if (number == -1)//if no improvement
//			{
//				I++;
//				continue;
//			}
//			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP= "<<kGain[number][4]<<" ToP= "<<kGain[number][5]<< endl;
//			
//			initializeEachRfullsearchstatus (max_level);
//			//cost_of_removing is updated in insert()
//			insert_1_0_2nd(cost_of_removing, k, number, kGain, VEHICLE, sameroute);
//			cout<<endl<<endl;
//			cout<<"========== vehicle after insert in level 2 find_k_1_0==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//
//		else if (level == 3)
//		{
//			//operand[level].levelID = level;
//			int number = find_k_1_1_sameR(cost_of_removing, kGain, k, level);
//			cout<<"in level "<<level<<": find_k_1_1_sameR, best number= "<<number << endl;
//			sameroute = true; //same route
//			if (number == -1)//if no improvement
//			{
//				I++;
//				continue;
//			}
//			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP= "<<kGain[number][4]<<" ToP= "<<kGain[number][5]<< endl;
//			
//			initializeEachRfullsearchstatus (max_level);
//			//cost_of_removing is updated in insert()
//			insert_1_1_2nd(cost_of_removing, k, number, kGain, VEHICLE, sameroute);
//			cout<<endl<<endl;
//			cout<<"========== vehicle after insert in level 3: find_k_1_1_sameR==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//
//		else if (level == 4)
//		{
//			//operand[level].levelID = level;
//			//int number = -1;
//			//if (rand() % 2 == 0)
//			//{
//				int number = find_k_1_1(cost_of_removing, kGain, k, level);//in every position
//				cout<<"in level "<<level<<": find_k_1_1, best number= "<<number << endl;
//			//}
//			//else 
//			//{
//			//	number = find_swap_1_1(cost_of_removing, kGain, k, level);//in every position
//			//	cout<<"in level 4: find_swap_1_1, best number= "<<number <<endl;
//			//}
//
//			sameroute = false; //different route
//			if (number == -1)//if no improvement
//			{
//				//operand[level].fullsearchstatus = 1;
//				I++;
//				continue;
//				//goto redo_LS;//if no improvement found in each level of LS
//			}
//			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//			
//			initializeEachRfullsearchstatus (max_level);
//			//cost_of_removing is updated in insert()
//			insert_1_1_2nd(cost_of_removing, k, number, kGain, VEHICLE, sameroute);
//			cout<<endl<<endl;
//			cout<<"========== vehicle after insert in level 4: find_k_1_1==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//
//		else if (level == 5)
//		{
//			//operand[level].levelID = level;
//			int number = find_k_2_1_sameR(cost_of_removing, kGain, k, level);
//			cout<<"in level "<<level<<": find_k_2_1_sameR, best number= "<<number << endl;
//			sameroute = true; //same route
//			if (number == -1)//if no improvement
//			{
//				//operand[level].fullsearchstatus = 1;
//				I++;
//				continue;
//				//goto redo_LS;//if no improvement found in each level of LS
//			}
//			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP= "<<kGain[number][4]<<" ToP= "<<kGain[number][5]<< endl;
//			
//			initializeEachRfullsearchstatus (max_level);
//			//cost_of_removing is updated in insert()
//			insert_2_1_2nd(cost_of_removing, k, number, kGain, VEHICLE, sameroute);
//			cout<<endl<<endl;
//			cout<<"========== vehicle after insert in level 5: find_k_2_1_sameR==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//
//		else if (level == 6)
//		{
//			//operand[level].levelID = level;
//			int number = find_k_2_1(cost_of_removing, kGain, k, level);
//			cout<<"in level "<<level<<": find_k_2_1, best number= "<<number << endl;
//			sameroute = false; //different route
//			if (number == -1)//if no improvement
//			{
//				//operand[level].fullsearchstatus = 1;
//				I++;
//				continue;
//				//goto redo_LS;//if no improvement found in each level of LS
//			}
//			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//			
//			initializeEachRfullsearchstatus (max_level);
//
//			//cost_of_removing is updated in insert()
//			insert_2_1_2nd(cost_of_removing, k, number, kGain, VEHICLE, sameroute);
//			cout<<endl<<endl;
//			cout<<"========== vehicle after insert in level 6: find_k_2_1==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//
//		else if (level == 7)
//		{
//			//operand[level].levelID = level;
//			int number = find_k_2_0_sameR(cost_of_removing, kGain, k, level);
//			cout<<"in level "<<level<<": find_k_2_0_sameR, best number= "<<number << endl;
//			sameroute = true; //same route
//			if (number == -1)//if no improvement
//			{
//				//operand[level].fullsearchstatus = 1;
//				I++;
//				continue;
//				//goto redo_LS;//if no improvement found in each level of LS
//			}
//			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP= "<<kGain[number][4]<<" ToP= "<<kGain[number][5]<< endl;
//			
//			initializeEachRfullsearchstatus (max_level);
//
//			//cost_of_removing is updated in insert()
//			insert_2_0_2nd(cost_of_removing, k, number, kGain, VEHICLE, sameroute);
//			cout<<endl<<endl;
//			cout<<"========== vehicle after insert in level 5: find_k_2_0_sameR==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//		
//		else if (level == 8)
//		{
//			//operand[level].levelID = level;
//			int number = find_k_2_0(cost_of_removing, kGain, k, level);
//			cout<<"in level "<<level<<": find_k_2_0, best number= "<<number << endl;
//			sameroute = false; //different route
//			if (number == -1)//if no improvement found
//			{
//				//operand[level].fullsearchstatus = 1;
//				I++;
//				continue;
//				//goto redo_LS;//if no improvement found in each level of LS
//			}
//			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//			
//			initializeEachRfullsearchstatus (max_level);
//
//			//cost_of_removing is updated in insert()
//			insert_2_0_2nd(cost_of_removing, k, number, kGain, VEHICLE, sameroute);
//			cout<<endl<<endl;
//			cout<<"========== vehicle after insert in level 8: find_k_2_0==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//		else if (level == 9)
//		{
//			//operand[level].levelID = level;
//			int number = find_k_2_2_swapsameR(cost_of_removing, kGain, k, level);
//			cout<<"in level "<<level<<": find_k_2_2_swapsameR, best number= "<<number << endl;
//			sameroute = true; //same route
//			if (number == -1)//if no improvement
//			{
//				//operand[level].fullsearchstatus = 1;
//				I++;
//				continue;
//				//goto redo_LS;//if no improvement found in each level of LS
//			}
//			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP= "<<kGain[number][4]<<" ToP= "<<kGain[number][5]<< endl;
//			
//			initializeEachRfullsearchstatus (max_level);
//
//			//cost_of_removing is updated in insert()
//			insert_2_2_swap2nd(cost_of_removing, k, number, kGain, VEHICLE, sameroute);
//			cout<<endl<<endl;
//			cout<<"========== vehicle after insert in level"<<level<<": find_k_2_2_swapsameR==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//		
//		else if (level == 10)
//		{
//			//operand[level].levelID = level;
//			int number = find_k_2_2swap(cost_of_removing, kGain, k, level);
//			cout<<"in level "<<level<<": find_k_2_2swap, best number= "<<number << endl;
//			sameroute = false; //different route
//			if (number == -1)//if no improvement found
//			{
//				//operand[level].fullsearchstatus = 1;
//				I++;
//				continue;
//				//goto redo_LS;//if no improvement found in each level of LS
//			}
//			cout<<" Gain= "<<kGain[number][1]<<" FromR= "<<kGain[number][2]<<" ToR= "<<kGain[number][3]<<" FromP_r1= "<<kGain[number][4]<<" ToP_r2= "<<kGain[number][5]<<" FromP_r2= "<<kGain[number][6]<<" ToP_r1= "<<kGain[number][7]<< endl;
//			
//			initializeEachRfullsearchstatus (max_level);
//
//			//cost_of_removing is updated in insert()
//			insert_2_2_swap2nd(cost_of_removing, k, number, kGain, VEHICLE, sameroute);
//			cout<<endl<<endl;
//			cout<<"========== vehicle after insert in level "<<level<<": find_k_2_2swap==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//		else if (level == 11)
//		{
//			//operand[level].levelID = level;
//			int *routeChange = new int[no_routes];
//			int num_routeChange = 0;
//			cout<<"VEHICLE Before entering 2-opt in VNS Adaptive"<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<< ' ';
//				}
//				cout<<endl;
//			}
//			num_routeChange = two_opt(VEHICLE, routeChange);
//			//update cost of removing
//			cout<<"Out of 2-opt"<<endl;
//			if (num_routeChange != 0)
//			{
//				for (int i = 0; i < num_routeChange; i++)
//				{
//					int f = routeChange[i];
//					
//					initializeEachRfullsearchstatus (max_level);
//
//					//cost of removing for single customer
//					int before = -1, after = -1;
//					for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
//					{
//						before = VEHICLE[f][h - 1];
//						after = VEHICLE[f][h + 1];
//						cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];
//					}
//
//				}
//			
//			}
//			cout<<"========== VEHICLE after 2-opt in level 9: 2-opt intra route==================="<<endl;
//			for (int i = 0; i < no_routes; i++)
//			{
//				cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					cout<<VEHICLE[i][j]<<' ';
//				}
//				cout<<endl;
//			}
//			
//			//added 2-optInter on 1July2015
//			//num_routeChange = two_optInter(VEHICLE, routeChange);
//			num_routeChange = 0;//reinitialize
//			num_routeChange = two_optInter2(VEHICLE, routeChange);
//			//update cost of removing
//			cout<<"Out of 2-optInter"<<endl;
//			if (num_routeChange != 0)
//			{
//				for (int i = 0; i < num_routeChange; i++)
//				{
//					int f = routeChange[i];
//					
//					initializeEachRfullsearchstatus (max_level);
//					//cost of removing for single customer
//					int before = -1, after = -1;
//					for (int h = 1; h <= saiz[f] ; h++) //first customer start from element [1]
//					{
//						before = VEHICLE[f][h - 1];
//						after = VEHICLE[f][h + 1];
//						
//						cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];
//					}
//				}
//				cout<<"========== VEHICLE after 2-opt in level 9: 2-opt inter route==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//			}
//			
//			//added crossTail2 on 10July2015
//			num_routeChange = 0;//reinitialize
//			num_routeChange = crossTail2(VEHICLE, routeChange);
//			//update cost of removing
//			cout<<"Out of crossTail2"<<endl;
//			if (num_routeChange != 0)
//			{
//				for (int i = 0; i < num_routeChange; i++)
//				{
//					int f = routeChange[i];
//
//					initializeEachRfullsearchstatus (max_level);
//					//cost of removing for single customer
//					int before = -1, after = -1;
//					for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
//					{
//						before = VEHICLE[f][h - 1];
//						after = VEHICLE[f][h + 1];
//						
//						cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];
//					}
//				}
//				cout<<"========== vehicle after crossTail2 in level 9: crossTail2==================="<<endl;
//				for (int i = 0; i < no_routes; i++)
//				{
//					cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						cout<<VEHICLE[i][j]<<' ';
//					}
//					cout<<endl;
//				}
//			}
//			srand ( time(NULL) ); //seed it
//			if (rand()%2 == 0)//only perform CROSS base don probability
//			{
//				num_routeChange = 0;//reinitialize
//				num_routeChange = CROSS(VEHICLE, routeChange);
//				//update cost of removing
//				cout<<"Out of CROSS"<<endl;
//				if (num_routeChange != 0)
//				{
//					for (int i = 0; i < num_routeChange; i++)
//					{
//						int f = routeChange[i];
//						
//						initializeEachRfullsearchstatus (max_level);
//						//cost of removing for single customer
//						int before = -1, after = -1;
//						for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
//						{
//							before = VEHICLE[f][h - 1];
//							after = VEHICLE[f][h + 1];
//						
//							cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];
//						}
//					}
//					cout<<"========== VEHICLE after CROSS in level 9: CROSS==================="<<endl;
//					for (int i = 0; i < no_routes; i++)
//					{
//						cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//						for (int j = 0; j <= saiz[i]+1; j++)
//						{
//							cout<<VEHICLE[i][j]<<' ';
//						}
//						cout<<endl;
//					}
//				}
//			}
//			delete[] routeChange;
//			goto check_sol;//after each LS , compare with aftershakecost
//		}
//
//	check_sol://after each LS or if no feasible shake at last module(neighborhod), goto check_sol, compare with aftershakecost
//
//		//======================================================Display route=======================================================================//
//
//		cout << "================= Routes (in VNS Adaptive) after LS Level " << level <<" ===================================== " << endl;
//		//route_file << "==================Routes (in VNS Adaptive) after LS Level " << level <<" ===================================== " << endl;
//		float total_cost = 0.0;
//		int total_cust = 0;
//		int sum_demand=0;
//		for (int g = 0; g < no_routes; g++)
//		{
//			if (saiz[g] == 0)
//			{
//				route_cost[g] = 0;
//				distance_cost[g] = 0;
//			}
//			cout<<g<<' '<<route_cost[g]<<' '<<saiz[g]<<' '<<total_demand[g]<<' '<<' ';
//			for (int h = 0; h <= saiz[g]+1; h++)
//			{
//				cout << VEHICLE[g][h] << ' ';		
//			}
//			cout << endl;
//
//			total_cust = total_cust+saiz[g];
//			total_cost = total_cost + route_cost[g];
//			sum_demand = sum_demand+total_demand[g];
//			
//			if (total_demand[g] < 0)
//				getchar();
//			//route_file << "d=" << total_demand[g] <<endl;
//		}
//		cout << "Total customers = " << total_cust << endl<< "Total cost = " << total_cost << endl<< "Sum_demand = " << sum_demand <<endl<< "TOTAL_DEMAMD = " << TOTAL_DEMAND<<endl;
//		//route_file<< "Total customers = " << total_cust << endl<< "Total cost = " << total_cost << endl<< "Sum_demand = " << sum_demand <<endl<< "TOTAL_DEMAMD = " << TOTAL_DEMAND<<endl;
//		//route_file<< "After shake cost = " <<afterShake_cost <<endl;
//		//route_file<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
//	
//		if ((afterShake_cost - total_cost) > epsilonn)//t_cost < afterShake_cost
//		{
//			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ for infeasibility check @@@@@@@@@@@@@@@@@@@@@@@@//
//			for (int i = 0; i < no_routes; i++)
//			{
//				if ((total_demand[i]>CAPACITY) || (distance_cost[i] - DISTANCE > epsilon))//for infeasibility check
//				{
//					violation << "In VNS multi-level after module "<< Neighbour_k <<endl;
//					violation << i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//					for (int j = 0; j <= saiz[i]+1; j++)
//					{
//						violation << VEHICLE[i][j]<<' ';
//					}
//					violation<<endl;
//					violateThreshold = violateThreshold/2; //divide by half if the solution after LS still violate constraint
//					//if (Neighbour_k < max_neigbor)
//					//{
//						Neighbour_k++; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//						goto skip_check_local;
//					
//					//}
//					//else //if maximum neighbourhood is reached
//					//	goto violateDiverstart; //if violation, skip all the process of compare LOCAL_BEST and GLOBAL_BEST, check if max diversification is reached
//				}
//			}
//			//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ End of (for infeasibility check) @@@@@@@@@@@@@@@@@@@@@@@@//
//			//try checking fesibility here next time!!!!!!!!!!!!!!!! to avoid  time wasted if route is infeasible after so many LS
//			//if((abs(t_cost-LOCAL_BEST_COST) <= 0.05) || (abs(t_cost-GLOBAL_BEST_COST) <= 0.05)) //added on 3April2015 to avoid recalculate if it is the same as LOCAl or GLOBAL BEST
//			//	goto check_local_best; //compare with LOCAL_BEST_COST after all LS are finished
//			afterShake_cost = total_cost;
//			I=1;
//			//goto redo_LS;//2) if cost is better than aftershake cost set level=1
//		}
//	
//		else
//		{
//			I = I+1;
//			//if (I > rand_numLS) //when module has been completed, including last module, 2opt
//			//	goto check_local_best; //compare with LOCAL_BEST_COST after all LS are finished
//			//goto redo_LS; //3) if LS level has not finished all level level++ and goto redo_LS
//		}
//	}//end while (I <= rand_numLS)
//
//	check_LOCAL_BEST://if all LS are done, goto check_LOCAL_BEST //compare with LOCAL_BEST_COST
//		//======================================================Display route========================================================================
//		//route_file<< "==================Routes (in VNS Adaptive) after all LS before check LOCAL BEST===================================== " << endl;
//		cout << "==================Routes (in VNS Adaptive) after all LS before check LOCAL BEST===================================== " << endl;
//		cout<< "Current Neighbour_k is "<<Neighbour_k<<endl;
//
//		float total_cost = 0.0;
//		int total_cust = 0;
//		for (int g = 0; g < no_routes; g++)
//		{
//			if (saiz[g] == 0)
//			{
//				route_cost[g] = 0;
//				distance_cost[g] = 0;
//			}
//		
//			total_cust = total_cust + saiz[g];
//			cout<<g<<' '<<route_cost[g]<<' '<<saiz[g]<<' '<<total_demand[g]<<' '<<' ';
//			for (int h = 0; h <= saiz[g]+1; h++)
//			{
//				cout << VEHICLE[g][h] << ' ';
//			}
//			cout << endl;
//			total_cost = total_cost + route_cost[g];
//		}
//		cout << "Total customers = " << total_cust << endl << "Total cost = " << total_cost << endl;
//		cout<<"LOCAL_BEST_COST = " << LOCAL_BEST_COST <<endl<<"GLOBAL_BEST_COST = " << GLOBAL_BEST_COST <<endl;
//
//
//	
//		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ for infeasibility check @@@@@@@@@@@@@@@@@@@@@@@@//
//		
//		for (int i = 0; i < no_routes; i++)
//		{
//			if ((total_demand[i]>CAPACITY) || (distance_cost[i] - DISTANCE > epsilon))//for infeasibility check
//			{
//				cout << "Infeasibility check In VNS adaptive in Neighbour "<< Neighbour_k <<endl;
//				violation << "In VNS adaptive after module "<< Neighbour_k <<endl;
//				violation << i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					violation << VEHICLE[i][j]<<' ';
//				}
//				violation<<endl;
//				violateThreshold = violateThreshold/2; //divide by half if the solution after LS still violate constraint
//				//if (Neighbour_k != max_neigbor)
//				//{
//					Neighbour_k++;
//					goto skip_check_local;
//				//}
//				//else //if maximum neighbourhood is reached
//				//{
//				//	goto violateDiverstart; //if violation, skip all the process of compare LOCAL_BEST and GLOBAL_BEST, check if max diversification is reached
//				//}
//			}
//		}
//		//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ End of (for infeasibility check) @@@@@@@@@@@@@@@@@@@@@@@@//
//
//		if ((LOCAL_BEST_COST - total_cost) > epsilonn )//t_cost < LOCAL_BEST_COST
//		{
//			//reinitialize vehicle[][] so that it can be passed to shaking afterwards
//			//============= Record the best solution ====================//
//			//sort_solution(VEHICLE); //dont sort here, cost_of_removing will be affected
//			LOCAL_BEST_COST = total_cost;
//			
//			for (int i = 0; i < no_routes; i++)
//			{
//				LOCAL_SAIZ[i] = saiz[i];
//				LOCAL_capa[i] = total_demand[i];
//				LOCAL_Rcost[i] = route_cost[i];
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					LOCAL[i][j] = VEHICLE[i][j];	
//				}
//			}
//			
//			if (no_routes != LOCAL_NO_ROUTE)
//			{
//				cout<<"ERROR no_routes != LOCAL_NO_ROUTE in Adaptive VNS)"<<endl;
//				getchar();
//			}
//			//LOCAL_NO_ROUTE = no_routes;//number of routes is the same as LOCAL_NO_ROUTE, noneed to initialize
//
//			//=================================record localbest solution in txt=====================================//
//			cout << "========================================================================== " << endl;
//			//=================================display final solution=====================================//
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			{
//				localbest_sol << i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
//				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//				{
//					localbest_sol << LOCAL[i][j] << ' ';
//				}
//				localbest_sol<<endl;
//			}
//			localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"in VNS multi-level adaptive after shake Neighbour_k "<<Neighbour_k <<"level " <<level<< endl;	
//			for (int i = 1; i <= rand_numLS; i++)
//			{
//				localbest_sol <<"modu["<<i<<"]= "<<modu[i]<<' '<<' ';
//			}
//			localbest_sol <<endl;
//			//once found local best, copy to Oricost_of_removing, //if not better than local best solution and neighbourhood havent finished, use the Oricost_of_removing because shaking is based on local best, oriGain is updated evrytime found local best
//			//============================= copy to Oricost_of_removing ================================//
//			for (int i = 0; i < SIZE; i++)
//			{
//				Oricost_of_removing[i] = cost_of_removing[i];
//			}
//			calculate_centreGravity(x, y);
//			Neighbour_k = 1;
//			
//		}//end if (total cost < LOCAL_BEST_COST)
//		else  //if is is not better than the incumbent
//		{
//			if (Neighbour_k == max_neigbor) //current neighbourhood is the last neighbourhood, go to dijkstra refinement
//			//if (Neighbour_k > max_neigbor)
//			{
//				float cost = 0.0; //to be passed to dijkstra_refinement
//				//dijkstra_refinement always refine the LOCAL_BEST_SOL (incumbent best)
//				cout<<"Entering Dijkstra refinement in Adaptive VNS"<<endl;
//				
//				if(foundLOCALinCurrNeighbor == true)//if found LOCAL BEST, use local best, else just use current VEHICLE
//				{
//					for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//					{
//						for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//						{
//							VEHICLE[i][j] = LOCAL[i][j];
//						}
//						route_cost[i] = LOCAL_Rcost[i];
//						saiz[i] = LOCAL_SAIZ[i];
//						total_demand[i] = LOCAL_capa[i];
//						space_available[i] = CAPACITY - total_demand[i];
//						distance_available[i] = DISTANCE - distance_cost[i];
//					}
//					no_routes = LOCAL_NO_ROUTE;
//				}
//				//sort_solution(VEHICLE);//sort first so that empty route will be at the end, dijkstra refinement does not consider empty route
//				//sort_solutionSMALLEST(VEHICLE);//sort first so that empty route will be at the end, dijkstra refinement does not consider empty route
//				//=============== copyto tempvec to compare whether route have changed or not after Dijkstra refinement ================//
//				//float *tempRcost = new float[no_routes];
//				//int *tempRcapa = new int[no_routes];
//				//int *tempRsize = new int[no_routes];
//				//int **tempvec = new int* [no_routes];
//				//for (int j = 0; j < no_routes; j++)
//				//{
//				//	tempvec[j] = new int [SIZE];
//				//}
//				//	
//				//for (int i = 0; i < no_routes; i++)
//				//{
//				//	tempRcost[i] = route_cost[i];
//				//	tempRcapa[i] = total_demand[i];
//				//	tempRsize[i] = saiz[i];
//				//	for (int j = 0; j <= saiz[i]+1; j++)
//				//	{
//				//		tempvec[i][j] = VEHICLE[i][j]; //copy to tempvec[][] and both tempvec[][] and vehicle[][] are sorted
//				//	}
//				//}//copyto tempvec to compare whether route have changed or not after Dijkstra refinement
//				
//				dijkstra_refinement(cost, VEHICLE); 
//				cout<<"cost after dijkstra_refinement= "<<cost<<endl;
//				
//				if ((LOCAL_BEST_COST-cost) > epsilonn)//cost < LOCAL_BEST_COST //changed t_cost to cost on 1April2015
//				{
//					cout<< "Found new LOCAL_BEST right after Dijkstra REfinement"<<endl;
//					getchar();
//					//sort_solution(VEHICLE);
//					//sort_solutionSMALLEST(VEHICLE);
//					//reinitializeRchangedStatus ();//initialize to 0 for every route
//
//					//******************* Begin of check each route has been modified or not after Dijkstra refinement *********************//
//					//for (int i = 0; i < no_routes; i++)
//					//{
//					//	if (abs(tempRcost[i] - route_cost[i]) > 0.001) //if route_cost is different
//					//	{
//					//		RchangedStatus[i] = true;
//					//		//Route[i].RchangedStatus = 1;
//					//	}
//					//	else //if cost is the same, compare size
//					//	{
//					//		if(tempRsize[i] != saiz[i]) //if saiz different, flag route
//					//		{
//					//			RchangedStatus[i] = true;
//					//			//Route[i].RchangedStatus = 1;
//					//			goto nex;//noneed to check total demand and order
//					//		}
//					//		if (tempRcapa[i] != total_demand[i])//if total demand different, flag route
//					//		{
//					//			RchangedStatus[i] = true;
//					//			//Route[i].RchangedStatus = 1;
//					//			goto nex; //noneed to check order
//					//		}
//					//			
//					//		bool reverse = false;
//					//		//check normal order
//					//		for (int w = 1; w <= saiz[i]; w++)
//					//		{
//					//			if(VEHICLE[i][w] == LOCAL[i][w])
//					//			{
//					//				reverse = false;
//					//				continue;
//					//			}
//					//			else
//					//				goto checkreverse;
//					//		}//end check normal order
//					//			
//					//		if (reverse == false)
//					//			goto nex;//confirm route no change
//					//		checkreverse:
//					//		//checkReverseSingleR(); //route maybe reverse because consider connecting endpoint in dijkstra refinement, if it is reverse, need to rewrite in original order because information recorded based on original order
//					//		int u = saiz[i];
//					//		for (int w = 1; w <= saiz[i]; w++)
//					//		{
//					//			if(VEHICLE[i][w] == LOCAL[i][u])
//					//			{
//					//				reverse = true;
//					//				continue;
//					//			}
//					//			else
//					//			{
//					//				RchangedStatus[i] = true;
//					//				//Route[i].RchangedStatus = 1;
//					//				goto nex;
//					//			}
//					//			u--;		
//					//		}//end check reverse
//					//		if(reverse == true) //if confirm it is reverse, copy the original route
//					//		{
//					//			for (int q = 1; q <= saiz[i]; q++)
//					//			{
//					//				VEHICLE[i][q] = tempvec[i][q];
//					//			}
//					//		}
//					//	}//end //if cost is the same, compare size
//					//	nex:;
//					//} 
//					////=========== update cost_of_removing, best_gain[][] if Dijkstra refinement found local_best
//					//partialupdate_costremoving(cost_of_removing);
//					//reinitializeRchangedStatus ();//initialize to 0 for every route
//					//******************* ENd of check each route has been modified or not after Dijkstra refinement *********************//
//					
//					//find all cost_of_removing and reinitialize the route because routes have been sorted and changed after Dijkstra refinement
//					find_all_cost_of_removing (cost_of_removing);//cost_of_removing[][] only used in LS, not shaking, this is the first time calculating cost_of_removing
//					reinitializeRchangedStatus ();//initialize to 0 for every route
//					initializeEachRfullsearchstatus (max_level);
//
//					//============= Record the best solution ====================//
//					LOCAL_BEST_COST = cost;
//					cout << endl << "LOCAL_BEST COST IS " << LOCAL_BEST_COST << endl;
//
//					for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//					{
//						for (int j = 0; j <= saiz[i]+1; j++)
//						{
//							LOCAL[i][j] = VEHICLE[i][j];
//						}
//			
//						LOCAL_SAIZ[i] = saiz[i]; //copy temp saiz[] to best SAIZ[]
//						LOCAL_capa[i] = total_demand[i]; // capa[] record the BEST_ROUTE demand
//						LOCAL_Rcost[i] = route_cost[i];
//					}
//					calculate_centreGravity(x, y);
//		
//					//=================================record localbest solution in txt=====================================//
//					cout << "========================================================================== " << endl;
//					//ofstream localbest_sol("30.LOCAL_BEST_SOLUTION.txt", ios::app);
//					//=================================display final solution=====================================//
//					for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//					{
//						cout<< i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
//						localbest_sol << i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
//						for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//						{
//							cout<< LOCAL[i][j] << ' ';
//							localbest_sol << LOCAL[i][j] << ' ';
//						}
//						cout<<endl;
//						localbest_sol<<endl;
//					}
//					cout << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
//					localbest_sol << "LOCAL_BEST COST= " << LOCAL_BEST_COST <<' '<<"after Dijkstra Refinement" << endl;	
//					
//					localbest_sol<<"FOUND LOCAL BEST AFTER DIJKSTRA REFINEMENT, UPDATE ORIGAIN"<<endl;
//					//============================= copy to oriGain because oriGain always hold the gain for local_best, it is used in shaking later================================//
//					for (int i = 0; i < SIZE; i++)
//					{
//						Oricost_of_removing[i] = cost_of_removing[i];
//					}
//					//localbest_sol.close();
//					//======================= End localrecord best solution ===========================//
//					Neighbour_k = 1;
//					foundLOCALinCurrNeighbor = false; //initialize to false when found LOCAL solution after Dijkstra refinement, so it will not go through Dijkstra refinement later
//					//delete[] tempRcost;
//					//for (int i = 0; i < no_routes; i++)
//					//{
//					//	delete[] tempvec[i];
//					//}
//					//delete[] tempvec;
//					//delete[] tempRcapa;
//					//delete[] tempRsize;
//					goto skip_check_local; //if Dijkstra refinement produce better result, noneed to copy original gain matrix because we use this solution now
//		
//				}//end if (cost < LOCAL_BEST_COST)	
//			}//end if (Neighbour_k == max_neigbor) //current neighbourhood is the last neighbourhood
//			Neighbour_k++;//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //put at the back on 18June2015, previously before    if (Neighbour_k > max_neigbor)  
//			//if not better than local best solution and neighbourhood havent finished, use the original gain because shaking is based on local best, oriGain is updated evrytime found local best
//			//============================= copy from oriGain ================================//
//			cout<<"Entering copy from Oricost_of_removing because shaking is based on local best"<<endl;
//			for (int f = 0; f < SIZE; f++) //find cost of removing for each customer and put in array
//			{
//				cost_of_removing[f] = Oricost_of_removing[f];
//			}
//		}//if is is not better than the incumbent
//	
//	skip_check_local:;
//		
//		//float *cutPoint = new float[maxLocalSearch+1]; //from 0 to 1 inclusive // 8 modules, 9 cutpoint
//		//int rand_numLS=0; //number of LS
//		//bool *flag_m = new bool[maxLocalSearch+1]; // starts from 1
//		//for (int i = 0; i <= maxLocalSearch; i++)//flag_m[0] has no meaning because modul starts from 1
//		//{
//		//	flag_m[i]=false; //initially all false
//		//}
//		//
//		//rand_numLS = (rand() % (maxLS-minLS+1))+minLS; //generate number of LS between minLS and maxLS
//		//int *modu = new int[rand_numLS+2]; //modul[] starts from 1;//+1 to add last module 2-opt, compulsory LS
//
//		for (int i = 0; i < k; i++)
//		{
//			delete[] kGain[i];
//		}delete[] kGain;
//		//rand_numLS = rand_numLS-1; //before delete minus back, otherwise   got heap error //add one to accomodate 2-opt as the last LS
//		delete[] cutPoint; //from 0 to 1 inclusive // 8 modules, 9 cutpoint
//		delete[] flag_m; // starts from 1
//		delete[] modu; //modul[] starts from 1;//+1 to add last module 2-opt, compulsory LS
//		
//	}//end while (Neighbour_k <= max_neigbor) //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@	
//
//
//
//	//violateDiverstart: //if violation, skip all the process of compare LOCAL_BEST, check if max diversification is reached
//		if ((GLOBAL_BEST_COST-LOCAL_BEST_COST) > epsilonn)//LOCAL_BEST_COST < GLOBAL_BEST_COST
//		{
//			GLOBAL_BEST_COST = LOCAL_BEST_COST;
//			cout << endl << "GLOBAL_BEST COST IS " <<GLOBAL_BEST_COST << endl;
//			found_GLOBEST_status = 1; //for diversification use
//
//			//record the best solution in GLOBAL_BEST_ROUTE in sorted order
//			int r=0;
//			for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//			{
//				if (LOCAL_SAIZ[i] == 0)
//					continue;
//				for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//				{
//					GLOBAL[r][j] = LOCAL[i][j];
//				}
//				GLOBAL_Rcost[r] = LOCAL_Rcost[i];
//				GLOBAL_SAIZ[r] = LOCAL_SAIZ[i];
//				GLOBAL_capa[r] = LOCAL_capa[i];
//				r++;
//			}
//			GLOBAL_NO_ROUTE = r;
//
//
//			//=================================record the best solution=====================================//
//			cout << "========================================================================== " << endl;
//
//			//=================================display final solution=====================================//
//			best_sol <<endl<<"From VNS multi level after Neighbour_k " <<Neighbour_k<<' ' <<' '<<"NbDiv= "<<NbDiv<<  endl;
//			for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
//			{
//				best_sol << i<<' '<<GLOBAL_Rcost[i]<<' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<<' ';
//				cout << i <<' '<< GLOBAL_Rcost[i] <<' '<< GLOBAL_SAIZ[i] <<' '<< GLOBAL_capa[i]<<' '<<' ';
//				for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
//				{
//					best_sol << GLOBAL[i][j] << ' ';
//					cout << GLOBAL[i][j] << ' ';
//				}
//				best_sol << endl;
//				cout << endl;	
//			}
//			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
//			best_sol << "From VNS adaptive after Neighbour_k " <<Neighbour_k<<' ' <<' '<<"NbDiv= "<<NbDiv<<  endl;
//			cout << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
//			cout<<"Found GLOBAL_BEST"<<endl;
//			getchar();
//		}
//
//		//MUST COPY TO VEHICLE before diversification because VEHICLE is not necessarily GLOBAL bEST now// added 30Sept2015
//		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
//		{
//			for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
//			{
//				VEHICLE[i][j] = GLOBAL[i][j];
//			}
//			route_cost[i] = GLOBAL_Rcost[i];
//			saiz[i] = GLOBAL_SAIZ[i];
//			total_demand[i] = GLOBAL_capa[i];
//			space_available [i] = CAPACITY - total_demand[i];
//			distance_available[i] = DISTANCE - distance_cost[i];
//		}
//		no_routes = GLOBAL_NO_ROUTE;
//		cout<<"ENTERING  DIVERSIFICATION PHASE"<<endl;
//		for (int i = 0; i < no_routes; i++)
//		{
//			cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//			for (int j = 0; j <= saiz[i]+1; j++)
//			{
//				cout<<VEHICLE[i][j]<<' ';
//			}
//			cout<<endl;
//		}
//		cout<<"GLOBAL_BEST_COST= "<<GLOBAL_BEST_COST<<endl;
//		NbDiv++;
//		if (NbDiv > maxDiv) //if this is last diversification, dont perform LNS because it will not go through the LS //added on 12 Oct2015
//		{
//			goto lastdiversify;//check if this one ever executed or change to if (NbDiv == maxDiv) 
//		}
//		float cost=0;
//		//vehicle[][] to be passed in to make_giant_tour() is not important, make_giant_tour always consider LOCAL_BEST_SOL (incumbent best solution)
//		
//		if (found_GLOBEST_status == 1)
//		{
//			cost = make_giant_tour(VEHICLE); //pass in vehicle and no_routes to be updated in Dijkstra, they will be initialize in the Dijkstra function
//			Ndel_min = LB_Ndel_min;//back to initial value
//		}
//		else if (found_GLOBEST_status == 0) //if not found global_best
//		{
//			cout<<"Entering Diversify LNS/conflict sector in VNS kth improvement: ";
//			if(div == RS)
//				div=0;
//			//div=(rand() % 4); 
//			if (div == 0)
//			{
//				cout<<"LNS Diversification 2"<<endl;
//				cost = Diversify_LNS2(VEHICLE, Ndel_min);
//				cost = Diversify_LNS3(VEHICLE, Ndel_min);
//				div++;
//			}
//			
//			else if (div == 1)
//			{
//				cout<<"LNS Diversification overlap route"<<endl;
//				//Diversification_overlapRoute( float **(&vehicle), int (&no_routes), int K, float *x, float *y)
//				cost = Diversification_overlapRoute2(VEHICLE,  Ndel_min, x, y);
//				div++;
//			}
//			else if (div == 2)
//			{
//				cout<<"deleteHeadTail Diversification "<<endl;
//				cost = deleteHeadTail(VEHICLE, Ndel_min);
//				div++;
//			}
//			else if (div == 3)
//			{
//				cout<<"LNS Diversification LongestArc"<<endl;
//				cost = Diversify_LongestArc(VEHICLE, Ndel_min);
//				div++;
//			}
//			else if (div == 4)
//			{
//				cout<<"Conflict sector Diversification: "<<endl;
//				cost = Diversification_conflict_sector(VEHICLE, Ndel_min, x, y);
//				div++;
//			}
//			int incre = ceil(0.05*SIZE);
//			Ndel_min+=incre;
//			if (Ndel_min > UB_Ndel_min)
//				Ndel_min = UB_Ndel_min; //added this on 2March2016
//
//		}
//		//AFter diversification, record in LOCAL_BEST_ROUTE, when it enters restartAfterDiver, it will restart with add empty route to LOCAL_BEST_ROUTE again
//		//sort_solution (VEHICLE);
//		sort_solutionSMALLEST (VEHICLE);
//		int k=0;//for LOCAL[][] route index
//		for (int i = 0; i < no_routes; i++)
//		{
//			if (saiz[i] == 0)
//				continue;
//			for (int j = 0; j <= saiz[i]+1; j++)
//			{
//				LOCAL[k][j] = VEHICLE[i][j];
//			}
//			LOCAL_SAIZ[k] = saiz[i];
//			LOCAL_capa[k] = total_demand[i];
//			LOCAL_Rcost[k] = route_cost[i];
//			k++;
//		}
//		LOCAL_BEST_COST = cost;
//		LOCAL_NO_ROUTE = k;
//
//		cout<<"AFTER DIVERSIFICATION"<<endl;
//		for (int i = 0; i < LOCAL_NO_ROUTE; i++)
//		{
//			cout<<i<<' '<<LOCAL_Rcost[i]<<' '<<LOCAL_SAIZ[i]<<' '<<LOCAL_capa[i]<<' '<<' ';
//			for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
//			{
//				cout<<LOCAL[i][j]<<' ';
//			}
//			cout<<endl;
//		}
//		cout<<"LOCAL_BEST_COST ="<<LOCAL_BEST_COST<<endl;
//
//		//if the cost better than GLOBAL_BEST_COST 
//		if (GLOBAL_BEST_COST-cost > epsilonn)//cost < GLOBAL_BEST_COST
//		{
//			int m=0;//for GLOBAL[][] route index
//			for (int i = 0; i < no_routes; i++)
//			{
//				if (saiz[i] == 0)
//					continue;
//				best_sol << m<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
//				for (int j = 0; j <= saiz[i]+1; j++)
//				{
//					GLOBAL[m][j] = VEHICLE[i][j];
//					best_sol << GLOBAL[m][j] << ' ';
//				}
//				best_sol <<endl;
//				GLOBAL_SAIZ[m] = saiz[i];
//				GLOBAL_capa[m] = total_demand[i];
//				GLOBAL_Rcost[m] = route_cost[i];
//				m++;
//				
//			}
//			GLOBAL_BEST_COST = cost;
//			GLOBAL_NO_ROUTE = m;
//
//			best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
//			best_sol << "From VNS adaptive after Diversification type "<<div<<  endl;
//			cout << "Found GLOBAL_BEST in VNS multi-level right after Diversification (only basic LNS no other improvement) type "<<div<<  endl;
//			getchar();
//		}
//		best_sol << "Type of div = "<<div<<endl;
//		
//		reinitializeRchangedStatus();//initialize to 0 for every route	
//		lastdiversify:;
//		delete[] cost_of_removing;
//		delete[] Oricost_of_removing;
//
//	}//end while (NbDiv <= maxDiv) //#############################################################################################################################
//
//	timefile << "VNS adaptive "<<maxDiv <<" diversification time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << " second"<<endl;
//	recordTIME << "VNS  adaptive ("<<std::to_string( I )<<") with "<<maxDiv <<" diversification time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << " second"<<endl;
//	recordFreq.close();
//	best_sol.close();
//	localbest_sol.close();
//}