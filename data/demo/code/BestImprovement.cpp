#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

#include "Bestimprovement.h"
//#include "CheckFreq.h"
//#include "DiversificationDijkstra.h"
#include "VNS.h"
#define INFIN 99999999 
using namespace std;
const float BIGC = 20;
int tabuoperator;

void inter_route_improvement()
//void inter_route_improvement(float **distance, int *demand, float **(&best_routes), bool **(NR_FLAG))
{
	//================= copy from BEST_ROUTE, can copy from other if necessary later ====================//
	//================= route is to be passed to insert_one_cust(), delete_one_cust() ... ==================//
	int num_attributes = 13;//[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
	//++++++++++++++++++++++++++++++++++++++++++++++++++DATA STRUCTURE++++++++++++++++++++++++++++++++++++++++++++++++++++++//
	int start_s = clock();
	//ofstream timefile("16.Time_" + std::to_string( I ) + ".txt", ios::app);
	//ofstream gain_matrix("23.GAIN_MATRIX_" + std::to_string( I ) + ".txt");
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);

	
	int r=0;//for vehicle[][]
	for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	{
		if (GLOBAL_SAIZ[i] == 0)
			continue;

		for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
		{
			VEHICLE[r][j] = GLOBAL[i][j];
			CumDist[r][j] = GLOBALCumDist[i][j];
		}
		distance_cost[r] = GLOBAL_distance_cost[i];
		route_cost[r] = GLOBAL_Rcost[i];
		saiz[r] = GLOBAL_SAIZ[i];
		total_demand[r] = GLOBAL_capa[i];
		space_available[r] = CAPACITY - total_demand[r];
		distance_available[r] = DISTANCE - distance_cost[r];
		r++;
	}
	no_routes = r;

	r_change previousChange;
	r_change *sPtr = &previousChange; //sPtr point to struct

	float *cost_of_removing = new float[SIZE + 1];

	//========================================2-D matrices=============================================//
	float **gain_matrix_1_0 = new float*[no_routes];
	float **same_r_info = new float*[no_routes]; // to record the best improvement within the same route
	for (int i = 0; i < no_routes; ++i)
	{
		gain_matrix_1_0[i] = new float[no_routes];
		same_r_info[i] = new float[num_attributes];
	}

	int s = (((no_routes*no_routes) - no_routes) / 2) + 1;
	float** info_1_0 = new float*[s];
	for (int i = 0; i < s; ++i)
	{
		info_1_0[i] = new float[num_attributes];
	}

	int* route = new int[SIZE];


	//========================================Initialize Gain matrix ============================================//

	int i = ((no_routes*no_routes) - no_routes) / 2;
	for (int u = (no_routes - 1); u > 0; u--)
	{
		for (int v = u - 1; v >= 0; v--)
		{
			gain_matrix_1_0[u][v] = i;
			i--;
		}
	}

	float T_COST = 0;
	for (int g = 0; g < no_routes; g++)
	{
		T_COST = T_COST + route_cost[g];

	}
	

	//========================================Cost of removing ============================================//
	//cost of removing for single customer
	for (int f = 0; f < no_routes; f++) //find cost of removing for each customer and put in array
	{
		int before = -1, after = -1;
		for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		{
			before = VEHICLE[f][h - 1];
			after = VEHICLE[f][h + 1];	
			float origain = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];
			int remainingcust = saiz[f]-h;
			if (remainingcust < 0)//if negative
					remainingcust = 0;
			cost_of_removing[VEHICLE[f][h]] = CumDist[f][h] + remainingcust*origain;
		
		}
	}

	//for (int i = 0; i < no_routes; i++)
	//{
	//	for (int j = 1; j <= saiz[i]; j++)
	//	{
	//		CumuDCust[VEHICLE[i][j]]= CumDist[i][j];
	//	}
	//}

	best_gain_1_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	best_gain_1_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	best_gain_2_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	best_gain_2_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	//best_gain_2_2swap(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	//two_optintra(same_r_info, gain_matrix_1_0);
	two_optinter(info_1_0, gain_matrix_1_0);
	//crossTail2(info_1_0, gain_matrix_1_0);
	//CROSS(info_1_0, gain_matrix_1_0);

	float max_gain=INT_MIN;
	int set=0;
	do
	{
		set++;
		                      
		//============================================to find the best gain out of all modules=============================================//
		max_gain = INT_MIN; //############################################################################################################################
		bool same_route = false; //if it is the same route, then it is true(1)
		//int number;
		for (int i = 0; i < no_routes; i++)
		{
			//for (int m = i + 1; m < no_routes; m++)
			for (int m = i ; m < no_routes; m++)
			{
				if (gain_matrix_1_0[i][m]-max_gain > epsilon)//gain_matrix_1_0[i][m] > max_gain
				{
					if (i!=m)
					{
						max_gain = gain_matrix_1_0[i][m];
						number = gain_matrix_1_0[m][i];
						module = info_1_0[(int)gain_matrix_1_0[m][i]][8];
						route_change[0] = info_1_0[(int)gain_matrix_1_0[m][i]][2]; //route (from)
						route_change[1] = info_1_0[(int)gain_matrix_1_0[m][i]][3]; //route (to)
						sPtr->r1 = info_1_0[(int)gain_matrix_1_0[m][i]][2]; //route (from)
						sPtr->r2 = info_1_0[(int)gain_matrix_1_0[m][i]][3]; //route (to)
						same_route = false;
						
					}

					else
					{
						max_gain = gain_matrix_1_0[i][m];
						number = i;
						module = same_r_info[number][8];
						route_change[0] = same_r_info[number][2]; //route (from)
						route_change[1] = same_r_info[number][3]; //route (to)
						sPtr->r1 = same_r_info[number][2]; //route (from)
						sPtr->r2 = same_r_info[number][3]; //route (to)
						same_route = true; //it is from the same route, then is true
						
					}
				}
			}
		}

		//=================================insert to corresponding module based on the highest gain=====================================//
		if (max_gain > epsilon)
		{
			//see<<"============================================"<<endl;
			//see<<max_gain<<' '<<module<<' '<<route_change[0]<<' '<<route_change[1]<<' '<<same_route<<endl;
			if (module == 1)
			{
				
				insert_1_0(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 2)
			{
				
				insert_1_1(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 3)
			{
				
				insert_2_1(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 4)
			{
				
				insert_2_0(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 5)
			{
				insert_2_2swap(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 6)
			{
				insert_2optintra(max_gain, cost_of_removing, same_r_info, VEHICLE);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 7)
			{
				insert_2optinter(max_gain, cost_of_removing, info_1_0, VEHICLE);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 8)
			{
				insert_crosstail(max_gain, cost_of_removing, info_1_0, VEHICLE);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 9)
			{
				insert_cross(max_gain, cost_of_removing, info_1_0, VEHICLE);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
		

			//noneed reopt single route because in the reopt will consider thise single route
			reoptimize_1_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			reoptimize_1_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			reoptimize_2_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			reoptimize_2_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			//reoptimize_2_2swap(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			//reoptimize_two_optintra(same_r_info, gain_matrix_1_0);
			reoptimize_two_optinter(info_1_0, gain_matrix_1_0);
			//reoptimize_crossTail2(info_1_0, gain_matrix_1_0);
			//reoptimize_CROSS(info_1_0, gain_matrix_1_0);
		}//end if 

	} while (max_gain > epsilon);
	
	cout << ' ' << endl;

	//======================================================Display route========================================================================
	//cout << "==================Routes (in inter_route_improvement)===================================== " << endl;
	float total_cost = 0.0;
	int total_cust = 0;
	for (int g = 0; g < no_routes; g++)
	{
		total_cust = total_cust + saiz[g];
		//cout << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] +1; h++)
		//{
		//	cout << VEHICLE[g][h] << ' ';
		//}
		//cout <<  endl;
		total_cost = total_cost + route_cost[g];

	}
	//cout << "Total customers = " << total_cust << endl;
	//cout << "Total cost = " << total_cost << endl;

			
	//=================================record the best solution=====================================//
	if ((GLOBAL_BEST_COST-total_cost) > epsilon)//total_cost < GLOBAL_BEST_COST
	{		
		GLOBAL_BEST_COST = total_cost;
		cout << endl << "GLOBAL_BEST COST IS " << GLOBAL_BEST_COST << endl;

		int r=0;//for GLOBAL[r][]
		for (int i = 0; i < no_routes; i++)
		{
			if (saiz[i] == 0)
				continue;

			GLOBAL_SAIZ[r] = saiz[i]; //copy temp saiz[] to SAIZ[]
			GLOBAL_capa[r] = total_demand[i];
			GLOBAL_Rcost[r] = route_cost[i];
			GLOBAL_distance_cost[r] = distance_cost[i];

			for (int k = 0; k <= saiz[i]+1; k++)
			{
				GLOBAL[r][k] = VEHICLE[i][k]; //VEHICLE start with depot, end with depot
				GLOBALCumDist[r][k] = CumDist[i][k];
			}
			r++;
		}
		GLOBAL_NO_ROUTE = r;
		//GLOBAL_NO_ROUTE = LOCAL_NO_ROUTE = r;

	}

	cout << "========================================================================== " << endl;
	cout << "NO MORE GAIN" << endl;
	cout << "Iter = " << set + 1 << endl;//because iteration starts from 0

	int stop_s = clock();
	//timefile << "Inter-route improvement time: " << (stop_s - start_s) / float(CLOCKS_PER_SEC) << endl;
	//timefile.close();
	cout << "time: " << (stop_s - start_s) / float(CLOCKS_PER_SEC) << endl;

	ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
	best_sol <<"=============================================="<<endl;
	best_sol << "Initial BEST_SOLUTION from Best Improvement"<<endl;
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
	best_sol <<"=============================================="<<endl;
	best_sol.close();

	for (int i = 0; i < no_routes; i++)
	{
		delete[] gain_matrix_1_0[i];
		delete[] same_r_info[i];
	}
	delete[] gain_matrix_1_0;
	delete[] same_r_info;
	delete[] cost_of_removing;

	s = (((no_routes*no_routes) - no_routes) / 2) + 1;
	for (int i = 0; i < s; ++i)
	{
		delete[] info_1_0[i];
	}
	delete[] info_1_0;
	delete[] route;


}

void inter_route_improvementforLNS()
//void inter_route_improvement(float **distance, int *demand, float **(&best_routes), bool **(NR_FLAG))
{
	//================= copy from BEST_ROUTE, can copy from other if necessary later ====================//
	//================= route is to be passed to insert_one_cust(), delete_one_cust() ... ==================//
	int num_attributes = 13;//[0]index, [1]gain, [2]r1, [3]r2, [4]from_r1_p, [5]to_r2_p, [6]from_r2_p, [7]to_r1_p, [8]same route status, [9]reverse status, [10]gain1, [11]gain1 in distance, [12]gain2 in distance
	//++++++++++++++++++++++++++++++++++++++++++++++++++DATA STRUCTURE++++++++++++++++++++++++++++++++++++++++++++++++++++++//
	int start_s = clock();
	//ofstream timefile("16.Time_" + std::to_string( I ) + ".txt", ios::app);
	//ofstream gain_matrix("23.GAIN_MATRIX_" + std::to_string( I ) + ".txt");
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);

	//make comment if need this when running LNS
	//int r=0;//for vehicle[][]
	//for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	//{
	//	if (GLOBAL_SAIZ[i] == 0)
	//		continue;

	//	for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
	//	{
	//		VEHICLE[r][j] = GLOBAL[i][j];
	//		CumDist[r][j] = GLOBALCumDist[i][j];
	//	}
	//	distance_cost[r] = GLOBAL_distance_cost[i];
	//	route_cost[r] = GLOBAL_Rcost[i];
	//	saiz[r] = GLOBAL_SAIZ[i];
	//	total_demand[r] = GLOBAL_capa[i];
	//	space_available[r] = CAPACITY - total_demand[r];
	//	distance_available[r] = DISTANCE - distance_cost[r];
	//	r++;
	//}
	//no_routes = r;
	//end of make comment if need this when running LNS
	r_change previousChange;
	r_change *sPtr = &previousChange; //sPtr point to struct

	float *cost_of_removing = new float[SIZE + 1];

	//========================================2-D matrices=============================================//
	float **gain_matrix_1_0 = new float*[no_routes];
	float **same_r_info = new float*[no_routes]; // to record the best improvement within the same route
	for (int i = 0; i < no_routes; ++i)
	{
		gain_matrix_1_0[i] = new float[no_routes];
		same_r_info[i] = new float[num_attributes];
	}

	int s = (((no_routes*no_routes) - no_routes) / 2) + 1;
	float** info_1_0 = new float*[s];
	for (int i = 0; i < s; ++i)
	{
		info_1_0[i] = new float[num_attributes];
	}

	int* route = new int[SIZE];


	//========================================Initialize Gain matrix ============================================//

	int i = ((no_routes*no_routes) - no_routes) / 2;
	for (int u = (no_routes - 1); u > 0; u--)
	{
		for (int v = u - 1; v >= 0; v--)
		{
			gain_matrix_1_0[u][v] = i;
			i--;
		}
	}

	float T_COST = 0;
	for (int g = 0; g < no_routes; g++)
	{
		T_COST = T_COST + route_cost[g];

	}
	

	//========================================Cost of removing ============================================//
	//cost of removing for single customer
	for (int f = 0; f < no_routes; f++) //find cost of removing for each customer and put in array
	{
		int before = -1, after = -1;
		for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		{
			before = VEHICLE[f][h - 1];
			after = VEHICLE[f][h + 1];	
			float origain = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];
			int remainingcust = saiz[f]-h;
			if (remainingcust < 0)//if negative
					remainingcust = 0;
			cost_of_removing[VEHICLE[f][h]] = CumDist[f][h] + remainingcust*origain;
		
		}
	}



	best_gain_1_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	best_gain_1_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	best_gain_2_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	best_gain_2_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	//best_gain_2_2swap(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
	//two_optintra(same_r_info, gain_matrix_1_0);
	two_optinter(info_1_0, gain_matrix_1_0);
	//crossTail2(info_1_0, gain_matrix_1_0);
	//CROSS(info_1_0, gain_matrix_1_0);

	float max_gain=INT_MIN;
	int set=0;
	do
	{
		set++;
		                      
		//============================================to find the best gain out of all modules=============================================//
		max_gain = INT_MIN; //############################################################################################################################
		bool same_route = false; //if it is the same route, then it is true(1)
		//int number;
		for (int i = 0; i < no_routes; i++)
		{
			//for (int m = i + 1; m < no_routes; m++)
			for (int m = i ; m < no_routes; m++)
			{
				if (gain_matrix_1_0[i][m]-max_gain > epsilon)//gain_matrix_1_0[i][m] > max_gain
				{
					if (i!=m)
					{
						max_gain = gain_matrix_1_0[i][m];
						number = gain_matrix_1_0[m][i];
						module = info_1_0[(int)gain_matrix_1_0[m][i]][8];
						route_change[0] = info_1_0[(int)gain_matrix_1_0[m][i]][2]; //route (from)
						route_change[1] = info_1_0[(int)gain_matrix_1_0[m][i]][3]; //route (to)
						sPtr->r1 = info_1_0[(int)gain_matrix_1_0[m][i]][2]; //route (from)
						sPtr->r2 = info_1_0[(int)gain_matrix_1_0[m][i]][3]; //route (to)
						same_route = false;
						
					}

					else
					{
						max_gain = gain_matrix_1_0[i][m];
						number = i;
						module = same_r_info[number][8];
						route_change[0] = same_r_info[number][2]; //route (from)
						route_change[1] = same_r_info[number][3]; //route (to)
						sPtr->r1 = same_r_info[number][2]; //route (from)
						sPtr->r2 = same_r_info[number][3]; //route (to)
						same_route = true; //it is from the same route, then is true
						
					}
				}
			}
		}

		//=================================insert to corresponding module based on the highest gain=====================================//
		if (max_gain > epsilon)
		{
			//see<<"============================================"<<endl;
			//see<<max_gain<<' '<<module<<' '<<route_change[0]<<' '<<route_change[1]<<' '<<same_route<<endl;
			if (module == 1)
			{
				
				insert_1_0(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 2)
			{
				
				insert_1_1(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 3)
			{
				
				insert_2_1(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 4)
			{
				
				insert_2_0(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 5)
			{
				insert_2_2swap(max_gain, cost_of_removing, info_1_0, same_r_info, VEHICLE, same_route);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 6)
			{
				insert_2optintra(max_gain, cost_of_removing, same_r_info, VEHICLE);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 7)
			{
				insert_2optinter(max_gain, cost_of_removing, info_1_0, VEHICLE);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 8)
			{
				insert_crosstail(max_gain, cost_of_removing, info_1_0, VEHICLE);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
			else if (module == 9)
			{
				insert_cross(max_gain, cost_of_removing, info_1_0, VEHICLE);//VEHICLE will be updated //cost_of_removing is updated in insert()
			}
		}//end if 

			//noneed reopt single route because in the reopt will consider thise single route
			reoptimize_1_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			reoptimize_1_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			reoptimize_2_1(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			reoptimize_2_0(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			//reoptimize_2_2swap(cost_of_removing, info_1_0, same_r_info, gain_matrix_1_0);
			//reoptimize_two_optintra(same_r_info, gain_matrix_1_0);
			reoptimize_two_optinter(info_1_0, gain_matrix_1_0);
			//reoptimize_crossTail2(info_1_0, gain_matrix_1_0);
			//reoptimize_CROSS(info_1_0, gain_matrix_1_0);
		

	} while (max_gain > epsilon);
	
	cout << ' ' << endl;

	//======================================================Display route========================================================================
	//cout << "==================Routes (in inter_route_improvement)===================================== " << endl;
	float total_cost = 0.0;
	int total_cust = 0;
	for (int g = 0; g < no_routes; g++)
	{
		total_cust = total_cust + saiz[g];
		//cout << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] +1; h++)
		//{
		//	cout << VEHICLE[g][h] << ' ';
		//}
		//cout <<  endl;
		total_cost = total_cost + route_cost[g];

	}
	//cout << "Total customers = " << total_cust << endl;
	//cout << "Total cost = " << total_cost << endl;

			
	//=================================record the best solution=====================================//
	//if ((GLOBAL_BEST_COST-total_cost) > epsilon)//total_cost < GLOBAL_BEST_COST
	//{		
	//	GLOBAL_BEST_COST = total_cost;
	//	cout << endl << "GLOBAL_BEST COST IS " << GLOBAL_BEST_COST << endl;

	//	int r=0;//for GLOBAL[r][]
	//	for (int i = 0; i < no_routes; i++)
	//	{
	//		if (saiz[i] == 0)
	//			continue;

	//		GLOBAL_SAIZ[r] = saiz[i]; //copy temp saiz[] to SAIZ[]
	//		GLOBAL_capa[r] = total_demand[i];
	//		GLOBAL_Rcost[r] = route_cost[i];
	//		GLOBAL_distance_cost[r] = distance_cost[i];

	//		for (int k = 0; k <= saiz[i]+1; k++)
	//		{
	//			GLOBAL[r][k] = VEHICLE[i][k]; //VEHICLE start with depot, end with depot
	//			GLOBALCumDist[r][k] = CumDist[i][k];
	//		}
	//		r++;
	//	}
	//	GLOBAL_NO_ROUTE = r;
	//	//GLOBAL_NO_ROUTE = LOCAL_NO_ROUTE = r;

	//}

	//cout << "========================================================================== " << endl;
	//cout << "NO MORE GAIN" << endl;
	//cout << "Iter = " << set + 1 << endl;//because iteration starts from 0

	//int stop_s = clock();
	////timefile << "Inter-route improvement time: " << (stop_s - start_s) / float(CLOCKS_PER_SEC) << endl;
	////timefile.close();
	//cout << "time: " << (stop_s - start_s) / float(CLOCKS_PER_SEC) << endl;

	//ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt", ios::app);
	//best_sol <<"=============================================="<<endl;
	//best_sol << "Initial BEST_SOLUTION from Best Improvement"<<endl;
	////=================================display final solution=====================================//
	//
	//for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	//{
	//	best_sol << i <<' '<< GLOBAL_Rcost[i] <<' '<< GLOBAL_SAIZ[i] <<' '<< GLOBAL_capa[i]<<' '<<' ';
	//	cout << i <<' '<< GLOBAL_Rcost[i] <<' '<< GLOBAL_SAIZ[i] <<' '<< GLOBAL_capa[i]<<' '<<' ';
	//	for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
	//	{
	//		best_sol << GLOBAL[i][j] << ' ';
	//		cout << GLOBAL[i][j] << ' ';
	//	}
	//	best_sol <<  endl;
	//	cout << endl;	
	//}

	//best_sol << "GLOBAL_BEST COST= " << GLOBAL_BEST_COST << endl;
	//best_sol <<"=============================================="<<endl;
	//best_sol.close();

	for (int i = 0; i < no_routes; i++)
	{
		delete[] gain_matrix_1_0[i];
		delete[] same_r_info[i];
	}
	delete[] gain_matrix_1_0;
	delete[] same_r_info;
	delete[] cost_of_removing;

	s = (((no_routes*no_routes) - no_routes) / 2) + 1;
	for (int i = 0; i < s; ++i)
	{
		delete[] info_1_0[i];
	}
	delete[] info_1_0;
	delete[] route;


}

//DONE
void best_gain_1_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{

	int element=-1, before_ele, after_ele,  before_pos, after_pos;
	float gain;
	float cost_without_i, cost_with_i;


	/////////////================================== SAME ROUTE =========================////////////////////////////
	////////////============ insertion position is based on the modified route ================/////////////////////////
	// only 1-0 (same route) insertion position is based on modified route, 1-1 and 2-1 are based on non-modified route
	for (int i = 0; i < no_routes; i++) 
	{	
		//if ((saiz[i]==0) || (saiz[i]==1)) //if it is empty route or saiz=1
		if (saiz[i]<=1)//if saiz=2, front and back is depot, so D 1 2 D = D 2 1 D can be considered here for CCVRP
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
					same_r_info[i][10] = gain; //gain1
					same_r_info[i][11] = origain - oriloss; //gain1 in distance
				}	
			}//end of n
			delete[]tempCumDist;
			delete[] temp_r;
		}//end of j
	}


	////////////////////////====================== different routes =================================/////////////////////////
	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from)
	{
		if (saiz[i]==0) //if it is empty route
			continue;
		for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from)
		{
			element = VEHICLE[i][j];			//elemet to be inserted into other route
		
	

			for (int m = 0; m < no_routes; m++)  //insert (to) which route
			{
				if (demand[element] > space_available[m] || i == m ) //if demand exceed avaialable space in route and different route, if same route, still need to find the best even the capacity is violated
				{
					continue;
				}
				
				for (int n = 1; n < saiz[m] + 2; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
				{
					float available_dist_r2 = distance_available[m];
		
					before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[i][j + 1];
					before_pos = VEHICLE[m][n - 1];
					after_pos = VEHICLE[m][n];  //noneed to add 1 because it is insert in between n-1 and n
					
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

					if (i < m)
					{
						if (gain-gain_matrix_1_0[i][m] > epsilon)//gain > gain_matrix_1_0[i][m]
						{
							gain_matrix_1_0[i][m] = gain;
							info_1_0[(int)gain_matrix_1_0[m][i]][0] = gain_matrix_1_0[m][i];
							info_1_0[(int)gain_matrix_1_0[m][i]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[m][i]][2] = i;
							info_1_0[(int)gain_matrix_1_0[m][i]][3] = m;
							info_1_0[(int)gain_matrix_1_0[m][i]][4] = j;
							info_1_0[(int)gain_matrix_1_0[m][i]][5] = n;
							info_1_0[(int)gain_matrix_1_0[m][i]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[m][i]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[m][i]][8] = 1; //1 is (1-0)
							info_1_0[(int)gain_matrix_1_0[m][i]][10] = cost_without_i;//gain1
							info_1_0[(int)gain_matrix_1_0[m][i]][11] = origain;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[m][i]][12] = -oriloss;//gain2 in distance
						}
					}

					else if (m < i)
					{
						if (gain-gain_matrix_1_0[m][i] > epsilon)
						{
							gain_matrix_1_0[m][i] = gain;
							info_1_0[(int)gain_matrix_1_0[i][m]][0] = gain_matrix_1_0[i][m];
							info_1_0[(int)gain_matrix_1_0[i][m]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[i][m]][2] = i;
							info_1_0[(int)gain_matrix_1_0[i][m]][3] = m;
							info_1_0[(int)gain_matrix_1_0[i][m]][4] = j;
							info_1_0[(int)gain_matrix_1_0[i][m]][5] = n;
							info_1_0[(int)gain_matrix_1_0[i][m]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[i][m]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[i][m]][8] = 1; //1 is (1-0)
							info_1_0[(int)gain_matrix_1_0[i][m]][10] = cost_without_i;//gain1
							info_1_0[(int)gain_matrix_1_0[i][m]][11] = origain;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[i][m]][12] = -oriloss;//gain2 in distance

						}
					}
				}//end for n
			}//end for m
		}//end for j
	}//end for i 
	
}

//insert_1_0 (same route based on modified route, diff based on nothing
void insert_1_0(float max_gain, float *(&cost_of_removing), float **info_1_0, float **(same_r_info), int **(&VEHICLE), bool same_route) //from is fromroute 
{
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);
	float total_cost = 0.0;
	//float r1_cost = 0.0, r2_cost = 0.0;
	int old_pos = -1;
	int new_pos = -1;
	int ele = -1;
	int from = -1;
	int to = -1;
	float gain = 0.0;
	
	//////////////////////// POSITION OF INSERTION IS FOR MODIFIED ROUTE !!!!!!!!!!!!!!! ////////////////////////////
	if (same_route == true) //if it is from the same route
	{
		int route = (int)same_r_info[number][2];
		old_pos = same_r_info[number][4];
		new_pos = same_r_info[number][5];  //this position is based on modified route!!!!!!!!, assumed one has been deleted
		ele =VEHICLE[route][old_pos];
		from = route;
		to = route;
		gain = same_r_info[number][1];
		//see<<route_cost[route]<<endl;
		//see<<"cost_of_removing[ele]= "<<cost_of_removing[ele]<<endl;
		//see<<route<<' '<<old_pos<<' '<<new_pos<<' '<<ele<<' '<<from<<' '<<to<<' '<<gain<<endl;
		
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
		
		int i = saiz[route]-1+1; //1 has been deleted, so saiz-1. +1 so that copy the last element shift right
		///changed from new_pos+1 to new_pos on 11 Feb 2015
		while ( (i >= new_pos) && (i>=1) )
		{
			temp_r[i+1] = temp_r[i];
			i--;
		}
	
		temp_r[i+1] = ele;


		for (int k = 0; k < saiz[route]+2; k++)
		{
			VEHICLE[route][k] = temp_r[k];  //copy to VEHICLE matrix
		}
		distance_cost[route] = distance_cost[route] - same_r_info[number][11];
		//distance_cost[route] = 0.0; //initialize
		//for (int c = 0; c < saiz[route] + 1; c++)
		//{
		//	distance_cost[route] += dist[VEHICLE[route][c]][VEHICLE[route][c + 1]] + service_time[VEHICLE[route][c]];
		//	
		//}
		
		float check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[route] + 1; c++)
		{
			check_cost+= dist[VEHICLE[route][c]][VEHICLE[route][c + 1]] + service_time[VEHICLE[route][c]];
		}
		if (abs(check_cost-distance_cost[route]) > 1.5)
		{
			cout<<"Cost diff in insert 1-0"<<endl;
			getchar();
		}
		route_cost[route] = route_cost[route] - gain; //////////////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		distance_available[route] = DISTANCE - distance_cost[route];

		//update CumDist starting from the affecting pos until last cust
		int start = old_pos;//the start of update point, always hold old or new hold whichever comes first
		if (new_pos < old_pos)
			start = new_pos;
		for (int j = start; j <=saiz[route]; j++)
		{
			CumDist[route][j] = CumDist[route][j-1]+dist[VEHICLE[route][j-1]][VEHICLE[route][j]] + service_time[VEHICLE[route][j]];

			//CumuDCust[VEHICLE[route][j]]= CumDist[route][j];
	
		}

		delete[] temp_r;
		
	}
	else if (same_route == false) //if it is from different route
	{
		old_pos = info_1_0[number][4];//r1_w/o
		new_pos = info_1_0[number][5];//r2_w
		from = (int)info_1_0[number][2]; //from r1
		to = (int)info_1_0[number][3]; //to r2
		ele = VEHICLE[from][old_pos];
		gain = info_1_0[number][1];
		//see<<route_cost[from]<<' '<<route_cost[to]<<endl;
		//see<<"cost_of_removing[ele]= "<<cost_of_removing[ele]<<endl;
		//see<<old_pos<<' '<<new_pos<<' '<<ele<<' '<<from<<' '<<to<<' '<<gain<<endl;
		

		//r1 (delete 1)
		for (int i = old_pos; i<=saiz[from]; i++)
		{
			VEHICLE[from][i] = VEHICLE[from][i+1];
		}
		VEHICLE[from][saiz[from]+1] = -1;//optional
		saiz[from] = saiz[from] - 1;
		distance_cost[from] = distance_cost[from] - info_1_0[number][11];
		//distance_cost[from] = 0.0; //reinitialize
		//for (int c = 0; c < saiz[from] + 1; c++)
		//{
		//	distance_cost[from] += dist[VEHICLE[from][c]][VEHICLE[from][c + 1]] + service_time[VEHICLE[from][c]];
		//}
		float check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[from] + 1; c++)
		{
			check_cost+= dist[VEHICLE[from][c]][VEHICLE[from][c + 1]] + service_time[VEHICLE[from][c]];
		}
		if (abs(check_cost-distance_cost[from]) > 1.5)
		{
			cout<<"Cost diff in insert 1-0 from"<<endl;
			getchar();
		}
		route_cost[from] = route_cost[from] - info_1_0[number][10];////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
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
		route_cost[to] = route_cost[to] - (gain - info_1_0[number][10]); ////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		distance_cost[to] = distance_cost[to] - info_1_0[number][12];
		//distance_cost[to] = 0.0; //reinitialize
		//for (int c = 0; c < saiz[to] + 1; c++)
		//{
		//	distance_cost[to] += dist[VEHICLE[to][c]][VEHICLE[to][c + 1]] + service_time[VEHICLE[to][c]];
		//}
		check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[to] + 1; c++)
		{
			check_cost+= dist[VEHICLE[to][c]][VEHICLE[to][c + 1]] + service_time[VEHICLE[to][c]];
		}
		if (abs(check_cost-distance_cost[to]) > 1.5)
		{
			cout<<"Cost diff in insert 1-0 to"<<endl;
			getchar();
		}
		total_demand[to] = total_demand[to] + demand[ele];
		distance_available[to] = DISTANCE - distance_cost[to];
		space_available[to] = CAPACITY - total_demand[to];
		//update CumDist starting from the affecting pos until last cust
		int start2 = new_pos;//the start of update point
		for (int j = start2; j <=saiz[to]; j++)
		{
			CumDist[to][j] = CumDist[to][j-1]+dist[VEHICLE[to][j-1]][VEHICLE[to][j]] + service_time[VEHICLE[to][j]];
			//CumuDCust[VEHICLE[to][j]]= CumDist[to][j];
		}
	}
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	//route_file << ' ' << endl;
	////======================================================Display route========================================================================
	//route_file << "==================From insert (1-0)===================================== " <<"same_route = " << same_route << endl;
	//if (same_route == false)
	//	route_file << "Max_gain= " <<max_gain<<" r1= "<<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r1w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
	//else
	//	route_file << "Max_gain= " <<max_gain<< " r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;

		//see << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h < saiz[g] + 2; h++)
		//{
		//	see << VEHICLE[g][h] << ' ';
		//}
		//see<<endl;

		total_cost = total_cost + route_cost[g];
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}

	}
	//route_file << "Total cost = " << total_cost << endl;
	//route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	//route_file << "========================================================================== " << endl;
	//////=================================================update cost of removing=====================================//////////////////////
	
	//see<<endl;
	//see<<"CUmDIst after 1-0"<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{	
	//	for (int j = 0; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
	//	{
	//		see<<CumDist[i][j]<<' ';
	//	}
	//	see<<endl;
	//}

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

			//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
			//{
			//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]]; //old-new //(old + service_time – new)
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


		//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		//{
		//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]]; //(old + service_time – new)
		//}
		cout<<"Insert 1-0 same R"<<endl;
		//int status=comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<old_pos<<' '<<' '<<new_pos<<endl;
	}

		
	float *checkR_cost = new float[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		
		checkR_cost[i]=0;
		for (int j = 1; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
		{
			checkR_cost[i] += CumDist[i][j];
			
		}	
	}
	bool wrong = false;
	for (int i = 0; i < no_routes; i++)
	{
		//cout<< "checkR_cost["<< i <<"]= " << checkR_cost[i];
		if (abs(route_cost[i]-checkR_cost[i]) > BIGC)
		{
			wrong = true;
			cout<<i<<"  This cost is different from original "<<endl;
			cout<<route_cost[i]<<' '<<checkR_cost[i]<<' ';
		}
		//if (checkR_cost[i] > DISTANCE)
		//	cout<<" Exceed DISTANCE"<<endl;
		//cout<<endl;
	}
	delete[] checkR_cost;
	if (wrong ==true)
		getchar();
}

//DONE
void reoptimize_1_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{

	int element=-1, before_ele, after_ele, before_pos, after_pos;
	float gain;
	int r, from1, to1;
	float cost_without_i, cost_with_i;
	int r1 = route_change[0]; //route (from)
	int r2 = route_change[1]; //route (to)
	int num_attribute = 13;

	//reinitialize the gain for affected routes to be INT_MIN, only do this once in reoptimize 1_0, noneed to do in others because this will delete the values found in 1_0
	int iter=2; //added this on 1Nov15, previous change may only involve 1 route
	if (r1 == r2)
		iter=1;
	for (int s = 0; s < iter; s++)
	{
		if (s == 0)
			r = r1;
		else
			r = r2;
		for (int t = 0; t < no_routes; t++)//////////////////////////////////////////////////////////////////////////////////////////////////////
		{
			if (r<t)
			{
				gain_matrix_1_0[r][t] = INT_MIN;
				for (int u = 1; u < num_attribute; u++)
				{
					info_1_0[(int)gain_matrix_1_0[t][r]][u] = -1;

				}
			}
			else if (t<r)
			{
				gain_matrix_1_0[t][r] = INT_MIN;
				for (int u = 1; u < num_attribute; u++)
				{
					info_1_0[(int)gain_matrix_1_0[r][t]][u] = -1;
				}
			}
		}
		gain_matrix_1_0[r][r] = INT_MIN; //initialize diagonal for intra-route gain	
		for (int u = 1; u < num_attribute; u++)
		{
			same_r_info[r][u] = -1;
		}

	}

	int i = -1; //initilize for affected route
	//////////////////================================== SAME ROUTE =========================////////////////////////////////////
	for (int r = 0; r < iter; r++) 
	{
		if(r==0)
			i=r1;
		else
			i=r2;
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
				


				if ((before_pos == SIZE) || (after_pos == SIZE) ) //if the insertion is between customer and depot
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
				
				
				//if (abs(gain) > available_dist_r2)//if negative gain, it will not be recorded anyway, so this code can be omitted actually
				//{
				//	continue;
				//}
				
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
					same_r_info[i][10] = gain; //gain1
					same_r_info[i][11] = origain - oriloss; //gain1 in cost

				}	
			}
			delete[]tempCumDist;
			delete[] temp_r;
		}//end of j

	}

	///////////////// ============================== DIFERENT ROUTES ======================================//////////////////////
	//######################################update gain matrix################################################//
	//from 2 routes to every position=======================update row============================= ///////////////////////////////////
	for (int i = 0; i < iter; i++)  //consider each and other routes (from)
	{
		if (i == 0)
			from1 = r1;

		else
			from1 = r2;
	

		if (saiz[from1]==0)//if it is empty route
			continue;
		for (int j = 1; j < saiz[from1] + 1; j++)  // consider each customer (from)
		{
			element = VEHICLE[from1][j];			//elemet to be inserted into other route
			
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
					
					if (oriloss - available_dist_r2 > epsilon) //only consider r2
					{
						continue;
					}
					
					float newCumDist = CumDist[m][n-1] + dist[before_pos][element] + service_time[element]; 
					int remainingCustafterEle = saiz[m] - n + 1;//different from sameroute
					if (remainingCustafterEle < 0)//if negative
							remainingCustafterEle = 0;
					cost_with_i = newCumDist + remainingCustafterEle*oriloss; 

					//cost_with_i = dist[before_pos][element] + dist[element][after_pos] + service_time[element] - dist[before_pos][after_pos]; //cost of r2 with i //(new+service_time – old)
				
					
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
							info_1_0[(int)gain_matrix_1_0[m][from1]][11] = origain; //gain1 in cost
							info_1_0[(int)gain_matrix_1_0[m][from1]][12] = -oriloss; //gain2 in cost
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
							info_1_0[(int)gain_matrix_1_0[from1][m]][11] = origain; //gain1 in cost
							info_1_0[(int)gain_matrix_1_0[from1][m]][12] = -oriloss; //gain2 in cost
						}
					}
				}//end for n
			}//end for m
		}//end for j
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
	

			for (int m = 0; m < iter; m++)  //insert (to) which route
			{
				if (m == 0)
					to1 = r1;

				else
					to1 = r2;

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
void best_gain_1_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{
	
	int element1, element2, before_pos1, before_pos2, after_pos1, after_pos2;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_j, gain;
	int before_ele1, before_ele2, after_ele1, after_ele2;

	std::vector<int> vector_r1;
	std::vector<int> vector_r2;
	std::vector<int> vector_r3;
	
	int temp = -1;

	//================================  if in the same route, just swap customer  ====================================//
	for(int r=0; r < no_routes; r++)
	{
		//if ((saiz[r]==0)||(saiz[r]==1)) //if it is empty route or saiz=1, nothing to reshuffle
		if (saiz[r]<=2)
			continue;
		//copy route in vector
		vector_r3.clear();
		for (int i = 0; i < saiz[r] + 2; i++) //first and last is depot
		{
			vector_r3.push_back(VEHICLE[r][i]);
		}
		float available_dist_r2 = distance_available[r];

		for (int j = 1; j < saiz[r] ; j++) //from which position
		{
			element1 = vector_r3[j];
			before_ele1 = vector_r3[j-1];
			after_ele1 = vector_r3[j+1];
			
			for (int k = j+1; k <= saiz[r]; k++)
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

	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	for (int i = 0; i < no_routes-1; i++)  //consider each and other routes (from R1)	//changed to no_routes-1 on 6April2015
	{
		if (saiz[i]==0) //if it is empty route
			continue;
		for (int j = 1; j < saiz[i] + 1; j++)  // consider each customer (from R1), start from 1 because [0] is depot
		{
			element1 = VEHICLE[i][j];			//elemet to be inserted into other route
			before_ele1 = VEHICLE[i][j-1];		
			after_ele1	 = VEHICLE[i][j+1];
			
			
			float origain1 = dist[element1][before_ele1] + dist[element1][after_ele1] + service_time[element1] - dist[before_ele1][after_ele1];//old-new positive is good //for tempCumDist

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

			for (int m = i+1; m < no_routes; m++)  //insert (to) which route (to R2)//changed to m=i+1 on 6April2015
			{
				if ((i == m) || (saiz[m]==0))// || (demand[element2] > space_available[m])) //if customer already in the route or demand exceed avaialable space in route
				{
					continue;

				}
				for (int n = 1; n < saiz[m] + 1; n++) //which customer in r2 to remove
				{
					before_ele2 = VEHICLE[m][n-1];
					element2 = VEHICLE[m][n];			//element to be removed from r2
					after_ele2	 = VEHICLE[m][n+1];		
					int space_r1 = space_available[i];
					int space_r2 = space_available[m];
					space_r1 = space_r1 + demand[element1];
					space_r2 = space_r2 + demand[element2];

					if ((demand[element1] > space_r2) || (demand[element2] > space_r1))
						continue;//go to next element to delete
												
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
									info_1_0[(int)gain_matrix_1_0[m][i]][5] = p;
									info_1_0[(int)gain_matrix_1_0[m][i]][6] = n;
									info_1_0[(int)gain_matrix_1_0[m][i]][7] = q;
									info_1_0[(int)gain_matrix_1_0[m][i]][8] = 2;//2 is (1-1)
									//to check
									//if((j==q) && (p==n))
									//{
									//	float cost_without_i2 = dist[before_pos1][element1] + dist[element1][after_pos1] + service_time[element1] - dist[before_pos1][after_pos1];//to check 
									//	float cost_without_j2 = dist[before_pos2][element2] + dist[element2][after_pos2] + service_time[element2] - dist[before_pos2][after_pos2];//to check 
									//	if ((abs(cost_without_i2-cost_of_removing[VEHICLE[i][j]])>0.1) || (abs(cost_without_j2-cost_of_removing[VEHICLE[m][n]])>0.1))
									//	{
									//		cout<<"best_gain_1_1 got problem!!"<<endl;
									//		getchar();
									//	}
									//
									//}
									info_1_0[(int)gain_matrix_1_0[m][i]][10] = cost_without_i  - cost_with_i;//gain1
									info_1_0[(int)gain_matrix_1_0[m][i]][11] = origain1 - oriloss1; //gain1 in distance
									info_1_0[(int)gain_matrix_1_0[m][i]][12] = origain2 - oriloss2; //gain2 in distance

								}//end if
							}

							else if (i > m)//m will always greater than i because m=i+1 //this one can be deleted actually
							{
								if (gain-gain_matrix_1_0[m][i] > epsilon)//gain > gain_matrix_1_0[m][i]
								{
									gain_matrix_1_0[m][i] = gain;////////////////////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! (CHANGED ON 1 OCT 2014!!!!!!!!!!!!!!!!!!)

									//info_1_0[number_matrix_1_0[i][m]][0] = gain_matrix_1_0[i][m];
									info_1_0[(int)gain_matrix_1_0[i][m]][1] = gain;
									info_1_0[(int)gain_matrix_1_0[i][m]][2] = i;
									info_1_0[(int)gain_matrix_1_0[i][m]][3] = m;
									info_1_0[(int)gain_matrix_1_0[i][m]][4] = j;
									info_1_0[(int)gain_matrix_1_0[i][m]][5] = p;
									info_1_0[(int)gain_matrix_1_0[i][m]][6] = n;
									info_1_0[(int)gain_matrix_1_0[i][m]][7] = q;
									info_1_0[(int)gain_matrix_1_0[i][m]][8] = 2;//2 is (1-1)
									//to check
									//if((j==q) && (p==n))
									//{
									//	float cost_without_i2 = dist[before_pos1][element1] + dist[element1][after_pos1] + service_time[element1] - dist[before_pos1][after_pos1];//to check 
									//	float cost_without_j2 = dist[before_pos2][element2] + dist[element2][after_pos2] + service_time[element2] - dist[before_pos2][after_pos2];//to check 
									//	if ((abs(cost_without_i2-cost_of_removing[VEHICLE[i][j]])>0.1) || (abs(cost_without_j2-cost_of_removing[VEHICLE[m][n]])>0.1))
									//	{
									//		cout<<"best_gain_1_1 got problem!!"<<endl;
									//		getchar();
									//	}
									//
									//}
									info_1_0[(int)gain_matrix_1_0[i][m]][10] = cost_without_i  - cost_with_i;//gain1
									info_1_0[(int)gain_matrix_1_0[i][m]][11] = origain1 - oriloss1; //gain1 in distance
									info_1_0[(int)gain_matrix_1_0[i][m]][12] = origain2 - oriloss2; //gain2 in distance
								}//end if
							}//end else if							
					
						}//end for q
					}//end for p
					delete[]tempCumDist2;
				}//end for n
			}//end for m
			delete[]tempCumDist1;
		}//end for j
	}//end for i 	
}

//DONE
//insert_1_1 (same route based on non-modified route, diff based on modified route
void insert_1_1(float max_gain, float *(&cost_of_removing),  float **info_1_0, float **(same_r_info), int **(&VEHICLE), bool same_route)
{
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);
	float total_cost = 0.0;
	int r1 = -1;
	int r2 = -1;
	
	float gain = 0.0;
	
	//========================  ALL CUSTOMER POSITION IN NON_MODIFIED ROUTE ===================//
	if(same_route == true) //if it is the same route, the position is to swap
	{
		int route = (int)same_r_info[number][2];
		int old_pos = same_r_info[number][4];
		int new_pos = same_r_info[number][5];
		int ele1 = VEHICLE[route][old_pos];
		int ele2 = VEHICLE[route][new_pos];
		r1 = route;
		r2 = route;
		gain = same_r_info[number][1];
		//see<<route_cost[route]<<endl;
		//see<<"cost_of_removing[ele1]= "<<cost_of_removing[ele1]<<"  cost_of_removing[ele2]= "<<cost_of_removing[ele2]<<endl;
		//see<<route<<' '<<old_pos<<' '<<new_pos<<' '<<ele1<<' '<<ele2<<' '<<r1<<' '<<r2<<' '<<gain<<endl;
		

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
		distance_cost[route] = distance_cost[route] - same_r_info[number][11];
		//distance_cost[route] = 0.0;//reinitialize
		//for (int c = 0; c < saiz[route] + 1; c++)
		//{
		//	distance_cost[route] += dist[VEHICLE[route][c]][VEHICLE[route][c + 1]] + service_time[VEHICLE[route][c]];
		//}
		float check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[route] + 1; c++)
		{
			check_cost+= dist[VEHICLE[route][c]][VEHICLE[route][c + 1]] + service_time[VEHICLE[route][c]];
		}
		if (abs(check_cost-distance_cost[route]) > 1.5)
		{
			cout<<"Cost diff in insert 1-1"<<endl;
			getchar();
		}
		route_cost[route] = route_cost[route] - gain; //////////////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		distance_available[route] = DISTANCE - distance_cost[route];
		
		//update CumDist starting from the affecting pos until last cust
		int start = old_pos;//the start of update point, always hold old or new hold whichever comes first
		if (new_pos < old_pos)
			start = new_pos;
		for (int j = start; j <=saiz[route]; j++)
		{
			CumDist[route][j] = CumDist[route][j-1]+dist[VEHICLE[route][j-1]][VEHICLE[route][j]] + service_time[VEHICLE[route][j]];
			//CumuDCust[VEHICLE[route][j]]= CumDist[route][j];
		}

	}

	else if (same_route == false) //if from different route
	{
		//##############################################################################################################################################
		//======================================================Route 1 ===================================================================================
		int r1_without = info_1_0[number][4]; //r1_w/o
		int r2_with = info_1_0[number][5]; //r2_w
		int r2_without = info_1_0[number][6];//r2_w/o
		int r1_with = info_1_0[number][7];//r1_w
		r1 = info_1_0[number][2];//from r1
		r2 = info_1_0[number][3];//to r2
		gain = info_1_0[number][1];
		//see<<route_cost[r1]<<' '<<route_cost[r2]<<endl;
		//see<<"cost_of_removing[ele1]= "<<cost_of_removing[VEHICLE[r1][r1_without]]<<"  cost_of_removing[ele2]= "<<cost_of_removing[VEHICLE[r2][r2_without]]<<endl;
		//see<<r1_without<<' '<<r2_with<<' '<<r2_without<<' '<<r1_with<<' '<<r1<<' '<<r2<<' '<<gain<<endl;
		//================to indicate which customers have been affected to update cost_of_removing ==============route is before inertion
		int delete1=1;//, if delete 0 customer, flag 2 customer , if delete 1 customer flag 3 customer, if delete 2 flag 4, if delete 3 flag 5
		
		
		std::vector<int> myvector1;
		std::vector<int> myvector2;
		int ele2 = -1, ele1 = -1;
		for (int i = 0; i<saiz[r1] + 2; i++)
		{
			if (i==r1_without)
			{
				ele2 = VEHICLE[r1][i]; //ele2 original from r1
				continue;
			}
			myvector1.push_back(VEHICLE[r1][i]);
		}

		for (int j = 0; j<saiz[r2] + 2; j++)
		{
			if (j==r2_without)
			{
				ele1 = VEHICLE[r2][j]; //ele1 original from r2
				continue;
			}
			myvector2.push_back(VEHICLE[r2][j]);
		}

		

		myvector1.insert(myvector1.begin() + r1_with, ele1);
		myvector2.insert(myvector2.begin() + r2_with, ele2);

		for (int k = 0; k < myvector1.size(); k++)
		{
			VEHICLE[r1][k] = myvector1[k];  //copy to VEHICLE matrix
		}
		float new_sum_cost = route_cost[r1] + route_cost[r2] - gain;
		distance_cost[r1] = distance_cost[r1] - info_1_0[number][11];
		//distance_cost[r1] = 0.0; //reinitialize
		//for (int c = 0; c < saiz[r1] + 1; c++)
		//{
		//	distance_cost[r1] += dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]] + service_time[VEHICLE[r1][c]];
		//}
		float check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[r1] + 1; c++)
		{
			check_cost+= dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]] + service_time[VEHICLE[r1][c]];
		}
		if (abs(check_cost-distance_cost[r1]) > 1.5)
		{
			cout<<"Cost diff in insert 1-1 r1"<<endl;
			getchar();
		}
		route_cost[r1] = route_cost[r1] - info_1_0[number][10];////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		total_demand[r1] = total_demand[r1] - demand[ele2] + demand[ele1];
		distance_available[r1] = DISTANCE - distance_cost[r1];
		space_available[r1] = CAPACITY - total_demand[r1];

		for (int p = 0; p < myvector2.size(); p++)
		{
			VEHICLE[r2][p] = myvector2[p];
		}
		distance_cost[r2] = distance_cost[r2] - info_1_0[number][12];
		//distance_cost[r2] = 0.0; //reinitialize
		//for (int c = 0; c < saiz[r2] + 1; c++)
		//{
		//	distance_cost[r2] += dist[VEHICLE[r2][c]][VEHICLE[r2][c + 1]] + service_time[VEHICLE[r2][c]];
		//}
		check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[r2] + 1; c++)
		{
			check_cost+= dist[VEHICLE[r2][c]][VEHICLE[r2][c + 1]] + service_time[VEHICLE[r2][c]];
		}
		if (abs(check_cost-distance_cost[r2]) > 1.5)
		{
			cout<<"Cost diff in insert 1-1 r2"<<endl;
			getchar();
		}
		route_cost[r2] = new_sum_cost - route_cost[r1] ;

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
			//CumuDCust[VEHICLE[r1][j]]= CumDist[r1][j];
		}
		//update CumDist starting from the affecting pos until last cust
		int start2 = r2_without;//the start of update point, always hold old or new hold whichever comes first
		if (r2_with < r2_without)
			start2 = r2_with;
		for (int j = start2; j <=saiz[r2]; j++)
		{
			CumDist[r2][j] = CumDist[r2][j-1]+dist[VEHICLE[r2][j-1]][VEHICLE[r2][j]] + service_time[VEHICLE[r2][j]];
			//CumuDCust[VEHICLE[r2][j]]= CumDist[r2][j];
		}
	}//end of else not the same route

	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	//route_file << ' ' << endl;
	////======================================================Display route========================================================================//
	//route_file << "==================From insert (1-1)===================================== " <<"same_route = " << same_route<< endl;
	//if (same_route == false)
	//	route_file << "Max_gain= " <<max_gain<< " r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r1w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
	//else
	//	route_file << "Max_gain= " <<max_gain<< " r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		//see << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] + 1; h++)
		//{
		//	see << VEHICLE[g][h] << ' ';	
		//}
		//see<<endl;

		total_cost = total_cost + route_cost[g];
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
	}

	//see<<endl;
	//	see<<"CUmDIst after 1-1"<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{	
	//	for (int j = 0; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
	//	{
	//		see<<CumDist[i][j]<<' ';
	//	}
	//	see<<endl;
	//}
	//route_file << "Total cost = " << total_cost << endl;
	//route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	//route_file << "========================================================================== " << endl;
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
		
			
			//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
			//{
			//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];//(old + service_time – new)
			//}
			cout<<"Insert 1-1 diff R"<<endl;
			//int status	= comprareCost (cost_of_removing, f);//to check 
			//if (status == 1)
			//	cout<<info_1_0[number][4]<<' '<<' '<<info_1_0[number][5]<<' '<<' '<<info_1_0[number][6]<<' '<<' '<<info_1_0[number][7]<<endl;
		}
	}
	
	else // if the same route
	{
		int f = r1;

		
		updateCostofRemoving2 (f, cost_of_removing);
		
		
		//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		//{
		//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];//(old + service_time – new)
		//}
		cout<<"Insert 1-1 same R"<<endl;
		//int status	= comprareCost (cost_of_removing, f); //to check 
		//if (status == 1)
		//	cout<<same_r_info[number][4]<<' '<<' '<<same_r_info[number][5]<<endl;
	}
	
	
	float *checkR_cost = new float[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		
		checkR_cost[i]=0;
		for (int j = 1; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
		{
			checkR_cost[i] += CumDist[i][j];
		}	
	}
	bool wrong = false;
	for (int i = 0; i < no_routes; i++)
	{
		//cout<< "checkR_cost["<< i <<"]= " << checkR_cost[i];
		if (abs(route_cost[i]-checkR_cost[i]) > BIGC)
		{
			cout<<i<<"  This cost is different from original "<<endl;
			cout<<route_cost[i]<<' '<<checkR_cost[i]<<' ';
			wrong =true;
		}
		//if (checkR_cost[i] > DISTANCE)
		//	cout<<" Exceed DISTANCE"<<endl;
		//cout<<endl;
	}
	delete[] checkR_cost;
	if (wrong ==true)
		getchar();
}


//DONE
void reoptimize_1_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{

	int element1, element2, before_pos1, before_pos2, after_pos1, after_pos2;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_j, gain;
	int before_ele1, before_ele2, after_ele1, after_ele2;
	std::vector<int> vector_r1;
	std::vector<int> vector_r2;
	std::vector<int> vector_r3;
	int r;
	int r1 = route_change[0]; //route (from)
	int r2 = route_change[1]; //route (to)

	//////////////////////////////////============================= SAME ROUTE ===============================================//////////////////////
	int iter=2; //added this on 1Nov15, previous change may only involve 1 route
	if (r1 == r2)
		iter=1;
	for (int I = 0; I < iter; I++)
	{
		if (I==0)
			r=r1;
		else
			r=r2;
		float available_dist_r2 = distance_available[r];
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
				//same route, noneed to consider service_time[]
				//if (k==(j+1)) // if consecutive customer, delete 3 arcs and insert 3 arcs
				//{
				//	cost_without_i = dist[before_ele1][element1] + dist[element1][after_ele1] + dist[element2][after_ele2]; //route_cost is global variable //minus 4 arcs 
				//	cost_with_i = dist[before_ele1][element2] + dist[element2][element1] +dist[element1][after_ele2];
				//}
				//else  // if non-consecutive customer, delete 4 arcs and insert 4 arcs
				//{
				//	cost_without_i = dist[before_ele1][element1] + dist[element1][after_ele1] + dist[before_ele2][element2] + dist[element2][after_ele2]; //route_cost is global variable //minus 4 arcs 
				//	cost_with_i = dist[before_ele1][element2] + dist[element2][after_ele1] +dist[before_ele2][element1] +dist[element1][after_ele2];
				//}
				//gain = cost_without_i - cost_with_i; //gain = old-new
				////if (abs(gain) > distance_available[r])//gain is negative, change to positive using abs //if negative gain, it will be omitted actually
				////{
				////	continue;
				////}

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
	
	int k = 0;
	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//================================================(from) 2 affected routes to other routes (update ROW)================================================================
	for (int I = 0; I < iter; I++)  //consider each and other routes (from R1)
	{
		if (I == 0)
			k = r1;

		else
			k = r2;

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
						if (a==j)
							continue;
						vector_r1.push_back(VEHICLE[k][a]);
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
											
					float available_dist_r1 = distance_available[k];
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
									info_1_0[(int)gain_matrix_1_0[m][k]][4] = j;
									info_1_0[(int)gain_matrix_1_0[m][k]][5] = p;
									info_1_0[(int)gain_matrix_1_0[m][k]][6] = n;
									info_1_0[(int)gain_matrix_1_0[m][k]][7] = q;
									info_1_0[(int)gain_matrix_1_0[m][k]][8] = 2; //2 is (1-1)
									info_1_0[(int)gain_matrix_1_0[m][k]][10] = cost_without_i- cost_with_i;//gain1
									info_1_0[(int)gain_matrix_1_0[m][k]][11] = origain1 - oriloss1; //gain1 in distance
									info_1_0[(int)gain_matrix_1_0[m][k]][12] = origain2 - oriloss2; //gain2 in distance


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
									info_1_0[(int)gain_matrix_1_0[k][m]][4] = j;
									info_1_0[(int)gain_matrix_1_0[k][m]][5] = p;
									info_1_0[(int)gain_matrix_1_0[k][m]][6] = n;
									info_1_0[(int)gain_matrix_1_0[k][m]][7] = q;
									info_1_0[(int)gain_matrix_1_0[k][m]][8] = 2;//2 is (1-1)
									info_1_0[(int)gain_matrix_1_0[k][m]][10] = cost_without_i- cost_with_i;//gain1
									info_1_0[(int)gain_matrix_1_0[k][m]][11] = origain1 - oriloss1; //gain1 in distance
									info_1_0[(int)gain_matrix_1_0[k][m]][12] = origain2 - oriloss2; //gain2 in distance
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
}




//DONE
//No same route, different route only
void best_gain_2_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{

	int element1, element1f, element2, before_ele1, before_ele2, after_ele1=-1, after_ele2=-1,  before_pos1, before_pos2, after_pos1, after_pos2;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_j, cost_with_jRev, gain;
	int element3, before_ele3, after_ele3, reverseStatus;
	std::vector<int> vector_r1;
	std::vector<int> vector_r2;
	std::vector<int> vector_r3;

	//================================  if in the same route, just swap a pair with a single customer  ====================================//
	//==========================   all customer position in non-modified route (very important for insertion)!!!!!!============================= ////////////////////////
	//for(int r=0; r < no_routes; r++)
	//{
	//	//if ((saiz[r]==0)||(saiz[r]==1)||(saiz[r]==2)) //if it is empty route, saiz=1, or saiz=2, oting to move around
	//	if (saiz[r]<=2)	
	//		continue;
	//	//copy route in vector
	//	vector_r3.clear();
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
	//			reverseStatus = 0; //initialize reverseStatus to 0
	//			element3 = vector_r3[k];
	//			before_ele3 = vector_r3[k-1];
	//			after_ele3 = vector_r3[k+1];
	//			
	//			if ( (before_ele3 == SIZE || after_ele3 == SIZE) && (before_ele1 == SIZE || after_ele2 == SIZE) ) //if mn and ij depot
	//			{	
	//				if ( (NR_FLAG_DEPOT[element1][before_ele3] == false && NR_FLAG_DEPOT[element2][after_ele3] == false) || (NR_FLAG_DEPOT[element3][before_ele1] == false && NR_FLAG_DEPOT[element3][after_ele2] == false) )
	//					continue;
	//			}
	//			else if ( before_ele3 == SIZE || after_ele3 == SIZE ) //if mn depot
	//			{
	//				if ( (NR_FLAG_DEPOT[element1][before_ele3] == false && NR_FLAG_DEPOT[element2][after_ele3] == false) || (NR_FLAG[element3][before_ele1] == false && NR_FLAG[element3][after_ele2] == false) )
	//					continue;
	//			}
	//			else if ( before_ele1 == SIZE || after_ele2 == SIZE ) //if ij depot
	//			{
	//				if ( (NR_FLAG[element1][before_ele3] == false && NR_FLAG[element2][after_ele3] == false) || (NR_FLAG_DEPOT[element3][before_ele1] == false && NR_FLAG_DEPOT[element3][after_ele2] == false) )
	//					continue;
	//			}
	//			else //if no depot
	//			{
	//				if ( (NR_FLAG[element1][before_ele3] == false && NR_FLAG[element2][after_ele3] == false) || (NR_FLAG[element3][before_ele1] == false && NR_FLAG[element3][after_ele2] == false) )
	//					continue;
	//			}
	//			//same route, noneed to consider service_time[]
	//			if (k > j) //if single customer is after pair customer
	//			{
	//				if (after_ele2 == element3) //in consecutive, so minus 3 arcs, for eg: 4 2 3 7 5 1 (swap pair 2 3 with 7) 
	//				{
	//					cost_without_i = dist[before_ele1][element1] + dist[element2][after_ele2] + dist[element3][after_ele3]; //route_cost is global variable //minus 4 arcs 
	//					cost_with_i = dist[before_ele1][element3]  + dist[element3][element1] +dist[element2][after_ele3];
	//					cost_with_iRev = dist[before_ele1][element3]  + dist[element3][element2] +dist[element1][after_ele3];
	//				}
	//			
	//				else 
	//				{
	//					cost_without_i = dist[before_ele1][element1] + dist[element2][after_ele2] + dist[before_ele3][element3] + dist[element3][after_ele3]; //route_cost is global variable //minus 4 arcs 
	//					cost_with_i = dist[before_ele1][element3] + dist[element3][after_ele2] +dist[before_ele3][element1] +dist[element2][after_ele3];
	//					cost_with_iRev = dist[before_ele1][element3] + dist[element3][after_ele2] +dist[before_ele3][element2] +dist[element1][after_ele3];
	//				}
	//			}
	//			else //if single customer is before pair customer
	//			{
	//				if(element3 == before_ele1) //in consecutive, so minus 3 arcs, for eg: 4 2 3 7 5 1 (swap 2 with pair 3 7) 
	//				{
	//					cost_without_i = dist[before_ele3][element3] + dist[before_ele1][element1] + dist[element2][after_ele2];
	//					cost_with_i = dist[before_ele3][element1] + dist[element2][element3] + dist[element3][after_ele2];
	//					cost_with_iRev = dist[before_ele3][element2] + dist[element1][element3] + dist[element3][after_ele2];
	//				}
	//				else //all other sequence, so minus 4 arcs， for eg: 4 2 3 7 5 1 (swap 4 with pair 7 5) 
	//				{
	//					cost_without_i =  dist[before_ele3][element3] + dist[element3][after_ele3] +dist[before_ele1][element1] +dist[element2][after_ele2];
	//					cost_with_i = dist[before_ele3][element1] + dist[element2][after_ele3] +dist[before_ele1][element3] +dist[element3][after_ele2];
	//					cost_with_iRev = dist[before_ele3][element2] + dist[element1][after_ele3] +dist[before_ele1][element3] +dist[element3][after_ele2];
	//				}
	//			}
	//			if (cost_with_iRev - cost_with_i < -epsilon)
	//			{
	//				cost_with_i = cost_with_iRev;
	//				reverseStatus = 1;
	//			}
	//			gain = cost_without_i - cost_with_i; //gain = old-new
	//			
	//			//if (abs(gain) > distance_available[r])//gain is negative, change to positive using abs //if negative gain, it will  be omitted actually
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
	//	
	//}

	//======================================================== DIfferent routes ==============================================================//
	//##########################################calculate GAIN when remove 2 and insert 1 simultaneously######################################################//
	
	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from R1)
	{
		//if ((saiz[i]==0) || (saiz[i]==1)) //if it is empty route or saiz=1
		if (saiz[i]<=1)
			continue;
		for (int j = 1; j < saiz[i]; j++)  // consider each customer (from R1), start from 1 because [0] is depot
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

			for (int m = 0; m < no_routes; m++)  //consider each and other routes (from R2)
			{
				if ((i == m) || (saiz[m]==0)) // || (demand[element2] > space_available[m])) //if same route or saiz=0, cannot perform (2-1)
				{
					continue;
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
						continue; //to get next element2
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
							if ( NR_FLAG_DEPOT[element1][before_pos2] == false && NR_FLAG_DEPOT[element1f][after_pos2] == false )
								continue;
						}
						else //if no depot
						{
							if ( NR_FLAG[element1][before_pos2] == false && NR_FLAG[element1f][after_pos2] == false )
								continue;
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
							
							if (i<m)
							{
								if (gain- gain_matrix_1_0[i][m] > epsilon)//gain > gain_matrix_1_0[i][m]
								{
									gain_matrix_1_0[i][m] = gain;

									//info[number_matrix_1_0[i][m]][0] = number_matrix_1_0[m][i];
									info_1_0[(int)gain_matrix_1_0[m][i]][1] = gain;
									info_1_0[(int)gain_matrix_1_0[m][i]][2] = i;
									info_1_0[(int)gain_matrix_1_0[m][i]][3] = m;
									info_1_0[(int)gain_matrix_1_0[m][i]][4] = j; //from r1 which position to delete
									info_1_0[(int)gain_matrix_1_0[m][i]][5] = p; //to position in r2
									info_1_0[(int)gain_matrix_1_0[m][i]][6] = n; //from position in r2
									info_1_0[(int)gain_matrix_1_0[m][i]][7] = q; //to position in r1
									info_1_0[(int)gain_matrix_1_0[m][i]][8] = 3; //3 is (2-1)
									info_1_0[(int)gain_matrix_1_0[m][i]][9] = reverseStatus; 
									info_1_0[(int)gain_matrix_1_0[m][i]][10] = cost_without_i - cost_with_i;//gain1
									info_1_0[(int)gain_matrix_1_0[m][i]][11] = origain1 - oriloss1;//gain1 in distance
									info_1_0[(int)gain_matrix_1_0[m][i]][12] = origain2 - oriloss2;//gain2 in distance

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
							}//end if (i>m)
						}//end for q
					}//end for p
					delete[] tempCumDist2;
				}//end for n
			}//end for m
			delete[] tempCumDist1;
		}//end for j
	}//end for i 
}

//DONE
//insert_2_1 (same route based on non-modified route, diff based on modified route
void insert_2_1(float max_gain, float *(&cost_of_removing), float **info_1_0, float **(same_r_info), int **(&VEHICLE), bool same_route)
{
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);
	float total_cost = 0.0;
	int r1=-1, r2=-1;
	
	float gain=0.0;
	int reverseStatus =0;
	/////////// ================  ALL CUSTOMER POSITION IN NON MODIFIED ROUTE!!!!!!!!!!!!!!!!!! =====================///////////////
	
	if(same_route == true) //if it is the same route, the position is to swap
	{
		int route = (int)same_r_info[number][2];
		int pos1 = same_r_info[number][4];
		int pos2 = pos1+1;
		int pos3 = same_r_info[number][5];
		int ele1 = VEHICLE[route][pos1];
		int ele2 = VEHICLE[route][pos2];
		int ele3 = VEHICLE[route][pos3];
		r1 = route;
		r2 = route;
		gain = same_r_info[number][1];
		//see<<route_cost[r1]<<' '<<route_cost[r2]<<endl;
		//see<<pos1<<' '<<pos3<<' '<<ele1<<' '<<ele2<<' '<<' '<<ele3<<gain<<endl;
		//non modified route
		
		reverseStatus = same_r_info[number][9];
		if (reverseStatus == 1)
		{
			int temp = ele1;
			ele1 = ele2;
			ele2 = temp;
		}

		int *temp_r3 = new int[saiz[route] + 2];

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
		distance_cost[route] = distance_cost[route] - same_r_info[number][11];
		//distance_cost[route] = 0.0;//reinitialize
		//for (int c = 0; c < saiz[route] + 1; c++)
		//{
		//	distance_cost[route] += dist[VEHICLE[route][c]][VEHICLE[route][c + 1]]  + service_time[VEHICLE[route][c]];
		//}
		route_cost[route] = route_cost[route] - gain; //////////////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		distance_available[route] = DISTANCE - distance_cost[route];
		
		//update CumDist starting from the affecting pos until last cust
		int start = pos1;//the start of update point, always hold old or new hold whichever comes first
		if (pos3 < pos1)
			start = pos3;
		for (int j = start; j <=saiz[route]; j++)
		{
			CumDist[route][j] = CumDist[route][j-1]+dist[VEHICLE[route][j-1]][VEHICLE[route][j]] + service_time[VEHICLE[route][j]];
			//CumuDCust[VEHICLE[route][j]]= CumDist[route][j];
		}
		delete[] temp_r3;
	}//end of if same route

	else if (same_route == false) //if different routes
	{
		int r1_without = info_1_0[number][4];//r1_w/o
		int r1f_without = (info_1_0[number][4]) + 1; //the next element
		int r2_with = info_1_0[number][5]; //r2_w
		int r2_without = info_1_0[number][6]; //r2_w/o
		int r1_with = info_1_0[number][7];//r1_with
		r1 = info_1_0[number][2];//from r1
		r2 = info_1_0[number][3];//to r2
		gain = info_1_0[number][1];
		//see<<route_cost[r1]<<' '<<route_cost[r2]<<endl;
		//see<<r1<<' '<<r2<<endl;
		//see<<r1_without<<' '<< r1f_without<<' '<<r2_with<<' '<<r2_without<<' '<<' '<<r1_with<<gain<<endl;
		//based on modified route
		
		std::vector<int> myvector1;
		std::vector<int> myvector2;
		int ele2=-1, ele3=-1, ele1=-1;

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

		for (int j = 0; j<=saiz[r2] + 1; j++)
		{
			if (j==r2_without)
			{
				ele1 = VEHICLE[r2][j];
				continue;
			}
			myvector2.push_back(VEHICLE[r2][j]);
		}

		reverseStatus = info_1_0[number][9];
		if (reverseStatus == 1)
		{
			int temp = ele2;
			ele2 = ele3;
			ele3 = temp;
		}

		//based on modified route
		

		myvector1.insert(myvector1.begin() + r1_with, ele1);
		myvector2.insert(myvector2.begin() + r2_with, ele3);
		myvector2.insert(myvector2.begin() + r2_with, ele2);

		saiz[r1] = myvector1.size() - 2; //minus front and back depot
		saiz[r2] = myvector2.size() - 2;

		for (int k = 0; k < myvector1.size(); k++)
		{
			VEHICLE[r1][k] = myvector1[k];  //copy to VEHICLE matrix
		}
		float new_sum_cost = route_cost[r1] + route_cost[r2] - gain;
		distance_cost[r1] = distance_cost[r1] - info_1_0[number][11];
		//distance_cost[r1]=0.0, distance_cost[r2]=0.0;	//reinitialize
		//for (int c = 0; c < saiz[r1] + 1; c++)
		//{
		//	distance_cost[r1] += dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]]+ service_time[VEHICLE[r1][c]];
		//}
		float check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[r1] + 1; c++)
		{
			check_cost+= dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]] + service_time[VEHICLE[r1][c]];
		}
		if (abs(check_cost-distance_cost[r1]) > 1.5)
		{
			cout<<"Cost diff in insert 2-1 r1"<<endl;
			getchar();
		}
		route_cost[r1] = route_cost[r1] - info_1_0[number][10];////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		total_demand[r1] = total_demand[r1] - demand[ele2] - demand[ele3] + demand[ele1];
		distance_available[r1] = DISTANCE - distance_cost[r1];
		space_available[r1] = CAPACITY - total_demand[r1];

		for (int p = 0; p < myvector2.size(); p++)
		{
			VEHICLE[r2][p] = myvector2[p];
		}
		distance_cost[r2] = distance_cost[r2] - info_1_0[number][12];
		//for (int c = 0; c < saiz[r2] + 1; c++)
		//{
		//	distance_cost[r2] += dist[VEHICLE[r2][c]][VEHICLE[r2][c + 1]]+ service_time[VEHICLE[r2][c]];
		//}
		check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[r2] + 1; c++)
		{
			check_cost+= dist[VEHICLE[r2][c]][VEHICLE[r2][c + 1]] + service_time[VEHICLE[r2][c]];
		}
		if (abs(check_cost-distance_cost[r2]) > 1.5)
		{
			cout<<"Cost diff in insert 2-1 r2"<<endl;
			getchar();
		}
		route_cost[r2] = new_sum_cost - route_cost[r1] ;

		total_demand[r2] = total_demand[r2] + demand[ele2] + demand[ele3] - demand[ele1];
		distance_available[r2] = DISTANCE - distance_cost[r2];
		space_available[r2] = CAPACITY - total_demand[r2];
		//update CumDist starting from the affecting pos until last cust
		int start = r1_without;//the start of update point, always hold old or new hold whichever comes first
		if (r1_with < r1_without)
			start = r1_with;
		for (int j = start; j <=saiz[r1]; j++)
		{
			CumDist[r1][j] = CumDist[r1][j-1]+dist[VEHICLE[r1][j-1]][VEHICLE[r1][j]] + service_time[VEHICLE[r1][j]];
			//CumuDCust[VEHICLE[r1][j]]= CumDist[r1][j];
		}
		//update CumDist starting from the affecting pos until last cust
		int start2 = r2_without;//the start of update point, always hold old or new hold whichever comes first
		if (r2_with < r2_without)
			start2 = r2_with;
		for (int j = start2; j <=saiz[r2]; j++)
		{
			CumDist[r2][j] = CumDist[r2][j-1]+dist[VEHICLE[r2][j-1]][VEHICLE[r2][j]] + service_time[VEHICLE[r2][j]];
			//CumuDCust[VEHICLE[r2][j]]= CumDist[r2][j];
		}


	}//end of else (different route)


	//route_file << ' ' << endl;
	//======================================================Display route========================================================================
	//route_file << "==================From insert (2-0)===================================== " <<"same_route = " << same_route<< endl;
	//if (same_route == false)
	//	route_file << "Max_gain= " <<max_gain<<" r1= "<<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r1w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
	//else
	//	route_file << "Max_gain= " <<max_gain<< " r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		//see << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] + 1; h++)
		//{
		//	see << VEHICLE[g][h] << ' ';
		//}
		//see<<endl;

		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
	}
	
	//see<<endl;
	//	see<<"CUmDIst after 2-1"<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{	
	//	for (int j = 0; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
	//	{
	//		see<<CumDist[i][j]<<' ';
	//	}
	//	see<<endl;
	//}
	//route_file << "Total cost = " << total_cost << endl;
	//route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	//route_file << "========================================================================== " << endl;
	//////=================================================update cost of removing=====================================//////////////////////
	if (same_route == false) //if diferent routes
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
	
			
			//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
			//{
			//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
			//}
			cout<<"Insert 2-1 diff R"<<endl;
			//int status=comprareCost (cost_of_removing, f);//to check 
			//if (status == 1)
			//	cout<<info_1_0[number][4]<<' '<<' '<<info_1_0[number][5]<<endl;
		}
	}

	else // if the same route
	{
		int f = r1;

		
		updateCostofRemoving2 (f, cost_of_removing);
	

		//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		//{
		//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
		//}
		cout<<"Insert 2-1 same R"<<endl;
		//int status=comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<same_r_info[number][4]<<' '<<' '<<same_r_info[number][5]<<endl;
	}


		float *checkR_cost = new float[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		
		checkR_cost[i]=0;
		for (int j = 1; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
		{
			checkR_cost[i] += CumDist[i][j];
		}	
	}
	bool wrong = false;
	for (int i = 0; i < no_routes; i++)
	{
		//cout<< "checkR_cost["<< i <<"]= " << checkR_cost[i];
		if (abs(route_cost[i]-checkR_cost[i]) > BIGC)
		{
			cout<<i<<"  This cost is different from original "<<endl;
			cout<<route_cost[i]<<' '<<checkR_cost[i]<<' ';
			wrong =true;
		}
		//if (checkR_cost[i] > DISTANCE)
		//	cout<<" Exceed DISTANCE"<<endl;
		//cout<<endl;
	}
	delete[] checkR_cost;
	if (wrong==true)
		getchar();

}


//DONE
//No same route, different route only
void reoptimize_2_1(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{

	int element1, element1f, element2, before_ele1, before_ele2, after_ele1=-1, after_ele2=-1,  before_pos1, before_pos2, after_pos1, after_pos2;
	int element3, before_ele3, after_ele3;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_jRev, cost_with_j, gain;
	int reverseStatus;

	std::vector<int> vector_r1;
	std::vector<int> vector_r2;
	std::vector<int> vector_r3;
	int r;

	int r1 = route_change[0]; //route (from)
	int r2 = route_change[1]; //route (to)
	
	int iter=2; //added this on 1Nov15, previous change may only involve 1 route
	if (r1 == r2)
		iter=1;

	///////////////////////////////=================================== SAME ROUTE =================================////////////////////////
	//for (int I = 0; I < iter; I++)
	//{
	//	if (I==0)
	//		r=r1;
	//	else
	//		r=r2;
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
	//			reverseStatus = 0; //initialize to 0
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
	//				if ( (NR_FLAG_DEPOT[element1][before_ele3] == false && NR_FLAG_DEPOT[element2][after_ele3] == false) || (NR_FLAG_DEPOT[element3][before_ele1] == false && NR_FLAG_DEPOT[element3][after_ele2] == false) )
	//					continue;
	//			}
	//			else if ((before_ele3 == SIZE) || (after_ele3 == SIZE) ) //if mn depot
	//			{
	//				if ( (NR_FLAG_DEPOT[element1][before_ele3] == false && NR_FLAG_DEPOT[element2][after_ele3] == false) || (NR_FLAG[element3][before_ele1] == false && NR_FLAG[element3][after_ele2] == false) )
	//					continue;
	//			}
	//			else if ((before_ele1 == SIZE) || (after_ele2 == SIZE) ) //if ij depot
	//			{
	//				if ( (NR_FLAG[element1][before_ele3] == false && NR_FLAG[element2][after_ele3] == false) || (NR_FLAG_DEPOT[element3][before_ele1] == false && NR_FLAG_DEPOT[element3][after_ele2] == false) )
	//					continue;
	//			}
	//			else //if no depot
	//			{
	//				if ( (NR_FLAG[element1][before_ele3] == false && NR_FLAG[element2][after_ele3] == false) || (NR_FLAG[element3][before_ele1] == false && NR_FLAG[element3][after_ele2] == false) )
	//					continue;
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
	//}//end I

	//////////////////////////////////////// ====================== DIFFERENT ROUTES =========================================////////////////////////////
	int k = 0;
	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//============================================(From) Affected routes: R1 and R2 to other routes (2-1)===========================================================
	for (int I = 0; I < iter; I++)  //consider each and other routes (from R1)
	{
		if (I == 0)
			k = r1;

		else
			k = r2;

		if ((saiz[k]==0) || (saiz[k]==1)) //if it is empty route
			continue;
		for (int j = 1; j < saiz[k]; j++)  // consider each customer (from R1), start from 1 because [0] is depot //customer considered in pair, from 1 until saiz-1
		{
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
							if (NR_FLAG_DEPOT[element1][before_pos2] == false && NR_FLAG_DEPOT[element1f][after_pos2] == false)
								continue;
						}
						else //if no depot
						{
							if (NR_FLAG[element1][before_pos2] == false && NR_FLAG[element1f][after_pos2] == false)
								continue;
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
	}//end I
	//============================================(To) From other routes to Affected routes: R1 and R2 (1-2) 1 from affected route, 2 from other route ===========================================================
	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from R1)
	{
		//if ((saiz[i]==0) || (saiz[i]==1)) //if it is empty route or saiz =1
		if (saiz[i]<=1)
			continue;
		for (int j = 1; j < saiz[i]; j++)  // consider each customer (from R1), start from 1 because [0] is depot
		{
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
			
			for (int I = 0; I < iter; I++)  //consider each and other routes (from R2)
			{
				int m;
				if (I == 0)
					m = r1;

				else
					m = r2;

				if ((i == m)||(saiz[m]==0)) // || (demand[element2] > space_available[m])) //if customer already in the route or demand exceed avaialable space in route
				{
					continue;
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
							if (NR_FLAG_DEPOT[element1][before_pos2] == false && NR_FLAG_DEPOT[element1f][after_pos2] == false)
								continue;
						}
						else //if no depot
						{
							if (NR_FLAG[element1][before_pos2] == false && NR_FLAG[element1f][after_pos2] == false)
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
}


void best_gain_2_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{
	//cout<<"In 2-0 "<<endl;
	//for (int g = 0; g < no_routes; g++)
	//{
	//	if (saiz[g] == 0)
	//		route_cost[g]=0;

	//	cout << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
	//	for (int h = 0; h < saiz[g] + 2; h++)
	//	{
	//		cout << VEHICLE[g][h] << ' ';
	//	}
	//	cout<<endl;
	//}
	
	int element1, element1f, before_ele, after_ele, before_pos, after_pos;
	float cost_without_i, cost_with_i, cost_with_iRev, gain;
	int reverseStatus;

	/////////////================================== SAME ROUTE =========================////////////////////////////
	////////////============ insertion position is based on the modified route ================/////////////////////////
	// only 1-0 (same route) insertion position is based on modified route, 1-1 and 2-1 are based on non-modified route
	for (int i = 0; i < no_routes; i++) 
	{
		//if ((saiz[i]==0) || (saiz[i]==1) ||(saiz[i]==2)) //if it is empty route
		if (saiz[i]<=2)	
			continue;
		for (int j = 1; j < saiz[i]; j++) //which one to delete (always take a pair, so from 1 until SAIZ-1
		{
			float available_dist_r2 = distance_available[i];
			//copy a temporary route
			int b=0;
		
			int* temp_r = new int[saiz[i]+2];
			for (int a = 0; a < saiz[i] + 2; a++)
			{
				if ((a == j) || (a == j+1))
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
			element1f = VEHICLE[i][j+1];
		
			before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
			after_ele = VEHICLE[i][j + 2];
			
			float origain = dist[element1][before_ele] + dist[element1][element1f] + dist[element1f][after_ele] + service_time[element1] + service_time[element1f] - dist[before_ele][after_ele]; //cost of r1 without i and i+1 // old-new
			float *tempCumDist= new float[SIZE];
			tempCumDist[0] = 0;
			for (int t = 1; t <= saiz[i]-2; t++)//have 2 size less because 2 customers are deleted
			{
				if (t<j)
					tempCumDist[t]= CumDist[i][t];//before the delete pos, the CumDist is the same
				else if(t>=j)
					tempCumDist[t]=CumDist[i][t+2]-origain;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
					
			}

			for (int n = 1; n < saiz[i] ; n++) //has 1 customer less because it has been deleted
			{
				reverseStatus = 0; //initilaize to 0
				//element1 = VEHICLE[i][j];//must initialize again because elemnt1 and 1f might be reversed just now
				//element1f = VEHICLE[i][j+1];

				if (j == n) //originally from this position, so noneed to consider
					continue; 
				else
				{
					before_pos = temp_r[n - 1];
					after_pos = temp_r[n];  //noneed to add 1 because it is insert in between n-1 and n
					
				}	
				
				if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
				{
					if ( NR_FLAG_DEPOT[element1][before_pos] == false && NR_FLAG_DEPOT[element1f][after_pos] == false ) 
						continue;
				}

				else //if not between customer and depot
				{	
					if ( NR_FLAG[element1][before_pos] == false && NR_FLAG[element1f][after_pos] == false )
						continue;
				}
				
				//First, delete the two elements
				//float origain = dist[element1][before_ele] + dist[element1][element1f] + dist[element1f][after_ele] - dist[before_ele][after_ele]; //cost of r1 without i and i+1 // old-new
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

				float oriloss =  dist[before_pos][element1] + dist[element1][element1f]+dist[element1f][after_pos] + service_time[element1] + service_time[element1f] - dist[before_pos][after_pos]; //cost of r2 with i and i+1 //new-old , big not good, small is good
				float orilossRev =  dist[before_pos][element1f] + dist[element1f][element1]+dist[element1][after_pos] + service_time[element1] + service_time[element1f] - dist[before_pos][after_pos]; //cost of r2 with i+1 and i //new-old
				
				float newCumDist1= tempCumDist[n-1] + dist[before_pos][element1] + service_time[element1]; 
				float newCumDist2= newCumDist1 + dist[element1][element1f] + service_time[element1f]; 
				
				float newCumDist1Rev= tempCumDist[n-1] + dist[before_pos][element1f] + service_time[element1f]; 
				float newCumDist2Rev= newCumDist1Rev + dist[element1f][element1] + service_time[element1]; 
				
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

	//########################################## calculate GAIN when remove 2 and insert 2 in other route ######################################################//
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	for (int i = 0; i < no_routes; i++)  //consider each and other routes (from R1)
	{
		//if ((saiz[i]==0) || (saiz[i]==1)) //if it is empty route or saiz=1
		if (saiz[i]<=1)
			continue;
		for (int j = 1; j < saiz[i]; j++)  // consider each customer (from R1), start from 1 because [0] is depot //customer considered in pair, from 1 until saiz-1
		{
			element1 = VEHICLE[i][j];			//element to be inserted into other route
			element1f = VEHICLE[i][j + 1];
	
			for (int m = 0; m < no_routes; m++)  //consider each and other routes (from R2)
			{
				if ((i == m)  || (demand[element1]+demand[element1f] > space_available[m])) //if same route or demand exceed avaialable space in route
				{
					continue;		
				}
				for (int n = 1; n <= saiz[m] + 1; n++) //which position in r2 to insert
				{		
					reverseStatus = 0; //initialize to 0
					//element1 = VEHICLE[i][j];			//must initialize again because elemnt1 and 2 might be reversed just now
					//element1f = VEHICLE[i][j + 1];
					before_pos = VEHICLE[m][n - 1];
					after_pos = VEHICLE[m][n];  //noneed to add 1 because it is insert in between n-1 and n

					if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
					{
						if ( NR_FLAG_DEPOT[element1][before_pos] == false && NR_FLAG_DEPOT[element1f][after_pos] == false )
							continue;
					}

					else //if not between customer and depot
					{	
						if ( NR_FLAG[element1][before_pos] == false && NR_FLAG[element1f][after_pos] == false )
							continue;
					}	

					float available_dist_r2 = distance_available[m];
					before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[i][j + 2];

					//for r1 delete customer
					float origain = (dist[element1][before_ele] + dist[element1][element1f] + dist[element1f][after_ele]+ service_time[element1]+ service_time[element1f]) - (dist[before_ele][after_ele]); //cost of r1 without i and i+1 //old-new
					int remainingcust = saiz[i]-(j+1);//always take the second customer j+1
					if (remainingcust < 0)//if negative
							remainingcust = 0;
					float oriCumDist1 = CumDist[i][j];
					float oriCumDist2 = CumDist[i][j+1];
					cost_without_i = oriCumDist1 +  oriCumDist2 + remainingcust*origain;
				
					//for r2, insert customer
					float oriloss = (dist[before_pos][element1] + dist[element1][element1f] + dist[element1f][after_pos]+ service_time[element1]+ service_time[element1f]) - (dist[before_pos][after_pos]); //cost of r2 with i and i+1 //new-old
					float orilossRev = (dist[before_pos][element1f] + dist[element1f][element1] + dist[element1][after_pos]+ service_time[element1]+ service_time[element1f]) - (dist[before_pos][after_pos]); //cost of r2 with i and i+1 //new-old

					float newCumDist1 = CumDist[m][n-1] + dist[before_pos][element1] + service_time[element1]; 
					float newCumDist2 = newCumDist1 + dist[element1][element1f] + service_time[element1f];

					float newCumDist1Rev = CumDist[m][n-1] + dist[before_pos][element1f] + service_time[element1f]; 
					float newCumDist2Rev = newCumDist1Rev + dist[element1f][element1] + service_time[element1];

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
					if (i < m)
					{
						if (gain-gain_matrix_1_0[i][m] > epsilon)//gain > gain_matrix_1_0[i][m]
						{
							gain_matrix_1_0[i][m] = gain;
							info_1_0[(int)gain_matrix_1_0[m][i]][0] = gain_matrix_1_0[m][i];
							info_1_0[(int)gain_matrix_1_0[m][i]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[m][i]][2] = i; //from_route
							info_1_0[(int)gain_matrix_1_0[m][i]][3] = m; //to_route
							info_1_0[(int)gain_matrix_1_0[m][i]][4] = j; //from position
							info_1_0[(int)gain_matrix_1_0[m][i]][5] = n; //to position
							info_1_0[(int)gain_matrix_1_0[m][i]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[m][i]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[m][i]][8] = 4; //4 is (2-0)
							info_1_0[(int)gain_matrix_1_0[m][i]][9] = reverseStatus; 
							info_1_0[(int)gain_matrix_1_0[m][i]][10] = cost_without_i ;//gain1
							info_1_0[(int)gain_matrix_1_0[m][i]][11] = origain;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[m][i]][12] = -oriloss;//gain2 in distance
						}
					}

					else if (m < i)
					{
						if (gain-gain_matrix_1_0[m][i] > epsilon)//gain > gain_matrix_1_0[m][i]
						{
							gain_matrix_1_0[m][i] = gain;
							info_1_0[(int)gain_matrix_1_0[i][m]][0] = gain_matrix_1_0[i][m];
							info_1_0[(int)gain_matrix_1_0[i][m]][1] = gain;
							info_1_0[(int)gain_matrix_1_0[i][m]][2] = i;
							info_1_0[(int)gain_matrix_1_0[i][m]][3] = m;
							info_1_0[(int)gain_matrix_1_0[i][m]][4] = j;
							info_1_0[(int)gain_matrix_1_0[i][m]][5] = n;
							info_1_0[(int)gain_matrix_1_0[i][m]][6] = -1;
							info_1_0[(int)gain_matrix_1_0[i][m]][7] = -1;
							info_1_0[(int)gain_matrix_1_0[i][m]][8] = 4; //4 is (2-0)
							info_1_0[(int)gain_matrix_1_0[i][m]][9] = reverseStatus;
							info_1_0[(int)gain_matrix_1_0[i][m]][10] = cost_without_i;//gain1
							info_1_0[(int)gain_matrix_1_0[i][m]][11] = origain;//gain1 in distance
							info_1_0[(int)gain_matrix_1_0[i][m]][12] = -oriloss;//gain2 in distance
						}
					}
				}//end for n
			}//end for m
		}//end for j
	}//end for i 
	
}
//DONE
//insert_2_0 (same route based on modified route, diff based on nothing
void insert_2_0(float max_gain, float *(&cost_of_removing), float **info_1_0, float **(same_r_info), int **(&VEHICLE), bool same_route) //from is fromroute 
{
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);
	float total_cost = 0.0;
	int old_pos = -1;
	int new_pos = -1;
	int ele1 = -1, ele2=-1;
	int from = -1;
	int to = -1;
	
	float gain = 0.0;
	int reverseStatus;


	if (same_route == true)//if it is from the same route //////////////////////// POSITION OF INSERTION IS FOR MODIFIED ROUTE !!!!!!!!!!!!!!! ////////////////////////////
	{
		int route = (int)same_r_info[number][2]; //from_route = to_route
		old_pos = same_r_info[number][4];
		new_pos = same_r_info[number][5];  //this position is based on modified route!!!!!!!!, assumed two cust has been deleted
		ele1 = VEHICLE[route][old_pos];
		ele2 = VEHICLE[route][old_pos+1]; 
		from = route;
		to = route;
		gain = same_r_info[number][1];
		reverseStatus = same_r_info[number][9];
		//see<<route_cost[from]<<' '<<route_cost[to]<<endl;
		//see<<old_pos<<' '<<new_pos<<' '<<ele1<<' '<<ele2<<' '<<' '<<from<<' '<<' '<<to<<' '<<gain<<' '<<reverseStatus<<endl;
		
		if (reverseStatus == 1)
		{
			int temp = ele1;
			ele1 = ele2;
			ele2 = temp;
		}

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
		distance_cost[route] = distance_cost[route] - same_r_info[number][11]; 
		//distance_cost[route] = 0; //initialize
		//for (int c = 0; c < saiz[route] + 1; c++)
		//{
		//	distance_cost[route] += dist[VEHICLE[route][c]][VEHICLE[route][c + 1]]+ service_time[VEHICLE[route][c]];
		//}
		float check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[route] + 1; c++)
		{
			check_cost+= dist[VEHICLE[route][c]][VEHICLE[route][c + 1]] + service_time[VEHICLE[route][c]];
		}
		if (abs(check_cost-distance_cost[route]) > 1.5)
		{
			cout<<"Cost diff in insert 2-0 "<<endl;
			cout<<check_cost<<' '<<distance_cost[route]<<endl;
			getchar();
		}
		route_cost[route] = route_cost[route] - gain;
		distance_available[route] = DISTANCE - distance_cost[route];

		//update CumDist starting from the affecting pos until last cust
		int start = old_pos;//the start of update point, always hold old or new hold whichever comes first
		if (new_pos < old_pos)
			start = new_pos;
		for (int j = start; j <=saiz[route]; j++)
		{
			CumDist[route][j] = CumDist[route][j-1]+dist[VEHICLE[route][j-1]][VEHICLE[route][j]] + service_time[VEHICLE[route][j]];
			//CumuDCust[VEHICLE[route][j]]= CumDist[route][j];
		}
		
		delete[] temp_r;
		
	}
	//based on nothing
	else if (same_route == false) //if it is from different route
	{
		old_pos = (int)info_1_0[number][4]; //from_position
		new_pos = (int)info_1_0[number][5]; //to_position in other route
		from = (int)info_1_0[number][2]; //from_route
		to = (int)info_1_0[number][3]; //to_route
		gain = info_1_0[number][1];
		//see<<route_cost[from]<<' '<<route_cost[to]<<endl;
		//see<<old_pos<<' '<<new_pos<<' '<<from<<' '<<to<<' '<<gain<<endl;
		reverseStatus = info_1_0[number][9];

		
		ele1 = VEHICLE[from][old_pos]; //must state before from route changed
		ele2 = VEHICLE[from][old_pos+1];

		//r1 (delete 2)	
		for (int i = old_pos; i<=saiz[from]-1; i++)// cust 0-4, can delete 5 customers
		{
			VEHICLE[from][i] = VEHICLE[from][i+2];
		}
		VEHICLE[from][saiz[from]] = -1;//optional
		VEHICLE[from][saiz[from]] = -1;//optional
		saiz[from] = saiz[from] - 2;
		float new_sum_cost = route_cost[from] + route_cost[to] - gain;
		distance_cost[from] = distance_cost[from] - info_1_0[number][11];
		//distance_cost[from]=0.0, distance_cost[to]=0.0;	//reinitialize
		//for (int c = 0; c < saiz[from] + 1; c++)
		//{
		//	distance_cost[from] += dist[VEHICLE[from][c]][VEHICLE[from][c + 1]]+ service_time[VEHICLE[from][c]];
		//}
		float check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[from] + 1; c++)
		{
			check_cost+= dist[VEHICLE[from][c]][VEHICLE[from][c + 1]] + service_time[VEHICLE[from][c]];
		}
		if (abs(check_cost-distance_cost[from]) > 1.5)
		{
			cout<<"Cost diff in insert 2-0 from"<<endl;
			getchar();
		}
		route_cost[from] = route_cost[from] - info_1_0[number][10];////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		total_demand[from] = total_demand[from] - demand[ele1] - demand[ele2];
		distance_available[from] = DISTANCE - distance_cost[from];
		space_available[from] = CAPACITY - total_demand[from];
		//update CumDist starting from the affecting pos until last cust
		int start = old_pos;//the start of update point
		for (int j = start; j <=saiz[from]; j++)
		{
			CumDist[from][j] = CumDist[from][j-1]+dist[VEHICLE[from][j-1]][VEHICLE[from][j]] + service_time[VEHICLE[from][j]];
			//CumuDCust[VEHICLE[from][j]]= CumDist[from][j];
		}

		//r2 (insert 2)	
		if (reverseStatus == 1 )
		{
			int temp = ele1;
			ele1 = ele2;
			ele2 = temp;
		}

		int j = saiz[to]+1; 

		while ( j >= new_pos && j>0 )
		{
			VEHICLE[to][j+2] = VEHICLE[to][j];
			j--;
		}	
		VEHICLE[to][new_pos] = ele1;
		VEHICLE[to][new_pos+1] = ele2;
		saiz[to] = saiz[to] + 2; //size of new route is plus 2
		route_cost[to] = new_sum_cost - route_cost[from];
		distance_cost[to] = distance_cost[to] - info_1_0[number][12];	
		//distance_cost[to] = 0;//initialize
		//for (int c = 0; c < saiz[to] + 1; c++)
		//{
		//	distance_cost[to] += dist[VEHICLE[to][c]][VEHICLE[to][c + 1]]+ service_time[VEHICLE[to][c]];
		//}
		check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[to] + 1; c++)
		{
			check_cost+= dist[VEHICLE[to][c]][VEHICLE[to][c + 1]] + service_time[VEHICLE[to][c]];
		}
		if (abs(check_cost-distance_cost[to]) > 1.5)
		{
			cout<<"Cost diff in insert 2-0 to"<<endl;
			getchar();
		}
		total_demand[to] = total_demand[to] + demand[ele1] + demand[ele2];
		distance_available[to] = DISTANCE - distance_cost[to];
		space_available[to] = CAPACITY - total_demand[to];
		//update CumDist starting from the affecting pos until last cust
		int start2 = new_pos;//the start of update point
		for (int j = start2; j <=saiz[to]; j++)
		{
			CumDist[to][j] = CumDist[to][j-1]+dist[VEHICLE[to][j-1]][VEHICLE[to][j]] + service_time[VEHICLE[to][j]];
			//CumuDCust[VEHICLE[to][j]]= CumDist[to][j];
		}

		
	}
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	//route_file << ' ' << endl;
	//======================================================Display route========================================================================
	//route_file << "==================From insert (2-0)===================================== " <<"same_route = " << same_route<< endl;
	//if (same_route == false)
	//	route_file << "Max_gain= " <<max_gain<<" r1= "<<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r1w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
	//else
	//	route_file << "Max_gain= " <<max_gain<< " r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		//see << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] + 1; h++)
		//{
		//	see << VEHICLE[g][h] << ' ';
		//}
		//see<<endl;

		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
	}
	
	//see<<endl;
	//	see<<"CUmDIst after 2-0"<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{	
	//	for (int j = 0; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
	//	{
	//		see<<CumDist[i][j]<<' ';
	//	}
	//	see<<endl;
	//}
	//route_file << "Total cost = " << total_cost << endl;
	//route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	//route_file << "========================================================================== " << endl;
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
			
			
			//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
			//{
			//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
			//}
			cout<<"Insert 2-0 diff R"<<endl;
			//int status=comprareCost (cost_of_removing, f);//to check 
			//if (status == 1)
			//	cout<<info_1_0[number][4]<<' '<<' '<<info_1_0[number][5]<<endl;
		}
	}

	else // if the same route
	{
		int f = from;

		
		updateCostofRemoving2 (f, cost_of_removing);


		//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		//{
		//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
		//}
		cout<<"Insert 2-0 same R"<<endl;
		//int status=comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<same_r_info[number][4]<<' '<<' '<<same_r_info[number][5]<<endl;
	}

		float *checkR_cost = new float[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		
		checkR_cost[i]=0;
		for (int j = 1; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
		{
			checkR_cost[i] += CumDist[i][j];
		}	
	}
	bool wrong = false;
	for (int i = 0; i < no_routes; i++)
	{
		//cout<< "checkR_cost["<< i <<"]= " << checkR_cost[i];
		if (abs(route_cost[i]-checkR_cost[i]) > BIGC)
		{
			cout<<i<<"  This cost is different from original "<<endl;
			cout<<route_cost[i]<<' '<<checkR_cost[i]<<' ';
			wrong =true;
		}
		//if (checkR_cost[i] > DISTANCE)
		//	cout<<" Exceed DISTANCE"<<endl;
		//cout<<endl;
	}
	delete[] checkR_cost;
	//if (wrong==true)
	//	getchar();
	
}

//DONE
void reoptimize_2_0(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{
	//cout<<"In 2-0 reoptimize"<<endl;
	//for (int g = 0; g < no_routes; g++)
	//{
	//	if (saiz[g] == 0)
	//		route_cost[g]=0;

	//	cout << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
	//	for (int h = 0; h < saiz[g] + 2; h++)
	//	{
	//		cout << VEHICLE[g][h] << ' ';
	//	}
	//	cout<<endl;

	//}
	int element1=-1, element2=-1, before_ele, after_ele,  before_pos, after_pos;
	float gain;
	int from1, to1;
	float cost_without_i, cost_with_i, cost_with_iRev;
	int r1 = route_change[0]; //route (from)
	int r2 = route_change[1]; //route (to)
	int reverseStatus;
	//int r1 = sPtr->r1; //route (from)
	//int r2 = sPtr->r1; //route (to)

	int i = -1; //initilize for affected route
	//////////////////================================== SAME ROUTE =========================////////////////////////////////////
	int iter=2; //added this on 1Nov15, previous change may only involve 1 route
	if (r1 == r2)
		iter=1;
	for (int r = 0; r < iter; r++) 
	{
		if(r==0)
			i=r1;
		else
			i=r2;
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
			if (element1 == SIZE || element2==SIZE)
			{
				cout<<"in reoptimize2_0"<<endl;
				getchar();
			}
			before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
			after_ele = VEHICLE[i][j + 2];
			
			float origain = dist[element1][before_ele] + dist[element1][element2] + dist[element2][after_ele] + service_time[element1]+ service_time[element2] - dist[before_ele][after_ele]; //cost of r1 without i and i+1 // old-new		
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
				reverseStatus = 0;//initialize to 0
				//element1 = VEHICLE[i][j];//must initialize again because elemnt1 and 1f might be reversed just now
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
					if ( NR_FLAG_DEPOT[element1][before_pos] == false && NR_FLAG_DEPOT[element2][after_pos] == false )
						continue;
				}

				else //if not between customer and depot
				{	
					if ( NR_FLAG[element1][before_pos] == false && NR_FLAG[element2][after_pos] == false )
						continue;
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
	for (int i = 0; i < iter; i++)  //consider each and other routes (from)
	{
		if (i == 0)
			from1 = r1;

		else
			from1 = r2;
		
		//if ((saiz[from1]==0) || (saiz[from1]==1)) //if it is empty route, or saiz=1
		if (saiz[from1]<=1)
			continue;
		for (int j = 1; j < saiz[from1]; j++)  // consider each customer (from)//customer considered in pair, from 1 until saiz-1
		{
			element1 = VEHICLE[from1][j];			//elemet to be inserted into other route
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
					reverseStatus = 0; //initialize to 0
					//element1 = VEHICLE[from1][j];			//must reinitialize because element1 and 2 might have been reversed just now
					//element2 = VEHICLE[from1][j+1];
					before_ele = VEHICLE[from1][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[from1][j + 2];
					before_pos = VEHICLE[m][n - 1];
					after_pos = VEHICLE[m][n];  //noneed to add 1 because it is insert in between n-1 and n

					if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
					{
						if ( NR_FLAG_DEPOT[element1][before_pos] == false && NR_FLAG_DEPOT[element2][after_pos] == false )
							continue;
					}
					else //if not between customer and depot
					{	
						if ( NR_FLAG[element1][before_pos] == false && NR_FLAG[element2][after_pos] == false )
							continue;
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
			if (element1==SIZE || element2 ==SIZE)
			{
				cout<<"in reoptimize 2_0"<<endl;
				getchar();
			}

			//cout << element;

			for (int m = 0; m < iter; m++)  //insert (to) which route
			{
				if (m == 0)
					to1 = r1;

				else
					to1 = r2;

				float available_dist_r2 = distance_available[to1];
				if ((i == to1) || (demand[element1] + demand[element2] > space_available[to1])) //if demand exceed avaialable space in route && differnt route, if same route, capacity exceed stil have to find best gain
				{
					continue;
	
				}

				for (int n = 1; n < saiz[to1] + 2; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
				{
					reverseStatus = 0; //initialize to 0
					//element1 = VEHICLE[i][j];			//elemet to be inserted into other route
					//element2 = VEHICLE[i][j+1];
					before_ele = VEHICLE[i][j - 1];   //initially, unless stated otherwise
					after_ele = VEHICLE[i][j + 2];
					before_pos = VEHICLE[to1][n - 1];
					after_pos = VEHICLE[to1][n];  //noneed to add 1 because it is insert in between n-1 and n

					if ( before_pos == SIZE || after_pos == SIZE  ) //if the insertion is between customer and depot
					{
						if ( NR_FLAG_DEPOT[element1][before_pos] == false && NR_FLAG_DEPOT[element2][after_pos] == false )
							continue;
					}
					else //if not between customer and depot
					{	
						if ( NR_FLAG[element1][before_pos] == false && NR_FLAG[element2][after_pos] == false )
							continue;
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
void best_gain_2_2swap(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{

	int element1, element1f, element2, element2f, before_ele1, before_ele2, after_ele1=-1, after_ele2=-1;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_j, cost_with_i_Reverse, cost_with_j_Reverse, cost_with_both_Reverse, gain, dist_gain;
	float dist_without_i, dist_without_j, dist_with_i, dist_with_j, dist_with_i_Reverse, dist_with_j_Reverse, dist_with_both_Reverse, gain_with_i_Reverse, gain_with_j_Reverse, gain_with_both_Reverse;

	int temp = -1;
	int reverseType = 0;//initialize zero means no reverse for both
	//================================  if in the same route, just swap customer  ====================================//
	for(int r=0; r < no_routes; r++)
	{
		//if ((saiz[r]==0)||(saiz[r]==1)) //if it is empty route or saiz=1, nothing to reshuffle
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
				reverseType = 0;//initialize to 0
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
			}		
		}		
	}

	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	reverseType = 0;//initialize to zero means no reverse
	for (int i = 0; i < no_routes-1; i++)  //consider each and other routes (from R1)	//changed to no_routes-1 on 6April2015
	{
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
	}//end for i 	
}

//insert_2_2swap (same route based on non-modified route, diff based on non-modified route
void insert_2_2swap(float max_gain, float *(&cost_of_removing), float **info_1_0, float **(same_r_info), int **(&VEHICLE), bool same_route)
{
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);
	float total_cost = 0.0;
	int r1=-1, r2=-1;
	
	float gain=0.0;
	int reverseStatus =0;
	/////////// ================  ALL CUSTOMER POSITION IN NON MODIFIED ROUTE!!!!!!!!!!!!!!!!!! =====================///////////////
	int pos1=-1, pos2=-1, pos3=-1, pos4=-1, ele1=-1, ele2=-1, ele3=-1, ele4=-1;

	if(same_route == true) //if it is the same route, the position is to swap
	{
		gain = same_r_info[number][1];
		r1 = r2 = (int)same_r_info[number][2];
		pos1 = same_r_info[number][4]; //same_r_info[number][4] = same_r_info[number][7] because this is swap
		pos2 = pos1+1;
		pos3 = same_r_info[number][6];//same_r_info[number][5] = same_r_info[number][6] because this is swap
		pos4 = pos3+1;
		reverseStatus = same_r_info[number][9];

	}
	else //if different route
	{
		gain = info_1_0[number][1];
		r1 = info_1_0[number][2];//from r1
		r2 = info_1_0[number][3];//to r2
		pos1 = info_1_0[number][4]; //info_1_0[number][4] = info_1_0[number][7] because this is swap
		pos2 = pos1+1;
		pos3 = info_1_0[number][6];//info_1_0[number][5] = info_1_0[number][6] because this is swap
		pos4 = pos3+1;
		reverseStatus = info_1_0[number][9];

	}
	ele1 = VEHICLE[r1][pos1];
	ele2 = VEHICLE[r1][pos2];
	ele3 = VEHICLE[r2][pos3];
	ele4 = VEHICLE[r2][pos4];
	//see<< gain<<' '<<r1<<' '<<r2<<' '<<pos1<< ' '<<pos3<< ' '<<reverseStatus<<endl;
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
		
	if (same_route == true) //if it is the same route, swap for one route
	{
		
		VEHICLE[r1][pos1]= ele3;
		VEHICLE[r1][pos2]= ele4;
		VEHICLE[r1][pos3]= ele1;
		VEHICLE[r1][pos4]= ele2;
		distance_cost[r1]= distance_cost[r1] - same_r_info[number][11];
		//distance_cost[r1]= 0.0;
		//for (int c = 0; c < saiz[r1] + 1; c++)
		//{
		//	distance_cost[r1] += dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]]  + service_time[VEHICLE[r1][c]];
		//}
		route_cost[r1] = route_cost[r1] - gain; //////////////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		float check_dist= 0.0; //initialize
		for (int c = 0; c < saiz[r1] + 1; c++)
		{
			check_dist+= dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]] + service_time[VEHICLE[r1][c]];
		}
		if (abs(check_dist-distance_cost[r1]) > 1.5)
		{
			cout<<"Cost diff in insert 2-2 sameR "<<endl;
			cout<<check_dist<<' '<<distance_cost[r1]<<endl;
			getchar();
		}
		distance_available[r1] = DISTANCE - distance_cost[r1];
		//update CumDist starting from the affecting pos until last cust
		int start = pos1;//the start of update point, always hold old or new hold whichever comes first
		if (pos3 < pos1)
			start = pos3;
		for (int j = start; j <=saiz[r1]; j++)
		{
			CumDist[r1][j] = CumDist[r1][j-1]+dist[VEHICLE[r1][j-1]][VEHICLE[r1][j]] + service_time[VEHICLE[r1][j]];
			//CumuDCust[VEHICLE[r1][j]]= CumDist[r1][j];
		}
	}
		
	else //different route
	{
		

		VEHICLE[r1][pos1]= ele3;
		VEHICLE[r1][pos2]= ele4;
		VEHICLE[r2][pos3]= ele1;
		VEHICLE[r2][pos4]= ele2;

		float new_sum_cost = route_cost[r1] + route_cost[r2] - gain;
		distance_cost[r1]= distance_cost[r1] - info_1_0[number][11];
		//distance_cost[r1]=0.0, distance_cost[r2]=0.0;	//reinitialize
		//for (int c = 0; c < saiz[r1] + 1; c++)
		//{
		//	distance_cost[r1] += dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]]+ service_time[VEHICLE[r1][c]];
		//}
		route_cost[r1] = route_cost[r1] - info_1_0[number][10];////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!
		float check_dist= 0.0; //initialize
		for (int c = 0; c < saiz[r1] + 1; c++)
		{
			check_dist+= dist[VEHICLE[r1][c]][VEHICLE[r1][c + 1]] + service_time[VEHICLE[r1][c]];
		}
		if (abs(check_dist-distance_cost[r1]) > 1.5)
		{
			cout<<"Dist diff in insert 2-2 diffR "<<endl;
			cout<<check_dist<<' '<<distance_cost[r1]<<endl;
			getchar();
		}

		total_demand[r1] = total_demand[r1] - demand[ele1] - demand[ele2] + demand[ele3] + demand[ele4];
		distance_available[r1] = DISTANCE - distance_cost[r1];
		space_available[r1] = CAPACITY - total_demand[r1];
		
		//update CumDist starting from the affecting pos until last cust
		int start = pos1;//the start of update point, always hold old or new hold whichever comes first
		for (int j = start; j <=saiz[r1]; j++)
		{
			CumDist[r1][j] = CumDist[r1][j-1]+dist[VEHICLE[r1][j-1]][VEHICLE[r1][j]] + service_time[VEHICLE[r1][j]];
			//CumuDCust[VEHICLE[r1][j]]= CumDist[r1][j];
		}
		distance_cost[r2]= distance_cost[r2] - info_1_0[number][12];
		check_dist= 0.0; //initialize
		for (int c = 0; c < saiz[r2] + 1; c++)
		{
			check_dist+= dist[VEHICLE[r2][c]][VEHICLE[r2][c + 1]] + service_time[VEHICLE[r2][c]];
		}
		if (abs(check_dist-distance_cost[r2]) > 1.5)
		{
			cout<<"Dist diff in insert 2-2 diffR "<<endl;
			cout<<check_dist<<' '<<distance_cost[r2]<<endl;
			getchar();
		}
		route_cost[r2] = new_sum_cost - route_cost[r1] ;

		total_demand[r2] = total_demand[r2] - demand[ele3] - demand[ele4] + demand[ele1] + demand[ele2];
		distance_available[r2] = DISTANCE - distance_cost[r2];
		space_available[r2] = CAPACITY - total_demand[r2];

		//update CumDist starting from the affecting pos until last cust
		int start2 = pos3;//the start of update point, always hold old or new hold whichever comes first
		for (int j = start2; j <=saiz[r2]; j++)
		{
			CumDist[r2][j] = CumDist[r2][j-1]+dist[VEHICLE[r2][j-1]][VEHICLE[r2][j]] + service_time[VEHICLE[r2][j]];
			//CumuDCust[VEHICLE[r2][j]]= CumDist[r2][j];
		}
	}//end of else (different route)


	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);
	//route_file << ' ' << endl;
	//======================================================Display route========================================================================
	
	//route_file << "==================From insert (2-2)============================== " <<"same_route = " << same_route<< endl;
	//if (same_route == false)
	//	route_file << "Max_gain= " <<max_gain<< " r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r1w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
	//else
	//	route_file << "Max_gain= " <<max_gain<< " r1= " <<same_r_info[number][2]<<" r2= " <<same_r_info[number][3]<<" r1_w/o= "<<same_r_info[number][4]<<" r2w= "<<same_r_info[number][5]<<" || r2w/o= "<<same_r_info[number][6]<<" r1w= "<<same_r_info[number][7]<<" reverse status= "<<same_r_info[number][9]<<endl;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		//route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] + 1; h++)
		//{
		//	route_file << VEHICLE[g][h] << ' ';
		//}
		//route_file<<endl;
		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
		
	}

	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		//see << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] + 1; h++)
		//{
		//	see << VEHICLE[g][h] << ' ';
		//}
		//see<<endl;

		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
	}
	
	//see<<endl;
	//see<<"CUmDIst after 2-2swap"<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{	
	//	for (int j = 0; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
	//	{
	//		see<<CumDist[i][j]<<' ';
	//	}
	//	see<<endl;
	//}

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
	
			
			//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
			//{
			//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]]- dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
			//}
			cout<<"Insert 2-2 diff R"<<endl;
			//int status=comprareCost (cost_of_removing, f);//to check 
			//if (status == 1)
			//	cout<<info_1_0[number][4]<<' '<<' '<<info_1_0[number][6]<<endl;
		}
	}
	
	else // if the same route
	{
		int f = r1;

		
		updateCostofRemoving2 (f, cost_of_removing);


		//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		//{
		//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]]- dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
		//}
		cout<<"Insert 2-2 same R"<<endl;
		//int status=comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<same_r_info[number][4]<<' '<<' '<<same_r_info[number][5]<<endl;

	}
	
	float *checkR_cost = new float[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		
		checkR_cost[i]=0;
		for (int j = 1; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
		{
			checkR_cost[i] += CumDist[i][j];
		}	
	}
	bool wrong = false;
	for (int i = 0; i < no_routes; i++)
	{
		//cout<< "checkR_cost["<< i <<"]= " << checkR_cost[i];
		if (abs(route_cost[i]-checkR_cost[i]) > BIGC)
		{
			cout<<i<<"  This cost is different from original "<<endl;
			cout<<route_cost[i]<<' '<<checkR_cost[i]<<' ';
			wrong =true;
		}
		//if (checkR_cost[i] > DISTANCE)
		//	cout<<" Exceed DISTANCE"<<endl;
		//cout<<endl;
	}
	delete[] checkR_cost;
}

//DONE
void reoptimize_2_2swap(float *cost_of_removing, float **(&info_1_0), float **(&same_r_info), float **(&gain_matrix_1_0))
{

	int element1, element1f, element2, element2f, before_ele1, before_ele2, after_ele1=-1, after_ele2=-1;
	float cost_without_i, cost_without_j, cost_with_i, cost_with_i_Reverse, cost_with_j_Reverse, cost_with_both_Reverse, cost_with_j, gain, dist_gain;
	float dist_without_i, dist_without_j, dist_with_i, dist_with_j, dist_with_i_Reverse, dist_with_j_Reverse, dist_with_both_Reverse, gain_with_i_Reverse, gain_with_j_Reverse, gain_with_both_Reverse;

	int r;

	int r1 = route_change[0]; //route (from)
	int r2 = route_change[1]; //route (to)

	
	int temp = -1;
	int reverseType = 0;//initialize zero means no reverse for both
	///////////////////////////////=================================== SAME ROUTE =================================////////////////////////
	int iter=2; //added this on 1Nov15, previous change may only involve 1 route
	if (r1 == r2)
		iter=1;

	for (int I = 0; I < iter; I++)
	{
		if (I==0)
			r=r1;
		else
			r=r2;
		
		//================================  if in the same route, just swap customer  ====================================//
		//if ((saiz[r]==0)||(saiz[r]==1)) //if it is empty route or saiz=1, nothing to reshuffle
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
		
	//////////////////////////////////////// ====================== DIFFERENT ROUTES =========================================////////////////////////////
	int i = 0;
	//##########################################calculate GAIN when remove and insert simultaneously######################################################//
	//============================================(From) Affected routes: R1 and R2 to other routes (2-1)===========================================================
	for (int I = 0; I < iter; I++)  //consider each and other routes (from R1)
	{
		if (I == 0)
			i = r1;
		else
			i = r2;

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
	}//end for i 
}


//DONE
void two_optintra(float **(&same_r_info), float **(&gain_matrix_1_0)) 
{

	//bool *flag_routeChange = new bool[no_routes];//to calculate how many route have been changed at the end, return to original function how many route have been changed
	//for (int i = 0; i < no_routes; i++)
	//{
	//	flag_routeChange[i] = false; //initially, all routes have not been changed, flag false
	//}

	for (int i = 0; i < no_routes; i++)  //from route 0 to last route
	{
		
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
void reoptimize_two_optintra(float **(&same_r_info), float **(&gain_matrix_1_0)) 
{

	int r1 = route_change[0]; //route (from)
	int r2 = route_change[1]; //route (to)
	bool* routeChange = new bool [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		if (i==r1 || i==r2)
			routeChange[i] = true;
		else
			routeChange[i] = false; //initialize to false
	}

	for (int i = 0; i < no_routes; i++)  //from route 0 to last route
	{
		if (routeChange[i] == false)//if route has not been changed previously, just skip it
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
	delete[] routeChange;
}

//reverse the order, the size follows the other route, Eg:
//R1: D - 1 - 2 - 3 - D				(start point j=3)
//R2: D - 4 - 5 - 6 - 7 - 8 - D		(end point n=5)
//becomes
//R1: D - 1 - 2 - 5 - 4 - D
//R2: D - 3 - 6 - 7 - 8 - D
//Route 1: Take from j until the last
//Route 2: Take from the first until n
//must consider i=0 and m=0

//DONE
void two_optinter(float **(&info_1_0), float **(&gain_matrix_1_0)) //return number of route change for updating cost_of_removing in function call
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
					//R1: D 2 4 6 3 5 9 D
					//R2: D 8 1 0 7 10 12 11 D

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
						//continue;
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
}

//DONE
void reoptimize_two_optinter(float **(&info_1_0), float **(&gain_matrix_1_0)) //return number of route change for updating cost_of_removing in function call
{

	int r1 = route_change[0]; //route (from)
	int r2 = route_change[1]; //route (to)
	bool* routeChange = new bool [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		if (i==r1 || i==r2)
			routeChange[i] = true;
		else
			routeChange[i] = false; //initialize to false
	}

	std::vector<int> myvector2; //add depot at beginning and end, this is for swapping

	for (int i = 0; i < no_routes; i++)  //from route 0 to last route
	{
		if (saiz[i] == 0) 
			continue;
		
		for (int m = 0; m < no_routes; m++)//from route 0 to last route
		{
			if ((saiz[m] == 0) || (i==m))
				continue;
			
			if(routeChange[i] == false && routeChange[m] == false)//if both routes have not been changed previosuly, skip it
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
					//R1: D 2 4 6 3 5 9 D
					//R2: D 8 1 0 7 10 12 11 D
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
	delete[] routeChange;
}

//crossTail2 consider reverse order of tail
//Route 1: Take from j until the last
//Route 2: Take from n until the last
//must consider i=0,...,no_routes-1, m=i+1,...no_routes

//DONE
void crossTail2(float **(&info_1_0), float **(&gain_matrix_1_0)) //return number of route change for updating cost_of_removing in function call
{

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

			for (int j = 1; j <= saiz[i]; j++) //the start of tail1 //j++ means delete less in r1
			{
				int startTail1 = VEHICLE[i][j];
				int endTail1 = VEHICLE[i][saiz[i]];
				//tail1.clear();


				for (int n = 1; n <= saiz[m]; n++)//the start of tail2 //n++ means delete less in r2
				{
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
}


//DONE
void reoptimize_crossTail2(float **(&info_1_0), float **(&gain_matrix_1_0)) //return number of route change for updating cost_of_removing in function call
{

	int r1 = route_change[0]; //route (from)
	int r2 = route_change[1]; //route (to)
	bool* routeChange = new bool [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		if (i==r1 || i==r2)
			routeChange[i] = true;
		else
			routeChange[i] = false; //initialize to false
	}
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

			if(routeChange[i] == false && routeChange[m] == false)//if both routes have not been changed previosuly, skip it
				continue;

			for (int j = 1; j <= saiz[i]; j++) //the start of tail1 //j++ means delete less in r1
			{
				int startTail1 = VEHICLE[i][j];
				int endTail1 = VEHICLE[i][saiz[i]];
				//tail1.clear();


				for (int n = 1; n <= saiz[m]; n++)//the start of tail2 //n++ means delete less in r2
				{
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
						//continue;
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
					float dist_with2Rev=0;//for check condition;//r2 with tail1
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
					
					float dist_with1Rev=0;//for check condition;//r1 with tail2
		
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
						info_1_0[(int)gain_matrix_1_0[m][i]][10] = cost_of_removing1  - cost_of_inserting1;//gain1
						info_1_0[(int)gain_matrix_1_0[m][i]][11] = dist_without1 - dist_with1;//gain1 in distance
						info_1_0[(int)gain_matrix_1_0[m][i]][12] = dist_without2 - dist_with2;//gain2 in distance

					}//end if better gain found
				}//end of n
				increasej:;
			}//end of j

		}//end of for m
	}//end of i
	delete[] routeChange;
}

void CROSS(float **(&info_1_0), float **(&gain_matrix_1_0)) //return number of route change for updating cost_of_removing in function call
{

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
							info_1_0[(int)gain_matrix_1_0[m][i]][11] = old_cost1 - new_cost1;//gain1
							info_1_0[(int)gain_matrix_1_0[m][i]][12] = old_cost1 - new_cost1;//gain1

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
							info_1_0[(int)gain_matrix_1_0[i][m]][11] = old_cost1 - new_cost1;//gain1
							info_1_0[(int)gain_matrix_1_0[i][m]][12] = old_cost1 - new_cost1;//gain1

						}//end if
					}//end if (i>m)					
				}//end of n
			}//end of j
		}//end of for m
	}//end of i

}


void reoptimize_CROSS(float **(&info_1_0), float **(&gain_matrix_1_0)) //return number of route change for updating cost_of_removing in function call
{

	int r1 = route_change[0]; //route (from)
	int r2 = route_change[1]; //route (to)
	bool* routeChange = new bool [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		if (i==r1 || i==r2)
			routeChange[i] = true;
		else
			routeChange[i] = false; //initialize to false
	}
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

			if(routeChange[i] == false && routeChange[m] == false)//if both routes have not been changed previosuly, skip it
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
	delete[] routeChange;

}


//DONE
void insert_2optintra(float max_gain, float *(&cost_of_removing), float **(same_r_info), int **(&VEHICLE))
{
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);
	
	std::vector<int> subvector; //for reverse order
	int num_c_in_subroute=0;
	//float ori_cost=0.0, temp_cost=0.0;
	float gain = same_r_info[number][1];//gain
	int i = same_r_info[number][2];//route number
	int m = same_r_info[number][4]; //eleStart
	int n = same_r_info[number][5]; //eleEnd

	//see<<i<<' '<<m <<' '<< n<<' '<<gain<<endl;
	
	//copy the subroute in subvector in reverse order, noneed to reverse again later
	for (int z = n; z >= m; z--)
	{
		subvector.push_back(VEHICLE[i][z]);
		num_c_in_subroute++;	
	}

	int s=0;//for subvector
	for (unsigned z = m; z <= n; z++)
	{
		VEHICLE[i][z] = subvector[s];  //copy to VEHICLE 
		s++;
	}
	route_cost[i] = route_cost[i] - gain;  //overwrite the cost //check rounding error	
	distance_cost[i] = distance_cost[i] - same_r_info[number][11]; 
	//distance_cost[i]=0;
	//for (int c = 0; c < saiz[i] + 1; c++)
	//{
	//	distance_cost[i] += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]]+ service_time[VEHICLE[i][c]];
	//}
	float check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[i] + 1; c++)
		{
			check_cost+= dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		}
		if (abs(check_cost-distance_cost[i]) > 1.5)
		{
			cout<<"Cost diff in insert 2-opt intra"<<endl;
			cout<<check_cost<<' '<<distance_cost[i]<<endl;
			getchar();
		}
	//update CumDist starting from the affecting pos until last cust
	int start = m;//the start of update point
	for (int j = start; j <=saiz[i]; j++)
	{
		CumDist[i][j] = CumDist[i][j-1]+dist[VEHICLE[i][j-1]][VEHICLE[i][j]] + service_time[VEHICLE[i][j]];
		//CumuDCust[VEHICLE[i][j]]= CumDist[i][j];
	}

	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);
	//route_file << ' ' << endl;
	////======================================================Display route========================================================================
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		//see << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] + 1; h++)
		//{
		//	see << VEHICLE[g][h] << ' ';
		//}
		//see<<endl;

		
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
	}
	
	//see<<endl;
	//see<<"CUmDIst after 2-opt intra"<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{
	//	for (int j = 0; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
	//	{
	//		see<<CumDist[i][j]<<' ';
	//	}
	//	see<<endl;
	//}
	
	int f = i;
	
	
	updateCostofRemoving2 (f, cost_of_removing);

	cout<<"Insert 2-opt Intra"<<endl;

	

	float *checkR_cost = new float[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		
		checkR_cost[i]=0;
		for (int j = 1; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
		{
			checkR_cost[i] += CumDist[i][j];
		}	
	}
	bool wrong = false;
	for (int i = 0; i < no_routes; i++)
	{
		//cout<< "checkR_cost["<< i <<"]= " << checkR_cost[i];
		if (abs(route_cost[i]-checkR_cost[i]) > BIGC)
		{
			cout<<i<<"  This cost is different from original "<<endl;
			cout<<route_cost[i]<<' '<<checkR_cost[i]<<' ';
			wrong =true;
		}
		//if (checkR_cost[i] > DISTANCE)
		//	cout<<" Exceed DISTANCE"<<endl;
		//cout<<endl;
	}
	delete[] checkR_cost;
	//if (wrong==true)
	//	getchar();
	

}
//DONE
void insert_2optinter(float max_gain, float *(&cost_of_removing), float **info_1_0, int **(&VEHICLE))
{
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);
	
	std::vector<int> myvector1;
	std::vector<int> myvector2; 
	int i = info_1_0[number][2];//r1
	int m = info_1_0[number][3];//r2
	int j = info_1_0[number][4]; //eleStart
	int n = info_1_0[number][5]; //eleEnd
	float gain = info_1_0[number][1];//gain
	//see<<i<<' '<<m<<' '<<j<<' '<<n<<' '<<gain<<endl;
	
	

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
	
	int k = j; //start of route1
	for (int z = 0; z < myvector2.size(); z++)//insert segment2 to route1
	{
		VEHICLE[i][k] = myvector2[z];
		k++;
	}

	std::vector<int> tempR2; //to hold second part of route2 from [n+1] to saiz[m] because route 2 is to replace the front part until j, but the size maybe more than original because it is based on how many deleted from r1
	for (int k = n+1; k <= saiz[m]; k++)//push the rest starting from [n+1] until the end depot, saiz[] havent updated yet
	{
		tempR2.push_back(VEHICLE[m][k]);
	}

	saiz[i] = saiz[i] - r1Size + r2Size; //update size because the size has changed
	saiz[m] = saiz[m] - r2Size + r1Size;
	
	k = 1; //start of route2
	for (int z = 0; z < myvector1.size(); z++)//insert segment1 to route2
	{
		VEHICLE[m][k] = myvector1[z];
		k++;
	}
	//copy the remaining part of route2 (unchanged part)
	int t=0;//for tempR2
	for (int t = 0; t < tempR2.size(); t++)//copy the unchanged part to route2
	{
		VEHICLE[m][k] = tempR2[t];//k continue from previous one
		k++;
	}
	//put the depot as the last element
	VEHICLE[i][saiz[i]+1] = SIZE;
	VEHICLE[m][saiz[m]+1] = SIZE;//already put the depot hen copy from tempR2

	float new_sumcost = route_cost[i] + route_cost[m] - gain;
	distance_cost[i] = distance_cost[i] - info_1_0[number][11];
	//distance_cost[i] = distance_cost[m] = 0.0;//reinitialize
	//for (int c = 0; c <= saiz[i]; c++)
	//{
	//	distance_cost[i] += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
	//}
	float check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[i] + 1; c++)
		{
			check_cost+= dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		}
		if (abs(check_cost-distance_cost[i]) > 1.5)
		{
			cout<<"Cost diff in insert 2-opt inter i"<<endl;
			cout<<check_cost<<' '<<distance_cost[i]<<endl;
			getchar();
		}
	distance_cost[m] = distance_cost[m] - info_1_0[number][12];
	//for (int c = 0; c <= saiz[m]; c++)
	//{
	//	distance_cost[m] += dist[VEHICLE[m][c]][VEHICLE[m][c + 1]] + service_time[VEHICLE[m][c]];
	//}
	check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[m] + 1; c++)
		{
			check_cost+= dist[VEHICLE[m][c]][VEHICLE[m][c + 1]] + service_time[VEHICLE[m][c]];
		}
		if (abs(check_cost-distance_cost[m]) > 1.5)
		{
			cout<<"Cost diff in insert 2-opt inter m"<<endl;
			cout<<check_cost<<' '<<distance_cost[m]<<endl;
			getchar();
		}
	route_cost[i] = route_cost[i]-info_1_0[number][10];
	route_cost[m] = new_sumcost - route_cost[i]; //avoid calculate the second route cost, just use gain

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
		//CumuDCust[VEHICLE[i][q]]= CumDist[i][q];
	}

	//update CumDist starting from the affecting pos until last cust
	int start2 = 1;//the start of update point
	for (int q = start2; q <=saiz[m]; q++)
	{
		CumDist[m][q] = CumDist[m][q-1]+dist[VEHICLE[m][q-1]][VEHICLE[m][q]] + service_time[VEHICLE[m][q]];
		//CumuDCust[VEHICLE[m][q]]= CumDist[m][q];
	}
		
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		//see << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] + 1; h++)
		//{
		//	see << VEHICLE[g][h] << ' ';
		//}
		//see<<endl;

		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
	}
	
	//see<<endl;
	//	see<<"CUmDIst after 2-opt inter"<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{		
	//	for (int j = 0; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
	//	{
	//		see<<CumDist[i][j]<<' ';
	//	}
	//	see<<endl;
	//}

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

	}

	float *checkR_cost = new float[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		
		checkR_cost[i]=0;
		for (int j = 1; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
		{
			checkR_cost[i] += CumDist[i][j];
		}	
	}
	bool wrong = false;
	for (int i = 0; i < no_routes; i++)
	{
		//cout<< "checkR_cost["<< i <<"]= " << checkR_cost[i];
		if (abs(route_cost[i]-checkR_cost[i]) > BIGC)
		{
			cout<<i<<"  This cost is different from original "<<endl;
			cout<<route_cost[i]<<' '<<checkR_cost[i]<<' ';
			wrong =true;
		}
		//if (checkR_cost[i] > DISTANCE)
		//	cout<<" Exceed DISTANCE"<<endl;
		//cout<<endl;
	}
	delete[] checkR_cost;
	//if (wrong==true)
	//	getchar();

}

void insert_crosstail(float max_gain, float *(&cost_of_removing), float **info_1_0, int **(&VEHICLE))
{
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);
	
	std::vector<int> myvector1;
	std::vector<int> myvector2; 
	int i = info_1_0[number][2];//r1
	int m = info_1_0[number][3];//r2
	int j = info_1_0[number][4]; //eleStart in r1
	int n = info_1_0[number][5]; //eleStart in r2
	float gain = info_1_0[number][1];//gain
	int reverseType = info_1_0[number][9];//reverseType
	//see<<i<<' '<<m<<' '<<j<<' '<<n<<' '<<gain<<' '<<reverseType<<endl;


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
	
	int k = j; //start of route1
	for (int z = 0; z < myvector2.size(); z++)//insert segment2 to route1
	{
		VEHICLE[i][k] = myvector2[z];
		k++;
	}

	k = n; //start of route2
	for (int z = 0; z < myvector1.size(); z++)//insert segment1 to route2
	{
		VEHICLE[m][k] = myvector1[z];
		k++;
	}

	saiz[i] = saiz[i] - tail1Size + tail2Size; //update size because the size has changed
	saiz[m] = saiz[m] - tail2Size + tail1Size;
	
	//put the depot as the last element
	VEHICLE[i][saiz[i]+1] = SIZE;
	VEHICLE[m][saiz[m]+1] = SIZE;

	float new_sumcost = route_cost[i] + route_cost[m] - gain;
	distance_cost[i] = distance_cost[i] - info_1_0[number][11];
	//distance_cost[i] = distance_cost[m] = 0.0;//reinitialize
	//for (int c = 0; c <= saiz[i]; c++)
	//{
	//	distance_cost[i] += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
	//}
	float check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[i] + 1; c++)
		{
			check_cost+= dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
		}
		if (abs(check_cost-distance_cost[i]) > 1.5)
		{
			cout<<"Cost diff in insert crosstail i"<<endl;
			cout<<check_cost<<' '<<distance_cost[i]<<endl;
			getchar();
		}
	distance_cost[m] = distance_cost[m] - info_1_0[number][12];
	//for (int c = 0; c <= saiz[m]; c++)
	//{
	//	distance_cost[m] += dist[VEHICLE[m][c]][VEHICLE[m][c + 1]] + service_time[VEHICLE[m][c]];
	//}
	 check_cost= 0.0; //initialize
		for (int c = 0; c < saiz[m] + 1; c++)
		{
			check_cost+= dist[VEHICLE[m][c]][VEHICLE[m][c + 1]] + service_time[VEHICLE[m][c]];
		}
		if (abs(check_cost-distance_cost[m]) > 1.5)
		{
			cout<<"Cost diff in insert crosstail m"<<endl;
			cout<<check_cost<<' '<<distance_cost[m]<<endl;
			getchar();
		}
	route_cost[i] = route_cost[i]-info_1_0[number][10];
	route_cost[m] = new_sumcost - route_cost[i]; //avoid calculate the second route cost, just use gain

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
		//CumuDCust[VEHICLE[i][q]]= CumDist[i][q];
	}
	//update CumDist starting from the affecting pos until last cust
	int start2 = n;//the start of update point
	for (int q = start2; q <=saiz[m]; q++)
	{
		CumDist[m][q] = CumDist[m][q-1]+dist[VEHICLE[m][q-1]][VEHICLE[m][q]] + service_time[VEHICLE[m][q]];
		//CumuDCust[VEHICLE[m][q]]= CumDist[m][q];
	}
	
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		//see << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] + 1; h++)
		//{
		//	see << VEHICLE[g][h] << ' ';
		//}
		//see<<endl;

		if (route_cost[g] < 0)
		{
			cout<<"In insert cross-tail"<<endl;
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
	}
	
	//see<<endl;
	//	see<<"CUmDIst after CROSS-tail"<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{
	//	for (int j = 0; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
	//	{
	//		see<<CumDist[i][j]<<' ';
	//	}
	//	see<<endl;
	//}


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
		
		
		//for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
		//{
		//	cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
		//}
		cout<<"Insert crosstail"<<endl;
		//int status = comprareCost (cost_of_removing, f);//to check 
		//if (status == 1)
		//	cout<<info_1_0[number][4]<<' '<<' '<<info_1_0[number][5]<<endl;
	}

	
	
	float *checkR_cost = new float[no_routes];

	for (int i = 0; i < no_routes; i++)
	{
		
		checkR_cost[i]=0;
		for (int j = 1; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
		{
			checkR_cost[i] += CumDist[i][j];
		}	
	}
	bool wrong = false;
	for (int i = 0; i < no_routes; i++)
	{
		//cout<< "checkR_cost["<< i <<"]= " << checkR_cost[i];
		if (abs(route_cost[i]-checkR_cost[i]) > BIGC)
		{
			cout<<i<<"  This cost is different from original "<<endl;
			cout<<route_cost[i]<<' '<<checkR_cost[i]<<' ';
			wrong =true;
		}
		//if (checkR_cost[i] > DISTANCE)
		//	cout<<" Exceed DISTANCE"<<endl;
		//cout<<endl;
	}
	delete[] checkR_cost;
	//if (wrong==true)
	//	getchar();
	

}

void insert_cross(float max_gain, float *(&cost_of_removing), float **info_1_0, int **(&VEHICLE))
{
	
	std::vector<int> myvector1;
	std::vector<int> myvector2; 
	int i = info_1_0[number][2];//r1
	int m = info_1_0[number][3];//r2
	int j = info_1_0[number][4]; //eleStart in r1
	int n = info_1_0[number][5]; //eleStart in r2
	float gain = info_1_0[number][1];//gain
	int reverseType = info_1_0[number][9];//reverseType
	int L1 = 3;//length of segment1
	int L2 = 2;//length of segment2

	
	
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
	distance_cost[i] = distance_cost[i] - info_1_0[number][11];
	//distance_cost[i] = distance_cost[m] = 0.0;//reinitialize
	//for (int c = 0; c <= saiz[i]; c++)
	//{
	//	distance_cost[i] += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
	//}
	distance_cost[m] = distance_cost[m] - info_1_0[number][12];
	//for (int c = 0; c <= saiz[m]; c++)
	//{
	//	distance_cost[m] += dist[VEHICLE[m][c]][VEHICLE[m][c + 1]] + service_time[VEHICLE[m][c]];
	//}

	route_cost[i] = route_cost[i]-info_1_0[number][10];
	route_cost[m] = new_sumcost - route_cost[i]; //avoid calculate the second route cost, just use gain

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
		//CumuDCust[VEHICLE[i][q]]= CumDist[i][q];
	}
	//update CumDist starting from the affecting pos until last cust
	int start2 = n;//the start of update point
	for (int q = start2; q <=saiz[m]; q++)
	{
		CumDist[m][q] = CumDist[m][q-1]+dist[VEHICLE[m][q-1]][VEHICLE[m][q]] + service_time[VEHICLE[m][q]];
		//CumuDCust[VEHICLE[m][q]]= CumDist[m][q];
	}

	//float r_cost = 0.0;//to check
	//for (int c = 0; c <= saiz[m]; c++)
	//{
	//	r_cost += dist[VEHICLE[m][c]][VEHICLE[m][c + 1]] + service_time[VEHICLE[m][c]];
	//}

	//if (abs (r_cost-route_cost[m]) > 0.1)
	//{
	//	cout<<endl<<"In insert_cross!!!!!!!!!"<<' ';
	//	cout<<"route_cost["<<m<<"]= "<<route_cost[m]<<' '<<"r_cost= "<<r_cost<<endl;
	//	cout<<gain<<endl;
	//	getchar();
	//}
	//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out | ios::app);

	//route_file << ' ' << endl;
	////======================================================Display route========================================================================
	//route_file << "==================From insert cross============================== " << endl;
	//route_file << "Max_gain= " <<max_gain<< " r1= " <<info_1_0[number][2]<<" r2= " <<info_1_0[number][3]<<" r1_w/o= "<<info_1_0[number][4]<<" r2w= "<<info_1_0[number][5]<<" || r2w/o= "<<info_1_0[number][6]<<" r1w= "<<info_1_0[number][7]<<" reverse status= "<<info_1_0[number][9]<<endl;
	float total_cost=0;
	for (int g = 0; g < no_routes; g++)
	{
		if (saiz[g] == 0)
			route_cost[g]=0;
		
		//route_file << g <<' '<< route_cost[g] <<' '<< saiz[g] <<' '<< total_demand[g]<<' '<<' ';
		//for (int h = 0; h <= saiz[g] + 1; h++)
		//{
		//	route_file << VEHICLE[g][h] << ' ';
		//}
		//route_file<<endl;
		total_cost = total_cost + route_cost[g];
		
		if (route_cost[g] < 0)
		{
			cout<< "route_cost["<<g<<"]= " <<route_cost[g]<<" is negative!"<<"saiz[g]= "<<saiz[g]<<endl;
			getchar();
		}
		
	}
	//route_file << "Total cost = " << total_cost << endl;
	//route_file << "GLOBAL_BEST_COST = " << GLOBAL_BEST_COST << endl;
	//route_file << "========================================================================== " << endl;

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
		//	cout<<info_1_0[number][4]<<' '<<' '<<info_1_0[number][5]<<endl;
	}

}

//void updateCostofRemoving (int f, int fromP, int toP, float *(&cost_of_removing), bool *(&tempFlag)) //update cost of removing from fromP to toP, , bool *(&tempFlag) to avoid finding many times if it already finds, happen when customer delete and insert next to each other
//{
//	for (int h = fromP; h <= toP; h++) 
//	{
//		if (h <= 0 || h >= saiz[f]+1 || tempFlag[VEHICLE[f][h]]==false)//if it is depot, noneed to find 
//			continue;
//		cost_of_removing[VEHICLE[f][h]] = dist[VEHICLE[f][h]][VEHICLE[f][h - 1]] + dist[VEHICLE[f][h]][VEHICLE[f][h + 1]] + service_time[VEHICLE[f][h]] - dist[VEHICLE[f][h - 1]][VEHICLE[f][h + 1]];
//		tempFlag[VEHICLE[f][h]] = false;//false noneed to find, true need to find
//	
//	}
//	//to check 
//	//for (int i=0 ; i < no_routes; i++)
//	//{
//	//	bool stat = false;
//	//	for (int j=1 ; j <=saiz[i]; j++)
//	//	{
//	//		if (cost_of_removing[VEHICLE[i][j]] <= -500)
//	//		{
//	//			stat = true;
//	//			cout<<"["<<i<<"]["<<j<<"]= "<<cost_of_removing[VEHICLE[i][j]]<<endl;
//	//		}
//	//	}
//	//	if (stat == true)
//	//	{
//	//		cout<<"In updateCostofRemoving!!!!"<<endl;
//	//		getchar();
//	//	}
//	//}
//}

//*****************************************IMPORTANT*********************************************//
//cannot update from start because cost_of_removing for the entire route change when a customer is deleted from the route, because number_of_remaining customer changes
void updateCostofRemoving2 (int f, float *(&cost_of_removing)) //update cost of removing from start point to avoid finding many times if it already finds, happen when customer delete and insert next to each other
{
	int before = -1, after = -1;
	for (int h = 1; h <= saiz[f]; h++) 
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

int comprareCost (float *cost_of_removing, int f)
{
	int status=0;
	float *cost2 = new float [SIZE];
	int before = -1, after = -1;
	for (int h = 1; h <= saiz[f]; h++) //first customer start from element [1]
	{
		
		before = VEHICLE[f][h - 1];
		after = VEHICLE[f][h + 1];	
		float origain = dist[VEHICLE[f][h]][before] + dist[VEHICLE[f][h]][after] + service_time[VEHICLE[f][h]] - dist[before][after];
		int remainingcust = saiz[f]-h;
		if (remainingcust < 0)//if negative
				remainingcust = 0;
		cost2[VEHICLE[f][h]] = CumDist[f][h] + remainingcust*origain;
		
		
		if (abs(cost2[VEHICLE[f][h]] - cost_of_removing[VEHICLE[f][h]]) > 0.1)
		{
			cout<<"["<<f<<"]["<<h<<"]= "<<cost2[VEHICLE[f][h]]<<' '<<cost_of_removing[VEHICLE[f][h]]<<endl;
			status=1;
		}

	}
	if (status==1)
		getchar();

	delete[] cost2;
	return status;

}

