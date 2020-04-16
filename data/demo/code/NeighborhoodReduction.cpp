#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include "NeighborhoodReduction.h"

#include <algorithm>
//#include <iomanip>
//#include <ios>

using namespace std;

void neighborhood_reduction(float *x, float *y, int TOTAL_DEMAND)
{
	//===============================================NEIGHBOURHOOD REDUCTION (method 1)========================================================//
	int start_s = clock();
	ofstream timefile("16.Time_" + std::to_string( I ) + ".txt", ios::app);
	ofstream anglefile("21.N_R_ANGLE_" + std::to_string( I ) + ".txt");
	ofstream neighborhoodreduction("22.Neighbourhood Reduction_" + std::to_string( I ) + ".txt");
	ofstream depotreduction("22.Neighbourhood ReductionDEPOT_" + std::to_string( I ) + ".txt");
	ofstream route_file("route_" + std::to_string( I ) + ".txt"); //open for first time so that it deletes previous record

	for (int i = 0; i < SIZE + 1; i++)
	{
		for (int j = 0; j < SIZE + 1; j++)
		{
			NR_FLAG[i][j] = false;
			NR_FLAG_DEPOT[i][j] = false;

		}
	}

	//float dbar = 0;
	//float dhat = 0, dtilde = 0, std = 0;
	//float dist1 = 0.0, dist2 = 0.0;
	//const float alpha = 2; //how many alpha of standard deviation
	//const float beta = 0.5; //how many beta of xbar
	//float numerator = 0, sum_numerator = 0, numerator_sq = 0;

	//int track = 0;
	//for (int i = 0; i < SIZE; i++) //distance between all the customer
	//{
	//	//for (int j = 0; j < SIZE; j++)
	//	for (int j = i + 1; j < SIZE; j++) //avoid recalculate distance, for eg: distance [2][3] = dist[3][2]
	//	{
	//		dist1 = dist1 + dist[i][j];
	//		track++;
	//	}
	//}
	//cout << "dist1 = " << dist1 << endl;
	//cout << "track = " << track << endl;
	//neighborhoodreduction<< "dist1 = " << dist1 << endl;
	//neighborhoodreduction<< "track = " << track << endl;

	////dbar = dist / (SIZE * (SIZE-1));
	//dbar = dist1 / ((SIZE * (SIZE - 1)) / 2);

	//for (int j = 0; j < SIZE; j++)
	//{
	//	dist2 = dist2 + dist[SIZE][j]; //distance from depot

	//}
	//cout << "dist2 = " << dist2 << endl;
	//neighborhoodreduction<< "dist2 = " << dist2 << endl;
	//dtilde = dist2 / SIZE;
	//int tr=0;
	////compute standard deviation
	//for (int m = 0; m < SIZE; m++)
	//{
	//	for (int n = m+1; n < SIZE; n++)
	//	{
	//		numerator = (dist[m][n] - dbar) * (dist[m][n] - dbar);
	//		sum_numerator = sum_numerator + numerator;
	//		tr++;
	//	}
	//}

	//std = sqrt (sum_numerator / tr-1);
	//neighborhoodreduction << "std = " << std <<endl;
	//neighborhoodreduction << "tr = " << tr <<endl;
	//dhat = dbar - (alpha * std);
	//int track1=0;
	////###################################FIRST CONDITION: distance between customers (mean method)###################################//
	//for (int i = 0; i < SIZE; i++)
	//{
	//	for (int j = 0; j < SIZE; j++)
	//	{
	//		//if (dist[i][j] <= dbar) //FIRST CONDITION: distance between customers (mean method)
	//		if (dist[i][j] <= (beta*dbar) )//FIRST CONDITION: distance between customers (mean method)
	//		//if (dist[i][j] <= dhat) //FIRST CONDITION: distance between customers (mean method)
	//		{
	//			NR_FLAG_DEPOT[i][j] = true;
	//			NR_FLAG_DEPOT[j][i] = true;
	//			track1++;
	//		}

	//	}

	//	if (dist[SIZE][i] <= dtilde) //distance from depot
	//	{
	//		NR_FLAG_DEPOT[SIZE][i] = true;
	//		NR_FLAG_DEPOT[i][SIZE] = true;
	//		track1++;
	//	}
	//}
	//depotreduction << "FIRST CONDITION Nuum of flag = " << track1 <<endl;

	//int num_vehic_depotCust = (float)((float)TOTAL_DEMAND / (float)CAPACITY) + 1;
	//int num_of_cust_k = SIZE / num_vehic_depotCust;
	//float *kth_smallest_distance = new float[SIZE];

	//cout << "num_of_cust_k is " << num_of_cust_k << endl;
	//neighborhoodreduction << "num_of_cust_k is " << num_of_cust_k << endl;

	//float hold;
	//============================= Method of kth quartile (for NR_FLAG[][]) ====================================//
	//find the quantile for each 
	float *kthQuartile = new float[SIZE+1];

	vector<float> quantile;
	for (int i = 0; i <= SIZE; i++)
	{
		quantile.clear();
		for (int j = 0; j <= SIZE; j++)
		{
			quantile.push_back(dist[i][j]);
		}
		
		auto const Q1 = quantile.size() / 15;//3%=divide by 33, 5%=divide by 20, 25%=divide by 4, 10%=divide by 10, 6.67%=divide by 15
		auto const Q2 = quantile.size() / 2;
		auto const Q3 = Q1 + Q2;

		std::nth_element(quantile.begin(),          quantile.begin() + Q1, quantile.end());
		//neighborhoodreduction<< "The Q1 is " << quantile[Q1] << endl;
		kthQuartile[i] = quantile[Q1];
		//std::nth_element(quantile.begin() + Q1 + 1, quantile.begin() + Q2, quantile.end());
		//neighborhoodreduction << "The Q2 is " << quantile[Q2] << endl;
		//std::nth_element(quantile.begin() + Q2 + 1, quantile.begin() + Q3, quantile.end());
		//neighborhoodreduction << "The Q3 is " << quantile[Q3] << endl;

	}
	quantile.clear();
	quantile.shrink_to_fit();
	//between customer
	for (int i = 0; i <= SIZE; i++)
	{
		for (int j = 0; j <= SIZE; j++)
		{
			if (dist[i][j] <= kthQuartile[i])
			{
				NR_FLAG[i][j] = NR_FLAG[j][i] = true;
				NR_FLAG_DEPOT[i][j] = NR_FLAG_DEPOT[j][i] = true;
			}
		}
	}
    delete[] kthQuartile;
	//======================================Sort the distance ==============================================//

	//for (int j = 0; j < SIZE + 1; j++)
	//{
	//	for (int pass = 0; pass < SIZE; pass++)
	//	{
	//		for (int i = 0; i < SIZE; i++)
	//		{
	//			if (distance2[j][i] > distance2[j][i + 1])
	//			{
	//				hold = distance2[j][i];
	//				distance2[j][i] = distance2[j][i + 1];
	//				distance2[j][i + 1] = hold;
	//			}
	//		}
	//	}
	//}

	///*for (int i = 0; i < SIZE; i++)
	//{
	//for (int j = 0; j < SIZE; j++)
	//{
	//cout << "sorted distance[" << i << "]" << "[" << j << "]" << "=" << distance2[i][j] << endl;

	//}
	//cout<<endl;
	//}
	//cout<<endl;*/



	//for (int i = 0; i < SIZE; i++)
	//{
	//	for (int j = 0; j < SIZE; j++)
	//	{
	//		kth_smallest_distance[i] = distance2[i][num_of_cust_k];
	//	}
	//}


	//for (int i = 0; i < SIZE; ++i)
	//{
	//	delete[] distance2[i];
	//}
	//delete[] distance2;


	///*for (int i = 0; i < SIZE; i++)
	//{
	//cout << kth_smallest_distance[i] <<' ';
	//}

	//cout<<endl;*/

	//float max_dbar = 0.0;
	//float max_sum = 0.0;
	//float max;
	//max = kth_smallest_distance[0]; //method (ii):
	//for (int i = 0; i < SIZE; i++)
	//{
	//	//max_sum = max_sum + kth_smallest_distance[i]; //method (i):

	//	if (kth_smallest_distance[i] > max) //method (ii):
	//		max = kth_smallest_distance[i]; //method (ii):
	//}

	////max_dbar = max_sum/SIZE; //method (i): take the average of the maximum
	//max_dbar = max; //method (ii): take the maximum of the maximum

	//cout << "max_dbar= " << max_dbar << endl;

	//for (int i = 0; i < SIZE; i++)
	//{
	//	for (int j = 0; j < SIZE; j++)
	//	{
	//		if (dist[i][j] <= max_dbar) //FIRST CONDITION: distance between customers (maximum method)
	//		{
	//			NR_FLAG[i][j] = true;
	//			NR_FLAG[j][i] = true;

	//		}

	//	}
	//}

	//###################################SECOND CONDITION: Insert customer j between i and depot###################################//
	//float **delta_nod = new float*[SIZE];
	//int *cust_near_depot = new int[SIZE];

	//for (int i = 0; i < SIZE; ++i)
	//{
	//	delta_nod[i] = new float[SIZE];
	//}

	////Step 1: find average customer distance from depot
	//float sum_dist_fr_depot = 0.0, avg_dist_fr_depot = 0.0;

	//for (int i = 0; i < SIZE; i++)
	//{
	//	sum_dist_fr_depot = sum_dist_fr_depot + dist[SIZE][i];

	//}
	//avg_dist_fr_depot = sum_dist_fr_depot / SIZE;
	////cout << "Average distance from depot is " << avg_dist_fr_depot << endl;
	//depotreduction << "Average distance from depot is " << avg_dist_fr_depot << endl;
	//
	////Step 2: Determine the number of customer near to depot, N_0 whose (d < avg_dist_fr_depot)
	//int N_nod = 0;
	//for (int i = 0; i < SIZE; i++)
	//{
	//	if (dist[SIZE][i] < avg_dist_fr_depot)
	//	{
	//		cust_near_depot[N_nod] = i;
	//		N_nod++;
	//	}

	//}

	//depotreduction << "Number of customer near to depot, N_nod is " << N_nod << endl;

	////Step 3:For each customer i, find from set N_0 for the cost of insertion

	////the formula of delta_nod (insert j between i and depot) 
	//int track2=0;
	//for (int i = 0; i < SIZE; i++) //for each customer i, find among those j customer whose are near to depot to be inserted
	//{
	//	float sum_delta_nod = 0.0, avg_delta_nod = 0.0;
	//	for (int j = 0; j < N_nod; j++)
	//	{
	//		//delta_nod[i][j] = dist[i][j] + dist[SIZE][j] - dist[SIZE][i];
	//		delta_nod[i][cust_near_depot[j]] = dist[i][cust_near_depot[j]] + dist[SIZE][cust_near_depot[j]] - dist[SIZE][i]; //cost of insertion for the set of customers near to depot N_nod
	//		sum_delta_nod = sum_delta_nod + delta_nod[i][cust_near_depot[j]];
	//	}

	//	//Step 4: For each customer i, determine the average cost of insertion for customers near to depot
	//	avg_delta_nod = sum_delta_nod / N_nod;
	//
	//	for (int j = 0; j < N_nod; j++)
	//	{
	//		//Step 5: Use this condition rule, for each customer i and j∈N_0, if cost of insertion < average cost of insertion
	//		if (dist[i][cust_near_depot[j]] < avg_delta_nod)
	//		{
	//			NR_FLAG_DEPOT[i][cust_near_depot[j]] = true;
	//			NR_FLAG_DEPOT[cust_near_depot[j]][i] = true;
	//			track2++;
	//		}
	//	}
	//}

	//depotreduction << "SECOND CONDITION Nuum of flag = " << track2 <<endl;


	//for (int i = 0; i < SIZE; ++i)
	//{
	//	delete[] delta_nod[i];
	//}
	//delete[] delta_nod;
	//delete[] cust_near_depot;
	////compute standard deviation
	///*for (int m = 0; m < SIZE; m++)
	//{
	//for (int n = 0; n < SIZE; n++)
	//{
	//numerator = (dist[m][n] - dbar) * (dist[m][n] - dbar);
	//sum_numerator = sum_numerator + numerator;
	//}

	//}*/

	////###################################THIRD CONDITION: Angle (follow paper)###################################//

	////compute angle
	////=================================Find angle between depot and customers ===================================//

	//float dot = 0.0, pcross = 0.0;
	//float angl = 0.0;
	//float **angle = new float*[SIZE];

	//for (int i = 0; i < SIZE; ++i)
	//{
	//	angle[i] = new float[SIZE];
	//}

	//for (int m = 0; m < SIZE; m++)
	//{
	//	float BA_x = 0.0, BA_y = 0.0, CA_x = 0.0, CA_y = 0.0;
	//	BA_x = x[m] - x[SIZE];
	//	BA_y = y[m] - y[SIZE];

	//	for (int n = 0; n < SIZE; n++)
	//	{
	//		CA_x = x[n] - x[SIZE];
	//		CA_y = y[n] - y[SIZE];

	//		dot = BA_x * CA_x + BA_y * CA_y;
	//		pcross = BA_x * CA_y - BA_y * CA_x;
	//		angl = atan2(pcross, dot);
	//		angle[m][n] = angl;  //anti clockwise is positive angle, clockwise is negative angle
	//	}
	//}

	//for (int i = 0; i < SIZE; i++)
	//{
	//	for (int j = 0; j < SIZE; j++)
	//	{
	//		anglefile << "angle[" << i << "][" << j << "]=" << angle[i][j] << endl;
	//	}
	//	anglefile << endl;
	//}

	//int track3 = 0;

	//for (int i = 0; i < SIZE; i++)
	//{
	//	for (int j = 0; j < SIZE; j++)
	//	{
	//		if (angle[i][j] < 0) //if negative angle //added on 11 MAC 2015
	//			angle[i][j] = abs(angle[i][j]); //make it positive

	//		if (angle[i][j] <= alphamin) //&& (dist[i][j] <= dbar))//added distance on 12 April 2015
	//		//if (angle[i][j] <= alphamin)
	//		{
	//			NR_FLAG_DEPOT[i][j] = true;
	//			NR_FLAG_DEPOT[j][i] = true;
	//			track3++;
	//		}

	//		else ////////////////change this on 25Feb2015
	//		{
	//			if (angle[i][j] <= alphamax)////////////////change this on 25Feb2015
	//			{
	//				//dtilde = average distance from depot
	//				if (((dist[SIZE][i] <= dtilde) && (dist[SIZE][j] <= dtilde)) || ((dist[SIZE][i] <= (dist[SIZE][j]) / 2) || (dist[SIZE][j] <= (dist[SIZE][i]) / 2)))
	//				{
	//					NR_FLAG_DEPOT[i][j] = true;
	//					NR_FLAG_DEPOT[j][i] = true;
	//					track3++;
	//				}
	//			}
	//		}	
	//	}//end for j
	//}//end for i

	///*std = sqrt (sum_numerator / SIZE-1);
	//cout << "std = " << std <<endl;
	//dhat = dbar - (alpha * std);*/

	////for (int i = 0; i < SIZE; i++)//check if this one needed!!!!!!!!!!!!!!!!!! 50% customers will be flagged, omit this one reduce to 33%
	////{
	////	for (int j = 0; j < SIZE; j++)
	////	{
	////		if (dist[i][j] < dbar)  //first condition
	////		//if (dist[i][j] < dhat)  //first condition
	////		{
	////			NR_FLAG_DEPOT[i][j] = true;
	////		}

	////	}
	////}
	//depotreduction << "track3 = " << track3 << endl;
	//depotreduction << "dhat = " << dhat << endl;
	//depotreduction << "dbar = " << dbar << endl;
	//depotreduction << "dtilde = " << dtilde << endl;
	//========================= added this only for CCVRP =====================================//
	//last element to depot should be all true because last arc is not counted
	//but not [depot][customer]
	//here, [depot][cust] != [cust][depot]
	for (int i = 0; i < SIZE; i++)
	{
		NR_FLAG_DEPOT[i][SIZE] = true;
	}
	//========================= end of added this only for CCVRP =====================================//

	for (int i = 0; i < SIZE + 1; i++)
	{	
		NR_FLAG_DEPOT[i][i] = false; //make diagonal false because it has been overwritten just now
		NR_FLAG[i][i] = false;

	}
	//===============================================END OF NEIGHBOURHOOD REDUCTION =============================================================//
	float track_flag = 0;
	float percent = 0.0;
		
	for (int i = 0; i < SIZE+1; i++)
	{
		int sum=0;
		for (int j = 0; j < SIZE+1; j++)
		{
			//neighborhoodreduction<<NR_FLAG[i][j]<<' ';
			if (NR_FLAG[i][j] == true)
			{
				track_flag++;//total flag
				sum++;//flag for each row
			}
		}
		neighborhoodreduction<<' '<<sum<<endl;
	}
	percent = track_flag/((SIZE+1)*(SIZE+1));
	neighborhoodreduction << "track_flag = " << track_flag <<endl;
	neighborhoodreduction << "percent = " << percent <<endl;

	for (int i = 0; i < SIZE+1; i++)
	{
		for (int j = 0; j < SIZE+1; j++)
		{
			neighborhoodreduction<<"NR_FLAG["<<i<<"]["<<j<<"]="<< NR_FLAG[i][j]<<endl;
		}
		neighborhoodreduction<<endl;
	}

	track_flag = 0;
	percent = 0.0;
	
	for (int i = 0; i < SIZE+1; i++)
	{
		int sum=0;
		for (int j = 0; j < SIZE+1; j++)
		{
			//depotreduction<<NR_FLAG_DEPOT[i][j]<<' ';
			if (NR_FLAG_DEPOT[i][j] == true)
			{
				track_flag++;//total flag
				sum++;//flag for each row
			}
		}
		//depotreduction<<' '<<sum<<endl;
	}
	percent = track_flag/((SIZE+1)*(SIZE+1));
	depotreduction << "track_flag = " << track_flag <<endl;
	depotreduction << "percent = " << percent <<endl;
	//for (int i = 0; i < SIZE+1; i++)
	//{
	//	for (int j = 0; j < SIZE+1; j++)
	//	{
	//		depotreduction<<"NR_FLAG_DEPOT["<<i<<"]["<<j<<"]="<< NR_FLAG_DEPOT[i][j]<<endl;
	//	}
	//	depotreduction<<endl;
	//}
	int stop_s = clock();
	timefile << "Neighborhood reduction time: " << (stop_s - start_s) / float(CLOCKS_PER_SEC) << endl;
	timefile.close();



	//for (int i = 0; i < SIZE; ++i)
	//{
	//	delete[] angle[i];
	//}
	//delete[] angle;
	//delete[] kth_smallest_distance;

	route_file.close();
	anglefile.close();
	neighborhoodreduction.close();

}