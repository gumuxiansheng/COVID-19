#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <algorithm>    // std::random_shuffle
#include <cstdlib>      // std::rand, std::srand
#include "SectorConflict.h"
#include "LocalSearch.h"
#include "DiversificationLNS.h"
#include "Bestimprovement.h"// for inter_route_improvement();//after delete, improve route first, added on 15/06/2016, inter_route_improvement() is modified
extern const float PI= 3.141592653589793238463;
using namespace std;

float Diversification_conflict_sector(int **(&VEHICLE), int K, float *x, float *y)
{
	ofstream diversify_conflictSector("32.DiversifySector_" + std::to_string( I ) + ".txt", ios::app);
	diversify_conflictSector<<"K= "<<K<<endl;
	diversify_conflictSector<<"In diversify_conflictSector, before doing anything"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversify_conflictSector<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversify_conflictSector<<VEHICLE[i][j]<<' ';
		}
		diversify_conflictSector<<endl;
	}
	diversify_conflictSector<<"========================================================="<<endl;

	int num_cust_remove = K; //remove K number of customer
	//define sector size
	//float sectorSize = PI/6; //30degree
	float sectorSize = PI/12; //15degree
	int num_Sector = 2*PI / sectorSize;

	float *cutpoint = new float[num_Sector+1]; //0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360
	cutpoint[0] = 0; //first one is 0
	for (int i = 1; i <= num_Sector; i++)
	{
		cutpoint[i] = cutpoint[i-1] + sectorSize;
	}

	int **sectorRoute = new int*[num_Sector]; //to record number of customer in different sector different route, eg: sectorRoute[4][5] = 15 -----------> there are 15 customers in sector 4, route 5
	int *which_sector = new int [SIZE];//to record customer from which sector, eg: which_sector[5] = 4 -------> customer [5] from sector 4 
	int *which_route = new int [SIZE];//to record customer from which route , eg: which_sector[5] = 3 -------> customer [5] from route 3 

	for (int i = 0; i < num_Sector; i++)
	{
		sectorRoute[i] = new int[no_routes];
	}
	for (int i = 0; i < num_Sector; i++)
	{
		for (int j = 0; j < no_routes; j++)
		{
			sectorRoute[i][j] = 0;
		}
	}
	
	//go through each route, each customer, categorize customer in the sectorRoute[][]
	int sector=-1; //sector starts from [0], [1],...
	int route=-1;
	bool found = false;
	for (int i = 0; i < no_routes; i++)
	{
		for (int j = 1; j <= saiz[i]; j++)
		{
			int cust = VEHICLE[i][j];
			found = false; //reinitialize
			for (int k = 0; k < num_Sector; k++) //0...12 -> 13 values
			{
				if ((theta[cust]>=cutpoint[k]) && (theta[cust] < cutpoint[k+1]))
				{
					sector = k;
					route = i;
					sectorRoute[sector][route]++;//frequency +1
					which_sector[cust] = sector;
					which_route[cust] = route;
					found=true;
				}
				if (found == true)
					goto next_cust;
			}
			next_cust:;
		}
	}

	int *freq_Sector = new int [num_Sector]; //record the frequency of sector
	int *freq_Sector2 = new int [num_Sector]; //record the frequency of sector in ascending order
	int *sorted_sectorIndex = new int[num_Sector]; //hold the index of sorted sector

	for (int i = 0; i < num_Sector; i++)
	{
		freq_Sector[i] = freq_Sector2[i] = 0;//initialize
		sorted_sectorIndex[i] = i; //initilize
	}
	for (int i = 0; i < num_Sector; i++)
	{
		for (int j = 0; j < no_routes; j++)
		{
			if(sectorRoute[i][j] > 0)
				freq_Sector[i]++;
		}
		
	}
	int highest_freq = INT_MIN;
	for (int i = 0; i < num_Sector; i++)
	{
		freq_Sector2[i] = freq_Sector[i]; //copy to freq_Sector2[i] which will be sorted whereas freq_Sector[i] remains the same
		//find the highest frequency to create the matrix recording how many sector under each frequency
		if (freq_Sector[i] > highest_freq)
			highest_freq = freq_Sector[i];

	}

	diversify_conflictSector<<"column -> route "<<endl;
	diversify_conflictSector<<"row , (going down) is sector "<<endl;
	for (int i = 0; i < num_Sector; i++)
	{
		for (int j = 0; j < no_routes; j++)
		{
			diversify_conflictSector<<sectorRoute[i][j] <<' ';
		}
		diversify_conflictSector<< ' '<<' '<<' '<<freq_Sector[i]<<endl;
	}
	int hold=0, keep=0;
	//======================================Sort the sector according to freq==============================================//
	for (int i = 0; i < num_Sector - 1; i++)
	{
		for (int j = 0; j < num_Sector - 1; j++)
		{
			if ((freq_Sector2[j] >= 0) && (freq_Sector2[j] < freq_Sector2[j + 1]))
			{
				hold = freq_Sector2[j];
				keep = sorted_sectorIndex[j]; //hold the index of the sorted sector based on frequency

				freq_Sector2[j] = freq_Sector2[j + 1];
				sorted_sectorIndex[j] = sorted_sectorIndex[j + 1];

				freq_Sector2[j + 1] = hold;
				sorted_sectorIndex[j + 1] = keep;
			}
		}
	}
	int *freq_sec = new int[highest_freq+1]; //to record how many sector fall under each frequency
	for (int i = 0; i <= highest_freq; i++)
	{
		freq_sec[i] = 0;
	}
	//check if more than 1 sector has the same frequency, randomize the sequence so that not the same sector to be selected every time
	int curr_freq = highest_freq;//hold the highest freq because freq_Sector2[] has the highest freq on top
	for (int i = 0; i < num_Sector; i++)
	{
		if (freq_Sector2[i] == curr_freq)
		{
			freq_sec[curr_freq]++;
		}

		else
		{
			curr_freq = freq_Sector2[i];
			//curr_freq--;//not necessary next frequency because next maybe there is no sector under this freq, eg, 2 sectors have 4 freq, 0 sector have 3 freq
			freq_sec[curr_freq]++;
		}

	}


	for (int i = 0; i < num_Sector; i++)
	{
		diversify_conflictSector << "sector= "<<sorted_sectorIndex[i] << ' ' << ' ' << ' '<<"freq= "<<freq_Sector2[i]<<endl;
	}
	
	for (int i = 0; i <= highest_freq; i++)
	{
		diversify_conflictSector << freq_sec[i] <<" sector fall under frequency "<<i<<endl;
	}

	int *cumuFreq = new int[highest_freq+1];
	for (int i = 0; i <= highest_freq; i++)
	{
		cumuFreq[i] = 0;//initialize
	}
	int p = highest_freq;
	for (int i = 1; i <= highest_freq; i++)
	{
		cumuFreq[i] = cumuFreq[i-1] + freq_sec[p];
		p--;
	}
	//============== Randomize the sequence =========================//
	std::srand ( unsigned ( std::time(0) ) );
	std::vector<int> myvector;
	
	int m=0;//for cumuFreq[] which starts from 0
	for (int i = highest_freq; i >= 0; i--)
	{
		myvector.clear();
		for (int j = cumuFreq[m]; j < (cumuFreq[m]+freq_sec[i]); j++)//next one not from 0, from middle !!!!!!!!!!!!!!!!!!!!!!!!!!
		{
			myvector.push_back(sorted_sectorIndex[j]);
		}
		std::random_shuffle ( myvector.begin(), myvector.end() );
		for (int k = 0; k < freq_sec[i]; k++)
		{
			//copy to sorted_sectorIndex[j]
			diversify_conflictSector << myvector[k]<<' ';
		}
		diversify_conflictSector <<endl;
		//copy to sorted_sectorIndex[j]
		int q=0;
		for (int j = cumuFreq[m]; j < (cumuFreq[m]+freq_sec[i]); j++)
		{
			sorted_sectorIndex[j] = myvector[q];
			q++;
		}
		
		m++;
	}
	myvector.clear();
	myvector.shrink_to_fit();

	diversify_conflictSector << "After randomize "<<endl;
	for (int i = 0; i < num_Sector; i++)
	{
		diversify_conflictSector << "sector= "<<sorted_sectorIndex[i] << ' ' << ' ' << ' '<<"freq= "<<freq_Sector2[i]<<endl;
	}
	
	for (int i = 0; i <= highest_freq; i++)
	{
		diversify_conflictSector << freq_sec[i] <<" sector fall under frequency "<<i<<endl;
	}
	
	//bool *thisRchange= new bool[no_routes];//only do the rest if this route has changed (meaning there is customer removed from this route)//added 6May2016
	//for (int i = 0; i < no_routes; i++)
	//{
	//	thisRchange[i]= false;
	//}	
	int *custRemoved = new int[SIZE];//to record customer removed
	int c=0;//record how many customer removed

	//************added 7May2016**********************//
	bool *flag_delete = new bool[SIZE];//to flag which customer to delete
	for (int i = 0; i < SIZE; i++)
	{
		flag_delete[i] = false; 
	}
	int n=0;//while flagged cust less than Kappa
	int a=0;//to keep track sector index, start from 0
	while(n<K)
	{
		int maxFreqsector = sorted_sectorIndex[a];//search from the sector 0 (highest frequency sector), see which customer is from this sector
		for (int i = 0; i < no_routes; i++)
		{
			for (int j = 1; j <= saiz[i]; j++)
			{
				if(which_sector[VEHICLE[i][j]] == maxFreqsector) //if the customer is from this sector
				{
					flag_delete[VEHICLE[i][j]] = true;
					n++;
				}
				if (n>=K)
					goto enoughdelete;
			}
		}
		a++;//if Kappa is not reached, go to next highest sector
	}
	enoughdelete:
	//=============== to delete customer ==================//
	int **temp_r = new int* [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later
	for (int i = 0; i < SIZE; i++)
	{
		temp_r[i] = new int [SIZE];
	}
	for (int r = 0; r < no_routes; r++)
	{
		if (saiz[r] == 0) //if empty route, skip
			continue;
		bool thisRchange=false;
		// Delete customers from the route based on flagged
		temp_r[r][0] = SIZE; //first one is depot
		int t=1;//for temp_r[][] use
		for (int j = 1; j <= saiz[r]; j++)//make sure starts from 1 to saiz[i] because we dont want to delete the first and last one as depot
		{
			int cust = -1;
			if (flag_delete[VEHICLE[r][j]] == false)//if position is not flagged, copy the customer to temp_r[][]
			{
				temp_r[r][t] = VEHICLE[r][j];
				t++;
			}
			else if(flag_delete[VEHICLE[r][j]] == true)
			{
				//if position is flagged, dont copy to temp_r[][]
				cust = VEHICLE[r][j];
				custRemoved[c] = cust;
				c++;
				
				total_demand[r] = total_demand[r] - demand[cust]; //update total_demand for temp solution
				thisRchange=true;
				//delete_one_cust ( r, j, vehicle);//cannot use this because after delete, the position changed, previously at [1] becomes [0]
			}
		}
		saiz[r] = t-1;//update size of route //must update outside the for loop because for loop use size[] as a termination criteria, dont update inside
		temp_r[r][t] = SIZE; //last one is depot
		if (thisRchange==true)//only do the rest if this route has changed (meaning there is customer removed from this route)
		{
			//compute distance
			route_cost[r] = 0;//initialize
			distance_cost[r] = 0;//initialize
			CumDist[r][0]= 0;
		
			for (int j = 0; j <= saiz[r]+1; j++) //check if the saiz has been updated in delete()!!!!!!!!!!!!!!!!!!!!!!!!!
			{
				VEHICLE[r][j] = temp_r[r][j];
				if(j==0)
					continue;
				CumDist[r][j]=CumDist[r][j-1]+dist[temp_r[r][j-1]][temp_r[r][j]] + service_time[temp_r[r][j]];
				
			}
			for (int j = 0; j <= saiz[r]; j++)
			{
				distance_cost[r] = distance_cost[r]+dist[temp_r[r][j]][temp_r[r][j+1]] + service_time[temp_r[r][j]];
				route_cost[r] += CumDist[r][j];
			}

			space_available[r] = CAPACITY - total_demand[r] ; //update space_available for temp solution
			distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp solution

			//if ((saiz[r] != 0) || (saiz[r] != 1) )//skip if saiz=0 or saiz=1
			if (saiz[r]>=3)//changed on 28JUn2015
			{
				two_opt_singleR(VEHICLE, r); 
				//or_opt_singleR2(VEHICLE, r); 
			}
		}//end of if (thisRchange==true)//only do the rest if this route has changed (meaning there is customer removed from this route)
		
	}
	//inter_route_improvementforLNS();//after delete, improve route first, added on 15/06/2016, inter_route_improvement() is modified

	//**********end of added 7May2016 *****************//

//	for (int a = 0; a < K; a++) //K is how many sector to remove, passed from function call
//	{
//		int maxFreqsector = sorted_sectorIndex[a];
//		if (freq_Sector2[a] == 0) //if the frequency of this sector is 0, means no overalap route in this sector, so dont remove the good sector
//			goto finish_remove;
//		for (int i = 0; i < SIZE; i++) //run all customer to find which customer from this sector
//		{
//			if (which_sector[i] == maxFreqsector) //if the customer is from this sector
//			{
//				int route = which_route[i];
//				int position = -1;
//				for (int j = 1; j <= saiz[route]; j++) //this is to find the position of the customer in it's route
//				{
//					if (VEHICLE[route][j] == i)//i is the customer
//					{
//						position = j;
//						delete_one_cust (route, position, VEHICLE);
//						custRemoved[c] = i;
//						c++;
//						thisRchange[route]= true;
//					}
//					if (c > num_cust_remove)
//						goto finish_remove;
//				}
//			}
//		}
//	}
//finish_remove:
//
//	//improve routes //added 29Jun2015
//	for (int i = 0; i < no_routes; i++)
//	{
//		if (saiz[i]>=3 && thisRchange[i]==true)//only do 2-opt if this route has  been changed
//		{
//			two_opt_singleR(VEHICLE, i); 
//			or_opt_singleR2(VEHICLE, i); 
//		}
//	}


	diversify_conflictSector<<"======================================================================== "<<endl;
	diversify_conflictSector<<"custRemoved[i] are "<<endl;
	for (int i = 0; i < c; i++)
	{
		//cout<<custRemoved[i]<<' '<<' ';
		diversify_conflictSector<<custRemoved[i]<<' '<<' ';
	}
	//cout<<endl;
	diversify_conflictSector<<endl;
	diversify_conflictSector<<"number of cust removed= "<<c<<endl;

	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
	{
		cout<<"Perform greedyInsertion2"<<endl;
		diversify_conflictSector<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		cout<<"Perform regretInsertion"<<endl;
		diversify_conflictSector<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}

	int total_size=0;
	float t_cost=0.0;
	for (int i = 0; i < no_routes; i++)
	{
		total_size += saiz[i];
		t_cost += route_cost[i];
		diversify_conflictSector<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversify_conflictSector<<VEHICLE[i][j]<<' ';
		}
		diversify_conflictSector<<endl;
	}
	diversify_conflictSector<<"t_cost= "<<t_cost<<endl;

	//no need to copy to LOCAL_BEST_ROUTE because it will be copied in the original function

	if (total_size != SIZE)
	{
		cout<<"total_size is wrong in sector conflict diversification, cant continue"<<total_size<<endl;
		getchar();
	}

	//delete[] thisRchange;
	delete[] custRemoved;
	delete[] cutpoint; 

	for (int i = 0; i < num_Sector; i++)
	{
		delete[] sectorRoute[i];
	}
	delete[] sectorRoute;
	delete[] which_sector;//to record customer from which sector
	delete[] which_route;//to record customer from which route 
	delete[] freq_Sector; //record the frequency of sector
	delete[] freq_Sector2; //record the frequency of sector in ascending order
	delete[] sorted_sectorIndex; //hold the index of sorted sector
	delete[] freq_sec; //to record how many sector fall under each frequency
	delete[] cumuFreq;
	for (int i = 0; i < SIZE; i++)
	{
		delete[] temp_r[i];
	}
	delete[] temp_r;
	return t_cost;
}


