#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <string>
#include <vector>
#include <cstdlib>      // std::rand, std::srand
#include "Sort_solution.h"
#include <numeric>      // std::iota
#include <algorithm>    // std::sort


template <typename Container>
struct compare_indirect_indexS //sort from smallest to largest
  {
  const Container& container;
  compare_indirect_indexS( const Container& container ): container( container ) { }
  bool operator () ( size_t lindex, size_t rindex ) const
    {
		//return container[ lindex ] > container[ rindex ]; //in decreasing order
		return container[ lindex ] < container[ rindex ]; //in increasing order
    }
  };
template <typename Container>
struct compare_indirect_indexB //sort from smallest to largest
  {
  const Container& container;
  compare_indirect_indexB( const Container& container ): container( container ) { }
  bool operator () ( size_t lindex, size_t rindex ) const
    {
		return container[ lindex ] > container[ rindex ]; //in decreasing order
		//return container[ lindex ] < container[ rindex ]; //in increasing order
    }
  };
using namespace std;

void sort_GLOBALsolution (int **(&GLOBAL)) //big to small
{
	int num_routes = GLOBAL_NO_ROUTE;
	int *sai = new int[num_routes];
	int *capa = new int[num_routes];
	float  *cost = new float[num_routes];
	float *dist_cost = new float[no_routes];
	int**vec = new int* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		vec[i] = new int [SIZE];
	}

	float** cumD = new float* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		cumD[i] = new float [SIZE];
	}
	
	vector <float> data;
	for (int i = 0; i < num_routes; i++)//must use GLOBAL_NO_ROUTE
	{
		for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
		{
			vec[i][j] = GLOBAL[i][j];
			cumD[i][j] = GLOBALCumDist[i][j];
		}
	}
	for (int i = 0; i < num_routes; i++)
	{
		sai[i] = GLOBAL_SAIZ[i];
		capa[i] = GLOBAL_capa[i];
		cost[i] = GLOBAL_Rcost[i];
		dist_cost[i] = GLOBAL_distance_cost[i];
		data.push_back (GLOBAL_Rcost[i]); //data[] holds the cost
	}

	//sort in descending order, route with highest cost to lowest
	vector <size_t> indices( data.size(), 0 ); 
	iota( indices.begin(), indices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

	sort( indices.begin(), indices.end() , compare_indirect_indexB <decltype(data)> ( data ) ); //sort the data and index in decreasing order


	//cout << "Sorted route cost (from highest to lowest) "<<endl;
	//for (int i = 0; i < num_routes; i++)
	//{
	//	cout << "route= "<<indices[i] << ' ' << ' ' << ' '<<"cost= "<<data[indices[i]]<<endl;
	//}

	//if cost is the same, sort based on number of customer
	sortagain:
	for (int i = 0; i < num_routes-1; i++)
	{
		if (GLOBAL_Rcost[indices[i]] == GLOBAL_Rcost[indices[i+1]]) //if cost is the same
		{
			if (GLOBAL_SAIZ[indices[i]] == GLOBAL_SAIZ[indices[i+1]]) //if size is the same
			{
				if (GLOBAL[indices[i]][1] < GLOBAL[indices[i+1]][1]) //compare first customer
				{
					int temp = indices[i];
					indices [i] = indices [i+1];
					indices [i+1] = temp;
					goto sortagain;
				}
			}
			else if(GLOBAL_SAIZ[indices[i]] < GLOBAL_SAIZ[indices[i+1]])
			{
				int temp = indices[i];
				indices [i] = indices [i+1];
				indices [i+1] = temp;
				goto sortagain;
			}
		}
	}

	int r = 0; //for GLOBAL_NO_ROUTE becuase do not consider empty route	
	for (int i = 0; i < num_routes; i++)
	{
		if (GLOBAL_SAIZ[indices[i]] == 0)
			continue;
		GLOBAL_SAIZ[r] = sai[indices[i]]; //update GLOBAL_SAIZ[]

		for (int j = 0; j <= GLOBAL_SAIZ[r]+1; j++)
		{
			GLOBAL[r][j] = vec[indices[i]][j];
			GLOBALCumDist[r][j] = cumD[indices[i]][j];	
		}	
		GLOBAL_capa[r] = capa[indices[i]];
		GLOBAL_Rcost[r] = cost[indices[i]];
		GLOBAL_distance_cost[r] = dist_cost[indices[i]];
		r++;
	}
	GLOBAL_NO_ROUTE = r;

	delete[] sai;
	delete[] capa;
	delete[] cost;
	delete[] dist_cost;
	for (int i = 0; i < no_routes; i++)
	{
		delete[] vec[i];
		delete[] cumD[i];
	}
	delete[] vec;
	delete[] cumD;

}


void sort_LOCALsolution (int **(&LOCAL))//big to small
{
	int num_routes = LOCAL_NO_ROUTE;
	int *sai = new int[num_routes];
	int *capa = new int[num_routes];
	float  *cost = new float[num_routes];
	float *dist_cost = new float[no_routes];
	int**vec = new int* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		vec[i] = new int [SIZE];
	}

	float** cumD = new float* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		cumD[i] = new float [SIZE];
	}
	vector <float> data;

	for (int i = 0; i < num_routes; i++)//must use LOCAL_NO_ROUTE
	{
		for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
		{
			vec[i][j] = LOCAL[i][j];
			cumD[i][j] = LOCALCumDist[i][j];
		}
	}
	
	for (int i = 0; i < num_routes; i++)
	{
		sai[i] = LOCAL_SAIZ[i];
		capa[i] = LOCAL_capa[i];
		cost[i] = LOCAL_Rcost[i];
		dist_cost[i] = LOCAL_distance_cost[i];
		data.push_back (LOCAL_Rcost[i]); //data[] holds the cost

	}

	//sort in descending order, route with highest cost to lowest
	vector <size_t> indices( data.size(), 0 ); 
	iota( indices.begin(), indices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

	sort( indices.begin(), indices.end() , compare_indirect_indexB <decltype(data)> ( data ) ); //sort the data and index in decreasing order


	//cout << "Sorted route cost (from highest to lowest) "<<endl;
	//for (int i = 0; i < num_routes; i++)
	//{
	//	cout << "route= "<<indices[i] << ' ' << ' ' << ' '<<"cost= "<<data[indices[i]]<<endl;
	//}

	//if cost is the same, sort based on number of customer
	sortagain:
	for (int i = 0; i < num_routes-1; i++)
	{
		if (LOCAL_Rcost[indices[i]] == LOCAL_Rcost[indices[i+1]]) //if cost is the same
		{
			if (LOCAL_SAIZ[indices[i]] == LOCAL_SAIZ[indices[i+1]]) //if size is the same
			{
				if (LOCAL[indices[i]][1] < LOCAL[indices[i+1]][1]) //compare first customer
				{
					int temp = indices[i];
					indices [i] = indices [i+1];
					indices [i+1] = temp;
					goto sortagain;
				}
			}
			else if(LOCAL_SAIZ[indices[i]] < LOCAL_SAIZ[indices[i+1]])
			{
				int temp = indices[i];
				indices [i] = indices [i+1];
				indices [i+1] = temp;
				goto sortagain;
			}
		}
	}

	//do not delete empty route because the index will be different if delete empty
	for (int i = 0; i < num_routes; i++)
	{
		LOCAL_SAIZ[i] = sai[indices[i]]; //update LOCAL_SAIZ[] based on saiz[] //different from GLOBAL
		
		for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
		{
			LOCAL[i][j] = vec[indices[i]][j];
			LOCALCumDist[i][j] = cumD[indices[i]][j];
			
		}
		LOCAL_capa[i] = capa[indices[i]];
		LOCAL_Rcost[i] = cost[indices[i]];
		LOCAL_distance_cost[i] = dist_cost[indices[i]];
	}
	
	delete[] sai;
	delete[] capa;
	delete[] cost;
	delete[] dist_cost;
	for (int i = 0; i < no_routes; i++)
	{
		delete[] vec[i];
		delete[] cumD[i];
	}
	delete[] vec;
	delete[] cumD;
}


void sort_solution (int **(&VEHICLE))//big to small
{
	int *sai = new int[no_routes];
	int *capa = new int[no_routes];
	float *cost = new float[no_routes];
	float *dist_cost = new float[no_routes];
	int**vec = new int* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		vec[i] = new int [SIZE];
	}

	float** cumD = new float* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		cumD[i] = new float [SIZE];
	}

	vector <float> data;
	
	for (int i = 0; i < no_routes; i++)
	{
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			vec[i][j] = VEHICLE[i][j];
			cumD[i][j] = CumDist[i][j];
		}
	}
	for (int i = 0; i < no_routes; i++)
	{
		sai[i] = saiz[i];
		capa[i] = total_demand[i];
		cost[i] = route_cost[i];
		dist_cost[i] = distance_cost[i];
		data.push_back (route_cost[i]); //data[] holds the cost

	}

	//sort in descending order, route with highest cost to lowest
	vector <size_t> indices( data.size(), 0 ); 
	iota( indices.begin(), indices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

	sort( indices.begin(), indices.end() , compare_indirect_indexB <decltype(data)> ( data ) ); //sort the data and index in decreasing order


	//cout << "Sorted route cost (from highest to lowest) "<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{
	//	cout << "route= "<<indices[i] << ' ' << ' ' << ' '<<"cost= "<<data[indices[i]]<<endl;
	//}

	//if cost is the same, sort based on number of customer
	sortagain:
	for (int i = 0; i < no_routes-1; i++)
	{
		if (route_cost[indices[i]] == route_cost[indices[i+1]]) //if cost is the same
		{
			if (saiz[indices[i]] == saiz[indices[i+1]]) //if size is the same
			{
				if (VEHICLE[indices[i]][1] < VEHICLE[indices[i+1]][1]) //compare first customer
				{
					int temp = indices[i];
					indices [i] = indices [i+1];
					indices [i+1] = temp;
					goto sortagain;
				}
			}
			else if(saiz[indices[i]] < saiz[indices[i+1]])
			{
				int temp = indices[i];
				indices [i] = indices [i+1];
				indices [i+1] = temp;
				goto sortagain;
			}
		}
	}

	//do not delete empty route because the index will be different if delete empty
	for (int i = 0; i < no_routes; i++)
	{
		saiz[i] = sai[indices[i]]; //update LOCAL_SAIZ[] based on saiz[] //different from GLOBAL
		
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			VEHICLE[i][j] = vec[indices[i]][j];	
			CumDist[i][j] = cumD[indices[i]][j];	
		}
		total_demand[i] = capa[indices[i]];
		route_cost[i] = cost[indices[i]];
		distance_cost[i] = dist_cost[indices[i]];
		space_available[i] = CAPACITY - total_demand[i];
		distance_available[i] = DISTANCE - distance_cost[i];
	}
	
	cout<<"after sort vEHICLE"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			cout<<VEHICLE[i][j]<<' ';
		}
		cout<<endl;
	}

	delete[] sai;
	delete[] capa;
	delete[] cost;
	delete[] dist_cost;
	for (int i = 0; i < no_routes; i++)
	{
		delete[] vec[i];
		delete[] cumD[i];
	}
	delete[] vec;
	delete[] cumD;
}



void sort_GLOBALsolutionSMALLEST (int **(&GLOBAL))//small to big
{
	int num_routes = GLOBAL_NO_ROUTE;
	int *sai = new int[num_routes];
	int *capa = new int[num_routes];
	float  *cost = new float[num_routes];
	float *dist_cost = new float[no_routes];
	int**vec = new int* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		vec[i] = new int [SIZE];
	}

	float** cumD = new float* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		cumD[i] = new float [SIZE];
	}
	
	vector <float> data;
	for (int i = 0; i < num_routes; i++)//must use GLOBAL_NO_ROUTE
	{
		for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
		{
			vec[i][j] = GLOBAL[i][j];
			cumD[i][j] = GLOBALCumDist[i][j];
		}
	}
	for (int i = 0; i < num_routes; i++)
	{
		sai[i] = GLOBAL_SAIZ[i];
		capa[i] = GLOBAL_capa[i];
		cost[i] = GLOBAL_Rcost[i];
		dist_cost[i] = GLOBAL_distance_cost[i];
		data.push_back (GLOBAL_Rcost[i]); //data[] holds the cost
	}

	//sort in ascending order, route with lowest cost to highest
	vector <size_t> indices( data.size(), 0 ); 
	iota( indices.begin(), indices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

	sort( indices.begin(), indices.end() , compare_indirect_indexS <decltype(data)> ( data ) ); //sort the data and index in increasing order


	//cout << "Sorted route cost (from lowest to highest) "<<endl;
	//for (int i = 0; i < num_routes; i++)
	//{
	//	cout << "route= "<<indices[i] << ' ' << ' ' << ' '<<"cost= "<<data[indices[i]]<<endl;
	//}

	//if cost is the same, sort based on number of customer
	sortagain:
	for (int i = 0; i < num_routes-1; i++)
	{
		if (GLOBAL_Rcost[indices[i]] == GLOBAL_Rcost[indices[i+1]]) //if cost is the same
		{
			if (GLOBAL_SAIZ[indices[i]] == GLOBAL_SAIZ[indices[i+1]]) //if size is the same
			{
				if (GLOBAL[indices[i]][1] > GLOBAL[indices[i+1]][1]) //compare first customer
				{
					int temp = indices[i];
					indices [i] = indices [i+1];
					indices [i+1] = temp;
					goto sortagain;
				}
			}
			else if(GLOBAL_SAIZ[indices[i]] > GLOBAL_SAIZ[indices[i+1]])
			{
				int temp = indices[i];
				indices [i] = indices [i+1];
				indices [i+1] = temp;
				goto sortagain;
			}
		}
	}

	int r = 0; //for GLOBAL_NO_ROUTE becuase do not consider empty route	
	for (int i = 0; i < num_routes; i++)
	{
		if (GLOBAL_SAIZ[indices[i]] == 0)
			continue;
		GLOBAL_SAIZ[r] = sai[indices[i]]; //update GLOBAL_SAIZ[]

		for (int j = 0; j <= GLOBAL_SAIZ[r]+1; j++)
		{
			GLOBAL[r][j] = vec[indices[i]][j];
			GLOBALCumDist[r][j] = cumD[indices[i]][j];
		}	
		GLOBAL_capa[r] = capa[indices[i]];
		GLOBAL_Rcost[r] = cost[indices[i]];
		GLOBAL_distance_cost[r] = dist_cost[indices[i]];
		r++;
	}
	GLOBAL_NO_ROUTE = r;

	delete[] sai;
	delete[] capa;
	delete[] cost;
	delete[] dist_cost;
	for (int i = 0; i < no_routes; i++)
	{
		delete[] vec[i];
		delete[] cumD[i];
	}
	delete[] vec;
	delete[] cumD;

}


void sort_LOCALsolutionSMALLEST (int **(&LOCAL))
{
	int num_routes = LOCAL_NO_ROUTE;
	int *sai = new int[num_routes];
	int *capa = new int[num_routes];
	float  *cost = new float[num_routes];
	float *dist_cost = new float[no_routes];
	int**vec = new int* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		vec[i] = new int [SIZE];
	}

	float** cumD = new float* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		cumD[i] = new float [SIZE];
	}
	vector <float> data;

	for (int i = 0; i < num_routes; i++)//must use LOCAL_NO_ROUTE
	{
		for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
		{
			vec[i][j] = LOCAL[i][j];
			cumD[i][j] = LOCALCumDist[i][j];
		}
	}
	
	for (int i = 0; i < num_routes; i++)
	{
		sai[i] = LOCAL_SAIZ[i];
		capa[i] = LOCAL_capa[i];
		cost[i] = LOCAL_Rcost[i];
		dist_cost[i] = LOCAL_distance_cost[i];
		data.push_back (LOCAL_Rcost[i]); //data[] holds the cost

	}

	//sort in descending order, route with highest cost to lowest
	vector <size_t> indices( data.size(), 0 ); 
	iota( indices.begin(), indices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

	sort( indices.begin(), indices.end() , compare_indirect_indexS <decltype(data)> ( data ) ); //sort the data and index in decreasing order


	//cout << "Sorted route cost (from highest to lowest) "<<endl;
	//for (int i = 0; i < num_routes; i++)
	//{
	//	cout << "route= "<<indices[i] << ' ' << ' ' << ' '<<"cost= "<<data[indices[i]]<<endl;
	//}

	//if cost is the same, sort based on number of customer
	sortagain:
	for (int i = 0; i < num_routes-1; i++)
	{
		if (LOCAL_Rcost[indices[i]] == LOCAL_Rcost[indices[i+1]]) //if cost is the same
		{
			if (LOCAL_SAIZ[indices[i]] == LOCAL_SAIZ[indices[i+1]]) //if size is the same
			{
				if (LOCAL[indices[i]][1] > LOCAL[indices[i+1]][1]) //compare first customer
				{
					int temp = indices[i];
					indices [i] = indices [i+1];
					indices [i+1] = temp;
					goto sortagain;
				}
			}
			else if(LOCAL_SAIZ[indices[i]] > LOCAL_SAIZ[indices[i+1]])
			{
				int temp = indices[i];
				indices [i] = indices [i+1];
				indices [i+1] = temp;
				goto sortagain;
			}
		}
	}
	int emptyR=0;
	int r=0;
	for (int i = 0; i < num_routes; i++)
	{
		if (LOCAL_SAIZ[indices[i]] == 0)
		{
			emptyR++;
			continue;
		}
		LOCAL_SAIZ[r] = sai[indices[i]]; //update LOCAL_SAIZ[] based on saiz[] //different from GLOBAL
		
		for (int j = 0; j <= LOCAL_SAIZ[i]+1; j++)
		{
			LOCAL[r][j] = vec[indices[i]][j];
			LOCALCumDist[r][j] = cumD[indices[i]][j];
			
		}
		LOCAL_capa[r] = capa[indices[i]];
		LOCAL_Rcost[r] = cost[indices[i]];
		LOCAL_distance_cost[r] = dist_cost[indices[i]];
		r++;
	}
	//add empty route at back
	for (int i = 0; i < emptyR; i++)
	{
		LOCAL_capa[r] = 0;
		LOCAL_Rcost[r] = 0.0;
		LOCAL_SAIZ[r] = 0;
		LOCAL[r][0]=SIZE;
		LOCAL[r][1]=SIZE;
		r++;
	}

	delete[] sai;
	delete[] capa;
	delete[] cost;
	delete[] dist_cost;
	for (int i = 0; i < no_routes; i++)
	{
		delete[] vec[i];
		delete[] cumD[i];
	}
	delete[] vec;
	delete[] cumD;
}


void sort_solutionSMALLEST (int **(&VEHICLE))
{
	int *sai = new int[no_routes];
	int *capa = new int[no_routes];
	float *cost = new float[no_routes];
	float *dist_cost = new float[no_routes];
	int**vec = new int* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		vec[i] = new int [SIZE];
	}

	float** cumD = new float* [no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		cumD[i] = new float [SIZE];
	}
	vector <float> data;
	
	for (int i = 0; i < no_routes; i++)
	{
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			vec[i][j] = VEHICLE[i][j];
			cumD[i][j] = CumDist[i][j];
		}
	}
	for (int i = 0; i < no_routes; i++)
	{
		sai[i] = saiz[i];
		capa[i] = total_demand[i];
		cost[i] = route_cost[i];
		dist_cost[i] = distance_cost[i];
		data.push_back (route_cost[i]); //data[] holds the cost

	}

	//sort in ascending order, route with highest cost to lowest
	vector <size_t> indices( data.size(), 0 ); 
	iota( indices.begin(), indices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

	sort( indices.begin(), indices.end() , compare_indirect_indexS <decltype(data)> ( data ) ); //sort the data and index in decreasing order


	//cout << "Sorted route cost (from lowest to highest) "<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{
	//	cout << "route= "<<indices[i] << ' ' << ' ' << ' '<<"cost= "<<data[indices[i]]<<endl;
	//}

	//if cost is the same, sort based on number of customer
	sortagain:
	for (int i = 0; i < no_routes-1; i++)
	{
		if (route_cost[indices[i]] == route_cost[indices[i+1]]) //if cost is the same
		{
			if (saiz[indices[i]] == saiz[indices[i+1]]) //if size is the same
			{
				if (VEHICLE[indices[i]][1] > VEHICLE[indices[i+1]][1]) //compare first customer
				{
					int temp = indices[i];
					indices [i] = indices [i+1];
					indices [i+1] = temp;
					goto sortagain;
				}
			}
			else if(saiz[indices[i]] > saiz[indices[i+1]])
			{
				int temp = indices[i];
				indices [i] = indices [i+1];
				indices [i+1] = temp;
				goto sortagain;
			}
		}
	}

	int emptyR=0;
	int r=0;
	for (int i = 0; i < no_routes; i++)
	{
		if(sai[indices[i]] == 0)
		{
			emptyR++;
			continue;
		}

		saiz[r] = sai[indices[i]]; //update LOCAL_SAIZ[] based on saiz[] //different from GLOBAL
		
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			VEHICLE[r][j] = vec[indices[i]][j];	
			CumDist[r][j] = cumD[indices[i]][j];	
		}
		total_demand[r] = capa[indices[i]];
		route_cost[r] = cost[indices[i]];
		distance_cost[r] = dist_cost[indices[i]];
		space_available[r] = CAPACITY - total_demand[r];
		distance_available[r] = DISTANCE - distance_cost[r];
		r++;
	}
	//add empty route at back
	for (int i = 0; i < emptyR; i++)
	{
		total_demand[r] = 0;
		route_cost[r] = 0.0;
		saiz[r] = 0;
		space_available[r] = CAPACITY ;
		distance_available[r] = DISTANCE ;
		VEHICLE[r][0]=SIZE;
		VEHICLE[r][1]=SIZE;
		r++;
	}
	cout<<"after sort vEHICLE"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			cout<<VEHICLE[i][j]<<' ';
		}
		cout<<endl;
	}

	delete[] sai;
	delete[] capa;
	delete[] cost;
	delete[] dist_cost;
	for (int i = 0; i < no_routes; i++)
	{
		delete[] vec[i];
		delete[] cumD[i];
	}
	delete[] vec;
	delete[] cumD;
}
