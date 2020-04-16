#include <time.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include "DiversificationLNS.h"
//#include "LocalSearch.h"
#include "Bestimprovement.h"
#include "VNS.h"
//#include "VND.h"
#include "LocalSearch.h"//for delte and insert customer in perturb
//to sort and store index
#include <numeric>      // std::iota
#include <algorithm>    // std::sort


template <typename Container>
struct compare_indirect_indexB
  {
  const Container& container;
  compare_indirect_indexB( const Container& container ): container( container ) { }
  bool operator () ( size_t lindex, size_t rindex ) const
    {
    return container[ lindex ] > container[ rindex ]; //in decreasing order
    }
  };

struct MyStruct { 
    float distant;//sort index based on distant
    int index;
	MyStruct(float k, int i) : distant(k), index(i) {}
};

struct more_than_key//sort from big to small
{
    inline bool operator() (const MyStruct& struct1, const MyStruct& struct2)
    {
        return (struct1.distant > struct2.distant);
    }
};
using namespace std;
//vehicle[][] entering here to reinitialize to LOCAL_BEST_ROUTE
//info saiz[], route_cost[], total_demand[], distance_available[], space_available[] all being updated 
//struct cust
//{
//	int custID;
//	float costInsert;
//	int bestR;
//	int bestPos;
//	int insertStatus;
//	int perturbStatus;
//	int VNDStatus;
//};

//struct cust2
//{
//	int custID;
//	float costInsert;
//	int bestPos;
//};
bool *affectedR;//to flag which route has been changed after perturb

//always remove the largest arc in every route, K entered increases the loop if there is no improvement from the function
float Diversify_LongestArc(int **(&VEHICLE), int Kappa) //vehicle[][] entering here to reinitialize to GLOBAL_BEST_ROUTE //k is how many pair to remove
{
	//Kappa=1;
	Kappa = floor((float)Kappa/(float)no_routes) + rand() % 5 ; //to create randomization, otherwise always delete the same customer
	
	ofstream diversifyLNS("31.DiversifyLNS_" + std::to_string( I ) + ".txt", ios::app);
	diversifyLNS<<"K= "<<Kappa<<endl;
	diversifyLNS<<"In Diversify_LongestArc, before doing anything"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversifyLNS<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversifyLNS<<VEHICLE[i][j]<<' ';
		}
		diversifyLNS<<endl;
	}
	diversifyLNS<<"========================================================="<<endl;

	int *custRemoved = new int[SIZE];//to record customer removed
	float average_dist = 0; //the average distance between customer
	float variance = 0; 
	float std_dev = 0; 
	//int **vehic_depotCust = new int* [SIZE];
	int **temp_r = new int* [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later
	//float percentRemoved = 0.40; //remove 40% of customer

	for (int i = 0; i < SIZE; i++)
	{
		temp_r[i]= new int [SIZE];
	}
	int no_empty_route = 0; //for average_dist use
	
	//VEHICLE now is not necessary equal to GLOBAL, so need to copy from GLOBAL, already copied in function
	
	int c=0;//for custRemoved []
	float sum1=0;
	//find the pair of customer which has distance greater than average distance
	average_dist = (GLOBAL_BEST_COST- (SIZE*SERVICE_T))/ (SIZE+GLOBAL_NO_ROUTE-no_empty_route);//if 4 customer in a route, there are 5 arcs//minus empty route
	
	for (int i = 0; i < no_routes; i++)
    {
        for (int j = 0; j <= saiz[i]; j++)
		{
			sum1 = sum1 + pow((dist[VEHICLE[i][j]][VEHICLE[i][j+1]] - average_dist), 2);
		}
		
    }
    variance = sum1 / (SIZE+GLOBAL_NO_ROUTE-no_empty_route-1);
    std_dev = sqrt(variance);
	diversifyLNS<<"average_dist= "<<average_dist<<' '<<"std_dev= "<<std_dev<<endl;

	//############################ Delete the Kappa biggest pair in a route #############################// added on 19 June2015
	
	//Step 1: Sort the distance in a route, store the index
	//================= sort and record index ====================//
	for (int r = 0; r < no_routes; r++)
	{
		if (saiz[r] == 0) //if empty route, skip
			continue;
		bool *flag_delete = new bool[saiz[r]+2];//including depot at beginning and last, eg: DEPOT 1 2 3 DEPOT //flag_delete[] hold the position, not customer 
		for (int m = 0; m <= saiz[r]+1; m++)
		{
			flag_delete[m] = false; //initialize everytime enter a route
		}
		vector <float> data;
		for (int j = 0; j <= saiz[r]; j++) //push in distance data starting from DEPOT 1 2 3 DEPOT
		{
			data.push_back( dist[VEHICLE[r][j]][VEHICLE[r][j+1]]);
		}
		vector <size_t> indices( data.size(), 0 ); 
		iota( indices.begin(), indices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

		sort( indices.begin(), indices.end() , compare_indirect_indexB <decltype(data)> ( data ) ); //sort the data and index
		
		//std::cout << "data:   "; for (int j = 0; j <= saiz[i]; j++) std::cout << data[j] << " "; std::cout << "\n";
		//std::cout << "index:  "; for (int j = 0; j <= saiz[i]; j++) std::cout << indices[j] << " "; std::cout << "\n";
		//std::cout << "sorted: "; for (int j = 0; j <= saiz[i]; j++) std::cout << data[indices[j]] << " "; std::cout << "\n";
		
		//Step 2: Flag the customers position to be deleted
		int n=0;
		while (n<Kappa && n<=saiz[r]) //kappa is how many customer to be flagged, maked sure the saiz[] is not exceed
		{
			//if (data[indices[n]]<average_dist+0.76*std_dev)//control distance so that not too many to be deleted
			//	break;//if distance less than average, leave loop early
			flag_delete[indices[n]] = flag_delete[indices[n]+1] = true; //flag position [arc] and position[arc+1] true //the first one (depot) might be flagged, but the last one (depot) wil never be flagged. the indices: DEPOT 1 2 3 4 5 (no DEPOT) 
			n++;
		}
		
		bool thisRchange=false;//only do 2-opt and update route if this route has changed (meaning there is customer removed from this route)
		
		//Step 3: Delete customers from the route based on flagged
		temp_r[r][0] = SIZE; //first one is depot
		int t=1;//for temp_r[][] use
		for (int j = 1; j <= saiz[r]; j++)//make sure starts from 1 to saiz[i] because we dont want to delete the first and last one as depot
		{
			int cust = -1, c_before = -1, c_after = -1;
			if (flag_delete[j] == false)//if position is not flagged, copy the customer to temp_r[][]
			{
				temp_r[r][t] = VEHICLE[r][j];
				t++;
			}
			else if(flag_delete[j]==true)
			{
				//if position is flagged, dont copy to temp_r[][]
				cust = VEHICLE[r][j];
				custRemoved[c] = cust;
				c++;
				total_demand[r] = total_demand[r] - demand[cust]; //update total_demand for temp solution
				thisRchange= true;//if this route has changed (meaning there is customer removed from this route)
				//delete_one_cust (r, j, vehicle);//cannot use this because after delete, the position changed, previously at [1] becomes [0]
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
		}
		delete[] flag_delete;
		data.clear();
		indices.clear();
	}
	//inter_route_improvementforLNS();//after delete, improve route first, added on 15/06/2016, inter_route_improvement() is modified
	cout<<"custRemoved[i] are "<<endl;
	diversifyLNS<<"======================================================================== "<<endl;
	diversifyLNS<<"custRemoved[i] are "<<endl;
	for (int i = 0; i < c; i++)
	{
		cout<<custRemoved[i]<<' '<<' ';
		diversifyLNS<<custRemoved[i]<<' '<<' ';
	}
	cout<<endl;
	diversifyLNS<<endl;
	diversifyLNS<<"number of cust removed= "<<c<<endl;

	cout<<"VEHICLE[][] after remove customer "<<endl;
	diversifyLNS<<"VEHICLE[][] after remove customer "<<endl;
	int tcust=0;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		tcust+=saiz[i];
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			cout<<VEHICLE[i][j]<<' ';
			diversifyLNS<<VEHICLE[i][j]<<' ';
		}
		cout<<endl;
		diversifyLNS<<endl;
	}
	cout<<"current total cust is "<<tcust<<endl;
	diversifyLNS<<"current total cust is "<<tcust<<endl;

	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
	{
		diversifyLNS<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		diversifyLNS<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}

	float t_cost=0.0;
	diversifyLNS<<"=======================AFTER INSERTION=========================="<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversifyLNS<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		t_cost = t_cost + route_cost[i];
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversifyLNS<<VEHICLE[i][j]<<' ';
		}
		diversifyLNS<<endl;
	}
	diversifyLNS<<"t_cost= "<<t_cost<<endl;

	for (int i = 0; i < SIZE; i++)
	{
		delete[] temp_r[i];
	}
	delete[] temp_r;
	delete[] custRemoved;

	return t_cost;
}

//always remove the largest arc in OVERALL routes, K entered increases the loop if there is no improvement from the function
//added on 6MAY2016
float Diversify_LongestArc2(int **(&VEHICLE), int Kappa) //vehicle[][] entering here to reinitialize to GLOBAL_BEST_ROUTE 
{
	ofstream Diversify_LongestArc2("31.Diversify_LongestArc2_" + std::to_string( I ) + ".txt", ios::app);
	Diversify_LongestArc2<<"K= "<<Kappa<<endl;
	Diversify_LongestArc2<<"In Diversify_LongestArc2, before doing anything"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		Diversify_LongestArc2<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			Diversify_LongestArc2<<VEHICLE[i][j]<<' ';
		}
		Diversify_LongestArc2<<endl;
	}
	Diversify_LongestArc2<<"========================================================="<<endl;
	bool *flagcust = new bool[SIZE]; 
	int *custRemoved = new int[SIZE];//to record customer removed
	
	int **temp_r = new int* [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later


	for (int i = 0; i < SIZE; i++)
	{
		flagcust[i] = false; //initially all flag false
		temp_r[i]= new int [SIZE];
	}
	
	
	int c=0;//for custRemoved []


	//############################ Delete the Kappa customers from routes #############################
	
	//Step 1: Sort the distance, store the index
	//================= sort and record index ====================//
	std::vector < MyStruct > Pair1;//consist of dist, and cust1
	std::vector < MyStruct > Pair2;//consist of dist, and cust2

	for (int r = 0; r < no_routes; r++)
	{
		if (saiz[r] == 0) //if empty route, skip
			continue;
		
		for (int j = 0; j <= saiz[r]; j++) //push in distance data starting from DEPOT 1 2 3 DEPOT
		{
			//data.push_back( dist[VEHICLE[r][j]][VEHICLE[r][j+1]]);
			Pair1.push_back(MyStruct(dist[VEHICLE[r][j]][VEHICLE[r][j+1]], VEHICLE[r][j]));
			Pair2.push_back(MyStruct(dist[VEHICLE[r][j]][VEHICLE[r][j+1]], VEHICLE[r][j+1]));
		}
	}
	std::sort(Pair1.begin(), Pair1.end(), more_than_key());//sort from big to small
	std::sort(Pair2.begin(), Pair2.end(), more_than_key());//sort from big to small

	
	//Step 2: Flag the customers  to be deleted
	srand ( time(NULL) ); //seed it
	float rand_y; //generate uniform number
	int p = 4; //user defined parameter setting

	
	int n=0;
	while (n<Kappa) //kappa is how many customer to be flagged
	{
		rand_y = (float)rand() / (float)((unsigned)RAND_MAX + 1); //exclusion of 1.0 was intentional.
		int t = floor (pow(rand_y, p) * (float)SIZE);//for Pair index favourable to small number //if use ceil, there is a changce to generate SIZE
		if (t>= SIZE || t< 0)
		{
			Diversify_LongestArc2<<"pseudo-random is wrong when generating t "<<endl;;
			Diversify_LongestArc2<<c<<' '<<endl;
		}


		if (flagcust[Pair1[t].index] == false && Pair1[t].index != SIZE)
		{
			flagcust[Pair1[t].index] = true;
			n++;
		}
		if (flagcust[Pair2[t].index] == false && Pair2[t].index != SIZE)
		{
			flagcust[Pair2[t].index] = true;
			n++;
		}
	
	}
		
	//Step 3: Delete customers based on flagged
	for (int r = 0; r < no_routes; r++)
	{
		temp_r[r][0] = SIZE; //first one is depot
		int t=1;//for temp_r[][] use
		bool thisRchange=false;//for the use of 2-opt later, only do 2-opt if this route has changed
		for (int j = 1; j <= saiz[r]; j++)//make sure starts from 1 to saiz[i] because we dont want to delete the first and last one as depot
		{
			int cust = -1, c_before = -1, c_after = -1;
			if (flagcust[VEHICLE[r][j]] == false)//if position is not flagged, copy the customer to temp_r[][]
			{
				temp_r[r][t] = VEHICLE[r][j];
				t++;
			}
			else if(flagcust[VEHICLE[r][j]]==true)
			{
				//if position is flagged, dont copy to temp_r[][]
				cust = VEHICLE[r][j];
				custRemoved[c] = cust;
				c++;
				total_demand[r] = total_demand[r] - demand[cust]; //update total_demand for temp solution
				thisRchange=true;
				//delete_one_cust (r, j, vehicle);//cannot use this because after delete, the position changed, previously at [1] becomes [0]
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
		}//end of (thisRchange==true)

	}//end of r
	//inter_route_improvementforLNS();//after delete, improve route first, added on 15/06/2016, inter_route_improvement() is modified

	cout<<"custRemoved[i] are "<<endl;
	Diversify_LongestArc2<<"======================================================================== "<<endl;
	Diversify_LongestArc2<<"custRemoved[i] are "<<endl;
	for (int i = 0; i < c; i++)
	{
		cout<<custRemoved[i]<<' '<<' ';
		Diversify_LongestArc2<<custRemoved[i]<<' '<<' ';
	}
	cout<<endl;
	Diversify_LongestArc2<<endl;
	Diversify_LongestArc2<<"number of cust removed= "<<c<<endl;

	cout<<"VEHICLE[][] after remove customer "<<endl;
	Diversify_LongestArc2<<"VEHICLE[][] after remove customer "<<endl;
	int tcust=0;
	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		tcust+=saiz[i];
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			cout<<VEHICLE[i][j]<<' ';
			Diversify_LongestArc2<<VEHICLE[i][j]<<' ';
		}
		cout<<endl;
		Diversify_LongestArc2<<endl;
	}
	cout<<"current total cust is "<<tcust<<endl;
	Diversify_LongestArc2<<"current total cust is "<<tcust<<endl;

	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
	{
		Diversify_LongestArc2<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		Diversify_LongestArc2<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}

	float t_cost=0.0;
	Diversify_LongestArc2<<"=======================AFTER INSERTION=========================="<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		Diversify_LongestArc2<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		t_cost = t_cost + route_cost[i];
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			Diversify_LongestArc2<<VEHICLE[i][j]<<' ';
		}
		Diversify_LongestArc2<<endl;
	}
	Diversify_LongestArc2<<"t_cost= "<<t_cost<<endl;

	for (int i = 0; i < SIZE; i++)
	{
		delete[] temp_r[i];
	}
	delete[] temp_r;
	delete[] custRemoved;
	delete[] flagcust; 

	return t_cost;
}

//always remove the smallest ratio=demand/(gain from deletion) //based on paper Li 
float Diversify_LNS2(int **(&VEHICLE), int Kappa) 
{
	//int Nremove = min(72+Kappa,SIZE/10);//Kappa is added 1 each time if no improvement found in original function
	int Nremove = Kappa;
	ofstream diversifyLNS2("31.DiversifyLNS2_" + std::to_string( I ) + ".txt", ios::app);
	diversifyLNS2<<"K= "<<Kappa<<endl;
	diversifyLNS2<<"In diversifyLNS2, before doing anything"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversifyLNS2<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversifyLNS2<<VEHICLE[i][j]<<' ';
		}
		diversifyLNS2<<endl;
	}
	diversifyLNS2<<"========================================================="<<endl;
	diversifyLNS2<<"Nremove= "<<Nremove <<endl;
	bool *flag_delete = new bool[SIZE]; 
	int *custRemoved = new int[SIZE];//to record customer removed
	
	int **temp_r = new int* [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later

	for (int i = 0; i < SIZE; i++)
	{
		flag_delete[i] = false; //initially all flag false //flag_delete[] hold the customer index!!!!!!!!!!!!!! different from LNS
		temp_r[i]= new int [SIZE];	
	}
	//already copied in fucntion
	//no_routes = GLOBAL_NO_ROUTE;//initialize
	//for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	//{
	//	for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
	//	{
	//		VEHICLE[i][j] = GLOBAL[i][j];		
	//	}
	//	saiz[i] = GLOBAL_SAIZ[i];
	//	route_cost[i] = GLOBAL_Rcost[i];
	//	total_demand[i]= GLOBAL_capa[i]; 
	//	space_available[i] = CAPACITY - total_demand[i];
	//	distance_available[i] = DISTANCE - distance_cost[i];
	//}
	
	float *ratio = new float[SIZE];
	int priorC=-1, nextC=-1;
	int Cust=-1;

	for (int i = 0; i < no_routes; i++)
	{
		if (saiz[i] == 0)
			continue;
		for (int j = 1; j <= saiz[i]; j++)
		{
			priorC = VEHICLE[i][j-1];
			nextC = VEHICLE[i][j+1];
			Cust = VEHICLE[i][j];
			float origain = dist[priorC][Cust] + dist[Cust][nextC] - dist[priorC][nextC];//old-new
			int remainingcust = saiz[i]-1;
			if (remainingcust < 0)//if negative
				remainingcust= 0;
			float gain_from_deletion = CumDist[i][j] + remainingcust*origain;

			ratio[Cust] = demand[Cust]/gain_from_deletion;

		}
	}
	//sort the index in ascending order
	vector <float> ration;

	for (int j = 0; j <SIZE; j++) //push in distance ratio because iota and sort only accept vector parameter
	{
		ration.push_back( ratio[j]);
	}
	vector <size_t> Rindices( ration.size(), 0 ); 
	iota( Rindices.begin(), Rindices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

	sort( Rindices.begin(), Rindices.end() , compare_indirect_indexB <decltype(ration)> ( ration ) ); //sort the data and index from the highest to lowest

	//when remove customer, cannot use the position pre-recorded becaus the index changes after every customer removed
	int n=0;//for custRemoved []
	srand ( time(NULL) ); //seed it
	float rand_y; //generate uniform number
	float p = 0.2; //user defined parameter setting //prefer Rindices big the bottom of the list because ration is sorted in descending order but we want to take the smallest ratio
	diversifyLNS2<<"Selecting customer pseudo-randomly"<<endl;
	while (n < Nremove)
	{
		rand_y = (float)rand() / (float)((unsigned)RAND_MAX + 1); //exclusion of 1.0 was intentional.
		int c = floor (pow(rand_y, p) * (float)SIZE);	//if use ceil, there is a changce to generate SIZE
		if (c>= SIZE || c< 0)
		{
			diversifyLNS2<<"pseudo-random is wrong when generating c "<<endl;;
			diversifyLNS2<<c<<' '<<endl;
		}
		int cust = Rindices[c];
		
		diversifyLNS2<<cust<<' ';

		if (flag_delete[cust] == false)//if this customer has not been flagged 
		{
			flag_delete[cust] = true;//flag it true, so we know that is the customer to delete later
			n++;//record how many cust flag, meaning how many to be removed later
		//i--;
		}
		
	}
	for (int i = 0; i < SIZE; i++)
	{
		diversifyLNS2<<flag_delete[i]<<' ';
	}
	diversifyLNS2<<endl;
	for (int i = 0; i < SIZE; i++)
	{
		diversifyLNS2<<Rindices[i]<<' ';
	}
	diversifyLNS2<<endl;
	//=============== to delete customer ==================//
	int c=0;//for custRemoved []
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
	ration.clear();
	Rindices.clear();

	//inter_route_improvementforLNS();//after delete, improve route first, added on 15/06/2016, inter_route_improvement() is modified
	cout<<"custRemoved[i] are "<<endl;
	diversifyLNS2<<"======================================================================== "<<endl;
	diversifyLNS2<<"custRemoved[i] are "<<endl;
	for (int i = 0; i < c; i++)
	{
		cout<<custRemoved[i]<<' '<<' ';
		diversifyLNS2<<custRemoved[i]<<' '<<' ';
	}
	cout<<endl;
	diversifyLNS2<<endl;
	diversifyLNS2<<"number of cust removed= "<<c<<endl;

	cout<<"vehic_depotCust[][] after remove customer "<<endl;
	diversifyLNS2<<"vehic_depotCust[][] after remove customer "<<endl;

	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			cout<<VEHICLE[i][j]<<' ';
			diversifyLNS2<<VEHICLE[i][j]<<' ';
		}
		cout<<endl;
		diversifyLNS2<<endl;
	}
	
	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
	{
		diversifyLNS2<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		diversifyLNS2<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}
	
	float t_cost=0.0;
	for (int i = 0; i < no_routes; i++)
	{
		t_cost = t_cost + route_cost[i];
	}
	diversifyLNS2<<"=======================AFTER INSERTION=========================="<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversifyLNS2<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversifyLNS2<<VEHICLE[i][j]<<' ';
		}
		diversifyLNS2<<endl;
	}
	diversifyLNS2<<"t_cost= "<<t_cost<<endl;

	for (int i = 0; i < SIZE; i++)
	{
		delete[] temp_r[i];
	}
	delete[] temp_r;
	delete[] flag_delete ; 
	delete[] custRemoved ;//to record customer removed
	delete[] ratio;

	return t_cost;
}

//always remove the largest gain from deletion 
float Diversify_LNS3(int **(&VEHICLE), int Kappa) 
{
	//int Nremove = min(72+Kappa,SIZE/10);//Kappa is added 1 each time if no improvement found in original function
	int Nremove = Kappa;
	ofstream diversifyLNS3("31.DiversifyLNS3_" + std::to_string( I ) + ".txt", ios::app);
	diversifyLNS3<<"K= "<<Kappa<<endl;
	diversifyLNS3<<"In diversifyLNS3, before doing anything"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversifyLNS3<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversifyLNS3<<VEHICLE[i][j]<<' ';
		}
		diversifyLNS3<<endl;
	}
	diversifyLNS3<<"========================================================="<<endl;
	diversifyLNS3<<"Nremove= "<<Nremove <<endl;
	bool *flag_delete = new bool[SIZE]; 
	int *custRemoved = new int[SIZE];//to record customer removed
	
	int **temp_r = new int* [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later

	for (int i = 0; i < SIZE; i++)
	{
		flag_delete[i] = false; //initially all flag false //flag_delete[] hold the customer index!!!!!!!!!!!!!! different from LNS
		temp_r[i]= new int [SIZE];	
	}
	//already copied in fucntion
	//no_routes = GLOBAL_NO_ROUTE;//initialize
	//for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	//{
	//	for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
	//	{
	//		VEHICLE[i][j] = GLOBAL[i][j];		
	//	}
	//	saiz[i] = GLOBAL_SAIZ[i];
	//	route_cost[i] = GLOBAL_Rcost[i];
	//	total_demand[i]= GLOBAL_capa[i]; 
	//	space_available[i] = CAPACITY - total_demand[i];
	//	distance_available[i] = DISTANCE - distance_cost[i];
	//}
	
	//float *ratio = new float[SIZE];
	float *GAIN = new float[SIZE];
	int priorC=-1, nextC=-1;
	int Cust=-1;

	for (int i = 0; i < no_routes; i++)
	{
		if (saiz[i] == 0)
			continue;
		for (int j = 1; j <= saiz[i]; j++)
		{
			priorC = VEHICLE[i][j-1];
			nextC = VEHICLE[i][j+1];
			Cust = VEHICLE[i][j];
			float origain = dist[priorC][Cust] + dist[Cust][nextC] - dist[priorC][nextC];//old-new
			int remainingcust = saiz[i]-1;
			if (remainingcust < 0)//if negative
				remainingcust= 0;
			GAIN[Cust] = CumDist[i][j] + remainingcust*origain;

		}
	}
	//sort the index in ascending order
	vector <float> ration;

	for (int j = 0; j <SIZE; j++) //push in distance ratio because iota and sort only accept vector parameter
	{
		//ration.push_back( ratio[j]);
		ration.push_back( GAIN[j]);
	}
	vector <size_t> Rindices( ration.size(), 0 ); 
	iota( Rindices.begin(), Rindices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

	sort( Rindices.begin(), Rindices.end() , compare_indirect_indexB <decltype(ration)> ( ration ) ); //sort the data and index from the highest to lowest

	//when remove customer, cannot use the position pre-recorded becaus the index changes after every customer removed
	int n=0;//for custRemoved []
	//int i = SIZE-1;//traverse from bottom of the list because ration is sorted in descending order but we want to take the smallest ratio

	srand ( time(NULL) ); //seed it
	float rand_y; //generate uniform number
	int p = 4; //user defined parameter setting
	diversifyLNS3<<"Selecting customer pseudo-randomly "<<endl;;
	while (n < Nremove)
	{
		rand_y = (float)rand() / (float)((unsigned)RAND_MAX + 1); //exclusion of 1.0 was intentional.
		int c = floor (pow(rand_y, p) * (float)SIZE);//if use ceil, there is a changce to generate SIZE
		
		if (c>= SIZE || c< 0)
		{
			diversifyLNS3<<"pseudo-random is wrong when generating c "<<endl;;
			diversifyLNS3<<c<<' '<<endl;
		}
		int cust = Rindices[c];
		
		diversifyLNS3<<cust<<' ';
		if (flag_delete[cust] == false)//if this customer has not been flagged 
		{
			flag_delete[cust] = true;//flag it true, so we know that is the customer to delete later
			n++;//record how many cust flag, meaning how many to be removed later
		//i--;
		}
		
	}
	for (int i = 0; i < SIZE; i++)
	{
		diversifyLNS3<<flag_delete[i]<<' ';
	}
	diversifyLNS3<<endl;
	for (int i = 0; i < SIZE; i++)
	{
		diversifyLNS3<<Rindices[i]<<' ';
	}
	diversifyLNS3<<endl;
	//=============== to delete customer ==================//
	int c=0;//for custRemoved []
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
	ration.clear();
	Rindices.clear();

	//inter_route_improvementforLNS();//after delete, improve route first, added on 15/06/2016, inter_route_improvement() is modified
	cout<<"custRemoved[i] are "<<endl;
	diversifyLNS3<<"======================================================================== "<<endl;
	diversifyLNS3<<"custRemoved[i] are "<<endl;
	for (int i = 0; i < c; i++)
	{
		cout<<custRemoved[i]<<' '<<' ';
		diversifyLNS3<<custRemoved[i]<<' '<<' ';
	}
	cout<<endl;
	diversifyLNS3<<endl;
	diversifyLNS3<<"number of cust removed= "<<c<<endl;

	cout<<"vehic_depotCust[][] after remove customer "<<endl;
	diversifyLNS3<<"vehic_depotCust[][] after remove customer "<<endl;

	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			cout<<VEHICLE[i][j]<<' ';
			diversifyLNS3<<VEHICLE[i][j]<<' ';
		}
		cout<<endl;
		diversifyLNS3<<endl;
	}
	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
	{
		diversifyLNS3<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		diversifyLNS3<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}
	float t_cost=0.0;
	for (int i = 0; i < no_routes; i++)
	{
		t_cost = t_cost + route_cost[i];
	}
	diversifyLNS3<<"=======================AFTER INSERTION=========================="<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversifyLNS3<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversifyLNS3<<VEHICLE[i][j]<<' ';
		}
		diversifyLNS3<<endl;
	}
	diversifyLNS3<<"t_cost= "<<t_cost<<endl;

	for (int i = 0; i < SIZE; i++)
	{
		delete[] temp_r[i];
	}
	delete[] temp_r;
	delete[] flag_delete ; 
	delete[] custRemoved ;//to record customer removed
	//delete[] ratio;
	delete[] GAIN;
	return t_cost;
}

float Diversify_Relatedness(int **(&VEHICLE), int Kappa) 
{
	//int Nremove = min(72+Kappa,SIZE/10);//Kappa is added 1 each time if no improvement found in original function
	int Nremove = Kappa;
	ofstream diversifyRelatedness("31.DiversifyRelatedness_" + std::to_string( I ) + ".txt", ios::app);
	diversifyRelatedness<<"K= "<<Kappa<<endl;
	diversifyRelatedness<<"In diversifyRelatedness, before doing anything"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversifyRelatedness<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversifyRelatedness<<VEHICLE[i][j]<<' ';
		}
		diversifyRelatedness<<endl;
	}
	diversifyRelatedness<<"========================================================="<<endl;
	diversifyRelatedness<<"Nremove= "<<Nremove <<endl;
	bool *flag_delete = new bool[SIZE]; 
	int *custRemoved = new int[SIZE];//to record customer removed
	
	int **temp_r = new int* [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later

	for (int i = 0; i < SIZE; i++)
	{
		flag_delete[i] = false; //initially all flag false //flag_delete[] hold the customer index!!!!!!!!!!!!!! different from LNS
		temp_r[i]= new int [SIZE];	
	}
	//already copied in fucntion
	//no_routes = GLOBAL_NO_ROUTE;//initialize
	//for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	//{
	//	for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
	//	{
	//		VEHICLE[i][j] = GLOBAL[i][j];		
	//	}
	//	saiz[i] = GLOBAL_SAIZ[i];
	//	route_cost[i] = GLOBAL_Rcost[i];
	//	total_demand[i]= GLOBAL_capa[i]; 
	//	space_available[i] = CAPACITY - total_demand[i];
	//	distance_available[i] = DISTANCE - distance_cost[i];
	//}
	

	int del=0;
	bool *randomselect = new bool [SIZE];
	for (int i = 0; i < SIZE; i++)
	{
		randomselect[i] =false;//to flag it true once this is the random customer selected
	}
	srand ( time(NULL) ); //seed it
	while(del < Nremove)
	{
		regenerate:
		int rand_num = (rand() % (SIZE)); //generate random customer
		
		if (randomselect[rand_num] == true)
			goto regenerate;
		else
		{
			randomselect[rand_num] = true;//once select, flag randomselect true so that it will not be selected next time
			if (flag_delete[rand_num] == false)//if it has not been previously flagged as neigbouring customer, need to add in to del++
			{
				flag_delete[rand_num] = true;
				del++;
			}
			if (del>=Nremove)
				goto enoughdelete;
		}
		
		for (int i = 0; i < SIZE; i++)
		{
			if (NR_FLAG[rand_num][i] == true && flag_delete[rand_num] == false)//flag the neighbouring customer and //if it has not been previously flagged
			{
				flag_delete[i] = true;
				del++;
			}
			if (del>=Nremove)
				goto enoughdelete;
		}

	}
	enoughdelete:
	for (int i = 0; i < SIZE; i++)
	{
		diversifyRelatedness<<flag_delete[i]<<' ';
	}
	diversifyRelatedness<<endl;
	
	//=============== to delete customer ==================//
	int c=0;//for custRemoved []
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

	cout<<"custRemoved[i] are "<<endl;
	diversifyRelatedness<<"======================================================================== "<<endl;
	diversifyRelatedness<<"custRemoved[i] are "<<endl;
	for (int i = 0; i < c; i++)
	{
		cout<<custRemoved[i]<<' '<<' ';
		diversifyRelatedness<<custRemoved[i]<<' '<<' ';
	}
	cout<<endl;
	diversifyRelatedness<<endl;
	diversifyRelatedness<<"number of cust removed= "<<c<<endl;

	cout<<"vehic_depotCust[][] after remove customer "<<endl;
	diversifyRelatedness<<"vehic_depotCust[][] after remove customer "<<endl;

	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			cout<<VEHICLE[i][j]<<' ';
			diversifyRelatedness<<VEHICLE[i][j]<<' ';
		}
		cout<<endl;
		diversifyRelatedness<<endl;
	}
	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
	{
		diversifyRelatedness<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		diversifyRelatedness<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}
	
	float t_cost=0.0;
	for (int i = 0; i < no_routes; i++)
	{
		t_cost = t_cost + route_cost[i];
	}
	diversifyRelatedness<<"=======================AFTER INSERTION=========================="<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversifyRelatedness<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversifyRelatedness<<VEHICLE[i][j]<<' ';
		}
		diversifyRelatedness<<endl;
	}
	diversifyRelatedness<<"t_cost= "<<t_cost<<endl;

	for (int i = 0; i < SIZE; i++)
	{
		delete[] temp_r[i];
	}
	delete[] temp_r;
	delete[] flag_delete ; 
	delete[] randomselect;
	delete[] custRemoved ;//to record customer removed


	return t_cost;
}

float Diversify_EntireRoute(int **(&VEHICLE), int Kappa, float *x, float *y) 
{
	
	int Nremove = Kappa;
	ofstream Diversify_EntireRoute("31.Diversify_EntireRoute_" + std::to_string( I ) + ".txt", ios::app);
	Diversify_EntireRoute<<"K= "<<Kappa<<endl;
	Diversify_EntireRoute<<"In Diversify_EntireRoute, before doing anything"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		Diversify_EntireRoute<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			Diversify_EntireRoute<<VEHICLE[i][j]<<' ';
		}
		Diversify_EntireRoute<<endl;
	}
	Diversify_EntireRoute<<"========================================================="<<endl;
	Diversify_EntireRoute<<"Nremove= "<<Nremove <<endl;
	
	int *custRemoved = new int[SIZE];//to record customer removed

	//find first route randomly, find the nearest route to the random route
	//select route by random
	srand ( time(NULL) ); //seed it
	int routetodelete=2;
	int r1 = (rand() % no_routes); 
	cout<<r1<<endl;
	float **route_CGravity = new float*[no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		route_CGravity[i] = new float [2];
	}
	for (int i = 0; i < no_routes; i++)
	{
		float sumX = 0, sumY=0;
		for (int j = 1; j <= saiz[i]; j++)
		{
			sumX = sumX + x[VEHICLE[i][j]];
			sumY = sumY + y[VEHICLE[i][j]];
		}
		route_CGravity[i][0] = (sumX+x[SIZE]) / (saiz[i]+1);  //record center gravity x coordinate //sum+depot/saiz+1
		route_CGravity[i][1] = (sumY+y[SIZE]) / (saiz[i]+1);  //record center gravity x coordinate //sum+depot/saiz+1
	}
	float **DistRGravity = new float*[no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		DistRGravity[i] = new float [no_routes];
	}
	//find distance between centroid
	for (int i = 0; i < no_routes; i++)
	{
		for (int j = i+1; j < no_routes; j++)
		{
			float sum =  (route_CGravity[i][0]-route_CGravity[j][0])*(route_CGravity[i][0]-route_CGravity[j][0]) + (route_CGravity[i][1]-route_CGravity[j][1])*(route_CGravity[i][1]-route_CGravity[j][1]) ;
			DistRGravity[i][j] = DistRGravity[j][i] = sqrt(sum);
		}
		
	}
	float nearestDist=INT_MAX;
	int nearestR=0;
	for (int i = 0; i < no_routes; i++)
	{
		if (i==r1)
			continue;
		if (DistRGravity[r1][i]<nearestDist)
		{
			nearestDist=DistRGravity[r1][i];
			nearestR= i;
		}
	}
	
	int r2 = nearestR;//nearest route based on centroid
	cout<<r2;
	for (int i = 0; i < no_routes; i++)
	{
		delete[] route_CGravity[i];
	}
	delete[] route_CGravity;

	

	//=============== to delete customer ==================//
	int c=0;//for custRemoved []
	for (int w = 0; w < routetodelete; w++)
	{
		int r;
		if (w==0)
			r=r1;
		else
			r=r2;
		if (saiz[r] == 0) //if empty route, skip
			continue;
		
		// Delete customers from the route 
		
		for (int j = 1; j <= saiz[r]; j++)//make sure starts from 1 to saiz[i] because we dont want to delete the first and last one as depot
		{		
			custRemoved[c] = VEHICLE[r][j];
			c++;
		}
		VEHICLE[r][1]=SIZE;
		route_cost[r] = 0;//initialize
		distance_cost[r] = 0;//initialize
		total_demand[r] = 0; //update total_demand for temp solution
		for (int j = 0; j <= saiz[r]+1; j++) //check if the saiz has been updated in delete()!!!!!!!!!!!!!!!!!!!!!!!!!
		{
			CumDist[r][j]=0;		
		}
		space_available[r] = CAPACITY  ; //update space_available for temp solution
		distance_available[r] = DISTANCE ; //update distance_available for temp solution
		saiz[r] = 0;//update size of route //must update outside the for loop because for loop use size[] as a termination criteria, dont update inside
	}

	//inter_route_improvementforLNS();//after delete, improve route first, added on 15/06/2016, inter_route_improvement() is modified

	cout<<"custRemoved[i] are "<<endl;
	Diversify_EntireRoute<<"======================================================================== "<<endl;
	Diversify_EntireRoute<<"custRemoved[i] are "<<endl;
	for (int i = 0; i < c; i++)
	{
		cout<<custRemoved[i]<<' '<<' ';
		Diversify_EntireRoute<<custRemoved[i]<<' '<<' ';
	}
	cout<<endl;
	Diversify_EntireRoute<<endl;
	Diversify_EntireRoute<<"number of cust removed= "<<c<<endl;

	cout<<"vehic_depotCust[][] after remove customer "<<endl;
	Diversify_EntireRoute<<"vehic_depotCust[][] after remove customer "<<endl;

	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			cout<<VEHICLE[i][j]<<' ';
			Diversify_EntireRoute<<VEHICLE[i][j]<<' ';
		}
		cout<<endl;
		Diversify_EntireRoute<<endl;
	}
	srand(( unsigned )time(0));
	if (rand() % 2 ==0) 
	{
		Diversify_EntireRoute<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		Diversify_EntireRoute<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}
	
	float t_cost=0.0;
	for (int i = 0; i < no_routes; i++)
	{
		t_cost = t_cost + route_cost[i];
	}
	Diversify_EntireRoute<<"=======================AFTER INSERTION=========================="<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		Diversify_EntireRoute<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			Diversify_EntireRoute<<VEHICLE[i][j]<<' ';
		}
		Diversify_EntireRoute<<endl;
	}
	Diversify_EntireRoute<<"t_cost= "<<t_cost<<endl;


	delete[] custRemoved ;//to record customer removed


	return t_cost;
}

//
float Diversify_RelatednessDemand(int **(&VEHICLE), int Kappa) 
{
	//int Nremove = min(72+Kappa,SIZE/10);//Kappa is added 1 each time if no improvement found in original function
	int Nremove = Kappa;
	ofstream diversifyRelatednessDemand("31.DiversifyRelatednessDemand_" + std::to_string( I ) + ".txt", ios::app);
	diversifyRelatednessDemand<<"K= "<<Kappa<<endl;
	diversifyRelatednessDemand<<"In diversifyRelatednessDemand, before doing anything"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversifyRelatednessDemand<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversifyRelatednessDemand<<VEHICLE[i][j]<<' ';
		}
		diversifyRelatednessDemand<<endl;
	}
	diversifyRelatednessDemand<<"========================================================="<<endl;
	diversifyRelatednessDemand<<"Nremove= "<<Nremove <<endl;
	bool *flag_delete = new bool[SIZE]; 
	int *custRemoved = new int[SIZE];//to record customer removed
	
	int **temp_r = new int* [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later

	for (int i = 0; i < SIZE; i++)
	{
		flag_delete[i] = false; //initially all flag false //flag_delete[] hold the customer index!!!!!!!!!!!!!! different from LNS
		temp_r[i]= new int [SIZE];	
	}
	//already copied in fucntion
	//no_routes = GLOBAL_NO_ROUTE;//initialize
	//for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	//{
	//	for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
	//	{
	//		VEHICLE[i][j] = GLOBAL[i][j];		
	//	}
	//	saiz[i] = GLOBAL_SAIZ[i];
	//	route_cost[i] = GLOBAL_Rcost[i];
	//	total_demand[i]= GLOBAL_capa[i]; 
	//	space_available[i] = CAPACITY - total_demand[i];
	//	distance_available[i] = DISTANCE - distance_cost[i];
	//}
	

	int del=0;
	bool *randomselect = new bool [SIZE];
	for (int i = 0; i < SIZE; i++)
	{
		randomselect[i] =false;//to flag it true once this is the random customer selected
	}
	srand ( time(NULL) ); //seed it
	while(del < Nremove)
	{
		regenerate:
		int rand_num = (rand() % (SIZE)); //generate random customer
		
		if (randomselect[rand_num] == true)
			goto regenerate;
		else
		{
			randomselect[rand_num] = true;//once select, flag randomselect true so that it will not be selected next time
			if (flag_delete[rand_num] == false)//if it has not been previously flagged as related customer, need to add in to del++
			{
				flag_delete[rand_num] = true;
				del++;
			}
			if (del>=Nremove)
				goto enoughdelete;
		}
		
		for (int i = 0; i < SIZE; i++)
		{
			if (DEMAND_FLAG[rand_num][i] == true && flag_delete[i] == false)//flag the neighbouring customer and //if it has not been previously flagged
			{
				flag_delete[i] = true;
				del++;
			}
			if (del>=Nremove)
				goto enoughdelete;
		}

	}
	enoughdelete:
	for (int i = 0; i < SIZE; i++)
	{
		diversifyRelatednessDemand<<flag_delete[i]<<' ';
	}
	diversifyRelatednessDemand<<endl;
	
	//=============== to delete customer ==================//
	int c=0;//for custRemoved []
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

	cout<<"custRemoved[i] are "<<endl;
	diversifyRelatednessDemand<<"======================================================================== "<<endl;
	diversifyRelatednessDemand<<"custRemoved[i] are "<<endl;
	for (int i = 0; i < c; i++)
	{
		cout<<custRemoved[i]<<' '<<' ';
		diversifyRelatednessDemand<<custRemoved[i]<<' '<<' ';
	}
	cout<<endl;
	diversifyRelatednessDemand<<endl;
	diversifyRelatednessDemand<<"number of cust removed= "<<c<<endl;

	cout<<"vehic_depotCust[][] after remove customer "<<endl;
	diversifyRelatednessDemand<<"vehic_depotCust[][] after remove customer "<<endl;

	for (int i = 0; i < no_routes; i++)
	{
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			cout<<VEHICLE[i][j]<<' ';
			diversifyRelatednessDemand<<VEHICLE[i][j]<<' ';
		}
		cout<<endl;
		diversifyRelatednessDemand<<endl;
	}
	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
	{
		diversifyRelatednessDemand<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		diversifyRelatednessDemand<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}
	
	float t_cost=0.0;
	for (int i = 0; i < no_routes; i++)
	{
		t_cost = t_cost + route_cost[i];
	}
	diversifyRelatednessDemand<<"=======================AFTER INSERTION=========================="<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversifyRelatednessDemand<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversifyRelatednessDemand<<VEHICLE[i][j]<<' ';
		}
		diversifyRelatednessDemand<<endl;
	}
	diversifyRelatednessDemand<<"t_cost= "<<t_cost<<endl;

	for (int i = 0; i < SIZE; i++)
	{
		delete[] temp_r[i];
	}
	delete[] temp_r;
	delete[] flag_delete ; 
	delete[] randomselect;
	delete[] custRemoved ;//to record customer removed


	return t_cost;
}

//always remove those arc with NR flag false
float Diversify_BadArcNR(int **(&VEHICLE), int Kappa) 
{
	int Nremove = Kappa;
	ofstream Diversify_BadArcNR("31.Diversify_BadArcNR_" + std::to_string( I ) + ".txt", ios::app);
	Diversify_BadArcNR<<"K= "<<Kappa<<endl;
	Diversify_BadArcNR<<"In Diversify_BadArcNR, before doing anything"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		Diversify_BadArcNR<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			Diversify_BadArcNR<<VEHICLE[i][j]<<' ';
		}
		Diversify_BadArcNR<<endl;
	}
	Diversify_BadArcNR<<"========================================================="<<endl;
	Diversify_BadArcNR<<"Nremove= "<<Nremove <<endl;
	bool *flag_delete = new bool[SIZE]; 
	int *custRemoved = new int[SIZE];//to record customer removed
	
	int **temp_r = new int* [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later

	for (int i = 0; i < SIZE; i++)
	{
		flag_delete[i] = false; //initially all flag false //flag_delete[] hold the customer index!!!!!!!!!!!!!! different from LNS
		temp_r[i]= new int [SIZE];	
	}


	int totalFlag=0;


	
	for (int i = 0; i < no_routes; i++)
	{
		for (int j = 0; j <= saiz[i]; j++)
		{

			if (j==0 || j== saiz[i])//if this is end-point, use NR_FLAG_DEPOT
			{
				if (NR_FLAG_DEPOT[VEHICLE[i][j]][VEHICLE[i][j+1]] == false)
				{
					if (flag_delete[VEHICLE[i][j]] == false && VEHICLE[i][j] != SIZE)//it this has not been flagged and not depot, dont want depot to be flagged and added to totalFlag
					{
						flag_delete[VEHICLE[i][j]] = true;
						totalFlag++;
					}
					if (flag_delete[VEHICLE[i][j+1]] == false && VEHICLE[i][j+1] != SIZE)//it this has not been flagged and not depot, dont want depot to be flagged and added to totalFlag
					{
						flag_delete[VEHICLE[i][j+1]] = true;
						totalFlag++;
					}
				}
				if (totalFlag >= Nremove)//cannot use while loop because totalFlag might be zero or forever less than Nremove
				goto enoughdelete;

				continue;//noneed to chekc the NR_FLAG, so skip the rest
			}


			if (NR_FLAG[VEHICLE[i][j]][VEHICLE[i][j+1]] == false)
			{
				if (flag_delete[VEHICLE[i][j]] == false)
				{
					flag_delete[VEHICLE[i][j]] = true;
					totalFlag++;
				}
				if (flag_delete[VEHICLE[i][j+1]] == false)
				{
					flag_delete[VEHICLE[i][j+1]] = true;
					totalFlag++;
				}
			}
			if (totalFlag >= Nremove)
				goto enoughdelete;
		}
	}
	

	srand ( time(NULL) ); //seed it
	while (totalFlag < Nremove)//if no customer selected or customer selected less than the number it suppose to be, randomly select customer to remove
	{
		int rand_num = (rand() % (SIZE)); //generate random customer
		if (flag_delete[rand_num] == false)
		{
			flag_delete[rand_num] = true;
			totalFlag++;
		}
	}

enoughdelete:
	
	int c=0;//for custRemoved []
	//while(c < Nremove)//customer flagged maybe more than customer need to be deleted, so change to this condition//i dont think this while function works, it should be inside for (int r) loop to work
	//{

		for (int i = 0; i < SIZE; i++)
		{
			Diversify_BadArcNR<<flag_delete[i]<<' ';
		}
		Diversify_BadArcNR<<endl;
	
		//=============== to delete customer ==================//

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
		
	}//end for r
	//}//end while

		//inter_route_improvementforLNS();//after delete, improve route first, added on 15/06/2016, inter_route_improvement() is modified

		cout<<"custRemoved[i] are "<<endl;
		Diversify_BadArcNR<<"======================================================================== "<<endl;
		Diversify_BadArcNR<<"custRemoved[i] are "<<endl;
		for (int i = 0; i < c; i++)
		{
			cout<<custRemoved[i]<<' '<<' ';
			Diversify_BadArcNR<<custRemoved[i]<<' '<<' ';
		}
		cout<<endl;
		Diversify_BadArcNR<<endl;
		Diversify_BadArcNR<<"number of cust removed= "<<c<<endl;

		cout<<"vehic_depotCust[][] after remove customer "<<endl;
		Diversify_BadArcNR<<"vehic_depotCust[][] after remove customer "<<endl;

		for (int i = 0; i < no_routes; i++)
		{
			cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
			for (int j = 0; j <= saiz[i]+1; j++)
			{
				cout<<VEHICLE[i][j]<<' ';
				Diversify_BadArcNR<<VEHICLE[i][j]<<' ';
			}
			cout<<endl;
			Diversify_BadArcNR<<endl;
		}
	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
	{
		Diversify_BadArcNR<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		Diversify_BadArcNR<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}
	
	float t_cost=0.0;
	for (int i = 0; i < no_routes; i++)
	{
		t_cost = t_cost + route_cost[i];
	}
	Diversify_BadArcNR<<"=======================AFTER INSERTION=========================="<<endl;
	cout<<"=======================AFTER INSERTION=========================="<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		Diversify_BadArcNR<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			Diversify_BadArcNR<<VEHICLE[i][j]<<' ';
			cout<<VEHICLE[i][j]<<' ';
		}
		Diversify_BadArcNR<<endl;
		cout<<endl;
	}
	Diversify_BadArcNR<<"t_cost= "<<t_cost<<endl;
	cout<<"t_cost= "<<t_cost<<endl;

	for (int i = 0; i < SIZE; i++)
	{
		delete[] temp_r[i];
	}
	delete[] temp_r;
	delete[] flag_delete ; 
	delete[] custRemoved ;//to record customer removed



	return t_cost;

}

float deleteHeadTail(int **(&VEHICLE), int Kappa)
{
	//ofstream diversify_headTail("34.DiversifyheadTail_.txt", ios::out);
	//diversify_headTail<<"In diversify_headTail, before doing anything"<<endl;
	//for (int i = 0; i < no_routes; i++)
	//{
	//	diversify_headTail<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
	//	for (int j = 0; j <= saiz[i]+1; j++)
	//	{
	//		diversify_headTail<<VEHICLE[i][j]<<' ';
	//	}
	//	diversify_headTail<<endl;
	//}


	int num_cust_removePerRoute = ceil((float)Kappa/(float)no_routes); 
	if (num_cust_removePerRoute %2 != 0)//if not even number, make it even because we want to delete equal length of head and tail
	{
		num_cust_removePerRoute++;
	}
	
	int *custRemoved = new int[SIZE];//to record customer removed
	int c=0;//record how many customer removed
	int *temp_r = new int [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later
	
	for (int i = 0; i < no_routes; i++)
	{
		if(saiz[i] <= num_cust_removePerRoute)//delete all
		{
			for (int s = 1; s <= saiz[i]; s++)
			{
				custRemoved[c] = VEHICLE[i][s];
				c++;
			}
			saiz[i] = 0;
			total_demand[i] = 0;
			route_cost[i] =  0.0;
			distance_cost[i] =  0.0;
			distance_available[i] = DISTANCE;
			space_available[i] = CAPACITY;
			VEHICLE[i][1] = SIZE;//the second element is depot, otherwise it is still the customer //added on 18Oct 2015
			continue;//go to next route
		}

		//============= partial delete ======================//

		//delete the head part
		int lengthdelete = num_cust_removePerRoute/2; //divide the length to equal length of delete for head and tail
		int cust = -1;
			
		for (int j = 1; j <= lengthdelete; j++)
		{
			cust = VEHICLE[i][j];
			custRemoved[c] = cust;
			total_demand[i] = total_demand[i] - demand[cust]; //update total_demand for temp solution
			c++;
		}
		//copy the middle part
		temp_r[0] = SIZE; //first one is depot
		int t=1;//for temp_r[]
		for (int j = lengthdelete+1; j <= saiz[i]-lengthdelete; j++)
		{
			temp_r[t] = VEHICLE[i][j];
			t++;
		}

		//delete the tail part
		for (int j = saiz[i]-lengthdelete+1; j <= saiz[i]; j++)
		{
			cust = VEHICLE[i][j];
			custRemoved[c] = cust;
			total_demand[i] = total_demand[i] - demand[cust]; //update total_demand for temp solution
			c++;
		}

			
		saiz[i] = t-1;//update size of route //must update outside the for loop because for loop use size[] as a termination criteria, dont update inside
		temp_r[t] = SIZE; //last one is depot

		//compute distance
		route_cost[i] = 0;//initialize
		distance_cost[i] = 0;//initialize
		CumDist[i][0]= 0;
		for (int j = 0; j <= saiz[i]; j++)
		{
			distance_cost[i] = distance_cost[i]+dist[temp_r[j]][temp_r[j+1]] + service_time[temp_r[j]];
				
		}
		for (int j = 0; j <= saiz[i]+1; j++) //check if the saiz has been updated in delete()!!!!!!!!!!!!!!!!!!!!!!!!!
		{
			VEHICLE[i][j] = temp_r[j];
			CumDist[i][j]=CumDist[i][j-1]+dist[temp_r[j-1]][temp_r[j]] + service_time[temp_r[j]];
		}
		space_available[i] = CAPACITY - total_demand[i] ; //update space_available for temp solution
		distance_available[i] = DISTANCE - distance_cost[i]; //update distance_available for temp solution
		////if ((saiz[r] != 0) || (saiz[r] != 1) )//skip if saiz=0 or saiz=1
		//if (saiz[i]>=3)//changed on 28JUn2015
		//{
		//	two_opt_singleR(vehicle, i); 
		//	or_opt_singleR2(vehicle, i); 
		//}	
	}
	
	//diversify_headTail<<endl<<"======================================================================== "<<endl;
	//diversify_headTail<<"custRemoved[i] are "<<endl;
	//for (int i = 0; i < c; i++)
	//{
	//	diversify_headTail<<custRemoved[i]<<' '<<' ';
	//}
	//diversify_headTail<<endl;
	//diversify_headTail<<"number of cust removed= "<<c<<endl;
	//
	//diversify_headTail<<"VEHICLE after remove"<<endl;
	int tcust=0;
	for (int i = 0; i < no_routes; i++)
	{
		//diversify_headTail<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		tcust+=saiz[i];
		//for (int j = 0; j <= saiz[i]+1; j++)
		//{
		//	diversify_headTail<<VEHICLE[i][j]<<' ';
		//}
		//diversify_headTail<<endl;
		if ((total_demand[i] < 0) || (route_cost[i] < 0) )
			getchar();
	}
	//diversify_headTail<<"Current number of cust is "<<tcust<<endl;
	
	//bool overallchange and bool *routeChange have no function, just because the function alreday declared to return value and receive argument
	//bool *routeChange = new bool[no_routes];
	//bool overallchange = miniVND(VEHICLE, routeChange);
	//delete[] routeChange;
	//miniVND(VEHICLE);

	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
		greedyInsertion2(VEHICLE, c, custRemoved);
	
	else
		regretInsertion(VEHICLE, c, custRemoved);

	float t_cost=0.0;
	
	//diversify_headTail<<"=======================AFTER INSERTION=========================="<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		t_cost = t_cost + route_cost[i];
		//diversify_headTail<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		//for (int j = 0; j <= saiz[i]+1; j++)
		//{
		//	diversify_headTail<<VEHICLE[i][j]<<' ';
		//}
		//diversify_headTail<<endl;
	}
	//diversify_headTail<<"t_cost= "<<t_cost<<endl;


	delete[] temp_r;
	delete[] custRemoved ;//to record customer removed

	return t_cost;
}

int pertub1_1 (int **(&VEHICLE), int cust,  int &pos, float &bestCost, float &bestDist)//return pos, bestCost, and bestDist because insertion is done by function call, 
{
	//ofstream perturb("32.Perturb.txt", ios::out);
	
	int before_pos2 = -1, after_pos2 = -1, before_pos3 = -1, after_pos3 = -1, route3=-1, to_position3 = -1, to_position = -1, pos_delete = -1;
	float cost_without_i, cost_with_i, gain1, gain2, cost_of_insertion;
	float gainr1, gainr2, gainDistr1, gainDistr2; //added this so that when call function delete or insert the function dont need to recompute cost and distance
	float best_insertion = INT_MAX;
	bool posNotFound = true;
	std::vector<int> vector_r2;
	int to_route = -1; //to be passed back to function
	int custtoRemove =-1;
	for (int m = 0; m < no_routes; m++)  //insert (to) which route , and remove one from this route
	{
		if (saiz[m]==0)// if it is an empty route, nothing to be removed
		{
			continue;

		}
		for (int n = 1; n <= saiz[m]; n++) //which customer in r2 to remove
		{
			custtoRemove = VEHICLE[m][n];			//element to be removed and insert to route3
										
			if (demand[cust] > (space_available[m] + demand[custtoRemove])) 
				continue;//go to next element to delete

			//==================================================copy r2 without customer n=================================================================//
			vector_r2.clear();
			for (int a = 0; a <= saiz[m] + 1; a++)
			{
				if (a == n)
					continue;
				vector_r2.push_back(VEHICLE[m][a]); //added this on 26JUNE 2015 //SHOULD CHANGE THE REST IF THIS IS CORRECT

			}

			//############################################## To Insert in route m ##########################################################////////////////////
			for (int p = 1; p < vector_r2.size(); p++) //insert to route
			{
				before_pos2 = vector_r2[p - 1];
				after_pos2 = vector_r2[p];  //noneed to add 1 because it is insert in between n-1 and n
				if ( before_pos2 == SIZE || after_pos2 == SIZE ) //if mn depot //changed this to here on 25JUN2015!!!!!!!!!!!!!!!!!!!!! OMG!!! before that put after n loop
				{
					if ( NR_FLAG_DEPOT[cust][before_pos2] == false && NR_FLAG_DEPOT[cust][after_pos2] == false )//if both does not fulfill, only skip, considering that they originally not good because the good ones are disconnected
						continue;
				}
				else //if no depot
				{
					if ( NR_FLAG[cust][before_pos2] == false && NR_FLAG[cust][after_pos2] == false ) //if both does not fulfill, only skip, considering that they originally not good because the good ones are disconnected
						continue;
				}
				
				float origain1 = dist[VEHICLE[m][n]][VEHICLE[m][n - 1]] + dist[VEHICLE[m][n]][VEHICLE[m][n + 1]] + service_time[VEHICLE[m][n]] - dist[VEHICLE[m][n - 1]][VEHICLE[m][n + 1]]; //(old + service_time – new)
				float oriloss1 = dist[before_pos2][cust] + dist[cust][after_pos2] + service_time[cust]- dist[before_pos2][after_pos2]; //element inserted to route (new-old)
				if (oriloss1 - origain1 > distance_available[m] + epsilon)
				{
					continue; //go to next position to insert, element deleted is still the same
				}

				int remainingcust = saiz[m]-n;
				if (remainingcust < 0)//if negative
					remainingcust= 0;
				 cost_without_i = CumDist[m][n] + remainingcust*origain1;//cannot update from a certain start point because cost_of_removing for the entire route change when a customer is deleted from the route, because number_of_remaining customer changes
		
				//to find cost_with_i which is besed on the modified route
				//find a temporaryCumDCust
				float *tempCumDist1= new float[SIZE];
				tempCumDist1[0] = 0;
							
				for (int t = 1; t < saiz[m]; t++)//have 1 size less because 1 customer is deleted
				{
					if (t<n)
						tempCumDist1[t]= CumDist[m][t];//before the delete pos, the CumDist is the same
					else if(t>=n)
						tempCumDist1[t]=CumDist[m][t+1]-origain1;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
							
				}

				float CumAtEle1 = tempCumDist1[p-1] + dist[before_pos2][cust] + service_time[cust]; 
				int remainingCustafterEle1 = saiz[m] - p;
				if (remainingCustafterEle1 < 0)//if negative
					remainingCustafterEle1 = 0;
				cost_with_i = CumAtEle1 + remainingCustafterEle1*oriloss1; 

				gain1 = cost_without_i - cost_with_i;//check if this is correct


				//=======================to insert in the third route============================// third route must fulfill constraint
				for (int r = 0; r < no_routes; r++)
				{
					if ((r==m) || (demand[custtoRemove] > space_available[r]))//not the same as original route
						continue;
					for (int s = 1; s <= saiz[r]+1; s++)
					{
						before_pos3 = VEHICLE[r][s-1];
						after_pos3 = VEHICLE[r][s];
						if ((before_pos3 == SIZE) || (after_pos3 == SIZE) ) //if  depot //changed this to here on 25JUN2015!!!!!!!!!!!!!!!!!!!!! OMG!!! before that put after n loop
						{
							if ( NR_FLAG_DEPOT[custtoRemove][before_pos3] == false && NR_FLAG_DEPOT[custtoRemove][after_pos3] == false )
								continue;
						}
						else //if no depot
						{
							if ( NR_FLAG[custtoRemove][before_pos3] == false && NR_FLAG[custtoRemove][after_pos3] == false )
								continue;
						}
						
						float oriloss2 = dist[before_pos3][custtoRemove] + dist[custtoRemove][after_pos3] + service_time[custtoRemove]- dist[before_pos3][after_pos3]; //element inserted to route (new-old)
				
						if (oriloss2 - distance_available[r] > epsilon) //only consider r2
						{
							continue;//go to next position to insert, element deleted is still the same
						}

						float CumAtEle = CumDist[r][s-1] + dist[before_pos3][custtoRemove] + service_time[custtoRemove]; 
						int remainingCustafterEle = saiz[r] - s;
						if (remainingCustafterEle < 0)//if negative
									remainingCustafterEle = 0;
						gain2 = CumAtEle + remainingCustafterEle*oriloss2; //element inserted to route (new-old)


						cost_of_insertion = gain1 + gain2;

						if (cost_of_insertion - best_insertion < -epsilon)
						{
							to_route = m;
							best_insertion= cost_of_insertion;
							
							gainr1= cost_without_i ;//just record gain delete because insert will be done by original function call
							gainDistr1 = origain1;//just record gain delete because insert will be done by original function call
							
							gainr2= gain2;//insert third cust in r3
							gainDistr2 = oriloss2;
							
							bestCost = cost_with_i; //insert cust in r1
							bestDist = oriloss1;//dist of insert cust in r1

							pos_delete = n;//record the position to delete
							pos = p; //record the position to insert after the customer is removed
							posNotFound = false; //pos found
							route3 = r;
							to_position3 = s;
							
						}//end if gain>best_gain
					}//end for s (insert 3rd route position
				}//end r (insert third route)
				delete[] tempCumDist1;
			}//end p (insert to route 2 position)
		}//end n (which customer in route2 to remove)
	}//end m (route 1 -> to insert and remove customer from this route)

	if (posNotFound == false)//pos found
	{
		//perturb<<"Before deletion, vehicle["<<to_route<<"]"<<' ';
		//for (int t = 0; t <= saiz[to_route]+1; t++)
		//{
		//	perturb<<VEHICLE[to_route][t]<<' ';
		//}
		//perturb<<"capa= "<<total_demand[to_route]<<endl;
		custtoRemove=VEHICLE[to_route][pos_delete];//must update this because this might change after finding the best one in cost_of_insertion
		//perturb<<"custtoRemove= "<<custtoRemove<<endl;
		
		gainvec[0] = 1;//status yes
		gainvec[1] = gainr1;
		gainvec[2] = gainDistr1;

		delete_one_cust (to_route, pos_delete, VEHICLE);

		gainvec[0] = 1;//status yes
		gainvec[1] = gainr2;
		gainvec[2] = gainDistr2;
		insert_one_cust (route3, to_position3, VEHICLE, custtoRemove);
	
		affectedR[to_route] = true;
		affectedR[route3] = true;

		//perturb<<"After deletion in route1, vehicle["<<to_route<<"]"<<' ';
		//for (int t = 0; t <= saiz[to_route]+1; t++)
		//{
		//	perturb<<VEHICLE[to_route][t]<<' ';
		//}
		//perturb<<"capa= "<<total_demand[to_route]<<endl;
		//perturb<<"After insertion in route3, vehicle["<<route3<<"]"<<' ';
		//for (int t = 0; t <= saiz[route3]+1; t++)
		//{
		//	perturb<<VEHICLE[route3][t]<<' ';
		//}
		//perturb<<"capa= "<<total_demand[route3]<<endl;
		//perturb<< "In ejection chain, found route= "<<to_route<<" position= "<<pos<<endl;

	}

	else
	{
		//cout<<"DId not find route and position in pertube 1-1 for cust "<<cust<<", cannot continue, try perturb 1-2"<<endl;
		//for (int i = 0; i < no_routes; i++)
		//{
		//	cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		//	for (int j = 0; j <= saiz[i]+1; j++)
		//	{
		//		cout<<VEHICLE[i][j]<<' ';
		//	}
		//	cout<<endl;
		//}
		to_route = -1;//return -1 if not found in perturb
	}

	return to_route;//return route
}

//full perturb until all customer have been inserted (05/06/2016)
//first, find if there is a place to insert, if no, find the bets customer to withdraw and insert the current customer in, repeat until all customer are inserted
bool fullPerturb (int **(&VEHICLE), int cust)//insertion is done here!!!!!!!!!!!!!! 
{
	ofstream fullperturb("FullPerturb.txt", ios::out);
	float cost_with_i, cost_without_i, gain1;
	bool status = true;//to be returned to function call, if successful, return true

	std::vector<int> vector_r2;
	std::vector<int> tabulist;//to store customer that has been deleted , so that next time this list will not be deleted
	int fixedListSize = 5;//user-defined tabu list size

	int custtoRemove =-1;


	int before_pos, after_pos, before_pos2, after_pos2 ;
	float *bestGain = new float[4];//to record bestGain of insertion without removing a cust//[0] gain, [1] dist_gain, [2]route number, [3]position
	bestGain[0] = INT_MAX;

	float *bestGainremove = new float[5];//to record bestGain of insertion by removing a cust//[0] gain, [1] dist_gain, [2]route number, [3]position remove (old), [4] position insert (new) ----->modified route
	bestGainremove[0] = INT_MIN;

	bool AllInsert = false;
	while (AllInsert == false)
	{
		//======================= find insertion without deletion =======================//
		bool foundbestGain=false;
		bestGain[0] = INT_MAX;//reinitialize because it is a new  loop now
		for (int m = 0; m < no_routes; m++)  //insert (to) which route , NO remove one from this route
		{
			if (demand[cust] > space_available[m] ) //if demand exceed avaialable space in route
			{
				continue;
			}
			for (int n = 1; n < saiz[m] + 2; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
			{
				
				float available_dist_r2 = distance_available[m];
			
				before_pos = VEHICLE[m][n - 1];
				after_pos = VEHICLE[m][n];  //noneed to add 1 because it is insert in between n-1 and n
					
				if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
				{
					if ( NR_FLAG_DEPOT[cust][before_pos] == false && NR_FLAG_DEPOT[cust][after_pos] == false ) //use NR_FLAG_DEPOT
						continue;
				}

				else //if not between customer and depot
				{	
					if ( NR_FLAG[cust][before_pos] == false && NR_FLAG[cust][after_pos] == false ) //if insertion cannot between cust before and cust after
						continue;
				}					
				float oriloss = dist[before_pos][cust] + dist[cust][after_pos] + service_time[cust] - dist[before_pos][after_pos]; //(new+service_time – old)
				float CumAtEle = CumDist[m][n-1] + dist[before_pos][cust] + service_time[cust]; 

				int remainingCustafterEle = saiz[m] - n +1; 
				if (remainingCustafterEle < 0)//if negative
						remainingCustafterEle = 0;
				cost_with_i = CumAtEle + remainingCustafterEle*oriloss; 			
				if (oriloss - available_dist_r2 > epsilon) //only consider r2
				{
					continue;
				}		
				if (cost_with_i > 99999)
				{
					cout<<"cost_with_i is too big, sth wrong"<<endl;
					getchar();
				}		
				//oriloss = 5, oriloss = 10, oriloss = -20, the best is -20, current best value bestGain[0]= -15
				if (cost_with_i < bestGain[0])//gain < bestgain
				{
					bestGain[0] = cost_with_i;   //gain in cost //minus because in function insert, route_cost - gain
					bestGain[1] = oriloss; //gain in distance
					bestGain[2] = m;//route number
					bestGain[3] = n;//which position
					foundbestGain=true;						
				}				
			}//end of n
			if (foundbestGain == true)
			{
				fullperturb<<"Found bestGain for cust "<<cust<<endl;
				//if found a place to insert, just insert noneed to do the rest
				affectedR[(int) bestGain[2]] = true;
				gainvec[0] = 1;//status yes
				gainvec[1] = -bestGain[0];//in function it is minus
				gainvec[2] = -bestGain[1];//in function it is minus
				insert_one_cust (bestGain[2], bestGain[3], VEHICLE, cust); //1 in 
				AllInsert = true;
				goto endoffunction;
			}
		}//end of m


		//=================== if not found, find insertion with 1 deletion
		bool foundbestGainremove=false;
		bestGainremove[0] = INT_MIN;//reinitialize because it is a new  loop now
		for (int m = 0; m < no_routes; m++)  //insert (to) which route , and remove one from this route
		{
			if (saiz[m]==0)// if it is an empty route, nothing to be removed
			{
				continue;

			}
			for (int n = 1; n <= saiz[m]; n++) //which customer in r2 to remove
			{
				custtoRemove = VEHICLE[m][n];			//element to be removed and to be inserted to other route later
				
				for (int t = 0; t < tabulist.size(); t++)//go through the list, if customer in the list, do not delete this customer
				{
					if (custtoRemove == tabulist[t])
						goto nextN;
				}
				if (demand[cust] > (space_available[m] + demand[custtoRemove])) 
					continue;//go to next element to delete

				//==================================================copy r2 without customer n=================================================================//
				vector_r2.clear();
				for (int a = 0; a <= saiz[m] + 1; a++)
				{
					if (a == n)
						continue;
					vector_r2.push_back(VEHICLE[m][a]); 

				}

				//############################################## To Insert in route m ##########################################################////////////////////
				for (int p = 1; p < vector_r2.size(); p++) //insert to route
				{
					before_pos2 = vector_r2[p - 1];
					after_pos2 = vector_r2[p];  //noneed to add 1 because it is insert in between n-1 and n
					//if ( before_pos2 == SIZE || after_pos2 == SIZE ) //if mn depot //changed this to here on 25JUN2015!!!!!!!!!!!!!!!!!!!!! OMG!!! before that put after n loop
					//{
					//	if ( NR_FLAG_DEPOT[cust][before_pos2] == false && NR_FLAG_DEPOT[cust][after_pos2] == false )//if both does not fulfill, only skip, considering that they originally not good because the good ones are disconnected
					//		continue;
					//}
					//else //if no depot
					//{
					//	if ( NR_FLAG[cust][before_pos2] == false && NR_FLAG[cust][after_pos2] == false ) //if both does not fulfill, only skip, considering that they originally not good because the good ones are disconnected
					//		continue;
					//}
				
					float origain1 = dist[VEHICLE[m][n]][VEHICLE[m][n - 1]] + dist[VEHICLE[m][n]][VEHICLE[m][n + 1]] + service_time[VEHICLE[m][n]] - dist[VEHICLE[m][n - 1]][VEHICLE[m][n + 1]]; //(old + service_time – new)
					float oriloss1 = dist[before_pos2][cust] + dist[cust][after_pos2] + service_time[cust]- dist[before_pos2][after_pos2]; //element inserted to route (new-old)
					if (oriloss1 - origain1 > distance_available[m] + epsilon)
					{
						continue; //go to next position to insert, element deleted is still the same
					}

					int remainingcust = saiz[m]-n;
					if (remainingcust < 0)//if negative
						remainingcust= 0;
					 cost_without_i = CumDist[m][n] + remainingcust*origain1;
		
					//to find cost_with_i which is besed on the modified route
					//find a temporaryCumDCust
					float *tempCumDist1= new float[SIZE];
					tempCumDist1[0] = 0;
							
					for (int t = 1; t < saiz[m]; t++)//have 1 size less because 1 customer is deleted
					{
						if (t<n)
							tempCumDist1[t]= CumDist[m][t];//before the delete pos, the CumDist is the same
						else if(t>=n)
							tempCumDist1[t]=CumDist[m][t+1]-origain1;//at or after  the delete pos, the CumDist is different by subtracting the orignal gain
							
					}

					float CumAtEle1 = tempCumDist1[p-1] + dist[before_pos2][cust] + service_time[cust]; 
					int remainingCustafterEle1 = saiz[m] - p;
					if (remainingCustafterEle1 < 0)//if negative
						remainingCustafterEle1 = 0;
					cost_with_i = CumAtEle1 + remainingCustafterEle1*oriloss1; 

					gain1 =  cost_without_i - cost_with_i;//check if this is correct

					delete[] tempCumDist1;
					if (gain1 > bestGainremove[0])//gain < bestgainremove
					{
						bestGainremove[0] = gain1;   //gain in cost 
						bestGainremove[1] = origain1 - oriloss1; //gain in distance
						bestGainremove[2] = m;//route number
						bestGainremove[3] = n;//old position
						bestGainremove[4] = p;//new position
						foundbestGainremove=true;						
					}	
				}//end p (insert to route 2 position)
				nextN:;
			}//end n (which customer in route2 to remove)
		}//end m (route 1 -> to insert and remove customer from this route)

		if (foundbestGainremove == true)
		{
			fullperturb<<"Found bestGain remove for cust "<<cust<<endl;
			gainvec[0] = 1;//status yes
			gainvec[1] = bestGainremove[0];
			gainvec[2] = bestGainremove[1];
			custtoRemove = VEHICLE[(int)bestGainremove[2]][(int)bestGainremove[3]]; ;//custtoRemove may not holding exactlt the one should be removed 
			affectedR[(int) bestGainremove[2]] = true;
			insert_1_0_sameRoute (bestGainremove[2], bestGainremove[3], bestGainremove[4], VEHICLE, cust);//only used by fullPerturb
			
			//if (tabulist.size() < fixedListSize)//if list is not full, just add to top
			//{
				tabulist.push_back(cust);
			//}
			//else//if list is full
			//{
			//	tabulist.erase (tabulist.begin());//delete the [0] element 
			//	tabulist.push_back(cust);//push new customer on top
			//}
			cust = custtoRemove;//now customer removed become the new customer to be inserted
		}
		else //if not found something to remove and insert current one, dont know what to do???
		{
			//cout<< "NOT foundbestGainremove,"<<"What should I do now? "<<endl;
			//cout<<"cust is "<<cust<<endl;
			//cout<<"current VEHICLE[][] "<<endl;
			//for (int i = 0; i < no_routes; i++)
			//{
			//	cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<space_available[i]<<' '<<' ';
			//	for (int j = 0; j <= saiz[i]+1; j++)
			//	{
			//		cout<<VEHICLE[i][j]<<' ';	
			//	}
			//	cout<<endl;
			//}
			//cout<<"tabulist[] "<<endl;
			//for (int t = 0; t < tabulist.size(); t++)//go through the list, if customer in the list, do not delete this customer
			//{
			//	cout<<tabulist[t]<<' ';
			//}
			//cout<<endl;
			//status = false;
			//goto endoffunction;

			//added this on 13June2016 with capacity violated//
			cout<< "NOT foundbestGainremove,"<<"go to bestInsertionwithCapacityExceed "<<endl;
			cout<<"cust is "<<cust<<endl;
			bestInsertionwithCapacityExceed (VEHICLE, cust);
			status = true;
			//getchar();
			AllInsert = true;
			goto endoffunction;	
		}
	}//end while 
endoffunction:;
	delete[] bestGain;
	delete[] bestGainremove;
	vector_r2.clear();
	vector_r2.shrink_to_fit();
	tabulist.clear();
	tabulist.shrink_to_fit();
	return status;
}

void bestInsertionwithCapacityExceed (int **(&VEHICLE), int cust)
{
	ofstream exceed("CapacityExceedInsertion.txt", ios::out);
	float cost_with_i, cost_without_i, gain1;


	int before_pos, after_pos, before_pos2, after_pos2 ;
	float *bestGain = new float[4];//to record bestGain of insertion without removing a cust//[0] gain, [1] dist_gain, [2]route number, [3]position
	bestGain[0] = INT_MAX;

	
	bool AllInsert = false;
	//======================= find insertion without deletion =======================//
	bool foundbestGain=false;
	bestGain[0] = INT_MAX;//reinitialize because it is a new  loop now
	int m=0; //best route to insert with smallest exceed

	int smallestexceed = demand[cust] - space_available[0];
	for (int i = 0; i < no_routes; i++)  //find route with the smallest exceed
	{
		int exceed = demand[cust] - space_available[i];
		if (exceed < smallestexceed)
		{
			smallestexceed = exceed;
			m = i;
		}
	}
		
		for (int n = 1; n < saiz[m] + 2; n++) //insert (to) which position, from 0 to size + 1, insert at 1 means replace element [1] which is the first customer
		{
				
			float available_dist_r2 = distance_available[m];
			
			before_pos = VEHICLE[m][n - 1];
			after_pos = VEHICLE[m][n];  //noneed to add 1 because it is insert in between n-1 and n
					
			//if ( before_pos == SIZE || after_pos == SIZE ) //if the insertion is between customer and depot
			//{
			//	if ( NR_FLAG_DEPOT[cust][before_pos] == false && NR_FLAG_DEPOT[cust][after_pos] == false ) //use NR_FLAG_DEPOT
			//		continue;
			//}

			//else //if not between customer and depot
			//{	
			//	if ( NR_FLAG[cust][before_pos] == false && NR_FLAG[cust][after_pos] == false ) //if insertion cannot between cust before and cust after
			//		continue;
			//}					
			float oriloss = dist[before_pos][cust] + dist[cust][after_pos] + service_time[cust] - dist[before_pos][after_pos]; //(new+service_time – old)
			float CumAtEle = CumDist[m][n-1] + dist[before_pos][cust] + service_time[cust]; 

			int remainingCustafterEle = saiz[m] - n +1; 
			if (remainingCustafterEle < 0)//if negative
					remainingCustafterEle = 0;
			cost_with_i = CumAtEle + remainingCustafterEle*oriloss; 			
			if (oriloss - available_dist_r2 > epsilon) //only consider r2
			{
				continue;
			}		
			if (cost_with_i > 99999)
			{
				cout<<"cost_with_i is too big, sth wrong"<<endl;
				getchar();
			}		
			//oriloss = 5, oriloss = 10, oriloss = -20, the best is -20, current best value bestGain[0]= -15
			if (cost_with_i < bestGain[0])//gain < bestgain
			{
				bestGain[0] = cost_with_i;   //gain in cost //minus because in function insert, route_cost - gain
				bestGain[1] = oriloss; //gain in distance
				bestGain[2] = m;//route number
				bestGain[3] = n;//which position
				foundbestGain=true;						
			}				
		}//end of n
		if (foundbestGain == true)
		{
			exceed<<"Found bestGain for cust with capacity exceed"<<cust<<endl;
			//if found a place to insert, just insert noneed to do the rest
			affectedR[(int) bestGain[2]] = true;
			gainvec[0] = 1;//status yes
			gainvec[1] = -bestGain[0];//in function it is minus
			gainvec[2] = -bestGain[1];//in function it is minus
			insert_one_cust (bestGain[2], bestGain[3], VEHICLE, cust); //1 in 
			AllInsert = true;
		
		}
		cout<<"Cust is  "<<cust<<endl;
		exceed<<"Cust is  "<<cust<<endl;
		cout<<"After insert exceedCapacity VEHICLE[][] "<<endl;
		exceed<<"After insert exceedCapacity VEHICLE[][] "<<endl;
		for (int i = 0; i < no_routes; i++)
		{
			cout<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<space_available[i]<<' '<<' ';
			exceed<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<space_available[i]<<' '<<' ';
			for (int j = 0; j <= saiz[i]+1; j++)
			{
				cout<<VEHICLE[i][j]<<' ';
				exceed	<<VEHICLE[i][j]<<' ';
			}
			cout<<endl;
			exceed<<endl;
		}
	//}//end of m
		delete[] bestGain;
}



//DONE
//done this two_opt on 2March2016
void two_opt_singleR(int **(&VEHICLE), int route) //used by DiversificationLNS
{
	std::vector<int> subvector; //to cpoy in reverse order if gain>0

findagain:
	
	for (int m = 1; m <= saiz[route]-1; m++)
	{
		for (int n = m + 1; n <= saiz[route]; n++)
		{
			int prevC = VEHICLE[route][m-1];
			int afterC= VEHICLE[route][n+1];
			int cStart = VEHICLE[route][m];//start of customer
			int cEnd = VEHICLE[route][n];//end of customer
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
			tempCumDist[0]=CumDist[route][m-1];
			int u=1;//for tempCumDist
				
			for (int t = n; t >= m; t--)//recalculate CumDist in reverse order
			{
				oldCumDist += CumDist[route][t];
				tempCumDist[u] = tempCumDist[u-1] + dist[prevC][VEHICLE[route][t]] + service_time[VEHICLE[route][t]];
				newCumDist += tempCumDist[u];
				prevC = VEHICLE[route][t];
				u++;
			}

			int remainingCust = saiz[route] - n;
			if (remainingCust <0)
				remainingCust =0;

			float gain = (oldCumDist - newCumDist) + remainingCust*origain;
				
			delete[] tempCumDist;
			
			
			if (gain > epsilon)
			{
				subvector.clear();
				//copy in reverse order
				for (int z = n; z >= m; z--)//push in reverse order
				{
					subvector.push_back(VEHICLE[route][z]);
				}

				//copy in vehicle[][]
				int a=0;//for subvector
				for (int z = m; z <= n; z++)
				{
					VEHICLE[route][z] = subvector[a];
					a++;
				}
				route_cost[route] -= gain;
				distance_cost[route] -= origain;
				int start = m;//the start of update point
				for (int j = start; j <=saiz[route]; j++)
				{
					CumDist[route][j] = CumDist[route][j-1]+dist[VEHICLE[route][j-1]][VEHICLE[route][j]] + service_time[VEHICLE[route][j]];
					
				}
				goto findagain; //route has been altered, need to redo 2-opt from the beginning //added on 1Mac2016
			}			
		}
	}
	distance_available[route] = DISTANCE - distance_cost[route]; //update distance_available for temp solution
}


//NOT DONE
void or_opt_singleR2(int **(&VEHICLE), int r) 
{
	//ofstream diversifyLNS("31.DiversifyLNS_" + std::to_string( I ) + ".txt", ios::app);

	int c1=-1, c2=-1, c3=-1, beforeF=-1, afterF=-1, beforeTo=-1, currTo=-1, insertPos=-1;
	int type;//record it is 1-0 or 2-0 or 3-0
	float newcost = 0;

redo:
	type=0;//initial value
	float ori_cost=route_cost[r];

	for (int j = 1; j <= saiz[r]; j++) //which one to delete (TAKE CUSTOMER)
	{				
		/////////////////////////////////////////////////////// 1 - 0 ///////////////////////////////////////////////////////////////
		for (int k = 1; k <= saiz[r]+1; k++) 		//where to insert for  1-0
		{
			
			if (k != j && k != j+1)//if from original pos or j+1, skip
			{
				//if ok, put j in position k
				beforeF = VEHICLE[r][j-1];
				afterF = VEHICLE[r][j+1];

				beforeTo = VEHICLE[r][k-1];
				currTo = VEHICLE[r][k];

				c1 =  VEHICLE[r][j];

				if (beforeTo == SIZE || currTo == SIZE)  //if the insertion is between customer and depot //insertion can be at the beginning or end of route
				{
					if ( NR_FLAG_DEPOT[c1][beforeTo] == false && NR_FLAG_DEPOT[c1][currTo] == false ) //use NR_FLAG_DEPOT
						continue;
				}

				else //if not between customer and depot
				{	
					if ( NR_FLAG[c1][beforeTo] == false && NR_FLAG[c1][currTo] == false ) //if insertion cannot between cust before and cust after
						continue;
				}

				newcost = route_cost[r] - dist[beforeF][c1] - dist[afterF][c1] - dist[beforeTo][currTo] 
							+ dist[beforeF][afterF] + dist[beforeTo][c1] + dist[currTo][c1];
				if (newcost - ori_cost < -epsilon)
				{
					ori_cost = newcost;
					insertPos = k;
					type=1;
				}

			}
		}
		if (j>=saiz[r])
			goto check;
		/////////////////////////////////////////////////////// 2 - 0 ///////////////////////////////////////////////////////////////
		for (int k = 1; k <= saiz[r]+1; k++)	//where to insert for  2-0
		{
			if (k != j && k != j+1 && k != j+2)//if from original pos, j+1, j+2, skip
			{
				//if ok, put j in position k
				beforeF = VEHICLE[r][j-1];
				afterF = VEHICLE[r][j+2];

				beforeTo = VEHICLE[r][k-1];
				currTo = VEHICLE[r][k];

				c1 =  VEHICLE[r][j];
				c2 =  VEHICLE[r][j+1];

				if (beforeTo == SIZE || currTo == SIZE)  //if the insertion is between customer and depot //insertion can be at the beginning or end of route
				{
					if ( NR_FLAG_DEPOT[c1][beforeTo] == false && NR_FLAG_DEPOT[c2][currTo] == false )//use NR_FLAG_DEPOT
						continue;
				}

				else //if not between customer and depot
				{	
					if ( NR_FLAG[c1][beforeTo] == false && NR_FLAG[c2][currTo] == false ) //if insertion cannot between cust before and cust after
						continue;
				}
				newcost = route_cost[r] - dist[beforeF][c1] - dist[afterF][c2] - dist[beforeTo][currTo] 
							+ dist[beforeF][afterF] + dist[beforeTo][c1] + dist[currTo][c2];
				if (newcost - ori_cost < -epsilon)
				{
					ori_cost = newcost;
					insertPos = k;
					type=2;
				}

			}


		}
		if (j>=saiz[r]-1)
			goto check;
		/////////////////////////////////////////////////////// 3 - 0 ///////////////////////////////////////////////////////////////
		for (int k = 1; k <= saiz[r]+1; k++)	//where to insert for  3-0
		{
			if (k != j && k != j+1 && k != j+2 && k != j+3)//if from original pos, j+1, j+2, j+3 skip
			{
				//if ok, put j in position k
				beforeF = VEHICLE[r][j-1];
				afterF = VEHICLE[r][j+3];

				beforeTo = VEHICLE[r][k-1];
				currTo = VEHICLE[r][k];

				c1 =  VEHICLE[r][j];
				c2 =  VEHICLE[r][j+1];
				c3 =  VEHICLE[r][j+2];

				if (beforeTo == SIZE || currTo == SIZE)  //if the insertion is between customer and depot //insertion can be at the beginning or end of route
				{
					if ( NR_FLAG_DEPOT[c1][beforeTo] == false && NR_FLAG_DEPOT[c3][currTo] == false )//use NR_FLAG_DEPOT
						continue;
				}

				else //if not between customer and depot
				{	
					if ( NR_FLAG[c1][beforeTo] == false && NR_FLAG[c3][currTo] == false ) //if insertion cannot between cust before and cust after
						continue;
				}

				newcost = route_cost[r] - dist[beforeF][c1] - dist[afterF][c3] - dist[beforeTo][currTo] 
							+ dist[beforeF][afterF] + dist[beforeTo][c1] + dist[currTo][c3];
				if (newcost - ori_cost < -epsilon)
				{
					ori_cost = newcost;
					insertPos = k;
					type=3;
				}
			}

		}
		check:
		if (type != 0)
		{
			route_cost[r] = ori_cost;
			distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp solution
			//overwrite the route
			relocatesameR(VEHICLE, r, j, insertPos, type);
			goto redo;//once there is change, redo
		}

	}
}

//i is delete psoition, j is insert position, the positions are based on non modified route
//just relocate and edit VEHICLE, route_cost already changed in main function
//NOT DONE
void relocatesameR(int **(&VEHICLE), int r, int i, int j, int type)
{
	int start, end;

	if (i<j)
	{
		start = i;
		end = j;
	}

	else
	{
		start = j;
		end = i;
	}
	if (type == 1)
	{
		int ele = VEHICLE[r][i];
		int b=0; //for temp_r
		int* temp_r = new int[end-start+1];
	
		for (int v = start; v <= end; v++)
		{
			if (v == i) //to delete
			{
				continue; //dont copy the element, assume the element to be deleted //VEHICLE continue, temp_r not continue
			}
			else if (v==j)//to insert, after insert copy
			{
				temp_r[b] = ele;
				b++;// temp_r continue, VEHICLE not continue

				temp_r[b] = VEHICLE[r][v];
				b++;
			}
			else //if not insert or delete, just copy 
			{
				temp_r[b] = VEHICLE[r][v];
				b++;
			}
		}
		int e=0;//for temp_r[]
		for (int a = start; a <= end; a++)//copy to VEHILCE
		{
			VEHICLE[r][a] = temp_r[e];
			e++;
		}
		delete[] temp_r;
	}

	if (type==2)
	{
		end = end+1;//have one more because gonna delete 2
		int ele = VEHICLE[r][i];
		int ele2 = VEHICLE[r][i+1];
		int b=0; //for temp_r
		int* temp_r = new int[end-start+1];

		for (int v = start; v <= end; v++)
		{
			if (v == i || v == i+1) //to delete
			{
				continue; //dont copy the element, assume the element to be deleted //VEHICLE continue, temp_r not continue
			
			}
			else if (v==j)//to insert, after insert, copy
			{
				temp_r[b] = ele;
				b++;// temp_r continue, VEHICLE not continue
				temp_r[b] = ele2;
				b++;// temp_r continue, VEHICLE not continue

				temp_r[b] = VEHICLE[r][v];
				b++;

			}
			else //if not insert or delete, just copy 
			{
				temp_r[b] = VEHICLE[r][v];
				b++;
			}
		}
		int e=0;//for temp_r[]
		for (int a = start; a <= end; a++)//copy to VEHILCE
		{
			VEHICLE[r][a] = temp_r[e];
			e++;
		}
		delete[] temp_r;
	}
	if (type==3)
	{
		end = end+2;//have two  more because gonna delete 3
		int ele = VEHICLE[r][i];
		int ele2 = VEHICLE[r][i+1];
		int ele3 = VEHICLE[r][i+2];
		int b=0; //for temp_r
		int* temp_r = new int[end-start+1];
		
		for (int v = start; v <= end; v++)
		{
			if (v == i || v == i+1 || v == i+2) //to delete
			{
				continue; //dont copy the element, assume the element to be deleted//VEHICLE continue, temp_r not continue

			}
			else if (v==j)//to insert, after insert, copy		
			{
				temp_r[b] = ele;
				b++;// temp_r continue, VEHICLE not continue
				temp_r[b] = ele2;
				b++;// temp_r continue, VEHICLE not continue
				temp_r[b] = ele3;
				b++;// temp_r continue, VEHICLE not continue

				temp_r[b] = VEHICLE[r][v];
				b++;
			}
			else //if not insert or delete, just copy 
			{
				temp_r[b] = VEHICLE[r][v];
				b++;

			}
		}
		int e=0;//for temp_r[]
		for (int a = start; a <= end; a++)//copy to VEHILCE
		{
			VEHICLE[r][a] = temp_r[e];
			e++;
		}
		delete[] temp_r;
	}

}


//same but shorter version of greedyInsertion
void greedyInsertion2(int **(&VEHICLE), int numcustRemoved, int* custRemoved)//changed on 17Feb2016
{
	ofstream greedy("32.Greedy insertion_.txt");
	//ofstream perturb("32.Perturb.txt");
	greedy<<"Just in greedy, havent done any"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		greedy<< i <<' '<< route_cost[i] <<' '<< saiz[i] <<' '<< total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			greedy<<VEHICLE[i][j]<<' ';
		}
		greedy<<endl;
	}
	int original_num_route = no_routes;

	//================ Step 1: record the cheapest insert for customer based on existing number of routes =========================//
	int numExtraR = 0;
	int ori_routes = no_routes; //for deletion later because no_routs may increase
	cust2 **cheapInsert = new cust2* [no_routes+numExtraR];//row //put 8 extra route as reserve space, no_routes can be more if add empty route
	for (int i = 0; i < no_routes+numExtraR; i++)
	{
		cheapInsert[i] = new cust2 [numcustRemoved];//column
	}
	//cheapInsert[][] will record every route's and every position's cost for each customer in custRemoved. 
	//if customer k cannot be inserted in route i, position j, then cheapInsert[i][j].costInsert = INT_MAX
	//everytime the lowest cost of insertion is selected from cheapInsert[][]
	//cheapInsert[i][j] =====> [i]=Route i, [j]=cust position in custRemoved
	for (int i = 0; i < no_routes; i++)
	{
		for (int j = 0; j < numcustRemoved; j++)
		{
			cheapInsert[i][j].custID = custRemoved[j];//the customer number
			cheapInsert[i][j].costInsert = INT_MAX;
			cheapInsert[i][j].distInsert = INT_MAX;
			cheapInsert[i][j].bestPos = -1;	
		}	
	}

	bool *insertStatus = new bool[numcustRemoved];

	for (int i = 0; i < numcustRemoved; i++) 
	{
		insertStatus[i] = false; //initially all false
		
	}
	
	//=========== First iteration, best insertion =============//
	for (int k = 0; k < numcustRemoved; k++)
	{	
		int customer = custRemoved[k];
		for (int i = 0; i < no_routes; i++)
		{
			if (demand[customer] > space_available[i]) 
				continue; //if route already full
			for (int j = 1; j <= saiz[i]+1; j++) //insertion is more than 1 of the saiz
			{
				int cust_before = VEHICLE[i][j-1];
				int cust_after = VEHICLE[i][j]; //insert at current position
				//========== just for checking
				if (cust_before == customer || cust_after ==customer)
				{
					cout<<"customer= "<<customer<<" in greedy insert"<<endl;
					getchar();
				}
				if (cust_before == -1 || cust_after==-1 )
				{
					for (int y = 0; y < no_routes ;y++)
					{
						for (int z = 0; z <= saiz[y]+1; z++)
						{
							cout<<VEHICLE[y][z]<<' ';
						}
						cout<<endl;
					}
					cout<<"cust= "<<customer<<"custbefore= "<<cust_before<<' '<<"custafter= "<<cust_after<<" in greedy insert"<<endl;
					getchar();	
				}
				//========== end just for checking
				float oriloss = dist[cust_before][customer] + dist[customer][cust_after] + service_time[customer] - dist[cust_before][cust_after]; //cost of r2 with i //(new+service_time – old)
		
				float CumAtEle = CumDist[i][j-1] + dist[cust_before][customer] + service_time[customer]; 

				int remainingCustafterEle = saiz[i] - j +1; //different from sameroute
				if (remainingCustafterEle < 0)//if negative
						remainingCustafterEle = 0;
				float cost_insert = CumAtEle + remainingCustafterEle*oriloss; 
				
	
				if (oriloss - distance_available[i] > epsilon) 
				{
					continue;
				}

				if (cost_insert < cheapInsert[i][k].costInsert) 
				{	
					cheapInsert[i][k].costInsert = cost_insert;
					cheapInsert[i][k].distInsert = oriloss;
					cheapInsert[i][k].bestPos = j;
					cheapInsert[i][k].custID = customer;
				}//end if cost < cheapInsert.cost
			}//end j
		}//end i
	}//end k

	affectedR = new bool [no_routes+numExtraR];//to flag which route has been changed after perturb
	for (int p = 0; p < no_routes+numExtraR; p++)
	{
		affectedR[p] = false;//initialize route not changed
	}
	
	int numcustInserted=0;
	
	//**********************************************************************************************************//
	//*************************** while customer not finished *****************************//
	while (numcustInserted < numcustRemoved)
	{
		//============== Find the lowest cost_insert among all customers and routes ==========//
		
		int bestR = -1, bestPos = -1, currentid = -1; //these values will be returned from foundleastCoststatus
		float bestCost=0, bestDist=0;
		//find the overall lowest value
		bool foundleastCoststatus = findbestinsertion(numcustRemoved, insertStatus, cheapInsert, custRemoved, currentid, bestR, bestPos, bestCost, bestDist);
		int currentcust = -1;

		if (foundleastCoststatus == true)
		{
			currentcust = custRemoved[currentid];
			gainvec[0] = 1;//status yes
			gainvec[1] = -bestCost;//in function it is minus
			gainvec[2] = -bestDist;//in function it is minus
			insert_one_cust (bestR, bestPos, VEHICLE, currentcust);
			greedy<<"After insert_one_cust"<<endl;
				for (int i = 0; i < no_routes; i++)
				{
					greedy<< i <<' '<< route_cost[i] <<' '<< saiz[i] <<' '<< total_demand[i]<<' '<<' ';
					for (int j = 0; j <= saiz[i]+1; j++)
					{
						greedy<<VEHICLE[i][j]<<' ';
					}
					greedy<<endl;
				}
			insertStatus[currentid] = true;//after insert, status=1
			numcustInserted++;
			affectedR[bestR] = true;
			ReevaluateAffectedR (numcustRemoved, insertStatus, cheapInsert, custRemoved);
		}
		
		else//if not found least cost insertion, need to do pertub, VND and/or add empty route
		{
			//need to find which one is not inserted
			for (int k = 0; k < numcustRemoved; k++)
			{
				if(insertStatus[k] == false)
				{
					currentid = k;///////////////////////////////////////////////
					currentcust = custRemoved[k]; //////////////////////////////////////////////
				}
			}

			//if this customer hasnt gone through perturb
			//if (perturbStatus[currentid] == false) //if this customer hasnt gone through perturb
			//{
			//	greedy<<endl<<"Cannot find a place for cust "<<currentcust<<" Entering perturb1_1 "<<endl;
			//	int pos = -1;
			//	float bestCost=0, bestDist=0;//return from pertub function so that insert_one_cust noneed to recompute
			//	int route = pertub1_1 (VEHICLE, currentcust, pos, bestCost, bestDist);//affectedR[] is flagged in pertub function
			//	perturbStatus[currentid] = true; //if done perturb
			//	
			//	if (route != -1)//perturb is success
			//	{
			//		gainvec[0] = 1;//status yes
			//		gainvec[1] = bestCost;
			//		gainvec[2] = bestDist;
			//		insert_one_cust (route, pos, VEHICLE, currentcust);
			//		insertStatus[currentid] = true;//after insert, status=1
			//		numcustInserted++;
			//		ReevaluateAffectedR (numcustRemoved, insertStatus, cheapInsert, custRemoved);
			//		continue;//see if customer already finish inserting
			//	}
			//}

			//if (VNDStatus[currentid] == false) //if this customer hasnt gone through VND
			//{
			//	greedy<<endl<<"Cannot find a place for cust "<<currentcust<<" Entering miniVND "<<endl;
			//	bool overallchange = miniVND(VEHICLE, affectedR);
			//	VNDStatus[currentid] = true;//if done VND

			//	if (overallchange == true)//if there is changes from VND, need to recalculate bestinsertion for the affected routes
			//	{
			//		ReevaluateAffectedR (numcustRemoved, insertStatus, cheapInsert, custRemoved);
			//		continue;
			//	}
			//}
			//
			//if (VNDStatus[currentid] == true)//if done  VND
			//{
				cout<<endl<<"Cannot find a place for cust "<<currentcust<<" Perform full perturb "<<endl;
				greedy<<endl<<"Cannot find a place for cust "<<currentcust<<" Perform full perturb "<<endl;
				//******************** CANNOT ADD NEW ROUTE, THINK OF THIS ***************************//
				bool status = fullPerturb (VEHICLE, currentcust);
				if (status == false)//if full perturb not success
				{
					//copy the best solution as diversfication ///MUST THINK OF A WAY TO CHANGE THIS, THIS IS THE LAST THING I WAN TO SEE!!!!!!!!!!!!!!
					for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
					{
						for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
						{
							VEHICLE[i][j] = GLOBAL[i][j];
							CumDist[i][j] = GLOBALCumDist[i][j];
						}
						saiz[i] = GLOBAL_SAIZ[i];
						route_cost[i] = GLOBAL_Rcost[i];
						distance_cost[i] = GLOBAL_distance_cost[i];
						total_demand[i] = GLOBAL_capa[i];
						space_available[i] = CAPACITY - total_demand[i];
						distance_available[i] = DISTANCE - distance_cost[i];

					}
					goto fullperturbNotsucess;
				}
				greedy<<"After fullperturb"<<endl;
				for (int i = 0; i < no_routes; i++)
				{
					greedy<< i <<' '<< route_cost[i] <<' '<< saiz[i] <<' '<< total_demand[i]<<' '<<' ';
					for (int j = 0; j <= saiz[i]+1; j++)
					{
						greedy<<VEHICLE[i][j]<<' ';
					}
					greedy<<endl;
				}
				insertStatus[currentid] = true;//after insert, status=1
				numcustInserted++;
				ReevaluateAffectedR (numcustRemoved, insertStatus, cheapInsert, custRemoved);
			//}

		}//end if not found least cost insertion, need to do pertub, VND and/or add empty route
	} //while customer not finish
	//**********************************************************************************************************//

	fullperturbNotsucess:
	greedy <<endl<<"Before miniVND after customer finished"<<endl;
	float tcost=0;
	for (int i = 0; i < no_routes; i++)
	{
		greedy << i<< ' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		tcost+=route_cost[i];
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			greedy << VEHICLE[i][j]<<' ';
		}
		greedy<<endl;
	}
	greedy<<"tcost= "<<tcost<<endl;
	//bool overallchange and bool *routeChange have no function, just because the function alreday declared to return value and receive argument
	//bool *routeChange = new bool[no_routes];
	//bool overallchange = miniVND(VEHICLE, routeChange);
	//delete[] routeChange;

	
	greedy <<endl<<"Out of miniVND after customer finished"<<endl;
	tcost=0;
	for (int i = 0; i < no_routes; i++)
	{
		greedy << i<< ' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		tcost+=route_cost[i];
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			greedy << VEHICLE[i][j]<<' ';
		}
		greedy<<endl;
		if (total_demand[i] < 0)
		{
			cout<<"in miniVND"<<endl;
			getchar();
		}
	}
	greedy<<"tcost= "<<tcost<<endl;
	delete[] insertStatus;

	for (int i = 0; i < ori_routes+numExtraR; i++)//put 8 extra route as reserve space
	{
		delete[] cheapInsert[i]; 
	}
	delete[] cheapInsert;
	delete[] affectedR;
}

//DONE
bool findbestinsertion(int numcustRemoved, bool *insertStatus, cust2 **cheapInsert, int *custRemoved, int &id, int &bestR, int &bestPos, float &bestCost, float &bestDist)
{
	//============== Find the lowest cost_insert ==========//
	float lowestC = INT_MAX;

	for (int k = 0; k < numcustRemoved; k++)
	{
		if (insertStatus[k] == true)
		{
			continue; //status = 1 means already inserted
		}
		for (int i = 0; i < no_routes; i++)
		{
			if (cheapInsert[i][k].costInsert < lowestC) 
			{
				id = k;///////////////////////////////////////////////
				//currentcust = custRemoved[k]; //////////////////////////////////////////////
				lowestC = cheapInsert[i][k].costInsert;
				bestCost = cheapInsert[i][k].costInsert;
				bestDist = cheapInsert[i][k].distInsert;
				//bestwhichcust = k;//customer position in array custRemoved
				bestR = i;//the route to insert
				bestPos = cheapInsert[i][id].bestPos;
			}	
		}
	}
	
	if (bestR == -1)
		return false;//not found
	else
		return true;//found lest cost insertion
}

//DONE
void ReevaluateAffectedR (int numcustRemoved, bool *insertStatus, cust2 **(&cheapInsert), int *custRemoved)
{
	for (int i = 0; i < no_routes; i++)
	{
		if (affectedR[i] == false)
			continue;
		for (int k = 0; k < numcustRemoved; k++)
		{
			cheapInsert[i][k].costInsert = INT_MAX; //reinitialize because route have been changed
			cheapInsert[i][k].distInsert = INT_MAX;
		}
	}
	
	for (int i = 0; i < no_routes; i++)
	{
		if (affectedR[i] == false)
			continue;
		
		for (int k = 0; k < numcustRemoved; k++) 
		{	
			if (insertStatus[k] == true)
				continue; //status = 1 means already inserted
		
			int customer = custRemoved[k];
			
			if (demand[customer] > space_available[i]) 
				continue; //if route already full
			for (int j = 1; j <= saiz[i]+1; j++) //insertion is more than 1 of the saiz
			{
				int cust_before = VEHICLE[i][j-1];
				int cust_after = VEHICLE[i][j]; //insert at current position
				
				float oriloss = dist[cust_before][customer] + dist[customer][cust_after] + service_time[customer] - dist[cust_before][cust_after]; //cost of r2 with i //(new+service_time – old)
				float CumAtEle = CumDist[i][j-1] + dist[cust_before][customer] + service_time[customer]; 

				int remainingCustafterEle = saiz[i] - j +1; //different from sameroute
				if (remainingCustafterEle < 0)//if negative
						remainingCustafterEle = 0;
				float cost_insert = CumAtEle + remainingCustafterEle*oriloss; 
				

				if (oriloss - distance_available[i] > epsilon) 
				{
					continue;
				}

				if (cost_insert < cheapInsert[i][k].costInsert) 
				{
					cheapInsert[i][k].costInsert = cost_insert;
					cheapInsert[i][k].distInsert = oriloss;
					cheapInsert[i][k].custID = customer;
					cheapInsert[i][k].bestPos = j;	
				}
			}
		}
	}
	//after reevaluate, initialize 
	for (int p = 0; p < no_routes; p++)
	{
		affectedR[p] = false;//initialize route not changed
	}
}

void regretInsertion(int **(&VEHICLE), int numcustRemoved, int* custRemoved)//added on 25Feb2016
{
	ofstream regret("32.regret insertion_.txt");

	regret<<"Just in regret insertion, havent done any"<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		regret<< i <<' '<< route_cost[i] <<' '<< saiz[i] <<' '<< total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			regret<<VEHICLE[i][j]<<' ';
		}
		regret<<endl;
	}
	int original_num_route = no_routes;

	//================ Step 1: record the cheapest insert for customer based on existing number of routes =========================//
	int numExtraR = 0;
	int ori_routes = no_routes; //for deletion later because no_routs may increase
	cust2 **cheapInsert1 = new cust2* [no_routes+numExtraR];//row //put 8 extra route as reserve space, no_routes can be more if add empty route
	cust2 **cheapInsert2 = new cust2* [no_routes+numExtraR];//record the second best insertion
	for (int i = 0; i < no_routes+numExtraR; i++)
	{
		cheapInsert1[i] = new cust2 [numcustRemoved];//column
		cheapInsert2[i] = new cust2 [numcustRemoved];//column
	}
	//cheapInsert[][] will record every route's and every position's cost for each customer in custRemoved. 
	//if customer k cannot be inserted in route i, position j, then cheapInsert[i][j].costInsert = INT_MAX
	//everytime the lowest cost of insertion is selected from cheapInsert[][]
	//cheapInsert[i][j] =====> [i]=Route i, [j]=cust position in custRemoved
	for (int i = 0; i < no_routes; i++)
	{
		for (int j = 0; j < numcustRemoved; j++)
		{
			cheapInsert1[i][j].custID = custRemoved[j];//the customer number
			cheapInsert1[i][j].costInsert = INT_MAX;
			cheapInsert1[i][j].distInsert = INT_MAX;
			cheapInsert1[i][j].bestPos = -1;

			cheapInsert2[i][j].custID = custRemoved[j];//the customer number
			cheapInsert2[i][j].costInsert = INT_MAX;
			cheapInsert2[i][j].distInsert = INT_MAX;
			cheapInsert2[i][j].bestPos = -1;
		}	
	}

	bool *insertStatus = new bool[numcustRemoved];
	bool *perturbStatus = new bool[numcustRemoved];
	bool *VNDStatus = new bool [numcustRemoved];
	for (int i = 0; i < numcustRemoved; i++) 
	{
		insertStatus[i] = false; //initially all false
		perturbStatus[i] = false;
		VNDStatus[i] = false;
	}
	
	//=========== First iteration, find best and and second insertion =============//
	for (int k = 0; k < numcustRemoved; k++)
	{	
		int customer = custRemoved[k];
		for (int i = 0; i < no_routes; i++)
		{
			if (demand[customer] > space_available[i]) 
				continue; //if route already full
			for (int j = 1; j <= saiz[i]+1; j++) //insertion is more than 1 of the saiz
			{
				int cust_before = VEHICLE[i][j-1];
				int cust_after = VEHICLE[i][j]; //insert at current position
				//========== just for checking
				if (cust_before == customer || cust_after ==customer)
				{
					cout<<"customer= "<<customer<<" in regret insert"<<endl;
					getchar();
				}
				if (cust_before == -1 || cust_after==-1 )
				{
					for (int y = 0; y < no_routes ;y++)
					{
						for (int z = 0; z <= saiz[y]+1; z++)
						{
							cout<<VEHICLE[y][z]<<' ';
						}
						cout<<endl;
					}
					cout<<"cust= "<<customer<<"custbefore= "<<cust_before<<' '<<"custafter= "<<cust_after<<" in greedy insert"<<endl;
					getchar();	
				}
				//========== end just for checking
				
				float oriloss = dist[cust_before][customer] + dist[customer][cust_after] + service_time[customer] - dist[cust_before][cust_after]; //cost of r2 with i //(new+service_time – old)
				float CumAtEle = CumDist[i][j-1] + dist[cust_before][customer] + service_time[customer]; 

				int remainingCustafterEle = saiz[i] - j +1; //different from sameroute
				if (remainingCustafterEle < 0)//if negative
						remainingCustafterEle = 0;
				float cost_insert = CumAtEle + remainingCustafterEle*oriloss; 
				
				
				if (oriloss - distance_available[i] > epsilon) 
				{
					continue;
				}

				//if only found one best insertion, the cheapInsert2[i][k].costInsert remains very big value, regret will be very high, because this customer has only one insertion
				if (cost_insert < cheapInsert1[i][k].costInsert) 
				{	
					//copy the previous best as the second best now
					cheapInsert2[i][k].costInsert = cheapInsert1[i][k].costInsert;
					cheapInsert2[i][k].distInsert = cheapInsert1[i][k].distInsert;
					cheapInsert2[i][k].bestPos = cheapInsert1[i][k].bestPos;
					cheapInsert2[i][k].custID = cheapInsert1[i][k].custID;
					
					cheapInsert1[i][k].costInsert = cost_insert;
					cheapInsert1[i][k].distInsert = oriloss;
					cheapInsert1[i][k].bestPos = j;
					cheapInsert1[i][k].custID = customer;
				}//end if cost < cheapInsert.cost
			}//end j
		}//end i
	}//end k

	affectedR = new bool [no_routes+numExtraR];//to flag which route has been changed after perturb
	for (int p = 0; p < no_routes+numExtraR; p++)
	{
		affectedR[p] = false;//initialize route not changed
	}
	
	int numcustInserted=0;
	
	//**********************************************************************************************************//
	//*************************** while customer not finished *****************************//
	while (numcustInserted < numcustRemoved)
	{
		//============== Find the lowest cost_insert among all customers and routes ==========//
		
		int bestR = -1, bestPos = -1, currentid = -1; //these values will be returned from foundleastCoststatus
		float bestCost=0, bestDist=0;//return from findmaxRegret so that insert_one_cust noneed to recompute cost and dist
		bool foundmaxRegretstatus = findmaxRegret(numcustRemoved, insertStatus, cheapInsert1, cheapInsert2, custRemoved, currentid, bestR, bestPos, bestCost, bestDist);
		int currentcust = -1;

		if (foundmaxRegretstatus == true)
		{
			gainvec[0] = 1;//status yes
			gainvec[1] = -bestCost;//in function it is minus
			gainvec[2] = -bestDist;//in function it is minus
			currentcust = custRemoved[currentid];
			insert_one_cust (bestR, bestPos, VEHICLE, currentcust);
				regret<<"AFter insert_one_cust"<<endl;
			for (int i = 0; i < no_routes; i++)
			{
				regret<< i <<' '<< route_cost[i] <<' '<< saiz[i] <<' '<< total_demand[i]<<' '<<' ';
				for (int j = 0; j <= saiz[i]+1; j++)
				{
					regret<<VEHICLE[i][j]<<' ';
				}
				regret<<endl;
			}
			insertStatus[currentid] = true;//after insert, status=1
			numcustInserted++;
			affectedR[bestR] = true;
			ReevaluateAffectedRinsertions (numcustRemoved, insertStatus, cheapInsert1, cheapInsert2, custRemoved);
		}
		
		else//if not found max regret, need to do pertub, VND and/or add empty route
		{
			//need to find which one is not inserted
			for (int k = 0; k < numcustRemoved; k++)
			{
				if(insertStatus[k] == false)
				{
					currentid = k;///////////////////////////////////////////////
					currentcust = custRemoved[k]; //////////////////////////////////////////////
				}
			}

			////if this customer hasnt gone through perturb
			//if (perturbStatus[currentid] == false) //if this customer hasnt gone through perturb
			//{
			//	regret<<endl<<"Cannot find a place for cust "<<currentcust<<" Entering perturb1_1, number of cust left uninserted= "<< numcustRemoved-numcustInserted <<endl;
			//	int pos = -1;
			//	float bestCost=0, bestDist=0;//return from pertub so that insert_one_cust noneed to recompute cost and dist
			//	int route = pertub1_1 (VEHICLE, currentcust, pos, bestCost, bestDist);//affectedR[] is flagged in pertub function
			//	perturbStatus[currentid] = true; //if done perturb
			//	
			//	if (route != -1)//perturb is success
			//	{
			//		gainvec[0] = 1;//status yes
			//		gainvec[1] = -bestCost;//in function it is minus
			//		gainvec[2] = -bestDist;//in function it is minus
			//		insert_one_cust (route, pos, VEHICLE, currentcust);
			//		insertStatus[currentid] = true;//after insert, status=1
			//		numcustInserted++;
			//		ReevaluateAffectedRinsertions (numcustRemoved, insertStatus, cheapInsert1, cheapInsert2, custRemoved);
			//		continue;//see if customer already finish inserting
			//	}
			//}
			//if (VNDStatus[currentid] == false) //if this customer hasnt gone through VND
			//{
			//	regret<<endl<<"Cannot find a place for cust "<<currentcust<<" Entering miniVND, number of cust left uninserted= "<< numcustRemoved-numcustInserted <<endl;
			//	bool overallchange = miniVND(VEHICLE, affectedR);
			//	VNDStatus[currentid] = true;//if done VND
			//	if (overallchange == true)//if there is changes from VND, need to recalculate bestinsertion for the affected routes
			//	{
			//		ReevaluateAffectedRinsertions (numcustRemoved, insertStatus, cheapInsert1, cheapInsert2, custRemoved);
			//		continue;
			//	}
			//}
			//
			//if (perturbStatus[currentid] == true && VNDStatus[currentid] == true)//if done pertub and VND
			//{
				cout<<endl<<"Cannot find a place for cust "<<currentcust<<" Perform full perturb "<<endl;
				regret<<endl<<"Cannot find a place for cust "<<currentcust<<" Perform full perturb "<<endl;
				//******************** CANNOT ADD NEW ROUTE, THINK OF THIS ***************************//
				bool status = fullPerturb (VEHICLE, currentcust);
				if (status == false)//if full perturb not success
				{
					//copy the best solution as diversfication ///MUST THINK OF A WAY TO CHANGE THIS, THIS IS THE LAST THING I WAN TO SEE!!!!!!!!!!!!!!
					for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
					{
						for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
						{
							VEHICLE[i][j] = GLOBAL[i][j];
							CumDist[i][j] = GLOBALCumDist[i][j];
						}
						saiz[i] = GLOBAL_SAIZ[i];
						route_cost[i] = GLOBAL_Rcost[i];
						distance_cost[i] = GLOBAL_distance_cost[i];
						total_demand[i] = GLOBAL_capa[i];
						space_available[i] = CAPACITY - total_demand[i];
						distance_available[i] = DISTANCE - distance_cost[i];

					}
					goto fullperturbNotsucess;
				}

					regret<<"AFter full perturb"<<endl;
					for (int i = 0; i < no_routes; i++)
					{
						regret<< i <<' '<< route_cost[i] <<' '<< saiz[i] <<' '<< total_demand[i]<<' '<<' ';
						for (int j = 0; j <= saiz[i]+1; j++)
						{
							regret<<VEHICLE[i][j]<<' ';
						}
						regret<<endl;
					}
				insertStatus[currentid] = true;//after insert, status=1
				numcustInserted++;
				ReevaluateAffectedRinsertions (numcustRemoved, insertStatus, cheapInsert1, cheapInsert2, custRemoved);
			//}

		}//end if not found least cost insertion, need to do pertub, VND and/or add empty route
	} //while customer not finish
	//**********************************************************************************************************//

	fullperturbNotsucess:
	regret <<endl<<"Before miniVND after customer finished"<<endl;
	float tcost=0;
	for (int i = 0; i < no_routes; i++)
	{
		regret << i<< ' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		tcost+=route_cost[i];
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			regret << VEHICLE[i][j]<<' ';
		}
		regret<<endl;
	}
	regret<<"tcost= "<<tcost<<endl;
	//bool overallchange and bool *routeChange have no function, just because the function alreday declared to return value and receive argument
	//bool *routeChange = new bool[no_routes];
	//bool overallchange = miniVND(VEHICLE, routeChange);
	//delete[] routeChange;
	//miniVND(VEHICLE, routeChange);
	
	regret <<endl<<"Out of miniVND after customer finished"<<endl;
	tcost=0;
	for (int i = 0; i < no_routes; i++)
	{
		regret << i<< ' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		tcost+=route_cost[i];
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			regret << VEHICLE[i][j]<<' ';
		}
		regret<<endl;
		if (total_demand[i] < 0)
		{
			cout<<"in miniVND"<<endl;
			getchar();
		}
	}
	regret<<"tcost= "<<tcost<<endl;
	delete[] insertStatus;
	delete[] perturbStatus;
	delete[] VNDStatus;
	for (int i = 0; i < ori_routes+numExtraR; i++)
	{
		delete[] cheapInsert1[i] ;//column
		delete[] cheapInsert2[i];//column
	}
	delete[] cheapInsert1;//column
	delete[] cheapInsert2;//column
	delete[] affectedR;
	
}

//CHECK later bestCost = cheapInsert1 or cheapInsert2
bool findmaxRegret(int numcustRemoved, bool *insertStatus, cust2 **cheapInsert1, cust2 **cheapInsert2, int *custRemoved, int &id, int &bestR, int &bestPos, float &bestCost, float &bestDist)
{
	ofstream regret("32.regret insertion_.txt", ios::app);
	//============== Find the maximum regret ==========//
	float maxR = INT_MIN;

	//================== Find 1st best and 2nd best among all routes =============//
	float *firstbest = new float[numcustRemoved];
	int *firstbestRoute = new int[numcustRemoved];
	int *firstbestPos = new int[numcustRemoved];
	float *firstbestDist = new float[numcustRemoved];
	float *secondbest = new float[numcustRemoved];
	for (int i = 0; i < numcustRemoved; i++)
	{
		firstbest[i] = INT_MAX;
		secondbest[i] = INT_MAX;
	}
	for (int k = 0; k < numcustRemoved; k++)
	{
		if (insertStatus[k] == true)
		{
			continue; //status = 1 means already inserted
		}
		for (int i = 0; i < no_routes; i++)
		{
			if (cheapInsert1[i][k].costInsert < firstbest[k])
			{
				firstbest[k] = cheapInsert1[i][k].costInsert;
				firstbestRoute[k] = i;
				firstbestPos[k] = cheapInsert1[i][k].bestPos;
				firstbestDist[k] = cheapInsert1[i][k].distInsert;
			}
			if (cheapInsert2[i][k].costInsert < secondbest[k])
			{
				secondbest[k] = cheapInsert2[i][k].costInsert;
			}
		}
	}
	for (int k = 0; k < numcustRemoved; k++)
	{
		if (insertStatus[k] == true)
		{
			continue; //status = 1 means already inserted
		}
		
		float currRegret = secondbest[k]-firstbest[k];
			
		if (currRegret == 0)//if both costInsert is INT_MAX, means not found best insertion
			continue;
		if (currRegret > maxR) 
		{
			id = k;///////////////////////////////////////////////
			//currentcust = custRemoved[k]; //////////////////////////////////////////////
			maxR = currRegret;
			bestCost = firstbest[k];
			bestDist = firstbestDist[k];
			bestR = firstbestRoute[k];//the route to insert
			bestPos = firstbestPos[k];
		}	
		
	}

	//for (int k = 0; k < numcustRemoved; k++)
	//{
	//	if (insertStatus[k] == true)
	//	{
	//		continue; //status = 1 means already inserted
	//	}
	//	for (int i = 0; i < no_routes; i++)
	//	{
	//		float currRegret = cheapInsert2[i][k].costInsert-cheapInsert1[i][k].costInsert; //this is wrong, just consider current route regret, not overall, as aresult, always pick up the one with highest regret of a route only
	//		if (saiz[i] == 0) //if empty route, checking
	//		{
	//			regret<<cheapInsert2[i][k].costInsert<<' '<<cheapInsert1[i][k].costInsert<<endl;
	//		}
	//		if (currRegret == 0)//if both costInsert is INT_MAX, means not found best insertion
	//			continue;
	//		if (currRegret > maxR) 
	//		{
	//			id = k;///////////////////////////////////////////////
	//			//currentcust = custRemoved[k]; //////////////////////////////////////////////
	//			maxR = currRegret;
	//			bestCost = cheapInsert1[i][k].costInsert;
	//			bestDist = cheapInsert1[i][k].distInsert;
	//			//bestwhichcust = k;//customer position in array custRemoved
	//			bestR = i;//the route to insert
	//			bestPos = cheapInsert1[i][id].bestPos;
	//		}	
	//	}
	//}
	delete[] firstbest;
	delete[] firstbestRoute;
	delete[] firstbestPos;
	delete[] firstbestDist;
	delete[] secondbest;
	if (bestR == -1)
		return false;//not found
	else
		return true;//found lest cost insertion
}

void ReevaluateAffectedRinsertions (int numcustRemoved, bool *insertStatus, cust2 **(&cheapInsert1), cust2 **(&cheapInsert2), int *custRemoved)
{
	for (int i = 0; i < no_routes; i++)
	{
		if (affectedR[i] == false)
			continue;
		for (int k = 0; k < numcustRemoved; k++)
		{
			cheapInsert1[i][k].costInsert = INT_MAX; //reinitialize because route have been changed
			cheapInsert2[i][k].costInsert = INT_MAX; //reinitialize because route have been changed

			cheapInsert1[i][k].distInsert = INT_MAX; 
			cheapInsert2[i][k].distInsert = INT_MAX; 
		}
	}
	
	for (int i = 0; i < no_routes; i++)
	{
		if (affectedR[i] == false)
			continue;
		
		for (int k = 0; k < numcustRemoved; k++) 
		{	
			if (insertStatus[k] == true)
				continue; //status = 1 means already inserted
		
			int customer = custRemoved[k];
			
			if (demand[customer] > space_available[i]) 
				continue; //if route already full
			for (int j = 1; j <= saiz[i]+1; j++) //insertion is more than 1 of the saiz
			{
				int cust_before = VEHICLE[i][j-1];
				int cust_after = VEHICLE[i][j]; //insert at current position
	
				float oriloss = dist[cust_before][customer] + dist[customer][cust_after] + service_time[customer] - dist[cust_before][cust_after]; //cost of r2 with i //(new+service_time – old)
				float CumAtEle = CumDist[i][j-1] + dist[cust_before][customer] + service_time[customer]; 

				int remainingCustafterEle = saiz[i] - j +1; //different from sameroute
				if (remainingCustafterEle < 0)//if negative
						remainingCustafterEle = 0;
				float cost_insert = CumAtEle + remainingCustafterEle*oriloss; 
				
				

				if (oriloss - distance_available[i] > epsilon) 
				{
					continue;
				}

				//if only found one best insertion, the cheapInsert2[i][k].costInsert remains very big value, regret will be very high, because this customer has only one insertion
				if (cost_insert < cheapInsert1[i][k].costInsert)
				{
					//copy the previous best as the second best now
					cheapInsert2[i][k].costInsert = cheapInsert1[i][k].costInsert;
					cheapInsert2[i][k].distInsert = cheapInsert1[i][k].distInsert;
					cheapInsert2[i][k].bestPos = cheapInsert1[i][k].bestPos;
					cheapInsert2[i][k].custID = cheapInsert1[i][k].custID;
					
					cheapInsert1[i][k].costInsert = cost_insert;
					cheapInsert1[i][k].distInsert = oriloss;
					cheapInsert1[i][k].bestPos = j;
					cheapInsert1[i][k].custID = customer;
				}
			}
		}
	}
	//after reevaluate, initialize 
	for (int p = 0; p < no_routes; p++)
	{
		affectedR[p] = false;//initialize route not changed
	}
}