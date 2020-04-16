#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <string>
#include <vector>
#include <cstdlib>      // std::rand, std::srand
//#include "SectorConflict.h"
//#include "LocalSearch.h"
#include "DiversificationLNS.h"
#include "Diversification_overlapRoute.h"
//to sort and store index
#include <numeric>      // std::iota
#include <algorithm>    // std::sort
#include "Bestimprovement.h"// for inter_route_improvement();//after delete, improve route first, added on 15/06/2016, inter_route_improvement() is modified

template <typename Container>
struct compare_indirect_indexB //sort from largest to smallest
  {
  const Container& container;
  compare_indirect_indexB( const Container& container ): container( container ) { }
  bool operator () ( size_t lindex, size_t rindex ) const
    {
    return container[ lindex ] > container[ rindex ]; //in decreasing order
    }
  };
template <typename Container>
struct compare_indirect_indexS//sort from smallest to largest
  {
  const Container& container;
  compare_indirect_indexS( const Container& container ): container( container ) { }
  bool operator () ( size_t lindex, size_t rindex ) const
    {
    return container[ lindex ] < container[ rindex ]; //in decreasing order
    }
  };
using namespace std;

/*struct Point  
{   
    float x, y;  
};  */ 
      
// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(Point p, Point q, Point r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
       return true;
 
    return false;
}
 

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
    // See 10th slides from following link for derivation of the formula
    // http://www.dcs.gla.ac.uk/~pat/52233/slides/Geometry1x1.pdf
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);
 
    if (val == 0) return 0;  // colinear
 
    return (val > 0)? 1: 2; // clock or counterclock wise
}
 
// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
 
    // General case
    if (o1 != o2 && o3 != o4)
        return true;
 
    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
 
    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
 
    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
 
     // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
 
    return false; // Doesn't fall in any of the above cases
}




//overlapRoute find the number of overlap for each route and always delete the one with highest overlap but the problem is it delete all customers in the route until kappa is reached
float Diversification_overlapRoute(int**(&VEHICLE), int K, float *x, float *y)
{
	ofstream diversify_overlap("32.Diversifyoverlap_" + std::to_string( I ) + ".txt", ios::app);
	diversify_overlap<<"K= "<<K<<endl;
	diversify_overlap<<"In diversify_overlap, before doing anything"<<endl;
	
	for (int i = 0; i < no_routes; i++)
	{
		diversify_overlap<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversify_overlap<<VEHICLE[i][j]<<' ';
		}
		diversify_overlap<<endl;
	}
	diversify_overlap<<"========================================================="<<endl;
	int num_cust_remove = K; //initially remove 8% of customer, each time add (K/2)%

	//======================================== calculate intersect ===========================================//
	int *routeIntersect = new int[no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		routeIntersect[i] =0;//initially, no intersect for all routes
	}
	Point p1, p2, p3, p4;  

	for (int i = 0; i < no_routes; i++)
	{
		if (saiz[i] == 0) //added on 16/02/2016
			continue;
		for (int j = 0; j <=saiz[i]; j++)
		{
			int cust1 = VEHICLE[i][j];
			int cust2 = VEHICLE[i][j+1]; 
			p1.x = x[cust1];  
			p1.y = y[cust1];  
			p2.x = x[cust2]; 
			p2.y = y[cust2];   
			for (int m = i+1; m < no_routes; m++)
			{
				if (i==m || saiz[m]==0)
					continue;
				for (int n = 0; n <=saiz[m]; n++)
				{
					int cust3 = VEHICLE[m][n];
					int cust4 = VEHICLE[m][n+1]; 
					p3.x = x[cust3];   
					p3.y = y[cust3];  
					p4.x = x[cust4]; 
					p4.y = y[cust4]; 

					//================== added this on 17/02/2016 =======================//
					bool endPStatus = false;
					int nonEnd1, nonEnd2;  
					if (j==0 && n==0) //if they are endpoint arc
					{
						nonEnd1 = cust2; 
						nonEnd2 = cust4; 
						endPStatus = true;
					}
					else if (j==0 && n==saiz[m])
					{
						nonEnd1 = cust2; 
						nonEnd2 = cust3; 
						endPStatus = true;
					}
					else if (j==saiz[i] && n==0)
					{
						nonEnd1 = cust1; 
						nonEnd2 = cust4; 
						endPStatus = true;
					}

					else if (j==saiz[i] && n==saiz[m])
					{
						nonEnd1 = cust1; 
						nonEnd2 = cust3; 
						endPStatus = true;
					}

					if (endPStatus == true) //if they are endpoint arc
					{
						//the non-endpoint might lie on the same line
						//determine if angle to the customer is the same, if same, there is overlap (lie on the same line), if not, no overlap

						if (theta[nonEnd1] == theta[nonEnd2])
						{
							routeIntersect[i]++;
							routeIntersect[m]++;
						}
		
						goto skipIntersectTest;
	
					}
					//========================== end of added this on 17/02/2016 ================================//
      
					if( doIntersect(p1, p2, p3, p4))
					{
						routeIntersect[i]++;
						routeIntersect[m]++;
					}	
					skipIntersectTest:;
				}//end n
			}//end m		
		}//end j
	}//end i

	vector <float> data;
	for (int i = 0; i < no_routes; i++)
	{
		//made comment on 16/02/2016
		//routeIntersect[i] = routeIntersect[i] - ((no_routes -1)*4);//each edge from depot will be considered as intersect
		diversify_overlap<<"routeIntersect["<<i<<"]= "<< routeIntersect[i] <<endl;
		data.push_back (routeIntersect[i]);
	}

	//sort in descending order, route with highest number of overlap to lowest
	vector <size_t> indices( data.size(), 0 ); 
	iota( indices.begin(), indices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

	sort( indices.begin(), indices.end() , compare_indirect_indexB <decltype(data)> ( data ) ); //sort the data and index in decreasing order

	//============ start of randomize is the number of overlaps is the same ==================//
	//if the number of overlaps is the same for more than a route, randomize the order so that not the same route be selected
	int r=0;
	int prevOverlap = 0, currOverlap = 0, currOverlapfreq = 0;
	currOverlap = data[indices[0]];
	currOverlapfreq++;

	while (r <= no_routes-2)
	{
		prevOverlap = currOverlap;
		r++;
		currOverlap = data[indices[r]];

		if (currOverlap == prevOverlap)//if curr one equal to previous one
		{
			currOverlapfreq++;
		}
		
		else if (currOverlap != prevOverlap)//if curr one not equal to previous one
		{
			if (currOverlapfreq > 1 )//if the frequency is more than 1
			{
				int start = r - currOverlapfreq;
				int end = r-1;
				std::srand ( unsigned ( std::time(0) ) );
				std::vector<int> myvector;
				for (int i = start; i <= end; i++)
				{
					myvector.push_back(indices[i]);//push in the route index that has the same frequency
					diversify_overlap<<indices[i]<<' ';
				}
				diversify_overlap<<endl;
				std::random_shuffle ( myvector.begin(), myvector.end() );
				int j=0;//for myvector
				for (int i = start; i <= end; i++)
				{

					indices[i] = myvector[j];//copy back to indices after randomize
					diversify_overlap<<indices[i]<<' ';
					j++;
				}
				
				currOverlapfreq=0;//initialize the frequency to 0
				currOverlap = data[indices[r]];
				currOverlapfreq++;
				//r--;
				myvector.clear();
				myvector.shrink_to_fit();
			}
		}

		if (r == no_routes-2)//if the last route
		{
			diversify_overlap<<"I m the last "<<currOverlapfreq<<endl;
			currOverlap = data[indices[r]];

			if (currOverlap == prevOverlap)//if curr one equal to previous one
			{
				currOverlapfreq++;
				int start = r +2- currOverlapfreq;
				int end = r+1;
				std::srand ( unsigned ( std::time(0) ) );
				std::vector<int> myvector;
				for (int i = start; i <= end; i++)
				{
					myvector.push_back(indices[i]);//push in the route index that has the same frequency
					diversify_overlap<<indices[i]<<' ';
				}
				diversify_overlap<<endl;
				std::random_shuffle ( myvector.begin(), myvector.end() );
				int j=0;//for myvector
				for (int i = start; i <= end; i++)
				{

					indices[i] = myvector[j];//copy back to indices after randomize
					diversify_overlap<<indices[i]<<' ';
					j++;
				}
				myvector.clear();
				myvector.shrink_to_fit();
			}
			else
				continue;//last route has unique number
			
		}

	}
	//if there is number of overlap is zero, sort from the shortest route to the longest
	int hold=0;
	if (data[indices[0]] == 0)
	{
		for (int i = 0; i < no_routes-1; i++)
		{
			for (int j = 0; j < no_routes-1; j++)
			{
				if (saiz[indices[j]] > saiz[indices[j+1]])//sort from smallest to largest
				{
					hold = indices[j];
					indices[j] = indices[j+1];
					indices[j+1] = hold;	
				}
			}		
		}
	}
	diversify_overlap << "Sorted route index (from highest to lowest) number of overlapss "<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversify_overlap << "route= "<<indices[i] << ' ' << ' ' << ' '<<"overlap= "<<data[indices[i]]<<endl;
	}

	int *custRemoved = new int[SIZE];//to record customer removed
	int c=0;//record how many customer removed
	int i=0;//for indices
	int more_delete = num_cust_remove;
	bool *flag_delete;
	int *temp_r = new int [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later
	while (c < num_cust_remove)
	{
		int route = indices[i];
		if (saiz[route] <= more_delete) //remove all
		{
			for (int s = 1; s <= saiz[route]; s++)
			{
				custRemoved[c] = VEHICLE[route][s];
				if (s==1)
					VEHICLE[route][s] = SIZE;
				else
					VEHICLE[route][s] = -1;//reinitialize
				c++;
			}
			saiz[route] = 0;
			total_demand[route] = 0;
			route_cost[route] = 0.0;
			distance_cost[route] =  0.0;
			distance_available[route] = DISTANCE;
			space_available[route] = CAPACITY;
			more_delete = num_cust_remove - c;	
		}

		else//partial delete
		{
			flag_delete = new bool[saiz[route]+1];//flag_delete starts from [0]
			for (int i = 0; i <= saiz[route] ; i++)
			{
				flag_delete[i]=false;//initialize
			}
		
			flagpartial_delete (route, more_delete, flag_delete);

			// Delete customers from the route based on flagged
			temp_r[0] = SIZE; //first one is depot
			int t=1;//for temp_r[][] use
			for (int j = 1; j <= saiz[route]; j++)//make sure starts from 1 to saiz[i] because we dont want to delete the first and last one as depot
			{
				int cust = -1;
				if (flag_delete[j] == false)//if position is not flagged, copy the customer to temp_r[][]
				{
					temp_r[t] = VEHICLE[route][j];
					t++;
				}
				else if(flag_delete[j] == true)
				{
					//if position is flagged, dont copy to temp_r[][]
					cust = VEHICLE[route][j];
					custRemoved[c] = cust;
					c++;
				
					total_demand[route] = total_demand[route] - demand[cust]; //update total_demand for temp solution
					//delete_one_cust (distance, demand, r, j, vehicle);//cannot use this because after delete, the position changed, previously at [1] becomes [0]
				}
			}
			saiz[route] = t-1;//update size of route //must update outside the for loop because for loop use size[] as a termination criteria, dont update inside
			temp_r[t] = SIZE; //last one is depot

			//compute distance
			route_cost[route] = 0;//initialize
			distance_cost[route] = 0;//initialize
			CumDist[route][0]= 0;
			for (int j = 0; j <= saiz[route]; j++)
			{
				distance_cost[route] = distance_cost[route]+dist[temp_r[j]][temp_r[j+1]] + service_time[temp_r[j]];
				
			}
			for (int j = 0; j <= saiz[route]+1; j++) //check if the saiz has been updated in delete()!!!!!!!!!!!!!!!!!!!!!!!!!
			{
				VEHICLE[route][j] = temp_r[j];
				CumDist[route][j]=CumDist[route][j-1]+dist[temp_r[j-1]][temp_r[j]] + service_time[temp_r[j]];
			}

			space_available[r] = CAPACITY - total_demand[r] ; //update space_available for temp solution
			distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp solution
			
			//if ((saiz[r] != 0) || (saiz[r] != 1) )//skip if saiz=0 or saiz=1
			if (saiz[route]>=3)//changed on 28JUn2015
			{
				two_opt_singleR(VEHICLE, route); 
				//or_opt_singleR2(VEHICLE, route); 
			}
		}//end else (partial delete)
		i++;//goto next route with highest overlap
	}

	diversify_overlap<<"======================================================================== "<<endl;
	diversify_overlap<<"custRemoved[i] are "<<endl;
	//cout<<"custRemoved[i] are "<<endl;
	for (int i = 0; i < c; i++)
	{
		//cout<<custRemoved[i]<<' '<<' ';
		diversify_overlap<<custRemoved[i]<<' '<<' ';
	}
	//cout<<endl;
	diversify_overlap<<endl;
	diversify_overlap<<"number of cust removed= "<<c<<endl;


	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
	{
		cout<<"Perform greedyInsertion2"<<endl;
		diversify_overlap<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		cout<<"Perform regretInsertion"<<endl;
		diversify_overlap<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}

	int total_size=0;
	float t_cost=0.0;
	for (int i = 0; i < no_routes; i++)
	{
		total_size += saiz[i];
		t_cost += route_cost[i];
		diversify_overlap<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversify_overlap<<VEHICLE[i][j]<<' ';
		}
		diversify_overlap<<endl;
	}
	diversify_overlap<<"t_cost= "<<t_cost<<endl;
	//no need to copy to LOCAL_BEST_ROUTE because it will be copied in the original function

	if (total_size != SIZE)
	{
		cout<<"total_size is wrong in diversification overlap, cant continue"<<total_size<<endl;
		getchar();
	}

	delete[] custRemoved;

	return t_cost;
}

//Diversification_overlapRoute2 will delete the overlap arc and the neighbourhood of the customer that overlap
float Diversification_overlapRoute2(int**(&VEHICLE), int K, float *x, float *y)
{
	ofstream diversify_overlap2("32.Diversifyoverlap2_" + std::to_string( I ) + ".txt", ios::app);
	diversify_overlap2<<"K= "<<K<<endl;
	diversify_overlap2<<"In diversify_overlap, before doing anything"<<endl;
	
	for (int i = 0; i < no_routes; i++)
	{
		diversify_overlap2<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversify_overlap2<<VEHICLE[i][j]<<' ';
		}
		diversify_overlap2<<endl;
	}
	diversify_overlap2<<"========================================================="<<endl;
	int num_cust_remove = K; //initially remove 8% of customer, each time add (K/2)%
	//the following make it deifference orm the previous function
	bool *custoverlap = new bool[SIZE+1];//flag the customer if the arc is overlap, including depot but depot will never be deleted
	for (int i = 0; i <= SIZE; i++)
	{
		custoverlap[i] = false;
	}
	//flag the customer around the overlap customer to be true
	bool *custoverlap2 = new bool[SIZE+1];
	for (int i = 0; i <= SIZE; i++)//to flag the neighbouring customer of custoverlap, cannot used the custoverlap because the  neighbouring of neighbouring will be comnsidered
	{
		custoverlap2[i] = false;
	}
	//======================================== calculate intersect ===========================================//
	int *routeIntersect = new int[no_routes];
	for (int i = 0; i < no_routes; i++)
	{
		routeIntersect[i] =0;//initially, no intersect for all routes
	}
	Point p1, p2, p3, p4;  
	bool overlap = false;//to indicate if there is overlap
	//int num_overlap = 0;//to calculate num_overlap so that we know how many more to flagged
	int total_flag = 0;//total flag including overlapped custiomer and neighbouring customer
	for (int i = 0; i < no_routes; i++)
	{
		if (saiz[i] == 0) //added on 16/02/2016
			continue;
		for (int j = 0; j <=saiz[i]; j++)
		{
			int cust1 = VEHICLE[i][j];
			int cust2 = VEHICLE[i][j+1]; 
			p1.x = x[cust1];  
			p1.y = y[cust1];  
			p2.x = x[cust2]; 
			p2.y = y[cust2];   
			for (int m = i+1; m < no_routes; m++)
			{
				if (i==m || saiz[m]==0)
					continue;
				for (int n = 0; n <=saiz[m]; n++)
				{
					int cust3 = VEHICLE[m][n];
					int cust4 = VEHICLE[m][n+1]; 
					p3.x = x[cust3];   
					p3.y = y[cust3];  
					p4.x = x[cust4]; 
					p4.y = y[cust4]; 

					//================== added this on 17/02/2016 =======================//
					bool endPStatus = false;
					int nonEnd1, nonEnd2;  
					if (j==0 && n==0) //if they are endpoint arc
					{
						nonEnd1 = cust2; 
						nonEnd2 = cust4; 
						endPStatus = true;
					}
					else if (j==0 && n==saiz[m])
					{
						nonEnd1 = cust2; 
						nonEnd2 = cust3; 
						endPStatus = true;
					}
					else if (j==saiz[i] && n==0)
					{
						nonEnd1 = cust1; 
						nonEnd2 = cust4; 
						endPStatus = true;
					}

					else if (j==saiz[i] && n==saiz[m])
					{
						nonEnd1 = cust1; 
						nonEnd2 = cust3; 
						endPStatus = true;
					}

					if (endPStatus == true) //if they are endpoint arc
					{
						//the non-endpoint might lie on the same line
						//determine if angle to the customer is the same, if same, there is overlap (lie on the same line), if not, no overlap

						if (theta[nonEnd1] == theta[nonEnd2])
						{
							routeIntersect[i]++;
							routeIntersect[m]++;
							if (custoverlap[cust1] == false && cust1 != SIZE)
							{
								custoverlap[cust1] = true;//these customers involved in overlap arc
								total_flag++;
							}
							if(custoverlap[cust2] == false && cust2 != SIZE)
							{
								custoverlap[cust2] = true;//these customers involved in overlap arc
								total_flag++;
							}
							if(custoverlap[cust3] == false && cust3 != SIZE)
							{
								custoverlap[cust3] = true;//these customers involved in overlap arc
								total_flag++;
							}
							if(custoverlap[cust4] == false && cust4 != SIZE)
							{
								custoverlap[cust4] = true;//these customers involved in overlap arc
								total_flag++;
							}
							if(total_flag >= K)
								goto enoughK;
							//custoverlap[cust1] = custoverlap[cust2] = custoverlap[cust3] = custoverlap[cust4] = true;//these customers involved in overlap arc
							overlap = true;//indicate there is overlap
						}
		
						goto skipIntersectTest;
	
					}
					//========================== end of added this on 17/02/2016 ================================//
      
					if( doIntersect(p1, p2, p3, p4))
					{
						routeIntersect[i]++;
						routeIntersect[m]++;
						if (custoverlap[cust1] == false && cust1 != SIZE)
						{
							custoverlap[cust1] = true;//these customers involved in overlap arc
							total_flag++;
						}
						if(custoverlap[cust2] == false && cust2 != SIZE)
						{
							custoverlap[cust2] = true;//these customers involved in overlap arc
							total_flag++;
						}
						if(custoverlap[cust3] == false && cust3 != SIZE)
						{
							custoverlap[cust3] = true;//these customers involved in overlap arc
							total_flag++;
						}
						if(custoverlap[cust4] == false && cust4 != SIZE)
						{
							custoverlap[cust4] = true;//these customers involved in overlap arc
							total_flag++;
						}
						if(total_flag >= K)
							goto enoughK;
						//custoverlap[cust1] = custoverlap[cust2] = custoverlap[cust3] = custoverlap[cust4] = true;//these customers involved in overlap arc
						overlap = true;//indicate there is overlap
					}	
					skipIntersectTest:;
				}//end n
			}//end m		
		}//end j
	}//end i


	
	
	if (overlap == true && total_flag <= K)//if there is overlap and the number is less than K
	{
		std::vector<int> myvector;//contain the overlapped customer index
		//identify the overlap and randomize the order so that the neighbourhood can be flagged later
		for (int i = 0; i < SIZE; i++)
		{
			if (custoverlap[i] == false) //
				continue;
			
			myvector.push_back(i);//push in the customer index that have overlap so that we can randomize the order later so that not the same neighbourhodd to be selected if flag less than kappa
			//num_overlap++;
			//if (num_overlap >= K) //maybe the flag custoverlap have flagged more than K customers
			//{
			//	total_flag = num_overlap;
			//	goto enoughK;
			//}
		}
		cout<<"size of vector = "<<myvector.size();
		cout<<"total_flag = "<<total_flag<<endl;
		//randomize the order of the overlap customer index	
		std::srand ( unsigned ( std::time(NULL) ) );
		std::random_shuffle ( myvector.begin(), myvector.end() );
		
		//do not use total_flag in this condition here because total_flag++ is used in the for loop!!!!!!!!!!!!!!
		for (int i = 0; i < myvector.size(); i++)//for each overlapped cust, find the neighbouring cust
		{
			int cust = myvector[i];
			for (int j = 0; j < SIZE; j++)
			{
				if (NR_FLAG[cust][j] == false)//not neighbouring customer
					continue;
				if (custoverlap2[j] == false && custoverlap[j] == false)//means this is the first time flagged
				{
					custoverlap2[j] = true;
					total_flag++;//total number flagged
				}
				if (total_flag >= K)//enough flag now
					goto enoughK;
			}
		}
		myvector.clear();
		myvector.shrink_to_fit();
	}//end if there is overlap
enoughK:;
	
	diversify_overlap2<<"K is "<<K<<endl;
	diversify_overlap2<<"Total flag customer including overlap and neighbouring customer = "<<total_flag<<endl;
	srand ( time(NULL) ); //seed it
	while (total_flag < K)//if flagged customer less than K or there is no flag at all, we will flag the customer randomly
	{
		
		int rand_num = (rand() % (SIZE)); //generate random customer 

		if (custoverlap[rand_num] == false && custoverlap2[rand_num] == false)//if this has not been flagged
		{
			custoverlap[rand_num] = true;//flag it true
			total_flag++;
		}
	}

	int *custRemoved = new int[SIZE];//to record customer removed
	
	
	int **temp_r = new int* [SIZE]; //to delete customer use //initially temp_r[][] does not hold anything, it will hold the route after customer deleted later
	for (int i = 0; i < SIZE; i++)
	{
		temp_r[i] = new int [SIZE];
	}
	for (int i = 0; i < SIZE; i++)
	{
		diversify_overlap2<<custoverlap[i]<<' ';
	}
	diversify_overlap2<<endl;
	
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
			if (custoverlap[VEHICLE[r][j]] == false)//if position is not flagged, copy the customer to temp_r[][]
			{
				temp_r[r][t] = VEHICLE[r][j];
				t++;
			}
			else if(custoverlap[VEHICLE[r][j]] == true)
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

	diversify_overlap2<<"======================================================================== "<<endl;
	diversify_overlap2<<"custRemoved[i] are "<<endl;
	//cout<<"custRemoved[i] are "<<endl;
	for (int i = 0; i < c; i++)
	{
		//cout<<custRemoved[i]<<' '<<' ';
		diversify_overlap2<<custRemoved[i]<<' '<<' ';
	}
	//cout<<endl;
	diversify_overlap2<<endl;
	diversify_overlap2<<"number of cust removed= "<<c<<endl;
	diversify_overlap2<<"After removed customers "<<endl;
	for (int i = 0; i < no_routes; i++)
	{
		diversify_overlap2<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversify_overlap2<<VEHICLE[i][j]<<' ';
		}
		diversify_overlap2<<endl;
	}
	diversify_overlap2<<endl;

	srand(( unsigned )time(0));
	if (rand() % 2 ==0) //generate random customer
	{
		cout<<"Perform greedyInsertion2"<<endl;		
		diversify_overlap2<<"Perform greedyInsertion2"<<endl;
		greedyInsertion2(VEHICLE, c, custRemoved);
	}
	
	else
	{
		cout<<"Perform regretInsertion"<<endl;
		diversify_overlap2<<"Perform regretInsertion"<<endl;
		regretInsertion(VEHICLE, c, custRemoved);
	}

	int total_size=0;
	float t_cost=0.0;
	for (int i = 0; i < no_routes; i++)
	{
		total_size += saiz[i];
		t_cost += route_cost[i];
		diversify_overlap2<<i<<' '<<route_cost[i]<<' '<<saiz[i]<<' '<<total_demand[i]<<' '<<' ';
		for (int j = 0; j <= saiz[i]+1; j++)
		{
			diversify_overlap2<<VEHICLE[i][j]<<' ';
		}
		diversify_overlap2<<endl;
	}
	diversify_overlap2<<"t_cost= "<<t_cost<<endl;
	//no need to copy to LOCAL_BEST_ROUTE because it will be copied in the original function

	if (total_size != SIZE)
	{
		cout<<"total_size is wrong in diversification overlap2, cant continue"<<total_size<<endl;
		getchar();
	}
	for (int i = 0; i < SIZE; i++)
	{
		delete[] temp_r[i];
	}
	delete[] temp_r;

	delete[] custRemoved;
	delete[] custoverlap;
	delete[] custoverlap2;
	delete[] routeIntersect;
	return t_cost;
}

//this function only used by Diversification_overlapRoute
void flagpartial_delete (int route, int more_delete, bool *(&flag_delete))
{
	//ofstream diversify_overlap("32.Diversifyoverlap_" + std::to_string( I ) + ".txt", ios::app);

	float *ratio = new float[saiz[route]+1];
	int priorC=-1, nextC=-1;
	int Cust=-1;

	int k=0;//for ratio[k]//now cannot use ratio[Cust] because cust doesn not reach the end, we consider only single route
	for (int j = 1; j <= saiz[route]; j++)
	{
		priorC = VEHICLE[route][j-1];
		nextC = VEHICLE[route][j+1];
		Cust = VEHICLE[route][j];
		float gain_from_deletion = dist[priorC][Cust] + dist[Cust][nextC] - dist[priorC][nextC];
		ratio[k] = demand[Cust]/gain_from_deletion;
		k++;
	}
	
	//sort the index in ascending order
	vector <float> ration;

	for (int j = 0; j < saiz[route]; j++) //push in distance ratio because iota and sort only accept vector parameter
	{
		ration.push_back( ratio[j]);//ratio[] start from [1] but ration start from [0]
	}
	vector <size_t> Rindices( ration.size(), 0 ); 
	iota( Rindices.begin(), Rindices.end(), 0 );  // found in <numeric> //the index size is same as data size: [0][1][2][3]

	sort( Rindices.begin(), Rindices.end() , compare_indirect_indexS <decltype(ration)> ( ration ) ); //sort the data and index from smallest to highest
	
	//diversify_overlap<<"Ratio based on Position (start from 1 to saiz): "<<endl;
	//for (int j = 0; j < saiz[route]; j++)
	//{
	//	diversify_overlap<<ration[j]<<' ';
	//}
	//diversify_overlap<<endl;
	//diversify_overlap<<"Best Position to remove (sorted ratio smallest to highest): "<<endl;
	for (int j = 0; j < saiz[route]; j++)
	{
		Rindices[j] = Rindices[j]+1;//because Rindices[j] now start from [0] but we want to start from [1]
		//diversify_overlap<<Rindices[j]<<' ';
	}
	//diversify_overlap<<endl;

	//when remove customer, cannot use the position pre-recorded becaus the index changes after every customer removed
	int n=0;//for custRemoved []
	int i = 0;//traverse from bottom of the list because ration is sorted in descending order but we want to take the smallest ratio
	while (n < more_delete)
	{
		int index = Rindices[i];
		flag_delete[index] = true;//flag it true, so we know that is the position to delete later
		n++;//record how many cust flag, meaning how many to be removed later
		i++;
	}
	//for (int i = 1; i <= saiz[route]; i++)
	//{
	//	diversify_overlap<<flag_delete[i]<<' ';
	//}
	//diversify_overlap<<"number of flag= "<<n<<endl;
}






