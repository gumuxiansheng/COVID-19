#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include "LocalSearch.h"

using namespace std;
const float BIGC = 20;

void insert_one_cust (int route_number, int position, int **(&VEHICLE), int customer)
{
	//////////////////IMPORTANT: position entered this function starts from 0///////////////////////
	int r = route_number;
	float cost = route_cost[r];
	float distCost = distance_cost[r];
	int siz = saiz[r];
	int c = customer; //customer to be inserted

	if (c<0)
		cout << "customer to be deleted cannot be negative!!!" << endl;
	//VEHICLE in the format: [DEPOT 3 2 1 0 4 DEPOT]
	int cust_before = VEHICLE[r][position-1]; 
	int cust_after = VEHICLE[r][position]; //current cust in that position

	if (gainvec[0] == 1)
	{
		cost = cost - gainvec[1];
		distCost = distCost - gainvec[2];
	}

	else//the following is wrong, change later if status =0;
	{
		cost = cost + dist[cust_before][c] + dist[c][cust_after] + service_time[c] - dist[cust_before][cust_after]; //(new+service_time – old)
	}
	

	if (distCost > (DISTANCE + (0.1*DISTANCE)) || (demand[c] > space_available[r]))
	{
		cout<< "Route " << route_number << "insert 12distance or capacity is violated" <<endl;
		//getchar();
	}
	
	int i = siz+1; //start copy from the ending depot

	while ( (i >= (position)) && (i>0) )
	{
		VEHICLE[r][i+1] = VEHICLE[r][i];
		i--;
	}
	VEHICLE[r][i+1] = c;
	saiz[r] = siz+1; //update saiz for temp solution
	route_cost[r] = cost; //update route_cost for temp solution
	distance_cost[r] = distCost; //update distance_cost for temp solution
	total_demand[r] = total_demand[r] + demand[customer]; //update total_demand for temp solution
	space_available[r] = CAPACITY - total_demand[r] ; //update space_available for temp solution
	distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp solution
	
	//float r_cost = 0.0;//to check
	//for (int c = 0; c <= saiz[r]; c++)
	//{
	//	r_cost += dist[VEHICLE[r][c]][VEHICLE[r][c + 1]] + service_time[VEHICLE[r][c]];
	//}
	//
	//if (abs (r_cost-route_cost[r]) > 0.1)
	//{
	//	cout<<endl<<"In insert_one_cust!!!!!!!!!"<<' ';
	//	cout<<"route_cost["<<r<<"]= "<<route_cost[r]<<' '<<"r_cost= "<<r_cost<<endl;
	//	getchar();
	//}
	//update CumDist starting from the affecting pos until last cust
	int start = position;//the start of update point, always hold old or new hold whichever comes first
	for (int j = start; j <=saiz[r]; j++)
	{
		CumDist[r][j] = CumDist[r][j-1]+dist[VEHICLE[r][j-1]][VEHICLE[r][j]] + service_time[VEHICLE[r][j]];	
	}
	float checkR_cost=0;
	for (int j = 1; j <=saiz[r]; j++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[r][j];
	}	
	if (abs(route_cost[r]-checkR_cost) > BIGC)
	{
		cout<<r<<"  This cost is different from original "<<endl;
		cout<<route_cost[r]<<' '<<checkR_cost<<' ';
		getchar();
	}
}

void swap_in_oneCust (int route_number, int position, int **(&VEHICLE), int customer)
{
	int r = route_number;
	float cost = route_cost[r];
	float distCost = distance_cost[r];
	int siz = saiz[r];
	int c = customer; //customer to be swap in with the old cust
	
	if (c<0)
		cout << "customer to be deleted cannot be negative!!!" << endl;
	
	int custToRemove = VEHICLE[r][position];
	int cust_before = VEHICLE[r][position-1]; 
	int cust_after = VEHICLE[r][position+1]; 

	if (siz == 0)
	{
		cout<< "This is an empty route!, cannot perform 1-1" << endl;
		getchar();
	}

	if (gainvec[0] == 1)
	{
		cost = cost - gainvec[1];
		distCost = distCost - gainvec[2];
	}

	else//the following is wrong, change later if status =0;
	{
		cost = cost - dist[cust_before][custToRemove] - dist[custToRemove][cust_after] - service_time[custToRemove] + dist[cust_before][c] + dist[c][cust_after] + service_time[c];
	}
	

	if (distCost > (DISTANCE + (0.1*DISTANCE) ))
	{
		cout<< "Route " << route_number << "insert distance is violated" <<endl;
		getchar();
	}
	
	VEHICLE[r][position] = c;	
	//saiz[r] = siz; //update saiz for temp solution //it is still the same
	route_cost[r] = cost; //update route_cost for temp solution
	distance_cost[r] = distCost; //update distance_cost for temp solution
	total_demand[r] = total_demand[r] + demand[customer] - demand[custToRemove]; //update total_demand for temp solution
	space_available[r] = CAPACITY - total_demand[r] ; //update space_available for temp solution
	distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp solution
	//float r_cost = 0.0;//to check
	//for (int c = 0; c <= saiz[r]; c++)
	//{
	//	r_cost += dist[VEHICLE[r][c]][VEHICLE[r][c + 1]] + service_time[VEHICLE[r][c]];
	//}
	//
	//if (abs (r_cost-route_cost[r]) > 0.1)
	//{
	//	cout<<endl<<"In swap_in_oneCust!!!!!!!!!"<<' ';
	//	cout<<"route_cost["<<r<<"]= "<<route_cost[r]<<' '<<"r_cost= "<<r_cost<<endl;
	//	getchar();
	//}

	//update CumDist starting from the affecting pos until last cust
	int start = position;//the start of update point, always hold old or new hold whichever comes first
	for (int j = start; j <=saiz[r]; j++)
	{
		CumDist[r][j] = CumDist[r][j-1]+dist[VEHICLE[r][j-1]][VEHICLE[r][j]] + service_time[VEHICLE[r][j]];	
	}
	float checkR_cost=0;
	for (int j = 1; j <=saiz[r]; j++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[r][j];
	}	
	if (abs(route_cost[r]-checkR_cost) > BIGC)
	{
		cout<<r<<"  This cost is different from original "<<endl;
		cout<<route_cost[r]<<' '<<checkR_cost<<' ';
		getchar();
	}
}

void swap2_1_cust (int route_number, int position, int **(&VEHICLE), int customer) //2 cust out, 1 cust in
{
	int r = route_number;
	float cost = route_cost[r];
	float distCost = distance_cost[r];
	int siz = saiz[r];
	int c = customer; //customer to be swap in with the old cust
	
	if (c<0)
		cout << "customer to be deleted cannot be negative!!!" << endl;
	
	int custToRemove = VEHICLE[r][position];
	int cust2ToRemove = VEHICLE[r][position+1];
	int cust_before = VEHICLE[r][position-1]; 
	int cust_after = VEHICLE[r][position+2]; 

	if (siz == 0)
	{
		cout<< "This is an empty route!, cannot perform 2-1" << endl;
		getchar();
	}
	if (gainvec[0] == 1)
	{
		cost = cost - gainvec[1];
		distCost = distCost - gainvec[2];
	}

	else//the following is wrong, change later if status =0;
	{
		cost = cost - dist[cust_before][custToRemove] - dist[custToRemove][cust2ToRemove] - dist[cust2ToRemove][cust_after] - service_time[custToRemove] - service_time[cust2ToRemove]+ dist[cust_before][c] + dist[c][cust_after] + service_time[c];
	}
	
	
	if (distCost > (DISTANCE + (0.1*DISTANCE)) )
	{
		cout<< "Route " << route_number << "insert distance is violated" <<endl;
		getchar();
	}
	
	VEHICLE[r][position] = c;
	for (int i = position+1; i <= siz; i++)
	{
		VEHICLE[r][i] = VEHICLE[r][i+1];
	}

	VEHICLE[r][siz+1] = -1; //initialize last one -1, otherwise it is depot //optional
	saiz[r] = siz-1; //update saiz for temp solution
	route_cost[r] = cost; //update route_cost for temp solution
	distance_cost[r] = distCost; //update distance_cost for temp solution
	total_demand[r] = total_demand[r] - demand[custToRemove] -demand[cust2ToRemove] + demand[customer]; //update total_demand for temp solution
	space_available[r] = CAPACITY - total_demand[r] ; //update space_available for temp solution
	distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp solution
	//float r_cost = 0.0;//to check
	//for (int c = 0; c <= saiz[r]; c++)
	//{
	//	r_cost += dist[VEHICLE[r][c]][VEHICLE[r][c + 1]] + service_time[VEHICLE[r][c]];
	//}
	//
	//if (abs (r_cost-route_cost[r]) > 0.1)
	//{
	//	cout<<endl<<"In swap2_1_cust!!!!!!!!!"<<' ';
	//	cout<<"route_cost["<<r<<"]= "<<route_cost[r]<<' '<<"r_cost= "<<r_cost<<endl;
	//	getchar();
	//}
	//update CumDist starting from the affecting pos until last cust
	int start = position;//the start of update point, always hold old or new hold whichever comes first
	for (int j = start; j <=saiz[r]; j++)
	{
		CumDist[r][j] = CumDist[r][j-1]+dist[VEHICLE[r][j-1]][VEHICLE[r][j]] + service_time[VEHICLE[r][j]];	
	}
	float checkR_cost=0;
	for (int j = 1; j <=saiz[r]; j++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[r][j];
	}	
	if (abs(route_cost[r]-checkR_cost) > BIGC)
	{
		cout<<r<<"  This cost is different from original "<<endl;
		cout<<route_cost[r]<<' '<<checkR_cost<<' ';
		getchar();
	}
}


void swap1_2_cust (int route_number, int position, int **(&VEHICLE), int customer1, int customer2) //1 cust out, 2 cust in
{
	int r = route_number;
	float cost = route_cost[r];
	float distCost = distance_cost[r];
	int siz = saiz[r];
	int c1 = customer1; //customer1 to be swap in with the old cust
	int c2 = customer2; //customer2 to be swap in with the old cust
	
	if ((c1<0) || (c2<0))
		cout << "customer to be deleted cannot be negative!!!" << endl;
	
	int custToRemove = VEHICLE[r][position];
	int cust_before = VEHICLE[r][position-1]; 
	int cust_after = VEHICLE[r][position+1]; 

	if (siz == 0)
	{
		cout<< "This is an empty route!, cannot perform 2-1" << endl;
		getchar();
	}

	if (gainvec[0] == 1)
	{
		cost = cost - gainvec[1];
		distCost = distCost - gainvec[2];
	}

	else//the following is wrong, change later if status =0;
	{
		cost = cost - dist[cust_before][custToRemove] - dist[custToRemove][cust_after] - service_time[custToRemove] + dist[cust_before][c1] + dist[c1][c2] + dist[c2][cust_after] + service_time[c1] + service_time[c2];
	}
	

	if (distCost > (DISTANCE + (0.1*DISTANCE)))
	{
		cout<< "Route " << route_number << "insert distance is violated" <<endl;
		getchar();
	}
	
	for (int i = siz+2; i > position+1; i--)
	{
		VEHICLE[r][i] = VEHICLE[r][i-1];
	}
	VEHICLE[r][position] = c1;
	VEHICLE[r][position+1] = c2;

	saiz[r] = siz+1; //update saiz 
	route_cost[r] = cost; //update route_cost 
	distance_cost[r] = distCost; //update distance_cost for temp solution
	total_demand[r] = total_demand[r] - demand[custToRemove] + demand[c1] + demand[c2]; //update total_demand 
	space_available[r] = CAPACITY - total_demand[r] ; //update space_available 
	distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available 
	//float r_cost = 0.0;//to check
	//for (int c = 0; c <= saiz[r]; c++)
	//{
	//	r_cost += dist[VEHICLE[r][c]][VEHICLE[r][c + 1]] + service_time[VEHICLE[r][c]];
	//}
	//
	//if (abs (r_cost-route_cost[r]) > 0.1)
	//{
	//	cout<<endl<<"In swap1_2_cust!!!!!!!!!"<<' ';
	//	cout<<"route_cost["<<r<<"]= "<<route_cost[r]<<' '<<"r_cost= "<<r_cost<<endl;
	//	getchar();
	//}

	//update CumDist starting from the affecting pos until last cust
	int start = position;//the start of update point, always hold old or new hold whichever comes first
	for (int j = start; j <=saiz[r]; j++)
	{
		CumDist[r][j] = CumDist[r][j-1]+dist[VEHICLE[r][j-1]][VEHICLE[r][j]] + service_time[VEHICLE[r][j]];	
	}
	float checkR_cost=0;
	for (int j = 1; j <=saiz[r]; j++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[r][j];
	}	
	if (abs(route_cost[r]-checkR_cost) > BIGC)
	{
		cout<<r<<"  This cost is different from original "<<endl;
		cout<<route_cost[r]<<' '<<checkR_cost<<' ';
		getchar();
	}
}


void delete_one_cust (int route_number, int position, int **(&VEHICLE)) //make sure deletion cannot be made from the empty route
{
	int r = route_number;
	float cost = route_cost[r];
	float distCost = distance_cost[r];
	int siz = saiz[r];
	int c = VEHICLE[r][position]; //customer to be deleted 

	int cust_before = VEHICLE[r][position-1]; 
	int cust_after = VEHICLE[r][position+1]; 

	if (siz == 0)
	{
		cout<< "Error!! Deletion of one cannot be made from empty route" << endl;
		getchar();
	}
	
	if (siz == 1) //if only one customer, cost=0, if not the computer calculates the cost = very small number
	{
		cost = 0;
		distCost = 0;
	}
	else
	{
		if (gainvec[0] == 1)
		{
			cost = cost - gainvec[1];
			distCost = distCost - gainvec[2];
		}

		else//the following is wrong, change later if status =0;
		{
			cost = cost - dist[cust_before][c] - dist[c][cust_after] - service_time[c] + dist[cust_before][cust_after];
		}
	
	}

	if (distCost > (DISTANCE + (0.1*DISTANCE)))
	{
		cout<< "Route " << route_number << "distance is violated" <<endl;
		getchar();
	}

	for (int i = position; i<=siz; i++)
	{
		VEHICLE[r][i] = VEHICLE[r][i+1];
	}
	VEHICLE[r][siz+1] = -1;//optional
	saiz[r] = siz-1; //update saiz for temp solution
	route_cost[r] = cost; //update route_cost for temp solution
	distance_cost[r] = distCost; //update distance_cost for temp solution
	total_demand[r] = total_demand[r] - demand[c]; //update total_demand for temp solution
	space_available[r] = CAPACITY - total_demand[r] ; //update space_available for temp solution
	distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp solution
	//float r_cost = 0.0;//to check
	//for (int c = 0; c <= saiz[r]; c++)
	//{
	//	r_cost += dist[VEHICLE[r][c]][VEHICLE[r][c + 1]] + service_time[VEHICLE[r][c]];
	//}
	//
	//if (abs (r_cost-route_cost[r]) > 0.1)
	//{
	//	cout<<endl<<"In delete_one_cust!!!!!!!!!"<<' ';
	//	cout<<"route_cost["<<r<<"]= "<<route_cost[r]<<' '<<"r_cost= "<<r_cost<<endl;
	//	getchar();
	//}

	//update CumDist starting from the affecting pos until last cust
	int start = position;//the start of update point, always hold old or new hold whichever comes first
	for (int j = start; j <=saiz[r]; j++)
	{
		CumDist[r][j] = CumDist[r][j-1]+dist[VEHICLE[r][j-1]][VEHICLE[r][j]] + service_time[VEHICLE[r][j]];	
	}
	float checkR_cost=0;
	for (int j = 1; j <=saiz[r]; j++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[r][j];
	}	
	if (abs(route_cost[r]-checkR_cost) > BIGC)
	{
		cout<<r<<"  This cost is different from original "<<endl;
		cout<<route_cost[r]<<' '<<checkR_cost<<' ';
		getchar();
	}
}

void delete_two_cust (int route_number, int position, int **(&VEHICLE)) //make sure deletion cannot be made from the empty route
{
	int r = route_number;
	float cost = route_cost[r];
	float distCost = distance_cost[r];
	int siz = saiz[r];
	
	int c = VEHICLE[r][position]; //first customer to be deleted
	int c2 = VEHICLE[r][position+1]; //second customer to be deleted
	
	int cust_before = VEHICLE[r][position-1]; 
	int cust_after = VEHICLE[r][position+2]; 

	if ((siz == 0) || (siz == 1))
	{
		cout<< "Error!! Deletion of two cannot be made from empty route or route with one cust" << endl;
		getchar();
	}

	if (siz == 2) //if only two customer, cost=0, if not the computer calculates the cost = very small number
	{	
		cost = 0;
		distCost = 0;
	}
	else
	{
		if (gainvec[0] == 1)
		{
			cost = cost - gainvec[1];
			distCost = distCost - gainvec[2];
		}

		else//the following is wrong, change later if status =0;
		{
			cost = cost - dist[cust_before][c] - dist[c][c2] - dist[c2][cust_after] - service_time[c]- service_time[c2]+ dist[cust_before][cust_after];
		}
		
	}
	if (distCost > (DISTANCE + (0.1*DISTANCE)))
	{
		cout<< "Route " << route_number << "distance is violated" <<endl;
		getchar();
	}

	for (int i = position; i<=siz-1; i++)// cust 0-4, can delete 5 customers
	{
		VEHICLE[r][i] = VEHICLE[r][i+2];
	}
	VEHICLE[r][siz] = -1;//optional
	VEHICLE[r][siz+1] = -1;//optional

	saiz[r] = siz-2; //update saiz for temp solution
	route_cost[r] = cost; //update route_cost for temp solution
	distance_cost[r] = distCost; //update distance_cost for temp solution
	total_demand[r] = total_demand[r] - demand[c] - demand[c2]; //update total_demand for temp solution
	space_available[r] = CAPACITY - total_demand[r] ; //update space_available for temp solution
	distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp solution	
	//float r_cost = 0.0;//to check
	//for (int c = 0; c <= saiz[r]; c++)
	//{
	//	r_cost += dist[VEHICLE[r][c]][VEHICLE[r][c + 1]] + service_time[VEHICLE[r][c]];
	//}
	//
	//if (abs (r_cost-route_cost[r]) > 0.1)
	//{
	//	cout<<endl<<"In delete_two_cust!!!!!!!!!"<<' ';
	//	cout<<"route_cost["<<r<<"]= "<<route_cost[r]<<' '<<"r_cost= "<<r_cost<<endl;
	//	getchar();
	//}

	//update CumDist starting from the affecting pos until last cust
	int start = position;//the start of update point, always hold old or new hold whichever comes first
	for (int j = start; j <=saiz[r]; j++)
	{
		CumDist[r][j] = CumDist[r][j-1]+dist[VEHICLE[r][j-1]][VEHICLE[r][j]] + service_time[VEHICLE[r][j]];	
	}
	float checkR_cost=0;
	for (int j = 1; j <=saiz[r]; j++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[r][j];
	}	
	if (abs(route_cost[r]-checkR_cost) > BIGC)
	{
		cout<<r<<"  This cost is different from original "<<endl;
		cout<<route_cost[r]<<' '<<checkR_cost<<' ';
		getchar();
	}
}

void insert_two_cust (int route_number, int position, int **(&VEHICLE), int customer, int customer2)
{
	int r = route_number;
	float cost = route_cost[r];
	float distCost = distance_cost[r];
	int siz = saiz[r];
	int c = customer; //customer to be inserted
	int c2 = customer2;
	
	int cust_before = VEHICLE[r][position-1]; 
	int cust_after = VEHICLE[r][position]; //current cust in that position

	if (gainvec[0] == 1)
	{
		cost = cost - gainvec[1];
		distCost = distCost - gainvec[2];
	}

	else//the following is wrong, change later if status =0;
	{
		cost = cost + dist[cust_before][c] + dist[c][c2] + dist[c2][cust_after] + service_time[c] + service_time[c2]- dist[cust_before][cust_after];
	}

	
	if (distCost > (DISTANCE + (0.1*DISTANCE)))
	{
		cout<< "Route " << route_number << "distance is violated" <<endl;
		getchar();
	}
	
	int i = siz+1; 

	while ( i >= position && i>0 )
	{
		VEHICLE[r][i+2] = VEHICLE[r][i];
		i--;
	}	
	VEHICLE[r][position] = c;
	VEHICLE[r][position+1] = c2;

	saiz[r] = siz+2; //update saiz for temp solution
	route_cost[r] = cost; //update route_cost for temp solution
	distance_cost[r] = distCost; //update distance_cost for temp solution
	total_demand[r] = total_demand[r] + demand[c] + demand[c2]; //update total_demand for temp solution
	space_available[r] = CAPACITY - total_demand[r] ; //update space_available for temp solution
	distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp solution

	//float r_cost = 0.0;//to check
	//for (int c = 0; c <= saiz[r]; c++)
	//{
	//	r_cost += dist[VEHICLE[r][c]][VEHICLE[r][c + 1]] + service_time[VEHICLE[r][c]];
	//}
	//
	//if (abs (r_cost-route_cost[r]) > 0.1)
	//{
	//	cout<<endl<<"In insert_two_cust!!!!!!!!!"<<' ';
	//	cout<<"route_cost["<<r<<"]= "<<route_cost[r]<<' '<<"r_cost= "<<r_cost<<endl;
	//	getchar();
	//}
	//update CumDist starting from the affecting pos until last cust
	int start = position;//the start of update point, always hold old or new hold whichever comes first
	for (int j = start; j <=saiz[r]; j++)
	{
		CumDist[r][j] = CumDist[r][j-1]+dist[VEHICLE[r][j-1]][VEHICLE[r][j]] + service_time[VEHICLE[r][j]];	
	}
	float checkR_cost=0;
	for (int j = 1; j <=saiz[r]; j++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[r][j];
	}	
	if (abs(route_cost[r]-checkR_cost) > BIGC)
	{
		cout<<r<<"  This cost is different from original "<<endl;
		cout<<route_cost[r]<<' '<<checkR_cost<<' ';
		getchar();
	}
}

void swap_in_twoCust (int route_number, int position, int **(&VEHICLE), int customer1, int customer2)
{
	int r = route_number;
	float cost = route_cost[r];
	float distCost = distance_cost[r];
	int siz = saiz[r];
	int c1 = customer1; //customer to be swap in with the old cust
	int c2 = customer2; //customer to be swap in with the old cust
	
	if ((c1<0) || (c2<0))
	{
		cout << "customer to be deleted cannot be negative!!!" << endl;
		getchar();
	}
	
	int custToRemove1 = VEHICLE[r][position];
	int custToRemove2 = VEHICLE[r][position+1];
	int cust_before = VEHICLE[r][position-1]; 
	int cust_after = VEHICLE[r][position+2]; 

	if (siz <= 1)
	{
		cout<< "This is an empty or one cust route !, cannot perform 2-2" << endl;
		getchar();
	}
	
	if (gainvec[0] == 1)
	{
		cost = cost - gainvec[1];
		distCost = distCost - gainvec[2];
	}

	else//the following is wrong, change later if status =0;
	{
		cost = cost - dist[cust_before][custToRemove1] - dist[custToRemove1][custToRemove2] - dist[custToRemove2][cust_after] - service_time[custToRemove1] - service_time[custToRemove2]+ dist[cust_before][c1] + dist[c1][c2] + dist[c2][cust_after] + service_time[c1] + service_time[c2];
	}
	

	if (distCost > (DISTANCE + (0.1*DISTANCE)))
	{
		cout<< "Route " << route_number << "insert distance is violated" <<endl;
		getchar();
	}
	VEHICLE[r][position] = c1;
	VEHICLE[r][position+1] = c2;
	//saiz[r] = siz; //update saiz for temp solution //it is still the same
	route_cost[r] = cost; //update route_cost for temp solution
	distance_cost[r] = distCost; //update distance_cost for temp solution
	total_demand[r] = total_demand[r] + demand[customer1] + demand[customer2] - demand[custToRemove1]- demand[custToRemove2]; //update total_demand for temp solution
	space_available[r] = CAPACITY - total_demand[r] ; //update space_available for temp solution
	distance_available[r] = DISTANCE - distance_cost[r]; //update distance_available for temp solution

	//float r_cost = 0.0;//to check
	//for (int c = 0; c <= saiz[r]; c++)
	//{
	//	r_cost += dist[VEHICLE[r][c]][VEHICLE[r][c + 1]] + service_time[VEHICLE[r][c]];
	//}
	//
	//if (abs (r_cost-route_cost[r]) > 0.1)
	//{
	//	cout<<endl<<"In swap_in_twoCust!!!!!!!!!"<<' ';
	//	cout<<"route_cost["<<r<<"]= "<<route_cost[r]<<' '<<"r_cost= "<<r_cost<<endl;
	//	getchar();
	//}
	//update CumDist starting from the affecting pos until last cust
	int start = position;//the start of update point, always hold old or new hold whichever comes first
	for (int j = start; j <=saiz[r]; j++)
	{
		CumDist[r][j] = CumDist[r][j-1]+dist[VEHICLE[r][j-1]][VEHICLE[r][j]] + service_time[VEHICLE[r][j]];	
	}
	
	float checkR_cost=0;
	for (int j = 1; j <=saiz[r]; j++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[r][j];
	}	
	if (abs(route_cost[r]-checkR_cost) > BIGC)
	{
		cout<<r<<"  This cost is different from original "<<endl;
		cout<<route_cost[r]<<' '<<checkR_cost<<' ';
		getchar();
	}

}

void reverseASegment (int route_number, int position, int L1, int **(&VEHICLE))//added 11June2016 for new shake_intraReverseSegment 
{
	
	std::vector<int> subvector; //for reverse order

	//float ori_cost=0.0, temp_cost=0.0;
	int i = route_number;//route number
	int m = position; //eleStart
	int n = position+L1-1; //eleEnd

	
	//copy the subroute in subvector in reverse order, noneed to reverse again later
	for (int z = n; z >= m; z--)
	{
		subvector.push_back(VEHICLE[i][z]);

	}

	int s=0;//for subvector
	for (unsigned z = m; z <= n; z++)
	{
		VEHICLE[i][z] = subvector[s];  //copy to VEHICLE 
		s++;
	}
	if (gainvec[0] == 1)
	{
		route_cost[i] = route_cost[i] - gainvec[1];
		distance_cost[i] = distance_cost[i] - gainvec[2];
	}
	distance_available[i] = DISTANCE - distance_cost[i]; //update distance_available for temp solution
	
	
	float check_cost= 0.0; //initialize
	for (int c = 0; c < saiz[i] + 1; c++)
	{
		check_cost+= dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
	}
	if (abs(check_cost-distance_cost[i]) > 1.5)
	{
		cout<<"Cost diff in reverseASegment"<<endl;
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

	
	
	float checkR_cost=0;
	for (int j = 1; j <=saiz[i]; j++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[i][j];
	}	
	if (abs(route_cost[i]-checkR_cost) > BIGC)
	{
		cout<<i<<"  This cost is different from original in reverseASegment"<<endl;
		cout<<route_cost[i]<<' '<<checkR_cost<<' ';
		getchar();
	}

}

void cross_insert_fromshaking (int L1, int L2, int r1, int r2, int r1p, int r2p, int **(&VEHICLE), int reverseType)
{
	std::vector<int> myvector1;
	std::vector<int> myvector2; 
	int i = r1;//r1
	int m = r2;//r2
	int j = r1p; //eleStart in r1
	int n = r2p; //eleStart in r2

	//cout<<"Original R1"<<endl;
	//cout<<"start= "<<j<<' '<<"length= "<<L1<<endl;
	//for (int k = 0; k <=saiz[i]; k++)
	//{
	//	cout<<VEHICLE[i][k]<<' ';
	//}
	//cout<<route_cost[i]<<endl;
	//for (int k = 0; k <=saiz[i]; k++)
	//{
	//	cout<<CumDist[i][k]<<' ';
	//}
	//cout<<endl<<"Original R2"<<endl;
	//cout<<"start= "<<n<<' '<<"length= "<<L2<<endl;
	//for (int k = 0; k <=saiz[m]; k++)
	//{
	//	cout<<VEHICLE[m][k]<<' ';
	//}
	//cout<<route_cost[m]<<endl;
	//for (int k = 0; k <=saiz[m]; k++)
	//{
	//	cout<<CumDist[m][k]<<' ';
	//}
	//cout<<endl<<gainvec[1]<<' '<<gainvec[2]<<' '<<gainvec[3]<<' '<<gainvec[4]<<endl;



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
		temp1.shrink_to_fit();
		temp2.shrink_to_fit();
	}

	//float new_sumcost = route_cost[i] + route_cost[m] - gain;
	route_cost[i] -= gainvec[1];
	distance_cost[i] -= gainvec[2];
	//route_cost[i] = route_cost[m] = 0.0;//reinitialize
	//for (int c = 0; c <= saiz[i]; c++)
	//{
	//	route_cost[i] += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
	//}

	route_cost[m] -= gainvec[3];
	distance_cost[m] -= gainvec[4];
	//route_cost[m] = new_sumcost - route_cost[i]; //avoid calculate the second route cost, just use gain
	
	total_demand[i] = total_demand[i] - r1demandDelete + r2demandDelete; //check if this is correct
	total_demand[m] = total_demand[m] - r2demandDelete + r1demandDelete; //check if this is correct
	space_available[i] = CAPACITY - total_demand[i];
	space_available[m] = CAPACITY - total_demand[m];
	distance_available[i] = DISTANCE - distance_cost[i];
	distance_available[m] = DISTANCE - distance_cost[m];

	//update CumDist starting from the affecting pos until last cust
	int start1 = j;//the start of update point, always hold old or new hold whichever comes first
	for (int k = start1; k <=saiz[i]; k++)
	{
		CumDist[i][k] = CumDist[i][k-1]+dist[VEHICLE[i][k-1]][VEHICLE[i][k]] + service_time[VEHICLE[i][k]];	
	}

	//update CumDist starting from the affecting pos until last cust
	int start2 = n;//the start of update point, always hold old or new hold whichever comes first
	for (int k = start2; k <=saiz[m]; k++)
	{
		CumDist[m][k] = CumDist[m][k-1]+dist[VEHICLE[m][k-1]][VEHICLE[m][k]] + service_time[VEHICLE[m][k]];	
	}
	cout<<"After shaking R1"<<endl;
	for (int k = 0; k <=saiz[i]; k++)
	{
		cout<<VEHICLE[i][k]<<' ';
	}
	cout<<endl<<"After shaking R2"<<endl;
	for (int k = 0; k <=saiz[m]; k++)
	{
		cout<<VEHICLE[m][k]<<' ';
	}
	cout<<endl;

	float checkR_cost=0;
	for (int k = 1; k <=saiz[i]; k++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[i][k];
	}	
	if (abs(route_cost[i]-checkR_cost) > BIGC)
	{
		cout<<i<<"  Cross shaking This cost is different from original "<<endl;
		cout<<route_cost[i]<<' '<<checkR_cost<<' ';
		getchar();
	}
	checkR_cost=0;
	for (int k = 1; k <=saiz[m]; k++)//until th elast customer, last depot no need to calculate 
	{
		checkR_cost += CumDist[m][k];
	}	
	if (abs(route_cost[m]-checkR_cost) > BIGC)
	{
		cout<<m<<"  Cross shaking This cost is different from original "<<endl;
		cout<<route_cost[m]<<' '<<checkR_cost<<' ';
		getchar();
	}
	float d_cost = 0.0;//to check
	for (int c = 0; c <= saiz[i]; c++)
	{
		d_cost += dist[VEHICLE[i][c]][VEHICLE[i][c + 1]] + service_time[VEHICLE[i][c]];
	}

	if (abs(d_cost-distance_cost[i]) > BIGC)
	{
		cout<<endl<<"In cross_insert_fromshaking!!"<<' ';
		cout<<"d_cost["<<i<<"]= "<<distance_cost[i]<<' '<<"d_cost= "<<d_cost<<endl;
		getchar();
	}
	d_cost = 0.0;//to check
	for (int c = 0; c <= saiz[m]; c++)
	{
		d_cost += dist[VEHICLE[m][c]][VEHICLE[m][c + 1]] + service_time[VEHICLE[m][c]];
	}
	if ( abs(d_cost-distance_cost[m]) > 0.5)
	{
		cout<<endl<<"In cross_insert_fromshaking!!"<<' ';
		cout<<"d_cost["<<m<<"]= "<<distance_cost[m]<<' '<<"d_cost= "<<d_cost<<endl;
		getchar();
	}
	
	myvector1.clear();
	myvector1.shrink_to_fit();
	myvector2.clear();
	myvector2.shrink_to_fit();
}

//delete one and insertion based on modified route
void insert_1_0_sameRoute (int route, int old_pos, int new_pos, int **(&VEHICLE), int customer)//only used by fullPerturb
{
	
	float total_cost = 0.0;
	//float r1_cost = 0.0, r2_cost = 0.0;
	
	int ele = customer;
	

	

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

	//must do this before copy to vehicle because vehicle will be modified once copied
	total_demand[route] = total_demand[route] - demand[VEHICLE[route][old_pos]] + demand[customer];
	space_available[route] = CAPACITY - total_demand[route];

	for (int k = 0; k < saiz[route]+2; k++)
	{
		VEHICLE[route][k] = temp_r[k];  //copy to VEHICLE matrix
	}
		
	distance_cost[route] = distance_cost[route] - gainvec[2];
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
	route_cost[route] = route_cost[route] - gainvec[1]; //////////////////////check this if correct !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
