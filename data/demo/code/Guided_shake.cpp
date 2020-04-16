#include "Guided_shake.h"
#include <cmath>

using namespace std;
const float alpha = 1; //for 1/distance^alpha

void calculate_centreGravity(float *x, float *y)
{

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
}

//select route based on probability, if no feasible move, the route is removed
void findDist_from_CGravity (int cust, float *x, float *y, float *custRGravity, float **(&sorted_custRGravity))
{
	//sorted_custRGravity[i][0] = sorted distance from route center of gravity 
	//sorted_custRGravity[i][1] = route index
	//sorted_custRGravity[i][2] = 1/distance
	//sorted_custRGravity[i][3] = [2]/sum_distance
	//sorted_custRGravity[i][4] = cumulative of [3]

	//find distance between cust and route's center of gravity
	for (int i = 0; i < no_routes; i++)
	{
		float sum=0;
		sum =  (x[cust]-route_CGravity[i][0])*(x[cust]-route_CGravity[i][0]) + (y[cust]-route_CGravity[i][1])*(y[cust]-route_CGravity[i][1]) ;
		custRGravity[i] = sqrt(sum);
	}
	//before sorted, sorted_custRGravity[i][0] = custRGravity[i], sorted_custRGravity[i][1] = index
	for (int i = 0; i < no_routes; i++)
	{
		sorted_custRGravity[i][0] = custRGravity[i];
		sorted_custRGravity[i][1] = i;
	}
	//sort dist and route
	float hold = 0, keep=0;

	for (int i = 0; i < no_routes - 1; i++)
	{
		for (int j = 0; j < no_routes - 1; j++)
		{
			if (sorted_custRGravity[j][0] > sorted_custRGravity[j + 1][0])
			{
				hold = sorted_custRGravity[j][0]; //route center of gravity distance
				keep = sorted_custRGravity[j][1]; //route index

				sorted_custRGravity[j][0] = sorted_custRGravity[j + 1][0];
				sorted_custRGravity[j][1] = sorted_custRGravity[j + 1][1];

				sorted_custRGravity[j + 1][0] = hold;
				sorted_custRGravity[j + 1][1] = keep;
			}
		}
	}

	//compute 1/distance
	float sum=0;
	for (int i = 0; i <  no_routes; i++)
	{
		sorted_custRGravity[i][2] = 1/pow(sorted_custRGravity[i][0], alpha);
		sum = sum+sorted_custRGravity[i][2];
	}
	
	//compute d/sum(d)
	for (int i = 0; i < no_routes; i++)
	{
		sorted_custRGravity[i][3] = sorted_custRGravity[i][2]/sum;
	}

	//compute cumulative
	for (int i = 0; i < no_routes; i++)
	{
		if (i==0)
			sorted_custRGravity[i][4] = sorted_custRGravity[i][3];
		else
			sorted_custRGravity[i][4] = sorted_custRGravity[i-1][4] + sorted_custRGravity[i][3];
	}

}

int findRoute_fromRand (float **sorted_custRGravity, float rand)
{
	//based on rand, find the route number and return
	for (int i = 0; i < no_routes-1; i++)
	{
		if (rand < sorted_custRGravity[i][4])
		{
			int route = (int)sorted_custRGravity[i][1];
			int index = i;
			onceSelected_shift_up (route, index, sorted_custRGravity);
			return route;
		}

		if ( (rand>=sorted_custRGravity[i][4]) && (rand < sorted_custRGravity[i+1][4]) )
		{
			int route = (int)sorted_custRGravity[i+1][1];
			int index = i+1;
			onceSelected_shift_up (route, index, sorted_custRGravity);
			return route;
		}
	}
}

void onceSelected_shift_up (int route, int index, float **sorted_custRGravity)
{
	if (index == 0)//get below route index
		sorted_custRGravity[index][1] = sorted_custRGravity[index+1][1]; //copy next route index
	else 
		sorted_custRGravity[index][1] = sorted_custRGravity[index-1][1]; //copy previous route index
	
	//compute 1/distance
	sorted_custRGravity[index][2] = 0;

	//compute d/sum(d)
	float sum=0;
	for (int i = 0; i <  no_routes; i++)
	{
		sum = sum+sorted_custRGravity[i][2];
	}
	
	
	for (int i = 0; i < no_routes; i++)
	{
		sorted_custRGravity[i][3] = sorted_custRGravity[i][2]/sum;
	}
	//compute cumulative
	for (int i = 0; i < no_routes; i++)
	{
		if (i==0)
			sorted_custRGravity[i][4] = sorted_custRGravity[i][3];
		else
			sorted_custRGravity[i][4] = sorted_custRGravity[i-1][4] + sorted_custRGravity[i][3];
	}
}






