#include <time.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include "InverseMethod.h"

using namespace std;

void adjustFreq(float *(&freq), int num_module)//if probability = 0, add  so that it will have a small chance to be selected
{
	float k = 0.01;
	float half = 0.5;
	float sum_freq=0;
	for (int i = 1; i <= num_module; i++)
	{
		sum_freq = sum_freq + freq[i];
	}
	//find the smallest non-zero, add half freq of the smallest
	float smallest = freq[1];
	for (int i = 1; i <= num_module; i++)
	{
		if(freq[i]!=0 && freq[i]< smallest)
			smallest = freq[i];
	}

	for (int i = 1; i <= num_module; i++)
	{
		if (freq[i] <= k)
			freq[i] = half*smallest;
	}

	freq[9] = freq[3] = freq[5] = 0;//make sure these three are not selected

}


void findCumu(float *freq1, int num_module, float *(&cutPoint), bool *(flag_m))
{
	float *freq = new float[SIZE];
	for (int i = 0; i < SIZE; i++)
	{
		freq[i] = freq1[i]; //to avoid freq1 being overwritten
	}
	//ofstream recordFreq("27.Frequency_of_module_" + std::to_string( I ) + ".txt", ios::app);

	float *prob = new float[num_module+1];//prob[] starts from 1
	//float *cutPoint = new float[num_module+1]; //from 0 to 1 inclusive
	float sum_freq=0;
	//find sum score
	int no_m_selected=0; //number of module that has been already selected before
	for (int i = 1; i <= num_module; i++)
	{
		if(flag_m[i] == true) //if the module has been previously selected
		{
			freq[i] = 0;
		}
		sum_freq = sum_freq + freq[i];
	}

	//compute probability 
	for (int i = 1; i <= num_module; i++)
	{
		prob[i] = freq[i]/sum_freq;
	}
	float cum_prob = 0;//cumulative probability
	cutPoint[0] = 0; //5 modules, 6 cutpoint including 0 and 1
	//compute cumulative probability and cutting point
	for (int i = 1; i < num_module; i++)
	{
		cutPoint[i] = cum_prob + prob[i];
		cum_prob = cum_prob + prob[i];
	}
	cutPoint[num_module] = 1;
	//switch case

	//cout<<"========= This is cutpoint: ==========="<<endl;
	//for (int i = 0; i <= num_module; i++)
	//{
	//	cout<<cutPoint[i] << ' ';
	//	recordFreq<<cutPoint[i] << ' ';
	//}
	//cout<<endl;
	//recordFreq<<endl;
	//recordFreq.close();

	delete[] freq;
	delete[] prob;//prob[] starts from 1
}

int find_module (float rand, float *cutPoint, int num_module, bool *(flag_m))
{
	if ((rand>=cutPoint[0]) && (rand<cutPoint[1]))
	{
		if (flag_m[1] == false)//if module 1 has not been selected
			return 1;
	}
	else if ((rand>=cutPoint[1]) && (rand<cutPoint[2]))
	{
		if (flag_m[2] == false)//if module 2 has not been selected
			return 2;
	}
	else if ((rand>=cutPoint[2]) && (rand<cutPoint[3]))
	{
		if (flag_m[3] == false)//if module 3 has not been selected
			return 3;
	}
	else if ((rand>=cutPoint[3]) && (rand<cutPoint[4]))
	{
		if (flag_m[4] == false)//if module 4 has not been selected
			return 4;
	}
	else if ((rand>=cutPoint[4]) && (rand<=cutPoint[5]))
	{
		if (flag_m[5] == false)//if module 5 has not been selected
			return 5;
	}
	else if ((rand>=cutPoint[5]) && (rand<cutPoint[6]))
	{
		if (flag_m[6] == false)//if module 6 has not been selected
			return 6;
	}
	else if ((rand>=cutPoint[6]) && (rand<cutPoint[7]))
	{
		if (flag_m[7] == false)//if module 7 has not been selected
			return 7;
	}
	else if ((rand>=cutPoint[7]) && (rand<cutPoint[8]))
	{
		if (flag_m[8] == false)//if module 8 has not been selected
			return 8;
	}
	else if ((rand>=cutPoint[8]) && (rand<cutPoint[9]))
	{
		if (flag_m[9] == false)//if module 9 has not been selected
			return 9;
	}
	//else if ((rand>=cutPoint[9]) && (rand<=cutPoint[10]))
	//{
	//	if (flag_m[10] == false)//if module 10 has not been selected
	//		return 10;
	//}

}