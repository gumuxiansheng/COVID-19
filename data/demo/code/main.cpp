#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <string>
#include "NeighborhoodReduction.h"
//#include "NN.h"
//#include "sweep.h"
//#include "saving.h"
//#include "dijkstra.h"
//#include "compareString.h"
//#include "2_opt.h"
//#include "indicator_matrix.h"
//#include "checkRepetitionILP2.h"
#include "BestImprovement.h"
#include "VNS.h"
//#include "CheckFreq.h"
#include "VNS_multiLevel.h"
//#include "SetCovering.h"
//#include "SectorConflict.h"
//#include "VND.h"
//#include <ilcplex/ilocplex.h>  //links it with cplex. (for concert technology)
//#include <ilcplex/cplex.h> // for callable library

#define INFIN 99999999 //make sure the INFIN is large enough, all other values doesnt exceed INFIN, otherwise is error


using namespace std;
const float epsilon = 0.05;
const float epsilonn = 0.01;
const float PI = 3.141592653589793238463;
const float alphamin = PI/12; //pi/4 = 45 degree, //pi/12 = 15 degree
const float alphamax = PI/6; //pi/2 = 90 degree, //pi/6 = 30 degree
//const float alphamin = PI/6; //pi/6 = 30 degree
//const float alphamax = PI/3; //pi/3 = 60 degree
//const float alphamin = PI/3; //pi/3 = 60 degree
//const float alphamax = 2*PI/3; //2pi/3 = 120 degree
const float chi = 1; //parameter for capacity constraint in constructive strategy
const float delta = 1; //parameter for distance constraint in constructive strategy

//ILOSTLBEGIN				//format from concert cplex (callable does not need this).

//================================= GLOBAL VARIABLE =====================================//
bool **NR_FLAG_DEPOT; //global variable for neighborhood reduction only used when considered inserting at the beginning or at the end going back to depot
bool **NR_FLAG; //global variable for neighborhood reduction
bool **DEMAND_FLAG;//flag if they have similar demand, for DiversifyRelatednessDemand
int* service_time; //global variable for service time
float* theta; //angle[SIZE][i] //computed from sweep, used in diversification_sector_conflicting and diversification_overlapRoute

float **dist;
int *demand;


//================================= LOCAL BEST  SOLUTION ==================================================//
float LOCAL_BEST_COST = INT_MAX;
int LOCAL_NO_ROUTE;
int *LOCAL_capa; //record capacity of the best solution 
float *LOCAL_Rcost;
float *LOCAL_distance_cost;
int **LOCAL; //to store the best solution!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
int *LOCAL_SAIZ;
float **LOCALCumDist;//record cumulative distance

//int **LOCAL_B;//to store the best of LOCAL, best of the best in all neighbourhoods, if it is not too far, eg 1% from the GLOBAL, use it for diversification (added 25Feb2016)
//float LOCAL_BCOST;//record the LOCAL_B cost
//int LOCAL_Br;//record number of routes for LOCAL_B
//float *LOCAL_Bc;
//float *LOCAL_Bdistance_cost;
//int *LOCAL_Bs;//record the saiz
//int *LOCAL_Bq;//record capacity
//float **LOCAL_BCumDist;//record cumulative distance



//================================= GLOBAL VARIABLE FOR BEST SOLUTION =====================================//
int SIZE, CAPACITY, DISTANCE, SERVICE_T;
float GLOBAL_BEST_COST = INT_MAX;
int GLOBAL_NO_ROUTE;
int big_index = 0; //to keep track of total routes for big_matrix file
int *GLOBAL_capa; //record capacity of the best solution //the same as total_demand???????????????????????????????????
float *GLOBAL_Rcost;
float *GLOBAL_distance_cost;
int **GLOBAL;
int *GLOBAL_SAIZ;
float **GLOBALCumDist;//record cumulative distance

//================================= GLOBAL VARIABLE FOR TEMPORARY SOLUTION =====================================//
//for inter-route imrovement
int number, module; //number is corresponding to number in lower diagonal of gain matrix
int route_change[2]; //last two routes that have been changed  
int after_shake_route_change[6]; //record the route change in shake to update cost of removing accordingly, noneed to update all routes
int after_shake_route_changePtr; //to record how many routes have been changed, this if for multi-shake so that all routes chnged can be recorded
bool *RchangedStatus;//to record which route has changed in VNS, multilevel VNS and adaptive, false no change, true has changed
//bool *CustaffectedStatus;//record customer affected status for update cost_of_removing, noneed tfor CCVRP because cost_of_removing for the entire route needs to be updated if the route changes
float **CumDist;//record cumulative distance

int* space_available;	//to check if the space available is enough for the new customer to be inserted
int* total_demand;
float* distance_available;
int* saiz;
float* distance_cost;//record cost in distance
float* route_cost;
int no_routes;
float Bestcost;
int **VEHICLE;

//============= Guided shake ==================//
float **route_CGravity; //for guided shake //record centre of gravity for routes for guided shake, once found LOCAL_BEST, update because shaking based on LOCAL_BEST
float *custRGravity;
float **sorted_custRGravity;
//============= End (Guided shake) ==================//

int *which_module;
//================================= GLOBAL VARIABLE =====================================//

std::string getLastLine(std::ifstream& in)
{
	std::string line;
	while (in >> std::ws && std::getline(in, line)) // skip empty lines
		;

	return line;
}

string filename; //global variable to run many data sets
int I;//for running different data sets loop
void checkReverseOrder();
void FindAngleBetweenDepotCustomer (float *x, float *y);
void findDemandFlag(int TOTAL_DEMAND);


int main()
{
	
	
	for (int Z = 0; Z < 200; Z++)//to run 10 times each
	{

	
	//ofstream recordTIME("TIME.txt", ios::out);
	//ofstream recordRESULT("RESULT.txt", ios::out);
	ofstream greedy("32.Greedy insertion_.txt", ios::out);
	ofstream initialSol("INITIALSOLUTION.txt",ios::out);
	for (I = 9; I <= 15; I++) //how many data sets to run
	{
		//if (I==5)
		if ( I==2 || I==3 || I==4  || I==5 || I==6 || I==10  ||I==12  ||I==14 )
			continue;
		ofstream recordresult("RESULT_" + std::to_string( I ) + ".txt", ios::out);
		ofstream see("See_" + std::to_string( I ) + ".txt", ios::out);
		
		cout<<"I= "<<I<<endl;
		ofstream constructive("CONSTRUCTIVE_SOLUTION_" + std::to_string( I ) + ".txt");
		big_index = 0;//reinitialize big index
		//int i=1;
		//filename = "C:\\DataSet\\small_prog\\New folder\\Christofides_" + std::to_string( I ) + ".txt";
		//filename = "C:\\DataSet\\small_prog\\New folder\\Fisher_" + std::to_string( I ) + ".txt";
		filename = "C:\\DataSet\\Medium\\New folder\\Golden_"+ std::to_string( I ) + ".txt";
		//filename = "C:\\DataSet\\Large\\New folder\\Li_"+ std::to_string( I ) + ".txt";

		
		//ifstream file("Christofides_" + std::to_string( I ), ios::in); //read the file
		//ifstream file("Fisher_" + std::to_string( I ), ios::in); //read the file
		ifstream file("Golden_" + std::to_string( I ), ios::in);
		//ifstream file("Li_" + std::to_string( I ), ios::in);

		//ofstream routesfile("3.ROUTES_" + std::to_string( I ) + ".txt", ios::out);
		ofstream best_sol("7.BEST_SOLUTION_" + std::to_string( I ) + ".txt");
		ofstream localbest_sol("30.LOCAL_BEST_SOLUTION_" + std::to_string( I ) + ".txt",ios::out);
		//ofstream giant("29.Make_giant_tour_" + std::to_string( I ) + ".txt", ios::out);
		ofstream diversify_conflictSector("32.DiversifySector_" + std::to_string( I ) + ".txt", ios::out);
		//ofstream diversifyLNS("31.DiversifyLNS_" + std::to_string( I ) + ".txt", ios::out);
		ofstream diversifyLNS2("31.DiversifyLNS2_" + std::to_string( I ) + ".txt", ios::out);
		ofstream diversifyLNS3("31.DiversifyLNS3_" + std::to_string( I ) + ".txt", ios::out);
		ofstream Diversify_LongestArc2("31.Diversify_LongestArc2_" + std::to_string( I ) + ".txt", ios::out);
		ofstream Diversify_BadArcNR("31.Diversify_BadArcNR_" + std::to_string( I ) + ".txt", ios::out);
		ofstream diversify_overlap("32.Diversifyoverlap_" + std::to_string( I ) + ".txt", ios::out);
		ofstream diversifyRelatedness("31.DiversifyRelatedness_" + std::to_string( I ) + ".txt", ios::out);
		ofstream CGravityfile("33.CGravity_" + std::to_string( I ) + ".txt", ios::out);
		//ofstream route_file("route_" + std::to_string( I ) + ".txt", ios::out);
		ofstream crossTail("crossTail_"+ std::to_string( I ) + ".txt", ios::out);
		ofstream twoOptInte2r("2-optInter2_"+ std::to_string( I ) + ".txt", ios::out);
		ofstream cross("CROSS_"+ std::to_string( I ) + ".txt", ios::out);
		float start_s = clock();// the code you wish to time goes here



		//=========================================Read File=============================================//
		//string filename = "C:\\DataSet\\Medium\\Golden_1_240.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_2_320.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_3_400.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_4_480.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_5_200.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_6_280.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_7_360.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_8_440.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_9_255.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_10_323.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_11_399.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_12_483.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_13_252.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_14_320.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_15_396.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_16_480.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_17_240.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_18_300.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_19_360.txt";
		//string filename = "C:\\DataSet\\Medium\\Golden_20_420.txt";

		//string filename = "C:\\DataSet\\Large\\Li_21_560.txt";
		//string filename = "C:\\DataSet\\Large\\Li_22_600.txt";
		//string filename = "C:\\DataSet\\Large\\Li_23_640.txt";
		//string filename = "C:\\DataSet\\Large\\Li_24_720.txt";
		//string filename = "C:\\DataSet\\Large\\Li_25_760.txt";
		//string filename = "C:\\DataSet\\Large\\Li_26_800.txt";
		//string filename = "C:\\DataSet\\Large\\Li_27_840.txt";
		//string filename = "C:\\DataSet\\Large\\Li_28_880.txt";
		//string filename = "C:\\DataSet\\Large\\Li_29_960.txt";
		//string filename = "C:\\DataSet\\Large\\Li_30_1040.txt";
		//string filename = "C:\\DataSet\\Large\\Li_31_1120.txt";
		//string filename = "C:\\DataSet\\Large\\Li_32_1200.txt";


		//string filename = "C:\\DataSet\\small_prog\\Christofides_21.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_22.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_29.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_32.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_1_50.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_2_75.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_3_100.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_4_150.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_5_199.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_6_50.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_7_75.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_8_100.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_9_150.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_10_199.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_11_120.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_12_100.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_13_120.txt";
		//string filename = "C:\\DataSet\\small_prog\\Christofides_14_100.txt";
		//string filename = "C:\\DataSet\\small_prog\\Fisher_44.txt";
		//string filename = "C:\\DataSet\\small_prog\\Fisher_71.txt";
		//string filename = "C:\\DataSet\\small_prog\\Fisher_134.txt";
		//char w;
		float temp;
		int c = 0;


		//ofstream savingfile("saving.txt");
		ofstream distancefile("1.Distance_" + std::to_string( I ) + ".txt", ios::out); 
		//ofstream routesfile("ROUTES.txt");
		//ofstream solfile("BEST_SOLUTION.txt");
		//ofstream bigmatrixfile("Big_Matrix.txt");
		//ofstream indicatorfile("Indicator_Matrix.txt");
		//ofstream anglefile("Angle.txt");
		//ofstream dijkstradistfile("DIJKSTRA_DISTANCE.txt");
		ofstream timefile("16.Time_" + std::to_string( I ) + ".txt", ios::out); 

	
		//ifstream file("Golden_1_240", ios::in);
		//ifstream file("Golden_2_320", ios::in);
		//ifstream file("Golden_3_400", ios::in);
		//ifstream file("Golden_4_480", ios::in);
		//ifstream file("Golden_5_200", ios::in);
		//ifstream file("Golden_6_280", ios::in);
		//ifstream file("Golden_7_360", ios::in);
		//ifstream file("Golden_8_440", ios::in);
		//ifstream file("Golden_9_255", ios::in);
		//ifstream file("Golden_10_323", ios::in);
		//ifstream file("Golden_11_399", ios::in);
		//ifstream file("Golden_12_483", ios::in);
		//ifstream file("Golden_13_252", ios::in);
		//ifstream file("Golden_14_320", ios::in);
		//ifstream file("Golden_15_396", ios::in);
		//ifstream file("Golden_16_480", ios::in);
		//ifstream file("Golden_17_240", ios::in);
		//ifstream file("Golden_18_300", ios::in);
		//ifstream file("Golden_19_360", ios::in);
		//ifstream file("Golden_20_420", ios::in);
	
		//ifstream file("Li_21_560", ios::in);
		//ifstream file("Li_22_600", ios::in);
		//ifstream file("Li_23_640", ios::in);
		//ifstream file("Li_24_720", ios::in);
		//ifstream file("Li_25_760", ios::in);
		//ifstream file("Li_26_800", ios::in);
		//ifstream file("Li_27_840", ios::in);
		//ifstream file("Li_28_880", ios::in);
		//ifstream file("Li_29_960", ios::in);
		//ifstream file("Li_30_1040", ios::in);
		//ifstream file("Li_31_1120", ios::in);
		//ifstream file("Li_32_1200", ios::in);


		//ifstream file("Christofides_21", ios::in);
		//ifstream file("Christofides_22", ios::in);
		//ifstream file("Christofides_29", ios::in);
		//ifstream file("Christofides_32", ios::in);
		//ifstream file("Christofides_1_50", ios::in);
		//ifstream file("Christofides_2_75", ios::in);
		//ifstream file("Christofides_3_100", ios::in);
		//ifstream file("Christofides_4_150", ios::in);
		//ifstream file("Christofides_5_199", ios::in);
		//ifstream file("Christofides_6_50", ios::in);
		//ifstream file("Christofides_7_75", ios::in);
		//ifstream file("Christofides_8_100", ios::in);
		//ifstream file("Christofides_9_150", ios::in);
		//ifstream file("Christofides_10_199", ios::in);
		//ifstream file("Christofides_11_120", ios::in);
		//ifstream file("Christofides_12_100", ios::in);
		//ifstream file("Christofides_13_120", ios::in);
		//ifstream file("Christofides_14_100", ios::in);
		//ifstream file("Fisher_44", ios::in);
		//ifstream file("Fisher_71", ios::in);
		//ifstream file("Fisher_134", ios::in);

		file.open(filename.c_str());

		if (!file)
		{
			cerr << "File could not be opened\n";
			exit(1);
		}

		ofstream myfile1("2.Demand_" + std::to_string( I ) + ".txt", ios::out); 
		//ofstream route_file("route.txt");

		file >> temp;//to skip size
		SIZE = temp;
		file >> temp;//to skip capacity
		CAPACITY = temp;
		file >> temp;//to skip distance
		DISTANCE = temp;
		file >> temp;//to skip service time
		//SERVICE_T = 0;
		SERVICE_T = temp;

		//if (DISTANCE == 0)
			DISTANCE = INT_MAX;   //there is no distance constraint for CCVRP, so put a big value

		float* x = new float[SIZE + 1];
		float* y = new float[SIZE + 1];
		demand = new int[SIZE + 1];
		service_time = new int[SIZE+1]; //service time is a global variable
		int* giant_tour = new int[SIZE + 2]; //start and end with depot

		//==================all 2-d arrays==============================//
		for (int i = 0; i < SIZE; i++)
		{
			service_time[i] = SERVICE_T;
		}
		service_time[SIZE] = 0; //service time for depot is 0

		dist = new float*[SIZE + 1];
		for (int i = 0; i < SIZE + 1; ++i)
		{
			dist[i] = new float[SIZE + 1];

		}
		
		//==============================================================//
		int overall_route_index = 0; //to keep track of the overall route index for the use of big matrix, overall_route_index-1 because first route starts at index 0
		int route_index = 0; //index for long_matrix only

		file >> temp;
		x[SIZE] = temp; //depot
		file >> temp;
		y[SIZE] = temp; //depot
		file >> temp;
		demand[SIZE] = temp; //depot

		int TOTAL_DEMAND = 0;

		for (int i = 0; i < SIZE; i++)
		{

			file >> temp;
			x[i] = temp;
			file >> temp;
			y[i] = temp;
			file >> temp;
			demand[i] = temp;
			myfile1 << i << ' ' << ' ' << x[i] << ' ' << ' ' << y[i] << ' ' << ' ' << demand[i] << ' ' << ' ' << endl;
			//cout << setw(15) << x[i] << setw(15) << y[i] << setw(15) << demand[i] << setw(15) << endl;
			TOTAL_DEMAND = TOTAL_DEMAND + demand[i];

		}
		myfile1.close();

		for (int i = 0; i < SIZE + 2; i++)
		{
			giant_tour[i] = i; //initialize the giant_tour to i, later it will be the sorted customer based on angle
		}

		float d_1, d_2;


		//========================================Find distance matrix ============================================//

		for (int j = 0; j < SIZE + 1; j++)
		{
			for (int k = 0; k < SIZE + 1; k++)
			{
				d_1 = ((x[j] - x[k])*(x[j] - x[k])) + ((y[j] - y[k])*(y[j] - y[k]));
				d_2 = sqrt(d_1);

				dist[j][k] = d_2;

				distancefile << "distance[" << j << "]" << "[" << k << "]" << "=" << dist[j][k] << endl;

			}
			dist[j][j] = 0;			//distance at a  point itself is 0
			distancefile << endl;
		}

		distancefile.close();

		DEMAND_FLAG	= new bool*[SIZE + 1];
		NR_FLAG = new bool*[SIZE + 1]; //NR_FLAG is a global variable
		NR_FLAG_DEPOT = new bool*[SIZE + 1]; //NR_FLAG is a global variable
		for (int i = 0; i < SIZE + 1; i++)
		{
			NR_FLAG[i] = new bool[SIZE + 1];
			NR_FLAG_DEPOT[i] = new bool[SIZE + 1];
			DEMAND_FLAG[i] = new bool[SIZE + 1];
		}
		neighborhood_reduction (x, y, TOTAL_DEMAND);
		findDemandFlag(TOTAL_DEMAND);

		theta = new float[SIZE];
		FindAngleBetweenDepotCustomer (x, y);
		
		//============================= RUN FROM CONSTRUCTIVE_SOLUTION GOES HERE ========================================//
		//TO GET GLOBAL_NO_ROUTE, THIS VALUE NEVER CHANGED
		float CONSTRUCTIVE_GLOBAL_BEST = 0.0;
		//string opensol2 = "C:\\DataSet\\CCVRP\\CCVRP Christofides_" + std::to_string( I ) + ".txt";
		//string opensol2 = "C:\\DataSet\\CCVRP\\CCVRP Fisher_" + std::to_string( I ) + ".txt";
		string opensol2 = "C:\\DataSet\\CCVRP\\CCVRP Golden_" + std::to_string( I ) + ".txt";
		//string opensol2 = "C:\\DataSet\\CCVRP\\CCVRP Li_" + std::to_string( I ) + ".txt";

		//ifstream sol2("CCVRP Christofides_" + std::to_string( I ) + ".txt", ios::in);
		//ifstream sol2("CCVRP Fisher_" + std::to_string( I ) + ".txt", ios::in);
		ifstream sol2("CCVRP Golden_" + std::to_string( I ) + ".txt", ios::in);
		//ifstream sol2("CCVRP Li_" + std::to_string( I ) + ".txt", ios::in);

		sol2.open(opensol2.c_str());
		if (sol2)
		{
			std::string line = getLastLine(sol2);
			GLOBAL_NO_ROUTE = atoi(line.c_str()) + 1; //value = 45 

		}
		else
		{
			cerr << "File could not be opened\n";
			exit(1);
	
		}
		sol2.close();

		//string opensol = "C:\\DataSet\\CCVRP\\CCVRP Christofides_" + std::to_string( I ) + ".txt";
		//string opensol = "C:\\DataSet\\CCVRP\\CCVRP Fisher_" + std::to_string( I ) + ".txt";
		string opensol = "C:\\DataSet\\CCVRP\\CCVRP Golden_" + std::to_string( I ) + ".txt";
		//string opensol = "C:\\DataSet\\CCVRP\\CCVRP Li_" + std::to_string( I ) + ".txt";
		
		//ifstream sol("CCVRP Christofides_" + std::to_string( I ) + ".txt", ios::in);
		//ifstream sol("CCVRP Fisher_" + std::to_string( I ) + ".txt", ios::in);
		ifstream sol("CCVRP Golden_" + std::to_string( I ) + ".txt", ios::in);
		//ifstream sol("CCVRP Li_" + std::to_string( I ) + ".txt", ios::in);

		sol.open(opensol.c_str());
		if (!sol)
		{
			cerr << "File could not be opened\n";
			exit(1);
	
		}
		cout << "GLOBAL_NO_ROUTE= " << GLOBAL_NO_ROUTE << endl;

		GLOBAL = new int*[GLOBAL_NO_ROUTE]; //GLOBAL_BEST_ROUTE will not change
		LOCAL = new int*[GLOBAL_NO_ROUTE]; //LOCAL_BEST_ROUTE is global variable
		//LOCAL_B = new int*[GLOBAL_NO_ROUTE]; //to record the best of local for diversification purpose
		VEHICLE = new int*[GLOBAL_NO_ROUTE]; 

		GLOBALCumDist = new float*[GLOBAL_NO_ROUTE]; 
		LOCALCumDist = new float*[GLOBAL_NO_ROUTE]; 
		//LOCAL_BCumDist = new float*[GLOBAL_NO_ROUTE]; 
		CumDist = new float*[GLOBAL_NO_ROUTE]; //record Cumu Dist at each posiition

	
		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		{

			GLOBAL[i] = new int[SIZE];
			LOCAL[i] = new int[SIZE];
			//LOCAL_B[i] = new int[SIZE];
			VEHICLE[i] = new int[SIZE];
			GLOBALCumDist[i] = new float[SIZE]; 
			LOCALCumDist[i] = new float[SIZE]; 
			//LOCAL_BCumDist[i] = new float[SIZE]; 
			CumDist[i] = new float[SIZE]; 
	
		}

		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		{
			for (int j = 0; j < SIZE; j++)
			{
				
				GLOBAL[i][j] = -1;
				LOCAL[i][j] = -1;
				//LOCAL_B[i][j] = -1;
			}
		}

		no_routes = LOCAL_NO_ROUTE = GLOBAL_NO_ROUTE; //GLOBAL_NO_ROUTE is global variable
		//declare more because I dont know the no_routes yet, previously declare no_routes and cause heap error
		space_available = new int[GLOBAL_NO_ROUTE]; 
		distance_available = new float[GLOBAL_NO_ROUTE];
		total_demand = new int[GLOBAL_NO_ROUTE];
		saiz = new int[GLOBAL_NO_ROUTE];
		route_cost = new float[GLOBAL_NO_ROUTE];
		distance_cost = new float[GLOBAL_NO_ROUTE];
		
		GLOBAL_SAIZ = new int[GLOBAL_NO_ROUTE];
		GLOBAL_capa = new int [GLOBAL_NO_ROUTE]; //record the capacity of the best solution
		GLOBAL_Rcost = new float[GLOBAL_NO_ROUTE];
		GLOBAL_distance_cost = new float[GLOBAL_NO_ROUTE];

		LOCAL_SAIZ = new int[GLOBAL_NO_ROUTE];
		LOCAL_capa = new int [GLOBAL_NO_ROUTE]; 
		LOCAL_Rcost = new float[GLOBAL_NO_ROUTE];
		LOCAL_distance_cost = new float[GLOBAL_NO_ROUTE];

		//LOCAL_Bs = new int[SIZE];
		//LOCAL_Bq = new int [SIZE]; 
		//LOCAL_Bc = new float[SIZE];
		//LOCAL_Bdistance_cost = new float[SIZE];

		float *freq = new float[SIZE];  //declare up to SIZE //only freq[] for 8 modules, starts from 1 to 8
			//freq[1]= 23.5717;
			//freq[2]= 1453.14;
			//freq[3]= 0.914086;
			//freq[4]= 123.521;
			//freq[5]= 83.7607;
			//freq[6]= 41.2814;
			//freq[7]= 2.66631;
			//freq[8]= 30.1177;  

		GLOBAL_BEST_COST=0;//reinitialize
		temp=-1;
		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		{
			sol>>temp; //skip index
			sol>>temp; //cost
			GLOBAL_Rcost[i] = temp;
			sol>>temp; //saiz
			GLOBAL_SAIZ[i] = temp;
			sol>>temp; //capa
			GLOBAL_capa[i] = temp;
			
			for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
			{
				sol>>temp;
				GLOBAL[i][j] = temp;
			
			}
			GLOBAL_BEST_COST += GLOBAL_Rcost[i];
		}
		//calculate distance_cost
		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		{
			GLOBAL_distance_cost[i]=0;
			for (int j = 0; j <= GLOBAL_SAIZ[i]; j++)
			{
				GLOBAL_distance_cost[i] += dist[GLOBAL[i][j]][GLOBAL[i][j+1]] + service_time[GLOBAL[i][j]]; 
			}
		}
		CONSTRUCTIVE_GLOBAL_BEST = GLOBAL_BEST_COST;
		//============================= END OF READ CONSTRUCTIVE_SOLUTION FROM TXT FILE========================================//

		//============================= READ FROM GLOBAL_BEST_SOLUTION ========================================//
		//float INITIAL_GLOBAL_BEST = 0.0;
		//string opensol2 = "C:\\DataSet\\BEST_SOLUTION_Christofides_" + std::to_string( I ) + ".txt";
		////string opensol2 = "C:\\DataSet\\BEST_SOLUTION_Fisher_" + std::to_string( I ) + ".txt";
		////string opensol2 = "C:\\DataSet\\BEST_SOLUTION_Golden_" + std::to_string( I ) + ".txt";
		////string opensol2 = "C:\\DataSet\\BEST_SOLUTION_Li_" + std::to_string( I ) + ".txt";
		//ifstream sol2("BEST_SOLUTION_Christofides_" + std::to_string( I ) + ".txt", ios::in);
		////ifstream sol2("BEST_SOLUTION_Fisher_" + std::to_string( I ) + ".txt", ios::in);
		////ifstream sol2("BEST_SOLUTION_Golden_" + std::to_string( I ) + ".txt", ios::in);
		////ifstream sol2("BEST_SOLUTION_Li_" + std::to_string( I ) + ".txt", ios::in);
		//sol2.open(opensol2.c_str());
		//if (sol2)
		//{
		//	std::string line = getLastLine(sol2);
		//	GLOBAL_NO_ROUTE = atoi(line.c_str()) + 1; //value = 45 
		//}
		//else
		//{
		//	cerr << "File could not be opened\n";
		//	exit(1);
		//}
		//sol2.close();
		//string opensol = "C:\\DataSet\\BEST_SOLUTION_Christofides_" + std::to_string( I ) + ".txt";
		////string opensol = "C:\\DataSet\\BEST_SOLUTION_Fisher_" + std::to_string( I ) + ".txt";
		////string opensol = "C:\\DataSet\\BEST_SOLUTION_Golden_" + std::to_string( I ) + ".txt";
		////string opensol = "C:\\DataSet\\BEST_SOLUTION_Li_" + std::to_string( I ) + ".txt";
		//ifstream sol("BEST_SOLUTION_"+ std::to_string( I ) + ".txt", ios::in);
		//sol.open(opensol.c_str());
		//if (!sol)
		//{
		//	cerr << "File could not be opened\n";
		//	exit(1);
		//}
		//cout << "GLOBAL_NO_ROUTE= " << GLOBAL_NO_ROUTE << endl;
		//
		//temp=-1;
		//GLOBAL_BEST_COST=0;//reinitialize
		//for (int i = 0; i < GLOBAL_NO_ROUTE; i++) //format saved in file [index] [cost] [saiz] [capa] [D 0 1 2 3 5 D -1] but we did not record -1
		//{
		//	sol>>temp; //skip index
		//	sol>>temp;//this is cost
		//	GLOBAL_Rcost[i] = temp;
		//	sol>>temp;//this is saiz
		//	GLOBAL_SAIZ[i] = temp;
		//	sol>>temp;//this is capa
		//	GLOBAL_capa[i] = temp;
		//	for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
		//	{
		//		sol>>temp;
		//		GLOBAL[i][j] = temp;
		//	}
		//	GLOBAL_BEST_COST += GLOBAL_Rcost[i];
		//}
		//
		//INITIAL_GLOBAL_BEST = GLOBAL_BEST_COST;
		//============================= END OF GLOBAL_BEST_SOLUTION ========================================//

		//nearest_neighbourhood2();
		//sweep(x, y, giant_tour);
		//doublesweep(x, y, giant_tour);
		//saving2();
		//dijkstra_partition(giant_tour);
		//if (SIZE<199)
		//{
		//	compare_string();
		//	two_opt_big_matrix();
		//	indicator_matrix();
		//	check_repetition_ILP2 ();
		//}
		//
		//		//for checking
		//cout<<"before VND"<<endl;
		//	
		//	for (int r = 0; r < GLOBAL_NO_ROUTE; r++)
		//	{
		//		float checkc=0.0;
		//		int checkd=0;
		//		for (int t = 0; t <= GLOBAL_SAIZ[r]; t++)
		//		{
		//			checkc+=dist[GLOBAL[r][t]][GLOBAL[r][t+1]]+service_time[GLOBAL[r][t]];
		//			checkd+=demand[GLOBAL[r][t]];
		//		}
		//		if (abs(checkc - GLOBAL_Rcost[r]) >epsilon)
		//		{
		//		
		//			cout<<r<<"cost is wrong"<<endl;
		//			getchar();
		//		}
		//		if (checkd != GLOBAL_capa[r])
		//		{
		//			cout<<"demand is wrong"<<endl;
		//			getchar();
		//		}
		//	}
		//calculate CumDist for GLOBAL
		//float totalcost=0;
		see<<endl;
		see<<"Original GLOBAL"<<endl;
		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		{
			see<<i <<' '<< GLOBAL_Rcost[i]<< ' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<< ' ';
			for (int j = 0; j <=GLOBAL_SAIZ[i]+1; j++)//until th elast customer, last depot also need to calculate because if deletion will need this value
			{
				see<<GLOBAL[i][j]<<' ';
			}
			see<<endl;
			//totalcost+=GLOBAL_Rcost[i];
		}

			see<<endl;
		see<<"Original CUmDIst"<<endl;
		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		{
			GLOBALCumDist[i][0]= 0;
			see<<GLOBALCumDist[i][0]<<' ';
			for (int j = 1; j <=GLOBAL_SAIZ[i]+1; j++)//until th elast customer, last depot also need to calculate because if deletion will need this value
			{
				GLOBALCumDist[i][j]=GLOBALCumDist[i][j-1]+dist[GLOBAL[i][j-1]][GLOBAL[i][j]] + service_time[GLOBAL[i][j]];
				see<<GLOBALCumDist[i][j]<<' ';
			}
			see<<endl;
			//totalcost+=GLOBAL_Rcost[i];
		}
		//VND(x, y, VEHICLE); //only call in main because it will read GLOBAL[][] in VND
		//checkReverseOrder();
		//inter_route_improvement();

		//timefile << "Initial solution time: " << (clock() - start_s) / float(CLOCKS_PER_SEC) << endl;

		//VND(x, y, VEHICLE); //only call in main because it will read GLOBAL[][] in VND
		//float start_s2 = clock();// the code you wish to time goes here
		VNS(x, y, freq);
		//cout<<"Start multi-level VNS"<<endl;
		//getchar();
		//VNS_kth_improvement(x, y);
		//cout<<"Start adaptive VNS"<<endl;
		//getchar();
		adaptiveVNS_kth_improvement(x, y, freq);
		//float time = (clock() - start_s2) / float(CLOCKS_PER_SEC) ;
	

		checkReverseOrder();
		if (SERVICE_T != 0)
		{
			constructive<<endl<<"============== BEST ROUTE IN MAIN " <<"("<< I <<") ========================="<<endl;
			cout<<endl<<"============== BEST ROUTE IN MAIN ========================="<<endl;
			for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
			{
				cout<<i <<' '<< GLOBAL_Rcost[i]- (GLOBAL_SAIZ[i] * SERVICE_T)<< ' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<< ' ';
				constructive<<i <<' '<< GLOBAL_Rcost[i]- (GLOBAL_SAIZ[i] * SERVICE_T)<< ' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<< ' ';

				for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
				{
					cout<<GLOBAL[i][j] <<' ';
					constructive<<GLOBAL[i][j] <<' ';
				}
				cout<<endl;
				constructive<<endl;
			}
			cout<< "GLOBAL_BEST_COST= " <<(GLOBAL_BEST_COST-(SIZE*SERVICE_T))<<endl;
			constructive<< "GLOBAL_BEST_COST(with ST)= " <<GLOBAL_BEST_COST<<endl;
			constructive<< "GLOBAL_BEST_COST(without ST)= " <<(GLOBAL_BEST_COST-(SIZE*SERVICE_T))<<endl;
			cout << "CAPACITY= " << CAPACITY << endl;
			cout << "DISTANCE= " << DISTANCE << endl;
			goto jumptoend;
		}
		best_sol<<endl<<"============== BEST ROUTE IN MAIN " <<"("<< I <<") ========================="<<endl;
		constructive<<endl<<"============== BEST ROUTE IN MAIN " <<"("<< I <<") ========================="<<endl;
		cout<<endl<<"============== BEST ROUTE IN MAIN ========================="<<endl;
		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		{
			best_sol<<i <<' '<< GLOBAL_Rcost[i]<< ' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<< ' ';
			cout<<i <<' '<< GLOBAL_Rcost[i]<< ' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<< ' ';
			constructive<<i <<' '<< GLOBAL_Rcost[i]<< ' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<< ' ';

			for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
			{
				best_sol<<GLOBAL[i][j] <<' ';
				cout<<GLOBAL[i][j] <<' ';
				constructive<<GLOBAL[i][j] <<' ';

			}
			best_sol<<endl;
			cout<<endl;
			constructive<<endl;
		}
		best_sol<< "GLOBAL_BEST_COST= " <<GLOBAL_BEST_COST<<endl;
		cout<< "GLOBAL_BEST_COST= " <<GLOBAL_BEST_COST<<endl;
		constructive<< "GLOBAL_BEST_COST(with ST)= " <<GLOBAL_BEST_COST<<endl;
		constructive<< "GLOBAL_BEST_COST(without ST)= " <<(GLOBAL_BEST_COST-(SIZE*SERVICE_T))<<endl;
		cout << "CAPACITY= " << CAPACITY << endl;
		cout << "DISTANCE= " << DISTANCE << endl;

jumptoend:
		initialSol<<endl<<"Data Set "<<I<<endl;
		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		{
			initialSol<<i <<' '<< GLOBAL_Rcost[i]<< ' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<< ' ';

			for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
			{
				initialSol<<GLOBAL[i][j] <<' ';
			}
			initialSol<<endl;
		}
		initialSol<< "Inial solution (without ST)= " <<(GLOBAL_BEST_COST-(SIZE*SERVICE_T))<<endl;
		initialSol<< "GLOBAL_BEST_COST(with ST)= " <<GLOBAL_BEST_COST<<endl;

		////=============== for overwrite if using BEST SOLUTION if run data from BEST SOLUTION=======================================//
		//if (INITIAL_GLOBAL_BEST - GLOBAL_BEST_COST > 0.001)
		//{
		//	//string overwritesol = "C:\\DataSet\\BEST_SOLUTION_Christofides_" + std::to_string( I ) + ".txt";
		//	//string overwritesol = "C:\\DataSet\\BEST_SOLUTION_Fisher_" + std::to_string( I ) + ".txt";
		//	string overwritesol = "C:\\DataSet\\BEST_SOLUTION_Golden_" + std::to_string( I ) + ".txt";
		//	//string overwritesol = "C:\\DataSet\\BEST_SOLUTION_Li_" + std::to_string( I ) + ".txt";
		//	ofstream overwrite;
		//	overwrite.open(overwritesol.c_str());

		//	file.open(filename.c_str());

		//	if (!overwrite)
		//	{
		//		cerr << "File overwrite could not be opened\n";
		//		exit(1);
		//	}
		//
		//	cout<<"============== BEST ROUTE IN MAIN ========================="<<endl;
		//	overwrite.clear();
		//	for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		//	{
		//		overwrite<<i <<' '<< GLOBAL_Rcost[i]<< ' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' ';
		//		for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
		//		{
		//			overwrite<<GLOBAL[i][j] <<' ';
		//		}
		//		overwrite<<endl;
		//	}
		//	//=============== End of for overwrite if using BEST SOLUTION =======================================//
		
		//================ To record the result and time for each run ============================//
		//recordresult << I <<' '<< ' '<<GLOBAL_BEST_COST<<' '<< ' '<<time<<endl;

		////=============== for overwrite if using CONSTRUCTIVE SOLUTION if run data from CONSTRUCTIVE SOLUTION=======================================//
		if (CONSTRUCTIVE_GLOBAL_BEST - GLOBAL_BEST_COST > 0.001)
		{
			//string overwritesol = "C:\\DataSet\\CCVRP\\CCVRP Christofides_" + std::to_string( I ) + ".txt";
			string overwritesol = "C:\\DataSet\\CCVRP\\CCVRP Golden_" + std::to_string( I ) + ".txt";
			//string overwritesol = "C:\\DataSet\\CCVRP\\CCVRP Li_" + std::to_string( I ) + ".txt";
	
			ofstream overwrite;
			overwrite.open(overwritesol.c_str());

			file.open(filename.c_str());

			if (!overwrite)
			{
				cerr << "File overwrite could not be opened\n";
				exit(1);
			}
		
			cout<<"============== BEST ROUTE IN MAIN ========================="<<endl;
			overwrite.clear();
			for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
			{
				overwrite<<i <<' '<< GLOBAL_Rcost[i]<< ' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<<' ';
				for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
				{
					overwrite<<GLOBAL[i][j] <<' ';
				}
				overwrite<<endl;
			}
		}
		//=============== End of for overwrite if using CONSTRUCTIVE SOLUTION =======================================//

		//=============== to save in CONSTRUCTIVE SOLUTION if run data from the very beginning=======================================//
		
			//string overwritesol = "C:\\DataSet\\CCVRP\\CCVRP Christofides_" + std::to_string( I ) + ".txt";
			////string overwritesol = "C:\\DataSet\\CCVRP\\CCVRP Golden_" + std::to_string( I ) + ".txt";
			////string overwritesol = "C:\\DataSet\\CCVRP\\CCVRP Li_" + std::to_string( I ) + ".txt";
	
			//ofstream overwrite;
			//overwrite.open(overwritesol.c_str());

			//file.open(filename.c_str());

			//if (!overwrite)
			//{
			//	cerr << "File overwrite could not be opened\n";
			//	exit(1);
			//}
		
			////cout<<"============== BEST ROUTE IN MAIN ========================="<<endl;
			//overwrite.clear();
			//for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
			//{
			//	overwrite<<i <<' '<< GLOBAL_Rcost[i]<< ' '<<GLOBAL_SAIZ[i]<<' '<<GLOBAL_capa[i]<<' '<<' ';
			//	for (int j = 0; j <= GLOBAL_SAIZ[i]+1; j++)
			//	{
			//		overwrite<<GLOBAL[i][j] <<' ';
			//	}
			//	overwrite<<endl;
			//}
	
		
		//=============== End of to save in CONSTRUCTIVE SOLUTION if run data from the very beginning=======================================//

		//		//	//================================ for gnuplot ==================================//
		//		//	//ofstream sol_plot("SolPlot_" + std::to_string( I ) + ".txt");
		//		//	//ofstream AllPlot("AllPlot_" + std::to_string( I ) + ".txt");
		//		//
		//		//	//for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		//		//	//{
		//		//	//	if (GLOBAL_SAIZ[i] == 0)
		//		//	//		continue;
		//		//	//	sol_plot << x[SIZE] <<' '<<y[SIZE]<<endl; //depot
		//		//	//	for (int j = 1; j <= GLOBAL_SAIZ[i]; j++)
		//		//	//	{
		//		//	//		sol_plot << x[GLOBAL[i][j]] << ' ' << y[GLOBAL[i][j]]<<endl; //without depot because need to put more new line after depot to tell gnuplot they are diff routes
		//		//	//	}
		//		//	//	sol_plot << x[SIZE] <<' '<<y[SIZE]<<endl <<endl<<endl ;
		//		//	//}

		//		//	//for (int i = 0; i <= SIZE; i++)
		//		//	//{
		//		//	//	AllPlot << x[i] << ' ' <<y[i]<<endl;
		//		//	//}
		//}
		////=============== End of for overwrite if using BEST SOLUTION if run data from BEST SOLUTION=======================================//
		
		//============================= START OF GNU PLOT =====================================//	
		//================================ for gnuplot ==================================//
		//ofstream sol_plot("SolPlot_" + std::to_string( I ) + ".txt");
		//ofstream AllPlot("AllPlot_" + std::to_string( I ) + ".txt");
		//
		//for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		//{
		//	if (GLOBAL_SAIZ[i] == 0)
		//		continue;
		//	sol_plot << x[SIZE] <<' '<<y[SIZE]<<endl; //depot
		//	for (int j = 1; j <= GLOBAL_SAIZ[i]; j++)
		//	{
		//		sol_plot << x[GLOBAL[i][j]] << ' ' << y[GLOBAL[i][j]]<<endl; //without depot because need to put more new line after depot to tell gnuplot they are diff routes
		//	}
		//	sol_plot << x[SIZE] <<' '<<y[SIZE]<<endl <<endl<<endl ;
		//}

		//for (int i = 0; i <= SIZE; i++)
		//{
		//	AllPlot << x[i] << ' ' <<y[i]<<endl;
		//}
		////========== To generate GNU Code ===========//
		//int tem;
		//string tempp;
		//int colorC=0;
		//ofstream gnucommand ("GNUCommand_" + std::to_string( I ) + ".txt");

		//string gnufile = "C:\\DataSet\\colorCode.txt";
		//ifstream gnu("colorCode.txt");

		//gnu.open(gnufile.c_str());

		//if (!gnu)
		//{
		//	cerr << "File could not be opened\n";
		//	exit(1);
		//}
		//
		//gnu >> tem; //how many color code
		//colorC = tem;
		//string *color = new string[colorC];
		//for (int e = 0; e < colorC; e++)
		//{
		//	gnu >> tempp;
		//	color[e] = tempp; //get the color code
		//}
		//int line=1;
		//for (int r = 0; r < GLOBAL_NO_ROUTE; r++)
		//{
		//	if (GLOBAL_SAIZ[r] == 0)
		//		continue;
		//	gnucommand << "set style line "<< std::to_string( line )<<" lc rgb '#"<<color[r]<<"' lt 1 lw 2 pt 7 ps 1.5"<<endl;
		//	line++;
		//}

		//	
		//for (int r = 1; r < line; r++)
		//{
		//	if (r==1)
		//		gnucommand << "plot 'C:\\\\Users\\\\jfs9\\\\Documents\\\\visual Studio 2012\\\\Projects\\\\VNS_try\\\\SolPlot_"<<std::to_string( I )<<".txt' index "<<std::to_string( r-1 )<<" with linespoints ls "<<std::to_string (r)<<" notitle, \\"<<endl;
		//	else
		//		gnucommand << " 'C:\\\\Users\\\\jfs9\\\\Documents\\\\visual Studio 2012\\\\Projects\\\\VNS_try\\\\SolPlot_"<<std::to_string( I )<<".txt' index "<<std::to_string( r-1 )<<" with linespoints ls "<<std::to_string (r)<<" notitle, \\"<<endl;
		//}
		//gnucommand << " 'C:\\\\Users\\\\jfs9\\\\Documents\\\\visual Studio 2012\\\\Projects\\\\VNS_try\\\\AllPlot_"<<std::to_string(I)<<".txt' using 1:2:($0) with labels offset 1.0,1.0 notitle"<<endl; 
		//gnu.close();
		//gnucommand.close();
		//delete[] color;
		//============================= END OF GNU PLOT =====================================//

		//realtimeVRP ();
		

		delete[] x;
		delete[] y;
		delete[] theta;
		delete[] demand;
		delete[] giant_tour;
		delete[] freq;
		delete[] space_available;
		delete[] total_demand;
		delete[] distance_available;
		delete[] service_time;
		delete[] saiz;
		delete[] GLOBAL_SAIZ;
		delete[] LOCAL_SAIZ;
		delete[] route_cost;
		delete[] distance_cost;
		delete[] GLOBAL_capa;
		delete[] LOCAL_capa;
		delete[] GLOBAL_Rcost;
		delete[] GLOBAL_distance_cost;
		delete[] LOCAL_Rcost;
		delete[] LOCAL_distance_cost;
		
		x = NULL; 
		y = NULL; 
		demand = NULL; 
		giant_tour = NULL; 
		freq = NULL; 
		space_available = NULL; 
		total_demand = NULL; 
		distance_available = NULL; 
		service_time = NULL; 
		saiz = NULL; 
		GLOBAL_SAIZ = NULL; 
		LOCAL_SAIZ = NULL; 
		route_cost = NULL; 
		distance_cost = NULL; 
		GLOBAL_capa = NULL; 
		LOCAL_capa = NULL; 
		GLOBAL_Rcost = NULL; 
		GLOBAL_distance_cost = NULL; 
		LOCAL_Rcost = NULL; 
		LOCAL_distance_cost = NULL; 
		//delete[] LOCAL_Bdistance_cost;
		//delete[] LOCAL_Bc;
		//delete[] LOCAL_Bs;
		//delete[] LOCAL_Bq;

		for (int i = 0; i < SIZE + 1; ++i)
		{
			delete[] dist[i];
			delete[] NR_FLAG[i];
			delete[] NR_FLAG_DEPOT[i];
			delete[] DEMAND_FLAG[i];

		}
		delete[] dist;
		delete[] NR_FLAG;
		delete[] NR_FLAG_DEPOT;
		delete[] DEMAND_FLAG;
		dist = NULL; 
		NR_FLAG = NULL; 
		NR_FLAG_DEPOT = NULL; 
		for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
		{
			delete[] GLOBAL[i];
			delete[] LOCAL[i];
			delete[] VEHICLE[i];
			//delete[] LOCAL_B[i];
			delete[] GLOBALCumDist[i];
			delete[] LOCALCumDist[i];
			//delete[] LOCAL_BCumDist[i]; 
			delete[] CumDist[i];
		}
		delete[] GLOBAL;
		delete[] LOCAL;
		delete[] VEHICLE;
		//delete[] LOCAL_B;
		delete[] GLOBALCumDist;
		delete[] LOCALCumDist;
		//delete[] LOCAL_BCumDist;
		delete[] CumDist;
		GLOBAL = NULL;     // Be sure the deallocated memory isn't used.
		LOCAL = NULL;
		VEHICLE = NULL; 
		GLOBALCumDist = NULL; 
		LOCALCumDist = NULL; 
		CumDist = NULL; 

		int stop_s = clock();
		timefile << "Main function time: " << (stop_s - start_s) / float(CLOCKS_PER_SEC) << endl;
		cout << "Main function time: " << (stop_s - start_s) / float(CLOCKS_PER_SEC) << endl;

		timefile.close();
		recordresult << endl;
	}
	
	}//to run many times
	return 0;
}

void checkReverseOrder()
{
	//ofstream see("See_" + std::to_string( I ) + ".txt", ios::app);	
	float reversecost=0;

	for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	{
		reversecost = GLOBAL_SAIZ[i]*GLOBAL_distance_cost[i] + GLOBAL_SAIZ[i]*SERVICE_T - GLOBAL_Rcost[i];

		if (reversecost < GLOBAL_Rcost[i])//if reverse is better
		{
			//int *temp = new int[GLOBAL_SAIZ[i]+2];
			//float *tempCumDist = new float [GLOBAL_SAIZ[i]+2];
			GLOBAL_Rcost[i] = reversecost;
			
			int count = GLOBAL_SAIZ[i]+2;
			int temp;

			for (int t = 0; t < count/2; t++)
			{
				temp = GLOBAL[i][count-t-1];
				GLOBAL[i][count-t-1] = GLOBAL[i][t];
				GLOBAL[i][t] = temp;

			}

			GLOBALCumDist[i][0] =0 ;
			for (int j = 1; j <= GLOBAL_SAIZ[i]+1; j++)
			{
				GLOBALCumDist[i][j] = GLOBALCumDist[i][j-1]+dist[GLOBAL[i][j-1]][GLOBAL[i][j]] + service_time[GLOBAL[i][j]];
				
			}
		}
	}

	//float **RevCumDist = new float* [SIZE];
	//for (int i = 0; i < SIZE; i++)
	//{
	//	RevCumDist[i] = new float [SIZE];
	//}
	//bool change=false;
	//for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	//{
	//	RevCumDist[i][0]= 0;
	//	float RevRCost=0;
	//	std::vector<int> vector_r1;
	//	int k=1;//for RevCumDist
	//	for (int j = GLOBAL_SAIZ[i]; j >= 1; j--)
	//	{
	//		RevCumDist[i][k] = RevCumDist[i][k-1]+dist[GLOBAL[i][j+1]][GLOBAL[i][j]] + service_time[GLOBAL[i][j]];
	//		RevRCost+=RevCumDist[i][k];
	//		vector_r1.push_back(GLOBAL[i][j]);
	//		k++;
	//	}
	//	if (RevRCost < GLOBAL_Rcost[i])//if reverse is better
	//	{
	//		change=true;
	//		GLOBAL_Rcost[i] = RevRCost;
	//		int w=0;//for vector_r1
	//		for (int t = 1; t <= GLOBAL_SAIZ[i]; t++)
	//		{
	//			GLOBAL[i][t] = vector_r1[w];
	//			GLOBALCumDist[i][t]=RevCumDist[i][t];
	//			w++;
	//		}
	//	}
	//	vector_r1.clear();
	//	vector_r1.shrink_to_fit();
	//}
	//if (change==true)//if there is change, display
	//{
	//	see<<endl;
	//	cout<<endl;
	//	see<<"AFter reverse order"<<endl;
	//	cout<<"AFter reverse order"<<endl;
	//	for (int g = 0; g < GLOBAL_NO_ROUTE; g++)
	//	{		
	//		see << g <<' '<< GLOBAL_Rcost[g] <<' '<< GLOBAL_SAIZ[g] <<' '<< GLOBAL_capa[g]<<' '<<' ';
	//		cout << g <<' '<< GLOBAL_Rcost[g] <<' '<< GLOBAL_SAIZ[g] <<' '<< GLOBAL_capa[g]<<' '<<' ';
	//		for (int h = 0; h <= GLOBAL_SAIZ[g] + 1; h++)
	//		{
	//			see << GLOBAL[g][h] << ' ';	
	//			cout<< GLOBAL[g][h] << ' ';	
	//		}
	//		see<<endl;
	//		cout<<endl;
	//	}
	//	for (int i = 0; i < GLOBAL_NO_ROUTE; i++)
	//	{
	//
	//		for (int j = 1; j <=GLOBAL_SAIZ[i]+1; j++)//until th elast customer, last depot also need to calculate because if deletion will need this value
	//		{
	//			
	//			see<<GLOBALCumDist[i][j]<<' ';
	//		}
	//		see<<endl;
	//		//totalcost+=GLOBAL_Rcost[i];
	//	}
	//	see<<endl;
	//}
	//for (int i = 0; i < SIZE; i++)
	//{
	//	delete[] RevCumDist[i];
	//}
	//delete[] RevCumDist;

}

void FindAngleBetweenDepotCustomer (float *x, float *y)
{
	//=================================Find angle between depot to customers ===================================//
	ofstream anglefile("5.Angle_" + std::to_string( I ) + ".txt");
	
	float a = 0;
	float p = 0;
	float q = 0;

	for (int m = 0; m < SIZE; m++)
	{
		p = (y[m] - y[SIZE]);
		q = (x[m] - x[SIZE]);
		a = p / q;

		theta[m] =  atan(a);

		if ((theta[m] < 0) || (theta[m] == -0))  //i added  (theta[n] == -0) on 13 October 2014
		{
			if ((x[m] <= x[SIZE]) && (y[m] >= y[SIZE])) //2nd quadrant
				theta[m] =  PI + theta[m];  //i changed this formula on 13 October 2014, theta[m] is negative so becomes PI+(-theta[m])


			if ((x[m] >= x[SIZE]) && (y[m] <= y[SIZE])) //4th quadrant
				theta[m] =  PI + PI + theta[m];  //i changed this formula on 13 October 2014
		}

		if (theta[m] > 0)
		{
			if ((x[m] < x[SIZE]) && (y[m] < y[SIZE])) //3rd quadrant
				theta[m] =  PI + theta[m];  //i changed this formula on 13 October 2014
		}
		anglefile << "angle [depot][" << m << "]= " << theta[m] << endl;
	}
}

void findDemandFlag(int TOTAL_DEMAND)
{
	ofstream demandflag("demandflag_" + std::to_string( I ) + ".txt");
	for (int i = 0; i < SIZE; i++)
	{
		for (int j = 0; j < SIZE; j++)
		{
			
			DEMAND_FLAG[i][j] = false;//initialize
			
		}
	}
	//find standard deviation
	float sumsquare=0;
	float mean = (float)TOTAL_DEMAND/ float(SIZE);
	for (int i = 0; i < SIZE; i++)
	{
		sumsquare+=(demand[i] - mean)*(demand[i] - mean);
	
	}
	float stddev = sqrt(sumsquare/(float)SIZE);
	demandflag<<"stddev = "<<stddev<<endl;
	for (int i = 0; i < SIZE-1; i++)
	{
		int track=0;
		for (int j = i+1; j < SIZE; j++)
		{	
			if (abs(demand[i] - demand[j]) <= 0)
			{
				DEMAND_FLAG[i][j] = true;
				DEMAND_FLAG[j][i] = true;
				track++;
			}
		}
	}

		float track_flag = 0;
	float percent = 0.0;
		
	for (int i = 0; i < SIZE+1; i++)
	{
		int sum=0;
		for (int j = 0; j < SIZE+1; j++)
		{
			//neighborhoodreduction<<NR_FLAG[i][j]<<' ';
			if (DEMAND_FLAG[i][j] == true)
			{
				track_flag++;//total flag
				sum++;//flag for each row
			}
		}
		demandflag<<' '<<sum<<endl;
	}
	percent = track_flag/((SIZE+1)*(SIZE+1));
	demandflag << "track_flag = " << track_flag <<endl;
	demandflag << "percent = " << percent <<endl;
}