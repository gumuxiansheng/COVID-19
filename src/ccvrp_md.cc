//
// Created by MikeZhu on 2020/5/5.
//

#include <numeric>
#include "algorithms/vns.h"
#include "algorithms/cost.h"
#include "algorithms/init_solution.h"
#include "utils/file_util.h"
#include "utils/distance_util.h"
#include "utils/requirements_util.h"
#include "utils/vector_help.h"
#include <random>
#include <ctime>
#include <algorithm>
#include <map>

namespace covid19
{

const int16_t INNER_ROUND = 1;
const int16_t OUTER_ROUND = 1;
const std::string ASSIGN_VEHICLES_ALG = "regret"; // regret, uniform, uniform_random, uniform_reverse, random
const std::map<std::string, int> VEHICLE_NUM_MAP = {
    {"p01_1.txt", 11},
    {"p02_1.txt", 5},
    {"p03_1.txt", 11},
    {"p04_1.txt", 15},
    {"p05_1.txt", 8},
    {"p06_1.txt", 16},
    {"p07_1.txt", 16},
    {"p08_1.txt", 25},
    {"p09_1.txt", 26},
    {"p10_1.txt", 26},
    {"p11_1.txt", 26},
    {"p12_1.txt", 8},
    {"p13_1.txt", 9},
    {"p14_1.txt", 10},
    {"p15_1.txt", 16},
    {"p16_1.txt", 17},
    {"p17_1.txt", 18},
    {"p18_1.txt", 24},
    {"p19_1.txt", 25},
    {"p20_1.txt", 26},
    {"p21_1.txt", 34},
    {"p22_1.txt", 35},
    {"p23_1.txt", 36},
    {"pr01_1.txt", 4},
    {"pr02_1.txt", 8},
    {"pr03_1.txt", 11},
    {"pr04_1.txt", 14},
    {"pr05_1.txt", 19},
    {"pr06_1.txt", 23},
    {"pr07_1.txt", 6},
    {"pr08_1.txt", 12},
    {"pr09_1.txt", 17},
    {"pr10_1.txt", 24},
    {"lr01_1.txt", 5},
    {"lr02_1.txt", 5},
    {"lr03_1.txt", 5},
    {"lr04_1.txt", 10},
    {"lr05_1.txt", 10},
    {"lr06_1.txt", 10},
    {"lr07_1.txt", 20},
    {"lr08_1.txt", 20},
    {"lr09_1.txt", 20},
    {"lr10_1.txt", 20},
    {"lr11_1.txt", 20},
    {"lr12_1.txt", 20},
    {"lr13_1.txt", 25},
    {"lr14_1.txt", 25},
    {"lr15_1.txt", 25},
    {"lr16_1.txt", 25},
    {"lr17_1.txt", 25},
    {"lr18_1.txt", 25},
};
const std::vector<std::string> P_FILES{
    // "p01_1.txt",
    // "p02_1.txt",
    // "p03_1.txt",
    // "p04_1.txt",
    // "p05_1.txt",
    // "p06_1.txt",
    // "p07_1.txt",
    // "p08_1.txt",
    // "p09_1.txt",
    // "p10_1.txt",
    // "p11_1.txt",
    // "p12_1.txt",
    // "p13_1.txt",
    // "p14_1.txt",
    // "p15_1.txt",
    // "p16_1.txt",
    // "p17_1.txt",
    // "p18_1.txt",
    // "p19_1.txt",
    // "p20_1.txt",
    //  "p21_1.txt",
    // "p22_1.txt",
   "p23_1.txt",
};
const std::vector<std::string> PR_FILES{
    // "pr01_1.txt",
    // "pr02_1.txt",
    // "pr03_1.txt",
    // "pr04_1.txt",
    // "pr05_1.txt",
    // "pr06_1.txt",
    "pr07_1.txt",
    "pr08_1.txt",
    "pr09_1.txt",
    "pr10_1.txt",
};
const std::vector<std::string> LR_FILES{
    // "lr01_1.txt",
    // "lr02_1.txt",
    // "lr03_1.txt",
    // "lr04_1.txt",
    // "lr05_1.txt",
    // "lr06_1.txt",
    // "lr07_1.txt",
    // "lr08_1.txt",
    // "lr09_1.txt",
    "lr10_1.txt",
    // "lr11_1.txt",
    // "lr12_1.txt",
    // "lr13_1.txt",
    // "lr14_1.txt",
    // "lr15_1.txt",
    // "lr16_1.txt",
    // "lr17_1.txt",
    // "lr18_1.txt",
};

struct DataModel
{
    std::vector<std::vector<int64_t>> distance_matrix{};
    std::vector<int64_t> demands{};
    int64_t vehicle_capacity{1};
    int vehicles_num_total{5};
    std::vector<int> num_vehicles{5};
    std::vector<int> depot{0};
};

void AssignVehicles(DataModel &data, const int &input_vehicles, std::string type);

DataModel initDataModel()
{
    DataModel data;

    std::string file_url;
    std::cout << "please enter the Christofides file url:" << std::endl;
    std::cin >> file_url;

    covid19::ChristofidesDataModel christofides_data = covid19::ReadChristofides(file_url);

    data.distance_matrix = std::move(covid19::CalcDistances(christofides_data.nodes));
    data.demands = std::move(covid19::GetNodesRequirements(christofides_data.nodes));
    int64_t sum_demands = std::accumulate(data.demands.begin(), data.demands.end(), 0);
    data.vehicle_capacity = christofides_data.capacity;

    return data;
}

DataModel initPRDataModel(const std::string file_url, const std::string type = "p", const int assign_vehicles = -1)
{
    DataModel data;

    // covid19::PRDataModel pr_data = covid19::ReadPR(file_url);
    covid19::PRDataModel pr_data = type == "lr" ? covid19::ReadLRFloat(file_url) : covid19::ReadPRFloat(file_url);

    std::cout << "distance_matrix start: ";
    int locx, locy, locr{};
    if (type == "p" || type == "pr")
    {
        locx = 1;
        locy = 2;
        locr = 4;
    } else if (type == "lr")
    {
        locx = 0;
        locy = 1;
        locr = 2;
    }
    data.distance_matrix = std::move(covid19::CalcDistances(pr_data.nodes, DistanceType::euclidean, locx, locy));
    data.demands = std::move(covid19::GetNodesRequirements(pr_data.nodes, locr));
    data.vehicles_num_total = pr_data.vehicles;
    data.vehicle_capacity = pr_data.capacity;
    data.depot = pr_data.depot_indexes;

    if (assign_vehicles > 0)
    {
        data.vehicles_num_total = assign_vehicles;
    }
    AssignVehicles(data, data.vehicles_num_total, ASSIGN_VEHICLES_ALG);

    std::cout << std::endl << "initPRDataModel succeed" << std::endl;

    return data;
}

void AssignVehicles(DataModel &data, const int &input_vehicles, std::string type)
{
    int64_t sum_demands = std::accumulate(data.demands.begin(), data.demands.end(), 0);
    int total_vehicles = std::ceil((float_t)sum_demands / (float_t)data.vehicle_capacity);
    std::cout << "sum_demands: " << sum_demands << ", total_vehicles: " << total_vehicles << std::endl;
    if (input_vehicles > total_vehicles)
    {
        total_vehicles = input_vehicles;
    }

    if (type == "uniform")
    {
        // distribute vehicles uniformly
        std::vector<int> vehicles{};
        int depots_num = data.depot.size();
        int mean_vehicle_num = total_vehicles / depots_num;
        for (size_t i = 0; i < depots_num; i++)
        {
            vehicles.push_back(mean_vehicle_num);
        }

        for (size_t i = 0; i < total_vehicles % depots_num; i++)
        {
            vehicles[i] += 1;
        }

        data.num_vehicles = vehicles;
    } else if (type == "uniform_reverse")
    {
        // distribute vehicles uniformly
        std::vector<int> vehicles{};
        int depots_num = data.depot.size();
        int mean_vehicle_num = total_vehicles / depots_num;
        for (size_t i = 0; i < depots_num; i++)
        {
            vehicles.push_back(mean_vehicle_num);
        }

        for (size_t i = 0; i < total_vehicles % depots_num; i++)
        {
            int index = depots_num - 1 - i;
            vehicles[index] += 1;
        }

        data.num_vehicles = vehicles;
    } else if (type == "uniform_random")
    {
        // distribute vehicles uniformly
        std::vector<int> vehicles{};
        int depots_num = data.depot.size();
        int mean_vehicle_num = total_vehicles / depots_num;
        for (size_t i = 0; i < depots_num; i++)
        {
            vehicles.push_back(mean_vehicle_num);
        }

        std::random_device rd;
        std::default_random_engine e{rd()};
        std::uniform_int_distribution<int> u(0, total_vehicles % depots_num - 1);

        for (size_t i = 0; i < total_vehicles % depots_num; i++)
        {
            int index = u(e);
            vehicles[index] += 1;
        }

        data.num_vehicles = vehicles;
    } else if (type == "random")
    {
        // randomise vehicle distribution
        std::random_device rd;
        std::default_random_engine e{rd()};
        std::uniform_int_distribution<unsigned> u(0, total_vehicles);
        std::vector<unsigned> vehicles_splits{};
        std::cout << "data.depot.size(): " << data.depot.size() << std::endl;
        for (size_t i = 0; i < data.depot.size() - 1; i++)
        {
            unsigned num = u(e);
            vehicles_splits.push_back(num);
        }
        std::sort(vehicles_splits.begin(), vehicles_splits.end());

        std::vector<int> vehicles{};
        for (size_t i = 0; i < data.depot.size(); i++)
        {
            if (i == 0)
            {
                vehicles.push_back(vehicles_splits[i]);
            }
            else if (i == data.depot.size() - 1)
            {
                vehicles.push_back(total_vehicles - vehicles_splits[i - 1]);
            }
            else
            {
                vehicles.push_back(vehicles_splits[i] - vehicles_splits[i - 1]);
            }
        }
        data.num_vehicles = vehicles;
    } else
    {
        data.num_vehicles = std::move(RegretInsersionAssign(type, data.distance_matrix,  data.demands, data.vehicle_capacity, data.depot, total_vehicles));
    }
    
}

void PrintSolution(const DataModel &data, const std::vector<int> &solution)
{
    int num_travel = 0;
    std::vector<int> sub_solution;
    for (int i = 0; i < solution.size(); ++i) {
        int index = solution[i];
        std::cout << index << " -> ";
    }
    std::cout << std::endl;

    for (int i = 0; i < solution.size(); ++i) {
        int index = solution[i];
        sub_solution.push_back(index);
        if (IsIn(index, data.depot)) {
            if (sub_solution.size() != 1){
                std::cout << index << " Load(" << data.demands[index] << ")" << std::endl;
                std::cout << "Travel " << num_travel << "'s distance: " << covid19::CalcDistanceCost(sub_solution, data.distance_matrix, data.depot) << std::endl;
                std::cout << "Travel " << num_travel << "'s min-sum distance: " << covid19::CalcDistanceCumCost(sub_solution, data.distance_matrix, data.depot) << std::endl;
                std::cout << "Travel " << num_travel << "'s load: " << covid19::CalcRequirements(sub_solution, data.demands) << std::endl;
                sub_solution.clear();
            } else if (i != solution.size() - 1){
                std::cout << "Start travel " << ++num_travel << std::endl;
                std::cout << index << " Load(" << data.demands[index] << ")" << " -> ";
            }

        } else {
            std::cout << index << " Load(" << data.demands[index] << ")" << " -> ";
        }

    }

    std::cout << std::endl;
    std::cout << "Total route distance: " << covid19::CalcDistanceCost(solution, data.distance_matrix, data.depot) << std::endl;
    std::cout << "Total route min-sum distance: " << covid19::CalcDistanceCumCost(solution, data.distance_matrix, data.depot) << std::endl;

}

void WriteResults(const DataModel &data, const std::vector<int> &nodes_permutation, const double run_time, const std::string &file_name)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.is_open())
    {
        std::cout << "file open failed" << std::endl;
    }

    int num_travel = 0;
    std::vector<int> sub_solution;
    for (int i = 0; i < nodes_permutation.size(); ++i) {
        int index = nodes_permutation[i];
        sub_solution.push_back(index);
        if (IsIn(index, data.depot)) {
            if (sub_solution.size() != 1){
                outfile << num_travel++ << " ";
                outfile << covid19::CalcDistanceCumCost(sub_solution, data.distance_matrix, data.depot) << " ";
                outfile << sub_solution.size() - 2 << " ";
                outfile << covid19::CalcRequirements(sub_solution, data.demands) << "    ";
                for (auto item : sub_solution)
                {
                    outfile << item << " ";
                }
                outfile << std::endl;
                
                sub_solution.clear();
            }

        }

    }
    outfile << "BEST COST= " << covid19::CalcDistanceCumCost(nodes_permutation, data.distance_matrix, data.depot);

    outfile << "    RUN TIME= " << run_time;

    outfile << "    TOTAL VEHICLES= " << data.vehicles_num_total;

    outfile.close();
}

void OverwriteInitial(const DataModel &data, const std::vector<int> &nodes_permutation, const double run_time, const std::string &file_name)
{
    covid19::WriteResults(data, nodes_permutation, run_time, file_name);
}

void VrpCapacity(const std::string &data_folder, const std::string &file_name, const int round, std::string type)
{
    std::vector<int64_t> round_costs{};
    std::vector<std::vector<int>> round_solutions{};
    std::vector<double> round_time{};

    std::string file_url = data_folder + file_name;
    std::string initial_solution_file_url = data_folder + "/initial_solution/" + file_name;

    DataModel data;

    std::vector<int> init_solution = ReadResultSolution(initial_solution_file_url);
    // std::cout << "INIT SOLUTION" << std::endl;
    // PrintSolution(data, init_solution);

    for (size_t i = 0; i < INNER_ROUND; i++)
    {
        clock_t start_time, end_time;
        std::cout << "Round: " << i << std::endl;
        auto vehicle_map_iter = VEHICLE_NUM_MAP.find(file_name);
        int assign_vehicles = -1;
        if (vehicle_map_iter != VEHICLE_NUM_MAP.end())
        {
            assign_vehicles = vehicle_map_iter->second;
        }
        data = initPRDataModel(file_url, type, assign_vehicles);

        start_time = clock();

        std::vector<int> solution = covid19::Vns("cumdistance", init_solution, data.distance_matrix, data.demands, data.vehicle_capacity, data.depot, data.vehicles_num_total);

        end_time = clock();

        double run_time = (double)(end_time - start_time) / CLOCKS_PER_SEC;
        round_time.push_back(run_time);

        PrintSolution(data, solution);
        round_costs.push_back(covid19::CalcDistanceCumCost(solution, data.distance_matrix, data.depot));
        round_solutions.push_back(solution);
    }

    std::cout << "All costs:" << std::endl;

    int64_t min_cost = INT64_MAX;
    int min_index = -1;
    for (size_t i = 0; i < round_costs.size(); i++)
    {
        int64_t cost = round_costs[i];
        if (cost < min_cost)
        {
            min_cost = cost;
            min_index = i;
        }
        std::cout << cost << ", ";
    }
    std::cout << std::endl;
    std::string out_file = file_url + "." + std::to_string(round) + "." + std::to_string(min_cost) + ".res";
    covid19::WriteResults(data, round_solutions[min_index], round_time[min_index], out_file);

    int64_t initial_cost = covid19::CalcDistanceCumCost(init_solution, data.distance_matrix, data.depot);
    if (min_cost < initial_cost)
    {
        covid19::OverwriteInitial(data, round_solutions[min_index], round_time[min_index], initial_solution_file_url);
    }
}

std::vector<std::string> GetFileNames(std::string type, std::string data_folder)
{
    if (type == "p")
    {
        return P_FILES;
    }
    else if (type == "pr")
    {
        return PR_FILES;
    }
    else if (type == "lr")
    {
        return LR_FILES;
    } else
    {
        return {};
    }
    

}

void InitialSolution(const std::string &type, const std::string &data_folder,  const std::string &file_name)
{
    std::string file_url = data_folder + file_name;
    auto vehicle_map_iter = VEHICLE_NUM_MAP.find(file_name);
    int assign_vehicles = -1;
    if (vehicle_map_iter != VEHICLE_NUM_MAP.end())
    {
        assign_vehicles = vehicle_map_iter->second;
    }
    DataModel data = initPRDataModel(file_url, type, assign_vehicles);
    std::vector<int> init_solution = RegretInsersion("cumdistance", data.distance_matrix, data.demands, data.vehicle_capacity, data.num_vehicles, data.depot);
    // std::vector<int> init_solution = Greedy(data.demands, data.vehicle_capacity, data.num_vehicles, data.depot);

    const char *mkdir_code = ("mkdir " + data_folder + "initial_solution/").c_str();
    system(mkdir_code);

    std::string out_file = data_folder + "initial_solution/" + file_name;

    std::cout << "WriteResults:" << out_file << std::endl;
    covid19::WriteResults(data, init_solution, 0, out_file);
}

void InitialSolutionWithFolder(const std::string& type, const std::string& data_folder)
{
    auto files = GetFileNames(type, data_folder);

    for (auto &&file_name : files)
    {
        InitialSolution(type, data_folder, file_name);
    }
}

void VrpCapacityWithFolder(const std::string &type, const std::string &data_folder, const int round)
{

    auto files = GetFileNames(type, data_folder);

    for (auto &&file_name : files)
    {
        VrpCapacity(data_folder, file_name, round, type);
    }
}
}

int main(int argc, char **argv)
{
    std::cout << "p or pr or lr?" << std::endl;
    std::string type;
    std::cin >> type;
    std::string folder = "/Users/mikezhu/Dev/CPP/COVID-19/data/demo2/";
    if (type == "lr")
    {
        folder += "lr/";
    }

    covid19::InitialSolutionWithFolder(type, folder);

    for (size_t i = 0; i < covid19::OUTER_ROUND; i++)
    {
        // covid19::InitialSolutionWithFolder(type, folder);
        covid19::VrpCapacityWithFolder(type, folder, i);
    }

    return 0;
}