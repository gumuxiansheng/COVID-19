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

namespace covid19
{

const int16_t INNER_ROUND = 1;
const int16_t OUTER_ROUND = 3;

struct DataModel
{
    std::vector<std::vector<int64_t>> distance_matrix{};
    std::vector<int64_t> demands{};
    int64_t vehicle_capacity{1};
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

DataModel initPRDataModel(std::string file_url)
{
    DataModel data;

    // covid19::PRDataModel pr_data = covid19::ReadPR(file_url);
    covid19::PRDataModel pr_data = covid19::ReadPRFloat(file_url);

    std::cout << "distance_matrix start: ";
    data.distance_matrix = std::move(covid19::CalcDistances(pr_data.nodes, DistanceType::euclidean, 1, 2));
    data.demands = std::move(covid19::GetNodesRequirements(pr_data.nodes, 4));
    data.vehicle_capacity = pr_data.capacity;
    data.depot = pr_data.depot_indexes;

    AssignVehicles(data, pr_data.vehicles, "uniform");

    std::cout << std::endl << "initPRDataModel succeed" << std::endl;

    return data;
}

void AssignVehicles(DataModel &data, const int &input_vehicles, std::string type)
{
    int64_t sum_demands = std::accumulate(data.demands.begin(), data.demands.end(), 0);
    int total_vehicles = std::ceil((float_t)sum_demands / (float_t)data.vehicle_capacity);
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
    } else
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
        std::cout << "vehicles: ";
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
            std::cout << vehicles[i] << "  ";
        }
        data.num_vehicles = vehicles;
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

    outfile.close();
}

void VrpCapacity(const std::string &data_folder, const std::string &file_name, const int round)
{
    std::vector<int64_t> round_costs{};
    std::vector<std::vector<int>> round_solutions{};
    std::vector<double> round_time{};

    std::string file_url = data_folder + file_name;
    std::string initial_solution_file_url = data_folder + "/initial_solution/" + file_name;

    DataModel data;
    for (size_t i = 0; i < INNER_ROUND; i++)
    {
        clock_t start_time, end_time;
        std::cout << "Round: " << i << std::endl;
        data = initPRDataModel(file_url);

        start_time = clock();

        std::vector<int> init_solution = ReadResultSolution(initial_solution_file_url);

        std::vector<int> solution = covid19::Vns("cumdistance", init_solution, data.distance_matrix, data.demands, data.vehicle_capacity, data.depot);

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

}

void InitialSolutionWithFolder(std::string type, std::string data_folder)
{
    int file_num;
    if (type == "p")
    {
        file_num = 18;
    }
    else if (type == "pr")
    {
        file_num = 10;
    }
    else
    {
        return;
    }

    for (size_t i = 1; i < file_num + 1; i++)
    {
        if (i == 16 || i == 17)
        {
            continue;
        }
        std::string item = i < 10 ? "0" + std::to_string(i) : std::to_string(i);
        std::string file_url = data_folder + type + item + "_1.txt";

        DataModel data = initPRDataModel(file_url);
        std::vector<int> init_solution = RegretInsersion("cumdistance", data.distance_matrix, data.demands, data.vehicle_capacity, data.num_vehicles, data.depot);

        const char *mkdir_code = ("mkdir " + data_folder + "initial_solution/").c_str();
        system(mkdir_code);

        std::string out_file = data_folder + "initial_solution/" + type + item + "_1.txt";

        std::cout << "WriteResults:" << out_file << std::endl;
        covid19::WriteResults(data, init_solution, 0, out_file);
    }
}

void VrpCapacityWithFolder(const std::string &type, const std::string &data_folder, const int round)
{

    int file_num;
    if (type == "p")
    {
        file_num = 18;
    } else if (type == "pr")
    {
        file_num = 10;
    } else 
    {
        return;
    }

    for (size_t i = 1; i < file_num + 1; i++)
    {
        if (i == 16 || i == 17)
        {
            continue;
        }

        std::string item = i < 10 ? "0"  + std::to_string(i) : std::to_string(i);

        VrpCapacity(data_folder, type + item + "_1.txt", round);
    }
}
}

int main(int argc, char **argv)
{
    std::cout << "p or pr?" << std::endl;
    std::string type;
    std::cin >> type;
    std::string folder = "/Users/mikezhu/Dev/CPP/COVID-19/data/demo/";

    covid19::InitialSolutionWithFolder(type, folder);

    for (size_t i = 0; i < covid19::OUTER_ROUND; i++)
    {
        covid19::VrpCapacityWithFolder(type, folder, i);
    }

    return 0;
}