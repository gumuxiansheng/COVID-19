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

namespace covid19
{
struct DataModel
{
    std::vector<std::vector<int64_t>> distance_matrix{};
    std::vector<int64_t> demands{};
    int64_t vehicle_capacity{1};
    std::vector<int> num_vehicles{5};
    std::vector<int> depot{0};
};

DataModel initDataModel()
{
    DataModel data;

    std::string file_url;
    std::cout << "please enter the Christofides file url:" << std::endl;
    std::cin >> file_url;

    covid19::ChristofidesDataModel christofides_data = covid19::ReadChristofides(file_url);

    data.distance_matrix = std::move(covid19::CalcDistances(christofides_data.nodes));
    data.demands = std::move(covid19::GetNodesRequirements(christofides_data.nodes));
    int64_t sum_demands = accumulate(data.demands.begin(), data.demands.end(), 0);
    data.vehicle_capacity = christofides_data.capacity;

    return data;
}

DataModel initPRDataModel(std::string file_url)
{
    DataModel data;

    covid19::PRDataModel pr_data = covid19::ReadPR(file_url);

    std::cout << "distance_matrix start: ";
    data.distance_matrix = std::move(covid19::CalcDistances(pr_data.nodes, DistanceType::euclidean));
    data.demands = std::move(covid19::GetNodesRequirements(pr_data.nodes, 4));
    int64_t sum_demands = accumulate(data.demands.begin(), data.demands.end(), 0);
    data.vehicle_capacity = pr_data.capacity;
    data.depot = pr_data.depot_indexes;

    int total_vehicles = 11;
    std::random_device rd;
    std::default_random_engine e{rd()};
    std::uniform_int_distribution<unsigned> u(0, total_vehicles);
    std::vector<unsigned> vehicles_splits{};
    std::cout << "data.depot.size(): " << data.depot.size() << std::endl;
    for (size_t i = 0; i < data.depot.size() - 1; i++)
    {
        unsigned num = u(e);
        std::cout << num << "  ";
        vehicles_splits.push_back(num);
    }
    sort(vehicles_splits.begin(), vehicles_splits.end());

    std::vector<int> vehicles{};
    std::cout << "vehicles: ";
    for (size_t i = 0; i < data.depot.size(); i++)
    {
        if (i == 0)
        {
            vehicles.push_back(vehicles_splits[i]);
        } else if (i == data.depot.size() - 1)
        {
            vehicles.push_back(total_vehicles - vehicles_splits[i - 1]);
        } else 
        {
            vehicles.push_back(vehicles_splits[i] - vehicles_splits[i - 1]);
        }
        std::cout << vehicles[i] << "  ";
    }
    data.num_vehicles = vehicles;

    std::cout << "initPRDataModel succeed" << std::endl;

    return data;
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

    covid19::WriteResults(solution, "solution.txt");
}

void VrpCapacity()
{
    std::vector<int64_t> round_costs{};
    std::string file_url;
    std::cout << "please enter the PR file url:" << std::endl;
    std::cin >> file_url;
    for (size_t i = 0; i < 2; i++)
    {
        std::cout << "Round: " << i << std::endl;
        // DataModel data = initDataModel();
        DataModel data = initPRDataModel(file_url);
        // Use distance type to calc the inital solution
        // std::vector<int> init_solution = Vns ("distance", data.distance_matrix, data.demands, data.vehicle_capacity, data.num_vehicles, data.depot);
        std::vector<int> init_solution = RegretInsersion ("cumdistance", data.distance_matrix, data.demands, data.vehicle_capacity, data.num_vehicles, data.depot);
        std::vector<int> solution = covid19::Vns("cumdistance", init_solution, data.distance_matrix, data.demands, data.vehicle_capacity, data.depot);

        // PrintSolution(data, solution);
        round_costs.push_back(covid19::CalcDistanceCumCost(solution, data.distance_matrix, data.depot));
    }

    std::cout << "All costs:" << std::endl;
    for (auto cost : round_costs)
    {
        std::cout << cost << ", ";
    }
    std::cout << std::endl;
    

}
}

int main(int argc, char **argv)
{
    covid19::VrpCapacity();
    return 0;
}