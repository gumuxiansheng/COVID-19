//
// Created by MikeZhu on 2020/5/5.
//

#include <numeric>
#include "algorithms/vns.h"
#include "algorithms/cost.h"
#include "utils/file_util.h"
#include "utils/distance_util.h"
#include "utils/requirements_util.h"

namespace covid19
{
struct DataModel
{
    std::vector<std::vector<int64_t>> distance_matrix{};
    std::vector<int64_t> demands{};
    int64_t vehicle_capacity{1};
    int num_vehicles = 1;
    int depot{0};
};

DataModel initDataModel(){
    DataModel data;

    std::string file_url;
    std::cout << "please enter the Christofides file url:" << std::endl;
    std::cin >> file_url;

    covid19::ChristofidesDataModel christofides_data = covid19::ReadChristofides(file_url);

    data.distance_matrix = std::move(covid19::CalcDistances(christofides_data.nodes));
    data.demands = std::move(covid19::GetNodesRequirements(christofides_data.nodes));
    int64_t sum_demands = accumulate(data.demands.begin(), data.demands.end(), 0);
    data.num_vehicles = sum_demands/christofides_data.capacity + 1;
    data.vehicle_capacity = christofides_data.capacity;

    return data;
}

void PrintSolution(const DataModel &data, const std::vector<int> &solution)
{
    const int START_NODE = solution[0];
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
        if (index == START_NODE) {
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
    DataModel data = initDataModel();
    std::vector<int> solution = covid19::Vns("cumdistance", data.distance_matrix, data.demands, data.vehicle_capacity, data.num_vehicles, data.depot);

    PrintSolution(data, solution);
}
}

int main(int argc, char **argv)
{
    covid19::VrpCapacity();
    return 0;
}