//
// Created by MikeZhu on 2020/4/13.
//

#include "vns.h"
#include "../utils/distance_util.h"
#include "../utils/requirements_util.h"
#include "../utils/vector_help.h"
#include "init_solution.h"
#include "cost.h"
#include <algorithm>

namespace covid19
{

std::vector<int> TwoSwap(const std::vector<int>& nodes_permutation, int swap_start_index, int swap_end_index)
{
    const int TRAVEL_SIZE = nodes_permutation.size();
    std::vector<int> v;
    for (int i = 0; i < TRAVEL_SIZE; ++i)
    {
        if (i == swap_start_index)
        {
            v.push_back(nodes_permutation[swap_end_index]);
        } else if(i == swap_end_index)
        {
            v.push_back(nodes_permutation[swap_start_index]);
        } else {
            v.push_back(nodes_permutation[i]);
        }
    }

    return v;
}

std::vector<int> TwoOptSwap(const std::vector<int>& nodes_permutation, int swap_start_index, int swap_end_index)
{
    const int TRAVEL_SIZE = nodes_permutation.size();
    std::vector<int> v;
    for (int i = 0; i < swap_start_index; i++)
    {
        v.push_back(nodes_permutation[i]);
    }
    for (int i = swap_end_index; i >= swap_start_index; i--)
    {
        v.push_back(nodes_permutation[i]);
    }
    for (int i = swap_end_index + 1; i < TRAVEL_SIZE; i++)
    {
        v.push_back(nodes_permutation[i]);
    }

    return v;

}

std::vector<int> TwoHOptSwap(const std::vector<int>& nodes_permutation, int swap_1, int swap_2)
{
    const int TRAVEL_SIZE = nodes_permutation.size();
    std::vector<int> v;
    if (swap_1 > swap_2){
        int temp = swap_1;
        swap_1 = swap_2;
        swap_2 = temp;
    }

    v.push_back(nodes_permutation[swap_1]);
    v.push_back(nodes_permutation[swap_2]);

    for (int i = 1; i < TRAVEL_SIZE; ++i)
    {
        int idx = (swap_1 + i) % TRAVEL_SIZE;
        if (idx != swap_2)
        {
            v.push_back(nodes_permutation[idx]);
        }
    }

    return v;

}

std::vector<int> Vns (const std::string type, const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& depot_indexes)
{
    const int TRAVEL_SIZE = nodes_permutation.size();
    std::vector<int> current_permutation = nodes_permutation;
    int64_t current_cost = CalcCost(type, nodes_permutation, distances, depot_indexes);

    int count = 0;
    int max_no_improve = 10;

    std::cout << "STAGE 1: " << std::endl;
    do
    {
        count++;
        for (int i = 1; i < TRAVEL_SIZE - 2; i++)
        {
            for (int k = i + 1; k < TRAVEL_SIZE - 1; k++)
            {
                std::vector<int> neighbour = TwoOptSwap(current_permutation, i, k);

                if (!covid19::CheckMultiDepotRequirements(neighbour, nodes_requirements, capacity, depot_indexes))
                {
                    continue;
                }
                int64_t neighbour_cost = CalcCost(type, neighbour, distances, depot_indexes);

                if (current_cost > neighbour_cost)
                {
                    current_permutation = neighbour;
                    current_cost = neighbour_cost;
                    std::cout << "current_cost: " << current_cost << std::endl;
                    count = 0;
                }

            }
        }

    } while (count <= max_no_improve);

    std::cout << "STAGE 2: " << std::endl;
    do
    {
        count++;
        for (int i = 1; i < TRAVEL_SIZE - 2; i++)
        {
            for (int k = i + 1; k < TRAVEL_SIZE - 1; k++)
            {
                std::vector<int> neighbour = TwoSwap(current_permutation, i, k);

                if (!covid19::CheckMultiDepotRequirements(neighbour, nodes_requirements, capacity, depot_indexes))
                {
                    continue;
                }
                int64_t neighbour_cost = CalcCost(type, neighbour, distances, depot_indexes);

                if (current_cost > neighbour_cost)
                {
                    current_permutation = neighbour;
                    current_cost = neighbour_cost;
                    std::cout << "current_cost: " << current_cost << std::endl;
                    count = 0;
                }

            }
        }

    } while (count <= max_no_improve);

    std::cout << "STAGE 3: " << std::endl;

    std::vector<int> inline_permutation = nodes_permutation;
    inline_permutation.erase(std::begin(inline_permutation));
    inline_permutation.pop_back();
    do
    {
        count++;
        for (int i = 1; i < TRAVEL_SIZE - 3; i++)
        {
            for (int k = i + 1; k < TRAVEL_SIZE - 2; k++)
            {
                inline_permutation = TwoHOptSwap(inline_permutation, i, k);
                std::vector<int> neighbour;
                neighbour.push_back(nodes_permutation[0]);
                for (int j : inline_permutation) {
                    neighbour.push_back(j);
                }
                neighbour.push_back(nodes_permutation[nodes_permutation.size() - 1]);

                if (!covid19::CheckMultiDepotRequirements(neighbour, nodes_requirements, capacity, depot_indexes))
                {
                    continue;
                }
                int64_t neighbour_cost = CalcCost(type, neighbour, distances, depot_indexes);

                if (current_cost > neighbour_cost)
                {
                    current_permutation = neighbour;
                    current_cost = neighbour_cost;
                    std::cout << "current_cost: " << current_cost << std::endl;
                    count = 0;
                }

            }
        }

    } while (count <= max_no_improve);

    return current_permutation;
}

std::vector<int> Vns (const std::string type, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes)
{
    std::vector<int> initial_permutation = Greedy(nodes_requirements, capacity, num_vehicles, depot_indexes);
    return Vns(type, initial_permutation, distances, nodes_requirements, capacity, depot_indexes);
}

std::vector<int> Vns (const std::string type, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, int num_vehicles, int depot_index)
{
    std::vector<int> num_vehicles_list{num_vehicles};
    std::vector<int> depot_indexes{depot_index};
    return Vns(type, distances, nodes_requirements, capacity, num_vehicles_list, depot_indexes);
}
}