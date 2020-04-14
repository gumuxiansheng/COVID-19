//
// Created by MikeZhu on 2020/4/13.
//

#include "vns.h"
#include "../utils/distance_util.h"
#include "../utils/requirements_util.h"
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

std::vector<int> Vns (const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity)
{
    const int TRAVEL_SIZE = nodes_permutation.size();
    std::vector<int> current_permutation = nodes_permutation;
    int64_t current_cost = covid19::CalcDistanceCost(nodes_permutation, distances);

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

                if (!covid19::CheckRequirements(neighbour, nodes_requirements, capacity))
                {
                    continue;
                }
                int64_t neighbour_cost = covid19::CalcDistanceCost(neighbour, distances);

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

                if (!covid19::CheckRequirements(neighbour, nodes_requirements, capacity))
                {
                    continue;
                }
                int64_t neighbour_cost = covid19::CalcDistanceCost(neighbour, distances);

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

                if (!covid19::CheckRequirements(neighbour, nodes_requirements, capacity))
                {
                    continue;
                }
                int64_t neighbour_cost = covid19::CalcDistanceCost(neighbour, distances);

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

int DecentCmp(int a,int b)
{
    return b<a;
}

std::vector<int> Vns (const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, int num_vehicles, int depot_index)
{
    const int NODE_SIZE = nodes_requirements.size();
    std::vector<int> initial_permutation;
    initial_permutation.push_back(depot_index);

    std::vector<int64_t> sorted_requirements = nodes_requirements;
    std::sort(sorted_requirements.begin(), sorted_requirements.end(), DecentCmp); // sort requirements from large to small

    int64_t cum_requirements  = 0;

    std::vector<int> remain_nodes; // exclude the depot
    for (int j = 1; j < NODE_SIZE; ++j) {
        remain_nodes.push_back(j);
    }
    while (!remain_nodes.empty()) {
        std::vector<int> used_sort_index{};
        for (int i = 0; i < sorted_requirements.size(); ++i) { // start from the max requirement
            if (cum_requirements <= capacity - sorted_requirements[i]){
                cum_requirements += sorted_requirements[i];
                // find the node in remain_nodes
                for (int j = 0; j < remain_nodes.size(); ++j) {
                    int idx = remain_nodes[j];
                    if (nodes_requirements[idx] == sorted_requirements[i]){ // found the node
                        initial_permutation.push_back(idx);
                        used_sort_index.push_back(i);
                        remain_nodes.erase(std::begin(remain_nodes) + j);
                        break;
                    }
                }

            }
        }
        for (int k = 0; k < used_sort_index.size(); ++k) {
            int idx = used_sort_index[k] - k;
            sorted_requirements.erase(std::begin(sorted_requirements) + idx);
        }

        used_sort_index.clear();

        initial_permutation.push_back(depot_index);
        cum_requirements = 0;
    }
    return Vns(initial_permutation, distances, nodes_requirements, capacity);
}
}