//
// Created by MikeZhu on 2020/5/12.
//

#include "init_solution.h"
#include "../utils/vector_help.h"

namespace covid19
{

int DecentCmp(int a,int b)
{
    return b<a;
}

std::vector<int> Greedy (const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes)
{
    const int NODE_SIZE = nodes_requirements.size();
    const int DEPOT_SIZE = depot_indexes.size();
    std::vector<int> initial_permutation;
    std::vector<int> num_vehicles_t(num_vehicles);
    int current_depot_i = 0;

    std::vector<int64_t> sorted_requirements = nodes_requirements;
    std::sort(sorted_requirements.begin(), sorted_requirements.end(), DecentCmp); // sort requirements from large to small

    int64_t cum_requirements  = 0;

    std::vector<int> remain_nodes;
    for (int j = 0; j < NODE_SIZE; ++j) 
    {
        // exclude the depot
        if (IsIn(j, depot_indexes)){
            continue;
        }
        remain_nodes.push_back(j);
    }
    while (!remain_nodes.empty()) 
    {
        int depot_iter_count = 0;
        while (num_vehicles_t[current_depot_i] <= 0 && depot_iter_count < DEPOT_SIZE)
        {
            current_depot_i = (current_depot_i + 1) % DEPOT_SIZE;
            depot_iter_count++;
        }
        depot_iter_count = 0;
        
        initial_permutation.push_back(depot_indexes[current_depot_i]);
        num_vehicles_t[current_depot_i]--;
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

        initial_permutation.push_back(depot_indexes[current_depot_i]);
        current_depot_i = (current_depot_i + 1) % DEPOT_SIZE;
        cum_requirements = 0;
    }
    for (size_t i = 0; i < num_vehicles_t.size(); i++)
    {
        while (num_vehicles_t[i] > 0) 
        {
            initial_permutation.push_back(depot_indexes[i]);
            initial_permutation.push_back(depot_indexes[i]); // push twice as start and end respectively
            num_vehicles_t[i]--;
            
        }
    }
    
    std::cout << "initial solution: " << std::endl;
    for (size_t i = 0; i < initial_permutation.size(); ++i) {
        int index = initial_permutation[i];
        std::cout << index << " -> ";
    }
    std::cout << std::endl;
    return initial_permutation;
}

std::vector<int> RegretInsersion (const std::string type, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes)
{
    std::vector<int> initial_permutation;
    return initial_permutation;
}
} // namespace covid19
