//
// Created by MikeZhu on 2020/4/13.
//

#ifndef COVID19_VNS_H_
#define COVID19_VNS_H_

#include <iostream>
#include <vector>

namespace covid19
{

std::vector<int> Shaking(const std::vector<int> &nodes_permutation, const std::vector<std::vector<bool>> &potential_depots, const std::vector<std::vector<int>> &potentialDepotsList, const std::vector<int> &depot_indexes, std::vector<int> (*shakingMethod)(const std::vector<int> &, int, int));

std::vector<std::vector<bool>> PotentialDepots(const std::vector<std::vector<int64_t>> &distances, const std::vector<int> &depot_indexes, std::vector<std::vector<int>> &potentialDepotsList);

std::vector<std::vector<bool>> NeighbourReduction(const std::vector<std::vector<int64_t>> &distances);

std::vector<int> LocalSearch(int iStart, int iEnd, int kStart, int kEnd, const std::string type, const std::vector<int> &permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes, int64_t &current_cost, int &count, std::vector<int> (*localSearchMethod)(const std::vector<int> &, int, int), const std::vector<std::vector<bool>> &neighbourReduction, const bool intraOnly = false);

std::vector<int> LocalSearch(int iStart, int iEnd, int kStart, int kEnd, const std::string type, const std::vector<int> &permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes, int64_t &current_cost, const int64_t global_cost_min, int &count, std::vector<int> (*localSearchMethod)(const std::vector<int> &, int, int), const std::vector<std::vector<bool>> &neighbourReduction, const bool intraOnly = false);
/**
 * VNS(Variable Neighborhood Search) for multi depots with initial solution
 * @param type, type of the cost function, "distance" or "cumdistance"
 * @param nodes_permutation
 * @param distances
 * @param nodes_requirements
 * @param capacity
 * @param depot_indexes
 * @return
 */
std::vector<int> Vns (const std::string type, const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& depot_indexes, const int vehicles_num = 35);

/**
 * VNS(Variable Neighborhood Search) for multi depots
 * @param type, type of the cost function, "distance" or "cumdistance"
 * @param distances
 * @param nodes_requirements
 * @param capacity
 * @param num_vehicles
 * @param depot_indexes
 * @return
 */
std::vector<int> VnsGreedy (const std::string type, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes);

/**
 * VNS(Variable Neighborhood Search) for single depot
 * @param type, type of the cost function, "distance" or "cumdistance"
 * @param distances
 * @param nodes_requirements
 * @param capacity
 * @param num_vehicles
 * @param depot_index
 * @return
 */
std::vector<int> VnsSingle (const std::string type, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, int num_vehicles, int depot_index);

}

#endif //COVID19_VNS_H_
