//
// Created by MikeZhu on 2020/4/13.
//

#ifndef COVID19_VNS_H_
#define COVID19_VNS_H_

#include <iostream>
#include <vector>

namespace covid19
{

/**
 * swap two nodes
 * @param nodes_permutation
 * @param swap_start_index
 * @param swap_end_index
 * @return
 */
std::vector<int> TwoSwap(const std::vector<int>& nodes_permutation, int swap_start_index, int swap_end_index);

/**
 * two_opt_swap
 * revert the items between start and end(both include).
 *             start  end
 *               +     +
 *               v     v
 *          +----+-----+----+
 *          |1|2|3|4|5|6|7|8|
 *          +---------------+
 *                  |
 *                  v
 *          +---------------+
 *          |1|2|6|5|4|3|7|8|
 *          +---------------+
 *
 * @param nodes_permutation
 * @param swap_start_index
 * @param swap_end_index
 * @return
 */
std::vector<int> TwoOptSwap(const std::vector<int>& nodes_permutation, int swap_start_index, int swap_end_index);

/**
 * two_h_opt_swap
 * move two nodes to the head and order the remain nodes.
 *            swap_1 swap_2
 *               +     +
 *               v     v
 *          +----+-----+----+
 *          |1|2|3|4|5|6|7|8|
 *          +---------------+
 *                  |
 *                  v
 *          +---------------+
 *          |3|6|4|5|7|8|1|2|
 *          +---------------+
 *
 * @param nodes_permutation
 * @param swap_start_index
 * @param swap_end_index
 * @return
 */
std::vector<int> TwoHOptSwap(const std::vector<int>& nodes_permutation, int swap_1, int swap_2);

std::vector<int> ArcNodeSwap(const std::vector<int> &nodes_permutation, int swap_1, int swap_2);

std::vector<int> ArcNodeMove(const std::vector<int> &nodes_permutation, int index_1, int index_2);

std::vector<int> RelocationMove(const std::vector<int>& nodes_permutation, int item_index, int insert_index);

std::vector<int> Shaking(const std::vector<int> &nodes_permutation, std::vector<int> (*shakingMethod)(const std::vector<int> &, int, int));

std::vector<std::vector<bool>> NeighbourReduction(const std::vector<std::vector<int64_t>> &distances);

std::vector<int> LocalSearch(int iStart, int iEnd, int kStart, int kEnd, const std::string type, const std::vector<int> &permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes, int64_t &current_cost, int &count, std::vector<int> (*localSearchMethod)(const std::vector<int> &, int, int), const std::vector<std::vector<bool>> &neighbourReduction);

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
std::vector<int> Vns (const std::string type, const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& depot_indexes);

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
std::vector<int> Vns (const std::string type, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes);

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
std::vector<int> Vns (const std::string type, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, int num_vehicles, int depot_index);

}

#endif //COVID19_VNS_H_
