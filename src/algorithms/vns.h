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

/**
 * VNS(Variable Neighborhood Search)
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

#endif //HELLO_VNS_H_
