//
// Created by MikeZhu on 2020/5/12.
//

#ifndef COVID19_INIT_SOLUTION_H_
#define COVID19_INIT_SOLUTION_H_

#include <iostream>
#include <vector>
#include <set>
#include "cost.h"
#include "../utils/distance_util.h"
#include "../utils/vector_help.h"
#include "../utils/requirements_util.h"

namespace covid19
{
int DecentCmp(int a,int b);
/**
 * Greedy algorithm to find a feasible solution
 * @param nodes_requirements
 * @param capacity
 * @param num_vehicles
 * @param depot_indexes
 * @return
 */
std::vector<int> Greedy (const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes);

/**
 * regret-insertion-based heuristic(G. M. Ribeiro and G. Laporte, “An adaptive large neighborhood search heuristic for the cumulative capacitated vehicle routing problem,” Comput. Oper. Res., vol. 39, no. 3, pp. 728–735, 2012.)
 */
std::vector<int> RegretInsersion (const std::string type, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes);

void RegretInsersionRecur (const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes, std::vector<std::vector<int>> &current_routes, std::vector<int> &remain_customers);
} // namespace covid19


#endif // COVID19_INIT_SOLUTION_H_