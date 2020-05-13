//
// Created by MikeZhu on 2020/5/13.
//

#ifndef COVID19_COST_H_
#define COVID19_COST_H_

#include <iostream>
#include <vector>
#include "../utils/vector_help.h"
#include "../utils/distance_util.h"

namespace covid19
{
int64_t CalcDistanceCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, const std::vector<int> &depot_indexes);

int64_t CalcDistanceCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, int depot_index);

int64_t CalcDistanceCumCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, const std::vector<int> &depot_indexes);

int64_t CalcDistanceCumCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, int depot_index);

int64_t CalcCost(const std::string type, const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, const std::vector<int>& depot_indexes);

int64_t CalcCost(const std::string type, const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, int depot_index);
} // namespace covid19


#endif //COVID19_COST_H_