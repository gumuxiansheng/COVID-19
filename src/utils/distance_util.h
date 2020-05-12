#ifndef COVID19_DISTANCE_H_
#define COVID19_DISTANCE_H_

#include <iostream>
#include <fstream>
#include <vector>
#include "vector_help.h"

namespace covid19
{
class DistanceAlgorithm
{
public:
    virtual int64_t CalcDistance(int64_t x1, int64_t y1, int64_t x2, int64_t y2)=0;
};

enum class DistanceType
{
    euclidean,
    manhattan
};

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>>& locations, int loc_x_i = 0, int loc_y_i = 1);

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>>& locations, DistanceType disttype, int loc_x_i = 0, int loc_y_i = 1);

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>>& locations,
    DistanceAlgorithm *algorithm, int loc_x_i = 0, int loc_y_i = 1);

int64_t CalcDistanceCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, const std::vector<int> &depot_indexes);

int64_t CalcDistanceCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, int depot_index);

int64_t CalcDistanceCumCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, const std::vector<int> &depot_indexes);

int64_t CalcDistanceCumCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, int depot_index);

void WriteResults(const std::vector<int> &nodes_permutation, const std::string file_name);
} // namespace covid19

#endif // COVID19_DISTANCE_H_