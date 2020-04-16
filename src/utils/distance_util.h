#ifndef COVID19_DISTANCE_H_
#define COVID19_DISTANCE_H_

#include <iostream>
#include <vector>

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
    const std::vector<std::vector<int64_t>>& locations);

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>>& locations, DistanceType disttype);

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>>& locations,
    DistanceAlgorithm *algorithm);

int64_t CalcDistanceCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances);

int64_t CalcDistanceCumCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, int depot_index);
} // namespace covid19

#endif // COVID19_DISTANCE_H_