#ifndef COVID19_DISTANCE_H_
#define COVID19_DISTANCE_H_

#include <math.h>
#include <iostream>
#include <vector>

namespace covid19
{
class DistanceAlgorithm
{
public:
    int64_t virtual CalcDistance(int64_t x1, int64_t y1, int64_t x2, int64_t y2)=0;
};

enum class DistanceType
{
    euclidean,
    manhattan
};

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>> locations);

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>> locations, DistanceType disttype);

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>> locations,
    DistanceAlgorithm *algorithm);
} // namespace covid19

#endif // COVID19_DISTANCE_H_