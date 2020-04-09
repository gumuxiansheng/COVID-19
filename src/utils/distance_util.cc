#include "distance_util.h"

namespace covid19
{
class DistanceEuclidean : public DistanceAlgorithm
{
public:
    int64_t CalcDistance(int64_t x1, int64_t y1, int64_t x2, int64_t y2)
    {
        return (int64_t)pow(pow(x1 - x2, 2) + pow(y1 - y2, 2), 0.5);
    };
};

class DistanceManhattan : public DistanceAlgorithm
{
public:
    int64_t CalcDistances(int64_t x1, int64_t y1, int64_t x2, int64_t y2)
    {
        return abs(x1 - x2) + abs(y1 - y2);
    };
};

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>> locations)
{
    return CalcDistances(locations, DistanceManhattan());
}

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>> locations, DistanceType disttype)
{
    if (disttype == DistanceType::euclidean)
    {
        return CalcDistances(locations, DistanceEuclidean());
    }
    else if (disttype == DistanceType::manhattan)
    {
        return CalcDistances(locations, DistanceManhattan());
    }
    else
    {
        std::cerr << "Distance type not supported, you can use self defined "
                     "DistanceAlgorithm instead.\n";
    }
}

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>> locations,
    DistanceAlgorithm algorithm)
{
    int N = locations.size;
    std::vector<std::vector<int64_t>> distances(N, std::vector<int64_t>(N));
    for (size_t i = 0; i < N; i++)
    {
        distances[i][i] = 0;
        for (size_t j = i + 1; j < N; j++)
        {
            distances[i][j] = algorithm.CalcDistance(locations[i][0], locations[i][1], locations[j][0], locations[j][1]);
            distances[j][i] = distances[i][j];
        }
    }
    return distances;
}
} // namespace covid19
