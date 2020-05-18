#include <cmath>
#include <algorithm>
#include "distance_util.h"

namespace covid19
{
class DistanceEuclidean : public DistanceAlgorithm
{
public:
    int64_t CalcDistance(int64_t x1, int64_t y1, int64_t x2, int64_t y2) override
    {
        // x1 *= 100;
        // y1 *= 100;
        // x2 *= 100;
        // y2 *= 100;
        return (int64_t)std::round(pow(pow(x1 - x2, 2) + pow(y1 - y2, 2), 0.5));
    };
};

class DistanceManhattan : public DistanceAlgorithm
{
public:
    int64_t CalcDistance(int64_t x1, int64_t y1, int64_t x2, int64_t y2) override
    {
        // x1 *= 100;
        // y1 *= 100;
        // x2 *= 100;
        // y2 *= 100;
        return (int64_t)(abs(x1 - x2) + abs(y1 - y2));
    };
};

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>> &locations, int loc_x_i, int loc_y_i)
{
    auto *dis_algorithm = new DistanceManhattan;
    return CalcDistances(locations, dis_algorithm, loc_x_i, loc_y_i);
}

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>> &locations, DistanceType disttype, int loc_x_i, int loc_y_i)
{
    if (disttype == DistanceType::euclidean)
    {
        auto *dis_algorithm = new DistanceEuclidean;
        return CalcDistances(locations, dis_algorithm, loc_x_i, loc_y_i);
    }
    else if (disttype == DistanceType::manhattan)
    {
        DistanceAlgorithm *dis_algorithm = new DistanceManhattan;
        return CalcDistances(locations, dis_algorithm, loc_x_i, loc_y_i);
    }
    else
    {
        std::cerr << "Distance type not supported, change to manhattan instead. You can use self defined DistanceAlgorithm instead.\n";
    }

    DistanceAlgorithm *dis_algorithm = new DistanceManhattan;
    return CalcDistances(locations, dis_algorithm, loc_x_i, loc_y_i);
}

std::vector<std::vector<int64_t>> CalcDistances(
    const std::vector<std::vector<int64_t>> &locations,
    DistanceAlgorithm *algorithm, int loc_x_i, int loc_y_i)
{
    int N = locations.size();
    std::vector<std::vector<int64_t>> distances(N, std::vector<int64_t>(N));
    for (size_t i = 0; i < N; i++)
    {
        distances[i][i] = 0;
        for (size_t j = i + 1; j < N; j++)
        {
            distances[i][j] = algorithm->CalcDistance(locations[i][loc_x_i], locations[i][loc_y_i], locations[j][loc_x_i], locations[j][loc_y_i]);
            distances[j][i] = distances[i][j];
        }
    }
    return distances;
}

} // namespace covid19
