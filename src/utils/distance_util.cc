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
        return (int64_t)pow(pow(x1 - x2, 2) + pow(y1 - y2, 2), 0.5);
    };
};

class DistanceManhattan : public DistanceAlgorithm
{
public:
    int64_t CalcDistance(int64_t x1, int64_t y1, int64_t x2, int64_t y2) override
    {
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

/*
function: CalcDistanceCost
! @brief: calculate the total distance for the permutation.
! @param [in]: nodes_permutation, the permutation of one trip, e.g. {0, 2, 5, 1, 3, 4, 0}, the first element is the start node index and the last is the end node index.
! @param [in]: distances, distances matrix between any two nodes.
! @param [in]: depot_indexes
! @param [out]: the total distance for this trip.
*/
int64_t CalcDistanceCost(const std::vector<int> &nodes_permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int> &depot_indexes)
{
    const int ARCH_SIZE = nodes_permutation.size() - 1;
    int64_t total_distance = 0;

    for (int i = 0; i < ARCH_SIZE; i++)
    {
        int start_node_index = nodes_permutation[i];
        int end_node_index = nodes_permutation[i + 1];
        if (IsIn(start_node_index, depot_indexes) && IsIn(end_node_index, depot_indexes))
        {
            continue;
        }
        total_distance += distances[start_node_index][end_node_index];
    }

    return total_distance;
}

int64_t CalcDistanceCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, int depot_index)
{
    std::vector<int> depot_indexes{depot_index};
    return CalcDistanceCost(nodes_permutation, distances, depot_indexes);
}

int64_t CalcDistanceCumCost(const std::vector<int> &nodes_permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int> &depot_indexes)
{
    const int ARCH_SIZE = nodes_permutation.size() - 1;
    int64_t total_distance = 0;
    int64_t temp_distance = 0;

    std::vector<int64_t> node_cum_distance{};
    for (int i = 0; i < ARCH_SIZE; i++)
    {
        int start_node_index = nodes_permutation[i];
        int end_node_index = nodes_permutation[i + 1];
        if (!IsIn(end_node_index, depot_indexes))
        {
            temp_distance += distances[start_node_index][end_node_index];
        }
        else
        {
            temp_distance = 0;
        }
        node_cum_distance.push_back(temp_distance);
    }

    for (size_t i = 0; i < node_cum_distance.size(); i++)
    {
        total_distance += node_cum_distance[i];
    }

    return total_distance;
}

int64_t CalcDistanceCumCost(const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, int depot_index)
{
    std::vector<int> depot_indexes{depot_index};
    return CalcDistanceCumCost(nodes_permutation, distances, depot_indexes);
}

void WriteResults(const std::vector<int> &nodes_permutation, const std::string file_name)
{
    std::ofstream outfile;
    outfile.open(file_name);
    if (!outfile.is_open())
    {
        std::cout << "file open failed" << std::endl;
    }

    for (size_t i = 0; i < nodes_permutation.size(); i++)
    {
        outfile << nodes_permutation[i] << " ";
    }
    outfile.close();
}

} // namespace covid19
