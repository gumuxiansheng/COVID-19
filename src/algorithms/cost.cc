//
// Created by MikeZhu on 2020/5/13.
//

#include "cost.h"

namespace covid19
{
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

int64_t CalcCost(const std::string type, const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, const std::vector<int>& depot_indexes)
{
    if (type == "distance")
    {
        return covid19::CalcDistanceCost(nodes_permutation, distances, depot_indexes);
    } else if (type == "cumdistance")
    {
        return covid19::CalcDistanceCumCost(nodes_permutation, distances, depot_indexes);
    }

    return covid19::CalcDistanceCost(nodes_permutation, distances, depot_indexes);
}

int64_t CalcCost(const std::string type, const std::vector<int>& nodes_permutation, const std::vector<std::vector<int64_t>>& distances, int depot_index)
{
    std::vector<int> depot_indexes{depot_index};
    return covid19::CalcCost(type, nodes_permutation, distances, depot_indexes);
}
} // namespace covid19
