#include "requirements_util.h"
#include "vector_help.h"

namespace covid19
{

bool CheckMultiDepotRequirements(const std::vector<int>& nodes_permutation, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& depot_indexes)
{
    int current_depot = nodes_permutation[0];
    int64_t current_requirements = 0;

    for (int i = 1; i < nodes_permutation.size() - 1; i++)
    {
        // we only need to start from the one after start node because no requirement at the start node.

        if (!IsIn(nodes_permutation[i], depot_indexes))
        {
            current_requirements += nodes_requirements[nodes_permutation[i]];
            if (current_requirements > capacity)
            { // capacity check
                return false;
            }
        } else if (IsIn(nodes_permutation[i + 1], depot_indexes)){
            if (current_depot != nodes_permutation[i])
            {
                return false;
            }
            current_requirements = 0;

        } else if (IsIn(nodes_permutation[i - 1], depot_indexes)){
            current_depot = nodes_permutation[i];
            current_requirements = 0;

        } else
        {
            return false;
        }

    }

    return true;
}

bool CheckRequirements(const std::vector<int>& nodes_permutation, const std::vector<int64_t>& nodes_requirements, int64_t capacity)
{
    const int START_NODE_INDEX = nodes_permutation[0];
    int64_t current_requirements = 0;

    for (int i = 1; i < nodes_permutation.size(); i++)
    {
        // we only need to start from the one after start node because no requirement at the start node.

        if (nodes_permutation[i] != START_NODE_INDEX)
        {
            current_requirements += nodes_requirements[nodes_permutation[i]];
            if (current_requirements > capacity)
            {
                return false;
            }
        } else if (!(nodes_permutation[i - 1] == START_NODE_INDEX) || (nodes_permutation[i + 1] == START_NODE_INDEX)){
            return false;

        } else 
        {
            current_requirements = 0;
        }

    }

    return true;
}

int64_t CalcRequirements(const std::vector<int>& nodes_permutation, const std::vector<int64_t>& nodes_requirements)
{
    int64_t current_requirements = 0;

    for (int i : nodes_permutation)
    {
        current_requirements += nodes_requirements[i];

    }

    return current_requirements;
}
}