#include "requirements_util.h"

namespace covid19
{

bool CheckRequirements(const std::vector<int64_t>& nodes_permutation, const std::vector<int64_t>& nodes_requirements, int64_t capacity)
{
    const int START_NODE_INDEX = nodes_permutation[0];
    int64_t current_requirements = 0;

    for (int i = 1; i < nodes_permutation.size(); i++)
    {
        // we only need to start from the one after start node because no requirement at the start node.

        if (nodes_permutation[i] != START_NODE_INDEX)
        {
            current_requirements += nodes_requirements[i];
            if (current_requirements > capacity)
            {
                return false;
            }
        } else {
            current_requirements = 0;
        }

    }

    return true;
}
}