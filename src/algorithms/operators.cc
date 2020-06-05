#include "operators.h"

#include <algorithm>
#include <random>

namespace covid19
{
    std::vector<int> TwoSwap(const std::vector<int> &nodes_permutation, int swap_start_index, int swap_end_index)
    {
        std::vector<int> v{nodes_permutation};
        v[swap_start_index] = nodes_permutation[swap_end_index];
        v[swap_end_index] = nodes_permutation[swap_start_index];

        return v;
    }

    std::vector<int> TwoOptSwap(const std::vector<int> &nodes_permutation, int swap_start_index, int swap_end_index)
    {
        std::vector<int> v{nodes_permutation};
        for (int i = swap_start_index; i <= swap_end_index; i++)
        {
            v[i] = nodes_permutation[swap_start_index + swap_end_index - i];
        }

        return v;
    }

    std::vector<int> TwoHOptSwap(const std::vector<int> &nodes_permutation, int swap_1, int swap_2)
    {
        const int TRAVEL_SIZE = nodes_permutation.size();
        std::vector<int> v(TRAVEL_SIZE);
        if (swap_1 > swap_2)
        {
            int temp = swap_1;
            swap_1 = swap_2;
            swap_2 = temp;
        }

        int v_index = 0;

        v[v_index++] = nodes_permutation[swap_1];
        v[v_index++] = nodes_permutation[swap_2];

        for (int i = 1; i < TRAVEL_SIZE; ++i)
        {
            int idx = (swap_1 + i) % TRAVEL_SIZE;
            if (idx != swap_2)
            {
                v[v_index++] = nodes_permutation[idx];
            }
        }

        return v;
    }

    std::vector<int> ArcNodeSwap(const std::vector<int> &nodes_permutation, int swap_1, int swap_2)
    {
        if (swap_2 - swap_1 <= 1 && swap_2 - swap_1 >= -1)
        {
            return nodes_permutation;
        }

        const int TRAVEL_SIZE = nodes_permutation.size();
        std::vector<int> v(TRAVEL_SIZE);

        int v_index = 0;

        for (size_t i = 0; i < TRAVEL_SIZE; i++)
        {
            if (i == swap_1)
            {
                v[v_index++] = nodes_permutation[swap_2];
            } else if (i == swap_1 + 1)
            {
                continue;
            } else if (i == swap_2)
            {
                v[v_index++] = nodes_permutation[swap_1];
                v[v_index++] = nodes_permutation[swap_1 + 1];
            } else
            {
                v[v_index++] = nodes_permutation[i];
            }
        }
        
        return v;
    }

    std::vector<int> ArcNodeMove(const std::vector<int> &nodes_permutation, int index_1, int index_2)
    {
        if (index_2 - index_1 <= 1 && index_2 - index_1 >= -1)
        {
            return nodes_permutation;
        }

        const int TRAVEL_SIZE = nodes_permutation.size();
        std::vector<int> v(TRAVEL_SIZE);

        int v_index = 0;

        for (size_t i = 0; i < TRAVEL_SIZE; i++)
        {
            if (i == index_1 || i == index_1 + 1)
            {
                continue;
            } else if (i == index_2)
            {
                v[v_index++] = nodes_permutation[i];
                v[v_index++] = nodes_permutation[index_1];
                v[v_index++] = nodes_permutation[index_1 + 1];
            } else
            {
                v[v_index++] = nodes_permutation[i];
            }
        }
        
        return v;
    }

    std::vector<int> RelocationMove(const std::vector<int> &nodes_permutation, int item_index, int insert_index)
    {
        if (item_index == insert_index)
        {
            // std::vector<int> v{nodes_permutation};
            return nodes_permutation;
        }
        const int TRAVEL_SIZE = nodes_permutation.size();
        std::vector<int> v(TRAVEL_SIZE);

        int v_index = 0;

        for (size_t i = 0; i < TRAVEL_SIZE; i++)
        {
            if (i == item_index)
            {
                continue;
            }
            else if (i == insert_index)
            {
                v[v_index++] = nodes_permutation[item_index];
            }
            v[v_index++] = nodes_permutation[i];
        }

        return v;
    }
} // covid19