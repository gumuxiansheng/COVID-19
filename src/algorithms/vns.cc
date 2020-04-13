//
// Created by MikeZhu on 2020/4/13.
//

#include "vns.h"

namespace covid19
{

std::vector<int> TwoSwap(const std::vector<int>& nodes_permutation, int swap_start_index, int swap_end_index)
{
    const int NODES_SIZE = nodes_permutation.size();
    std::vector<int> v(NODES_SIZE);
    for (int i = 0; i < NODES_SIZE; ++i)
    {
        if (i == swap_start_index)
        {
            v.push_back(nodes_permutation[swap_end_index]);
        } else if(i == swap_end_index)
        {
            v.push_back(nodes_permutation[swap_start_index]);
        } else {
            v.push_back(nodes_permutation[i]);
        }
    }

    return v;
}

std::vector<int> TwoOptSwap(const std::vector<int>& nodes_permutation, int swap_start_index, int swap_end_index)
{
    const int NODES_SIZE = nodes_permutation.size();
    std::vector<int> v(NODES_SIZE);
    for (int i = 0; i < swap_start_index; i++)
    {
        v.push_back(nodes_permutation[i]);
    }
    for (int i = swap_end_index; i >= swap_start_index; i--)
    {
        v.push_back(nodes_permutation[i]);
    }
    for (int i = swap_end_index + 1; i < NODES_SIZE; i++)
    {
        v.push_back(nodes_permutation[i]);
    }

    return v;

}

std::vector<int> TwoHOptSwap(const std::vector<int>& nodes_permutation, int swap_1, int swap_2)
{
    const int NODES_SIZE = nodes_permutation.size();
    std::vector<int> v(NODES_SIZE);
    if (swap_1 > swap_2){
        int temp = swap_1;
        swap_1 = swap_2;
        swap_2 = temp;
    }

    v.push_back(nodes_permutation[swap_1]);
    v.push_back(nodes_permutation[swap_2]);

    for (int i = 1; i < NODES_SIZE; ++i)
    {
        int idx = (swap_1 + i) % NODES_SIZE;
        if (idx != swap_2)
        {
            v.push_back(nodes_permutation[idx]);
        }
    }

    return v;

}

    void vns (SOLUTION & solution, CITIES * cities)
    {
        SOLUTION current_solution = solution;

        int count = 0;
        int max_no_improve = 10;

        do
        {
            count++;
            for (int i = 0; i < CITY_SIZE - 1; i++)
            {
                for (int k = i + 1; k < CITY_SIZE; k++)
                {
                    current_solution = solution;
                    swap_element(current_solution.permutation, i, k);

                    current_solution.cost = cost_total(current_solution.permutation, cities);
                    if (current_solution.cost < solution.cost)
                    {
                        solution = current_solution;
                        count = 0; //count复位
                    }

                }
            }

        } while (count <= max_no_improve);



    }
}