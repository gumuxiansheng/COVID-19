//
// Created by MikeZhu on 2020/4/13.
//

#include "vns.h"
#include "../utils/distance_util.h"
#include "../utils/requirements_util.h"
#include "../utils/vector_help.h"
#include "init_solution.h"
#include "cost.h"
#include <algorithm>
#include <random>

namespace covid19
{

    std::vector<int> TwoSwap(const std::vector<int> &nodes_permutation, int swap_start_index, int swap_end_index)
    {
        const int TRAVEL_SIZE = nodes_permutation.size();
        std::vector<int> v;
        for (int i = 0; i < TRAVEL_SIZE; ++i)
        {
            if (i == swap_start_index)
            {
                v.push_back(nodes_permutation[swap_end_index]);
            }
            else if (i == swap_end_index)
            {
                v.push_back(nodes_permutation[swap_start_index]);
            }
            else
            {
                v.push_back(nodes_permutation[i]);
            }
        }

        return v;
    }

    std::vector<int> TwoOptSwap(const std::vector<int> &nodes_permutation, int swap_start_index, int swap_end_index)
    {
        const int TRAVEL_SIZE = nodes_permutation.size();
        std::vector<int> v;
        for (int i = 0; i < swap_start_index; i++)
        {
            v.push_back(nodes_permutation[i]);
        }
        for (int i = swap_end_index; i >= swap_start_index; i--)
        {
            v.push_back(nodes_permutation[i]);
        }
        for (int i = swap_end_index + 1; i < TRAVEL_SIZE; i++)
        {
            v.push_back(nodes_permutation[i]);
        }

        return v;
    }

    std::vector<int> TwoHOptSwap(const std::vector<int> &nodes_permutation, int swap_1, int swap_2)
    {
        const int TRAVEL_SIZE = nodes_permutation.size();
        std::vector<int> v;
        if (swap_1 > swap_2)
        {
            int temp = swap_1;
            swap_1 = swap_2;
            swap_2 = temp;
        }

        v.push_back(nodes_permutation[swap_1]);
        v.push_back(nodes_permutation[swap_2]);

        for (int i = 1; i < TRAVEL_SIZE; ++i)
        {
            int idx = (swap_1 + i) % TRAVEL_SIZE;
            if (idx != swap_2)
            {
                v.push_back(nodes_permutation[idx]);
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

        std::vector<int> v{};

        for (size_t i = 0; i < nodes_permutation.size(); i++)
        {
            if (i == swap_1)
            {
                v.push_back(nodes_permutation[swap_2]);
            } else if (i == swap_1 + 1)
            {
                continue;
            } else if (i == swap_2)
            {
                v.push_back(nodes_permutation[swap_1]);
                v.push_back(nodes_permutation[swap_1 + 1]);
            } else
            {
                v.push_back(nodes_permutation[i]);
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

        std::vector<int> v{};

        for (size_t i = 0; i < nodes_permutation.size(); i++)
        {
            if (i == index_1 || i == index_1 + 1)
            {
                continue;
            } else if (i == index_2)
            {
                v.push_back(nodes_permutation[i]);
                v.push_back(nodes_permutation[index_1]);
                v.push_back(nodes_permutation[index_1 + 1]);
            } else
            {
                v.push_back(nodes_permutation[i]);
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
        std::vector<int> v;

        for (size_t i = 0; i < TRAVEL_SIZE; i++)
        {
            if (i == item_index)
            {
                continue;
            }
            else if (i == insert_index)
            {
                v.push_back(nodes_permutation[item_index]);
            }
            v.push_back(nodes_permutation[i]);
        }

        return v;
    }

    std::vector<int> Shaking(const std::vector<int> &nodes_permutation, std::vector<int> (*shakingMethod)(const std::vector<int> &, int, int))
    {
        std::random_device rd;
        std::default_random_engine e{rd()};
        std::uniform_int_distribution<unsigned> u(1, nodes_permutation.size() - 1);

        int index1 = u(e);
        int index2 = u(e);

        return shakingMethod(nodes_permutation, index1, index2);
    }

    std::vector<int> LocalSearch(int iStart, int iEnd, int kStart, int kEnd, const std::string type, const std::vector<int> &permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes, int64_t &current_cost, int &count, std::vector<int> (*localSearchMethod)(const std::vector<int> &, int, int))
    {
        int search_better_depth = 3;
        std::vector<int64_t> local_search_better_cost{};
        std::vector<std::vector<int>> local_search_better_permutation{};
        std::vector<int> current_permutation{permutation};

        for (int i = iStart; i < iEnd; i++)
        {
            if (kStart == -1)
            {
                kStart = i + 1;
            }
            for (int k = kStart; k < kEnd; k++)
            {
                std::vector<int> neighbour = localSearchMethod(current_permutation, i, k);

                if (!covid19::CheckMultiDepotRequirements(neighbour, nodes_requirements, capacity, depot_indexes))
                {
                    continue;
                }
                int64_t neighbour_cost = CalcCost(type, neighbour, distances, depot_indexes);

                if (current_cost > neighbour_cost)
                {
                    current_cost = neighbour_cost;

                    local_search_better_cost.push_back(neighbour_cost);
                    local_search_better_permutation.push_back(neighbour);

                    if (local_search_better_cost.size() == search_better_depth)
                    { // choose the search_better_depth's better cost
                        current_permutation = neighbour;
                        local_search_better_cost.clear();
                        local_search_better_permutation.clear();
                    }
                    std::cout << "current_cost: " << current_cost << std::endl;
                    count = 0;
                }
            }
        }

        if (!local_search_better_cost.empty())
        { // local search may not found enough better solutions as much as search_better_depth, but we should also choose the best solution here.
            for (size_t i = 0; i < local_search_better_cost.size(); i++)
            {
                if (current_cost == local_search_better_cost[i])
                {
                    current_permutation = local_search_better_permutation[i];
                    local_search_better_cost.clear();
                    local_search_better_permutation.clear();
                }
            }
        }

        return current_permutation;
    }

    std::vector<int> Vns(const std::string type, const std::vector<int> &nodes_permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes)
    {
        const int TRAVEL_SIZE = nodes_permutation.size();
        std::vector<int> current_permutation = nodes_permutation;
        int64_t current_cost = CalcCost(type, nodes_permutation, distances, depot_indexes);

        int count = 0;
        int shakeCount = 0;
        std::random_device rd;
        std::default_random_engine e{rd()};
        std::uniform_int_distribution<unsigned> u(2, 11);
        int max_no_improve = u(e);
        int shake_max_no_improve = 10;
        std::uniform_int_distribution<unsigned> shakeMethodU(0, 2);

        std::cout << "STAGE 1: " << std::endl;
        do
        {
            count++;

            current_permutation = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, TwoSwap);

            current_permutation = LocalSearch(1, TRAVEL_SIZE - 2, 1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, RelocationMove);

            current_permutation = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, TwoOptSwap);
            
            current_permutation = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, ArcNodeSwap);

            current_permutation = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, ArcNodeMove);


        } while (count <= max_no_improve);

        do
        {
            shakeCount++;
            std::cout << "STAGE 2: " << std::endl;
            int shakeTimes = 0;
            std::vector<int> shaking{};
            int64_t shaking_cost;
            std::vector<int> (*shakingMethod)(const std::vector<int> &, int, int);
            unsigned shakeMethod = shakeMethodU(e);
            std::cout << "shakeMethod:" << shakeMethod << std::endl;
            switch (shakeMethod)
            {
            case 0:
                shakingMethod = TwoOptSwap;
                break;
            case 1:
                shakingMethod = RelocationMove;
                break;
            case 2:
            default:
                shakingMethod = TwoSwap;
                break;
            }

            do
            {
                while (true)
                {
                    shaking = Shaking(current_permutation, shakingMethod);
                    if (covid19::CheckMultiDepotRequirements(shaking, nodes_requirements, capacity, depot_indexes))
                    {
                        break;
                    }
                }
            } while (shakeTimes++ < 5);

            shaking_cost = CalcCost(type, shaking, distances, depot_indexes);

            do
            {
                count++;
                shaking = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, ArcNodeMove);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, TwoSwap);

                shaking = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, ArcNodeSwap);

            } while (count <= max_no_improve);

            shaking_cost = CalcCost(type, shaking, distances, depot_indexes);
            if (shaking_cost < current_cost)
            {
                current_cost = shaking_cost;
                current_permutation = shaking;
            }

            std::cout << "STAGE 3: " << std::endl;
            shakeMethod = shakeMethodU(e);
            switch (shakeMethod)
            {
            case 0:
                shakingMethod = RelocationMove;
                break;
            case 1:
                shakingMethod = TwoOptSwap;
                break;
            case 2:
            default:
                shakingMethod = TwoSwap;
                break;
            }

            do
            {
                while (true)
                {
                    shaking = Shaking(current_permutation, shakingMethod);
                    if (covid19::CheckMultiDepotRequirements(shaking, nodes_requirements, capacity, depot_indexes))
                    {
                        break;
                    }
                }
            } while (shakeTimes++ < 5);

            shaking_cost = CalcCost(type, shaking, distances, depot_indexes);
            do
            {
                count++;

                shaking = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, ArcNodeMove);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, RelocationMove);

                shaking = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, ArcNodeSwap);

            } while (count <= max_no_improve);

            if (shaking_cost < current_cost)
            {
                current_cost = shaking_cost;
                current_permutation = shaking;
            }

            std::cout << "STAGE 4: " << std::endl;

            std::vector<int> inline_permutation{nodes_permutation};
            inline_permutation.erase(inline_permutation.begin());
            inline_permutation.pop_back();
            do
            {
                count++;
                for (int i = 1; i < TRAVEL_SIZE - 3; i++)
                {
                    for (int k = 1; k < TRAVEL_SIZE - 2; k++)
                    {
                        if (k == i)
                        {
                            continue;
                        }
                        inline_permutation = TwoHOptSwap(inline_permutation, i, k);
                        std::vector<int> neighbour;
                        neighbour.push_back(nodes_permutation[0]);
                        for (int j : inline_permutation)
                        {
                            neighbour.push_back(j);
                        }
                        neighbour.push_back(nodes_permutation[nodes_permutation.size() - 1]);

                        if (!covid19::CheckMultiDepotRequirements(neighbour, nodes_requirements, capacity, depot_indexes))
                        {
                            continue;
                        }
                        int64_t neighbour_cost = CalcCost(type, neighbour, distances, depot_indexes);

                        if (current_cost > neighbour_cost)
                        {
                            current_permutation = neighbour;
                            current_cost = neighbour_cost;
                            std::cout << "current_cost: " << current_cost << std::endl;
                            count = 0;
                            shakeCount = 0;
                        }
                    }
                }

            } while (count <= max_no_improve);
        } while (shakeCount <= shake_max_no_improve);

        return current_permutation;
    }

    std::vector<int> Vns(const std::string type, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &num_vehicles, const std::vector<int> &depot_indexes)
    {
        std::vector<int> initial_permutation = Greedy(nodes_requirements, capacity, num_vehicles, depot_indexes);
        return Vns(type, initial_permutation, distances, nodes_requirements, capacity, depot_indexes);
    }

    std::vector<int> Vns(const std::string type, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, int num_vehicles, int depot_index)
    {
        std::vector<int> num_vehicles_list{num_vehicles};
        std::vector<int> depot_indexes{depot_index};
        return Vns(type, distances, nodes_requirements, capacity, num_vehicles_list, depot_indexes);
    }
} // namespace covid19