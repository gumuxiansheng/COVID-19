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
#include "operators.h"

namespace covid19
{
    const int SHAKE_TIMES = 9; // how many shake times should a shaking method run
    // const int search_better_depth = 10;

    std::vector<int> Shaking(const std::vector<int> &nodes_permutation, std::vector<int> (*shakingMethod)(const std::vector<int> &, int, int))
    {
        std::random_device rd;
        std::default_random_engine e{rd()};
        std::uniform_int_distribution<unsigned> u(1, nodes_permutation.size() - 1);

        int index1 = u(e);
        int index2 = u(e);

        return shakingMethod(nodes_permutation, index1, index2);
    }

    std::vector<std::vector<bool>> NeighbourReduction(const std::vector<std::vector<int64_t>> &distances)
    {
        std::vector<std::vector<bool>> neighbourReduction{};
        for (size_t i = 0; i < distances.size(); i++)
        {
            std::vector<int64_t> sort_distance{distances[i]};
            std::sort(sort_distance.begin(), sort_distance.end());
            int64_t reduction_anchor = sort_distance[sort_distance.size() * 0.08 + 1];

            std::vector<bool> nrx{};
            for (auto &&item : distances[i])
            {
                nrx.push_back(item <= reduction_anchor);
            }
            
            neighbourReduction.push_back(nrx);
        }
        
        return neighbourReduction;
    }

    std::vector<int> LocalSearch(int iStart, int iEnd, int kStart, int kEnd, const std::string type, const std::vector<int> &permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes, int64_t &current_cost, int &count, std::vector<int> (*localSearchMethod)(const std::vector<int> &, int, int), const std::vector<std::vector<bool>> &neighbourReduction, const bool intraOnly)
    {
        count++;

        std::vector<int64_t> local_search_better_cost{};
        std::vector<std::vector<int>> local_search_better_permutation{};
        std::vector<int> current_permutation{permutation};
        std::vector<int> neighbour;

        std::random_device rd;
        std::default_random_engine e{rd()};
        const int tt = (nodes_requirements.size() - depot_indexes.size()) * depot_indexes.size();
        std::uniform_int_distribution<unsigned> u(3000/tt, 9000/tt);
        int search_better_depth = u(e);

        for (int i = iStart; i < iEnd; i++)
        {
            if (kStart == -1)
            {
                kStart = i + 1;
            }
            for (int k = kStart; k < kEnd; k++)
            {
                if (intraOnly)
                {
                    int l = i;
                    int r = k;
                    if (k < i)
                    {
                        l = k;
                        r = i;
                    }
                    for (size_t j = l; j < r; j++)
                    {
                        if (IsIn(current_permutation[j], depot_indexes))
                        {
                            if (i < iEnd - 1)
                            {
                                i++;
                                k = kStart;
                            }
                        }
                    }
                }
                if (!neighbourReduction[current_permutation[k]][current_permutation[i]] && !neighbourReduction[current_permutation[k]][current_permutation[i + 1]])
                {
                    continue;
                }

                neighbour = localSearchMethod(current_permutation, i, k);

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
                        i = i > iStart ? i - 1 : iStart;
                        k = kStart;
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

    std::vector<int> LocalSearchDepot(const std::string type, const std::vector<int> &permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes, int64_t &current_cost, int &count)
    {
        // count++;

        std::vector<int64_t> local_search_better_cost{};
        std::vector<std::vector<int>> local_search_better_permutation{};
        std::vector<int> current_permutation{permutation};
        std::vector<int> neighbour;

        int search_better_depth = 1;

        int numRoutes = (permutation.size() - distances.size() + depot_indexes.size()) / 2;
        int depotSize = depot_indexes.size();

        for (int i = 0; i < numRoutes; i++)
        {
            for (int k = 0; k < depotSize; k++)
            {
                neighbour = ExchangeDepot(permutation, depot_indexes, i, depot_indexes[k]);

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
                        i = i > 0 ? i - 1 : 0;
                        k = 0;
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
        std::uniform_int_distribution<unsigned> u(8, 20);
        int max_no_improve = u(e);
        int shake_max_no_improve = u(e);
        int shake_times = u(e);
        std::uniform_int_distribution<unsigned> shakeMethodU(0, 2);

        std::vector<std::vector<bool>> neighbourReduction = NeighbourReduction(distances);

        std::cout << "STAGE 1: Local Search" << std::endl;
        do
        {

            current_permutation = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, TwoSwap, neighbourReduction);

            current_permutation = LocalSearch(1, TRAVEL_SIZE - 2, 1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, RelocationMove, neighbourReduction);

            current_permutation = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, TwoOptSwap, neighbourReduction);
            
            current_permutation = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, ArcNodeSwap, neighbourReduction);

            current_permutation = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, ArcNodeMove, neighbourReduction);

            // current_permutation = FitDepot(current_permutation, distances, depot_indexes);
            current_permutation = LocalSearchDepot(type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count);

        } while (count <= max_no_improve);

        do
        {
            shakeCount++;
            shake_times++;
            std::cout << "STAGE 2: Shake" << std::endl;
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
            } while (shakeTimes++ < shake_times);
            shake_times = u(e);

            shaking_cost = CalcCost(type, shaking, distances, depot_indexes);
            if (shaking_cost - current_cost > 0.5 * current_cost)
            {
                count++;
                continue;
            }

            std::cout << "STAGE 2: Local Search" << std::endl;
            do
            {
                shaking = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, ArcNodeMove, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, TwoSwap, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, RelocationMove, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, ArcNodeSwap, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, TwoOptSwap, neighbourReduction);

                // shaking = FitDepot(shaking, distances, depot_indexes);
                shaking = LocalSearchDepot(type, shaking, distances, nodes_requirements, capacity, depot_indexes, current_cost, count);


            } while (count <= max_no_improve);

            shaking_cost = CalcCost(type, shaking, distances, depot_indexes);
            if (shaking_cost < current_cost)
            {
                current_cost = shaking_cost;
                current_permutation = shaking;
            }

            std::cout << "STAGE 3: Shake Again" << std::endl;
            shakeMethod = shakeMethodU(e);
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
            } while (shakeTimes++ < shake_times);
            shake_times = u(e);

            shaking_cost = CalcCost(type, shaking, distances, depot_indexes);
            if (shaking_cost - current_cost > 0.5 * current_cost)
            {
                count++;
                continue;
            }

            std::cout << "STAGE 3: Local Search Again" << std::endl;
            do
            {

                shaking = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, ArcNodeMove, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, TwoOptSwap, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, RelocationMove, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, TwoSwap, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, ArcNodeSwap, neighbourReduction);

                shaking = FitDepot(shaking, distances, depot_indexes);

            } while (count <= max_no_improve);

            if (shaking_cost < current_cost)
            {
                current_cost = shaking_cost;
                current_permutation = shaking;
            }

            current_permutation = FitDepot(current_permutation, distances, depot_indexes);

            std::cout << "STAGE 4: Shake Depot" << std::endl;
            shaking = ChangeDepot(current_permutation, distances, depot_indexes, CalcDistanceCumCost);
            do
            {

                shaking = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, ArcNodeMove, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, TwoOptSwap, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, RelocationMove, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, TwoSwap, neighbourReduction);

                shaking = LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, ArcNodeSwap, neighbourReduction);

                shaking = FitDepot(shaking, distances, depot_indexes);

            } while (count <= max_no_improve);

            if (shaking_cost < current_cost)
            {
                current_cost = shaking_cost;
                current_permutation = shaking;
            }
        
        } while (shakeCount <= shake_max_no_improve);

        current_permutation = LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, RelocationMove, neighbourReduction, true);

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