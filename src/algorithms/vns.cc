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
    // const int SHAKE_TIMES = 9; // how many shake times should a shaking method run
    // const int search_better_depth = 10;
    const int WHILE_NUM = 8;

    bool CheckShaking(const std::vector<int> &nodes_permutation, const std::vector<std::vector<bool>> &potential_depots, const std::vector<int> &depot_indexes, int index1, int index2)
    {
        int node1 = nodes_permutation[index1];
        int node2 = nodes_permutation[index2];

        if (IsIn(node1, depot_indexes) || IsIn(node2, depot_indexes))
        {
            return false;
        }

        int depot_1 = 0;
        int depot_2 = 0;
        for (size_t i = index1; i < nodes_permutation.size(); i++)
        {
            if (IsIn(nodes_permutation[i], depot_indexes))
            {
                depot_1 = nodes_permutation[i];
                break;
            }
        }
        for (size_t i = index2; i < nodes_permutation.size(); i++)
        {
            if (IsIn(nodes_permutation[i], depot_indexes))
            {
                depot_2 = nodes_permutation[i];
                break;
            }
        }

        if (!potential_depots[node1][depot_2] || !potential_depots[node2][depot_1])
        {
            return false;
        }

        return true;
    }

    bool CheckShakingGuided(const std::vector<int> &nodes_permutation, const std::vector<std::vector<bool>> &potential_depots, const std::vector<int> &depot_indexes, int index1, int index2, const std::vector<std::vector<int>> &potentialDepotsList)
    {
        int node1 = nodes_permutation[index1];
        int node2 = nodes_permutation[index2];

        if (IsIn(node1, depot_indexes) || IsIn(node2, depot_indexes))
        {
            return false;
        }

        int depot_1 = 0;
        int depot_2 = 0;
        for (size_t i = index1; i < nodes_permutation.size(); i++)
        {
            if (IsIn(nodes_permutation[i], depot_indexes))
            {
                depot_1 = nodes_permutation[i];
                break;
            }
        }
        for (size_t i = index2; i < nodes_permutation.size(); i++)
        {
            if (IsIn(nodes_permutation[i], depot_indexes))
            {
                depot_2 = nodes_permutation[i];
                break;
            }
        }

        if (!potential_depots[node1][depot_2] || !potential_depots[node2][depot_1])
        {
            return false;
        }

        if (potentialDepotsList[node1].size() == 1)
        {
            return depot_1 == depot_2;
        }

        return true;
    }

    std::vector<int> Shaking(const std::vector<int> &nodes_permutation, const std::vector<std::vector<bool>> &potential_depots, const std::vector<std::vector<int>> &potentialDepotsList, const std::vector<int> &depot_indexes, std::vector<int> (*shakingMethod)(const std::vector<int> &, int, int))
    {
        std::random_device rd;
        std::default_random_engine e{rd()};
        std::uniform_int_distribution<unsigned> u(1, nodes_permutation.size() - 1);

        int index1 = u(e);
        int index2 = u(e);

        int max_while = 10;
        int count = 0;
        while (!CheckShakingGuided(nodes_permutation, potential_depots, depot_indexes, index1, index2, potentialDepotsList) && (count++ < max_while))
        {
            index1 = u(e);
            index2 = u(e);
        }

        return shakingMethod(nodes_permutation, index1, index2);
    }

    std::vector<std::vector<bool>> NeighbourReduction(const std::vector<std::vector<int64_t>> &distances, const std::vector<std::vector<bool>> &potentialDepots)
    {
        std::vector<std::vector<bool>> neighbourReduction{};
        for (size_t i = 0; i < distances.size(); i++)
        {
            std::vector<int64_t> sort_distance{distances[i]};
            std::sort(sort_distance.begin(), sort_distance.end());
            int64_t reduction_anchor = sort_distance[sort_distance.size() * 0.02 + 1];

            std::vector<bool> nrx{};
            for (size_t j = 0; j < distances[i].size(); j++)
            {
                auto item = distances[i][j];
                nrx.push_back(item <= reduction_anchor || potentialDepots[i][j]);
            }

            neighbourReduction.push_back(nrx);
        }

        return neighbourReduction;
    }

    std::vector<std::vector<bool>> PotentialDepots(const std::vector<std::vector<int64_t>> &distances, const std::vector<int> &depot_indexes, std::vector<std::vector<int>> &potentialDepotsList)
    {
        std::vector<std::vector<bool>> potentialDepots{};
        std::vector<std::vector<int64_t>> distancesToDepots{};
        for (size_t i = 0; i < distances.size(); i++)
        {
            std::vector<int> potentialDepotsListx{};
            std::vector<int64_t> distancesToDepotsx{};
            for (auto &&depotIndex : depot_indexes)
            {
                distancesToDepotsx.push_back(distances[i][depotIndex]);
            }
            distancesToDepots.push_back(distancesToDepotsx);

            std::vector<int64_t> sort_distance{distancesToDepotsx};
            std::sort(sort_distance.begin(), sort_distance.end());

            std::vector<int> distanceDepotSort{};
            std::vector<int> depot_indexes_temp{depot_indexes};
            for (auto &&dis : sort_distance)
            {
                for (auto iter = depot_indexes_temp.begin(); iter < depot_indexes_temp.end(); iter++)
                {
                    int depot_i = *iter;
                    if (distances[i][depot_i] == dis) {
                        distanceDepotSort.push_back(depot_i);
                        depot_indexes_temp.erase(iter);
                        break;
                    }
                }
                
            }

            std::vector<bool> potentialDepotsx(distances.size());
            potentialDepotsx[distanceDepotSort[0]] = true;
            int potentialCount = 0;
            potentialDepotsListx.push_back(distanceDepotSort[0]);
            for (size_t j = 1; j < distanceDepotSort.size(); j++)
            {
                int depot_j_0 = distanceDepotSort[j - 1];
                int depot_j_1 = distanceDepotSort[j];
                float dis_j_0 = distances[i][depot_j_0];
                float dis_j_1 = distances[i][depot_j_1];
                float ratio = dis_j_0 / dis_j_1;

                if (ratio > 0.7) {
                    potentialDepotsx[depot_j_1] = true;
                    potentialDepotsListx.push_back(depot_j_1);
                    potentialCount++;
                } else {
                    potentialDepotsx[depot_j_1] = false;
                }

            }
            potentialDepots.push_back(potentialDepotsx);
            potentialDepotsList[i] = std::move(potentialDepotsListx);
        }

        return potentialDepots;
    }

    std::vector<int> LocalSearch(int iStart, int iEnd, int kStart, int kEnd, const std::string type, const std::vector<int> &permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes, int64_t &current_cost, int &count, std::vector<int> (*localSearchMethod)(const std::vector<int> &, int, int), const std::vector<std::vector<bool>> &neighbourReduction, const bool intraOnly)
    {
        return LocalSearch(iStart, iEnd, kStart, kEnd, type, permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, 0, count, localSearchMethod, neighbourReduction, intraOnly);
    }

    std::vector<int> LocalSearch(int iStart, int iEnd, int kStart, int kEnd, const std::string type, const std::vector<int> &permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes, int64_t &current_cost, const int64_t global_cost_min, int &count, std::vector<int> (*localSearchMethod)(const std::vector<int> &, int, int), const std::vector<std::vector<bool>> &neighbourReduction, const bool intraOnly)
    {
        count++;
        int g_cost = global_cost_min;
        if (current_cost < g_cost)
        {
            g_cost = current_cost;
        }

        std::vector<int64_t> local_search_better_cost{};
        std::vector<std::vector<int>> local_search_better_permutation{};
        std::vector<int> current_permutation{permutation};
        std::vector<int> neighbour;

        std::random_device rd;
        std::default_random_engine e{rd()};
        const int tt = (nodes_requirements.size() - depot_indexes.size()) * depot_indexes.size();
        std::uniform_int_distribution<unsigned> u(3000 / tt, 9000 / tt);
        // std::uniform_int_distribution<unsigned> u(1, 10);
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

                neighbour = std::move(localSearchMethod(current_permutation, i, k));

                if (!covid19::CheckMultiDepotRequirements(neighbour, nodes_requirements, capacity, depot_indexes))
                {
                    continue;
                }
                int64_t neighbour_cost = CalcCost(type, neighbour, distances, depot_indexes);

                if (current_cost > neighbour_cost)
                {
                    if (neighbour_cost == g_cost)
                    {
                        i = iEnd;
                        break;
                    }
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

    std::vector<int> LocalSearchDepot(const std::string type, const std::vector<int> &permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes, int64_t current_cost, int count)
    {
        // count++;

        std::vector<int64_t> local_search_better_cost{};
        std::vector<std::vector<int>> local_search_better_permutation{};
        std::vector<int> current_permutation{permutation};
        std::vector<int> neighbour;

        int search_better_depth = 2;

        int numRoutes = (permutation.size() - distances.size() + depot_indexes.size()) / 2;
        int depotSize = depot_indexes.size();

        for (int i = 0; i < numRoutes; i++)
        {
            for (int k = 0; k < depotSize; k++)
            {
                neighbour = std::move(ExchangeDepot(permutation, depot_indexes, i, depot_indexes[k]));

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

    std::vector<int> Vns(const std::string type, const std::vector<int> &nodes_permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &depot_indexes, const int vehicles_num)
    {
        const int TRAVEL_SIZE = nodes_permutation.size();
        std::vector<int> current_permutation = nodes_permutation;
        int64_t current_cost = CalcCost(type, nodes_permutation, distances, depot_indexes);

        int count = 0;
        int shakeCount = 0;
        std::uniform_int_distribution<unsigned> u(6, 14);
        std::uniform_int_distribution<unsigned> us(3, 14);
        std::uniform_int_distribution<unsigned> u2(2, std::min(vehicles_num / 2, 10));
        std::uniform_int_distribution<unsigned> shakeMethodU(0, 2);

        std::vector<std::vector<int>> potentialDepotsList(distances.size());
        std::vector<std::vector<bool>> potentialDepots = std::move(PotentialDepots(distances, depot_indexes, potentialDepotsList));
        std::vector<std::vector<bool>> neighbourReduction = std::move(NeighbourReduction(distances, potentialDepots));
        

        int whileCount = 0;
        int64_t prior_cost = current_cost;
        int stappedSteps = 0;
        int current_min_stapped_count = 1;
        while (whileCount < WHILE_NUM)
        {
            std::cout << "XROUND " << whileCount << std::endl;

            std::random_device rd;
            std::default_random_engine e{rd()};
            int max_no_improve = u(e);
            int shake_max_no_improve = us(e);
            int shake_times = u2(e);

            std::cout << "STAGE 1: Local Search" << std::endl;
            do
            {

                current_permutation = std::move(LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, TwoSwap, neighbourReduction));

                current_permutation = std::move(LocalSearch(1, TRAVEL_SIZE - 2, 1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, RelocationMove, neighbourReduction));

                current_permutation = std::move(LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, TwoOptSwap, neighbourReduction));

                current_permutation = std::move(LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, ArcNodeSwap, neighbourReduction));

                current_permutation = std::move(LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count, ArcNodeMove, neighbourReduction));

                // current_permutation = FitDepot(current_permutation, distances, depot_indexes);
                current_permutation = std::move(LocalSearchDepot(type, current_permutation, distances, nodes_requirements, capacity, depot_indexes, current_cost, count));

            } while (count <= max_no_improve);

            do
            {
                shakeCount++;
                shake_times++;
                std::cout << "STAGE 2: Shake" << std::endl;
                int shakeTimes = 0;
                std::vector<int> shaking{current_permutation};
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
                        std::vector<int> priorShaking{shaking};
                        shaking = std::move(Shaking(shaking, potentialDepots, potentialDepotsList, depot_indexes, shakingMethod));
                        if (covid19::CheckMultiDepotRequirements(shaking, nodes_requirements, capacity, depot_indexes))
                        {
                            break;
                        }
                        shaking = priorShaking;
                    }
                } while (shakeTimes++ < shake_times);
                shake_times = u2(e);

                shaking_cost = std::move(CalcCost(type, shaking, distances, depot_indexes));
                // if (shaking_cost - current_cost > 0.6 * current_cost)
                // {
                //     count++;
                //     continue;
                // }

                // shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count, TwoSwap, neighbourReduction, true));

                std::cout << "STAGE 2: Local Search" << std::endl;
                do
                {
                    shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, ArcNodeMove, neighbourReduction));

                    shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, TwoSwap, neighbourReduction));

                    shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 2, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, RelocationMove, neighbourReduction));

                    shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, ArcNodeSwap, neighbourReduction));

                    shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, TwoOptSwap, neighbourReduction));

                    shaking = FitDepot(shaking, distances, depot_indexes);
                    shaking = std::move(LocalSearchDepot(type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count));

                } while (count <= max_no_improve);

                shaking_cost = CalcCost(type, shaking, distances, depot_indexes);
                if (shaking_cost < current_cost)
                {
                    current_cost = shaking_cost;
                    current_permutation = shaking;
                    stappedSteps = 0;
                } else {
                    stappedSteps++;
                }

                std::cout << "STAGE 3: Shake Again" << std::endl;
                shakeMethod = (shakeMethod + 1) % 3;
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
                        std::vector<int> priorShaking{shaking};
                        shaking = std::move(Shaking(shaking, potentialDepots, potentialDepotsList, depot_indexes, shakingMethod));
                        if (covid19::CheckMultiDepotRequirements(shaking, nodes_requirements, capacity, depot_indexes))
                        {
                            break;
                        }
                        shaking = priorShaking;
                    }
                } while (shakeTimes++ < shake_times);
                shake_times = u2(e);

                shaking_cost = CalcCost(type, shaking, distances, depot_indexes);
                // if (shaking_cost - current_cost > 0.5 * current_cost)
                // {
                //     count++;
                //     continue;
                // }

                std::cout << "STAGE 3: Local Search Again" << std::endl;
                do
                {

                    shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, ArcNodeMove, neighbourReduction));

                    shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, TwoOptSwap, neighbourReduction));

                    shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 2, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, RelocationMove, neighbourReduction));

                    shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, TwoSwap, neighbourReduction));

                    shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, ArcNodeSwap, neighbourReduction));

                    shaking = std::move(LocalSearchDepot(type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count));

                    // shaking = FitDepot(shaking, distances, depot_indexes);

                } while (count <= max_no_improve);

                if (shaking_cost < current_cost)
                {
                    current_cost = shaking_cost;
                    current_permutation = shaking;
                    stappedSteps = 0;
                } else {
                    stappedSteps++;
                }

                current_permutation = std::move(FitDepot(current_permutation, distances, depot_indexes));

                std::cout << "STAGE 4: Shake Depot" << std::endl;
                depotBias = 0;
                do
                {
                    // shaking = std::move(ChangeDepot(current_permutation, distances, depot_indexes, CalcDistanceCumCost));
                    shaking = std::move(ChangeDepotGuided(current_permutation, distances, depot_indexes, potentialDepotsList, potentialDepots, CalcDistanceCumCost));
                    do
                    {

                        shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, ArcNodeMove, neighbourReduction));

                        shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, TwoOptSwap, neighbourReduction));

                        shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 2, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, RelocationMove, neighbourReduction));

                        shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 2, -1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, TwoSwap, neighbourReduction));

                        shaking = std::move(LocalSearch(1, TRAVEL_SIZE - 3, 1, TRAVEL_SIZE - 1, type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, stappedSteps > current_min_stapped_count ? current_cost : 0, count, ArcNodeSwap, neighbourReduction));

                        shaking = std::move(LocalSearchDepot(type, shaking, distances, nodes_requirements, capacity, depot_indexes, shaking_cost, count));

                    } while (count <= max_no_improve);

                    if (shaking_cost < current_cost)
                    {
                        current_cost = shaking_cost;
                        current_permutation = shaking;
                        break;
                    }
                    else
                    {
                        depotBias++;
                    }
                } while (depotBias < 1);

            } while (shakeCount <= shake_max_no_improve);

            if (current_cost < prior_cost)
            {
                whileCount = -1;
            }
            whileCount++;
            prior_cost = current_cost;
        }

        return current_permutation;
    }

    std::vector<int> VnsGreedy(const std::string type, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, const std::vector<int> &num_vehicles, const std::vector<int> &depot_indexes)
    {
        std::vector<int> initial_permutation = Greedy(nodes_requirements, capacity, num_vehicles, depot_indexes);
        int vehicles_num_total{0};
        for (auto &&num : num_vehicles)
        {
            vehicles_num_total += num;
        }

        return Vns(type, initial_permutation, distances, nodes_requirements, capacity, depot_indexes, vehicles_num_total);
    }

    std::vector<int> VnsSingle(const std::string type, const std::vector<std::vector<int64_t>> &distances, const std::vector<int64_t> &nodes_requirements, int64_t capacity, int num_vehicles, int depot_index)
    {
        std::vector<int> num_vehicles_list{num_vehicles};
        std::vector<int> depot_indexes{depot_index};
        return VnsGreedy(type, distances, nodes_requirements, capacity, num_vehicles_list, depot_indexes);
    }
} // namespace covid19