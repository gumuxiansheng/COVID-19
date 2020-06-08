#include "operators.h"

#include <algorithm>
#include <random>
#include "../utils/vector_help.h"
#include <float.h>

namespace covid19
{
    const static double DEPOT_RATIO_THETA = 0.2;
    const static double DEPOT_INDEX_THETA = 0.2;
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

    std::vector<std::vector<int>> GetSubRoutes(const std::vector<int> &nodes_permutation, const std::vector<int> &depotIndexes)
    {
        std::vector<std::vector<int>> subRoutes{};
        std::vector<int> subRoute{};
        bool isInRoute{false};
        for (auto &&node : nodes_permutation)
        {
            subRoute.push_back(node);
            if (IsIn(node, depotIndexes))
            {
                if (!isInRoute)
                {
                    isInRoute = true;
                } else
                {
                    subRoutes.push_back(subRoute);
                    subRoute = std::vector<int>{};
                    isInRoute = false;
                }
                
            }
        }
        
        return subRoutes;
    }

    std::vector<int> RevertSubroutes(const std::vector<std::vector<int>> &subroutes)
    {
        std::vector<int> route_permutation{};
        for (auto &&route : subroutes)
        {
            for (auto &&node : route)
            {
                route_permutation.push_back(node);
            }
            
        }
        return route_permutation;

    }

    std::vector<int> ChangeDepot(const std::vector<int> &nodes_permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int> &depotIndexes, int64_t (*costCalc)(const std::vector<int> &, const std::vector<std::vector<int64_t>> &, const std::vector<int> &))
    {
        std::vector<std::vector<int>> subRoutes = GetSubRoutes(nodes_permutation, depotIndexes);
        const int numRoutes = subRoutes.size();
        std::vector<int> routesLength(numRoutes);
        std::vector<int64_t> routesCost(numRoutes);
        std::vector<double> routesRatio(numRoutes);
        std::vector<std::vector<int>> depotLens(depotIndexes.size());
        std::vector<double> depotAverageLen(depotIndexes.size());
        for (size_t i = 0; i < numRoutes; i++)
        {
            routesLength[i] = subRoutes[i].size();
            routesCost[i] = costCalc(subRoutes[i], distances, depotIndexes);
            routesRatio[i] = double(routesCost[i]) / routesLength[i];
            int depot = subRoutes[i][0];
            for (size_t j = 0; j < depotIndexes.size(); j++)
            {
                if(depot == depotIndexes[j])
                {
                    depotLens[j].push_back(routesLength[i]);
                    break;
                }
                
            }
        }

        std::random_device rd;
        std::default_random_engine e{rd()};
        std::uniform_int_distribution<double> u(0, 1);

        // choose route to be replaced
        std::vector<double> routesRatioSort{routesRatio};
        std::sort(routesRatioSort.begin(), routesRatioSort.end());
        double z = u(e);
        int pickI = std::ceil(std::pow(z, DEPOT_RATIO_THETA) * (numRoutes - 1));
        double pickRatio = routesRatioSort[pickI];

        // choose deopt to replace
        for (size_t i = 0; i < depotLens.size(); i++)
        {
            int totalLen = 0;
            for (auto &&len : depotLens[i])
            {
                totalLen += len;
            }

            if (depotLens[i].empty())
            {
                depotAverageLen[i] = DBL_MAX;
            } else
            {
                depotAverageLen[i] = double(totalLen) / depotLens[i].size();
            }

        }
        std::vector<double> depotAverageLenSort{depotAverageLen};
        std::sort(depotAverageLenSort.begin(), depotAverageLenSort.end());
        z = u(e);
        pickI = std::ceil(std::pow(z, DEPOT_INDEX_THETA) * (depotIndexes.size() - 1));
        double pickIndex = depotAverageLenSort[pickI];
        
        int depotReplace = -1;
        for (size_t i = 0; i < depotIndexes.size(); i++)
        {
            if (depotAverageLen[i] == pickIndex)
            {
                depotReplace = depotIndexes[i];
                break;
            }
        }

        // replace
        for (size_t i = 0; i < subRoutes.size(); i++)
        {
            if (routesRatio[i] == pickRatio)
            {
                *subRoutes[i].begin() = depotReplace;
                *(subRoutes[i].end() - 1) = depotReplace;
                break;
            }
        }
        
        return RevertSubroutes(subRoutes);

    }

    std::vector<int> FitDepot(const std::vector<int> &nodes_permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int> &depotIndexes)
    {
        std::vector<std::vector<int>> subRoutes = GetSubRoutes(nodes_permutation, depotIndexes);
        const int numRoutes = subRoutes.size();
        for (size_t i = 0; i < numRoutes; i++)
        {
            int currentDepot = *subRoutes[i].begin();
            for (auto &&depot : depotIndexes)
            {
                int64_t distDepot = distances[depot][subRoutes[i][1]];
                int64_t currDistDepot = distances[currentDepot][subRoutes[i][1]];
                if (currentDepot != depot && (distDepot < currDistDepot))
                {
                    *subRoutes[i].begin() = depot;
                    *(subRoutes[i].end() - 1) = depot;
                }
            }
            
        }

        return RevertSubroutes(subRoutes);

    }

    std::vector<int> ExchangeDepot(const std::vector<int> &nodes_permutation, const std::vector<int> &depotIndexes, const int subRouteNum, const int changeDepot)
    {
        std::vector<std::vector<int>> subRoutes = GetSubRoutes(nodes_permutation, depotIndexes);
        const int numRoutes = subRoutes.size();
        if (subRouteNum > numRoutes)
        {
            return nodes_permutation;
        }
        *subRoutes[subRouteNum].begin() = changeDepot;
        *(subRoutes[subRouteNum].end() - 1) = changeDepot;
        return RevertSubroutes(subRoutes);

    }

} // covid19