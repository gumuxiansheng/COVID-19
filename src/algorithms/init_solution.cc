//
// Created by MikeZhu on 2020/5/12.
//

#include "init_solution.h"

namespace covid19
{

int DecentCmp(int a,int b)
{
    return b<a;
}

std::vector<int> Greedy (const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes)
{
    const int NODE_SIZE = nodes_requirements.size();
    const int DEPOT_SIZE = depot_indexes.size();
    std::vector<int> initial_permutation;
    std::vector<int> num_vehicles_t(num_vehicles);
    int current_depot_i = 0;

    std::vector<int64_t> sorted_requirements = nodes_requirements;
    std::sort(sorted_requirements.begin(), sorted_requirements.end(), DecentCmp); // sort requirements from large to small

    int64_t cum_requirements  = 0;

    std::vector<int> remain_nodes;
    for (int j = 0; j < NODE_SIZE; ++j) 
    {
        // exclude the depot
        if (IsIn(j, depot_indexes)){
            continue;
        }
        remain_nodes.push_back(j);
    }
    while (!remain_nodes.empty()) 
    {
        int depot_iter_count = 0;
        while (num_vehicles_t[current_depot_i] <= 0 && depot_iter_count < DEPOT_SIZE)
        {
            current_depot_i = (current_depot_i + 1) % DEPOT_SIZE;
            depot_iter_count++;
        }
        depot_iter_count = 0;
        
        initial_permutation.push_back(depot_indexes[current_depot_i]);
        num_vehicles_t[current_depot_i]--;
        std::vector<int> used_sort_index{};
        for (int i = 0; i < sorted_requirements.size(); ++i) { // start from the max requirement
            if (cum_requirements <= capacity - sorted_requirements[i]){
                cum_requirements += sorted_requirements[i];
                // find the node in remain_nodes
                for (int j = 0; j < remain_nodes.size(); ++j) {
                    int idx = remain_nodes[j];
                    if (nodes_requirements[idx] == sorted_requirements[i]){ // found the node
                        initial_permutation.push_back(idx);
                        used_sort_index.push_back(i);
                        remain_nodes.erase(std::begin(remain_nodes) + j);
                        break;
                    }
                }

            }
        }
        for (int k = 0; k < used_sort_index.size(); ++k) {
            int idx = used_sort_index[k] - k;

            sorted_requirements.erase(std::begin(sorted_requirements) + idx);
        }

        used_sort_index.clear();

        initial_permutation.push_back(depot_indexes[current_depot_i]);
        current_depot_i = (current_depot_i + 1) % DEPOT_SIZE;
        cum_requirements = 0;
    }
    for (size_t i = 0; i < num_vehicles_t.size(); i++)
    {
        while (num_vehicles_t[i] > 0) 
        {
            initial_permutation.push_back(depot_indexes[i]);
            initial_permutation.push_back(depot_indexes[i]); // push twice as start and end respectively
            num_vehicles_t[i]--;
            
        }
    }
    
    std::cout << "initial solution: " << std::endl;
    for (size_t i = 0; i < initial_permutation.size(); ++i) {
        int index = initial_permutation[i];
        std::cout << index << " -> ";
    }
    std::cout << std::endl;
    return initial_permutation;
}

std::vector<int> RegretInsersion (const std::string type, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes)
{
    // current_routes records routes from each depots
    // +-----------+
    // |A|1|2|5|8|A| -- Route start from depot A
    // +-----------+
    // |B|3|6|B| | | -- Route start from depot B
    // +-----------+
    // |C|4|C| | | | -- Route start from depot C
    // +-------+-+-+
    //
    std::vector<std::vector<int>> current_routes{};
    std::vector<int> remain_customers;

    // STEP 0: initial routes and remain_customers
    for (size_t i = 0; i < distances.size(); i++)
    {
        if (!IsIn(i, depot_indexes))
        {
            remain_customers.push_back(i);
        }
    }
    for (size_t i = 0; i < depot_indexes.size(); i++)
    {
        int depot = depot_indexes[i];
        for (size_t j = 0; j < num_vehicles[i]; j++)
        {
            std::vector<int> depot_route{depot};
            current_routes.push_back(depot_route);
        }

    }

    // std::cout << "Step 0 current_routes" << std::endl;
    // for (auto depot_route : current_routes)
    // {
    //     for (auto depot : depot_route)
    //     {
    //         std::cout << depot << "    ";
    //     }
    //     std::cout << std::endl;
    // }
    
    // STEP 1: intial routes with the nearest customer to the depot
    for (auto &depot_route : current_routes)
    {
        int depot = depot_route[0];
        int min_point = 0;
        if (remain_customers.empty())
        {
            break;
        }
        int64_t min_dis = distances[depot][remain_customers[min_point]];
        for (size_t index = 0; index < remain_customers.size(); index++)
        {
            int remain_cus = remain_customers[index];
            if (distances[depot][remain_cus] < min_dis)
            {
                min_dis = distances[depot][remain_cus];
                min_point = index;
            }
        }

        depot_route.push_back(remain_customers[min_point]); // the first customer in route start from this depot
        remain_customers.erase(remain_customers.begin() + min_point);

    }

    RegretInsersionRecur(distances, nodes_requirements, capacity, num_vehicles, depot_indexes, current_routes, remain_customers);

    for (auto &route : current_routes)
    {
        route.push_back(route[0]); // ends the route with the start depot
    }

    std::vector<int> initial_route{};
    for (auto route : current_routes)
    {
        for (auto node : route)
        {
            initial_route.push_back(node);
        }
        
    }
    
    return initial_route;
}

/* Calculate remain cost
    Remain Customer x cost
    +-----------+
    |30|35|60|23| -- insertion to route 1
    +-----------+
    |40|21|  |  | -- insertion to route 2
    +-----------+
    |35|56|38|  | -- insertion to route 3
    +-----------+
 */
std::vector<std::vector<std::vector<int64_t>>> CalcRemainCosts(const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& depot_indexes, std::vector<std::vector<int>> &current_routes, std::vector<int> &remain_customers)
{
    std::vector<std::vector<std::vector<int64_t>>> remain_costs;
    for (auto cus : remain_customers)
    {
        std::vector<std::vector<int64_t>> cus_costs{};
        for (const auto& route : current_routes)
        {
            std::vector<int64_t> cus_pos_cost{};
            for (size_t i = 1; i <= route.size(); i++)
            {
                std::vector<int> nodes_permutation{route};
                nodes_permutation.insert(nodes_permutation.begin() + i, cus);
                int64_t cost = 0;
                if (CheckRequirements(nodes_permutation, nodes_requirements, capacity))
                {
                    cost = CalcCost("cumdistance", nodes_permutation, distances, depot_indexes);
                } else 
                {
                    cost = INT64_MAX;
                }
                cus_pos_cost.push_back(cost);
            }
            cus_costs.push_back(cus_pos_cost);
            
        }
        remain_costs.push_back(cus_costs);
    }

    return remain_costs;
}

std::pair<std::vector<int64_t>, std::vector<std::vector<int>>> CalcRegretValue(const std::vector<std::vector<std::vector<int64_t>>> &remain_costs)
{
    std::vector<int64_t> regret_value{};
    std::vector<std::vector<int>> min_positions{};

    for (auto cus_costs : remain_costs)
    {
        int64_t cus_regret{};
        std::vector<int> min_pos{0, 1}; // position inserted to one route should not insert before the start depot, so here min_pos[1] is one larger than the index in cus_costs
        int64_t min_cost = cus_costs[min_pos[0]][min_pos[1] - 1];
        std::vector<int64_t> costs_for_sort{};
        int available_routes_num = 0;
        int last_available_route = 0;
        for (size_t i = 0; i < cus_costs.size(); i++)
        {
            if (cus_costs[i][0] < INT64_MAX)
            {
                available_routes_num++;
                last_available_route = i;
            }
            for (size_t j = 0; j < cus_costs[i].size(); j++)
            {
                if (cus_costs[i][j] < min_cost)
                {
                    min_cost = cus_costs[i][j];
                    min_pos[0] = i;
                    min_pos[1] = j + 1;
                }
                costs_for_sort.push_back(cus_costs[i][j]);
            }
        }
        std::sort(costs_for_sort.begin(), costs_for_sort.end());
        cus_regret = costs_for_sort[2] - costs_for_sort[0];
        if (available_routes_num < 2)
        { // some customer have only one route to add in.
            cus_regret = INT64_MAX;
        }
        regret_value.push_back(cus_regret);
        min_positions.push_back(min_pos);
    }
    std::pair<std::vector<int64_t>, std::vector<std::vector<int>>> result{regret_value, min_positions};
    return result;
}

void RegretInsersionRecur (const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& num_vehicles, const std::vector<int>& depot_indexes, std::vector<std::vector<int>> &current_routes, std::vector<int> &remain_customers)
{
    // std::cout << "current_routes" << std::endl;
    // for (auto depot_route : current_routes)
    // {
    //     int64_t req = 0;
    //     for (auto depot : depot_route)
    //     {
    //         std::cout << depot << "    ";
    //         req += nodes_requirements[depot];
    //     }
    //     std::cout << "requirments: " << req << std::endl;
    // }

    if (remain_customers.empty())
    {
        return;
    }
    // STEP 2: Calculate remain cost
    // Remain Customer x cost
    // +-----------+
    // |30|35|60|23| -- insertion to route 1
    // +-----------+
    // |40|21|  |  | -- insertion to route 2
    // +-----------+
    // |35|56|38|  | -- insertion to route 3
    // +-----------+
    //
    auto remain_costs = std::move(CalcRemainCosts(distances, nodes_requirements, capacity, depot_indexes, current_routes, remain_customers));
    for (size_t i = 0; i < remain_costs.size(); i++)
    {
        if (remain_customers[i] != 15)
        {
            continue;
        }
        auto x = remain_costs[i];
        std::cout << "customer: " << remain_customers[i] << std::endl;
        for (auto &&j : x)
        {
            for (auto &&k : j)
            {
                std::cout << k << "    ";
            }
            std::cout << std::endl;
        }
        
    }
    
    
    // STEP 3: Calculate regret value

    // regret_value
    // +--+
    // |20| -- customer 1's min regret
    // +--+
    // |31| -- customer 2's min regret
    // +--+
    // |48| -- customer 3's min regret, this is the most regret, so customer 3 should be chosen to add to one route
    // +--+
    // |38| -- customer 4's min regret
    // +--+
    //
    // min_positions
    // +-----+
    // | 0| 3| -- customer 1's min regret insertion position
    // +-----+
    // | 2| 1| -- customer 2's min regret insertion position
    // +-----+
    // | 0| 1| -- customer 3's min regret insertion position, as this is what we chosen from least regret_value, customer 3 will be insert to route 0's position 1
    // +-----+
    // | 1| 2| -- customer 4's min regret insertion position
    // +-----+
    //
    auto regretCalcResult = std::move(CalcRegretValue(remain_costs));
    std::vector<int64_t> regret_value{std::move(regretCalcResult.first)};
    std::vector<std::vector<int>> min_positions{std::move(regretCalcResult.second)};
    

    // STEP 4: choose most regret
    int max_regret_pointer = 0;
    int64_t max_regret_value = regret_value[max_regret_pointer];
    for (size_t i = 1; i < regret_value.size(); i++)
    {
        if (regret_value[i] > max_regret_value)
        {
            max_regret_value = regret_value[i];
            max_regret_pointer = i;
        }
    }

    int max_regret_customer = remain_customers[max_regret_pointer];
    std::vector<int> max_regret_position = min_positions[max_regret_pointer];
    // std::cout << "max_regret_customer: " << max_regret_customer << ", max_regret_value: " << max_regret_value << std::endl;

    current_routes[max_regret_position[0]].insert(current_routes[max_regret_position[0]].begin() + max_regret_position[1], max_regret_customer);
    remain_customers.erase(remain_customers.begin() + max_regret_pointer);

    RegretInsersionRecur(distances, nodes_requirements, capacity, num_vehicles, depot_indexes, current_routes, remain_customers);
}

bool CheckVechicleAssign(const std::vector<std::vector<int>> &current_routes, const int &input_vehicles)
{
    int count{0};
    for (const auto &route : current_routes)
    {
        if (route.size() > 1)
        {
            count++;
        }
    }
    
    if (count < input_vehicles)
    {
        return true;
    }
    return false;
}

std::vector<int> RegretInsersionAssign (const std::string type, const std::vector<std::vector<int64_t>>& distances, const std::vector<int64_t>& nodes_requirements, int64_t capacity, const std::vector<int>& depot_indexes, const int &input_vehicles)
{
    std::vector<std::vector<int>> current_routes{};
    std::vector<int> remain_customers;

    // STEP 0: initial routes and remain_customers
    for (size_t i = 0; i < distances.size(); i++)
    {
        if (!IsIn(i, depot_indexes))
        {
            remain_customers.push_back(i);
        }
    }
    for (size_t i = 0; i < depot_indexes.size(); i++)
    {
        int depot = depot_indexes[i];
        std::vector<int> depot_route{depot};
        current_routes.push_back(depot_route);
    }

    while(CheckVechicleAssign(current_routes, input_vehicles))
    {
        // calc remain cost and regret values
        auto remain_costs = std::move(CalcRemainCosts(distances, nodes_requirements, capacity, depot_indexes, current_routes, remain_customers));
        auto regretCalcResult = std::move(CalcRegretValue(remain_costs));
        std::vector<int64_t> regret_value{std::move(regretCalcResult.first)};
        std::vector<std::vector<int>> min_positions{std::move(regretCalcResult.second)};

        // choose most regret
        int max_regret_pointer = 0;
        int64_t max_regret_value = regret_value[max_regret_pointer];
        for (size_t i = 1; i < regret_value.size(); i++)
        {
            if (regret_value[i] > max_regret_value)
            {
                max_regret_value = regret_value[i];
                max_regret_pointer = i;
            }
        }

        int max_regret_customer = remain_customers[max_regret_pointer];
        std::vector<int> max_regret_position = min_positions[max_regret_pointer];

        current_routes[max_regret_position[0]].insert(current_routes[max_regret_position[0]].begin() + max_regret_position[1], max_regret_customer);
        remain_customers.erase(remain_customers.begin() + max_regret_pointer);
        if (current_routes[max_regret_position[0]].size() == 2)
        {
            std::vector<int> depot_route{current_routes[max_regret_position[0]][0]};
            current_routes.push_back(depot_route); // add a vehicle to the depot
        }
    }

    std::vector<int> vehicles(depot_indexes.size());
    int depot_0_index = current_routes[0][0];
    for (const auto &route : current_routes)
    {
        if (route.size() > 1)
        {
            vehicles[route[0] - depot_0_index] += 1;
        }
    }
    return vehicles;

}
} // namespace covid19
