#include <vector>
#include <numeric>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

#include "utils/file_util.h"
#include "utils/distance_util.h"

namespace operations_research
{
struct DataModel
{
    std::vector<std::vector<int64>> distance_matrix{};
    std::vector<int64> demands{};
    std::vector<int64> vehicle_capacities{1};
    int num_vehicles = 1;
    RoutingIndexManager::NodeIndex depot{0};
};

//! @brief Print the solution.
//! @param[in] data Data of the problem.
//! @param[in] manager Index manager used.
//! @param[in] routing Routing solver used.
//! @param[in] solution Solution found by the solver.
void PrintSolution(const DataModel &data, const RoutingIndexManager &manager,
                   const RoutingModel &routing, const Assignment &solution)
{
    int64 total_distance{0};
    int64 total_load{0};
    for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id)
    {
        int64 index = routing.Start(vehicle_id);
        LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
        int64 route_distance{0};
        int64 route_load{0};
        std::stringstream route;
        while (!routing.IsEnd(index))
        {
            int64 node_index = manager.IndexToNode(index).value();
            route_load += data.demands[node_index];
            route << node_index << " Load(" << route_load << ") -> ";
            int64 previous_index = index;
            index = solution.Value(routing.NextVar(index));
            route_distance += routing.GetArcCostForVehicle(previous_index, index,
                                                           int64{vehicle_id});
        }
        LOG(INFO) << route.str() << manager.IndexToNode(index).value();
        LOG(INFO) << "Distance of the route: " << route_distance << "m";
        LOG(INFO) << "Load of the route: " << route_load;
        total_distance += route_distance;
        total_load += route_load;
    }
    LOG(INFO) << "Total distance of all routes: " << total_distance << "m";
    LOG(INFO) << "Total load of all routes: " << total_load;
    LOG(INFO) << "";
    LOG(INFO) << "Advanced usage:";
    LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}

DataModel initDataModel(){
    DataModel data;

    std::string file_url;
    std::cout << "please enter the Christofides file url:" << std::endl;
    std::cin >> file_url;

    covid19::ChristofidesDataModel christofides_data = covid19::ReadChristofides(file_url);
    std::vector<std::vector<int64_t>> distances = covid19::CalcDistances(christofides_data.nodes);

    data.distance_matrix = std::move(distances);
    data.demands = std::move(covid19::GetChristofidesRequirements(christofides_data.nodes));
    int64_t sum_demands = accumulate(data.demands.begin(), data.demands.end(), 0);
    data.num_vehicles = sum_demands/christofides_data.capacity + 1;
    data.vehicle_capacities.clear();
    data.vehicle_capacities.assign(data.num_vehicles, christofides_data.capacity);

    return data;
}

void VrpCapacity()
{
    DataModel data = initDataModel();

    // Create Routing Index Manager
    RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                                data.depot);

    // Create Routing Model.
    RoutingModel routing(manager);

    // Create and register a transit callback.
    const int transit_callback_index = routing.RegisterTransitCallback(
        [&data, &manager](int64 from_index, int64 to_index) -> int64 {
            // Convert from routing variable Index to distance matrix NodeIndex.
            int from_node = manager.IndexToNode(from_index).value();
            int to_node = manager.IndexToNode(to_index).value();
            return data.distance_matrix[from_node][to_node];
        });

    // Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

    // Add Capacity constraint.
    const int demand_callback_index = routing.RegisterUnaryTransitCallback(
        [&data, &manager](int64 from_index) -> int64 {
            // Convert from routing variable Index to demand NodeIndex.
            int from_node = manager.IndexToNode(from_index).value();
            return data.demands[from_node];
        });
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,   // transit callback index
        int64{0},                // null capacity slack
        data.vehicle_capacities, // vehicle maximum capacities
        true,                    // start cumul to zero
        "Capacity");

    // Setting first solution heuristic.
    RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
    searchParameters.set_first_solution_strategy(
        FirstSolutionStrategy::PATH_CHEAPEST_ARC);

    // Solve the problem.
    const Assignment *solution = routing.SolveWithParameters(searchParameters);

    // Print solution on console.
    PrintSolution(data, manager, routing, *solution);
}
} // namespace operations_research

int main(int argc, char **argv)
{
    operations_research::VrpCapacity();
    return EXIT_SUCCESS;
}