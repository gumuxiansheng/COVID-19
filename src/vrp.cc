#include <utility>
#include <vector>

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
    int64 max_route_distance{0};
    for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id)
    {
        int64 index = routing.Start(vehicle_id);
        LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
        int64 route_distance{0};
        std::stringstream route;
        while (!routing.IsEnd(index))
        {
            route << manager.IndexToNode(index).value() << " -> ";
            int64 previous_index = index;
            index = solution.Value(routing.NextVar(index));
            route_distance += routing.GetArcCostForVehicle(previous_index, index,
                                                           int64{vehicle_id});
        }
        LOG(INFO) << route.str() << manager.IndexToNode(index).value();
        LOG(INFO) << "Distance of the route: " << route_distance << "m";
        max_route_distance = std::max(route_distance, max_route_distance);
    }
    LOG(INFO) << "Maximum of the route distances: " << max_route_distance << "m";
    LOG(INFO) << "";
    LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}

void VrpGlobalSpan(std::vector<std::vector<int64_t>> distances)
{
    // Instantiate the data problem.
    DataModel data;
    data.distance_matrix = std::move(distances);
    data.num_vehicles = 3;

    // Create Routing Index Manager
    RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                                data.depot);

    // Create Routing Model.
    RoutingModel routing(manager);

    // Create and register a transit callback.
    const int transit_callback_index = routing.RegisterTransitCallback(
        [&data, &manager](int64 from_index, int64 to_index) -> int64 {
            // Convert from routing variable Index to distance matrix NodeIndex.
            auto from_node = manager.IndexToNode(from_index).value();
            auto to_node = manager.IndexToNode(to_index).value();
            return data.distance_matrix[from_node][to_node];
        });

    // Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

    // Add Distance constraint.
    routing.AddDimension(transit_callback_index, 0, 3000,
                         true, // start cumul to zero
                         "Distance");
    routing.GetMutableDimension("Distance")->SetGlobalSpanCostCoefficient(100);

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
    std::string file_url;
    std::cout << "please enter the Christofides file url:" << std::endl;
    std::cin >> file_url;

    // std::string file_url = "/Volumes/Mike_External/Dev/COVID-19/data/demo/Christofides_1_50.txt";

    covid19::ChristofidesDataModel christofides_data = covid19::ReadChristofides(file_url);
    std::vector<std::vector<int64_t>> locations = christofides_data.nodes;
    std::vector<std::vector<int64_t>> distances = covid19::CalcDistances(locations);

    // // print result
    // for (size_t i = 0; i < distances.size(); i++)
    // {
    //     for (size_t j = 0; j < distances[i].size(); j++)
    //     {
    //         std::cout << distances[i][j] << "  ";
    //     }
    //     std::cout << std::endl;
    // }

    operations_research::VrpGlobalSpan(distances);
    return EXIT_SUCCESS;
}