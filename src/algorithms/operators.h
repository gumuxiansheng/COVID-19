//
// Created by MikeZhu on 2020/6/5.
//

#ifndef COVID19_OP_H_
#define COVID19_OP_H_

#include <iostream>
#include <vector>

namespace covid19
{

/**
 * swap two nodes
 * @param nodes_permutation
 * @param swap_start_index
 * @param swap_end_index
 * @return
 */
std::vector<int> TwoSwap(const std::vector<int>& nodes_permutation, int swap_start_index, int swap_end_index);

/**
 * two_opt_swap
 * revert the items between start and end(both include).
 *             start  end
 *               +     +
 *               v     v
 *          +----+-----+----+
 *          |1|2|3|4|5|6|7|8|
 *          +---------------+
 *                  |
 *                  v
 *          +---------------+
 *          |1|2|6|5|4|3|7|8|
 *          +---------------+
 *
 * @param nodes_permutation
 * @param swap_start_index
 * @param swap_end_index
 * @return
 */
std::vector<int> TwoOptSwap(const std::vector<int>& nodes_permutation, int swap_start_index, int swap_end_index);

/**
 * two_h_opt_swap
 * move two nodes to the head and order the remain nodes.
 *            swap_1 swap_2
 *               +     +
 *               v     v
 *          +----+-----+----+
 *          |1|2|3|4|5|6|7|8|
 *          +---------------+
 *                  |
 *                  v
 *          +---------------+
 *          |3|6|4|5|7|8|1|2|
 *          +---------------+
 *
 * @param nodes_permutation
 * @param swap_start_index
 * @param swap_end_index
 * @return
 */
std::vector<int> TwoHOptSwap(const std::vector<int>& nodes_permutation, int swap_1, int swap_2);

std::vector<int> ArcNodeSwap(const std::vector<int> &nodes_permutation, int swap_1, int swap_2);

std::vector<int> ArcNodeMove(const std::vector<int> &nodes_permutation, int index_1, int index_2);

std::vector<int> RelocationMove(const std::vector<int>& nodes_permutation, int item_index, int insert_index);

static int depotBias{0};
std::vector<int> ChangeDepot(const std::vector<int> &nodes_permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int> &depotIndexes, int64_t (*costCalc)(const std::vector<int> &, const std::vector<std::vector<int64_t>> &, const std::vector<int> &));

std::vector<int> ChangeDepotGuided(const std::vector<int> &nodes_permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int> &depotIndexes, std::vector<std::vector<int>> potentialDepotsList, std::vector<std::vector<bool>> potentialDepots, int64_t (*costCalc)(const std::vector<int> &, const std::vector<std::vector<int64_t>> &, const std::vector<int> &));

std::vector<int> FitDepot(const std::vector<int> &nodes_permutation, const std::vector<std::vector<int64_t>> &distances, const std::vector<int> &depotIndexes);

std::vector<int> ExchangeDepot(const std::vector<int> &nodes_permutation, const std::vector<int> &depotIndexes, const int subRouteNum, const int changeDepot);


} // covid19

#endif // COVID19_OP_H_