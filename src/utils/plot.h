#ifndef COVID19_PLOT_H_
#define COVID19_PLOT_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include "vector_help.h"

namespace covid19
{
    void PlotUsingGnuplot(const std::string fileUrl, const std::vector<std::vector<int64_t>> &nodes, const std::vector<int> &depot_indexes, const std::vector<int> &nodes_permutation);
} // namespace covid19

#endif