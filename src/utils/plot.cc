#include "plot.h"

namespace covid19
{
    std::vector<std::string> colors{"#0060ad",
                                    "#dd181f",
                                    "#006400",
                                    "#ffa500",
                                    "#87ceeb",
                                    "#ee82ee",
                                    "#a0522d",
                                    "#4b0082",
                                    "#000000",
                                    "#808000",
                                    "#ffd700",
                                    "#4682b4",
                                    "#fa8072",
                                    "#ff1493",
                                    "#7e2f8e",
                                    "#77ac30",
                                    "#0000ff",
                                    "#8a2be2",
                                    "#a52a2a",
                                    "#5f9ea0",
                                    "#d2691e",
                                    "#6495ed",
                                    "#008b8b",
                                    "#b8860b",
                                    "#006400",
                                    "#ff8c00",
                                    "#ff1493",
                                    "#daa520",
                                    "#7f7f7f",
                                    "#ff69b4",
                                    "#f08080",
                                    "#800000",
                                    "#0000cd",
                                    "#ba55d3"};
    void PlotUsingGnuplot(const std::string fileUrl, const std::vector<std::vector<int64_t>> &nodes, const std::vector<int> &depot_indexes, const std::vector<int> &nodes_permutation)
    {
        std::ofstream outfile;
        std::ofstream outCmdFile;
        outfile.open(fileUrl + ".plot");
        outCmdFile.open(fileUrl + ".cmd");
        if (!outfile.is_open() || !outCmdFile.is_open())
        {
            std::cout << "file open failed" << std::endl;
        }

        for (size_t i = 0; i < colors.size(); i++)
        {
            outCmdFile << "set style line " << std::to_string(i + 1) << " lc rgb '" << colors[i] << "' lt 1 lw 2 pt 7 ps 1.5" << std::endl;
        }
        outCmdFile << std::endl;
        outCmdFile << "plot ";

        int num_travel = 0;
        std::vector<int> sub_solution;
        for (int i = 0; i < nodes_permutation.size(); ++i)
        {
            int index = nodes_permutation[i];
            sub_solution.push_back(index);
            if (IsIn(index, depot_indexes))
            {
                if (sub_solution.size() != 1)
                {
                    for (size_t j = 0; j < sub_solution.size() - 1; j++)
                    { // ignore the last depot node
                        auto item = sub_solution[j];
                        outfile << nodes[item][0] << " " << nodes[item][1] << std::endl;
                    }
                    ++num_travel;
                    outCmdFile << "'" << fileUrl << ".plot" << "' using 2:1 index " << std::to_string(num_travel - 1) << " with linespoints ls " << std::to_string(num_travel % colors.size()) << " notitle, ";

                    sub_solution.clear();
                    outfile << std::endl
                            << std::endl;
                }
            }
        }

        outfile.close();
        outCmdFile.close();
    }

} // namespace covid19
