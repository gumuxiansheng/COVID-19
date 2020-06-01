#ifndef COVID19_FILE_H_
#define COVID19_FILE_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
// #include <dirent.h>
// #include <stdlib.h>

namespace covid19
{
struct ChristofidesDataModel
{
    int64_t capacity = 1;
    std::vector<std::vector<int64_t>> nodes{};
};

struct PRDataModel
{
    int vehicles = 1;
    int64_t capacity = 1;
    std::vector<std::vector<int64_t>> nodes{};
    std::vector<int> depot_indexes{};
};

ChristofidesDataModel ReadChristofides(const std::string &file_name);

PRDataModel ReadPR(const std::string &file_name);

PRDataModel ReadPRFloat(const std::string &file_name);

PRDataModel ReadLRFloat(const std::string &file_name);

std::vector<int> ReadResultSolution(const std::string &file_name);

std::vector<int64_t> GetNodesRequirements(std::vector<std::vector<int64_t>> nodes, int requirement_col = 2);

std::vector<std::vector<int64_t>> ReadDistancesCSV(const std::string &file_name);

// std::vector<std::string> ListFiles(std::string folder);

} // namespace covid19

#endif // COVID19_FILE_H_