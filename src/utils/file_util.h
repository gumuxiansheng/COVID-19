#ifndef COVID19_FILE_H_
#define COVID19_FILE_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string>

namespace covid19
{
struct ChristofidesDataModel
{
    int64_t capacity = 1;
    std::vector<std::vector<int64_t>> nodes{};
};

ChristofidesDataModel ReadChristofides(const std::string &file_name);

std::vector<int64_t> GetChristofidesRequirements(std::vector<std::vector<int64_t>> nodes);

std::vector<std::vector<int64_t>> ReadDistancesCSV(const std::string &file_name);

} // namespace covid19

#endif // COVID19_FILE_H_