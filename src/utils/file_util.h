#ifndef COVID19_FILE_H_
#define COVID19_FILE_H_

#include <iostream>
#include <vector>
#include <fstream>
#include <string>

namespace covid19
{
std::vector<std::vector<int64_t>> ReadChristofides(const std::string& file_name);

} // namespace covid19

#endif // COVID19_FILE_H_