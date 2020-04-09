#include "file_util.h"

namespace covid19
{
std::vector<std::vector<int64_t>> ReadChristofides(const std::string file_name)
{
    std::cout << "Hello ReadChristofides" << std::endl;
    std::vector<std::vector<int64_t>> locations(5, std::vector<int64_t>(5));
    std::fstream read_in_file;
    read_in_file.open(file_name, std::ios::in);
    if (read_in_file.is_open())
    { //checking whether the file is open
        std::string line_str;
        while (getline(read_in_file, line_str))
        {
            std::cout << line_str << "\n"; //print the data of the string
        }
        read_in_file.close(); //close the file object.
    } else {
        std::cout << "ReadChristofides read file failed." << std::endl;
    }
    return locations;
}

} // namespace covid19