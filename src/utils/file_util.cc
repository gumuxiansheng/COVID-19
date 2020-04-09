#include "file_util.h"

namespace covid19
{
std::vector<std::vector<int64_t>> ReadChristofides(const std::string file_name)
{
    std::cout << "Hello ReadChristofides" << std::endl;
    std::vector<std::vector<int64_t>> locations;
    std::fstream read_in_file;
    read_in_file.open(file_name, std::ios::in);
    if (read_in_file.is_open())
    { //checking whether the file is open
        std::string line_str;
        int line_index = 0;
        while (getline(read_in_file, line_str))
        {
            if (line_index == 0)
            {
                std::cout << "STARTL: " + line_str << "\n"; //print the data info at line 1
            }
            else
            {
                std::vector<int64_t> loc_vec;
                std::string loc_t_s;
                for (size_t i = 0; i < line_str.size() + 1; i++)
                {
                    if (i == line_str.size() || line_str[i] == ' ')
                    {
                        loc_vec.push_back(std::stoi(loc_t_s));
                        loc_t_s = "";
                    }
                    else
                    {
                        loc_t_s.append(1, line_str[i]);
                    }
                }
                locations.push_back(loc_vec);
            }

            line_index++;
        }
        read_in_file.close(); //close the file object.
    }
    else
    {
        std::cerr << "ReadChristofides read file failed." << std::endl;
    }

    // // print result
    // for (size_t i = 0; i < locations.size(); i++)
    // {
    //     for (size_t j = 0; j < locations[i].size(); j++)
    //     {
    //         std::cout << locations[i][j] << "  ";
    //     }
    //     std::cout << std::endl;
    // }

    return locations;
}

} // namespace covid19