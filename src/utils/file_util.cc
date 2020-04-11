#include "file_util.h"

namespace covid19
{

/*
function: ReadChristofides
! @brief: read in christofides file to matrix.
! @param[in]: file_name, file to be read
! @param[out]: N * 3 matrix for locations and requirements

Christofides file introduction can be found below.

************************************************************************

DATA FORMAT

#customers, capacity, distance, (service time)
depot x 	depot y		depot requirement (always 0)
customer x 	customer y	customer requirement
customer x	customer y	customer requirement
.
.
.
customer x	customer y	customer requirement

************************************************************************

DATA SOURCE

Small: http://www.coin-or.org/SYMPHONY/branchandcut/VRP/data/index.htm
Medium and Large: http://www.rhsmith.umd.edu/faculty/bgolden/vrp_data.htm

For 

Small (classical ones): 21 instances including Fisher's.
    Christofides_k_n (k=1,...14, n=21,...199
    Fisher_n (n=44,71, 134).

Medium: 
    Golden_k_240 (k=1,...20) with n=240,...,483

Large ones:
    Li_n: k=21,..32, n=560,...,1200
*/
ChristofidesDataModel ReadChristofides(const std::string &file_name)
{
    ChristofidesDataModel reData;

    std::cout << "Read in ReadChristofides" << std::endl;
    std::vector<std::vector<int64_t>> nodes;
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

                std::string loc_t_s;
                int space_count = 0;
                for (size_t i = 0; i < line_str.size() + 1; i++)
                {
                    if (line_str[i] == ' ')
                    {
                        if (++space_count == 2)
                        {
                            reData.capacity = std::stoi(loc_t_s);
                        }
                        loc_t_s = "";
                    }
                    else
                    {
                        loc_t_s.append(1, line_str[i]);
                    }
                }
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
                nodes.push_back(loc_vec);
            }

            line_index++;
        }
        read_in_file.close(); //close the file object.
    }
    else
    {
        std::cerr << "ReadChristofides read file failed." << std::endl;
    }

    reData.nodes = nodes;

    // // print result
    // for (size_t i = 0; i < nodes.size(); i++)
    // {
    //     for (size_t j = 0; j < nodes[i].size(); j++)
    //     {
    //         std::cout << nodes[i][j] << "  ";
    //     }
    //     std::cout << std::endl;
    // }

    return reData;
}

std::vector<int64_t> GetChristofidesRequirements(std::vector<std::vector<int64_t>> nodes)
{
    std::vector<int64_t> requirements{};
    for (size_t i = 0; i < nodes.size(); i++)
    {
        requirements.push_back(nodes[i][2]);
    }
    
    return requirements;
}

} // namespace covid19