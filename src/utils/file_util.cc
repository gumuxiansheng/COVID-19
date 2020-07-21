#include "file_util.h"

namespace covid19
{

const int SCALE = 1000; // to convert float to int, times 1000 as the file contains 3 digits decimal.

std::string& trim(std::string &s) 
{
    if (s.empty()) 
    {
        return s;
    }

    s.erase(0, s.find_first_not_of(" "));
    int size = s.size();
    for (int i = size - 1; i >= 0; i--)
    {
        if (s[i] != ' ' && s[i] != '\0' && s[i] != '\n' && s[i] != '\r')
        {
            break;
        }

        if (s[i] == ' ')
        {
            s.erase(i);
        }
    }
    
    return s;
}

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
                    if (i == line_str.size() || (line_str[i] == ' ' && i != 0))
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

    // std::cout << "Nodes:" << std::endl;
    // for (auto node : nodes)
    // {
    //     for (auto item : node)
    //     {
    //         std::cout << item << "    ";
    //     }
    //     std::cout << std::endl;
    // }
    

    return reData;
}

/*
function: GetNodesRequirements
! @brief: get a list of the requirements from data.
! @param[in]: nodes
! @param[in]: requirement_col, at which column the requirement locates, in Christofides data, it's 2, which is the default, in PR data, it's 4
! @param[out]: the list of requirements
*/
std::vector<int64_t> GetNodesRequirements(std::vector<std::vector<int64_t>> nodes, int requirement_col)
{
    std::vector<int64_t> requirements{};
    requirements.reserve(nodes.size());
    for (auto &node : nodes)
    {
        requirements.push_back(node[requirement_col]);
    }

    return requirements;
}

/*
function: ReadPR
! @brief: read in PR file to matrix.
! @param[in]: file_name, file to be read
! @param[out]: N * 5 matrix for locations and requirements

#vehicles, #customers, #depots, MaxDur, VehicCapacity

customer number, x coordinate, y coordinate, service duration, demand
 */
PRDataModel ReadPR(const std::string &file_name)
{
    PRDataModel reData;

    std::cout << "Read in ReadCordeau" << std::endl;
    std::vector<std::vector<int64_t>> nodes;
    int node_size{0};
    int depot_size{0};
    std::fstream read_in_file;
    read_in_file.open(file_name, std::ios::in);
    if (read_in_file.is_open())
    { //checking whether the file is open
        std::string line_str;
        int line_index = 0;
        while (getline(read_in_file, line_str))
        {
            line_str = trim(line_str);
            if (line_index == 0)
            {
                std::cout << "STARTL: " + line_str << std::endl; //print the data info at line 1

                std::string loc_t_s;
                int space_count = 0;
                for (size_t i = 0; i < line_str.size(); i++)
                {
                    if (line_str[i] == ' ' || i == line_str.size() - 1)
                    {
                        if (i == 0 || line_str[i - 1] == ' ')
                        {
                            continue;
                        }
                        if (++space_count == 1)
                        {
                            reData.vehicles = std::stoi(loc_t_s);
                        } else if (space_count == 2)
                        {
                            node_size = std::stoi(loc_t_s);
                        } else if (space_count == 3)
                        {
                            depot_size = std::stoi(loc_t_s);
                        } else if (space_count == 5)
                        {
                            if (i == line_str.size() - 1)
                            {
                                loc_t_s.append(1, line_str[i]);
                            }
                            
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
                if (line_index == 1) // there is a space line after head
                {
                    line_index++;
                    continue;
                }
                std::vector<int64_t> loc_vec;
                std::string loc_t_s;
                for (size_t i = 0; i < line_str.size() + 1; i++)
                {
                    if (i == line_str.size() || (line_str[i] == ' ' && i != 0 && line_str[i - 1] != ' '))
                    {
                        loc_vec.push_back(std::stoi(loc_t_s));
                        loc_t_s = "";
                    }
                    else if (line_str[i] != ' ')
                    {
                        loc_t_s.append(1, line_str[i]);
                    }
                }
                if (loc_vec[0] > node_size)
                { // it's depot
                    if (loc_vec.size() == 3)
                    { // it's depot
                        loc_vec.push_back(0);
                        loc_vec.push_back(0); // make the depot has the same colums with other nodes
                    }

                    reData.depot_indexes.push_back(nodes.size());
                }
                nodes.push_back(loc_vec);
            }

            line_index++;
        }
        read_in_file.close(); //close the file object.
        // std::cerr << "PR read file succeed." << std::endl;
    }
    else
    {
        std::cerr << "Cordeau read file failed." << std::endl;
    }

    reData.nodes = nodes;

    // std::cout << "Nodes:" << std::endl;
    // for (auto node : nodes)
    // {
    //     for (auto item : node)
    //     {
    //         std::cout << item << "    ";
    //     }
    //     std::cout << std::endl;
    // }
    return reData;
}

PRDataModel ReadPRFloat(const std::string &file_name)
{

    PRDataModel reData;

    std::cout << "Read in ReadCordeau: " << file_name << std::endl;
    std::vector<std::vector<int64_t>> nodes;
    int node_size{0};
    int depot_size{0};
    std::fstream read_in_file;
    read_in_file.open(file_name, std::ios::in);
    if (read_in_file.is_open())
    { //checking whether the file is open
        std::string line_str;
        int line_index = 0;
        while (getline(read_in_file, line_str))
        {
            line_str = trim(line_str);
            if (line_index == 0)
            {
                std::cout << "STARTL: " + line_str << std::endl; //print the data info at line 1

                std::string loc_t_s;
                int space_count = 0;
                for (size_t i = 0; i < line_str.size(); i++)
                {
                    if (line_str[i] == ' ' || i == line_str.size() - 1)
                    {
                        if (i == 0 || line_str[i - 1] == ' ')
                        {
                            continue;
                        }
                        if (++space_count == 1)
                        {
                            reData.vehicles = std::stoi(loc_t_s);
                        } else if (space_count == 2)
                        {
                            node_size = std::stoi(loc_t_s);
                        } else if (space_count == 3)
                        {
                            depot_size = std::stoi(loc_t_s);
                        }  else if (space_count == 5)
                        {
                            if (i == line_str.size() - 1)
                            {
                                loc_t_s.append(1, line_str[i]);
                            }
                            
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
                if (line_index == 1) // there is a space line after head
                {
                    line_index++;
                    continue;
                }
                std::vector<int64_t> loc_vec;
                std::string loc_t_s;
                for (size_t i = 0; i < line_str.size() + 1; i++)
                {
                    if (i == line_str.size() || (line_str[i] == ' ' && i != 0 && line_str[i - 1] != ' '))
                    {

                        if (loc_vec.size() == 1 || loc_vec.size() == 2)
                        {
                            loc_vec.push_back(std::stof(loc_t_s) * SCALE);
                        } else {
                            loc_vec.push_back(std::stoi(loc_t_s));
                        }
                        loc_t_s = "";
                    }
                    else if (line_str[i] != ' ')
                    {
                        loc_t_s.append(1, line_str[i]);
                    }
                }
                if (loc_vec[0] > node_size)
                { // it's depot
                    if (loc_vec.size() == 3)
                    { // it's depot
                        loc_vec.push_back(0);
                        loc_vec.push_back(0); // make the depot has the same colums with other nodes
                    }

                    reData.depot_indexes.push_back(nodes.size());
                }
                nodes.push_back(loc_vec);
            }

            line_index++;
        }
        read_in_file.close(); //close the file object.
        // std::cerr << "PR read file succeed." << std::endl;
    }
    else
    {
        std::cerr << "Cordeau read file failed." << std::endl;
    }

    reData.nodes = nodes;

    // std::cout << "Nodes:" << std::endl;
    // for (auto node : nodes)
    // {
    //     for (auto item : node)
    //     {
    //         std::cout << item << "    ";
    //     }
    //     std::cout << std::endl;
    // }

    return reData;
}

PRDataModel ReadLRFloat(const std::string &file_name)
{
    PRDataModel reData;

    std::cout << "Read in ReadLR: " << file_name << std::endl;
    std::vector<std::vector<int64_t>> nodes;
    std::fstream read_in_file;
    int nodes_num{0};
    int depots_num{0};
    read_in_file.open(file_name, std::ios::in);
    if (read_in_file.is_open())
    { //checking whether the file is open
        std::string line_str;
        int line_index = 0;
        while (getline(read_in_file, line_str))
        {
            line_str = trim(line_str);
            if (line_index == 0)
            {
                std::cout << "STARTL: " + line_str << std::endl; //print the data info at line 1

                std::string loc_t_s;
                int space_count = 0;
                for (size_t i = 0; i < line_str.size(); i++)
                {
                    if (line_str[i] == ' ' || i == line_str.size() - 1)
                    {
                        if (i == 0 || line_str[i - 1] == ' ')
                        {
                            continue;
                        }
                        if (++space_count == 1)
                        {
                            reData.vehicles = std::stoi(loc_t_s);
                        } else if(space_count == 2)
                        {
                            nodes_num = std::stoi(loc_t_s);
                        }  else if(space_count == 3)
                        {
                            depots_num = std::stoi(loc_t_s);
                        } else if (space_count == 4)
                        {
                            if (i == line_str.size() - 1)
                            {
                                loc_t_s.append(1, line_str[i]);
                            }
                            
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
            else if (line_index == 1 || line_index == nodes_num + depots_num + 2)
            { // there is a space line after head
                line_index++;
                continue;
            }
            else if (line_index < nodes_num + depots_num + 2)
            {
                std::vector<int64_t> loc_vec;
                std::string loc_t_s;
                for (size_t i = 0; i < line_str.size() + 1; i++)
                {
                    if (i == line_str.size() || (line_str[i] == ' ' && i != 0 && line_str[i - 1] != ' '))
                    {
                        loc_vec.push_back(std::stof(loc_t_s) * SCALE);
                        loc_t_s = "";
                    }
                    else if (line_str[i] != ' ')
                    {
                        loc_t_s.append(1, line_str[i]);
                    }
                }
                if (line_index >= nodes_num + 2)
                { // it's depot
                    reData.depot_indexes.push_back(nodes.size());
                }
                nodes.push_back(loc_vec);
            } else
            {
                std::vector<int64_t> *p_line_node = &nodes[line_index - (nodes_num + depots_num + 2) - 1];
                std::string loc_t_s;
                for (size_t i = 0; i < line_str.size() + 1; i++)
                {
                    if (i == line_str.size() || (line_str[i] == ' ' && i != 0 && line_str[i - 1] != ' '))
                    {
                        p_line_node->push_back(std::stof(loc_t_s));
                        loc_t_s = "";
                    }
                    else if (line_str[i] != ' ')
                    {
                        loc_t_s.append(1, line_str[i]);
                    }
                }

            }

            line_index++;
        }
        read_in_file.close(); //close the file object.
        // std::cerr << "PR read file succeed." << std::endl;
    }
    else
    {
        std::cerr << "Cordeau read file failed." << std::endl;
    }

    std::cout << "Nodes:" << std::endl;
    for (auto &&node : nodes)
    {
        for (auto item : node)
        {
            std::cout << item << "    ";
        }
        std::cout << std::endl;
    }

    std::cout << "Depots:" << std::endl;
    for (auto item : reData.depot_indexes)
    {
        std::cout << item << "    ";
        std::cout << std::endl;
    }

    reData.nodes = nodes;

    return reData;
}

std::vector<int> ReadResultSolution(const std::string &file_name)
{
    std::vector<int> solution{};
    std::fstream read_in_file;
    read_in_file.open(file_name, std::ios::in);
    if (read_in_file.is_open())
    { //checking whether the file is open
        std::string line_str;
        int line_index = 0;
        while (getline(read_in_file, line_str))
        {
            line_str = trim(line_str);

            std::string loc_t_s;
            int8_t space_continued = 0;
            bool start_route = false;
            for (size_t i = 0; i < line_str.size() + 1; i++)
            {
                if (line_str[i] == 'B')
                { // last line to log total cost and run time
                    break;
                }
                if (start_route)
                {
                    if (i == line_str.size() || (line_str[i] == ' ' && i != 0 && line_str[i - 1] != ' '))
                    {

                        solution.push_back(std::stoi(loc_t_s));
                        loc_t_s = "";
                    }
                    else if (line_str[i] != ' ')
                    {
                        loc_t_s.append(1, line_str[i]);
                    }
                }

                if (line_str[i] == ' ')
                {
                    space_continued++;
                } else
                {
                    space_continued = 0;
                }
                if (space_continued == 4)
                {
                    start_route = true;
                }

            }

            line_index++;
        }
        read_in_file.close(); //close the file object.
    }
    else
    {
        std::cerr << "ResultSolution read file failed." << std::endl;
    }

    return solution;
}

/*
function: ReadDistancesCSV
! @brief: read in csv file to distances matrix.
! @param[in]: file_name, file to be read
! @param[out]: matrix for bilateral distances
*/
std::vector<std::vector<int64_t>> ReadDistancesCSV(const std::string &file_name)
{

    std::cout << "Read in CSV" << std::endl;
    std::vector<std::vector<int64_t>> distances;
    std::fstream read_in_file;
    read_in_file.open(file_name, std::ios::in);
    if (read_in_file.is_open())
    { //checking whether the file is open
        std::string line_str;
        int line_index = 0;
        while (getline(read_in_file, line_str))
        {
            if (++line_index == 1)
            {
                continue; // ignore the head
            }

            std::vector<int64_t> loc_vec;
            std::string loc_t_s;
            int commaCount = 0;
            for (size_t i = 0; i < line_str.size() + 1; i++)
            {
                if (i == line_str.size() || line_str[i] == ',')
                {
                    if (++commaCount == 1)
                    {
                        loc_t_s = "";
                        continue;
                    }
                    loc_vec.push_back(std::stoi(loc_t_s));
                    loc_t_s = "";

                }
                else
                {
                    loc_t_s.append(1, line_str[i]);
                }
            }
            distances.push_back(loc_vec);

            line_index++;
        }
        read_in_file.close(); //close the file object.
    }
    else
    {
        std::cerr << "CSV read failed." << std::endl;
    }

    return distances;
}

std::vector<int64_t> ReadRequirementsCSV(const std::string &file_name)
{

    std::cout << "Read in CSV" << std::endl;
    std::vector<int64_t> requirements;
    std::fstream read_in_file;
    read_in_file.open(file_name, std::ios::in);
    if (read_in_file.is_open())
    { //checking whether the file is open
        std::string line_str;
        int line_index = 0;
        while (getline(read_in_file, line_str))
        {
            std::string loc_t_s;
            for (size_t i = 0; i < line_str.size() + 1; i++)
            {
                if (line_str[i] == ',') 
                {
                    loc_t_s = "";
                } else if (i == line_str.size())
                {
                    requirements.push_back(std::stoi(loc_t_s));
                    
                }
                else
                {
                    loc_t_s.append(1, line_str[i]);
                }
            }

            line_index++;
        }
        read_in_file.close(); //close the file object.
    }
    else
    {
        std::cerr << "CSV read failed." << std::endl;
    }

    return requirements;
}


std::vector<std::vector<int64_t>> ReadLocationsCSV(const std::string &file_name)
{

    std::cout << "Read in CSV" << std::endl;
    std::vector<std::vector<int64_t>> locations;
    std::fstream read_in_file;
    read_in_file.open(file_name, std::ios::in);
    if (read_in_file.is_open())
    { //checking whether the file is open
        std::string line_str;
        int line_index = 0;
        while (getline(read_in_file, line_str))
        {
            if (++line_index == 1)
            {
                continue; // ignore the head
            }

            std::vector<int64_t> loc_vec;
            std::string loc_t_s;
            int commaCount = 0;
            int digitInt = 0;
            for (size_t i = 0; i < line_str.size() + 1; i++)
            {
                if (i == line_str.size() || line_str[i] == ',' || line_str[i] == '\r')
                {
                    if (loc_t_s.empty())
                    {
                        continue;
                    }
                    if (++commaCount < 4)
                    { // latitude in column 4 and longtitude in column 5
                        loc_t_s = "";
                        continue;
                    }
                    if(loc_t_s.size() - digitInt < 6)
                    {
                        loc_t_s.append(6 - loc_t_s.size() + digitInt, '0');
                    }
                    loc_vec.push_back(std::stoi(loc_t_s));
                    loc_t_s = "";

                } else if (line_str[i] == '.')
                {
                    digitInt = loc_t_s.size();
                }
                else
                {
                    loc_t_s.append(1, line_str[i]);
                }
            }
            locations.push_back(loc_vec);

            line_index++;
        }
        read_in_file.close(); //close the file object.
    }
    else
    {
        std::cerr << "CSV read failed." << std::endl;
    }

    return locations;
}

} // namespace covid19