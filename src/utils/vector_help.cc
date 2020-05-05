#include "vector_help.h"

namespace covid19
{
bool IsIn(const int &value, const std::vector<int> &vec)
{
    for (auto i : vec)
    {
        if (i == value)
        {
            return true;
        }
        
    }
    return false;
}
} // namespace covid19
