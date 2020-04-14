#ifndef COVID19_REQUIREMENTS_H_
#define COVID19_REQUIREMENTS_H_

#include <iostream>
#include <vector>

namespace covid19
{
/**
 * Check the route to see if it matches the requirements constraint.
 * @param nodes_permutation the route permutation
 * @param nodes_requirements requirements of each node
 * @param capacity each route's requirements should not exceed capacity.
 * @return if the route is valid, return true, else false
 */
bool CheckRequirements(const std::vector<int>& nodes_permutation, const std::vector<int64_t>& nodes_requirements, int64_t capacity);

int64_t CalcRequirements(const std::vector<int>& nodes_permutation, const std::vector<int64_t>& nodes_requirements);
}

#endif // COVID19_REQUIREMENTS_H_