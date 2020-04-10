#include <iostream>
#include <string>

namespace covid19
{
void run()
{
  std::cout << "Hello, here you can choose what to run:" << std::endl;
  std::cout << "[vrp|cvrp]" << std::endl;
  char program[20];
  std::cin >> program;

  char pro_r[20];
  strcpy(pro_r, "./");
  strcat(pro_r, program);
  std::system(pro_r);
}
} // namespace covid19

int main()
{
  covid19::run();
  return EXIT_SUCCESS;
}