#include <iostream>
#include <cstdlib>
#include <unistd.h>

namespace covid19
{
void run()
{
  
  char current_folder[100];
  getcwd(current_folder, sizeof(current_folder));
  strcat(current_folder, "/");

  std::cout << "Hello, here you can choose what to run:" << std::endl;
  std::cout << "[vrp|cvrp]" << std::endl;
  char program[10];
  std::cin >> program;

  char pro_r[110];
  strcpy(pro_r, current_folder);
  strcat(pro_r, program);
  system(pro_r);
}
} // namespace covid19

int main()
{
  covid19::run();
  return EXIT_SUCCESS;
}