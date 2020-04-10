#include <iostream>
#include <string>

namespace covid19
{
void run()
{
  const char* folder_spl = "./";
  std::cout << "Hello, here you can choose what to run:" << std::endl;
  std::cout << "[vrp|cvrp]" << std::endl;
  char program[10];
  std::cin >> program;

  char pro_r[20];
  strcpy(pro_r, folder_spl);
  strcat(pro_r, program);
  std::cout << pro_r;
  std::system(pro_r);
}
} // namespace covid19

int main()
{
  covid19::run();
  return EXIT_SUCCESS;
}