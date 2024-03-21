#include "Controller.h"
#include "Params.h"

#include <climits>
#include <iostream>

const int MAX_ITERATIONS = INT_MAX;

int main(int argc, char **argv) {
  StartupParams params;
  std::cin >> params;

  Controller controller;
  for (int i = 0; i < MAX_ITERATIONS; i++) {
    StepParams params;
    std::cin >> params;

    StepResult result = controller.calculate_step(params);
    std::cout << result << std::endl;

    if (result.finish == true) {
      break;
    }
  }

  return 0;
}
