#pragma once

#include "Params.h"

class Controller {
public:
  Controller();
  StepResult calculate_step(StepParams params);
};
