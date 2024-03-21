#pragma once

#include "Params.h"

/// Основной класс контроллера спутника.
class Controller {
public:
  Controller();

  /// Рассчитывает следующее действие спутника на основе переданных данных о
  /// текущем состоянии. Вызывается на каждом шагу.
  StepResult calculate_step(StepParams params);
};
