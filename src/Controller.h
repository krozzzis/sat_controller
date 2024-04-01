#pragma once

#include "Params.h"

enum ControllerState {
  INITIAL_STATE = 0,
  WAIT_FOR_FIRST_IMPULSE = 1,
  WAIT_FOR_SECOND_IMPULSE = 2,
  END = 3,
};

enum Task {
    TASK_ZERO = 0,
    TASK_ONE = 1,
    TASK_TWO = 2,
    TASK_THREE = 3,
};

/// Основной класс контроллера спутника.
class Controller {
private:
  // TODO: заменить на нормальные параметры
  StartupParams st_params;

  double start_orbit_radius;

  double earth_mass;
  double earth_radius;
  double g;
  double first_impulse_speed;
  double second_impulse_speed;
  Vec2 previous_pos;
  Vec2 first_pos;
  Task task;
  ControllerState state;

  double normalize_angle(double angle);
  double angle_diff(double angle1, double angle2);
  bool is_between(Vec2 a, Vec2 b, Vec2 c);

public:
  Controller(StartupParams params);

  /// Рассчитывает следующее действие спутника на основе переданных данных о
  /// текущем состоянии. Вызывается на каждом шагу.
  StepResult calculate_step(StepParams params);
};
