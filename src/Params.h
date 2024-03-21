#pragma once

#include "Vec.h"

class StartupParams {
public:
  double semi_major_axis;
  double eccentricity;
  double orbit_pitch;
  double fuel_capacity;

  friend std::istream &operator>>(std::istream &cin, StartupParams &params);
};

class StepParams {
public:
  int time;
  Vec2 position;
  Vec2 velocity;
  double free_fuel;

  friend std::istream &operator>>(std::istream &cin, StepParams &params);
};

class StepResult {
public:
  Vec2 velocity;
  bool finish;

  StepResult(Vec2 v, bool f);
  friend std::ostream &operator<<(std::ostream &out, const StepResult &result);
};
