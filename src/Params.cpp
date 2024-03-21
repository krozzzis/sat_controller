#include "Params.h"

std::istream &operator>>(std::istream &cin, StartupParams &params) {
  cin >> params.semi_major_axis >> params.eccentricity >> params.orbit_pitch >>
      params.fuel_capacity;
  return cin;
}

std::istream &operator>>(std::istream &cin, StepParams &params) {
  cin >> params.time >> params.position >> params.velocity >> params.free_fuel;
  return cin;
}

StepResult::StepResult(Vec2 v, bool f) {
  velocity = v;
  finish = f;
}

std::ostream &operator<<(std::ostream &out, const StepResult &result) {
  out << result.velocity << std::endl;
  out << (int)result.finish;
  return out;
}
