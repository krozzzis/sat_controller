#include "Controller.h"
#include "Params.h"

Controller::Controller() {}

StepResult Controller::calculate_step(StepParams params) {
  return StepResult(Vec2(), true);
}
