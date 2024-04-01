#include "Controller.h"
#include <cmath>
#include <fstream>

#define NODEBUG

Controller::Controller(StartupParams params) {
	st_params = params;
	start_orbit_radius = 0.0;
	earth_mass = 6e24;
	earth_radius = 6.357e6;
	g = 6.6743e-11;
	first_impulse_speed = 0.0;
  second_impulse_speed = 0.0;
  previous_pos = Vec2();
  first_pos = Vec2();

  if (params.eccentricity < 0.001)
    task = TASK_ONE;
  else
    task = TASK_ZERO;

	state = ControllerState::INITIAL_STATE;

  #ifdef DEBUG
	// Очистка файла логов
	std::ofstream file;
	file.open("log.txt");
	file.close();
  #endif
}

StepResult Controller::calculate_step(StepParams params) {
  Vec2 result_impulse;
  bool complete = false;
  double new_radius = 0, radius_ratio = 0, initial_orbit_speed = 0, current_angle = 0, required_angle = 0;
  std::ofstream fi;

  current_angle = normalize_angle(atan2(params.position.y, params.position.x) * 180 / M_PI);

  switch (state) {
    case INITIAL_STATE:
      start_orbit_radius = params.position.length();
      previous_pos = params.position;
      first_pos = params.position;

      new_radius = st_params.eccentricity * st_params.semi_major_axis + st_params.semi_major_axis;
      radius_ratio = new_radius / start_orbit_radius;
      initial_orbit_speed = sqrt(g * earth_mass * (2 / (start_orbit_radius) - 1 / start_orbit_radius));

      // Рассчет Гомановской траектории https://en.wikipedia.org/wiki/Hohmann_transfer_orbit
      first_impulse_speed = initial_orbit_speed * (sqrt(2 * radius_ratio / (radius_ratio + 1)) - 1);
      second_impulse_speed = initial_orbit_speed * (1 / sqrt(radius_ratio)) * (1 - sqrt(2 / (radius_ratio + 1)));

      #ifdef DEBUG
      fi.open("log.txt", std::ios::app);
      fi << "Impulse 1: " << first_impulse_speed << std::endl;
      fi << "Impulse 2: " << second_impulse_speed << std::endl;
      fi.close();
      #endif

      state = WAIT_FOR_FIRST_IMPULSE;

    case WAIT_FOR_FIRST_IMPULSE:
      if (task == TASK_ZERO)
        required_angle = normalize_angle(st_params.orbit_pitch + 180);
      else if (task == TASK_ONE)
        required_angle = normalize_angle(atan2(first_pos.y, first_pos.x) * 180 / M_PI);

      #ifdef DEBUG
      fi.open("log.txt", std::ios::app);
      fi << "Time: " << params.time << " State: " << state << " Angle: " << current_angle << " PrevAngle: " << normalize_angle(atan2(previous_pos.y, previous_pos.x) * 180 / M_PI) << " ReqAngle: " << required_angle << " Diff: " << angle_diff(current_angle, required_angle) << std::endl;
      fi << required_angle << std::endl;
      fi.close();
      #endif

      // Определение положения в перицентре

      if (is_between(previous_pos, Vec2(cos(required_angle / 180 * M_PI), sin(required_angle / 180 * M_PI)), params.position)) {
        result_impulse = params.velocity.normalized() * first_impulse_speed;

        #ifdef DEBUG
        fi.open("log.txt", std::ios::app);
        fi << "Time: " << params.time << " State: " << state << " Made impulse: " << result_impulse.x << " " << result_impulse.y << std::endl;
        fi.close();
        #endif

        if (task == TASK_ZERO)
          complete = true;

        #ifdef DEBUG
        fi.open("log.txt", std::ios::app);
        fi << "Time: " << params.time << " State: " << state << " Set state " << WAIT_FOR_SECOND_IMPULSE << std::endl;
        fi.close();
        #endif

        state = WAIT_FOR_SECOND_IMPULSE;
      }
      break;

    case WAIT_FOR_SECOND_IMPULSE:
      if (task == TASK_ZERO)
        required_angle = normalize_angle(st_params.orbit_pitch);
      else if (task == TASK_ONE)
        required_angle = normalize_angle(atan2(first_pos.y, first_pos.x) * 180 / M_PI + 180);

      if (is_between(previous_pos, Vec2(cos(required_angle / 180 * M_PI), sin(required_angle / 180 * M_PI)), params.position)) {
        result_impulse = params.velocity.normalized() * second_impulse_speed;

        #ifdef DEBUG
        fi.open("log.txt", std::ios::app);
        fi << "Time: " << params.time << " State: " << state << " Made impulse: " << result_impulse.x << " " << result_impulse.y << std::endl;
        fi.close();
        #endif

        if (task == TASK_ONE)
          complete = true;

        #ifdef DEBUG
        fi.open("log.txt", std::ios::app);
        fi << "Time: " << params.time << " State: " << state << " Set state " << END << std::endl;
        fi.close();
        #endif

        state = END;
      }
      break;
  }

  previous_pos = params.position;
  return StepResult(result_impulse, complete);
}

double Controller::normalize_angle(double angle) {
  angle = fmod(angle, 360);
  if (angle < 0)
    angle += 360;
  return angle;
}

double Controller::angle_diff(double angle1, double angle2) {
  double a = fabs(angle1 - angle2);
  double b = fabs(360 - angle1 - angle2);
  if (a >= b)
    return b;
  else
    return a;
}

bool Controller::is_between(Vec2 a, Vec2 b, Vec2 c) {
  double ba = Vec2::cross(b, a);
  double bc = Vec2::cross(b, c);
  return std::signbit(ba) != std::signbit(bc) && angle_diff(atan2(c.y, c.x), atan2(b.y, b.x)) < 1;
}