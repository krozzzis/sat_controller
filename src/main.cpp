#include <climits>
#include <iostream>
#include <cmath>

#define _USE_MATH_DEFINES // for C++

#define M_PI 3.14159265358979323846

double normalize_angle(double angle) {
    angle = fmod(angle, 360);
    if (angle < 0)
      angle += 360;
    return angle;
  }

  double angle_diff(double angle1, double angle2) {
    double a = fabs(angle1 - angle2);
    double b = fabs(360 - angle1 - angle2);
    if (a >= b)
      return b;
    else
      return a;
  }

  

class Vec2 {
public:
  double x, y;

  Vec2() {
    x = 0.0;
    y = 0.0;
  }

  Vec2(double _x, double _y) {
    x = _x;
    y = _y;
  }

  double length() {
    return sqrt(x*x + y*y);
  }

  Vec2 normalized() {
    return *this / length();
  }

  static double cross(Vec2 a, Vec2 b) {
    return a.x * b.y - b.x * a.y;
  }

  double dot(Vec2 a, Vec2 b) {
    return a.x * b.x + a.y * b.y;
  }

  friend std::istream &operator>>(std::istream &cin, Vec2 &vec) {
    cin >> vec.x >> vec.y;
    return cin;
  }

  friend std::ostream &operator<<(std::ostream &out, const Vec2 &vec) {
    out << vec.x << std::endl;
    out << vec.y;
    return out;
  }

  Vec2 operator/(const double &rhs) {
    return Vec2(x / rhs, y / rhs);
  }

  Vec2 operator*(const double &rhs) {
    return Vec2(x * rhs, y * rhs);
  }
};

bool is_between(Vec2 a, Vec2 b, Vec2 c) {
    double ba = Vec2::cross(b, a);
    double bc = Vec2::cross(b, c);
    return std::signbit(ba) != std::signbit(bc) && angle_diff(atan2(c.y, c.x), atan2(b.y, b.x)) < 1;
  }

/// Параметры, передаваемые в программу через стандартный поток во время старта
/// программы.
class StartupParams {
public:
  /// Большая полуось эллипса конечной орбиты в метрах.
  double semi_major_axis;
  /// Эксцентриситет эллипса конечной орбиты в метрах. Равен нулю, если конечная
  /// орбита круглая.
  double eccentricity;
  /// Угол наклона большой оси эллипса конечной орбиты относительно оси Ox в
  /// градусах от 0 до 90.
  double orbit_pitch;
  /// Запас имеющегося топлива в виде потенциального запаса изменения скорости.
  double fuel_capacity;

  friend std::istream &operator>>(std::istream &cin, StartupParams &params) {
    cin >> params.semi_major_axis >> params.eccentricity >> params.orbit_pitch >>
        params.fuel_capacity;
    return cin;
  }
};

/// Параметры, передаваемые в программу через стандартный поток на каждом шагу
/// моделирования.
class StepParams {
public:
  /// Время в секундах от начала моделирования.
  int time;
  /// Текущие координаты спутника.
  Vec2 position;
  /// Вектор скорости спутника.
  Vec2 velocity;
  /// Запас оставщегося топлива в виде потенциального запаса изменения скорости.
  double free_fuel;

  friend std::istream &operator>>(std::istream &cin, StepParams &params) {
    cin >> params.time >> params.position >> params.velocity >> params.free_fuel;
    return cin;
  }

  friend std::ostream &operator<<(std::ostream &out, const StepParams &result) {
      out << "Time: " << result.time << " Pos: " << result.position << " Vel: " << result.velocity << " Fuel: " << result.free_fuel;
      return out;
  }
};

/// Параметры действия спутника по результату 
/// моделирования шага контроллером.
class StepResult {
public:
  /// Требуемые изменения вектора скорости спутника.
  Vec2 velocity;
  /// Признак завершения работы.
  bool finish;

  StepResult(Vec2 v, bool f) {
    velocity = v;
    finish = f;
  }

  friend std::ostream &operator<<(std::ostream &out, const StepResult &result) {
    out << result.velocity << std::endl;
    out << (int)result.finish;
    return out;
  }
};



enum ControllerState {
  INITIAL_STATE = 0,
  WAIT_FOR_FIRST_IMPULSE = 1,
  WAIT_FOR_SECOND_IMPULSE = 2,
  WAIT_FOR_THIRD_IMPULSE = 3,
  END = 4,
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
  StartupParams st_params;

    double start_orbit_radius;

    double earth_mass;
    double earth_radius;
    double g;
    double first_impulse_speed;
    double second_impulse_speed;
    double third_impulse_speed;
    Vec2 previous_pos;
    Vec2 first_pos;
    Task task;
    ControllerState state;

public:
  Controller(StartupParams params) {
    st_params = params;
    start_orbit_radius = 0.0;
    earth_mass = 6e24;
    earth_radius = 6.357e6;
    g = 6.6743e-11;
    first_impulse_speed = 0.0;
    second_impulse_speed = 0.0;
    third_impulse_speed = 0.0;
    previous_pos = Vec2();
    first_pos = Vec2();

    if (params.eccentricity > 0.001) {
      if ((1 - st_params.eccentricity) * st_params.semi_major_axis == start_orbit_radius) {
        task = TASK_ZERO;
      } else {
        task = TASK_TWO;
      }
    } else {
      task = TASK_ONE;
    }

    state = ControllerState::INITIAL_STATE;
  }

  /// Рассчитывает следующее действие спутника на основе переданных данных о
  /// текущем состоянии. Вызывается на каждом шагу.
  StepResult calculate_step(StepParams params) {
    Vec2 result_impulse;
    bool complete = false;
    double new_radius1 = 0, new_radius2 = 0, radius_ratio1 = 0, radius_ratio2 = 0, initial_orbit_speed = 0, new_orbit_speed = 0, current_angle = 0, required_angle = 0;
    double angle1 = 0, angle2 = 0, angle3 = 0, angle4 = 0;
    current_angle = normalize_angle(atan2(params.position.y, params.position.x) * 180 / M_PI);

    angle1 = normalize_angle(st_params.orbit_pitch + 180);
    angle2 = normalize_angle(st_params.orbit_pitch);
    angle3 = normalize_angle(st_params.orbit_pitch + 180);

    switch (state) {
      case INITIAL_STATE:
        start_orbit_radius = params.position.length();
        previous_pos = params.position;
        first_pos = params.position;

        if (task == TASK_TWO) {
          new_radius1 = st_params.semi_major_axis - st_params.eccentricity * st_params.semi_major_axis;
          new_radius2 = st_params.eccentricity * st_params.semi_major_axis + st_params.semi_major_axis;
        } else if (task == TASK_ZERO || task ==  TASK_ONE) {
          new_radius1 = st_params.eccentricity * st_params.semi_major_axis + st_params.semi_major_axis;
          new_radius2 = 0;
        }
        radius_ratio1 = new_radius1 / start_orbit_radius;
        radius_ratio2 = new_radius2 / new_radius1;
        initial_orbit_speed = sqrt(g * earth_mass * (2 / (start_orbit_radius)- 1 / start_orbit_radius));
        new_orbit_speed = sqrt(g * earth_mass * (2 / (new_radius1)- 1 / new_radius1));

        // Рассчет Гомановской траектории https://en.wikipedia.org/wiki/Hohmann_transfer_orbit
        first_impulse_speed = initial_orbit_speed * (sqrt(2 * radius_ratio1 / (radius_ratio1 + 1)) - 1);
        second_impulse_speed = initial_orbit_speed * (1 / sqrt(radius_ratio1)) * (1 - sqrt(2 / (radius_ratio1 + 1)));
        third_impulse_speed = new_orbit_speed * (sqrt(2 * radius_ratio2 / (radius_ratio2 + 1)) - 1);

        state = WAIT_FOR_FIRST_IMPULSE;

      case WAIT_FOR_FIRST_IMPULSE:
        required_angle = angle1;
        // Определение положения в перицентре
        if (is_between(previous_pos, Vec2(cos(required_angle / 180 * M_PI), sin(required_angle / 180 * M_PI)), params.position)) {
          result_impulse = params.velocity.normalized() * first_impulse_speed;

          if (task == TASK_ZERO)
            complete = true;

          state = WAIT_FOR_SECOND_IMPULSE;
        }
        break;

      case WAIT_FOR_SECOND_IMPULSE: {
        required_angle = angle2;

        if (task == TASK_ZERO || task == TASK_TWO)
          required_angle = normalize_angle(st_params.orbit_pitch);
        else if (task == TASK_ONE)
          required_angle = normalize_angle(atan2(first_pos.y, first_pos.x) * 180 / M_PI + 180);


        if (is_between(previous_pos, Vec2(cos(required_angle / 180 * M_PI), sin(required_angle / 180 * M_PI)), params.position)) {
          result_impulse = params.velocity.normalized() * second_impulse_speed;

          if (task == TASK_ONE)
            complete = true;

          state = WAIT_FOR_THIRD_IMPULSE;
        }
      } break;

      case WAIT_FOR_THIRD_IMPULSE:
        required_angle = angle3;

        if (is_between(previous_pos, Vec2(cos(required_angle / 180 * M_PI), sin(required_angle / 180 * M_PI)), params.position)) {
          result_impulse = params.velocity.normalized() * third_impulse_speed;

          if (task == TASK_TWO)
            complete = true;

          state = END;
        }
        break;
    }

    previous_pos = params.position;
    return StepResult(result_impulse, complete);
  }
};




const int MAX_ITERATIONS = INT_MAX;

int main(int argc, char **argv) {
  StartupParams params;
  std::cin >> params;

  Controller controller(params);
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
