#pragma once

#include "Vec.h"

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

  friend std::istream &operator>>(std::istream &cin, StartupParams &params);
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

  friend std::istream &operator>>(std::istream &cin, StepParams &params);
  friend std::ostream &operator<<(std::ostream &out, const StepParams &result);
};

/// Параметры действия спутника по результату моделирования шага контроллером.
class StepResult {
public:
  /// Требуемые изменения вектора скорости спутника.
  Vec2 velocity;
  /// Признак завершения работы.
  bool finish;

  StepResult(Vec2 v, bool f);
  friend std::ostream &operator<<(std::ostream &out, const StepResult &result);
};
