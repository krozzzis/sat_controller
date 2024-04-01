#pragma once

#include <istream>

class Vec2 {
public:
  double x, y;

  Vec2();
  Vec2(double _x, double _y);

  double length();
  Vec2 normalized();
  static double cross(Vec2 a, Vec2 b);
  static double dot(Vec2 a, Vec2 b);

  friend std::istream &operator>>(std::istream &cin, Vec2 &vec);
  friend std::ostream &operator<<(std::ostream &out, const Vec2 &vec);

  Vec2 operator / (const double &rhs);
  Vec2 operator * (const double &rhs);
};
