#pragma once

#include <istream>

class Vec2 {
public:
  double x, y;

  Vec2();

  Vec2(double _x, double _y);

  friend std::istream &operator>>(std::istream &cin, Vec2 &vec);
  friend std::ostream &operator<<(std::ostream &out, const Vec2 &vec);
};
