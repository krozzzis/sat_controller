#include "Vec.h"
#include <istream>
#include <ostream>

Vec2::Vec2() {
  x = 0.0;
  y = 0.0;
}

Vec2::Vec2(double _x, double _y) {
  x = _x;
  y = _y;
}

std::istream &operator>>(std::istream &cin, Vec2 &vec) {
  cin >> vec.x >> vec.y;
  return cin;
}

std::ostream &operator<<(std::ostream &out, const Vec2 &vec) {
  out << vec.x << std::endl;
  out << vec.y;
  return out;
}
