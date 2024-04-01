#include "Vec.h"
#include <istream>
#include <ostream>
#include <cmath>

Vec2::Vec2() {
  x = 0.0;
  y = 0.0;
}

Vec2::Vec2(double _x, double _y) {
  x = _x;
  y = _y;
}

double Vec2::length() {
  return sqrt(x*x + y*y);
}

Vec2 Vec2::normalized() {
  return *this / length();
}

double Vec2::cross(Vec2 a, Vec2 b) {
  return a.x * b.y - b.x * a.y;
}

double Vec2::dot(Vec2 a, Vec2 b) {
  return a.x * b.x + a.y * b.y;
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

Vec2 Vec2::operator/(const double &rhs) {
  return Vec2(x / rhs, y / rhs);
}

Vec2 Vec2::operator*(const double &rhs) {
  return Vec2(x * rhs, y * rhs);
}
