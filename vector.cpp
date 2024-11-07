#include "vector.hpp"

Vector::Vector(float x, float y) : x_{x}, y_{y} {};

float Vector::get_x() const { return x_; };
float Vector::get_y() const { return y_; };

Vector Vector::operator+(const Vector& v2) const {
    return Vector(x_ + v2.get_x(), y_ + v2.get_y());
};
