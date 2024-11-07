#include "vector.hpp"
#include <cmath>

namespace sim{
Vector::Vector(float x, float y) : x_{x}, y_{y} {};

float Vector::get_x() const { return x_; };
float Vector::get_y() const { return y_; };

Vector Vector::operator+=(const Vector& vec2){
x_= x_ + vec2.x_;
y_= y_ + vec2.y_;
return *this;
}

Vector Vector::operator+(const Vector& v2) const {
    return Vector(x_ + v2.get_x(), y_ + v2.get_y());
};

float Vector::norm_vector() const{
    return std::sqrt((x_ * x_) + (y_ * y_));
}

}
