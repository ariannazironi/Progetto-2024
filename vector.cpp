#include "vector.hpp"

#include <cassert>
#include <cmath>

namespace sim {

Vector::Vector() {};
Vector::Vector(float x, float y) : x_{x}, y_{y} {};

float Vector::get_x() const { return x_; };
float Vector::get_y() const { return y_; };

Vector Vector::operator+(const Vector& vec2) const{
    return Vector(x_ + vec2.x_ , y_ + vec2.y_);
}

Vector Vector::operator+=(const Vector& vec2) {
x_= x_ + vec2.x_;
y_= y_ + vec2.y_;
return *this;
}

Vector Vector::operator-(const Vector& vec2) const{
    Vector neg_vector= {-vec2.x_ , -vec2.y_};
    return *this + neg_vector;
}

float Vector::distance(const Vector& vec2) const{
    const Vector difference= *this - vec2;
    const float distance= std::sqrt(std::pow(difference.x_, 2) + std::pow(difference.y_, 2));
    assert(distance >= 0.);
    return distance;
}

float Vector::norm_vector() const{
    float const norm= std::sqrt((x_ * x_) + (y_ * y_));
    assert(norm >= 0.);
    return norm;
}

bool Vector::operator!=(const Vector& vec2) const{
    return (x_ != vec2.x_ || y_ != vec2.y_);
}

bool Vector::operator==(const Vector& vec2) const{
    return (x_ == vec2.x_ && y_ == vec2.y_);
}

Vector Vector::operator*(float scalar) const{
    return Vector(scalar * x_ , scalar *y_);
}

float Vector::product(const Vector& vec2) const{
    const Vector result= {x_ * vec2.x_ , y_ * vec2.y_};
    const float product= result.x_ + result.y_;
    return product;
}

}
