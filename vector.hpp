#ifndef VECTOR_HPP
#define VECTOR_HPP

namespace sim {

class Vector {
 private:
  float x_;
  float y_;

 public:
  Vector();
  Vector(float x, float y);
  float get_x() const;
  float get_y() const;
  void set_x(float new_x);
  void set_y(float new_y);
  Vector operator+(const Vector&) const;
  Vector& operator+=(const Vector&);
  Vector operator-(const Vector&) const;
  float distance(const Vector&) const;
  float norm_vector() const;
  bool operator==(const Vector&) const;
  Vector operator*(float scalar) const;
  float product(const Vector&) const;
};

}  // namespace sim

#endif