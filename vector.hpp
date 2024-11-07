#ifndef VECTOR_HPP
#define VECTOR_HPP

namespace sim{

class Vector {
 private:
  float x_;
  float y_;

 public:
  Vector(float x, float y);

  // metodi get per accedere alle coordinate del vettore
  float get_x() const;
  float get_y() const;

  // Somma vettoriale
  Vector operator+(const Vector&) const;
  Vector operator+=(const Vector&);

  Vector operator-(const Vector&) const;

  float distance(const Vector&) const;

  float norm_vector() const;

  bool operator!=(const Vector&) const;

  bool operator==(const Vector&) const;

  Vector operator*(float scalar) const;

};
}

#endif