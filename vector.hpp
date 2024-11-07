#ifndef VECTOR_HPP
#define VECTOR_HPP

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
  Vector operator+(const Vector& v2) const;

};

#endif