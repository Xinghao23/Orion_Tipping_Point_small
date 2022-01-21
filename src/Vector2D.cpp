#include "main.h"
#include "Vector2D.hpp"

using namespace std;

Vector2D Vector2D::getHeadingBased(double heading)  {
  Vector2D temp;
  temp.y = (this->y * (cos(heading))) + ( this->x * (sin(heading)));
  temp.x = (this->x * (cos(heading))) + (-this->y * (sin(heading)));
  return temp;
}

double Vector2D::getLength() {
  return sqrt(pow(x, 2) + pow(y, 2));
}

double Vector2D::getAngle() {
  double l = this->getLength();

  double theta = 0;

  if (l != 0) {
    if (y > 0 && x > 0 ) {
      theta = asin(x / l);
    }
    else if (y < 0 && x > 0) {
      theta = 3.14159 - asin(x / l);
    }
    else if (y > 0 && x < 0) {
      theta = asin(x / l);
    }
    else if (y < 0 && x < 0) {
      theta = -3.14159 - asin(x / l);
    }
    else if (y == 0 && x > 0) {
      theta = 3.14159 / 2;
    }
    else if (y == 0 && x < 0) {
      theta = -3.14159 / 2;
    }
    else if (y < 0 && x == 0) {
      theta = 3.14159;
    }
    else if (y > 0 && x == 0) {
      theta = 0;
    }
    else {
      theta = 0;
    }

    return theta;
  }
  else {
    return 0;
  }
}

Vector2D Vector2D::operator+ (const Vector2D& param) {
  Vector2D temp;
  temp.x = x + param.x;
  temp.y = y + param.y;
  return temp;
}

Vector2D Vector2D::operator- (const Vector2D& param) {
  Vector2D temp;
  temp.x = x - param.x;
  temp.y = y - param.y;
  return temp;
}

Vector2D Vector2D::operator* (const double& param) {
  Vector2D temp;
  temp.x = x * param;
  temp.y = y * param;
  return temp;
}

double Vector2D::operator* (const Vector2D& param) {
  double temp = 0;
  temp = (x * param.x) + (y * param.y);
  return temp;
}

double Vector2D::getAngleBetween(Vector2D b) {
  double dot_product = *(this) * b;
  double theta = 0;

  theta = dot_product / (this->getLength() * b.getLength());
  theta = acos(theta);

  return theta;
}
