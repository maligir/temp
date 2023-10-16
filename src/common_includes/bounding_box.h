#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <string>

struct BoundingBox {
  float probability;
  float xmin;
  float ymin;
  float xmax;
  float ymax;
  unsigned id;
  std::string Class;

  BoundingBox() : probability(0),
                  xmin(0),
                  ymin(0),
                  xmax(0),
                  ymax(0),
                  id(0),
                  Class("") {}

  BoundingBox(float prob,
              float xmin,
              float ymin,
              float xmax,
              float ymax,
              unsigned id,
              std::string Class) :
    probability(prob),
    xmin(xmin),
    ymin(ymin),
    xmax(xmax),
    ymax(ymax),
    id(id),
    Class(Class) {}

  BoundingBox(const BoundingBox& other) :
    probability(other.probability),
      xmin(other.xmin),
      ymin(other.ymin),
      xmax(other.xmax),
      ymax(other.ymax),
      id(other.id),
      Class(other.Class) {}
  
  void operator=(const BoundingBox& other) {
    probability = other.probability;
    xmin = other.xmin;
    ymin = other.ymin;
    xmax = other.xmax;
    ymax = other.ymax;
    id = other.id;
    Class = other.Class;
  }

  float getArea() const {
    return (xmax - xmin) * (ymax - ymin);
  }
};

#endif