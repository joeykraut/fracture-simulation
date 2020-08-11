#ifndef SPRING_H
#define SPRING_H

#include <vector>

#include "CGL/CGL.h"
#include "pointMass.h"

using namespace std;

namespace CGL {

enum e_spring_type { STRUCTURAL = 0, SHEARING = 1, BENDING = 2 };

struct EdgeSpring {
  EdgeSpring(PointMass *a, PointMass *b, e_spring_type spring_type)
      : pm_a(a), pm_b(b), spring_type(spring_type) {
    rest_length = (pm_a->position - pm_b->position).norm();
  }

  EdgeSpring(PointMass *a, PointMass *b, double rest_length)
      : pm_a(a), pm_b(b), rest_length(rest_length) {
      }

  EdgeSpring() {}
  
  double rest_length;

  e_spring_type spring_type = STRUCTURAL;

  PointMass *pm_a;
  PointMass *pm_b;
  SingleCube *single_cube;
  
  // percentage of rest_length that a spring cannot surpass without breaking
  double fracture_thresh = 0;
  // when a spring is fractured, this is set to True instead of deleting it from the cloth
  bool fractured = false;

}; // struct EdgeSpring
}
#endif /* SPRING_H */
