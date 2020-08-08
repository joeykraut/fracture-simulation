#ifndef COLLISIONOBJECT_PLANE_H
#define COLLISIONOBJECT_PLANE_H

#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "collisionObject.h"

using namespace nanogui;
using namespace CGL;
using namespace std;

struct Plane : public CollisionObject {
public:
  Plane(const Vector3D &point, const Vector3D &normal, double friction)
      : point(point), normal(normal.unit()), friction(friction) {}

  void render(GLShader &shader);
  void collide(PointMass &pm);
  void simulate(double, Vector3D, vector<CollisionObject *> *);
  void zero_forces() {
      return;
  }
  void reset() {
      return;
  }

  Vector3D unit(Vector3D v);
  double norm(Vector3D v);
  
  Vector3D point;
  Vector3D normal;
  Vector3D forces;

  double friction;
};

#endif /* COLLISIONOBJECT_PLANE_H */
