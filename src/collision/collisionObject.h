#ifndef COLLISIONOBJECT
#define COLLISIONOBJECT

#include <nanogui/nanogui.h>

#include "../clothMesh.h"

using namespace CGL;
using namespace std;
using namespace nanogui;

class CollisionObject {
public:
  virtual void render(GLShader &shader) = 0;
  virtual void collide(PointMass &pm) = 0;
  virtual void simulate(double, Vector3D) = 0;
  virtual void zero_forces() = 0;

  Vector3D forces;

private:
  double friction;
};

#endif /* COLLISIONOBJECT */
