#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

double Plane::norm(Vector3D v) {
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Vector3D Plane::unit(Vector3D v) {
  return v / norm(v);
}

void Plane::collide(PointMass &pm) {
  if ((dot(pm.position - point, normal) > 0))
    return;

  double intersect_t = dot(point - pm.last_position, normal) / dot(unit(pm.position - pm.last_position), normal);
  
  if (intersect_t > 0) {
    intersect_t -= SURFACE_OFFSET;
  } else {
    intersect_t += SURFACE_OFFSET;
  }
  intersect_t *= (1 - friction);
  pm.position = pm.last_position + (unit(pm.position - pm.last_position) * intersect_t);
}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 2 * (sCross + sParallel);
  positions.col(1) << sPoint + 2 * (sCross - sParallel);
  positions.col(2) << sPoint + 2 * (-sCross + sParallel);
  positions.col(3) << sPoint + 2 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
