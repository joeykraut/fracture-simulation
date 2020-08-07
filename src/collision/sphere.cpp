#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

double norm(Vector3D v) {
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Vector3D unit(Vector3D v) {
  return v / norm(v);
}

void Sphere::collide(PointMass &pm) {
  Vector3D origin_to_position = pm.position - origin;
  if (!(norm(origin_to_position) <= radius))
    return;

  Vector3D correct_point = (unit(origin_to_position) * radius) + origin;
  double length_with_friction = norm(correct_point - pm.last_position) * friction;
  Vector3D correction_vector = unit(correct_point - pm.last_position) * length_with_friction;
  pm.position = correction_vector + pm.last_position;
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}

void Sphere::set_pinned(bool pinned) {
    this->pinned = pinned;
}

void Sphere::simulate(double delta_t, Vector3D gravity_vec) {
    // Add forces
    forces = Vector3D(0.0, 0.0, 0.0);
    // Add gravity
    forces += gravity_vec * GRAVITY_MULTIPLIER * mass;

    // Update position
    update_moments(delta_t);
}

void Sphere::update_moments(double delta_t) {
    // Add the forces to the sphere
    if (pinned) {
        return;
    }

    velocity += pow(delta_t, 2) * forces / mass;
    origin = origin + delta_t * velocity;
}
