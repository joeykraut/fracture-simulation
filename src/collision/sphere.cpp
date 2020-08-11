#include <nanogui/nanogui.h>

#include "../mesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"
#include "plane.h"

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

  // The old position
  Vector3D old_position = pm.position;

  Vector3D correct_point = (unit(origin_to_position) * radius) + origin;
  double length_with_friction = norm(correct_point - pm.last_position) * friction;
  Vector3D correction_vector = unit(correct_point - pm.last_position) * length_with_friction;
  pm.position = correction_vector + pm.last_position;

  // Add the reactionary force to the sphere which has:
  //    magnitude: The spring collision resistance constant time the displacement caused by sphere
  //    direction: The new position towards the sphere origin
  double force_mag = SPRING_COLLISION_RESISTANCE * (old_position - pm.position).norm();
  Vector3D direction = this->origin - pm.position;

  this->forces += force_mag * direction;
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}

// ============================== //
// ====== Setters/Getters ======= //
// ============================== //

void Sphere::set_pinned(bool pinned) {
    this->pinned = pinned;
}

void Sphere::zero_forces() {
    this->forces = Vector3D(0);
}

void Sphere::set_initial_position(Vector3D position) {
    initial_position = position;
}

void Sphere::reset() {
    origin = initial_position;
    velocity = Vector3D(0);
}

// ============================= //
// ======== Simulation ========= //
// ============================= //

void Sphere::simulate(double delta_t, Vector3D gravity_vec, vector<CollisionObject *> *objects) {
    // Add forces
    // Add gravity
    forces += gravity_vec * mass;

    // Multiply the forces by an amplifier constant
    forces *= FORCE_MULTIPLIER;

    // Update position and velocity
    update_moments(delta_t);
    for (auto object : *objects) {
        if (object == this) {
            continue;
        }

        // Otherwise it's a plane, collide
    }
}

void Sphere::update_moments(double delta_t) {
    // Add the forces to the sphere
    if (pinned) {
        return;
    }

    // Damped velocity added with acceleration
    velocity = DAMPING_COEFF * velocity + pow(delta_t, 2) * forces / mass;
    origin = origin + delta_t * velocity;
}
