#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cube.h"
#include "mesh.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

// ========================
// ===== Constructors =====
// ========================
Cube::Cube(Vector3D center, double width, double height, double depth) {
  this->width = width;
  this->height = height;
  this->depth = depth;
  this->center = center;

  buildCubeMesh();
  setFractureThreshold();
}

Cube::~Cube() {
  point_masses.clear();
  springs.clear();

  if (cubeMesh) {
    delete cubeMesh;
  }
}

// =========================
// ===== Mesh Creation =====
// =========================
void Cube::createCube(Vector3D left, vector<SingleCube *> *cubies) {
    // Create all the point masses
    PointMass *vertex_a = new PointMass(left, false);
    PointMass *vertex_b = new PointMass(Vector3D(left.x, left.y, left.z + CUBE_DIM), false);
    PointMass *vertex_c = new PointMass(Vector3D(left.x, left.y + CUBE_DIM, left.z), false);
    PointMass *vertex_d = new PointMass(Vector3D(left.x, left.y + CUBE_DIM, left.z + CUBE_DIM), false);
    PointMass *vertex_e = new PointMass(Vector3D(left.x + CUBE_DIM, left.y, left.z), false);
    PointMass *vertex_f = new PointMass(Vector3D(left.x + CUBE_DIM, left.y, left.z + CUBE_DIM), false);
    PointMass *vertex_g = new PointMass(Vector3D(left.x + CUBE_DIM, left.y + CUBE_DIM, left.z), false);
    PointMass *vertex_h = new PointMass(Vector3D(left.x + CUBE_DIM, left.y + CUBE_DIM, left.z + CUBE_DIM), false);

    point_masses.push_back(*vertex_a);    
    point_masses.push_back(*vertex_b);
    point_masses.push_back(*vertex_c); 
    point_masses.push_back(*vertex_d);
    point_masses.push_back(*vertex_e);
    point_masses.push_back(*vertex_f);    
    point_masses.push_back(*vertex_g);
    point_masses.push_back(*vertex_h);   

    // Dummy uv coords
    Vector3D uv(0, 0, 0);

    // Add the structural constraints
    vector<EdgeSpring> springs;
    springs.emplace_back(vertex_a, vertex_b, STRUCTURAL);
    springs.emplace_back(vertex_b, vertex_d, STRUCTURAL);
    springs.emplace_back(vertex_d, vertex_c, STRUCTURAL);
    springs.emplace_back(vertex_c, vertex_a, STRUCTURAL);

    springs.emplace_back(vertex_a, vertex_e, STRUCTURAL);
    springs.emplace_back(vertex_b, vertex_f, STRUCTURAL);
    springs.emplace_back(vertex_c, vertex_g, STRUCTURAL);
    springs.emplace_back(vertex_d, vertex_h, STRUCTURAL);

    springs.emplace_back(vertex_e, vertex_f, STRUCTURAL);
    springs.emplace_back(vertex_f, vertex_h, STRUCTURAL);
    springs.emplace_back(vertex_h, vertex_g, STRUCTURAL);
    springs.emplace_back(vertex_g, vertex_e, STRUCTURAL);

    // Add the face constraints
    springs.emplace_back(vertex_a, vertex_g, BENDING);
    springs.emplace_back(vertex_a, vertex_d, BENDING);
    springs.emplace_back(vertex_b, vertex_h, BENDING);
    springs.emplace_back(vertex_h, vertex_e, BENDING);
    springs.emplace_back(vertex_a, vertex_f, BENDING);
    springs.emplace_back(vertex_c, vertex_h, BENDING);

    // Build the triangles
    vector<Triangle *> triangles;

    // Face: ABCD
    triangles.push_back(new Triangle(vertex_a, vertex_b, vertex_d, uv, uv, uv));
    triangles.push_back(new Triangle(vertex_a, vertex_c, vertex_d, uv, uv, uv));

    // Face: BDHF
    triangles.push_back(new Triangle(vertex_b, vertex_h, vertex_d, uv, uv, uv));
    triangles.push_back(new Triangle(vertex_b, vertex_f, vertex_h, uv, uv, uv));

    // Face: EFGH
    triangles.push_back(new Triangle(vertex_f, vertex_h, vertex_e, uv, uv, uv));
    triangles.push_back(new Triangle(vertex_g, vertex_h, vertex_e, uv, uv, uv));

    // Face: ACGE
    triangles.push_back(new Triangle(vertex_a, vertex_c, vertex_g, uv, uv, uv));
    triangles.push_back(new Triangle(vertex_a, vertex_e, vertex_g, uv, uv, uv));

    // Face: CDHG
    triangles.push_back(new Triangle(vertex_h, vertex_d, vertex_c, uv, uv, uv));
    triangles.push_back(new Triangle(vertex_g, vertex_c, vertex_h, uv, uv, uv));

    // Face: ABEF
    triangles.push_back(new Triangle(vertex_a, vertex_b, vertex_f, uv, uv, uv));
    triangles.push_back(new Triangle(vertex_a, vertex_e, vertex_f, uv, uv, uv));
    SingleCube *singleCube = new SingleCube(triangles, springs);
    cubies->push_back(singleCube);
}

void Cube::buildCubeMesh() { 
    // Build the cube mesh
    vector<SingleCube *> cubies;
    createCube(Vector3D(5, 0, 0), &cubies);

    this->cubeMesh = new CubeMesh(cubies);
    // Build add_cube_above and add_cube_right
}

// =========================
// ===== Render Logic =====
// =========================

void Cube::render(GLShader &shader) {
    MatrixXf positions;

    // Cube
    int positions_size = 0;
    for (auto cubie : cubeMesh->single_cubes) {
        for (auto spring : cubie->edges) {
          positions_size++;
        }
    }
    positions = MatrixXf(4, positions_size * 2);

    // Loop over and add all the point masses
    int si = 0;
    for (auto cubie : cubeMesh->single_cubes) {
        for (auto spring : cubie->edges) {
          Vector3D pa = spring.pm_a->position;
          Vector3D pb = spring.pm_b->position;

          positions.col(si) << pa.x, pa.y, pa.z, 1.0;
          positions.col(si + 1) << pb.x, pb.y, pb.z, 1.0;
          si += 2;
        }
    }
  shader.uploadAttrib("in_position", positions, false);

  shader.drawArray(GL_LINES, 0, positions_size * 2);
}


// ============================
// ===== Simulation Logic =====
// ============================

void Cube::simulate(double frames_per_sec, double simulation_steps, CubeParameters *cp,
                    vector<Vector3D> external_accelerations, vector<CollisionObject *> *collision_objects) {

  
  double mass = 100;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // Clear collision object forces
  for (int i = 0; i < collision_objects->size(); i++) {
      collision_objects->at(i)
      ->zero_forces();
  }

  // Add external forces
  Vector3D external_force = Vector3D(0);
  for (int i = 0; i < external_accelerations.size(); i++) {
    external_force += mass * external_accelerations[i];
  }

  for (int i = 0; i < point_masses.size(); i++) {
    // reset forces
    point_masses[i].forces = Vector3D(0);
    // forces calculated from external forces
    point_masses[i].forces += external_force;
  }

  // compute correction forces
  for (int i = 0; i < springs.size(); i++) {
    EdgeSpring *s = &springs[i];
    
    double force_mag;
    if (!s->fractured) {
      force_mag = cp->ks * ((s->pm_a->position - s->pm_b->position).norm() - s->rest_length);
    }
    // TODO maybe add lower ks for internal springs
    s->pm_b->forces += (s->pm_a->position - s->pm_b->position).unit() * force_mag;
    s->pm_a->forces += (s->pm_b->position - s->pm_a->position).unit() * force_mag;
  }
  
  // calculate new positions
  for (int i = 0; i < point_masses.size(); i++) {
    PointMass *p = &point_masses[i];

    if (p->pinned)
      continue;
    
    Vector3D accel = p->forces / mass;
    Vector3D new_position = p->position + ((1. - (cp->damping / 100.)) * (p->position - p->last_position)) + (accel * (delta_t * delta_t));
    p->last_position = p->position;
    p->position = new_position;
  
    cout << p->position << endl;
    cout << p->last_position << endl;

  }

  // Handle collisions with other primitives.
  for (int i = 0; i < point_masses.size(); i++) {
    for (int j = 0; j < collision_objects->size(); j++) {
      collision_objects->at(j)->collide(point_masses[i]);
    }
  }

  // check if springs have crossed their threshold
  for (int i = 0; i < springs.size(); i++) {
    EdgeSpring *s = &springs[i];
    if (s->fracture_thresh != 0 && (s->pm_a->position - s->pm_b->position).norm() > (s->rest_length * s->fracture_thresh)) {
      break_spring(s);
    }
  }


  // in length more than 10% per timestep [Provot 1995].
  for (int i = 0; i < springs.size(); i++) {
    EdgeSpring *s = &springs[i];
    if (s->fractured) {
      // ignore if spring is fractured
      continue;
    } else if ((s->pm_a->position - s->pm_b->position).norm() > (s->rest_length * 1.2)) {
      if (s->pm_a->pinned) {
        s->pm_b->position = ((s->pm_b->position - s->pm_a->position).unit() * (s->rest_length * 1.2)) + s->pm_a->position;
      } else if (s->pm_b->pinned) {
        s->pm_a->position = ((s->pm_a->position - s->pm_b->position).unit() * (s->rest_length * 1.2)) + s->pm_b->position;
      } else {
        Vector3D mid = ((s->pm_a->position - s->pm_b->position) / 2) + s->pm_b->position;
        s->pm_a->position = (s->pm_a->position - mid).unit() * ((s->rest_length * 1.2) / 2.) + mid;
        s->pm_b->position = (s->pm_b->position - mid).unit() * ((s->rest_length * 1.2) / 2.) + mid;
      }
    }
  }

}

void Cube::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  for (int i = 0; i < point_masses.size(); i++) {
    PointMass p = point_masses.at(i);
    float uid = hash_position(p.position);

    if (map.find(uid) == map.end()) {
      map.insert({uid, new vector<PointMass *>});
    }

    map[uid]->push_back(&point_masses.at(i));
  }

}

void Cube::self_collide(PointMass &pm, double simulation_steps) {
  float uid = hash_position(pm.position);
  vector<PointMass *>* possible_collisions = map.at(uid);

  Vector3D correction = Vector3D(0);
  int correction_count = 0;

  for (int i = 0; i < possible_collisions->size(); i++) {
    PointMass *p = possible_collisions->at(i);

    if (p->start_position == pm.start_position) {
      continue;
    }

    double length = (pm.position - p->position).norm();
    if (length < 2. * thickness) {
      Vector3D correction_point = ((pm.position - p->position).unit() * 2. * (thickness)) + p->position;
      correction += (correction_point - pm.position);
      correction_count++;
    }
  }

  // average and scale correction vector
  if (correction_count > 0) {
    correction /= (double) correction_count;
    correction /= simulation_steps;
    pm.position = pm.position + correction;
  }
}

float Cube::hash_position(Vector3D pos) {
  double w = 3. * width / (double) num_width_points;
  double h = 3. * height / (double) num_height_points;
  double t = max(w, h);

  double x = floor(pos[0] / w);
  double y = floor(pos[1] / h);
  double z = floor(pos[2] / t);

  float uid = (x * w * h) + (y * h) + z;
  return uid; 
}

void Cube::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }

  for (int i = 0; i < springs.size(); i++) {
    springs[i].fractured = false;
  }
}

// Fracture logic
double Cube::getRandomFractureThresh(double min, double max) {
  return min + (rand() / (RAND_MAX / (max - min)));
}

void Cube::setFractureThreshold() {
  double min = 1.2;
  double max = 1.8; 
  for (int i = 0; i < springs.size(); i++) {
    // Add random fracture threshold values to all springs
    EdgeSpring *s = &springs[i];
    s->fracture_thresh = getRandomFractureThresh(min, max);
  }

}

void Cube::break_spring(EdgeSpring *s) {
  s->fractured = true;
  
  // fracture all springs in cube
  for (auto edge: s->single_cube->edges) {
    edge.fractured = true;
  }

  // mark all triangles as fractured for the shader
  for (auto tri: s->single_cube->triangles) {
    tri->fractured = true;
  }

  return;
}

