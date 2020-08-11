#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cube.h"
#include "edgeSpring.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

// TODO do we need thickness
Cube::Cube(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->depth = width;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->num_depth_points = num_width_points;
  this->thickness = thickness;

  buildCubeMesh();
  setFractureThreshold();
}

Cube::Cube(double width, double height, int num_width_points,
             int num_height_points, float thickness, float depth, int num_depth_points) {
  this->width = width;
  this->height = height;
  this->depth = depth;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->num_depth_points = num_depth_points;
  this->thickness = thickness;

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

// Mesh logic

/***
 * Laundry list
 * 
 * populate point_masses list
 * populate springs list
 * populate triangles list
 * populate cubes list
 * 
 * add pointer from edge to cube
 * populate Cube lists
 *    list of triangles
 *    list of edges
 * 
 * connect cubes
 */

void singleCubePoints(double width, double height, double depth, vector<Vector3D> *points) {
  points->emplace_back(0, 0, 0);
  points->emplace_back(width, 0, 0);
  points->emplace_back(width, height, 0);
  points->emplace_back(width, height, depth);
  points->emplace_back(0, height, 0);
  points->emplace_back(0, height, depth);
  points->emplace_back(0, 0, depth);
  points->emplace_back(width, 0, depth);
}

void Cube::buildCubeMesh() {
  CubeMesh *cubeMesh = new CubeMesh();
  vector<Triangle *> triangles;
  vector<SingleCube *> cubes;

  // build one cube
  SingleCube cube = new SingleCube();

  double cube_width = width / num_width_points;
  double cube_height = height / num_height_points;
  double cube_depth = depth / num_depth_points;
  // create point masses
  vector<Vector3D> cube_points = singleCubePoints(cube_width, cube_height, cube_depth, cube_points);
  for (Vector3D p: cube_points) {
    // TODO add pinned logic
    // TODO is this appropriate to create point mass
    PointMass p = new PointMass(p, false);
    point_masses.emplace_back(p);

  }



  cubeMesh->triangles = triangles;
  cubeMesh->cubes = cubes;
  this->cubeMesh = cubeMesh;
}

// Simulation logic
void Cube::simulate(double frames_per_sec, double simulation_steps, CubeParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {

  double mass = width * height * cp->density / num_width_points / num_height_points;
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
  }
  

  // Handle self-collisions.
  build_spatial_map();
  for (int i = 0; i < point_masses.size(); i++) {
    self_collide(point_masses[i], simulation_steps);
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
// TODO all of this @joey
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

  // Split the triangle mesh to account for fracture
  Halfedge *h1 = s->pm_a->halfedge;
  Halfedge *h2 = s->pm_b->halfedge;

  return;
}
