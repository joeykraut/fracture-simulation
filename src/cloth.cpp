#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "edgeSpring.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::getPointMassPos(int i, int j, Vector3D* out) {
  double x = j * (width / num_width_points);
  double y;
  double z;
  if (orientation) {
    // is vertical
    y = i * (height / num_height_points);
    z = - (1. / 1000.) + (rand() / ( RAND_MAX / (2. / 1000.)));
  } else {
    y = 1.;
    z = i * (height / num_height_points);
  }
  out->x = x;
  out->y = y;
  out->z = z;
}

void Cloth::buildGrid() {
  // Build all point masses
  for (int i = 0; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {
      double x = j * (width / num_width_points);
      double y;
      double z;
      if (orientation) {
        // is vertical
        y = i * (height / num_height_points);
        z = - (1. / 1000.) + (rand() / ( RAND_MAX / (2. / 1000.)));
      } else {
        y = 1.;
        z = i * (height / num_height_points);
      }
      Vector3D pos = Vector3D(x, y, z);
      bool isPinned = std::any_of(pinned.begin(), pinned.end(), [=](vector<int> x){return x[0] == j && x[1] == i;});
      PointMass p = PointMass(pos, isPinned);
      point_masses.emplace_back(pos, isPinned);
    }
  }

  // Build all curve springs only
  // Build curve springs
  for (int i = 0; i < num_height_points; i++) {
    for (int j = 2; j < num_width_points; j++) {
      PointMass *prev = &point_masses[(i * num_width_points) + j - 2];
      PointMass *curr = &point_masses[(i * num_width_points) + j];
      springs.emplace_back(prev, curr, BENDING);
    }
  }

  // Build curve springs
  for (int i = 2; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {
      PointMass *prev = &point_masses[((i - 2) * num_width_points) + j];
      PointMass *curr = &point_masses[(i * num_width_points) + j];
      springs.emplace_back(prev, curr, BENDING);
    }
  }
}

double Cloth::norm(Vector3D v) {
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

Vector3D Cloth::unit(Vector3D v) {
  return v / norm(v);
}

bool Cloth::isSpringActive(EdgeSpring *s, ClothParameters *cp) {
  if (s->fractured) {
    return false;
  }

  if ((s->spring_type == STRUCTURAL && cp->enable_structural_constraints) ||
      (s->spring_type == SHEARING && cp->enable_shearing_constraints) ||
      (s->spring_type == BENDING && cp->enable_bending_constraints)) {
      return true;
  }

  return false;
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
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
    if (isSpringActive(s, cp)) {
      force_mag = cp->ks * (norm(s->pm_a->position - s->pm_b->position) - s->rest_length);
    } 
    if (isSpringActive(s, cp) && s->spring_type == BENDING) {
      force_mag *= 0.2;
    }
    s->pm_b->forces += unit(s->pm_a->position - s->pm_b->position) * force_mag;
    s->pm_a->forces += unit(s->pm_b->position - s->pm_a->position) * force_mag;
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
  

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (int i = 0; i < point_masses.size(); i++) {
    self_collide(point_masses[i], simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
  for (int i = 0; i < point_masses.size(); i++) {
    for (int j = 0; j < collision_objects->size(); j++) {
      collision_objects->at(j)->collide(point_masses[i]);
    }
  }

  // check if springs have crossed their threshold
  for (int i = 0; i < springs.size(); i++) {
    EdgeSpring *s = &springs[i];
    if (s->fracture_thresh != 0 && norm(s->pm_a->position - s->pm_b->position) > (s->rest_length * s->fracture_thresh)) {
      break_spring(s);
    }
  }


  // in length more than 10% per timestep [Provot 1995].
  for (int i = 0; i < springs.size(); i++) {
    EdgeSpring *s = &springs[i];
    if (s->fractured) {
      continue;
    } else if (norm(s->pm_a->position - s->pm_b->position) > (s->rest_length * 1.2)) {
      if (s->pm_a->pinned) {
        // check if correct
        s->pm_b->position = (unit(s->pm_b->position - s->pm_a->position) * (s->rest_length * 1.2)) + s->pm_a->position;
      } else if (s->pm_b->pinned) {
        s->pm_a->position = (unit(s->pm_a->position - s->pm_b->position) * (s->rest_length * 1.2)) + s->pm_b->position;
      } else {
        Vector3D mid = ((s->pm_a->position - s->pm_b->position) / 2) + s->pm_b->position;
        s->pm_a->position = unit(s->pm_a->position - mid) * ((s->rest_length * 1.2) / 2.) + mid;
        s->pm_b->position = unit(s->pm_b->position - mid) * ((s->rest_length * 1.2) / 2.) + mid;
      }
    }
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (int i = 0; i < point_masses.size(); i++) {
    PointMass p = point_masses.at(i);
    float uid = hash_position(p.position);

    if (map.find(uid) == map.end()) {
      map.insert({uid, new vector<PointMass *>});
    }

    map[uid]->push_back(&point_masses.at(i));
  }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
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

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  double w = 3. * width / (double) num_width_points;
  double h = 3. * height / (double) num_height_points;
  double t = max(w, h);

  double x = floor(pos[0] / w);
  double y = floor(pos[1] / h);
  double z = floor(pos[2] / t);

  float uid = (x * w * h) + (y * h) + z;
  return uid; 
}

void Cloth::reset() {
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

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    EdgeSpring *e1 = new EdgeSpring();
    EdgeSpring *e2 = new EdgeSpring();
    EdgeSpring *e3 = new EdgeSpring();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;

    // Update all edgespring pointers
    e1->pm_a = t->pm1;
    e1->pm_b = t->pm2;
    e1->rest_length = (e1->pm_a->position - e1->pm_b->position).norm();
    
    e2->pm_a = t->pm2;
    e2->pm_b = t->pm3;
    e2->rest_length = (e2->pm_a->position - e2->pm_b->position).norm();

    e3->pm_a = t->pm3;
    e3->pm_b = t->pm1;
    e3->rest_length = (e3->pm_a->position - e3->pm_b->position).norm();

    // Add edge springs to springs list
    springs.emplace_back(*e1);
    springs.emplace_back(*e2);
    springs.emplace_back(*e3);
  }
  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
  
  setFractureThreshold();
}

double Cloth::getRandomFractureThresh(double min, double max) {
  return min + (rand() / (RAND_MAX / (max - min)));
}

void Cloth::setFractureThreshold() {
  double min = 1.2;
  double max = 1.8; 
  for (int i = 0; i < springs.size(); i++) {
    // Add random fracture threshold values to all springs
    EdgeSpring *s = &springs[i];
    s->fracture_thresh = getRandomFractureThresh(min, max);
  }

}

void Cloth::break_spring(EdgeSpring *s) {
  s->fractured = true;

  // Split the triangle mesh to account for fracture
  Halfedge *h1 = s->pm_a->halfedge;
  Halfedge *h2 = s->pm_b->halfedge;

  return;
}
