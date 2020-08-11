#ifndef CUBE_H
#define CUBE_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "cubeMesh.h"
#include "collision/collisionObject.h"
#include "collision/plane.h"
#include "edgeSpring.h"

using namespace CGL;
using namespace std;

enum e_orientation { HORIZONTAL = 0, VERTICAL = 1 };

struct CubeParameters {
  CubeParameters() {}
  CubeParameters(bool enable_structural_constraints,
                  bool enable_shearing_constraints,
                  bool enable_bending_constraints, double damping,
                  double density, double ks)
      : enable_structural_constraints(enable_structural_constraints),
        enable_shearing_constraints(enable_shearing_constraints),
        enable_bending_constraints(enable_bending_constraints),
        damping(damping), density(density), ks(ks) {}
  ~CubeParameters() {}

  // Global simulation parameters

  bool enable_structural_constraints;
  bool enable_shearing_constraints;
  bool enable_bending_constraints;

  double damping;

  // Mass-spring parameters
  double density;
  double ks;
};

struct Cube {
  Cube() {}
  Cube(double width, double height, int num_width_points,
        int num_height_points, float thickness);
  ~Cube();

  void simulate(double frames_per_sec, double simulation_steps, CubeParameters *cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();
  void buildCubeMesh();

  void build_spatial_map();
  void self_collide(PointMass &pm, double simulation_steps);
  float hash_position(Vector3D pos);

  double getRandomFractureThresh(double min, double max);
  void setFractureThreshold();
  void break_spring(EdgeSpring *s);
  
  // Cube properties
  double width;
  double height;
  double depth;
  int num_width_points;
  int num_height_points;
  int num_depth_points;
  double thickness;
  e_orientation orientation;

  // Cube components
  vector<PointMass> point_masses;
  vector<vector<int>> pinned;
  vector<EdgeSpring> springs;
  CubeMesh *clothMesh;
  Halfedge *halfedge;

  // Spatial hashing
  unordered_map<float, vector<PointMass *> *> map;
};

#endif /* CUBE_H */
