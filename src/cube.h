#ifndef CUBE_H
#define CUBE_H

#define CUBE_DIM 50

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "collision/collisionObject.h"
#include "collision/plane.h"
#include "mesh.h"

using namespace CGL;
using namespace std;

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
public:
  Cube(Vector3D center, double width, double height, double depth);
  ~Cube();

  void simulate(double frames_per_sec, double simulation_steps, CubeParameters *cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();

  void build_spatial_map();
  void self_collide(PointMass &pm, double simulation_steps);
  float hash_position(Vector3D pos);

  double getRandomFractureThresh(double min, double max);
  void setFractureThreshold();
  void break_spring(EdgeSpring *s);
  
  // Cube properties
  double width{};
  double height{};
  double depth{};
  int num_width_points{};
  int num_height_points{};
  int num_depth_points{};
  double thickness{};
  Vector3D center;

  // Cube components
  vector<PointMass> point_masses;
  vector<vector<int>> pinned;
  vector<EdgeSpring> springs;
  CubeMesh *cubeMesh{};
  Halfedge *halfedge{};

private:
  void buildCubeMesh();

  // Spatial hashing
  unordered_map<float, vector<PointMass *> *> map;
};

#endif /* CUBE_H */
