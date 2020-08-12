#ifndef MESH_H
#define MESH_H

#include <vector>

#include "CGL/CGL.h"
#include "pointMass.h"

using namespace CGL;
using namespace std;

// Forward declaration of edgeSpring
struct EdgeSpring;

class Triangle {
public:
  Triangle(PointMass *pm1, PointMass *pm2, PointMass *pm3, Vector3D uv1, Vector3D uv2, Vector3D uv3)
      : pm1(pm1), pm2(pm2), pm3(pm3), uv1(uv1), uv2(uv2), uv3(uv3) {}

  // Static references to constituent mesh objects
  PointMass *pm1;
  PointMass *pm2;
  PointMass *pm3;
  
  // UV values for each of the points.
  // Uses Vector3D for convenience. This means that the z dimension
  // is not used, and xy corresponds to uv.
  Vector3D uv1;
  Vector3D uv2;
  Vector3D uv3;

  // Tag for ignoring the triangle in the shaders
  bool fractured = false;

  Halfedge *halfedge;
}; // struct Triangle

class Halfedge {
public:
  EdgeSpring *edge;
  Halfedge *next;
  Halfedge *twin;
  Triangle *triangle;
  PointMass *pm;
}; // struct Halfedge

class ClothMesh {
public:
  ~ClothMesh() {}

  vector<Triangle *> triangles;
}; // struct ClothMesh

class SingleCube {
  vector<EdgeSpring *> edges;
  vector<Triangle *> triangles;
}; // struct SingleCube

class CubeMesh {
  ~CubeMesh() {}
  // TODO determine if we need cube list
  vector<SingleCube *> single_cubes;
  vector<Triangle *> triangles;
}; // struct CubeMesh

enum e_spring_type { STRUCTURAL = 0, SHEARING = 1, BENDING = 2 };

struct EdgeSpring {
    EdgeSpring(PointMass *a, PointMass *b, e_spring_type spring_type)
            : pm_a(a), pm_b(b), spring_type(spring_type) {
        rest_length = (pm_a->position - pm_b->position).norm();
    }

    EdgeSpring(PointMass *a, PointMass *b, double rest_length)
            : pm_a(a), pm_b(b), rest_length(rest_length) {
    }

    EdgeSpring() {}

    double rest_length;

    e_spring_type spring_type = STRUCTURAL;

    PointMass *pm_a;
    PointMass *pm_b;
    SingleCube *single_cube;

    // percentage of rest_length that a spring cannot surpass without breaking
    double fracture_thresh = 0;
    // when a spring is fractured, this is set to True instead of deleting it from the cloth
    bool fractured = false;

}; // struct EdgeSpring

#endif // CLOTH_MESH_H
