#ifndef COLLISIONOBJECT_SPHERE_H
#define COLLISIONOBJECT_SPHERE_H

#define GRAVITY_MULTIPLIER 1000

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "collisionObject.h"

using namespace CGL;
using namespace std;

struct Sphere : public CollisionObject {
public:
    Sphere(const Vector3D &origin, double radius, double friction, int num_lat = 40, int num_lon = 40)
            : origin(origin), radius(radius), radius2(radius * radius),
              friction(friction), m_sphere_mesh(Misc::SphereMesh(num_lat, num_lon)) {}

    void render(GLShader &shader);
    void collide(PointMass &pm);
    void set_pinned(bool);
    void update_moments(double);
    void simulate(double, Vector3D);

private:
    Vector3D origin;
    double radius;
    double radius2;

    double friction;

    // Sets whether or not the sphere should move
    bool pinned = false;

    // The velocity of the sphere and forces acting on it
    Vector3D velocity = Vector3D(0.0, 0.0, 0.0);
    Vector3D forces = Vector3D(0.0, 0.0, 0.0);
    double mass = 100;
    const double gravity_multiplier = 1000;


    Misc::SphereMesh m_sphere_mesh;
};

#endif /* COLLISIONOBJECT_SPHERE_H */
