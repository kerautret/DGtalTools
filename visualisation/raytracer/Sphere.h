/**
@file Sphere.h
@author JOL
*/
#pragma once
#ifndef _SPHERE_H_
#define _SPHERE_H_

// In order to call opengl commands in all graphical objects
#include "GraphicalObject.h"

/// Namespace RayTracer
namespace rt {
  /// A sphere is a concrete GraphicalObject that represents a sphere in 3D space.
  struct Sphere : public GraphicalObject {
    
    static const int NLAT = 16; ///< number of different latitudes for display
    static const int NLON = 24; ///< number of different longitudes for display

    /// Virtual destructor since object contains virtual methods.
    virtual ~Sphere() {}

    /// Creates a sphere of center \a xc and radius \a r.
    Sphere( Point3 xc, Real r, const Material& m  )
      : GraphicalObject(), center( xc ), radius( r ), material( m )
    {}

    /// Given latitude and longitude in degrees, returns the point on
    /// the sphere at these coordinates.
    Point3 localize( Real latitude, Real longitude ) const;

    // ---------------- GraphicalObject services ----------------------------
  public:

    /// This method is called by Scene::init() at the beginning of the
    /// display in the OpenGL window. May be useful for some
    /// precomputations.
    void init( Viewer& /* viewer */ ) {}

    /// This method is called by Scene::draw() at each frame to
    /// redisplay objects in the OpenGL window.
    void draw( Viewer& viewer );


    /// @return the normal vector at point \a p on the sphere (\a p
    /// should be on or close to the sphere).
    Vector3 getNormal( Point3 p );

    /// @return the material associated to this part of the object
    Material getMaterial( Point3 p );

    /// @param[in] ray the incoming ray
    /// @param[out] returns the point of intersection with the object
    /// (if any), or the closest point to it.
    ///
    /// @return either a real < 0.0 if there is an intersection, or a
    /// kind of distance to the closest point of intersection.
    Real rayIntersection( const Ray& ray, Point3& p );

  public:
    /// The center of the sphere
    Point3 center;
    /// The radius of the sphere
    Real radius;
    /// The material (global to the sphere).
    Material material;
  };

} // namespace rt

#endif // #define _SPHERE_H_
