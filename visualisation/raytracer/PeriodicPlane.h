/**
@file PeriodicPlane.h
@author JOL
*/
#pragma once
#ifndef _PERIODIC_PLANE_H_
#define _PERIODIC_PLANE_H_

// In order to call opengl commands in all graphical objects
#include "Viewer.h"
#include "GraphicalObject.h"

/// Namespace RayTracer
namespace rt {


  /// This is an interface specifying methods that any graphical
  /// object should have. It is also drawable to be seen in QGLViewer
  /// window.
  /// Concrete exemples of a GraphicalObject include spheres.
  struct PeriodicPlane : public GraphicalObject {
    /// The point at coordinate (0,0) on this plane.
    Point3 center;
    /// The x-vector on this plane (may not be unitary)
    Vector3 e0;
    /// The y-vector on this plane (may not be unitary)
    Vector3 e1;
    /// The unit normal vector to this plane (orthogonal to e0 and e1
    /// and [e0, e1, n] is a direct basis).
    Vector3 n;
    /// Material used everywhere except on the band along the axes.
    Material main_material;
    /// Material used on the band along the axes.
    Material band_material;
    /// Width of the bands
    Real width;
    /// Far-away material
    Material faraway;
    
    /// Creates a periodic infinite plane passing through \a c and
    /// tangent to \a u and \a v. Then \a w defines the width of the
    /// band around (0,0) and its period to put material \a band_m,
    /// otherwise \a main_m is used.
    inline
    PeriodicPlane( Point3 c, Vector3 u, Vector3 v,
                   Material main_m, Material band_m, Real w)
      : center( c ), e0( u ), e1( v ),
        main_material( main_m ), band_material( band_m ), width( w )
    {
      n = u.cross( v );
      n /= n.norm();
      faraway = Material::mix( w, main_m, band_m );
    }
    
    /// Virtual destructor since object contains virtual methods.
    ~PeriodicPlane() {}

    void coordinates( Point3 p, Real& x, Real& y );

    /// This method is called by Scene::init() at the beginning of the
    /// display in the OpenGL window. May be useful for some
    /// precomputations.
    void init( Viewer& /* viewer */ ) {}

    /// This method is called by Scene::draw() at each frame to
    /// redisplay objects in the OpenGL window.
    virtual void draw( Viewer& viewer );

    /// @return the normal vector at point \a p on the object (\a p
    /// should be on or close to the sphere).
    virtual Vector3 getNormal( Point3 p );

    /// @return the material associated to this part of the object
    virtual Material getMaterial( Point3 p );

    /// @param[in] ray the incoming ray
    /// @param[out] returns the point of intersection with the object
    /// (if any), or the closest point to it.
    ///
    /// @return either a real < 0.0 if there is an intersection, or a
    /// kind of distance to the closest point of intersection.
    virtual Real rayIntersection( const Ray& ray, Point3& p );
                    

  };

} // namespace rt

#endif // #define _PERIODIC_PLANE_H_
