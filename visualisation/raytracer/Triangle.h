/**
@file Triangle.h
@author JOL
*/
#pragma once
#ifndef _TRIANGLE_H_
#define _TRIANGLE_H_

// In order to call opengl commands in all graphical objects
#include "Viewer.h"
#include "GraphicalObject.h"

/// Namespace RayTracer
namespace rt {


  /// A triangle is a model of GraphicalObject.
  struct Triangle : public GraphicalObject {
    /// The first vertex of the triangle (coordinates (0,0)).
    Point3 A;
    /// The second vertex of the triangle (coordinates (1,0)).
    Point3 B;
    /// The third vertex of the triangle (coordinates (0,1)).
    Point3 C;
    /// The unit normal vector to this plane (orthogonal to e0 and e1
    /// and [e0, e1, n] is a direct basis).
    Vector3 N;
    /// A vector orthogonal to AC and N.
    Vector3 U;
    /// A vector orthogonal to AB and N.
    Vector3 V;
    /// Material used everywhere except on the band along the edges.
    Material main_material;
    /// Material used on the band along the edges.
    Material band_material;
    /// Width of the bands
    Real width;
    
    /// Creates a triangle of vertices \a a, \a b and \a c. The
    /// triangle has material \a main_material everywhere except along
    /// a band along the edges of relative width \a w.
    inline
    Triangle( Point3 a, Point3 b, Point3 c, 
              Material main_m, Material band_m, Real w )
      : A( a ), B( b ), C( c ),
        main_material( main_m ), band_material( band_m ), width( w )
    {
      Vector3 AB = B - A;
      Vector3 AC = C - A;
      N = AB.cross( AC );
      N /= N.norm();
      U = AC.cross( N );
      V = N.cross( AB );
    }
    
    /// Virtual destructor since object contains virtual methods.
    ~Triangle() {}

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

#endif // #define _TRIANGLE_H_
