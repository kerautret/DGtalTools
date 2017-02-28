/**
@file Ray.h
@author JOL
*/
#pragma once
#ifndef _RAY_H_
#define _RAY_H_

#include "PointVector.h"

// @see http://devernay.free.fr/cours/opengl/materials.html

/// Namespace RayTracer
namespace rt {

  /// This structure stores a ray having an origin and a direction. It
  /// also stores its depth.
  struct Ray {
    /// origin of the ray.
    Point3 origin;
    /// unit direction of the ray.
    Vector3 direction;
    /// depth of the ray, i.e. the number of times it can bounce on an object.
    int depth;
    
    /// Default constructor
    Ray() {}
    
    /// Constructor from origin and vector. The vector may not be unitary.
    Ray( const Point3& o, const Vector3& dir, int d = 1 )
      : origin( o ), direction( dir ), depth( d )
    {
      Real l = direction.norm();
      if ( l != 1.0f ) direction /= l;
    }
  };

  

} // namespace rt

#endif // #define _RAY_H_
