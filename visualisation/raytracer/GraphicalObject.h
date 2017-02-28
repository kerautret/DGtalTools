/**
@file GraphicalObject.h
@author JOL
*/
#pragma once
#ifndef _GRAPHICAL_OBJECT_H_
#define _GRAPHICAL_OBJECT_H_

// In order to call opengl commands in all graphical objects
#include "Viewer.h"
#include "PointVector.h"
#include "Material.h"
#include "Ray.h"

/// Namespace RayTracer
namespace rt {

  /// This is an interface specifying methods that any graphical
  /// object should have. It is also drawable to be seen in QGLViewer
  /// window.
  /// Concrete exemples of a GraphicalObject include spheres.
  struct GraphicalObject {

    /// Default constructor. Nothing to do.
    GraphicalObject() {}

    /// Virtual destructor since object contains virtual methods.
    virtual ~GraphicalObject() {}

    /// This method is called by Scene::init() at the beginning of the
    /// display in the OpenGL window. May be useful for some
    /// precomputations.
    virtual void init( Viewer& /* viewer */ ) = 0;

    /// This method is called by Scene::draw() at each frame to
    /// redisplay objects in the OpenGL window.
    virtual void draw( Viewer& /* viewer */ ) = 0;

    /// @return the normal vector at point \a p on the object (\a p
    /// should be on or close to the sphere).
    virtual Vector3 getNormal( Point3 p ) = 0;

    /// @return the material associated to this part of the object
    virtual Material getMaterial( Point3 p ) = 0;

    /// @param[in] ray the incoming ray
    /// @param[out] returns the point of intersection with the object
    /// (if any), or the closest point to it.
    ///
    /// @return either a real < 0.0 if there is an intersection, or a
    /// kind of distance to the closest point of intersection.
    virtual Real rayIntersection( const Ray& ray, Point3& p ) = 0;
                    

  };

} // namespace rt

#endif // #define _GRAPHICAL_OBJECT_H_
