/**
@file Scene.h
@author JOL
*/
#pragma once
#ifndef _SCENE_H_
#define _SCENE_H_

#include <cassert>
#include <vector>
#include "GraphicalObject.h"
#include "Light.h"

/// Namespace RayTracer
namespace rt {

  /**
  Models a scene, i.e. a collection of lights and graphical objects
  (could be a tree, but we keep a list for now for the sake of
  simplicity).

  @note Once the scene receives a new object, it owns the object and
  is thus responsible for its deallocation.
  */

  struct Scene {
    /// The list of lights modelled as a vector.
    std::vector< Light* > myLights;
    /// The list of objects modelled as a vector.
    std::vector< GraphicalObject* > myObjects;

    /// Default constructor. Nothing to do.
    Scene() {}

    /// Destructor. Frees objects.
    ~Scene() 
    {
      for ( Light* light : myLights )
        delete light;
      for ( GraphicalObject* obj : myObjects )
        delete obj;
      // The vector is automatically deleted.
    }

    /// This function calls the init method of each of its objects.
    void init( Viewer& viewer )
    {
      for ( GraphicalObject* obj : myObjects )
        obj->init( viewer );
      for ( Light* light : myLights )
        light->init( viewer );
    }
    /// This function calls the draw method of each of its objects.
    void draw( Viewer& viewer )
    {
      for ( GraphicalObject* obj : myObjects )
        obj->draw( viewer );
      for ( Light* light : myLights )
        light->draw( viewer );
    }
    /// This function calls the light method of each of its lights
    void light(Viewer& viewer )
    {
      for ( Light* light : myLights )
        light->light( viewer );
    }

    /// Adds a new object to the scene.
    void addObject( GraphicalObject* anObject )
    {
      myObjects.push_back( anObject );
    }

    /// Adds a new light to the scene.
    void addLight( Light* aLight )
    {
      myLights.push_back( aLight );
    }
    
    /// returns the closest object intersected by the given ray.
    Real
    rayIntersection( const Ray& ray,
                     GraphicalObject*& object, Point3& p )
    {
      object = 0;
      Real dist2 = 0.0;
      Point3 tmp_p;
      for ( GraphicalObject* obj : myObjects )
        {
          if ( obj->rayIntersection( ray, tmp_p ) < 0.0f )
            {
              Real new_dist2 = distance( ray.origin, tmp_p );
              if ( ( object == 0 ) || ( new_dist2 < dist2 ) )
                {
                  object = obj;
                  p      = tmp_p;
                  dist2  = new_dist2;
                }
            }
        }
      return object != 0 ? -1.0f : 1.0f;
    }

  private:
    /// Copy constructor is forbidden.
    Scene( const Scene& ) = delete;
    /// Assigment is forbidden.
    Scene& operator=( const Scene& ) = delete;
  };

} // namespace rt

#endif // #define _SCENE_H_
