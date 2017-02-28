/**
@file PointLight.h
@author JOL
*/
#pragma once
#ifndef _POINT_LIGHT_H_
#define _POINT_LIGHT_H_

#include <QGLViewer/manipulatedFrame.h>
#include "Light.h"

/// Namespace RayTracer
namespace rt {

  /// This structure defines a point light, which may be at an
  /// infinite distance. Such light does not suffer from any
  /// attenuation. One can also draw it in order to be
  /// displayed and manipulated.
  struct PointLight : public Light {
    /// Specifies which OpenGL light it is (necessary for draw())
    GLenum number; // GL_LIGHT0, GL_LIGHT1, etc
    /// The position of the light in homogeneous coordinates
    Point4 position;
    /// The emission color of the light.
    Color emission;
    /// The material (global to the light).
    Material material;
    /// Used to store a manipulator to move the light in space.
    qglviewer::ManipulatedFrame* manipulator;

    /// Constructor. \a light_number must be different for every light
    /// (GL_LIGHT0, GL_LIGHT1, etc).
    PointLight( GLenum light_number,
                Point4 pos, 
                Color emission_color,
                Color ambient_color  = Color( 0.0, 0.0, 0.0 ),
                Color diffuse_color  = Color( 1.0, 1.0, 1.0 ),
                Color specular_color = Color( 1.0, 1.0, 1.0 ) )
      : number( light_number ), position( pos ), emission( emission_color ),
        material( ambient_color, diffuse_color, specular_color ),
        manipulator( 0 )
    {}
    
    /// Destructor.
    ~PointLight()
    {
      if ( manipulator != 0 ) delete manipulator;
    }

    /// This method is called by Scene::init() at the beginning of the
    /// display in the OpenGL window.
    void init( Viewer& viewer ) 
    {
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      glEnable( number );
      glLightfv( number, GL_AMBIENT,  material.ambient );
      glLightfv( number, GL_DIFFUSE,  material.diffuse );
      glLightfv( number, GL_SPECULAR, material.specular );
      std::cout << "Init  light at " << position << std::endl;
      if ( position[ 3 ] != 0.0 ) // the point light is not at infinity
        {
          std::cout << "Init manipulator for light at " << position << std::endl;
          manipulator = new qglviewer::ManipulatedFrame;
          viewer.setMouseTracking( true );
          manipulator->setPosition( position[ 0 ] / position[ 3 ], 
                                    position[ 1 ] / position[ 3 ], 
                                    position[ 2 ] / position[ 3 ] );
        }
    }

    /// This method is called by Scene::light() at each frame to
    /// set the lights in the OpenGL window.
    void light( Viewer& /* viewer */ ) 
    {
      Point4 pos = position;
      if ( manipulator != 0 )
        {
          qglviewer::Vec pos2 = manipulator->position();
          pos[0] = float(pos2.x);
          pos[1] = float(pos2.y);
          pos[2] = float(pos2.z);
          pos[3] = 1.0f;
          position = pos;
        }
      glLightfv( number, GL_POSITION, pos);
    }

    /// This method is called by Scene::draw() at each frame to
    /// redisplay objects in the OpenGL window.
    void draw( Viewer& viewer )
    {
      if ( manipulator != 0 && manipulator->grabsMouse() )
        viewer.drawSomeLight( number, 1.2f );
      else
	viewer.drawSomeLight( number );
    }

    /// Given the point \a p, returns the normalized direction to this light.
    Vector3 direction( const Vector3& p ) const
    {
      Vector3 pos( position.data() );
      if ( position[ 3 ] == 0.0 ) return pos / pos.norm() ;
      pos /= position[ 3 ];
      pos -= p;
      return pos / pos.norm();
    }

    /// @return the color of this light viewed from the given point \a p.
    Color color( const Vector3& /* p */ ) const
    {
      return emission;
    }
    
  };

  

} // namespace rt

#endif // #define _POINT_LIGHT_H_
