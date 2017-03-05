/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#pragma once

/**
 * @file PointLight.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(PointLight_RECURSES)
#error Recursive header files inclusion detected in PointLight.h
#else // defined(PointLight_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PointLight_RECURSES

#if !defined PointLight_h
/** Prevents repeated inclusion of headers. */
#define PointLight_h

#include <QGLViewer/manipulatedFrame.h>
#include "raytracer/Light.h"

namespace DGtal {
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
      RealColor emission;
      /// The material (global to the light).
      Material material;
      /// Used to store a manipulator to move the light in space.
      qglviewer::ManipulatedFrame* manipulator;

      /// Constructor. \a light_number must be different for every light
      /// (GL_LIGHT0, GL_LIGHT1, etc).
      PointLight( GLenum light_number,
                  Point4 pos, 
                  RealColor emission_color,
                  RealColor ambient_color  = RealColor( 0.0, 0.0, 0.0 ),
                  RealColor diffuse_color  = RealColor( 1.0, 1.0, 1.0 ),
                  RealColor specular_color = RealColor( 1.0, 1.0, 1.0 ) )
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
      void init( RTViewer& viewer ) 
      {
        Material m = material;
        viewer.setFillColor( m.diffuse );
        if ( position[ 3 ] != 0.0 )
          {
            Vector3 pos3( position[ 0 ] / position[ 3 ], 
                          position[ 1 ] / position[ 3 ], 
                          position[ 2 ] / position[ 3 ] );
            viewer.addBall( pos3, 1.0, 20 );
          }
        
        // glMatrixMode(GL_MODELVIEW);
        // glLoadIdentity();
        // glEnable( number );
        // glLightfv( number, GL_AMBIENT,  material.ambient );
        // glLightfv( number, GL_DIFFUSE,  material.diffuse );
        // glLightfv( number, GL_SPECULAR, material.specular );
        // std::cout << "Init  light at " << position << std::endl;
        // if ( position[ 3 ] != 0.0 ) // the point light is not at infinity
        //   {
        //     std::cout << "Init manipulator for light at " << position << std::endl;
        //     manipulator = new qglviewer::ManipulatedFrame;
        //     viewer.setMouseTracking( true );
        //     manipulator->setPosition( position[ 0 ] / position[ 3 ], 
        //                               position[ 1 ] / position[ 3 ], 
        //                               position[ 2 ] / position[ 3 ] );
        //   }
      }

      /// This method is called by Scene::light() at each frame to
      /// set the lights in the OpenGL window.
      void light( RTViewer& /* viewer */ ) 
      {
        // GLfloat pos[ 4 ];
        // pos[ 0 ] = (GLfloat) position[ 0 ];
        // pos[ 1 ] = (GLfloat) position[ 1 ];
        // pos[ 2 ] = (GLfloat) position[ 2 ];
        // pos[ 3 ] = (GLfloat) position[ 3 ];
        // if ( manipulator != 0 )
        //   {
        //     qglviewer::Vec pos2 = manipulator->position();
        //     pos[0] = float(pos2.x);
        //     pos[1] = float(pos2.y);
        //     pos[2] = float(pos2.z);
        //     pos[3] = 1.0f;
        //     position[ 0 ] = (Real) pos[ 0 ];
        //     position[ 1 ] = (Real) pos[ 1 ];
        //     position[ 2 ] = (Real) pos[ 2 ];
        //     position[ 3 ] = (Real) pos[ 3 ];
        //   }
        // glLightfv( number, GL_POSITION, pos);
      }

      /// This method is called by Scene::draw() at each frame to
      /// redisplay objects in the OpenGL window.
      void draw( RTViewer& viewer )
      {
        // if ( manipulator != 0 && manipulator->grabsMouse() )
        //   viewer.drawSomeLight( number, 1.2f );
        // else
        //   viewer.drawSomeLight( number );
      }

      /// Given the point \a p, returns the normalized direction to this light.
      Vector3 direction( const Vector3& p ) const
      {
        Vector3 pos( position[ 0 ], position[ 1 ], position[ 2 ] );
        if ( position[ 3 ] == 0.0 ) return pos / pos.norm() ;
        pos /= position[ 3 ];
        pos -= p;
        return pos / pos.norm();
      }

      /// @return the color of this light viewed from the given point \a p.
      RealColor color( const Vector3& /* p */ ) const
      {
        return emission;
      }
    
    };

  } // namespace rt
} // namespace DGtal

#endif // !defined PointLight_ha

#undef PointLight_RECURSES
#endif // else defined(PointLight_RECURSES)

