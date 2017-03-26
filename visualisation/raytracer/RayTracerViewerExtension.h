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
 * @file RayTracerViewerExtension.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(RayTracerViewerExtension_RECURSES)
#error Recursive header files inclusion detected in RayTracerViewerExtension.h
#else // defined(RayTracerViewerExtension_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RayTracerViewerExtension_RECURSES

#if !defined RayTracerViewerExtension_h
/** Prevents repeated inclusion of headers. */
#define RayTracerViewerExtension_h

#include <vector>
#include <QKeyEvent>
//#include <QGLViewer/qglviewer.h>
#include "raytracer/RealColor.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/viewers/Viewer3D.h"

namespace DGtal {
  namespace rt {
    
    /// Forward declaration of class Scene
    struct Scene;

    /// The viewer used for the ray tracer.
    typedef DGtal::Viewer3D<Space3,KSpace3> RTViewer;

    /// This class displays the interface for placing the camera and the
    /// lights, and the user may call the renderer from it.
    class RayTracerViewerExtension
      : public DGtal::Viewer3D<Space3,KSpace3>::Extension
    {
    public:
      typedef DGtal::Viewer3D<Space3,KSpace3>            Viewer;
      typedef DGtal::Viewer3D<Space3,KSpace3>::Extension Base;
      
      struct RealColor2Color {
        inline Color operator()( const RealColor& rc ) const
        {
          Color tmp;
          tmp.setRGBf( (float) rc.r(), (float) rc.g(), (float) rc.b() );
          return tmp;
        }
                       
      };

    public:
      /// Default constructor. Scene is empty.
      RayTracerViewerExtension() : ptrScene( 0 ), maxDepth( 6 ),
                                   nbSamples( 10 ), nbCasts( 5 ) {}

      /// Constructor with scene and depth.
      RayTracerViewerExtension( DGtal::rt::Scene& aScene,
                                int depth = 6 )
        : ptrScene( &aScene ), maxDepth( depth ),
          nbSamples( 10 ), nbCasts( 5 ) {}
      

      static Point3 toPoint3( const qglviewer::Vec& v )
      {
        return Point3( (Real) v[ 0 ], (Real) v[ 1 ], (Real) v[ 2 ] );
      }
      static qglviewer::Vec toVec( const Point3& v )
      {
        return qglviewer::Vec( (float) v[ 0 ], (float) v[ 1 ], (float) v[ 2 ] );
      }

      /// Sets the scene
      void setScene( DGtal::rt::Scene& aScene )
      {
        ptrScene = &aScene;
      }

      /// Changes the rendering mode for digital volumes.
      void setDigitalVolumeMode( bool estimatedNormals, bool Phong, bool smoothed );
    
      /// Called at each draw of the window
      virtual void draw( Viewer& viewer );

      /// Called before the first draw
      virtual void init( Viewer& viewer );

      /// Called when pressing help.
      virtual QString helpString( const Viewer& viewer ) const;

      /// Celled when pressing a key.
      virtual bool keyPressEvent( Viewer& viewer, QKeyEvent *e);
    
      /// Stores the scene
      DGtal::rt::Scene* ptrScene;

      /// Maximum depth
      int maxDepth;

      /// Number of random samples per pixel
      int nbSamples;
      /// Number of cast per random sample
      int nbCasts;
    };
  }
}
#endif // !defined RayTracerViewerExtension_h

#undef RayTracerViewerExtension_RECURSES
#endif // else defined(RayTracerViewerExtension_RECURSES)

