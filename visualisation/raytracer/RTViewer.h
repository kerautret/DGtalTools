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
 * @file RTViewer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(RTViewer_RECURSES)
#error Recursive header files inclusion detected in RTViewer.h
#else // defined(RTViewer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RTViewer_RECURSES

#if !defined RTViewer_h
/** Prevents repeated inclusion of headers. */
#define RTViewer_h

#include <vector>
#include <QKeyEvent>
//#include <QGLViewer/qglviewer.h>
#include "raytracer/Ray.h"
#include "raytracer/RealColor.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/viewers/Viewer3D.h"

namespace DGtal {
  namespace rt {
    
    /// Forward declaration of class Scene
    struct Scene;

    /// This class displays the interface for placing the camera and the
    /// lights, and the user may call the renderer from it.
    class RTViewer : public DGtal::Viewer3D<Space3,KSpace3>
    {
    public:
      typedef DGtal::Viewer3D<Space3,KSpace3> Base;
      
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
      RTViewer( const KSpace3& K ) : Base( K ), ptrScene( 0 ), maxDepth( 6 ) {}
    
      /// Sets the scene
      void setScene( DGtal::rt::Scene& aScene )
      {
        ptrScene = &aScene;
        // // Inits the scene
        // if ( ptrScene != 0 )
        //   ptrScene->init( *this );
      }
    
      /// To call the protected method `drawLight`.
      void drawSomeLight( GLenum light ) const
      {
        drawLight( light );
      }
      /// To call the protected method `drawLight`.
      void drawSomeLight( GLenum light, float zoom ) const
      {
        drawLight( light, zoom );
      }

    protected :
      // /// Called at each draw of the window
      virtual void draw(){}
      /// Called before the first draw
      virtual void init();
      /// Called when pressing help.
      virtual QString helpString() const;
      /// Celled when pressing a key.
      virtual void keyPressEvent(QKeyEvent *e);
    
      /// Stores the scene
      DGtal::rt::Scene* ptrScene;

      /// Maximum depth
      int maxDepth;
    };
  }
}
#endif // !defined RTViewer_h

#undef RTViewer_RECURSES
#endif // else defined(RTViewer_RECURSES)

