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
 * @file GraphicalObject.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(GraphicalObject_RECURSES)
#error Recursive header files inclusion detected in GraphicalObject.h
#else // defined(GraphicalObject_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GraphicalObject_RECURSES

#if !defined GraphicalObject_h
/** Prevents repeated inclusion of headers. */
#define GraphicalObject_h

#include "raytracer/GeometricalObject.h"
#include "raytracer/RayTracerViewerExtension.h"
#include "raytracer/Material.h"

namespace DGtal {

  namespace rt {

    /// This is an interface specifying methods that any ray-traced
    /// graphical object should have. It is also drawable to be seen in
    /// QGLViewer window.  Concrete exemples of a GraphicalObject
    /// include Spheres.
    struct GraphicalObject : public virtual GeometricalObject {
      /// Temporary float array holder.
      GLfloat tmp[ 4 ];

      /// Default constructor. Nothing to do.
      GraphicalObject() {}

      /// Virtual destructor since object contains virtual methods.
      virtual ~GraphicalObject() {}

      /// Utility to convert Vector3 to GLfloat[3]
      inline
      GLfloat* GL( const Vector3& v )
      {
        tmp[ 0 ] = (float) v[ 0 ];
        tmp[ 1 ] = (float) v[ 1 ];
        tmp[ 2 ] = (float) v[ 2 ];
        return tmp;
      }

      /// Utility to convert Vector4 to GLfloat[4]
      inline
      GLfloat* GL( const Vector4& v )
      {
        tmp[ 0 ] = (float) v[ 0 ];
        tmp[ 1 ] = (float) v[ 1 ];
        tmp[ 2 ] = (float) v[ 2 ];
        tmp[ 3 ] = (float) v[ 3 ];
        return tmp;
      }
      /// This method is called by Scene::init() at the beginning of the
      /// display in the OpenGL window. May be useful for some
      /// precomputations.
      virtual void init( RTViewer& /* viewer */ ) = 0;

      /// This method is called by Scene::draw() at each frame to
      /// redisplay objects in the OpenGL window.
      virtual void draw( RTViewer& /* viewer */ ) = 0;

      /// @return the material associated to this part of the object
      virtual Material getMaterial( Point3 p ) = 0;
                    
    };

  } // namespace rt
} // namespace DGtal

#endif // !defined GraphicalObject_h

#undef GraphicalObject_RECURSES
#endif // else defined(GraphicalObject_RECURSES)

