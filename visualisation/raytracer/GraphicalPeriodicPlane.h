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
 * @file GraphicalPeriodicPlane.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(GraphicalPeriodicPlane_RECURSES)
#error Recursive header files inclusion detected in GraphicalPeriodicPlane.h
#else // defined(GraphicalPeriodicPlane_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GraphicalPeriodicPlane_RECURSES

#if !defined GraphicalPeriodicPlane_h
/** Prevents repeated inclusion of headers. */
#define GraphicalPeriodicPlane_h

// In order to call opengl commands in all graphical objects
#include "raytracer/PeriodicPlane.h"
#include "raytracer/GraphicalObject.h"

namespace DGtal {
  namespace rt {

    /// This is an interface specifying methods that any graphical
    /// object should have. It is also drawable to be seen in QGLViewer
    /// window.
    /// Concrete exemples of a GraphicalObject include spheres.
    struct GraphicalPeriodicPlane : public PeriodicPlane,
                                    public GraphicalObject {
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
      GraphicalPeriodicPlane( Point3 c, Vector3 u, Vector3 v,
                              Material main_m, Material band_m, Real w);
    
      /// Virtual destructor since object contains virtual methods.
      virtual ~GraphicalPeriodicPlane();

      /// This method is called by Scene::init() at the beginning of the
      /// display in the OpenGL window. May be useful for some
      /// precomputations.
      virtual void init( RTViewer& /* viewer */ ) {}

      /// This method is called by Scene::draw() at each frame to
      /// redisplay objects in the OpenGL window.
      virtual void draw( RTViewer& viewer );

      /// @return the material associated to this part of the object
      virtual Material getMaterial( Point3 p );

    };

  } // namespace rt
} // namespace DGtal

#endif // !defined GraphicalPeriodicPlane_ha

#undef GraphicalPeriodicPlane_RECURSES
#endif // else defined(GraphicalPeriodicPlane_RECURSES)

