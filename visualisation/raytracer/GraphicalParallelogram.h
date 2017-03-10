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
 * @file GraphicalParallelogram.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(GraphicalParallelogram_RECURSES)
#error Recursive header files inclusion detected in GraphicalParallelogram.h
#else // defined(GraphicalParallelogram_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GraphicalParallelogram_RECURSES

#if !defined GraphicalParallelogram_h
/** Prevents repeated inclusion of headers. */
#define GraphicalParallelogram_h

#include "raytracer/Parallelogram.h"
#include "raytracer/GraphicalObject.h"

namespace DGtal {
  namespace rt {

    /// A triangle is a model of GraphicalObject.
    struct GraphicalParallelogram : public Parallelogram,
                                    public GraphicalObject {
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
      GraphicalParallelogram( Point3 a, Point3 b, Point3 c, 
                              Material main_m, Material band_m, Real w )
        : Parallelogram( a, b, c ),
          main_material( main_m ), band_material( band_m ), width( w )
      {}
    
      /// Virtual destructor since object contains virtual methods.
      ~GraphicalParallelogram() {}

      /// This method is called by Scene::init() at the beginning of the
      /// display in the OpenGL window. May be useful for some
      /// precomputations.
      virtual void init( RTViewer& viewer );

      /// This method is called by Scene::draw() at each frame to
      /// redisplay objects in the OpenGL window.
      virtual void draw( RTViewer& viewer );

      /// @return the material associated to this part of the object
      virtual Material getMaterial( Point3 p );
                    
    };

  } // namespace rt
} // namespace DGtal

#endif // !defined GraphicalParallelogram_ha

#undef GraphicalParallelogram_RECURSES
#endif // else defined(GraphicalParallelogram_RECURSES)
