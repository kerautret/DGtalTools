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

#include "DGtal/kernel/SpaceND."


namespace DGtal {
  /// Namespace RayTracer
  namespace rt {
    
    typedef SpaceND<2>          Space2;
    typedef Space2::Point       Point2i;

    typedef SpaceND<3>          Space3;
    typedef Space3::Point       Point3i;
    typedef Space3::RealVector  Vector3;
    typedef Space3::RealPoint   Point3;
    typedef Vector3::Component  Real;

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
} // namespace DGtal

#endif // #define _RAY_H_
