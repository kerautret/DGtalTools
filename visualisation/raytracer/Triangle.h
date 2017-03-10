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
 * @file Triangle.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(Triangle_RECURSES)
#error Recursive header files inclusion detected in Triangle.h
#else // defined(Triangle_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Triangle_RECURSES

#if !defined Triangle_h
/** Prevents repeated inclusion of headers. */
#define Triangle_h

#include "raytracer/GeometricalObject.h"

namespace DGtal {
  namespace rt {

    /// A triangle is a model of GraphicalObject.
    struct Triangle : public virtual GeometricalObject {
      /// The first vertex of the triangle (coordinates (0,0)).
      Point3 A;
      /// The second vertex of the triangle (coordinates (1,0)).
      Point3 B;
      /// The third vertex of the triangle (coordinates (0,1)).
      Point3 C;
      /// The unit normal vector to this plane (orthogonal to e0 and e1
      /// and [e0, e1, n] is a direct basis).
      Vector3 N;
      /// A vector orthogonal to AC and N.
      Vector3 U;
      /// A vector orthogonal to AB and N.
      Vector3 V;
    
      /// Creates a triangle of vertices \a a, \a b and \a c. The
      /// triangle has material \a main_material everywhere except along
      /// a band along the edges of relative width \a w.
      inline
      Triangle( Point3 a, Point3 b, Point3 c )
        : A( a ), B( b ), C( c )
      {
        Vector3 AB = B - A;
        Vector3 AC = C - A;
        N = AB.crossProduct( AC );
        N /= N.norm();
        U = AC.crossProduct( N );
        V = N.crossProduct( AB );
      }
    
      /// Virtual destructor since object contains virtual methods.
      virtual ~Triangle() {}

      void coordinates( Point3 p, Real& x, Real& y );

      /// @return the normal vector at point \a p on the object (\a p
      /// should be on or close to the sphere).
      virtual Vector3 getNormal( Point3 p );

      /// @param[in] ray the incoming ray
      /// @param[out] returns the point of intersection with the object
      /// (if any), or the closest point to it.
      ///
      /// @return either a real < 0.0 if there is an intersection, or a
      /// kind of distance to the closest point of intersection.
      virtual Real rayIntersection( const Ray& ray, Point3& p );
                    
    };

  } // namespace rt
} // namespace DGtal

#endif // !defined Triangle_ha

#undef Triangle_RECURSES
#endif // else defined(Triangle_RECURSES)
