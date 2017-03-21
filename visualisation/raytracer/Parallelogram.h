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
 * @file Parallelogram.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(Parallelogram_RECURSES)
#error Recursive header files inclusion detected in Parallelogram.h
#else // defined(Parallelogram_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Parallelogram_RECURSES

#if !defined Parallelogram_h
/** Prevents repeated inclusion of headers. */
#define Parallelogram_h

#include "raytracer/GeometricalObject.h"

namespace DGtal {
  namespace rt {

    /// A triangle is a model of GraphicalObject.
    struct Parallelogram : public virtual GeometricalObject {
      /// The first vertex of the parallelogram (coordinates (0,0)).
      Point3 A;
      /// The second vertex of the parallelogram (coordinates (1,0)).
      Point3 B;
      /// The third vertex of the parallelogram (coordinates (0,1)).
      Point3 C;
      /// The fourth vertex of the parallelogram (coordinates (1,1)).
      Point3 D;
      /// The unit normal vector to this plane (orthogonal to e0 and e1
      /// and [e0, e1, n] is a direct basis).
      Vector3 N;
      /// A vector orthogonal to AC and N.
      Vector3 U;
      /// A vector orthogonal to AB and N.
      Vector3 V;

      /// Default constructor.
      inline
      Parallelogram() {}
      /// Creates a parallelogram of vertices \a a, \a b and \a c, and
      /// last vertex is computed as \f$ a + b-a + c-a \f$.
      inline
      Parallelogram( Point3 a, Point3 b, Point3 c )
        : A( a ), B( b ), C( c ), D( b+c-a )
      {
        Vector3 AB = B - A;
        Vector3 AC = C - A;
        N = AB.crossProduct( AC );
        N /= N.norm();
        U = AC.crossProduct( N );
        V = N.crossProduct( AB );
      }
    
      /// Virtual destructor since object contains virtual methods.
      ~Parallelogram() {}

      void coordinates( Point3 p, Real& x, Real& y );

      /// @return the normal vector at point \a p on the object (\a p
      /// should be on or close to the sphere).
      virtual Vector3 getNormal( Point3 p );

      // /// @param[in] ray the incoming ray
      // /// @param[out] returns the point of intersection with the object
      // /// (if any), or the closest point to it.
      // ///
      // /// @return either a real < 0.0 if there is an intersection, or a
      // /// kind of distance to the closest point of intersection.
      // virtual Real rayIntersection( const Ray& ray, Point3& p );

      /// @param[in,out] ray_inter as input the incoming ray, as
      /// output information abour intersection.
      ///
      /// @return true if there was an intersection, false otherwise
      /// (more information is stored in ray_inter)
      virtual bool intersectRay( RayIntersection& ray_inter );

    };

  } // namespace rt
} // namespace DGtal

#endif // !defined Parallelogram_h

#undef Parallelogram_RECURSES
#endif // else defined(Parallelogram_RECURSES)
