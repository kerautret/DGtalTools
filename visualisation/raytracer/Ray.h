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
 * @file Ray.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(Ray_RECURSES)
#error Recursive header files inclusion detected in Ray.h
#else // defined(Ray_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Ray_RECURSES

#if !defined Ray_h
/** Prevents repeated inclusion of headers. */
#define Ray_h

#include "DGtal/kernel/SpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"

#define RT_EPSILON 0.00001
// precision of line digitization
#define RT_PRECISION 1000
// inverse of RT_PRECISION
#define RT_BANDWIDTH 0.001

namespace DGtal {
  /// Namespace RayTracer
  namespace rt {
    typedef int                 Integer;
    typedef SpaceND<2,int>      Space2;
    typedef Space2::Point       Point2i;
    typedef Space2::RealVector  Vector2;
    typedef Space2::RealPoint   Point2;

    typedef SpaceND<3,int>      Space3;
    typedef KhalimskySpaceND<3,int> KSpace3;
    typedef Space3::Point       Point3i;
    typedef Space3::RealVector  Vector3;
    typedef Space3::RealPoint   Point3;

    typedef SpaceND<4,int>      Space4;
    typedef KhalimskySpaceND<4,int> KSpace4;
    typedef Space4::Point       Point4i;
    typedef Space4::RealVector  Vector4;
    typedef Space4::RealPoint   Point4;

    typedef Vector3::Component  Real;

    
    /// Calcule le vecteur réfléchi à V selon la direction N.
    inline
    Vector3 reflect( const Vector3& V, Vector3 N )
    {
      // V : light ray from eye
      // V_reflect = V - 2 (N . V) N
      Real n_dot_v = N.dot( V );
      if ( n_dot_v > 0.0 ) { n_dot_v = -n_dot_v; N *= -1.0; }
      return V - ( 2.0 * n_dot_v ) * N;
    }

    
    /// This structure stores a ray having an origin and a direction. It
    /// also stores its depth.
    struct Ray {
      /// origin of the ray.
      Point3 origin;
      /// unit direction of the ray.
      Vector3 direction;
      /// depth of the ray, i.e. the number of times it can bounce on an object.
      int depth;
      /// refractive index of media
      Real refractive_index;

      /// Default constructor
      Ray() {}
      
      /// Constructor from origin and vector. The vector may not be unitary.
      Ray( const Point3& o, const Vector3& dir, int d = 1, Real index = 1.0 )
        : origin( o ), direction( dir ), depth( d ), refractive_index( index )
      {
        Real l = direction.norm();
        if ( l != 1.0f ) direction /= l;
      }
    };
    
  } // namespace rt
} // namespace DGtal

#endif // !defined Ray_h

#undef Ray_RECURSES
#endif // else defined(Ray_RECURSES)

