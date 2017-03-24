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
 * @file RayIntersection.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(RayIntersection_RECURSES)
#error Recursive header files inclusion detected in RayIntersection.h
#else // defined(RayIntersection_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RayIntersection_RECURSES

#if !defined RayIntersection_h
/** Prevents repeated inclusion of headers. */
#define RayIntersection_h

#include "Ray.h"

namespace DGtal {
  /// Namespace RayTracer
  namespace rt {

    /// This structure stores the intersection of a ray with an object.
    struct RayIntersection {
      /// The incident ray
      Ray ray;
      /// A real number, <0 if intersection, >0 if not, ==0 if it grazes the object
      Real distance;
      /// Normal at intersection
      Vector3 normal;
      /// Point of intersection with object
      Point3 intersection;
      /// Origin for reflexion ray (if any)
      Point3 reflexion;
      /// Origin for refraction ray (if any)
      Point3 refraction;
      /// Refractive index inside media.
      Real   in_refractive_index;
      /// Refractive index outside media.
      Real   out_refractive_index;
      
      /// Default constructor
      RayIntersection() {}

      /// Constructor from parameters.
      RayIntersection( const Ray& inputRay, Real d = 0.0,
                       Vector3 N = Vector3(), Point3 i = Point3(),
                       Point3 rx = Point3(), Point3 rc = Point3(),
                       Real in_index = 1.0, Real out_index = 1.0 )
        : ray( inputRay ), distance( d ), normal( N ),
          intersection( i ), reflexion( rx ), refraction( rc ),
          in_refractive_index( in_index ), out_refractive_index( out_index )
      {}

      /// Builds reflexion ray.
      Ray reflexionRay() const
      {
        return Ray( reflexion, reflect( ray.direction, normal ),
                    ray.depth - 1, ray.refractive_index );
      }

      /// Builds refraction ray.
      Ray refractionRay() const
      {
        // Snell's law (wikipedia)
        // l : light ray
        // n1 : refraction index in-media, n2 : refraction index out-media
        // let r = n1/n2, c = - n . l
        // v_refract = r l + ( rc - sqrt( 1 - r^2( 1 - c^2 ) ) ) n
        Vector3 n = normal;
        Real    c = - n.dot( ray.direction );
        if ( c < 0.0 )
          { // ray is inside and must go outside
            Real r = in_refractive_index / out_refractive_index;
            c = -c; n *= -1.0;
            Vector3 v_refract = r * ray.direction
              + ( r*c - (Real)sqrt( std::max( 0.0, 1.0 - r*r*(1-c*c) ) ) ) * n;
            return Ray( refraction, v_refract, ray.depth - 1, out_refractive_index );
          }
        else
          { // ray is outside and must go inside
            Real r = out_refractive_index / in_refractive_index;
            Vector3 v_refract = r * ray.direction
              + ( r*c - (Real)sqrt( std::max( 0.0, 1.0 - r*r*(1-c*c) ) ) ) * n;
            return Ray( refraction, v_refract, ray.depth - 1, in_refractive_index );
          }
      }

    };
    
  } // namespace rt
} // namespace DGtal

#endif // !defined RayIntersection_h

#undef RayIntersection_RECURSES
#endif // else defined(RayIntersection_RECURSES)

