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
 * @file PeriodicPlane.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(PeriodicPlane_RECURSES)
#error Recursive header files inclusion detected in PeriodicPlane.h
#else // defined(PeriodicPlane_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PeriodicPlane_RECURSES

#if !defined PeriodicPlane_h
/** Prevents repeated inclusion of headers. */
#define PeriodicPlane_h

// In order to call opengl commands in all graphical objects
#include "raytracer/GeometricalObject.h"

namespace DGtal {
  namespace rt {

    /// This is an interface specifying methods that any graphical
    /// object should have. It is also drawable to be seen in QGLViewer
    /// window.
    /// Concrete exemples of a GraphicalObject include spheres.
    struct PeriodicPlane : public virtual GeometricalObject {
      /// The point at coordinate (0,0) on this plane.
      Point3 center;
      /// The x-vector on this plane (may not be unitary)
      Vector3 e0;
      /// The y-vector on this plane (may not be unitary)
      Vector3 e1;
      /// The unit normal vector to this plane (orthogonal to e0 and e1
      /// and [e0, e1, n] is a direct basis).
      Vector3 n;
    
      /// Creates a periodic infinite plane passing through \a c and
      /// tangent to \a u and \a v. Then \a w defines the width of the
      /// band around (0,0) and its period to put material \a band_m,
      /// otherwise \a main_m is used.
      PeriodicPlane( Point3 c, Vector3 u, Vector3 v );
    
      /// Virtual destructor since object contains virtual methods.
      virtual ~PeriodicPlane();
      
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

#endif // !defined PeriodicPlane_ha

#undef PeriodicPlane_RECURSES
#endif // else defined(PeriodicPlane_RECURSES)

