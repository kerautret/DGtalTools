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
 * @file WaterPlane.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(WaterPlane_RECURSES)
#error Recursive header files inclusion detected in WaterPlane.h
#else // defined(WaterPlane_RECURSES)
/** Prevents recursive inclusion of headers. */
#define WaterPlane_RECURSES

#if !defined WaterPlane_h
/** Prevents repeated inclusion of headers. */
#define WaterPlane_h

#include <vector>
#include "raytracer/GraphicalObject.h"

namespace DGtal {
  namespace rt {

    /// This is an interface specifying methods that any graphical
    /// object should have. It is also drawable to be seen in QGLViewer
    /// window.
    /// Concrete exemples of a GraphicalObject include spheres.
    struct WaterPlane : public GraphicalObject {
      /// The point at coordinate (0,0) on this plane.
      Point3 center;
      /// Material used for simulating the water
      Material water;
    
      struct Sinusoide {
        Real r; ///< amplitude
        Real a; ///< angle
        Real l; ///< wavelength
        Real phi; ///< phase
        Real ca; ///< cos angle
        Real sa; ///< sin angle
        Real beta; ///< 2*pi/l
        Sinusoide( Real amplitude, Real angle, Real wavelength, Real phase )
          : r( amplitude ), a( angle ), l( wavelength ), phi( phase )
        {
          ca = cos( a ); sa = sin( a ); beta = 2.0 * M_PI / l;
        }
        inline
        Real t( Real x, Real y ) const
        {
          return x*cos( a ) + y*sin( a );
        }
        inline
        Real eval( Real x, Real y ) const
        {
          return r * cos( t( x, y ) * beta + phi );
        }
        inline
        Real gradx( Real x, Real y ) const
        {
          return - (ca * r / l) * sin( t( x, y ) * beta + phi );
        }
        inline
        Real grady( Real x, Real y ) const
        {
          return - (sa * r / l) * sin( t( x, y ) * beta + phi );
        }
      };

      /// The "shape" of the water is given by a sum of oriented
      /// sinusoidal functions in the plane.
      std::vector<Sinusoide> shape;
    
      /// Creates a periodic water plane passing through \a c and
      /// tangent to \a u and \a v. Then \a w defines the width of the
      /// band around (0,0) and its period to put material \a band_m,
      /// otherwise \a main_m is used.
      WaterPlane( Point3 c, Material w )
        : center( c ), water( w )
      {
        shape.push_back( Sinusoide( 0.2f, 0.15f, 1.6f, 0.0f ) ); // big one
        shape.push_back( Sinusoide( 0.18f, 2.0f, 2.1f, 0.5f ) ); // another big one
        shape.push_back( Sinusoide( 0.12f, 1.5f, 3.7f, 1.5f ) ); // medium
        shape.push_back( Sinusoide( 0.15f, 2.8f, 2.2f, 0.7f ) ); // medium
        shape.push_back( Sinusoide( 0.07f, 1.15f, 1.3f, 2.6f ) ); // small
        shape.push_back( Sinusoide( 0.05f, 0.71f, 3.0f, 2.0f ) ); // small
        shape.push_back( Sinusoide( 0.08f, 1.6f, 2.0f, 0.25 ) ); // small 
        shape.push_back( Sinusoide( 1.0f, 0.44f, 15.2f, 0.1f ) ); // large
        /* shape.push_back( Sinusoide( 1.0f, 0.2f, 1.0f, 0.6f ) ); // medium */
      }
    
      /// Virtual destructor since object contains virtual methods.
      virtual ~WaterPlane() {}

      /// Given a point on the plane, returns its two coordinates in a
      /// plane parametrization.
      void coordinates( Point3 p, Real& x, Real& y );

      /// This method is called by Scene::init() at the beginning of the
      /// display in the OpenGL window. May be useful for some
      /// precomputations.
      virtual void init( RTViewer& /* viewer */ ) {}

      /// This method is called by Scene::draw() at each frame to
      /// redisplay objects in the OpenGL window.
      virtual void draw( RTViewer& viewer );

      /// @return the normal vector at point \a p on the object (\a p
      /// should be on or close to the sphere).
      virtual Vector3 getNormal( Point3 p );

      /// @return the material associated to this part of the object
      virtual Material getMaterial( Point3 p );

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

#endif // !defined WaterPlane_ha

#undef WaterPlane_RECURSES
#endif // else defined(WaterPlane_RECURSES)

