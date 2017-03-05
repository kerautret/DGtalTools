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
 * @file Sphere.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(Sphere_RECURSES)
#error Recursive header files inclusion detected in Sphere.h
#else // defined(Sphere_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Sphere_RECURSES

#if !defined Sphere_h
/** Prevents repeated inclusion of headers. */
#define Sphere_h


// In order to call opengl commands in all graphical objects
#include "raytracer/GraphicalObject.h"

namespace DGtal {
  namespace rt {
    /// A sphere is a concrete GraphicalObject that represents a sphere in 3D space.
    struct Sphere : public GraphicalObject {
    
      static const int NLAT = 16; ///< number of different latitudes for display
      static const int NLON = 24; ///< number of different longitudes for display

      /// Virtual destructor since object contains virtual methods.
      virtual ~Sphere() {}

      /// Creates a sphere of center \a xc and radius \a r.
      Sphere( Point3 xc, Real r, const Material& m  )
        : GraphicalObject(), center( xc ), radius( r ), material( m )
      {}

      /// Given latitude and longitude in degrees, returns the point on
      /// the sphere at these coordinates.
      Point3 localize( Real latitude, Real longitude ) const;

      // ---------------- GraphicalObject services ----------------------------
    public:

      /// This method is called by Scene::init() at the beginning of the
      /// display in the OpenGL window. May be useful for some
      /// precomputations.
      void init( RTViewer& viewer );

      /// This method is called by Scene::draw() at each frame to
      /// redisplay objects in the OpenGL window.
      void draw( RTViewer& viewer );


      /// @return the normal vector at point \a p on the sphere (\a p
      /// should be on or close to the sphere).
      Vector3 getNormal( Point3 p );

      /// @return the material associated to this part of the object
      Material getMaterial( Point3 p );

      /// @param[in] ray the incoming ray
      /// @param[out] returns the point of intersection with the object
      /// (if any), or the closest point to it.
      ///
      /// @return either a real < 0.0 if there is an intersection, or a
      /// kind of distance to the closest point of intersection.
      Real rayIntersection( const Ray& ray, Point3& p );

    public:
      /// The center of the sphere
      Point3 center;
      /// The radius of the sphere
      Real radius;
      /// The material (global to the sphere).
      Material material;
    };

  } // namespace rt
} // namespace DGtal

#endif // !defined Sphere_ha

#undef Sphere_RECURSES
#endif // else defined(Sphere_RECURSES)

