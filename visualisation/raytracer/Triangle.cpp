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
/**
 * @file Triangle.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include "Triangle.h"


DGtal::rt::Vector3
DGtal::rt::Triangle::getNormal( Point3 /* p */ )
{
  return N;
}
void
DGtal::rt::Triangle::coordinates( Point3 p, Real& x, Real& y )
{
  // // First project p onto the plane
  Point3 cp = p - A;
  // Point3 pp = cp - N.dot( cp ) * N;
  // Then computes its local coordinates
  x = cp.dot( U ) / ( B - A ).dot( U );
  y = cp.dot( V ) / ( C - A ).dot( V );
}

bool
DGtal::rt::Triangle::intersectRay( RayIntersection& ray_inter )
{
  const Ray& ray = ray_inter.ray;
  Real cos_a     = ray.direction.dot( N );
  Real dist      = N.dot( ray.origin - A );
  if ( fabs( cos_a ) < RT_EPSILON ) // vector is tangent
    {
      ray_inter.distance     = fabs( dist );
      ray_inter.intersection = ray.origin;
      ray_inter.normal       = N;
      ray_inter.reflexion    = ray_inter.intersection;
      ray_inter.refraction   = ray_inter.intersection;
      return false;
    }
  Real gamma = - dist/ cos_a;
  if ( gamma < RT_EPSILON ) // ray is going away from the plane.
    {
      ray_inter.distance     = fabs( dist );
      ray_inter.intersection = ray.origin;
      ray_inter.normal       = N;
      ray_inter.reflexion    = ray_inter.intersection;
      ray_inter.refraction   = ray_inter.intersection;
      return false;
    }
  Real x, y;
  Point3 p               = ray.origin + gamma * ray.direction;
  ray_inter.intersection = p;
  coordinates( p, x, y );
  ray_inter.distance     = ( ( x >= 0.0f ) && ( y >= 0.0f ) && ( (x+y) <= 1.0f ) )
    ? -1.0f : fabs(x) + fabs( y );
  ray_inter.normal       = N;
  ray_inter.reflexion    = ray_inter.intersection;
  ray_inter.refraction   = ray_inter.intersection;
  return ray_inter.distance < 0.0;
}
