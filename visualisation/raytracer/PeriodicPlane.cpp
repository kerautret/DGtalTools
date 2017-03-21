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
 * @file PeriodicPlane.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include "raytracer/PeriodicPlane.h"


DGtal::rt::PeriodicPlane::PeriodicPlane
( Point3 c, Vector3 u, Vector3 v )
  : center( c ), e0( u ), e1( v )
{
  n = u.crossProduct( v );
  n /= n.norm();
}
    
DGtal::rt::PeriodicPlane::~PeriodicPlane()
{}

DGtal::rt::Vector3
DGtal::rt::PeriodicPlane::getNormal( Point3 /* p */ )
{
  return n;
}

void
DGtal::rt::PeriodicPlane::coordinates( Point3 p, Real& x, Real& y )
{
  // First project p onto the plane
  Point3 cp = p - center;
  Point3 pp = cp - n.dot( cp ) * n;
  // Then computes its local coordinates
  x = pp.dot( e0 ) / e0.dot( e0 );
  y = pp.dot( e1 ) / e1.dot( e1 );
}

bool 
DGtal::rt::PeriodicPlane::intersectRay( RayIntersection& ray_inter )
{
  const Ray& ray = ray_inter.ray;
  Real cos_a     = ray.direction.dot( n );
  Real dist      = n.dot( ray.origin - center );
  if ( fabs( cos_a ) < RT_EPSILON ) // vector is tangent
    {
      ray_inter.distance     = fabs( dist );
      ray_inter.intersection = ray.origin;
      ray_inter.normal       = n;
      ray_inter.reflexion    = ray_inter.intersection;
      ray_inter.refraction   = ray_inter.intersection;
      return false;
    }
  Real gamma = - dist/ cos_a;
  if ( gamma < 0.00001 ) // ray is going away from the plane.
    {
      ray_inter.distance     = fabs( dist );
      ray_inter.intersection = ray.origin;
      ray_inter.normal       = n;
      ray_inter.reflexion    = ray_inter.intersection;
      ray_inter.refraction   = ray_inter.intersection;
      return false;
    }
  ray_inter.distance     = -fabs( cos_a );
  ray_inter.intersection = ray.origin + gamma * ray.direction;
  ray_inter.normal       = n;
  ray_inter.reflexion    = ray_inter.intersection;
  ray_inter.refraction   = ray_inter.intersection;
  return true;
}
