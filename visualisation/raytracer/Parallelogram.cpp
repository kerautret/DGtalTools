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
 * @file Parallelogram.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include "Parallelogram.h"


DGtal::rt::Vector3
DGtal::rt::Parallelogram::getNormal( Point3 /* p */ )
{
  return N;
}
void
DGtal::rt::Parallelogram::coordinates( Point3 p, Real& x, Real& y )
{
  // // First project p onto the plane
  Point3 cp = p - A;
  // Point3 pp = cp - N.dot( cp ) * N;
  // Then computes its local coordinates
  x = cp.dot( U ) / ( B - A ).dot( U );
  y = cp.dot( V ) / ( C - A ).dot( V );
}

DGtal::rt::Real
DGtal::rt::Parallelogram::rayIntersection( const Ray& ray, Point3& p )
{
  Real cos_a = ray.direction.dot( N );
  Real dist  = N.dot( ray.origin - A );
  if ( fabs( cos_a ) < 0.00001 ) // vector is tangent
    {
      p = ray.origin;
      return fabs( dist );
    }
  Real gamma = - dist/ cos_a;
  if ( gamma < 0.00001 ) // ray is going away from the plane.
    return fabs( dist );
  p = ray.origin + gamma * ray.direction;
  Real x, y;
  coordinates( p, x, y );
  return ( ( x >= 0.0f ) && ( y >= 0.0f ) && ( x <= 1.0f ) && ( y <= 1.0f ) )
    ? -1.0f : std::max( std::max( -x, x-1.0 ), std::max( -y, y-1.0 ) );
}
