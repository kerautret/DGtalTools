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
 * @file WaterPlane.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#include <cmath>
#include "raytracer/WaterPlane.h"

void
DGtal::rt::WaterPlane::draw( RTViewer& /* viewer */ )
{
  // Taking care of in-between poles
  glBegin( GL_QUAD_STRIP);
  glColor4fv( water.ambient );
  glMaterialfv(GL_FRONT, GL_DIFFUSE, water.diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, water.specular);
  glMaterialf(GL_FRONT, GL_SHININESS, water.shinyness );
  const Real factor = 50.0;
  Vector3 e0( 1, 0, 0 );
  Vector3 e1( 0, 1, 0 );
  Vector3 n ( 0, 0, 1 );
  Point3 p00 = center - factor*e0 - factor*e1;
  Point3 p01 = center + factor*e0 - factor*e1;
  Point3 p10 = center - factor*e0 + factor*e1;
  Point3 p11 = center + factor*e0 + factor*e1;
  glNormal3fv( GL( n ) );
  glVertex3fv( GL( p00 ) );
  glNormal3fv( GL( n ) );
  glVertex3fv( GL( p01 ) );
  glNormal3fv( GL( n ) );
  glVertex3fv( GL( p10 ) );
  glNormal3fv( GL( n ) );
  glVertex3fv( GL( p11 ) );
  glEnd();
}

DGtal::rt::Vector3
DGtal::rt::WaterPlane::getNormal( Point3 p )
{
  Real gx = 0.0;
  Real gy = 0.0;
  Real x, y;
  coordinates( p, x, y );
  Real d2 = pow( x*x + y*y, 0.3 );
  Real att = std::max( 100.0 - d2, 0.0 ) / 100.0;
  for ( const Sinusoide& s : shape )
    {
      gx += s.gradx( x, y );
      gy += s.grady( x, y );
    }
  Vector3 u( 1.0, 0.0, att*gx );
  Vector3 v( 0.0, 1.0, att*gy );
  Vector3 n = u.crossProduct( v );
  n /= n.norm();
  return n;
}
void
DGtal::rt::WaterPlane::coordinates( Point3 p, Real& x, Real& y )
{
  // First project p onto the plane
  Point3 cp = p - center;
  // Then computes its local coordinates
  x = cp[ 0 ];
  y = cp[ 1 ];
}
DGtal::rt::Material
DGtal::rt::WaterPlane::getMaterial( Point3 /* p */ )
{
  return water;
}

DGtal::rt::Real
DGtal::rt::WaterPlane::rayIntersection( const Ray& ray, Point3& p )
{
  Real cos_a = ray.direction[ 2 ];
  Real dist  = (ray.origin - center )[ 2 ];
  if ( fabs( cos_a ) < 0.00001 ) // vector is tangent
    {
      p = ray.origin;
      return fabs( dist );
    }
  Real gamma = - dist/ cos_a;
  if ( gamma < 0.00001 ) // ray is going away from the plane.
    return fabs( dist );
  p = ray.origin + gamma * ray.direction;
  return -1.0f;
}
