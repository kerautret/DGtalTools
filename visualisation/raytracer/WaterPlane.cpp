/**
@file WaterPlane.cpp
*/
#include <cmath>
#include "WaterPlane.h"

void
rt::WaterPlane::draw( Viewer& /* viewer */ )
{
  // Taking care of in-between poles
  glBegin( GL_QUAD_STRIP);
  glColor4fv( water.ambient );
  glMaterialfv(GL_FRONT, GL_DIFFUSE, water.diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, water.specular);
  glMaterialf(GL_FRONT, GL_SHININESS, water.shinyness );
  const Real factor = 20.0;
  Vector3 e0( 1, 0, 0 );
  Vector3 e1( 0, 1, 0 );
  Vector3 n ( 0, 0, 1 );
  Point3 p00 = center - factor*e0 - factor*e1;
  Point3 p01 = center + factor*e0 - factor*e1;
  Point3 p10 = center - factor*e0 + factor*e1;
  Point3 p11 = center + factor*e0 + factor*e1;
  glNormal3fv( n );
  glVertex3fv( p00 );
  glNormal3fv( n );
  glVertex3fv( p01 );
  glNormal3fv( n );
  glVertex3fv( p10 );
  glNormal3fv( n );
  glVertex3fv( p11 );
  glEnd();
}

rt::Vector3
rt::WaterPlane::getNormal( Point3 p )
{
  Real gx = 0.0f;
  Real gy = 0.0f;
  Real x, y;
  coordinates( p, x, y );
  Real d2 = pow( x*x + y*y, 0.3 );
  Real att = std::max( 100.0f - d2, 0.0f ) / 100.0f;
  for ( const Sinusoide& s : shape )
    {
      gx += s.gradx( x, y );
      gy += s.grady( x, y );
    }
  Vector3 u( 1.0f, 0.0f, att*gx );
  Vector3 v( 0.0f, 1.0f, att*gy );
  Vector3 n = u.cross( v );
  n /= n.norm();
  return n;
}
void
rt::WaterPlane::coordinates( Point3 p, Real& x, Real& y )
{
  // First project p onto the plane
  Point3 cp = p - center;
  // Then computes its local coordinates
  x = cp[ 0 ];
  y = cp[ 1 ];
}
rt::Material
rt::WaterPlane::getMaterial( Point3 /* p */ )
{
  return water;
}

rt::Real
rt::WaterPlane::rayIntersection( const Ray& ray, Point3& p )
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
