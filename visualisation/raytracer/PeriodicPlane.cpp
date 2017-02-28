/**
@file PeriodicPlane.cpp
*/
#include <cmath>
#include "PeriodicPlane.h"

void
rt::PeriodicPlane::draw( Viewer& /* viewer */ )
{
  // Taking care of in-between poles
  glBegin( GL_QUAD_STRIP);
  glColor4fv( main_material.ambient );
  glMaterialfv(GL_FRONT, GL_DIFFUSE, main_material.diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, main_material.specular);
  glMaterialf(GL_FRONT, GL_SHININESS, main_material.shinyness );
  const Real factor = 20.0;
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
rt::PeriodicPlane::getNormal( Point3 /* p */ )
{
  return n;
}
void
rt::PeriodicPlane::coordinates( Point3 p, Real& x, Real& y )
{
  // First project p onto the plane
  Point3 cp = p - center;
  Point3 pp = cp - n.dot( cp ) * n;
  // Then computes its local coordinates
  x = pp.dot( e0 ) / e0.dot( e0 );
  y = pp.dot( e1 ) / e1.dot( e1 );
}
rt::Material
rt::PeriodicPlane::getMaterial( Point3 p )
{
  Real x, y;
  coordinates( p, x, y );
  Real d2 = pow( x*x + y*y, 0.3 );
  Real att = std::max( 20.0f - d2, 0.0f ) / 20.0f;

  x -= floor( x );
  y -= floor( y );

  return ( ( x < width ) || ( y < width ) )
    ? Material::mix( att, faraway, band_material ) 
    : Material::mix( att, faraway, main_material );
}

rt::Real
rt::PeriodicPlane::rayIntersection( const Ray& ray, Point3& p )
{
  Real cos_a = ray.direction.dot( n );
  Real dist  = n.dot( ray.origin - center );
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
