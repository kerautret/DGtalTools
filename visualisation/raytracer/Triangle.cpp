/**
@file Triangle.cpp
*/
#include <cmath>
#include "Triangle.h"

void
rt::Triangle::draw( Viewer& /* viewer */ )
{
  // Taking care of in-between poles
  glBegin( GL_TRIANGLES );
  glColor4fv( main_material.ambient );
  glMaterialfv(GL_FRONT, GL_DIFFUSE, main_material.diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, main_material.specular);
  glMaterialf(GL_FRONT, GL_SHININESS, main_material.shinyness );
  glNormal3fv( N );
  glVertex3fv( A );
  glNormal3fv( N );
  glVertex3fv( B );
  glNormal3fv( N );
  glVertex3fv( C );
  glEnd();
}

rt::Vector3
rt::Triangle::getNormal( Point3 /* p */ )
{
  return N;
}
void
rt::Triangle::coordinates( Point3 p, Real& x, Real& y )
{
  // // First project p onto the plane
  Point3 cp = p - A;
  // Point3 pp = cp - N.dot( cp ) * N;
  // Then computes its local coordinates
  x = cp.dot( U ) / ( B - A ).dot( U );
  y = cp.dot( V ) / ( C - A ).dot( V );
}
rt::Material
rt::Triangle::getMaterial( Point3 p )
{
  Real x, y;
  coordinates( p, x, y );
  return ( ( x < width ) || ( y < width ) || ( ( 1.0f-x-y ) < width ) )
    ? band_material : main_material;
}

rt::Real
rt::Triangle::rayIntersection( const Ray& ray, Point3& p )
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
  return ( ( x >= 0.0f ) && ( y >= 0.0f ) && ( (x+y) <= 1.0f ) )
    ? -1.0f : fabs(x) + fabs( y );
  // if ( ( x >= 0.0f ) && ( y >= 0.0f ) && ( (x+y) <= 1.0f ) )
  //   return -1.0f;
  // else
  //   {
  //     x = std::max( x, 0.0f );
  //     y = std::max( y, 0.0f );
  //     if ( ( x + y ) > 1.0f ) { x /= (x+y); y /= (x+y); }
  //     return ( p - ( A + x * (B-A) + y * (C-A) ) ).norm();
  //   }
}
