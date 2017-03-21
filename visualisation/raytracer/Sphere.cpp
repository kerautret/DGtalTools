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
 * @file Sphere.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include "raytracer/Sphere.h"

void
DGtal::rt::Sphere::init( RTViewer& viewer )
{
  Material m = material;
  viewer.setFillColor( m.diffuse ); 
  viewer.addBall( center, radius, 20 );
}

void
DGtal::rt::Sphere::draw( RTViewer& /* viewer */ )
{
  // //std::cout << "[DGtal::rt::Sphere::draw]" << std::endl;
  // Material m = material;
  // // Taking care of south pole
  // glBegin( GL_TRIANGLE_FAN );
  // glColor4fv( m.ambient );
  // glMaterialfv(GL_FRONT, GL_DIFFUSE, m.diffuse);
  // glMaterialfv(GL_FRONT, GL_SPECULAR, m.specular);
  // glMaterialf(GL_FRONT, GL_SHININESS, m.shinyness );
  // Point3 south_pole = localize( -90, 0 );
  // glNormal3fv( GL( getNormal( south_pole ) ) );
  // glVertex3fv( GL( south_pole ) );
  // for ( int x = 0; x <= NLON; ++x )
  //   {
  //     Point3 p = localize( -90 + 180/NLAT, x * 360 / NLON );
  //     glNormal3fv( GL( getNormal( p ) ) );
  //     glVertex3fv( GL( p ) );
  //   }
  // glEnd();
  // // Taking care of in-between poles
  // for ( int y = 1; y < NLAT - 1; ++y )
  //   {
  //     glBegin( GL_QUAD_STRIP);
  //     glColor4fv( m.ambient );
  //     glMaterialfv(GL_FRONT, GL_DIFFUSE, m.diffuse);
  //     glMaterialfv(GL_FRONT, GL_SPECULAR, m.specular);
  //     glMaterialf(GL_FRONT, GL_SHININESS, m.shinyness );
  //     for ( int x = 0; x <= NLON; ++x )
  //       {
  //         Point3 p = localize( -90 + y*180/NLAT,     x * 360 / NLON );
  //         Point3 q = localize( -90 + (y+1)*180/NLAT, x * 360 / NLON );
  //         glNormal3fv( GL( getNormal( p ) ) );
  //         glVertex3fv( GL( p ) );
  //         glNormal3fv( GL( getNormal( q ) ) );
  //         glVertex3fv( GL( q ) );
  //       }
  //     glEnd();
  //   }
  // // Taking care of north pole
  // glBegin( GL_TRIANGLE_FAN );
  // glColor4fv( m.ambient );
  // glMaterialfv(GL_FRONT, GL_DIFFUSE, m.diffuse);
  // glMaterialfv(GL_FRONT, GL_SPECULAR, m.specular);
  // glMaterialf(GL_FRONT, GL_SHININESS, m.shinyness );
  // Point3 north_pole = localize( 90, 0 );
  // glNormal3fv( GL( getNormal( north_pole ) ) );
  // glVertex3fv( GL( north_pole ) );
  // for ( int x = NLON; x >= 0; --x )
  //   {
  //     Point3 p = localize( -90 + (NLAT-1)*180/NLAT, x * 360 / NLON );
  //     glNormal3fv( GL( getNormal( p ) ) );
  //     glVertex3fv( GL( p ) );
  //   }
  // glEnd();
}

DGtal::rt::Point3
DGtal::rt::Sphere::localize( Real latitude, Real longitude ) const
{
  static const Real conv_deg_rad = 2.0 * M_PI / 360.0;
  latitude  *= conv_deg_rad;
  longitude *= conv_deg_rad;
  return center 
    + radius * Point3( cos( longitude ) * cos( latitude ),
                       sin( longitude ) * cos( latitude ),
                       sin( latitude ) );
}

DGtal::rt::Vector3
DGtal::rt::Sphere::getNormal( Point3 p )
{
  Vector3 u = p - center;
  Real   l2 = u.dot( u );
  if ( l2 != 0.0 ) u /= sqrt( l2 );
  return u;
}

DGtal::rt::Material
DGtal::rt::Sphere::getMaterial( Point3 /* p */ )
{
  return material; // the material is constant along the sphere.
}

bool
DGtal::rt::Sphere::intersectRay( RayIntersection& ray_inter )
{
  // Calcul de la distance d'intersection
  const Ray& ray = ray_inter.ray;
  Vector3    pa  = center - ray.origin;
  Real     l_ppa = pa.dot( ray.direction );  // longueur du projete de pa sur dir
  Real        d2 = pa.dot( pa ) - l_ppa * l_ppa; // car |w|=1
  Real        r2 = radius * radius;
  Real      diff = d2 - r2;
  if ( diff >= 0.0 )
    { // pas d'intersection. Point le plus proche = projection à
      // l'orthogonal de la droite et du centre de la sphere.
      ray_inter.distance     = sqrt( diff );
      ray_inter.intersection = ray.origin + l_ppa * ray.direction; // closest point
      ray_inter.reflexion    = ray_inter.intersection;
      ray_inter.refraction   = ray_inter.intersection;
      ray_inter.normal       = getNormal( ray_inter.intersection );
      return false;
    }
  // Intersection. On cherche ou:
  // std::cout << "Intersection: diff=" << diff << std::endl;
  Real delta_over_2 = sqrt( -diff );
  Real gamma1 = l_ppa - delta_over_2;
  Real gamma2 = l_ppa + delta_over_2;
  if ( gamma2 < RT_EPSILON ) // ray starts after the object
    { // no intersection
      ray_inter.distance     = 100000.0f; 
      ray_inter.intersection = ray.origin + gamma2 * ray.direction; // closest pt
      ray_inter.reflexion    = ray_inter.intersection;
      ray_inter.refraction   = ray_inter.intersection;
      return false;
    }
  if ( gamma1 < 0.0001 ) // if ray is starting from inside or from this surface
    ray_inter.intersection = ray.origin + gamma2 * ray.direction;
  else // ray is coming from the outside
    ray_inter.intersection = ray.origin + gamma1 * ray.direction;
  ray_inter.distance    = -1.0f;
  ray_inter.normal      = getNormal( ray_inter.intersection );
  ray_inter.reflexion   = ray_inter.intersection;
  ray_inter.refraction  = ray_inter.intersection;
  ray_inter.in_refractive_index  = material.in_refractive_index;
  ray_inter.out_refractive_index = material.out_refractive_index;
  return true;
}
