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
 * @file GraphicalPeriodicPlane.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include "raytracer/GraphicalPeriodicPlane.h"


DGtal::rt::GraphicalPeriodicPlane::GraphicalPeriodicPlane
( Point3 c, Vector3 u, Vector3 v,
  Material main_m, Material band_m, Real w)
  : PeriodicPlane( c, u, v ),
    main_material( main_m ), band_material( band_m ), width( w )
{
  faraway = Material::mix( w, main_m, band_m );
}
    
DGtal::rt::GraphicalPeriodicPlane::~GraphicalPeriodicPlane()
{}

void
DGtal::rt::GraphicalPeriodicPlane::draw( RTViewer& viewer )
{
  // Taking care of in-between poles
  glBegin( GL_QUAD_STRIP);
  glColor4fv( main_material.ambient );
  glMaterialfv(GL_FRONT, GL_DIFFUSE, main_material.diffuse);
  glMaterialfv(GL_FRONT, GL_SPECULAR, main_material.specular);
  glMaterialf(GL_FRONT, GL_SHININESS, main_material.shinyness );
  const Real factor = 50.0;
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

DGtal::rt::Material
DGtal::rt::GraphicalPeriodicPlane::getMaterial( Point3 p )
{
  Real x, y;
  coordinates( p, x, y );
  Real d2 = pow( x*x + y*y, 0.3 );
  Real att = std::max( 20.0 - d2, 0.0 ) / 20.0;

  x -= floor( x );
  y -= floor( y );

  return ( ( x < width ) || ( y < width ) )
    ? Material::mix( att, faraway, band_material ) 
    : Material::mix( att, faraway, main_material );
}

