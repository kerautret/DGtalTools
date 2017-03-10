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
 * @file GraphicalTriangle.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include "GraphicalTriangle.h"

void
DGtal::rt::GraphicalTriangle::init( RTViewer& viewer )
{
  viewer.setFillColor( main_material.diffuse ); 
  viewer.addTriangle( A, B, C );
}

void
DGtal::rt::GraphicalTriangle::draw( RTViewer& /* viewer */ )
{
  // // Taking care of in-between poles
  // glBegin( GL_TRIANGLES );
  // glColor4fv( main_material.ambient );
  // glMaterialfv(GL_FRONT, GL_DIFFUSE, main_material.diffuse);
  // glMaterialfv(GL_FRONT, GL_SPECULAR, main_material.specular);
  // glMaterialf(GL_FRONT, GL_SHININESS, main_material.shinyness );
  // glNormal3fv( GL( N ) );
  // glVertex3fv( GL( A ) );
  // glNormal3fv( GL( N ) );
  // glVertex3fv( GL( B ) );
  // glNormal3fv( GL( N ) );
  // glVertex3fv( GL( C ) );
  // glEnd();
}

DGtal::rt::Material
DGtal::rt::GraphicalTriangle::getMaterial( Point3 p )
{
  Real x, y;
  coordinates( p, x, y );
  return ( ( x < width ) || ( y < width ) || ( ( 1.0f-x-y ) < width ) )
    ? band_material : main_material;
}

