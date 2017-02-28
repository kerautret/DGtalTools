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
 * @file 3dVolRayTracer.cpp
 * @ingroup Tools
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * A tool file named 3dVolRayTracer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <qapplication.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include "raytracer/Viewer.h"
#include "raytracer/Scene.h"
#include "raytracer/Sphere.h"
#include "raytracer/Material.h"
#include "raytracer/PointLight.h"
#include "raytracer/PeriodicPlane.h"
#include "raytracer/WaterPlane.h"
#include "raytracer/Triangle.h"

using namespace std;
using namespace DGtal;

void addBubble( Scene& scene, Point3 c, Real r, Material transp_m )
{
  Material revert_m = transp_m;
  std::swap( revert_m.in_refractive_index, revert_m.out_refractive_index );
  Sphere* sphere_out = new Sphere( c, r, transp_m );
  Sphere* sphere_in  = new Sphere( c, r-0.02f, revert_m );
  scene.addObject( sphere_out );
  scene.addObject( sphere_in );
}

void groundBlackAndGrey( Scene& scene, Point3 c )
{
  PeriodicPlane* pplane =
    new PeriodicPlane( c, Vector3( 7, 0, 0 ), Vector3( 0, 7, 0 ),
                       Material::greyMatter(), Material::blackMatter(), 0.05f );
  scene.addObject( pplane );
}
void groundWhite( Scene& scene )
{
  PeriodicPlane* pplane =
    new PeriodicPlane( Point3( 0, 0, 0 ), Vector3( 5, 0, 0 ), Vector3( 0, 5, 0 ),
                       Material::whitePlastic(), Material::whitePlastic(), 0.00f );
  scene.addObject( pplane );
}
void groundWhiteAndBlack( Scene& scene, Real depth )
{
  PeriodicPlane* pplane =
    new PeriodicPlane( Point3( 0, 0, -depth ), Vector3( 5, 0, 0 ), Vector3( 0, 5, 0 ),
                       Material::whitePlastic(), Material::blackMatter(), 0.05f );
  scene.addObject( pplane );
}
void shallowSand( Scene& scene, Real depth )
{
  PeriodicPlane* pplane =
    new PeriodicPlane( Point3( 0, 0, -depth ), Vector3( 1, 0, 0 ), Vector3( 0, 1, 0 ),
                       Material::sand(), Material::sand(), 0.00f );
  scene.addObject( pplane );
}

void water( Scene& scene, Point3 p )
{
  WaterPlane* pplane =
    new WaterPlane( p, Material::blueWater() );
  scene.addObject( pplane );
}

void backBuilding( Scene& scene, Real d )
{
  PeriodicPlane* plane =
    new PeriodicPlane( Point3( 0, d, 0 ), Vector3( -2, 0, 0 ), Vector3( 0, 0, 4 ),
                       Material::mirror(), Material::blackMatter(), 0.025f );
  scene.addObject( plane );
}
void frontBuilding( Scene& scene, Real d )
{
  PeriodicPlane* plane =
    new PeriodicPlane( Point3( 0, -d, 0 ), Vector3( 2, 0, 0 ), Vector3( 0, 0, 4 ),
                       Material::mirror(), Material::blackMatter(), 0.025f );
  scene.addObject( plane );
}

void rightBuilding( Scene& scene, Real d )
{
  PeriodicPlane* plane =
    new PeriodicPlane( Point3( d, 0, 0 ), Vector3( 0, -2, 0 ), Vector3( 0, 0, 4 ),
                       Material::mirror(), Material::blackMatter(), 0.025f );
  scene.addObject( plane );
}

void leftBuilding( Scene& scene, Real d )
{
  PeriodicPlane* pplane =
    new PeriodicPlane( Point3( -d, 0, 0 ), Vector3( 0, 2, 0 ), Vector3( 0, 0, 4 ),
                       Material::mirror(), Material::blackMatter(), 0.025f );
  scene.addObject( pplane );
}

void pyramid( Scene& scene, Point3 C, Real side, Material main, Material band, Real w )
{
  Point3 A1 = C + Point3( -side/2.0f, -side/2.0f, 0.0f );
  Point3 A2 = C + Point3(  side/2.0f, -side/2.0f, 0.0f );
  Point3 A3 = C + Point3(  side/2.0f,  side/2.0f, 0.0f );
  Point3 A4 = C + Point3( -side/2.0f,  side/2.0f, 0.0f );
  Point3 T  = C + Point3(       0.0f,       0.0f, sqrt(2.0f)*side/2.0f );
  scene.addObject( new Triangle( A1, A2, T, main, band, w ) );
  scene.addObject( new Triangle( A2, A3, T, main, band, w ) );
  scene.addObject( new Triangle( A3, A4, T, main, band, w ) );
  scene.addObject( new Triangle( A4, A1, T, main, band, w ) );
}

int main(int argc, char** argv)
{
  // Read command lines arguments.
  QApplication application(argc,argv);
  
  // Creates a 3D scene
  Scene scene;
  
  // Light at infinity
  Light* light0 = new PointLight( GL_LIGHT0, Point4( 0,0,1,0 ),
                                      Color( 1.0, 1.0, 1.0 ) );
  // Light* light1 = new PointLight( GL_LIGHT1, Point4( -1,-1,11,1 ),
  //                                 Color( 1.0, 1.0, 1.0 ) );
  scene.addLight( light0 );
  // scene.addLight( light1 );

  shallowSand( scene, 1.0f );
  // groundWhiteAndBlack( scene, 0.0f );
  // groundBlackAndGrey( scene, Point3( 0, 0, 0 ) );
  // leftBuilding( scene, 10.0 );
  water( scene, Point3( 0, 0, 0 ) );

  // Objects
  // Sphere* sphere1 = new Sphere( Point3( 0, 0, 3), 3.0, Material::mirror() );
  Sphere* sphere1 = new Sphere( Point3( 0, 0, 0), 2.0, Material::bronze() );
  // Sphere* sphere2 = new Sphere( Point3( 0, 4, 0.5), 1.0, Material::emerald() );
  // Sphere* sphere3 = new Sphere( Point3( 6, 6, 0), 3.0, Material::whitePlastic() );
  // Sphere* sphere4 = new Sphere( Point3( 5, 0, 0), 3.0, Material::bronze() );
  scene.addObject( sphere1 );
  // scene.addObject( sphere2 );
  // scene.addObject( sphere3 );
  // scene.addObject( sphere4 );
  // pyramid( scene, Point3( -5, -5, -1 ), 6.0f, Material::ruby(), Material::blackMatter(), 0.025f );
  // pyramid( scene, Point3( -5, -5, -1 ), 5.0f, Material::mirror(), Material::mirror(), 0.00f );
  // pyramid( scene, Point3( 30, 40, -3 ), 25.0f, Material::emerald(), Material::blackMatter(), 0.025f );
  // pyramid( scene, Point3( 30, 40, -3 ), 23.0f, Material::mirror(), Material::mirror(), 0.00f );

  // addBubble( scene, Point3( -3, 4, 8 ), 4.0, Material::mirror() );

  // Instantiate the viewer.
  Viewer viewer;
  // Give a name
  viewer.setWindowTitle("Ray-tracer preview");

  // Sets the scene
  viewer.setScene( scene );

  // Make the viewer window visible on screen.
  viewer.show();
  // Run main loop.
  application.exec();
  return 0;
}
