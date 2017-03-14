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

#include <DGtal/io/readers/VolReader.h>
#include <DGtal/images/ImageContainerBySTLVector.h>
#include <DGtal/images/SimpleThresholdForegroundPredicate.h>

#include "raytracer/RayTracerViewerExtension.h"
#include "raytracer/Scene.h"
#include "raytracer/Sphere.h"
#include "raytracer/Material.h"
#include "raytracer/PointLight.h"
#include "raytracer/GraphicalPeriodicPlane.h"
#include "raytracer/WaterPlane.h"
#include "raytracer/GraphicalTriangle.h"
#include "raytracer/GraphicalParallelogram.h"
#include "raytracer/GraphicalDigitalVolume.h"

using namespace std;
using namespace DGtal;
using namespace DGtal::rt;

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
  GraphicalPeriodicPlane* pplane =
    new GraphicalPeriodicPlane( c, Vector3( 7, 0, 0 ), Vector3( 0, 7, 0 ),
                       Material::greyMatter(), Material::blackMatter(), 0.05f );
  scene.addObject( pplane );
}
void groundWhite( Scene& scene )
{
  GraphicalPeriodicPlane* pplane =
    new GraphicalPeriodicPlane( Point3( 0, 0, 0 ), Vector3( 5, 0, 0 ), Vector3( 0, 5, 0 ),
                       Material::whitePlastic(), Material::whitePlastic(), 0.00f );
  scene.addObject( pplane );
}
void groundWhiteAndBlack( Scene& scene, Real depth )
{
  GraphicalPeriodicPlane* pplane =
    new GraphicalPeriodicPlane( Point3( 0, 0, -depth ), Vector3( 5, 0, 0 ), Vector3( 0, 5, 0 ),
                       Material::whitePlastic(), Material::blackMatter(), 0.05f );
  scene.addObject( pplane );
}
void shallowSand( Scene& scene, Real depth )
{
  GraphicalPeriodicPlane* pplane =
    new GraphicalPeriodicPlane( Point3( 0, 0, -depth ), Vector3( 1, 0, 0 ), Vector3( 0, 1, 0 ),
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
  GraphicalPeriodicPlane* plane =
    new GraphicalPeriodicPlane( Point3( 0, d, 0 ), Vector3( -2, 0, 0 ), Vector3( 0, 0, 4 ),
                       Material::mirror(), Material::blackMatter(), 0.025f );
  scene.addObject( plane );
}
void frontBuilding( Scene& scene, Real d )
{
  GraphicalPeriodicPlane* plane =
    new GraphicalPeriodicPlane( Point3( 0, -d, 0 ), Vector3( 2, 0, 0 ), Vector3( 0, 0, 4 ),
                       Material::mirror(), Material::blackMatter(), 0.025f );
  scene.addObject( plane );
}

void rightBuilding( Scene& scene, Real d )
{
  GraphicalPeriodicPlane* plane =
    new GraphicalPeriodicPlane( Point3( d, 0, 0 ), Vector3( 0, -2, 0 ), Vector3( 0, 0, 4 ),
                       Material::mirror(), Material::blackMatter(), 0.025f );
  scene.addObject( plane );
}

void leftBuilding( Scene& scene, Real d )
{
  GraphicalPeriodicPlane* pplane =
    new GraphicalPeriodicPlane( Point3( -d, 0, 0 ), Vector3( 0, 2, 0 ), Vector3( 0, 0, 4 ),
                       Material::mirror(), Material::blackMatter(), 0.025f );
  scene.addObject( pplane );
}

void cube( Scene& scene, Point3 C, Real side, Material main, Material band, Real w )
{
  Point3 A000 = C + Point3( -side/2.0f, -side/2.0f, -side/2.0f );
  Point3 A001 = C + Point3( -side/2.0f, -side/2.0f,  side/2.0f );
  Point3 A010 = C + Point3( -side/2.0f,  side/2.0f, -side/2.0f );
  Point3 A011 = C + Point3( -side/2.0f,  side/2.0f,  side/2.0f );
  Point3 A100 = C + Point3(  side/2.0f, -side/2.0f, -side/2.0f );
  Point3 A101 = C + Point3(  side/2.0f, -side/2.0f,  side/2.0f );
  Point3 A110 = C + Point3(  side/2.0f,  side/2.0f, -side/2.0f );
  Point3 A111 = C + Point3(  side/2.0f,  side/2.0f,  side/2.0f );
  scene.addObject( new GraphicalParallelogram( A000, A010, A100, main, band, w ) );
  scene.addObject( new GraphicalParallelogram( A000, A001, A010, main, band, w ) );
  scene.addObject( new GraphicalParallelogram( A000, A100, A001, main, band, w ) );
  scene.addObject( new GraphicalParallelogram( A111, A011, A101, main, band, w ) );
  scene.addObject( new GraphicalParallelogram( A111, A101, A110, main, band, w ) );
  scene.addObject( new GraphicalParallelogram( A111, A110, A011, main, band, w ) );
}

Material string2material( const std::string& s )
{
  if      ( s == "bronze"   ) return Material::bronze();
  else if ( s == "emerald"  ) return Material::emerald();
  else if ( s == "ruby"     ) return Material::ruby();
  else if ( s == "mirror"   ) return Material::mirror();
  else if ( s == "wPlastic" ) return Material::whitePlastic();
  else if ( s == "rPlastic" ) return Material::redPlastic();
  else if ( s == "glass"    ) return Material::glass();
  else if ( s == "bMatter"  ) return Material::blackMatter();
  else if ( s == "gMatter"  ) return Material::greyMatter();
  else if ( s == "sand"     ) return Material::sand();
  else if ( s == "bWater"   ) return Material::blueWater();
  else if ( s == "steel"    ) return Material::steel();
  else if ( s == "gMetallic") return Material::greyMettalic();
  else return Material::whitePlastic();
}
void pyramid( Scene& scene, Point3 C, Real side, Material main, Material band, Real w )
{
  Point3 A1 = C + Point3( -side/2.0f, -side/2.0f, 0.0f );
  Point3 A2 = C + Point3(  side/2.0f, -side/2.0f, 0.0f );
  Point3 A3 = C + Point3(  side/2.0f,  side/2.0f, 0.0f );
  Point3 A4 = C + Point3( -side/2.0f,  side/2.0f, 0.0f );
  Point3 T  = C + Point3(       0.0f,       0.0f, sqrt(2.0f)*side/2.0f );
  scene.addObject( new GraphicalTriangle( A1, A2, T, main, band, w ) );
  scene.addObject( new GraphicalTriangle( A2, A3, T, main, band, w ) );
  scene.addObject( new GraphicalTriangle( A3, A4, T, main, band, w ) );
  scene.addObject( new GraphicalTriangle( A4, A1, T, main, band, w ) );
}

int main(int argc, char** argv)
{
  // Read command lines arguments.
  QApplication application(argc,argv);
  
  // Creates a 3D scene
  Scene scene;
  
  // Light at infinity
  Light* light0   = new PointLight( GL_LIGHT0, Vector4( 0, 0, 1, 0),
                                    RealColor( 1.0, 1.0, 1.0 ) );
  Light* light1   = new PointLight( GL_LIGHT1, Vector4( -50,-50,50,1),
                                    RealColor( 0.8, 0.8, 0.8 ) );
  Light* light2   = new PointLight( GL_LIGHT2, Vector4( -50,-50,40,1),
                                    RealColor( 0.7, 0.7, 0.7 ) );
  scene.addLight( light0 );
  scene.addLight( light1 );
  scene.addLight( light2 );

  if ( argc > 1 )
    {
      std::string  inputFilename = argv[ 1 ];
      unsigned int threshold     = argc > 2 ? atoi( argv[ 2 ] ) : 1;
      std::string  material      = argc > 3 ? argv[ 3 ] : "rPlastic";
      trace.beginBlock( "Reading vol file into an image." );
      typedef HyperRectDomain< Space3 >                 Domain;
      typedef ImageContainerBySTLVector< Domain, int >  Image;
      typedef ImageContainerBySTLVector< Domain, bool > BooleanImage;
      typedef functors::SimpleThresholdForegroundPredicate<Image> ThresholdedImage;
      typedef GraphicalDigitalVolume< BooleanImage >    Volume;
      Image image = VolReader<Image>::importVol(inputFilename);
      ThresholdedImage thresholdedImage( image, threshold );
      trace.endBlock();
      trace.beginBlock( "Making binary image and building digital volume." );
      BooleanImage bimage( image.domain() );
      for ( auto p : bimage.domain() )
        bimage.setValue( p, thresholdedImage( p ) );
      Volume* vol = new Volume( bimage, string2material( material ) );
      scene.addObject( vol );
      trace.endBlock();
    }

  
  // shallowSand( scene, 1.0f );
  groundWhiteAndBlack( scene, 0.0f );
  // groundBlackAndGrey( scene, Point3( 0, 0, 0 ) );
  // leftBuilding( scene, 10.0 );
  // water( scene, Point3( 0, 0, -2.0f ) );

  // Objects
  Sphere* sphere0 = new Sphere( Point3( -600, 200, 800), 800.0, Material::emerald() );
  Sphere* sphere1 = new Sphere( Point3( -20, 0, 40), 40.0, Material::bronze() );
  // Sphere* sphere2 = new Sphere( Point3( 0, 4, 0.5), 1.0, Material::emerald() );
  // Sphere* sphere3 = new Sphere( Point3( 6, 6, 0), 3.0, Material::whitePlastic() );
  // Sphere* sphere4 = new Sphere( Point3( 5, 0, 0), 3.0, Material::bronze() );
  scene.addObject( sphere0 );
  scene.addObject( sphere1 );
  // scene.addObject( sphere2 );
  // scene.addObject( sphere3 );
  // scene.addObject( sphere4 );
  // cube( scene, Point3( -5, -5, -1 ), 6.0f, Material::ruby(), Material::blackMatter(), 0.025f );
  // cube( scene, Point3( -5, -5, -1 ), 5.0f, Material::mirror(), Material::mirror(), 0.00f );
  // pyramid( scene, Point3( -5, -5, -1 ), 6.0f, Material::ruby(), Material::blackMatter(), 0.025f );
  // pyramid( scene, Point3( -5, -5, -1 ), 5.0f, Material::mirror(), Material::mirror(), 0.00f );
  // pyramid( scene, Point3( 0, 0, -3 ), 20.0f, Material::glass(), Material::blackMatter(), 0.025f );
  // pyramid( scene, Point3( 0, 0, -3 ), 19.5f, Material::glass().revert(), Material::mirror(), 0.00f );

  // addBubble( scene, Point3( 0, 0, -3+10.0*sqrt(2.0) ), 2.0, Material::mirror() );
  // addBubble( scene, Point3( 10.0, 10.0, -3 ), 2.0, Material::mirror() );
  // addBubble( scene, Point3( -10.0, 10.0, -3 ), 2.0, Material::mirror() );
  // addBubble( scene, Point3( -10.0, -10.0, -3 ), 2.0, Material::mirror() );
  // addBubble( scene, Point3( 10.0, -10.0, -3 ), 2.0, Material::mirror() );

  // Instantiate the viewer.
  typedef Viewer3D<Space3,KSpace3> Viewer;
  KSpace3 K;
  K.init( Point3::diagonal( -100 ),Point3::diagonal( 100 ), true );
  Viewer viewer( K );
  viewer.setExtension( new RayTracerViewerExtension( scene ) );
  // Give a name
  viewer.setWindowTitle("Ray-tracer preview");

  // Make the viewer window visible on screen.
  viewer.show();

  // Must be done after viewer.show() !
  scene.init( viewer );

  // Forces update
  viewer << Viewer::updateDisplay;

  // Run main loop.
  application.exec();
  return 0;
}
