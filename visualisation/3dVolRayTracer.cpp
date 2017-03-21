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

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <DGtal/io/readers/VolReader.h>
#include <DGtal/images/ImageContainerBySTLVector.h>
#include <DGtal/images/IntervalForegroundPredicate.h>
#include <DGtal/topology/SetOfSurfels.h>
#include <DGtal/topology/DigitalSurface.h>
#include <DGtal/topology/helpers/Surfaces.h>

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
#include "raytracer/NormalEstimation.h"

using namespace std;
using namespace DGtal;
using namespace DGtal::rt;
namespace po = boost::program_options;

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
void groundWhiteAndBlack( Scene& scene, Real depth, Real size = 5.0 )
{
  GraphicalPeriodicPlane* pplane =
    new GraphicalPeriodicPlane( Point3( 0, 0, -depth ), Vector3( size, 0, 0 ), Vector3( 0, size, 0 ),
                       Material::whitePlastic(), Material::blackMatter(), 0.05 );
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


///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  // Read command lines arguments.
  QApplication application(argc,argv);

  // parse command line ----------------------------------------------
  namespace po = boost::program_options;
  po::options_description general_opt("Allowed options are: ");
  general_opt.add_options()
    ("help,h", "display this message")
    ("input,i", po::value<std::string>(), "vol file (.vol) , pgm3d (.p3d or .pgm3d, pgm (with 3 dims)) file or sdp (sequence of discrete points)" )
    ("material,w", po::value<std::string>()->default_value( "rPlastic" ), "the material chosen for the volume." )
    ("thresholdMin,m",  po::value<int>()->default_value(0), "threshold min (excluded) to define binary shape" )
    ("thresholdMax,M",  po::value<int>()->default_value(255), "threshold max (included) to define binary shape" )
    ("gridstep,g", po::value< double >()->default_value( 1.0 ), "the gridstep that defines the digitization (often called h). " )
    ("estimator,e", po::value<string>()->default_value( "True" ), "the chosen normal estimator: True | VCM | II | Trivial" )
    ("R-radius,R", po::value<double>()->default_value( 5 ), "the constant for parameter R in R(h)=R h^alpha (VCM)." )
    ("r-radius,r", po::value<double>()->default_value( 3 ), "the constant for parameter r in r(h)=r h^alpha (VCM,II,Trivial)." )
    ("kernel,k", po::value<string>()->default_value( "hat" ), "the function chi_r, either hat or ball." )
    ("alpha", po::value<double>()->default_value( 0.0 ), "the parameter alpha in r(h)=r h^alpha (VCM)." )
    ("trivial-radius,t", po::value<double>()->default_value( 3 ), "the parameter t defining the radius for the Trivial estimator. Also used for reorienting the VCM." )
    ("embedding,E", po::value<int>()->default_value( 0 ), "the surfel -> point embedding for VCM estimator: 0: Pointels, 1: InnerSpel, 2: OuterSpel." )
    ("scale,s", po::value<double>()->default_value( 1.0 ), "the scale for the ground.")
    ("lights,l", po::value<int>()->default_value( 1 ), "the number of additional lights.")
    ("display,d", po::value<int>()->default_value( 0 ), "the display mode, obtained by sumation: flat shading(0)/Phong shading(1), digital geometry(0)/shifted geometry(2)")
    ("positions,p", po::value<int>()->default_value( 0 ), "the number of iterations for interpolating positions.")
    ;
  bool parseOK=true;
  po::variables_map vm;
  try{
    po::store(po::parse_command_line(argc, argv, general_opt), vm);
  }catch(const exception& ex){
    parseOK=false;
    cerr << "Error checking program options: "<< ex.what()<< endl;
  }
  po::notify(vm);
  if( ! parseOK || vm.count("help") )
    {
      cerr << "Usage: " << argv[0] << " -i <volume.vol> [options]\n"
           << "Displays then may compute a 3D raytracer rendering of the volume given as input. You may choose several normal estimators (II|VCM|Trivial|True), specified with -e."
           << endl
           << general_opt << "\n";
      cerr << "Example of use:\n"
           << "./3dVolRayTracer -i \"fandisk-128.vol\" -e VCM -R 3 -r 3 -t 2 -E 0" << endl << endl;
        return 0;
    }
  
  // Creates a 3D scene
  Scene scene;

  // Get scene parameters
  int nb_lights   = vm["lights"].as<int>();
  double scale    = vm["scale"].as<double>();
  // Light at infinity
  scene.addLight( new PointLight( GL_LIGHT0, Vector4( 0, 0, 1, 0),
                                  RealColor( 1.0, 1.0, 1.0 ) ) );
  
  if ( nb_lights > 0 )
    scene.addLight( new PointLight( GL_LIGHT1, Vector4( 0,0,10,1),
                                    RealColor( 0.8, 0.8, 0.8 ) ) );
  if ( nb_lights > 1 )
    scene.addLight( new PointLight( GL_LIGHT2, Vector4( 0,0,10,1),
                                    RealColor( 0.7, 0.7, 0.7 ) ) );

  if ( vm.count( "input" ) )
    {
      string inputFilename = vm["input"].as<string>();
      int thresholdMin     = vm["thresholdMin"].as<int>();
      int thresholdMax     = vm["thresholdMax"].as<int>();
      string estimator     = vm["estimator"].as<string>();
      std::string  material= vm["material"].as<string>();
      int     displayMode  = vm["display"].as<int>();
      int     nbIterations = vm["positions"].as<int>();
      trace.beginBlock( "Reading vol file into an image." );
      typedef HyperRectDomain< Space3 >                 Domain;
      typedef ImageContainerBySTLVector< Domain, int >  Image;
      typedef ImageContainerBySTLVector< Domain, bool > BooleanImage;
      typedef functors::IntervalForegroundPredicate<Image> ThresholdedImage;
      typedef GraphicalDigitalVolume< BooleanImage >    Volume;
      Image image = VolReader<Image>::importVol(inputFilename);
      ThresholdedImage thresholdedImage( image, thresholdMin, thresholdMax );
      trace.endBlock();
      trace.beginBlock( "Making binary image and building digital volume." );
      BooleanImage bimage( image.domain() );
      for ( auto p : bimage.domain() )
        bimage.setValue( p, thresholdedImage( p ) );
      Volume* vol = new Volume( bimage, string2material( material ) );
      // typedef KSpace3::SCellSet                SCellSet;
      // typedef SetOfSurfels<KSpace3, SCellSet>  SurfaceStorage;
      // typedef DigitalSurface< SurfaceStorage > Surface;
      // SCellSet boundary;
      // Surfaces<KSpace3>::sMakeBoundary( boundary, vol->space(), vol->image(), 
      //                                   vol->space().lowerBound(), vol->space().upperBound() );
      // SurfaceStorage surface_storage( vol->space(), true, boundary );
      // Surface surface( surface_storage );
      if ( estimator == "VCM" || estimator == "II" )
        {
          chooseKernel( vm, vol->space(), vol->surface(), vol->image(), vol->normals );
          trace.beginBlock( "Interpolating normals vector field." );
          vol->interpolateVectorField( vol->normals, true );
          trace.endBlock();
        }
      trace.beginBlock( "Interpolating positions." );
      vol->computePositions( nbIterations );
      trace.endBlock();
      vol->setMode( vol->interpolatedEstimatedNormals, // estimated normals if specified
                    displayMode & 1, // Phong or flat shading
                    displayMode & 2  // Shifted or digital surface
                    );
      // else
      //   {
      //     trace.beginBlock( "Computing Trivial normals." );
      //     for ( auto surfel : vol->surface() )
      //       vol->normals[ surfel ] = vol->trivialNormal( surfel );
      //     trace.endBlock();
      //   }
      // Force interpolation of vector field
      trace.info() << "- Volume has interpolated normal: "
                   << ( vol->interpolatedEstimatedNormals ? "Yes" : "No" ) << std::endl;
      scene.addObject( vol );
      trace.endBlock();
    }

  
  // shallowSand( scene, 1.0f );
  
  groundWhiteAndBlack( scene, 0.0f, 40.0f*scale );
  // groundBlackAndGrey( scene, Point3( 0, 0, 0 ) );
  // leftBuilding( scene, 10.0 );
  // water( scene, Point3( 0, 0, 8.0f ) );

  // Objects
  // Sphere* sphere0 = new Sphere( Point3( -600, 200, 800), 800.0, Material::emerald() );
  Sphere* sphere1 = new Sphere( Point3( -40.0*scale, 0, 40.0*scale), 40.0*scale, Material::mirror() );
  // Sphere* sphere2 = new Sphere( Point3( 0, 4, 0.5), 10.0*scale, Material::glass() );
  // Sphere* sphere3 = new Sphere( Point3( 6, 6, 0), 3.0, Material::whitePlastic() );
  // Sphere* sphere4 = new Sphere( Point3( 5, 0, 0), 3.0, Material::bronze() );
  //scene.addObject( sphere0 );
  scene.addObject( sphere1 );
  // scene.addObject( sphere2 );
  // scene.addObject( sphere3 );
  // scene.addObject( sphere4 );
  // cube( scene, Point3( 10, -10, -1 ), 6.0f, Material::ruby(), Material::blackMatter(), 0.025f );
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
