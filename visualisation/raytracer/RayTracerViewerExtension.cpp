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
 * @file RayTracerViewerExtension.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////


#include <fstream>
#include "raytracer/RayTracerViewerExtension.h"
#include "raytracer/Scene.h"
#include "raytracer/Renderer.h"
#include "raytracer/Background.h"
#include "raytracer/DigitalVolume.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/io/writers/PPMWriter.h"

using namespace std;

void 
DGtal::rt::RayTracerViewerExtension::setDigitalVolumeMode
( bool estimatedNormals, bool Phong, bool smoothed )
{
  typedef HyperRectDomain< Space3 >                 Domain;
  typedef ImageContainerBySTLVector< Domain, bool > BooleanImage;
  typedef DigitalVolume< BooleanImage >             Volume;
  if ( ptrScene != 0 )
    {
      for ( GraphicalObject* obj : ptrScene->myObjects )
        {
          Volume* vol = dynamic_cast<Volume*>( obj );
          if ( vol != 0 )
            {
              std::cout << "- Volume " << vol << ": setting mode "
                        << (estimatedNormals ? "EstimatedNormals" : "TrivialNormals" )
                        << "/" << ( Phong ? "PhongShading" : "FlatShading" )
                        << "/" << ( smoothed ? "ShiftedGeometry" : "RawGeometry" )
                        << std::endl;
              vol->setMode( estimatedNormals, Phong, smoothed );
            }
        }
    }
}

void 
DGtal::rt::RayTracerViewerExtension::draw( Viewer& viewer ) 
{
  // Set up lights
  if ( ptrScene != 0 )
    ptrScene->light( viewer );
  // Draw all objects
  if ( ptrScene != 0 )
    ptrScene->draw( viewer );
}


void 
DGtal::rt::RayTracerViewerExtension::init( Viewer& viewer ) 
{
  // Restore previous viewer state.
  // viewer.restoreStateFromFile();
  
  // Add custom key description (see keyPressEvent).
  viewer.setKeyDescription(Qt::Key_U, "Renders the scene with a ray-tracer (low resolution)");
  viewer.setKeyDescription(Qt::SHIFT+Qt::Key_U, "Renders the scene with standard ray-tracer (high resolution)");
  viewer.setKeyDescription(Qt::CTRL+Qt::Key_U, "Renders the scene with a random ray-tracer (high resolution)");
  viewer.setKeyDescription(Qt::Key_V, "Augments the max depth of ray-tracing algorithm");
  viewer.setKeyDescription(Qt::SHIFT+Qt::Key_V, "Decreases the max depth of ray-tracing algorithm");
  viewer.setKeyDescription(Qt::Key_1, "Chooses flat shading, digital geometry and trivial normals");
  viewer.setKeyDescription(Qt::Key_2, "Chooses Phong shading, digital geometry and trivial normals");
  viewer.setKeyDescription(Qt::Key_3, "Chooses flat shading, offset geometry and estimated normals");
  viewer.setKeyDescription(Qt::Key_4, "Chooses Phong shading, offset geometry and estimated normals");
  viewer.setKeyDescription(Qt::Key_9, "Increases the number of random samples per pixel (random rendering)");
  viewer.setKeyDescription(Qt::CTRL+Qt::Key_9, "Decreases the number of random samples per pixel (random rendering)");
  viewer.setKeyDescription(Qt::Key_9, "Increases the number of ray-casts per random sample (random rendering)");
  viewer.setKeyDescription(Qt::CTRL+Qt::Key_9, "Decreases the number of ray-casts per random sample (random rendering)");
  
  // Opens help window
  // help();
  // To move lights around
  // setMouseTracking(true);
  // Gives a bounding box to the camera
  //viewer.camera()->setSceneBoundingBox( qglviewer::Vec( -7, -7, -2 ),qglviewer::Vec( 7, 7, 12 ) );
}

bool
DGtal::rt::RayTracerViewerExtension::keyPressEvent( Viewer& viewer, QKeyEvent *e)
{
  // The type for representing a 2d rectangular domain.
  typedef DGtal::HyperRectDomain<Space2> Domain2D;
  // The type chosen for representing color images.
  typedef DGtal::ImageContainerBySTLVector<Domain2D,RealColor> Image2D;
  // Get event modifiers key
  const Qt::KeyboardModifiers modifiers = e->modifiers();
  bool handled = false;
  if ((e->key()==Qt::Key_U) && ptrScene != 0 )
    {
      int w = viewer.camera()->screenWidth();
      int h = viewer.camera()->screenHeight();
      Renderer renderer( *ptrScene );
      qglviewer::Vec orig, dir;
      viewer.camera()->convertClickToLine( QPoint( 0,0 ), orig, dir );
      Vector3 origin( toPoint3( orig ) );
      Vector3 dirUL( toPoint3( dir ) );
      viewer.camera()->convertClickToLine( QPoint( w,0 ), orig, dir );
      Vector3 dirUR( toPoint3( dir ) );
      viewer.camera()->convertClickToLine( QPoint( 0, h ), orig, dir );
      Vector3 dirLL( toPoint3( dir ) );
      viewer.camera()->convertClickToLine( QPoint( w, h ), orig, dir );
      Vector3 dirLR( toPoint3( dir ) );
      renderer.setViewBox( origin, dirUL, dirUR, dirLL, dirLR );
      if ( modifiers == Qt::NoModifier ) { w /= 4; h /= 4; }
      DuskWithChessboard bg;
      renderer.setBackground( &bg ); 
      Domain2D domain( Point2i( 0, 0 ) , Point2i( w, h ) );
      Image2D  image ( domain );
      renderer.setResolution( w, h );
      if ( modifiers == Qt::ShiftModifier )
        {
          if ( nbSamples <= 1 )
            renderer.render( image, maxDepth );
          else
            renderer.renderAntiAliased( image, maxDepth, nbSamples );
        }
      else
        renderer.renderRandom( image, maxDepth, nbSamples, nbCasts );
      PPMWriter<Image2D,RealColor2Color>::exportPPM( "output.ppm", image,
                                                     RealColor2Color(), false );
      handled = true;
    }
  if (e->key()==Qt::Key_V)
    {
      if ( modifiers == Qt::ShiftModifier )
        { maxDepth = std::max( 1, maxDepth - 1 ); handled = true; }
      if ( modifiers == Qt::NoModifier )
        { maxDepth = std::min( 40, maxDepth + 1 ); handled = true; }
      std::cout << "Max depth is " << maxDepth << std::endl; 
    }
  if (e->key()==Qt::Key_1)
    { setDigitalVolumeMode( false, false, false ); handled = true; }
  if (e->key()==Qt::Key_2)
    { setDigitalVolumeMode( false, true,  false ); handled = true; }
  if (e->key()==Qt::Key_3)
    { setDigitalVolumeMode( true,  false, true  ); handled = true; }
  if (e->key()==Qt::Key_4)
    { setDigitalVolumeMode( true,  true,  true  ); handled = true; }
  if (e->key()==Qt::Key_9 || e->key()==Qt::Key_0)
    {
      if (e->key()==Qt::Key_9)
        nbSamples = ( modifiers != Qt::ShiftModifier ) ?
          (int) ceil( nbSamples * 1.2 ) : (int) ceil( nbSamples * 0.5 );
      if (e->key()==Qt::Key_0)
        nbCasts   = ( modifiers != Qt::ShiftModifier ) ?
          (int) ceil( nbCasts * 1.2 ) : (int) ceil( nbCasts * 0.5 );
      handled = true;
      std::cout << "Nb samples=" << nbSamples
                << " Nb casts="   << nbCasts << std::endl; 
    }
  return handled;
}

QString 
DGtal::rt::RayTracerViewerExtension::helpString( const Viewer& viewer )  const
{
  QString text( "<h2>Ray-tracer rendering</h2>" );
  text += "Press <b>U</b> to render the scene (low resolution).";
  text += "Press <b>Shift+U</b> to render the scene (medium resolution).";
  text += "Press <b>Ctrl+U</b> to render the scene (high resolution).";
  text += "Press <b>V</b> and <b>Shift+V</b> to change the maximal depth of raytracing.";
  text += "Press <b>1</b>, <b>2</b>, <b>3</b>, or <b>4</b> to change rendering mode of digital volumes.";
  return text;
}
