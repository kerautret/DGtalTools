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
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/io/writers/PPMWriter.h"

using namespace std;

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
  viewer.restoreStateFromFile();
  
  // Add custom key description (see keyPressEvent).
  viewer.setKeyDescription(Qt::Key_U, "Renders the scene with a ray-tracer (low resolution)");
  viewer.setKeyDescription(Qt::SHIFT+Qt::Key_U, "Renders the scene with a ray-tracer (medium resolution)");
  viewer.setKeyDescription(Qt::CTRL+Qt::Key_U, "Renders the scene with a ray-tracer (high resolution)");
  viewer.setKeyDescription(Qt::Key_V, "Augments the max depth of ray-tracing algorithm");
  viewer.setKeyDescription(Qt::SHIFT+Qt::Key_V, "Decreases the max depth of ray-tracing algorithm");
  
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
      Vector3 origin( orig );
      Vector3 dirUL( dir );
      viewer.camera()->convertClickToLine( QPoint( w,0 ), orig, dir );
      Vector3 dirUR( dir );
      viewer.camera()->convertClickToLine( QPoint( 0, h ), orig, dir );
      Vector3 dirLL( dir );
      viewer.camera()->convertClickToLine( QPoint( w, h ), orig, dir );
      Vector3 dirLR( dir );
      renderer.setViewBox( origin, dirUL, dirUR, dirLL, dirLR );
      if ( modifiers == Qt::ShiftModifier ) { w /= 2; h /= 2; }
      else if ( modifiers == Qt::NoModifier ) { w /= 8; h /= 8; }
      DuskWithChessboard bg;
      renderer.setBackground( &bg ); 
      Domain2D domain( Point2i( 0, 0 ) , Point2i( w, h ) );
      Image2D  image ( domain );
      renderer.setResolution( w, h );
      renderer.render( image, maxDepth );
      PPMWriter<Image2D,RealColor2Color>::exportPPM( "output.ppm", image,
                                                     RealColor2Color(), false );
      handled = true;
    }
  if (e->key()==Qt::Key_V)
    {
      if ( modifiers == Qt::ShiftModifier )
        { maxDepth = std::max( 1, maxDepth - 1 ); handled = true; }
      if ( modifiers == Qt::NoModifier )
        { maxDepth = std::min( 20, maxDepth + 1 ); handled = true; }
      std::cout << "Max depth is " << maxDepth << std::endl; 
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
  return text;
}
