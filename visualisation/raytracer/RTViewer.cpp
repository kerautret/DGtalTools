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
 * @file RTViewer.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////


#include <fstream>
#include "raytracer/RTViewer.h"
#include "raytracer/Scene.h"
#include "raytracer/Renderer.h"
#include "raytracer/Background.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/io/writers/PPMWriter.h"

using namespace std;

// Draws a tetrahedron with 4 colors.
// void 
// DGtal::rt::RTViewer::draw()
// {
//   // Set up lights
//   if ( ptrScene != 0 )
//     ptrScene->light( *this );
//   // Draw all objects
//   if ( ptrScene != 0 )
//     ptrScene->draw( *this );
// }


void 
DGtal::rt::RTViewer::init()
{
  // Restore previous viewer state.
  restoreStateFromFile();

  // Add custom key description (see keyPressEvent).
  setKeyDescription(Qt::Key_R, "Renders the scene with a ray-tracer (low resolution)");
  setKeyDescription(Qt::SHIFT+Qt::Key_R, "Renders the scene with a ray-tracer (medium resolution)");
  setKeyDescription(Qt::CTRL+Qt::Key_R, "Renders the scene with a ray-tracer (high resolution)");
  setKeyDescription(Qt::Key_D, "Augments the max depth of ray-tracing algorithm");
  setKeyDescription(Qt::SHIFT+Qt::Key_D, "Decreases the max depth of ray-tracing algorithm");
  
  // Opens help window
  // help();

  // To move lights around
  // setMouseTracking(true);

  // Gives a bounding box to the camera
  camera()->setSceneBoundingBox( qglviewer::Vec( -7, -7, -2 ),qglviewer::Vec( 7, 7, 12 ) );

  Base::init();

}

void
DGtal::rt::RTViewer::keyPressEvent(QKeyEvent *e)
{
  // The type for representing a 2d rectangular domain.
  typedef DGtal::HyperRectDomain<Space2> Domain2D;
  // The type chosen for representing color images.
  typedef DGtal::ImageContainerBySTLVector<Domain2D,RealColor> Image2D;
  // Get event modifiers key
  const Qt::KeyboardModifiers modifiers = e->modifiers();
  bool handled = false;
  if ((e->key()==Qt::Key_R) && ptrScene != 0 )
    {
      int w = camera()->screenWidth();
      int h = camera()->screenHeight();
      Renderer renderer( *ptrScene );
      qglviewer::Vec orig, dir;
      camera()->convertClickToLine( QPoint( 0,0 ), orig, dir );
      Vector3 origin( orig );
      Vector3 dirUL( dir );
      camera()->convertClickToLine( QPoint( w,0 ), orig, dir );
      Vector3 dirUR( dir );
      camera()->convertClickToLine( QPoint( 0, h ), orig, dir );
      Vector3 dirLL( dir );
      camera()->convertClickToLine( QPoint( w, h ), orig, dir );
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
  if (e->key()==Qt::Key_D)
    {
      if ( modifiers == Qt::ShiftModifier )
        { maxDepth = std::max( 1, maxDepth - 1 ); handled = true; }
      if ( modifiers == Qt::NoModifier )
        { maxDepth = std::min( 20, maxDepth + 1 ); handled = true; }
      std::cout << "Max depth is " << maxDepth << std::endl; 
    }
    
  if (!handled) QGLViewer::keyPressEvent(e);
}

QString 
DGtal::rt::RTViewer::helpString() const
{
  QString text("<h2>S i m p l e V i e w e r</h2>");
  text += "Use the mouse to move the camera around the object. ";
  text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
  text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
  text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
  text += "Simply press the function key again to restore it. Several keyFrames define a ";
  text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
  text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
  text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
  text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
  text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
  text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
  text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
  text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
  text += "Press <b>Escape</b> to exit the viewer.";
  text += "Press <b>R</b> to render the scene (low resolution).";
  text += "Press <b>Shift+R</b> to render the scene (medium resolution).";
  text += "Press <b>Ctrl+R</b> to render the scene (high resolution).";
  return text;
}
