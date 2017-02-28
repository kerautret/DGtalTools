/**
@file Viewer.cpp
@author JOL
*/
#include <fstream>
#include "Viewer.h"
#include "Scene.h"
#include "Renderer.h"
#include "Image2D.h"
#include "Image2DWriter.h"
#include "Image2DReader.h"
#include "Background.h"

using namespace std;

// Draws a tetrahedron with 4 colors.
void 
rt::Viewer::draw()
{
  // Set up lights
  if ( ptrScene != 0 )
    ptrScene->light( *this );
  // Draw all objects
  if ( ptrScene != 0 )
    ptrScene->draw( *this );
}


void 
rt::Viewer::init()
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
  help();

  // To move lights around
  setMouseTracking(true);

  // Inits the scene
  if ( ptrScene != 0 )
    ptrScene->init( *this );
  
  // Gives a bounding box to the camera
  camera()->setSceneBoundingBox( qglviewer::Vec( -7, -7, -2 ),qglviewer::Vec( 7, 7, 12 ) );

}

void
rt::Viewer::keyPressEvent(QKeyEvent *e)
{
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
      Image2D<Color> sky;
      ifstream input( "fisheye-sky-2.ppm" );
      bool ok = Image2DReader<Color>::read( sky, input );
      DuskWithChessboard bg;
      ImageSkyBackground bgsky;
      if ( ok )
        {
          bgsky.setSkyImage( sky );
          renderer.setBackground( &bgsky );
        }
      else renderer.setBackground( &bg ); 
      Image2D<Color> image( w, h );
      renderer.setResolution( image.w(), image.h() );
      renderer.render( image, maxDepth );
      ofstream output( "output.ppm" );
      Image2DWriter<Color>::write( image, output, true );
      output.close();
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
rt::Viewer::helpString() const
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
