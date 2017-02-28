/**
@file Viewer.h
*/
#pragma once

#ifndef _VIEWER_H_
#define _VIEWER_H_

#include <vector>
#include <QKeyEvent>
#include <QGLViewer/qglviewer.h>

namespace rt {
  
  /// Forward declaration of class Scene
  struct Scene;

  /// This class displays the interface for placing the camera and the
  /// lights, and the user may call the renderer from it.
  class Viewer : public QGLViewer
  {
  public:
    /// Default constructor. Scene is empty.
    Viewer() : QGLViewer(), ptrScene( 0 ), maxDepth( 6 ) {}
    
    /// Sets the scene
    void setScene( rt::Scene& aScene )
    {
      ptrScene = &aScene;
    }
    
    /// To call the protected method `drawLight`.
    void drawSomeLight( GLenum light ) const
    {
      drawLight( light );
    }
    /// To call the protected method `drawLight`.
    void drawSomeLight( GLenum light, float zoom ) const
    {
      drawLight( light, zoom );
    }

  protected :
    /// Called at each draw of the window
    virtual void draw();
    /// Called before the first draw
    virtual void init();
    /// Called when pressing help.
    virtual QString helpString() const;
    /// Celled when pressing a key.
    virtual void keyPressEvent(QKeyEvent *e);
    
    /// Stores the scene
    rt::Scene* ptrScene;

    /// Maximum depth
    int maxDepth;
  };
}

#endif
