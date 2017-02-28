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

#pragma once

/**
 * @file GraphicalObject.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(GraphicalObject_RECURSES)
#error Recursive header files inclusion detected in GraphicalObject.h
#else // defined(GraphicalObject_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GraphicalObject_RECURSES

#if !defined GraphicalObject_h
/** Prevents repeated inclusion of headers. */
#define GraphicalObject_h

#include "Viewer.h"
#include "PointVector.h"
#include "Material.h"
#include "Ray.h"

namespace DGtal {

namespace rt {

  /// This is an interface specifying methods that any ray-traced
  /// graphical object should have. It is also drawable to be seen in
  /// QGLViewer window.  Concrete exemples of a GraphicalObject
  /// include Spheres.
  struct GraphicalObject {

    /// Default constructor. Nothing to do.
    GraphicalObject() {}

    /// Virtual destructor since object contains virtual methods.
    virtual ~GraphicalObject() {}

    /// This method is called by Scene::init() at the beginning of the
    /// display in the OpenGL window. May be useful for some
    /// precomputations.
    virtual void init( Viewer& /* viewer */ ) = 0;

    /// This method is called by Scene::draw() at each frame to
    /// redisplay objects in the OpenGL window.
    virtual void draw( Viewer& /* viewer */ ) = 0;

    /// @return the normal vector at point \a p on the object (\a p
    /// should be on or close to the sphere).
    virtual Vector3 getNormal( Point3 p ) = 0;

    /// @return the material associated to this part of the object
    virtual Material getMaterial( Point3 p ) = 0;

    /// @param[in] ray the incoming ray
    /// @param[out] returns the point of intersection with the object
    /// (if any), or the closest point to it.
    ///
    /// @return either a real < 0.0 if there is an intersection, or a
    /// kind of distance to the closest point of intersection.
    virtual Real rayIntersection( const Ray& ray, Point3& p ) = 0;
                    

  };

} // namespace rt

} // namespace DGtal

#endif // #define _GRAPHICAL_OBJECT_H_
