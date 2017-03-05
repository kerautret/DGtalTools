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
 * @file Light.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(Light_RECURSES)
#error Recursive header files inclusion detected in Light.h
#else // defined(Light_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Light_RECURSES

#if !defined Light_h
/** Prevents repeated inclusion of headers. */
#define Light_h

// In order to call opengl commands in all graphical objects
#include "raytracer/Ray.h"
#include "raytracer/RealColor.h"
#include "raytracer/RayTracerViewerExtension.h"

namespace DGtal {
  namespace rt {

    /// Lights are used to give lights in a scene.
    struct Light {

      /// Default constructor. Nothing to do.
      Light() {}

      /// Virtual destructor since object contains virtual methods.
      virtual ~Light() {}
    
      /// This method is called by Scene::init() at the beginning of the
      /// display in the OpenGL window.
      virtual void init( RTViewer& /* viewer */ ) = 0;

      /// This method is called by Scene::light() at each frame to
      /// set the lights in the OpenGL window.
      virtual void light( RTViewer& /* viewer */ ) = 0;

      /// This method is called by Scene::draw() at each frame to
      /// redisplay objects in the OpenGL window.
      virtual void draw( RTViewer& /* viewer */) = 0;

      /// Given the point \a p, returns the normalized direction to this
      /// light.
      virtual Vector3 direction( const Vector3& /* p */ ) const = 0;

      /// @return the color of this light viewed from the given point \a
      /// p.
      virtual RealColor color( const Vector3& /* p */ ) const = 0;

    };

  } // namespace rt
} // namespace DGtal

#endif // !defined Light_h

#undef Light_RECURSES
#endif // else defined(Light_RECURSES)

