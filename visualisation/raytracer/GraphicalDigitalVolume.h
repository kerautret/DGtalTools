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
 * @file GraphicalDigitalVolume.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(GraphicalDigitalVolume_RECURSES)
#error Recursive header files inclusion detected in GraphicalDigitalVolume.h
#else // defined(GraphicalDigitalVolume_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GraphicalDigitalVolume_RECURSES

#if !defined GraphicalDigitalVolume_h
/** Prevents repeated inclusion of headers. */
#define GraphicalDigitalVolume_h

#include "DGtal/topology/helpers/Surfaces.h"
#include "raytracer/DigitalVolume.h"
#include "raytracer/GraphicalObject.h"

namespace DGtal {
  namespace rt {

    /// A graphical digital volume is a model of GraphicalObject.
    template <typename TBooleanImage>
    struct GraphicalDigitalVolume : public DigitalVolume<TBooleanImage>,
                                    public GraphicalObject {
      typedef DigitalVolume<TBooleanImage> Base;
      using typename Base::BooleanImage;
      using typename Base::SCell;
      using typename Base::Cell;
      using Base::image;
      using Base::space;
      
      /// Material used everywhere except on the band along the edges.
      Material material;
    
      inline
      GraphicalDigitalVolume( Clone<BooleanImage> anImage,
                              const Material& m )
        : Base( anImage ), material( m )
      {}
    
      /// Virtual destructor since object contains virtual methods.
      ~GraphicalDigitalVolume() {}

      /// This method is called by Scene::init() at the beginning of the
      /// display in the OpenGL window. May be useful for some
      /// precomputations.
      virtual void init( RTViewer& viewer )
      {
        Cell           dummy;
        std::set<Cell> bdry;
        const BooleanImage& I = image();
        const KSpace3&      K = space();
        Surfaces<KSpace3>::uMakeBoundary
          ( bdry, K, I, // [  ] (Point3i p) { return I( p ) != 0; },
            K.lowerBound(), K.upperBound() );
        viewer << SetMode3D( dummy.className(), "Basic" );
        viewer.setFillColor( material.diffuse );
        for ( auto surfel : bdry ) viewer << surfel;
      }

      /// This method is called by Scene::draw() at each frame to
      /// redisplay objects in the OpenGL window.
      virtual void draw( RTViewer& viewer )
      {}

      /// @return the material associated to this part of the object
      virtual Material getMaterial( Point3 p )
      {
        return material;
      }

                    
    };

  } // namespace rt
} // namespace DGtal

#endif // !defined GraphicalDigitalVolume_ha

#undef GraphicalDigitalVolume_RECURSES
#endif // else defined(GraphicalDigitalVolume_RECURSES)
