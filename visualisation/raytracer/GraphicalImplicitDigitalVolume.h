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
 * @file GraphicalImplicitDigitalVolume.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(GraphicalImplicitDigitalVolume_RECURSES)
#error Recursive header files inclusion detected in GraphicalImplicitDigitalVolume.h
#else // defined(GraphicalImplicitDigitalVolume_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GraphicalImplicitDigitalVolume_RECURSES

#if !defined GraphicalImplicitDigitalVolume_h
/** Prevents repeated inclusion of headers. */
#define GraphicalImplicitDigitalVolume_h

#include "DGtal/topology/helpers/Surfaces.h"
#include "raytracer/ImplicitDigitalVolume.h"
#include "raytracer/GraphicalObject.h"

namespace DGtal {
  namespace rt {

    /// A graphical digital volume is a model of GraphicalObject.
    template <typename TImage>
    struct GraphicalImplicitDigitalVolume : public ImplicitDigitalVolume<TImage>,
                                            public GraphicalObject {
      typedef TImage Image;
      typedef ImplicitDigitalVolume<Image> Base;
      // using typename Base::Image;
      using typename Base::ThresholdedImage;
      using typename Base::SCell;
      using typename Base::Cell;
      using Base::image;
      using Base::thresholdedImage;
      using Base::space;
      
      /// Material used everywhere except on the band along the edges.
      Material material;
    
      inline
      GraphicalImplicitDigitalVolume( Clone<Image> anImage,
                                      int threshold,
                                      const Material& m )
        : Base( anImage, threshold ), material( m )
      {}
    
      /// Virtual destructor since object contains virtual methods.
      ~GraphicalImplicitDigitalVolume() {}

      /// This method is called by Scene::init() at the beginning of the
      /// display in the OpenGL window. May be useful for some
      /// precomputations.
      virtual void init( RTViewer& viewer )
      {
        Cell           dummy;
        std::set<Cell> bdry;
        const ThresholdedImage& I = thresholdedImage();
        const KSpace3&          K = space();
        Surfaces<KSpace3>::uMakeBoundary
          ( bdry, K, I, K.lowerBound(), K.upperBound() );
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

#endif // !defined GraphicalImplicitDigitalVolume_ha

#undef GraphicalImplicitDigitalVolume_RECURSES
#endif // else defined(GraphicalImplicitDigitalVolume_RECURSES)
