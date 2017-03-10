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
 * @file DigitalVolume.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalVolume_RECURSES)
#error Recursive header files inclusion detected in DigitalVolume.h
#else // defined(DigitalVolume_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalVolume_RECURSES

#if !defined DigitalVolume_h
/** Prevents repeated inclusion of headers. */
#define DigitalVolume_h

#include "DGtal/base/ConstAlias.h"
#include "raytracer/GeometricalObject.h"
#include "raytracer/Parallelogram.h"

namespace DGtal {
  namespace rt {

    /// A triangle is a model of GraphicalObject.
    template <typename TBooleanImage>
    struct DigitalVolume : public virtual GeometricalObject {

      typedef TBooleanImage BooleanImage;

      /// A reference to the boolean image.
      const BooleanImage& bimage;
      /// the Khalimsky space
      KSpace space;
      /// Sides of the bounding box of the digital volume.
      std::vector<Parallelogram> sides;
      
      /// Creates a parallelogram of vertices \a a, \a b and \a c, and
      /// last vertex is computed as \f$ a + b-a + c-a \f$.
      inline
      DigitalVolume( ConstAlias<BooleanImage> anImage )
        : bimage( anImage )
      {
        space.init( anImage.domain().lowerBound(),
                    anImage.domain().upperBound(), true );
        Point3i lo   = space.uKCoords( space.lowerCell() ); 
        Point3i hi   = space.uKCoords( space.upperCell() );
        Point3i A000 = lo;
        Point3i A111 = hi;
        Point3i A100( hi[ 0 ], lo[ 1 ], lo[ 2 ] );
        Point3i A010( lo[ 0 ], hi[ 1 ], lo[ 2 ] );
        Point3i A001( lo[ 0 ], lo[ 1 ], hi[ 2 ] );
        Point3i A110( hi[ 0 ], hi[ 1 ], lo[ 2 ] );
        Point3i A011( lo[ 0 ], hi[ 1 ], hi[ 2 ] );
        Point3i A101( hi[ 0 ], lo[ 1 ], hi[ 2 ] );
        sides.push_back( Parallelogram( A000, A010, A100 ) ); // bottom (xy0)
        sides.push_back( Parallelogram( A000, A001, A010 ) ); // left   (0yz)
        sides.push_back( Parallelogram( A000, A100, A001 ) ); // front  (x0z)
        sides.push_back( Parallelogram( A111, A011, A101 ) ); // top    (xy1)
        sides.push_back( Parallelogram( A111, A101, A110 ) ); // right  (1yz)
        sides.push_back( Parallelogram( A111, A110, A011 ) ); // back   (x1z)
      }
    
      /// Virtual destructor since object contains virtual methods.
      ~DigitalVolume() {}

      void coordinates( Point3 p, Real& x, Real& y );

      /// @return the normal vector at point \a p on the object (\a p
      /// should be on or close to the sphere).
      virtual Vector3 getNormal( Point3 p );

      /// @param[in] ray the incoming ray
      /// @param[out] returns the point of intersection with the object
      /// (if any), or the closest point to it.
      ///
      /// @return either a real < 0.0 if there is an intersection, or a
      /// kind of distance to the closest point of intersection.
      virtual Real rayIntersection( const Ray& ray, Point3& p )
      {
        // Checks first that it intersects the bounding box.
        Point3 p  [ 6 ];
        bool   hit[ 6 ];
        int    nb_hit = 0;
        Real   dist   = 100000000.0;
        for ( unsigned int i = 0; i < 6; i++ )
          {
            Real d   = sides[ i ].rayIntersection( ray, p[ i ] );
            dist     = std::min( d, dist );
            hit[ i ] = d < 0.0;
            if ( hit[ i ] ) nb_hit++;
          }
        if ( nb_hit == 0 ) return dist;
        // Now casts the ray in the volume
      }
                    
    };

  } // namespace rt
} // namespace DGtal

#endif // !defined DigitalVolume_h

#undef DigitalVolume_RECURSES
#endif // else defined(DigitalVolume_RECURSES)
