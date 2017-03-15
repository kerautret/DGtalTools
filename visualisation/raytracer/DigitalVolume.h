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

#include <map>
#include "DGtal/base/Clone.h"
#include "raytracer/GeometricalObject.h"
#include "raytracer/Parallelogram.h"
#include "raytracer/StandardDSL3d.h"

namespace DGtal {
  namespace rt {

    /// A triangle is a model of GraphicalObject.
    template <typename TBooleanImage>
    struct DigitalVolume : public virtual GeometricalObject {

      typedef TBooleanImage           BooleanImage;
      typedef KSpace3                 KSpace;
      typedef KSpace3::SCell          SCell;
      typedef KSpace3::Cell           Cell;
      typedef std::map<SCell,Vector3> NormalMap;
      
      /// Creates a parallelogram of vertices \a a, \a b and \a c, and
      /// last vertex is computed as \f$ a + b-a + c-a \f$.
      inline
      DigitalVolume( Clone<BooleanImage> anImage )
        : bimage( anImage )
      {
        K.init( bimage.domain().lowerBound(),
                bimage.domain().upperBound(), true );
        Point3i lo   = K.uCoords( K.lowerCell() ); 
        Point3i hi   = K.uCoords( K.upperCell() );
        Point3 A000 = Point3( lo ) - Point3::diagonal( 0.5 );
        Point3 A111 = Point3( hi ) - Point3::diagonal( 0.5 );
        Point3 A100( A111[ 0 ], A000[ 1 ], A000[ 2 ] );
        Point3 A010( A000[ 0 ], A111[ 1 ], A000[ 2 ] );
        Point3 A001( A000[ 0 ], A000[ 1 ], A111[ 2 ] );
        Point3 A110( A111[ 0 ], A111[ 1 ], A000[ 2 ] );
        Point3 A011( A000[ 0 ], A111[ 1 ], A111[ 2 ] );
        Point3 A101( A111[ 0 ], A000[ 1 ], A111[ 2 ] );
        sides.push_back( Parallelogram( A000, A010, A100 ) ); // bottom (xy0)
        sides.push_back( Parallelogram( A000, A001, A010 ) ); // left   (0yz)
        sides.push_back( Parallelogram( A000, A100, A001 ) ); // front  (x0z)
        sides.push_back( Parallelogram( A111, A011, A101 ) ); // top    (xy1)
        sides.push_back( Parallelogram( A111, A101, A110 ) ); // right  (1yz)
        sides.push_back( Parallelogram( A111, A110, A011 ) ); // back   (x1z)
      }
    
      /// Virtual destructor since object contains virtual methods.
      ~DigitalVolume() {}

      BooleanImage& image() { return bimage; }
      KSpace3&      space() { return K; }
      
      /// @return the normal vector at point \a p on the object (\a p
      /// should be on or close to the sphere).
      virtual Vector3 getNormal( Point3 p )
      {
        if ( p == last_p )
          return last_n;
        trace.warning() << "[DigitalVolume::getNormal] Movement after ray ?" << std::endl;
        return last_n;
      }

      /// @param[in] ray the incoming ray
      /// @param[out] p_intersect returns the point of intersection with the object
      /// (if any), or the closest point to it.
      ///
      /// @return either a real < 0.0 if there is an intersection, or a
      /// kind of distance to the closest point of intersection.
      virtual Real rayIntersection( const Ray& ray, Point3& p_intersect )
      {
        // Checks first that it intersects the bounding box.
        Point3 p  [ 6 ];
        bool   hit[ 6 ];
        int    nb_hit = 0;
        Real   dist   = 100000000.0;
        Real   alpha  = 0.0;
        unsigned int j = 6;
        for ( unsigned int i = 0; i < 6; i++ )
          {
            Real d   = sides[ i ].rayIntersection( ray, p[ i ] );
            dist     = std::min( d, dist );
            hit[ i ] = d < 0.0;
            if ( hit[ i ] )
              {
                nb_hit++;
                Real beta = ( p[ i ] - ray.origin ).dot( ray.direction );
                if ( ( beta > RT_EPSILON )
                     && ( ( j == 6 ) || ( beta < alpha ) ) )
                  {
                    j = i;
                    alpha = beta;
                  }
              }
          }
        if ( nb_hit == 0 ) return dist;
        if ( j == 6 ) {
          trace.warning() << "No closest point !" << std::endl;
          return 0.0;
        }
        // To check that the bounding box is correct.
        // p_intersect = p[ j ];
        // last_n = sides[ j ].getNormal( p_intersect );
        // return -1.0;
        
        // Now casts the ray in the volume
        // (1) build a digital ray
        Ray ray2( p[ j ], ray.direction, ray.depth );
        StandardDSL3d D( ray2,
                         RT_PRECISION*( K.upperBound() - K.lowerBound() ).norm1() );
        // (2) sort points along ray
        Point3i origin = D.getPoint( ray.origin + 0.1 * ray.direction ); //RT_BANDWIDTH * ray.direction );
        Point3i first  = bimage.domain().isInside( origin )
          ? origin : D.getPoint( p[ j ] );
        bool prev_state = bimage.domain().isInside( first )
          ? bimage( first ) != 0 : false;
        StandardDSL3d::ConstIterator it  = D.begin( first );
        // std::cout << "- beg=" << *it << " end=" << *itE << std::endl;
        if ( ! D.isInDSL( *it  ) ) trace.warning() << " *it not inside" << std::endl;
        for ( unsigned int i = 0; ! bimage.domain().isInside( *it  ) && i < 10; i++ )
          it++;
        // Look for intersection.
        //StandardDSL3d::ConstIterator itp = it; --itp;
        Point3i prev_p = *it++;
        // first          = *it;
        while ( true )
          {
            Point3i p  = *it;
            if ( ! bimage.domain().isInside( *it  ) )
              {
                // trace.info() << " " << p << " not inside." << std::endl;
                break;
              }
            bool state = bimage( p );
            if ( state != prev_state ) // intersection
              {
                Dimension k = ( p[ 0 ] != prev_p[ 0 ] ) ? 0
                  : ( ( p[ 1 ] != prev_p[ 1 ] ) ? 1 : 2 );
                SCell voxel  = K.sSpel( prev_p, prev_state ? K.POS : K.NEG );
                SCell surfel = K.sIncident( voxel, k, p[ k ] > prev_p[ k ] );
                // p_intersect = 0.5 * Point3( K.sKCoords( surfel ) );
                // return -1.0;
                return intersectSurfel( ray, surfel, p_intersect );
              }
            prev_p     = p;
            prev_state = state;
            ++it;
          }
        // trace.warning() << "No intersection found !" << std::endl;
        return 1.0;
      }

      Real intersectSurfel( const Ray& ray, SCell surfel, Point3& p )
      {
        Real d = 0.0;
        if ( surfel == last_surfel )
          {
            d = last_square.rayIntersection( ray, p );
            // Take care of normal if it is interpolated.
            last_p = p;
            return -1.0; // d; //-1.0;
          }
        // Build parallelogram
        Dimension k  = K.sOrthDir( surfel );
        Dimension i  = (k+1)%3;
        Dimension j  = (k+2)%3;
        bool ext_dir = ! K.sDirect( surfel, k );
        // Point3i A  = K.uCoords( K.uIncident( K.uIncident( c, i, false ), j, false ) );
        Cell      c  = K.unsigns( surfel );
        Point3    A  = Point3( K.uCoords( c ) ) - Point3::diagonal( 0.5+RT_BANDWIDTH );
        Point3    B  = A; B[ i ] += 1.0 + 2.0*RT_BANDWIDTH;
        Point3    C  = A; C[ j ] += 1.0 + 2.0*RT_BANDWIDTH;
        Point3    N = (B-A).crossProduct(C-A);
        if ( normals.find( surfel ) == normals.end() )
          normals[ surfel ] = N;
        last_square = ( ( N[ k ] > 0.0 ) == ext_dir )
          ? Parallelogram( A, B, C ) : Parallelogram( A, C, B );
        last_surfel = surfel;
        d           = last_square.rayIntersection( ray, p );
        last_p      = p;
        last_n      = normals[ surfel ];  //last_square.getNormal( p );
        return -1.0; // d
      }
        
      /// A reference to the boolean image.
      BooleanImage  bimage;
      /// the Khalimsky space
      KSpace3       K;
      /// Sides of the bounding box of the digital volume.
      std::vector<Parallelogram> sides;
      /// Map surfel -> normal vector
      NormalMap     normals;

      /// Last point of intersection
      Point3        last_p;
      /// Last normal at intersection
      Vector3       last_n;
      /// Last surfel at intersection
      SCell         last_surfel;
      /// Last parallelogram at intersection
      Parallelogram last_square;
    };

  } // namespace rt
} // namespace DGtal

#endif // !defined DigitalVolume_h

#undef DigitalVolume_RECURSES
#endif // else defined(DigitalVolume_RECURSES)
