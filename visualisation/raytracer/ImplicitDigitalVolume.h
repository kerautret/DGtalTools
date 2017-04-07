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
 * @file ImplicitDigitalVolume.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(ImplicitDigitalVolume_RECURSES)
#error Recursive header files inclusion detected in ImplicitDigitalVolume.h
#else // defined(ImplicitDigitalVolume_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImplicitDigitalVolume_RECURSES

#if !defined ImplicitDigitalVolume_h
/** Prevents repeated inclusion of headers. */
#define ImplicitDigitalVolume_h

#include <map>
#include "raytracer/GeometricalObject.h"
#include "raytracer/Parallelogram.h"
#include "raytracer/StandardDSL3d.h"
#include "DGtal/base/Clone.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.h"

namespace DGtal {
  namespace rt {

    /// A triangle is a model of GraphicalObject.
    template <typename TImage>
    struct ImplicitDigitalVolume : public virtual GeometricalObject {

      typedef TImage                           Image;
      typedef KSpace3                          KSpace;
      typedef typename TImage::Value           Value;
      typedef functors::SimpleThresholdForegroundPredicate<Image> ThresholdedImage;
      typedef KSpace3::SCell                   SCell;
      typedef KSpace3::Cell                    Cell;
      typedef KSpace3::SCellSet                SCellSet;
      typedef SetOfSurfels<KSpace3, SCellSet>  SurfaceStorage;
      typedef DigitalSurface< SurfaceStorage > Surface;
      typedef std::map<SCell,Vector3>          VectorField;
      typedef VectorField                      NormalMap;

      typedef KSpace::Space                         Space;
      typedef Space::Point                          Point;
      typedef Space::RealVector                     RealVector;
      typedef RealVector::Component                 Scalar;
      typedef KSpace::Surfel                        Surfel;
      typedef HyperRectDomain<Space>                Domain;
      typedef ImplicitPolynomial3Shape<Space>       PolynomialShape;              
      typedef typename PolynomialShape::Polynomial3 Polynomial;
      
      BOOST_STATIC_ASSERT(( KSpace::dimension == 3 ));

      
      inline
      ImplicitDigitalVolume( Clone<TImage> anImage, Value threshold )
        : myImage( anImage ), myThresholdedImage( myImage, threshold ),
          myT( threshold ), ptrSurface( 0 )
      {
        K.init( myImage.domain().lowerBound(),
                myImage.domain().upperBound(), true );
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
        SCellSet boundary;
        Surfaces<KSpace3>::sMakeBoundary( boundary, K, myThresholdedImage,
                                          K.lowerBound(), K.upperBound() );
        ptrSurface = new Surface( new SurfaceStorage( K, true, boundary ) );
        // interpolateVectorField( trivialNormals, true );
        // Init polynomials
        Polynomial X[ 2 ][ 3 ];
        X[ 1 ][ 0 ] = mmonomial<Scalar>( 1, 0, 0 );
        X[ 1 ][ 1 ] = mmonomial<Scalar>( 0, 1, 0 );
        X[ 1 ][ 2 ] = mmonomial<Scalar>( 0, 0, 1 );
        for ( Dimension i = 0; i < 3; ++i )
          X[ 0 ][ i ] = mmonomial<Scalar>( 0, 0, 0 ) - X[ 1 ][ i ];
        for ( Dimension i = 0; i < 8; ++i )
          P[ i ] = X[ i & 1 ? 1 : 0 ][ 0 ]
            *      X[ i & 2 ? 1 : 0 ][ 1 ]
            *      X[ i & 4 ? 1 : 0 ][ 2 ];
      }
    
      /// Virtual destructor since object contains virtual methods.
      ~ImplicitDigitalVolume() {}

      
      Image&            image()   { return myImage; }
      ThresholdedImage& thresholdedImage()   { return myThresholdedImage; }
      KSpace3&          space()   { return K; }
      const Surface&    surface() { return *ptrSurface; }

      /// @return the normal vector at point \a p on the object (\a p
      /// should be on or close to the sphere).
      virtual Vector3 getNormal( Point3 p )
      {
        if ( p == last_p )
          return last_n;
        trace.warning() << "[ImplicitDigitalVolume::getNormal] Movement after ray ?"
                        << " p=" << p << " last_p=" << last_p
                        << " last_n=" << last_n << std::endl;
        return last_n;
      }

      /// @param[in,out] ray_inter as input the incoming ray, as
      /// output information abour intersection.
      ///
      /// @return true if there was an intersection, false otherwise
      /// (more information is stored in ray_inter)
      virtual bool intersectRay( RayIntersection& ray_inter )
      {
        // Checks first that it intersects the bounding box.
        Point3 p  [ 6 ];
        bool   hit[ 6 ];
        const Ray& ray = ray_inter.ray;
        int    nb_hit  = 0;
        Real   dist    = 100000000.0;
        Real   alpha   = 0.0;
        unsigned int j = 6;
        for ( unsigned int i = 0; i < 6; i++ )
          {
            hit[ i ] = sides[ i ].intersectRay( ray_inter );
            Real d   = ray_inter.distance;
            p[ i ]   = ray_inter.intersection;
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
        ray_inter.distance = dist;
        if ( nb_hit == 0 ) return false;
        if ( j == 6 ) {
          trace.warning() << "No closest point !" << std::endl;
          return false;
        }
        // To check that the bounding box is correct.
        // p_intersect = p[ j ];
        // last_n = sides[ j ].getNormal( p_intersect );
        // return -1.0;

        // (0) Checks if origin is in the volume
        Point3i origin ( (Integer) round( ray.origin[ 0 ] ),
                         (Integer) round( ray.origin[ 1 ] ),
                         (Integer) round( ray.origin[ 2 ] ) );
        bool origin_in = myImage.domain().isInside( origin );
        Point3  q      = origin_in ? ray.origin : p[ j ];
        trace.info() << "Inside: q=" << q << std::endl;
        // Now casts the ray in the volume
        // (1) build a digital ray
        Ray ray2( q, ray.direction, ray.depth ); //p[ j ]
        StandardDSL3d D( ray2,
                         RT_PRECISION*( K.upperBound() - K.lowerBound() ).norm1() );
        // (2) sort points along ray
        // Point3i origin = D.getPoint( ray.origin + RT_BANDWIDTH * ray.direction );  // 0.5*delta );
        Point3i first  = origin_in ? origin : D.getPoint( p[ j ] );
        bool prev_state = myImage.domain().isInside( first )
          ? myThresholdedImage( first ) != 0 : false;
        StandardDSL3d::ConstIterator it  = D.begin( first );
        // std::cout << "- beg=" << *it << " end=" << *itE << std::endl;
        if ( ! D.isInDSL( *it  ) )
          {
            trace.warning() << " *it=" << *it << " not inside"
                            << " Dxy=" << D.xy
                            << " Dxz=" << D.xz
                            << " Dyz=" << D.yz << std::endl;
            ray_inter.distance = 1.0f;
            return false;
          }
        Point3i prev_p  = *it++;
        // bool prev_state = bimage.domain().isInside( prev_p )
        //   ? bimage( prev_p ) != 0 : false;
        while ( true )
          {
            Point3i p  = *it;
            if ( ! myImage.domain().isInside( *it  ) )
              {
                // trace.info() << " " << p << " not inside." << std::endl;
                break;
              }
            bool state = myThresholdedImage( p );
            if ( state != prev_state ) // intersection
              {
                Dimension k = ( p[ 0 ] != prev_p[ 0 ] ) ? 0
                  : ( ( p[ 1 ] != prev_p[ 1 ] ) ? 1 : 2 );
                SCell voxel  = K.sSpel( prev_p, prev_state ? K.POS : K.NEG );
                SCell surfel = K.sIncident( voxel, k, p[ k ] > prev_p[ k ] );
                // p_intersect = 0.5 * Point3( K.sKCoords( surfel ) );
                // return -1.0;
                bool local_i = intersectSurfel( ray_inter, surfel );
                if ( local_i ) return true;
              }
            prev_p     = p;
            prev_state = state;
            ++it;
          }
        // trace.warning() << "No intersection found !" << std::endl;
        return false;
      }

      bool intersectSurfel( RayIntersection& ray_inter, SCell surfel )
      { 
        Dimension k = K.sOrthDir( surfel );
        Dimension i = (k+1)%3;
        Dimension j = (k+2)%3;
        if ( surfel != last_surfel )
          {
            // Build parallelogram
            Cell      c = K.unsigns( surfel );
            Point3    A = Point3( K.uCoords( c ) ) - Point3::diagonal( 0.5 );
            Point3    B = A; B[ i ] += 1.0; // + 2.0*RT_BANDWIDTH;
            Point3    C = A; C[ j ] += 1.0; // + 2.0*RT_BANDWIDTH;
            last_square = Parallelogram( A, B, C );
            last_surfel = surfel;
          }
        Real x,y,a,b;
        bool intersect = last_square.intersectRay( ray_inter );
        Point3       p = ray_inter.intersection;
        last_square.coordinates( p, x, y );
        // std::cout << " p=" << p << " x=" << x << " y=" << y << std::endl;
        x           = std::min( 1.0, std::max( 0.0, x ) );
        y           = std::min( 1.0, std::max( 0.0, y ) );
        a           = 2.0*x-1.0;
        b           = 2.0*y-1.0;
        // We build the correct 8-cube around the surfel
        Point3i base = K.sCoords( surfel );
        base[ i ]   -= a >= 0.0 ? 0 : 1;
        base[ j ]   -= b >= 0.0 ? 0 : 1;
        // Make the local polynomial isosurface
        Polynomial L;
        for ( Dimension l = 0; l < 8; l++ )
          {
            Point3i voxel( base[ 0 ] + (l & 1) ? 1 : 0,
                           base[ 1 ] + (l & 2) ? 1 : 0,
                           base[ 2 ] + (l & 4) ? 1 : 0 );
            Value v = myImage.domain().isInside( voxel ) ? myImage( voxel ) : 0;
            L      += ((Scalar) ( v - myT )) * P[ l ];
          }
        PolynomialShape PS( L );
        // Checks for intersection along ray
        Point3 q   = p;
        bool found = intersectPolynomialShape( PS, ray_inter.ray.direction, q );
        if ( ! found ) return false;
        last_n     = PS.gradient( q );
        last_n    /= last_n.norm();
        last_p     = q;
        ray_inter.normal       = last_n;
        ray_inter.intersection = q;
        ray_inter.reflexion    = q;
        ray_inter.refraction   = q;
        ray_inter.distance     = -1.0;
        return true;
      }

      bool intersectPolynomialShape( const PolynomialShape& PS, const Vector3& u,
                                     Point3& q )
      {
        const int   iter = 10;
        const Scalar att = 0.5;
        const Scalar eps = 0.1;
        Scalar      diff = PS( q );
        for ( int n = 0; n < iter && ( fabs( diff ) >= eps ); n++ )
          {
            Vector3  g = PS.gradient( q );
            Scalar dgu = g.dot( u );
            q         += (att * dgu) * u;
            diff       = PS( q );
          }
        return ( fabs( diff ) >= eps );
      }
      
      // ----------------------------------------------------------------------
      
      /// A clone of the image.
      Image            myImage;
      /// The thresholded image.
      ThresholdedImage myThresholdedImage;
      /// The threshold
      Value            myT;
      /// the Khalimsky space
      KSpace3       K;
      /// Polynomial associated with each vertex of the 8-cube;
      Polynomial    P[ 8 ];
      /// Sides of the bounding box of the digital volume.
      std::vector<Parallelogram> sides;
      /// The digital surface corresponding to the boundary of bimage.
      Surface*      ptrSurface;
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

#endif // !defined ImplicitDigitalVolume_h

#undef ImplicitDigitalVolume_RECURSES
#endif // else defined(ImplicitDigitalVolume_RECURSES)
