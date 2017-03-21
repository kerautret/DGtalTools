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
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "raytracer/GeometricalObject.h"
#include "raytracer/Parallelogram.h"
#include "raytracer/StandardDSL3d.h"

namespace DGtal {
  namespace rt {

    /// A triangle is a model of GraphicalObject.
    template <typename TBooleanImage>
    struct DigitalVolume : public virtual GeometricalObject {

      typedef TBooleanImage                    BooleanImage;
      typedef KSpace3                          KSpace;
      typedef KSpace3::SCell                   SCell;
      typedef KSpace3::Cell                    Cell;
      typedef KSpace3::SCellSet                SCellSet;
      typedef SetOfSurfels<KSpace3, SCellSet>  SurfaceStorage;
      typedef DigitalSurface< SurfaceStorage > Surface;
      typedef std::map<SCell,Vector3>          VectorField;
      typedef VectorField                      NormalMap;
      
      /// Creates a parallelogram of vertices \a a, \a b and \a c, and
      /// last vertex is computed as \f$ a + b-a + c-a \f$.
      inline
      DigitalVolume( Clone<BooleanImage> anImage )
        : bimage( anImage ), ptrSurface( 0 ),
          interpolatedEstimatedNormals( false ), shiftFactor( 1.0 )
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
        SCellSet boundary;
        Surfaces<KSpace3>::sMakeBoundary( boundary, K, bimage,
                                          K.lowerBound(), K.upperBound() );
        ptrSurface = new Surface( new SurfaceStorage( K, true, boundary ) );
        // Build trivial normal field.
        for ( auto surfel : boundary )
          {
            Vector3 N = trivialNormal( surfel );
            trivialNormals[ surfel ] = N;
          }
        interpolateVectorField( trivialNormals, true );
      }
    
      /// Virtual destructor since object contains virtual methods.
      ~DigitalVolume() {}

      /// Chooses the rendering mode according to parameters
      void setMode( bool estimatedNormals, bool Phong, bool smoothed )
      {
        // Need given estimated normals to use estimated normals !
        useEstimatedNormals = interpolatedEstimatedNormals ? estimatedNormals : false;
        shiftFactor         = smoothed ? 1.0 : 0.0;
        PhongShading        = Phong;
      }
      
      BooleanImage&  image()   { return bimage; }
      KSpace3&       space()   { return K; }
      const Surface& surface() { return *ptrSurface; }
      
      /// @return the normal vector at point \a p on the object (\a p
      /// should be on or close to the sphere).
      virtual Vector3 getNormal( Point3 p )
      {
        if ( p == last_p )
          return last_n;
        trace.warning() << "[DigitalVolume::getNormal] Movement after ray ?"
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
        
        // Now casts the ray in the volume
        // (1) build a digital ray
        Ray ray2( p[ j ], ray.direction, ray.depth );
        StandardDSL3d D( ray2,
                         RT_PRECISION*( K.upperBound() - K.lowerBound() ).norm1() );
        // (2) sort points along ray
        // Point3  delta  = ray.direction / ray.direction.normInfinity();
        Point3i origin = D.getPoint( ray.origin + RT_BANDWIDTH * ray.direction );  // 0.5*delta );
        Point3i first  = bimage.domain().isInside( origin )
          ? origin : D.getPoint( p[ j ] );
        bool prev_state = bimage.domain().isInside( first )
          ? bimage( first ) != 0 : false;
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
                return intersectSurfel( ray_inter, surfel );
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
        Real d = 0.0;
        if ( surfel != last_surfel )
          {
            // Build parallelogram
            Dimension k = K.sOrthDir( surfel );
            Dimension i = (k+1)%3;
            Dimension j = (k+2)%3;
            Cell      c = K.unsigns( surfel );
            Point3    A = Point3( K.uCoords( c ) ) - Point3::diagonal( 0.5 );
            Point3    B = A; B[ i ] += 1.0; // + 2.0*RT_BANDWIDTH;
            Point3    C = A; C[ j ] += 1.0; // + 2.0*RT_BANDWIDTH;
            Point3    T = trivialNormal( surfel );
            last_square = Parallelogram( A, B, C );
            last_surfel = surfel;
          }
        Real x,y,a,b;
        bool intersect = last_square.intersectRay( ray_inter );
        Point3       p = ray_inter.intersection;
        last_square.coordinates( p, x, y );
        // std::cout << " p=" << p << " x=" << x << " y=" << y << std::endl;
        x       = std::min( 1.0, std::max( 0.0, x ) );
        y       = std::min( 1.0, std::max( 0.0, y ) );
        a       = 2.0*x-1.0;
        b       = 2.0*y-1.0;
        interpolatePositionAndNormal( p, last_n, ray_inter.ray, surfel, a, b );
        // last_n  = interpolateNormal( surfel, 2.0*x-1.0, 2.0*y-1.0 );
        // p       = last_square.A + x * last_square.U + y * last_square.V; 
        // p      += last_n / ( 2.0 * last_n.normInfinity() );
        last_n     = ( last_n != Vector3::zero ) ? last_n : trivialNormal( surfel );
        last_p     = p;
        Real shift = shiftFactor / last_n.normInfinity()
          * ( (last_n.dot( trivialNormal( surfel ) ) >= 0.0) ? 1.0 : -1.0 );
        ray_inter.normal       = last_n;
        ray_inter.intersection = p + shift * last_n;
        ray_inter.reflexion    = ray_inter.intersection;
        ray_inter.refraction   = p - shift * last_n;
        ray_inter.distance     = -1.0;
        return true;
      }

      // Real intersectSurfel( const Ray& ray, SCell surfel, Point3& p )
      // {
      //   Real d = 0.0;
      //   if ( surfel != last_surfel )
      //     {
      //       // Build parallelogram
      //       Dimension k = K.sOrthDir( surfel );
      //       Dimension i = (k+1)%3;
      //       Dimension j = (k+2)%3;
      //       Cell      c = K.unsigns( surfel );
      //       Point3    A = Point3( K.uCoords( c ) ) - Point3::diagonal( 0.5 );
      //       Point3    B = A; B[ i ] += 1.0; // + 2.0*RT_BANDWIDTH;
      //       Point3    C = A; C[ j ] += 1.0; // + 2.0*RT_BANDWIDTH;
      //       Point3    T = trivialNormal( surfel );
      //       last_square = Parallelogram( A, B, C );
      //       last_surfel = surfel;
      //     }
      //   Real x,y,a,b;
      //   last_square.rayIntersection( ray, p );
      //   last_square.coordinates( p, x, y );
      //   // std::cout << " p=" << p << " x=" << x << " y=" << y << std::endl;
      //   x       = std::min( 1.0-RT_EPSILON, std::max( RT_EPSILON, x ) );
      //   y       = std::min( 1.0-RT_EPSILON, std::max( RT_EPSILON, y ) );
      //   a       = 2.0*x-1.0;
      //   b       = 2.0*y-1.0;
      //   interpolatePositionAndNormal( p, last_n, ray, surfel, a, b );
      //   // last_n  = interpolateNormal( surfel, 2.0*x-1.0, 2.0*y-1.0 );
      //   // p       = last_square.A + x * last_square.U + y * last_square.V; 
      //   // p      += last_n / ( 2.0 * last_n.normInfinity() );
      //   last_p  = p;
        
      //   return -1.0; // d
      // }

      // Vector3 interpolateNormal( SCell surfel, Real alpha, Real beta )
      // {
      //   if ( ! interpolatedEstimatedNormals ) return normals[ surfel ];
      //   else return interpolateVector( normals, surfel, alpha, beta, true );
      // }

      // /// Given a reference to a vector field, some \a surfel and
      // /// parameters \a alpha and \b beta between -1 and 1,
      // /// returns the interpolated vector. 
      // ///
      // /// @param vf any vector field, for instance normals.
      // /// @param surfel any surfel (of orthogonal direction say k).
      // /// @param alpha the relative displacement along orientation (k+1)%3
      // /// @param beta  the relative displacement along orientation (k+2)%3
      // Vector3 interpolateVector( VectorField& vf,
      //                            SCell surfel, Real alpha, Real beta, bool normalize )
      // {
      //   Dimension k = K.sOrthDir( surfel );
      //   bool   dir1 = alpha >= 0.0;
      //   bool   dir2 = beta  >= 0.0;
      //   SCell  l1   = K.sIncident( surfel, (k+1)%3, dir1 );
      //   SCell  l2   = K.sIncident( surfel, (k+2)%3, dir2 );
      //   SCell  pt   = K.sIncident(     l1, (k+2)%3, dir2 );
      //   K.sSetSign( pt, K.POS );
      //   Real    a   = fabs( alpha );
      //   Real    b   = fabs( beta );
      //   // std::cout << "  a=" << a << " b=" << b << std::endl;
      //   Vector3 N   = (1.0-a)*(1.0-b)*vf[ surfel ] + a*(1.0-b)*vf[ l1 ]
      //     +           (1.0-a)*b      *vf[ l2 ]     + a*b      *vf[ pt ];
      //   return normalize ? N.getNormalized() : N;
      // }

      void computePositions( int nb_iterations )
      {
        const double D = sqrt(3.0)/2.0;
        for ( SCell surfel : surface() )
          {
            positions[ surfel ] = 0.5 * Point3( K.sKCoords( surfel ) )
              - Point3::diagonal( 0.5 );
          }
        if ( nb_iterations == 0 ) return;
        VectorField x = positions;
        for ( int i = 0; i < nb_iterations; i++ )
          {
            // new positions
            VectorField xx;
            double max_dx = 0.0;
            for ( SCell surfel : surface() )
              {
                std::vector<SCell> neighbors;
                auto outIt = back_inserter( neighbors );
                surface().writeNeighbors( outIt, surfel );
                Vector3 m = Vector3::zero;
                for ( auto n : neighbors ) m += x[ n ];
                m /= (Real) neighbors.size();
                // Project onto normal (if any)
                Vector3 N = normals[ surfel ];
                Vector3 p = positions[ surfel ];
                Real    l = ( m - p ).dot( N );
                l         = std::min( D, std::max( l, -D ) );
                xx[ surfel ] = p + l * N;
                max_dx    = std::max( max_dx, (xx[ surfel ] - x[ surfel ]).norm1() );
              }
            x = xx;
            trace.info() << "- Iteration " << i << " max_dx=" << max_dx << std::endl;
          }
        positions = x; 
        interpolateVectorField( positions, false );
      }
      
      Point3 standardPosition( SCell scell, Vector3 N )
      {
        // Point3 pos = 0.5 * Point3( K.sKCoords( scell ) ) - Point3::diagonal( 0.5 );
        Point3 pos = positions[ scell ];
        return pos;
      }
      
      void interpolatePositionAndNormal( Point3& pos, Vector3& N, const Ray & ray,
                                         SCell surfel, Real alpha, Real beta )
      {
        // if ( ! PhongShading )
        //   {
        //     N   = useEstimatedNormals ? normals[ surfel ] : trivialNormals[ surfel ];
        //     pos = standardPosition( surfel, N );
        //     return;
        //   }
        VectorField& NF = useEstimatedNormals ? normals : trivialNormals;
        Dimension k = K.sOrthDir( surfel );
        bool   dir1 = alpha >= 0.0;
        bool   dir2 = beta  >= 0.0;
        SCell  l1   = K.sIncident( surfel, (k+1)%3, dir1 );
        SCell  l2   = K.sIncident( surfel, (k+2)%3, dir2 );
        SCell  pt   = K.sIncident(     l1, (k+2)%3, dir2 );
        K.sSetSign( pt, K.POS );
        Real    a   = fabs( alpha );
        Real    b   = fabs( beta );
        // std::cout << "  a=" << a << " b=" << b << std::endl;
        Vector3 n00 = NF[ surfel ];
        Vector3 n10 = PhongShading ? NF[ l1 ] : n00;
        Vector3 n01 = PhongShading ? NF[ l2 ] : n00;
        Vector3 n11 = PhongShading ? NF[ pt ] : n00;
        Point3  p00 = standardPosition( surfel, n00 );
        Point3  p10 = standardPosition( l1,     n10 );
        Point3  p01 = standardPosition( l2,     n01 );
        Point3  p11 = standardPosition( pt,     n11 );
        Point3  p   = (1.0-a)*(1.0-b)*p00 + a*(1.0-b)*p10 + (1.0-a)*b*p01 + a*b*p11;
        if ( PhongShading )
          {
            N           = (1.0-a)*(1.0-b)*n00 + a*(1.0-b)*n10 + (1.0-a)*b*n01 + a*b*n11;
            N           = N.getNormalized();
          }
        else N = n00;
        Real gamma  = ( p00 - ray.origin ).dot( N ) / ray.direction.dot( N );
        pos         = ray.origin + gamma * ray.direction;
      }
      
      void interpolateVectorField
      ( VectorField& vField, bool normalize )
      {
        // for each surfel computes its weight
        std::map<SCell, Real> weights;
        for ( SCell surfel : surface() )
          {
            Vector3 n = normals[ surfel ];
            Vector3 t = trivialNormal( surfel );
            // area
            // weights[ surfel ] = std::max( 0.001, n.dot( t ) );
            weights[ surfel ] =
              normalize ? std::max( 0.001, fabs( n.dot( t ) ) ) : 1.0;
            // weights[ surfel ] =
            //   normalize ? std::max( 0.001, fabs( n.dot( t ) ) )
            //   : 1.0 / std::max( 0.001, fabs( n.dot( t ) ) );
          }
        // Then enrich the vector field with linel vectors.
        for ( SCell surfel : surface() )
          {
            auto out_arcs   = surface().outArcs( surfel );
            for ( auto arc : out_arcs )
              {
                SCell linel = surface().separator( arc );
                SCell   s0 = surface().tail( arc );
                SCell   s1 = surface().head( arc );
                Vector3& V = vField[ linel ];
                Real    w0 = weights[ s0 ];
                Real    w1 = weights[ s1 ];
                V  = w0 * vField[ s0 ] + w1 * vField[ s1 ];
                if ( normalize )           V  = V.getNormalized();
                else if ( (w0+w1) != 0.0 ) V /= (w0+w1);
              }
          }
        // Then enrich the vector field with pointel vectors.
        auto faces = surface().allClosedFaces();
        for ( auto f : faces )
          {
            auto vertices = surface().verticesAroundFace( f );
            SCell pointel = surface().pivot( f );
            Vector3& V    = vField[ pointel ];
            V             = Point3::zero;
            Real  total_w = 0.0;
            for ( auto s : vertices )
              {
                Real w   = weights[ s ];
                V       += w * vField[ s ];
                total_w += w;
              }
            if ( normalize )         V  = V.getNormalized();
            else if ( total_w != 0 ) V /= total_w;
          }
        // Update flag if it was the normal vector field
        if ( &vField == &normals ) interpolatedEstimatedNormals = true;
      }

      /// @return the trivial normal to the given surfel.
      Vector3 trivialNormal( SCell surfel ) const
      {
        Dimension k  = K.sOrthDir( surfel );
        bool ext_dir = ! K.sDirect( surfel, k );
        Vector3 t; // zero vector
        t[ k ] = ext_dir ? 1.0 : -1.0;
        return t;
      }
      // ----------------------------------------------------------------------
      
      /// A reference to the boolean image.
      BooleanImage  bimage;
      /// the Khalimsky space
      KSpace3       K;
      /// Sides of the bounding box of the digital volume.
      std::vector<Parallelogram> sides;
      /// The digital surface corresponding to the boundary of bimage.
      Surface*       ptrSurface;
      /// Map cell -> trivial normal vector
      VectorField   trivialNormals;
      /// Map cell -> normal vector
      VectorField   normals;
      /// Map cell -> position
      VectorField   positions;
      /// Tells if we wish to use estimated normals or trivial normals
      bool          useEstimatedNormals;
      /// Tells if the normal vector field has been interpolated (for Phong).
      bool          interpolatedEstimatedNormals;
      /// Mode for rendering digital object (0: flat, 1: Phong)
      bool          PhongShading;
      /// Tells how intersected points are displaced from the digitized boundary
      /// 0: digital, 1: "smooth"
      Real          shiftFactor;
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
