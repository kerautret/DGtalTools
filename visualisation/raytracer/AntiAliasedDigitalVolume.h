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
 * @file AntiAliasedDigitalVolume.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(AntiAliasedDigitalVolume_RECURSES)
#error Recursive header files inclusion detected in AntiAliasedDigitalVolume.h
#else // defined(AntiAliasedDigitalVolume_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AntiAliasedDigitalVolume_RECURSES

#if !defined AntiAliasedDigitalVolume_h
/** Prevents repeated inclusion of headers. */
#define AntiAliasedDigitalVolume_h

#include <map>
#include "raytracer/GeometricalObject.h"
#include "raytracer/Parallelogram.h"
#include "raytracer/StandardDSL3d.h"
#include "DGtal/base/Clone.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"
#include "DGtal/dec/DiscreteExteriorCalculusSolver.h"

namespace DGtal {
  namespace rt {

    /// A triangle is a model of GraphicalObject.
    template <typename TBooleanImage>
    struct AntiAliasedDigitalVolume : public virtual GeometricalObject {

      typedef TBooleanImage                    BooleanImage;
      typedef KSpace3                          KSpace;
      typedef KSpace3::SCell                   SCell;
      typedef KSpace3::Cell                    Cell;
      typedef KSpace3::SCellSet                SCellSet;
      typedef SetOfSurfels<KSpace3, SCellSet>  SurfaceStorage;
      typedef DigitalSurface< SurfaceStorage > Surface;
      typedef std::map<SCell,Vector3>          VectorField;
      typedef VectorField                      NormalMap;

      typedef EigenLinearAlgebraBackend        LinearAlgebra;
      typedef KSpace::Space                         Space;
      typedef Space::Point                          Point;
      typedef Space::RealVector                     RealVector;
      typedef RealVector::Component                 Scalar;
      typedef KSpace::Surfel                        Surfel;
      typedef HyperRectDomain<Space>                         Domain;
      typedef DiscreteExteriorCalculus<3,3, LinearAlgebra>   Calculus;
      typedef DiscreteExteriorCalculusFactory<LinearAlgebra> CalculusFactory;
      typedef Calculus::Index                       Index;
      typedef Calculus::PrimalForm0                 PrimalForm0;
      typedef Calculus::PrimalForm1                 PrimalForm1;
      typedef Calculus::PrimalForm2                 PrimalForm2;
      typedef Calculus::DualIdentity0               DualIdentity0;
      typedef Calculus::PrimalDerivative0           PrimalDerivative0;
      typedef Calculus::PrimalDerivative1           PrimalDerivative1;
      typedef Calculus::DualDerivative0             DualDerivative0;
      typedef Calculus::DualDerivative1             DualDerivative1;
      typedef Calculus::PrimalAntiderivative1       PrimalAntiderivative1;
      typedef Calculus::PrimalAntiderivative2       PrimalAntiderivative2;
      typedef Calculus::DualAntiderivative1         DualAntiderivative1;
      typedef Calculus::DualAntiderivative2         DualAntiderivative2;
      typedef Calculus::PrimalHodge0                PrimalHodge0;
      typedef Calculus::PrimalHodge1                PrimalHodge1;
      typedef Calculus::PrimalHodge2                PrimalHodge2;
      typedef Calculus::DualHodge0                  DualHodge0;
      typedef Calculus::DualHodge1                  DualHodge1;
      typedef Calculus::DualHodge2                  DualHodge2;
      typedef LinearAlgebra::SolverSimplicialLLT    LinearAlgebraSolver;
      typedef DiscreteExteriorCalculusSolver<Calculus, LinearAlgebraSolver, 0, DUAL, 0, DUAL> 
      SolverU;

      BOOST_STATIC_ASSERT(( KSpace::dimension == 3 ));

      
      /// Creates a parallelogram of vertices \a a, \a b and \a c, and
      /// last vertex is computed as \f$ a + b-a + c-a \f$.
      inline
      AntiAliasedDigitalVolume( Clone<BooleanImage> anImage )
        : bimage( anImage ), ptrSurface( 0 )
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
            inputNormals[ surfel ] = N;
          }
        // interpolateVectorField( trivialNormals, true );
      }
    
      /// Virtual destructor since object contains virtual methods.
      ~AntiAliasedDigitalVolume() {}

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

      /// computes the implicit function
      void computeImplicitFunction()
      {
        DGtal::trace.beginBlock( "Computing DEC" );
        /// The discrete exterior calculus instance.
        Calculus calculus;
        DGtal::trace.info() << "- add cells" << std::endl;
        calculus.myKSpace = space();
        // Adds all the cell
        for ( auto surfel : surface() )
          {
            calculus.insertSCell( surfel );
            Dimension k = space().sOrthDir( surfel );
            calculus.insertSCell( space().sIncident( surfel, k, true ) );
            calculus.insertSCell( space().sIncident( surfel, k, false ) );
          }
        calculus.updateIndexes();
        DGtal::trace.info() << "- dual_D0" << std::endl;
        DualDerivative0 dual_D0  = calculus.template derivative<0,DUAL>();
        DualIdentity0   dual_Id0 = calculus.template identity  <0,DUAL>();
        DualIdentity0   M        = dual_D0.transpose() * dual_D0 + 0.001 * dual_Id0;
        DualForm1       n( calculus );
        DualForm0       f( calculus );
        DGtal::trace.info() << "- dual 1-form n and dual 0-form f" << std::endl;
        for ( auto sn : inputNormals )
          {
            SCell  surfel = sn.first;
            Dimension   k = space().sOrthDir( surfel );
            SCell  in_vox = space().sDirectIncident( surfel, k );
            SCell out_vox = space().sIndirectIncident( surfel, k );
            Vector3 n_est = sn.second;
            Index    idx  = calculus.getCellIndex( space().unsigns( surfel ) );
            Index in_idx  = calculus.getCellIndex( space().unsigns( in_vox ) );
            Index out_idx = calculus.getCellIndex( space().unsigns( out_vox ) );
            n.myContainer( idx )     = in_n[ k ];
            f.myContainer( in_idx )  = 0.5;
            f.myContainer( out_idx ) = -0.5;
          }
        DGtal::trace.info() << "- prefactoring matrix M := A'^t A' + a Id" << std::endl;
        SolverU solver( calculus );
        solver.compute( M );
        DGtal::trace.info() << "- solving M u = A'^t n + a f" << std::endl;
        DualForm0 v = dual_D0.transpose() * n + 0.001 * f;
        DualForm0 u = solver.solve( v );
        DGtal::trace.info() << ( solver.isValid() ? "=> OK" : "ERROR" )
                            << " " << solver.myLinearAlgebraSolver.info() << std::endl;
        // TODO
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
      /// Map cell -> input normal vector
      VectorField   inputNormals;
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

#endif // !defined AntiAliasedDigitalVolume_h

#undef AntiAliasedDigitalVolume_RECURSES
#endif // else defined(AntiAliasedDigitalVolume_RECURSES)
