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
 * @file NormalEstimation.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(NormalEstimation_RECURSES)
#error Recursive header files inclusion detected in NormalEstimation.h
#else // defined(NormalEstimation_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NormalEstimation_RECURSES

#if !defined NormalEstimation_h
/** Prevents repeated inclusion of headers. */
#define NormalEstimation_h

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/graph/DepthFirstVisitor.h"
#include "DGtal/graph/GraphVisitorRange.h"
#include "DGtal/images/SimpleThresholdForegroundPredicate.h"
#include "DGtal/geometry/surfaces/estimation/CNormalVectorEstimator.h"
#include "DGtal/geometry/surfaces/estimation/VoronoiCovarianceMeasureOnDigitalSurface.h"
#include "DGtal/geometry/surfaces/estimation/VCMDigitalSurfaceLocalEstimator.h"
#include "DGtal/geometry/surfaces/estimation/TrueDigitalSurfaceLocalEstimator.h"
#include "DGtal/geometry/surfaces/estimation/IIGeometricFunctors.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantCovarianceEstimator.h"


namespace DGtal {
  namespace rt {

    /**
    Computes the normal estimations. Outputs statistics or export cell geometry.
    */
    template <typename KSpace,
              typename Surface,
              typename Estimator, 
              typename NormalMap>
    void computeEstimation
    ( const boost::program_options::variables_map& vm,     //< command-line parameters
      const KSpace& K,                 //< cellular grid space
      const Surface& surface,          //< digital surface approximating shape
      Estimator& estimator,            //< an initialized estimator
      NormalMap& n_estimations         //< a map for storing normal estimations
      )
    {
      typedef typename Surface::Surfel Surfel;
      typedef typename Estimator::Quantity Quantity;
      typedef double Scalar;
      typedef DepthFirstVisitor< Surface > Visitor;
      typedef GraphVisitorRange< Visitor > VisitorRange;
      typedef typename VisitorRange::ConstIterator VisitorConstIterator;
      
      string nameEstimator = vm[ "estimator" ].as<string>();
      trace.beginBlock( "Computing " + nameEstimator + "estimations component per component." );
      std::set<Surfel> M;
      for ( Surfel surfel : surface )
        {
          if ( M.find( surfel ) == M.end() )
            {
              CountedPtr<VisitorRange> range( new VisitorRange( new Visitor( surface, surfel ) ) );
              std::vector<Quantity> v_estimations;
              estimator.eval( range->begin(), range->end(), std::back_inserter( v_estimations ) );
              trace.info() << "- from " << surfel << ", nb estimations  = " << v_estimations.size() << std::endl;
              CountedPtr<VisitorRange> range2( new VisitorRange( new Visitor( surface, surfel ) ) );
              typename std::vector<Quantity>::iterator it = v_estimations.begin();
              for ( Surfel s : *range2 ) 
                {
                  n_estimations[ surfel ] = *it++;
                  M.insert( s );
                }
            }
        }
      trace.endBlock();
            
      trace.beginBlock( "Correcting orientations." );
      // TODO
      trace.endBlock();
    }

    template <typename KSpace,
              typename Surface,
              typename KernelFunction,
              typename PointPredicate,
              typename NormalMap>
    void chooseEstimator
    ( const boost::program_options::variables_map& vm,     //< command-line parameters
      const KSpace& K,                 //< cellular grid space
      const Surface& surface,          //< digital surface approximating shape
      const KernelFunction& chi,       //< the kernel function
      const PointPredicate& ptPred,    //< analysed implicit digital shape as a PointPredicate
      NormalMap& n_estimations         //< a map for storing normal estimations
      )
    {
      string nameEstimator = vm[ "estimator" ].as<string>();
      double h = vm["gridstep"].as<double>();
      if ( nameEstimator == "VCM" )
        {
          trace.beginBlock( "Chosen estimator is: VCM." );
          typedef typename KSpace::Space Space;
          typedef typename Surface::DigitalSurfaceContainer SurfaceContainer;
          typedef ExactPredicateLpSeparableMetric<Space,2> Metric;
          typedef VoronoiCovarianceMeasureOnDigitalSurface<SurfaceContainer,Metric,
                                                           KernelFunction> VCMOnSurface;
          typedef functors::VCMNormalVectorFunctor<VCMOnSurface> NormalFunctor;
          typedef VCMDigitalSurfaceLocalEstimator<SurfaceContainer,Metric,
                                                  KernelFunction, NormalFunctor> VCMNormalEstimator;
          int embedding = vm["embedding"].as<int>();
          Surfel2PointEmbedding embType = embedding == 0 ? Pointels :
            embedding == 1 ? InnerSpel : OuterSpel;
          double R = vm["R-radius"].as<double>();
          double r = vm["r-radius"].as<double>();
          double t = vm["trivial-radius"].as<double>();
          double alpha = vm["alpha"].as<double>();
          if ( alpha != 0.0 ) R *= pow( h, alpha-1.0 );
          if ( alpha != 0.0 ) r *= pow( h, alpha-1.0 );
          trace.info() << "- R=" << R << " r=" << r << " t=" << t << std::endl;
          VCMNormalEstimator estimator;
          estimator.attach( surface );
          estimator.setParams( embType, R, r, chi, t, Metric(), true );
          estimator.init( h, surface.begin(), surface.end() );
          trace.endBlock();
          computeEstimation( vm, K, surface, estimator, n_estimations );
        }
      else if ( nameEstimator == "II" )
        {
          trace.beginBlock( "Chosen estimator is: II." );
          typedef typename KSpace::Space Space;
          typedef HyperRectDomain<Space> Domain;
          typedef ImageContainerBySTLVector< Domain, bool> Image;
          typedef typename Domain::ConstIterator DomainConstIterator;
          typedef functors::SimpleThresholdForegroundPredicate<Image> ThresholdedImage;
          typedef functors::IINormalDirectionFunctor<Space> IINormalFunctor;
          typedef IntegralInvariantCovarianceEstimator<KSpace, ThresholdedImage, IINormalFunctor> IINormalEstimator;
          double r = vm["r-radius"].as<double>();
          double alpha = vm["alpha"].as<double>();
          if ( alpha != 0.0 ) r *= pow( h, alpha-1.0 );
          trace.info() << " r=" << r << std::endl;
          trace.beginBlock( "Preparing characteristic set." );
          Domain domain( K.lowerBound(), K.upperBound() );
          Image image( domain );
          for ( DomainConstIterator it = domain.begin(), itE = domain.end(); it != itE; ++it )
            {
              image.setValue( *it, ptPred( *it ) );
            }
          trace.endBlock();
          trace.beginBlock( "Initialize II estimator." );
          ThresholdedImage thresholdedImage( image, false );
          IINormalEstimator ii_estimator( K, thresholdedImage );
          ii_estimator.setParams( r );
          ii_estimator.init( h, surface.begin(), surface.end() );
          trace.endBlock();
          trace.endBlock();
          computeEstimation( vm, K, surface, ii_estimator, n_estimations );
        }

    }

    template <typename KSpace,
              typename Surface,
              typename PointPredicate,
              typename NormalMap >
    void chooseKernel
    ( const boost::program_options::variables_map& vm,     //< command-line parameters
      const KSpace& K,                 //< cellular grid space
      const Surface& surface,          //< digital surface approximating shape
      const PointPredicate& ptPred,    //< analysed implicit digital shape as a PointPredicate
      NormalMap& n_estimations         //< a map for storing normal estimations
      )
    {
      string kernel = vm[ "kernel" ].as<string>();
      double h = vm["gridstep"].as<double>();
      double r = vm["r-radius"].as<double>();
      double alpha = vm["alpha"].as<double>();
      if ( alpha != 0.0 ) r *= pow( h, alpha-1.0 );
      if ( kernel == "hat" ) {
        typedef typename KSpace::Point Point;
        typedef functors::HatPointFunction<Point,double> KernelFunction;
        KernelFunction chi_r( 1.0, r );
        chooseEstimator( vm, K, surface, chi_r, ptPred, n_estimations );
      } else if ( kernel == "ball" ) {
        typedef typename KSpace::Point Point;
        typedef functors::BallConstantPointFunction<Point,double> KernelFunction;
        KernelFunction chi_r( 1.0, r );
        chooseEstimator( vm, K, surface, chi_r, ptPred, n_estimations );
      }
    }

  } // namespace rt
} // namespace DGtal

#endif // !defined NormalEstimation_h

#undef NormalEstimation_RECURSES
#endif // else defined(NormalEstimation_RECURSES)
