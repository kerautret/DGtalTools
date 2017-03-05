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
 * @file Background.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(Background_RECURSES)
#error Recursive header files inclusion detected in Background.h
#else // defined(Background_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Background_RECURSES

#if !defined Background_h
/** Prevents repeated inclusion of headers. */
#define Background_h

#include <cmath>
#include "raytracer/RealColor.h"
#include "raytracer/Ray.h"

namespace DGtal {
  namespace rt {

    struct Background {
      virtual RealColor backgroundColor( const Ray& ray ) = 0;
    };

    struct DuskWithChessboard : public Background
    {
      RealColor backgroundColor( const Ray& ray )
      {
        RealColor result( 0.0, 0.0, 0.0 );
        if ( ray.direction[ 2 ] >= 0.5 )
          result += RealColor( 0.0, 0.0, 1.5 - ray.direction[ 2 ] );
        else if ( ray.direction[ 2 ] >= 0.0 )
          result += RealColor( 1.0 - 2.0*ray.direction[ 2 ],
                           1.0 - 2.0*ray.direction[ 2 ], 1.0 );
        else
          {
            Real x = -0.5 * ray.direction[ 0 ] / ray.direction[ 2 ];
            Real y = -0.5 * ray.direction[ 1 ] / ray.direction[ 2 ];
            Real d = sqrt( x*x + y*y );
            Real t = std::min( d, 30.0 ) / 30.0;
            x -= floor( x );
            y -= floor( y );
            if ( ( ( x >= 0.5 ) && ( y >= 0.5 ) )
                 || ( ( x < 0.5 ) && ( y < 0.5 ) ) )
              result += (1.0 - t)*RealColor( 0.2, 0.2, 0.2 ) + t * RealColor( 1.0, 1.0, 1.0 );
            else
              result += (1.0 - t)*RealColor( 0.4, 0.4, 0.4 ) + t * RealColor( 1.0, 1.0, 1.0 );
          }
        return result;
      }
    };

    struct DawnWithChessboard : public Background
    {
      RealColor backgroundColor( const Ray& ray )
      {
        RealColor result( 0.0, 0.0, 0.0 );
        Real z = ray.direction[ 2 ];
        if ( z >= 0.6f )
          return RealColor( 0.0, 0.0, 0.1 );
        else if ( z >= 0.4 )
          {
            Real t = (z-0.4) * 5.0;
            result += (1.0 - t) * RealColor( 0.1, 0.1, 0.2 ) + t * RealColor( 0.0, 0.0, 0.1 );
          }
        else if ( z >= 0.2 ) 
          {
            Real t = (z-0.2) * 5.0;
            result += (1.0 - t) * RealColor( 0.5, 0.1, 0.1 ) + t * RealColor( 0.1, 0.1, 0.2 );
          }
        else if ( z >= 0.0 ) 
          {
            Real t = z * 5.0;
            result += (1.0 - t) * RealColor( 1.0, 1.0, 0.3 ) + t * RealColor( 0.5, 0.1, 0.1 );
          }
        else
          {
            Real x = -0.5 * ray.direction[ 0 ] / ray.direction[ 2 ];
            Real y = -0.5 * ray.direction[ 1 ] / ray.direction[ 2 ];
            Real d = sqrt( x*x + y*y );
            Real t = std::min( d, 30.0 ) / 30.0;
            x -= floor( x );
            y -= floor( y );
            if ( ( ( x >= 0.5 ) && ( y >= 0.5 ) )
                 || ( ( x < 0.5 ) && ( y < 0.5 ) ) )
              result += (1.0 - t)*RealColor( 0.2, 0.2, 0.2 ) + t * RealColor( 1.0, 1.0, 0.3 );
            else
              result += (1.0 - t)*RealColor( 0.4, 0.4, 0.4 ) + t * RealColor( 1.0, 1.0, 0.3 );
          }
        return result;
      }
    };
    
  } // namespace rt
} // namespace DGtal

#endif // !defined Background_ha

#undef Background_RECURSES
#endif // else defined(Background_RECURSES)

