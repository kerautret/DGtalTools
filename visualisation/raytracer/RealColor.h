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
 * @file RealColor.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(RealColor_RECURSES)
#error Recursive header files inclusion detected in RealColor.h
#else // defined(RealColor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RealColor_RECURSES

#if !defined RealColor_h
/** Prevents repeated inclusion of headers. */
#define RealColor_h

#include "raytracer/Ray.h"
#include "DGtal/io/Color.h"

namespace DGtal {
  namespace rt {
    /// This structure codes the color of an object, as well as its
    /// transparency. Color channels are stored as floating point values
    /// between 0 and 1.
    struct RealColor {
    private:
      Vector3 my_channels;
      mutable float gl_conv[ 4 ];
      
    public:
      RealColor() : my_channels( 0.0, 0.0, 0.0 ) {}
      RealColor( Real red, Real green, Real blue )
        : my_channels( red, green, blue ) 
      {
        clamp();
      }
      /// Garantees that color channels are between 0 and 1.
      RealColor& clamp()
      {
        my_channels[ 0 ] = std::max( 0.0, std::min( 1.0, my_channels[ 0 ] ) );
        my_channels[ 1 ] = std::max( 0.0, std::min( 1.0, my_channels[ 1 ] ) );
        my_channels[ 2 ] = std::max( 0.0, std::min( 1.0, my_channels[ 2 ] ) );
        return *this;
      }
      // Useful for conversion to OpenGL vectors
      operator float*()
      {
        gl_conv[ 0 ] = (float) my_channels[ 0 ];
        gl_conv[ 1 ] = (float) my_channels[ 1 ];
        gl_conv[ 2 ] = (float) my_channels[ 2 ];
        gl_conv[ 3 ] = 1.0f;
        return gl_conv;
      }
      // Useful for conversion to OpenGL vectors
      operator const float*() const
      {
        gl_conv[ 0 ] = (float) my_channels[ 0 ];
        gl_conv[ 1 ] = (float) my_channels[ 1 ];
        gl_conv[ 2 ] = (float) my_channels[ 2 ];
        gl_conv[ 3 ] = 1.0f;
        return gl_conv;
      }
      operator Color() const
      {
        unsigned char red   = (unsigned char) round( 255.0 * std::max( 0.0, std::min( 1.0, my_channels[ 0 ] ) ) );
        unsigned char green = (unsigned char) round( 255.0 * std::max( 0.0, std::min( 1.0, my_channels[ 1 ] ) ) );
        unsigned char blue  = (unsigned char) round( 255.0 * std::max( 0.0, std::min( 1.0, my_channels[ 2 ] ) ) );
        return Color( red, green, blue );
      }
      
      Real  r() const { return my_channels[ 0 ]; }
      Real  g() const { return my_channels[ 1 ]; }
      Real  b() const { return my_channels[ 2 ]; }
      Real& r()       { return my_channels[ 0 ]; }
      Real& g()       { return my_channels[ 1 ]; }
      Real& b()       { return my_channels[ 2 ]; }

      // Operations between colors
      RealColor operator*( Real v ) const
      {
        RealColor tmp( *this );
        tmp.my_channels[ 0 ] *= v;
        tmp.my_channels[ 1 ] *= v;
        tmp.my_channels[ 2 ] *= v;
        return tmp;
      }

      // Operations between colors
      RealColor operator*( RealColor other ) const
      {
        RealColor tmp( *this );
        tmp.my_channels[ 0 ] *= other.my_channels[ 0 ];
        tmp.my_channels[ 1 ] *= other.my_channels[ 1 ];
        tmp.my_channels[ 2 ] *= other.my_channels[ 2 ];
        return tmp;
      }

      // Operations between colors
      RealColor operator+( RealColor other ) const
      {
        RealColor tmp( *this );
        tmp.my_channels[ 0 ] += other.my_channels[ 0 ];
        tmp.my_channels[ 1 ] += other.my_channels[ 1 ];
        tmp.my_channels[ 2 ] += other.my_channels[ 2 ];
        return tmp;
      }

      // Operations between colors
      RealColor& operator+=( RealColor other )
      {
        my_channels[ 0 ] += other.my_channels[ 0 ];
        my_channels[ 1 ] += other.my_channels[ 1 ];
        my_channels[ 2 ] += other.my_channels[ 2 ];
        return *this;
      }

      RealColor sup( RealColor other ) const
      {
        other[ 0 ] = std::max( (*this)[ 0 ], other[ 0 ] );
        other[ 1 ] = std::max( (*this)[ 1 ], other[ 1 ] );
        other[ 2 ] = std::max( (*this)[ 2 ], other[ 2 ] );
        return other;
      }
    
      enum Channel { Red, Green, Blue };
      Channel argmax() const 
      {
        if ( r() >= g() ) return r() >= b() ? Red : Blue;
        else              return g() >= b() ? Green : Blue;
      }
      Real max() const { return std::max( std::max( r(), g() ), b() ); }
      Real min() const { return std::min( std::min( r(), g() ), b() ); }

      void getHSV( int & h, Real & s, Real & v ) const
      {
        // Taking care of hue
        if ( max() == min() ) h = 0;
        else {
          switch ( argmax() ) {
          case Red:   h = ( (int) ( 60.0 * ( g() - b() ) / ( max() - min() ) + 360.0 ) ) % 360;
            break;
          case Green: h = ( (int) ( 60.0 * ( b() - r() ) / ( max() - min() ) + 120.0 ) );
            break;
          case Blue:  h = ( (int) ( 60.0 * ( r() - g() ) / ( max() - min() ) + 240.0 ) );
            break;
          }
        }
        // Taking care of saturation
        s = max() == 0.0 ? 0.0 : 1.0 - min() / max();
        // Taking care of value
        v = max();
      }

      void setHSV( int h, Real s, Real v )
      {
        int t = ( h / 60 ) % 6;
        Real f = ( (Real) h / 60.0 ) - (Real) t;
        Real bv = v;
        Real bl = (int) ( v * ( 1 - s ) );
        Real bm = (int) ( v * ( 1 - f * s ) );
        Real bn = (int) ( v * ( 1 - ( 1 - f ) * s ) );
        switch ( t ) {
        case 0: r() = bv; g() = bn; b() = bl; break;
        case 1: r() = bm; g() = bv; b() = bl; break;
        case 2: r() = bl; g() = bv; b() = bn; break;
        case 3: r() = bl; g() = bm; b() = bv; break;
        case 4: r() = bn; g() = bl; b() = bv; break;
        case 5: r() = bv; g() = bl; b() = bm; break;
        }
        clamp();
      }
    };

    // Operations between colors
    inline RealColor operator*( Real v, const RealColor& other )
    {
      RealColor tmp( other );
      return tmp * v;
    }

    inline Real distance( const RealColor& c1, const RealColor& c2 )
    {
      return std::max( std::fabs( c1.r() - c2.r() ),
                       std::max( std::fabs( c1.g() - c2.g() ),
                                 std::fabs( c1.b() - c2.b() ) ) );
    }

  } // namespace rt
} // namespace DGtal

#endif // !defined RealColor_ha

#undef RealColor_RECURSES
#endif // else defined(RealColor_RECURSES)

