/**
@file Color.h
@author JOL
*/
#pragma once
#ifndef _COLOR_HPP_
#define _COLOR_HPP_

#include "PointVector.h"

namespace rt {
  /// This structure codes the color of an object, as well as its
  /// transparency. Color channels are stored as floating point values
  /// between 0 and 1.
  struct Color {
  private:
    Vector3 my_channels;

  public:
    Color() : my_channels( 0.0, 0.0, 0.0 ) {}
    Color( Real red, Real green, Real blue )
      : my_channels( red, green, blue ) 
    {
      clamp();
    }
    /// Garantees that color channels are between 0 and 1.
    Color& clamp()
    {
      my_channels[ 0 ] = std::max( 0.0f, std::min( 1.0f, my_channels[ 0 ] ) );
      my_channels[ 1 ] = std::max( 0.0f, std::min( 1.0f, my_channels[ 1 ] ) );
      my_channels[ 2 ] = std::max( 0.0f, std::min( 1.0f, my_channels[ 2 ] ) );
      return *this;
    }
    // Useful for conversion to OpenGL vectors
    operator float*()             { return my_channels; }
    // Useful for conversion to OpenGL vectors
    operator const float*() const { return my_channels; }

    Real  r() const { return my_channels[ 0 ]; }
    Real  g() const { return my_channels[ 1 ]; }
    Real  b() const { return my_channels[ 2 ]; }
    Real& r()       { return my_channels[ 0 ]; }
    Real& g()       { return my_channels[ 1 ]; }
    Real& b()       { return my_channels[ 2 ]; }

    // Operations between colors
    Color operator*( Real v ) const
    {
      Color tmp( *this );
      tmp.my_channels[ 0 ] *= v;
      tmp.my_channels[ 1 ] *= v;
      tmp.my_channels[ 2 ] *= v;
      return tmp;
    }

    // Operations between colors
    Color operator*( Color other ) const
    {
      Color tmp( *this );
      tmp.my_channels[ 0 ] *= other.my_channels[ 0 ];
      tmp.my_channels[ 1 ] *= other.my_channels[ 1 ];
      tmp.my_channels[ 2 ] *= other.my_channels[ 2 ];
      return tmp;
    }

    // Operations between colors
    Color operator+( Color other ) const
    {
      Color tmp( *this );
      tmp.my_channels[ 0 ] += other.my_channels[ 0 ];
      tmp.my_channels[ 1 ] += other.my_channels[ 1 ];
      tmp.my_channels[ 2 ] += other.my_channels[ 2 ];
      return tmp;
    }

    // Operations between colors
    Color& operator+=( Color other )
    {
      my_channels[ 0 ] += other.my_channels[ 0 ];
      my_channels[ 1 ] += other.my_channels[ 1 ];
      my_channels[ 2 ] += other.my_channels[ 2 ];
      return *this;
    }

    Color sup( Color other ) const
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
  inline Color operator*( Real v, const Color& other )
    {
      Color tmp( other );
      return tmp * v;
    }

  inline Real distance( const Color& c1, const Color& c2 )
  {
    return std::max( std::fabs( c1.r() - c2.r() ),
                     std::max( std::fabs( c1.g() - c2.g() ),
                               std::fabs( c1.b() - c2.b() ) ) );
  }
} // namespace rt

#endif //_COLOR_HPP_
