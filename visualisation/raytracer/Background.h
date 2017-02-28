/**
@file Background.h
@author JOL
*/
#pragma once
#ifndef _BACKGROUND_H_
#define _BACKGROUND_H_

#include <cmath>
#include "Color.h"
#include "Image2D.h"
#include "Ray.h"

/// Namespace RayTracer
namespace rt {

  struct Background {
    virtual Color backgroundColor( const Ray& ray ) = 0;
  };

  struct DuskWithChessboard : public Background
    {
      Color backgroundColor( const Ray& ray )
      {
        Color result( 0.0, 0.0, 0.0 );
        if ( ray.direction[ 2 ] >= 0.5f )
          result += Color( 0.0f, 0.0f, 1.5f - ray.direction[ 2 ] );
        else if ( ray.direction[ 2 ] >= 0.0f )
          result += Color( 1.0f - 2.0f*ray.direction[ 2 ],
                           1.0f - 2.0f*ray.direction[ 2 ], 1.0f );
        else
          {
            Real x = -0.5f * ray.direction[ 0 ] / ray.direction[ 2 ];
            Real y = -0.5f * ray.direction[ 1 ] / ray.direction[ 2 ];
            Real d = sqrt( x*x + y*y );
            Real t = std::min( d, 30.0f ) / 30.0f;
            x -= floor( x );
            y -= floor( y );
            if ( ( ( x >= 0.5f ) && ( y >= 0.5f ) )
                 || ( ( x < 0.5f ) && ( y < 0.5f ) ) )
              result += (1.0f - t)*Color( 0.2f, 0.2f, 0.2f ) + t * Color( 1.0f, 1.0f, 1.0f );
            else
              result += (1.0f - t)*Color( 0.4f, 0.4f, 0.4f ) + t * Color( 1.0f, 1.0f, 1.0f );
          }
        return result;
      }
  };

  struct DawnWithChessboard : public Background
    {
      Color backgroundColor( const Ray& ray )
      {
        Color result( 0.0, 0.0, 0.0 );
        Real z = ray.direction[ 2 ];
        if ( z >= 0.6f )
          return Color( 0.0f, 0.0f, 0.1f );
        else if ( z >= 0.4f )
          {
            Real t = (z-0.4f) * 5.0f;
            result += (1.0f - t) * Color( 0.1f, 0.1f, 0.2f ) + t * Color( 0.0f, 0.0f, 0.1f );
          }
        else if ( z >= 0.2f ) 
          {
            Real t = (z-0.2f) * 5.0f;
            result += (1.0f - t) * Color( 0.5f, 0.1f, 0.1f ) + t * Color( 0.1f, 0.1f, 0.2f );
          }
        else if ( z >= 0.0f ) 
          {
            Real t = z * 5.0f;
            result += (1.0f - t) * Color( 1.0f, 1.0f, 0.3f ) + t * Color( 0.5f, 0.1f, 0.1f );
          }
        else
          {
            Real x = -0.5f * ray.direction[ 0 ] / ray.direction[ 2 ];
            Real y = -0.5f * ray.direction[ 1 ] / ray.direction[ 2 ];
            Real d = sqrt( x*x + y*y );
            Real t = std::min( d, 30.0f ) / 30.0f;
            x -= floor( x );
            y -= floor( y );
            if ( ( ( x >= 0.5f ) && ( y >= 0.5f ) )
                 || ( ( x < 0.5f ) && ( y < 0.5f ) ) )
              result += (1.0f - t)*Color( 0.2f, 0.2f, 0.2f ) + t * Color( 1.0f, 1.0f, 0.3f );
            else
              result += (1.0f - t)*Color( 0.4f, 0.4f, 0.4f ) + t * Color( 1.0f, 1.0f, 0.3f );
          }
        return result;
      }
  };

  struct ImageSkyBackground : public Background
    {
      Image2D<Color> sky;
      Real radius;
      Real xc, yc;

      ImageSkyBackground() {}
      ImageSkyBackground( const Image2D<Color>& skyimage )
      {
        setSkyImage( skyimage );
      }

      Color getColor( Real x, Real y ) const
      {
        int i  = (int) floor( x );
        int j  = (int) floor( y );
        Real s = x - (Real) i;
        Real t = y - (Real) j;
        Color c = (1.0f - s) * (1.0f - t) * sky.at( i,   j )
          +            s     * (1.0f - t) * sky.at( i+1, j )
          +            s     *      t     * sky.at( i+1, j+1 )
          +       (1.0f - s) *      t     * sky.at( i,   j+1 );
        return c;
      }
      void setSkyImage( const Image2D<Color>& image )
      {
        sky = image;
        int size = std::min( image.w(), image.h() );
        radius = ((Real) size - 30.0f)/2.0;
        xc = ( (Real) image.w() ) / 2.0f;
        yc = ( (Real) image.h() ) / 2.0f;
      }

      Color fisheye( Vector3 dir )
      {
        Real t = atan2( dir[ 1 ], dir[ 0 ] );
        Real a = sqrt( dir[ 0 ] * dir[ 0 ] + dir[ 1 ] * dir[ 1 ] );
        Real ta= std::max( 0.0f, 1.0f - (Real)sqrt( std::max( 0.0f, 1.0f - a*a ) ) );
        Real x = ta * cos(t) * radius + xc;
        Real y = ta * sin(t) * radius + yc;
        return getColor( x, y );
      }
      
      Color backgroundColor( const Ray& ray )
      {
        Color result( 0.0, 0.0, 0.0 );
        if ( ray.direction[ 2 ] >= 0.05f )
          return fisheye( ray.direction );
        else if ( ray.direction[ 2 ] >= 0.0f )
          {
            Real t = ( 0.05f - ray.direction[ 2 ] ) / 0.05f;
            return (1.0f-t) * fisheye( ray.direction ) + t * Color(0.5f,0.5f,0.5f);
          }
        else
          {
            Real x = -0.5f * ray.direction[ 0 ] / ray.direction[ 2 ];
            Real y = -0.5f * ray.direction[ 1 ] / ray.direction[ 2 ];
            Real d = sqrt( x*x + y*y );
            Real t = std::min( d, 30.0f ) / 30.0f;
            x -= floor( x );
            y -= floor( y );
            if ( ( ( x >= 0.5f ) && ( y >= 0.5f ) )
                 || ( ( x < 0.5f ) && ( y < 0.5f ) ) )
              result += (1.0f - t)*Color( 0.2f, 0.2f, 0.2f ) + t * Color( 1.0f, 1.0f, 1.0f );
            else
              result += (1.0f - t)*Color( 0.4f, 0.4f, 0.4f ) + t * Color( 1.0f, 1.0f, 1.0f );
          }
        return result;
      }
  };

  
} // namespace rt
#endif // define _BACKGROUND_H_
