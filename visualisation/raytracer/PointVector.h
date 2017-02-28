/**
@file PointVector.h
@author JOL
*/
#pragma once
#ifndef _POINT_VECTOR_H_
#define _POINT_VECTOR_H_

#include <cassert>
#include <cmath>
#include <array>

/// Namespace RayTracer
namespace rt {

  /**
  Model a static vector T[N], with some operations.
  */
  template <typename T, std::size_t N>
  struct PointVector : public std::array<T, N> {
    typedef PointVector<T, N> Self;
    typedef std::array<T, N>   Base;
    typedef std::size_t        Size;

    using Base::begin;
    using Base::end;
    using Base::size;
    using Base::max_size;
    using Base::operator[];
    using Base::at;
    using Base::front;
    using Base::back;
    using Base::data;
    
    PointVector() {}

    PointVector( std::initializer_list<T> L )
    {
      Size i = 0;
      for ( auto v : L ) if ( i < N ) (*this)[ i++ ] = v;
    }
    PointVector( T val0 )
    {
      assert( 0 < N );
      (*this)[ 0 ] = val0;
    }
    PointVector( T val0, T val1 )
    {
      assert( 1 < N );
      (*this)[ 0 ] = val0;
      (*this)[ 1 ] = val1;
    }
    PointVector( T val0, T val1, T val2 )
    {
      assert( 2 < N );
      (*this)[ 0 ] = val0;
      (*this)[ 1 ] = val1;
      (*this)[ 2 ] = val2;
    }
    PointVector( T val0, T val1, T val2, T val3 )
    {
      assert( 3 < N );
      (*this)[ 0 ] = val0;
      (*this)[ 1 ] = val1;
      (*this)[ 2 ] = val2;
      (*this)[ 3 ] = val3;
    }
    PointVector( const T* vals )
    {
      for ( Size i = 0; i < N; i++ ) (*this)[ i ] = *vals++;
    }
    // Useful for conversion to OpenGL vectors
    operator T*()             { return data(); }
    // Useful for conversion to OpenGL vectors
    operator const T*() const { return data(); }

    void selfDisplay( std::ostream& out ) const
    {
      out << "(";
      for ( Size i = 0; i < N; i++ )
        out << (*this)[ i ] << ( ( i < N-1 ) ? ',' : ')' );
    }
  
    Self& operator+=( const Self& other )
    {
      for ( Size i = 0; i < N; ++i ) (*this)[ i ] += other[ i ];
      return *this;
    }
    Self& operator-=( const Self& other )
    {
      for ( Size i = 0; i < N; ++i ) (*this)[ i ] -= other[ i ];
      return *this;
    }
    Self& operator*=( T val )
    {
      for ( Size i = 0; i < N; ++i ) (*this)[ i ] *= val;
      return *this;
    }
    Self& operator/=( T val )
    {
      for ( Size i = 0; i < N; ++i ) (*this)[ i ] /= val;
      return *this;
    }

    /// dot product (produit scalaire).
    T dot( const Self& other ) const
    {
      T result = 0;
      for ( Size i = 0; i < N; ++i ) result += (*this)[ i ] * other[ i ];
      return result;
    }
    /// cross product (produit vectoriel).
    Self cross( const Self& other ) const
    {
      assert( N == 3 );
      return Self( (*this)[1]*other[2] - (*this)[2]*other[1],
                   (*this)[2]*other[0] - (*this)[0]*other[2],
                   (*this)[0]*other[1] - (*this)[1]*other[0] );
    }

    Self operator+( const Self& other ) const
    {
      Self result( *this );
      result += other;
      return result;
    }

    Self operator-( const Self& other ) const
    {
      Self result( *this );
      result -= other;
      return result;
    }

    T norm() const
    {
      return sqrt( dot( *this ) );
    }
  };

  ///////////////////////////////////////////////////////////////////////////////
  // A few useful operators
  ///////////////////////////////////////////////////////////////////////////////

  template <typename T, std::size_t N>
  std::ostream& operator<<( std::ostream& out, const PointVector<T,N>& PV )
  { 
    PV.selfDisplay( out );
    return out;
  }

  template <typename T, std::size_t N>
  PointVector<T,N> operator*( T val, const PointVector<T,N>& PV )
  {
    typedef typename PointVector<T,N>::Size Size;
    PointVector<T,N> result( PV );
    for ( Size i = 0; i < N; ++i ) result[ i ] *= val;
    return result;
  }

  template <typename T, std::size_t N>
  PointVector<T,N> operator*( const PointVector<T,N>& PV, T val )
  {
    typedef typename PointVector<T,N>::Size Size;
    PointVector<T,N> result( PV );
    for ( Size i = 0; i < N; ++i ) result[ i ] *= val;
    return result;
  }

  template <typename T, std::size_t N>
  PointVector<T,N> operator/( T val, const PointVector<T,N>& PV )
  {
    typedef typename PointVector<T,N>::Size Size;
    PointVector<T,N> result( PV );
    for ( Size i = 0; i < N; ++i ) result[ i ] = val / result[ i ];
    return result;
  }

  template <typename T, std::size_t N>
  PointVector<T,N> operator/( const PointVector<T,N>& PV, T val )
  {
    typedef typename PointVector<T,N>::Size Size;
    PointVector<T,N> result( PV );
    for ( Size i = 0; i < N; ++i ) result[ i ] /= val;
    return result;
  }

  template <typename T, std::size_t N>
  inline
  T distance2( const PointVector<T,N>& p1, const PointVector<T,N>& p2 )
  {
    PointVector<T,N> tmp = p2 - p1;
    return tmp.dot( tmp );
  } 

  template <typename T, std::size_t N>
  inline
  T distance( const PointVector<T,N>& p1, const PointVector<T,N>& p2 )
  {
    return sqrt( distance2( p1, p2 ) );
  } 
  ///////////////////////////////////////////////////////////////////////////////
  // Used types
  ///////////////////////////////////////////////////////////////////////////////
  /// the type for representing a real number.
  typedef float                   Real;
  typedef PointVector< Real, 2 > Vector2;
  typedef PointVector< Real, 2 > Point2;
  /// The type for representing a 3d point.
  typedef PointVector< Real, 3 > Vector3;
  /// The type for representing a 3d vector;
  typedef PointVector< Real, 3 > Point3;
  typedef PointVector< Real, 4 > Vector4;
  typedef PointVector< Real, 4 > Point4;

} // namespace rt

#endif // _POINT_VECTOR_H_

