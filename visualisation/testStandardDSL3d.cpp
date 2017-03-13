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
/**
 * @file testStandardDSL3d.cpp
 * @ingroup Tools
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * A tool file named testStandardDSL3d.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "raytracer/Ray.h"
#include "raytracer/StandardDSL3d.h"

using namespace std;
using namespace DGtal;
using namespace DGtal::rt;

bool testDSLRay( const Ray& ray, int precision )
{
  static const Real distance = sqrt(3.0);
  int nbok = 0, nb = 0;
  StandardDSL3d D( ray, precision );
  auto it = D.begin( D.getPoint( ray.origin ) );
  Point3i prev = *it;
  Vector3 u    = ray.direction / ray.direction.norm();
  for ( unsigned i = 0; i < precision; i++ )
    {
      // Point is close to ray
      Point3 pos( *it );
      Vector3 n = ( pos - ray.origin )  - u.dot( pos - ray.origin ) * u;
      if ( n.norm() <= distance ) nbok++; 
      // Point is correct
      if ( D.isInDSL( *it ) ) nbok++;
      it++;
      // Point is just beside previous one.
      if ( (*it - prev).norm1() == 1 ) nbok++;
      // Point is after previous one.
      if ( D.before( prev, *it ) ) nbok++;
      prev = *it;
      nb  += 4;
    }
  if ( nbok != nb )
    std::cout << "(" << nbok << "/" << nb << ")"
              << " for 3d ray orig=" << ray.origin
              << " dir=" << ray.direction << " prec=" << precision << std::endl;
  return nbok == nb;
}

double rand01()
{
  return (double) rand() / (double) RAND_MAX;
}

int main( int argc, char** argv )
{
  int nbok = 0, nb = 10000;
  bool ok = true;
  for ( int i = 0; i < nb; ++i )
    {
      Vector3 dir( rand01()-0.5, rand01()-0.5, rand01()-0.5 );
      while ( fabs( dir.norm() ) < 0.000001 )
        dir = Vector3( rand01()-0.5, rand01()-0.5, rand01()-0.5 );
      dir /= dir.norm();
      Vector3 p  ( 10.0*(rand01()-0.5), 10.0*(rand01()-0.5), 10.0*(rand01()-0.5) );
      int precision = (int) ( rand01()*1000.0+5.0 );
      if ( testDSLRay( Ray( p, dir ), precision ) )
        nbok++;
      else
        ok = false;
    }
  std::cout << "(" << nbok << "/" << nb << ") tests passed." << std::endl;
  return 0;
}
