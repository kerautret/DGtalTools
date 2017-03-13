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
 * @file Renderer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(Renderer_RECURSES)
#error Recursive header files inclusion detected in Renderer.h
#else // defined(Renderer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Renderer_RECURSES

#if !defined Renderer_h
/** Prevents repeated inclusion of headers. */
#define Renderer_h

#include "raytracer/RealColor.h"
#include "raytracer/Ray.h"
#include "raytracer/Background.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/base/Trace.h"

namespace DGtal {
  namespace rt {

  
    /// This structure takes care of rendering a scene.
    /// object should have.
    struct Renderer {
      /// The scene to render
      Scene* ptrScene;
      /// The origin of the camera in space.
      Point3 myOrigin;
      /// (myOrigin, myOrigin+myDirUL) forms a ray going through the upper-left
      /// corner pixel of the viewport, i.e. pixel (0,0)
      Vector3 myDirUL;
      /// (myOrigin, myOrigin+myDirUR) forms a ray going through the upper-right
      /// corner pixel of the viewport, i.e. pixel (width,0)
      Vector3 myDirUR;
      /// (myOrigin, myOrigin+myDirLL) forms a ray going through the lower-left
      /// corner pixel of the viewport, i.e. pixel (0,height)
      Vector3 myDirLL;
      /// (myOrigin, myOrigin+myDirLR) forms a ray going through the lower-right
      /// corner pixel of the viewport, i.e. pixel (width,height)
      Vector3 myDirLR;

      int myWidth;
      int myHeight;

      /// The background model
      Background* ptrBackground;

    Renderer() : ptrScene( 0 ), ptrBackground( 0 ) {}
    Renderer( Scene& scene ) : ptrScene( &scene ), ptrBackground( 0 ) {}
      void setScene( rt::Scene& aScene ) { ptrScene = &aScene; }
    
      void setViewBox( Point3 origin, 
                       Vector3 dirUL, Vector3 dirUR, Vector3 dirLL, Vector3 dirLR )
      {
        myOrigin = origin;
        myDirUL = dirUL;
        myDirUR = dirUR;
        myDirLL = dirLL;
        myDirLR = dirLR;
      }

      void setResolution( int width, int height )
      {
        myWidth  = width;
        myHeight = height;
      }

      void setBackground( Background* ptr ) { ptrBackground = ptr; }

      /// The main rendering routine
      template <typename Image2D>
      void render( Image2D& image, int max_depth )
      {
        std::cout << "Rendering into image ... might take a while." << std::endl;
        setResolution( image.extent()[ 0 ], image.extent()[ 1 ] );
        for ( int y = 0; y < myHeight; ++y ) 
          {
            Real    ty   = (Real) y / (Real)(myHeight-1);
            DGtal::trace.progressBar( ty, 1.0 );
            Vector3 dirL = (1.0 - ty) * myDirUL + ty * myDirLL;
            Vector3 dirR = (1.0 - ty) * myDirUR + ty * myDirLR;
            dirL        /= dirL.norm();
            dirR        /= dirR.norm();
            for ( int x = 0; x < myWidth; ++x ) 
              {
                Real    tx   = (Real) x / (Real)(myWidth-1);
                Vector3 dir  = (1.0 - tx) * dirL + tx * dirR;
                Ray eye_ray  = Ray( myOrigin, dir, max_depth );
                RealColor result = trace( eye_ray );
                result.clamp();
                image.setValue( Point2i( x, y ), result );
              }
          }
        std::cout << "Done." << std::endl;
      }


      /// The rendering routine for one ray.
      /// @return the color for the given ray.
      RealColor trace( const Ray& ray )
      {
        assert( ptrScene != 0 );
        RealColor result = RealColor( 0.0, 0.0, 0.0 );
        GraphicalObject* obj_i = 0; // pointer to intersected object
        Point3           p_i;       // point of intersection

        // Look for intersection in this direction.
        Real ri = ptrScene->rayIntersection( ray, obj_i, p_i );
        // Nothing was intersected
        if ( ri >= 0.0 ) // should be some background color
          return background( ray );
        Material m = obj_i->getMaterial( p_i );
        Vector3  N = obj_i->getNormal  ( p_i );
        if ( ( ray.depth > 0 ) && ( m.coef_reflexion > 0.0 ) )
          {
            RealColor C_refl = trace( reflexionRay( ray, p_i, N ) );
            result      += ( C_refl * m.specular ) * m.coef_reflexion;
          }
        if ( ( ray.depth > 0 ) && ( m.coef_refraction > 0.0 ) )
          {
            RealColor C_refr = trace( refractionRay( ray, p_i, N, m ) );
            result      += ( C_refr * m.diffuse ) * m.coef_refraction;
          }
        result += ( ( ray.depth > 0 ) ? m.coef_diffusion : 1.0 ) * illumination( ray, m, N, p_i );
        return result;
      }

      Ray reflexionRay( const Ray& aRay, const Point3& p, const Vector3& n )
      {
        return Ray( p, reflect( aRay.direction, n ), aRay.depth - 1 );
      }

      Ray refractionRay( const Ray& aRay, const Point3& p,
                         Vector3 n, const Material& m )
      {
        // Snell's law (wikipedia)
        // l : light ray
        // n1 : refraction index in-media, n2 : refraction index out-media
        // let r = n1/n2, c = - n . l
        // v_refract = r l + ( rc - sqrt( 1 - r^2( 1 - c^2 ) ) ) n
        Real c = - n.dot( aRay.direction );
        if ( c < 0.0 )
          { // ray is inside and must go outside
            Real r = m.in_refractive_index / m.out_refractive_index;
            c = -c; n *= -1.0;
            Vector3 v_refract = r * aRay.direction
              + ( r*c - (Real)sqrt( std::max( 0.0, 1.0 - r*r*(1-c*c) ) ) ) * n;
            return Ray( p, v_refract, aRay.depth - 1 );
          }
        else
          { // ray is outside and must go inside
            Real r = m.out_refractive_index / m.in_refractive_index;
            Vector3 v_refract = r * aRay.direction
              + ( r*c - (Real)sqrt( std::max( 0.0, 1.0 - r*r*(1-c*c) ) ) ) * n;
            return Ray( p, v_refract, aRay.depth - 1 );
          }
      }

    
      /// Calcule le vecteur réfléchi à V selon la normale N.
      Vector3 reflect( const Vector3& V, Vector3 N ) const
      {
        // V : light ray from eye
        // V_reflect = V - 2 (N . V) N
        Real n_dot_v = N.dot( V );
        if ( n_dot_v > 0.0 ) { n_dot_v = -n_dot_v; N *= -1.0; }
        return V - ( 2.0 * n_dot_v ) * N;
      }

      RealColor shadow( const Ray& ray, RealColor light_color )
      {
        GraphicalObject* shadow_obj = 0;
        Point3 moving_p = ray.origin;
        Point3 shadow_p;
        while ( light_color.max() > 0.003 )
          {
            Ray shadow_ray( moving_p + RT_EPSILON*ray.direction, ray.direction );
            Real shadow_d2 = ptrScene->rayIntersection( shadow_ray, shadow_obj, shadow_p );
            if ( shadow_d2 >= 0.0 ) break; // found no intersection.
            // Checks if the intersected material was transparent
            Material shadow_m = shadow_obj->getMaterial( shadow_p );
            light_color = ( light_color * shadow_m.diffuse ) * shadow_m.coef_refraction;
            moving_p = shadow_p; // go on
          }
        return light_color;
      }

      /// Calcule l'illumination du matériau \a m de normal \a N au point \a p, sachant que l'observateur est le rayon \a ray.
      RealColor illumination( const Ray& ray, const Material& m, const Vector3& N, const Point3& p )
      {
        RealColor result( 0,0,0 );
        for ( Light* light : ptrScene->myLights )
          {
            Vector3 L = light->direction( p );
            Vector3 W = reflect( ray.direction, N );
            Real N_cos_angle = N.dot( L );
            Real W_cos_angle = W.dot( L ); 
            Real k_d = std::max( N_cos_angle, 0.0 );
            Real k_s = pow( std::max( W_cos_angle, 0.0 ), m.shinyness );
            RealColor light_color = shadow( Ray( p, L ), light->color( p ) );
            result += m.diffuse  * light_color * k_d;
            result += m.specular * light_color * k_s;
          }
        return result + m.ambient;
      }

      // Affiche les sources de lumières avant d'appeler la fonction qui
      // donne la couleur de fond.
      RealColor background( const Ray& ray )
      {
        RealColor result = RealColor( 0.0, 0.0, 0.0 );
        for ( Light* light : ptrScene->myLights )
          {
            Real cos_a = light->direction( ray.origin ).dot( ray.direction );
            if ( cos_a > 0.99f )
              {
                Real a = acos( cos_a ) * 360.0 / M_PI / 8.0;
                a = std::max( 1.0 - a, 0.0 );
                result += light->color( ray.origin ) * a * a;
              }
          }
        if ( ptrBackground != 0 ) 
          // pas réaliste, mais suffisant dans notre cas.
          result += ptrBackground->backgroundColor( ray );
        return result;
      }

    }; // class Renderer

  } // namespace rt
} // namespace DGtal
  
#endif // !defined Renderer_h

#undef Renderer_RECURSES
#endif // else defined(Renderer_RECURSES)

