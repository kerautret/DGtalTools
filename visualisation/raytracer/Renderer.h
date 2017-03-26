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

      /// @return a random real number between 0 and 1.
      static Real rand01() { return (Real) ((double) random() / (double) RAND_MAX); }

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
                RealColor result = traceWithAccumulation( eye_ray, 1.0 );
                result.clamp();
                image.setValue( Point2i( x, y ), result );
              }
          }
        std::cout << "Done." << std::endl;
      }
      
      /// The main rendering routine
      template <typename Image2D>
      void renderRandom( Image2D& image, int max_depth, int nb_samples, int nb_casts )
      {
        std::cout << "Random rendering into image ... might take a while." << std::endl;
        setResolution( image.extent()[ 0 ], image.extent()[ 1 ] );
        int nb_ray = 0;
        for ( int y = 0; y < myHeight; ++y ) 
          {
            Real ty_up = (((Real) y)-0.5f) / (Real)(myHeight);
            Real ty_lo = (((Real) y)+0.5f) / (Real)(myHeight);
            DGtal::trace.progressBar( ty_up, 1.0 );
            Vector3 dirL_up = (1.0f - ty_up) * myDirUL + ty_up * myDirLL;
            Vector3 dirR_up = (1.0f - ty_up) * myDirUR + ty_up * myDirLR;
            Vector3 dirL_lo = (1.0f - ty_lo) * myDirUL + ty_lo * myDirLL;
            Vector3 dirR_lo = (1.0f - ty_lo) * myDirUR + ty_lo * myDirLR;
            dirL_up /= dirL_up.norm();
            dirR_up /= dirR_up.norm();
            dirL_lo /= dirL_lo.norm();
            dirR_lo /= dirR_lo.norm();
            for ( int x = 0; x < myWidth; ++x ) 
              {
                Real tx_lt = (((Real) x)-0.5f) / (Real)(myWidth);
                Real tx_rt = (((Real) x)+0.5f) / (Real)(myWidth);
                Vector3 dir_ul = (1.0f - tx_lt) * dirL_up + tx_lt * dirR_up;
                Vector3 dir_ur = (1.0f - tx_rt) * dirL_up + tx_rt * dirR_up;
                Vector3 dir_ll = (1.0f - tx_lt) * dirL_lo + tx_lt * dirR_lo;
                Vector3 dir_lr = (1.0f - tx_rt) * dirL_lo + tx_rt * dirR_lo;
                dir_ul /= dir_ul.norm();
                dir_ur /= dir_ur.norm();
                dir_ll /= dir_ll.norm();
                dir_lr /= dir_lr.norm();
                RealColor total_color = RealColor( 0.0, 0.0, 0.0 );
                int k = 0;
                const int max_k = nb_samples*nb_casts;
                while ( k < max_k )
                  {
                    Real rx = rand01();
                    Real ry = rand01();
                    Vector3 dir = (1.0f - rx ) * (1.0f - ry ) * dir_ul
                      + rx * (1.0f - ry ) * dir_ur
                      + (1.0f - rx) * ry * dir_ll + rx * ry * dir_lr;
                    dir /= dir.norm();
                    Ray eye_ray( myOrigin, dir, max_depth );
                    for ( int l = 0; l < nb_casts; ++l )
                      {
                        RealColor result = randomTrace( eye_ray );
                        total_color += result;
                        nb_ray++;
                        k++;
                      }
                    /* Real diff = distance( result, total_color * (1.0f / (Real) k ) ); */
                    /* if ( ( k > 2 ) && ( diff < ( 0.005f * (Real) k ) ) ) break; */
                  }
                total_color = total_color * (1.0f / (Real) k );
                total_color.clamp();
                image.setValue( Point2i( x, y ), total_color );
              }
          }
        std::cout << "Done." << std::endl
                  << "Nb rays casted = " << nb_ray
                  << " Avg rays/pixel = "
                  << ( (double) nb_ray / (double) (myWidth*myHeight) ) << std::endl;
      }
      

      /// The rendering routine for one ray.
      /// @return the color for the given ray.
      RealColor trace( const Ray& ray )
      {
        assert( ptrScene != 0 );
        RealColor result = RealColor( 0.0, 0.0, 0.0 );
        GraphicalObject* obj_i = 0; // pointer to intersected object
        RayIntersection  ray_inter( ray );
        ray_inter.ray.origin += RT_EPSILON * ray_inter.ray.direction;
        // Look for intersection in this direction.
        bool intersection = ptrScene->intersectRay( ray_inter, obj_i );
        // Nothing was intersected: should be some background color
        if ( ! intersection  ) return background( ray );
        Material m = obj_i->getMaterial( ray_inter.intersection );
        Vector3  N = ray_inter.normal; // obj_i->getNormal  ( p_i );
        bool out2in = N.dot( ray_inter.ray.direction ) <= 0.0;
        ray_inter.in_refractive_index =
          out2in ? m.in_refractive_index : m.out_refractive_index;
        ray_inter.out_refractive_index =
          out2in ? m.out_refractive_index : m.in_refractive_index;
        if ( ( ray.depth > 0 ) && ( m.coef_reflexion > 0.0 ) )
          {
            // RealColor C_refl = trace( reflexionRay( ray, p_i, N ) );
            RealColor C_refl = trace( ray_inter.reflexionRay() );
            result      += ( C_refl * m.specular ) * m.coef_reflexion;
          }
        if ( ( ray.depth > 0 ) && ( m.coef_refraction > 0.0 ) )
          {
            // RealColor C_refr = trace( refractionRay( ray, p_i, N, m ) );
            // JOL: TODO, set refractive indices in ray_inter
            RealColor C_refr = trace( ray_inter.refractionRay() );
            result      += ( C_refr * m.diffuse ) * m.coef_refraction;
          }
        result += ( ( ray.depth > 0 ) ? m.coef_diffusion : 1.0 )
          * illumination( ray, m, N, ray_inter.intersection );
        return result + m.ambient;
      }

      /// The rendering routine for one ray.
      /// @return the color for the given ray.
      RealColor traceWithAccumulation( const Ray& ray, Real accumulation )
      {
        assert( ptrScene != 0 );
        RealColor result = RealColor( 0.0, 0.0, 0.0 );
        if ( accumulation < 0.003 ) return result;
        GraphicalObject* obj_i = 0; // pointer to intersected object
        RayIntersection  ray_inter( ray );
        ray_inter.ray.origin += RT_EPSILON * ray_inter.ray.direction;
        // Look for intersection in this direction.
        bool intersection = ptrScene->intersectRay( ray_inter, obj_i );
        // Nothing was intersected: should be some background color
        if ( ! intersection  ) return accumulation * background( ray );
        Material m = obj_i->getMaterial( ray_inter.intersection );
        Vector3  N = ray_inter.normal; // obj_i->getNormal  ( p_i );
        bool out2in = N.dot( ray_inter.ray.direction ) <= 0.0;
        ray_inter.in_refractive_index =
          out2in ? m.in_refractive_index : m.out_refractive_index;
        ray_inter.out_refractive_index =
          out2in ? m.out_refractive_index : m.in_refractive_index;
        if ( ( ray.depth > 0 ) && ( m.coef_reflexion > 0.0 ) )
          {
            // RealColor C_refl = trace( reflexionRay( ray, p_i, N ) );
            RealColor C_refl =
              traceWithAccumulation( ray_inter.reflexionRay(),
                                     accumulation * m.coef_reflexion );
            result      += ( C_refl * m.specular );
          }
        if ( ( ray.depth > 0 ) && ( m.coef_refraction > 0.0 ) )
          {
            // RealColor C_refr = trace( refractionRay( ray, p_i, N, m ) );
            // JOL: TODO, set refractive indices in ray_inter
            RealColor C_refr =
              traceWithAccumulation( ray_inter.refractionRay(),
                                     accumulation * m.coef_refraction );
            result      += ( C_refr * m.diffuse );
          }
        result += accumulation * ( ( ray.depth > 0 ) ? m.coef_diffusion : 1.0 )
          * ( illumination( ray, m, N, ray_inter.intersection ) + m.ambient);
        return result;
      }

      /// The rendering routine for one ray.
      /// @return the color for the given ray.
      RealColor randomTrace( const Ray& ray )
      {
        assert( ptrScene != 0 );
        RealColor result = RealColor( 0.0, 0.0, 0.0 );
        GraphicalObject* obj_i = 0; // pointer to intersected object
        RayIntersection  ray_inter( ray );
        ray_inter.ray.origin += RT_EPSILON * ray_inter.ray.direction;
        // Look for intersection in this direction.
        bool intersection = ptrScene->intersectRay( ray_inter, obj_i );
        // Nothing was intersected: should be some background color
        if ( ! intersection  ) return background( ray );

        // Compute material and normal at intersection.
        Material m = obj_i->getMaterial( ray_inter.intersection );
        Vector3  N = ray_inter.normal; // obj_i->getNormal  ( p_i );
        bool out2in = N.dot( ray_inter.ray.direction ) <= 0.0;
        ray_inter.in_refractive_index =
          out2in ? m.in_refractive_index : m.out_refractive_index;
        ray_inter.out_refractive_index =
          out2in ? m.out_refractive_index : m.in_refractive_index;

        // This random number tells whether we compute
        // diffusion, reflexion or refraction
        Real     c = m.coef_diffusion + m.coef_reflexion + m.coef_refraction;
        Real     p = ray.depth > 0 ? rand01()*c : 0.0f;
        if ( p <= m.coef_diffusion )
          { // Diffusion + ambient
            return m.ambient + c*illumination( ray, m, N, ray_inter.intersection );
          }
        else if ( p <= m.coef_diffusion + m.coef_reflexion )
          { // réflexion
            RealColor C_refl = randomTrace( ray_inter.reflexionRay() );
            return (c*C_refl) * m.specular;
          }
        else 
          { // réfraction
            RealColor C_refr = randomTrace( ray_inter.refractionRay() );
            return (c*C_refr) * m.diffuse;
          }
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

    
      RealColor shadow( const Ray& ray, RealColor light_color )
      {
        GraphicalObject* shadow_obj = 0;
        Point3 moving_p = ray.origin;
        while ( light_color.max() > 0.003 )
          {
            Ray shadow_ray( moving_p + RT_EPSILON*ray.direction, ray.direction );
            RayIntersection shadow_ray_inter( shadow_ray );
            bool intersection = ptrScene->intersectRay( shadow_ray_inter, shadow_obj );
            if ( ! intersection ) break; // found no intersection.
            // Checks if the intersected material was transparent
            Material shadow_m = shadow_obj->getMaterial( shadow_ray_inter.intersection );
            light_color = ( light_color * shadow_m.diffuse ) * shadow_m.coef_refraction;
            moving_p = shadow_ray_inter.intersection; // refraction; // go on
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

