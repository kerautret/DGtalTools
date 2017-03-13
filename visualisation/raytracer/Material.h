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
 * @file Material.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/02/28
 *
 * This file is part of the DGtal library.
 */

#if defined(Material_RECURSES)
#error Recursive header files inclusion detected in Material.h
#else // defined(Material_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Material_RECURSES

#if !defined Material_h
/** Prevents repeated inclusion of headers. */
#define Material_h

#include "raytracer/RealColor.h"

// @see http://devernay.free.fr/cours/opengl/materials.html

namespace DGtal {
  namespace rt {

    /// This structure stores the material associated to a graphical object.
    /// object should have.
    struct Material {
      /// ambient color
      RealColor ambient;
      /// diffuse color
      RealColor diffuse;
      /// specular color
      RealColor specular;
      /// shinyness (50: very high, 1: low )
      Real shinyness;

      // --------- The following data is used in later questions. --------------

      /// The material is perfectly diffuse if coef_diffusion == 1.0,
      /// otherwise it may have reflexion / transparency properties.
      Real coef_diffusion;
      /// The material has some specularity if coef_reflexion > 0.0
      Real coef_reflexion;
      /// The material is transparent if coef_refraction > 0.0
      Real coef_refraction;
      /// Refractive index (1.0f in vacuum, 1.31 in ice, 1.52 window glass, 1.33 water
      /// Inside refractive index (for transparent medium) (>= 1.0f)
      Real in_refractive_index;
      /// Outside refractive index (1.0f if object is in the air otherwise >= 1.0f)
      Real out_refractive_index;

      /// Mixes two material (t=0 gives m1, t=1 gives m2, t=0.5 gives their average)
      static Material mix( Real t, const Material& m1, const Material& m2 )
      {
        Material m;
        Real s = 1.0f - t;
        m.ambient = s * m1.ambient + t * m2.ambient;
        m.diffuse = s * m1.diffuse + t * m2.diffuse;
        m.specular = s * m1.specular + t * m2.specular;
        m.shinyness = s * m1.shinyness + t * m2.shinyness;
        m.coef_diffusion = s * m1.coef_diffusion + t * m2.coef_diffusion;
        m.coef_reflexion = s * m1.coef_reflexion + t * m2.coef_reflexion;
        m.coef_refraction = s * m1.coef_refraction + t * m2.coef_refraction;
        m.in_refractive_index = s * m1.in_refractive_index + t * m2.in_refractive_index;
        m.out_refractive_index = s * m1.out_refractive_index + t * m2.out_refractive_index;
        return m;
      }
    
      /// Default constructor
      Material() {}

      /// Constructor from colors and shinyness.
      Material( RealColor amb, RealColor diff, RealColor spec, Real shiny = 0.0f,
                Real cdiff = 1.0f, Real crefl = 0.0f, Real crefr = 0.0f,
                Real in_ridx = 1.0f, Real out_ridx = 1.0f )
        : ambient( amb ), diffuse( diff ), specular( spec ), shinyness( shiny ),
          coef_diffusion( cdiff ), coef_reflexion( crefl ), coef_refraction( crefr ),
          in_refractive_index( in_ridx ), out_refractive_index( out_ridx )
      {}

      Material revert() const
      {
        Material m = *this;
        std::swap( m.in_refractive_index, m.out_refractive_index );
        return m;
      }
      
      static Material whitePlastic() 
      {
        Material m;
        m.ambient   = RealColor( 0.1, 0.1, 0.1 );
        m.diffuse   = RealColor( 0.7, 0.7, 0.7 );
        m.specular  = RealColor( 1.0, 1.0, 0.98 );
        m.shinyness = 5.0;
        m.coef_diffusion  = 0.9f;
        m.coef_reflexion  = 0.1f;
        m.coef_refraction = 0.0f;
        m.in_refractive_index  = 1.0f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material redPlastic() 
      {
        Material m;
        m.ambient   = RealColor( 0.1, 0.0, 0.0 );
        m.diffuse   = RealColor( 0.85, 0.05, 0.05 );
        m.specular  = RealColor( 1.0, 0.8, 0.8 );
        m.shinyness = 5.0;
        m.coef_diffusion  = 1.0f;
        m.coef_reflexion  = 0.05f;
        m.coef_refraction = 0.0f;
        m.in_refractive_index  = 1.0f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material bronze() 
      {
        Material m;
        m.ambient   = RealColor( 0.1125, 0.0675, 0.054 );
        m.diffuse   = RealColor( 0.714, 0.4284, 0.18144 );
        m.specular  = RealColor( 0.9, 0.8, 0.7 );
        // m.specular  = RealColor( 0.393548, 0.271906, 0.166721 );
        m.shinyness = 56; // 25.6;
        m.coef_diffusion  = 0.5f;
        m.coef_reflexion  = 0.75f;
        m.coef_refraction = 0.0f;
        m.in_refractive_index  = 1.0f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material emerald() 
      {
        Material m;
        m.ambient   = RealColor( 0.0f, 0.01f, 0.0f ); //RealColor( 0.0215, 0.1745, 0.0215 );
        m.diffuse   = RealColor( 0.09568, 0.77424, 0.10 );
        m.specular  = RealColor( 0.9, 1.0, 0.9 ); // RealColor( 0.633, 0.727811, 0.633 );
        m.shinyness = 0.6*128.0;
        m.coef_diffusion  = 0.15f;
        m.coef_reflexion  = 0.5f;
        m.coef_refraction = 0.65f;
        m.in_refractive_index  = 1.5f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material glass() 
      {
        Material m;
        m.ambient   = RealColor( 0.0, 0.0, 0.0 );
        m.diffuse   = RealColor( 0.95, 0.95, 1.0 );
        m.specular  = RealColor( 1.0, 1.0, 1.0 );
        m.shinyness = 80.0f;
        m.coef_diffusion  = 0.01f;
        m.coef_reflexion  = 0.05f;
        m.coef_refraction = 0.98f;
        m.in_refractive_index  = 1.5f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material blackMatter() 
      {
        Material m;
        m.ambient   = RealColor( 0.05, 0.05, 0.05 );
        m.diffuse   = RealColor( 0.1, 0.1, 0.1 );
        m.specular  = RealColor( 0.2, 0.2, 0.2 );
        m.shinyness = 0.5f;
        m.coef_diffusion  = 1.0f;
        m.coef_reflexion  = 0.0f;
        m.coef_refraction = 0.0f;
        m.in_refractive_index  = 1.0f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material sand() 
      {
        Material m;
        m.ambient   = RealColor( 0.1, 0.075, 0.05 );
        m.diffuse   = RealColor( 1.0, 0.9, 0.7 );
        m.specular  = RealColor( 1.0, 1.0, 0.98 );
        m.shinyness = 2.0;
        m.coef_diffusion  = 1.0f;
        m.coef_reflexion  = 0.0f;
        m.coef_refraction = 0.0f;
        m.in_refractive_index  = 1.0f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material mirror() 
      {
        Material m;
        m.ambient   = RealColor( 0.0, 0.0, 0.0 );
        m.diffuse   = RealColor( 0.05, 0.05, 0.05 );
        m.specular  = RealColor( 0.98, 0.99, 1.0 );
        m.shinyness = 80.0f;
        m.coef_diffusion  = 0.01f;
        m.coef_reflexion  = 0.99f;
        m.coef_refraction = 0.0f;
        m.in_refractive_index  = 1.0f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material blueWater() 
      {
        Material m;
        m.ambient   = RealColor( 0.1, 0.1, 0.2 );
        m.diffuse   = RealColor( 0.1, 0.6, 0.8 );
        m.specular  = RealColor( 1.0, 1.0, 1.0 );
        m.shinyness = 70.0f;
        m.coef_diffusion  = 0.01f;
        m.coef_reflexion  = 0.2f;
        m.coef_refraction = 0.8f;
        m.in_refractive_index  = 1.3f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material greyMatter() 
      {
        Material m;
        m.ambient   = RealColor( 0.0, 0.0, 0.0 );
        m.diffuse   = RealColor( 0.45, 0.45, 0.45 );
        m.specular  = RealColor( 0.5, 0.5, 0.5 );
        m.shinyness = 0.5f;
        m.coef_diffusion  = 1.0f;
        m.coef_reflexion  = 0.00f;
        m.coef_refraction = 0.0f;
        m.in_refractive_index  = 1.0f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material steel() 
      {
        Material m;
        m.ambient   = RealColor( 0.05, 0.05, 0.05 );
        m.diffuse   = RealColor( 0.3, 0.3, 0.31 );
        m.specular  = RealColor( 0.7, 0.7, 0.71 );
        m.shinyness = 10.0f;
        m.coef_diffusion  = 0.8f;
        m.coef_reflexion  = 0.2f;
        m.coef_refraction = 0.0f;
        m.in_refractive_index  = 1.0f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material ruby() 
      {
        Material m;
        m.ambient   = RealColor( 0.15f, 0.00f, 0.0f ); 
        m.diffuse   = RealColor( 0.7, 0.0, 0.05 );
        m.specular  = RealColor( 1.0, 0.9, 0.9 ); // RealColor( 0.633, 0.727811, 0.633 );
        m.shinyness = 0.6*128.0;
        m.coef_diffusion  = 0.25f;
        m.coef_reflexion  = 0.25f;
        m.coef_refraction = 0.5f;
        m.in_refractive_index  = 1.5f;
        m.out_refractive_index = 1.0f;
        return m;
      }
      static Material greyMettalic() 
      {
        Material m;
        m.ambient   = RealColor( 0.05, 0.05, 0.05 );
        m.diffuse   = RealColor( 0.75, 0.75, 0.8 );
        m.specular  = RealColor( 1.0, 1.0, 1.0 );
        m.shinyness = 30.0f;
        m.coef_diffusion  = 0.9f;
        m.coef_reflexion  = 0.1f;
        m.coef_refraction = 0.0f;
        m.in_refractive_index  = 1.0f;
        m.out_refractive_index = 1.0f;
        return m;
      }
    };

  } // namespace rt
} // namespace DGtal

#endif // !defined Material_ha

#undef Material_RECURSES
#endif // else defined(Material_RECURSES)

