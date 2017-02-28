#ifndef _IMAGE2DREADER_H_
#define _IMAGE2DREADER_H_

#include <iostream>
#include <string>
#include <sstream>
#include "Image2D.h"
#include "Color.h"

namespace rt {

  template <typename TValue>
  class Image2DReader {
  public:
    typedef TValue Value;
    typedef Image2D<Value> Image;

    static bool read( Image & img, std::istream & input );
  };

  template <typename TValue>
  bool
  Image2DReader<TValue>::read( Image & img, std::istream & input )
  {
    std::cerr << "[Image2DReader<TValue>::read] not implemented." << std::endl;
    return false;
  }

  /// Specialization for gray-level images.
  template <>
  class Image2DReader<unsigned char> {
  public:
    typedef unsigned char Value;
    typedef Image2D<Value> Image;

    static bool read( Image & img, std::istream & input );
  };

  /// Specialization for color images.
  template <>
  class Image2DReader<Color> {
  public:
    typedef Color Value;
    typedef Image2D<Value> Image;

    static bool read( Image & img, std::istream & input );
  };



  bool
  Image2DReader<unsigned char>::read( Image & img, std::istream & input )
  {
    typedef unsigned char GrayLevel;
    std::string str;
    std::getline( input, str );
    if ( ! input.good() ) throw "[Image2DReader<unsigned char>::read] Unreadable input.";
    if ( str != "P5" &&  str != "P2") throw "[Image2DReader<unsigned char>::read] Requires P2 or P5 PGM format.";
    bool isASCIImode = ( str == "P2");
    do
      {
        std::getline( input, str );
        if ( ! input.good() ) {
          std::cerr << "[Image2DReader<unsigned char>::read] Unreadable input." << std::endl;
          return false;
        }
      }
    while ( str[ 0 ] == '#' || str=="");
    std::istringstream str_in( str );
    int w, h, max;
    str_in >> w >> h;
    std::getline( input, str );
    if ( ! input.good() ) {
      std::cerr << "[Image2DReader<unsigned char>::read] Unreadable width or height." << std::endl;
      return false;
    }
    str_in >> max;

    img = Image( w, h );
    int nb_read = 0;
    if ( isASCIImode )
      {
        input >> std::skipws;
        for ( Image::Iterator it = img.begin(), itE = img.end(); it != itE; ++it )
          {
            int c; 
            input >> c;
            if ( input.good() ) { ++nb_read; *it = (GrayLevel) c; }
          }
      }
    else // ! isASCIImode
      {
        input >> std::noskipws;
        for ( Image::Iterator it = img.begin(), itE = img.end(); it != itE; ++it )
          {
            unsigned char c; 
            input >> c;
            if ( input.good() ) { ++nb_read; *it = (GrayLevel) c; }
          }
        input >> std::skipws;
      }
    if ( input.fail() || input.bad() )
      {
        std::cerr << "[Image2DReader<unsigned char>::read] Incorrect file." << std::endl;
        std::cerr << "# nbread=" << nb_read << " / " << w*h << std::endl;
        return false;
      }
    return true;
  }


  bool
  Image2DReader<Color>::read( Image & img, std::istream & input )
  {
    std::string str;
    std::getline( input, str );
    if ( ! input.good() ) throw "[Image2DReader<Color>::read] Unreadable input.";
    if ( str != "P6" &&  str != "P3") throw "[Image2DReader<Color>::read] Requires P3 or P6 PPM format.";
    bool isASCIImode = ( str == "P3");
    do
      {
        std::getline( input, str );
        if ( ! input.good() ) {
          std::cerr << "[Image2DReader<Color>::read] Unreadable input." << std::endl;
          return false;
        }
      }
    while ( str[ 0 ] == '#' || str=="");
    std::istringstream str_in( str );
    int w, h, max;
    str_in >> w >> h;
    std::getline( input, str );
    if ( ! input.good() ) {
      std::cerr << "[Image2DReader<Color>::read] Unreadable width or height." << std::endl;
      return false;
    }
    str_in >> max;

    img = Image( w, h );
    int nb_read = 0;
    if ( isASCIImode )
      {
        input >> std::skipws;
        for ( Image::Iterator it = img.begin(), itE = img.end(); it != itE; ++it )
          {
            int r,g,b; 
            input >> r >> g >> b;
            float red   = ( (float) r ) / 255.0f;
            float green = ( (float) g ) / 255.0f;
            float blue  = ( (float) b ) / 255.0f;
            if ( input.good() ) { ++nb_read; *it = Color( red, green, blue ); }
          }
      }
    else // ! isASCIImode
      {
        input >> std::noskipws;
        for ( Image::Iterator it = img.begin(), itE = img.end(); it != itE; ++it )
          {
            unsigned char r, g, b;
            input >> r >> g >> b;
            float red   = ( (float) r ) / 255.0f;
            float green = ( (float) g ) / 255.0f;
            float blue  = ( (float) b ) / 255.0f;
            if ( input.good() ) { ++nb_read; *it = Color( red, green, blue ); }
          }
        input >> std::skipws;
      }
    if ( input.fail() || input.bad() )
      {
        std::cerr << "[Image2DReader<Color>::read] Incorrect file." << std::endl;
        std::cerr << "# nbread=" << nb_read << " / " << w*h << std::endl;
        return false;
      }
    return true;
  }

} // namespace rt

#endif // _IMAGE2DREADER_H_
