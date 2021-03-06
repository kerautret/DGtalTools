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
 * @file
 * @ingroup volumetric
 * @author Bertrand Kerautret (\c bertrand.kerautret@univ-lyon2.fr )
 * @author Jonas Lamy (\c jonas.lamy@univ-lyon2.fr )
 *
 *
 * @date 2019/03/01
 *
 * Source file of the tool volMask
 *
 * This file is part of the DGtal library/DGtalTools Project.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/base/Common.h"
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/images/ImageContainerBySTLVector.h>

#include <DGtal/io/readers/GenericReader.h>
#include <DGtal/io/writers/GenericWriter.h>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#ifdef WITH_ITK
#include "DGtal/io/readers/ITKReader.h"
#include "DGtal/io/writers/ITKWriter.h"

#endif


///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;
///////////////////////////////////////////////////////////////////////////////
namespace po = boost::program_options;


/**
 @page volMask volMask
 
 @brief  Extracts a new image from the a mask image that represents the regions of the image which are selected and copied in the resulting image. Elements outside the regions defined by the mask are set to 0.
 
 @b Usage:   volMask [input]
 
 @b Allowed @b options @b are :
 
 @code
 -h [ --help ]               display this message
 -i [ --input ] arg          an input vol (or ITK: .nii, mha, ... ) file.
 -t [ --inputType ] arg      to sepcify the input image type (int or double).
 -a [ --mask ] arg           the mask image that represents the elements that
                             are copied as output in the resulting image (by
                             default set to 1 you can change this value by
                             using --maskValue).
 -o [ --output ] arg         the output masked image.
 -m [ --maskValue ] arg (=1) the masking value.
 @endcode
 
 @b Example:
 
 @code
 volMask -i ${DGtal}/examples/samples/lobster.vol  -a ${DGtal}/examples/samples/lobster.vol -o lobsMasked.vol -m 100
 @endcode
 
 @image html resvolMask.png "Example of result. "
 
 @see
 @ref volMask.cpp
 
 */




typedef ImageContainerBySTLVector < Z3i::Domain, unsigned char > Image3D;
#ifdef WITH_ITK
typedef ImageContainerBySTLVector < Z3i::Domain,  double > Image3D_D;
typedef ImageContainerBySTLVector < Z3i::Domain,  int > Image3D_I;
#endif



template<typename TImage, typename TImageMask>
typename TImageMask::Domain
subDomainMasked(const TImage &image, const  TImageMask &maskImage,
                typename TImageMask::Value maskValue){
  typename TImageMask::Domain res;
  Z3i::Point minP = image.domain().upperBound();
  Z3i::Point maxP = image.domain().lowerBound();
  
  Z3i::Point::Iterator minIt;
  Z3i::Point::Iterator maxIt;
  
  for(const auto &p: image.domain())
  {
    minIt = minP.begin();
    maxIt = maxP.begin();
    maxIt = maxP.begin();
    if( maskImage.domain().isInside(p) && maskImage(p) ==  maskValue ) // no noise on mask image
    {
      for(auto pIt=p.begin(); pIt!=p.end();pIt++ )
      {
        if( *pIt < *minIt ){*minIt = *pIt;}
        if( *pIt > *maxIt ){*maxIt = *pIt;}
        minIt++;
        maxIt++;
      }
    }
  }
  
  // offset to avoid problems on borders
  Z3i::Point offset(5,5,5);
  minP -= offset;
  maxP += offset;
  trace.info() << "sub-domain:" << minP << " " << maxP << std::endl;
  return typename TImageMask::Domain(minP, maxP);
}


template<typename TImage, typename TImageMask>
void
applyMask(const TImage &inputImage,TImage &outputImage,
          const TImageMask &maskImage, typename TImageMask::Value maskValue)
{
  for (const auto &p: outputImage.domain())
  {
    if (inputImage.domain().isInside(p) &&  maskImage(p) ==  maskValue)
    {
      outputImage.setValue(p, inputImage(p) );
    }
  }
  
}

template<typename TImage, typename TImageMask>
void
processImage(const TImage &inputImage, const TImageMask &maskImage,
             typename TImageMask::Value maskValue, std::string outputFileName ){
  // First step getting the bounding box of the domain:
  auto subDm = subDomainMasked(inputImage, maskImage, maskValue);
  TImage outputImage( subDm );
  // Second step: masking source image
  applyMask(inputImage, outputImage, maskImage,  maskValue);
  trace.info() << "writing output image...";
  GenericWriter<TImage>::exportFile(outputFileName, outputImage);
}


int main( int argc, char** argv )
{
  // parse command line -------------------------------------------------------
  po::options_description general_opt("Allowed options are");
  general_opt.add_options()
  ("help,h", "display this message")
#ifdef WITH_ITK
  ("input,i", po::value<std::string >(), "an input vol (or ITK: .nii, mha, ... ) file. " )
  ("inputType,t", po::value<std::string>()->default_value(""), "to sepcify the input image type (int or double)." )
#else 
  ("input,i", po::value<std::string >(), "an input vol file. " )
#endif
  ("mask,a", po::value<std::string >(), "the mask image that represents the elements that are copied as output in the resulting image (by default set to 1 you can change this value by using --maskValue).  " )
  ("output,o", po::value<std::string >()->default_value("result.vol"), "the output masked image." )
  ("maskValue,m", po::value<int>()->default_value(1), "the masking value." );
  
  
  bool parseOK=true;
  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, general_opt), vm);
  }
  catch(const std::exception& ex)
  {
    parseOK=false;
    trace.info()<< "Error checking program options: "<< ex.what()<< endl;
  }
  
  
  // check if min arguments are given and tools description ------------------
  po::notify(vm);
  if( !parseOK || vm.count("help")||argc<=1)
  {
    std::cout << "Usage: " << argv[0] << " [input]\n"
    << "Extracts a new image from the a mask image that represents the regions of the image which are selected and copied in the resulting image. Elements outside the regions defined by the mask are set to 0.\n"
    << general_opt << "\n"
    << "Typical use example:\n \t volMask -i ${DGtal}/examples/samples/lobster.vol  -a ${DGtal}/examples/samples/lobster.vol -o lobsMasked.vol -m 100  \n";
    return 0;
  }
  if(! vm.count("input"))
  {
    trace.error() << " The file name was not defined" << endl;
    return 1;
  }
  
  
  
  //  recover the  args ----------------------------------------------------
  string inputFileName = vm["input"].as<string>();
  string maskFileName = vm["mask"].as<string>();
  string outputFileName = vm["output"].as<string>();
#ifdef WITH_ITK
  string inputType = vm["inputType"].as<std::string>();
#endif
  
  
  int maskValue = vm["maskValue"].as<int>();
  trace.info() << "Reading mask image...";
  Image3D maskImage = DGtal::GenericReader<Image3D>::import(maskFileName);
  trace.info() << "[done]"<< std::endl;
  trace.info() << "Reading input image...";
#ifdef WITH_ITK
  if (inputType=="double")
  {
    Image3D_D inputImage = DGtal::GenericReader<Image3D_D>::import(inputFileName);
    trace.info() << "[done]"<< std::endl;
    processImage(inputImage, maskImage, maskValue, outputFileName);
  }
  else if (inputType=="int")
  {
    Image3D_I inputImage = DGtal::GenericReader<Image3D_I>::import(inputFileName);
    trace.info() << "[done]"<< std::endl;
    processImage(inputImage, maskImage, maskValue, outputFileName);
  }
  else
  {
    Image3D inputImage = DGtal::GenericReader<Image3D>::import(inputFileName);
    trace.info() << "[done]"<< std::endl;
    processImage(inputImage, maskImage, maskValue, outputFileName);
  }
#else
  Image3D inputImage = DGtal::GenericReader<Image3D>::import(inputFileName);
  processImage(inputImage, maskImage, maskValue, outputFileName);
#endif
  
  trace.info() << "[Done]" << std::endl;
  return 0;
}

