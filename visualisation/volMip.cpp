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
 * @file volMip.cpp
 * @ingroup Visualisation
 * @author Bertrand Kerautret (\c bertrand.kerautret@univ-lyon2.fr )
 * LIRIS (CNRS, UMR 5205), University of Lyon 2, France
 *
 * @date 2020/05/04
 *
 *
 *
 * This file is part of the DGtalTools.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <algorithm>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/io/writers/GenericWriter.h"
#include "DGtal/io/readers/GenericReader.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/kernel/BasicPointFunctors.h"

#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/base/BasicFunctors.h"


#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
namespace po = boost::program_options;






/**
 @page volMip volMip
 @brief Generates a Maximum Intensity Image from a volumetric image file.
 
 The 3D volume is scanned in this normal direction N starting from P with a step 1.  The intensity value of each pixel is given by the maximal value on the ray.
 
 
 @b Usage: volMip [input] [output]
 
 @b Allowed @b options @b are:
 
 @code
 -h [ --help ]                    display this message
 -i [ --input ] arg               vol file (.vol, .longvol .p3d, .pgm3d and if
 WITH_ITK is selected: dicom, dcm, mha, mhd).
 For longvol, dicom, dcm, mha or mhd formats,
 the input values are linearly scaled between
 0 and 255.
 -o [ --output ] arg              sequence of discrete point file (.sdp)
 -m [ --thresholdMin ] arg (=128) min threshold (default 128)
 -M [ --thresholdMax ] arg (=255) max threshold (default 255)
 --nx arg (=0)                    set the x component of the projection
 direction.
 --ny arg (=0)                    set the y component of the projection
 direction.
 --nz arg (=1)                    set the z component of the projection
 direction.
 -x [ --centerX ] arg (=0)        choose x center of the projected image.
 -y [ --centerY ] arg (=0)        choose y center of the projected image.
 -z [ --centerZ ] arg (=1)        choose z center of the projected image.
 --width arg (=100)               set the width of the resulting height Field
 image.
 --height arg (=100)              set the height of the resulting height Field
 image.
 --maxScan arg (=255)  set the maximal scan deep.
 --setBackgroundLastDepth         change the default background (black with
 the last filled intensity).
 --rescaleInputMin arg (=0)       min value used to rescale the input
 intensity (to avoid basic cast into 8  bits
 image).
 --rescaleInputMax arg (=255)     max value used to rescale the input
 intensity (to avoid basic cast into 8 bits
 image).
 
 
 @endcode
 
 @b Example:
 @code
 $ volMip -i ${DGtal}/examples/samples/lobster.vol -m 60 -M 500  --nx 0 --ny 0.7 --nz -1 -x 150 -y 0 -z 150 --width 300 --height 300 --maxScan 350  -o resultingHeightMap.pgm
 @endcode
 
 You should obtain such a resulting image:
 @image html resVol2heightfield.png "resulting image."
 @see
 @ref volMip.cpp
 
 */


typedef ImageContainerBySTLVector < Z3i::Domain, unsigned char > Image3D;
typedef ImageContainerBySTLVector < Z2i::Domain, unsigned char> Image2D;
typedef ImageContainerBySTLVector < Z2i::Domain, double> Image2Dd;
typedef DGtal::ConstImageAdapter<Image3D, Z2i::Domain, DGtal::functors::Point2DEmbedderIn3D<DGtal::Z3i::Domain>,
Image3D::Value,  DGtal::functors::Identity >  ImageAdapterExtractor;



template<typename ImageT>
Image2D
mapProject(const ImageT &inputImage, unsigned int maxScan,
           Z3i::Point ptCenter, Z3i::RealPoint normalDir, unsigned int widthImageScan,
           const Z2i::Domain &aDomain2D, bool thresholdMin=false, bool thresholdMax=false,
           typename ImageT::Value minTh=0, typename ImageT::Value maxTh=0 ){
  Image2D resultingImage(aDomain2D);
  trace.info() << "Processing input with center" << ptCenter << "in direction: "<< normalDir
                << " max scan:" << maxScan << std::endl;
   for(Image2D::Domain::ConstIterator it = resultingImage.domain().begin();
       it != resultingImage.domain().end(); it++){
     resultingImage.setValue(*it, 0);
   }
   
  
  trace.info() << "Processing input with center" << ptCenter << "in direction: "<< normalDir
                << " max scan:" << maxScan << std::endl;
   for(Image2D::Domain::ConstIterator it = resultingImage.domain().begin();
       it != resultingImage.domain().end(); it++){
     resultingImage.setValue(*it, 0);
   }
   
  DGtal::functors::Identity idV;
   unsigned int maxDepthFound = 0;
    
   for(unsigned int k=0; k < maxScan; k++){
     Z3i::Point c (ptCenter+normalDir*k, DGtal::functors::Round<>());
     DGtal::functors::Point2DEmbedderIn3D<DGtal::Z3i::Domain >  embedder(inputImage.domain(),
                                                                         c,
                                                                         normalDir,
                                                                         widthImageScan);
     ImageAdapterExtractor extractedImage(inputImage, aDomain2D, embedder, idV);
     for(Image2D::Domain::ConstIterator it = extractedImage.domain().begin();
         it != extractedImage.domain().end(); it++){
       if((!thresholdMin ||  extractedImage(*it) > minTh) &&
          (!thresholdMax ||  extractedImage(*it) < maxTh))
       {
         if(extractedImage(*it)>=resultingImage(*it) )
         {
           resultingImage.setValue(*it, extractedImage(*it));
         }
       }
     }
   }
  return resultingImage;
}



ColorGradientPreset getPreset(std::string p) {
  static const std::map<std::string, ColorGradientPreset> cmap {
        { "cool", ColorGradientPreset::CMAP_COOL },
        { "hot", ColorGradientPreset::CMAP_HOT },
        { "jet", ColorGradientPreset::CMAP_JET },
        { "cooper", ColorGradientPreset::CMAP_COPPER },
        { "spring", ColorGradientPreset::CMAP_SPRING },
        { "winter", ColorGradientPreset::CMAP_WINTER }
     };
  auto it = cmap.find(p);
  if( it != cmap.end() ) {
    return (*it).second;
  }
  return ColorGradientPreset::CMAP_CUSTOM;
}





template<typename TImage>
void
exportImageWithGrad(const TImage &image, string gradType, string outputFilename )
{
  auto minMax = std::minmax_element(image.constRange().begin(), image.constRange().end());
  typename TImage::Value  valueMin = *(minMax.first);
   typename TImage::Value valueMax = *(minMax.second);
      DGtal::HueShadeColorMap<typename TImage::Value> gradMap (valueMin, valueMax);
    DGtal::GenericWriter<TImage, 2, DGtal::Color, DGtal::HueShadeColorMap<typename TImage::Value> >::exportFile(outputFilename, image, gradMap );
   

  
}




int main( int argc, char** argv )
{
  
  // parse command line ----------------------------------------------
  po::options_description general_opt("Allowed options are: ");
  general_opt.add_options()
  ("help,h", "display this message")
  ("input,i", po::value<std::string>(), "vol file (.vol, .longvol .p3d, .pgm3d and if WITH_ITK is selected: dicom, dcm, mha, mhd). For longvol, dicom, dcm, mha or mhd formats, the input values are linearly scaled between 0 and 255." )
  ("autoDisplay,a", po::value<double>()->default_value(1.0), "auto set the display settings (center point of the volume, and camera direction) and set zoom ajustement (by default 1.0 set > 1.0 to enlarge the view). " )
  ("output,o", po::value<std::string>(), "sequence of discrete point file (.sdp) ")
  ("inputType,t", po::value<std::string>()->default_value("int"), "to sepcify the input image type (int or double)." )
  ("thresholdMin,m", po::value<int>(), "min threshold" )
  ("thresholdMax,M", po::value<int>(), "max threshold" )
  ("colorMapRendering",po::value<string>(), "apply a color map rendering (spring, cooper, jet, hot, cool")
  ("nx", po::value<double>()->default_value(0), "set the x component of the projection direction." )
  ("ny", po::value<double>()->default_value(0), "set the y component of the projection direction." )
  ("nz", po::value<double>()->default_value(1), "set the z component of the projection direction." )
  ("centerX,x", po::value<unsigned int>()->default_value(0), "choose x center of the first projected image." )
  ("centerY,y", po::value<unsigned int>()->default_value(0), "choose y center of the first projected image." )
  ("centerZ,z", po::value<unsigned int>()->default_value(1), "choose z center of the first projected image." )
  ("width", po::value<unsigned int>()->default_value(100), "set the width of the resulting MIP rendering." )
  ("height", po::value<unsigned int>()->default_value(100), "set the height of the resulting MIP rendering." )
  ("maxScan", po::value<unsigned int>()->default_value(255), "set the maximal scan deep of a ray." )
  ("setBackgroundLastDepth", "change the default background (black with the last filled intensity).")
  ("rescaleInputMin", po::value<double>()->default_value(0), "min value used to rescale the input intensity (to avoid basic cast into 8  bits image).")
  ("rescaleInputMax", po::value<double>()->default_value(255), "max value used to rescale the input intensity (to avoid basic cast into 8 bits image).");
  
  
  
  bool parseOK=true;
  po::variables_map vm;
  try{
    po::store(po::parse_command_line(argc, argv, general_opt), vm);
  }catch(const std::exception& ex){
    parseOK=false;
    trace.info()<< "Error checking program options: "<< ex.what()<< endl;
  }
  po::notify(vm);
  if( !parseOK || vm.count("help")||argc<=1)
  {
    std::cout << "Usage: " << argv[0] << " [input] [output]\n"
    << "Convert volumetric  file into a projected 2D image given from a normal direction N and from a starting point P. The 3D volume is scanned in this normal direction N starting from P with a step 1. If the intensity of the 3d point is inside the given thresholds its 2D gray values are set to the current scan number."
    << general_opt << "\n";
    std::cout << "Example:\n"
    << "volMip -i ${DGtal}/examples/samples/lobster.vol -m 60 -M 500  --nx 0 --ny 0.7 --nz -1 -x 150 -y 0 -z 150 --width 300 --height 300 --maxScan 350  -o mipView.pgm \n";
    return 0;
  }
  
  if(! vm.count("input") ||! vm.count("output"))
  {
    trace.error() << " Input and output filename are needed to be defined" << endl;
    return 0;
  }
  string inputType = vm["inputType"].as<std::string>();
  int minTh = 0;
  int maxTh = 0;
  bool thresholdMin = vm.count("thresholdMin");
  bool thresholdMax = vm.count("thresholdMax");
  if (thresholdMin)
   {
      minTh = vm["thresholdMin"].as<int>();
   }
   if (thresholdMax)
   {
      maxTh = vm["thresholdMax"].as<int>();
   }
  
  string inputFilename = vm["input"].as<std::string>();
  string outputFilename = vm["output"].as<std::string>();
   

  
  double rescaleInputMin = vm["rescaleInputMin"].as<double>();
  double rescaleInputMax = vm["rescaleInputMax"].as<double>();
  
  trace.info() << "Reading input file " << inputFilename ;
  
  typedef DGtal::functors::Rescaling<double ,unsigned char > RescalFCTd;
  typedef DGtal::functors::Rescaling<int64_t ,unsigned char > RescalFCT;
  typedef DGtal::functors::Rescaling<unsigned char ,unsigned char > RescalFCTc;

  Image3D inputImage (Z3i::Domain(Z3i::Point(0,0,0), Z3i::Point(1,1,1))) ;
  if (inputType=="double"){
    inputImage =  GenericReader< Image3D >::importWithValueFunctor( inputFilename,
                                                                    RescalFCTd(rescaleInputMin,
                                                                                rescaleInputMax,
                                                                                 0, 255) );
  }else {
    inputImage =  GenericReader< Image3D >::importWithValueFunctor( inputFilename,
                                                                     RescalFCT(rescaleInputMin,
                                                                               rescaleInputMax,
                                                                               0, 255) );
  }

  trace.info() << " [done] " << std::endl ;
  std::ofstream outStream;
  outStream.open(outputFilename.c_str());
  trace.info() << "Processing image to output file " << outputFilename << std::endl;

  unsigned int widthImageScan = vm["height"].as<unsigned int>();
  unsigned int heightImageScan = vm["width"].as<unsigned int>();
  unsigned int maxScan = vm["maxScan"].as<unsigned int>();
  if(maxScan > std::numeric_limits<Image2D::Value>::max()){
    maxScan = std::numeric_limits<Image2D::Value>::max();
    trace.warning()<< "value --setBackgroundLastDepth outside mox value of image. Set to max value:" << maxScan << std::endl;
  }
  
  unsigned int centerX = vm["centerX"].as<unsigned int>();
  unsigned int centerY = vm["centerY"].as<unsigned int>();
  unsigned int centerZ = vm["centerZ"].as<unsigned int>();
  Z3i::Point ptCenter (centerX, centerY, centerZ);

  double nx = vm["nx"].as<double>();
  double ny = vm["ny"].as<double>();
  double nz = vm["nz"].as<double>();
  Z3i::RealPoint normalDir (nx, ny, nz);
  Image2D::Domain aDomain2D(DGtal::Z2i::Point(0,0),
                            DGtal::Z2i::Point(widthImageScan, heightImageScan));
  
  if (vm.count("autoDisplay"))
  {
    auto coefZoom = vm["autoDisplay"].as<double>();
    auto domC = (inputImage.domain().upperBound() + inputImage.domain().lowerBound())/2;
    normalDir = (domC - inputImage.domain().upperBound()).getNormalized();
    auto dx = inputImage.domain().upperBound()[0] - inputImage.domain().lowerBound()[0];
    auto dy = inputImage.domain().upperBound()[1] - inputImage.domain().lowerBound()[1];
    auto dz = inputImage.domain().upperBound()[2] - inputImage.domain().lowerBound()[2];
    auto maxDim = std::max(dx, std::max(dz, dy));
    widthImageScan = maxDim*coefZoom;
    maxScan = (inputImage.domain().upperBound() - inputImage.domain().lowerBound()).norm();
    aDomain2D = Image2D::Domain(DGtal::Z2i::Point(0, 0), DGtal::Z2i::Point(maxDim*coefZoom, maxDim*coefZoom));
    ptCenter = inputImage.domain().upperBound();
  }
  
  Image2D resultingImage = mapProject(inputImage,   maxScan, ptCenter, normalDir, widthImageScan,
                                      aDomain2D, thresholdMin, thresholdMax, minTh, maxTh);
             

  if (vm.count("colorMapRendering"))
  {
    exportImageWithGrad(resultingImage, vm["colorMapRendering"].as<std::string>(),  outputFilename );
  }
  else
  {
    auto minMax = std::minmax_element(resultingImage.range().begin(), resultingImage.range().end());
    GenericWriter< Image2D, 2, unsigned char,RescalFCTc >::exportFile( outputFilename,
                                                                      resultingImage,
                                                                      RescalFCTc(*(minMax.first),
                                                                                 *(minMax.second), 0, 255) );


  }
  trace.info() << " [done] " << std::endl ;
  return 0;
}

