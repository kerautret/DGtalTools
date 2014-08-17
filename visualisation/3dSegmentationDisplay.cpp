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
 * @file 3dSegmentationDisplay.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2014/08/17
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <QtGui/qapplication.h>

#include "DGtal/base/Common.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/io/readers/MeshReader.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/topology/SurfelAdjacency.h"

#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/readers/GenericReader.h"
#ifdef WITH_ITK
#include "DGtal/io/readers/DicomReader.h"
#endif

#include "DGtal/images/ImageSelector.h"



#include "specificClasses/Viewer3DImage.cpp"

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>


using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
namespace po = boost::program_options;

int main( int argc, char** argv )
{

  typedef DGtal::ImageContainerBySTLVector<DGtal::Z3i::Domain,  unsigned char > Image3D;
  typedef DGtal::ImageContainerBySTLVector<DGtal::Z2i::Domain,  unsigned char > Image2D;

  
  // parse command line ----------------------------------------------
  po::options_description general_opt("Allowed options are: ");
  general_opt.add_options()
    ("help,h", "display this message")
    ("input,i", po::value<std::string>(), "segmentation result input file: vol file (.vol)  representing the label of the segmentation (all label should be numbered from 1)" )
    ("sourceImage,s", po::value<std::string>(), "source image for which the segmentation is applied.") 
    ("grid", "draw slice images using grid mode. " ) 
    ("intergrid", "draw slice images using inter grid mode. " ) 
    ("emptyMode", "remove the default boundingbox display " ) 
    ("displayLabels,l", po::value<std::vector<unsigned int> >()->multitoken(), "specify the labels to be displayed (else by default it display all labels different from 0).")  
    ("scaleX,x",  po::value<float>()->default_value(1.0), "set the scale value in the X direction (default 1.0)" )
    ("scaleY,y",  po::value<float>()->default_value(1.0), "set the scale value in the Y direction (default 1.0)" )
    ("scaleZ,z",  po::value<float>()->default_value(1.0), "set the scale value in the Z direction (default 1.0)")
    
#ifdef WITH_ITK
    ("dicomMin", po::value<int>()->default_value(-1000), "set minimum density threshold on Hounsfield scale")
    ("dicomMax", po::value<int>()->default_value(3000), "set maximum density threshold on Hounsfield scale")
#endif    
    ("transparency,t",  po::value<uint>()->default_value(255), "transparency") ; 
  
  bool parseOK=true;
  po::variables_map vm;
  try{
    po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  }catch(const std::exception& ex){
    parseOK=false;
    trace.info()<< "Error checking program options: "<< ex.what()<< endl;
  }
  po::notify(vm);    
  if( !parseOK || vm.count("help") ||argc<=1 ||  !vm.count("input")||  !vm.count("sourceImage"))
    {
      std::cout << "Usage: " << argv[0] << " [input]\n"
		<< "Display segmentation result (image of labels) with inital source image."
		<< general_opt << "\n";
      return 0;
    }
  
  if(! vm.count("input"))
    {
      trace.error() << " The file name was not defined" << endl;      
      return 0;
    }
  if(! vm.count("sourceImage"))
    {
      trace.error() << " The source Image file name was not defined" << endl;      
      return 0;
    }

  string inputFilename = vm["input"].as<std::string>();
  string sourceFilename = vm["sourceImage"].as<std::string>();    
  unsigned char transp = vm["transparency"].as<uint>();
 
  QApplication application(argc,argv);

  float sx = vm["scaleX"].as<float>();
  float sy = vm["scaleY"].as<float>();
  float sz = vm["scaleZ"].as<float>();

  string extension = sourceFilename.substr(sourceFilename.find_last_of(".") + 1);
  if(extension!="vol" && extension != "p3d" && extension != "pgm3D" && extension != "pgm3d" && extension != "sdp" && extension != "pgm"
#ifdef WITH_ITK
     && extension !="dcm"
#endif
     ){
    trace.info() << "File extension not recognized: "<< extension << std::endl;
    return 0;
  }
  Viewer3DImage<>::ModeVisu mode;
  if(vm.count("emptyMode"))
    mode=Viewer3DImage<>::Empty;
  else if(vm.count("grid"))
    mode=Viewer3DImage<>::Grid;
  else if(vm.count("intergrid"))
    mode=Viewer3DImage<>::InterGrid;
  else
    mode=Viewer3DImage<>::BoundingBox;
   
  Viewer3DImage<> viewer(mode);
  viewer.setWindowTitle("simple Volume Viewer");
  viewer.show();
  viewer.setGLScale(sx, sy, sz);  
  
#ifdef WITH_ITK
  int dicomMin = vm["dicomMin"].as<int>();
  int dicomMax = vm["dicomMax"].as<int>();
  typedef DGtal::functors::Rescaling<int ,unsigned char > RescalFCT;
   
  Image3D sourceImage = extension == "dcm" ? DicomReader< Image3D,  RescalFCT  >::importDicom( sourceFilename, 
                                                                                             RescalFCT(dicomMin,
                                                                                                       dicomMax,
                                                                                                       0, 255) ) : 
    GenericReader<Image3D>::import( sourceFilename );
#else
  Image3D sourceImage = GenericReader<Image3D>::import( sourceFilename );
#endif
  Domain domain = sourceImage.domain();
  
  trace.info() << "source image loaded: "<< sourceImage  << std::endl;
  
  Image3D labelImage = GenericReader<Image3D>::import(inputFilename);
  trace.info() << "label image loaded: "<<labelImage<< std::endl;
  
  viewer.setVolImage(&sourceImage);
  
  viewer << Z3i::Point(512, 512, 0);


  

  unsigned int nbLabels=0;  
  std::vector<Z3i::DigitalSet> vectAllSets;
  std::set<unsigned int> labelSet;
  if(vm.count("displayLabels")){
    std::vector<unsigned int> vectLabels = vm["displayLabels"].as<std::vector<unsigned int> >();
    nbLabels=vectLabels.size();
    for(unsigned int i=0; i< nbLabels; i++){
      vectAllSets.push_back(Z3i::DigitalSet(domain));
      labelSet.insert(labelSet.begin(), vectLabels.at(i));
    }
  }else{
    for(Domain::ConstIterator it = domain.begin(), itend=domain.end(); it!=itend; ++it){
      unsigned char  val= labelImage( (*it) );           
      if(labelSet.find(val)==labelSet.end()){
        labelSet.insert(val);
        vectAllSets.push_back(Z3i::DigitalSet(domain));
      } 
    }    
    nbLabels = vectAllSets.size();
  }

  
  
  
  viewer << Viewer3D<>::updateDisplay;
  
  GradientColorMap<long> gradient(0, nbLabels);
  gradient.addColor(Color::Blue);
  gradient.addColor(Color::Green);
  gradient.addColor(Color::Yellow);
  gradient.addColor(Color::Red);

  for(Domain::ConstIterator it = domain.begin(), itend=domain.end(); it!=itend; ++it){
    unsigned char  val= labelImage( (*it) );
    if(labelSet.find(val)!=labelSet.end())
      vectAllSets.at(std::distance(labelSet.begin(),labelSet.find(val) )).insert(*it);
  }
  
  
  
  KSpace K;
  Point low = domain.lowerBound(); low[0]=low[0]-1; low[1]=low[1]-1; low[2]=low[2]-1;
  Point upp = domain.upperBound(); upp[0]=upp[0]+1; upp[1]=upp[1]+1; upp[2]=upp[2]+1;
  K.init(low, upp , true);
  SurfelAdjacency<3> SAdj( true );
  vector<vector<SCell> > vectConnectedSCell;
  unsigned int nbLabelsDiff0=0;
  unsigned int pos=0;
  for(unsigned int k=0; k < nbLabels; k++){
    trace.info() << "Extracting surface of label set num " << k ;    
    Surfaces<KSpace>::extractAllConnectedSCell(vectConnectedSCell,K, SAdj, vectAllSets.at(k), true);
    trace.info()<< " [done] " <<std::endl;


    for(unsigned int i= 0; i <vectConnectedSCell.size(); i++){
      for(unsigned int j= 0; j <vectConnectedSCell.at(i).size(); j++){
        viewer << DGtal::SetMode3D(vectConnectedSCell.at(0).at(0).className(), "Basic");
        DGtal::Color c= gradient(k);
        viewer << CustomColors3D(Color(250, 0,0, transp), Color(c.red(),
                                                                c.green(),
                                                                c.blue(), transp));	    
        
	viewer << vectConnectedSCell.at(i).at(j);
      }
    }
  }
  
  viewer << Viewer3D<>::updateDisplay;
  return application.exec();
}
