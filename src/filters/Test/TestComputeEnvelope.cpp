#include <iostream>
#include <fstream>
#ifdef _MSC_VER                  // Inhibit dll-interface warnings concerning
  # pragma warning(disable:4251) // gdal-1.11.4 internals (cpl_string.h) when
#endif                           // including ogrsf_frmts.h on VC++
#include <ogrsf_frmts.h>

#include "libcitygml/utils/tile.hpp"
#include "libcitygml/export/exportCityGML.hpp"
#include "filters/EnhanceCityGML/LinkCityGMLShape.hpp"

int main(int narg, char** argv)
{
  std::cout << "  Entering test " << argv[0] << std::endl;

  if ( narg < 4 )
  {
    std::cout << "Usage: " 
              << argv[0]
              << " CityGML_filename Shape_filename output_filename"
              << std::endl
              << "  - CityGML_filename is the filename of the CityGML file"
              << " holding LOD2 building data," 
              << std::endl
              << "  - Shape_filename is the filename of the Shapefile and,"
              << std::endl
              << "  - output_filename is well... the output file name."
              << std::endl;
    exit( EXIT_FAILURE );
  }
  
  std::string CityGML_filename( argv[1] );
  std::ifstream inGML( CityGML_filename );
  if( ! inGML )
  {
    std::cout << "Unfound CityGML file named "
              << CityGML_filename
              << std::endl;
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "  CityGML file used as first input: "
              << CityGML_filename
              << std::endl;
  }

  std::string Shape_filename( argv[2] );
  std::ifstream inShape( Shape_filename );
  if( ! inShape )
  {
    std::cout << "Unfound Shape file named "
              << Shape_filename
              << std::endl;
    exit(EXIT_FAILURE);
  }
  else
  {
    std::cout << "  Shape file used as second input: "
              << Shape_filename
              << std::endl;
  }

  std::cout << "  Launching the creation of the Tile." << std::endl;
  vcity::Tile* BatiLOD2CityGML = new vcity::Tile( CityGML_filename.c_str() );
  std::cout << "  Tile created." << std::endl;

  // FIXME: remove the following line and make the test _really_ effective !
  return 0;

  OGRDataSource* BatiShapeFile = 
     OGRSFDriverRegistrar::Open( Shape_filename.c_str(), FALSE);
  if( !BatiShapeFile )
  {
    std::cout << "OGR could not read shape file named "
              << Shape_filename
              << std::endl;
    exit(EXIT_FAILURE);
  }
  std::cout << "  Shape file succesfully opened." << std::endl;

  std::cout << "  Initiating cut of shapefile." << std::endl;
  std::vector< TextureCityGML* > ListTextures;
  citygml::CityModel* ModelOut =
     CutCityGMLwithShapefile( BatiLOD2CityGML, BatiShapeFile, &ListTextures);
  std::cout << "  Shapefile succesfully cut." << std::endl;

  std::cout << "  Launching computation of envelope." << std::endl;
  ModelOut->computeEnvelope();
  std::cout << "  Envelope computed." << std::endl;

  // Write the results to the CLI given outputfile:
  citygml::ExporterCityGML exporter( argv[3] );
  std::cout << "  Exporter to file " << argv[3] << " created" << std::endl;
  exporter.exportCityModelWithListTextures( *ModelOut, &ListTextures );
  std::cout << "  Exportation done." << std::endl;

  // Clean up round:
  for (TextureCityGML* Tex : ListTextures)
  {
     delete Tex;
  }
  delete BatiShapeFile;
  delete BatiLOD2CityGML;
  // Note: for some obscure yet well know reasons, one cannot delete both
  // ModelOut and BatiLOD2CityGML...
  
  return 0;
}
