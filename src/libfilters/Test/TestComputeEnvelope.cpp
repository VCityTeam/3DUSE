#include <iostream>
#include <fstream>
#include <ogrsf_frmts.h>
#include "libcitygml/utils/tile.hpp"
#include "libcitygml/export/exportCityGML.hpp"
#include "EnhanceCityGML/LinkCityGMLShape.hpp"

int main(int narg, char** argv)
{
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

  std::string Shape_filename( argv[2] );
  std::ifstream inShape( Shape_filename );
  if( ! inShape )
  {
    std::cout << "Unfound Shape file named "
              << Shape_filename
              << std::endl;
    exit(EXIT_FAILURE);
  }

  vcity::Tile* BatiLOD2CityGML = new vcity::Tile( CityGML_filename.c_str() );

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

  std::vector< TextureCityGML* > ListTextures;
  citygml::CityModel* ModelOut =
     CutCityGMLwithShapefile( BatiLOD2CityGML, BatiShapeFile, &ListTextures);

  ModelOut->computeEnvelope();

  // Write the results to the CLI given outputfile:
  citygml::ExporterCityGML exporter( argv[3] );
  exporter.exportCityModelWithListTextures( *ModelOut, &ListTextures );

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
