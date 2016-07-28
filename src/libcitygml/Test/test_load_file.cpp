#include <iostream>
#include <fstream>
#include "citygml.hpp"

int main(int narg, char** argv)
{
  if ( narg<2 )
  {
    std::cout << "Usage: " 
              << argv[0]
              << " filename; filename being a CityGML file."
              << std::endl;
    exit( EXIT_FAILURE );
  }
  
  std::string filename( argv[1] );
  std::ifstream in( filename );
  if( ! in )
  {
    std::cout << "Unable to read file " << filename << std::endl;
    exit(EXIT_FAILURE);
  }

  citygml::ParserParams params;
  citygml::CityModel*  model = citygml::load( filename, params );
  std::cout << "citygml::load(): OK" << std::endl;

  model->computeEnvelope();
  std::cout << "computeEnvelope(): OK" << std::endl;

  return 0;
}

