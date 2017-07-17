// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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

