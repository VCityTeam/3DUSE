// Copyright University of Lyon, 2012 - 2018
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include <stdio.h>
#include <stdlib.h>   // exit
#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <boost/algorithm/string/join.hpp>

#include "libcitygml/utils/tile.hpp"
#include "filters/EnhanceCityGML/LinkCityGMLShape.hpp"

#include "OGRGDALtoShpWriter.hpp"


void AssertFileHasCityGMLExtension( std::string filename )
{
  fs::path extension = fs::path(filename).extension();
  if( extension != ".citygml" && extension != ".gml" )
  {
    std::cout << "The extension of file "
              << filename
              << " is "
              << extension
              << " when it should be gml or citygml."
              << std::endl;
    exit (EXIT_FAILURE);
  }
}

// Historical note: the following main function is a CLI convertion of its
// GUI void MainWindow::slotSplitCityGMLBuildings() counterpart
int main(int argc, char** argv)
{
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Help screen")
    ("input-file",  po::value<std::string>()->required(),"Input GML filename")
    ("output-file", po::value<std::string>()->default_value("SplitBuildings.gml"),
                   "Output GML filename")
    ("output-dir",  po::value<std::string>(), "Output directory.");

  po::variables_map vm;
  try
  {
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);
  }
  catch (const po::error &ex)
  {
    std::cerr << ex.what() << std::endl;
    exit (EXIT_FAILURE);
  }

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    exit (EXIT_SUCCESS);
  }

  // Preparing the output directory (when provided)
  std::string fullOutputFilename = vm["output-file"].as<std::string>();
  if ( vm.count("output-dir") )
  {
    std::string outputFolderPathName = vm["output-dir"].as<std::string>();

    // Some basic checks on the output directory
    fs::path OutputDirectory( outputFolderPathName );

    if( fs::exists(OutputDirectory) && fs::is_regular_file( OutputDirectory ) )
    {
      std::cout << outputFolderPathName
                << " is an existing file and not a valid directory."
                << std::endl;
      exit (EXIT_FAILURE);
    }

    if( !is_directory( OutputDirectory ) )
    {
      std::cout << "Creating output directory "
                << outputFolderPathName
                << std::endl;
      fs::create_directories( OutputDirectory );
    }
    fullOutputFilename = outputFolderPathName + "/" + fullOutputFilename;
  }

  // Make sure we don't overwrite an existing output file
  fs::path OutputFile( fullOutputFilename );
  if( fs::exists(OutputFile) && fs::is_regular_file( OutputFile ) )
  {
    std::cout << OutputFile
              << " is an already existing file (exiting)."
              << std::endl;
    exit (EXIT_FAILURE);
  }

  std::string filename = vm["input-file"].as<std::string>();
  AssertFileHasCityGMLExtension( filename );
  vcity::Tile* BatiLOD2CityGML = new vcity::Tile( filename );
  std::cout << "CityGML file loaded :" << filename << std::endl;

  std::vector<TextureCityGML*> ListTextures;
  citygml::CityModel* ModelOut = SplitBuildingsFromCityGML( BatiLOD2CityGML,
                                                            &ListTextures );
  ModelOut->computeEnvelope();
  citygml::ExporterCityGML exporter( fullOutputFilename );

  exporter.exportCityModelWithListTextures( *ModelOut, &ListTextures );

  std::cout << "CityGML file " << fullOutputFilename << "created." << std::endl;

  // Cleanup roound
  delete BatiLOD2CityGML;
  delete ModelOut;
  for ( TextureCityGML* Tex : ListTextures )
  {
    delete Tex;
  }
}
