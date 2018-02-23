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

#include "libcitygml/utils/tile.hpp"
#include "filters/ChangeDetection/ChangeDetection.hpp"
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
// GUI void MainWindow::slotChangeDetection() counterpart
int main(int argc, char** argv)
{
  std::vector<std::string> optionDateOrders{"first",
                                           "second",
                                           "third"};
  std::vector<const char*> optionDateNames{"first_date",
                                           "second_date",
                                           "third_date"};
  std::vector<const char*> optionFilenames{"first_file",
                                           "second_file",
                                           "third_file"};

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help,h", "Help screen")
    ("output_dir", po::value<std::string>()->default_value("extractOutput"),
                   "Output directory.");

  const int maxNumberOfDates = 2;
  for(int date=0; date <= maxNumberOfDates; date++)
  {
     desc.add_options()
    (optionDateNames[date], po::value<int>(),
                        (optionDateOrders[date] + " date value.").c_str())
    (optionFilenames[date], po::value<std::string>(),
                        (optionDateOrders[date] + " GML filename.").c_str());
  }
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

  // Retrieve the number of provided Dates:
  int numberOfDates = -1;
  for(int date=0; date <= maxNumberOfDates; date++)
  {
    if (    vm.count(optionDateNames[date])
         && vm.count(optionFilenames[date]) ) 
    { 
      numberOfDates++;
      continue;
    }
    if (     vm.count(optionDateNames[date])
         && !vm.count(optionFilenames[date]) )
    {
      std::cout << "A " << optionDateOrders[date] 
                << " date was provided but unfound associated filename."
                << std::endl;
      exit (EXIT_FAILURE);
    } else {
      break;
    }
  }

  // At least two dates are mandatory
  if ( numberOfDates < 1 )
  {
      std::cout << "At least two dates (and their associated filenames) "
                << "must be provided." 
                << std::endl;
      exit (EXIT_FAILURE);
  }

  // Defaulting some options
  std::string outputFolderPathName;
  if ( vm.count("output_dir") ) 
  {
    outputFolderPathName = vm["output_dir"].as<std::string>();
  } else {
    outputFolderPathName = "extractOutput";
    std::cout << "Output directory defaulted to :" 
                << outputFolderPathName
                << std::endl;
  }

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
    fs::create_directory( OutputDirectory ); 
  }

  // Shared_ptr used as a lazy memory leak avoidance mechanism
  std::vector< std::shared_ptr<vcity::Tile> > tiles(maxNumberOfDates);
  for(int date=0; date <= numberOfDates; date++)
  {
    std::string filename = vm[optionFilenames[date]].as<std::string>();
    AssertFileHasCityGMLExtension( filename );
    tiles[date] = std::shared_ptr<vcity::Tile>(new vcity::Tile( filename ));
    std::cout << "CityGML " 
              << optionDateOrders[date] 
              << " file loaded :" << filename << std::endl;
  }

  // When they are numberOfDates files they are numberOfDates-1 differences
  // to compute
  for(int date=0; date < numberOfDates; date++)
  {
    ChangeDetectionRes Res = CompareTiles( outputFolderPathName,
                                           tiles[date]->getCityModel(),
                                           tiles[date+1]->getCityModel() );

    SaveGeometrytoShape( outputFolderPathName + "/BatisOld.shp",
                         Res.EnveloppeCityU1 );
    SaveGeometrytoShape( outputFolderPathName + "/BatisNew.shp",
                         Res.EnveloppeCityU2 );
    SaveGeometrytoShape( outputFolderPathName + "/BatisCrees.shp",
                         Res.BatiCrees );
    SaveGeometrytoShape( outputFolderPathName + "/BatisDetruits.shp",
                         Res.BatiDetruits );
    SaveGeometrytoShape( outputFolderPathName + "/BatisModifiesOld.shp",
                         Res.BatiModifies1 );
    SaveGeometrytoShape( outputFolderPathName + "/BatisModifiesNew.shp",
                         Res.BatiModifies2 );
    SaveGeometrytoShape( outputFolderPathName + "/BatisInchanges.shp",
                         Res.BatiInchanges );

    delete Res.BatiInchanges;
    delete Res.BatiModifies2;
    delete Res.BatiModifies1;
    delete Res.BatiCrees;
    delete Res.BatiDetruits;

    delete Res.EnveloppeCityU1;
    delete Res.EnveloppeCityU2;
  }

}
