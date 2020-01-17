#/bin/bash

# Check that parameters are correctly provided
if [ $# != 1 ]
  then
          echo "Awaited parameters: input directory to the thematized."
    exit 1
fi

source_dir=$1
echo "Thematizing the following CityGML directory ", $source_dir
target_dir="$1-Thematized"
echo "to target directory ", $target_dir

mkdir -p $target_dir
mkdir -p $target_dir/BATI
find $source_dir -name \*_BATI_\*.gml -exec cp {} $target_dir/BATI/ \;
mkdir -p $target_dir/PONT
find $source_dir -name \*_PONT_\*.gml -exec cp {} $target_dir/PONT/ \;
mkdir -p $target_dir/EAU
find $source_dir -name \*_EAU_\*.gml -exec cp {} $target_dir/EAU/ \;
mkdir -p $target_dir/TIN
find $source_dir -name \*_TIN_\*.gml -exec cp {} $target_dir/TIN/ \;
mkdir -p $target_dir/PYLONE
find $source_dir -name \*_PYLONE_\*.gml -exec cp {} $target_dir/PYLONE/ \;
