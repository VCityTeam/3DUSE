## To test change detection

### Find docker file :
[3DUse-DockerContext](https://github.com/VCityTeam/UD-Reproducibility/tree/master/Articles/2020-IJGIS-Temporal/Compute3DTiles/Shared/Docker/3DUse-DockerContext")

### Build it : 
`$docker build -t liris:3DUse 3DUse-DockerContext` 
ND: -it for interactiv shell

### Find input file : 
They should be CityGML file. You can find some [here link missing]()

### Run it : 
    $docker run --mount src=`pwd`,target=/Input,type=bind --mount src=`pwd`,target=/Output,type=bind --workdir=/root/3DUSE/Build       /src/utils/cmdline -t liris:3DUse extractBuildingDates --first_date 2009 --first_file /Input/GML_data/Input/LYON_1ER_BATI_2009.gml --second_date 2012 --second_file /Input/GML_Data/Input/LYON_1ER_BATI_2012.gml --output_dir /Output/ 
 
You will need to change first and second date argument and the path for your file

You will have as an output "DifferencesAsGraph.json". It represents a graph where nodes are buildings (ex: Church in 2009) and links are transformation (ex: modified). All the change are referenced in that file.
