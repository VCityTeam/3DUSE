#!/bin/bash

cd /home/maxime/src/VCity-build/build-Qt4-gcc-debug/

for I in {0..1}
do
	for J in {0..1}
	do
		FILEOUT="test_""$I"_"$J"".gml"
		XMIN=$((652000+$((250*$I))))
		YMIN=$((6863000+$((250*$J))))
		XMAX=$(($XMIN+250))
		YMAX=$(($YMIN+250))
		echo $FILEOUT "->" $XMIN $XMAX $YMIN $YMAX
		./parseCityGML
		./parseCityGML /home/maxime/docs/data/dd_backup/GIS_Data/Donnees_IGN/EXPORT_1304-13726/export-CityGML/ZoneAExporter.gml /tmp/output_"$I"_"$J"/$FILEOUT $XMIN $YMIN $XMAX $YMAX
	done
done

