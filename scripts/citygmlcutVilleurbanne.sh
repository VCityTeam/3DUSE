#!/bin/bash

#(1844500.000000,5173500.000000,?) (1849000.000000,5178500.000000,?)
#1849000 - 1844500 = 4500 -> 10 tiles x
#5178500 - 5173500 = 5000 -> 11 tiles y

#cd /home/maxime/src/VCity-build/build-Qt4-gcc-debug/

for I in {0..10}
do
	for J in {0..11}
	do
		FILEOUT="Villeurbanne_""$I"_"$J"".gml"
		XMIN=$((1844500+$((500*$I))))
		YMIN=$((5173500+$((500*$J))))
		XMAX=$(($XMIN+500))
		YMAX=$(($YMIN+500))
		echo $FILEOUT "->" $XMIN $XMAX $YMIN $YMAX
		#./CityGMLCut
		./CityGMLCut /home/frederic/Telechargements/Data/VILLEURBANNE_BATIS_CITYGML/VILLEURBANNE_BATIS.gml  /home/frederic/Telechargements/Data/VILLEURBANNE_BATIS_CITYGML/cut/output"$I"_"$J"/$FILEOUT $XMIN $YMIN $XMAX $YMAX
               # ./CityGMLCut /home/frederic/Telechargements/Data/VILLEURBANNE_MNT_CITYGML/VILLEURBANNE_MNT.gml  /home/frederic/Telechargements/Data/VILLEURBANNE_MNT_CITYGML/cut/output"$I"_"$J"/$FILEOUT $XMIN $YMIN $XMAX $YMAX
	done
done

