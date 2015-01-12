#!/bin/bash

#(1843009.000000,5172743.000000,158.583984) (1847682.375000,5175449.000000,331.890411)
#(1843000.000000,5172500.000000,158.583984) (1848000.000000,5175500.000000,331.890411)
#1848000 - 1843000 = 5000 -> 10 tiles x
#5175500 - 5172500 = 3000 -> 6 tiles y

#cd /home/maxime/src/VCity-build/build-Qt4-gcc-debug/

for I in {0..9}
do
	for J in {0..5}
	do
		FILEOUT="Lyon03_""$I"_"$J"".gml"
		XMIN=$((1843000+$((500*$I))))
		YMIN=$((5172500+$((500*$J))))
		XMAX=$(($XMIN+500))
		YMAX=$(($YMIN+500))
		echo $FILEOUT "->" $XMIN $XMAX $YMIN $YMAX
		#./CityGMLCut
		#./CityGMLCut /home/frederic/Telechargements/Data/Lyon03/LYON_3.gml  /home/frederic/Telechargements/Data/Lyon03/cut/output"$I"_"$J"/$FILEOUT $XMIN $YMIN $XMAX $YMAX
		./CityGMLCut /home/frederic/Telechargements/Data/Lyon03/Lyon03_BATI/Lyon03_BATI.gml  /home/frederic/Telechargements/Data/Lyon03/Lyon03_BATI/cut/output"$I"_"$J"/$FILEOUT $XMIN $YMIN $XMAX $YMAX
		./CityGMLCut /home/frederic/Telechargements/Data/Lyon03/Lyon03_MNT/Lyon03_MNT.gml  /home/frederic/Telechargements/Data/Lyon03/Lyon03_MNT/cut/output"$I"_"$J"/$FILEOUT $XMIN $YMIN $XMAX $YMAX
	done
done

