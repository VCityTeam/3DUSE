#!/bin/bash

# This scripts enables to cut a shapefile along a bounding box
# that evolves with the doubly nested loop

#tiles id : xmin - xmax / ymin - ymax
#1286 - 1315 / 13714 - 13734

# tiles pos : xmin - xmax / ymin - ymax
#643000 - 657500 / 6857000 - 6867000

# nb needed tiles :
#657500 - 643000 = 14500+500 -> 30 tiles x (500) / 15 (1000) / 8 (2000) / 4 (4000)
#6867000 - 6857000 = 10000+500 -> 21 tiles y (500) / 11 (1000) / 6 (2000) / 3 (4000)

cd /mnt/docs/data/dd_backup/GIS_Data_tmp/E_BATI
mkdir -pv tiles

LOD=1

for I in {0..29}
do
    for J in {0..20}
    do
        TILEX=$((1286+$I))
        TILEY=$((13714+$J))
        FILEOUT="tile_""$TILEX"_"$TILEY"".shp"
        XMIN=$((643000+$(($((500*$LOD))*$I))))
        YMIN=$((6857000+$(($((500*$LOD))*$J))))
        XMAX=$(($XMIN+$((500*$LOD))))
        YMAX=$(($YMIN+$((500*$LOD))))
        echo lod $LOD : $I $J : $FILEOUT "->" $XMIN $XMAX $YMIN $YMAX
        echo ogr2ogr -skipfailures -clipdst $XMIN $YMIN $XMAX $YMAX tiles/$FILEOUT BATI_INDIFFERENCIE.SHP
        ogr2ogr -skipfailures -clipdst $XMIN $YMIN $XMAX $YMAX tiles/$FILEOUT BATI_INDIFFERENCIE.SHP
    done
done
