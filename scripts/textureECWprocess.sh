#!/bin/bash

#tiles id : xmin - xmax / ymin - ymax
#1286 - 1315 / 13714 - 13734

# tiles pos : xmin - xmax / ymin - ymax
#643000 - 657500 / 6857000 - 6867000

# nb needed tiles :
#657500 - 643000 = 14500+500 -> 30 tiles x (500) / 15 (1000) / 8 (2000) / 4 (4000)
#6867000 - 6857000 = 10000+500 -> 21 tiles y (500) / 11 (1000) / 6 (2000) / 3 (4000)

#gdalwarp --config ECW_CACHE_MAXMEM 8000000 --config GDAL_CACHEMAX 8000 -wm 8000 -of JP2ECW 75-2008-0640-6860-LA93.ecw 75-2008-0640-6865-LA93.ecw /tmp/test.ecw
#python /usr/bin/gdal_merge.py -o /tmp/all.ecw -of HFA 75-2008-0640-6860-LA93.ecw 75-2008-0640-6865-LA93.ecw

# generate single image :
#python /usr/bin/gdal_merge.py -co COMPRESS=LZW -co PREDICTOR=2 -o tiff/all.tiff *.ecw -> fail with compression
#python /usr/bin/gdal_merge.py q-o tiff/all.tiff *.ecw

cd /mnt/docs/data/dd_backup/GIS_Data/IGN_Data/dpt_75/BDORTHO-75/BDORTHO/1_DONNEES_LIVRAISON_2010-10-00150/BDO_RVB_0M50_ECW_LAMB93_D75-ED08/tiff
mkdir -pv tiles

LOD=1

#for LOD in 1 2 4 8
#do
    #mkdir -pv lod$LOD
    #for (( I=0; I<32/$LOD; I++ ))
    for I in {0..29}
    do
        #for (( J=0; J<32/$LOD; J++ ))
        for J in {0..20}
        do
            TILEX=$((1286+$I))
            TILEY=$((13714+$J))
            FILEOUT="tile_""$TILEX"_"$TILEY"".tiff"
            XMIN=$((643000+$(($((500*$LOD))*$I))))
            YMIN=$((6857000+$(($((500*$LOD))*$J))))
            XMAX=$(($XMIN+$((500*$LOD))))
            YMAX=$(($YMIN+$((500*$LOD))))
            echo lod $LOD : $I $J : $FILEOUT "->" $XMIN $XMAX $YMIN $YMAX
            echo gdal_translate -co COMPRESS=LZW -co PREDICTOR=2 -projwin $XMIN $YMAX $XMAX $YMIN all.tiff tiles/$FILEOUT
            gdal_translate -co COMPRESS=LZW -co PREDICTOR=2 -projwin $XMIN $YMAX $XMAX $YMIN all.tiff tiles/$FILEOUT
        done
    done
#done
