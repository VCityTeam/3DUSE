// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "TiledFilesLayout.hpp"

#include <QDir>
#include <QFile>

#include <iostream>
#include <osgDB/fstream>
#include <map>

#include "libcitygml/citygml.hpp"
#include "libcitygml/utils/tile.hpp"

TiledFiles::TiledFiles(std::string Folderpath)
{
    Folder = Folderpath;
}

void TiledFiles::BuildListofLayers()
{
    QDir QFolder(Folder.c_str());

    if (!QFolder.exists())
    {
        std::cout << "Error, Folder does not exists." << std::endl;
        return;
    }

    for (QFileInfo LayerFolder : QFolder.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::NoSort)) //Parameters needed to correct unexpected results (see http://stackoverflow.com/questions/7872155/qdirentryinfolist-unexpected-behavior)
    {
        if (!LayerFolder.isDir())
            continue;

        if (LayerFolder.filePath().endsWith("ShpExtruded") || LayerFolder.filePath().endsWith("SkylineOutput")) //For visibility plugin
            continue;

        if (LayerFolder.filePath().endsWith("tmp")) //For FloodAR plugin
            continue;

        QDir QDirLayerFolder(LayerFolder.filePath());

        TiledLayer L;
        L.Name = LayerFolder.baseName().toStdString();
        L.TuileMinX = -1;
        L.TuileMinY = -1;
        L.TuileMaxX = -1;
        L.TuileMaxY = -1;

        for (QFileInfo TileFolder : QDirLayerFolder.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::NoSort))
        {
            if (!TileFolder.isDir())
                continue;

            std::string Tile = TileFolder.baseName().toStdString();

            int SplitPos = Tile.find("_"); //Position of the split character "_" between X and Y coordinates of the current tile

            std::string X = Tile.substr(0, SplitPos);
            std::string Y = Tile.substr(SplitPos + 1);

            int Xval = std::stoi(X);
            int Yval = std::stoi(Y);

            if (L.TuileMinX == -1 || L.TuileMinX > Xval)
                L.TuileMinX = Xval;
            if (L.TuileMaxX == -1 || L.TuileMaxX < Xval)
                L.TuileMaxX = Xval;
            if (L.TuileMinY == -1 || L.TuileMinY > Yval)
                L.TuileMinY = Yval;
            if (L.TuileMaxY == -1 || L.TuileMaxY < Yval)
                L.TuileMaxY = Yval;
        }

        std::cout << "Layer : " << L.Name << " : " << L.TuileMinX << " " << L.TuileMinY << std::endl << L.TuileMaxX << " " << L.TuileMaxY << std::endl;

        ListofLayers.push_back(L);
    }
}
