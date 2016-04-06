#ifndef _ASCCUT_
#define _ASCCUT_

#include "src/gui/osg/osgMnt.hpp"

/**
 * @brief: Cuts the MNT into tiles of tileSizeX*tileSizeY and outputs the .asc files at given path
 */
void ASCCut(MNT* asc, int tileSizeX, int tileSizeY, std::string path, std::string filename);

#endif //_ASCCUT_
