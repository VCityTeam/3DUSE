// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#include "ConvertTextures.hpp"

std::vector<TVec2f> ConvertGeoreferencedTextures( std::vector<TVec2f> texUV )
{
   std::vector<TVec2f> texUV_Converted;

   float size_X;
   float size_Y;

   //Here, we should open the corresponding image file to read its dimension. However, it would take too much time for process 
   //involving hundreds of files. That's why we choose here to set the value corresponding to Lyon CityGML datasets (4096x4096)

   size_X = 4096;
   size_Y = 4096;

   //In the same idea, we choose to not read jgw files that give information about the coordinates (fr.wikipedia.org/wiki/World_file)
   //In jgw file, the E is negative, which means that uv.y are negatives so we have to flip the y coordinate because of the orientation
   //of the picture

   for ( TVec2f uv : texUV )
   {
      uv.x = uv.x / ( size_X - 1 );
      uv.y = 1 + uv.y / ( size_Y - 1 );
      texUV_Converted.push_back( uv );
   }

   return texUV_Converted;
}
