#include "ASCCut.hpp"

MNT* BuildTile(MNT* in, int size_x, int size_y, int tile_x, int tile_y)
{
   //check if tile is in range
   bool inRange = tile_x*size_x > in->get_x_noeud_NO()
      && (tile_x + 1)*size_x < (in->get_x_noeud_NO() + in->get_dim_x()*in->get_pas_x())
      && tile_y*size_y > in->get_y_noeud_NO()
      && (tile_y + 1)*size_y < (in->get_y_noeud_NO() + in->get_dim_y()*in->get_pas_y());
   if (!inRange) return nullptr;
   //calc coordinates of SW-point, nbcols, nbrows, etc.
   float cornerX = tile_x*size_x;
   float cornerY = tile_y*size_y;
   float stepX = in->get_pas_x();
   float stepY = in->get_pas_y();
   int nbC = (size_x / stepX) + 1;
   int nbR = (size_y / stepY) + 1;
   //calc x/y (in the input ASC context) corresponding to first (NW) point of the extracted tile
   int xStart = floor((cornerX - in->get_x_noeud_NO()) / stepX);
   int ddY = floor((cornerY - in->get_y_noeud_NO()) / stepY);
   int yStart = in->get_dim_y() - ddY - nbR;
   //create new MNT, fill it with header values and data
   MNT* out = new MNT(nbC, nbR, in->get_nodata());
   out->set_pas_x(stepX);
   out->set_pas_y(stepY);
   out->set_x_noeud_NO(cornerX);
   out->set_y_noeud_NO(cornerY);
   for (int y = 0; y < nbR; y++)
   {
      for (int x = 0; x < nbC; x++)
      {
         float alt = in->get_altitude(xStart + x, yStart + y);
         out->set_altitude(x, y, alt);
      }
   }
   return out;
}
