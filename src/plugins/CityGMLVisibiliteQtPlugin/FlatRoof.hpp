#ifndef __FLATROOF_HPP__
#define __FLATROOF_HPP__

#include <string>

/**
*	@brief Detecte les batiments a toits plats dans un fichier citygml, et
*        ecrit un fichier shape
*	@param path Chemin vers le fichier citygml
*	@param minArea Aire minimum que doit avoir un toit plat pour etre pris en
         compte
*	@param slopeFactor Factor permettant de definir a partir de quel pente on
         prend en compte le toit : 1 pas de pente, 0 pente verticale (valeur
         conseillee : 0.98) 
*/
void DetectionToitsPlats(std::string path, float minArea, float slopeFactor);

#endif
