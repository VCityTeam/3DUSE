#include <string>

/**
*	@brief Detecte les batiments à toits plats dans un fichier citygml, et écris un fichier shape
*	@param path Chemin vers le fichier citygml
*	@param minArea Aire minimum que doit avoir un toit plat pour être pris en compte
*	@param slopeFactor Factor permettant de définir à partir de quel pente on prend en compte le toit : 1 pas de pente, 0 pente verticale (valeur conseillé : 0.98) 
*/
void DetectionToitsPlats(std::string path, float minArea, float slopeFactor);