
////////////////////////////////////////////////////////////////////////////////
#include "algo.hpp"
////////////////////////////////////////////////////////////////////////////////
#include "application.hpp"
#include <iostream>
#include <vector>
#include <set>
#include <utility>

typedef std::pair<double,double> Point;
typedef std::vector<Point> Polygon2D;
typedef std::set<Polygon2D> PolySet;

/*
1. Projection sur le plan (xy)
2. Fusion des polygones
3. Simplfication du polygone
4. Alignement au sol et généralisation du bloc (Lod1)
*/

////////////////////////////////////////////////////////////////////////////////
namespace vcity
{
////////////////////////////////////////////////////////////////////////////////

    /**
     * @brief projete les toits du CityObject selectioné sur le plan (xy)
     * @param obj CityObject séléctioné
     * @param roofProj un set de Polygon, le résultat de la projection
     */
    void projectRoof(citygml::CityObject* obj, PolySet &roofProj){
        Polygon2D poly;
        if(obj->getType() == citygml::COT_RoofSurface){ //Si surface de toit

            //std::cout << "Nouveau Toit trouvé" << std::endl;
            std::vector<citygml::Geometry*>& geoms = obj->getGeometries();
            std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
            for(; itGeom != geoms.end(); ++itGeom){ //pour toute les géométries ( /!\ gestion des LoD/LoA encore non présente)

                std::vector<citygml::Polygon*>& polys = (*itGeom)->getPolygons();
                std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
                for(; itPoly != polys.end(); ++itPoly){ //Pour chaque polygone

                    poly.clear();
                    citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                    const std::vector<TVec3d>& vertices = (*itPoly)->getVertices();
                    std::vector<TVec3d>::const_iterator itVertices = vertices.begin();
                    for(; itVertices != vertices.end(); ++itVertices){//pour Chaque point

                        TVec3d point = *itVertices;
                        poly.push_back(std::make_pair(point.x,point.y)); //on récupere le point
                        //std::cout << " (x,y) = (" << point.x<< "," << point.y<< ")" << std::endl;
                    }
                    roofProj.insert(poly); // on récupere le polygone
                }
            }
        }
        citygml::CityObjects& cityObjects = obj->getChildren();
        citygml::CityObjects::iterator itObj = cityObjects.begin();
        for(; itObj != cityObjects.end(); ++itObj){
            projectRoof(*itObj,roofProj);
        }
    }

    //Les deux polygones sont adjacents si ils tout deux un coté appartenant a une meme droite
    /** pseudo-code
     *  Pour chaque segment du polygone 1
     *      Calcul de a1 et b1 (y = a1*x+b1)
     *      Pour chaque segment du polygone 2
     *          Calcul de a2 et b2 (y = a2*x+b2)
     *          si (a1 == a2 et b1 == b2)
     *              renvoie vrai (les 4 points appartiennent a une même droite)
     *          fin si
     *      fin pour
     *  fin pour
     *  renvoie faux
     */

    /**
     * @brief Test si les deux polygones sont adjacents
     * @param poly1 le premier polygone
     * @param poly2 le second polygone
     * @return poly1 adjacent à poly 2?
     */
    std::pair<bool,std::pair<int,int>> estAdjacent(const Polygon2D &poly1, const Polygon2D &poly2){
        double a1,b1,a2,b2; //y=ax+b
        std::pair<int,int> indices = std::make_pair(-1,-1);
        for(unsigned int i=0;i<poly1.size();++i){
            //a = (y2-y1)/(x2-x1)
            if(i<poly1.size()-1){
                a1= (poly1[i+1].second - poly1[i].second)/(poly1[i+1].first - poly1[i].first);
            }
            else{
                a1= (poly1[0].second - poly1[i].second)/(poly1[0].first - poly1[i].first);
            }
            //b = y1-a*x1
            b1= poly1[i].second - a1 * poly1[i].first;

            for(unsigned int j=0; j< poly2.size(); ++j){
                if(i<poly2.size()-1){
                    a2= (poly2[i+1].second - poly2[j].second)/(poly2[j+1].first - poly2[j].first);
                }
                else{
                    a1= (poly2[0].second - poly2[j].second)/(poly2[0].first - poly2[j].first);
                }
                b2= poly2[i].second - a1 * poly2[i].first;

                if(a1==a2 &&  b1==b2){
                    indices = std::make_pair(i,j);
                    return std::make_pair(true,indices);
                }
            }
        }
        return std::make_pair(false,indices);
    }

    double calculAire(const Polygon2D &polygone)
    {
        double aire=0;
        //((x1y2-y1x2)+(x2y3-y2x3)+...+(xny1-ynx1))/2
        for(unsigned int i=0;i<polygone.size();++i){
            if(i==polygone.size()-1){
                aire+= polygone[i].first * polygone[0].second - polygone[i].second * polygone[0].first;
            }
            else{
                 aire+= polygone[i].first * polygone[i+1].second - polygone[i].second * polygone[i+1].first;
            }
        }
        return aire/2;

    }

    /**
     * @brief fusionne un set de polygone jusqu'à n'avoir plus qu'un unique polygone
     * @param polyset le set de polygone a fusionner
     */
    Polygon2D fusionPoly(PolySet &polyset){
        PolySet::iterator poly1 , poly2;
        Polygon2D poly3,tmpPoly,polyRes;
        std::pair<bool,std::pair<int,int>> adjRes;
        bool change;
        int i,j;
        //On prend le premier polygone
        poly1=poly2=polyset.begin(); ++poly2;
        polyRes.clear();
        while(polyset.size()>1 && poly1 !=polyset.end()){ //Tant qu'il y a plusieurs polygone et qu'on peut fusionner
            change=false;
            while(poly2!=polyset.end()){  //on parcourt tout les polygones suivants
                adjRes = estAdjacent(*poly1,*poly2);
                if(adjRes.first){ //si les 2 polygones sont adjacents
                    i=adjRes.second.first; j=adjRes.second.second;
                    poly3.resize(poly1->size()+poly2->size());
                    /// fusion de deux polygones
                    ///on prend le premier polygon, on va jusqu'au point  numéro i,
                    ///on ajoute les points du polygones 2 (en commencant par j+1->nbPointPoly2 ^puis 0 -> j)
                    ///et on ajoute les points de poly1 i+1->nbPointPoly1
                    int h=0;
                    for(unsigned int k=0;k<poly1->size();++k){
                        if(k!=i){
                            tmpPoly=*poly1;
                            if(h!=0){
                                if(poly3[h-1]!=tmpPoly[k]){
                                    poly3[h++]=tmpPoly[k];
                                }
                                else{
                                    poly3.resize(poly3.size()-1);
                                }
                            }
                            else{
                                poly3[h++]=tmpPoly[k];
                            }
                        }
                        else{
                            tmpPoly=*poly1;
                            poly3[h++]=tmpPoly[k];
                            tmpPoly=*poly2;
                            for(unsigned int l=j+1;l<poly2->size();++l){
                                if(h!=0){
                                    if(poly3[h-1]!=tmpPoly[l]){
                                        poly3[h++]=tmpPoly[l];
                                    }
                                    else{
                                        poly3.resize(poly3.size()-1);
                                    }
                                }
                                else{
                                    poly3[h++]=tmpPoly[l];
                                }
                             }
                             for(unsigned int l=0;l<j+1;++l){
                                 if(h!=0){
                                     if(poly3[h-1]!=tmpPoly[l]){
                                         poly3[h++]=tmpPoly[l];
                                     }
                                     else{
                                         poly3.resize(poly3.size()-1);
                                     }
                                 }
                                 else{
                                     poly3[h++]=tmpPoly[l];
                                 }
                             }
                        }
                    }
                    if(poly3[0]==poly3[poly3.size()-1]){
                        poly3.resize(poly3.size()-1);
                    }
                    //on fusione les deux polygone, on ajoute le résultat dans le set et on enleve les 2 autres
//                    tmpPoly=*poly1;
//                    std::cout << "Poly 1 : " << std::endl;
//                    for(unsigned int i=0; i<tmpPoly.size();++i){
//                        std::cout << " (x,y) = (" << tmpPoly[i].first << "," << tmpPoly[i].second << ")" << std::endl;
//                    }
//                    tmpPoly=*poly2;
//                    std::cout << "Poly 2 : " << std::endl;
//                    for(unsigned int i=0; i<tmpPoly.size();++i){
//                        std::cout << " (x,y) = (" << tmpPoly[i].first << "," << tmpPoly[i].second << ")" << std::endl;
//                    }
//                    tmpPoly=poly3;
//                    std::cout << "Poly 3 : " << std::endl;
//                    for(unsigned int i=0; i<tmpPoly.size();++i){
//                        std::cout << " (x,y) = (" << tmpPoly[i].first << "," << tmpPoly[i].second << ")" << std::endl;
//                    }
                    polyset.erase(poly1);
                    polyset.erase(poly2);
                    polyset.insert(poly3);
                    change=true; //On annonce un changement
                    break; //et on sort de la boucle
                }
                ++poly2;
            }

            if(change){ //Si il y a un eu changement, on reprend les 2 premiers polygones
                poly1=poly2=polyset.begin();poly2++;
            }
            else{ //Sinon, on prend polygone suivant
                ++poly1;
                poly2=poly1; ++poly2;
            }
        }
        if(polyset.empty()){
            return polyRes;
        }
        poly1=polyset.begin();
        polyRes=*poly1;
        /*if(polyset.size() > 1){
        int aireMax=0,aireTmp;
            for(PolySet::iterator it = polyset.begin(); it!= polyset.end(); ++it){
                aireTmp = calculAire(*it);
                if(aireMax<aireTmp){
                    aireMax=aireTmp;
                    polyRes=*it;
                }
            }
        }*/
        return polyRes;
    }

    citygml::Polygon* convertPoly(const Polygon2D& poly){
        citygml::Polygon* result = new citygml::Polygon("PolyTest");
        citygml::LinearRing* ring = new citygml::LinearRing("RingTest",true);
        for(Polygon2D::const_iterator point=poly.begin(); point != poly.end(); ++point){
            TVec3d newPoint;
            newPoint.x=point->first;
            newPoint.y=point->second;
            newPoint.z=100;
            ring->addVertex(newPoint);
        }
        result->addRing(ring);
        return result;
    }

    void Algo::generateLOD0(const URI& uri)
    {
        std::cout << "void Algo::generateLOD0(const URI& uri)" << std::endl;

        log() << "generateLOD0 on "<< uri.getStringURI() << "\n";
        citygml::CityObject* obj = app().getScene().getNode(uri);

        if(obj)
        {
            log() << uri.getStringURI() << "CityObject found\n";
            PolySet roofPoints;
            projectRoof(obj,roofPoints);
//            for(PolySet::const_iterator poly=roofPoints.begin(); poly!= roofPoints.end(); ++poly){ //Affichage des points récupérés
//                for(Polygon2D::const_iterator point = poly->begin(); point!= poly->end(); ++point){
//                    std::cout << " (x,y) = (" << point->first << "," << point->second << ")" << std::endl;
//                }
//            }
            Polygon2D polyResult = fusionPoly(roofPoints);


            citygml::Polygon* polyLOD0 = convertPoly(polyResult);
            citygml::Geometry* geom = new citygml::Geometry("idGeoTest",citygml::GT_Ground,0);
            geom->addPolygon(polyLOD0);
            obj->addGeometry(geom);
           /* std::vector<citygml::Geometry*>& geoms = obj->getGeometries();
            std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();

            for(; itGeom != geoms.end(); ++itGeom){ //pour toute les géométries ( /!\ gestion des LoD/LoA encore non présente)
                (*itGeom)->addPolygon(polyLOD0);
            }*/
        }
    }

    void Algo::generateLOD1(const URI& uri)
    {
        log() << "generateLOD1 on "<< uri.getStringURI() << "\n";
        citygml::CityObject* obj = app().getScene().getNode(uri);
        if(obj)
        {
            log() << uri.getStringURI() << "CityObject found\n";
        }
    }
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////













