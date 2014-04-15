
////////////////////////////////////////////////////////////////////////////////
#include "algo.hpp"
////////////////////////////////////////////////////////////////////////////////
#include "application.hpp"
#include <iostream>
#include <vector>
#include <set>
#include <utility>
#include <cmath>

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

        if(obj->getType() == citygml::COT_RoofSurface){ //Si surface de toit

            //std::cout << "Nouveau Toit trouvé" << std::endl;
            std::vector<citygml::Geometry*>& geoms = obj->getGeometries();
            std::vector<citygml::Geometry*>::iterator itGeom = geoms.begin();
            for(; itGeom != geoms.end(); ++itGeom){ //pour toute les géométries ( /!\ gestion des LoD/LoA encore non présente)

                std::vector<citygml::Polygon*>& polys = (*itGeom)->getPolygons();
                std::vector<citygml::Polygon*>::iterator itPoly = polys.begin();
                for(; itPoly != polys.end(); ++itPoly){ //Pour chaque polygone

                    Polygon2D poly;
                    citygml::LinearRing* ring = (*itPoly)->getExteriorRing();
                    const std::vector<TVec3d>& vertices = ring->getVertices();
                    std::vector<TVec3d>::const_iterator itVertices = vertices.begin();
                    for(; itVertices != vertices.end(); ++itVertices){//pour Chaque sommet

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
    double cosinus(Point pointA, Point pointB, Point pointC, Point pointD){
        double distAB,distCD,cosAB_CD;
        /*calcul du cosinus de l'angle entre les vecteurs AB et CD*/
        distAB=sqrt((pow(pointB.first-pointA.first,2)+pow(pointB.second-pointA.second,2)));
        distCD=sqrt((pow(pointD.first-pointC.first,2)+pow(pointD.second-pointC.second,2)));
        if(distAB==0 || distCD ==0){
            return 2; //si un des vecteurs est nul, retourne une valeur impossible
        }
        cosAB_CD=(pointB.first-pointA.first)*(pointD.first-pointC.first)+(pointB.second-pointA.second)*(pointD.second-pointC.second);
        cosAB_CD/=(distAB*distCD);

        return cosAB_CD;
    }

    bool segmentIntersect(Point pointA, Point pointB, Point pointC, Point pointD){ //idée : http://eli.thegreenplace.net/2008/08/15/intersection-of-1d-segments/
        double projA,projB,projC,projD;
        double tmp;
        Point pointH=pointA;
        pointH.first=pointB.first;
        //Angle entre AB et l'axe Ox
        if(fabs(cosinus(pointA,pointB,pointA,pointH))>0.71){ //Angle de 45degré ou moins, projection sur Ox
            projA=pointA.first; projB=pointB.first;
            projC=pointC.first;	projD=pointD.first;
        }
        else{  //Angle entre 45 et 90degré, projection sur Oy
            projA=pointA.second; projB=pointB.second;
            projC=pointC.second; projD=pointD.second;
        }
        //
        if(projB<projA){ //swap de A et B pour que A<B
            tmp=projA;projA=projB;projB=tmp;
        }
        if(projD<projC){ //swap de C et D pour que C<D
            tmp=projC;projC=projD;projD=tmp;
        }

    return (projB >= projC-0.07 && projD >= projA-0.07);

    }

    std::pair<bool,std::pair<int,int>> estAdjacent(const Polygon2D &poly1, const Polygon2D &poly2){
        double a,b; //y=ax+b
        double distDroites;
        Point pointA, pointB, pointC, pointD;
        std::pair<int,int> indices = std::make_pair(-1,-1);
        for(unsigned int i=0;i<poly1.size();++i){

            if(i<poly1.size()-1){
                pointA=poly1[i]; pointB=poly1[i+1];
            }
            else{
                pointA=poly1[i]; pointB=poly1[0];
            }
            for(unsigned int j=0; j< poly2.size(); ++j){
                if(j<poly2.size()-1){
                    pointC=poly2[j]; pointD=poly2[j+1];
                }
                else{
                    pointC=poly2[j]; pointD=poly2[0];
                }
                //Si les droites générées par les 2 segments sont colineaires
                if(fabs(cosinus(pointA,pointB,pointC,pointD))>0.99){
                    if(pointB.first==pointA.first){
                        distDroites=fabs(pointC.first-pointA.first);
                    }
                    else{
                        a= (pointB.second - pointA.second)/(pointB.first - pointA.first);
                        b= pointA.second - a * pointA.first;
                        distDroites=fabs(a*pointC.first-pointC.second+b)/sqrt(1+pow(a,2));
                    }

                    //Si les droites générées par les 2 segments sont suffisamment proches
                    if((distDroites)<0.1){
                        //et si les 2 segments on une zone en commun
                        if(segmentIntersect(pointA, pointB, pointC, pointD)){
                            indices = std::make_pair(i,j);
                            return std::make_pair(true,indices);
                        }
                    }
                }
            }
        }
        return std::make_pair(false,indices);
    }

    double distPtoP(const Point& pointA, const Point & pointB){
        return sqrt(pow(pointB.first-pointA.first,2)+pow(pointB.second-pointB.second,2));
    }
    /*
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
    */

    bool isInPoly(const Polygon2D & poly1, const Polygon2D &poly2){ // /!\ si un et un seul sommet de poly1 est dans le poly2, renvoie vrai
        bool result=false;
        Point pointO, pointA, pointB;
        pointO=poly1[0];
        for(unsigned int i=0,j=poly2.size()-1; i< poly2.size(); j=i++){
            pointA=poly2[j]; pointB=poly2[i];
            if( ((pointB.second>pointO.second) != (pointA.second>pointO.second)) &&
                ( pointO.first < (pointA.first-pointB.first) * (pointO.second-pointB.second) / (pointA.second-pointB.second) + pointB.first) ){
                result=!result;
            }
        }
        return result;
    }
    /**
     * @brief fusionne un set de polygone jusqu'à n'avoir plus qu'un unique polygone
     * @param polyset le set de polygone a fusionner
     */
    void fusionPoly(PolySet &polyset){
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
                    //std::cout << "i : " << i << "/ j : " << j << std::endl;
                    poly3.resize(poly1->size()+poly2->size());
                    /// fusion de deux polygones
                    ///on prend le premier polygon, on va jusqu'au point  numéro i,
                    ///on ajoute les points du polygones 2 (en commencant par j+1->nbPointPoly2 ^puis 0 -> j)
                    ///et on ajoute les points de poly1 i+1->nbPointPoly1
                    int h=0;
                    tmpPoly=*poly1;
                    for(unsigned int k=0;k<poly1->size();++k){
                        if(k!=i){
                            //std::cout << "Poly 1-" << k << " (" << tmpPoly[k].first << "," << tmpPoly[k].second << ")" << std::endl;
                            poly3[h++]=tmpPoly[k];
                        }
                        else{
                            //std::cout << "Poly 1-" << k << " (" << tmpPoly[k].first << "," << tmpPoly[k].second << ")" << std::endl;
                            poly3[h++]=tmpPoly[k];
                            tmpPoly=*poly2;
                            for(unsigned int l=j+1;l<poly2->size();++l){
                                  //std::cout << "Poly 2-" << l << " (" << tmpPoly[l].first << "," << tmpPoly[l].second << ")" << std::endl;
                                  poly3[h++]=tmpPoly[l];
                             }
                             for(unsigned int l=0;l<j+1;++l){
                                 //std::cout << "Poly 2-" << l << " (" << tmpPoly[l].first << "," << tmpPoly[l].second << ")" << std::endl;
                                 poly3[h++]=tmpPoly[l];

                             }
                             tmpPoly=*poly1;


                        }
                    }


                    for(unsigned int i=0,j=poly3.size()-1;i<poly3.size();){
                        if(distPtoP(poly3[j],poly3[i])<0.07){
                            poly3.erase(poly3.begin()+i);
                        }
                        else{
                            j=i++;
                        }

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
                    polyset.insert(polyset.begin(),poly3);
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
        //Suppression des poly a l'intérieur
        for(poly1=polyset.begin();poly1!=polyset.end();poly1++){ //
            poly2=poly1;poly2++;
            for(;poly2!=polyset.end();){
                if(isInPoly(*poly1,*poly2)){
                    //Poly 1 inside, remove poly1
                    polyset.erase(poly1);
                    poly1=polyset.begin();
                    poly2=poly1;poly2++;
                }
                else if(isInPoly(*poly2,*poly1)){
                    //Poly 2 inside, remove poly2;
                    polyset.erase(poly2);
                    poly2=poly1;poly2++;
                }
                else{
                    poly2++;
                }
            }
        }


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

        void lissagePoly(Polygon2D & poly){
        if(poly.size()>4){
            unsigned int size=poly.size();
            Point pointA,pointB,pointC;
            for(unsigned int k=0,j=size-1,i=size-2; k<size;){
                pointA=poly[i]; pointB=poly[j]; pointC=poly[k];

                if(distPtoP(pointA,pointB)<0.05 || distPtoP(pointB,pointC)<0.05 || distPtoP(pointA,pointC)<0.05){
                    poly.erase(poly.begin()+j);
                    size=poly.size();
                    k=0;j=size-1;i=size-2;
                }
                else{
                    if(fabs(cosinus(pointA,pointB,pointB,pointC))>0.999 || fabs(cosinus(pointA,pointB,pointA,pointC))>0.999 || fabs(cosinus(pointA,pointC,pointB,pointC))>0.999){
                        poly.erase(poly.begin()+j);
                        size=poly.size();
                        k=0;j=size-1;i=size-2;
                    }
                    else{
                        i=j;
                        j=k++;
                    }
                }
            }
        }
    }

        ////////////SIMPLFIFICATION////////////////////
        /// \brief simplification
        /// \param poly
        /// \return
        ///
        ///
        /*
         *Sn-2=AB Sn-1=BC Sn=CD Sn+1=DE Sn+2=EF
        *  ATTENTION : il faut que Sn > S min (à déterminer)
        Si Sn-1 et Sn+1 parallèles
             Si Sn-1 plus long que Sn+1
                     Suppression de Sn, Sn+1, raccourcissement de Sn-1 et prolongation de Sn+2
                     (attention, erreur dans le shéma, c'est l'inverse)
             Sinon
                     Suppression de Sn, Sn-1, raccourcissement de Sn+1 et prolongation de Sn-2
        Sinon
             Si intersection d'un point entre le prolongement de Sn-1 et Sn+1 éloignée
                     Supprision de Sn, connection de Sn-1 avec Sn+1 au milieu de Sn.
             Sinon
                     Suppression de Sn, polongement de Sn+1 et Sn jusqu'à intersection
     */
     Polygon2D simplification(const Polygon2D &poly){
               Polygon2D newPoly=poly;
               Point A, B, C, D, E, F, Z;
               double a, b, c, d;
               int i1, i2, i3, i4, i5, i6;
               double cosAB_BC, cosBC_CD, cosDE_EF, cosBC_DE;
               double distAB, distBC, distCD, distDE, distEF;
               int i=0;
               bool cond = false;

               i1=newPoly.size()-5;
               i2=newPoly.size()-4;
               i3=newPoly.size()-3;
               i4=newPoly.size()-2;
               i5=newPoly.size()-1;
               i6=0;
               while(i6<newPoly.size()){ // pour tout points

                   if(newPoly.size()>=6){ //on verifie qu'il reste 6 points
                       A = newPoly[i1];
                       B = newPoly[i2];
                       C = newPoly[i3];
                       D = newPoly[i4];
                       E = newPoly[i5];
                       F = newPoly[i6];
                       //Produit scalaire pour trouver angles droits entre AB et BC; BC et CD; DE et EF :
                       cosAB_BC=(B.first-A.first)*(C.first-B.first)+(B.second-A.second)*(C.second-B.second);
                       cosBC_CD=(C.first-B.first)*(D.first-C.first)+(C.second-B.second)*(D.second-C.second);
                       cosDE_EF=(E.first-D.first)*(F.first-E.first)+(E.second-D.second)*(F.second-E.second);
                       //Distances :
                       distAB=sqrt((pow(B.first-A.first,2)+pow(B.second-A.second,2)));
                       distBC=sqrt((pow(C.first-B.first,2)+pow(C.second-B.second,2)));
                       distCD=sqrt((pow(D.first-C.first,2)+pow(D.second-C.second,2)));
                       distDE=sqrt((pow(E.first-D.first,2)+pow(E.second-D.second,2)));
                       distEF=sqrt((pow(F.first-E.first,2)+pow(F.second-E.second,2)));
                       // Angles :
                       cosAB_BC/=(distAB*distBC);
                       cosBC_CD/=(distBC*distCD);
                       cosDE_EF/=(distDE*distEF);

                       // Si Sn > S min:
                       // Si 3 angles droit ou angles 45 degres
                       if((fabs(cosAB_BC)<0.2)&&(fabs(cosBC_CD)<0.2)&&(fabs(cosDE_EF)<0.2)){

                           if(distBC>distDE){
                               //on enlève CD, DE, rallonge EF
                               // on trouve l'intersection entre le prolongement de EF et BC, ce qui va nous donner un nouveau point à insérer
                               if((F.first - E.first)==0){
                                   a= (C.second - B.second)/(C.first - B.first);
                                   b= C.second - a * C.first;
                                   Z.first=E.first;
                                   Z.second=a*Z.first+b;
                               }
                               else{
                                    a= (F.second - E.second)/(F.first - E.first);
                                    b= E.second - a * E.first;
                                    // ax+b, equation de (EF)
                                    if((C.first - B.first)==0){
                                        Z.first=C.first;
                                        Z.second=a*Z.first+b;
                                    }
                                    else{
                                        c= (C.second - B.second)/(C.first - B.first);
                                        d= B.second - c * B.first;
                                        // cx+d, equation de (BC)
                                        // y = ax+b et y=cx+d donc ax+b=cx+d, donc  x=(d-b)/(a-c)
                                        Z.first=(d-b)/(a-c); //x
                                        Z.second=a*Z.first+b; //y
                                    }
                               }
                               newPoly[i3]=Z;
                               newPoly.erase(newPoly.begin()+i4);
                               newPoly.erase(newPoly.begin()+i5);
                               i1=newPoly.size()-5;
                               i2=newPoly.size()-4;
                               i3=newPoly.size()-3;
                               i4=newPoly.size()-2;
                               i5=newPoly.size()-1;
                               i6=0;

                           }
                           else{
                               //on enlève BC, CD, rallonge AB

                               // on trouve l'intersection entre le prolongement de AB et DE, ce qui va nous donner un nouveau point à insérer
                               if((A.first - B.first)==0){
                                   a= (D.second - E.second)/(D.first - E.first);
                                   b= D.second - a * D.first;
                                   Z.first=B.first;
                                   Z.second=a*Z.first+b;
                               }
                               else{
                                    a= (A.second - B.second)/(A.first - B.first);
                                    b= A.second - a * A.first;
                                    // ax+b, equation de (AB)
                                    if((D.first - E.first)==0){
                                        Z.first=D.first;
                                        Z.second=a*Z.first+b;
                                    }
                                    else{
                                        c= (D.second - E.second)/(D.first - E.first);
                                        d= D.second - c * D.first;
                                        // cx+d, equation de (DE)
                                        // y = ax+b et y=cx+d donc ax+b=cx+d, donc  x=(d-b)/(a-c)
                                        Z.first=(d-b)/(a-c); //x
                                        Z.second=a*Z.first+b; //y
                                    }

                               }
                               newPoly[i2]=Z;
                               newPoly.erase(newPoly.begin()+i3);
                               newPoly.erase(newPoly.begin()+i4);
                               i1=newPoly.size()-5;
                               i2=newPoly.size()-4;
                               i3=newPoly.size()-3;
                               i4=newPoly.size()-2;
                               i5=newPoly.size()-1;
                               i6=0;
                           }
                       }
                       // Angle 45 degres, pas encore fait
                       else{
                           i1=i2;
                           i2=i3;
                           i3=i4;
                           i4=i5;
                           i5=i6++;
                       }
                   }
                   i1=i2;
                   i2=i3;
                   i3=i4;
                   i4=i5;
                   i5=i6++;
               }
               return newPoly;
           }



        ///////////////////////////////////////////////////////

    void Algo::generateLOD0(const URI& uri)
    {
        std::cout << "void Algo::generateLOD0(const URI& uri)" << std::endl;
        Polygon2D tmp;
        log() << "generateLOD0 on "<< uri.getStringURI() << "\n";
        citygml::CityObject* obj = app().getScene().getCityObjectNode(uri);
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
            fusionPoly(roofPoints);
            citygml::CityObject* obj2 = new citygml::GroundSurface("tmpObj");
            citygml::Geometry* geom = new citygml::Geometry("idGeoTest",citygml::GT_Ground,0);

            for(PolySet::iterator it= roofPoints.begin(); it!= roofPoints.end(); ++it){
                tmp=(*it);
                lissagePoly(tmp);

                tmp=simplification(tmp); //fonction non exhaustive, il reste à fixer la distance S min pour déterminer si oui ou non on traite le dépassement
                lissagePoly(tmp); // on lisse une nouvelle fois les points suite à la simplification
                geom->addPolygon(convertPoly(tmp));
            }
            obj2->addGeometry(geom);
            obj->insertNode(obj2);

        }
    }

    void Algo::generateLOD1(const URI& uri)
    {
        log() << "generateLOD1 on "<< uri.getStringURI() << "\n";
        citygml::CityObject* obj = app().getScene().getCityObjectNode(uri);
        if(obj)
        {
            log() << uri.getStringURI() << "CityObject found\n";
        }
    }
////////////////////////////////////////////////////////////////////////////////
} // namespace vcity
////////////////////////////////////////////////////////////////////////////////


