////////////////////////////////////////////////////////////////////////////////
#include "exportJSON.hpp"
#include <libxml/parser.h>
#include <sstream>
////////////////////////////////////////////////////////////////////////////////
namespace citygml
{
////////////////////////////////////////////////////////////////////////////////
ExporterJSON::ExporterJSON()
    : m_temporalExport(false), m_date()
{

}
////////////////////////////////////////////////////////////////////////////////
/*void ExporterJSON::genConfigFiles()
{
    uint ct = tilesPerSide();
        if(ct == NAN || ct == INFINITY || ct == 0){
            return;
        }
        //Informations for the general configuration folder
        double sizeTuileX;
        double sizeTuileY;
        uint rangeCameraTuile = 2;//ct/2-1;
        (rangeCameraTuile <=0)? rangeCameraTuile = 1 : true ;
        uint distanceTuileBox = 2;
        uint mainTuileX = rangeCameraTuile;
        uint mainTuileY = rangeCameraTuile;
        const TVec3d* sceneMin = &bbox->getLowerBound();
        const TVec3d* sceneMax = &bbox->getUpperBound();
        TVec3d* axisX = new TVec3d((sceneMax->x - sceneMin->x), 0, 0);
        TVec3d* axisY = new TVec3d(0, (sceneMax->y - sceneMin->y), 0);
        TVec3d* axisZ = new TVec3d(0, 0, (sceneMax->z - sceneMin->z));

        double xLength = axisX->length();
        double yLength = axisY->length();

        sizeTuileX = xLength / ct;
        sizeTuileY = yLength / ct;
        //normalize axis
        axisX->normalEq();
        axisY->normalEq();
        axisZ->normalEq();

        //creation of the scene configuration file
        QString fileName = whereTo;
        fileName += "/tiling.conf";
        QFile config(fileName);
        if (!config.open(QIODevice::WriteOnly | QIODevice::Text))
            return;
        QTextStream out(&config);
        out.setRealNumberNotation(QTextStream::FixedNotation);
        out << "sizeTuile " << sizeTuileX << "\n";
        out << "rangeCameraTuile " << rangeCameraTuile << "\n";
        out << "distanceTuileBox " << distanceTuileBox << "\n";
        out << "mainTuileX " << mainTuileX << "\n";
        out << "mainTuileY " << mainTuileY << "\n";
        out << "boundingBoxSceneMin " << sceneMin->x << " " << sceneMin->y << " " << sceneMin->z << "\n";
        out << "boundingBoxSceneMax " << sceneMax->x << " " << sceneMax->y << " " << sceneMax->z  << "\n";
        out << "axisX " << axisX->x << " " << axisX->y << " " << axisX->z << "\n";
        out << "axisY " << axisY->x << " " << axisY->y << " " << axisY->z << "\n";
        out << "axisZ " << axisZ->x << " " << axisZ->y << " " << axisZ->z << "\n";

        config.close();

        QString tileDir = QObject::tr("TUILES%1_%1/").arg(ct);
        QDir base(whereTo);
        base.mkdir(tileDir);

        uint xCoord;
        uint yCoord;
        //let's write !
        foreach(Tuile tuile, tiles){
            fileName = whereTo+"/"+tileDir;
            xCoord = (tuile.min.x - sceneMin->x + util::depsi) / sizeTuileX;
            yCoord = (tuile.min.y - sceneMin->y + util::depsi) / sizeTuileY;

            //create tile file and put bbox data into it
            fileName += QObject::tr("tile%1_%2.conf").arg(xCoord).arg(yCoord);
            QFile currentConf(fileName);
            if (!currentConf.open(QIODevice::WriteOnly | QIODevice::Text))
                return;
            QTextStream outFile(&currentConf);
            outFile.setRealNumberNotation(QTextStream::FixedNotation);
            outFile << "boundingBoxTuileMin " << tuile.min.x << " " << tuile.min.y << " " << sceneMin->z << "\n";
            outFile << "boundingBoxTuileMax " << tuile.max.x << " " << tuile.max.y << " " << sceneMax->z << "\n";

            //start getting and writing objects
            QString bati;
            int nbBati = 0;
            QString autre;
            int nbAutre = 0;
            QMap<IOLayer*, vector<uint> >::const_iterator itr;

            //we look for land to store
            if(tuile.ground != NULL && !tuile.ground->empty()){
                bati += QObject::tr(" land_nv%3_t%1_%2").arg(tuile.id.x).arg(tuile.id.y).arg(subIterations);
                nbBati++;
            }

            for(itr = tuile.batiments.constBegin(); itr != tuile.batiments.constEnd(); ++itr){
                //check if the current object is into a subdirectory
                IOLayer* jsexp = itr.key();
                QString dir = "";
                if(!jsexp->srcFile.isEmpty()){
                    dir= jsexp->srcFile.remove(0,jsexp->srcFile.lastIndexOf("/"));
                    dir.truncate(dir.lastIndexOf("."));
                    dir+= "/";
                }
                //put informations into temp strings
                foreach(uint ind, itr.value()){
                    IOFeature* obj = jsexp->featureList->at(ind);
    //                if( obj->getType()== COT_Building){
                        //bati += " " + dir + obj->getId();
                        bati += " " + obj->getId();
                        nbBati++;
    //                }
    //                else {
    //                    autre += " " + dir + obj->getId();
    //                    nbAutre ++;
    //                }
                }
            }
            if(type.contains("JSON")){
                outFile << "nbBuilding" << QObject::tr(" %1\n").arg(nbBati);
                outFile << "building" << bati << "\n";
            }
            else if(type.contains("CAO")) {
                outFile << "nbbati" << QObject::tr(" %1\n").arg(nbBati);
                outFile << "bati" << bati << "\n";
                outFile << "nbdivers" << QObject::tr(" %1\n").arg(nbAutre);
                outFile << "divers" << autre << "\n";
            }

            currentConf.close();
        }
}
////////////////////////////////////////////////////////////////////////////////
void exportNode()
{
    QString json;
    json += QString::fromUtf8("\"nbFaces\":") + QString::number(nbFaces) + QString::fromUtf8(",");
    json += QString::fromUtf8("\"min\":[") + util::trimmedBy(QString::number(min.x, 'f')) + QString::fromUtf8(",") + util::trimmedBy(QString::number(min.y, 'f')) + QString::fromUtf8(",") + util::trimmedBy(QString::number(min.z, 'f')) + QString::fromUtf8("],");
    json += QString::fromUtf8("\"max\":[") + util::trimmedBy(QString::number(max.x, 'f')) + QString::fromUtf8(",") + util::trimmedBy(QString::number(max.y, 'f')) + QString::fromUtf8(",") + util::trimmedBy(QString::number(max.z, 'f')) + QString::fromUtf8("],");

    json += QString::fromUtf8("\"listIndices\":[");
    for (unsigned int i = 0; i<listIndices->size(); i++)
    {
        json += QString::number(listIndices->at(i));
        if (i+1 < listIndices->size()) json += QString::fromUtf8(",");
    }
    json += QString::fromUtf8("],");

    json += QString::fromUtf8("\"listGeometrie\":[");
    for (unsigned int i = 0; i<listGeom->size(); i++)
    {
        json += util::trimmedBy(QString::number(listGeom->at(i).x, 'f'))   + QString::fromUtf8(",") + util::trimmedBy(QString::number(listGeom->at(i).y, 'f'))   + QString::fromUtf8(",") + util::trimmedBy(QString::number(listGeom->at(i).z, 'f'));
        if (i+1 < listGeom->size()) json += QString::fromUtf8(",");
    }
    json += QString::fromUtf8("],");

    json += QString::fromUtf8("\"listNormal\":[");
    for (unsigned int i = 0; i<listNormalByFaces->size(); i++)
    {
        json += util::trimmedBy(QString::number(listNormalByFaces->at(i).x, 'f'))   + QString::fromUtf8(",") + util::trimmedBy(QString::number(listNormalByFaces->at(i).y, 'f'))   + QString::fromUtf8(",") + util::trimmedBy(QString::number(listNormalByFaces->at(i).z, 'f'));
        if (i+1 < listNormalByFaces->size()) json += QString::fromUtf8(",");
    }
    json += QString::fromUtf8("]");

    return json;
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::exportCityModel(CityModel* model, const std::string& fileName)
{

}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::exportCityObject(CityObject* model, const std::string& fileName)
{
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::setTemporalExport(bool param)
{
    m_temporalExport = param;
}
////////////////////////////////////////////////////////////////////////////////
void ExporterJSON::setDate(const QDateTime& date)
{
    m_date = date;
}*/
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
} // namespace citygml
////////////////////////////////////////////////////////////////////////////////
