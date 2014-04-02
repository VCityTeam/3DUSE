#include "moc/dialogTag.hpp"
#include "ui_dialogTag.h"
#include "gui/applicationGui.hpp"
#include "libcitygml/readerOsgCityGML.hpp"
#include <QSettings>
#include <QFileDialog>
#include <osg/ValueObject>
#include "moc/mainWindow.hpp"
////////////////////////////////////////////////////////////////////////////////
DialogTag::DialogTag(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogTag)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogTag::~DialogTag()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void loadRecTest(citygml::CityObject* node, osg::ref_ptr<osg::Group> parent, ReaderOsgCityGML& reader)
{
    osg::ref_ptr<osg::Group> grp = reader.createCityObject(node);
    parent->addChild(grp);
    citygml::CityObjects& cityObjects = node->getChildren();
    citygml::CityObjects::iterator it = cityObjects.begin();
    for( ; it != cityObjects.end(); ++it)
    {
        loadRecTest(*it, grp, reader);
    }
}
////////////////////////////////////////////////////////////////////////////////
void DialogTag::addTag(const vcity::URI& uri)
{
    appGui().getMainWindow()->m_osgView->setActive(false);

    citygml::CityObject* obj = nullptr;

    std::map<std::string, citygml::CityObject*> geoms;

    //if(m_ui->treeWidget->currentItem())
    {
        //std::cout << "select node : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << std::endl;
        obj = vcity::app().getScene().getCityObjectNode(uri);

        if(obj)
        {
            // add lod
            ui->comboBox->addItem(uri.getLastNode().c_str());
            geoms[uri.getLastNode()] = 0;

            // add all bldg


            // add flags
            std::vector<citygml::BuildingFlag*>::const_iterator it = obj->getFlags().begin();
            for(; it < obj->getFlags().end(); ++it)
            {
                ui->comboBox->addItem(QString(((*it)->getStringId()).c_str()));
                geoms[(*it)->getStringId()] = (*it)->getGeom();
            }


        }
        ui->comboBox->addItem("NULL");
        geoms["NULL"] = 0;
    }

    int res = exec();

    //std::cout << "diag res : " << res << std::endl;

    if(res && obj) // && m_ui->treeWidget->currentItem())
    {
        citygml::CityObject* geom = nullptr;
        if(ui->comboBox->currentText().size() > 4 && ui->comboBox->currentText().left(4) == "FLAG")
        {
            citygml::BuildingFlag* f = obj->getFlag(ui->comboBox->currentText().toStdString());
            if(f) geom = f->getGeom();
            std::cout << "use flag geom : " << ui->comboBox->currentText().toStdString() << " : " << f << " : " << geom << std::endl;
        }
        else if(ui->comboBox->currentText() != "NULL")
        {
            // use existing
            geom = vcity::app().getScene().getCityObjectNode(uri);
            std::cout << "use existing : " << geom << std::endl;
        }

        citygml::BuildingTag* tag = new citygml::BuildingTag(ui->dateTimeEdit->date().year(), geom);
        tag->m_date = ui->dateTimeEdit->dateTime();
        tag->m_name = ui->lineEdit->text().toStdString();
        tag->m_parent = obj;
        //tag->m_year = ui.dateTimeEdit->date().year();


        if(geom)
        {
            osg::ref_ptr<osg::Node> osgNode = appGui().getOsgScene()->getNode(uri);
            if(obj->getTags().size() == 0)
            {
                // mark osg tagged
                //osg::ref_ptr<osg::Node> osgNode = appGui().getOsgScene()->getNode(uri);
                if(osgNode->asGroup())
                {
                    osgNode->asGroup()->getChild(0)->setUserValue("TAGGED", 1);
                    //obj->getOsgNode()->getChild(0)->setUserValue("TAGGED", 1); //
                    //obj->getOsgNode()->setUserValue("TAGGED", 1);
                    std::cout << "osg parent tagged" << std::endl;
                }
            }

            /*vcity::URI uriTile = uri;
            while(uriTile.getDepth() > 2)
            {
                uriTile.pop();
            }
            uriTile.setType("Tile");
            vcity::Tile* tile = vcity::app().getScene().getTile(uriTile);*/

            size_t pos = geom->m_path.find_last_of("/\\");
            std::string path = geom->m_path.substr(0, pos);
            ReaderOsgCityGML readerOsgGml(path);
            readerOsgGml.m_settings.m_useTextures = vcity::app().getSettings().m_loadTextures;
            osg::ref_ptr<osg::Group> grp = readerOsgGml.createCityObject(geom);

            citygml::CityObjects& cityObjects = geom->getChildren();
            citygml::CityObjects::iterator it = cityObjects.begin();
            for( ; it != cityObjects.end(); ++it)
            {
                loadRecTest(*it, grp, readerOsgGml);
            }

            grp->setName(tag->getStringId()+tag->getGeom()->getId());
            grp->getChild(0)->setName(tag->getStringId()+tag->getGeom()->getId());
            grp->setUserValue("TAG", 1);

            std::cout << "insert osg geom" << std::endl;
            //obj->getOsgNode()->addChild(grp);
            osgNode->getParent(0)->addChild(grp);
            // obj->getOsgNode()->getParent(0)->addChild(grp);
            // geom->setOsgNode(grp);
            //m_osgScene->addChild(grp);
            //obj->getOsgNode()->setNodeMask(0);

            tag->setOsg(grp);
        }

        obj->addTag(tag);

        QTreeWidgetItem* item = new QTreeWidgetItem(QStringList(tag->getStringId().c_str()));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Checked);
        item->setText(1, "Tag");

        QTreeWidgetItem* item2 = new QTreeWidgetItem(QStringList(ui->comboBox->currentText()));
        item->addChild(item2);

        obj->checkTags();

        //m_ui->treeWidget->currentItem()->addChild(item);
        appGui().getTreeView()->addItemGeneric(uri, tag->getStringId().c_str(), "Tag");
    }

    appGui().getMainWindow()->m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
