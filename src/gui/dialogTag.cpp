// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogTag.hpp"
#include "ui_dialogTag.h"
#include "gui/applicationGui.hpp"
#include "osg/osgCityGML.hpp"
#include <QSettings>
#include <QFileDialog>
#include <osg/ValueObject>
#include "moc/mainWindow.hpp"
////////////////////////////////////////////////////////////////////////////////
/** Provide an simple example of customizing the default UserDataContainer.*/
class MyUserDataContainer : public osg::DefaultUserDataContainer
{
    public:
        MyUserDataContainer() {}
        MyUserDataContainer(const MyUserDataContainer& udc, const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY):
            DefaultUserDataContainer(udc, copyop) {}

        META_Object(MyNamespace, MyUserDataContainer)

        virtual Object* getUserObject(unsigned int i)
        {
            OSG_NOTICE<<"MyUserDataContainer::getUserObject("<<i<<")"<<std::endl;
            return osg::DefaultUserDataContainer::getUserObject(i);
        }

        virtual const Object* getUserObject(unsigned int i) const
        {
            OSG_NOTICE<<"MyUserDataContainer::getUserObject("<<i<<") const"<<std::endl;
            return osg::DefaultUserDataContainer::getUserObject(i);
        }


    protected:
        virtual ~MyUserDataContainer() {}
};
////////////////////////////////////////////////////////////////////////////////
REGISTER_OBJECT_WRAPPER( MyUserDataContainer,
                         new MyUserDataContainer,
                         MyUserDataContainer,
                         "osg::Object osg::UserDataContainer osg::DefaultUserDataContainer MyUserDataContainer" )
{
}
////////////////////////////////////////////////////////////////////////////////
class MyGetValueVisitor : public osg::ValueObject::GetValueVisitor
{
    public:
        virtual void apply(void* value) { OSG_NOTICE<<" void* "<<value; }
};
////////////////////////////////////////////////////////////////////////////////
template<typename T>
class GetNumeric : public osg::ValueObject::GetValueVisitor
{
    public:
        GetNumeric():
            _set(false),
            _value(0) {}

        virtual void apply(void* value) { _value = value; _set = true; }

        bool _set;
        T _value;
};
////////////////////////////////////////////////////////////////////////////////
template<typename T>
T getNumeric(osg::Object* object)
{
    osg::ValueObject* bvo = dynamic_cast<osg::ValueObject*>(object);
    if (bvo)
    {
        GetNumeric<T> gn;
        if (bvo->get(gn) && gn._set) return gn._value;
    }
    return T(0);
}
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
/*void loadRecTest(citygml::CityObject* node, osg::ref_ptr<osg::Group> parent, ReaderOsgCityGML& reader)
{
    osg::ref_ptr<osg::Group> grp = reader.createCityObject(node);
    parent->addChild(grp);
    citygml::CityObjects& cityObjects = node->getChildren();
    citygml::CityObjects::iterator it = cityObjects.begin();
    for( ; it != cityObjects.end(); ++it)
    {
        loadRecTest(*it, grp, reader);
    }
}*/
////////////////////////////////////////////////////////////////////////////////
void DialogTag::addTag(const vcity::URI& uri)
{
    appGui().getMainWindow()->m_osgView->setActive(false);

    citygml::CityObject* obj = nullptr;

    std::map<std::string, citygml::CityObject*> geoms;

    //if(m_ui->treeWidget->currentItem())
    {
        //std::cout << "select node : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << std::endl;
        uri.resetCursor();
        obj = vcity::app().getScene().getCityObjectNode(uri);

        if(obj)
        {
            // add states
            std::vector<citygml::CityObjectState*>::const_iterator it = obj->getStates().begin();
            for(; it < obj->getStates().end(); ++it)
            {
                ui->comboBox->addItem(QString(((*it)->getStringId()).c_str()));
                geoms[(*it)->getStringId()] = (*it)->getGeom();
            }

            ui->comboBox->addItem("NULL");

            // add lod
            ui->comboBox->addItem(uri.getLastNode().c_str());
            geoms[uri.getLastNode()] = nullptr;

            // add all bldg
        }
        geoms["NULL"] = nullptr;

        // auto adjust combobox
        ui->comboBox->setCurrentIndex(obj->getTags().size());
        QDate date(2000, 1, 1);
        date = date.addYears(obj->getTags().size()*10);
        ui->dateTimeEdit->setDate(date);
    }

    int res = exec();

    //std::cout << "diag res : " << res << std::endl;

    if(res && obj) // && m_ui->treeWidget->currentItem())
    {
        citygml::CityObject* geom = nullptr;
        citygml::CityObjectState* state = nullptr;
        if((ui->comboBox->currentText().size() > 5 && ui->comboBox->currentText().left(5) == "STATE") ||
           (ui->comboBox->currentText().size() > 8 && ui->comboBox->currentText().left(8) == "DYNSTATE"))
        {
            // get geom from state
            state = obj->getState(ui->comboBox->currentText().toStdString());
            if(state) geom = state->getGeom();
            std::cout << "use state geom : " << ui->comboBox->currentText().toStdString() << " : " << state << " : " << geom << std::endl;
        }
        else if(ui->comboBox->currentText() != "NULL")
        {
            // use existing
            uri.resetCursor();
            geom = vcity::app().getScene().getCityObjectNode(uri);
            std::cout << "use existing : " << geom << std::endl;
        }

        citygml::CityObjectTag* tag = new citygml::CityObjectTag(ui->dateTimeEdit->date().year(), geom);
        if(state) tag->m_state = state; // set state
        tag->m_date = ui->dateTimeEdit->dateTime();
        tag->m_name = ui->lineEdit->text().toStdString();
        tag->m_parent = obj;
        //tag->m_year = ui.dateTimeEdit->date().year();


        /*if(geom)
        {
            // get parent osg geom
            uri.resetCursor();
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

            // build osg geom for tag

            //vcity::URI uriTile = uri;
            //while(uriTile.getDepth() > 2)
            //{
            //    uriTile.pop();
            //}
            //uriTile.setType("Tile");
            //vcity::Tile* tile = vcity::app().getScene().getTile(uriTile);

            size_t pos = geom->m_path.find_last_of("/\\");
            std::string path = geom->m_path.substr(0, pos);
            //path = "/mnt/docs/data/dd_backup/Donnees_IGN_unzip/EXPORT_1296-13731/export-CityGML/";
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
            //grp->setUserDataContainer(new MyUserDataContainer);
            //grp->getOrCreateUserDataContainer();
            grp->setUserValue("TAG", 1);
            double ptr;
            memcpy(&ptr, &tag, sizeof(tag));
            grp->setUserValue("TAGPTR", ptr);

            std::cout << "insert osg geom" << std::endl;
            //obj->getOsgNode()->addChild(grp);
            osgNode->getParent(0)->addChild(grp);
            // obj->getOsgNode()->getParent(0)->addChild(grp);
            // geom->setOsgNode(grp);
            //m_osgScene->addChild(grp);
            //obj->getOsgNode()->setNodeMask(0);

            tag->setOsg(grp);
        }//*/

        obj->addTag(tag);

        // reorder tags
        obj->checkTags();

        // add in treeview
        uri.resetCursor();
        appGui().getTreeView()->addItemGeneric(uri, tag->getStringId().c_str(), "Tag");
        appGui().getControllerGui().addTag(uri, tag);
    }

    appGui().getMainWindow()->m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
