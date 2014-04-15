#include "moc/dialogDynFlag.hpp"
#include "ui_dialogDynFlag.h"
#include "gui/applicationGui.hpp"
#include <QSettings>
#include <QFileDialog>
////////////////////////////////////////////////////////////////////////////////
DialogDynFlag::DialogDynFlag(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogDynFlag)
{
    ui->setupUi(this);

    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(addFeatureArray()));
    connect(ui->pushButton_2, SIGNAL(clicked()), this, SLOT(addFeatureFile()));
}
////////////////////////////////////////////////////////////////////////////////
DialogDynFlag::~DialogDynFlag()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogDynFlag::addDynFlag(const vcity::URI& uri)
{
    citygml::CityObject* obj = 0;

    m_featureArrayNames.clear();
    m_featureArrayValues.clear();
    m_featureFileNames.clear();
    m_featureFilePaths.clear();

    //if(m_ui->treeWidget->currentItem())
    {
        //std::cout << "select node : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << std::endl;
        obj = vcity::app().getScene().getCityObjectNode(uri);

        if(obj)
        {
            // add lod
            ui->comboBox->addItem(uri.getLastNode().c_str());
        }
        ui->comboBox->addItem("NULL");
        ui->comboBox->addItem("NEW");
    }

    int res = exec();

    //std::cout << "diag res : " << res << std::endl;

    if(res && obj) // && m_ui->treeWidget->currentItem())
    {
        citygml::CityObject* geom = 0;
        //std::cout << ui.comboBox->currentText().toStdString() << std::endl;

        QString item2text;

        if(ui->comboBox->currentText() == "NEW")
        {
            // load
            std::cout << "load new" << std::endl;

            QSettings settings("liris", "virtualcity");
            QString lastdir = settings.value("lastdir").toString();
            QString filename = QFileDialog::getOpenFileName(0, "Load scene file", lastdir);
            citygml::ParserParams params;
            citygml::CityModel* mdl = citygml::load(filename.toStdString(), params);
            citygml::CityObject* bldg = mdl->getCityObjectsRoots()[0];
            geom = bldg;
            geom->m_path = filename.toStdString();
            std::cout << "nb : " << mdl->getCityObjectsRoots().size()<< std::endl;

            // create osg geometry
            /*size_t pos = filename.toStdString().find_last_of("/\\");
            std::string path = filename.toStdString().substr(0, pos);
            ReaderOsgCityGML readerOsgGml(path);

            osg::ref_ptr<osg::Group> grp = readerOsgGml.createCityObject(bldg);

            if(bldg->getType() == citygml::COT_Building)
            {
                int yearOfConstruction;
                int yearOfDemolition;

                std::istringstream(bldg->getAttribute("yearOfConstruction")) >> yearOfConstruction;
                std::istringstream(bldg->getAttribute("yearOfDemolition")) >> yearOfDemolition;

                grp->setUserValue("yearOfConstruction", yearOfConstruction);
                grp->setUserValue("yearOfDemolition", yearOfDemolition);
            }

            bldg->setOsgNode(grp);*/
            item2text = bldg->getId().c_str();
        }
        else if(ui->comboBox->currentText() == "NULL")
        {
            geom = NULL;
            item2text = "NULL";
        }
        else
        {
            // use existing
            geom = vcity::app().getScene().getCityObjectNode(uri); //findNode(ui.comboBox->currentText().toStdString());
            std::cout << "use existing : " << geom << std::endl;
            item2text = ui->comboBox->currentText();
        }

        citygml::BuildingDynFlag* flag = new citygml::BuildingDynFlag(geom);
        flag->m_name = ui->lineEdit->text().toStdString();
        flag->m_parent = obj;

        // parse features
        for(int i=0; i<m_featureArrayNames.size(); ++i)
        {
            citygml::DataSourceArray* ds = new citygml::DataSourceArray(m_featureArrayNames[i]->text().toStdString(), m_featureArrayValues[i]->text().toStdString());
            flag->addDataSource(ds);
        }
        for(int i=0; i<m_featureFileNames.size(); ++i)
        {
            citygml::DataSourceFile* ds = new citygml::DataSourceFile(m_featureFileNames[i]->text().toStdString(), m_featureFilePaths[i]->text().toStdString());
            flag->addDataSource(ds);
        }

        obj->addFlag(flag);

        QTreeWidgetItem* item = new QTreeWidgetItem(QStringList(flag->getStringId().c_str()));
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(0, Qt::Checked);
        item->setText(1, "DynFlag");

        QTreeWidgetItem* item2 = new QTreeWidgetItem(QStringList(item2text));
        item->addChild(item2);

        //appGui().m_ui treeWidget->currentItem()->addChild(item);
        appGui().getTreeView()->addItemGeneric(uri, flag->getStringId().c_str(), "DynFlag");
    }
}
////////////////////////////////////////////////////////////////////////////////
void DialogDynFlag::addFeatureArray()
{
    QHBoxLayout* hb = new QHBoxLayout();
    QLineEdit* key = new QLineEdit(); m_featureArrayNames.push_back(key);
    QLineEdit* val = new QLineEdit(); m_featureArrayValues.push_back(val);
    key->setToolTip("Feature name");
    val->setToolTip("Values : date(format : yyyy/MM/dd-HH:mm:ss) value (separator : |)");
    hb->addWidget(key);
    hb->addWidget(val);
    ui->verticalLayout_3->addLayout(hb);
}
////////////////////////////////////////////////////////////////////////////////
void DialogDynFlag::addFeatureFile()
{
    QHBoxLayout* hb = new QHBoxLayout();
    QLineEdit* key = new QLineEdit(); m_featureFileNames.push_back(key);
    QLineEdit* val = new QLineEdit(); m_featureFilePaths.push_back(val);
    key->setToolTip("Feature name");
    val->setToolTip("file path");
    hb->addWidget(key);
    hb->addWidget(val);
    ui->verticalLayout_4->addLayout(hb);
}
////////////////////////////////////////////////////////////////////////////////
