// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogState.hpp"
#include "ui_dialogState.h"
#include "gui/applicationGui.hpp"
#include <QSettings>
#include <QFileDialog>
#include "moc/mainWindow.hpp"
////////////////////////////////////////////////////////////////////////////////
DialogState::DialogState(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogState)
{
    ui->setupUi(this);
    connect(ui->toolButton,SIGNAL(clicked()), this, SLOT(addFeatureBox()));
}
////////////////////////////////////////////////////////////////////////////////
DialogState::~DialogState()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogState::addState(const vcity::URI& uri)
{
    appGui().getMainWindow()->m_osgView->setActive(false);

    citygml::CityObject* obj = nullptr;

    //if(m_ui->treeWidget->currentItem())
    {
        //std::cout << "select node : " << m_ui->treeWidget->currentItem()->text(0).toStdString() << std::endl;
        uri.resetCursor();
        obj = vcity::app().getScene().getCityObjectNode(uri);

        ui->comboBox->addItem("NEW");
        if(obj)
        {
            // add lod
            ui->comboBox->addItem(uri.getLastNode().c_str());
        }
        ui->comboBox->addItem("NULL");
    }

    int res = exec();

    //std::cout << "diag res : " << res << std::endl;

    bool newGeom = false;

    if(res && obj) // && m_ui->treeWidget->currentItem())
    {
        citygml::CityObject* geom = nullptr;
        //std::cout << ui.comboBox->currentText().toStdString() << std::endl;

        QString item2text;

        if(ui->comboBox->currentText() == "NEW")
        {
            newGeom = true;

            // load
            std::cout << "load new" << std::endl;

            QSettings settings("liris", "virtualcity");
            QString lastdir = settings.value("lastdir").toString();
            QStringList filenames = QFileDialog::getOpenFileNames(this, "Load scene files", lastdir);
            //QString filename = QFileDialog::getOpenFileName(0, "Load scene file", lastdir);
            for(const QString& filename : filenames)
            {
                if(!filename.isEmpty())
                {
                    citygml::ParserParams params;
                    citygml::CityModel* mdl = citygml::load(filename.toStdString(), params);
                    citygml::CityObject* bldg = mdl->getCityObjectsRoots()[0];
                    geom = bldg;
                    geom->m_path = filename.toStdString();

                    item2text = bldg->getId().c_str();
                    QFileInfo file(filename);
                    settings.setValue("lastdir", file.dir().absolutePath());

                    // create state using geom
                    citygml::CityObjectState* state = new citygml::CityObjectState(geom);
                    state->m_name = ui->lineEdit->text().toStdString();
                    state->m_parent = obj;
                    obj->addState(state);

                    // add in treeview
                    uri.resetCursor();
                    appGui().getControllerGui().addState(uri, state);
                }
            }
        }
        else if(ui->comboBox->currentText() == "NULL")
        {
            geom = nullptr;
            item2text = "NULL";
        }
        else
        {
            // use existing
            uri.resetCursor();
            geom = vcity::app().getScene().getCityObjectNode(uri); //findNode(ui.comboBox->currentText().toStdString());
            std::cout << "use existing : " << geom << std::endl;
            item2text = ui->comboBox->currentText();
        }

        if(!newGeom)
        {
            // create state using geom
            citygml::CityObjectState* state = new citygml::CityObjectState(geom);
            state->m_name = ui->lineEdit->text().toStdString();
            state->m_parent = obj;
            obj->addState(state);

            // add in treeview
            uri.resetCursor();
            appGui().getControllerGui().addState(uri, state);
        }
    }
    appGui().getMainWindow()->m_osgView->setActive(true);
}
////////////////////////////////////////////////////////////////////////////////
void DialogState::addFeatureBox()
{
    QHBoxLayout* hb = new QHBoxLayout();
    QLineEdit* key = new QLineEdit();
    QLineEdit* val = new QLineEdit();
    key->setToolTip("Feature name");
    val->setToolTip("Value");
    hb->addWidget(key);
    hb->addWidget(val);
    ui->verticalLayout_2->addLayout(hb);
}
////////////////////////////////////////////////////////////////////////////////
