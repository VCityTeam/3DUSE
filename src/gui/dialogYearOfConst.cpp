#include "moc/dialogYearOfConst.hpp"
#include "ui_dialogYearOfConst.h"
#include "gui/applicationGui.hpp"
#include "osg/ValueObject"

DialogYearOfConst::DialogYearOfConst(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogYearOfConst)
{
    ui->setupUi(this);
    ui->comboBox->insertItem(0, "", QVariant(0));
    ui->comboBox->insertItem(1, "yearOfConstruction", QVariant(1));
    ui->comboBox->insertItem(2, "creationDate", QVariant(2));
    //currentIndexChanged
    connect(ui->comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(indexChanged(int)));
}

DialogYearOfConst::~DialogYearOfConst()
{
    delete ui;
}

void DialogYearOfConst::editDates(const vcity::URI& uri)
{
    if (uri.getType() == "File")
    {
        editTileDates(uri);
    }
    else
    {
        editObjectDates(uri);
    }
}

void DialogYearOfConst::editTileDates(const vcity::URI& uri)
// Called if object is a Tile
{

    ui->comboBox->setCurrentIndex(0);
    ui->dateConstrEdit->setEnabled(false);
    yearOfConstruction = 2000;
    creationDate = "2000-01-01T00:00:00";

    //window execution
    int res = exec();
    if (res)
    {
        vcity::Tile* tile = vcity::app().getScene().getTile(uri);
        uri.resetCursor();
        osg::ref_ptr<osg::Group> grp = appGui().getOsgScene()->getNode(uri)->asGroup();
        QDateTime creaDate = ui->dateConstrEdit->dateTime();
        int index = ui->comboBox->currentIndex();
        // Edit OSG nodes (Tile children)
        if (grp)
        {
            for (unsigned int i = 0; i < grp->getNumChildren(); ++i)
            {
                osg::ref_ptr<osg::Node> child = grp->getChild(i);
                switch (index)
                {
                case 1: //user added/edited yearOfConstruction
                    child->setUserValue("yearOfConstruction", creaDate.date().year());
                    break;
                case 2: //user added/edited creationDate
                    child->setUserValue("creationDate", creaDate.toString(Qt::ISODate).toStdString());
                    break;
                default:
                    //do nothing
                    break;
                }
            }
        }
        // Edit CityGML Objects
        for (citygml::CityObject* o : tile->getCityModel()->getCityObjectsRoots())
        {
            switch (index)
            {
            case 1:
                o->setAttribute("yearOfConstruction", creaDate.toString("yyyy").toStdString());
            case 2:
            {
                o->setAttribute("creationDate", creaDate.toString(Qt::ISODate).toStdString());
            }
            break;
            default:
                //do nothing
                break;
            }
        }

    }
}

void DialogYearOfConst::editObjectDates(const vcity::URI& uri)
// Called if object is a not a Tile
{
    citygml::CityObject* obj = nullptr;
    uri.resetCursor();

    obj = vcity::app().getScene().getCityObjectNode(uri);
    osg::ref_ptr<osg::Node> node = appGui().getOsgScene()->getNode(uri);

    bool a = node->getUserValue("yearOfConstruction", yearOfConstruction);
    bool b = node->getUserValue("creationDate", creationDate);

    if (a)
    {
        ui->comboBox->setCurrentIndex(1);
        ui->dateConstrEdit->setEnabled(true);
        ui->dateConstrEdit->setDateTime(QDateTime(QDate(yearOfConstruction, 1, 1)));
        creationDate = std::to_string(yearOfConstruction) + "-01-01T00:00:00";
    }
    else if (b)
    {
        ui->comboBox->setCurrentIndex(2);
        ui->dateConstrEdit->setEnabled(true);
        std::string str_cDate = creationDate;
        QDateTime cDate = QDateTime::fromString(QString::fromStdString(str_cDate), Qt::ISODate);
        ui->dateConstrEdit->setDateTime(cDate);
        yearOfConstruction = cDate.date().year();
    }
    else
    {
        ui->comboBox->setCurrentIndex(0);
        ui->dateConstrEdit->setEnabled(false);
        yearOfConstruction = 2000;
        creationDate = "2000-01-01T00:00:00";
    }

    //window execution
    int res = exec();
    if (res)
    {
        QDateTime creaDate = ui->dateConstrEdit->dateTime();
        int index = ui->comboBox->currentIndex();
        switch (index)
        {
        case 1: //user added/edited yearOfConstruction
            obj->setAttribute("yearOfConstruction", creaDate.toString("yyyy").toStdString());
            node->setUserValue("yearOfConstruction", creaDate.date().year());
            break;
        case 2: //user added/edited creationDate
            obj->setAttribute("creationDate", creaDate.toString(Qt::ISODate).toStdString());
            node->setUserValue("creationDate", creaDate.toString(Qt::ISODate).toStdString());
            break;
        default:
            //do nothing
            break;
        }
    }
}

void DialogYearOfConst::indexChanged(int currentIndex)
{
    switch (currentIndex)
    {
    case 0:
        ui->dateConstrEdit->setEnabled(false);
        break;
    case 1:
    {
        ui->dateConstrEdit->setEnabled(true);
        ui->dateConstrEdit->setDateTime(QDateTime(QDate(yearOfConstruction, 1, 1)));
    }
    break;
    case 2:
    {
        ui->dateConstrEdit->setEnabled(true);
        std::string str_cDate = creationDate;
        QDateTime cDate = QDateTime::fromString(QString::fromStdString(str_cDate), Qt::ISODate);
        ui->dateConstrEdit->setDateTime(cDate);
    }
    break;
    default:
        //do nothing
        break;
    }
}