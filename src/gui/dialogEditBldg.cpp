// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogEditBldg.hpp"
#include "ui_dialogEditBldg.h"
#include "applicationGui.hpp"
#include <osg/PositionAttitudeTransform>
////////////////////////////////////////////////////////////////////////////////
DialogEditBldg::DialogEditBldg(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogEditBldg)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogEditBldg::~DialogEditBldg()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditBldg::edit(const vcity::URI& uri)
{
    DialogEditBldg diag;
    //diag.setName(m_tree->currentItem()->text(0));
    //diag.setEnvelope(0, 1, 0, 1);

    osg::ref_ptr<osg::Node> node = appGui().getOsgScene()->getNode(uri);

    if(node && node->asGeode())
    {
        //std::cout << "geode" << std::endl;
        node = node->getParent(0);
    }

    if(node)
        if(node->asTransform())
            if(node->asTransform()->asPositionAttitudeTransform())
            {
                osg::ref_ptr<osg::PositionAttitudeTransform> pos = node->asTransform()->asPositionAttitudeTransform();
                osg::Vec3d v = pos->getPosition();
                diag.setOffset(v.x(),v.y());
                //pos->setPosition(osg::Vec3d(x, y, 0));
                //std::cout << "pos : " << pos << std::endl;
            }


    //diag.setOffset(2, 2);

    if(diag.exec())
    {
        //diag.setName(m_tree->currentItem()->text(0));

        osg::ref_ptr<osg::Node> node = appGui().getOsgScene()->getNode(uri);

        if(node && node->asGeode())
        {
            //std::cout << "geode" << std::endl;
            node = node->getParent(0);
        }

        if(node)
            if(node->asTransform())
                if(node->asTransform()->asPositionAttitudeTransform())
                {
                    osg::ref_ptr<osg::PositionAttitudeTransform> pos = node->asTransform()->asPositionAttitudeTransform();
                    double x,y;
                    diag.getOffset(x,y);
                    pos->setPosition(osg::Vec3d(x, y, 0));
                    //std::cout << "pos : " << pos << std::endl;
                }


        //std::cout << node << std::endl;
    }
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditBldg::setName(const QString& str)
{
    ui->lineEdit->setText(str);
}
////////////////////////////////////////////////////////////////////////////////
QString DialogEditBldg::getName() const
{
    return ui->lineEdit->text();
}
////////////////////////////////////////////////////////////////////////////////
/*void DialogEditBldg::setEnvelope(double a, double b, double c, double d)
{
    ui->lineEditLowerBoundX->setText(QString::number(a));
    ui->lineEditLowerBoundY->setText(QString::number(b));
    //ui->lineEditUpperBoundX->setText(QString::number(c));
    //ui->lineEditUpperBoundY->setText(QString::number(d));
}*/
////////////////////////////////////////////////////////////////////////////////
void DialogEditBldg::setOffset(double x, double y)
{
    ui->lineEditLowerBoundX->setText(QString::number(x));
    ui->lineEditLowerBoundY->setText(QString::number(y));
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditBldg::getOffset(double& x, double& y)
{
 x = ui->lineEditLowerBoundX->text().toDouble();
 y = ui->lineEditLowerBoundY->text().toDouble();
}
////////////////////////////////////////////////////////////////////////////////
