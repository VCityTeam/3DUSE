s// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#include "moc/dialogEditAssimpNode.hpp"
#include "ui_dialogEditAssimpNode.h"
#include "applicationGui.hpp"
#include <osg/MatrixTransform>
////////////////////////////////////////////////////////////////////////////////
DialogEditAssimpNode::DialogEditAssimpNode(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogEditAssimpNode)
{
    ui->setupUi(this);
}
////////////////////////////////////////////////////////////////////////////////
DialogEditAssimpNode::~DialogEditAssimpNode()
{
    delete ui;
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditAssimpNode::editAssimpNode(const vcity::URI& uri)
{
    osg::ref_ptr<osg::Node> node = appGui().getOsgScene()->getNode(uri);
    if(node)
    {
        setName(node->getName().c_str());

		if(node->asGroup()->getChild(0))
			if(node->asGroup()->getChild(0)->asTransform())
				if(node->asGroup()->getChild(0)->asTransform()->asMatrixTransform())
				{
					osg::ref_ptr<osg::MatrixTransform> mt = node->asGroup()->getChild(0)->asTransform()->asMatrixTransform();
					osg::Vec3d v = mt->getMatrix().getTrans();
					setOffset(v.x(),v.y(),v.z());
					//std::cout << "getTrans ok" << std::endl;
				}

        if(exec() && !getName().isEmpty()) // todo : verifier valeur x, y, z
        {
            appGui().getControllerGui().setAssimpNodeName(uri, getName().toStdString());

			if(node->asGroup()->getChild(0))
				if(node->asGroup()->getChild(0)->asTransform())
					if(node->asGroup()->getChild(0)->asTransform()->asMatrixTransform())
					{
						osg::ref_ptr<osg::MatrixTransform> mt = node->asGroup()->getChild(0)->asTransform()->asMatrixTransform();
						double x,y,z;
						getOffset(x,y,z);
						osg::Matrix m = mt->getMatrix();
						m.setTrans(x,y,z);
						mt->setMatrix(m); 
						//std::cout << "setTrans ok" << std::endl;
					}
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditAssimpNode::setName(const QString& str)
{
    ui->lineEdit->setText(str);
}
////////////////////////////////////////////////////////////////////////////////
QString DialogEditAssimpNode::getName() const
{
    return ui->lineEdit->text();
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditAssimpNode::setOffset(double x, double y, double z)
{
    ui->lineEditLowerBoundX->setText(QString::number(x));
    ui->lineEditLowerBoundY->setText(QString::number(y));
	ui->lineEditLowerBoundZ->setText(QString::number(z));
}
////////////////////////////////////////////////////////////////////////////////
void DialogEditAssimpNode::getOffset(double& x, double& y, double& z)
{
	x = ui->lineEditLowerBoundX->text().toDouble();
	y = ui->lineEditLowerBoundY->text().toDouble();
	z = ui->lineEditLowerBoundZ->text().toDouble();
}
////////////////////////////////////////////////////////////////////////////////
