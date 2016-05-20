// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef DialogEditAssimpNode_H
#define DialogEditAssimpNode_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "libcitygml/URI.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
    class DialogEditAssimpNode;
}
////////////////////////////////////////////////////////////////////////////////
class DialogEditAssimpNode : public QDialog
{
    Q_OBJECT

public:
    explicit DialogEditAssimpNode(QWidget *parent = 0);
    ~DialogEditAssimpNode();

    void editAssimpNode(const vcity::URI& uri);

    void setName(const QString& str);
    QString getName() const;

    void setOffset(double x, double y, double z);
    void getOffset(double& x, double& y, double& z);

private:
    Ui::DialogEditAssimpNode *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DialogEditAssimpNode_H
