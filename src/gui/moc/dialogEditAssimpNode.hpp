// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

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
