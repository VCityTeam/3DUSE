// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef DIALOGEDITLAYER_HPP
#define DIALOGEDITLAYER_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "libcitygml/URI.hpp"

////////////////////////////////////////////////////////////////////////////////
namespace Ui {
    class DialogEditLayer;
}
////////////////////////////////////////////////////////////////////////////////
class DialogEditLayer : public QDialog
{
    Q_OBJECT

public:
    explicit DialogEditLayer(QWidget *parent = 0);
    ~DialogEditLayer();

    void editLayer(const vcity::URI& uri);

    void setName(const QString& str);
    QString getName() const;

private:
    Ui::DialogEditLayer *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGEDITLAYER_HPP
