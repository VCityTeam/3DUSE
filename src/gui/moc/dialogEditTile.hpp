// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

////////////////////////////////////////////////////////////////////////////////
#ifndef DIALOGEDITTILE_HPP
#define DIALOGEDITTILE_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "libcitygml/URI.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
    class DialogEditTile;
}
////////////////////////////////////////////////////////////////////////////////
class DialogEditTile : public QDialog
{
    Q_OBJECT

public:
    explicit DialogEditTile(QWidget *parent = 0);
    ~DialogEditTile();

    void editTile(const vcity::URI& uri);

    void setName(const QString& str);
    QString getName() const;

private:
    Ui::DialogEditTile *ui;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGEDITTILE_HPP
