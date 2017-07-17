// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef DIALOGYEAROFCONST_HPP
#define DIALOGYEAROFCONST_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "libcitygml/URI.hpp"
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
    class DialogYearOfConst;
}
////////////////////////////////////////////////////////////////////////////////
class DialogYearOfConst : public QDialog
{
    Q_OBJECT

public:
    explicit DialogYearOfConst(QWidget *parent = 0);
    ~DialogYearOfConst();

    void editDates(const vcity::URI& uri);

private:
    Ui::DialogYearOfConst *ui;
    std::string creationDate;
    int yearOfConstruction;

    void editTileDates(const vcity::URI& uri);
    void editObjectDates(const vcity::URI& uri);

    private	slots:
    void indexChanged(int);
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGYEAROFCONST_HPP
