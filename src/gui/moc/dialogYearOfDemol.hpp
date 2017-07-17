// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef DIALOGYEAROFDEMOL_HPP
#define DIALOGYEAROFDEMOL_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "libcitygml/URI.hpp"

////////////////////////////////////////////////////////////////////////////////
namespace Ui {
    class DialogYearOfDemol;
}
////////////////////////////////////////////////////////////////////////////////
class DialogYearOfDemol : public QDialog
{
    Q_OBJECT

public:
    explicit DialogYearOfDemol(QWidget *parent = 0);
    ~DialogYearOfDemol();

    void editDates(const vcity::URI& uri);

private:
    Ui::DialogYearOfDemol *ui;
    std::string terminationDate;
    int yearOfDemolition;

    void editTileDates(const vcity::URI& uri);
    void editObjectDates(const vcity::URI& uri);

    private	slots:
    void indexChanged(int);
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGYEAROFDEMOL_HPP
