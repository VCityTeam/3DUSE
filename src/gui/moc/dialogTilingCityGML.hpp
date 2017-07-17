// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef DIALOGTILINGCITYGML_HPP
#define DIALOGTILINGCITYGML_HPP

#include <QDialog>

namespace Ui {
    class DialogTilingCityGML;
}

class DialogTilingCityGML : public QDialog
{
    Q_OBJECT

        private slots:
    void chooseCityGMLPathSlot();
    void chooseOutputPathSlot();
    void TilingCityGMLSlot();

public:
    explicit DialogTilingCityGML(QWidget *parent = 0);
    ~DialogTilingCityGML();

private:
    Ui::DialogTilingCityGML *ui;
};

#endif // DIALOGTILINGCITYGML_HPP
