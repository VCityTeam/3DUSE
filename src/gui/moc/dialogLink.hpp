// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef _DIALOGLINK_HPP_
#define _DIALOGLINK_HPP_

#include <QDialog>
#include "libcitygml/URI.hpp"
#include "cityobject.hpp"

namespace Ui
{
    class DialogLink;
}

class DialogLink : public QDialog
{
    Q_OBJECT

public:
    explicit DialogLink(QWidget *parent = 0);
    ~DialogLink();

    void addLink(const vcity::URI& uri);

private:
    QStringList filenames;
    void initComboBox();
    citygml::CityObject * createXLink(int);

    Ui::DialogLink *ui;

    private slots:
    void loadNewObjects();
};

#endif
