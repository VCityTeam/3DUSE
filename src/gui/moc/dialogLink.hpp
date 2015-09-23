#ifndef _DIALOGLINK_HPP_
#define _DIALOGLINK_HPP_

#include <QDialog>
#include "core/URI.hpp"
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