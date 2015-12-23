#ifndef _DIALOG_OBJ_CITYGML_HPP
#define _DIALOG_OBJ_CITYGML_HPP

#include <QDialog>

namespace Ui {
	class DialogOBJCityGML;
}

class DialogOBJCityGML : public QDialog
{
	Q_OBJECT
public:
	explicit DialogOBJCityGML(QWidget *parent = 0);
	~DialogOBJCityGML(void);

	void convertOBJ();

private:
	Ui::DialogOBJCityGML* ui;
};

#endif //_DIALOG_OBJ_CITYGML_HPP

