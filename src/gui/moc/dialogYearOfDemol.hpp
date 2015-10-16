#ifndef DIALOGYEAROFDEMOL_HPP
#define DIALOGYEAROFDEMOL_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "core/URI.hpp"
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
	int terminationDate;
	int yearOfDemolition;
		
	void editTileDates(const vcity::URI& uri);
	void editObjectDates(const vcity::URI& uri);

	private	slots:
		void indexChanged(int);
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGYEAROFDEMOL_HPP