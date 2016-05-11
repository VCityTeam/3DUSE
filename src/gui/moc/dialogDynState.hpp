// -*-c++-*- VCity project, 3DUSE, Liris, 2013, 2014
////////////////////////////////////////////////////////////////////////////////
#ifndef DIALOGDYNSTATE_HPP
#define DIALOGDYNSTATE_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "libcitygml/URI.hpp"
class QLineEdit;
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
class DialogDynState;
}
////////////////////////////////////////////////////////////////////////////////
class DialogDynState : public QDialog
{
    Q_OBJECT

public:
    explicit DialogDynState(QWidget *parent = 0);
    ~DialogDynState();

    void addDynState(const vcity::URI& uri);

private slots:
    void addFeatureArray();
    void addFeatureFile();

private:
    Ui::DialogDynState *ui;
    std::vector<QLineEdit*> m_featureArrayNames;
    std::vector<QLineEdit*> m_featureArrayValues;

    std::vector<QLineEdit*> m_featureFileNames;
    std::vector<QLineEdit*> m_featureFilePaths;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGDYNSTATE_HPP
