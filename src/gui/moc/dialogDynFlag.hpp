#ifndef DIALOGDYNFLAG_HPP
#define DIALOGDYNFLAG_HPP
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include "core/URI.hpp"
class QLineEdit;
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
class DialogDynFlag;
}
////////////////////////////////////////////////////////////////////////////////
class DialogDynFlag : public QDialog
{
    Q_OBJECT

public:
    explicit DialogDynFlag(QWidget *parent = 0);
    ~DialogDynFlag();

    void addDynFlag(const vcity::URI& uri);

private slots:
    void addFeatureArray();
    void addFeatureFile();

private:
    Ui::DialogDynFlag *ui;
    std::vector<QLineEdit*> m_featureArrayNames;
    std::vector<QLineEdit*> m_featureArrayValues;

    std::vector<QLineEdit*> m_featureFileNames;
    std::vector<QLineEdit*> m_featureFilePaths;
};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOGDYNFLAG_HPP
