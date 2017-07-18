// Copyright University of Lyon, 2012 - 2017
// Distributed under the GNU Lesser General Public License Version 2.1 (LGPLv2)
// (Refer to accompanying file LICENSE.md or copy at
//  https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html )

#ifndef DIALOG_FloodAR_H
#define DIALOG_FloodAR_H
////////////////////////////////////////////////////////////////////////////////
#include <QDialog>
#include <QFileDialog>
////////////////////////////////////////////////////////////////////////////////
namespace Ui {
    class dialogFloodAR;
}
////////////////////////////////////////////////////////////////////////////////
class dialogFloodAR : public QDialog
{
    Q_OBJECT

public:
    explicit dialogFloodAR(QWidget *parent = 0);
    ~dialogFloodAR();

private:
    Ui::dialogFloodAR *ui;

    public slots:

    void browseWorkingDirectory();

    void browseInputASCCut();
    void cutASC();

    void browseInputTextureCut();
    void textureCut();

    void browseInputSHPCut();
    void SHPCut();

    void ASCtoWater();

    void ASCtoTerrain();

    void buildBoundingBoxes();
    void ShpExtrusion();

};
////////////////////////////////////////////////////////////////////////////////
#endif // DIALOG_FloodAR_H
