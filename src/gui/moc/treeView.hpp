#ifndef __TREEVIEW_HPP__
#define __TREEVIEW_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <QTreeWidget>
#include "core/URI.hpp"
////////////////////////////////////////////////////////////////////////////////
class MainWindow;
////////////////////////////////////////////////////////////////////////////////
/// \brief The TreeView class, handles everything relative to the Qt treeview
///
class TreeView : public QObject
{
    Q_OBJECT

public:
    TreeView(QTreeWidget* tree, MainWindow* widget);

    /// \brief init Init the treeview
    void init();

    /// \brief reset Reset the treeview (root item and one layer)
    void reset();

    QTreeWidgetItem* addItemGeneric(const QString& name, const QString& type);
    QTreeWidgetItem* addItemRoot();
    QTreeWidgetItem* addItemLayer(const QString& name);

    QTreeWidgetItem* addItemTag();
    QTreeWidgetItem* addItemFlag();

    void deleteItem(const std::string& URI);
    void deleteItem(const vcity::URI& URI);

    QTreeWidgetItem* findItem(const vcity::URI& URI) const;

    vcity::URI getURI(QTreeWidgetItem* item) const;

private slots:
    void slotAddTile();
    void slotEditTile();
    void slotDeleteTile();
    void slotAddLayer();
    void slotEditLayer();
    void slotDeleteLayer();
    void slotAddBuilding();
    void slotEditBuilding();
    void slotDeleteBuilding();
    void slotAddFlag();
    void slotAddDynFlag();
    void slotAddTag();
    void slotEditFlag();
    void slotEditDynFlag();
    void slotEditTag();
    void slotDeleteFlag();
    void slotDeleteDynFlag();
    void slotDeleteTag();

    void slotSelectNode(QTreeWidgetItem* item, int column);
    void slotItemChanged(QTreeWidgetItem*, int);
    void slotItemClicked(QTreeWidgetItem*,int);

private:
    void resetActions();

    QTreeWidget* m_tree;
    MainWindow* m_mainWindow;

    // all registered actions
    QAction* m_actionAddTile;
    QAction* m_actionEditTile;
    QAction* m_actionDeleteTile;
    QAction* m_actionAddLayer;
    QAction* m_actionEditLayer;
    QAction* m_actionDeleteLayer;
    QAction* m_actionAddBuilding;
    QAction* m_actionEditBuilding;
    QAction* m_actionDeleteBuilding;
    QAction* m_actionAddFlag;
    QAction* m_actionAddDynFlag;
    QAction* m_actionAddTag;
    QAction* m_actionEditFlag;
    QAction* m_actionEditDynFlag;
    QAction* m_actionEditTag;
    QAction* m_actionDeleteFlag;
    QAction* m_actionDeleteDynFlag;
    QAction* m_actionDeleteTag;
};
////////////////////////////////////////////////////////////////////////////////
#endif // __TREEVIEW_HPP__
