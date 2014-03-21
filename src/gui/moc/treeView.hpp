#ifndef __TREEVIEW_HPP__
#define __TREEVIEW_HPP__
////////////////////////////////////////////////////////////////////////////////
#include <QTreeWidget>
#include "core/URI.hpp"
#include "core/application.hpp"
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

    QTreeWidgetItem* addItemGeneric(const vcity::URI& uri, const QString& name, const QString& type);

    QTreeWidgetItem* createItemGeneric(const QString& name, const QString& type, const bool checkable = true);
    QTreeWidgetItem* createItemRoot();
    QTreeWidgetItem* createItemLayer(const QString& name);

    QTreeWidgetItem* addItemTag();
    QTreeWidgetItem* addItemFlag();

    void deleteItem(const std::string& URI);
    void deleteItem(const vcity::URI& URI);

    /// \brief getURI Compute URI of a treeview item
    /// \param item TreeView item
    /// \return URI
    vcity::URI getURI(QTreeWidgetItem* item) const;

    QTreeWidget* getTree();
    QTreeWidgetItem* getCurrentItem();

    void addLayer(const vcity::URI& uri);
    void setLayerName(const vcity::URI& uri, const std::string& name);
    void deleteLayer(const vcity::URI& uri);

    void addTile(const vcity::URI& uriLayer, vcity::Tile& tile);
    void setTileName(const vcity::URI& uri, std::string& name);
    void deleteTile(const vcity::URI& uri);

	void addNodeRecursively(QTreeWidgetItem* parent, const osg::ref_ptr<osg::Node> node, std::string strLevel);
	void addAssimpNode(const vcity::URI& uriLayer, const osg::ref_ptr<osg::Node> node);

    void addCityObject(QTreeWidgetItem* parent, citygml::CityObject* node);

    /// \brief selectItem Select item in treeview
    /// \param uri URI pointing to selected node
    void selectItem(const vcity::URI& uri);

    /// \brief resetSelection Deselect all items
    void resetSelection();

    /// \brief getNode Find an item using a URI
    /// \param uri URI pointing to the wanted item/node
    /// \return QTreeWidgetItem corresponding to uri
    QTreeWidgetItem* getNode(const vcity::URI& uri);

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
    void slotItemChanged(QTreeWidgetItem* item, int column);
    void slotItemClicked(QTreeWidgetItem* item,int column);
    void slotItemDoubleClicked(QTreeWidgetItem* item,int column);
    //void slotItemActivated(QTreeWidgetItem* item, int column);

    void slotFilter();

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
