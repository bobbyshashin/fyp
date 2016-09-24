#include <CGAL/Three/Polyhedron_demo_plugin_interface.h>

#include "Scene_polyhedron_item.h"
#include "Polyhedron_type.h"

#include <QApplication>
#include <QMainWindow>
#include <QInputDialog>
#include <QTime>
#include <QAction>

#include <CGAL/Surface_mesh_simplification/HalfedgeGraph_Polyhedron_3.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
using namespace CGAL::Three;
class Polyhedron_demo_mesh_simplification_plugin : 
  public QObject,
  public Polyhedron_demo_plugin_interface
{
  Q_OBJECT
  Q_INTERFACES(CGAL::Three::Polyhedron_demo_plugin_interface)
  Q_PLUGIN_METADATA(IID "com.geometryfactory.PolyhedronDemo.PluginInterface/1.0")

public:

  QList<QAction*> actions() const {
    return _actions;
  }

  void init(QMainWindow* mainWindow,
            Scene_interface* scene_interface,
            Messages_interface*)
  {
      mw = mainWindow;
      scene = scene_interface;
     QAction *actionSimplify = new QAction("Simplification", mw);
      actionSimplify->setProperty("subMenuName",
                                  "Triangulated Surface Mesh Simplification");
      connect(actionSimplify, SIGNAL(triggered()), this, SLOT(on_actionSimplify_triggered()));
      _actions <<actionSimplify;

  }
  bool applicable(QAction*) const { 
    return qobject_cast<Scene_polyhedron_item*>(scene->item(scene->mainSelectionIndex()));
  }
public Q_SLOTS:
  void on_actionSimplify_triggered();
private :
  Scene_interface *scene;
  QMainWindow *mw;
  QList<QAction*> _actions;

}; // end Polyhedron_demo_mesh_simplification_plugin

void Polyhedron_demo_mesh_simplification_plugin::on_actionSimplify_triggered()
{
  const CGAL::Three::Scene_interface::Item_id index = scene->mainSelectionIndex();
  
  Scene_polyhedron_item* item = 
    qobject_cast<Scene_polyhedron_item*>(scene->item(index));

  if(item)
  {
    Polyhedron* pMesh = item->polyhedron();

    // get option (#edges)
    bool ok;
  
    const int nb_edges = 
    QInputDialog::getInt(mw, tr("Stop condition"),
      tr("Number of edges:"),
      (int)(pMesh->size_of_halfedges () / 4), // default value: current #edges / 2 
      3, // min = one triangle
      (int)pMesh->size_of_halfedges(), // max #edges
      1, // step for the spinbox
      &ok);

    // check user cancellation
    if(!ok)
      return;

    // simplify
    QTime time;
    time.start();
    std::cout << "Simplify...";
    QApplication::setOverrideCursor(Qt::WaitCursor);
    namespace SMS = CGAL::Surface_mesh_simplification;
    SMS::Count_stop_predicate< Polyhedron > stop(nb_edges); // target #edges
    SMS::edge_collapse( *pMesh, stop,
                        CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index,*pMesh))
                                         .halfedge_index_map(get(CGAL::halfedge_external_index,*pMesh)));
    std::cout << "ok (" << time.elapsed() << " ms, " 
      << pMesh->size_of_halfedges() / 2 << " edges)" << std::endl;

    // update scene
    item->invalidateOpenGLBuffers();
    scene->itemChanged(index);
    QApplication::restoreOverrideCursor();
  }
}

#include "Mesh_simplification_plugin.moc"
