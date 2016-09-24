#include <QApplication>
#include "Scene_polyhedron_transform_item.h"
#include "Kernel_type.h"
#include "Polyhedron_type.h"
#include <CGAL/Three/Viewer_interface.h>

struct Scene_polyhedron_transform_item_priv
{
  Scene_polyhedron_transform_item_priv(const qglviewer::Vec& pos,const Scene_polyhedron_item* poly_item, Scene_polyhedron_transform_item *parent)
    : poly_item(poly_item),
      manipulable(false),
      frame(new CGAL::Three::Scene_item::ManipulatedFrame()),
      poly(poly_item->polyhedron()),
      center_(pos)
  {
    item = parent;
    frame->setPosition(pos);
    nb_lines = 0;
  }
  ~Scene_polyhedron_transform_item_priv()
{
  delete frame;
}
  void initialize_buffers(CGAL::Three::Viewer_interface *viewer) const;
  void compute_elements() const;
  enum VAOs {
      Edges=0,
      NbOfVaos
  };
  enum VBOs {
      Vertices = 0,
      NbOfVbos
  };

  const Scene_polyhedron_item* poly_item;
  bool manipulable;
  qglviewer::ManipulatedFrame* frame;
  const Polyhedron* poly;
  qglviewer::Vec center_;
  Scene_polyhedron_transform_item *item;


  mutable QOpenGLShaderProgram *program;
  mutable std::vector<float> positions_lines;
  mutable std::size_t nb_lines;
};

Scene_polyhedron_transform_item::Scene_polyhedron_transform_item(const qglviewer::Vec& pos,const Scene_polyhedron_item* poly_item_,const CGAL::Three::Scene_interface*):
    Scene_item(Scene_polyhedron_transform_item_priv::NbOfVbos,Scene_polyhedron_transform_item_priv::NbOfVaos)
{
  d = new Scene_polyhedron_transform_item_priv(pos,poly_item_, this);
    invalidateOpenGLBuffers();
}


void Scene_polyhedron_transform_item_priv::initialize_buffers(CGAL::Three::Viewer_interface *viewer =0) const
{
    //vao for the edges
    {
        program = item->getShaderProgram(Scene_polyhedron_transform_item::PROGRAM_WITHOUT_LIGHT, viewer);
        program->bind();

        item->vaos[Edges]->bind();
        item->buffers[Vertices].bind();
        item->buffers[Vertices].allocate(positions_lines.data(),
                            static_cast<int>(positions_lines.size()*sizeof(float)));
        program->enableAttributeArray("vertex");
        program->setAttributeBuffer("vertex",GL_FLOAT,0,3);
        item->buffers[Vertices].release();
        item->vaos[Edges]->release();

        program->release();
    }
    nb_lines = positions_lines.size();
    positions_lines.resize(0);
    std::vector<float>(positions_lines).swap(positions_lines);

    item->are_buffers_filled = true;
}


void Scene_polyhedron_transform_item_priv::compute_elements() const
{
    QApplication::setOverrideCursor(Qt::WaitCursor);
    positions_lines.resize(0);
    typedef Kernel::Point_3		        Point;
    typedef Polyhedron::Edge_const_iterator	Edge_iterator;

    Edge_iterator he;
    for(he = poly->edges_begin();
        he != poly->edges_end();
        he++)
    {
        const Point& a = he->vertex()->point();
        const Point& b = he->opposite()->vertex()->point();
        positions_lines.push_back(a.x()-center_.x);
        positions_lines.push_back(a.y()-center_.y);
        positions_lines.push_back(a.z()-center_.z);

        positions_lines.push_back(b.x()-center_.x);
        positions_lines.push_back(b.y()-center_.y);
        positions_lines.push_back(b.z()-center_.z);

    }
    QApplication::restoreOverrideCursor();
}

void Scene_polyhedron_transform_item::drawEdges(CGAL::Three::Viewer_interface* viewer) const
{
    if(!are_buffers_filled)
        d->initialize_buffers(viewer);
    vaos[Scene_polyhedron_transform_item_priv::Edges]->bind();
    d->program = getShaderProgram(PROGRAM_WITHOUT_LIGHT);
    attribBuffers(viewer,PROGRAM_WITHOUT_LIGHT);
    d->program->bind();
    QMatrix4x4 f_matrix;
    for (int i=0; i<16; ++i){
        f_matrix.data()[i] = (float)d->frame->matrix()[i];
    }
    QColor color = this->color();
    d->program->setAttributeValue("colors",color);
    d->program->setUniformValue("f_matrix", f_matrix);
    d->program->setUniformValue("is_selected", false);
    viewer->glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(d->nb_lines/3));
    vaos[Scene_polyhedron_transform_item_priv::Edges]->release();
    d->program->release();

}

QString Scene_polyhedron_transform_item::toolTip() const {
    return QObject::tr("<p>Affine transformation of <b>%1</b></p>"
                       "<p>Keep <b>Ctrl</b> pressed and use the arcball to define an affine transformation.<br />"
                       "Press <b>S</b> to apply the affine transformation to a copy of <b>%1</b>.</p>")
            .arg(getBase()->name());
}
bool Scene_polyhedron_transform_item::keyPressEvent(QKeyEvent* e){
    if (e->key()==Qt::Key_S){
    Q_EMIT stop();
        return true;
    }
    return false;
}

void
Scene_polyhedron_transform_item::compute_bbox() const {
    const Kernel::Point_3& p = *(d->poly->points_begin());
    CGAL::Bbox_3 bbox(p.x(), p.y(), p.z(), p.x(), p.y(), p.z());
    for(Polyhedron::Point_const_iterator it = d->poly->points_begin();
        it != d->poly->points_end();
        ++it) {
        bbox = bbox + it->bbox();
    }
    _bbox = Bbox(bbox.xmin(),bbox.ymin(),bbox.zmin(),
                bbox.xmax(),bbox.ymax(),bbox.zmax());
}


void Scene_polyhedron_transform_item::invalidateOpenGLBuffers()
{
    d->compute_elements();
    are_buffers_filled = false;
    compute_bbox();
}

bool Scene_polyhedron_transform_item::manipulatable() const { return d->manipulable; }
CGAL::Three::Scene_item::ManipulatedFrame* Scene_polyhedron_transform_item::manipulatedFrame() { return d->frame; }
void Scene_polyhedron_transform_item::setManipulatable(bool b = true) { d->manipulable = b;}
const Scene_polyhedron_item* Scene_polyhedron_transform_item::getBase() const{ return d->poly_item;  };
const qglviewer::Vec& Scene_polyhedron_transform_item::center() const { return d->center_; }
Scene_polyhedron_transform_item::~Scene_polyhedron_transform_item() { delete d; Q_EMIT killed(); }
