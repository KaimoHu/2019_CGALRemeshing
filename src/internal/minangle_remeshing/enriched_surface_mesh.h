#ifndef CGAL_SURFACE_MESH_PROPERTIES_H
#define CGAL_SURFACE_MESH_PROPERTIES_H

#include <CGAL/Surface_mesh.h>
#include <boost/function_output_iterator.hpp>

namespace CGAL {
namespace Polygon_mesh_processing {
namespace internal {

// const data
const double DOUBLE_MAX = 1000000.0;
const double DOUBLE_MIN = -1000000.0;
const int MAX_VALUE = 10000;
const double MIN_VALUE = 0.0001;  // specified for numerical stability
const double SQUARED_MIN_VALUE = 0.00000001;

template<typename Kernel>
class Surface_mesh_properties {
public:
  // TODO: finally, check whether we can modify these to private access

  // type definitions
  typedef typename Kernel::FT FT;
  typedef typename Kernel::Vector_3 Vector;
  typedef typename Kernel::Vector_3 Normal;
  typedef typename Kernel::Point_3 Point;
  typedef typename CGAL::Surface_mesh<Point> Surface_mesh;
  typedef typename boost::graph_traits<Surface_mesh>::halfedge_descriptor halfedge_descriptor;
  typedef typename boost::graph_traits<Surface_mesh>::edge_descriptor edge_descriptor;
  typedef typename boost::graph_traits<Surface_mesh>::vertex_descriptor vertex_descriptor;
  typedef typename boost::graph_traits<Surface_mesh>::face_descriptor face_descriptor;

  // struct definitions
  struct Point_comp {
    bool operator()(const Point &a, const Point &b) {
      Vector vec = a - b;
      if (vec * vec < SQUARED_MIN_VALUE) {
        return false;
      }
      else {
        return a < b;
      }
    }
  };

public:
  // life cycles
  Surface_mesh_properties() {

  }

  virtual ~Surface_mesh_properties() {}

  // normals

  // surface_mesh property access
  double get_average_length(const Surface_mesh &sm) const {
    double sum_length = 0.0;
    for (Surface_mesh::Edge_range::iterator e = sm.edges().begin();
        e != sm.edges().end(); ++e) {
      const Point &source = sm.point(sm.source(sm.halfedge(*e)));
      const Point &target = sm.point(sm.target(sm.halfedge(*e)));
      sum_length += CGAL::sqrt(CGAL::squared_distance(source, target));
    }
    if (sm.number_of_edges() == 0) {
      return sum_length;
    }
    else {
      return sum_length / sm.number_of_edges();
    }
  }


private:


private:

};

}
}
}

#endif // CGAL_SURFACE_MESH_PROPERTIES_H
