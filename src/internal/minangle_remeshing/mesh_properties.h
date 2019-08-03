#ifndef CGAL_MESH_PROPERTIES_H
#define CGAL_MESH_PROPERTIES_H

#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
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

namespace PMP = CGAL::Polygon_mesh_processing;
namespace NP = CGAL::parameters;

template<typename Kernel>
class Mesh_properties {

public:
  // type definitions
  // kernel related
  typedef typename Kernel::FT FT;
  typedef typename Kernel::Vector_3 Vector;
  typedef typename Kernel::Point_3 Point;
  typedef typename Vector Normal;
  // local link related
  typedef typename std::pair<Point, Point> Point_pair;        // for out links
  typedef typename std::pair<FT, Point_pair> Link;
  typedef typename std::list<Link> Link_list;
  typedef typename Link_list::iterator Link_list_iter;
  typedef typename Link_list::const_iterator Link_list_const_iter;
  typedef typename std::list<Link_list_iter> Link_iter_list;  // for in links
  typedef typename Link_iter_list::iterator Link_iter_list_iter;
  typedef typename Link_iter_list::const_iterator Link_iter_list_const_iter;
  typedef typename std::list<Link*> Link_pointer_list;
  typedef typename std::list<Link*>::iterator Link_pointer_iter;
  typedef typename std::list<Link*>::const_iterator Link_pointer_const_iter;
  // Surface_mesh related
  typedef typename CGAL::Surface_mesh<Point> Mesh;
  typedef typename boost::graph_traits<Mesh>::halfedge_descriptor halfedge_descriptor;
  typedef typename boost::graph_traits<Mesh>::edge_descriptor edge_descriptor;
  typedef typename boost::graph_traits<Mesh>::vertex_descriptor vertex_descriptor;
  typedef typename boost::graph_traits<Mesh>::face_descriptor face_descriptor;
  // Property related
  typedef typename boost::property_map<Mesh, CGAL::dynamic_face_property_t<int>>::type Face_tags;         // faces
  typedef typename boost::property_map<Mesh, CGAL::dynamic_face_property_t<Normal>>::type Face_normals;
  typedef typename boost::property_map<Mesh, CGAL::dynamic_face_property_t<FT>>::type Face_max_errors;
  typedef typename boost::property_map<Mesh, CGAL::dynamic_face_property_t<Link_list>>::type Face_link_list;
  typedef typename boost::property_map<Mesh, CGAL::dynamic_face_property_t<Link_iter_list>>::type Face_link_iter_list;
  typedef typename boost::property_map<Mesh, CGAL::dynamic_face_property_t<Link_pointer_list>>::type Face_link_pointer_list;
  typedef typename boost::property_map<Mesh, CGAL::dynamic_halfedge_property_t<int>>::type Halfedge_tags; // halfedges
  typedef typename boost::property_map<Mesh, CGAL::dynamic_halfedge_property_t<FT>>::type Halfedge_normal_dihedrals;
  typedef typename boost::property_map<Mesh, CGAL::dynamic_halfedge_property_t<bool>>::type Halfedge_are_creases;
  typedef typename boost::property_map<Mesh, CGAL::dynamic_halfedge_property_t<Link_list>>::type Halfedge_link_list;
  typedef typename boost::property_map<Mesh, CGAL::dynamic_vertex_property_t<int>>::type Vertex_tags;     // vertices
  typedef typename boost::property_map<Mesh, CGAL::dynamic_vertex_property_t<FT>>::type Vertex_max_dihedral;
  typedef typename boost::property_map<Mesh, CGAL::dynamic_vertex_property_t<FT>>::type Vertex_gaussian_curvature;
  typedef typename boost::property_map<Mesh, CGAL::dynamic_vertex_property_t<Link>>::type Vertex_link;

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
  Mesh_properties(Mesh &mesh)
  : mesh_(mesh) {
    // face related properties
    face_tags_ = get(CGAL::dynamic_face_property_t<int>(), mesh_);
    face_normals_ = get(CGAL::dynamic_face_property_t<Normal>(), mesh_);
    face_max_errors_ = get(CGAL::dynamic_face_property_t<FT>(), mesh_);
    face_out_links_ = get(CGAL::dynamic_face_property_t<Link_list>(), mesh_);
    face_face_in_links_ = get(CGAL::dynamic_face_property_t<Link_iter_list>(), mesh_);
    face_edge_in_links_ = get(CGAL::dynamic_face_property_t<Link_iter_list>(), mesh_);
    face_vertex_in_links_ = get(CGAL::dynamic_face_property_t<Link_pointer_list>(), mesh_);
    // halfedge related properties
    halfedge_tags_ = get(CGAL::dynamic_halfedge_property_t<int>(), mesh_);
    halfedge_normal_dihedrals_ = get(CGAL::dynamic_halfedge_property_t<FT>(), mesh_);
    halfedge_are_creases_ = get(CGAL::dynamic_halfedge_property_t<bool>(), mesh_);
    halfedge_out_links_ = get(CGAL::dynamic_halfedge_property_t<Link_list>(), mesh_);
    // vertex related properties
    vertex_tags_ = get(CGAL::dynamic_vertex_property_t<int>(), mesh_);                               
    vertex_max_dihedrals_ = get(CGAL::dynamic_vertex_property_t<FT>(), mesh_);
    vertex_gaussian_curvatures_ = get(CGAL::dynamic_vertex_property_t<FT>(), mesh_);
    vertex_out_link_ = get(CGAL::dynamic_vertex_property_t<Link>(), mesh_);
  }

  virtual ~Mesh_properties() {}

  // normals
  void calculate_normals() {
    for (Mesh::Face_range::iterator fi = mesh_.faces().begin();
      fi != mesh_.faces().end(); ++fi) {
      Normal normal = PMP::compute_face_normal(*fi, mesh_,
        PMP::parameters::vertex_point_map(mesh_.points()).geom_traits(Kernel()));
      put(face_normals_, *fi, normal);
    }
  }

  void calculate_local_normals(std::set<face_descriptor> *faces) {
    for (auto fit = faces->begin(); fit != faces->end(); ++fit) {
      Normal normal = PMP::compute_face_normal(*fit, mesh_, 
        PMP::parameters::vertex_point_map(mesh_.points()).geom_traits(Kernel()));
      put(face_normals_, *fit, normal);
    }
  }

  // max errors
  void calculate_max_squared_errors() {
    // precondition: mesh_ has been sampled
    reset_facet_max_squared_errors(-1.0);
    update_facet_in_max_squared_errors();
    update_facet_out_max_squared_errors();
    update_edge_in_max_squared_errors();
    update_edge_out_max_squared_errors();
    update_vertex_in_max_squared_errors();
    update_vertex_out_max_squared_errors();
  }

  void calculate_max_squared_errors(std::set<face_descriptor> *faces) {
    // precondition: mesh_ has been sampled
    for (auto fit = faces->begin(); fit != faces->end(); ++fit) {
      FT max_se = 0.0, se = 0.0;
      face_descriptor f = *fit;
      // face in links
      Link_iter_list &face_in_links = get(face_face_in_links_, f);
      for (Link_iter_list_iter it = face_in_links.begin(); 
          it != face_in_links.end(); ++it) {
        Link_list_iter llit = *it;
        const Link &link = *llit;
        se = CGAL::squared_distance(link.second.first, link.second.second);
        max_se = CGAL::max(max_se, se);
      }
      // edge in links
      Link_iter_list &edge_in_links = get(face_edge_in_links_, f);
      for (Link_iter_list_iter it = edge_in_links.begin(); 
          it != edge_in_links.end(); ++it) {
        Link_list_iter llit = *it;
        const Link &link = *llit;
        se = CGAL::squared_distance(link.second.first, link.second.second);
        max_se = CGAL::max(max_se, se);
      }
      // vertex in links
      Link_pointer_list &vertex_in_links = get(face_vertex_in_links_, f);
      for (Link_pointer_iter it = vertex_in_links.begin(); 
          it != vertex_in_links.end(); ++it) {
        Link *link = *it;
        se = CGAL::squared_distance(link->second.first, link->second.second);
        max_se = CGAL::max(max_se, se);
      }
      // face out links
      Link_list &face_out_links = get(face_out_links_, f);
      for (Link_list_iter it = face_out_links.begin(); 
          it != face_out_links.end(); ++it) {
        const Link &link = *it;
        se = CGAL::squared_distance(link.second.first, link.second.second);
        max_se = CGAL::max(max_se, se);
      }
      halfedge_descriptor h = mesh_.halfedge(f);
      for (int i = 0; i <= 2; ++i) {
        // edge out links
        halfedge_descriptor hi = h;
        if (get(halfedge_normal_dihedrals_, hi) == -1.0) {
          hi = mesh_.opposite(hi);
        }
        Link_list &edge_out_links = get(halfedge_out_links_, hi);
        for (Link_list_iter it = edge_out_links.begin(); 
            it != edge_out_links.end(); ++it) {
          const Link &link = *it;
          se = CGAL::squared_distance(link.second.first, link.second.second);
          max_se = CGAL::max(max_se, se);
        }
        // vervex out links
        vertex_descriptor v = mesh_.target(h);
        const Link &link = get(vertex_out_link_, v);
        se = CGAL::squared_distance(link.second.first, link.second.second);
        max_se = CGAL::max(max_se, se);
        h = mesh_.next(h);
      }
      put(face_max_errors_, f, max_se);
    }
  }

  /*halfedge_descriptor get_maximal_error(FT *max_error) {
    Mesh::Face_range::iterator fi = mesh_.faces().begin();
    face_descriptor max_error_face = *fi;
    FT max_se = get(face_max_errors_, max_error_face), se = 0.0;
    ++fi;
    for (; fi != mesh_.faces().end(); ++fi) {
      se = get(face_max_errors_, *fi);
      if (max_se < se) {
        max_se = se;
        max_error_face = *fi;
      }
    }
    *max_error = CGAL::sqrt(max_se);
    return get_longest_halfedge(max_error_face);
  }*/

  // mesh property access
  double get_average_length() const {
    double sum_length = 0.0;
    for (Mesh::Edge_range::iterator e = mesh_.edges().begin();
        e != mesh_.edges().end(); ++e) {
      const Point &source = mesh_.point(mesh_.source(mesh_.halfedge(*e)));
      const Point &target = mesh_.point(mesh_.target(mesh_.halfedge(*e)));
      sum_length += CGAL::sqrt(CGAL::squared_distance(source, target));
    }
    if (mesh_.number_of_edges() == 0) {
      return sum_length;
    }
    else {
      return sum_length / mesh_.number_of_edges();
    }
  }

  // NOTE: the following functions are made public only for visualization
  void compute_faces(const Color &color, std::vector<float> *pos, 
      std::vector<float> *normals, std::vector<float> *colors) {
    pos->clear();
    normals->clear();
    colors->clear();
    for (Mesh::Face_range::iterator f = mesh_.faces().begin();
        f != mesh_.faces().end(); ++f) {
      if (*f != boost::graph_traits<Mesh>::null_face()) {
        compute_face(*f, color, pos, normals, colors);
      }
    }
  }

  void compute_edges(std::vector<float> *pos) {
    pos->clear();
    for (Mesh::Edge_range::iterator e = mesh_.edges().begin();
        e != mesh_.edges().end(); ++e) {
      compute_edge(*e, pos);
    }
  }

private:
  // max errors
  void reset_facet_max_squared_errors(FT value) {
    for (Mesh::Face_range::iterator fi = mesh_.faces().begin();
        fi != mesh_.faces().end(); ++fi) {
      put(face_max_errors_, *fi, value);
    }
  }

  void update_facet_in_max_squared_errors() {
    FT se = 0.0;    // squared error
    for (Mesh::Face_range::iterator fi = mesh_.faces().begin();
      fi != mesh_.faces().end(); ++fi) {
      Link_iter_list &face_in_links = get(face_face_in_links_, *fi);
      for (Link_iter_list_iter it = face_in_links.begin(); 
          it != face_in_links.end(); ++it) {
        Link_list_iter llit = *it;
        const Link &link = *llit;
        se = CGAL::squared_distance(link.second.first, link.second.second);
        put(face_max_errors_, *fi, CGAL::max(get(face_max_errors_, *fi), se));
      }
    }
  }

  void update_facet_out_max_squared_errors() {
    FT se = 0.0;    // squared error
    for (Mesh::Face_range::iterator fi = mesh_.faces().begin();
      fi != mesh_.faces().end(); ++fi) {
      Link_list &face_out_links = get(face_out_links_, *fi);
      for (Link_list_iter it = face_out_links.begin();
        it != face_out_links.end(); ++it) {
        const Link &link = *it;
        se = CGAL::squared_distance(link.second.first, link.second.second);
        put(face_max_errors_, *fi, CGAL::max(get(face_max_errors_, *fi), se));
      }
    }
  }

  void update_edge_in_max_squared_errors() {
    FT se = 0.0;
    for (Mesh::Face_range::iterator fi = mesh_.faces().begin();
      fi != mesh_.faces().end(); ++fi) {
      Link_iter_list &edge_in_links = get(face_edge_in_links_, *fi);
      for (Link_iter_list_iter it = edge_in_links.begin(); 
        it != edge_in_links.end(); ++it) {
        Link_list_iter llit = *it;
        const Link &link = *llit;
        se = CGAL::squared_distance(link.second.first, link.second.second);
        put(face_max_errors_, *fi, CGAL::max(get(face_max_errors_, *fi), se));
      }
    }
  }

  void update_edge_out_max_squared_errors() {
    FT se = 0.0;
    for (Mesh::Edge_range::iterator ei = mesh_.edges().begin();
      ei != mesh_.edges().end(); ++ei) {
      halfedge_descriptor hd = mesh_.halfedge(*ei);
      if (get(halfedge_normal_dihedrals_, hd) == -1.0) {
        hd = mesh_.opposite(hd);
      }
      FT max_se = 0.0;
      Link_list &edge_out_links = get(halfedge_out_links_, hd);
      for (Link_list_iter it = edge_out_links.begin(); 
        it != edge_out_links.end(); ++it) {
        const Link &link = *it;
        se = CGAL::squared_distance(link.second.first, link.second.second);
        max_se = CGAL::max(max_se, se);
      }
      face_descriptor fd = mesh_.face(hd);
      put(face_max_errors_, fd, CGAL::max(get(face_max_errors_, fd), max_se));
      if (!mesh_.is_border(mesh_.opposite(hd))) {
        fd = mesh_.face(mesh_.opposite(hd));
        put(face_max_errors_, fd, CGAL::max(get(face_max_errors_, fd), max_se));
      }
    }
  }

  void update_vertex_in_max_squared_errors() {
    FT se = 0.0;
    for (Mesh::Face_range::iterator fi = mesh_.faces().begin(); 
      fi != mesh_.faces().end(); ++fi) {
      Link_pointer_list &vertex_in_links = get(face_vertex_in_links_, *fi);
      for (Link_pointer_iter it = vertex_in_links.begin();
        it != vertex_in_links.end(); ++it) {
        Link *link = *it;
        se = CGAL::squared_distance(link->second.first, link->second.second);
        put(face_max_errors_, *fi, CGAL::max(get(face_max_errors_, *fi), se));
      }
    }
  }

  void update_vertex_out_max_squared_errors() {
    FT se = 0.0;
    for (Mesh::Vertex_range::iterator vi = mesh_.vertices().begin(); 
      vi != mesh_.vertices().end(); ++vi) {
      const Link &link = get(vertex_out_link_, *vi);
      se = CGAL::squared_distance(link.second.first, link.second.second);
      CGAL::Face_around_target_circulator<Mesh> 
        vb(mesh_.halfedge(*vi), mesh_), ve(vb);
      do {
        put(face_max_errors_, *vb, CGAL::max(get(face_max_errors_, *vb) ,se));
        ++vb;
      } while (vb != ve);
    }
  }

  


  // NOTE: the following functions are made public only for visualization
  void compute_face(face_descriptor fh, const Color &color, 
      std::vector<float> *pos, std::vector<float> *normals, 
      std::vector<float> *colors) {
    const Normal &normal = get(face_normals_, fh);
    halfedge_descriptor hd = mesh_.halfedge(fh);
    do {
      const Point &point = mesh_.point(mesh_.source(hd));
      compute_point(point, pos);
      compute_normal(normal, normals);
      compute_color(color, colors);
      hd = mesh_.next(hd);
    } while (hd != mesh_.halfedge(fh));
  }

  void compute_edge(edge_descriptor ef, std::vector<float> *pos) {
    const Point &source = mesh_.point(mesh_.source(mesh_.halfedge(ef)));
    const Point &target = mesh_.point(mesh_.target(mesh_.halfedge(ef)));
    compute_point(source, pos);
    compute_point(target, pos);
  }

  void inline compute_point(const Point &point, std::vector<float> *pos) {
    pos->push_back(point.x());
    pos->push_back(point.y());
    pos->push_back(point.z());
  }

  void inline compute_normal(const Normal &normal,
    std::vector<float> *pos) {
    pos->push_back(normal.x());
    pos->push_back(normal.y());
    pos->push_back(normal.z());
  }

  void inline compute_color(const Color &color, std::vector<float> *pos) {
    pos->push_back(color.red() / 255.0f);
    pos->push_back(color.green() / 255.0f);
    pos->push_back(color.blue() / 255.0f);
  }

private:
  Mesh &mesh_;
  Face_tags face_tags_;                         // face related properties
  Face_normals face_normals_;
  Face_max_errors face_max_errors_;
  Face_link_list face_out_links_;
  Face_link_iter_list face_face_in_links_;
  Face_link_iter_list face_edge_in_links_;
  Face_link_pointer_list face_vertex_in_links_;
  Halfedge_tags halfedge_tags_;                 // halfedge related properties
  Halfedge_normal_dihedrals halfedge_normal_dihedrals_;
  Halfedge_are_creases halfedge_are_creases_;
  Halfedge_link_list halfedge_out_links_;
  Vertex_tags vertex_tags_;                     // vertex related properties
  Vertex_max_dihedral vertex_max_dihedrals_;
  Vertex_gaussian_curvature vertex_gaussian_curvatures_;
  Vertex_link vertex_out_link_;
};

}
}
}

#endif // CGAL_MESH_PROPERTIES_H
