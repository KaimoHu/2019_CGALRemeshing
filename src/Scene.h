#ifndef CGAL_SCENE_H
#define CGAL_SCENE_H

#include <QtOpenGL/qgl.h>
#include <QtCore/qglobal.h>
#include <QMap>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <CGAL/Qt/manipulatedFrame.h>
#include <CGAL/Qt/qglviewer.h>

// local
#include "Minangle_remesh.h"
#include "old_remesh.h"
#include "Viewer.h"
#include "types.h"

class Scene : public QObject {
  Q_OBJECT

 public:

  // life cycle
  Scene();
  virtual ~Scene();

  // disable copy/move construction
  Scene(const Scene &) = delete;
  Scene(const Scene &&) = delete;
  Scene &operator = (const Scene &) = delete;
  Scene &operator = (const Scene &&) = delete;

 public:
  typedef CGAL::qglviewer::ManipulatedFrame ManipulatedFrame;
  typedef CGAL::Color Color;

  // 1) for rendering
  QGLContext *context;      // TODO: do we really need this data?
  Bbox bbox() { return m_bbox; }
  ManipulatedFrame* manipulatedFrame() const { return m_frame; }
  void update_bbox();
  void initGL();
  void draw(CGAL::QGLViewer *viewer);

  // 2) file process
  bool open(QString file_name, bool is_surface_mesh);
  bool open_input(QString file_name, bool is_surface_mesh);
  bool open_remesh(QString file_name, bool is_sueface_mesh);
  void save_remesh_as(QString file_name);

  // 3) parameter settings access
  double get_target_edge_length() const { return m_target_edge_length; }
  void set_target_edge_length(double value) { m_target_edge_length = value; }
  int get_smooth_iteration_count() const { return m_smooth_iteration_count; }
  void set_smooth_iteration_count(int value) { m_smooth_iteration_count = value; }

  double get_max_error_threshold() const 
      { return old_remesh.get_max_error_threshold();}
  void set_max_error_threshold(double value) 
      { old_remesh.set_max_error_threshold(value); }
  double get_min_angle_threshold() const 
      { return old_remesh.get_min_angle_threshold(); }
  void set_min_angle_threshold(double value) 
      { old_remesh.set_min_angle_threshold(value); }
  int get_max_mesh_complexity() const 
      { return old_remesh.get_max_mesh_complexity(); }
  void set_max_mesh_complexity(int value) 
      { old_remesh.set_max_mesh_complexity(value); }
  double get_smooth_angle_delta() const
      { return old_remesh.get_smooth_angle_delta(); }
  void set_smooth_angle_delta(double value) 
      { old_remesh.set_smooth_angle_delta(value); }
  bool get_apply_edge_flip() const 
      { return old_remesh.get_apply_edge_flip(); }
  void set_apply_edge_flip(bool value)
      { old_remesh.set_apply_edge_flip(value); }
  EdgeFlipStrategy get_edge_flip_strategy() const 
      { return old_remesh.get_edge_flip_strategy(); }
  void set_edge_flip_strategy(EdgeFlipStrategy value) 
      { old_remesh.set_edge_flip_strategy(value); }
  bool get_flip_after_split_and_collapse() const 
      { return old_remesh.get_flip_after_split_and_collapse(); }
  void set_flip_after_split_and_collapse(bool value) 
      { old_remesh.set_flip_after_split_and_collapse(value); }
  bool get_relocate_after_local_operations() const 
      { return old_remesh.get_relocate_after_local_operations(); }
  void set_relocate_after_local_operations(bool value)
      { old_remesh.set_relocate_after_local_operations(value); }
  RelocateStrategy get_relocate_strategy() const
      { return old_remesh.get_relocate_strategy(); }
  void set_relocate_strategy(RelocateStrategy value) 
      { old_remesh.set_relocate_strategy(value); }
  bool get_keep_vertex_in_one_ring() const
      { return old_remesh.get_keep_vertex_in_one_ring(); }
  void set_keep_vertex_in_one_ring(bool value) 
      { old_remesh.set_keep_vertex_in_one_ring(value); }
  bool get_use_local_aabb_tree() const
      { return old_remesh.get_use_local_aabb_tree(); }
  void set_use_local_aabb_tree(bool value)
      { old_remesh.set_use_local_aabb_tree(value); }
  int get_collapsed_list_size() const 
      { return old_remesh.get_collapsed_list_size(); }
  void set_collapsed_list_size(int value)
      { old_remesh.set_collapsed_list_size(value); }
  bool get_decrease_max_errors() const 
      { return old_remesh.get_decrease_max_errors(); }
  void set_decrease_max_errors(bool value) 
      { old_remesh.set_decrease_max_errors(value); }
  bool get_track_information() const
      { return old_remesh.get_verbose_progress(); }
  void set_track_information(bool value) 
      { old_remesh.set_verbose_progress(value); }
  bool get_apply_initial_mesh_simplification() const
      { return old_remesh.get_apply_initial_mesh_simplification(); }
  void set_apply_initial_mesh_simplification(bool value) 
      { old_remesh.set_apply_initial_mesh_simplification(value); }
  bool get_apply_final_vertex_relocation() const 
      { return old_remesh.get_apply_final_vertex_relocation(); }
  void set_apply_final_vertex_relocation(bool value) 
      { old_remesh.set_apply_final_vertex_relocation(value); }
  
  int get_samples_per_facet_in() const 
      { return old_remesh.get_samples_per_facet_in(); }
  void set_samples_per_facet_in(int value)
      { old_remesh.set_samples_per_facet_in(value); }
  int get_samples_per_facet_out() const 
      { return old_remesh.get_samples_per_facet_out(); }
  void set_samples_per_facet_out(int value) 
      { old_remesh.set_samples_per_facet_out(value); }
  int get_max_samples_per_area() const
      { return old_remesh.get_max_samples_per_area(); }
  void set_max_samples_per_area(int value)
      { old_remesh.set_max_samples_per_area(value); }
  int get_min_samples_per_triangle() const
      { return old_remesh.get_min_samples_per_triangle(); }
  void set_min_samples_per_triangle(int value)
      { old_remesh.set_min_samples_per_triangle(value); }
  int get_bvd_iteration_count() const
      { return old_remesh.get_bvd_iteration_count(); }
  void set_bvd_iteration_count(int value)
      { old_remesh.set_bvd_iteration_count(value); }
  SampleNumberStrategy get_sample_number_strategy() const
      { return old_remesh.get_sample_number_strategy(); }
  void set_sample_number_strategy(SampleNumberStrategy value) 
      { old_remesh.set_sample_number_strategy(value); }
  SampleStrategy get_sample_strategy() const
      { return old_remesh.get_sample_strategy(); }
  void set_sample_strategy(SampleStrategy value)
      { old_remesh.set_sample_strategy(value); }
  bool get_use_stratified_sampling() const 
      { return old_remesh.get_use_stratified_sampling(); }
  void set_use_stratified_sampling(bool value)
      { old_remesh.set_use_stratified_sampling(value); }

  double get_sum_theta() const { return old_remesh.get_sum_theta(); }
  void set_sum_theta(double value) { old_remesh.set_sum_theta(value); }
  double get_sum_delta() const { return old_remesh.get_sum_delta(); }
  void set_sum_delta(double value) { old_remesh.set_sum_delta(value); }
  double get_dihedral_theta() const 
      { return old_remesh.get_dihedral_theta(); }
  void set_dihedral_theta(double value) 
      { old_remesh.set_dihedral_theta(value); }
  double get_dihedral_delta() const 
      { return old_remesh.get_dihedral_delta(); }
  void set_dihedral_delta(double value)
      { old_remesh.set_dihedral_delta(value); }
  double get_feature_difference_delta() const 
      { return old_remesh.get_feature_difference_delta(); }
  void set_feature_difference_delta(double value)
      { old_remesh.set_feature_difference_delta(value); }
  double get_feature_control_delta() const 
      { return old_remesh.get_feature_control_delta(); }
  void set_feature_control_delta(double value)
      { old_remesh.set_feature_control_delta(value); }
  bool get_inherit_element_types() const
      { return old_remesh.get_inherit_element_types(); }
  void set_inherit_element_types(bool value)
      { old_remesh.set_inherit_element_types(value); }
  bool get_use_feature_intensity_weights() const 
      { return old_remesh.get_use_feature_intensity_weights(); }
  void set_use_feature_intensity_weights(bool value)
      { old_remesh.set_use_feature_intensity_weights(value); }

  int get_vertex_optimize_count() const
      { return old_remesh.get_vertex_optimize_count(); }
  void set_vertex_optimize_count(int value) 
      { old_remesh.set_vertex_optimize_count(value); }
  double get_vertex_optimize_ratio() const 
      { return old_remesh.get_vertex_optimize_ratio(); }
  void set_vertex_optimize_ratio(double value) 
      { old_remesh.set_vertex_optimize_ratio(value); }
  int get_stencil_ring_size() const 
      { return old_remesh.get_stencil_ring_size(); }
  void set_stencil_ring_size(int value)
      { old_remesh.set_stencil_ring_size(value); }
  OptimizeStrategy get_optimize_strategy() const 
      { return old_remesh.get_optimize_strategy(); }
  void set_optimize_strategy(OptimizeStrategy value)
      { old_remesh.set_optimize_strategy(value); }
  OptimizeType get_facet_optimize_type() const
      { return old_remesh.get_facet_optimize_type(); }
  void set_facet_optimize_type(OptimizeType value) 
      { old_remesh.set_facet_optimize_type(value); }
  OptimizeType get_edge_optimize_type() const 
      { return old_remesh.get_edge_optimize_type(); }
  void set_edge_optimize_type(OptimizeType value) 
      { old_remesh.set_edge_optimize_type(value); }
  OptimizeType get_vertex_optimize_type() const
      { return old_remesh.get_vertex_optimize_type(); }
  void set_vertex_optimize_type(OptimizeType value)
      { old_remesh.set_vertex_optimize_type(value); }
  bool get_optimize_after_local_operations() const
      { return old_remesh.get_optimize_after_local_operations(); }
  void set_optimize_after_local_operations(bool value)
      { old_remesh.set_optimize_after_local_operations(value); }

  bool get_link_initialized() const { return m_links_initialized; }
  int get_optimize_type_index(OptimizeType ot) const;
  OptimizeType get_optimize_type(int index) const;

  // 4) toggle view option
  void toggle_view_input();
  void toggle_view_remesh();
  void toggle_view_input_remesh();
  void toggle_view_minimal_angle();              // the minimal angle
  void toggle_view_polyhedron_edges();
  void toggle_view_polyhedron_facets();
  void toggle_view_facet_errors();
  void toggle_view_interpolated_feature_intensities();
  void toggle_view_element_classifications();
  void toggle_view_gaussian_curvatures();
  void toggle_view_maximal_normal_dihedrals();
  void toggle_view_normal_dihedrals();
  void toggle_view_facet_in_start_points();           // facet in links
  void toggle_view_facet_in_end_points();
  void toggle_view_facet_in_links();
  void toggle_view_facet_out_start_points();          // facet out links
  void toggle_view_facet_out_end_points();
  void toggle_view_facet_out_links();
  void toggle_view_edge_in_start_points();            // edge in links
  void toggle_view_edge_in_end_points();
  void toggle_view_edge_in_links();
  void toggle_view_edge_out_start_points();           // edge out links
  void toggle_view_edge_out_end_points();
  void toggle_view_edge_out_links();
  void toggle_view_vertex_in_start_points();          // vertex in links
  void toggle_view_vertex_in_end_points();
  void toggle_view_vertex_in_links();
  void toggle_view_vertex_out_start_points();         // vertex out links
  void toggle_view_vertex_out_end_points();
  void toggle_view_vertex_out_links();
  void toggle_view_all_sample_feature_intensities();  // all samples
  void toggle_view_all_sample_capacities();
  void toggle_view_all_sample_weights();
  void toggle_view_vertex_feature_intensities();      // vertex samples
  void toggle_view_vertex_capacities();
  void toggle_view_vertex_weights();
  void toggle_view_edge_feature_intensities();        // edge samples
  void toggle_view_edge_capacities();
  void toggle_view_edge_weights();
  void toggle_view_facet_feature_intensities();       // facet samples
  void toggle_view_facet_capacities();
  void toggle_view_facet_weights();

  // 5) menu functions
  void eliminate_degenerations();              // input menu
  void split_input_long_edges();
  void input_properties();
  void split_border();                         // isotropic remeshing menu
  void isotropic_remeshing();
  void old_remesh_reset_from_input();     // min angle remeshing menu
  void old_remesh_generate_links_and_types();
  void old_remesh_properties();
  void old_remeshing();                           
  void old_remesh_initial_mesh_simplification();
  void old_remesh_split_local_longest_edge();  
  void old_remesh_increase_minimal_angle();
  void old_remesh_maximize_minimal_angle();
  void old_remesh_final_vertex_relocation();

  // 6) operations (may need update)
  void update_feature_intensities_and_clear_links();
  double calculate_average_length(const Mesh &mesh) const;

 private:
   // 1) for renderring
  void compile_shaders();
  void attrib_buffers(CGAL::QGLViewer*);
  void initialize_buffers();                      // initialize buffers
  void changed();                                 // compute elements
  void compute_elements();
  void set_draw_render_types(DrawType draw_type, RenderType render_type);
  void reset_draw_render_types();
  // 2) opeartions
  void reset();         // reset from input
  void generate();      // generate links

 private:
  // 1) general data
  // 1.1�� rendeing data
  ManipulatedFrame *m_frame;
  QOpenGLFunctions_2_1 *gl;
  bool gl_init;
  Bbox m_bbox;
  bool are_buffers_initialized;
  // 1.2) member data
  Mesh *m_pSurfacemeshInput, *m_pSurfacemeshRemesh;       // Surface_mesh
  Polyhedron *m_pPolyhedronInput, *m_pPolyhedronRemesh;   // Polyhedron_3
  Old_remesh<Kernel, Polyhedron> old_remesh;
  Facet_tree m_input_facet_tree, m_remesh_facet_tree;

  // test
  //PMP::Minangle_remesh<Kernel> m_minangle_remesh;
  PMP::internal::Minangle_remesher<Kernel> *m_minangle_remesher;
  //PMP::internal::Mesh_properties<Kernel> m_input_properties, m_remesh_properties;

  // 1.3) parameter settings
  double m_target_edge_length;
  int m_smooth_iteration_count;

  // 2) Shaders elements
  enum VAOs {
    ka_SInput_faces = 0,
    ka_SInput_edges,
    ka_PInput_faces,               // Polyhedron_3 input facets or cells
    ka_PInput_boundaries,          //       cell boundaries
    ka_PInput_samples,             //       voronoi samples
    ka_PInput_normal_edges,        //       edges
    ka_PInput_special_edges,       //       minimal radian edges or crease edges
    ka_SRemesh_faces,
    ka_SRemesh_edges,
    ka_PRemesh_faces,              // Polyhedron_3 remesh
    ka_PRemesh_boundaries,
    ka_PRemesh_samples,
    ka_PRemesh_normal_edges,
    ka_PRemesh_special_edges,
    ka_Facet_in_start,             // facet in links
    ka_Facet_in_end,
    ka_Facet_in_links,
    ka_Facet_out_start,            // facet out links
    ka_Facet_out_end,
    ka_Facet_out_links,
    ka_Edge_in_start,              // edge in links
    ka_Edge_in_end,
    ka_Edge_in_links,
    ka_Edge_out_start,             // edge out links
    ka_Edge_out_end,
    ka_Edge_out_links,
    ka_Vertex_in_start,            // vertex in links
    ka_Vertex_in_end,
    ka_Vertex_in_links,
    ka_Vertex_out_start,           // vertex out links
    ka_Vertex_out_end,
    ka_Vertex_out_links,
    ka_NbOfVaos
  };
  enum VBOs {
    kb_SInput_face_pos = 0,         // Surface_mesh input
    kb_SInput_face_normals,
    kb_SInput_face_colors,
    kb_SInput_edge_pos,
    kb_PInput_face_pos,             // Polyhedron_3 input
    kb_PInput_face_normals,
    kb_PInput_face_colors,
    kb_PInput_boundary_pos,
    kb_PInput_sample_pos,
    kb_PInput_normal_edge_pos,
    kb_PInput_special_edge_pos,
    kb_SRemesh_face_pos,            // Surface_mesh remesh
    kb_SRemesh_face_normals,
    kb_SRemesh_face_colors,
    kb_SRemesh_edge_pos,
    kb_PRemesh_face_pos,            // Polyhedron_3 remesh
    kb_PRemesh_face_normals,
    kb_PRemesh_face_colors,
    kb_PRemesh_boundary_pos,
    kb_PRemesh_sample_pos,
    kb_PRemesh_normal_edge_pos,
    kb_Remesh_special_edge_pos,
    kb_Facet_in_start_points,      // facet in links
    kb_Facet_in_end_points,
    kb_Facet_in_link_lines,
    kb_Facet_out_start_points,     // facet out links
    kb_Facet_out_end_points,
    kb_Facet_out_link_lines,
    kb_Edge_in_start_points,       // edge in links
    kb_Edge_in_end_points,
    kb_Edge_in_link_lines,
    kb_Edge_out_start_points,      // edge out links
    kb_Edge_out_end_points,
    kb_Edge_out_link_lines,
    kb_Vertex_in_start_points,     // vertex in links
    kb_Vertex_in_end_points,
    kb_Vertex_in_link_lines,
    kb_Vertex_out_start_points,    // vertex out links
    kb_Vertex_out_end_points,
    kb_Vertex_out_link_lines,
    kb_NbOfVbos
  };
  QOpenGLShaderProgram rendering_program, rendering_program_with_light;
  QOpenGLVertexArrayObject vao[VAOs::ka_NbOfVaos];
  QOpenGLBuffer vbo[VBOs::kb_NbOfVbos];
  // 2.1) rendering program
  int points_vertexLocation;
  int lines_vertexLocation;
  int mvpLocation;
  int fLocation;
  int colorLocation;
  // 2.2) rendering program with light
  int poly_vertexLocation_with_light;
  int normalLocation_with_light;
  int mvpLocation_with_light;
  int fLocation_with_light;
  int colorLocation_with_light;
  
  // 3) rendering variables
  bool m_view_surfacemesh_input;                  // Surface_mesh input
  bool m_view_surfacemesh_remesh;                 // Surface_mesh remesh
  bool m_view_polyhedron_input;                   // Polyhedron_3 input
  bool m_view_polyhedron_remesh;                  // Polyhedron_3 remesh
  bool m_view_facet_in_start_points;              // facet in links
  bool m_view_facet_in_end_points;
  bool m_view_facet_in_links;
  bool m_view_facet_out_start_points;             // facet out links
  bool m_view_facet_out_end_points;
  bool m_view_facet_out_links;
  bool m_view_edge_in_start_points;               // edge in links
  bool m_view_edge_in_end_points;
  bool m_view_edge_in_links;
  bool m_view_edge_out_start_points;              // edge out links
  bool m_view_edge_out_end_points;
  bool m_view_edge_out_links;
  bool m_view_vertex_in_start_points;             // vertex in links
  bool m_view_vertex_in_end_points;
  bool m_view_vertex_in_links;
  bool m_view_vertex_out_start_points;            // vertex out links
  bool m_view_vertex_out_end_points;
  bool m_view_vertex_out_links;
  bool m_view_minimal_angle;
  bool m_view_polyhedron_edges;
  DrawType m_draw_type;                           // draw options
  RenderType m_render_type;
  std::vector<float> pos_sinput_faces;            // Surface_mesh input
  std::vector<float> pos_sinput_face_normals;
  std::vector<float> pos_sinput_face_colors;
  std::vector<float> pos_sinput_edges;
  std::vector<float> pos_pinput_faces;            // Polyhedron_3 input
  std::vector<float> pos_pinput_face_normals;
  std::vector<float> pos_pinput_face_colors;
  std::vector<float> pos_pinput_boundaries;
  std::vector<float> pos_pinput_samples;
  std::vector<float> pos_pinput_normal_edges;
  std::vector<float> pos_pinput_special_edges;
  std::vector<float> pos_sremesh_faces;           // Surface_mesh remesh
  std::vector<float> pos_sremesh_face_normals;
  std::vector<float> pos_sremesh_face_colors;
  std::vector<float> pos_sremesh_edges;
  std::vector<float> pos_premesh_faces;           // Polyhedron_3 remesh
  std::vector<float> pos_premesh_face_normals;
  std::vector<float> pos_premesh_face_colors;
  std::vector<float> pos_premesh_boundaries;
  std::vector<float> pos_premesh_samples;
  std::vector<float> pos_premesh_normal_edges;
  std::vector<float> pos_premesh_special_edges;
  std::vector<float> pos_facet_in_start_points;   // facet in links
  std::vector<float> pos_facet_in_end_points;
  std::vector<float> pos_facet_in_links;
  std::vector<float> pos_facet_out_start_points;  // facet out links
  std::vector<float> pos_facet_out_end_points;
  std::vector<float> pos_facet_out_links;
  std::vector<float> pos_edge_in_start_points;    // edge in links
  std::vector<float> pos_edge_in_end_points;
  std::vector<float> pos_edge_in_links;
  std::vector<float> pos_edge_out_start_points;   // edge out links
  std::vector<float> pos_edge_out_end_points;
  std::vector<float> pos_edge_out_links;
  std::vector<float> pos_vertex_in_start_points;  // vertx in links
  std::vector<float> pos_vertex_in_end_points;
  std::vector<float> pos_vertex_in_links;
  std::vector<float> pos_vertex_out_start_points; // vertex out links
  std::vector<float> pos_vertex_out_end_points;
  std::vector<float> pos_vertex_out_links;

  // 4) status data
  bool m_input_aabb_tree_constructed;
  bool m_links_initialized;

}; // end class Scene

#endif // CGAL_SCENE_H
