#ifndef CGAL_MINANGLE_REMESH_IMPL_H
#define CGAL_MINANGLE_REMESH_IMPL_H

#include "enriched_surface_mesh.h"

// enumeration data (put outside the namespace so that others can use)
enum SampleNumberStrategy {
  k_fixed = 0,  // #samples per facet is roughtly fixed (with respect to the sample strategy)
  k_variable    // #samples per facet is variable with respect to size_of_facets()
};

enum SampleStrategy {
  k_uniform = 0,  // #samples per facet is proportional to its area
  k_adaptive      // #samples per facet is roughly the same
};

enum OptimizeType {
  k_none = 0,
  k_input_to_remesh,
  k_remesh_to_input,
  k_both
};

enum OptimizeStrategy {
  k_approximation = 0,
  k_Interpolation
};

enum EdgeFlipStrategy {
  k_improve_valence = 0,
  k_improve_angle
};

enum RelocateStrategy {
  k_barycenter = 0,
  k_cvt_barycenter
};

enum VertexType {
  k_feature_vertex = 0,
  k_crease_vertex,
  k_smooth_vertex
};

namespace CGAL {
namespace Polygon_mesh_processing {
namespace internal {

template<typename Kernel>
class Minangle_remesher {
public:
  // types
  typedef typename Surface_mesh_properties<Kernel> Surface_mesh_properties;
  typedef typename Surface_mesh_properties::FT FT;
  typedef typename Surface_mesh_properties::Point Point;
  typedef typename Surface_mesh_properties::Surface_mesh Surface_mesh;
  typedef typename Surface_mesh_properties::Point_comp Point_comp;

  // visit list and iterator
  typedef typename std::list<std::pair<Point, FT>> Visit_list;
  typedef typename std::list<std::pair<Point, FT>>::iterator Visit_iter;

public:
  // 1) life cycles
  Minangle_remesher() {
    // general paramters
    m_max_error_threshold = 0.2;          
    m_min_angle_threshold = 30.0;
    m_max_mesh_complexity = 100000000;
    m_smooth_angle_delta = 0.1;
    m_apply_edge_flip = true;
    m_edge_flip_strategy = EdgeFlipStrategy::k_improve_angle;
    m_flip_after_split_and_collapse = true;
    m_relocate_after_local_operations = true;
    m_relocate_strategy = RelocateStrategy::k_cvt_barycenter;
    m_keep_vertex_in_one_ring = false;
    m_use_local_aabb_tree = true;
    m_collapsed_list_size = 10;
    m_decrease_max_errors = true;
    m_verbose_progress = true;
    m_apply_initial_mesh_simplification = true;
    m_apply_final_vertex_relocation = true;
    // sample parameters
    m_samples_per_facet_in = 10;          
    m_samples_per_facet_out = 10;
    m_max_samples_per_area = 10000;
    m_min_samples_per_triangle = 1;
    m_bvd_iteration_count = 1;
    m_sample_number_strategy = SampleNumberStrategy::k_fixed;
    m_sample_strategy = SampleStrategy::k_adaptive;
    m_use_stratified_sampling = false;
    // feature parameters
    m_sum_theta = 1.0;                    
    m_sum_delta = 0.5;
    m_dihedral_theta = 1.0;
    m_dihedral_delta = 0.5;
    m_feature_difference_delta = 0.15;
    m_feature_control_delta = 0.5;
    m_inherit_element_types = false;
    m_use_feature_intensity_weights = false;
    // vertex optimization parameters
    m_vertex_optimize_count = 2;          
    m_vertex_optimize_ratio = 0.9;
    m_stencil_ring_size = 1;
    m_optimize_strategy = OptimizeStrategy::k_approximation;
    m_facet_optimize_type = OptimizeType::k_both;
    m_edge_optimize_type = OptimizeType::k_both;
    m_vertex_optimize_type = OptimizeType::k_both;
    m_optimize_after_local_operations = true;

    initialize_private_data();
  }

  Minangle_remesher(FT max_error_threshold,  // general parameters
    FT min_angle_threshold,
    int max_mesh_complexity,
    FT smooth_angle_delta,
    bool apply_edge_flip,
    EdgeFlipStrategy edge_flip_strategy,
    bool flip_after_split_and_collapse,
    bool relocate_after_local_operations,
    RelocateStrategy relocate_strategy,
    bool keep_vertex_in_one_ring,
    bool use_local_aabb_tree,
    int collapse_list_size,
    bool decrease_max_errors,
    bool verbose_progress,
    bool apply_initial_mesh_simplification,
    bool apply_final_vertex_relocation,
    int samples_per_facet_in,         // sample parameters
    int samples_per_facet_out,
    int max_samples_per_area,
    int min_samples_per_triangle,
    int bvd_iteration_count,
    SampleNumberStrategy sample_number_strategy,
    SampleStrategy sample_strategy,
    bool use_stratified_sampling,
    FT sum_theta,                 // feature intensity parameters
    FT sum_delta,
    FT dihedral_theta,
    FT dihedral_delta,
    FT feature_difference_delta,
    FT feature_control_delta,
    bool inherit_element_types,
    bool use_feature_intensity_weights,
    int vertex_optimize_count,      // vertex optimization parameters
    FT vertex_optimize_ratio,
    int stencil_ring_size,
    OptimizeStrategy optimize_strategy,
    OptimizeType facet_optimize_type,
    OptimizeType edge_optimize_type,
    OptimizeType vertex_optimize_type,
    bool optimize_after_local_operations) 
  : m_max_error_threshold(max_error_threshold),
    m_min_angle_threshold(min_angle_threshold),
    m_max_mesh_complexity(max_mesh_complexity),
    m_smooth_angle_delta(smooth_angle_delta),
    m_apply_edge_flip(apply_edge_flip),
    m_edge_flip_strategy(edge_flip_strategy),
    m_flip_after_split_and_collapse(flip_after_split_and_collapse),
    m_relocate_after_local_operations(relocate_after_local_operations),
    m_relocate_strategy(relocate_strategy),
    m_keep_vertex_in_one_ring(keep_vertex_in_one_ring),
    m_use_local_aabb_tree(use_local_aabb_tree),
    m_collapse_list_size(collapse_list_size),
    m_decrease_max_errors(decrease_max_errors),
    m_verbose_progress(verbose_progress),
    m_apply_initial_mesh_simplificaiton(apply_initial_mesh_simplification),
    m_apply_final_vertex_relocation(apply_final_vertex_relocation),
    m_samples_per_facet_in(samples_per_facet_in),
    m_samples_per_facet_out(samples_per_facet_out),
    m_max_samples_per_area(max_samples_per_area),
    m_min_samples_per_triangle(min_samples_per_triangle),
    m_bvd_iteration_count(bvd_iteration_count),
    m_sample_number_strategy(sample_number_strategy),
    m_sample_strategy(sample_strategy),
    m_use_stratified_sampling(use_stratified_sampling),
    m_sum_theta(sum_theta),
    m_sum_delta(sum_delta),
    m_dihedral_theta(dihedral_theta),
    m_dihedral_delta(dihedral_delta),
    m_feature_difference_delta(feature_difference_delta),
    m_feature_control_delta(feature_control_delta),
    m_inherit_element_types(inherit_element_types),
    m_use_feature_intensity_weights(use_feature_intensity_weights),
    m_vertex_optimize_count(vertex_optimize_count),
    m_vertex_optimize_ratio(vertex_optimize_ratio),
    m_stencil_ring_size(stencil_ring_size),
    m_optimize_strategy(optimize_strategy),
    m_facet_optimize_type(facet_optimize_type),
    m_edge_optimize_type(edge_optimize_type),
    m_vertex_optimize_type(vertex_optimize_type),
    m_optimize_after_local_operations(optimize_after_local_operations) {
    initialize_private_data();
  }

  virtual ~Minangle_remesher() {}

  // parameter access functions
  // general parameters
  FT get_max_error_threshold() const { return m_max_error_threshold; }
  void set_max_error_threshold(FT value) { m_max_error_threshold = value; }
  FT get_min_angle_threshold() const { return m_min_angle_threshold; }
  void set_min_angle_threshold(FT value) { m_min_angle_threshold = value; }
  int get_max_mesh_complexity() const { return m_max_mesh_complexity; }
  void set_max_mesh_complexity(int value) { m_max_mesh_complexity = value; }
  FT get_smooth_angle_delta() const { return m_smooth_angle_delta; }
  void set_smooth_angle_delta(FT value) { m_smooth_angle_delta = value; }
  bool get_apply_edge_flip() const { return m_apply_edge_flip; }
  void set_apply_edge_flip(bool value) { m_apply_edge_flip = value; }
  EdgeFlipStrategy get_edge_flip_strategy() const { return m_edge_flip_strategy; }
  void set_edge_flip_strategy(EdgeFlipStrategy value) { m_edge_flip_strategy = value; }
  bool get_flip_after_split_and_collapse() const { return m_flip_after_split_and_collapse; }
  void set_flip_after_split_and_collapse(bool value) { m_flip_after_split_and_collapse = value; }
  bool get_relocate_after_local_operations() const { return m_relocate_after_local_operations; }
  void set_relocate_after_local_operations(bool value) { m_relocate_after_local_operations = value; }
  RelocateStrategy get_relocate_strategy() const { return m_relocate_strategy; }
  void set_relocate_strategy(RelocateStrategy value) { m_relocate_strategy = value; }
  bool get_keep_vertex_in_one_ring() const { return m_keep_vertex_in_one_ring; }
  void set_keep_vertex_in_one_ring(bool value) { m_keep_vertex_in_one_ring = value; }
  bool get_use_local_aabb_tree() const { return m_use_local_aabb_tree; }
  void set_use_local_aabb_tree(bool value) { m_use_local_aabb_tree = value; }
  int get_collapsed_list_size() const { return m_collapsed_list_size; }
  void set_collapsed_list_size(int value) { m_collapsed_list_size = value; }
  bool get_decrease_max_errors() const { return m_decrease_max_errors; }
  void set_decrease_max_errors(bool value) { m_decrease_max_errors = value; }
  bool get_verbose_progress() const { return m_verbose_progress; }
  void set_verbose_progress(bool value) { m_verbose_progress = value; }
  bool get_apply_initial_mesh_simplification() const { return m_apply_initial_mesh_simplification; }
  void set_apply_initial_mesh_simplification(bool value) { m_apply_initial_mesh_simplification = value; }
  bool get_apply_final_vertex_relocation() const { return m_apply_final_vertex_relocation; }
  void set_apply_final_vertex_relocation(bool value) { m_apply_final_vertex_relocation = value; }
  // sample parameters
  int get_samples_per_facet_in() const { return m_samples_per_facet_in; }
  void set_samples_per_facet_in(int value) { m_samples_per_facet_in = value; }
  int get_samples_per_facet_out() const { return m_samples_per_facet_out; }
  void set_samples_per_facet_out(int value) { m_samples_per_facet_out = value; }
  int get_max_samples_per_area() const { return m_max_samples_per_area; }
  void set_max_samples_per_area(int value) { m_max_samples_per_area = value; }
  int get_min_samples_per_triangle() const { return m_min_samples_per_triangle; }
  void set_min_samples_per_triangle(int value) { m_min_samples_per_triangle = value; }
  int get_bvd_iteration_count() const { return m_bvd_iteration_count; }
  void set_bvd_iteration_count(int value) { m_bvd_iteration_count = value; }
  SampleNumberStrategy get_sample_number_strategy() const { return m_sample_number_strategy; }
  void set_sample_number_strategy(SampleNumberStrategy value) { m_sample_number_strategy = value; }
  SampleStrategy get_sample_strategy() const { return m_sample_strategy; }
  void set_sample_strategy(SampleStrategy value) { m_sample_strategy = value; }
  bool get_use_stratified_sampling() const { return m_use_stratified_sampling; }
  void set_use_stratified_sampling(bool value) { m_use_stratified_sampling = value; }
  // feature intensity parameters
  FT get_sum_theta() const { return m_sum_theta; }
  void set_sum_theta(FT value) { m_sum_theta = value; }
  FT get_sum_delta() const { return m_sum_delta; }
  void set_sum_delta(FT value) { m_sum_delta = value; }
  FT get_dihedral_theta() const { return m_dihedral_theta; }
  void set_dihedral_theta(FT value) { m_dihedral_theta = value; }
  FT get_dihedral_delta() const { return m_dihedral_delta; }
  void set_dihedral_delta(FT value) { m_dihedral_delta = value; }
  FT get_feature_difference_delta() const { return m_feature_difference_delta; }
  void set_feature_difference_delta(FT value) { m_feature_difference_delta = value; }
  FT get_feature_control_delta() const { return m_feature_control_delta; }
  void set_feature_control_delta(FT value) { m_feature_control_delta = value; }
  bool get_inherit_element_types() const { return m_inherit_element_types; }
  void set_inherit_element_types(bool value) { m_inherit_element_types = value; }
  bool get_use_feature_intensity_weights() const { return m_use_feature_intensity_weights; }
  void set_use_feature_intensity_weights(bool value) { m_use_feature_intensity_weights = value; }
  // vertex optimization parameters
  int get_vertex_optimize_count() const { return m_vertex_optimize_count; }
  void set_vertex_optimize_count(int value) { m_vertex_optimize_count = value; }
  FT get_vertex_optimize_ratio() const { return m_vertex_optimize_ratio; }
  void set_vertex_optimize_ratio(FT value) { m_vertex_optimize_ratio = value; }
  int get_stencil_ring_size() const { return m_stencil_ring_size; }
  void set_stencil_ring_size(int value) { m_stencil_ring_size = value; }
  OptimizeStrategy get_optimize_strategy() const{ return m_optimize_strategy;}
  void set_optimize_strategy(OptimizeStrategy value) { m_optimize_strategy = value; }
  OptimizeType get_facet_optimize_type() const { return m_facet_optimize_type; }
  void set_facet_optimize_type(OptimizeType value) { m_facet_optimize_type = value; }
  OptimizeType get_edge_optimize_type() const { return m_edge_optimize_type; }
  void set_edge_optimize_type(OptimizeType value) { m_edge_optimize_type = value; }
  OptimizeType get_vertex_optimize_type() const { return m_vertex_optimize_type; }
  void set_vertex_optimize_type(OptimizeType value) { m_vertex_optimize_type = value; }
  bool get_optimize_after_local_operations() const { return m_optimize_after_local_operations; }
  void set_optimize_after_local_operations(bool value) { m_optimize_after_local_operations = value; }

private:
  // initializaiton
  void initialize_private_data() {
    m_collapsed_list.clear();
    m_collapsed_map.clear();
  }

private:
  // 1) parameters
  FT m_max_error_threshold;                 // general parameters
  FT m_min_angle_threshold;
  int m_max_mesh_complexity;
  FT m_smooth_angle_delta;
  bool m_apply_edge_flip;
  EdgeFlipStrategy m_edge_flip_strategy;
  bool m_flip_after_split_and_collapse;
  bool m_relocate_after_local_operations;
  RelocateStrategy m_relocate_strategy;
  bool m_keep_vertex_in_one_ring;
  bool m_use_local_aabb_tree;
  int m_collapsed_list_size;
  bool m_decrease_max_errors;
  bool m_verbose_progress;
  bool m_apply_initial_mesh_simplification;
  bool m_apply_final_vertex_relocation;
  int m_samples_per_facet_in;               // sample parameters
  int m_samples_per_facet_out;
  int m_max_samples_per_area;
  int m_min_samples_per_triangle;
  int m_bvd_iteration_count;
  SampleNumberStrategy m_sample_number_strategy;
  SampleStrategy m_sample_strategy;
  bool m_use_stratified_sampling;
  FT m_sum_theta;                           // feature function parameters
  FT m_sum_delta;
  FT m_dihedral_theta;
  FT m_dihedral_delta;
  FT m_feature_difference_delta;
  FT m_feature_control_delta;
  bool m_inherit_element_types;
  bool m_use_feature_intensity_weights;
  int m_vertex_optimize_count;              // vertex relocate_parameters
  FT m_vertex_optimize_ratio;
  int m_stencil_ring_size;
  OptimizeStrategy m_optimize_strategy;
  OptimizeType m_facet_optimize_type;
  OptimizeType m_edge_optimize_type;
  OptimizeType m_vertex_optimize_type;
  bool m_optimize_after_local_operations;

  // 2) the collapse operator
  Visit_list m_collapsed_list;
  std::map<Point, std::map<FT, Visit_iter>, Point_comp> m_collapsed_map;

  // 3) member data and properties
  Surface_mesh_properties m_Input_properties, m_Remesh_properties;
};

}
}
}

#endif // CGAL_MINANGLE_REMESH_IMPL_H
