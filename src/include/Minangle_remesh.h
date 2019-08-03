#ifndef CGAL_MINANGLE_REMESH_H
#define CGAL_MINANGLE_REMESH_H

#include "internal\minangle_remeshing\minangle_remesh_impl.h"

namespace CGAL {
namespace Polygon_mesh_processing {

template<typename Kernel>
class Minangle_remesh {
public:
  // typedef typenames
  typedef typename internal::Minangle_remesher<Kernel> Minangle_remesher;
  typedef typename Minangle_remesher::FT FT;

  // life cycle
  Minangle_remesh() {
  }

  Minangle_remesh(FT max_error_threshold,   // general parameters
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
    int samples_per_facet_in,               // sample parameters
    int samples_per_facet_out,
    int max_samples_per_area,
    int min_samples_per_triangle,
    int bvd_iteration_count,
    SampleNumberStrategy sample_number_strategy,
    SampleStrategy sample_strategy,
    bool use_stratified_sampling,
    FT sum_theta,                           // feature intensity parameters
    FT sum_delta,
    FT dihedral_theta,
    FT dihedral_delta,
    FT feature_difference_delta,
    FT feature_control_delta,
    bool inherit_element_types,
    bool use_feature_intensity_weights,
    int vertex_optimize_count,              // vertex optimization parameters
    FT vertex_optimize_ratio,
    int stencil_ring_size,
    OptimizeStrategy optimize_strategy,
    OptimizeType facet_optimize_type,
    OptimizeType edge_optimize_type,
    OptimizeType vertex_optimize_type,
    bool optimize_after_local_operations) {
    // general parameters
    minangle_remesher_.set_max_error_threshold(max_error_threshold);
    minangle_remesher_.set_min_angle_threshold(min_angle_threshold);
    minangle_remesher_.set_max_mesh_complexity(max_mesh_complexity);
    minangle_remesher_.set_smooth_angle_delta(smooth_angle_delta);
    minangle_remesher_.set_apply_edge_flip(apply_edge_flip);
    minangle_remesher_.set_edge_flip_strategy(edge_flip_strategy);
    minangle_remesher_.set_flip_after_split_and_collapse(flip_after_split_and_collapse);
    minangle_remesher_.set_relocate_after_local_operations(relocate_after_local_operations);
    minangle_remesher_.set_relocate_strategy(relocate_strategy);
    minangle_remesher_.set_keep_vertex_in_one_ring(keep_vertex_in_one_ring);
    minangle_remesher_.set_use_local_aabb_tree(use_local_aabb_tree);
    minangle_remesher_.set_collapsed_list_size(collapse_list_size);
    minangle_remesher_.set_decrease_max_errors(decrease_max_errors);
    minangle_remesher_.set_verbose_progress(verbose_progress);
    minangle_remesher_.set_apply_initial_mesh_simplification(apply_initial_mesh_simplification);
    minangle_remesher_.set_apply_final_vertex_relocation(apply_final_vertex_relocation);
    // sample parameters
    minangle_remesher_.set_samples_per_facet_in(samples_per_facet_in);
    minangle_remesher_.set_samples_per_facet_out(samples_per_facet_out);
    minangle_remesher_.set_max_samples_per_area(max_samples_per_area);
    minangle_remesher_.set_min_samples_per_triangle(min_samples_per_triangle);
    minangle_remesher_.set_bvd_iteration_count(bvd_iteration_count);
    minangle_remesher_.set_sample_number_strategy(sample_number_strategy);
    minangle_remesher_.set_sample_strategy(sample_strategy);
    minangle_remesher_.set_use_stratified_sampling(use_stratified_sampling);
    // feature intensity parameters
    minangle_remesher_.set_sum_theta(sum_theta);
    minangle_remesher_.set_sum_delta(sum_delta);
    minangle_remesher_.set_dihedral_theta(dihedral_theta);
    minangle_remesher_.set_dihedral_delta(dihedral_delta);
    minangle_remesher_.set_feature_difference_delta(feature_difference_delta);
    minangle_remesher_.set_feature_control_delta(feature_control_delta);
    minangle_remesher_.set_inherit_element_types(inherit_element_types);
    minangle_remesher_.set_use_feature_intensity_weights(use_feature_intensity_weights);
      // vertex optimization parameters
    minangle_remesher_.set_vertex_optimize_count(vertex_optimize_count);
    minangle_remesher_.set_vertex_optimize_ratio(vertex_optimize_ratio);
    minangle_remesher_.set_stencil_ring_size(stencil_ring_size);
    minangle_remesher_.set_optimize_strategy(optimize_strategy);
    minangle_remesher_.set_facet_optimize_type(facet_optimize_type);
    minangle_remesher_.set_edge_optimize_type(edge_optimize_type);
    minangle_remesher_.set_vertex_optimize_type(vertex_optimize_type);
    minangle_remesher_.set_optimize_after_local_operations(optimize_after_local_operations);
  }

  virtual ~Minangle_remesh() {}

private:
  Minangle_remesher minangle_remesher_;
};

}
}

#endif // CGAL_MINANGLE_REMESH_H
