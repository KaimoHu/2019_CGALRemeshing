#ifndef CGAL_MINANGLE_REMESH_IMPL_H
#define CGAL_MINANGLE_REMESH_IMPL_H

#include "mesh_properties.h"

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
  // type definitions
  typedef typename Mesh_properties<Kernel> Mesh_properties;
  typedef typename Mesh_properties::FT FT;
  typedef typename Mesh_properties::Point Point;
  typedef typename Mesh_properties::Mesh Mesh;
  typedef typename Mesh_properties::halfedge_descriptor halfedge_descriptor;
  typedef typename Mesh_properties::edge_descriptor edge_descriptor;
  typedef typename Mesh_properties::vertex_descriptor vertex_descriptor;
  typedef typename Mesh_properties::face_descriptor face_descriptor;

  typedef typename Mesh_properties::Point_comp Point_comp;
  typedef typename std::list<std::pair<Point, FT>> Visit_list;  // visit list and iterator
  typedef typename std::list<std::pair<Point, FT>>::iterator Visit_iter;

public:
  // 1) life cycles
  Minangle_remesher(Mesh &input, Mesh &remesh) {
    // general paramters
    max_error_threshold_ = 0.2;          
    min_angle_threshold_ = 30.0;
    max_mesh_complexity_ = 100000000;
    smooth_angle_delta_ = 0.1;
    apply_edge_flip_ = true;
    edge_flip_strategy_ = EdgeFlipStrategy::k_improve_angle;
    flip_after_split_and_collapse_ = true;
    relocate_after_local_operations_ = true;
    relocate_strategy_ = RelocateStrategy::k_cvt_barycenter;
    keep_vertex_in_one_ring_ = false;
    use_local_aabb_tree_ = true;
    collapse_list_size_ = 10;
    decrease_max_errors_ = true;
    verbose_progress_ = true;
    apply_initial_mesh_simplification_ = true;
    apply_final_vertex_relocation_ = true;
    // sample parameters
    samples_per_facet_in_ = 10;          
    samples_per_facet_out_ = 10;
    max_samples_per_area_ = 10000;
    min_samples_per_triangle_ = 1;
    bvd_iteration_count_ = 1;
    sample_number_strategy_ = SampleNumberStrategy::k_fixed;
    sample_strategy_ = SampleStrategy::k_adaptive;
    use_stratified_sampling_ = false;
    // feature parameters
    sum_theta_ = 1.0;                    
    sum_delta_ = 0.5;
    dihedral_theta_ = 1.0;
    dihedral_delta_ = 0.5;
    feature_difference_delta_ = 0.15;
    feature_control_delta_ = 0.5;
    inherit_element_types_ = false;
    use_feature_intensity_weights_ = false;
    // vertex optimization parameters
    vertex_optimize_count_ = 2;          
    vertex_optimize_ratio_ = 0.9;
    stencil_ring_size_ = 1;
    optimize_strategy_ = OptimizeStrategy::k_approximation;
    facet_optimize_type_ = OptimizeType::k_both;
    edge_optimize_type_ = OptimizeType::k_both;
    vertex_optimize_type_ = OptimizeType::k_both;
    optimize_after_local_operations_ = true;

    initialize_private_data();

    input_properties_ = new Mesh_properties(input);
    remesh_properties_ = new Mesh_properties(remesh);
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
    bool optimize_after_local_operations,
    Mesh &input, 
    Mesh &remesh)
  : max_error_threshold_(max_error_threshold),
    min_angle_threshold_(min_angle_threshold),
    max_mesh_complexity_(max_mesh_complexity),
    smooth_angle_delta_(smooth_angle_delta),
    apply_edge_flip_(apply_edge_flip),
    edge_flip_strategy_(edge_flip_strategy),
    flip_after_split_and_collapse_(flip_after_split_and_collapse),
    relocate_after_local_operations_(relocate_after_local_operations),
    relocate_strategy_(relocate_strategy),
    keep_vertex_in_one_ring_(keep_vertex_in_one_ring),
    use_local_aabb_tree_(use_local_aabb_tree),
    collapse_list_size_(collapse_list_size),
    decrease_max_errors_(decrease_max_errors),
    verbose_progress_(verbose_progress),
    apply_initial_mesh_simplification_(apply_initial_mesh_simplification),
    apply_final_vertex_relocation_(apply_final_vertex_relocation),
    samples_per_facet_in_(samples_per_facet_in),
    samples_per_facet_out_(samples_per_facet_out),
    max_samples_per_area_(max_samples_per_area),
    min_samples_per_triangle_(min_samples_per_triangle),
    bvd_iteration_count_(bvd_iteration_count),
    sample_number_strategy_(sample_number_strategy),
    sample_strategy_(sample_strategy),
    use_stratified_sampling_(use_stratified_sampling),
    sum_theta_(sum_theta),
    sum_delta_(sum_delta),
    dihedral_theta_(dihedral_theta),
    dihedral_delta_(dihedral_delta),
    feature_difference_delta_(feature_difference_delta),
    feature_control_delta_(feature_control_delta),
    inherit_element_types_(inherit_element_types),
    use_feature_intensity_weights_(use_feature_intensity_weights),
    vertex_optimize_count_(vertex_optimize_count),
    vertex_optimize_ratio_(vertex_optimize_ratio),
    stencil_ring_size_(stencil_ring_size),
    optimize_strategy_(optimize_strategy),
    facet_optimize_type_(facet_optimize_type),
    edge_optimize_type_(edge_optimize_type),
    vertex_optimize_type_(vertex_optimize_type),
    optimize_after_local_operations_(optimize_after_local_operations) {
    initialize_private_data();
    input_properties_ = new Mesh_properties(input);
    remesh_properties_ = new Mesh_properties(remesh);
  }

  virtual ~Minangle_remesher() {
    delete input_properties_;
    delete remesh_properties_;
  }

  // 2) parameter access functions
  // 2.1) general parameters
  FT get_max_error_threshold() const { return max_error_threshold_; }
  void set_max_error_threshold(FT value) { max_error_threshold_ = value; }
  FT get_min_angle_threshold() const { return min_angle_threshold_; }
  void set_min_angle_threshold(FT value) { min_angle_threshold_ = value; }
  int get_max_mesh_complexity() const { return max_mesh_complexity_; }
  void set_max_mesh_complexity(int value) { max_mesh_complexity_ = value; }
  FT get_smooth_angle_delta() const { return smooth_angle_delta_; }
  void set_smooth_angle_delta(FT value) { smooth_angle_delta_ = value; }
  bool get_apply_edge_flip() const { return apply_edge_flip_; }
  void set_apply_edge_flip(bool value) { apply_edge_flip_ = value; }
  EdgeFlipStrategy get_edge_flip_strategy() const { return edge_flip_strategy_; }
  void set_edge_flip_strategy(EdgeFlipStrategy value) { edge_flip_strategy_ = value; }
  bool get_flip_after_split_and_collapse() const { return flip_after_split_and_collapse_; }
  void set_flip_after_split_and_collapse(bool value) { flip_after_split_and_collapse_ = value; }
  bool get_relocate_after_local_operations() const { return relocate_after_local_operations_; }
  void set_relocate_after_local_operations(bool value) { relocate_after_local_operations_ = value; }
  RelocateStrategy get_relocate_strategy() const { return relocate_strategy_; }
  void set_relocate_strategy(RelocateStrategy value) { relocate_strategy_ = value; }
  bool get_keep_vertex_in_one_ring() const { return keep_vertex_in_one_ring_; }
  void set_keep_vertex_in_one_ring(bool value) { keep_vertex_in_one_ring_ = value; }
  bool get_use_local_aabb_tree() const { return use_local_aabb_tree_; }
  void set_use_local_aabb_tree(bool value) { use_local_aabb_tree_ = value; }
  int get_collapsed_list_size() const { return collapse_list_size_; }
  void set_collapsed_list_size(int value) { collapse_list_size_ = value; }
  bool get_decrease_max_errors() const { return decrease_max_errors_; }
  void set_decrease_max_errors(bool value) { decrease_max_errors_ = value; }
  bool get_verbose_progress() const { return verbose_progress_; }
  void set_verbose_progress(bool value) { verbose_progress_ = value; }
  bool get_apply_initial_mesh_simplification() const { return apply_initial_mesh_simplification_; }
  void set_apply_initial_mesh_simplification(bool value) { apply_initial_mesh_simplification_ = value; }
  bool get_apply_final_vertex_relocation() const { return apply_final_vertex_relocation_; }
  void set_apply_final_vertex_relocation(bool value) { apply_final_vertex_relocation_ = value; }
  // 2.2) sample parameters
  int get_samples_per_facet_in() const { return samples_per_facet_in_; }
  void set_samples_per_facet_in(int value) { samples_per_facet_in_ = value; }
  int get_samples_per_facet_out() const { return samples_per_facet_out_; }
  void set_samples_per_facet_out(int value) { samples_per_facet_out_ = value; }
  int get_max_samples_per_area() const { return max_samples_per_area_; }
  void set_max_samples_per_area(int value) { max_samples_per_area_ = value; }
  int get_min_samples_per_triangle() const { return min_samples_per_triangle_; }
  void set_min_samples_per_triangle(int value) { min_samples_per_triangle_ = value; }
  int get_bvd_iteration_count() const { return bvd_iteration_count_; }
  void set_bvd_iteration_count(int value) { bvd_iteration_count_ = value; }
  SampleNumberStrategy get_sample_number_strategy() const { return sample_number_strategy_; }
  void set_sample_number_strategy(SampleNumberStrategy value) { sample_number_strategy_ = value; }
  SampleStrategy get_sample_strategy() const { return sample_strategy_; }
  void set_sample_strategy(SampleStrategy value) { sample_strategy_ = value; }
  bool get_use_stratified_sampling() const { return use_stratified_sampling_; }
  void set_use_stratified_sampling(bool value) { use_stratified_sampling_ = value; }
  // 2.3) feature intensity parameters
  FT get_sum_theta() const { return sum_theta_; }
  void set_sum_theta(FT value) { sum_theta_ = value; }
  FT get_sum_delta() const { return sum_delta_; }
  void set_sum_delta(FT value) { sum_delta_ = value; }
  FT get_dihedral_theta() const { return dihedral_theta_; }
  void set_dihedral_theta(FT value) { dihedral_theta_ = value; }
  FT get_dihedral_delta() const { return dihedral_delta_; }
  void set_dihedral_delta(FT value) { dihedral_delta_ = value; }
  FT get_feature_difference_delta() const { return feature_difference_delta_; }
  void set_feature_difference_delta(FT value) { feature_difference_delta_ = value; }
  FT get_feature_control_delta() const { return feature_control_delta_; }
  void set_feature_control_delta(FT value) { feature_control_delta_ = value; }
  bool get_inherit_element_types() const { return inherit_element_types_; }
  void set_inherit_element_types(bool value) { inherit_element_types_ = value; }
  bool get_use_feature_intensity_weights() const { return use_feature_intensity_weights_; }
  void set_use_feature_intensity_weights(bool value) { use_feature_intensity_weights_ = value; }
  // 2.4) vertex optimization parameters
  int get_vertex_optimize_count() const { return vertex_optimize_count_; }
  void set_vertex_optimize_count(int value) { vertex_optimize_count_ = value; }
  FT get_vertex_optimize_ratio() const { return vertex_optimize_ratio_; }
  void set_vertex_optimize_ratio(FT value) { vertex_optimize_ratio_ = value; }
  int get_stencil_ring_size() const { return stencil_ring_size_; }
  void set_stencil_ring_size(int value) { stencil_ring_size_ = value; }
  OptimizeStrategy get_optimize_strategy() const{ return optimize_strategy_;}
  void set_optimize_strategy(OptimizeStrategy value) { optimize_strategy_ = value; }
  OptimizeType get_facet_optimize_type() const { return facet_optimize_type_; }
  void set_facet_optimize_type(OptimizeType value) { facet_optimize_type_ = value; }
  OptimizeType get_edge_optimize_type() const { return edge_optimize_type_; }
  void set_edge_optimize_type(OptimizeType value) { edge_optimize_type_ = value; }
  OptimizeType get_vertex_optimize_type() const { return vertex_optimize_type_; }
  void set_vertex_optimize_type(OptimizeType value) { vertex_optimize_type_ = value; }
  bool get_optimize_after_local_operations() const { return optimize_after_local_operations_; }
  void set_optimize_after_local_operations(bool value) { optimize_after_local_operations_ = value; }

  // 3) reset surfaces
  void reset_mesh(Mesh &mesh, bool is_input) {
    if (is_input) {
      delete input_properties_;
      input_properties_ = new Mesh_properties(mesh);
    }
    else {
      delete remesh_properties_;
      remesh_properties_ = new Mesh_properties(mesh);
    }
  }

  // NOTE: the following functions are made public only for visualization
  void calculate_normals(bool calculate_input, bool calcualte_remesh) const {
    if (calculate_input) {
      input_properties_->calculate_normals();
    }
    if (calcualte_remesh) {
      remesh_properties_->calculate_normals();
    }
  }

  void compute_faces(bool is_input, std::vector<float> *pos,
      std::vector<float> *normals, std::vector<float> *colors) const {
    Color color = is_input ? Color(150, 150, 200) : Color(200, 150, 150);
    if (is_input) {
      input_properties_->compute_faces(color, pos, normals, colors);
    }
    else {
      remesh_properties_->compute_faces(color, pos, normals, colors);
    }
  }

  void compute_edges(bool is_input, std::vector<float> *pos) const {
    if (is_input) {
      input_properties_->compute_edges(pos);
    }
    else {
      remesh_properties_->compute_edges(pos);
    }
  }



  
  // test
  void test() {
    input_properties_->calculate_max_squared_errors();

    std::set<face_descriptor> faces;
    input_properties_->calculate_max_squared_errors(&faces);

    //FT max_error = 0.0;
    //halfedge_descriptor h = input_properties_->get_maximal_error(&max_error);
  }




private:
  // initializaiton
  void initialize_private_data() {
    collapsed_list_.clear();
    collapsed_map_.clear();
  }

private:
  // 1) parameters
  FT max_error_threshold_;                 // general parameters
  FT min_angle_threshold_;
  int max_mesh_complexity_;
  FT smooth_angle_delta_;
  bool apply_edge_flip_;
  EdgeFlipStrategy edge_flip_strategy_;
  bool flip_after_split_and_collapse_;
  bool relocate_after_local_operations_;
  RelocateStrategy relocate_strategy_;
  bool keep_vertex_in_one_ring_;
  bool use_local_aabb_tree_;
  int collapse_list_size_;
  bool decrease_max_errors_;
  bool verbose_progress_;
  bool apply_initial_mesh_simplification_;
  bool apply_final_vertex_relocation_;
  int samples_per_facet_in_;               // sample parameters
  int samples_per_facet_out_;
  int max_samples_per_area_;
  int min_samples_per_triangle_;
  int bvd_iteration_count_;
  SampleNumberStrategy sample_number_strategy_;
  SampleStrategy sample_strategy_;
  bool use_stratified_sampling_;
  FT sum_theta_;                           // feature function parameters
  FT sum_delta_;
  FT dihedral_theta_;
  FT dihedral_delta_;
  FT feature_difference_delta_;
  FT feature_control_delta_;
  bool inherit_element_types_;
  bool use_feature_intensity_weights_;
  int vertex_optimize_count_;              // vertex relocate_parameters
  FT vertex_optimize_ratio_;
  int stencil_ring_size_;
  OptimizeStrategy optimize_strategy_;
  OptimizeType facet_optimize_type_;
  OptimizeType edge_optimize_type_;
  OptimizeType vertex_optimize_type_;
  bool optimize_after_local_operations_;

  // 2) the collapse operator
  Visit_list collapsed_list_;
  std::map<Point, std::map<FT, Visit_iter>, Point_comp> collapsed_map_;

  // 3) member data and properties (do not contain Surface_mesh)
  Mesh_properties *input_properties_, *remesh_properties_;
};

}
}
}

#endif // CGAL_MINANGLE_REMESH_IMPL_H
