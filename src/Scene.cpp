#include "Scene.h"
#include <fstream>
#include <QFileInfo>
#include <QOpenGLShader>
#include <QDebug>

Scene::Scene() 
  : m_frame(new ManipulatedFrame()) 
  , gl_init(false)
  , are_buffers_initialized(false)
  , gl(NULL)
  , m_pSurfacemeshInput(NULL)
  , m_pSurfacemeshRemesh(NULL)
  , m_pPolyhedronInput(NULL)
  , m_pPolyhedronRemesh(NULL) {
  // 1) member data initialization
  startTimer(0);
  m_bbox = Bbox(DOUBLE_MAX, DOUBLE_MAX, DOUBLE_MAX, 
                DOUBLE_MIN, DOUBLE_MIN, DOUBLE_MIN);
  // 2) view option initialization
  m_view_surfacemesh_input = false;
  m_view_surfacemesh_remesh = false;
  m_view_polyhedron_input = false;
  m_view_polyhedron_remesh = true;
  m_view_facet_in_start_points = false;
  m_view_facet_in_end_points = false;
  m_view_facet_in_links = false;
  m_view_facet_out_start_points= false;
  m_view_facet_out_end_points = false;
  m_view_facet_out_links = false;
  m_view_edge_in_start_points = false;
  m_view_edge_in_end_points = false;
  m_view_edge_in_links = false;
  m_view_edge_out_start_points = false;
  m_view_edge_out_end_points = false;
  m_view_edge_out_links = false;
  m_view_vertex_in_start_points = false; 
  m_view_vertex_in_end_points = false;
  m_view_vertex_in_links = false;
  m_view_vertex_out_start_points = false;
  m_view_vertex_out_end_points = false;
  m_view_vertex_out_links = false;
  m_view_minimal_angle = true;
  m_view_polyhedron_edges = true;
  reset_draw_render_types();
  // 3) status initialization
  m_input_aabb_tree_constructed = false;
  m_links_initialized = false;
  // 4) parameters initialization
  m_target_edge_length = 0.2;
  m_smooth_iteration_count = 3;
}

Scene::~Scene() {
  // member data
  delete m_frame;
  if (gl != NULL) {
    delete gl;
  }
  for (int i = 0; i < VAOs::ka_NbOfVaos; ++i) {
    vao[i].destroy();
  }
  for (int i = 0; i < VBOs::kb_NbOfVbos; ++i) {
    vbo[i].destroy();
  }
  if (m_pSurfacemeshInput != NULL) {
    delete m_pSurfacemeshInput;
    m_pSurfacemeshInput = NULL;
  }
  if (m_pSurfacemeshRemesh != NULL) {
    delete m_pSurfacemeshRemesh;
    m_pSurfacemeshRemesh = NULL;
  }
  if (m_pPolyhedronInput != NULL) {
    delete m_pPolyhedronInput;
    m_pPolyhedronInput = NULL;
  }
  if (m_pPolyhedronRemesh != NULL) {
    delete m_pPolyhedronRemesh;
    m_pPolyhedronRemesh = NULL;
  }
}

void Scene::update_bbox() {
  std::cout << "Computing bbox...";
  m_bbox = Bbox(DOUBLE_MAX, DOUBLE_MAX, DOUBLE_MAX,
                DOUBLE_MIN, DOUBLE_MIN, DOUBLE_MIN);
  bool suc = false;
  // step 1: calcualte the bbox of the Polyhedron_3 based data structure
  if (m_pPolyhedronInput != NULL && !m_pPolyhedronInput->empty()) {
    Bbox bbox_input = m_pPolyhedronInput->get_bounding_box();
    m_bbox = m_bbox + bbox_input;
    suc = true;
  }
  // step 2: calculate the bbox of the Surface_mesh based data structure
  if (m_pSurfacemeshInput != NULL && !m_pSurfacemeshInput->is_empty()) {
    const Mesh::Property_map<vertex_descriptor, Point> &location =
      m_pSurfacemeshInput->points();
    BOOST_FOREACH(vertex_descriptor vd, m_pSurfacemeshInput->vertices()) {
      m_bbox += location[vd].bbox();
    }
    suc = true;
  }
  if (!suc) {
    std::cout << "failed (no polyhedron or empty polyhedron)." << std::endl;
  }
  else {
    std::cout << "Done" << std::endl;
  }
}

void Scene::initGL() {
  gl = new QOpenGLFunctions_2_1();
  if (!gl->initializeOpenGLFunctions()) {
    qFatal("ERROR : OpenGL Functions not initialized. Check your OpenGL Verison (should be >=3.3)");
    exit(1);
  }
  compile_shaders();
  gl_init = true;
}

void Scene::compile_shaders() {
  // step 1: initialize the VAOs and VBOs
  for (int i = 0; i < VAOs::ka_NbOfVaos; ++i) {
    if (!vao[i].create()) {
      std::cerr << "VAO Creation FAILED" << std::endl;
    }
  }
  for (int i = 0; i < VBOs::kb_NbOfVbos; ++i) {
    if (!vbo[i].create()) {
      std::cerr << "VBO Creation FAILED" << std::endl;
    }
  }

  // step 2: compile the rendering_program
  const char vertex_source[] = {        // vertex source code
    "#version 120 \n"
    "attribute highp vec4 vertex;\n"
    "uniform highp mat4 mvp_matrix;\n"
    "uniform highp mat4 f_matrix;\n"
    "void main(void)\n"
    "{\n"
    "   gl_Position = mvp_matrix * f_matrix * vertex;\n"
    "}"
  };
  const char fragment_source[] = {      // fragment source code
    "#version 120 \n"
    "uniform highp vec4 color; \n"
    "void main(void) { \n"
    "gl_FragColor = color; \n"
    "} \n"
    "\n"
  };
  QOpenGLShader *vertex_shader = new QOpenGLShader(QOpenGLShader::Vertex);
  if (!vertex_shader->compileSourceCode(vertex_source)) {
    std::cerr << "Compiling vertex source FAILED" << std::endl;
  }
  QOpenGLShader *fragment_shader = new QOpenGLShader(QOpenGLShader::Fragment);
  if (!fragment_shader->compileSourceCode(fragment_source)) {
    std::cerr << "Compiling fragmentsource FAILED" << std::endl;
  }
  if (!rendering_program.addShader(vertex_shader)) {
    std::cerr << "adding vertex shader FAILED" << std::endl;
  }
  if (!rendering_program.addShader(fragment_shader)) {
    std::cerr << "adding fragment shader FAILED" << std::endl;
  }
  if (!rendering_program.link()) {
    std::cerr << "linking Program FAILED" << std::endl;
  }
  rendering_program.bind();

  // step 3: compile the rendering_program_with_light
  const char light_vertex_source[] = {        // Vertex source code
    "#version 120\n"
    "attribute highp vec4 vertex;\n"
    "attribute highp vec3 normals;\n"
    "attribute highp vec3 colors;\n"
    "uniform highp mat4 mvp_matrix;\n"
    "uniform highp mat4 mv_matrix;\n"
    "varying highp vec4 fP;\n"
    "varying highp vec3 fN;\n"
    "varying highp vec4 color;\n"
    "void main(void) {\n"
    "  color = vec4(colors, 1.0);\n"
    "  gl_Position = mvp_matrix * vertex;\n"
    "  fP = mv_matrix * vertex;\n"
    "  fN = mat3(mv_matrix)* normals;\n"
    "}\n"
  };
  const char light_fragment_source[] = {    //Fragment source code
    "#version 120\n"
    "varying highp vec4 color;\n"
    "varying highp vec4 fP;\n"
    "varying highp vec3 fN;\n"
    "uniform highp vec4 light_pos;\n"
    "uniform highp vec4 light_diff;\n"
    "uniform highp vec4 light_spec;\n"
    "uniform highp vec4 light_amb;\n"
    "uniform highp float spec_power;\n"
    "uniform int is_two_side;\n"
    "uniform bool is_selected;\n"
    "void main(void) {\n"
    "  highp vec3 L = light_pos.xyz - fP.xyz;\n"
    "  highp vec3 V = -fP.xyz;\n"
    "  highp vec3 N;\n"
    "  if (fN == highp vec3(0.0, 0.0, 0.0))\n"
    "    N = highp vec3(0.0, 0.0, 0.0);\n"
    "  else\n"
    "    N = normalize(fN);\n"
    "  L = normalize(L);\n"
    "  V = normalize(V);\n"
    "  highp vec3 R = reflect(-L, N);\n"
    "  vec4 diffuse;\n"
    "  if (is_two_side == 1)\n"
    "    diffuse = abs(dot(N, L)) * light_diff * color;\n"
    "  else\n"
    "    diffuse = max(dot(N, L), 0.0) * light_diff * color;\n"
    "  highp vec4 specular = pow(max(dot(R, V), 0.0), spec_power) * light_spec;\n"
    "  vec4 ret_color = vec4((color*light_amb).xyz + diffuse.xyz + specular.xyz, 1);\n"
    "  if (is_selected)\n"
    "    gl_FragColor = vec4(ret_color.r + 70.0 / 255.0, ret_color.g + 70.0 / 255.0, ret_color.b + 70.0 / 255.0, 1.0);\n"
    "  else\n"
    "    gl_FragColor = ret_color;\n"
    "}\n"
  };
  QOpenGLShader *light_vertex_shader = 
      new QOpenGLShader(QOpenGLShader::Vertex);
  if (!light_vertex_shader->compileSourceCode(light_vertex_source)) {
    std::cerr << "Compiling light vertex source FAILED" << std::endl;
  }
  QOpenGLShader *light_fragment_shader = 
      new QOpenGLShader(QOpenGLShader::Fragment);
  if (!light_fragment_shader->compileSourceCode(light_fragment_source)) {
    std::cerr << "Compiling light fragment source FAILED" << std::endl;
  }
  if (!rendering_program_with_light.addShader(light_vertex_shader)) {
    std::cerr << "adding light vertex shader FAILED" << std::endl;
  }
  if (!rendering_program_with_light.addShader(light_fragment_shader)) {
    std::cerr << "adding light fragment shader FAILED" << std::endl;
  }
  if (!rendering_program_with_light.link()) {
    std::cerr << "linking Program FAILED" << std::endl;
  }
  rendering_program_with_light.bind();
}

void Scene::initialize_buffers() {
  // Surface_mesh input facets
  vao[VAOs::ka_SInput_faces].bind();
  // bind the pos_face
  vbo[VBOs::kb_SInput_face_pos].bind();
  vbo[VBOs::kb_SInput_face_pos].allocate(pos_sinput_faces.data(),
    static_cast<int>(pos_sinput_faces.size() * sizeof(float)));
  rendering_program_with_light.bind();
  poly_vertexLocation_with_light =
    rendering_program_with_light.attributeLocation("vertex");
  rendering_program_with_light.enableAttributeArray(
    poly_vertexLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
    poly_vertexLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[VBOs::kb_SInput_face_pos].release();
  // bind the pos_face_normal
  vbo[VBOs::kb_SInput_face_normals].bind();
  vbo[VBOs::kb_SInput_face_normals].allocate(pos_sinput_face_normals.data(),
    static_cast<int>(pos_sinput_face_normals.size() * sizeof(float)));
  rendering_program_with_light.bind();
  normalLocation_with_light =
    rendering_program_with_light.attributeLocation("normals");
  rendering_program_with_light.enableAttributeArray(
    normalLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
    normalLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[VBOs::kb_SInput_face_normals].release();
  // bind the pos_face_colors
  vbo[VBOs::kb_SInput_face_colors].bind();
  vbo[VBOs::kb_SInput_face_colors].allocate(pos_sinput_face_colors.data(),
    static_cast<int>(pos_sinput_face_colors.size() * sizeof(float)));
  rendering_program_with_light.bind();
  colorLocation_with_light =
    rendering_program_with_light.attributeLocation("colors");
  rendering_program_with_light.enableAttributeArray(colorLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
    colorLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[kb_SInput_face_colors].release();
  vao[VAOs::ka_SInput_faces].release();

  // Surface_mesh remesh facets
  vao[VAOs::ka_SRemesh_faces].bind();
  // bind the pos_face
  vbo[VBOs::kb_SRemesh_face_pos].bind();
  vbo[VBOs::kb_SRemesh_face_pos].allocate(pos_sremesh_faces.data(),
    static_cast<int>(pos_sremesh_faces.size() * sizeof(float)));
  rendering_program_with_light.bind();
  poly_vertexLocation_with_light =
    rendering_program_with_light.attributeLocation("vertex");
  rendering_program_with_light.enableAttributeArray(
    poly_vertexLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
    poly_vertexLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[VBOs::kb_SRemesh_face_pos].release();
  // bind the pos_face_normal
  vbo[VBOs::kb_SRemesh_face_normals].bind();
  vbo[VBOs::kb_SRemesh_face_normals].allocate(pos_sremesh_face_normals.data(),
    static_cast<int>(pos_sremesh_face_normals.size() * sizeof(float)));
  rendering_program_with_light.bind();
  normalLocation_with_light =
    rendering_program_with_light.attributeLocation("normals");
  rendering_program_with_light.enableAttributeArray(normalLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
    normalLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[VBOs::kb_SRemesh_face_normals].release();
  // bind the pos_ifi_colors
  vbo[VBOs::kb_SRemesh_face_colors].bind();
  vbo[VBOs::kb_SRemesh_face_colors].allocate(pos_sremesh_face_colors.data(),
    static_cast<int>(pos_sremesh_face_colors.size() * sizeof(float)));
  rendering_program_with_light.bind();
  colorLocation_with_light =
    rendering_program_with_light.attributeLocation("colors");
  rendering_program_with_light.enableAttributeArray(colorLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
    colorLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[kb_SRemesh_face_colors].release();
  vao[VAOs::ka_SRemesh_faces].release();

  // Surface_mesh input edges
  vao[VAOs::ka_SInput_edges].bind();
  vbo[VBOs::kb_SInput_edge_pos].bind();
  vbo[VBOs::kb_SInput_edge_pos].allocate(pos_sinput_edges.data(),
    static_cast<int>(pos_sinput_edges.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  rendering_program.release();
  vbo[VBOs::kb_SInput_edge_pos].release();
  vao[VAOs::ka_SInput_edges].release();

  // Surface_mesh remesh edges
  vao[VAOs::ka_SRemesh_edges].bind();
  vbo[VBOs::kb_SRemesh_edge_pos].bind();
  vbo[VBOs::kb_SRemesh_edge_pos].allocate(pos_sremesh_edges.data(),
    static_cast<int>(pos_sremesh_edges.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  rendering_program.release();
  vbo[VBOs::kb_SRemesh_edge_pos].release();
  vao[VAOs::ka_SRemesh_edges].release();

  // Polyhedron_3 input facets
  vao[VAOs::ka_PInput_faces].bind();
  // bind the pos_face
  vbo[VBOs::kb_PInput_face_pos].bind();
  vbo[VBOs::kb_PInput_face_pos].allocate(pos_pinput_faces.data(),
      static_cast<int>(pos_pinput_faces.size() * sizeof(float)));
  rendering_program_with_light.bind();
  poly_vertexLocation_with_light =
      rendering_program_with_light.attributeLocation("vertex");
  rendering_program_with_light.enableAttributeArray(
      poly_vertexLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
      poly_vertexLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[VBOs::kb_PInput_face_pos].release();
  // bind the pos_face_normal
  vbo[VBOs::kb_PInput_face_normals].bind();
  vbo[VBOs::kb_PInput_face_normals].allocate(pos_pinput_face_normals.data(),
      static_cast<int>(pos_pinput_face_normals.size() * sizeof(float)));
  rendering_program_with_light.bind();
  normalLocation_with_light =
      rendering_program_with_light.attributeLocation("normals");
  rendering_program_with_light.enableAttributeArray(
      normalLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
      normalLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[VBOs::kb_PInput_face_normals].release();
  // bind the pos_face_colors
  vbo[VBOs::kb_PInput_face_colors].bind();
  vbo[VBOs::kb_PInput_face_colors].allocate(pos_pinput_face_colors.data(),
    static_cast<int>(pos_pinput_face_colors.size() * sizeof(float)));
  rendering_program_with_light.bind();
  colorLocation_with_light =
      rendering_program_with_light.attributeLocation("colors");
  rendering_program_with_light.enableAttributeArray(colorLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
      colorLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[kb_PInput_face_colors].release();
  vao[VAOs::ka_PInput_faces].release();

  // Polyhedron_3 remesh facets
  vao[VAOs::ka_PRemesh_faces].bind();
  // bind the pos_face
  vbo[VBOs::kb_PRemesh_face_pos].bind();
  vbo[VBOs::kb_PRemesh_face_pos].allocate(pos_premesh_faces.data(),
      static_cast<int>(pos_premesh_faces.size() * sizeof(float)));
  rendering_program_with_light.bind();
  poly_vertexLocation_with_light =
      rendering_program_with_light.attributeLocation("vertex");
  rendering_program_with_light.enableAttributeArray(
      poly_vertexLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
      poly_vertexLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[VBOs::kb_PRemesh_face_pos].release();
  // bind the pos_face_normal
  vbo[VBOs::kb_PRemesh_face_normals].bind();
  vbo[VBOs::kb_PRemesh_face_normals].allocate(pos_premesh_face_normals.data(),
      static_cast<int>(pos_premesh_face_normals.size() * sizeof(float)));
  rendering_program_with_light.bind();
  normalLocation_with_light =
      rendering_program_with_light.attributeLocation("normals");
  rendering_program_with_light.enableAttributeArray(normalLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
      normalLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[VBOs::kb_PRemesh_face_normals].release();
  // bind the pos_ifi_colors
  vbo[VBOs::kb_PRemesh_face_colors].bind();
  vbo[VBOs::kb_PRemesh_face_colors].allocate(pos_premesh_face_colors.data(),
      static_cast<int>(pos_premesh_face_colors.size() * sizeof(float)));
  rendering_program_with_light.bind();
  colorLocation_with_light =
      rendering_program_with_light.attributeLocation("colors");
  rendering_program_with_light.enableAttributeArray(colorLocation_with_light);
  rendering_program_with_light.setAttributeBuffer(
      colorLocation_with_light, GL_FLOAT, 0, 3);
  rendering_program_with_light.release();
  vbo[kb_PRemesh_face_colors].release();
  vao[VAOs::ka_PRemesh_faces].release();

  // Input cell boundaries
  vao[VAOs::ka_PInput_boundaries].bind();
  vbo[VBOs::kb_PInput_boundary_pos].bind();
  vbo[VBOs::kb_PInput_boundary_pos].allocate(pos_pinput_boundaries.data(),
      static_cast<int>(pos_pinput_boundaries.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  rendering_program.release();
  vbo[VBOs::kb_PInput_boundary_pos].release();
  vao[VAOs::ka_PInput_boundaries].release();

  // Remesh cell boundaries
  vao[VAOs::ka_PRemesh_boundaries].bind();
  vbo[VBOs::kb_PRemesh_boundary_pos].bind();
  vbo[VBOs::kb_PRemesh_boundary_pos].allocate(pos_premesh_boundaries.data(),
      static_cast<int>(pos_premesh_boundaries.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  rendering_program.release();
  vbo[VBOs::kb_PRemesh_boundary_pos].release();
  vao[VAOs::ka_PRemesh_boundaries].release();

  // Input samples
  vao[VAOs::ka_PInput_samples].bind();
  vbo[VBOs::kb_PInput_sample_pos].bind();
  vbo[VBOs::kb_PInput_sample_pos].allocate(pos_pinput_samples.data(),
      static_cast<int>(pos_pinput_samples.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_PInput_sample_pos].release();
  rendering_program.release();
  vao[VAOs::ka_PInput_samples].release();

  // Remesh samples
  vao[VAOs::ka_PRemesh_samples].bind();
  vbo[VBOs::kb_PRemesh_sample_pos].bind();
  vbo[VBOs::kb_PRemesh_sample_pos].allocate(pos_premesh_samples.data(),
      static_cast<int>(pos_premesh_samples.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_PRemesh_sample_pos].release();
  rendering_program.release();
  vao[VAOs::ka_PRemesh_samples].release();

  // Input normal edges
  vao[VAOs::ka_PInput_normal_edges].bind();
  vbo[VBOs::kb_PInput_normal_edge_pos].bind();
  vbo[VBOs::kb_PInput_normal_edge_pos].allocate(pos_pinput_normal_edges.data(),
      static_cast<int>(pos_pinput_normal_edges.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  rendering_program.release();
  vbo[VBOs::kb_PInput_normal_edge_pos].release();
  vao[VAOs::ka_PInput_normal_edges].release();

  // Remesh normal edges
  vao[VAOs::ka_PRemesh_normal_edges].bind();
  vbo[VBOs::kb_PRemesh_normal_edge_pos].bind();
  vbo[VBOs::kb_PRemesh_normal_edge_pos].allocate(pos_premesh_normal_edges.data(),
      static_cast<int>(pos_premesh_normal_edges.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  rendering_program.release();
  vbo[VBOs::kb_PRemesh_normal_edge_pos].release();
  vao[VAOs::ka_PRemesh_normal_edges].release();

  // Input min radian edges
  vao[VAOs::ka_PInput_special_edges].bind();
  vbo[VBOs::kb_PInput_special_edge_pos].bind();
  vbo[VBOs::kb_PInput_special_edge_pos].allocate(pos_pinput_special_edges.data(),
      static_cast<int>(pos_pinput_special_edges.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  rendering_program.release();
  vbo[VBOs::kb_PInput_special_edge_pos].release();
  vao[VAOs::ka_PInput_special_edges].release();

  // Remesh min radian edges
  vao[VAOs::ka_PRemesh_special_edges].bind();
  vbo[VBOs::kb_Remesh_special_edge_pos].bind();
  vbo[VBOs::kb_Remesh_special_edge_pos].allocate(
      pos_premesh_special_edges.data(),
      static_cast<int>(pos_premesh_special_edges.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  rendering_program.release();
  vbo[VBOs::kb_Remesh_special_edge_pos].release();
  vao[VAOs::ka_PRemesh_special_edges].release();

  // Facet in start points
  vao[VAOs::ka_Facet_in_start].bind();
  vbo[VBOs::kb_Facet_in_start_points].bind();
  vbo[VBOs::kb_Facet_in_start_points].allocate(
      pos_facet_in_start_points.data(),
      static_cast<int>(pos_facet_in_start_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Facet_in_start_points].release();
  rendering_program.release();
  vao[VAOs::ka_Facet_in_start].release();

  // Facet in end points
  vao[VAOs::ka_Facet_in_end].bind();
  vbo[VBOs::kb_Facet_in_end_points].bind();
  vbo[VBOs::kb_Facet_in_end_points].allocate(pos_facet_in_end_points.data(),
      static_cast<int>(pos_facet_in_end_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Facet_in_end_points].release();
  rendering_program.release();
  vao[VAOs::ka_Facet_in_end].release();

  // Facet in links
  vao[VAOs::ka_Facet_in_links].bind();
  vbo[VBOs::kb_Facet_in_link_lines].bind();
  vbo[VBOs::kb_Facet_in_link_lines].allocate(pos_facet_in_links.data(),
      static_cast<int>(pos_facet_in_links.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  vbo[VBOs::kb_Facet_in_link_lines].release();
  rendering_program.release();
  vao[VAOs::ka_Facet_in_links].release();

  // Facet out start points
  vao[VAOs::ka_Facet_out_start].bind();
  vbo[VBOs::kb_Facet_out_start_points].bind();
  vbo[VBOs::kb_Facet_out_start_points].allocate(
      pos_facet_out_start_points.data(),
      static_cast<int>(pos_facet_out_start_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Facet_out_start_points].release();
  rendering_program.release();
  vao[VAOs::ka_Facet_out_start].release();

  // Facet out end points
  vao[VAOs::ka_Facet_out_end].bind();
  vbo[VBOs::kb_Facet_out_end_points].bind();
  vbo[VBOs::kb_Facet_out_end_points].allocate(pos_facet_out_end_points.data(),
      static_cast<int>(pos_facet_out_end_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Facet_out_end_points].release();
  rendering_program.release();
  vao[VAOs::ka_Facet_out_end].release();

  // Facet out links
  vao[VAOs::ka_Facet_out_links].bind();
  vbo[VBOs::kb_Facet_out_link_lines].bind();
  vbo[VBOs::kb_Facet_out_link_lines].allocate(pos_facet_out_links.data(),
      static_cast<int>(pos_facet_out_links.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  vbo[VBOs::kb_Facet_out_link_lines].release();
  rendering_program.release();
  vao[VAOs::ka_Facet_out_links].release();

  // Edge in start points
  vao[VAOs::ka_Edge_in_start].bind();
  vbo[VBOs::kb_Edge_in_start_points].bind();
  vbo[VBOs::kb_Edge_in_start_points].allocate(pos_edge_in_start_points.data(),
      static_cast<int>(pos_edge_in_start_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Edge_in_start_points].release();
  rendering_program.release();
  vao[VAOs::ka_Edge_in_start].release();

  // Edge in end points
  vao[VAOs::ka_Edge_in_end].bind();
  vbo[VBOs::kb_Edge_in_end_points].bind();
  vbo[VBOs::kb_Edge_in_end_points].allocate(pos_edge_in_end_points.data(),
      static_cast<int>(pos_edge_in_end_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Edge_in_end_points].release();
  rendering_program.release();
  vao[VAOs::ka_Edge_in_end].release();

  // Edge in links
  vao[VAOs::ka_Edge_in_links].bind();
  vbo[VBOs::kb_Edge_in_link_lines].bind();
  vbo[VBOs::kb_Edge_in_link_lines].allocate(pos_edge_in_links.data(),
      static_cast<int>(pos_edge_in_links.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  vbo[VBOs::kb_Edge_in_link_lines].release();
  rendering_program.release();
  vao[VAOs::ka_Edge_in_links].release();

  // Edge out start points
  vao[VAOs::ka_Edge_out_start].bind();
  vbo[VBOs::kb_Edge_out_start_points].bind();
  vbo[VBOs::kb_Edge_out_start_points].allocate(
      pos_edge_out_start_points.data(),
      static_cast<int>(pos_edge_out_start_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Edge_out_start_points].release();
  rendering_program.release();
  vao[VAOs::ka_Edge_out_start].release();

  // Edge out end points
  vao[VAOs::ka_Edge_out_end].bind();
  vbo[VBOs::kb_Edge_out_end_points].bind();
  vbo[VBOs::kb_Edge_out_end_points].allocate(pos_edge_out_end_points.data(),
      static_cast<int>(pos_edge_out_end_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Edge_out_end_points].release();
  rendering_program.release();
  vao[VAOs::ka_Edge_out_end].release();

  // Edge out links
  vao[VAOs::ka_Edge_out_links].bind();
  vbo[VBOs::kb_Edge_out_link_lines].bind();
  vbo[VBOs::kb_Edge_out_link_lines].allocate(pos_edge_out_links.data(),
      static_cast<int>(pos_edge_out_links.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  vbo[VBOs::kb_Edge_out_link_lines].release();
  rendering_program.release();
  vao[VAOs::ka_Edge_out_links].release();

  // Vertex in start points
  vao[VAOs::ka_Vertex_in_start].bind();
  vbo[VBOs::kb_Vertex_in_start_points].bind();
  vbo[VBOs::kb_Vertex_in_start_points].allocate(
      pos_vertex_in_start_points.data(),
      static_cast<int>(pos_vertex_in_start_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Vertex_in_start_points].release();
  rendering_program.release();
  vao[VAOs::ka_Vertex_in_start].release();

  // Vertex in end points
  vao[VAOs::ka_Vertex_in_end].bind();
  vbo[VBOs::kb_Vertex_in_end_points].bind();
  vbo[VBOs::kb_Vertex_in_end_points].allocate(pos_vertex_in_end_points.data(),
      static_cast<int>(pos_vertex_in_end_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Vertex_in_end_points].release();
  rendering_program.release();
  vao[VAOs::ka_Vertex_in_end].release();

  // Vertex in links
  vao[VAOs::ka_Vertex_in_links].bind();
  vbo[VBOs::kb_Vertex_in_link_lines].bind();
  vbo[VBOs::kb_Vertex_in_link_lines].allocate(pos_vertex_in_links.data(),
      static_cast<int>(pos_vertex_in_links.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  vbo[VBOs::kb_Vertex_in_link_lines].release();
  rendering_program.release();
  vao[VAOs::ka_Vertex_in_links].release();

  // Vertex out start points
  vao[VAOs::ka_Vertex_out_start].bind();
  vbo[VBOs::kb_Vertex_out_start_points].bind();
  vbo[VBOs::kb_Vertex_out_start_points].allocate(
      pos_vertex_out_start_points.data(),
      static_cast<int>(pos_vertex_out_start_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Vertex_out_start_points].release();
  rendering_program.release();
  vao[VAOs::ka_Vertex_out_start].release();

  // Vertex out end points
  vao[VAOs::ka_Vertex_out_end].bind();
  vbo[VBOs::kb_Vertex_out_end_points].bind();
  vbo[VBOs::kb_Vertex_out_end_points].allocate( 
      pos_vertex_out_end_points.data(),
      static_cast<int>(pos_vertex_out_end_points.size() * sizeof(float)));
  points_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(points_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(points_vertexLocation);
  vbo[VBOs::kb_Vertex_out_end_points].release();
  rendering_program.release();
  vao[VAOs::ka_Vertex_out_end].release();

  // Vertex out links
  vao[VAOs::ka_Vertex_out_links].bind();
  vbo[VBOs::kb_Vertex_out_link_lines].bind();
  vbo[VBOs::kb_Vertex_out_link_lines].allocate(pos_vertex_out_links.data(),
      static_cast<int>(pos_vertex_out_links.size() * sizeof(float)));
  lines_vertexLocation = rendering_program.attributeLocation("vertex");
  rendering_program.bind();
  rendering_program.setAttributeBuffer(lines_vertexLocation, GL_FLOAT, 0, 3);
  rendering_program.enableAttributeArray(lines_vertexLocation);
  vbo[VBOs::kb_Vertex_out_link_lines].release();
  rendering_program.release();
  vao[VAOs::ka_Vertex_out_links].release();

  are_buffers_initialized = true;
}

void Scene::attrib_buffers(CGAL::QGLViewer *viewer) {
  QMatrix4x4 mvMatrix, mvpMatrix;
  double mvMat[16], mvpMat[16];
  viewer->camera()->getModelViewMatrix(mvMat);
  viewer->camera()->getModelViewProjectionMatrix(mvpMat);
  for (int i = 0; i < 16; ++i) {
    mvMatrix.data()[i] = (float)mvMat[i];
    mvpMatrix.data()[i] = (float)mvpMat[i];
  }

  rendering_program.bind();
  mvpLocation = rendering_program.uniformLocation("mvp_matrix");
  fLocation = rendering_program.uniformLocation("f_matrix");
  colorLocation = rendering_program.uniformLocation("color");
  rendering_program.setUniformValue(mvpLocation, mvpMatrix);
  rendering_program.release();

  QVector4D position(0.0f, 0.0f, 1.0f, 1.0f);
  //QVector4D diffuse(0.8f, 0.8f, 0.8f, 1.0f);    // backup
  //QVector4D specular(0.2f, 0.2f, 0.2f, 1.0f);
  //QVector4D ambient(0.8f, 0.8f, 0.8f, 1.0f);
  //float spec_power = 51.8f;
  QVector4D diffuse(1.0f, 1.0f, 1.0f, 1.0f);
  QVector4D specular(0.05f, 0.05f, 0.05f, 1.0f);
  QVector4D ambient(0.4f, 0.4f, 0.4f, 1.0f);
  float spec_power = 20.0f;
  GLint is_both_sides = 0;
  rendering_program_with_light.bind();
  rendering_program_with_light.setUniformValue("light_pos", position);
  rendering_program_with_light.setUniformValue("light_diff", diffuse);
  rendering_program_with_light.setUniformValue("light_spec", specular);
  rendering_program_with_light.setUniformValue("light_amb", ambient);
  rendering_program_with_light.setUniformValue("spec_power", spec_power);
  rendering_program_with_light.setUniformValue("is_two_side", is_both_sides);
  rendering_program_with_light.setUniformValue("mv_matrix", mvMatrix);
  rendering_program_with_light.setUniformValue("mvp_matrix", mvpMatrix);
  rendering_program_with_light.release();
}

void Scene::draw(CGAL::QGLViewer *viewer) {
  if (!gl_init) {
    initGL();
  }
  if (!are_buffers_initialized) {
    initialize_buffers();
  }
  gl->glEnable(GL_DEPTH_TEST);
  QColor color;
  QMatrix4x4 fMatrix;
  fMatrix.setToIdentity();
  // Surface_mesh input
  if (m_view_surfacemesh_input) {
    if (pos_sinput_faces.size() > 0) {
      gl->glEnable(GL_LIGHTING);
      gl->glEnable(GL_POLYGON_OFFSET_FILL);
      gl->glPolygonOffset(1.0f, 1.0f);
      vao[VAOs::ka_SInput_faces].bind();
      attrib_buffers(viewer);
      rendering_program_with_light.bind();
      rendering_program_with_light.setUniformValue("is_selected", false);
      rendering_program_with_light.setUniformValue("f_matrix", fMatrix);
      gl->glDrawArrays(GL_TRIANGLES, 0,
        static_cast<GLsizei>(pos_sinput_faces.size() / 3));
      rendering_program_with_light.release();
      vao[VAOs::ka_SInput_faces].release();
      gl->glDisable(GL_POLYGON_OFFSET_FILL);
    }
    if (pos_sinput_edges.size() > 0) {
      gl->glDisable(GL_LIGHTING);
      gl->glLineWidth(1.0f);
      vao[VAOs::ka_SInput_edges].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 0, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_sinput_edges.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_SInput_edges].release();
    }
  }
  // Surface_mesh remesh
  if (m_view_surfacemesh_remesh) {
    if (pos_sremesh_faces.size() > 0) {
      gl->glEnable(GL_LIGHTING);
      gl->glEnable(GL_POLYGON_OFFSET_FILL);
      gl->glPolygonOffset(1.0f, 1.0f);
      vao[VAOs::ka_SRemesh_faces].bind();
      attrib_buffers(viewer);
      rendering_program_with_light.bind();
      rendering_program_with_light.setUniformValue("is_selected", false);
      rendering_program_with_light.setUniformValue("f_matrix", fMatrix);
      gl->glDrawArrays(GL_TRIANGLES, 0,
        static_cast<GLsizei>(pos_sremesh_faces.size() / 3));
      rendering_program_with_light.release();
      vao[VAOs::ka_SRemesh_faces].release();
      gl->glDisable(GL_POLYGON_OFFSET_FILL);
    }
    if (pos_sremesh_edges.size() > 0) {
      gl->glDisable(GL_LIGHTING);
      gl->glLineWidth(1.0f);
      vao[VAOs::ka_SRemesh_edges].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 0, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_sremesh_edges.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_SRemesh_edges].release();
    }
  }
  // Polyhedron_3 input
  if (m_view_polyhedron_input) {
    if (pos_pinput_faces.size() > 0) {
      gl->glEnable(GL_LIGHTING);
      gl->glEnable(GL_POLYGON_OFFSET_FILL);
      gl->glPolygonOffset(1.0f, 1.0f);
      vao[VAOs::ka_PInput_faces].bind();
      attrib_buffers(viewer);
      rendering_program_with_light.bind();
      rendering_program_with_light.setUniformValue("is_selected", false);
      rendering_program_with_light.setUniformValue("f_matrix", fMatrix);
      gl->glDrawArrays(GL_TRIANGLES, 0,
        static_cast<GLsizei>(pos_pinput_faces.size() / 3));
      rendering_program_with_light.release();
      vao[VAOs::ka_PInput_faces].release();
      gl->glDisable(GL_POLYGON_OFFSET_FILL);
    }
    if (pos_pinput_boundaries.size() > 0) {
      gl->glDisable(GL_LIGHTING);
      gl->glLineWidth(0.5f);
      vao[VAOs::ka_PInput_boundaries].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 200, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_pinput_boundaries.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_PInput_boundaries].release();
    }
    if (pos_pinput_samples.size() > 0) {
      gl->glDisable(GL_LIGHTING);
      gl->glPointSize(3.0f);
      vao[VAOs::ka_PInput_samples].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 255, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_pinput_samples.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_PInput_samples].release();
    }
    if (pos_pinput_normal_edges.size() > 0 &&
      (m_render_type == k_classifications || m_view_polyhedron_edges)) {
      gl->glDisable(GL_LIGHTING);
      if (m_render_type == k_classifications || pos_pinput_boundaries.empty()) {
        gl->glLineWidth(1.0f);
      }
      else {
        gl->glLineWidth(2.0f);
      }
      vao[VAOs::ka_PInput_normal_edges].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      m_render_type == k_classifications ? color.setRgb(0, 0, 255) :
        color.setRgb(0, 0, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_pinput_normal_edges.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_PInput_normal_edges].release();
    }
    if (pos_pinput_special_edges.size() > 0 &&
      (m_render_type == k_classifications ||
      (m_view_minimal_angle && m_render_type == k_plain_facets))) {
      gl->glDisable(GL_LIGHTING);
      gl->glLineWidth(2.0f);
      vao[VAOs::ka_PInput_special_edges].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(255, 0, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_pinput_special_edges.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_PInput_special_edges].release();
    }
  }
  // Polyhedron_3 remesh
  if (m_view_polyhedron_remesh) {
    if (pos_premesh_faces.size() > 0) {
      gl->glEnable(GL_LIGHTING);
      gl->glEnable(GL_POLYGON_OFFSET_FILL);
      gl->glPolygonOffset(1.0f, 1.0f);
      vao[VAOs::ka_PRemesh_faces].bind();
      attrib_buffers(viewer);
      rendering_program_with_light.bind();
      rendering_program_with_light.setUniformValue("is_selected", false);
      rendering_program_with_light.setUniformValue("f_matrix", fMatrix);
      gl->glDrawArrays(GL_TRIANGLES, 0,
        static_cast<GLsizei>(pos_premesh_faces.size() / 3));
      rendering_program_with_light.release();
      vao[VAOs::ka_PRemesh_faces].release();
      gl->glDisable(GL_POLYGON_OFFSET_FILL);
    }
    if (pos_premesh_boundaries.size() > 0) {
      gl->glDisable(GL_LIGHTING);
      gl->glLineWidth(0.5f);
      vao[VAOs::ka_PRemesh_boundaries].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 200, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_premesh_boundaries.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_PRemesh_boundaries].release();
    }
    if (pos_premesh_samples.size() > 0) {
      gl->glDisable(GL_LIGHTING);
      gl->glPointSize(3.0f);
      vao[VAOs::ka_PRemesh_samples].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 255, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_premesh_samples.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_PRemesh_samples].release();
    }
    if (pos_premesh_normal_edges.size() > 0 &&
      (m_render_type == k_classifications || m_view_polyhedron_edges)) {
      gl->glDisable(GL_LIGHTING);
      if (m_render_type == k_classifications || 
          pos_premesh_boundaries.empty()) {
        gl->glLineWidth(1.0f);
      }
      else {
        gl->glLineWidth(2.0f);
      }
      vao[VAOs::ka_PRemesh_normal_edges].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      m_render_type == k_classifications ? color.setRgb(0, 0, 255) :
        color.setRgb(0, 0, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_premesh_normal_edges.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_PRemesh_normal_edges].release();
    }
    if (pos_premesh_special_edges.size() > 0 &&
      (m_render_type == k_classifications ||
      (m_view_minimal_angle && m_render_type == k_plain_facets))) {
      gl->glDisable(GL_LIGHTING);
      gl->glLineWidth(2.0f);
      vao[VAOs::ka_PRemesh_special_edges].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(255, 0, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_premesh_special_edges.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_PRemesh_special_edges].release();
    }
  }
  // Samples and links
  if (m_draw_type == DrawType::k_polyhedron) {
    gl->glDisable(GL_LIGHTING);
    gl->glPointSize(3.0f);
    gl->glLineWidth(1.0f);
    // facet in start points
    if (m_view_facet_in_start_points && pos_facet_in_start_points.size() > 0) {
      vao[VAOs::ka_Facet_in_start].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 250, 250);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_facet_in_start_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Facet_in_start].release();
    }
    // facet in links
    if (m_view_facet_in_links && pos_facet_in_links.size() > 0) {
      vao[VAOs::ka_Facet_in_links].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 200, 200);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_facet_in_links.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Facet_in_links].release();
    }
    // facet in end points
    if (m_view_facet_in_end_points && pos_facet_in_end_points.size() > 0) {
      vao[VAOs::ka_Facet_in_end].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 150, 150);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_facet_in_end_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Facet_in_end].release();
    }
    // facet out start points
    if (m_view_facet_out_start_points && 
        pos_facet_out_start_points.size() > 0) {
      vao[VAOs::ka_Facet_out_start].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(250, 0, 250);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_facet_out_start_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Facet_out_start].release();
    }
    // facet out links
    if (m_view_facet_out_links && pos_facet_out_links.size() > 0) {
      vao[VAOs::ka_Facet_out_links].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(200, 0, 200);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_facet_out_links.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Facet_out_links].release();
    }
    // facet out end points
    if (m_view_facet_out_end_points && pos_facet_out_end_points.size() > 0) {
      vao[VAOs::ka_Facet_out_end].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(150, 0, 150);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_facet_out_end_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Facet_out_end].release();
    }
    // edge in start points
    if (m_view_edge_in_start_points && pos_edge_in_start_points.size() > 0) {
      vao[VAOs::ka_Edge_in_start].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 0, 250);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_edge_in_start_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Edge_in_start].release();
    }
    // edge in links
    if (m_view_edge_in_links && pos_edge_in_links.size() > 0) {
      vao[VAOs::ka_Edge_in_links].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 0, 200);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_edge_in_links.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Edge_in_links].release();
    }
    // edge in end points
    if (m_view_edge_in_end_points && pos_edge_in_end_points.size() > 0) {
      vao[VAOs::ka_Edge_in_end].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 0, 150);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_edge_in_end_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Edge_in_end].release();
    }
    // edge out start points
    if (m_view_edge_out_start_points && pos_edge_out_start_points.size() > 0) {
      vao[VAOs::ka_Edge_out_start].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(250, 250, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_edge_out_start_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Edge_out_start].release();
    }
    // edge out links
    if (m_view_edge_out_links && pos_edge_out_links.size() > 0) {
      vao[VAOs::ka_Edge_out_links].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(200, 200, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_edge_out_links.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Edge_out_links].release();
    }
    // edge out end points
    if (m_view_edge_out_end_points && pos_edge_out_end_points.size() > 0) {
      vao[VAOs::ka_Edge_out_end].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(150, 150, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_edge_out_end_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Edge_out_end].release();
    }
    // vertex in start points
    if (m_view_vertex_in_start_points &&
        pos_vertex_in_start_points.size() > 0) {
      vao[VAOs::ka_Vertex_in_start].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 250, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_vertex_in_start_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Vertex_in_start].release();
    }
    // vertex in links
    if (m_view_vertex_in_links && pos_vertex_in_links.size() > 0) {
      vao[VAOs::ka_Vertex_in_links].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 200, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_vertex_in_links.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Vertex_in_links].release();
    }
    // vertex in end points
    if (m_view_vertex_in_end_points && pos_vertex_in_end_points.size() > 0) {
      vao[VAOs::ka_Vertex_in_end].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(0, 150, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_vertex_in_end_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Vertex_in_end].release();
    }
    // vertex out start points
    if (m_view_vertex_out_start_points &&
        pos_vertex_out_start_points.size() > 0) {
      vao[VAOs::ka_Vertex_out_start].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(250, 0, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_vertex_out_start_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Vertex_out_start].release();
    }
    // vertex out links
    if (m_view_vertex_out_links && pos_vertex_out_links.size() > 0) {
      vao[VAOs::ka_Vertex_out_links].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(200, 0, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_LINES, 0,
        static_cast<GLsizei>(pos_vertex_out_links.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Vertex_out_links].release();
    }
    // vertex out end points
    if (m_view_vertex_out_end_points && pos_vertex_out_end_points.size() > 0) {
      vao[VAOs::ka_Vertex_out_end].bind();
      attrib_buffers(viewer);
      rendering_program.bind();
      color.setRgb(150, 0, 0);
      rendering_program.setUniformValue(colorLocation, color);
      rendering_program.setUniformValue(fLocation, fMatrix);
      gl->glDrawArrays(GL_POINTS, 0,
        static_cast<GLsizei>(pos_vertex_out_end_points.size() / 3));
      rendering_program.release();
      vao[VAOs::ka_Vertex_out_end].release();
    }
  }
}

void Scene::changed() {
  compute_elements();
  are_buffers_initialized = false;
}

void Scene::compute_elements() {
  if (m_view_surfacemesh_input && m_pSurfacemeshInput != NULL) {
    // step 1: compute the facets
    compute_faces(*m_pSurfacemeshInput, true, &pos_sinput_faces, 
                  &pos_sinput_face_normals, &pos_sinput_face_colors);
    // step 2: compute the edges
    compute_edges(*m_pSurfacemeshInput, &pos_sinput_edges);
  }
  if (m_view_surfacemesh_remesh && m_pSurfacemeshRemesh != NULL) {
    // step 1: compute the facets
    compute_faces(*m_pSurfacemeshRemesh, false, &pos_sremesh_faces, 
                  &pos_sremesh_face_normals, &pos_sremesh_face_colors);
    // step 2: compute the edges
    compute_edges(*m_pSurfacemeshRemesh, &pos_sremesh_edges);
  }
  if (m_view_polyhedron_input && m_pPolyhedronInput != NULL) {
    // step 1: compute the facets
    old_remesh.compute_facets(m_draw_type, m_render_type, m_bbox, true, 
      m_pPolyhedronInput, &pos_pinput_faces, &pos_pinput_face_normals,
      &pos_pinput_face_colors, &pos_pinput_boundaries, &pos_pinput_samples);
    // step 2: compute the edges
    if (m_render_type == k_classifications) {
      m_pPolyhedronInput->compute_classified_edges(&pos_pinput_normal_edges, 
                                         &pos_pinput_special_edges);
    }
    else {
      if (m_view_polyhedron_edges) {
        m_pPolyhedronInput->compute_edges(&pos_pinput_normal_edges);
      }
      if (m_view_minimal_angle && m_render_type == k_plain_facets) {
        m_pPolyhedronInput->compute_min_radian_edges(&pos_pinput_special_edges);
      }
    }
  }
  if (m_view_polyhedron_remesh && m_pPolyhedronRemesh != NULL) {
    // step 1: compute the facets
    old_remesh.compute_facets(m_draw_type, m_render_type, m_bbox, false, 
      m_pPolyhedronRemesh, &pos_premesh_faces, &pos_premesh_face_normals,
      &pos_premesh_face_colors, &pos_premesh_boundaries, &pos_premesh_samples);
    // step 2: compute the edges
    if (m_render_type == k_classifications) {
      m_pPolyhedronRemesh->compute_classified_edges(&pos_premesh_normal_edges,
                                          &pos_premesh_special_edges);
    }
    else {
      if (m_view_polyhedron_edges) {
        m_pPolyhedronRemesh->compute_edges(&pos_premesh_normal_edges);
      }
      if (m_view_minimal_angle && m_render_type == k_plain_facets) {
        m_pPolyhedronRemesh->compute_min_radian_edges(&pos_premesh_special_edges);
      }
    }
  }
  // step 3: compute samples and links
  if (m_pPolyhedronInput != NULL && m_pPolyhedronRemesh != NULL && m_draw_type == k_polyhedron) {
    // facet in links
    if (m_view_facet_in_start_points) {
      m_pPolyhedronInput->compute_facet_start_points(&pos_facet_in_start_points);
    }
    if (m_view_facet_in_end_points) {
      m_pPolyhedronInput->compute_facet_end_points(&pos_facet_in_end_points);
    }
    if (m_view_facet_in_links) {
      m_pPolyhedronInput->compute_facet_links(&pos_facet_in_links);
    }
    // edge in links
    if (m_view_edge_in_start_points) {
      m_pPolyhedronInput->compute_edge_start_points(&pos_edge_in_start_points);
    }
    if (m_view_edge_in_end_points) {
      m_pPolyhedronInput->compute_edge_end_points(&pos_edge_in_end_points);
    }
    if (m_view_edge_in_links) {
      m_pPolyhedronInput->compute_edge_links(&pos_edge_in_links);
    }
    // vertex in links
    if (m_view_vertex_in_start_points) {
      m_pPolyhedronInput->compute_vertex_start_points(&pos_vertex_in_start_points);
    }
    if (m_view_vertex_in_end_points) {
      m_pPolyhedronInput->compute_vertex_end_points(&pos_vertex_in_end_points);
    }
    if (m_view_vertex_in_links) {
      m_pPolyhedronInput->compute_vertex_links(&pos_vertex_in_links);
    }
    // facet out links
    if (m_view_facet_out_start_points) {
      m_pPolyhedronRemesh->compute_facet_start_points(&pos_facet_out_start_points);
    }
    if (m_view_facet_out_end_points) {
      m_pPolyhedronRemesh->compute_facet_end_points(&pos_facet_out_end_points);
    }
    if (m_view_facet_out_links) {
      m_pPolyhedronRemesh->compute_facet_links(&pos_facet_out_links);
    }
    // edge out links
    if (m_view_edge_out_start_points) {
      m_pPolyhedronRemesh->compute_edge_start_points(&pos_edge_out_start_points);
    }
    if (m_view_edge_out_end_points) {
      m_pPolyhedronRemesh->compute_edge_end_points(&pos_edge_out_end_points);
    }
    if (m_view_edge_out_links) {
      m_pPolyhedronRemesh->compute_edge_links(&pos_edge_out_links);
    }
    // vertex out links
    if (m_view_vertex_out_start_points) {
      m_pPolyhedronRemesh->compute_vertex_start_points(&pos_vertex_out_start_points);
    }
    if (m_view_vertex_out_end_points) {
      m_pPolyhedronRemesh->compute_vertex_end_points(&pos_vertex_out_end_points);
    }
    if (m_view_vertex_out_links) {
      m_pPolyhedronRemesh->compute_vertex_links(&pos_vertex_out_links);
    }
  }
}

bool Scene::open(QString file_name, bool is_surface_mesh) {
  QTextStream cerr(stderr);
  cerr << QString("Opening file \"%1\"\n").arg(file_name);
  cerr.flush();

  QFileInfo file_info(file_name);
  std::ifstream in(file_name.toUtf8());
  if (!file_info.isFile() || !file_info.isReadable() || !in) {
    std::cerr << "unable to open file" << std::endl;
    return false;
  }

  if (is_surface_mesh) {        // open Surface_mesh
    if (m_pSurfacemeshInput != NULL) {
      delete m_pSurfacemeshInput;
    }
    if (m_pSurfacemeshRemesh != NULL) {
      delete m_pSurfacemeshRemesh;
    }
    m_pSurfacemeshInput = new Mesh;
    in >> *m_pSurfacemeshInput;
    if (!in) {
      std::cerr << "invalid OFF file" << std::endl;
      delete m_pSurfacemeshInput;
      m_pSurfacemeshInput = NULL;
      m_pSurfacemeshRemesh = NULL;
      return false;
    }
    in.close();
    // TODO: do we need the normalization?
    // m_pPolyhedronInput->normalize(1.0);
    m_input_fnormals = m_pSurfacemeshInput->add_property_map
      <face_descriptor, Vector_3>("f:normals", CGAL::NULL_VECTOR).first;
    CGAL::Polygon_mesh_processing::compute_face_normals(*m_pSurfacemeshInput, 
      m_input_fnormals, 
      CGAL::Polygon_mesh_processing::parameters::vertex_point_map(
      m_pSurfacemeshInput->points()).geom_traits(Kernel()));
    update_bbox();
    m_pSurfacemeshRemesh = new Mesh(*m_pSurfacemeshInput);
    m_remesh_fnormals = m_pSurfacemeshRemesh->add_property_map
      <face_descriptor, Vector_3>("f:normals", CGAL::NULL_VECTOR).first;
    CGAL::Polygon_mesh_processing::compute_face_normals(*m_pSurfacemeshRemesh,
      m_remesh_fnormals,
      CGAL::Polygon_mesh_processing::parameters::vertex_point_map(
      m_pSurfacemeshRemesh->points()).geom_traits(Kernel()));
    double average_length = calculate_average_length(*m_pSurfacemeshRemesh);
    if (average_length > 0.0) {
      m_target_edge_length = average_length;
    }
    reset_draw_render_types();
    m_view_surfacemesh_input = false;
    m_view_surfacemesh_remesh = true;
    m_view_polyhedron_input = false;
    m_view_polyhedron_remesh = false;

    // test
    /*std::cout << "remesh normals:" << std::endl;
    for (face_descriptor fd : faces(*m_pSurfacemeshRemesh)) {
    std::cout << "normal info: " << m_remesh_fnormals[fd] << std::endl;
    }
    std::cout << "vertex points: " << std::endl;
    Mesh::Property_map<vertex_descriptor, Point> location = m_pSurfacemeshRemesh->points();
    Bbox bb;
    BOOST_FOREACH(vertex_descriptor vd, m_pSurfacemeshRemesh->vertices()) {
    std::cout << "vertex info: " << location[vd] << std::endl;
    }*/
  }
  else {                        // open Polyhedron_3
    if (m_pPolyhedronInput != NULL) {
      delete m_pPolyhedronInput;
    }
    if (m_pPolyhedronRemesh != NULL) {
      delete m_pPolyhedronRemesh;
    }
    m_pPolyhedronInput = new Polyhedron;
    in >> *m_pPolyhedronInput;
    if (!in) {
      std::cerr << "invalid OFF file" << std::endl;
      delete m_pPolyhedronInput;
      m_pPolyhedronInput = NULL;
      m_pPolyhedronRemesh = NULL;
      return false;
    }
    in.close();
    m_pPolyhedronInput->normalize(1.0);
    m_pPolyhedronInput->calculate_normals("Input");
    old_remesh.calculate_feature_intensities("Input", m_pPolyhedronInput);
    update_bbox();
    m_pPolyhedronRemesh = new Polyhedron(*m_pPolyhedronInput);
    m_input_aabb_tree_constructed = false;
    m_links_initialized = false;
    old_remesh.initialize_private_data();
    reset_draw_render_types();
    m_view_polyhedron_input = false;
    m_view_polyhedron_remesh = true;
  }

  changed();
  return true;
}

bool Scene::open_input(QString file_name, bool is_surface_mesh) {
  QTextStream cerr(stderr);
  cerr << QString("Opening input file \"%1\"\n").arg(file_name);
  cerr.flush();

  QFileInfo fileinfo(file_name);
  std::ifstream in(file_name.toUtf8());
  if (!in || !fileinfo.isFile() || !fileinfo.isReadable()) {
    std::cerr << "unable to open input file" << std::endl;
    return false;
  }

  if (m_pPolyhedronInput != NULL) {
    delete m_pPolyhedronInput;
  }
  m_pPolyhedronInput = new Polyhedron;
  in >> *m_pPolyhedronInput;
  if (!in) {
    std::cerr << "invalid OFF file" << std::endl;
    delete m_pPolyhedronInput;
    m_pPolyhedronInput = NULL;
    return false;
  }
  in.close();

  m_pPolyhedronInput->normalize(1.0);
  m_pPolyhedronInput->calculate_normals("Input");
  old_remesh.calculate_feature_intensities("Input", m_pPolyhedronInput);
  update_bbox();
  m_input_aabb_tree_constructed = false;

  if (m_pPolyhedronRemesh == NULL) {
    m_pPolyhedronRemesh = new Polyhedron(*m_pPolyhedronInput);
  }
  old_remesh.clear_links(m_pPolyhedronInput, m_pPolyhedronRemesh);
  m_links_initialized = false;
  old_remesh.initialize_private_data();

  reset_draw_render_types();
  m_view_polyhedron_input = true;
  m_view_polyhedron_remesh = false;

  changed();
  return true;
}

bool Scene::open_remesh(QString file_name, bool is_surface_mesh) {
  QTextStream cerr(stderr);
  cerr << QString("Opening remesh file \"%1\"\n").arg(file_name);
  cerr.flush();

  QFileInfo fileinfo(file_name);
  std::ifstream in(file_name.toUtf8());
  if (!in || !fileinfo.isFile() || !fileinfo.isReadable()) {
    std::cerr << "unable to open remesh file" << std::endl;
    return false;
  }

  if (m_pPolyhedronRemesh != NULL) {
    delete m_pPolyhedronRemesh;
  }
  m_pPolyhedronRemesh = new Polyhedron;
  in >> *m_pPolyhedronRemesh;
  if (!in) {
    std::cerr << "invalid OFF file" << std::endl;
    delete m_pPolyhedronRemesh;
    m_pPolyhedronRemesh = NULL;
    return false;
  }
  in.close();

  m_pPolyhedronRemesh->calculate_normals("Remesh");
  old_remesh.calculate_feature_intensities("Remesh", m_pPolyhedronRemesh);
  if (m_pPolyhedronInput == NULL) {
    m_pPolyhedronRemesh->normalize(1.0);
    m_pPolyhedronInput = new Polyhedron(*m_pPolyhedronRemesh);
    m_input_aabb_tree_constructed = false;
    update_bbox();
  }
  old_remesh.clear_links(m_pPolyhedronInput, m_pPolyhedronRemesh);
  m_links_initialized = false;
  old_remesh.initialize_private_data();

  reset_draw_render_types();
  m_view_polyhedron_input = false;
  m_view_polyhedron_remesh = true;

  changed();
  return true;
}

void Scene::save_remesh_as(QString file_name) {
  if (m_pPolyhedronRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    m_pPolyhedronRemesh->save_as(file_name.toStdString());
  }
}

int Scene::get_optimize_type_index(OptimizeType ot) const {
  switch (ot) {
  case OptimizeType::k_none:
    return 0;
  case OptimizeType::k_input_to_remesh:
    return 1;
  case OptimizeType::k_remesh_to_input:
    return 2;
  case OptimizeType::k_both:
    return 3;
  default:
    return -1;    // invalid case
  }
}

OptimizeType Scene::get_optimize_type(int index) const {
  switch (index) {
  case 0:
    return OptimizeType::k_none;
  case 1:
    return OptimizeType::k_input_to_remesh;
  case 2:
    return OptimizeType::k_remesh_to_input;
  case 3:
    return OptimizeType::k_both;
  default:
    return OptimizeType::k_none;
  }
}

void Scene::toggle_view_input() {
  m_view_surfacemesh_input = !m_view_surfacemesh_input;
  m_view_polyhedron_input = !m_view_polyhedron_input;
  if (m_view_surfacemesh_input || m_view_polyhedron_input) {
    changed();
  }
}

void Scene::toggle_view_remesh() { 
  m_view_surfacemesh_remesh = !m_view_surfacemesh_remesh;
  m_view_polyhedron_remesh = !m_view_polyhedron_remesh; 
  if (m_view_surfacemesh_remesh || m_view_polyhedron_remesh) {
    changed();
  }
}

void Scene::toggle_view_input_remesh() {
  m_view_surfacemesh_remesh = !m_view_surfacemesh_remesh;
  m_view_surfacemesh_input = !m_view_surfacemesh_remesh;
  m_view_polyhedron_remesh = !m_view_polyhedron_remesh;
  m_view_polyhedron_input = !m_view_polyhedron_remesh;
  if (m_view_surfacemesh_input || m_view_surfacemesh_remesh ||
      m_view_polyhedron_input || m_view_polyhedron_remesh) {
    changed();
  }
}

void Scene::toggle_view_minimal_angle() {
  m_view_minimal_angle = !m_view_minimal_angle;
  if (m_view_minimal_angle) {
    changed();
  }
}

void Scene::toggle_view_polyhedron_edges() {
  m_view_polyhedron_edges = !m_view_polyhedron_edges;
  if (m_view_polyhedron_edges) {
    changed();
  }
}

void Scene::toggle_view_polyhedron_facets() {
  reset_draw_render_types();
  changed();
}

void Scene::toggle_view_interpolated_feature_intensities() {
  set_draw_render_types(k_polyhedron, k_ifi_facets);
  changed();
}

void Scene::toggle_view_facet_errors() {
  set_draw_render_types(k_polyhedron, k_mr_facets);
  changed();
}

void Scene::toggle_view_element_classifications() {
  set_draw_render_types(k_vertex_voronoi, k_classifications);
  changed();
}

void Scene::toggle_view_gaussian_curvatures() {
  set_draw_render_types(k_vertex_voronoi, k_gaussian_curvature);
  changed();
}

void Scene::toggle_view_maximal_normal_dihedrals() {
  set_draw_render_types(k_vertex_voronoi, k_maximal_halfedge_dihedral);
  changed();
}

void Scene::toggle_view_normal_dihedrals() {
  set_draw_render_types(k_edge_voronoi, k_normal_dihedral);
  changed();
}

void Scene::toggle_view_facet_in_start_points() {
  m_view_facet_in_start_points = !m_view_facet_in_start_points;
  if (m_view_facet_in_start_points) {
    changed();
  }
}

void Scene::toggle_view_facet_in_end_points() {
  m_view_facet_in_end_points = !m_view_facet_in_end_points;
  if (m_view_facet_in_end_points) {
    changed();
  }
}

void Scene::toggle_view_facet_in_links() {
  m_view_facet_in_links = !m_view_facet_in_links;
  if (m_view_facet_in_links) {
    changed();
  }
  changed();
}

void Scene::toggle_view_facet_out_start_points() {
  m_view_facet_out_start_points = !m_view_facet_out_start_points;
  if (m_view_facet_out_start_points) {
    changed();
  }
}

void Scene::toggle_view_facet_out_end_points() {
  m_view_facet_out_end_points = !m_view_facet_out_end_points;
  if (m_view_facet_out_end_points) {
    changed();
  }
}

void Scene::toggle_view_facet_out_links() {
  m_view_facet_out_links = !m_view_facet_out_links;
  if (m_view_facet_out_links) {
    changed();
  }
}

void Scene::toggle_view_edge_in_start_points() {
  m_view_edge_in_start_points = !m_view_edge_in_start_points;
  if (m_view_edge_in_start_points) {
    changed();
  }
}

void Scene::toggle_view_edge_in_end_points() {
  m_view_edge_in_end_points = !m_view_edge_in_end_points;
  if (m_view_edge_in_end_points) {
    changed();
  }
}

void Scene::toggle_view_edge_in_links() {
  m_view_edge_in_links = !m_view_edge_in_links;
  if (m_view_edge_in_links) {
    changed();
  }
}

void Scene::toggle_view_edge_out_start_points() {
  m_view_edge_out_start_points = !m_view_edge_out_start_points;
  if (m_view_edge_out_start_points) {
    changed();
  }
}

void Scene::toggle_view_edge_out_end_points() {
  m_view_edge_out_end_points = !m_view_edge_out_end_points;
  if (m_view_edge_out_end_points) {
    changed();
  }
}

void Scene::toggle_view_edge_out_links() {
  m_view_edge_out_links = !m_view_edge_out_links;
  if (m_view_edge_out_links) {
    changed();
  }
}

void Scene::toggle_view_vertex_in_start_points() {
  m_view_vertex_in_start_points = !m_view_vertex_in_start_points;
  if (m_view_vertex_in_start_points) {
    changed();
  }
}

void Scene::toggle_view_vertex_in_end_points() {
  m_view_vertex_in_end_points = !m_view_vertex_in_end_points;
  if (m_view_vertex_in_end_points) {
    changed();
  }
}

void Scene::toggle_view_vertex_in_links() {
  m_view_vertex_in_links = !m_view_vertex_in_links;
  if (m_view_vertex_in_links) {
    changed();
  }
}

void Scene::toggle_view_vertex_out_start_points() {
  m_view_vertex_out_start_points = !m_view_vertex_out_start_points;
  if (m_view_vertex_out_start_points) {
    changed();
  }
}

void Scene::toggle_view_vertex_out_end_points() {
  m_view_vertex_out_end_points = !m_view_vertex_out_end_points;
  if (m_view_vertex_out_end_points) {
    changed();
  }
}

void Scene::toggle_view_vertex_out_links() {
  m_view_vertex_out_links = !m_view_vertex_out_links;
  if (m_view_vertex_out_links) {
    changed();
  }
}

void Scene::toggle_view_all_sample_feature_intensities() {  // all samples
  set_draw_render_types(k_all_voronoi, k_feature_intensity);
  changed();
}

void Scene::toggle_view_all_sample_capacities() {
  set_draw_render_types(k_all_voronoi, k_capacity);
  changed();
}

void Scene::toggle_view_all_sample_weights() {
  set_draw_render_types(k_all_voronoi, k_weight);
  changed();
}

void Scene::toggle_view_vertex_feature_intensities() {      // vertex samples
  set_draw_render_types(k_vertex_voronoi, k_feature_intensity);
  changed();
}

void Scene::toggle_view_vertex_capacities() {
  set_draw_render_types(k_vertex_voronoi, k_capacity);
  changed();
}

void Scene::toggle_view_vertex_weights() {
  set_draw_render_types(k_vertex_voronoi, k_weight);
  changed();
}

void Scene::toggle_view_edge_feature_intensities() {       // edge samples
  set_draw_render_types(k_edge_voronoi, k_feature_intensity);
  changed();
}

void Scene::toggle_view_edge_capacities() {
  set_draw_render_types(k_edge_voronoi, k_capacity);
  changed();
}

void Scene::toggle_view_edge_weights() {
  set_draw_render_types(k_edge_voronoi, k_weight);
  changed();
}

void Scene::toggle_view_facet_feature_intensities() {      // facet samples
  set_draw_render_types(k_facet_voronoi, k_feature_intensity);
  changed();
}

void Scene::toggle_view_facet_capacities() {
  set_draw_render_types(k_facet_voronoi, k_capacity);
  changed();
}

void Scene::toggle_view_facet_weights() {
  set_draw_render_types(k_facet_voronoi, k_weight);
  changed();
}

void Scene::eliminate_degenerations() {
  if (m_pPolyhedronInput == NULL) {
    std::cout << "Please open a file first" << std::endl;
    return;
  }
  CGAL::Timer timer;
  timer.start();
  std::cout << std::endl << "Eliminate Input degenerated facets...";
  size_t nb_eliminations =
    old_remesh.eliminate_degenerated_facets(m_pPolyhedronInput);
  std::cout << "Done (" << nb_eliminations << " facets eliminated, "
    << timer.time() << " s)" << std::endl;
  if (nb_eliminations > 0) {
    m_pPolyhedronInput->calculate_normals("Input");
    old_remesh.calculate_feature_intensities("Input", m_pPolyhedronInput);
    m_input_aabb_tree_constructed = false;
    reset();
    reset_draw_render_types();
    changed();
  }
}

void Scene::split_input_long_edges() {
  if (m_pPolyhedronInput == NULL) {
    std::cout << "Please open a file first" << std::endl;
    return;
  }
  CGAL::Timer timer;
  timer.start();
  std::cout << std::endl << "Split long edges for Input...";
  size_t nb_split = old_remesh.split_long_edges(m_pPolyhedronInput);
  std::cout << "Done (" << nb_split << " edges splited, "
    << timer.time() << " s)" << std::endl;
  if (nb_split > 0) {
    m_pPolyhedronInput->calculate_normals("Input");
    old_remesh.calculate_feature_intensities("Input", m_pPolyhedronInput);
    m_input_aabb_tree_constructed = false;
    reset();    // do we not need clear links because m_pPolyhedronRemesh is reset
    reset_draw_render_types();
    changed();
  }
}

void Scene::input_properties() {
  if (m_pPolyhedronInput == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    m_pPolyhedronInput->trace_properties("Input");
  }
}

void Scene::split_border() {
  if (m_pSurfacemeshRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    CGAL::Timer timer;
    timer.start();
    std::cout << std::endl << "Split boder...";
    std::vector<edge_descriptor> border;
    PMP::border_halfedges(faces(*m_pSurfacemeshRemesh), *m_pSurfacemeshRemesh,
        boost::make_function_output_iterator(halfedge2edge(
        *m_pSurfacemeshRemesh, border)));
    PMP::split_long_edges(border, m_target_edge_length, *m_pSurfacemeshRemesh);
    CGAL::Polygon_mesh_processing::compute_face_normals(*m_pSurfacemeshRemesh,
      m_remesh_fnormals,
      CGAL::Polygon_mesh_processing::parameters::vertex_point_map(
      m_pSurfacemeshRemesh->points()).geom_traits(Kernel()));
    std::cout << "Done (" << timer.time() << " s)" << std::endl;
    reset_draw_render_types();
    changed();
  }
}

void Scene::isotropic_remeshing() {
  if (m_pSurfacemeshRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    CGAL::Timer timer;
    timer.start();
    std::cout << std::endl << "Isotropic remeshing...";
    PMP::isotropic_remeshing(
      faces(*m_pSurfacemeshRemesh),
      m_target_edge_length,
      *m_pSurfacemeshRemesh,
      PMP::parameters::number_of_iterations(m_smooth_iteration_count)
      .protect_constraints(true)//i.e. protect border, here
      );
    CGAL::Polygon_mesh_processing::compute_face_normals(
      *m_pSurfacemeshRemesh,
      m_remesh_fnormals,
      CGAL::Polygon_mesh_processing::parameters::vertex_point_map(
      m_pSurfacemeshRemesh->points()).geom_traits(Kernel()));
    std::cout << "Done (" << timer.time() << " s)" << std::endl;
    reset_draw_render_types();
    changed();
  }
}

void Scene::old_remesh_reset_from_input() {
  if (m_pPolyhedronInput == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    CGAL::Timer timer;
    timer.start();
    std::cout << std::endl << "Resetting from Input...";
    reset();
    std::cout << "Done (" << timer.time() << " s)" << std::endl;
    reset_draw_render_types();
    changed();
  }
}

void Scene::old_remesh_generate_links_and_types() {
  if (m_pPolyhedronRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
    return;
  }
  else {
    generate();
    m_links_initialized = true;
    changed();
  }
}

void Scene::old_remesh_properties() {
  if (m_pPolyhedronRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    if (!m_links_initialized) {
      // special usage: we update inside generate_links_and_types()
      old_remesh_generate_links_and_types();
    }
    m_pPolyhedronRemesh->trace_properties("Remesh");  // basic properties
    m_pPolyhedronRemesh->trace_additional_properties(m_pPolyhedronInput->get_diagonal_length());
  }
}

void Scene::old_remeshing() {
  if (m_pPolyhedronRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    if (!m_links_initialized) {
      generate();
      m_links_initialized = true;
    }
    std::cout << std::endl;
    old_remesh.isotropic_remeshing(m_input_facet_tree, m_bbox, m_pPolyhedronRemesh);
    changed();
  }
}

void Scene::old_remesh_initial_mesh_simplification() {
  if (m_pPolyhedronRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    if (!m_links_initialized) {
      generate();
      m_links_initialized = true;
    }
    std::cout << std::endl;
    old_remesh.initial_mesh_simplification(m_input_facet_tree,
                                                m_bbox, m_pPolyhedronRemesh);
    reset_draw_render_types();
    changed();
  }
}

void Scene::old_remesh_split_local_longest_edge() {
  if (m_pPolyhedronRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    if (!m_links_initialized) {
      generate();
      m_links_initialized = true;
    }
    std::cout << std::endl;
    old_remesh.split_local_longest_edge(m_input_facet_tree,
                                             m_bbox, m_pPolyhedronRemesh);
    reset_draw_render_types();
    changed();
  }
}

void Scene::old_remesh_increase_minimal_angle() {
  if (m_pPolyhedronRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    if (!m_links_initialized) {
      generate();
      m_links_initialized = true;
    }
    std::cout << std::endl;
    old_remesh.increase_minimal_angle(m_input_facet_tree,
                                           m_bbox, m_pPolyhedronRemesh);
    reset_draw_render_types();
    changed();
  }
}

void Scene::old_remesh_maximize_minimal_angle() {
  if (m_pPolyhedronRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    if (!m_links_initialized) {
      generate();
      m_links_initialized = true;
    }
    std::cout << std::endl;
    old_remesh.maximize_minimal_angle(m_input_facet_tree, 
                                           m_bbox, m_pPolyhedronRemesh);
    reset_draw_render_types();
    changed();
  }
}

void Scene::old_remesh_final_vertex_relocation() {
  if (m_pPolyhedronRemesh == NULL) {
    std::cout << "Please open a file first" << std::endl;
  }
  else {
    if (!m_links_initialized) {
      generate();
      m_links_initialized = true;
    }
    std::cout << std::endl;
    old_remesh.final_vertex_relocation(m_input_facet_tree, 
                                            m_bbox, m_pPolyhedronRemesh);
    reset_draw_render_types();
    changed();
  }
}

void Scene::update_feature_intensities_and_clear_links() {
  if (m_pPolyhedronInput != NULL && m_pPolyhedronRemesh != NULL) {
    // step 1: update feature intensities
    old_remesh.calculate_feature_intensities("Input", m_pPolyhedronInput);
    old_remesh.calculate_feature_intensities("Remesh", m_pPolyhedronRemesh);
    // step 2: clear the links
    old_remesh.clear_links(m_pPolyhedronInput, m_pPolyhedronRemesh);
    m_links_initialized = false;
    old_remesh.initialize_private_data();
    changed();
  }
}

void Scene::set_draw_render_types(DrawType draw_type, RenderType render_type) {
  m_draw_type = draw_type;
  m_render_type = render_type;
}

void Scene::reset_draw_render_types() {
  set_draw_render_types(k_polyhedron, k_plain_facets);
}

void Scene::reset() {
  if (m_pPolyhedronInput != NULL) {
    if (m_pPolyhedronRemesh != NULL) {
      delete m_pPolyhedronRemesh;
    }
    m_pPolyhedronRemesh = new Polyhedron(*m_pPolyhedronInput);
    old_remesh.clear_links(m_pPolyhedronInput, m_pPolyhedronRemesh);
    m_links_initialized = false;
    old_remesh.initialize_private_data();
  }
}

void Scene::generate() {
  if (m_pPolyhedronInput != NULL && m_pPolyhedronRemesh != NULL) {
    std::cout << std::endl;
    if (!m_input_aabb_tree_constructed) {
      old_remesh.build_facet_tree(*m_pPolyhedronInput, "Input", 
                                       &m_input_facet_tree);
      m_input_aabb_tree_constructed = true;
    }
    old_remesh.build_facet_tree(*m_pPolyhedronRemesh, "Remesh", 
                                     &m_remesh_facet_tree);
    old_remesh.generate_links(m_input_facet_tree, m_remesh_facet_tree,
                                   m_pPolyhedronInput, m_pPolyhedronRemesh);
  }
}

double Scene::calculate_average_length(const Mesh &mesh) const {
  double sum_length = 0.0;
  for (Mesh::Edge_range::iterator e = mesh.edges().begin();
      e != mesh.edges().end(); ++e) {
    const Point &source = mesh.point(mesh.source(mesh.halfedge(*e)));
    const Point &target = mesh.point(mesh.target(mesh.halfedge(*e)));
    sum_length += CGAL::sqrt(CGAL::squared_distance(source, target));
  }
  if (mesh.number_of_edges() == 0) {
    return -1.0;    // invalid case
  }
  else {
    return sum_length / mesh.number_of_edges();
  }
}

void Scene::compute_faces(const Mesh &mesh, bool is_input, 
                          std::vector<float> *pos, std::vector<float> *normals,
                          std::vector<float> *colors) {
  pos->clear();
  normals->clear();
  colors->clear();
  for (Mesh::Face_range::iterator f = mesh.faces().begin(); 
      f != mesh.faces().end(); ++f) {
    if (*f != boost::graph_traits<Mesh>::null_face()) {
      compute_face(mesh, is_input, *f, pos, normals, colors);
    }
  }
}

void Scene::compute_edges(const Mesh &mesh, std::vector<float> *pos) {
  pos->clear();
  for (Mesh::Edge_range::iterator e = mesh.edges().begin(); 
      e != mesh.edges().end(); ++e) {
    compute_edge(mesh, *e, pos);
  }
}

void Scene::compute_face(const Mesh &mesh, bool is_input, face_descriptor fh,
    std::vector<float> *pos, std::vector<float> *normals, 
    std::vector<float> *colors) {
  Color color;
  Vector_3 normal;
  if (is_input) {
    color = Color(150, 150, 200);
    normal = m_input_fnormals[fh];
  }
  else {
    color = Color(200, 150, 150);
    normal = m_remesh_fnormals[fh];
  }
  halfedge_descriptor hd = mesh.halfedge(fh);
  do {
    const Point &point = mesh.point(mesh.source(hd));
    compute_point(point, pos);
    compute_normal(normal, normals);
    compute_color(color, colors);
    hd = mesh.next(hd);
  } while (hd != mesh.halfedge(fh));
}

void Scene::compute_edge(const Mesh &mesh, edge_descriptor ef,
                         std::vector<float> *pos) {
  const Point &source = mesh.point(mesh.source(mesh.halfedge(ef)));
  const Point &target = mesh.point(mesh.target(mesh.halfedge(ef)));
  compute_point(source, pos);
  compute_point(target, pos);
}

void inline Scene::compute_point(const Point &point, std::vector<float> *pos) {
  pos->push_back(point.x());
  pos->push_back(point.y());
  pos->push_back(point.z());
}

void inline Scene::compute_normal(const Vector_3 &normal, 
                                  std::vector<float> *pos) {
  pos->push_back(normal.x());
  pos->push_back(normal.y());
  pos->push_back(normal.z());
}

void inline Scene::compute_color(const Color &color, std::vector<float> *pos) {
  pos->push_back(color.red() / 255.0f);
  pos->push_back(color.green() / 255.0f);
  pos->push_back(color.blue() / 255.0f);
}
