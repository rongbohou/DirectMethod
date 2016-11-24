cmake_minimum_required( VERSION 2.8 )

set( CMAKE_CXX_FLAGS "-std=c++11 -Wall -O2 -march=native" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )

find_package( G2O REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
find_package( Cholmod )
#FIND_PACKAGE(GLFW3)
#FIND_PACKAGE(OpenGL)
FIND_PACKAGE( CSparse REQUIRED)
# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)

include_directories( ${EIGEN3_INCLUDE_DIR} 
                     ${CHOLMOD_INCLUDE_DIR}
                     ${CSPARSE_INCLUDE_DIR} 
                     ${G2O_INCLUDE_DIR}
                     ${PCL_INCLUDE_DIRS} 
                     )

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

ADD_LIBRARY(EdgeDirect  edge_se3expmap_direct.cpp )
TARGET_LINK_LIBRARIES( EdgeDirect 
    ${OpenCV_LIBS} 
    ${G2O_LIBS} )

ADD_EXECUTABLE(bilinear_interpolation  bilinear_interpolation.cpp )
TARGET_LINK_LIBRARIES(bilinear_interpolation
    ${OpenCV_LIBS}  )

ADD_EXECUTABLE(direct_sparse  direct_sparse.cpp )
TARGET_LINK_LIBRARIES(direct_sparse
    EdgeDirect
    ${OpenCV_LIBS}
    ${G2O_LIBS}   )

ADD_EXECUTABLE(direct_semidense  direct_semidense.cpp )
TARGET_LINK_LIBRARIES(direct_semidense
    EdgeDirect
    ${OpenCV_LIBS}
    ${G2O_LIBS}   )

ADD_EXECUTABLE(direct_sparse_gx  direct_sparse_gx.cpp )
TARGET_LINK_LIBRARIES(direct_sparse_gx
    ${OpenCV_LIBS}
    g2o_core g2o_types_sba g2o_solver_csparse g2o_stuff g2o_csparse_extension  )
