include_directories ("${PROJECT_SOURCE_DIR}/src")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(MATHGL2_LIBRARIES libmgl-glut.so libmgl.so)
set(MATHGL2_INCLUDE_DIRS /usr/include/mgl2/)
include_directories(${MATHGL2_INCLUDE_DIRS})
#LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/res)



add_executable(test_0 main.cpp)
target_link_libraries(test_0 CRRT ${MATHGL2_LIBRARIES})