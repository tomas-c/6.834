include_directories ("${PROJECT_SOURCE_DIR}/src")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(BULLET_INCLUDE_DIRS "/usr/include/bullet" )
include_directories(
 ${BULLET_INCLUDE_DIRS}
)

add_executable(test_2 
	main.cpp
	#CollisionChecker.cpp
	tiny_obj_loader.cpp
)
target_link_libraries(test_2 
	CRRT 
	${MATHGL2_LIBRARIES} 
	BulletDynamics BulletCollision LinearMath
)