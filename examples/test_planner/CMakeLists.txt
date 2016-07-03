
FILE(GLOB path_headers 	*.h)
FILE(GLOB path_sources 	*.cpp)
ADD_EXECUTABLE(test_planner ${path_sources} ${path_headers} )
MR_GLUT_EXAMPLE(test_planner)
		




