
SET(GLUT_FOUND 1)
SET(GLUT_LIBRARIES glut32)
SET(OPENGL_LIBRARIES opengl32 glu32 )
FILE(GLOB path_headers 	*.h)
FILE(GLOB path_sources 	*.cpp)
		ADD_EXECUTABLE(main_planner ${path_sources} ${path_headers} )
		IF(WIN32)
			IF(DEBUG)
				TARGET_LINK_LIBRARIES(main_planner mrcore_d  ${GLUT_LIBRARIES})
			ELSE(DEBUG)
				TARGET_LINK_LIBRARIES(main_planner mrcore  ${GLUT_LIBRARIES})
			ENDIF(DEBUG)
		ELSE(WIN32) #LINUX	
			TARGET_LINK_LIBRARIES(main_planner mrcore ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} )
		ENDIF(WIN32)
		




