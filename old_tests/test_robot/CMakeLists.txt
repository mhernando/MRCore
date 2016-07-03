#CMakeList.txt test ScaraSim

cmake_minimum_required (VERSION 2.8)

SET(GLUT_FOUND 1)
IF(WIN32)
	SET(GLUT_LIBRARIES glut32)
	SET(OPENGL_LIBRARIES opengl32 glu32)
ELSE(WIN32)
	SET(GLUT_LIBRARIES glut)
	SET(OPENGL_LIBRARIES GLU GL)
ENDIF(WIN32)

ADD_EXECUTABLE(testROBOT testrobot.cpp adeptonesim.cpp robotsim.cpp arm.cpp puma560sim.cpp tool.cpp arm.h tool.h puma560sim.h adeptonesim.h robotsim.h)

IF(WIN32)
	IF(DEBUG)
		TARGET_LINK_LIBRARIES(testROBOT mrcore_d  ${GLUT_LIBRARIES})
	ELSE(DEBUG)
		TARGET_LINK_LIBRARIES(testROBOT mrcore  ${GLUT_LIBRARIES})
	ENDIF(DEBUG)
	ELSE(WIN32) #LINUX
		TARGET_LINK_LIBRARIES(testROBOT mrcore ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES})
	ENDIF(WIN32)
		
	
		




