ADD_EXECUTABLE(worldTest worldtest.cpp)

ADD_EXECUTABLE(gluttestMHGtransformation gluttestMHGtransformation.cpp )
ADD_EXECUTABLE(mainMHGtests mainMHGtests.cpp )
MR_GLUT_EXAMPLE(worldTest )

MR_GLUT_EXAMPLE(mainMHGtests)
MR_GLUT_EXAMPLE(gluttestMHGtransformation)	

