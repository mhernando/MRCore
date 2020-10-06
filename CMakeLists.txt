cmake_minimum_required(VERSION 3.0)
project (MRCore )
set (MRCore_VERSION_MAJOR 2)
set (MRCore_VERSION_MINOR 0)
 
MESSAGE(STATUS "MRCORE: Checking time functions")
IF(NOT WIN32)
	include (CheckFunctionExists)
	check_function_exists(gettimeofday HAVE_GETTIMEOFDAY)
	check_function_exists(clock_gettime HAVE_CLOCK_GETTIME)
	IF(HAVE_GETTIMEOFDAY)
		ADD_DEFINITIONS(-DHAVE_GETTIMEOFDAY)
	ENDIF()
	IF(HAVE_CLOCK_GETTIME)
		ADD_DEFINITIONS(-DHAVE_CLOCK_GETTIME)
	ENDIF()
ENDIF()

SET(curdir ${CMAKE_CURRENT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${curdir}/include) 


####################################################################################################
# Iterate over children, creating SOURCE_GROUPs 
####################################################################################################
FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
SET(SOURCES "") 
SET(HEADERS "")
FOREACH(child ${children})
IF(IS_DIRECTORY ${curdir}/${child})
    FILE(GLOB_RECURSE child_sources ${curdir}/${child}/*.cpp) 
    FILE(GLOB_RECURSE child_headers ${curdir}/${child}/*.h)
    SET(SOURCES ${SOURCES} ${child_sources})
    SET(HEADERS ${HEADERS} ${child_headers})
    SOURCE_GROUP(${child} FILES ${child_sources} ${child_headers}) 
ENDIF()
ENDFOREACH()


# MR target library
IF (WIN32)	
	ADD_LIBRARY(mrcore STATIC ${SOURCES} ${HEADERS} )
	SET_TARGET_PROPERTIES(mrcore PROPERTIES DEBUG_POSTFIX "_d")
ELSE(WIN32)#LINUX
	find_package(OpenGL REQUIRED COMPONENTS OpenGL)
	ADD_LIBRARY(mrcore SHARED ${SOURCES})
	TARGET_LINK_LIBRARIES(mrcore  ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${JPEG_LIBRARIES}  rt)
ENDIF(WIN32)
target_include_directories(mrcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
	
########### To define a default install directory in Windows
IF (WIN32)
	SET(CMAKE_INSTALL_PREFIX "C:/mrcore" CACHE PATH "FOO install prefix" FORCE)
ELSE(WIN32)
	SET(CMAKE_INSTALL_PREFIX /usr/local/mrcore) 
ENDIF(WIN32)
MESSAGE(STATUS "Default install directory is: " ${CMAKE_INSTALL_PREFIX}) 

########### INSTALL PROJECT
INSTALL(TARGETS mrcore DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
FILE(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
INSTALL(FILES ${files} DESTINATION include)
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/ DESTINATION include
          FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
		  
MESSAGE(STATUS "mrcore lib will be installed on ${CMAKE_INSTALL_PREFIX}/lib")
	

#OPTIONS 
#LOCAL GENERATION OF HEADERS... only for develop
option(CREATE_HEADERS "Create headers, automatically from dir tree.  " OFF)
if(CREATE_HEADERS)
 message("headers update: ...")
 include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/update_headers.cmake)
endif() 
#OPTION :GENERATE EXAMPLE PROJECTS
option(CREATE_EXAMPLES "Create example projects  " OFF)
if(CREATE_EXAMPLES)
 message("Examples Project Generation")
 ADD_SUBDIRECTORY(examples)
endif(CREATE_EXAMPLES ) 
