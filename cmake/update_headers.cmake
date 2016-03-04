#MACRO FOR AUTOMATE THE CREATION OF MRCORE HEADERS
FUNCTION(CREATE_MR_HEADER name headers)
SET(CONTENT_H "#ifndef __MR${name}_H__\n")
SET(CONTENT_H ${CONTENT_H} "#define __MR${name}_H__\n")
FOREACH(header ${headers})
	SET(CONTENT_H ${CONTENT_H} "#include \"" ${header} "\"\n")
ENDFOREACH()
SET(CONTENT_H ${CONTENT_H} "#endif // __MR${name}_H__\n")
FILE(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/include/mr${name}.h" ${CONTENT_H})
ENDFUNCTION()

SET(curdir2 ${CMAKE_CURRENT_SOURCE_DIR}/src)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include")
MESSAGE( "The source directory to create headers is:" ${curdir2})
MESSAGE( "(DEV ONLY)The updated automatic headers are placed at: ${CMAKE_CURRENT_SOURCE_DIR}/include")
FILE(GLOB children RELATIVE ${curdir2} ${curdir2}/*)
SET(HEADERS "")
SET(MRCORE_DIRS)
FOREACH(child ${children})
IF(IS_DIRECTORY ${curdir2}/${child})
	
    FILE(GLOB_RECURSE child_headers RELATIVE ${curdir2} ${curdir2}/${child}/*.h)
	#MESSAGE("CREATE_MR_HEADER(${child} ${child_headers})")
	if(child_headers)
	 list(APPEND MRCORE_DIRS ${child})
	 CREATE_MR_HEADER(${child} "${child_headers}")
	endif()
ENDIF()
ENDFOREACH()

SET(MRCORE_H "#ifndef __MRCORE_H__\n")
SET(MRCORE_H ${MRCORE_H} "#define __MRCORE_H__\n")

FOREACH(mrdirs ${MRCORE_DIRS})
SET(MRCORE_H ${MRCORE_H} "#include \"" mr${mrdirs}.h "\"\n")
ENDFOREACH()
SET(MRCORE_H ${MRCORE_H} "#endif // __MRCORE_H__\n")
FILE(WRITE "${CMAKE_CURRENT_SOURCE_DIR}/include/mrcore.h" ${MRCORE_H})



