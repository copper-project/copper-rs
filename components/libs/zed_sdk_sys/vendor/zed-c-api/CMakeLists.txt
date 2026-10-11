cmake_minimum_required(VERSION 3.4)

set(PROJECT_NAME_INTERFACEC sl_zed_c)

project (${PROJECT_NAME} C CXX)

set(EXECUTABLE_OUTPUT_PATH .)

SET(LIB_TYPE SHARED)
add_definitions(-DINTERFACE_EXPORT)


FILE(GLOB_RECURSE HDRS_RS "include/*.hpp" "include/*.h")
FILE(GLOB_RECURSE SRCS_RS "src/*.cpp" "src/*.c")

find_package(ZED 3 REQUIRED)
find_package(CUDA)
message("CUDA Version : ${ZED_CUDA_VERSION}")


include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

if (MSVC)
	add_compile_options(/permissive- /O2)
else ()
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11")
	SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
	add_definitions(-std=c++0x -O3 -fPIC)
endif(MSVC)

add_library(${PROJECT_NAME_INTERFACEC} ${LIB_TYPE} ${SRCS_RS} ${HDRS_RS})

IF(MSVC)
	##Windows
	SET_PROPERTY(TARGET ${PROJECT_NAME_INTERFACEC} APPEND PROPERTY COMPILE_DEFINITIONS "WITH_UNLIMITED_CAMERA")

	target_link_libraries(${PROJECT_NAME_INTERFACEC}
						debug sl_zed64d optimized sl_zed64
						${CUDA_CUDART_LIBRARY_STATIC}
						${CUDA_CUDA_LIBRARY}
	)

	SET(SL_C_INTERFACE_LIB_DIR "$ENV{SL_ZED_SDK_INSTALL}/lib/")
	SET(SL_C_INTERFACE_BIN_DIR "$ENV{SL_ZED_SDK_INSTALL}/bin/")
	SET(SL_C_INTERFACE_INCLUDE_DIR "$ENV{SL_ZED_SDK_INSTALL}/include/sl/")

	INSTALL( FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/sl/c_api/zed_interface.h DESTINATION "${SL_C_INTERFACE_INCLUDE_DIR}/c_api/")
	INSTALL( FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/sl/c_api/types_c.h DESTINATION "${SL_C_INTERFACE_INCLUDE_DIR}/c_api/")
	INSTALL(TARGETS ${PROJECT_NAME_INTERFACEC} RUNTIME DESTINATION "${SL_C_INTERFACE_BIN_DIR}" ARCHIVE DESTINATION "${SL_C_INTERFACE_LIB_DIR}" COMPONENT library)

else(MSVC)

	SET_PROPERTY(TARGET ${PROJECT_NAME_INTERFACEC} APPEND PROPERTY COMPILE_DEFINITIONS "WITH_UNLIMITED_CAMERA")


	if(CMAKE_BUILD_TYPE STREQUAL "Debug")
		add_definitions(-D_DEBUG)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_DEBUG")
	endif()


	target_link_libraries(${PROJECT_NAME_INTERFACEC}
						${ZED_LIBRARIES}
						${CUDA_CUDA_LIBRARY}
						gomp
						pthread
						usb-1.0
					)

	##install
	SET(SL_C_INTERFACE_LIB_DIR "/usr/local/zed/lib")
	INSTALL(TARGETS ${PROJECT_NAME_INTERFACEC} DESTINATION "${SL_C_INTERFACE_LIB_DIR}")
	INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/sl/c_api/zed_interface.h DESTINATION "/usr/local/zed/include/sl/c_api")
	INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/sl/c_api/types_c.h DESTINATION "/usr/local/zed/include/sl/c_api")
endif(MSVC)

