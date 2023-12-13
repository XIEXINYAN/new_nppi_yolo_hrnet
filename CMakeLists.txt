#声明最低版本要求并定义项目名称
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(syzk-camera)

#选择Debug模式
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set( CMAKE_CXX_STANDARD 14)

#选择Release模式
#SET(CMAKE_BUILD_TYPE "Release")
#add_compile_options(-std=c++14)
#add_definitions(-DBOOST_LOG_DYN_LINK)
#add_definitions(-Wno-builtin-macro-redefined)

if(DEFINED orin)
    include(cmake/orin.cmake)
    message(STATUS "orin" )
    set(INSTALL_PERCEPTION_PATH $ENV{SYZKENVHOME_ORIN})
elseif(DEFINED local)
    include(cmake/local.cmake)
    message(STATUS "local" )
    set(INSTALL_PERCEPTION_PATH $ENV{SYZKENVHOMEX86})
else()
    include(cmake/cross.cmake)
    message(STATUS "cross" )
    set(INSTALL_PERCEPTION_PATH $ENV{SYZKENVHOME})
endif()



#设置可执行程序输出路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#设置库文件输出路径
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#使用相对路径,否则可执行程序和库文件更换路径后,程序无法运行
SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
SET(CMAKE_INSTALL_RPATH "\${ORIGIN}/../lib") #指定运行时动态库的加载路径,ORIGIN指运行文件所在目录

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${MY_DICTIONARYS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(src)
include_directories(src/driver)
include_directories(src/front_alg)
include_directories(src/front_alg/detect)
include_directories(src/front_alg/distance)
include_directories(src/back_alg)
include_directories(src/common)
include_directories(src/fault_alg)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} DIR_SRCS_CAMERA)
aux_source_directory(src DIR_SRCS_THREAD)
aux_source_directory(src/driver DIR_SRCS_DIRVER)
aux_source_directory(src/front_alg DIR_SRCS_FRONT_ALG)
aux_source_directory(src/front_alg/detect DIR_SRCS_FRONT_DETECT_ALG)
aux_source_directory(src/front_alg/distance DIR_SRCS_FRONT_DISTANCE_ALG)
aux_source_directory(src/back_alg DIR_SRCS_BACK_ALG)
aux_source_directory(src/common DIR_SRCS_COMMON)
aux_source_directory(src/fault_alg DIR_SRCS_FAULT_ALG)


FILE (GLOB SRC_LIST
    ${DIR_SRCS_CAMERA}
    ${DIR_SRCS_THREAD}
    ${DIR_SRCS_DIRVER}
    ${DIR_SRCS_FRONT_ALG}
    ${DIR_SRCS_FRONT_DETECT_ALG}
    ${DIR_SRCS_FRONT_DISTANCE_ALG}
    ${DIR_SRCS_BACK_ALG}
    ${DIR_SRCS_COMMON}
    ${DIR_SRCS_FAULT_ALG}
    )

add_executable(${PROJECT_NAME} 
                    ${SRC_LIST}
                    )

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
			myplugins
            perception_common
            
			${MY_LIBS} 
            )




if(DEFINED pub)
    message(STATUS "pub == " ${pub} )
    install(DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/bin  DESTINATION ${pub} FILES_MATCHING PATTERN "*" )
    # install(DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/../config  DESTINATION ${pub} FILES_MATCHING PATTERN "*")
endif()




