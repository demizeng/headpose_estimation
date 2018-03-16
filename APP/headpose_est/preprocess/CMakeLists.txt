set(MODULE_NAME preprocess)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
set(SRC_LIST preprocess.cpp preprocess.h)
#add_executable(${MODULE_NAME} ${SRC_LIST})
add_library(${MODULE_NAME} SHARED ${SRC_LIST})


