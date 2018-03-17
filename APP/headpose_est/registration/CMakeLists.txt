set(MODULE_NAME registration)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set(SRC_LIST registration.cpp registration.h)
#add_executable(${MODULE_NAME} ${SRC_LIST})
add_library(${MODULE_NAME} STATIC ${SRC_LIST})
#message(STATUS ${Super4PCS_LIBRARIES})
#TARGET_LINK_LIBRARIES(${MODULE_NAME} "-Wl,--whole-archive" ${Super4PCS_LIBRARIES} "-Wl,--no-whole-archive")

TARGET_LINK_LIBRARIES(${MODULE_NAME} ${Super4PCS_LIBRARIES})

