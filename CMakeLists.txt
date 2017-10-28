##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)

project(tictactoe_ros)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS 
	rqt_gui 
	rqt_gui_cpp 
	image_transport 
	sensor_msgs 
	geometry_msgs 
	cv_bridge
  message_filters
  tf)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(tictactoe_ros_SRCS
  src/main.cpp
  src/main_window.cpp
  src/ratio_layouted_frame.cpp
  src/ai.cpp
  src/board.cpp
  src/gamewidget.cpp
  src/qnode.cpp
)

set(tictactoe_ros_HDRS
  include/tictactoe_ros/main_window.hpp
  include/tictactoe_ros/ratio_layouted_frame.hpp
  include/tictactoe_ros/qnode.hpp
  include/tictactoe_ros/ai.h
  include/tictactoe_ros/board.h
  include/tictactoe_ros/globals.h
  include/tictactoe_ros/gamewidget.h
)

set(tictactoe_ros_UIS
  ui/main_window.ui
)

set(tictactoe_ros_RESS
  resources/images.qrc
)

set(tictactoe_ros_INCLUDE_DIRECTORIES
  include
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}"
)

if(NOT EXISTS "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
  file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")
endif()

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(
  INCLUDE_DIRS ${tictactoe_ros_INCLUDE_DIRECTORIES}
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge geometry_msgs message_filters tf
)

# catkin_python_setup()		// for plugin registration

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(tictactoe_ros_MOCS ${tictactoe_ros_HDRS})
else()
  qt4_wrap_cpp(tictactoe_ros_MOCS ${tictactoe_ros_HDRS})
endif()

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}")

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(tictactoe_ros_UIS_H ${tictactoe_ros_UIS})
else()
  qt4_wrap_ui(tictactoe_ros_UIS_H ${tictactoe_ros_UIS})
endif()

set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_add_resources(tictactoe_ros_RCC ${tictactoe_ros_RESS})
else()
  qt4_add_resources(tictactoe_ros_RCC ${tictactoe_ros_RESS})
endif()

include_directories(${tictactoe_ros_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/tictactoe_ros/*.hpp)

#QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
#QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

#file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(${PROJECT_NAME} ${tictactoe_ros_SRCS} ${tictactoe_ros_MOCS} ${tictactoe_ros_UIS_H} ${tictactoe_ros_RCC})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

#install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

