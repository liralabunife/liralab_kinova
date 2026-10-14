set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_C_COMPILER gcc-12)
#set(CMAKE_CXX_COMPILER g++-12)

cmake_minimum_required(VERSION 3.5)
project(liralab_kinova)
#set(OpenCV_DIR "/home/legion/opencv-4.13.0/build")

# Trova i pacchetti ROS 2 necessari
find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(trajectory_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(OpenCV REQUIRED)


# Aggiungi la directory include al progetto (include tutte le sottocartelle)
include_directories(${PROJECT_SOURCE_DIR}/include)

# KortexApiCpp
add_library(libKortexApiCpp STATIC IMPORTED)
set_target_properties(libKortexApiCpp PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/lib/libKortexApiCpp.a)


# Aggiungi l'eseguibile
add_executable(liralab_kinova src/mainGUI.cpp src/KinovaLiralab.cpp src/ftSenseHandler.cpp)

# Linka l'eseguibile con la libreria statica
target_link_libraries(liralab_kinova libKortexApiCpp Eigen3::Eigen)
target_link_libraries(liralab_kinova orocos-kdl urdf::urdf)
target_link_libraries(liralab_kinova ${OpenCV_LIBS})
target_link_libraries(liralab_kinova Threads::Threads)

add_compile_options(-Wdeprecated-declarations)

# Aggiungi le dipendenze ROS2
ament_target_dependencies(liralab_kinova urdf kdl_parser)
ament_package()

# Assicurati che l'eseguibile abbia accesso a include/
target_include_directories(liralab_kinova
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include/client
    ${PROJECT_SOURCE_DIR}/include/client_stubs
    ${PROJECT_SOURCE_DIR}/include/common
    ${PROJECT_SOURCE_DIR}/include/messages
    ${PROJECT_SOURCE_DIR}/include/liralab_kinova
)

add_compile_options(-Wno-deprecated-declarations)

# Installazione dell'eseguibile
install(TARGETS liralab_kinova DESTINATION lib/${PROJECT_NAME})

# Installazione della libreria statica libKortexApiCpp
install(FILES ${PROJECT_SOURCE_DIR}/lib/libKortexApiCpp.a DESTINATION lib/${PROJECT_NAME}/.lib)

# Installazione dei file di intestazione (header)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
add_definitions(-D_OS_UNIX)
