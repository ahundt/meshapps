cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE "Release")

project(apps)

find_package(PCL REQUIRED) 
find_package(OpenCV REQUIRED) 
find_package(OpenMP REQUIRED)
find_package(Boost COMPONENTS Chrono)
find_package(aruco REQUIRED )

IF(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  message (STATUS "Found OpenMP")
ENDIF(OPENMP_FOUND)

if (UNIX)
  message(status "Setting GCC flags")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions -g -Wall")
else()
  message(status "Setting MSVC flags")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()
message(status "** CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

include_directories(${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}) 
link_directories(${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS}) 

add_library(liblinear utility/liblinear/linear.h utility/liblinear/tron.h 
            utility/liblinear/linear.cpp utility/liblinear/predict.cpp utility/liblinear/tron.cpp 
            utility/liblinear/blas.h utility/liblinear/blasp.h utility/liblinear/daxpy.cpp 
            utility/liblinear/ddot.cpp utility/liblinear/dnrm2.cpp utility/liblinear/dscal.cpp)

add_library(Utility utility/typedef.h utility/utility.h utility/utility.cpp utility/mcqd.h utility/mcqd.cpp) 
target_link_libraries(Utility liblinear ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

#library
add_executable(PCDToPNG src/main_img.cpp)
target_link_libraries(PCDToPNG Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(RGBAToRGB src/main_rgba.cpp)
target_link_libraries(RGBAToRGB Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(PostMesh src/main_post_mesh.cpp)
target_link_libraries(PostMesh Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ReadPose src/main_read_pose.cpp)
target_link_libraries(ReadPose Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(Mesh_Builder src/main_build_mesh.cpp) 
target_link_libraries(Mesh_Builder Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${aruco_LIBS})

add_executable(Scene_Aligner src/main_align_scene.cpp) 
target_link_libraries(Scene_Aligner Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${aruco_LIBS})

add_executable(Cropper src/main_cropper.cpp)
target_link_libraries(Cropper Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(PreProcess src/main_normal_est.cpp)
target_link_libraries(PreProcess Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ProRegion src/main_pro.cpp)
target_link_libraries(ProRegion Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ICRADemo src/main_demo_icra.cpp)
target_link_libraries(ICRADemo Utility ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

