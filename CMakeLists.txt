#-----------Linux--------
cmake_minimum_required(VERSION 3.10)
project(rrtstar-kinematic-6DOF)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find yaml-cpp package
find_package(yaml-cpp REQUIRED)

# Add includes
include_directories(${YAML_CPP_INCLUDE_DIR})

# Find Google Test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add the main executable
add_executable(ivk_test
    src/inverse_kinematics.cpp
    src/forward_kinematics.cpp
    src/main.cpp
)

add_executable(fk_standalone
    src/inverse_kinematics.cpp
    src/fk_test_standalone.cpp
    src/forward_kinematics.cpp
)

add_executable(kinematics_test
    src/kinematics_test.cpp
    src/forward_kinematics.cpp
    src/inverse_kinematics.cpp
)

add_executable(ik_solution_eval_test
    src/ik_solution_evaluator_test.cpp
    src/inverse_kinematics.cpp
    src/ik_solution_evaluator.cpp
    src/obstacle.cpp
)

add_executable(rrt_test
    src/rrtstar.cpp
    src/test_rrtstar.cpp
)

add_executable(rrt_testmodified
    src/rrtstarmodified.cpp
    src/test_rrtstarmodified.cpp
    src/inverse_kinematics.cpp
    src/ik_solution_evaluator.cpp
    src/obstacle.cpp
    src/forward_kinematics.cpp
)

add_executable(rrt_testmodifiedsp
    src/rrtstar_main.cpp
    src/test_rrtstarmodifiedsp.cpp
    src/inverse_kinematics.cpp
    src/ik_solution_evaluator.cpp
    src/obstacle.cpp
    src/forward_kinematics.cpp
    src/path_planning.cpp
    src/path_optimization.cpp
    src/tree_management.cpp
    src/collision_detection.cpp
    src/robot_kinematics.cpp
    src/path_export.cpp
    src/path_return.cpp
)

# Add the new hand-eye calibration executable
# add_executable(hand_eye_calibration_test
#     src/hand_eye_calibration.cpp
#     src/test_hand_eye_calibration.cpp
# )

# Include directories
target_include_directories(ivk_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(fk_standalone
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(kinematics_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(rrt_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(rrt_testmodified
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(rrt_testmodifiedsp
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

target_include_directories(ik_solution_eval_test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)

# target_include_directories(hand_eye_calibration_test
#     PRIVATE
#         ${PROJECT_SOURCE_DIR}/include
# )

# Find Python and link against it
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Find and link TBB (Intel Threading Building Blocks)
find_package(TBB REQUIRED)

# Link TBB to the target
target_link_libraries(ivk_test PRIVATE TBB::tbb)

# Link Google Test and pthread
target_link_libraries(ivk_test PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)
target_link_libraries(kinematics_test PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)
target_link_libraries(rrt_test PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)
target_link_libraries(rrt_testmodified PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)
target_link_libraries(rrt_testmodifiedsp PRIVATE ${GTEST_LIBRARIES} ${YAML_CPP_LIBRARIES} GTest::gtest_main pthread)
# Link OpenCV libraries to the ik_solution_eval_test target
target_link_libraries(ik_solution_eval_test PRIVATE ${OpenCV_LIBS})
# target_link_libraries(ik_solution_eval_test PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)
# target_link_libraries(hand_eye_calibration_test PRIVATE ${GTEST_LIBRARIES} GTest::gtest_main pthread)

target_link_libraries(ivk_test
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        TBB::tbb
        ${GTEST_LIBRARIES}
        GTest::gtest_main
        pthread
)
target_link_libraries(fk_standalone
    PRIVATE
        Python3::Python
        Eigen3::Eigen
)
target_link_libraries(kinematics_test
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        ${GTEST_LIBRARIES}
        GTest::gtest_main
        pthread
)
target_link_libraries(ik_solution_eval_test
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        ${GTEST_LIBRARIES}
        GTest::gtest_main
        pthread
)
target_link_libraries(rrt_test
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        ${GTEST_LIBRARIES}
        GTest::gtest_main
        pthread
        OpenMP::OpenMP_CXX # Explicitly link OpenMP for rrt_test
)

target_link_libraries(rrt_testmodified
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        ${GTEST_LIBRARIES}
        GTest::gtest_main
        pthread
        OpenMP::OpenMP_CXX # Explicitly link OpenMP for rrt_test
)

target_link_libraries(rrt_testmodifiedsp
    PRIVATE
        Python3::Python
        Eigen3::Eigen
        ${GTEST_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        GTest::gtest_main
        pthread
        OpenMP::OpenMP_CXX # Explicitly link OpenMP for rrt_test
)

# Link libraries for the new hand-eye calibration executable
# target_link_libraries(hand_eye_calibration_test
#     PRIVATE
#         Python3::Python
#         Eigen3::Eigen
#         ${OpenCV_LIBS}
#         ${GTEST_LIBRARIES}
#         GTest::gtest_main
#         pthread
# )

# Enable testing
enable_testing()

# Compiler options
if(MSVC)
    target_compile_options(ivk_test PRIVATE /W4)
    target_compile_options(fk_standalone PRIVATE /W4)
    target_compile_options(kinematics_test PRIVATE /W4)
    target_compile_options(rrt_test PRIVATE /W4)
    target_compile_options(rrt_testmodified PRIVATE /W4)
    target_compile_options(rrt_testmodifiedsp PRIVATE /W4)
    target_compile_options(ik_solution_eval_test PRIVATE /W4)
    # target_compile_options(hand_eye_calibration_test PRIVATE /W4)
else()
    target_compile_options(ivk_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(fk_standalone PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(kinematics_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(rrt_test PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(rrt_testmodified PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(rrt_testmodifiedsp PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(ik_solution_eval_test PRIVATE -Wall -Wextra -Wpedantic)
    # target_compile_options(hand_eye_calibration_test PRIVATE -Wall -Wextra -Wpedantic)
endif()



# # ---------MacOS-------------
# cmake_minimum_required(VERSION 3.12)
# # Add policy for GTEST_ROOT
# cmake_policy(SET CMP0144 NEW)
# # Add policy for FindBoost warning
# if(POLICY CMP0167)
#     cmake_policy(SET CMP0167 NEW)
# endif()

# project(rrtstar-kinematic-6DOF)

# # Set C++ standard
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Explicitly set architecture for macOS (Apple Silicon)
# if(APPLE)
#     set(CMAKE_OSX_ARCHITECTURES "arm64")
# endif()

# # Point CMake to the correct Python installation
# set(Python3_ROOT_DIR "/opt/homebrew/opt/python@3.13")
# find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

# # Find OpenCV
# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})

# # Dynamically fetch NumPy include path
# execute_process(
#     COMMAND ${Python3_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
#     OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
#     OUTPUT_STRIP_TRAILING_WHITESPACE
# )

# # Include directories
# include_directories(
#     ${CMAKE_SOURCE_DIR}/src
#     ${CMAKE_SOURCE_DIR}/include
#     ${Python3_INCLUDE_DIRS}
#     ${NUMPY_INCLUDE_DIR}
# )

# # Find other dependencies
# find_package(Eigen3 3.3 REQUIRED)
# find_package(Boost REQUIRED COMPONENTS serialization)
# find_package(TBB REQUIRED)

# # Ensure we're building for ARM64 on Apple Silicon
# if(APPLE)
#     set(CMAKE_OSX_ARCHITECTURES "arm64")
# endif()

# # Find Google Test - FIXED SECTION
# if(APPLE)
#     # Use FetchContent instead of ExternalProject
#     include(FetchContent)
#     FetchContent_Declare(
#         googletest
#         GIT_REPOSITORY https://github.com/google/googletest.git
#         GIT_TAG v1.14.0  # Corrected tag format
#     )
    
#     # Configure Google Test build options
#     set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#     set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
#     set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    
#     # Make GoogleTest available - this downloads and builds at configure time
#     FetchContent_MakeAvailable(googletest)
# else()
#     find_package(GTest REQUIRED)
# endif()

# # OpenMP setup for macOS - FORCE ARM64 PATH
# if(APPLE)
#     # Force ARM64 path for OpenMP
#     set(OPENMP_PREFIX "/opt/homebrew/opt/libomp")
    
#     if(NOT EXISTS "${OPENMP_PREFIX}/lib/libomp.dylib")
#         message(FATAL_ERROR "ARM64 OpenMP not found at ${OPENMP_PREFIX}. 
#         Please install with: /opt/homebrew/bin/brew install libomp")
#     endif()
    
#     # Check the actual architecture of the library
#     execute_process(
#         COMMAND lipo -info "${OPENMP_PREFIX}/lib/libomp.dylib"
#         OUTPUT_VARIABLE OPENMP_ARCH
#         OUTPUT_STRIP_TRAILING_WHITESPACE
#     )
#     message(STATUS "OpenMP library architecture: ${OPENMP_ARCH}")
#     message(STATUS "OpenMP library path: ${OPENMP_PREFIX}/lib/libomp.dylib")

#     # Set OpenMP flags
#     set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp")
#     set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
#     set(OpenMP_C_LIB_NAMES "omp")
#     set(OpenMP_CXX_LIB_NAMES "omp")

#     # Include directories
#     include_directories("${OPENMP_PREFIX}/include")
    
#     # Link directories with explicit architecture
#     link_directories("${OPENMP_PREFIX}/lib")
    
#     # Create imported target
#     add_library(OpenMP::OpenMP_CXX SHARED IMPORTED)
#     set_target_properties(OpenMP::OpenMP_CXX PROPERTIES
#         IMPORTED_LOCATION "${OPENMP_PREFIX}/lib/libomp.dylib"
#         INTERFACE_INCLUDE_DIRECTORIES "${OPENMP_PREFIX}/include"
#         INTERFACE_COMPILE_OPTIONS "${OpenMP_CXX_FLAGS}"
#     )

#     message(STATUS "Manual OpenMP configuration successful")
#     set(OpenMP_CXX_FOUND TRUE)
# else()
#     find_package(OpenMP REQUIRED)
# endif()

# # Add the main executables
# add_executable(ivk_test
#     src/inverse_kinematics.cpp
#     src/forward_kinematics.cpp
#     src/main.cpp
# )

# add_executable(fk_standalone
#     src/inverse_kinematics.cpp
#     src/fk_test_standalone.cpp
#     src/forward_kinematics.cpp
# )

# add_executable(kinematics_test
#     src/kinematics_test.cpp
#     src/forward_kinematics.cpp
#     src/inverse_kinematics.cpp
# )

# add_executable(ik_solution_eval_test
#     src/ik_solution_evaluator_test.cpp
#     src/inverse_kinematics.cpp
#     src/ik_solution_evaluator.cpp
#     src/obstacle.cpp
# )

# add_executable(rrt_test
#     src/rrtstar.cpp
#     src/test_rrtstar.cpp
# )

# add_executable(rrt_testmodified
#     src/rrtstarmodified.cpp
#     src/test_rrtstarmodified.cpp
#     src/inverse_kinematics.cpp
#     src/forward_kinematics.cpp
#     src/ik_solution_evaluator.cpp
#     src/obstacle.cpp
# )

# add_executable(rrt_testmodifiedsp
#     src/rrtstar_main.cpp
#     src/test_rrtstarmodifiedsp.cpp
#     src/inverse_kinematics.cpp
#     src/ik_solution_evaluator.cpp
#     src/obstacle.cpp
#     src/forward_kinematics.cpp
#     src/path_planning.cpp
#     src/path_optimization.cpp
#     src/tree_management.cpp
#     src/collision_detection.cpp
#     src/robot_kinematics.cpp
# )


# # Link libraries - FIXED ALL TARGETS
# target_link_libraries(ivk_test
#     PRIVATE
#         Python3::Python
#         Eigen3::Eigen
#         TBB::tbb
#         gtest_main
#         pthread
# )

# target_link_libraries(fk_standalone
#     PRIVATE
#         Python3::Python
#         Eigen3::Eigen
# )

# target_link_libraries(kinematics_test
#     PRIVATE
#         Python3::Python
#         Eigen3::Eigen
#         gtest_main
#         pthread
# )

# target_link_libraries(ik_solution_eval_test
#     PRIVATE
#         Python3::Python
#         Eigen3::Eigen
#         gtest_main
#         pthread
#         ${OpenCV_LIBS}
# )

# target_link_libraries(rrt_test
#     PRIVATE
#         Python3::Python
#         Eigen3::Eigen
#         gtest_main
#         pthread
# )

# target_link_libraries(rrt_testmodified
#     PRIVATE
#         Python3::Python
#         Eigen3::Eigen
#         gtest_main
#         pthread
# )

# target_link_libraries(rrt_testmodifiedsp
#     PRIVATE
#         Python3::Python
#         Eigen3::Eigen
#         gtest_main
#         pthread
# )

# if(APPLE)
#     target_compile_options(rrt_test PRIVATE -Xpreprocessor -fopenmp)
#     target_link_libraries(rrt_test PRIVATE "${OPENMP_PREFIX}/lib/libomp.dylib")
#     target_include_directories(rrt_test PRIVATE "${OPENMP_PREFIX}/include")

#     target_compile_options(rrt_testmodified PRIVATE -Xpreprocessor -fopenmp)
#     target_link_libraries(rrt_testmodified PRIVATE "${OPENMP_PREFIX}/lib/libomp.dylib")
#     target_include_directories(rrt_testmodified PRIVATE "${OPENMP_PREFIX}/include")
    
# else()
#     target_link_libraries(rrt_test PRIVATE OpenMP::OpenMP_CXX)
#     target_link_libraries(rrt_testmodified PRIVATE OpenMP::OpenMP_CXX)
# endif()

# # Include directories for each target
# target_include_directories(ivk_test
#     PRIVATE
#         ${PROJECT_SOURCE_DIR}/include
# )

# target_include_directories(fk_standalone
#     PRIVATE
#         ${PROJECT_SOURCE_DIR}/include
# )

# target_include_directories(kinematics_test
#     PRIVATE
#         ${PROJECT_SOURCE_DIR}/include
# )

# target_include_directories(rrt_test
#     PRIVATE
#         ${PROJECT_SOURCE_DIR}/include
# )

# target_include_directories(rrt_testmodified
#     PRIVATE
#         ${PROJECT_SOURCE_DIR}/include
# )

# target_include_directories(rrt_testmodifiedsp
#     PRIVATE
#         ${PROJECT_SOURCE_DIR}/include
# )

# target_include_directories(ik_solution_eval_test
#     PRIVATE
#         ${PROJECT_SOURCE_DIR}/include
# )

# # Suppress deprecated declarations warnings
# target_compile_options(ivk_test PRIVATE -Wno-deprecated-declarations)
# target_compile_options(fk_standalone PRIVATE -Wno-deprecated-declarations)
# target_compile_options(kinematics_test PRIVATE -Wno-deprecated-declarations)
# target_compile_options(rrt_test PRIVATE -Wno-deprecated-declarations)
# target_compile_options(rrt_testmodified PRIVATE -Wno-deprecated-declarations)
# target_compile_options(rrt_testmodifiedsp PRIVATE -Wno-deprecated-declarations)
# target_compile_options(ik_solution_eval_test PRIVATE -Wno-deprecated-declarations)