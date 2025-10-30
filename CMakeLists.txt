cmake_minimum_required(VERSION 3.12)
project(ConstraintGeometry VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(CollisionAlgorithm REQUIRED)
find_package(Sofa.Component.StateContainer REQUIRED)
find_package(Sofa.Component.Constraint.Lagrangian.Solver REQUIRED)
find_package(Sofa.GL REQUIRED)

set(CONSTRAINTGEOMETRY_SRC "src/${PROJECT_NAME}")

set(HEADER_FILES
        ${CONSTRAINTGEOMETRY_SRC}/config.h.in
        ${CONSTRAINTGEOMETRY_SRC}/initConstraintGeometry.h

        ${CONSTRAINTGEOMETRY_SRC}/BaseConstraint.h
        ${CONSTRAINTGEOMETRY_SRC}/BaseNormalHandler.h
        ${CONSTRAINTGEOMETRY_SRC}/ConstraintDirection.h
        ${CONSTRAINTGEOMETRY_SRC}/ConstraintNormal.h
        ${CONSTRAINTGEOMETRY_SRC}/ConstraintProximity.h
        ${CONSTRAINTGEOMETRY_SRC}/ConstraintResponse.h
        ${CONSTRAINTGEOMETRY_SRC}/InternalConstraint.h

        ${CONSTRAINTGEOMETRY_SRC}/constraint/BilateralResolution.h
        ${CONSTRAINTGEOMETRY_SRC}/constraint/ConstraintBilateral.h
        ${CONSTRAINTGEOMETRY_SRC}/constraint/ConstraintInsertion.h
        ${CONSTRAINTGEOMETRY_SRC}/constraint/ConstraintUnilateral.h
        ${CONSTRAINTGEOMETRY_SRC}/constraint/InsertionResolution.h
        ${CONSTRAINTGEOMETRY_SRC}/constraint/UnilateralResolution.h

        ${CONSTRAINTGEOMETRY_SRC}/directions/BindDirection.h
        ${CONSTRAINTGEOMETRY_SRC}/directions/ContactDirection.h
        ${CONSTRAINTGEOMETRY_SRC}/directions/FirstDirection.h
        ${CONSTRAINTGEOMETRY_SRC}/directions/FixedFrameDirection.h
        ${CONSTRAINTGEOMETRY_SRC}/directions/SecondDirection.h

        ${CONSTRAINTGEOMETRY_SRC}/normalHandler/EdgeNormalHandler.h
        ${CONSTRAINTGEOMETRY_SRC}/normalHandler/GouraudTriangleNormalHandler.h
        ${CONSTRAINTGEOMETRY_SRC}/normalHandler/GravityPointNormalHandler.h
        ${CONSTRAINTGEOMETRY_SRC}/normalHandler/PhongTriangleNormalHandler.h
        ${CONSTRAINTGEOMETRY_SRC}/normalHandler/VectorPointNormalHandler.h

        ${CONSTRAINTGEOMETRY_SRC}/operations/ConstraintProximityOperation.h
)
file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS "${CONSTRAINTGEOMETRY_SRC}/*.cpp")

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)


target_link_libraries(${PROJECT_NAME} 
    CollisionAlgorithm
    Sofa.Component.StateContainer
    Sofa.Component.Constraint.Lagrangian.Solver
    Sofa.GL
)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME} 
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    EXAMPLE_INSTALL_DIR "scenes"
    RELOCATABLE "plugins"
)

enable_testing()

if(SOFA_BUILD_TESTS)
    add_subdirectory(tests/unit)
endif()
