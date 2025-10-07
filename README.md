# Constraint Geometry Plugin

A library for defining and managing geometric constraints in SOFA, particularly for contact modeling and response.

This plugin provides components to define constraint directions, normals, and resolutions, extending SOFA's constraint capabilities.

## Dependencies

- A C++11 compliant compiler (e.g., GCC, Clang, MSVC)
- CMake (version 3.10 or higher)
- SOFA Framework
- CollisionAlgorithm plugin

## How to Build

You can build this project as a plugin for the SOFA Framework.

1.  **Clone the repository:**
    ```bash
    git clone <your-repository-url>
    cd ConstraintGeometry
    ```

2.  **Configure with CMake:**
    ```bash
    cmake -B build .
    ```

3.  **Compile the project:**
    ```bash
    cmake --build build
    ```

## How to Use

*(A brief example of how to use the constraint components in a SOFA scene should be added here.)*

## Architecture Overview

The plugin is structured around a set of components that work together to define and solve constraints between geometries:

-   **`BaseConstraint`**: The core component that inherits from SOFA's `BaseLagrangianConstraint`. Concrete implementations like `ConstraintBilateral`, `ConstraintUnilateral`, and `ConstraintInsertion` take proximity detection results as input.

-   **`ConstraintDirection`**: These components define how the direction of the constraint is calculated. Examples include `BindDirection`, `ContactDirection`, `FirstDirection`, and `SecondDirection`.

-   **`BaseNormalHandler`**: Components that provide normal information for geometries, used by `ConstraintDirection` components. Examples are `EdgeNormalHandler` and `PhongTriangleNormalHandler`.

-   **Constraint Resolution**: Header files like `BilateralResolution.h`, `UnilateralResolution.h`, and `InsertionResolution.h` contain the logic for the constraint solver, defining how the constraints are mathematically resolved.
