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

### Quick Start

ConstraintGeometry works with CollisionAlgorithm to transform collision detections into constraints:

```python
# 1. Add collision detection (CollisionAlgorithm)
root.addObject("CollisionLoop")
root.addObject("InsertionAlgorithm", name="detection", ...)

# 2. Setup normal handlers
tissue.addObject("PhongTriangleNormalHandler", name="normals", geometry="@geom")

# 3. Setup constraint directions
root.addObject("SecondDirection", name="dir", handler="@Tissue/normals")

# 4. Create constraints
root.addObject("ConstraintUnilateral",
    input="@detection.collisionOutput",  # From CollisionAlgorithm
    directions="@dir",                   # Constraint directions
    mu=0.3)                              # Friction coefficient
```

### Constraint Types

- **ConstraintBilateral** - Fixed attachment (equality constraints)
- **ConstraintUnilateral** - Contact with optional friction (inequality constraints)  
- **ConstraintInsertion** - Needle insertion with sliding friction

**For complete usage guide with patterns and examples, see [doc/QUICK_REFERENCE.md](doc/QUICK_REFERENCE.md)**

## Architecture Overview

ConstraintGeometry transforms collision detections from **CollisionAlgorithm** into constraint-based contact resolution using SOFA's Lagrangian constraint solver.

### Plugin Relationship

```
CollisionAlgorithm → ConstraintGeometry → SOFA ConstraintSolver
   (Detection)          (Constraint)         (Resolution)
```

**CollisionAlgorithm** detects proximities (contacts) between geometries and outputs `DetectionOutputVector`.

**ConstraintGeometry** reads these detections and creates Lagrangian constraints with appropriate directions and resolution strategies.

**SOFA GenericConstraintSolver** solves the resulting LCP (Linear Complementarity Problem) to compute constraint forces.

### Core Components

The plugin is structured around components that work together to define and solve constraints:

-   **`BaseConstraint`**: Transforms collision detections into constraints. Inherits from SOFA's `BaseLagrangianConstraint`.
    - `ConstraintBilateral` - Equality constraints (rigid attachment)
    - `ConstraintUnilateral` - Inequality constraints (contact with friction)
    - `ConstraintInsertion` - Needle insertion with sliding friction

-   **`ConstraintDirection`**: Computes constraint directions from geometric normals.
    - `BindDirection` - Use first geometry's normal
    - `ContactDirection` - Use second geometry's normal  
    - `FirstDirection` / `SecondDirection` - Proximity-based directions
    - `FixedFrameDirection` - Fixed frame basis

-   **`BaseNormalHandler`**: Provides geometric normal information.
    - `EdgeNormalHandler` - For beam/edge geometries
    - `PhongTriangleNormalHandler` - Smooth triangle normals
    - `GouraudTriangleNormalHandler` - Vertex-interpolated normals

-   **Constraint Resolution**: Defines solver behavior for each constraint type.
    - `BilateralResolution.h` - Enforces exact attachment
    - `UnilateralResolution.h` - Non-penetration with optional friction
    - `InsertionResolution.h` - Anisotropic friction model

### Detailed Documentation

**For complete architecture, mathematical formulations, and implementation details, see [doc/ARCHITECTURE.md](doc/ARCHITECTURE.md)**

Topics covered:
- Plugin dependency diagram
- Data flow pipeline
- Component interaction diagrams
- Mathematical formulations for all constraint types
- Constraint building process
- Needle insertion complete example
- Performance optimization strategies
