# Normal Handler Component Group

This directory contains components responsible for computing normals on geometric primitives. These handlers are used by `ConstraintDirection` components to determine the orientation of constraints.

## Core Concepts

All normal handlers inherit from `BaseNormalHandler`. They link to a `BaseGeometry` and implement the logic to compute normals for the proximities associated with that geometry.

## Concrete Implementations

-   **`EdgeNormalHandler`**: Computes the normal for an edge, typically aligned with the edge's direction.
-   **`GouraudTriangleNormalHandler`**: Computes a smooth normal for a point on a triangle by interpolating the vertex normals (Gouraud shading style).
-   **`PhongTriangleNormalHandler`**: Computes a smooth normal for a point on a triangle by interpolating the vertex normals, which are themselves computed by averaging the normals of adjacent triangles (Phong shading style).
-   **`GravityPointNormalHandler`**: Computes a normal pointing from the center of gravity of a mesh to a given point on its surface.
-   **`VectorPointNormalHandler`**: Assigns normals to points from a user-provided list of vectors.
