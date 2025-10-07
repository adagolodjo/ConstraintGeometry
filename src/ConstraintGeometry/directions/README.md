# Directions Component Group

This directory contains components that define the direction in which a constraint is applied. They inherit from `ConstraintDirection` and are used by the main constraint components (e.g., `ConstraintBilateral`) to compute the constraint normals.

## Core Concepts

Each `ConstraintDirection` component implements the `createConstraintsNormal()` method, which takes two proximity objects and returns a `ConstraintNormal` object containing the direction(s) of the constraint.

## Concrete Implementations

-   **`BindDirection`**: Creates a constraint direction along the vector connecting the two proximity points.
-   **`ContactDirection`**: Computes a contact direction based on the normal of the destination geometry, ensuring it points outwards.
-   **`FirstDirection` / `SecondDirection`**: Computes the constraint direction based on the normal of the first or second proximity object, respectively.
-   **`FixedFrameDirection`**: Provides a fixed set of orthogonal directions, useful for creating constraints in a fixed reference frame.
