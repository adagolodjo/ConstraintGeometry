# Constraint Component Group

This directory contains the core constraint components of the plugin. These components inherit from `BaseConstraint` and implement different types of Lagrangian constraints.

## Core Concepts

Each constraint component takes a `DetectionOutput` from an algorithm in the `CollisionAlgorithm` plugin as input and creates the corresponding constraint equations to be solved by a SOFA `ConstraintSolver`.

## Concrete Implementations

-   **`ConstraintBilateral`**: Implements a bilateral (equality) constraint, forcing the distance between two proximity pairs to be zero.
-   **`ConstraintUnilateral`**: Implements a unilateral (inequality) constraint, preventing interpenetration between two proximity pairs.
-   **`ConstraintInsertion`**: A specialized component that models the forces involved in needle insertion, including friction.
