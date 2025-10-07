# Operations Component Group

This directory contains the dynamic dispatch operations specific to the `ConstraintGeometry` plugin.

## Core Concepts

Following the same pattern as `CollisionAlgorithm`, this directory uses the `GenericOperation` template to create type-safe, dynamically dispatched functions.

## Concrete Implementations

-   **`ConstraintProximityOperation`**: This is a key operation that creates a `ConstraintProximity` from a `BaseProximity`. It takes a `BaseNormalHandler` and a `BaseProximity` and, based on their types, returns a `ConstraintProximity` that knows how to compute its normal. This allows for flexible combinations of geometries and normal computation methods.
