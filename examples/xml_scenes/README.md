# xml_scenes — obsolete

`beamContact.scn` and `liver.scn` were written for an older version of SOFA and of the plugins and **do not load any more**
(checked on SOFA 26.12: `ConstraintGeometryPlugin`/`CollisionAlgorithmPlugin` are now `ConstraintGeometry`/`CollisionAlgorithm`,
`GenericConstraintSolver` is `BlockGaussSeidelConstraintSolver`, `FindClosestProximityAlgorithm` no longer exists, `EulerImplicit`
is `EulerImplicitIntegrationScheme` ..., and they need a Geomagic haptic plugin).  They are kept as a record only.

Working scenes: `../../../CollisionAlgorithm/examples/NeedleInsertion.py` (needle in a gel) and
`../../../CollisionAlgorithm/examples/RodOnSurface.py` (a flexible rod resting on a triangle surface, with friction).
