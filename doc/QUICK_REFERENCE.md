# ConstraintGeometry Quick Reference

Quick guide for using ConstraintGeometry plugin with CollisionAlgorithm.

## When to Use Which Constraint

| Constraint Type | Use Case | Key Feature | Example |
|----------------|----------|-------------|---------|
| **Bilateral** | Fixed attachment | Enforces exact position match | Rigid coupling, fixed joints |
| **Unilateral** | Contact with separation | Non-penetration + optional friction | Surface contact, puncture detection |
| **Insertion** | Sliding with friction | Bilateral perpendicular, friction tangent | Needle-tissue coupling |

## Basic Usage Pattern

### 1. Setup Collision Detection (CollisionAlgorithm)

```python
root.addObject("CollisionLoop")
root.addObject("InsertionAlgorithm", name="detection",
    tipGeom="@Needle/tip/geom",
    surfGeom="@Tissue/surface/geom",
    shaftGeom="@Needle/shaft/geom",
    volGeom="@Tissue/volume/geom")
```

### 2. Setup Normal Handlers

```python
# For surface (triangles)
tissue.addObject("PhongTriangleNormalHandler", 
    name="surfaceNormals",
    geometry="@geom")

# For shaft (edges/beams)
needle.addObject("EdgeNormalHandler",
    name="shaftNormals", 
    geometry="@geom")
```

### 3. Setup Constraint Directions

```python
# Use surface normal
root.addObject("SecondDirection", 
    name="contactDir",
    handler="@Tissue/surface/surfaceNormals")

# Use shaft direction
root.addObject("FirstDirection",
    name="bindDir",
    handler="@Needle/shaft/shaftNormals")
```

### 4. Create Constraints

```python
# For puncture (unilateral)
root.addObject("ConstraintUnilateral",
    input="@detection.collisionOutput",
    directions="@contactDir",
    mu=0.3)  # friction coefficient

# For insertion (bilateral with friction)
root.addObject("ConstraintInsertion",
    input="@detection.insertionOutput",
    directions="@bindDir",
    frictionCoeff=0.002)
```

## Constraint Type Details

### Bilateral Constraints

```python
root.addObject("ConstraintBilateral",
    input="@algo.output",
    directions="@dir",
    compliance=0.0,     # 0 = rigid, >0 = soft
    maxForce=1e10)      # force limit
```

**Enforces:** `(p₁ - p₂) · n = 0` for each direction

**Dimensions:** 1, 2, or 3 (based on ConstraintDirection)

**Use for:**
- Perfect attachment (no slip, no separation)
- Fixed joints
- Rigid coupling between objects

### Unilateral Constraints

```python
root.addObject("ConstraintUnilateral",
    input="@algo.output",
    directions="@dir",
    mu=0.0,                 # friction: 0=frictionless, >0=Coulomb
    frictionCone=False,     # True: project the friction force on the Coulomb cone; False: bound each tangent by mu*fn (pyramid)
    contactDistance=0.0,    # thickness of the contact, e.g. the radius of a rod
    maxForce0=1e10)         # also maxForce1, maxForce2 for the friction axes
```

**Enforces:**
- `(p₁ - p₂) · n - contactDistance ≥ 0` (no penetration)
- `λ ≥ 0` (no tensile force)
- `λ · Φ = 0` (complementarity)

**Dimensions:**
- 1 (frictionless): Normal direction only
- 3 (with friction): Normal + 2 tangents

**Use for:**
- Contact without attachment
- Collision response
- Puncture detection (before insertion)

### Insertion Constraints

```python
root.addObject("ConstraintInsertion",
    input="@algo.output",
    directions="@dir",
    frictionCoeff=0.002,  # sliding friction
    maxForce=1e10)
```

**Enforces:**
- Tangent: Friction resists sliding `|λ₁| ≤ μ·f`
- Perpendicular: Bilateral `Φ₂ = Φ₃ = 0`

**Dimensions:** Always 3 (tangent + 2 perpendicular)

**Use for:**
- Needle insertion
- Cable/catheter tracking
- Sliding contact with lateral coupling

## Constraint Directions

### Direction Components

| Component | Description | When to Use |
|-----------|-------------|-------------|
| `BindDirection` | First geometry's normal | Attach along object's surface |
| `ContactDirection` | Second geometry's normal | Contact along target surface |
| `FirstDirection` | First proximity direction | Generic, uses proximity 1 |
| `SecondDirection` | Second proximity direction | Generic, uses proximity 2 |
| `FixedFrameDirection` | Fixed coordinate frame | World-space constraints |

### Normal Handlers

| Handler | Geometry Type | Normal Computation |
|---------|---------------|-------------------|
| `EdgeNormalHandler` | Edges/Beams | Edge direction + perpendiculars |
| `PhongTriangleNormalHandler` | Triangles | Smoothed vertex normals |
| `GouraudTriangleNormalHandler` | Triangles | Barycentric interpolation |
| `GravityPointNormalHandler` | Points | Gravity direction |
| `VectorPointNormalHandler` | Points | Custom vector field |

## Common Patterns

### Pattern 1: Surface Contact

```python
# Detection
root.addObject("FindClosestProximity",
    sourceGeom="@Object1/geom",
    targetGeom="@Object2/geom")

# Direction (use target surface normal)
root.addObject("SecondDirection", 
    handler="@Object2/normalHandler")

# Constraint (unilateral with friction)
root.addObject("ConstraintUnilateral",
    input="@algo.output",
    directions="@dir",
    mu=0.5)
```

### Pattern 2: Rigid Attachment

```python
# Detection (single point or multiple)
root.addObject("MyDetectionAlgo", ...)

# Direction (any suitable direction)
root.addObject("FirstDirection",
    handler="@Object/normalHandler")

# Constraint (bilateral, 3D)
root.addObject("ConstraintBilateral",
    input="@algo.output",
    directions="@dir")
```

### Pattern 3: Needle Insertion

```python
# Detection (InsertionAlgorithm)
root.addObject("InsertionAlgorithm",
    tipGeom="@Needle/tip/geom",
    surfGeom="@Tissue/surface/geom",
    shaftGeom="@Needle/shaft/geom",
    volGeom="@Tissue/volume/geom",
    punctureForceThreshold=15.0)

# Puncture contact (before insertion)
root.addObject("SecondDirection", name="punctureDir",
    handler="@Tissue/surface/normalHandler")
root.addObject("ConstraintUnilateral",
    input="@InsertionAlgo.collisionOutput",
    directions="@punctureDir",
    mu=0.3)

# Coupling points (during insertion)
root.addObject("FirstDirection", name="bindDir",
    handler="@Needle/shaft/normalHandler")
root.addObject("ConstraintInsertion",
    input="@InsertionAlgo.insertionOutput",
    directions="@bindDir",
    frictionCoeff=0.002)
```

## Debugging

### Visualization

All constraint components support visualization:

```python
component.addObject("ConstraintUnilateral",
    ...,
    draw_scale=0.001)  # Scale for arrow drawing
```

- **Red arrow:** First constraint direction (typically normal)
- **Green arrow:** Second direction (friction tangent 1)
- **Blue arrow:** Third direction (friction tangent 2)

### Common Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| No constraints created | Empty detection output | Check CollisionAlgorithm output |
| Wrong direction | Incorrect handler link | Verify `@path/to/handler` |
| Unstable simulation | Too stiff constraints | Add compliance or reduce maxForce |
| Penetration | Solver not converging | Increase solver maxIt or tolerance |
| No friction effect | `mu=0` or wrong resolution | Set `mu > 0` for friction |

### Checking Constraint Count

Enable SOFA info messages to see constraint statistics:

```python
# In console or scene
root.addObject("RequiredPlugin", pluginName="Sofa.Component.Setting")
root.addObject("MessageSetting", 
    messageHandler="console",
    level="info")
```

Look for:
- "Building X constraints" (from BaseConstraint)
- "LCP size: N×N" (from GenericConstraintSolver)

## Performance Tips

1. **Limit constraint count:**
   - Use DistanceFilter to remove distant contacts
   - Adjust detection thresholds

2. **Choose appropriate resolution:**
   - Bilateral: More constraints but stable
   - Unilateral: Fewer but may need more iterations

3. **Solver settings:**
   ```python
   root.addObject("GenericConstraintSolver",
       tolerance=1e-3,      # Lower = more accurate, slower
       maxIt=1000,          # Higher = more robust, slower
       allVerified=False)   # Skip verification for speed
   ```

4. **Compliance regularization:**
   ```python
   # Soften constraints to improve convergence
   root.addObject("ConstraintBilateral",
       compliance=1e-6)  # Small > 0
   ```

## Data Link Paths

### Valid Input Sources

```python
# From CollisionAlgorithm
input="@InsertionAlgo.collisionOutput"
input="@InsertionAlgo.insertionOutput"
input="@MyAlgo.output"

# Direction handlers
directions="@MyDirection"

# Handler links (in Direction component)
handler="@Object/subnode/MyNormalHandler"
```

### Path Syntax

- `@name` - Same node
- `@../name` - Parent node
- `@/Root/path/name` - Absolute path
- `@Node/SubNode/name` - Relative path

## Mathematical Formulas Summary

### Bilateral (1D)
```
λ = -(δ) / (W + C)
-λₘₐₓ ≤ λ ≤ λₘₐₓ
```

### Unilateral (Frictionless)
```
λ = max(0, -δ / W)
0 ≤ λ ≤ λₘₐₓ
```

### Unilateral (With Friction)
```
Solve normal: λₙ = max(0, -δₙ / Wₙₙ)
If λₙ > 0:
    Solve tangents with cone: √(λₜ₁² + λₜ₂²) ≤ μ·λₙ
```

### Insertion
```
Tangent: λ₁ = -W₁₁⁻¹·δ₁·μ
Perpendicular: [λ₂, λ₃]ᵀ = -W₂₃⁻¹·(δ₂₃ + W₂₃,₁·λ₁)
```

Where:
- `λ` = constraint force (Lagrange multiplier)
- `δ` = constraint violation (velocity level)
- `W` = constraint compliance matrix `J·M⁻¹·Jᵀ`
- `C` = regularization compliance
- `μ` = friction coefficient

## References

- **Full Architecture:** See [ARCHITECTURE.md](ARCHITECTURE.md)
- **CollisionAlgorithm:** See `../CollisionAlgorithm/README.md`
- **SOFA Constraints:** https://www.sofa-framework.org/community/doc/simulation-principles/constraint/lagrange-constraint/
