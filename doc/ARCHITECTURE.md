# ConstraintGeometry Architecture Documentation

## Overview

ConstraintGeometry is a SOFA plugin that extends collision detection from CollisionAlgorithm into constraint-based contact resolution. It transforms detected proximities (contacts) into Lagrangian constraints that are solved by SOFA's constraint solver.

## Relationship with CollisionAlgorithm

### Plugin Dependencies

```
┌─────────────────────────────────────────────────┐
│            SOFA Framework                        │
│  (BaseLagrangianConstraint, ConstraintSolver)   │
└─────────────────┬───────────────────────────────┘
                  │
          ┌───────┴────────┐
          │                │
┌─────────▼─────────┐  ┌──▼──────────────────┐
│ CollisionAlgorithm │  │ ConstraintGeometry  │
│                    │  │                     │
│  - Collision       │  │  - Constraint       │
│    Detection       │  │    Building         │
│  - Proximity       │  │  - Direction        │
│    Output          │  │    Computation      │
│  - BaseProximity   │◄─┤  - Resolution       │
│  - BaseGeometry    │  │    Logic            │
└────────────────────┘  └─────────────────────┘
```

### Data Flow Pipeline

```
1. CollisionLoop (CollisionAlgorithm)
   │
   ├─ prepareDetection() → Update AABBBroadPhase
   │
   └─ doDetection()
      │
      └─ InsertionAlgorithm
         │
         ├─ Outputs: DetectionOutputVector
         │   ├─ collisionOutput (puncture contacts)
         │   └─ insertionOutput (coupling points)
         │
         ▼
2. ConstraintGeometry Components
   │
   ├─ ConstraintDirection (compute normals)
   │   │
   │   ├─ Reads: BaseNormalHandler
   │   └─ Outputs: ConstraintNormal
   │
   ├─ BaseConstraint (build constraints)
   │   │
   │   ├─ Reads: DetectionOutputVector + ConstraintDirection
   │   ├─ Builds: InternalConstraint objects
   │   └─ Outputs: Lagrangian constraints
   │
   └─ ConstraintResolution (solve constraints)
       │
       └─ Computes: Constraint forces
           │
           ▼
3. GenericConstraintSolver (SOFA)
   │
   └─ Solves LCP → constraint forces applied to objects
```

### Component Interaction Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    SOFA Scene Graph                              │
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │  Root                                                      │  │
│  │    │                                                       │  │
│  │    ├─ CollisionLoop ←────────────────────┐              │  │
│  │    │                                       │              │  │
│  │    ├─ InsertionAlgorithm ─────────────┐   │              │  │
│  │    │  (CollisionAlgorithm)             │   │              │  │
│  │    │                                    │   │              │  │
│  │    │  Output:                           │   │              │  │
│  │    │   • collisionOutput ───────┐       │   │              │  │
│  │    │   • insertionOutput ────┐  │       │   │              │  │
│  │    │                         │  │       │   │              │  │
│  │    ├─ DistanceFilter ◄──────┘  │       │   │              │  │
│  │    │                            │       │   │              │  │
│  │    ├─ SecondDirection ◄─────────┤       │   │              │  │
│  │    │  (ConstraintGeometry)      │       │   │              │  │
│  │    │                            │       │   │              │  │
│  │    ├─ ConstraintUnilateral ◄───┘       │   │              │  │
│  │    │  Input: collisionOutput            │   │              │  │
│  │    │  Directions: SecondDirection       │   │              │  │
│  │    │                                    │   │              │  │
│  │    ├─ FirstDirection ◄──────────────────┤   │              │  │
│  │    │                                    │   │              │  │
│  │    └─ ConstraintInsertion ◄────────────┘   │              │  │
│  │       Input: insertionOutput                │              │  │
│  │       Directions: FirstDirection            │              │  │
│  │                                              │              │  │
│  │  ┌────────────────────────┐ ┌──────────────┴──────────┐  │  │
│  │  │  Needle                │ │  Tissue                  │  │  │
│  │  │  ├─ tipCollision       │ │  ├─ collision           │  │  │
│  │  │  │   └─ PointGeometry ◄┼─┼──┼─ TriangleGeometry    │  │  │
│  │  │  │       EdgeGeometry ◄┼─┼──┤    AABBBroadPhase    │  │  │
│  │  │  │                     │ │  │    PhongNormalHandler│  │  │
│  │  │  └─ bodyCollision      │ │  └─ TetrahedronGeometry │  │  │
│  │  │      EdgeNormalHandler │ │                          │  │  │
│  │  └────────────────────────┘ └──────────────────────────┘  │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## Core Components

### 1. BaseConstraint

The main component that transforms collision detections into constraints.

**Hierarchy:**
```
sofa::core::behavior::BaseLagrangianConstraint
    └─ constraintgeometry::BaseConstraint<DataTypes>
        ├─ ConstraintBilateral
        ├─ ConstraintUnilateral
        └─ ConstraintInsertion
```

**Key Data Fields:**
- `d_input`: Link to `DetectionOutputVector` from collision detection
- `d_directions`: Link to `ConstraintDirection` component
- `d_constraints`: Internal constraint storage

**Key Methods:**
```cpp
// Build constraint matrix (Jacobians)
virtual void buildConstraintMatrix(
    const ConstraintParams* cParams,
    MultiMatrixDerivId cId, 
    unsigned int &cIndex
);

// Get constraint violations
virtual void getConstraintViolation(
    const ConstraintParams* cParams,
    defaulttype::BaseVector *v, 
    unsigned int cIndex
);

// Create resolution object
virtual void getConstraintResolution(
    const ConstraintParams* cParams,
    std::vector<ConstraintResolution*>& resTab,
    unsigned int& offset
);
```

### 2. ConstraintDirection

Computes constraint directions (normals) from geometric information.

**Hierarchy:**
```
ConstraintDirection (interface)
    ├─ BindDirection       // Use first geometry's normal
    ├─ ContactDirection    // Use second geometry's normal
    ├─ FirstDirection      // First proximity direction
    ├─ SecondDirection     // Second proximity direction
    └─ FixedFrameDirection // Fixed frame basis
```

**Operation:**
```cpp
// Convert proximity pair to constraint normals
virtual ConstraintNormal operator()(
    const BaseProximity::SPtr& prox1,
    const BaseProximity::SPtr& prox2
) const;
```

### 3. BaseNormalHandler

Provides geometric normal information to ConstraintDirection.

**Implementations:**
```
BaseNormalHandler
    ├─ EdgeNormalHandler           // Edge (beam) normals
    ├─ GouraudTriangleNormalHandler // Vertex-interpolated normals
    ├─ PhongTriangleNormalHandler   // Smooth triangle normals
    ├─ GravityPointNormalHandler    // Gravity direction
    └─ VectorPointNormalHandler     // Custom vector field
```

### 4. InternalConstraint

Internal representation of a constraint between two proximities.

**Structure:**
```cpp
template<class FIRST, class SECOND>
class InternalConstraint {
    // Proximity pairs
    std::vector<std::pair<FIRST::SPtr, SECOND::SPtr>> m_pairs;
    
    // Constraint directions/normals
    std::vector<ConstraintNormal> m_vecNormals;
    
    // Constraint resolution factory
    ResolutionCreator m_creator;
    
    // Constraint violation function
    ViolationFunc m_violationFunc;
};
```

### 5. ConstraintResolution

Solver-specific constraint resolution logic.

**Types:**
- `BilateralConstraintResolution` - Equality constraints
- `UnilateralConstraintResolution` - Inequality constraints  
- `InsertionConstraintResolution` - Needle insertion constraints

## Constraint Types and Mathematical Formulations

### Bilateral Constraints

**Purpose:** Enforce exact attachment between two bodies (e.g., needle-tissue coupling during insertion).

**Mathematical Formulation:**

For a 1D bilateral constraint:

```
Constraint equation:
    Φ = (p₁ - p₂) · n = 0

where:
    p₁, p₂ = positions of proximity points
    n      = constraint direction

Jacobian:
    J₁ = n^T    (for first object)
    J₂ = -n^T   (for second object)

LCP contribution:
    δ = J₁v₁ + J₂v₂ = 0    (no relative motion)

Resolution:
    λ = -δ / (W + C)

where:
    W = J·M⁻¹·J^T  (constraint compliance)
    C = compliance parameter (regularization)
    λ = constraint force (Lagrange multiplier)

Force bounds:
    -λₘₐₓ ≤ λ ≤ λₘₐₓ
```

**Multi-dimensional (2D/3D) bilateral:**

```
For N directions d₁, d₂, ..., dₙ:

Φᵢ = (p₁ - p₂) · dᵢ = 0,  i = 1..N

Constraint matrix:
    ┌     ┐
    │ W₁₁ W₁₂ ... W₁ₙ │   ┌ C₁  0  ... 0  ┐
W = │ W₂₁ W₂₂ ... W₂ₙ │ + │ 0  C₂  ... 0  │
    │  ⋮   ⋮   ⋱  ⋮  │   │  ⋮  ⋮   ⋱  ⋮  │
    └ Wₙ₁ Wₙ₂ ... Wₙₙ ┘   └ 0   0  ... Cₙ ┘

Resolution:
    λ = -W⁻¹ · δ

with per-direction force bounds:
    -λₘₐₓ,ᵢ ≤ λᵢ ≤ λₘₐₓ,ᵢ
```

**Implementation:**
```cpp
// BilateralConstraintResolution1 (1D)
void resolution(int line, double** w, double* d, double* force, double*) {
    force[line] -= d[line] / (w[line][line] + m_compliance);
    
    // Clamp to force limits
    if (force[line] > m_maxForce) 
        force[line] = m_maxForce;
    if (force[line] < -m_maxForce) 
        force[line] = -m_maxForce;
}
```

**Use Cases:**
- Rigid attachments
- Fixed joints
- Perfect coupling (no slip, no separation)

---

### Unilateral Constraints

**Purpose:** Enforce non-penetration while allowing separation (e.g., needle tip-tissue contact before puncture).

**Mathematical Formulation:**

For frictionless unilateral contact:

```
Signorini conditions:
    Φ = (p₁ - p₂) · n ≥ 0    (no penetration)
    λ ≥ 0                     (no tensile force)
    λ · Φ = 0                 (complementarity)

Interpretation:
    - If Φ > 0: no contact → λ = 0
    - If Φ = 0: contact → λ ≥ 0

Resolution:
    λ = -δ / W
    λ = max(0, λ)    // Project to feasible set

Force bounds:
    0 ≤ λ ≤ λₘₐₓ
```

**With Coulomb friction (3D):**

```
Directions:
    n  = normal direction
    t₁ = first tangent direction
    t₂ = second tangent direction

Forces:
    λₙ  = normal force
    λₜ₁ = tangent force 1
    λₜ₂ = tangent force 2

Friction cone:
    √(λₜ₁² + λₜ₂²) ≤ μ · λₙ

where μ = friction coefficient

Resolution algorithm:
    1. Solve normal constraint (unilateral)
    2. If λₙ = 0: no contact → λₜ₁ = λₜ₂ = 0
    3. If λₙ > 0: solve tangent constraints with friction bound
```

**Implementation:**
```cpp
// UnilateralConstraintResolution (frictionless)
void resolution(int line, double** w, double* d, double* force, double*) {
    force[line] -= d[line] / w[line][line];
    
    // Enforce unilaterality
    if (force[line] > m_maxForce)
        force[line] = m_maxForce;
    else if (force[line] < 0)
        force[line] = 0.0;  // No tensile force
}

// UnilateralFrictionResolution (with friction)
void resolution(int line, double** w, double* d, double* force, double*) {
    // 1. Solve normal direction
    double corr0 = -d[line+0] / w[line][line];
    force[line+0] += corr0;
    
    // 2. Check unilateral condition
    if (force[line] < 0) {
        force[line+0] = 0.0;
        force[line+1] = 0.0;
        force[line+2] = 0.0;
        return;
    }
    
    // 3. Clamp normal force
    if (force[line] > m_maxForce0) 
        force[line] = m_maxForce0;
    
    // 4. Solve tangent directions
    // ... (sequential back-substitution)
    
    // 5. Apply friction cone
    double slip_force = force[line+0] * m_friction;
    force[line+1] = clamp(force[line+1], -slip_force, slip_force);
    force[line+2] = clamp(force[line+2], -slip_force, slip_force);
}
```

**Use Cases:**
- Surface contact
- Collision response
- Puncture detection (pre-insertion)

---

### Insertion Constraints

**Purpose:** Model needle-tissue coupling during insertion with friction along the needle shaft.

**Mathematical Formulation:**

Insertion constraints combine bilateral attachment with anisotropic friction:

```
Constraint directions:
    d₁ = shaft tangent direction (sliding direction)
    d₂ = first perpendicular direction
    d₃ = second perpendicular direction

Constraint equations:
    Φ₁ = (p₁ - p₂) · d₁  (tangent)
    Φ₂ = (p₁ - p₂) · d₂ = 0  (bilateral)
    Φ₃ = (p₁ - p₂) · d₃ = 0  (bilateral)

Forces:
    λ₁ = friction force (resists sliding)
    λ₂ = lateral force (bilateral)
    λ₃ = lateral force (bilateral)

Friction model (anisotropic):
    |λ₁| ≤ μ · f_friction
    
where f_friction = friction coefficient (constant or pressure-dependent)

Resolution:
    1. Solve tangent direction with friction:
       λ₁ = -W₁₁⁻¹ · δ₁ · μ
       
    2. Solve perpendicular directions (bilateral) with friction on RHS:
       ┌ λ₂ ┐     ┌ W₂₂ W₂₃ ┐⁻¹  ┌ δ₂ + W₂₁·λ₁ ┐
       └ λ₃ ┘ = - └ W₃₂ W₃₃ ┘    └ δ₃ + W₃₁·λ₁ ┘
```

**Key characteristics:**
- **Tangent direction:** Friction resists sliding but allows motion
- **Perpendicular directions:** Bilateral (no relative motion)
- **Asymmetric compliance:** Easier to slide than to separate laterally

**Implementation:**
```cpp
void resolution(int line, double** w, double* d, double* force, double*) {
    std::array<SReal, 3> corr = {0.0, 0.0, 0.0};
    
    // 1. Solve tangent with friction coefficient
    for (int j = 0; j < 3; j++)
        corr[0] -= invW[0][j] * d[line+j];
    corr[0] *= m_frictionCoeff;
    
    // 2. Back-solve 2x2 system for perpendicular directions
    //    with friction force on RHS
    const SReal denom = w[line+1][line+1] * w[line+2][line+2] 
                      - w[line+1][line+2] * w[line+2][line+1];
    
    const SReal nom1 = w[line+1][line+2] * (d[line+2] + w[line+2][line+0] * corr[0]);
    const SReal nom2 = w[line+2][line+2] * (d[line+1] + w[line+1][line+0] * corr[0]);
    corr[1] = (nom1 - nom2) / denom;
    
    corr[2] = -(d[line+2] + w[line+2][line+0] * corr[0] 
                          + w[line+2][line+1] * corr[1]) / w[line+2][line+2];
    
    // 3. Apply corrections
    for (int i = 0; i < 3; i++) 
        force[line+i] += corr[i];
    
    // 4. Clamp to max forces
    for (int i = 0; i < 3; i++) {
        if (force[line+i] > m_maxForce[i]) 
            force[line+i] = m_maxForce[i];
        else if (force[line+i] < -m_maxForce[i]) 
            force[line+i] = -m_maxForce[i];
    }
}
```

**Use Cases:**
- Needle insertion simulation
- Catheter tracking
- Cable/wire interaction with tissue

---

## Data Structures

### ConstraintNormal

Stores constraint directions for a proximity pair.

```cpp
class ConstraintNormal {
    // 1 to 3 constraint directions
    std::vector<Vec3> m_dirs;
    
    // Get directions
    const std::vector<Vec3>& getDirs() const;
    
    // Compute constraint violations
    void computeViolations(
        unsigned int cid,
        const BaseProximity::SPtr& prox1,
        const BaseProximity::SPtr& prox2,
        linearalgebra::BaseVector* v
    ) const;
};
```

### DetectionOutput

Output from CollisionAlgorithm used as input to constraints.

```cpp
struct DetectionOutput {
    BaseProximity::SPtr prox1;  // First proximity
    BaseProximity::SPtr prox2;  // Second proximity
    double distance;             // Distance between points
    
    // Additional metadata
    unsigned int id;
    unsigned int pairId;
};

using DetectionOutputVector = std::vector<DetectionOutput>;
```

## Constraint Building Process

### Step-by-Step Workflow

```
1. Collision Detection Phase (CollisionAlgorithm)
   └─ InsertionAlgorithm::doDetection()
      └─ Outputs DetectionOutputVector

2. Constraint Building Phase (ConstraintGeometry)
   
   a) BaseConstraint::buildConstraintMatrix()
      │
      ├─ For each DetectionOutput:
      │   │
      │   ├─ Get constraint direction:
      │   │   direction = m_directions->operator()(prox1, prox2)
      │   │
      │   ├─ Create InternalConstraint:
      │   │   ic = InternalConstraint::create(
      │   │       prox1, prox2, 
      │   │       normalFunc,
      │   │       resolutionCreator
      │   │   )
      │   │
      │   └─ Build Jacobians:
      │       ic->buildConstraintMatrix(cId, constraintId)
      │           │
      │           ├─ prox1->buildJacobianConstraint(...,  1.0, ...)
      │           └─ prox2->buildJacobianConstraint(..., -1.0, ...)
      │
      └─ Store InternalConstraint in m_constraints

3. Violation Computation Phase
   
   BaseConstraint::getConstraintViolation()
   └─ For each InternalConstraint:
       └─ ic->getConstraintViolation(v)
           └─ Compute: v[cid] = (p₁ - p₂) · n

4. Resolution Phase
   
   BaseConstraint::getConstraintResolution()
   └─ For each InternalConstraint:
       └─ resTab[offset] = ic->createConstraintResolution()
           └─ Creates appropriate resolution object
               (Bilateral, Unilateral, or Insertion)

5. Solving Phase (SOFA GenericConstraintSolver)
   
   └─ Solves LCP: W·λ + δ = 0
       with constraints from resolution objects
       
6. Force Application Phase
   
   BaseConstraint::storeLambda()
   └─ For each InternalConstraint:
       └─ ic->storeLambda(...)
           ├─ prox1->storeLambda(..., λᵢ)
           └─ prox2->storeLambda(..., λᵢ)
```

## Example: Needle Insertion Pipeline

### Complete Data Flow

```
Needle Tip (PointGeometry) + Tissue Surface (TriangleGeometry)
                    │
                    ├─ AABBBroadPhase (spatial acceleration)
                    │
                    ▼
         InsertionAlgorithm::doDetection()
                    │
                    ├─ Pre-puncture: FindClosestProximity
                    │   → collisionOutput (tip to surface)
                    │
                    └─ Post-puncture: Track coupling points
                        → insertionOutput (shaft to volume)
                    │
                    ▼
    ┌───────────────────────────────────────┐
    │                                       │
    ▼                                       ▼
SecondDirection                     FirstDirection
(surface normal)                    (shaft normal)
    │                                       │
    ▼                                       ▼
ConstraintUnilateral               ConstraintInsertion
    │                                       │
    ├─ Input: collisionOutput              ├─ Input: insertionOutput
    ├─ Normals: from SecondDirection       ├─ Normals: from FirstDirection
    ├─ Creates: UnilateralResolution       ├─ Creates: InsertionResolution
    │   (frictionless or with friction)    │   (with sliding friction)
    │                                       │
    └─ Constraint forces enforce           └─ Constraint forces couple
        non-penetration                         needle and tissue
```

### Parameters

**ConstraintUnilateral (puncture contact):**
- `input`: `@InsertionAlgo.collisionOutput`
- `directions`: `@punctureDirection`
- `mu`: Friction coefficient (optional, default 0)
- `draw_scale`: Visualization scale

**ConstraintInsertion (coupling points):**
- `input`: `@InsertionAlgo.insertionOutput`
- `directions`: `@bindDirection`
- `frictionCoeff`: Sliding friction coefficient
- `draw_scale`: Visualization scale

## Performance Considerations

### Constraint Count

```
Number of constraints per frame:
    N_unilateral = n_contacts × (1 or 3)  // 1 for frictionless, 3 with friction
    N_insertion  = n_coupling × 3          // 3 per coupling point
    N_total      = N_unilateral + N_insertion

LCP size: N_total × N_total matrix
```

### Optimization Strategies

1. **Use DistanceFilter:** Reduce spurious contacts
2. **Limit coupling points:** Control `tipDistThreshold` in InsertionAlgorithm
3. **Coarser collision meshes:** Fewer triangles = fewer potential contacts
4. **Adjust solver tolerance:** Balance accuracy vs. speed

## References

- **SOFA Documentation:** Constraint-based simulation
- **Duriez et al. (2006):** "Realistic haptic rendering of interacting deformable objects in virtual environments"
- **Signorini Problem:** Classical unilateral contact formulation
- **Coulomb Friction:** Friction cone constraints

---

For implementation examples, see:
- `tests/scenes/` - Example SOFA scenes
- `examples/xml_scenes/` - XML scene configurations
- `src/ConstraintGeometry/constraint/` - Resolution implementations
