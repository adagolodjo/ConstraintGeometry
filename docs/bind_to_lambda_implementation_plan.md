# std::bind to Lambda Migration - Implementation Plan

## Overview

This document provides a step-by-step implementation plan for replacing `std::bind` with modern lambda expressions across the CollisionAlgorithm and ConstraintGeometry plugins.

---

## Pre-Implementation Checklist

- [x] Documentation analysis completed
- [x] Changes committed to git
- [ ] Ensure compilation environment is ready
- [ ] Backup current working branch
- [ ] Create feature branch for migration
- [ ] Identify test suite location/availability

---

## Implementation Phases

### Phase 1: Normal Handlers Migration (Low Risk)

**Goal**: Replace `std::bind` in all normal handler classes with lambdas

**Estimated Time**: 30-45 minutes

**Files to Modify**:
1. `src/ConstraintGeometry/normalHandler/EdgeNormalHandler.h`
2. `src/ConstraintGeometry/normalHandler/GravityPointNormalHandler.h`
3. `src/ConstraintGeometry/normalHandler/VectorPointNormalHandler.h`
4. `src/ConstraintGeometry/normalHandler/GouraudTriangleNormalHandler.h`
5. `src/ConstraintGeometry/normalHandler/PhongTriangleNormalHandler.h`

**Steps**:

#### Step 1.1: EdgeNormalHandler.h
```bash
# Line 33
# Replace:
std::bind(&EdgeNormalHandler::getNormal<PROXIMITY>, handler, std::placeholders::_1)

# With:
[handler](const typename PROXIMITY::SPtr & p) {
    return handler->getNormal<PROXIMITY>(p);
}
```

**Verification**:
- [ ] Code compiles without warnings
- [ ] No template instantiation errors
- [ ] Edge normal computation works correctly

#### Step 1.2: GravityPointNormalHandler.h
```bash
# Line 43
# Replace:
std::bind(&GravityPointNormalHandler::getNormal<PROXIMITY>, handler, std::placeholders::_1)

# With:
[handler](const typename PROXIMITY::SPtr & p) {
    return handler->getNormal<PROXIMITY>(p);
}
```

**Verification**:
- [ ] Code compiles without warnings
- [ ] Gravity-based normal computation works correctly

#### Step 1.3: VectorPointNormalHandler.h
```bash
# Line 35
# Replace:
std::bind(&VectorPointNormalHandler::getNormal<PROXIMITY>, handler, std::placeholders::_1)

# With:
[handler](const typename PROXIMITY::SPtr & p) {
    return handler->getNormal<PROXIMITY>(p);
}
```

**Verification**:
- [ ] Code compiles without warnings
- [ ] Vector-based normal computation works correctly

#### Step 1.4: GouraudTriangleNormalHandler.h
```bash
# Line 34 (expected)
# Apply same pattern as above
```

**Verification**:
- [ ] Code compiles without warnings
- [ ] Gouraud shading normal interpolation works correctly

#### Step 1.5: PhongTriangleNormalHandler.h
```bash
# Line 33 (expected)
# Apply same pattern as above
```

**Verification**:
- [ ] Code compiles without warnings
- [ ] Phong shading normal interpolation works correctly

#### Phase 1 Commit Strategy
```bash
git add src/ConstraintGeometry/normalHandler/*.h
git commit -m "refactor: Replace std::bind with lambdas in normal handlers

- EdgeNormalHandler: clearer capture semantics
- GravityPointNormalHandler: explicit parameter types
- VectorPointNormalHandler: improved readability
- GouraudTriangleNormalHandler: better type safety
- PhongTriangleNormalHandler: modern C++ best practices

Benefits:
- Improved code readability and maintainability
- Better compiler error messages
- Alignment with C++ Core Guidelines F.50"
```

---

### Phase 2: Core Logic Migration (Medium Risk)

**Goal**: Replace `std::bind` in constraint and algorithm core classes

**Estimated Time**: 45-60 minutes

**Files to Modify**:
1. `src/ConstraintGeometry/BaseConstraint.h`
2. `../CollisionAlgorithm/src/CollisionAlgorithm/BaseAlgorithm.h`

**Steps**:

#### Step 2.1: BaseConstraint.h (Lines 75-76)

**Before**:
```cpp
auto constraint = InternalConstraint<FIRST,SECOND>::create(
    input[i].first, input[i].second,
    std::bind(&TBaseConstraint::createConstraintNormal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TBaseConstraint::createConstraintResolution, this, std::placeholders::_1)
);
```

**After**:
```cpp
auto constraint = InternalConstraint<FIRST,SECOND>::create(
    input[i].first, input[i].second,
    [this](const typename FIRST::SPtr & first, const typename SECOND::SPtr & second) {
        return this->createConstraintNormal(first, second);
    },
    [this](const BaseInternalConstraint * cst) {
        return this->createConstraintResolution(cst);
    }
);
```

**Testing Requirements**:
- [ ] Constraint creation works correctly
- [ ] Constraint normal computation is accurate
- [ ] Constraint resolution behaves identically
- [ ] No performance regression in constraint building

**Commit**:
```bash
cd /Users/yadagolo/travail/plugin/needle_insertion/ConstraintGeometry
git add src/ConstraintGeometry/BaseConstraint.h
git commit -m "refactor: Replace std::bind with lambdas in BaseConstraint

Replace std::bind with lambdas in processGeometricalData method:
- createConstraintNormal: explicit capture of 'this' and parameter types
- createConstraintResolution: clearer intent and better type safety

Benefits:
- Explicit capture semantics for 'this' pointer
- Better IDE support and autocomplete
- Improved compiler error messages for template mismatches
- Alignment with modern C++ best practices"
```

#### Step 2.2: BaseAlgorithm.h (Line 80)

**Before**:
```cpp
FilterFUNC getFilterFunc() {
    return std::bind(&BaseAlgorithm::acceptFilter, this, std::placeholders::_1, std::placeholders::_2);
}
```

**After**:
```cpp
FilterFUNC getFilterFunc() {
    return [this](const BaseProximity::SPtr& p1, const BaseProximity::SPtr& p2) {
        return this->acceptFilter(p1, p2);
    };
}
```

**Testing Requirements**:
- [ ] Filter pipeline works correctly
- [ ] Filter chaining behaves identically
- [ ] No performance regression in filtering

**Commit**:
```bash
cd /Users/yadagolo/travail/plugin/needle_insertion/CollisionAlgorithm
git add src/CollisionAlgorithm/BaseAlgorithm.h
git commit -m "refactor: Replace std::bind with lambda in BaseAlgorithm

Replace std::bind with lambda in getFilterFunc method:
- Explicit capture of 'this'
- Clear parameter types for proximity filters
- Better inlining potential

Benefits:
- Improved readability of filter function creation
- Better performance due to aggressive inlining
- Alignment with modern C++ best practices"
```

---

### Phase 3: Cleanup and Optimization (Low Risk)

**Goal**: Simplify wrapper functions and update commented code

**Estimated Time**: 20-30 minutes

**Files to Modify**:
1. `src/ConstraintGeometry/ConstraintNormal.h`
2. `../CollisionAlgorithm/src/CollisionAlgorithm/BaseAABBBroadPhase.h`

**Steps**:

#### Step 3.1: ConstraintNormal.h (Line 31)

**Before**:
```cpp
static inline ViolationFunction getViolationFunc() {
    return std::bind(&ConstraintNormal::defaultViolationFunction, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
}
```

**After (Simplified)**:
```cpp
static inline ViolationFunction getViolationFunc() {
    return &defaultViolationFunction;
}
```

**Rationale**: `std::bind` with all placeholders is a no-op wrapper. Direct function pointer is simpler and more efficient.

**Testing Requirements**:
- [ ] Violation function computation unchanged
- [ ] Constraint violation values are identical
- [ ] No compilation issues with function pointer conversion

**Commit**:
```bash
cd /Users/yadagolo/travail/plugin/needle_insertion/ConstraintGeometry
git add src/ConstraintGeometry/ConstraintNormal.h
git commit -m "refactor: Simplify getViolationFunc by removing unnecessary std::bind

Remove std::bind wrapper that only forwards all arguments.
Direct function pointer is simpler, clearer, and more efficient.

Before: std::bind with three placeholders (no-op wrapper)
After: Direct function pointer

Benefits:
- Removes unnecessary indirection
- Clearer code intent
- Potentially faster compilation"
```

#### Step 3.2: BaseAABBBroadPhase.h (Line 30)

**Note**: This line is currently commented out. Document the change for when it's re-enabled.

**Before**:
```cpp
//c_nbox.addCallback(std::bind(&BaseAABBBroadPhase::updateBroadPhase, this));
```

**After**:
```cpp
c_nbox.addCallback([this]() {
    this->updateBroadPhase();
});
```

**Action**: Add comment or update documentation noting the lambda version when callback is re-enabled.

**Commit**:
```bash
cd /Users/yadagolo/travail/plugin/needle_insertion/CollisionAlgorithm
git add src/CollisionAlgorithm/BaseAABBBroadPhase.h
git commit -m "docs: Document lambda replacement for commented callback

Update commented callback code to use lambda instead of std::bind.
When this callback is re-enabled, use the lambda version.

Note: Currently commented out, no functional change."
```

---

## Testing Strategy

### Compilation Testing

After each phase:

```bash
# For ConstraintGeometry
cd /Users/yadagolo/travail/plugin/needle_insertion/ConstraintGeometry
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu)

# For CollisionAlgorithm
cd /Users/yadagolo/travail/plugin/needle_insertion/CollisionAlgorithm
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(sysctl -n hw.ncpu)
```

### Functional Testing

If test scenes are available:

```bash
# Run existing test scenes
# Document location of test scenes and how to run them
# Example:
# runSofa -l ConstraintGeometry scenes/test_constraints.scn
```

### Regression Testing

Compare outputs before and after:
- Constraint violation values
- Normal computations
- Filter acceptance results
- Performance metrics (optional)

---

## Rollback Strategy

Each phase is committed separately, allowing selective rollback:

```bash
# Rollback Phase 3
git revert <phase3-commit-hash>

# Rollback Phase 2
git revert <phase2-commit-hash>

# Rollback Phase 1
git revert <phase1-commit-hash>

# Or reset to before all changes
git reset --hard <commit-before-migration>
```

---

## Risk Mitigation

### Low-Risk Items (Phase 1, Phase 3)
- Simple pattern replacement
- No algorithmic changes
- Easy to verify correctness

### Medium-Risk Items (Phase 2)
- Core constraint logic
- Requires functional testing
- Performance validation recommended

### Mitigation Strategies
1. **Incremental commits**: Each file/phase committed separately
2. **Compilation verification**: Build after each change
3. **Functional testing**: Run test scenes if available
4. **Code review**: Have another developer review changes
5. **Performance profiling**: Optional but recommended for Phase 2

---

## Performance Validation (Optional)

If performance testing is available:

```bash
# Before migration
# Run performance benchmark and save results

# After migration
# Run same benchmark and compare

# Expected: 0-5% improvement or no change
# Red flag: Any performance degradation
```

---

## Post-Migration Tasks

- [ ] Update build documentation if needed
- [ ] Notify team of changes
- [ ] Update code style guide to prefer lambdas over std::bind
- [ ] Consider adding clang-tidy rule to catch future std::bind usage
- [ ] Archive this implementation plan with results

---

## Timeline

| Phase | Duration | Status |
|-------|----------|--------|
| Phase 1: Normal Handlers | 30-45 min | Not Started |
| Phase 2: Core Logic | 45-60 min | Not Started |
| Phase 3: Cleanup | 20-30 min | Not Started |
| Testing & Verification | 30-60 min | Not Started |
| **Total** | **2-3 hours** | **0% Complete** |

---

## Success Criteria

- ✅ All `std::bind` instances replaced with lambdas (except commented code documented)
- ✅ All code compiles without warnings
- ✅ All existing tests pass (if available)
- ✅ No performance regression
- ✅ Code review approved
- ✅ Changes committed with descriptive messages
- ✅ Documentation updated

---

## Notes and Observations

### Discovered During Implementation

*(To be filled during implementation)*

- Any unexpected issues?
- Compiler warnings or errors encountered?
- Performance observations?
- Edge cases discovered?

---

## References

- [Bind to Lambda Analysis](./bind_to_lambda_analysis.md)
- [C++ Core Guidelines F.50](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#f50-use-a-lambda-when-a-function-wont-do-for-function-objects)
- Original commit: [hash to be filled]

---

## Completion

**Date Started**: TBD  
**Date Completed**: TBD  
**Implemented By**: TBD  
**Reviewed By**: TBD  

**Final Status**: ⏳ Pending
