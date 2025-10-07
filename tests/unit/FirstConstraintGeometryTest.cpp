#include <gtest/gtest.h>
#include <sofa/core/objectmodel/BaseObject.h>

// Basic test to ensure the test framework is working.
TEST(ConstraintGeometryTest, SanityCheck) {
    // Create a basic SOFA object
    sofa::core::objectmodel::BaseObject::SPtr obj = SOFA_BASE_OBJECT_FACTORY_CREATE(sofa::core::objectmodel::BaseObject);
    
    // Check that it was created
    ASSERT_NE(obj, nullptr);
    EXPECT_EQ(obj->getName(), "BaseObject");
}

// You can add more tests here for your components.
