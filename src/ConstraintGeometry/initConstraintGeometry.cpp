#include <ConstraintGeometry/initConstraintGeometry.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/PluginManager.h>
#include <sofa/helper/system/SetDirectory.h>
#include <stdio.h>
#include <string.h>

#define Q(x) #x
#define QUOTE(x) Q(x)

namespace sofa::constraintgeometry {

// Constraints
extern void registerConstraintUnilateral(sofa::core::ObjectFactory* factory);
extern void registerConstraintBilateral(sofa::core::ObjectFactory* factory);
extern void registerConstraintInsertion(sofa::core::ObjectFactory* factory);

// Constraint Directions
extern void registerBindDirection(sofa::core::ObjectFactory* factory);
extern void registerContactDirection(sofa::core::ObjectFactory* factory);
extern void registerFirstDirection(sofa::core::ObjectFactory* factory);
extern void registerFixedFrameDirection(sofa::core::ObjectFactory* factory);
extern void registerSecondDirection(sofa::core::ObjectFactory* factory);

// Normal Handlers
extern void registerEdgeNormalHandler(sofa::core::ObjectFactory* factory);
extern void registerGouraudTriangleNormalHandler(sofa::core::ObjectFactory* factory);
extern void registerGravityPointNormalHandler(sofa::core::ObjectFactory* factory);
extern void registerPhongTriangleNormalHandler(sofa::core::ObjectFactory* factory);
extern void registerVectorPointNormalHandler(sofa::core::ObjectFactory* factory);

extern "C" {
SOFA_CONSTRAINTGEOMETRY_API void initExternalModule();
SOFA_CONSTRAINTGEOMETRY_API const char* getModuleName();
SOFA_CONSTRAINTGEOMETRY_API const char* getModuleVersion();
SOFA_CONSTRAINTGEOMETRY_API const char* getModuleLicense();
SOFA_CONSTRAINTGEOMETRY_API const char* getModuleDescription();
SOFA_CONSTRAINTGEOMETRY_API void registerObjects(sofa::core::ObjectFactory* factory);
}

void initConstraintGeometry() {
    static bool first = true;
    if (first) {
        // make sure that this plugin is registered into the PluginManager
        sofa::helper::system::PluginManager::getInstance().registerPlugin(MODULE_NAME);

        first = false;
#ifdef PLUGIN_DATA_DIR
        sofa::helper::system::DataRepository.addLastPath(std::string(QUOTE(PLUGIN_DATA_DIR)));
#endif
        sofa::helper::system::DataRepository.addLastPath(sofa::helper::system::SetDirectory::GetCurrentDir());
    }
}

void initExternalModule() {
    initConstraintGeometry();
}

const char* getModuleName() {
    return MODULE_NAME;
}

const char* getModuleVersion() {
#ifdef PLUGIN_GIT_INFO
    return QUOTE(PLUGIN_GIT_INFO);
#else
    return "??? to get the last git hash you must active the setupGit macro in CMakeLists";
#endif
}

const char* getModuleLicense() {
    return "LGPL";
}

const char* getModuleDescription() {
    return "Plugin to handle constraints";
}

void registerObjects(sofa::core::ObjectFactory* factory) {
    // Constraints
    registerConstraintUnilateral(factory);
    registerConstraintBilateral(factory);
    registerConstraintInsertion(factory);

    // Constraint Directions
    registerBindDirection(factory);
    registerContactDirection(factory);
    registerFirstDirection(factory);
    registerFixedFrameDirection(factory);
    registerSecondDirection(factory);

    // Normal Handlers
    registerEdgeNormalHandler(factory);
    registerGouraudTriangleNormalHandler(factory);
    registerGravityPointNormalHandler(factory);
    registerPhongTriangleNormalHandler(factory);
    registerVectorPointNormalHandler(factory);
}

}  // namespace sofa::constraintgeometry
