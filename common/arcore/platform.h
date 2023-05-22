#ifndef ARCORE_PLATFORM_H
#define ARCORE_PLATFORM_H

#include "arcore_c_api.h"
#include "huawei_arengine_interface.h"

//ANCHOR_CACHE * ANCHOR_LAYERS is size of max tracked anchors (AREngine's limit is 50)
#define ANCHOR_CACHE 24
#define ANCHOR_DENSITY_BASE 0.1f
#define ANCHOR_DENSITY_SCALE 10.0f
#define ANCHOR_LAYERS 2

#endif
