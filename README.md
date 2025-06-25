# CHAD TSDF

CHAD TSDF data structure for SLAM with 3D mesh reconstruction.

| Platform | Build Status |
|:--------:|:------------:|
| Linux    | [![Build on Ubuntu](https://github.com/M2-TE/chad_tsdf/actions/workflows/ubuntu.yaml/badge.svg)](https://github.com/M2-TE/chad_tsdf/actions/workflows/ubuntu.yaml) |

## Description
TSDF mapping backend with real-time capabilities and high compression.
It is tailored towards large-scale maps, while still being able to handle smaller voxel sizes (~0.05).

## Getting Started
### LVR2
The 3D mesh reconstruction currently relies on [LVR2](https://github.com/uos/lvr2), so you will likely need to build and install it yourself.
If not present, chad_tsdf will attempt to build it dynamically via CMake's FetchContent mechanism, but this method may fail on some systems and will still require the LVR2 dependencies.

### CMake
```cmake
include(FetchContent)
FetchContent_Declare(chad_tsdf
    GIT_REPOSITORY "https://github.com/M2-TE/chad_tsdf.git"
    GIT_TAG "main"
    GIT_SHALLOW ON)
FetchContent_MakeAvailable(chad_tsdf)
target_link_libraries(${PROJECT_NAME} PUBLIC chad::tsdf)
```
### C++

```cpp
#include <chad/tsdf.hpp>

int main() {
    float sdf_size = 0.05f;
    float sdf_trunc = 0.10f;
    chad::TSDFMap map(sdf_size, sdf_trunc);

    // insert glm points
    std::vector<glm::vec3> points_glm;
    glm::vec3 position_glm;
    map.insert(points_glm, position_glm);

    // insert eigen points
    std::vector<Eigen::Vector3f> points_eigen;
    Eigen::Vector3f position_eigen;
    map.insert(points_eigen, position_eigen);

    // reconstruct 3D mesh
    map.save("mesh.ply");
    return 0;
}
```

## Roadmap
* [x] Compressed sparse TSDF map
* [x] Submapping
* [ ] Loop closure
* [ ] Space carving

## Acknowledgments

* [gtl](https://github.com/greg7mdp/gtl)
* [libmorton](https://github.com/Forceflow/libmorton)
* [lvr2](https://github.com/uos/lvr2)
