#pragma once
#include <array>
#include <vector>
#include <cstdint>
#include <sys/mman.h>
#include <glm/glm.hpp>
#include <Eigen/Eigen>
#include "chad/level.hpp"

namespace chad {
    struct Submap {
        uint32_t _root_tsdfs;
        uint32_t _root_weights;

        // glm::vec3 _bb_min;
        // glm::vec3 _bb_max;
    };

    struct TSDFMap {
        // Default constructor for ChadTSDF
        TSDFMap(float voxel_resolution = 0.05f);
        // Pointcloud insertion using Eigen::Vector3f points
        void insert(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f position);
        // Pointcloud insertion using glm::vec3 points
        void insert(const std::vector<glm::vec3>& points, const glm::vec3 position);

    private:
        const float _voxel_resolution;
        std::vector<Submap> _submaps;
        std::array<NodeLevel, 20> _node_levels;
        LeafClusterLevel _leafcluster_level;
    };
}
