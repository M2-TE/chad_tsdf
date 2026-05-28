#pragma once
#include "chad/detail/pose.hpp"
#include "chad/detail/mapping/octree2.hpp"
#include "chad/detail/mapping/indices.hpp"

namespace chad::detail::mapping {
    struct ActiveSubmap {
        void add_data(const std::vector<glm::aligned_vec3>& points, const std::vector<glm::aligned_vec3>& normals, Pose pose) {
            _all_poses.push_back(pose);
            _sub_poses.push_back(pose);
            _sub_points.insert(_sub_points.end(), points.cbegin(), points.cend());
            _sub_normals.insert(_sub_normals.end(), normals.cbegin(), normals.cend());
        }

        void clear_sub() {
            _sub_poses.clear();
            _sub_points.clear();
            _sub_normals.clear();
        }
        void clear_all() {
            _all_poses.clear();
            clear_sub();
            _descriptor_indices.clear();
            _tsdf_octree.clear();
        }

        std::vector<Pose> _all_poses;
        // accumulated data for current sub-submap
        std::vector<Pose> _sub_poses;
        std::vector<glm::aligned_vec3> _sub_points;
        std::vector<glm::aligned_vec3> _sub_normals;
        // ndd descriptor indices for each sub-submap
        std::vector<DescriptorIndex> _descriptor_indices;
        // accumulated TSDF data for current submap
        Octree2 _tsdf_octree;
    };
}
