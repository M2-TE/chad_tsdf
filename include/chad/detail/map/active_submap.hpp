#pragma once
#include "chad/detail/pose.hpp"
#include "chad/detail/map/indices.hpp"



#include "chad/detail/octree.hpp"
#include "chad/detail/map/octree2.hpp"
#include "chad/detail/map/octree3.hpp"



namespace chad::detail::map {
    class ActiveSubmap {
    public:
        // clear only sub-submap related data
        void clear_sub() {
            _sub_poses.clear();
            _sub_points.clear();
        }
        // clear all data
        void clear() {
            clear_sub();
            _all_poses.clear();
            _descriptor_indices.clear();
            _tsdf_octree1.clear();
            _tsdf_octree2.clear();
            _tsdf_octree3.clear();
        }
        // add a single scan frame
        void add_frame(std::vector<glm::aligned_vec3>&& points, const std::vector<glm::aligned_vec3>& normals, Pose pose, float sdf_res, float sdf_trunc) {
            _all_poses.push_back(pose);
            _sub_poses.push_back(pose);

            // immediately integrate points into the tsdf octree
            write_octree<double>(points, normals, pose, sdf_res, sdf_trunc);

            // move data to avoid copies
            _sub_points.insert(_sub_points.end(), std::make_move_iterator(points.begin()), std::make_move_iterator(points.end()));
        }

    private:
        // TODO: spread out across multiple threads?
        // integrate all _sub_* data into TSDF octree via DDA raycast within truncation distance
        template<typename T = double> // needs to be a floating point type
        void write_octree(const std::vector<glm::aligned_vec3>& points, const std::vector<glm::aligned_vec3>& normals, Pose pose, T sdf_res, T sdf_trunc) {
            static_assert(std::is_floating_point_v<T>);
            const T sdf_res_reciprocal = 1.0 / double(sdf_res);
            const glm::aligned_dvec3 position = pose._position;
            std::vector<MortonCode> traversed_voxels;

            double time_total_a = 0.0f;
            double time_total_b = 0.0f;

            // raycast from pose center to each point's voxel
            auto points_it = std::cbegin(points);
            auto normals_it = std::cbegin(normals);
            for (/**/; points_it < std::cend(points); points_it++, normals_it++) {
                auto timestamp_start = std::chrono::steady_clock::now();
                // make it easy to switch between single and double precision
                using glm_float_t = T;
                using glm_vec3f_t = glm::vec<3, glm_float_t, glm::aligned_highp>;
                glm_vec3f_t point = *points_it;
                glm_vec3f_t normal = *normals_it;

                // calculate ray properties within truncation distance
                glm_vec3f_t ray_dir = glm::normalize(point - glm_vec3f_t{ position });
                glm_vec3f_t ray_pos = point - ray_dir * sdf_trunc;
                glm_vec3f_t ray_end = point + ray_dir * sdf_trunc;
                glm::aligned_ivec3 ray_pos_vox = glm::aligned_ivec3{ glm::floor(ray_pos * sdf_res_reciprocal) };
                glm::aligned_ivec3 ray_end_vox = glm::aligned_ivec3{ glm::floor(ray_end * sdf_res_reciprocal) };

                // the step direction corresponding to ray direction
                glm_vec3f_t ray_step = glm::sign(ray_dir);
                glm::aligned_ivec3 ray_step_vox = glm::aligned_ivec3{ ray_step };

                // the step distance to reach the next voxel in each dimension
                glm_vec3f_t ray_delta = glm::abs(sdf_res / ray_dir);

                // the step distance needed to reach the next voxel from current ray_pos
                glm_vec3f_t dim_step = ray_step * (glm_vec3f_t{ ray_pos_vox } * sdf_res - ray_pos);
                dim_step += (ray_step * static_cast<glm_float_t>(0.5) + static_cast<glm_float_t>(0.5)) * sdf_res;
                dim_step *= ray_delta * sdf_res_reciprocal;

                // can already add the first voxel
                traversed_voxels.emplace_back(ray_pos_vox);

                // 1 bit for each completed dimension
                uint32_t completion_mask = 0b000;
                if (ray_pos_vox.x == ray_end_vox.x) completion_mask |= 0b001;
                if (ray_pos_vox.y == ray_end_vox.y) completion_mask |= 0b010;
                if (ray_pos_vox.z == ray_end_vox.z) completion_mask |= 0b100;
                while (completion_mask != 0b111) {
                    if (dim_step.x < dim_step.y) {
                        if (dim_step.x < dim_step.z) {
                            dim_step.x += ray_delta.x;
                            ray_pos_vox.x += ray_step_vox.x;
                            if (ray_pos_vox.x == ray_end_vox.x) completion_mask |= 0b001;
                        }
                        else {
                            dim_step.z += ray_delta.z;
                            ray_pos_vox.z += ray_step_vox.z;
                            if (ray_pos_vox.z == ray_end_vox.z) completion_mask |= 0b100;
                        }
                    }
                    else {
                        if (dim_step.y < dim_step.z) {
                            dim_step.y += ray_delta.y;
                            ray_pos_vox.y += ray_step_vox.y;
                            if (ray_pos_vox.y == ray_end_vox.y) completion_mask |= 0b010;
                        }
                        else {
                            dim_step.z += ray_delta.z;
                            ray_pos_vox.z += ray_step_vox.z;
                            if (ray_pos_vox.z == ray_end_vox.z) completion_mask |= 0b100;
                        }
                    }
                    traversed_voxels.emplace_back(ray_pos_vox);
                }

                // update the traversed octree leaves
                for (const MortonCode& morton_code: traversed_voxels) {
                    // compute signed distance
                    glm_vec3f_t voxel_pos = glm_vec3f_t{ morton_code.decode() } + glm_vec3f_t{ 0.5, 0.5, 0.5 };
                    glm_vec3f_t point_to_voxel = voxel_pos * sdf_res - point;
                    float signed_distance = static_cast<float>(glm::dot(normal, point_to_voxel));
                    signed_distance = std::clamp<float>(signed_distance, -sdf_trunc, +sdf_trunc);
                    // integrate truncated sd measurement into octree

                    // _tsdf_octree1.insert(morton_code);
                    // _tsdf_octree2.insert(morton_code, signed_distance);
                    _tsdf_octree3.insert(morton_code, signed_distance);

                }
                traversed_voxels.clear();
            }
        }

    public:
        // accumulated poses for current submap
        std::vector<Pose> _all_poses;
        // accumulated data for current sub-submap (used for point-to-tsdf loop closure)
        std::vector<Pose> _sub_poses;
        std::vector<glm::aligned_vec3> _sub_points;
        // ndd descriptor indices for each sub-submap
        std::vector<DescriptorIndex> _descriptor_indices;
        // accumulated TSDF data for current submap
        Octree _tsdf_octree1; // DEBUG
        Octree2<17, 2> _tsdf_octree2; // DEBUG
        Octree3<17, 2> _tsdf_octree3;
        // mutex for async safety
        std::mutex _mutex;
    };
}
