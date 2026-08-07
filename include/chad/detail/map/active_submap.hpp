#pragma once
#include "chad/detail/misc/pose.hpp"
#include "chad/detail/map/indices.hpp"
#include "chad/detail/map/octree2.hpp"

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
            _tsdf_octree.clear();
        }
        // add a single scan frame
        void add_frame(std::vector<glm::aligned_vec3>&& points, const std::vector<glm::aligned_vec3>& normals, Pose pose, float sdf_res, float sdf_trunc) {
            std::lock_guard lock{ _mutex };
            _all_poses.push_back(pose);
            _sub_poses.push_back(pose);

            // immediately integrate points into the tsdf octree
            write_octree<double>(points, normals, pose, sdf_res, sdf_trunc);

            // move data to avoid copies
            _sub_points.insert(_sub_points.end(), std::make_move_iterator(points.begin()), std::make_move_iterator(points.end()));
        }

    private:
        // integrate all _sub_* data into TSDF octree via DDA raycast within truncation distance
        template<typename T = double> // needs to be a floating point type
        void write_octree(const std::vector<glm::aligned_vec3>& points, const std::vector<glm::aligned_vec3>& normals, Pose pose, T sdf_res, T sdf_trunc) {
            static_assert(std::is_floating_point_v<T>);
            const glm::aligned_dvec3 position = pose._position;
            const T sdf_res_reciprocal = 1.0 / double(sdf_res);
            std::vector<MortonCode> traversed_voxels;

            // raycast from pose center to each point's voxel
            auto points_it = std::cbegin(points);
            auto normals_it = std::cbegin(normals);
            for (/**/; points_it < std::cend(points); points_it++, normals_it++) {
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
                traversed_voxels.push_back(ray_pos_vox);

                // 1 bit for each completed dimension
                uint32_t completion_mask = 0b000;
                if (ray_pos_vox.x == ray_end_vox.x) completion_mask |= 0b001;
                if (ray_pos_vox.y == ray_end_vox.y) completion_mask |= 0b010;
                if (ray_pos_vox.z == ray_end_vox.z) completion_mask |= 0b100;

                // perform DDA
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
                    traversed_voxels.push_back(ray_pos_vox);
                }

                // update the traversed octree leaves
                for (const MortonCode& morton_code: traversed_voxels) {
                    // compute signed distance
                    glm_vec3f_t voxel_pos = glm_vec3f_t{ morton_code.decode() } + glm_vec3f_t{ 0.5, 0.5, 0.5 };
                    glm_vec3f_t point_to_voxel = voxel_pos * sdf_res - point;
                    float signed_distance = static_cast<float>(glm::dot(normal, point_to_voxel));
                    signed_distance = std::clamp<float>(signed_distance, -sdf_trunc, +sdf_trunc);
                    // integrate truncated sd measurement into octree
                    _tsdf_octree.insert(morton_code, signed_distance);
                }
                traversed_voxels.clear();
            }
        }

        // TODO
        template<typename T = double> // needs to be a floating point type
        void write_octree_async(const std::vector<glm::aligned_vec3>& points, const std::vector<glm::aligned_vec3>& normals, Pose pose, T sdf_res, T sdf_trunc) {
            static_assert(std::is_floating_point_v<T>);
            const glm::aligned_dvec3 position = pose._position;

            // make it easy to switch between single and double precision
            using glm_float_t = T;
            using glm_vec3f_t = glm::vec<3, glm_float_t, glm::aligned_highp>;
            constexpr std::size_t swapchain_slice_count = 3; // triple buffering usually works best
            constexpr std::size_t swapchain_slice_size = 1024; // every vector in swapchain should have a capacity guideline
            // swapchain of vectors containing traversed voxels
            std::array<std::vector<MortonCode>, swapchain_slice_count> swapchain_slices;
            // one atomic for each slice to signify whether it is valid
            // -> thread_dda: may only write "invalid" slices -> validates after
            // -> thread_oct: may only read "valid" slices -> invalidates after
            std::array<std::atomic_flag, 3> swapchain_slice_valid; // initialized to clear state in C++20
            std::atomic_flag swapchain_done; // initialized to clear state in C++20

            // thread work: raycast from pose center to each point's voxel
            std::jthread thread_dda{ [&, position, sdf_res, sdf_trunc]() {
                const T sdf_res_reciprocal = 1.0 / double(sdf_res);
                std::size_t swapchain_slice_i = 0;

                // swap local traversed_voxels with slice
                std::vector<MortonCode> traversed_voxels;
                auto perform_swap = [&]() {
                    // wait for slice to become invalidated
                    auto& flag = swapchain_slice_valid[swapchain_slice_i];
                    while (flag.test(std::memory_order_acquire)) {
                        flag.wait(true, std::memory_order_relaxed);
                    }
                    // efficiently swap data with slice
                    traversed_voxels.swap(swapchain_slices[swapchain_slice_i]);
                    traversed_voxels.clear();
                    // set slice to valid
                    flag.test_and_set(std::memory_order_release);
                    flag.notify_one();
                    // move to next slice
                    swapchain_slice_i = (swapchain_slice_i + 1) % swapchain_slice_count;
                };

                auto points_it = std::cbegin(points);
                auto normals_it = std::cbegin(normals);
                for (; points_it < std::cend(points); points_it++, normals_it++) {
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
                    traversed_voxels.push_back(ray_pos_vox);

                    // 1 bit for each completed dimension
                    uint32_t completion_mask = 0b000;
                    if (ray_pos_vox.x == ray_end_vox.x) completion_mask |= 0b001;
                    if (ray_pos_vox.y == ray_end_vox.y) completion_mask |= 0b010;
                    if (ray_pos_vox.z == ray_end_vox.z) completion_mask |= 0b100;

                    // perform DDA
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

                    // fill slice with data if enough voxels were accumulated
                    if (traversed_voxels.size() >= swapchain_slice_size) {
                        perform_swap();
                    }
                }
                // write the last bit of voxel data
                if (traversed_voxels.size() > 0) {
                    perform_swap();
                    swapchain_done.test_and_set(std::memory_order_release);
                }
            }};

            // thread work: insert traversed voxels into octree
            std::jthread thread_oct{ [&]() {
                std::size_t swapchain_slice_i = 0;
                while (true) {
                    // wait until the current slice becomes valid
                    auto& flag = swapchain_slice_valid[swapchain_slice_i];
                    while (!flag.test(std::memory_order_acquire)) {
                        flag.wait(false, std::memory_order_relaxed);
                    }

                    // // insert all the voxels into the octree
                    // for (const auto& morton_code: swapchain_slices[swapchain_slice_i]) {
                    //     // compute signed distance
                    //     glm_vec3f_t voxel_pos = glm_vec3f_t{ morton_code.decode() } + glm_vec3f_t{ 0.5, 0.5, 0.5 };
                    //     glm_vec3f_t point_to_voxel = voxel_pos * sdf_res - point;
                    //     float signed_distance = static_cast<float>(glm::dot(normal, point_to_voxel));
                    //     signed_distance = std::clamp<float>(signed_distance, -sdf_trunc, +sdf_trunc);
                    //     // integrate truncated sd measurement into octree
                    //     _tsdf_octree2.insert(morton_code, signed_distance);
                    // }

                    // invalidate the slice


                    // TODO: repeat until swapchain_done is triggered
                    // TODO: wont that be kind of a deadlock?
                }
            }};
        }
    public:
        // accumulated poses for current submap
        std::vector<Pose> _all_poses;
        // accumulated data for current sub-submap (used for point-to-tsdf loop closure)
        [[deprecated]] std::vector<Pose> _sub_poses; // TODO: these poses might be unnecessary
        [[deprecated]] std::vector<glm::aligned_vec3> _sub_points;
        // ndd descriptor indices for each sub-submap
        std::vector<DescriptorIndex> _descriptor_indices;
        // accumulated TSDF data for current submap
        Octree2<17, 2> _tsdf_octree;
        // mutex for async safety
        std::mutex _mutex;
    };
}
