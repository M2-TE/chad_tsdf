#pragma once
#include "chad/detail/morton_code.hpp"

namespace {
    // sourced from: https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
    auto inline estimate_normal(std::vector<glm::aligned_vec3>::const_iterator beg, std::vector<glm::aligned_vec3>::const_iterator end) -> glm::aligned_vec3 {
        // calculate centroid by through coefficient average
        glm::aligned_dvec3 centroid { 0, 0, 0 };
        for (auto it = beg; it != end; it++) {
            centroid += glm::aligned_dvec3{ *it };
        }
        double recip = 1.0 / double(std::distance(beg, end));
        centroid *= recip;

        // covariance matrix excluding symmetries
        double xx = 0.0; double xy = 0.0; double xz = 0.0;
        double yy = 0.0; double yz = 0.0; double zz = 0.0;
        for (auto it = beg; it != end; it++) {
            glm::aligned_dvec3 r = glm::aligned_dvec3{ *it } - centroid;
            xx += r.x * r.x;
            xy += r.x * r.y;
            xz += r.x * r.z;
            yy += r.y * r.y;
            yz += r.y * r.z;
            zz += r.z * r.z;
        }
        xx *= recip;
        xy *= recip;
        xz *= recip;
        yy *= recip;
        yz *= recip;
        zz *= recip;

        // weighting linear regression based on square determinant
        glm::aligned_dvec3 weighted_dir = { 0, 0, 0 };

        // determinant x
        {
            double det_x = yy*zz - yz*yz;
            glm::aligned_dvec3 axis_dir = {
                det_x,
                xz*yz - xy*zz,
                xy*yz - xz*yy
            };
            double weight = det_x * det_x;
            if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
            weighted_dir += axis_dir * weight;
        }
        // determinant y
        {
            double det_y = xx*zz - xz*xz;
            glm::aligned_dvec3 axis_dir = {
                xz*yz - xy*zz,
                det_y,
                xy*xz - yz*xx
            };
            double weight = det_y * det_y;
            if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
            weighted_dir += axis_dir * weight;
        }
        // determinant z
        {
            double det_z = xx*yy - xy*xy;
            glm::aligned_dvec3 axis_dir = {
                xy*yz - xz*yy,
                xy*xz - yz*xx,
                det_z
            };
            double weight = det_z * det_z;
            if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
            weighted_dir += axis_dir * weight;
        }

        // return normalized weighted direction as surface normal
        return glm::aligned_vec3(glm::normalize(weighted_dir));
    }
}

namespace chad::detail::funcs {
    // estimate normals for given vector of (sorted!) points
    auto inline estimate_normals(const std::vector<glm::aligned_vec3>& points, glm::aligned_vec3 position, float sdf_res) -> std::vector<glm::aligned_vec3> {
        // min points per neighbourhood for valid normal estimation
        constexpr std::uint32_t min_points = 8;
        // reciprocal of voxel resolution for later
        const float sdf_res_reciprocal = static_cast<float>(1.0 / double(sdf_res));

        // The idea here is to use points within the same discretized MortonCode group for normal estimation.
        // Since the input vector is already sorted, we can simply increment our iterator until the MortonCode mismatches
        // This makes it pretty cache friendly! Calculating the MortonCode on the fly is not too expensive.
        std::vector<glm::aligned_vec3> normals;
        normals.resize(points.size());
        for (auto it = points.cbegin(); it != points.cend(); /* increment is handled inside */) {

            // TODO: check that neighbourhood is as box shaped as possible
            // check morton neighbourhoods for nearby points with increasing discretization
            auto it_neigh_beg = it;
            auto it_neigh_end = it + 1;
            for (uint64_t depth = 0; depth < 3; depth++) {
                // mask to strip the depth*3 LSBs of the morton code to match current discretization level
                const uint64_t mc_mask = std::numeric_limits<uint64_t>::max() << uint64_t(depth * 3);
                const MortonCode mc_neigh = mc_mask & MortonCode{ *it, sdf_res_reciprocal }; // discretized morton code

                // increment forward until discretized morton code mismatches
                while (it_neigh_end != points.cend() - 1) /* bounds safety */ {
                    MortonCode mc_next = mc_mask & MortonCode{ *it_neigh_end, sdf_res_reciprocal }; // discretized morton code
                    if (mc_next == mc_neigh) it_neigh_end++;
                    else break;
                }

                // break out of loop once the neighbourhood has sufficient points for normal estimation
                if (std::distance(it_neigh_beg, it_neigh_end) >= min_points) break;
            }

            // estimate normal via neighbourhood if enough points where found
            uint32_t neigh_size = std::distance(it_neigh_beg, it_neigh_end);
            if (neigh_size >= min_points) {
                // estimate via neighbourhood
                glm::aligned_vec3 normal = estimate_normal(it_neigh_beg, it_neigh_end);

                // flip normal if needed (TODO: should this be moved into the it_neigh loop?)
                float normal_dot = glm::dot(normal, glm::normalize(position - *it));
                if (normal_dot < 0.0f) normal = -normal;

                // assign normal to all points within neighbourhood
                for (auto it_neigh = it_neigh_beg; it_neigh != it_neigh_end; it_neigh++) {
                    size_t index = std::distance(points.cbegin(), it_neigh);
                    normals[index] = normal;
                }
            }
            // if not, simply use normalized vector from point to position
            else {
                // assign normal to all points within neighbourhood
                for (auto it_neigh = it_neigh_beg; it_neigh != it_neigh_end; it_neigh++) {
                    glm::aligned_vec3 normal = glm::normalize(position - *it_neigh);
                    size_t index = std::distance(points.cbegin(), it_neigh);
                    normals[index] = normal;
                }
            }

            // increment point iterator to mark these points as handled
            it += neigh_size;
        }
        return normals;
    }
}
