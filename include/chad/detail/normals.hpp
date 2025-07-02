#pragma once
#include <chrono>
#include <cstdint>
#include <glm/glm.hpp>
#include <gtl/phmap.hpp>
#include "chad/detail/morton.hpp"

namespace chad::detail {
    // sourced from: https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
    auto inline estimate_normal(const MortonVector::const_iterator beg, const MortonVector::const_iterator end) -> glm::vec3 {
        // calculate centroid by through coefficient average
        glm::dvec3 centroid { 0, 0, 0 };
        for (auto it = beg; it != end; it++) {
            centroid += glm::dvec3(it->first);
        }
        double recip = 1.0 / double(std::distance(beg, end));
        centroid *= recip;

        // covariance matrix excluding symmetries
        double xx = 0.0; double xy = 0.0; double xz = 0.0;
        double yy = 0.0; double yz = 0.0; double zz = 0.0;
        for (auto it = beg; it != end; it++) {
            glm::dvec3 r = glm::dvec3(it->first) - centroid;
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
        glm::dvec3 weighted_dir = { 0, 0, 0 };

        // determinant x
        {
            double det_x = yy*zz - yz*yz;
            glm::dvec3 axis_dir = {
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
            glm::dvec3 axis_dir = {
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
            glm::dvec3 axis_dir = {
                xy*yz - xz*yy,
                xy*xz - yz*xx,
                det_z
            };
            double weight = det_z * det_z;
            if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
            weighted_dir += axis_dir * weight;
        }

        // return normalized weighted direction as surface normal
        return glm::vec3(glm::normalize(weighted_dir));
    }
    auto inline estimate_normals(const MortonVector& points_mc, const glm::vec3 position) -> std::vector<glm::vec3> {
        auto beg = std::chrono::high_resolution_clock::now();

        std::vector<glm::vec3> normals;
        normals.resize(points_mc.size());
        for (auto it = points_mc.cbegin(); it != points_mc.cend();) {

            const uint32_t min_points = 8;
            
            // TODO: check that neighbourhood is as box shaped as possible
            // check morton neighbourhoods for nearby points with increasing discretization
            auto it_neigh_beg = it;
            auto it_neigh_end = it + 1;
            for (uint64_t depth = 0; depth < 3; depth++) {
                // mask to strip the depth*3 LSBs of the morton code to match current discretization level
                const uint64_t mc_mask = std::numeric_limits<uint64_t>::max() << uint64_t(depth * 3);
                const MortonCode mc_neigh = mc_mask & it->second; // discretized morton code

                // walk forward until morton code mismatches
                while (it_neigh_end != points_mc.cend() - 1) /*bounds safety*/ {
                    MortonCode mc_next = mc_mask & it_neigh_end->second;
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
                glm::vec3 normal = estimate_normal(it_neigh_beg, it_neigh_end);

                // flip normal if needed (TODO: should this be moved into the it_neigh loop?)
                float normal_dot = glm::dot(normal, glm::normalize(position - it->first));
                if (normal_dot < 0.0f) normal = -normal;

                // assign normal to all points within neighbourhood
                for (auto it_neigh = it_neigh_beg; it_neigh != it_neigh_end; it_neigh++) {
                    size_t index = std::distance(points_mc.cbegin(), it_neigh);
                    normals[index] = normal;
                }
            }
            // if not, simply use normalized vector from point to position
            else {
                // assign normal to all points within neighbourhood
                for (auto it_neigh = it_neigh_beg; it_neigh != it_neigh_end; it_neigh++) {
                    glm::vec3 normal = glm::normalize(position - it_neigh->first);
                    size_t index = std::distance(points_mc.cbegin(), it_neigh);
                    normals[index] = normal;
                }
            }

            // increment point iterator to mark these points as handled
            it += neigh_size;
        }


        // for (auto& normal: normals) normal = -normal;
        // fmt::println("FLIPPED NORMALS FOR DEBUG RUN");

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("norm est {:.2f}", dur);
        return normals;
    }
}