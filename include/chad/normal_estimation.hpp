#pragma once
#include <chrono>
#include <vector>
#include <cstdint>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include "chad/morton_code.hpp"


namespace chad::detail {
    using MortonVector = std::vector<std::pair<glm::vec3, MortonCode>>;
    struct MortonNeighbourhood { MortonVector::const_iterator neigh_beg, neigh_end; };
    using MortonNeighbourhoodMap = gtl::parallel_flat_hash_map<MortonCode, MortonNeighbourhood>;
    auto inline build_neighbourhood_map(const MortonVector& points_mc, const uint32_t level) -> MortonNeighbourhoodMap {
        auto beg = std::chrono::high_resolution_clock::now();

        // create mask to group morton codes up to a certain level
        const uint64_t neigh_mask = std::numeric_limits<uint64_t>::max() << size_t(level * 3);

        // create hash map for neighbourhoods, reserving generous space beforehand
        MortonNeighbourhoodMap neigh_map;
        static constexpr size_t min_points_per_neighbourhood = 8;
        neigh_map.reserve(points_mc.size() / min_points_per_neighbourhood);

        // insert the first neighbourhood
        MortonCode neigh_mc = points_mc[0].second;
        neigh_mc._value &= neigh_mask;
        MortonNeighbourhood neigh { points_mc.cbegin(), points_mc.cbegin() };
        auto neigh_it = neigh_map.emplace(neigh_mc, neigh).first;

        uint32_t neigh_count = 0; // track number of neighbourhoods
        uint32_t neigh_points = 0; // track total points within all neighbourhoods
        static constexpr uint32_t neigh_threshhold_checkup = 256; // perform point count check after this neigh_count
        static constexpr uint32_t neigh_threshhold_abort = neigh_threshhold_checkup * min_points_per_neighbourhood;

        // iterate through all points to build neighbourhoods
        for (auto morton_it = points_mc.cbegin()+1; morton_it != points_mc.cend(); morton_it++) {
            // get morton codes from current point and current neighbourhood
            MortonCode point_mc = morton_it->second;
            MortonCode neigh_mc = neigh_it->second.neigh_beg->second;
            point_mc._value &= neigh_mask;
            neigh_mc._value &= neigh_mask;

            // check if the current point doesnt fit the neighbourhood
            if (point_mc != neigh_mc) {
                // set end() iterator for current neighbourhood
                neigh_it->second.neigh_end = morton_it;
                // count points and neighbourhoods
                neigh_count++;
                neigh_points += std::distance(neigh_it->second.neigh_beg, morton_it);
                if (neigh_count >= neigh_threshhold_checkup) {
                    // ensure enough points were placed in the neighbourhoods
                    if (neigh_points < neigh_threshhold_abort) {
                        fmt::println("neigh {}  (aborted)", level);
                        return {};
                    }
                }
                // create a new neighbourhood starting at current point
                MortonNeighbourhood neigh { morton_it, morton_it };
                neigh_it = neigh_map.emplace(point_mc, neigh).first;
            }
        }
        // set end() iterator for final neighbourhood
        neigh_it->second.neigh_end = points_mc.cend();

        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("neigh {}  {:.2f}", level, dur);
        return neigh_map;
    }

    // sourced from: https://www.ilikebigbits.com/2017_09_25_plane_from_points_2.html
    template<typename T, glm::qualifier P>
    auto inline estimate_normal(const std::vector<glm::vec<4, T, P>>& points) -> glm::vec3 {
        // calculate centroid by through coefficient average
        glm::vec<4, T, P> centroid { 0, 0, 0, 0 };
        for (const auto& point: points) {
            centroid += point;
        }
        T recip = 1.0 / (T)points.size();
        centroid *= recip;

        // covariance matrix excluding symmetries
        T xx = 0.0; T xy = 0.0; T xz = 0.0;
        T yy = 0.0; T yz = 0.0; T zz = 0.0;
        for (auto point_it = points.cbegin(); point_it != points.cend(); point_it++) {
            auto r = *point_it - centroid;
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
        glm::vec<4, T, P> weighted_dir = { 0, 0, 0, 0 };

        // determinant x
        {
            T det_x = yy*zz - yz*yz;
            glm::vec<4, T, P> axis_dir = {
                det_x,
                xz*yz - xy*zz,
                xy*yz - xz*yy,
                0.0
            };
            T weight = det_x * det_x;
            if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
            weighted_dir += axis_dir * weight;
        }
        // determinant y
        {
            T det_y = xx*zz - xz*xz;
            glm::vec<4, T, P> axis_dir = {
                xz*yz - xy*zz,
                det_y,
                xy*xz - yz*xx,
                0.0
            };
            T weight = det_y * det_y;
            if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
            weighted_dir += axis_dir * weight;
        }
        // determinant z
        {
            T det_z = xx*yy - xy*xy;
            glm::vec<4, T, P> axis_dir = {
                xy*yz - xz*yy,
                xy*xz - yz*xx,
                det_z,
                0.0
            };
            T weight = det_z * det_z;
            if (glm::dot(weighted_dir, axis_dir) < 0.0) weight = -weight;
            weighted_dir += axis_dir * weight;
        }

        // return normalized weighted direction as surface normal
        weighted_dir = glm::normalize(weighted_dir);
        return { float(weighted_dir.x), float(weighted_dir.y), float(weighted_dir.z) };
    }
    auto inline estimate_normals(const MortonVector& points_mc, const glm::vec3 position) -> std::vector<glm::vec3> {
        auto beg = std::chrono::high_resolution_clock::now();

        // build morton code neighbourhood maps 
        MortonNeighbourhoodMap neigh_map;
        for (uint32_t i = 0; i < 8; i++) {
            neigh_map = build_neighbourhood_map(points_mc, i);
            if (!neigh_map.empty()) break;
        }
        // verify that it found a fitting neighbourhood map
        if (neigh_map.empty()) fmt::println("Too few points inserted for this voxel resolution");


        auto end = std::chrono::high_resolution_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("nei calc {:.2f}", dur);
        
        beg = std::chrono::high_resolution_clock::now();
        std::vector<glm::vec3> normals;
        // TODO

        end = std::chrono::high_resolution_clock::now();
        dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("norm est {:.2f}", dur);
        return normals;
    }
}