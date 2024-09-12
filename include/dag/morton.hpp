#pragma once
#include <set>
#include <cstddef>
#include <vector>
#include <chrono>
#include <thread>
#include <utility>
#include <cstdint>
#include <execution>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include <parallel_hashmap/phmap.h>
#include <morton-nd/mortonND_BMI2.h>
#include "dag/normals.hpp"

struct MortonCode {
    MortonCode(glm::ivec3 vox_pos): _code(encode(vox_pos)) {}
    MortonCode(uint64_t code): _code(code) {}
    
    void static sort(std::vector<glm::vec3>& points, std::vector<std::pair<MortonCode, glm::vec3>>& morton_codes);
    auto static calc(std::vector<glm::vec3>& points)
        -> std::vector<std::pair<MortonCode, glm::vec3>>;
    auto static normals(std::vector<std::pair<MortonCode, glm::vec3>>& morton_codes, std::vector<glm::vec3>& points, glm::vec3 pose_pos)
        -> std::vector<glm::vec3>;

    auto static inline encode(glm::ivec3 vox_pos) -> uint64_t {
        // truncate from two's complement 32-bit to 21-bit integer
        uint32_t x, y, z;
        x = (1 << 20) + (uint32_t)vox_pos.x;
        y = (1 << 20) + (uint32_t)vox_pos.y;
        z = (1 << 20) + (uint32_t)vox_pos.z;
        return mortonnd::MortonNDBmi_3D_64::Encode(x, y, z);
    }
    auto static inline decode(uint64_t code) -> glm::ivec3 {
        auto [x, y, z] = mortonnd::MortonNDBmi_3D_64::Decode(code);
        x -= 1 << 20;
        y -= 1 << 20;
        z -= 1 << 20;
        return { (int32_t)x, (int32_t)y, (int32_t)z };
    }
    auto inline decode() -> glm::ivec3 {
        return decode(_code);
    }
    bool inline operator==(const MortonCode& other) const { return _code == other._code; }
    bool inline operator!=(const MortonCode& other) const { return _code != other._code; }
    bool inline operator<(const MortonCode& other) const { return _code < other._code; }
    bool inline operator>(const MortonCode& other) const { return _code > other._code; }
    uint64_t _code;
};

auto MortonCode::calc(std::vector<glm::vec3>& points) -> std::vector<std::pair<MortonCode, glm::vec3>> {
    auto beg = std::chrono::high_resolution_clock::now();

    // create a vector to hold sortable morton codes alongside position
    std::vector<std::pair<MortonCode, glm::vec3>> morton_codes;
    morton_codes.reserve(points.size());

    // iterate through all points to populate morton code vector
    for (auto points_it = points.cbegin(); points_it != points.cend(); points_it++) {
        glm::vec3 position = *points_it;
        // convert to voxel coordinate
        glm::vec3 position_chunk = position * (float)(1.0 / LEAF_RESOLUTION);
        // discretize using floor()
        position_chunk = glm::floor(position_chunk);
        // create morton code and insert into vector
        morton_codes.emplace_back((glm::ivec3)position_chunk, position);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("mort calc {:.2f}", dur);
    return morton_codes;
}
void MortonCode::sort(std::vector<glm::vec3>& points, std::vector<std::pair<MortonCode, glm::vec3>>& morton_codes) {
    auto beg = std::chrono::steady_clock::now();
    
    // sort morton code vector
    auto sorter = [](const auto& a, const auto& b){
        return a.first > b.first;
    };
    std::sort(std::execution::par_unseq, morton_codes.begin(), morton_codes.end(), sorter);
    
    // insert sorted morton code points back into points vector
    auto points_it = points.begin();
    for (auto morton_it = morton_codes.cbegin(); morton_it != morton_codes.cend(); morton_it++) {
        *points_it++ = morton_it->second;
    }
    
    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("pnts sort {:.2f}", dur);
}
auto MortonCode::normals(std::vector<std::pair<MortonCode, glm::vec3>>& morton_codes, std::vector<glm::vec3>& points, glm::vec3 pose_pos) -> std::vector<glm::vec3> {
    auto beg = std::chrono::steady_clock::now();
    
    typedef std::vector<std::pair<MortonCode, glm::vec3>>::const_iterator MortonIt;
    struct Neighbourhood {
        MortonIt beg_it;
        MortonIt end_it;
    };
    // lambda function to get a map of neighbourhoods
    auto fnc_get_neigh_map = [&morton_codes](uint64_t neigh_level) {
        // the level up to which is checked to see if two morton codes belong to the same neighbourhood
        phmap::flat_hash_map<uint64_t, Neighbourhood> neigh_map;

        // create mask to group morton codes up to a certain level
        const uint64_t mask = std::numeric_limits<uint64_t>::max() << neigh_level * (uint64_t)3;
        MortonCode neigh_mc = morton_codes.front().first;
        neigh_mc._code &= mask;
        
        // insert the first neighbourhood
        Neighbourhood neigh { morton_codes.cbegin(), morton_codes.cbegin() };
        auto neigh_it = neigh_map.emplace(neigh_mc._code, neigh).first;

        // iterate through all points to build neighbourhoods
        for (auto morton_it = morton_codes.cbegin() + 1; morton_it != morton_codes.cend(); morton_it++) {
            // get morton codes from current point and current neighbourhood
            MortonCode point_mc = morton_it->first;
            MortonCode neigh_mc = neigh_it->second.beg_it->first;
            point_mc._code &= mask;
            neigh_mc._code &= mask;

            // check if the current point doesnt fit the neighbourhood
            if (point_mc != neigh_mc) {
                // set past-the-end iterator for current neighbourhood
                neigh_it->second.end_it = morton_it;
                // create a new neighbourhood starting at current point
                Neighbourhood neigh { morton_it, morton_it };
                neigh_it = neigh_map.emplace(point_mc._code, neigh).first;
            }
        }
        // set end() iterator for final neighbourhood
        neigh_it->second.end_it = morton_codes.cend();
        return neigh_map;
    };

    // use chunk at level 2 (relative from leaves) for neighbourhoods by default
    static std::size_t neigh_level = 2;
    auto neigh_map = fnc_get_neigh_map(neigh_level);
    // rough approximation of points per neighbourhood
    std::size_t pts_per_neigh = morton_codes.size() / neigh_map.size();
    // decrease neigh level if too many points per neighbourhood are present
    while (pts_per_neigh > CHAD_NORM_NEIGH_UPPER && neigh_level > 0) {
        neigh_level--;
        neigh_map = fnc_get_neigh_map(neigh_level);
        // rough approximation of points per neighbourhood
        pts_per_neigh = morton_codes.size() / neigh_map.size();
        fmt::println("decreased normal neighbourhood level to: {}", neigh_level);
    }
    // increase neigh level until the desired amount of points per neighbourhood is reached
    while (pts_per_neigh < CHAD_NORM_NEIGH_LOWER) {
        neigh_level++;
        neigh_map = fnc_get_neigh_map(neigh_level);
        // rough approximation of points per neighbourhood
        pts_per_neigh = morton_codes.size() / neigh_map.size();
        fmt::println("increased normal neighbourhood level to: {}", neigh_level);
    }

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("neig calc {:.2f}", dur);
    beg = std::chrono::steady_clock::now();

    // rejection vector shared between threads
    std::set<uint32_t> rejected_points;
    std::mutex rejected_points_mutex;

    // approximate normals using local neighbourhoods
    const float dist_max = LEAF_RESOLUTION * (1 << neigh_level) * CHAD_NORM_RADIUS_MOD;
    std::vector<glm::vec3> normals(morton_codes.size());
    // create lambda function for norm approximations
    typedef decltype(neigh_map)::const_iterator NeighIt;
    auto fnc_approx_norms = [&normals, &neigh_map, &morton_codes, &rejected_points, &rejected_points_mutex, pose_pos, dist_max](NeighIt beg, NeighIt end) {
        std::set<uint32_t> rejected_points_local;
        for (auto neigh_it = beg; neigh_it != end; neigh_it++) {
            // get morton code for current neighbourhood
            const Neighbourhood& neigh = neigh_it->second;
            MortonCode neigh_mc = neigh_it->first;
            glm::ivec3 neigh_pos = neigh_mc.decode();

            // gather adjacent neighbourhoods
            std::vector<Neighbourhood*> adj_neighs;
            for (int32_t z = -1; z <= +1; z++) {
            for (int32_t y = -1; y <= +1; y++) {
            for (int32_t x = -1; x <= +1; x++) {
                // offset based on neighbourhood level
                glm::ivec3 offset { x, y, z };
                offset *= 1 << neigh_level;
                MortonCode near_mc { neigh_pos + offset };
                // attempt to find adjacent neighbourhood in map
                auto near_it = neigh_map.find(near_mc._code);
                if (near_it != neigh_map.cend()) {
                    adj_neighs.push_back(&near_it->second);	
                }
            }}}

            // count total points in all adjacent neighbourhoods
            std::size_t point_count = 0;
            for (auto neigh_it = adj_neighs.cbegin(); neigh_it != adj_neighs.cend(); neigh_it++) {
                point_count += (**neigh_it).end_it - (**neigh_it).beg_it;
            }

            // collect all points in all adjacent neighbourhoods
            std::vector<glm::vec3> points_local;
            points_local.reserve(point_count);
            for (auto neigh_it = adj_neighs.cbegin(); neigh_it != adj_neighs.cend(); neigh_it++) {
                // go over points in this neighbourhood
                for (auto point_it = (**neigh_it).beg_it; point_it != (**neigh_it).end_it; point_it++) {
                    points_local.push_back(point_it->second);
                }
            }

            //  reserve some generous space for nearest points
            std::vector<glm::dvec4> nearest_points;
            nearest_points.reserve(point_count);
            // for every point within the central neighbourhood, find its nearest neighbours for normal calc
            for (auto point_it = neigh.beg_it; point_it != neigh.end_it; point_it++) {
                // every point will have its own set of nearest points
                nearest_points.clear();

                // go over all points and store the nearest ones (within bounds)
                for (auto other_it = points_local.cbegin(); other_it != points_local.cend(); other_it++) {
                    glm::vec3 diff = *other_it - point_it->second;
                    float dist_sqr = glm::dot(diff, diff);
                    // add point if it falls within bounds
                    if (dist_sqr <= dist_max * dist_max) {
                        nearest_points.emplace_back(other_it->x, other_it->y, other_it->z, 0.0);
                    }
                }

                // figoure out index
                std::size_t normal_idx = point_it - morton_codes.cbegin();

                // use these filtered nearest points to approximate the normal
                if (nearest_points.size() >= CHAD_NORM_MIN_NEIGH) {
                    glm::vec3 normal = approximate_normal(nearest_points);
                    // flip normal if needed
                    float normal_dot = glm::dot(normal, point_it->second - pose_pos);
                    if (abs(normal_dot) < CHAD_NORM_MIN_DOT) {
                        rejected_points_local.emplace(normal_idx);
                        continue;
                    }
                    else if (normal_dot < 0.0f) normal = -normal;

                    // write to shared normal vector
                    normals[normal_idx] = normal;
                }
                // reject point if it doesnt have enough neighbours for estimation
                else rejected_points_local.emplace(normal_idx);
            }
        }
        
        // write rejected points into shared vector
        std::unique_lock<std::mutex> lock(rejected_points_mutex);
        rejected_points.insert(rejected_points_local.cbegin(), rejected_points_local.cend());
    };
    // balance load across several threads using lambda invocation
    std::vector<std::thread> threads;
    std::size_t thread_count = std::thread::hardware_concurrency();
    threads.reserve(thread_count);
    for (std::size_t thread_i = 0; thread_i < threads.capacity(); thread_i++) {
        // neighbours per thread
        std::size_t neighs_per_thread = neigh_map.size() / threads.capacity();
        // prep iterators
        NeighIt beg = neigh_map.cbegin();
        std::advance(beg, thread_i * neighs_per_thread);
        NeighIt end;
        if (thread_i == threads.capacity() - 1) {
            end = neigh_map.cend();
        }
        else {
            end = beg;
            std::advance(end, neighs_per_thread);
        }
        threads.emplace_back(fnc_approx_norms, beg, end);
    }

    // join all threads
    for (auto& thread: threads) thread.join();

    // remove rejected points from normals vector
    fmt::println("rejected points: {}", rejected_points.size());
    std::vector<glm::vec3> normals_filtered;
    std::vector<glm::vec3> points_filtered;
    normals_filtered.reserve(normals.size() - rejected_points.size());
    points_filtered.reserve(points.size() - rejected_points.size());
    for (std::size_t i = 0; i < normals.size(); i++) {
        if (rejected_points.find(i) == rejected_points.cend()) {
            normals_filtered.push_back(normals[i]);
            points_filtered.push_back(points[i]);
        }
    }
    normals = normals_filtered;
    points = points_filtered;

    end = std::chrono::steady_clock::now();
    dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("norm calc {:.2f}", dur);
    return normals;
}