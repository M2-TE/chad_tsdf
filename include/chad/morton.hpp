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
#include <glm/gtc/type_aligned.hpp>
#include <parallel_hashmap/phmap.h>
#include <morton-nd/mortonND_BMI2.h>
#include "chad/normals.hpp"

struct MortonCode {
    using Iterator = std::vector<std::pair<MortonCode, glm::vec3>>::const_iterator;
    struct Neighbourhood {
        MortonCode::Iterator it_beg;
        MortonCode::Iterator it_end;
    };
    using NeighbourhoodMap = phmap::flat_hash_map<uint64_t, Neighbourhood>;
    using NeighbourhoodIterator = NeighbourhoodMap::const_iterator;
    MortonCode(glm::ivec3 vox_pos): _code(encode(vox_pos)) {}
    MortonCode(uint64_t code): _code(code) {}
    
    void static sort(std::vector<glm::vec3>& points, std::vector<std::pair<MortonCode, glm::vec3>>& morton_codes) {
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
    auto static calc(std::vector<glm::vec3>& points) -> std::vector<std::pair<MortonCode, glm::vec3>> {
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
    // create morton neighbourhood on given level (starting from leaves)
    auto static neighbourhoods(std::vector<std::pair<MortonCode, glm::vec3>>& morton_codes, uint64_t neigh_level)
    -> NeighbourhoodMap {
        // the level up to which is checked to see if two morton codes belong to the same neighbourhood
        NeighbourhoodMap neigh_map;

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
            MortonCode neigh_mc = neigh_it->second.it_beg->first;
            point_mc._code &= mask;
            neigh_mc._code &= mask;

            // check if the current point doesnt fit the neighbourhood
            if (point_mc != neigh_mc) {
                // set past-the-end iterator for current neighbourhood
                neigh_it->second.it_end = morton_it;
                // create a new neighbourhood starting at current point
                Neighbourhood neigh { morton_it, morton_it };
                neigh_it = neigh_map.emplace(point_mc._code, neigh).first;
            }
        }
        // set end() iterator for final neighbourhood
        neigh_it->second.it_end = morton_codes.cend();
        return neigh_map;
    }
    // estimate normals and reject points with insufficient neighbours
    auto static normals(std::vector<std::pair<MortonCode, glm::vec3>>& morton_codes, std::vector<glm::vec3>& points, glm::vec3 pose_pos)
    -> std::vector<glm::vec3> {
        auto beg = std::chrono::steady_clock::now();

        // create neighbourhoods
        static constexpr size_t normal_est_point_count = 25;
        static constexpr size_t neigh_level_min = 1; // min index
        static constexpr size_t neigh_level_max = 8; // max index
        std::vector<NeighbourhoodMap> neigh_maps { neigh_level_max + 1 };
        for (std::size_t i = neigh_level_min - 1; i < neigh_maps.size(); i++) {
            neigh_maps[i] = neighbourhoods(morton_codes, i);
        }

        auto end = std::chrono::steady_clock::now();
        auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
        fmt::println("neig calc {:.2f}", dur);
        beg = std::chrono::steady_clock::now();

        // rejection vector shared between threads
        std::set<uint32_t> rejected_points;
        std::mutex rejected_points_mutex;

        // vector of normals to be filled
        std::vector<glm::vec3> normals { morton_codes.size() };
        auto fnc_approx_norms = [&neigh_maps, &morton_codes, &normals, &pose_pos, &rejected_points_mutex, &rejected_points](size_t offset_level, size_t offset_beg, size_t offset_end) {
            // set up assigned section of neighbourhoods
            NeighbourhoodIterator it_neigh_beg = neigh_maps[offset_level].cbegin();
            NeighbourhoodIterator it_neigh_end = neigh_maps[offset_level].cbegin();
            std::advance(it_neigh_beg, offset_beg);
            std::advance(it_neigh_end, offset_end);

            // set up local rejection vector to avoid contention
            std::vector<uint32_t> rejected_points_local;

            // iterate over assigned neighbourhoods
            for (auto it_neigh_cur = it_neigh_beg; it_neigh_cur != it_neigh_end; it_neigh_cur++) {
                const Neighbourhood& neigh = it_neigh_cur->second;
                // store neighbourhoods to use for normal estimation
                std::vector<Neighbourhood*> adj_neighs;
                size_t neigh_level = offset_level;
                std::size_t point_count = 0;

                // walk up levels until sufficient points are found 
                for (; neigh_level < neigh_maps.size(); neigh_level++) {
                    // get neighbourhood morton code and mask it for current level
                    const MortonCode level_mask = std::numeric_limits<uint64_t>::max() << neigh_level * (uint64_t)3;
                    MortonCode neigh_mc = it_neigh_cur->first;
                    neigh_mc._code &= level_mask._code;
                    glm::ivec3 neigh_pos = neigh_mc.decode();

                    // gather all the adjacent neighbourhoods
                    adj_neighs.clear();
                    for (int32_t z = -1; z <= +1; z++) {
                    for (int32_t y = -1; y <= +1; y++) {
                    for (int32_t x = -1; x <= +1; x++) {
                        // offset based on neighbourhood level
                        glm::ivec3 offset { x, y, z };
                        offset *= 1 << neigh_level;
                        MortonCode near_mc { neigh_pos + offset };
                        // attempt to find adjacent neighbourhood in map
                        auto it_adj_neigh = neigh_maps[neigh_level].find(near_mc._code);
                        if (it_adj_neigh != neigh_maps[neigh_level].cend()) {
                            adj_neighs.push_back(&it_adj_neigh->second);
                        }
                    }}}

                    // count total points in all adjacent neighbourhoods
                    point_count = 0;
                    for (auto it_adj_neigh = adj_neighs.cbegin(); it_adj_neigh != adj_neighs.cend(); it_adj_neigh++) {
                        Neighbourhood& neighbourhood = **it_adj_neigh;
                        point_count += std::distance(neighbourhood.it_beg, neighbourhood.it_end);
                    }

                    // break out early once sufficient points are present
                    if (point_count >= normal_est_point_count) break;
                }

                // check if sufficient points were found
                if (point_count < normal_est_point_count) {
                    // reject all points in this neighbourhood
                    for (auto it_point = neigh.it_beg; it_point != neigh.it_end; it_point++) {
                        size_t index = std::distance(morton_codes.cbegin(), it_point);
                        rejected_points_local.push_back(index);
                    }
                    // skip to next neighbourhood
                    continue;
                }

                // collect all points in all adjacent neighbourhoods
                std::vector<glm::vec3> adj_points;
                adj_points.reserve(point_count);
                for (auto& adj_neigh: adj_neighs) {
                    for (auto it_point = adj_neigh->it_beg; it_point != adj_neigh->it_end; it_point++) {
                        adj_points.push_back(it_point->second);
                    }
                }

                // go over every point
                std::vector<glm::aligned_vec4> nearest_points;
                nearest_points.reserve(point_count);
                for (auto& adj_point: adj_points) {
                    // glm::vec3 diff = adj_point - it_point->second;
                    // float dist_sqr = glm::dot(diff, diff);
                    // if (dist_sqr > dist_max * dist_max) continue;
                    nearest_points.emplace_back(adj_point.x, adj_point.y, adj_point.z, 0);
                }
                for (auto it_point = neigh.it_beg; it_point != neigh.it_end; it_point++) {
                    // const float dist_max = LEAF_RESOLUTION * static_cast<float>(1 << neigh_level) * 1.5f;
                    // get the N nearest points to the current point
                    // nearest_points.clear();
                    // for (auto& adj_point: adj_points) {
                    //     // glm::vec3 diff = adj_point - it_point->second;
                    //     // float dist_sqr = glm::dot(diff, diff);
                    //     // if (dist_sqr > dist_max * dist_max) continue;
                    //     nearest_points.emplace_back(adj_point.x, adj_point.y, adj_point.z, 0);
                    // }

                    // estimate normal for current point
                    glm::vec3 normal = approximate_normal(nearest_points);
                    // flip normal if needed
                    float normal_dot = glm::dot(normal, glm::normalize(it_point->second - pose_pos));
                    if (normal_dot < 0.0f) normal = -normal;

                    // write to shared normal vector
                    size_t point_index = std::distance(morton_codes.cbegin(), it_point);
                    normals[point_index] = normal;
                }
            }

            // write rejected points into shared vector
            std::unique_lock<std::mutex> lock(rejected_points_mutex);
            rejected_points.insert(rejected_points_local.cbegin(), rejected_points_local.cend());
        };

        // balance load across several threads
        std::vector<std::thread> threads;
        size_t thread_count = std::thread::hardware_concurrency();
        threads.reserve(thread_count);
        for (size_t thread_i = 0; thread_i < threads.capacity(); thread_i++) {
            size_t neighs_per_thread = neigh_maps[neigh_level_min].size() / threads.capacity();
            size_t offset_beg = thread_i * neighs_per_thread;
            size_t offset_end;
            if (thread_i == threads.capacity() - 1) offset_end = neigh_maps[neigh_level_min].size();
            else offset_end = offset_beg + neighs_per_thread;
            threads.emplace_back(fnc_approx_norms, neigh_level_min, offset_beg, offset_end);
        }
        // join all threads
        for (auto& thread: threads) thread.join();

        // remove rejected points from normals vector
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
        fmt::println("norm calc {:.2f} ({} rejected)", dur, rejected_points.size());
        return normals;
    }

    auto static normals_old(std::vector<std::pair<MortonCode, glm::vec3>>& morton_codes, std::vector<glm::vec3>& points, glm::vec3 pose_pos)
    -> std::vector<glm::vec3> {
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

        // use chunk at level N (relative from leaves) for neighbourhoods by default
        static std::size_t neigh_level = 1;
        auto neigh_map = fnc_get_neigh_map(neigh_level);
        // rough approximation of points per neighbourhood
        std::size_t pts_per_neigh = morton_codes.size() / neigh_map.size();
        // increase neigh level until the desired amount of points per neighbourhood is reached
        while (pts_per_neigh < CHAD_NORM_NEIGH_LOWER) {
            neigh_level++;
            neigh_map = fnc_get_neigh_map(neigh_level);
            // rough approximation of points per neighbourhood
            pts_per_neigh = morton_codes.size() / neigh_map.size();
            fmt::println("increased normal neighbourhood level to: {}", neigh_level);
        }
        // decrease neigh level if too many points per neighbourhood are present
        while (pts_per_neigh > CHAD_NORM_NEIGH_UPPER && neigh_level > 0) {
            neigh_level--;
            neigh_map = fnc_get_neigh_map(neigh_level);
            // rough approximation of points per neighbourhood
            pts_per_neigh = morton_codes.size() / neigh_map.size();
            fmt::println("decreased normal neighbourhood level to: {}", neigh_level);
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
                std::vector<glm::vec3> local_neighbours;
                local_neighbours.reserve(point_count);
                for (auto neigh_it = adj_neighs.cbegin(); neigh_it != adj_neighs.cend(); neigh_it++) {
                    // go over points in this neighbourhood
                    for (auto point_it = (**neigh_it).beg_it; point_it != (**neigh_it).end_it; point_it++) {
                        local_neighbours.push_back(point_it->second);
                    }
                }

                // reserve some space for a vector of nearest points
                std::vector<glm::aligned_vec4> nearest_points;
                nearest_points.reserve(point_count);

                // for every point within the central neighbourhood, find its nearest neighbours for normal calc
                for (auto point_it = neigh.beg_it; point_it != neigh.end_it; point_it++) {
                    // go over all points to approximate distance
                    nearest_points.clear();
                    for (auto other_it = local_neighbours.begin(); other_it != local_neighbours.end(); other_it++) {
                        glm::vec3 diff = *other_it - point_it->second;
                        float dist_sqr = glm::dot(diff, diff);
                        if (dist_sqr > dist_max * dist_max) continue;
                        else {
                            nearest_points.emplace_back(other_it->x, other_it->y, other_it->z, 0.0);
                        }
                    }

                    // figure out index
                    std::size_t normal_idx = point_it - morton_codes.cbegin();

                    // use these filtered nearest points to approximate the normal
                    if (nearest_points.size() >= CHAD_NORM_MIN_NEIGH) {
                        glm::vec3 normal = approximate_normal(nearest_points);

                        // flip normal if needed
                        float normal_dot = glm::dot(normal, glm::normalize(point_it->second - pose_pos));
                        if (normal_dot < 0.0f) normal = -normal;

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
        // fmt::println("rejected points: {}", rejected_points.size());
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