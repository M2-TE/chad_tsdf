#pragma once
#include <iostream>
#include <set>
#include <array>
#include <cstddef>
#include <sstream>
#include <vector>
#include <cstdint>
#include <condition_variable>
#include <fmt/base.h>
#include <glm/glm.hpp>
#include <parallel_hashmap/phmap.h>
#include "chad/morton.hpp"
#include <bitset>

struct Octree {
    typedef uint64_t Key; // only 63 bits valid
    typedef uint64_t Leaf;
    union Node {
        struct {
            glm::dvec3 accu_pos;
            glm::dvec3 accu_norm;
            size_t accu_count;
            // size_t padding;
        } leaf_accu;
        std::array<Node*, 8> children;
    };
    struct MemoryBlock {
        MemoryBlock(): _size(0) {
            void* mem_p = std::aligned_alloc(sizeof(Node), _capacity * sizeof(Node));
            _data_p = static_cast<Node*>(mem_p);
        }
        ~MemoryBlock() {
            if (_data_p) std::free(_data_p);
        }
        MemoryBlock(const MemoryBlock&) = delete;
        MemoryBlock(MemoryBlock&& other) {
            _size = other._size;
            _data_p = other._data_p;
            other._size = 0;
            other._data_p = nullptr;
        }
        MemoryBlock& operator=(const MemoryBlock&) = delete;
        MemoryBlock& operator=(MemoryBlock&&) = delete;

        auto allocate_node() -> Node* {
            Node* node_p = _data_p + _size++;
            std::memset(node_p, 0, sizeof(Node));
            return node_p;
        }

        Node* _data_p;
        std::size_t _size;
        static constexpr std::size_t _capacity = 1 << 16;
    };

    Octree() {
        // create initial memory block
        _mem_blocks.emplace_back();
        // insert root node
        _root_p = _mem_blocks.back().allocate_node();
        std::memset(_root_p, 0, sizeof(Node));
    }
    ~Octree() = default;
    Octree(const Octree&) = delete;
    Octree(Octree&& other) {
        _mem_blocks = std::move(other._mem_blocks);
        _root_p = other._root_p;
    }
    Octree& operator=(const Octree&) = delete;
    Octree& operator=(Octree&&) = delete;

    // find node at a given depth without checking for validity of path
    auto find(Key key, uint_fast32_t target_depth) -> Node* {
        // depth from 20 (root) to 0 (leaf)
        uint_fast32_t depth_r = 63/3 - 1;
        // reverse target depth to fit internal depth iterator
        target_depth = depth_r - target_depth;
        Node* node_p = _root_p;
        for (;depth_r >= target_depth; depth_r--) {
            uint64_t shift = 3 * depth_r;
            uint64_t idx = (key >> shift) & 0b111;
            node_p = node_p->children[idx];
        }
        return node_p;
    }
    auto insert(Key key) -> Node* {
        // check if memory block has enough space left
        if (_mem_blocks.back()._capacity - _mem_blocks.back()._size < 63/3) {
            _mem_blocks.emplace_back();
        }
        auto& mem_block = _mem_blocks.back();

        // depth from 20 (root) to 0 (leaf)
        int_fast32_t depth_r = 63/3 - 1;
        Node* node_p = _root_p;
        for (;depth_r >= 0; depth_r--) {
            uint64_t shift = 3 * depth_r;
            uint64_t idx = (key >> shift) & 0b111;
            if (node_p->children[idx] == nullptr) {
                node_p->children[idx] = mem_block.allocate_node();
                // TODO: could continue to create nodes here, since entire subtree is missing
            }
            node_p = node_p->children[idx];
        }
        return node_p;
    }
    auto insert(Key key, uint_fast32_t target_depth) -> Node* {
        // depth from 20 (root) to 0 (leaf)
        uint_fast32_t rDepth = 63/3 - 1;
        // reverse target depth to fit internal depth iterator
        target_depth = rDepth - target_depth;
        
        // check if memory block has enough space left
        if (_mem_blocks.back()._capacity - _mem_blocks.back()._size < 63/3) {
            _mem_blocks.emplace_back();
        }
        auto& mem_block = _mem_blocks.back();
        
        // go from root to leaf parent
        Node* pNode = _root_p;
        while (rDepth >= target_depth) {
            size_t index = (key >> rDepth*3) & 0b111;
            if (pNode->children[index] == nullptr) {
                pNode->children[index] = mem_block.allocate_node();
            }
            pNode = pNode->children[index];
            rDepth--;
        }
        // return pointer to the requested child node
        return pNode;
    }
    // allocate node from most recent memory block, creating a new one if necessary
    auto allocate_node() -> Node* {
        // check if remaining space is sufficient for a single node
        if (_mem_blocks.back()._capacity == _mem_blocks.back()._size) {
            _mem_blocks.emplace_back();
        }
        Node* node_p = _mem_blocks.back().allocate_node();
        return node_p;
    }
    // merge up to certain depth where collision target is met
    auto static merge(Octree& dst, Octree& src, uint_fast32_t collision_target) -> std::pair<std::vector<Key>, uint_fast32_t> {
        std::vector<Key> collisions;
        phmap::flat_hash_map<Key, Node*> a_parents;
        phmap::flat_hash_map<Key, Node*> a_layer, b_layer;
        uint32_t prev_collision_count = 0;
        uint_fast32_t depth = 0;

        // set up initial roots
        a_layer[0] = dst._root_p;
        b_layer[0] = src._root_p;

        // build initial hash map for a_layer
        for (uint_fast8_t i = 0; i < 8; i++) {

        }

        // move memory block to merge target (pointers remain stable)
        for (auto& memblock: src._mem_blocks) {
            dst._mem_blocks.emplace_back(std::move(memblock));
        }

        // breadth first iteration through both trees
        for (; depth < 63/3; depth++) {
            collisions.clear();

            // go through current layer and replace it with the next one
            phmap::flat_hash_map<Key, Node*> a_new;
            for (auto& node: a_layer) {
                // go over all children
                for (uint64_t i = 0; i < 8; i++) {
                    Node* child_p = node.second->children[i];
                    if (child_p != nullptr) {
                        // construct key for this child node
                        uint64_t key = i << (60 - depth*3);
                        key |= node.first;
                        a_new[key] = child_p;
                    }
                }
            }
            phmap::flat_hash_map<Key, Node*> b_new;
            for (auto& node: b_layer) {
                // go over all children
                for (uint64_t i = 0; i < 8; i++) {
                    Node* child_p = node.second->children[i];
                    if (child_p != nullptr) {
                        // construct key for this child node
                        uint64_t key = i << (60 - depth*3);
                        key |= node.first;
                        b_new[key] = child_p;
                    }
                }
            }

            // overwrite old layers
            a_parents = std::move(a_layer);
            a_layer = std::move(a_new);
            b_layer = std::move(b_new);

            // check for collisions between the two layers a and b
            std::vector<Key> b_queued_removals;
            for (auto& node: b_layer) {
                Key key = node.first;
                // if collision occurs, save it
                if (a_layer.contains(key)) {
                    collisions.push_back(key);
                }
                // when node is missing from layer_a, insert it as new child
                else {
                    // index of new child node (0 to 7)
                    Key child_key = key >> (60 - depth*3);
                    child_key &= 0b111;
                    // remove child part from key to get parent key
                    Key parent_key = key ^ (child_key << (60 - depth*3));

                    // use parent key to find the parent node in previous layer
                    Node* parent_p = a_parents[parent_key];
                    // insert new child into it
                    parent_p->children[child_key] = node.second;
                    // queue node for removal from layer_b
                    b_queued_removals.push_back(key);
                }
            }
            
            // remove nodes from b that were already inserted into a
            for (auto& key: b_queued_removals) b_layer.erase(key);
            if (b_layer.empty()) {
                // return early, merging is already complete
                fmt::println("breaking out of octree merge early");
                collisions.clear();
                depth = 63/3;
                break;
            }
            if (prev_collision_count > 0) {
                // break if collision target is met
                if (collisions.size() >= collision_target) break;
            }
            prev_collision_count = collisions.size();
        }
        // fmt::println("Collisions: {} ({}) {}", collisions.size(), collision_target, depth);
        // return the remaining collisions alongside their depth
        return { collisions, depth };
    }
    // fully  merge  via previously found collisions (invalidates src octree)
    void static merge(Octree& dst, Octree& src, Key start_key, uint_fast32_t start_depth, uint32_t thread_i) {
        const uint_fast32_t path_length = 63/3 - start_depth;
        if (path_length <= 1) {
            fmt::println("no merging job");
            return;
        }
        std::vector<uint_fast8_t> path(path_length);
        std::vector<Node*> nodes_a(path_length);
        std::vector<Node*> nodes_b(path_length);
        // find the colliding nodes in both trees
        path[0] = 0;
        nodes_a[0] = dst.find(start_key, start_depth);
        nodes_b[0] = src.find(start_key, start_depth);
        uint_fast32_t depth = 0;

        // begin traversal
        while (path[0] <= 8) {
            auto child_i = path[depth]++;
            if (child_i >= 8) {
                // go back up to parent
                depth--;
                continue;
            }
            
            // retrieve child nodes
            Node* a_child_p = nodes_a[depth]->children[child_i];
            Node* b_child_p = nodes_b[depth]->children[child_i];
            
            // if this node is missing from B, simply skip it
            // else if this node is missing from A, add from B
            if (b_child_p == nullptr) continue;
            else if (a_child_p == nullptr) {
                nodes_a[depth]->children[child_i] = b_child_p;
                continue;
            }
            
            // if child nodes are leaf clusters, resolve collision between leaves
            if (depth == path_length - 3) { // todo: adjust actual path_length variable?
                // reconstruct morton code from path
                uint64_t code = start_key;
                for (uint64_t k = 0; k < path_length - 2; k++) { // todo: adjust for new path length thing
                    uint64_t part = path[k] - 1;
                    uint64_t shift = path_length*3 - k*3 - 6;
                    code |= part << shift;
                }
                // convert to actual cluster chunk position
                glm::ivec3 cluster_chunk = MortonCode::decode(code);

                uint8_t leaf_i = 0;
                for (int32_t z = 0; z <= 1; z++) {
                for (int32_t y = 0; y <= 1; y++) {
                for (int32_t x = 0; x <= 1; x++, leaf_i++) {
                    // get clusters of closest points to this leaf
                    Node* leaf_a = a_child_p->children[leaf_i];
                    Node* leaf_b = b_child_p->children[leaf_i];

                    // check validity of leaf nodes
                    if (leaf_b == nullptr) continue;
                    else if (leaf_a == nullptr) {
                        a_child_p->children[leaf_i] = leaf_b;
                        continue;
                    }

                    // combine accumulated leaf candidates in both leaves
                    leaf_a->leaf_accu.accu_pos += leaf_b->leaf_accu.accu_pos;
                    leaf_a->leaf_accu.accu_norm += leaf_b->leaf_accu.accu_norm;
                    leaf_a->leaf_accu.accu_count += leaf_b->leaf_accu.accu_count;
                }}}
            }
            else {
                // walk down path
                depth++;
                path[depth] = 0;
                nodes_a[depth] = a_child_p;
                nodes_b[depth] = b_child_p;
            }
        }
    }

    Node* _root_p;
    std::vector<MemoryBlock> _mem_blocks;
};

// // todo: needs a better name to distinguish between octree::insert and this
// void static octree_insert_point_new(Octree& octree, const glm::vec3* point_p) {
//     // discretize position to leaf chunk
//     glm::vec3 chunk = *point_p * (float)(1.0 / LEAF_RESOLUTION);
//     glm::ivec3 chunk_center = (glm::ivec3)glm::floor(chunk);
//     glm::ivec3 chunk_base = chunk_center;
//     // write to leaves in a 4x4x4 voxel radius around point (+2 since point is floored)
//     for (int_fast32_t z = chunk_base.z - 1; z <= chunk_base.z + 2; z++) {
//     for (int_fast32_t y = chunk_base.y - 1; y <= chunk_base.y + 2; y++) {
//     for (int_fast32_t x = chunk_base.x - 1; x <= chunk_base.x + 2; x++) {
//         // get leaf position in voxel space and insert into octree
//         glm::ivec3 chunk_leaf = chunk_base + glm::ivec3(x, y, z);
//         MortonCode mc { chunk_leaf };
//         Octree::Node* leaf_p = octree.insert(mc._code);
//         // add point_p if leaf candidate array has space left
//         bool is_inserted = false;
//         for (uint_fast8_t i = 0; i < leaf_p->leaf_candidates.size(); i++) {
//             if (leaf_p->leaf_candidates[i] == nullptr) {
//                 leaf_p->leaf_candidates[i] = point_p;
//                 is_inserted = true;
//                 break;
//             }
//         }
//         if (is_inserted) continue;
//         // get leaf position in world space to calc distance
//         glm::vec3 pos_leaf = (glm::vec3)chunk_leaf * (float)LEAF_RESOLUTION;
//         glm::vec3 diff = *point_p - pos_leaf;
//         float dist_sqr = glm::dot(diff, diff);
//         // try to replace furthest point with this one
//         float dist_sqr_max = 0.0f;
//         uint_fast8_t dist_sqr_max_i = 0;
//         for (uint_fast8_t i = 0; i < leaf_p->leaf_candidates.size(); i++) {
//             glm::vec3 diff = *leaf_p->leaf_candidates[i] - pos_leaf;
//             float dist_sqr = glm::dot(diff, diff);
//             if (dist_sqr > dist_sqr_max) {
//                 dist_sqr_max = dist_sqr;
//                 dist_sqr_max_i = i;
//             }
//         }
//         // replace furthest point if closer
//         if (dist_sqr < dist_sqr_max) {
//             leaf_p->leaf_candidates[dist_sqr_max_i] = point_p;
//         }
//     }}}
// }

void inline octree_insert_point(Octree& octree, const glm::vec3 point, const glm::vec3 normal) {
    // discretize position to leaf chunk
    glm::vec3 chunk = point * (float)(1.0 / LEAF_RESOLUTION);
    glm::ivec3 chunk_center = (glm::ivec3)glm::floor(chunk);

    // set constraints for which leaf clusters get created
    glm::ivec3 min = chunk_center + glm::ivec3(-1, -1, -1);
    glm::ivec3 max = chunk_center + glm::ivec3(+2, +2, +2);

    // fill large 3x3x3 neighbourhood around point with signed distances. Most will be discarded
    glm::ivec3 chunk_base = chunk_center - chunk_center % 4;
    for (int32_t z4 = -4; z4 <= +4; z4 += 4) {
    for (int32_t y4 = -4; y4 <= +4; y4 += 4) {
    for (int32_t x4 = -4; x4 <= +4; x4 += 4) {
        // position of current main chunk
        glm::ivec3 chunk_base4 = chunk_base + glm::ivec3(x4, y4, z4);
        MortonCode mc{ chunk_base4 };
        // mc._code = mc._code >> 3; // shift to cover the 3 LSB (leaves), which wont be encoded
        Octree::Node* oct_node = octree.insert(mc._code, 18);

        // iterate over 2x2x2 sub chunks within the 4x4x4 main chunk
        uint8_t cluster_i = 0;
        for (int32_t z2 = 0; z2 <= 2; z2 += 2) {
        for (int32_t y2 = 0; y2 <= 2; y2 += 2) {
        for (int32_t x2 = 0; x2 <= 2; x2 += 2, cluster_i++) {
            glm::ivec3 chunk_base2 = chunk_base4 + glm::ivec3(x2, y2, z2);
            Octree::Node* cluster = oct_node->children[cluster_i];
            // if it doesnt exist yet, allocate it manually
            if (cluster == nullptr) {
                cluster = octree.allocate_node();
                oct_node->children[cluster_i] = cluster;
            }

            // iterate over leaves within clusters
            uint8_t leaf_i = 0;
            for (int32_t z1 = 0; z1 <= 1; z1++) {
            for (int32_t y1 = 0; y1 <= 1; y1++) {
            for (int32_t x1 = 0; x1 <= 1; x1++, leaf_i++) {
                glm::ivec3 chunk_leaf = chunk_base2 + glm::ivec3(x1, y1, z1);
                // test if leaf falls within the constraints for current scan point
                bool valid = true;
                if (chunk_leaf.x < min.x || chunk_leaf.x > max.x) valid = false;
                if (chunk_leaf.y < min.y || chunk_leaf.y > max.y) valid = false;
                if (chunk_leaf.z < min.z || chunk_leaf.z > max.z) valid = false;
                if (!valid) continue;

                // // if valid, calc real position of leaf
                // glm::vec3 pos_leaf = (glm::vec3)chunk_leaf * (float)LEAF_RESOLUTION;
                // // check if voxel is within truncation distance squared
                // glm::vec3 diff = point - pos_leaf;
                // float dist_sqr = glm::dot(diff, diff);
                // if (dist_sqr > LEAF_RESOLUTION*LEAF_RESOLUTION*2) continue;

                Octree::Node* accu_leaf_p = cluster->children[leaf_i];
                // create new leaf if it doesnt exist yet
                if (accu_leaf_p == nullptr) {
                    accu_leaf_p = octree.allocate_node();
                    cluster->children[leaf_i] = accu_leaf_p;
                }
                // accumulate the points contributing to this voxel
                accu_leaf_p->leaf_accu.accu_pos += (glm::dvec3)point;
                accu_leaf_p->leaf_accu.accu_norm += (glm::dvec3)normal;
                accu_leaf_p->leaf_accu.accu_count++;
            }}}
        }}}
    }}}
}

auto inline octree_build(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals) -> Octree {
    auto beg = std::chrono::steady_clock::now();

    // round threads down to nearest power of two
    uint_fast32_t lz = __builtin_clz(std::thread::hardware_concurrency());
    uint_fast32_t thread_count = 1 << (32 - lz - 1);
    std::vector<std::thread> threads;
    threads.reserve(thread_count);
    std::vector<Octree> octrees { thread_count };

    // build smaller octrees on several threads
    uint_fast32_t progress = 0;
    for (uint_fast32_t thread_i = 0; thread_i < thread_count; thread_i++) {
        uint_fast32_t point_count = points.size() / thread_count;
        if (thread_i == thread_count - 1) point_count = 0; // special value for final thread to read the rest
        threads.emplace_back([&points, &normals, &octrees, progress, thread_i, point_count](){
            // set start/end iterators for this thread (points + normals)
            auto norm_it = normals.cbegin() + progress;
            auto cur_it = points.cbegin() + progress;
            auto end_it = (point_count == 0) ? points.cend() : (cur_it + point_count);
            Octree& octree = octrees[thread_i];
            // Octree::PathCache cache(octree);
            for (; cur_it != end_it; cur_it++, norm_it++) {
                octree_insert_point(octree, *cur_it, *norm_it);
            }
        });
        progress += point_count;
    }
    for (auto& thread: threads) thread.join();
    threads.clear();

    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("trie ctor {:.2f}", dur);
    beg = std::chrono::steady_clock::now();

    // thread groups durng merge stages
    struct ThreadGroup {
        // no sync needed
        std::vector<Octree::Key> collisions;
        std::array<Octree*, 2> trees;
        uint32_t lead_thread_i;
        uint32_t collision_depth;
        // sync needed
        std::unique_ptr<std::condition_variable> cv;
        std::unique_ptr<std::mutex> mutex;
        uint32_t completed_count = 0;
        bool is_prepared = false;
    };
    struct Stage {
        uint32_t group_size; // threads per group
        std::vector<ThreadGroup> groups;
    };
    // calc number of total stages
    uint_fast32_t stage_count = std::sqrt(thread_count);
    std::vector<Stage> stages { stage_count };
    // initialize thread groups
    for (uint_fast32_t i = 0; i < stage_count; i++) {
        auto& stage = stages[i];

        // figure out how many groups there are
        stage.group_size = 2 << i;
        uint32_t group_count = thread_count / stage.group_size;

        // fill each group with info data
        stage.groups.resize(group_count);
        for (uint32_t group_i = 0; group_i < group_count; group_i++) {
            auto& group = stage.groups[group_i];
            group.lead_thread_i = group_i * stage.group_size;
            group.cv = std::make_unique<std::condition_variable>();
            group.mutex = std::make_unique<std::mutex>();
        }
    }
    // merge octrees in several stages
    for (uint_fast32_t thread_i = 0; thread_i < thread_count; thread_i++) {
        threads.emplace_back([&octrees, &stages, thread_i]() {
            // progress via stages
            for (uint_fast32_t stage_i = 0; stage_i < stages.size(); stage_i++) {
                auto& stage = stages[stage_i];
                uint32_t group_i = thread_i / stage.group_size;
                uint32_t local_i = thread_i % stage.group_size; // group-local index
                auto& group = stage.groups[group_i];

                // lead thread preps the data for merging
                if (group.lead_thread_i == thread_i) {
                    // wait for all dependent groups to finish
                    if (stage_i > 0) {
                        auto& stage_prev = stages[stage_i - 1];
                        uint32_t group_prev_i = thread_i / stage_prev.group_size;
                        // wait for group A to finish
                        auto& group_a = stage_prev.groups[group_prev_i];
                        std::unique_lock lock_a(*group_a.mutex);
                        group_a.cv->wait(lock_a, [&]{ 
                            return group_a.completed_count >= stage_prev.group_size; 
                        });
                        lock_a.release();
                        // wait for group B to finish
                        auto& group_b = stage_prev.groups[group_prev_i + 1];
                        std::unique_lock lock_b(*group_b.mutex);
                        group_b.cv->wait(lock_b, [&]{ 
                            return group_b.completed_count >= stage_prev.group_size; 
                        });
                        lock_b.release();
                    }

                    // set up pointers for this group's octrees
                    group.trees[0] = &octrees[thread_i];
                    group.trees[1] = &octrees[thread_i + stage.group_size / 2];

                    // target collisions equal to group size for a balanced load
                    uint_fast32_t collision_target = stage.group_size;
                    std::tie(group.collisions, group.collision_depth) = 
                        Octree::merge(*group.trees[0], *group.trees[1], collision_target);
                    // signal that this group is prepared
                    group.mutex->lock();
                    group.is_prepared = true;
                    group.mutex->unlock();
                    group.cv->notify_all();
                }
                // fmt::println("stage {} thread {} group {} local {}", stage_i, thread_i, group_i, local_i);
                // wait until this group is prepared
                std::unique_lock lock(*group.mutex);
                group.cv->wait(lock, [&]{ return group.is_prepared; });
                lock.unlock();

                // resolve assigned collision(s)
                uint32_t collision_i = local_i;
                while (collision_i < group.collisions.size()) {
                    Octree::merge(
                        *group.trees[0], *group.trees[1], 
                        group.collisions[collision_i], 
                        group.collision_depth, thread_i); // DEBUG: pass thread id for prints
                    collision_i += stage.group_size;
                }

                // increment completion count
                group.mutex->lock();
                group.completed_count++;
                group.mutex->unlock();
                group.cv->notify_one();
            }
        });
    }
    // join all threads
    for (auto& thread: threads) thread.join();
    threads.clear();

    end = std::chrono::steady_clock::now();
    dur = std::chrono::duration<double, std::milli> (end - beg).count();
    fmt::println("trie merg {:.2f}", dur);
    return std::move(octrees[0]);
}