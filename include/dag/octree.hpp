#pragma once
#include <array>
#include <cstddef>
#include <vector>
#include <cstdint>
#include <condition_variable>
#include <glm/glm.hpp>
#include <parallel_hashmap/phmap.h>
#include "dag/morton.hpp"

struct Octree {
    typedef uint64_t Key; // only 63 bits valid
    typedef uint64_t Leaf;
    union Node {
        std::array<const glm::vec3*, 8> leaf_candidates;
        std::array<Node*, 8> children;
    };
    struct MemoryBlock {
        MemoryBlock(): _size(0) {
            void* mem_p = std::aligned_alloc(sizeof(Node), _capacity * sizeof(Node));
            _data_p = static_cast<Node*>(mem_p);
        }
        ~MemoryBlock() {
            if (_data_p) {
                std::free(_data_p);
            }
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
            return node_p;
        }

        Node* _data_p;
        std::size_t _size;
        static constexpr std::size_t _capacity = 1 << 16;
    };

    Octree() = default;
    ~Octree() = default;
    Octree(const Octree&) = delete;
    Octree(Octree&&) = delete;
    Octree& operator=(const Octree&) = delete;
    Octree& operator=(Octree&&) = delete;

    // find node at a given depth without checking for validity of path
    auto find(Key key, uint_fast32_t target_depth) -> Node* {
        // depth from 20 (root) to 0 (leaf)
        uint_fast32_t depth_r = 63/3 - 1;
        // reverse target depth to fit internal depth iterator
        target_depth = depth_r - target_depth;
        Node* node_p = _root_p;
        for (;depth_r > target_depth; depth_r--) {
            uint64_t shift = 3 * depth_r;
            uint64_t idx = (key >> shift) & 0b111;
            node_p = node_p->children[idx];
        }
        return node_p;
    }
    // insert node at given depth while creating new nodes along path if necessary
    // TODO: cache node path to speed up access
    auto insert(Key key, uint_fast32_t target_depth) -> Node* {

        // check if memory block has enough space left
        if (_mem_blocks.back()._capacity - _mem_blocks.back()._size < 63/3) {
            _mem_blocks.emplace_back();
        }
        auto& mem_block = _mem_blocks.back();

        // depth from 20 (root) to 0 (leaf)
        uint_fast32_t depth_r = 63/3 - 1;
        // reverse target depth to fit internal depth iterator
        target_depth = depth_r - target_depth;
        Node* node_p = _root_p;
        for (;depth_r > target_depth; depth_r--) {
            uint64_t shift = 3 * depth_r;
            uint64_t idx = (key >> shift) & 0b111;
            if (node_p->children[idx] == nullptr) {
                node_p->children[idx] = mem_block.allocate_node();
                // TODO: can continue to create nodes here, since entire subtree is missing
            }
            node_p = node_p->children[idx];
        }
        return node_p;
    }
    // allocate node from most recent memory block, creating a new one if necessary
    auto allocate_node() -> Node* {
        // check if remaining space is sufficient for a single node
        if (_mem_blocks.back()._capacity == _mem_blocks.back()._size) {
            _mem_blocks.emplace_back();
        }
        Node* node_p = _mem_blocks.back().allocate_node();
        // zero out node contents
        std::memset(node_p, 0, sizeof(Node));
        return node_p;
    }

    // merge up to certain depth where collision target is met
    auto static merge(Octree& dst, Octree& src, uint_fast32_t collision_target) -> std::pair<std::vector<Key>, uint_fast32_t> {
        std::vector<Key> collisions;
        phmap::flat_hash_map<Key, Node*> a_parents;
        phmap::flat_hash_map<Key, Node*> a_layer, b_layer;
        uint32_t prev_collision_count = 0;
        uint32_t depth = 0;

        // set up initial roots
        a_layer[0] = dst._root_p;
        b_layer[0] = src._root_p;

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
            for (auto& node: a_parents) {
                Key key = node.first;
                // if a collision occurs, save it
                if (a_layer.contains(key)) {
                    collisions.push_back(key);
                }
                // when node is missing from layer_a, insert it as new child
                else {
                    // index of new child node (0 to 7)
                    Key child_key = key >> (60 - depth*3);
                    child_key &= 0b111;
                    // remove child part from key to get parent key
                    Key parentKey = key ^ (child_key << (60 - depth*3));
                    
                    // use parent key to find the parent node in previous layer
                    Node* parent_p = a_parents[parentKey];
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
                collisions.clear();
                return { collisions, 63/3 };
            }

            // only check for break condition if collision count has changed
            if (depth > 0 && collisions.size() != prev_collision_count) {
                // break if collision target is met or collisions are decreasing
                if (collisions.size() >= collision_target) break;
                if (collisions.size() < prev_collision_count) break;
            }
            prev_collision_count = collisions.size();
        }
        // return the remaining collisions alongside their depth
        return { collisions, depth };
    }
    // fully  merge  via previously found collisions (invalidates src octree)
    void static merge(Octree& dst, Octree& src, Key start_key, uint_fast32_t start_depth) {
        const uint32_t path_length = 63/3 - start_depth;
        if (path_length == 0) return;
        std::vector<uint8_t> path(path_length);
        std::vector<Node*> nodes_a(path_length);
        std::vector<Node*> nodes_b(path_length);
        
        // find the colliding nodes in both trees
        path[0] = 0;
        nodes_a[0] = dst.find(start_key, start_depth);
        nodes_b[0] = src.find(start_key, start_depth);
        uint32_t depth = 0;

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
            
            if (a_child_p != nullptr) {
                // if this node is missing from B, simply skip it
                if (b_child_p == nullptr) continue;
                // if child nodes are leaves, call resolver
                if (depth >= path_length - 2) {
                    // reconstruct morton code from path
                    uint64_t code = start_key;
                    for (uint64_t k = 0; k < path_length - 1; k++) {
                        uint64_t part = path[k] - 1;
                        uint64_t shift = path_length*3 - 6 - k*3;
                        code |= part << shift;
                    }
                    // revert shift on insertion
                    code = { code << 3 };
                    
                    // convert to actual cluster chunk position
                    glm::ivec3 cluster_chunk = MortonCode::decode(code);
                    
                    // get nodes containing the scanpoint pointers
                    Node* leafPoints_a = nodes_a[depth]->children[child_i];
                    Node* leafPoints_b = nodes_b[depth]->children[child_i];
                    
                    // iterate over leaves within clusters
                    uint8_t iLeaf = 0;
                    for (int32_t z = 0; z <= 1; z++) {
                        for (int32_t y = 0; y <= 1; y++) {
                            for (int32_t x = 0; x <= 1; x++, iLeaf++) {
                                glm::ivec3 leaf_chunk = cluster_chunk + glm::ivec3(x, y, z);
                                glm::vec3 leaf_pos = (glm::vec3)leaf_chunk * (float)LEAF_RESOLUTION;

                                // get clusters of closest points to this leaf
                                Node*& a_closest_points = leafPoints_a->children[iLeaf];
                                Node*& b_closest_points = leafPoints_b->children[iLeaf];

                                // check validity of pointer
                                if (b_closest_points == nullptr) continue;
                                else if (a_closest_points == nullptr) {
                                    a_closest_points = b_closest_points;
                                    continue;
                                }
                                
                                // count total children in both leaves
                                std::vector<const glm::vec3*> children;
                                children.reserve(a_closest_points->leaf_candidates.size() * 2);
                                for (uint32_t i = 0; i < a_closest_points->leaf_candidates.size(); i++) {
                                    if (a_closest_points->leaf_candidates[i] != nullptr) {
                                        children.push_back(a_closest_points->leaf_candidates[i]);
                                    }
                                    if (b_closest_points->leaf_candidates[i] != nullptr) {
                                        children.push_back(b_closest_points->leaf_candidates[i]);
                                    }
                                }

                                // simply merge if they fit
                                if (children.size() <= a_closest_points->leaf_candidates.size()) {
                                    // refill with merged children
                                    for (std::size_t i = 0; i < children.size(); i++) {
                                        a_closest_points->leaf_candidates[i] = children[i];
                                    }
                                }
                                // merge only closest points if not
                                else {
                                    // calc distances for each point to this leaf
                                    typedef std::pair<const glm::vec3*, float> PairedDist;
                                    std::vector<PairedDist> distances;
                                    distances.reserve(children.size());
                                    for (std::size_t i = 0; i < children.size(); i++) {
                                        glm::vec3 diff = *children[i] - leaf_pos;
                                        float dist_sqr = glm::dot(diff, diff);
                                        distances.emplace_back(children[i], dist_sqr);
                                    }
                                    // sort via distances
                                    auto sort_fnc = [](PairedDist& a, PairedDist& b){
                                        return a.second < b.second;
                                    };
                                    std::sort(distances.begin(), distances.end(), sort_fnc);

                                    // insert the 8 closest points into a
                                    for (std::size_t i = 0; i < a_closest_points->leaf_candidates.size(); i++) {
                                        a_closest_points->leaf_candidates[i] = distances[i].first;
                                    }
                                }
                            }
                        }
                    }
                }
                else {
                    // walk down path
                    depth++;
                    path[depth] = 0;
                    nodes_a[depth] = a_child_p;
                    nodes_b[depth] = b_child_p;
                }
            }
            else if (b_child_p != nullptr) {
                // add missing child to A
                nodes_a[depth]->children[child_i] = b_child_p;
            }
        }
    }

    Node* _root_p;
    std::vector<MemoryBlock> _mem_blocks;
};

void static octree_insert_point(Octree& octree, const glm::vec3* point_p, std::size_t thread_i) {
    // discretize position to leaf chunk
    glm::vec3 chunk = *point_p / (float)LEAF_RESOLUTION;
    glm::ivec3 chunk_i = (glm::ivec3)glm::floor(chunk);
}
void static octree_construct(std::vector<glm::vec3>& points) {
    auto beg = std::chrono::steady_clock::now();

    // round threads down to nearest power of two
    uint32_t lz = __builtin_clz(std::thread::hardware_concurrency());
    size_t thread_count = 1 << (32 - lz - 1);
    std::vector<std::thread> threads;
    threads.reserve(thread_count);

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
    uint32_t stage_count = std::sqrt(thread_count);
    std::vector<Stage> stages { stage_count };
    std::vector<Octree> octrees { thread_count };

    // initialize thread groups
    for (uint32_t i = 0; i < stage_count; i++) {
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

    // build smaller octrees on several threads
    std::size_t progress = 0;
    for (std::size_t thread_i = 0; thread_i < thread_count; thread_i++) {
        size_t point_count = points.size() / thread_count;
        if (thread_i == thread_count - 1) point_count = 0; // special value for final thread to read the rest
        // build one octree per thread
        threads.emplace_back([&points, &octrees, progress, thread_i, point_count](){
            auto cur_it = points.cbegin() + progress;
            auto end_it = (point_count == 0) ? points.cend() : (cur_it + point_count);
            Octree& octree = octrees[thread_i];
            // Octree::PathCache cache(octree);
            for (; cur_it != end_it; cur_it++) {
                const glm::vec3* point_p = &*cur_it;
                octree_insert_point(octree, point_p, thread_i);
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
}