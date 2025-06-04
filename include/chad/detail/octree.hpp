#pragma once
#include <array>
#include <cstdint>
#include <gtl/phmap.hpp>
#include "chad/detail/morton.hpp"
#include "chad/detail/virtual_array.hpp"

namespace chad::detail {
    struct Octree {
        using NodeAddr = uint32_t;
        using Node = std::array<NodeAddr, 8>;
        struct Leaf { float _signed_distance = 0.0f; uint32_t _weight = 0; };

        Octree() {
            _nodes.push_back({ 0, 0, 0, 0, 0, 0, 0, 0 }); // root node
            _leaves.push_back({}); // dummy leaf node
        }
        void clear() {
            _node_lookup.clear();
            _nodes.clear();
            _leaves.clear();

            _nodes.push_back({ 0, 0, 0, 0, 0, 0, 0, 0 }); // root node
            _leaves.push_back({}); // dummy leaf node
        }
        auto inline insert(MortonCode mc) -> Leaf& {
            // see if node at given level has been created already
            static constexpr uint32_t lookup_depth = 18;
            static constexpr uint64_t lookup_shift = (20 - lookup_depth) * 3;
            static constexpr uint64_t lookup_mask = ((0xffffffffffffffff - 1) >> lookup_shift) << lookup_shift;
            auto [node_it, node_emplaced] = _node_lookup.try_emplace(mc._value & lookup_mask, nullptr);
            
            // start at lookup_depth + 1 when node was found, otherwise start from root
            uint32_t depth = 0;
            Node* node_p = &_nodes[0];
            if (!node_emplaced) depth = lookup_depth + 1;
            if (!node_emplaced) node_p = node_it->second;
            
            while (depth < 20) {
                uint64_t shift_amount = (20 - depth) * 3; // 3 bits per depth, assuming 21 levels
                uint64_t child_index = (mc._value >> shift_amount) & 0b111;
                uint32_t child_addr = (*node_p)[child_index];
                if (child_addr == 0) {
                    // create new child
                    child_addr = _nodes.size();
                    _nodes.push_back({ 0, 0, 0, 0, 0, 0, 0, 0 });

                    // add the new child to current node
                    (*node_p)[child_index] = child_addr;
                    
                    // if node was missing from lookup table, add it now
                    if (node_emplaced && depth == lookup_depth) {
                        node_it->second = &_nodes[child_addr];
                    }
                }
                // walk to child
                node_p = &_nodes[child_addr];
                depth++;
            }

            uint64_t leaf_index = mc._value & 0b111;
            uint32_t leaf_addr = (*node_p)[leaf_index];
            if (leaf_addr == 0) {
                // create new leaf
                leaf_addr = _leaves.size();
                _leaves.push_back({ 0.0f, 0 });

                // add the new leaf to current node
                (*node_p)[leaf_index] = leaf_addr;
            }

            return _leaves[leaf_addr];
        }

        auto static get_root() -> uint32_t {
            return 1;
        }
        auto inline get_node(uint32_t node_addr) const -> const Node& {
            return _nodes[node_addr];
        }
        auto inline get_leaf(uint32_t leaf_addr) const -> const Leaf& {
            return _leaves[leaf_addr];
        }
        auto inline get_child_addr(uint32_t parent_addr, uint8_t child_i) const -> uint32_t {
            return _nodes[parent_addr][child_i];
        }

        VirtualArray<Node> _nodes;
        VirtualArray<Leaf> _leaves;
        gtl::flat_hash_map<MortonCode, Node*> _node_lookup; // depth 18
    };
}