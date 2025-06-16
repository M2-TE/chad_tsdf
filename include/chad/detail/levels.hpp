#pragma once
#include <cstdint>
#include <gtl/phmap.hpp>
#include "chad/detail/cluster.hpp"
#include "chad/detail/virtual_array.hpp"

namespace chad::detail {
    class NodeLevel {
        struct FncHash {
            FncHash(const VirtualArray<uint32_t>& node_data): _node_data(node_data) {
            }
            auto inline operator()(const uint32_t addr) const noexcept -> uint64_t {
                // count children
                uint8_t child_count = std::popcount(uint8_t(_node_data[addr]));
                // hash entire node
                uint64_t hash = 0;
                for (uint8_t i = 1; i <= child_count; i++) {
                    hash = gtl::HashState::combine(hash, _node_data[addr + i]);
                }
                return hash;
            }
            const VirtualArray<uint32_t>& _node_data;
        };
        struct FncEq {
            FncEq(const VirtualArray<uint32_t>& node_data): _node_data(node_data) {
            }
            bool inline operator()(const uint32_t addr_a, const uint32_t addr_b) const noexcept {
                // compare child masks
                uint8_t child_mask_a = uint8_t(_node_data[addr_a]);
                uint8_t child_mask_b = uint8_t(_node_data[addr_b]);
                if (child_mask_a != child_mask_b) return false;

                // count children
                uint8_t child_count_a = std::popcount(child_mask_a);
                uint8_t child_count_b = std::popcount(child_mask_b);
                if (child_count_a != child_count_b) return false;

                // compare entire nodes
                int cmp = std::memcmp(
                    &_node_data[addr_a + 1],
                    &_node_data[addr_b + 1],
                    child_count_a * sizeof(uint32_t));
                return cmp == 0;
            }
            const VirtualArray<uint32_t>& _node_data;
        };

    public:
        NodeLevel():
                _uniques_n(0), _dupes_n(0), _occupied_n(0), _raw_data(),
                _addr_set(0, FncHash(_raw_data), FncEq(_raw_data)) {
            // reserve first index
            _raw_data.push_back(0);
            _raw_data.push_back(0);
            _occupied_n = _raw_data.size();
        }
        // add a single node with the given children and return its address
        auto add(const std::array<uint32_t, 8>& children) -> uint32_t {

            // check if theres enough space for a placeholder node
            if (_occupied_n + 9 < _raw_data.size()) {
                _raw_data.resize(_raw_data.size() + 9);
            }

            // write placerholder node into raw data vector
            uint32_t* node_p = _raw_data.data() + _occupied_n;
            node_p[0] = 0;// first element is child mask

            // gather only valid children
            uint8_t children_n = 0;
            for (uint8_t i = 0; i < 8; i++) {
                if (children[i] == 0) continue;
                node_p[children_n + 1] = children[i];
                node_p[0] |= 1 << i; // set child mask bit
                children_n++;
            }
            
            // emplace placeholder node if it's a new one
            uint32_t new_addr = _occupied_n;
            auto [old_addr_it, new_addr_b] = _addr_set.emplace(new_addr);
            if (new_addr_b) {
                _uniques_n++;
                _occupied_n += children_n + 1;
                return new_addr;
            }
            else {
                _dupes_n++;
                return *old_addr_it;
            }
        }

        uint32_t _uniques_n, _dupes_n;
        uint32_t _occupied_n; // number of occupied 32-bit values in _raw_data vector
        VirtualArray<uint32_t> _raw_data; // raw unaligned node data
        gtl::parallel_flat_hash_set<uint32_t, FncHash, FncEq> _addr_set; // set of addresses
    };
    class LeafClusterLevel {
        struct FncHash {
            FncHash(const VirtualArray<LeafCluster>& lc_data): _lc_data(lc_data) {
            }
            auto inline operator()(const uint32_t addr) const noexcept -> uint64_t {
                // use the 64-bit value of the leaf cluster as the hash
                return _lc_data[addr]._value;
            }
            const VirtualArray<LeafCluster>& _lc_data;
        };
        struct FncEq {
            FncEq(const VirtualArray<LeafCluster>& lc_data): _lc_data(lc_data) {
            }
            bool inline operator()(const uint32_t addr_a, const uint32_t addr_b) const noexcept {
                // compare leaf clusters values directly
                return _lc_data[addr_a]._value == _lc_data[addr_b]._value;
            }
            const VirtualArray<LeafCluster>& _lc_data;
        };

    public:
        LeafClusterLevel():
                _uniques_n(0), _dupes_n(0), _raw_data(),
                _addr_set(0, FncHash(_raw_data), FncEq(_raw_data)) {
            // reserve first index
            _raw_data.push_back(LeafCluster{});
        }
        // add a leaf cluster with the given contents and return its address
        auto add(LeafCluster lc) -> uint32_t {
            // append a placeholder node
            uint32_t new_addr = _uniques_n + 1;
            if (_raw_data.size() <= new_addr) _raw_data.push_back(lc);
            else                              _raw_data.back() = lc;

            // emplace placeholder node if it's a new one
            auto [old_addr_it, new_addr_b] = _addr_set.emplace(new_addr);
            if (new_addr_b) {
                _uniques_n++;
                return new_addr;
            }
            else {
                _dupes_n++;
                return *old_addr_it;
            }
        }

        uint32_t _uniques_n, _dupes_n;
        VirtualArray<LeafCluster> _raw_data; // leaf cluster data
        gtl::parallel_flat_hash_set<uint32_t, FncHash, FncEq> _addr_set; // set of addresses
    };

    struct NodeLevels {
        auto get_child_addr(uint32_t depth, uint32_t parent_addr, uint8_t child_i) const -> uint32_t {
            uint32_t child_mask = _nodes[depth]._raw_data[parent_addr];
            uint32_t child_bit = 1 << child_i;

            // check if the child exists
            if (child_mask & child_bit) {
                // count the number of children that are stored before this one
                uint8_t masked = uint8_t(child_mask & (child_bit - 1));
                uint8_t child_count = std::popcount(masked);
                // child count will correspond to the requested child's index + 1 (accounting for child mask index)
                uint32_t child_addr = _nodes[depth]._raw_data[parent_addr + uint32_t(child_count + 1)];
                return child_addr;
            }
            else return 0;
        }

        auto try_get_leaf_cluster(uint32_t parent_addr, uint8_t child_i) const -> std::pair<LeafCluster, bool> {
            uint32_t child_mask = _nodes[MAX_DEPTH - 1]._raw_data[parent_addr];
            uint32_t child_bit = uint32_t(1) << child_i;

            // check if the leaf cluster exists
            if (child_mask & child_bit) {
                // count the number of children that are stored before this one
                uint8_t masked = uint8_t(child_mask & (child_bit - 1));
                uint8_t child_count = std::popcount(masked);
                // child count will correspond to the requested child's index
                uint32_t child_addr = _nodes.back()._raw_data[parent_addr + uint32_t(child_count + 1)];
                return { _leaf_clusters._raw_data[child_addr], true };
            }
            else return { {}, false };
        }

        // 21 levels total
        static constexpr uint64_t MAX_DEPTH = 20;
        // 20 levels of standard nodes
        std::array<NodeLevel, MAX_DEPTH> _nodes;
        // 1 level of leaf clusters
        LeafClusterLevel _leaf_clusters;
    };
}