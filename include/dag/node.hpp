#pragma once
#include <array>
#include <vector>
#include <cstdint>

// pure helper struct for managing DAG nodes, will never be explicitly created or destroyed
struct Node {
    Node() = delete;
    ~Node() = delete;
    Node(const Node&) = delete;
    Node(Node&&) = delete;
    Node& operator=(const Node&) = delete;
    Node& operator=(Node&&) = delete;

    // reinterpret a node address as a valid Node object
    auto static inline from_addr(std::vector<uint32_t>& data, uint_fast32_t addr) -> Node* {
        return reinterpret_cast<Node*>(&data[addr]);
    }
    // check header mask for a specific child
    bool inline contains_child(uint32_t child_i) const {
        uint32_t child_bit = 1 << child_i;
        return _header & child_bit;
    }
    // retrieve child addr from sparse children array
    auto inline get_child_addr(uint32_t child_i) const -> uint32_t {
        // count how many children come before this one
        uint32_t child_bit = 1 << child_i;
        uint32_t masked = _header & (child_bit - 1);
        uint32_t child_count;
#       ifdef CHAD_POPCOUNT_INSTRUCTION
            child_count = CHAD_POPCOUNT_INSTRUCTION(masked);
#       else
            // popcount lookup table: https://stackoverflow.com/a/51388543
            constexpr uint8_t popcount_table[] = {
                0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
            };
            child_count = popcount_table[masked];
#       endif
        return _children[child_count];
    }

    uint32_t _header;
    std::array<uint32_t, 8> _children; // at least 1 valid child, at most 8
};