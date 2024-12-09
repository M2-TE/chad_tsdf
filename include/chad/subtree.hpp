#pragma once
#include <cstdint>

// contain metadata for each scan
struct Subtree {
    Subtree(uint32_t root_addr): _root_addr(root_addr) {}
    uint32_t _root_addr;
};