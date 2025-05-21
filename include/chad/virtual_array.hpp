#pragma once
#include <cstdio>
#include <cstdint>

namespace chad {
    template<typename T> struct VirtualArray {
        VirtualArray(size_t virtual_capacity = 0xffffffff);
        ~VirtualArray();
        auto operator[](const uint32_t index) noexcept -> T& {
            return _virt_mem_p[index];
        }
        auto operator[](const uint32_t index) const noexcept -> const T& {
            return _virt_mem_p[index];
        }
        void inline push_back(const T value) noexcept {
            _virt_mem_p[_size++] = value;
        }
        auto inline size() noexcept -> size_t {
            return _size;
        }
        auto inline capacity() noexcept -> size_t {
            return _capacity;
        }
    private:
        T* _virt_mem_p;
        size_t _size;
        size_t _capacity;
    };
}