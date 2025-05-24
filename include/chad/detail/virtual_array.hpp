#pragma once
#include <algorithm>
#include <cstdio>
#include <cstdint>

namespace chad::detail {
    auto allocate_virtual(size_t virtual_capacity) -> void*;
    void deallocate_virtual(void* virt_mem_p, size_t virtual_capacity);
}

namespace chad::detail {
    template<typename T> struct VirtualArray {
        VirtualArray(size_t virtual_capacity = 0xffffffff): 
            _virt_mem_p(reinterpret_cast<T*>(allocate_virtual(virtual_capacity))),   
            _size(0), _capacity(virtual_capacity) {
        }
        ~VirtualArray() {
            deallocate_virtual(_virt_mem_p, _capacity);
        }
        auto operator[](const uint32_t index) -> T& {
            return _virt_mem_p[index];
        }
        auto operator[](const uint32_t index) const -> const T& {
            return _virt_mem_p[index];
        }
        auto inline push_back(const T& value) -> T& {
            _virt_mem_p[_size] = value;
            return _virt_mem_p[_size++];
        }
        auto inline push_back(T&& value) -> T& {
            _virt_mem_p[_size] = std::move(value);
            return _virt_mem_p[_size++];
        }
        auto inline front() const -> const T& {
            return _virt_mem_p[0];
        }
        auto inline back() const -> const T& {
            return _virt_mem_p[_size - 1];
        }

        auto inline size() const noexcept -> size_t {
            return _size;
        }
        auto inline capacity() const noexcept -> size_t {
            return _capacity;
        }
        
    private:
        T* _virt_mem_p;
        size_t _size;
        size_t _capacity;
    };
}