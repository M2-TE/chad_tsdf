#pragma once
#include <cstdio>
#include <cstring>
#include <algorithm>

namespace chad::detail {
    auto allocate_virtual(size_t virtual_capacity) -> void*;
    void deallocate_virtual(void* virt_mem_p, size_t virtual_capacity);
}

namespace chad::detail {
    template<typename T> struct VirtualArray {
        using const_iterator = const T*;
        using iterator = T*;

        VirtualArray(size_t virtual_capacity = 0xffffffff): 
            _virt_mem_p(reinterpret_cast<T*>(allocate_virtual(virtual_capacity))),   
            _size(0), _capacity(virtual_capacity) {
        }
        ~VirtualArray() {
            deallocate_virtual(_virt_mem_p, _capacity);
        }

        auto operator[](size_t index) -> T& {
            return _virt_mem_p[index];
        }
        auto operator[](size_t index) const -> const T& {
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

        template<class InputIt>
        void inline insert_back(InputIt first, InputIt last) {
            auto cur = first;
            while(cur != last) {
                _virt_mem_p = *cur;
                _size++;
            }
        }
        void inline insert_back(std::initializer_list<T> ilist) {
            std::memcpy(_virt_mem_p + _size, ilist.begin(), ilist.size());
            _size += ilist.size();
        }

        auto inline front() const -> const T& {
            return _virt_mem_p[0];
        }
        auto inline front() -> T& {
            return _virt_mem_p[0];
        }
        auto inline back() const -> const T& {
            return _virt_mem_p[_size - 1];
        }
        auto inline back() -> T& {
            return _virt_mem_p[_size - 1];
        }
        auto inline data() const noexcept -> const T* {
            return _virt_mem_p;
        }
        auto inline data() noexcept -> T* {
            return _virt_mem_p;
        }

        auto inline begin() const noexcept -> const_iterator {
            return _virt_mem_p;
        }
        auto inline begin() noexcept -> iterator {
            return _virt_mem_p;
        }
        auto inline end() const noexcept -> const_iterator {
            return _virt_mem_p + _size;
        }
        auto inline end() noexcept -> iterator {
            return _virt_mem_p + _size;
        }

        auto inline size() const noexcept -> size_t {
            return _size;
        }
        auto inline capacity() const noexcept -> size_t {
            return _capacity;
        }
        void inline resize(size_t new_size) noexcept {
            // no need to worry about de-/reallocation
            _size = new_size;
        }
        
    private:
        T* _virt_mem_p;
        size_t _size;
        size_t _capacity;
    };
}