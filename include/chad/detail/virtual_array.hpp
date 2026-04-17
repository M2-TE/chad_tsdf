#pragma once

namespace chad::detail {
    auto allocate_virtual(size_t bytes) -> void*;
    void deallocate_virtual(void* virtual_p, size_t bytes);
    void prefault_virtual(void* virtual_p, size_t bytes);
}

namespace chad::detail {
    static constexpr std::size_t KiB = 1024;
    static constexpr std::size_t MiB = 1024 * KiB;
    static constexpr std::size_t GiB = 1024 * MiB;

    template<typename T> struct VirtualArray {
        using const_iterator = const T*;
        using iterator = T*;

        VirtualArray(std::size_t virtual_capacity = 0xffffffff):
            _virtual_p(static_cast<T*>(allocate_virtual(virtual_capacity))),
            _size(0),
            _capacity(virtual_capacity) {
        }
        ~VirtualArray() {
            deallocate_virtual(_virtual_p, _capacity);
        }

        auto operator[](std::size_t index) const -> const T& {
            return _virtual_p[index];
        }
        auto operator[](std::size_t index) -> T& {
            return _virtual_p[index];
        }
        auto inline push_back(const T& value) -> T& {
            _virtual_p[_size] = value;
            return _virtual_p[_size++];
        }
        auto inline push_back(T&& value) -> T& {
            _virtual_p[_size] = std::move(value);
            return _virtual_p[_size++];
        }

        template<class InputIt>
        void inline insert_back(InputIt first, InputIt last) {
            InputIt cur = first;
            while(cur != last) {
                _virtual_p = *cur;
                std::next(cur);
            }
            _size += std::distance(first, last);
        }
        void inline insert_back(std::initializer_list<T> ilist) {
            std::memcpy(_virtual_p + _size, ilist.begin(), ilist.size());
            _size += ilist.size();
        }

        auto inline front() const -> const T& {
            return _virtual_p[0];
        }
        auto inline front() -> T& {
            return _virtual_p[0];
        }
        auto inline back() const -> const T& {
            return _virtual_p[_size - 1];
        }
        auto inline back() -> T& {
            return _virtual_p[_size - 1];
        }
        auto inline data() const -> const T* {
            return _virtual_p;
        }
        auto inline data() -> T* {
            return _virtual_p;
        }

        auto inline cbegin() const -> const_iterator {
            return _virtual_p;
        }
        auto inline begin() -> iterator {
            return _virtual_p;
        }
        auto inline cend() const -> const_iterator {
            return _virtual_p + _size;
        }
        auto inline end() -> iterator {
            return _virtual_p + _size;
        }

        auto inline size() const -> std::size_t {
            return _size;
        }
        auto inline capacity() const -> std::size_t {
            return _capacity;
        }
        // TODO: actually free the pages if smaller than before?
        // TODO: prefault pages if larger than capacity?
        void inline resize(std::size_t new_size) {
            _size = new_size;
        }
        // TODO
        void inline reserve(std::size_t new_capacity) {

        }
        // TODO: actually free the pages?
        void inline clear() {
            // MADV_FREE stuff
            _size = 0;
        }
        
    private:
        T* const _virtual_p;
        std::size_t _size;
        const std::size_t _capacity;
    };
}