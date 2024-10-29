#pragma once
#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.hpp>

template<typename T>
struct DeviceBuffer {
	void init(vma::Allocator vmalloc, const vk::BufferCreateInfo& info_buffer, const vma::AllocationCreateInfo& info_allocation) {
		std::tie(_data, _allocation) = vmalloc.createBuffer(info_buffer, info_allocation);

		// check for host coherency and visibility
		vk::MemoryPropertyFlags props = vmalloc.getAllocationMemoryProperties(_allocation);
		if (props & vk::MemoryPropertyFlagBits::eHostVisible) _require_staging = false;
		else _require_staging = true;
		if (props & vk::MemoryPropertyFlagBits::eHostCoherent) _require_flushing = false;
		else _require_flushing = true;
	}
	void destroy(vma::Allocator vmalloc) {
		vmalloc.destroyBuffer(_data, _allocation);
	}
	
	auto static constexpr size() -> vk::DeviceSize {
		return sizeof(T);
	}
	auto map(vma::Allocator vmalloc) -> void* {
		if (_require_staging) fmt::println("ReBAR required, staging buffer not yet implemented");
		return vmalloc.mapMemory(_allocation);
	}
	void write(vma::Allocator vmalloc, T& data) {
		if (_require_staging) fmt::println("ReBAR required, staging buffer not yet implemented");
		void* map_p = vmalloc.mapMemory(_allocation);
		std::memcpy(map_p, &data, sizeof(T));
		vmalloc.unmapMemory(_allocation);
		if (_require_flushing) vmalloc.flushAllocation(_allocation, 0, sizeof(T));
	}
	void write(vma::Allocator vmalloc, T& data, void* map_p) {
		std::memcpy(map_p, &data, sizeof(T));
		if (_require_flushing) vmalloc.flushAllocation(_allocation, 0, sizeof(T));
	}
	void write(vma::Allocator vmalloc, size_t dst_offset, size_t size, void* data) {
		if (_require_staging) fmt::println("ReBAR required, staging buffer not yet implemented");
		void* map_p = vmalloc.mapMemory(_allocation);
		std::memcpy((char*)map_p + dst_offset, data, size);
		vmalloc.unmapMemory(_allocation);
		if (_require_flushing) vmalloc.flushAllocation(_allocation, dst_offset, size);
	}
	void write(vma::Allocator vmalloc, size_t dst_offset, size_t size, void* data, void* map_p) {
		std::memcpy((char*)map_p + dst_offset, data, size);
		vmalloc.unmapMemory(_allocation);
		if (_require_flushing) vmalloc.flushAllocation(_allocation, dst_offset, size);
	}

	vk::Buffer _data;
	vma::Allocation _allocation;
	bool _require_staging;
	bool _require_flushing;
};