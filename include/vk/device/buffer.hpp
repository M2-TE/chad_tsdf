#pragma once
#include <fmt/base.h>
#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.hpp>

namespace dv {
struct Buffer {
	struct BufferCreateInfo {
		vma::Allocator vmalloc;
		vk::DeviceSize size;
		vk::BufferUsageFlags usage = vk::BufferUsageFlagBits::eUniformBuffer;
		vk::SharingMode sharing_mode = vk::SharingMode::eExclusive;
		const vk::ArrayProxy<uint32_t>& queue_families;
		bool host_accessible = false;
	};
	void init(const BufferCreateInfo& info) {
		vk::BufferCreateInfo info_buffer {
			.size = info.size,
			.usage = info.usage,
			.sharingMode = info.sharing_mode,
			.queueFamilyIndexCount = info.queue_families.size(),
			.pQueueFamilyIndices = info.queue_families.data(),
		};
		vma::AllocationCreateInfo info_allocation {
			.usage = vma::MemoryUsage::eAutoPreferDevice,
			.requiredFlags = vk::MemoryPropertyFlagBits::eDeviceLocal,
		};
		// add flags to allow host access if requested (ReBAR if available)
		if (info.host_accessible) {
			info_allocation.flags = 
				vma::AllocationCreateFlagBits::eHostAccessSequentialWrite;
			info_allocation.preferredFlags = 
				vk::MemoryPropertyFlagBits::eHostVisible |
				vk::MemoryPropertyFlagBits::eHostCoherent;
		}
		// create buffer
		std::tie(_data, _allocation) = info.vmalloc.createBuffer(info_buffer, info_allocation);
		_size = info.size;

		// check for host coherency and visibility
		if (info.host_accessible) {
			vk::MemoryPropertyFlags props = info.vmalloc.getAllocationMemoryProperties(_allocation);
			if (props & vk::MemoryPropertyFlagBits::eHostVisible) _require_staging = false;
			else _require_staging = true;
			if (props & vk::MemoryPropertyFlagBits::eHostCoherent) _require_flushing = false;
			else _require_flushing = true;
		}
		else {
			_require_staging = false;
			_require_flushing = false;
		}
	}
	void destroy(vma::Allocator vmalloc) {
		vmalloc.destroyBuffer(_data, _allocation);
	}

	void write(vma::Allocator vmalloc, void* data_p, size_t byte_count) {
		if (_require_staging) fmt::println("ReBAR required, staging buffer not yet implemented");
		void* map_p = vmalloc.mapMemory(_allocation);
		std::memcpy(map_p, data_p, byte_count);
		vmalloc.unmapMemory(_allocation);
		if (_require_flushing) vmalloc.flushAllocation(_allocation, 0, byte_count);
	}
	void read(vma::Allocator vmalloc, void* data_p, size_t byte_count) {
		if (_require_staging) fmt::println("ReBAR required, staging buffer not yet implemented");
		void* map_p = vmalloc.mapMemory(_allocation);
		std::memcpy(data_p, map_p, byte_count);
		vmalloc.unmapMemory(_allocation);
	}
	template<typename T> void write(vma::Allocator vmalloc, T& data) {
		write(vmalloc, &data, sizeof(T));
	}
	template<typename T> void read(vma::Allocator vmalloc, T& data) {
		read(vmalloc, &data, sizeof(T));
	}

	vk::Buffer _data;
	vma::Allocation _allocation;
	vk::DeviceSize _size;
	bool _require_staging;
	bool _require_flushing;
};
} // namespace dv