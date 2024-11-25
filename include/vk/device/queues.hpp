#pragma once
#include <vulkan/vulkan.hpp>

namespace dv{
struct Queues {
    void init(vk::Device device, std::span<uint32_t> queue_mappings) {
        _universal_i = queue_mappings[0];
        _compute_i = queue_mappings[1];
        _transfer_i = queue_mappings[2];
        _universal = device.getQueue(_universal_i, 0);
        _compute = device.getQueue(_compute_i, 0);
        _transfer = device.getQueue(_transfer_i, 0);

        vk::CommandPoolCreateInfo info = {
            .flags = vk::CommandPoolCreateFlagBits::eTransient,
            .queueFamilyIndex = _universal_i,
        };
        _universal_pool = device.createCommandPool(info);

    }
    void destroy(vk::Device device) {
        device.destroyCommandPool(_universal_pool);
    }

    auto oneshot_begin(vk::Device device) -> vk::CommandBuffer {
        vk::CommandBufferAllocateInfo info = {
            .commandPool = _universal_pool,
            .level = vk::CommandBufferLevel::ePrimary,
            .commandBufferCount = 1,
        };
        vk::CommandBuffer cmd = device.allocateCommandBuffers(info)[0];
        cmd.begin({ .flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit });
        return cmd;
    }
    void oneshot_end(vk::Device device, vk::CommandBuffer cmd, const vk::ArrayProxy<vk::Semaphore>& sign_semaphores = {}) {
        cmd.end();
        vk::SubmitInfo info = {
            .commandBufferCount = 1,
            .pCommandBuffers = &cmd,
            .signalSemaphoreCount = (uint32_t)sign_semaphores.size(),
            .pSignalSemaphores = sign_semaphores.data(),
        };
        _universal.submit(info);
        _universal.waitIdle();
        device.freeCommandBuffers(_universal_pool, cmd);
    }
    // queue handle
    vk::Queue _universal, _compute, _transfer;
    // queue family index
    uint32_t _universal_i, _compute_i, _transfer_i;
private:
    // oneshot command pool
    vk::CommandPool _universal_pool;
};
} // namespace dv