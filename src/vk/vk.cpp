#include <fmt/base.h>
#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.hpp>
#include "vk/vk.hpp"
#include "vk/device/queues.hpp"
#include "vk/device/buffer.hpp"
#include "vk/device/selector.hpp"
#include "vk/device/pipeline.hpp"

struct VkState {
    void init() {
        // dynamic dispatcher init 1/3
        VULKAN_HPP_DEFAULT_DISPATCHER.init();

        // instance ctor
        std::vector<const char*> layers;
        #ifdef VULKAN_VALIDATION_LAYERS
            std::string validation_layer = "VK_LAYER_KHRONOS_validation";
            // only request if validation layers are available
            auto layer_props = vk::enumerateInstanceLayerProperties();
            bool available = false;
            for (auto& layer: layer_props) {
                auto res = std::strcmp(layer.layerName, validation_layer.data());
                if (res) available = true;
            }
            if (available) layers.push_back(validation_layer.data());
            else fmt::println("Validation layers requested but not present");
        #endif
        vk::ApplicationInfo info_app {
            .applicationVersion = VK_MAKE_API_VERSION(0, 1, 0, 0),
            .apiVersion = vk::ApiVersion11
        };
        vk::InstanceCreateInfo info_instance {
            .pApplicationInfo = &info_app,
            .enabledLayerCount = (uint32_t)layers.size(),
            .ppEnabledLayerNames = layers.data()
        };
        _instance = vk::createInstance(info_instance);

        // dynamic dispatcher init 2/3
        VULKAN_HPP_DEFAULT_DISPATCHER.init(_instance);

        // select physical device
        dv::Selector device_selector {
            ._required_major = 1,
            ._required_minor = 1,
            ._preferred_device_type = vk::PhysicalDeviceType::eDiscreteGpu,
            ._required_extensions {
            },
            ._optional_extensions {
                vk::EXTMemoryBudgetExtensionName,
                vk::EXTMemoryPriorityExtensionName,
                vk::EXTPageableDeviceLocalMemoryExtensionName,
            },
            ._required_features {
            },
            ._required_vk11_features {
            },
            ._required_vk12_features {
                // .bufferDeviceAddress = true,
            },
            ._required_vk13_features {
                // .synchronization2 = true,
                // .dynamicRendering = true,
                // .maintenance4 = true,
            },
            ._required_queues {
                vk::QueueFlagBits::eCompute | vk::QueueFlagBits::eTransfer,
                vk::QueueFlagBits::eCompute,
                vk::QueueFlagBits::eTransfer,
            }
        };
        _phys_device = device_selector.select_physical_device(_instance);

        // create logical device
        vk::PhysicalDeviceMemoryPriorityFeaturesEXT memory_priority {
            .memoryPriority = vk::True,
        };
        vk::PhysicalDevicePageableDeviceLocalMemoryFeaturesEXT pageable_memory {
            .pNext = &memory_priority,
            .pageableDeviceLocalMemory = vk::True,
        };
        std::vector<uint32_t> queue_mappings;
        std::tie(_device, queue_mappings) = device_selector.create_logical_device(_phys_device, &pageable_memory);

        // dynamic dispatcher init 3/3
        VULKAN_HPP_DEFAULT_DISPATCHER.init(_device);

        // create vulkan memory allocator
        vma::VulkanFunctions vk_funcs {
            .vkGetInstanceProcAddr = VULKAN_HPP_DEFAULT_DISPATCHER.vkGetInstanceProcAddr,
            .vkGetDeviceProcAddr = VULKAN_HPP_DEFAULT_DISPATCHER.vkGetDeviceProcAddr,
        };
        vma::AllocatorCreateInfo info_vmalloc {
            .flags =
                vma::AllocatorCreateFlagBits::eExtMemoryBudget |
                vma::AllocatorCreateFlagBits::eExtMemoryPriority |
                vma::AllocatorCreateFlagBits::eKhrDedicatedAllocation,
            .physicalDevice = _phys_device,
            .device = _device,
            .pVulkanFunctions = &vk_funcs,
            .instance = _instance,
            .vulkanApiVersion = vk::ApiVersion13,
        };
        _vmalloc = vma::createAllocator(info_vmalloc);

        // set up device queues
        _queues.init(_device, queue_mappings);
        init_sync();
        init_buffers();
        init_pipe();

    }
    void destroy() {
        // vma related
        _buffer.destroy(_vmalloc);
        _vmalloc.destroy();
        // device related
        _pipe.destroy(_device);
        _queues.destroy(_device);
        _device.destroyCommandPool(_cmd_pool);
        _device.destroyFence(_ready_to_record);
        _device.destroy();
        //
        _instance.destroy();
    }

    void dostuff() {
        // wait until last buffer was executed
        while (vk::Result::eTimeout == _device.waitForFences(_ready_to_record, vk::True, UINT64_MAX));
        _device.resetFences(_ready_to_record);

        // reset and record command buffer
        _device.resetCommandPool(_cmd_pool, {});
        vk::CommandBuffer cmd = _cmd_buffer;
        cmd.begin({ .flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit });
        _pipe.execute(cmd, 1, 1, 1);
        cmd.end();

        // submit command buffer
        vk::PipelineStageFlags wait_stage = vk::PipelineStageFlagBits::eTopOfPipe;
        vk::SubmitInfo info_submit {
            .pWaitDstStageMask = &wait_stage,
            .commandBufferCount = 1,
            .pCommandBuffers = &cmd,
        };
        _queues._universal.submit(info_submit, _ready_to_record);

        // DEBUG
        while (vk::Result::eTimeout == _device.waitForFences(_ready_to_record, vk::True, UINT64_MAX));
        std::array<uint32_t, 64> data;
        _buffer.read(_vmalloc, data);
        for (auto& d: data) fmt::print("{} ", d);
        fmt::println("");
    }

private:
    void init_sync() {
        // allocate single command pool and buffer pair
        _cmd_pool = _device.createCommandPool({ .queueFamilyIndex = _queues._universal_i });
        vk::CommandBufferAllocateInfo bufferInfo {
            .commandPool = _cmd_pool,
            .level = vk::CommandBufferLevel::ePrimary,
            .commandBufferCount = 1,
        };
        _cmd_buffer = _device.allocateCommandBuffers(bufferInfo).front();
        // create simple fence for cpu-gpu sync
        _ready_to_record = _device.createFence({ .flags = vk::FenceCreateFlagBits::eSignaled });
    }
    void init_buffers() {
        _buffer.init({
            .vmalloc = _vmalloc,
            .size = sizeof(uint32_t) * 64,
            .usage = vk::BufferUsageFlagBits::eStorageBuffer,
            .sharing_mode = vk::SharingMode::eExclusive,
            .queue_families = _queues._compute_i,
            .host_accessible = true,
        });

        std::array<uint32_t, 64> data;
        for (uint32_t i = 0; i < 64; i++) data[i] = i;
        _buffer.write(_vmalloc, data);
    }
    void init_pipe() {
        _pipe.init({
            .device = _device,
            .cs_path = "hash.comp",
        });
        _pipe.write_descriptor(_device, 0, 0, _buffer, vk::DescriptorType::eStorageBuffer);
    }

    vk::Instance _instance;
    vk::PhysicalDevice _phys_device;
    vk::Device _device;
    vma::Allocator _vmalloc;
    dv::Queues _queues;
    dv::Compute _pipe;
    // exec sync
    vk::Fence _ready_to_record;
    vk::CommandPool _cmd_pool;
    vk::CommandBuffer _cmd_buffer;
    // other
    dv::Buffer _buffer;
};

void init_vk() {
    VkState vk_state;
    vk_state.init();
    vk_state.dostuff();
    vk_state.destroy();
}