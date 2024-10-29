#include <fmt/base.h>
#include <vulkan/vulkan.hpp>
#include <vk_mem_alloc.hpp>
#include "vk/vk.hpp"
#include "vk/queues.hpp"
#include "vk/device_selector.hpp"

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
        DeviceSelector device_selector {
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
                // vma::AllocatorCreateFlagBits::eKhrBindMemory2 |
                // vma::AllocatorCreateFlagBits::eKhrMaintenance4 |
                // vma::AllocatorCreateFlagBits::eKhrMaintenance5 |
                vma::AllocatorCreateFlagBits::eExtMemoryBudget |
                vma::AllocatorCreateFlagBits::eExtMemoryPriority |
                // vma::AllocatorCreateFlagBits::eBufferDeviceAddress |
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
    }
    void destroy() {
        _vmalloc.destroy();
        _queues.destroy(_device);
        _device.destroy();
        _instance.destroy();
    }

    vk::Instance _instance;
    vk::PhysicalDevice _phys_device;
    vk::Device _device;
    vma::Allocator _vmalloc;
    Queues _queues;
};

void init_vk() {
    VkState vk_state;
    vk_state.init();
    vk_state.destroy();
}