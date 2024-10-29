#pragma once
#include <set>
#include <vulkan/vulkan.hpp>
#include <fmt/base.h>

namespace dv {
struct Selector {
    // Selects a physical device based on requirements and preferences.
    auto select_physical_device(vk::Instance instance, vk::SurfaceKHR surface = nullptr) -> vk::PhysicalDevice;
    // Creates logical device from previously selected physical device. Returns device and queue family indices for requested queues.
    auto create_logical_device(vk::PhysicalDevice physical_device, void* additional_features_p) -> std::pair<vk::Device, std::vector<uint32_t>>;

private:
    bool check_api_ver(vk::PhysicalDeviceProperties& props);
    bool check_extensions(std::set<std::string> required_extensions, vk::PhysicalDevice physical_device);
    bool check_core_features(vk::PhysicalDevice phys_device);
    bool check_presentation(vk::PhysicalDevice physical_device, vk::SurfaceKHR surface);
    auto create_queue_infos(vk::PhysicalDevice physical_device) -> std::pair<std::vector<vk::DeviceQueueCreateInfo>, std::vector<uint32_t>>;

public:
    uint32_t _required_major = 1;
    uint32_t _required_minor = 0;
    vk::PhysicalDeviceType _preferred_device_type = vk::PhysicalDeviceType::eDiscreteGpu;
    std::vector<const char*> _required_extensions = {};
    std::vector<const char*> _optional_extensions = {};
    vk::PhysicalDeviceFeatures _required_features = {};
    vk::PhysicalDeviceVulkan11Features _required_vk11_features = {};
    vk::PhysicalDeviceVulkan12Features _required_vk12_features = {};
    vk::PhysicalDeviceVulkan13Features _required_vk13_features = {};
    std::vector<vk::QueueFlags> _required_queues = {};
private:
    static constexpr float queue_priority = 1.0f;
};
} // namespace dv