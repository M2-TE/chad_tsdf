#pragma once
#include <set>
#include <bit>
#include <vulkan/vulkan.hpp>
#include <fmt/base.h>

struct DeviceSelector {
    auto select_physical_device(vk::Instance instance, vk::SurfaceKHR surface = nullptr) -> vk::PhysicalDevice {
        // enumerate devices
        auto phys_devices = instance.enumeratePhysicalDevices();
        if (phys_devices.size() == 0) fmt::println("No device with vulkan support found");

        // create a std::set from the required extensions
        std::set<std::string> required_extensions {
            _required_extensions.cbegin(),
            _required_extensions.cend()
        };
        
        // check for matching devices (bool = is_preferred, vk::DeviceSize = memory_size)
        std::vector<std::pair<vk::PhysicalDevice, vk::DeviceSize>> matching_devices;
        fmt::println("Available devices:");
        for (vk::PhysicalDevice device: phys_devices) {
            auto props = device.getProperties();
            bool passed = true;
            passed &= check_api_ver(props);
            passed &= check_extensions(required_extensions, device);
            passed &= check_core_features(device);
            passed &= check_presentation(device, surface);

            // add device candidate if it passed tests
            if (passed) {
                vk::DeviceSize memory_size = 0;
                auto memory_props = device.getMemoryProperties();
                for (auto& heap: memory_props.memoryHeaps) {
                    if (heap.flags & vk::MemoryHeapFlagBits::eDeviceLocal) {
                        memory_size += heap.size;
                    }
                }
                if (props.deviceType == _preferred_device_type) memory_size += 1ull << 63ull;
                matching_devices.emplace_back(device, memory_size);
            }
            std::string pass_str = passed ? "passed" : "failed";
            fmt::println("-> {}: {}", pass_str, (const char*)props.deviceName);
        }

        // optionally bail out
        if (matching_devices.size() == 0) {
            fmt::println("None of the devices match the requirements");
            exit(0);
        }

        // sort devices by favouring certain gpu types and large local memory heaps
        typedef std::pair<vk::PhysicalDevice, vk::DeviceSize> DeviceEntry;
        auto fnc_sorter = [&](DeviceEntry& a, DeviceEntry& b) {
            return a.second > b.second;
        };
        std::sort(matching_devices.begin(), matching_devices.end(), fnc_sorter);
        vk::PhysicalDevice phys_device = std::get<0>(matching_devices.front());
        fmt::println("Picked device: {}", (const char*)phys_device.getProperties().deviceName);
        return phys_device;
    }
    auto create_logical_device(vk::PhysicalDevice physical_device, void* additional_features_p) -> std::pair<vk::Device, std::vector<uint32_t>>;

private:
    bool check_api_ver(vk::PhysicalDeviceProperties& props);
    bool check_extensions(std::set<std::string> required_extensions, vk::PhysicalDevice physical_device);
    bool check_core_features(vk::PhysicalDevice phys_device);
    bool check_presentation(vk::PhysicalDevice physical_device, vk::SurfaceKHR surface) {
        // check if presentation capabilties are required
        if (surface == nullptr) return true;

        // create dummy queues to check presentation capabilities
        auto [queue_infos, _] = create_queue_infos(physical_device);
        vk::Bool32 passed = false;
        // check each queue family for presentation capabilities
        for (auto& info: queue_infos) {
            vk::Bool32 b = physical_device.getSurfaceSupportKHR(info.queueFamilyIndex, surface);
            passed |= b;
        }

        if (!passed) fmt::print("Missing presentation capabilities");
        return passed;
    }
    auto create_queue_infos(vk::PhysicalDevice physical_device) -> std::pair<std::vector<vk::DeviceQueueCreateInfo>, std::vector<uint32_t>> {
        // <queue family index, count of queue capabilities>
        typedef std::pair<uint32_t, uint32_t> QueueCount;
        std::vector<std::vector<QueueCount>> queue_family_counters;
        queue_family_counters.resize(_required_queues.size());

        // iterate through all queue families to store relevant ones
        auto queue_families = physical_device.getQueueFamilyProperties();
        for (uint32_t i = 0; i < queue_families.size(); i++) {
            vk::QueueFlags flags = queue_families[i].queueFlags;
            uint32_t capability_count = std::popcount((uint32_t)flags);
            // check if this queue family works for each requested queue
            for (size_t q = 0; q < _required_queues.size(); q++) {
                vk::QueueFlags masked = flags & _required_queues[q];
                if (masked == _required_queues[q]) {
                    queue_family_counters[q].emplace_back(i, capability_count);
                }
            }
        }

        // track unique queue families
        std::set<uint32_t> unique_queue_families;
        // contains queue family indices for requested queues
        std::vector<uint32_t> queue_mappings(_required_queues.size());
        // set up queue create infos with unique family indices
        std::vector<vk::DeviceQueueCreateInfo> info_queues;
        for (size_t i = 0; i < _required_queues.size(); i++) {
            // sort queues by capability count
            auto& vec = queue_family_counters[i];
            std::sort(vec.begin(), vec.end(), [](QueueCount& a, QueueCount& b){
                return a.second < b.second;
            });
            // use the queue with the fewest capabilities possible
            uint32_t queue_family_index = vec.front().first;

            // map the requested queue
            queue_mappings[i] = queue_family_index;

            // create new queue family info if unique
            auto [unique_it, unique_b] = unique_queue_families.emplace(queue_family_index);
            if (!unique_b) continue;
            info_queues.push_back({
                .queueFamilyIndex = queue_family_index,
                .queueCount = 1,
                .pQueuePriorities = &queue_priority,
            });
        }
        return std::make_pair(info_queues, queue_mappings);
    }

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