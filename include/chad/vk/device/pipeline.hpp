#pragma once
#include <string_view>
#include <vulkan/vulkan.hpp>
#include <spvrc/spvrc.hpp>
#include <fmt/base.h>
#include "chad/vk/device/buffer.hpp"
#include "chad/vk/device/image.hpp"

namespace dv
{
struct Base {
public:
	typedef std::vector<std::tuple<uint32_t /*set*/, uint32_t /*binding*/, vk::SamplerCreateInfo>> SamplerInfos;
	void destroy(vk::Device device) {
		device.destroyPipeline(_pipeline);
		device.destroyPipelineLayout(_pipeline_layout);

		for (auto& layout: _desc_set_layouts) device.destroyDescriptorSetLayout(layout);
		for (auto& sampler: _immutable_samplers) device.destroySampler(sampler);
		device.destroyDescriptorPool(_pool);
		_desc_sets.clear();
		_desc_set_layouts.clear();
		_immutable_samplers.clear();
	}
	void write_descriptor(vk::Device device, uint32_t set, uint32_t binding, dv::Image& image, vk::DescriptorType type, vk::Sampler sampler = nullptr) {
		vk::DescriptorImageInfo info_image {
			.sampler = sampler,
			.imageView = image._view,
		};
		switch (type) {
			case vk::DescriptorType::eStorageImage:
				info_image.imageLayout = vk::ImageLayout::eGeneral;
				break;
			case vk::DescriptorType::eSampledImage:
			case vk::DescriptorType::eCombinedImageSampler:
				info_image.imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal;
				break;
			default:
				assert(false && "Invalid descriptor type");
				break;
		}
		vk::WriteDescriptorSet write_image {
			.dstSet = _desc_sets[set],
			.dstBinding = binding,
			.descriptorCount = 1,
			.descriptorType = type,
			.pImageInfo = &info_image,
		};
		device.updateDescriptorSets(write_image, {});
	}
	void write_descriptor(vk::Device device, uint32_t set, uint32_t binding, dv::Buffer& buffer, vk::DescriptorType type, vk::DeviceSize offset = 0) {
		vk::DescriptorBufferInfo info_buffer {
			.buffer = buffer._data,
			.offset = offset,
			.range = buffer._size,
		};
		vk::WriteDescriptorSet write_buffer {
			.dstSet = _desc_sets[set],
			.dstBinding = binding,
			.dstArrayElement = 0,
			.descriptorCount = 1,
			.descriptorType = type,
			.pBufferInfo = &info_buffer
		};
		device.updateDescriptorSets(write_buffer, {});
	}
	
protected:
	auto reflect(vk::Device device, const vk::ArrayProxy<std::string_view>& shaderPaths, const SamplerInfos& sampler_infos)
	-> std::pair<vk::VertexInputBindingDescription, std::vector<vk::VertexInputAttributeDescription>>;

protected:
	vk::Pipeline _pipeline;
	vk::PipelineLayout _pipeline_layout;
	vk::DescriptorPool _pool;
	std::vector<vk::DescriptorSet> _desc_sets;
	std::vector<vk::DescriptorSetLayout> _desc_set_layouts;
	std::vector<vk::Sampler> _immutable_samplers;
};
struct Compute: Base {
	struct CreateInfo {
		vk::Device device;
		//
		std::string_view cs_path;
		vk::SpecializationInfo* spec_info = nullptr;
		//
		SamplerInfos sampler_infos = {};
	};
	void init(const CreateInfo& info) {
		// reflect shader contents
		reflect(info.device, info.cs_path, info.sampler_infos);

		// create pipeline layout
		_pipeline_layout = info.device.createPipelineLayout({
			.setLayoutCount = (uint32_t)_desc_set_layouts.size(),
			.pSetLayouts = _desc_set_layouts.data(),
		});

		// create pipeline
		auto [cs_code, cs_size] = spvrc::load(info.cs_path);
		vk::ShaderModuleCreateInfo info_cs {
			.codeSize = cs_size * sizeof(uint32_t),
			.pCode = cs_code,
		};
		vk::ShaderModule cs_module = info.device.createShaderModule(info_cs);
		vk::ComputePipelineCreateInfo info_compute_pipe {
			.stage = {
				.stage = vk::ShaderStageFlagBits::eCompute,
				.module = cs_module,
				.pName = "main",
				.pSpecializationInfo = info.spec_info,
			},
			.layout = _pipeline_layout,
		};
		auto [result, pipeline] = info.device.createComputePipeline(nullptr, info_compute_pipe);
		if (result != vk::Result::eSuccess) fmt::println("error creating compute pipeline");
		_pipeline = pipeline;
		// clean up shader module
		info.device.destroyShaderModule(cs_module);
	}
	void execute(vk::CommandBuffer cmd, uint32_t nx, uint32_t ny, uint32_t nz) {
		cmd.bindPipeline(vk::PipelineBindPoint::eCompute, _pipeline);
		cmd.bindDescriptorSets(vk::PipelineBindPoint::eCompute, _pipeline_layout, 0, _desc_sets, {});
		cmd.dispatch(nx, ny, nz);
	}
};
} // namespace dv