#pragma once

namespace chad::detail {
    struct Ply {
        Ply(const std::string& filename) {
            std::ofstream ofs{ filename, std::ios::binary };
            ofs << std::string("ply\n");
            ofs << std::string("format binary_little_endian 1.0\n");
            ofs << std::string("comment Mesh reconstructed by CHAD TSDF\n");
            ofs << std::string("element vertex                     \n");
            ofs << std::string("property float32 x\n");
            ofs << std::string("property float32 y\n");
            ofs << std::string("property float32 z\n");
            ofs << std::string("property float32 nx\n");
            ofs << std::string("property float32 ny\n");
            ofs << std::string("property float32 nz\n");
            ofs << std::string("property uint8 red\n");
            ofs << std::string("property uint8 green\n");
            ofs << std::string("property uint8 blue\n");
            ofs << std::string("element face                     \n");
            ofs << std::string("property list uint8 uint32 vertex_indices\n");
            ofs << std::string("end_header\n");
            uint32_t ofs_header_pos = ofs.tellp();

            struct Vertex {
                void write(std::ofstream& ofs) const {
                    ofs.write(reinterpret_cast<const char*>(&_position), sizeof(_position));
                    ofs.write(reinterpret_cast<const char*>(&_normal), sizeof(_normal));
                    ofs.write(reinterpret_cast<const char*>(&_color), sizeof(_color));
                }
                glm::f32vec3 _position{ 0, 0, 0 };
                glm::f32vec3 _normal{ 0, 1, 0 };
                glm::u8vec3 _color{ 255, 0, 0 };
            } v;
            struct Face {
                void write(std::ofstream& ofs) const {
                    const uint8_t vertcount = 3;
                    ofs.write(reinterpret_cast<const char*>(&vertcount), sizeof(vertcount));
                    ofs.write(reinterpret_cast<const char*>(&_indices), sizeof(_indices));
                }
                glm::u32vec3 _indices{ 0, 1, 2 };
            } face;

            v._position = { 0, 0, 0 };
            v.write(ofs);
            v._position = { 1.124, 0, 0 };
            v.write(ofs);
            v._position = { 0, 1.6713, 0 };
            v.write(ofs);
            face.write(ofs);
        }
    };
}