#include <lvr2/geometry/PMPMesh.hpp>
#include <lvr2/geometry/BoundingBox.hpp>
#include <lvr2/reconstruction/HashGrid.hpp>
#include <lvr2/reconstruction/FastBox.hpp>
#include <lvr2/reconstruction/QueryPoint.hpp>
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include "chad/detail/lvr2.hpp"

namespace chad::detail {
    template<typename BaseVecT, typename BoxT>
    struct ChadGrid: public lvr2::GridBase {
        // ChadGrid(std::array<std::vector<uint32_t>*, 63/3 - 1> node_levels, std::vector<LeafCluster::ClusterValue>& leaf_level, uint32_t root_addr, double voxel_res): lvr2::GridBase(false) {
        //     m_globalIndex = 0;
        //     m_coordinateScales.x = 1.0;
        //     m_coordinateScales.y = 1.0;
        //     m_coordinateScales.z = 1.0;
        //     m_voxelsize = voxel_res;
        //     BoxT::m_voxelsize = voxel_res;

        //     // trackers that will be updated during traversal
        //     static constexpr std::size_t max_depth = 63/3 - 1;
        //     std::array<uint8_t, max_depth> path;
        //     std::array<Node*, max_depth> nodes;
        //     path.fill(0);
        //     nodes.fill(nullptr);
        //     // initialize
        //     nodes[0] = Node::from_addr(*node_levels[0], root_addr);

        //     uint_fast32_t depth = 0;
        //     while(true) {
        //         auto child_i = path[depth]++;
        //         if (child_i == 8) {
        //             if (depth > 0) depth--;
        //             else break; // exit main loop
        //         }

        //         // normal node
        //         else if (depth < max_depth - 1) {
        //             // read current parent node
        //             auto* node_p = nodes[depth];
        //             if (node_p->contains_child(child_i)) {
        //                 uint32_t child_addr = node_p->get_child_addr(child_i);
        //                 depth++;
        //                 path[depth] = 0;
        //                 nodes[depth] = Node::from_addr(*node_levels[depth], child_addr);
        //             }
        //         }
        //         // leaf cluster node
        //         else {
        //             // skip if child does not exist
        //             auto* node_p = nodes[depth];
        //             if (!node_p->contains_child(child_i)) continue;
        //             uint32_t child_addr = node_p->get_child_addr(child_i);
        //             // construct helper class for leaf cluster data
        //             LeafCluster leaf_cluster{ leaf_level[child_addr] };

        //             // reconstruct morton code from path
        //             uint64_t code = 0;
        //             for (uint64_t k = 0; k < 63/3 - 1; k++) {
        //                 uint64_t part = path[k] - 1;
        //                 code |= part << (60 - k*3);
        //             }
        //             // convert into chunk position of leaf cluster
        //             Eigen::Vector3i cluster_chunk;
        //             std::tie(cluster_chunk.x(), cluster_chunk.y(), cluster_chunk.z()) = mortonnd::MortonNDBmi_3D_64::Decode(code);
        //             // convert from 21-bit inverted to 32-bit integer
        //             cluster_chunk = cluster_chunk.unaryExpr([](auto i){ return i - (1 << 20); });
                    
        //             uint32_t leaf_i = 0;
        //             for (int32_t z = 0; z <= 1; z++) {
        //             for (int32_t y = 0; y <= 1; y++) {
        //             for (int32_t x = 0; x <= 1; x++, leaf_i++) {
        //                 // leaf position
        //                 Eigen::Vector3i leaf_chunk = cluster_chunk + Eigen::Vector3i(x, y, z);
        //                 Eigen::Vector3f leaf_pos = leaf_chunk.cast<float>() * m_voxelsize;
                        
        //                 auto sd_opt = leaf_cluster.get_leaf(leaf_i);
        //                 if (!sd_opt) continue; // skip invalid leaves
        //                 // signed distance for this leaf
        //                 float sd = sd_opt.value();
        //                 // float sd_perfect = leaf_pos.cast<double>().norm() - 5.0;
        //                 // sd_perfect = std::clamp(sd_perfect, -m_voxelsize, +m_voxelsize);
        //                 // sd = sd_perfect;
        //                 // std::cout << "perfect: " << sd_perfect << " actual: " << sd_opt.value() << '\n';
                        
        //                 // create query point
        //                 size_t querypoint_i = m_queryPoints.size();
        //                 m_queryPoints.emplace_back(BaseVecT(leaf_pos.x(), leaf_pos.y(), leaf_pos.z()), sd);
                        
        //                 // 8 cells around the query point
        //                 std::array<Eigen::Vector3f, 8> cell_offsets = {
        //                     Eigen::Vector3f(+0.5, +0.5, +0.5),
        //                     Eigen::Vector3f(-0.5, +0.5, +0.5),
        //                     Eigen::Vector3f(-0.5, -0.5, +0.5),
        //                     Eigen::Vector3f(+0.5, -0.5, +0.5),
        //                     Eigen::Vector3f(+0.5, +0.5, -0.5),
        //                     Eigen::Vector3f(-0.5, +0.5, -0.5),
        //                     Eigen::Vector3f(-0.5, -0.5, -0.5),
        //                     Eigen::Vector3f(+0.5, -0.5, -0.5),
        //                 };
        //                 for (size_t i = 0; i < 8; i++) {
        //                     // create cell
        //                     Eigen::Vector3f cell_center = leaf_pos + cell_offsets[i] * m_voxelsize;
        //                     // create morton code of cell
        //                     const float recip = 1.0 / m_voxelsize;
        //                     // convert position back to chunk index
        //                     Eigen::Vector3f cell_pos = cell_center * recip;
        //                     cell_pos = cell_pos.unaryExpr([](float f){ return std::floor(f); });
        //                     Eigen::Vector3i cell_chunk = cell_pos.cast<int32_t>();
        //                     // convert to 21-bit ints
        //                     cell_chunk = cell_chunk.unaryExpr([](auto i){ return i + (1 << 20); });
        //                     uint64_t mc = mortonnd::MortonNDBmi_3D_64::Encode(cell_chunk.x(), cell_chunk.y(), cell_chunk.z());
        //                     // emplace cell into map, check if it already existed
        //                     auto [box_it, emplaced] = m_cells.emplace(mc, nullptr);
        //                     if (emplaced) {
        //                         box_it->second = new BoxT(BaseVecT(cell_center.x(), cell_center.y(), cell_center.z()));
        //                     }
        //                     // place query point at the correct cell index
        //                     box_it->second->setVertex(i, querypoint_i);
        //                 }
        //             }}}
        //         }
        //     }

        //     // cull incomplete cells
        //     std::vector<uint64_t> incomplete_cells;
        //     for (auto it = m_cells.cbegin(); it != m_cells.cend(); it++) {
        //         bool incomplete = false;
        //         for (auto i = 0; i < 8; i++) {
        //             auto vertIndex = it->second->getVertex(i);
        //             if (vertIndex == BoxT::INVALID_INDEX) incomplete = true;
        //         }
        //         if (incomplete) incomplete_cells.push_back(it->first);
        //     }
        //     for (auto it = incomplete_cells.cbegin(); it != incomplete_cells.cend(); it++) {
        //         auto cell = m_cells.find(*it);
        //         delete cell->second;
        //         m_cells.erase(cell);
        //     }
        //     std::cout << "ChadGrid created" << std::endl;
        // }
        ~ChadGrid() {
            lvr2::GridBase::~GridBase();
        }

    private:
        vector<lvr2::QueryPoint<BaseVecT>> m_queryPoints;
        unordered_map<size_t, BoxT*> m_cells;
        unordered_map<size_t, size_t> m_qpIndices;
        // DEPRECATED, check if bounding box is even needed
        lvr2::BoundingBox<BaseVecT> qp_bb;
        lvr2::BoundingBox<BaseVecT> m_boundingBox;
        float m_voxelsize;
        std::size_t m_maxIndex;
        std::size_t m_maxIndexSquare;
        std::size_t m_maxIndexX;
        std::size_t m_maxIndexY;
        std::size_t m_maxIndexZ;
        std::string m_boxType;
        unsigned int m_globalIndex;
        BaseVecT m_coordinateScales;
    };

    template<typename BaseVecT, typename BoxT>
    struct ChadReconstruction: public lvr2::FastReconstructionBase<BaseVecT> {
        ChadReconstruction(shared_ptr<ChadGrid<BaseVecT, BoxT>> grid) {
            m_grid = grid;
        }
        virtual ~ChadReconstruction() {
        }
        void getMesh(
            [[maybe_unused]] lvr2::BaseMesh<BaseVecT>& mesh,
            [[maybe_unused]] lvr2::BoundingBox<BaseVecT>& bb,
            [[maybe_unused]] vector<unsigned int>& duplicates,
            [[maybe_unused]] float comparePrecision) override {
        }
        void getMesh(lvr2::BaseMesh<BaseVecT> &mesh) override {
            // Status message for mesh generation
            string comment = lvr2::timestamp.getElapsedTime() + "Creating mesh ";
            lvr2::ProgressBar progress(m_grid->getNumberOfCells(), comment);

            // Some pointers
            BoxT* b;
            unsigned int global_index = mesh.numVertices();

            // Iterate through cells and calculate local approximations
            typename unordered_map<size_t, BoxT*>::iterator it;
            for(it = m_grid->firstCell(); it != m_grid->lastCell(); it++)
            {
                b = it->second;
                b->getSurface(mesh, m_grid->getQueryPoints(), global_index);
                if(!lvr2::timestamp.isQuiet())
                    ++progress;
            }

            if(!lvr2::timestamp.isQuiet())
                cout << endl;

            lvr2::BoxTraits<BoxT> traits;

            if(traits.type == "SharpBox")  // Perform edge flipping for extended marching cubes
            {
                string SFComment = lvr2::timestamp.getElapsedTime() + "Flipping edges  ";
                lvr2::ProgressBar SFProgress(this->m_grid->getNumberOfCells(), SFComment);
                for(it = this->m_grid->firstCell(); it != this->m_grid->lastCell(); it++)
                {

                    lvr2::SharpBox<BaseVecT>* sb;
                    sb = reinterpret_cast<lvr2::SharpBox<BaseVecT>* >(it->second);
                    if(sb->m_containsSharpFeature)
                    {
                        lvr2::OptionalVertexHandle v1;
                        lvr2::OptionalVertexHandle v2;
                        lvr2::OptionalEdgeHandle e;

                        if(sb->m_containsSharpCorner)
                        {
                            // 1
                            v1 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][0]];
                            v2 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][1]];

                            if(v1 && v2)
                            {
                                e = mesh.getEdgeBetween(v1.unwrap(), v2.unwrap());
                                if(e)
                                {
                                    mesh.flipEdge(e.unwrap());
                                }
                            }

                            // 2
                            v1 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][2]];
                            v2 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][3]];

                            if(v1 && v2)
                            {
                                e = mesh.getEdgeBetween(v1.unwrap(), v2.unwrap());
                                if(e)
                                {
                                    mesh.flipEdge(e.unwrap());
                                }
                            }

                            // 3
                            v1 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][4]];
                            v2 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][5]];

                            if(v1 && v2)
                            {
                                e = mesh.getEdgeBetween(v1.unwrap(), v2.unwrap());
                                if(e)
                                {
                                    mesh.flipEdge(e.unwrap());
                                }
                            }

                        }
                        else
                        {
                            // 1
                            v1 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][0]];
                            v2 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][1]];

                            if(v1 && v2)
                            {
                                e = mesh.getEdgeBetween(v1.unwrap(), v2.unwrap());
                                if(e)
                                {
                                    mesh.flipEdge(e.unwrap());
                                }
                            }

                            // 2
                            v1 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][4]];
                            v2 = sb->m_intersections[lvr2::ExtendedMCTable[sb->m_extendedMCIndex][5]];

                            if(v1 && v2)
                            {
                                e = mesh.getEdgeBetween(v1.unwrap(), v2.unwrap());
                                if(e)
                                {
                                    mesh.flipEdge(e.unwrap());
                                }
                            }
                        }
                    }
                    ++SFProgress;
                }
                cout << endl;
            }

            if(traits.type == "BilinearFastBox")
            {
                string comment = lvr2::timestamp.getElapsedTime() + "Optimizing plane contours  ";
                lvr2::ProgressBar progress(this->m_grid->getNumberOfCells(), comment);
                for(it = this->m_grid->firstCell(); it != this->m_grid->lastCell(); it++)
                {
                // F... type safety. According to traits object this is OK!
                    lvr2::BilinearFastBox<BaseVecT>* box = reinterpret_cast<lvr2::BilinearFastBox<BaseVecT>*>(it->second);
                    box->optimizePlanarFaces(mesh, 5);
                    ++progress;
                }
                cout << endl;
            }
        }
    private:
        shared_ptr<ChadGrid<BaseVecT, BoxT>> m_grid;
    };

    void reconstruct(std::string_view filename) {

        // begin 3D mesh reconstruction using LVR2
        [[maybe_unused]] typedef lvr2::BaseVector<float> VecT;
        [[maybe_unused]] typedef lvr2::BilinearFastBox<VecT> BoxT;
        
        // create hash grid from entire tree
        // generate mesh from hash grid
        lvr2::PMPMesh<VecT> mesh{};
        std::string decomp_type = "PMC";
        if (decomp_type == "MC") {
        }
        else if (decomp_type == "PMC") {
            // auto node_levels = chad.get_node_levels();
            // auto leaf_level = chad.get_leaf_level();
            // auto grid_p = std::make_shared<ChadGrid<VecT, BoxT>>(node_levels, leaf_level, root_addr, LEAF_RESOLUTION);
            // if (save_grid) grid_p->saveGrid("hashgrid.grid");
            
            // ChadReconstruction<VecT, BoxT> reconstruction { grid_p };
            // reconstruction.getMesh(mesh);
            
        }
        
        // generate mesh buffer from reconstructed mesh
        auto norm_face = lvr2::calcFaceNormals(mesh);
        auto norm_vert = lvr2::calcVertexNormals(mesh, norm_face);
        lvr2::MeshBufferPtr mesh_buffer_p;
        if (false) {
            // coloring
            auto cluster_map = lvr2::planarClusterGrowing(mesh, norm_face, 0.85);
            lvr2::ClusterPainter cluster_painter { cluster_map };
            lvr2::ColorGradient::GradientType t = lvr2::ColorGradient::gradientFromString("GREY");
            auto cluster_colors = boost::optional<lvr2::DenseClusterMap<lvr2::RGB8Color>>(cluster_painter.colorize(mesh, t));
            lvr2::TextureFinalizer<lvr2::BaseVector<float>> finalizer { cluster_map };
            finalizer.setClusterColors(*cluster_colors);
            finalizer.setVertexNormals(norm_vert);
            mesh_buffer_p = finalizer.apply(mesh);
        }
        else {
            // calc normals for vertices
            lvr2::SimpleFinalizer<lvr2::BaseVector<float>> finalizer;
            finalizer.setNormalData(norm_vert);
            mesh_buffer_p = finalizer.apply(mesh);
        }

        // save to disk
        auto model_p = std::make_shared<lvr2::Model>(mesh_buffer_p);
        lvr2::ModelFactory::saveModel(model_p, filename.data());
        std::cout << "Saved mesh to " << filename << std::endl;
    }
}