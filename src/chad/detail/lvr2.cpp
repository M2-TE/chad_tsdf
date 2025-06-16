#include <fmt/base.h>
#include <lvr2/geometry/PMPMesh.hpp>
#include <lvr2/geometry/BoundingBox.hpp>
#include <lvr2/reconstruction/HashGrid.hpp>
#include <lvr2/reconstruction/FastBox.hpp>
#include <lvr2/reconstruction/QueryPoint.hpp>
#include <lvr2/reconstruction/FastReconstruction.hpp>
#include <lvr2/algorithm/NormalAlgorithms.hpp>
#include "chad/detail/lvr2.hpp"
#include "chad/detail/morton.hpp"

namespace chad::detail {
    template<typename BaseVecT, typename BoxT>
    struct ChadGrid: public lvr2::GridBase {
        ChadGrid(const detail::NodeLevels& node_levels, uint32_t root_addr, float voxel_res, float trunc_dist): lvr2::GridBase(false) {
            m_globalIndex = 0;
            m_coordinateScales.x = 1.0;
            m_coordinateScales.y = 1.0;
            m_coordinateScales.z = 1.0;
            m_voxelsize = voxel_res;
            BoxT::m_voxelsize = voxel_res;
            const float recip = 1.0 / m_voxelsize;

            // trackers for the traversed path and nodes
            std::array<uint8_t,  NodeLevels::MAX_DEPTH> path_child;
            std::array<uint32_t, NodeLevels::MAX_DEPTH> path_addr;
            path_child.fill(0);
            path_addr.fill(0);
            path_addr[0] = root_addr;

            uint32_t depth = 0;
            while(true) {
                auto child_i = path_child[depth]++;
                // fmt::println("depth: {:2}, child: {:1}", depth, child_i);

                // when all children at this depth were iterated
                if (child_i == 8) {
                    if (depth > 0) depth--;
                    else break; // exit main loop
                }

                // node contains node children
                else if (depth < NodeLevels::MAX_DEPTH - 1) {
                    // try to find the child in current node
                    uint32_t parent_addr = path_addr[depth];
                    uint32_t child_addr = node_levels.get_child_addr(depth, parent_addr, child_i);
                    // check if child address is valid
                    if (child_addr > 0) {
                        depth++;
                        path_child[depth] = 0; // reset child index for new depth
                        path_addr[depth] = child_addr;
                    }
                }
                // node contains leaf cluster children
                else {
                    // try to get the leaf cluster, skip if it doesn't exist
                    auto [cluster, cluster_exists] = node_levels.try_get_leaf_cluster(path_addr[depth], child_i);
                    if (!cluster_exists) continue;

                    // reconstruct morton code from path
                    uint64_t code = 0;
                    for (uint64_t k = 0; k < 63/3 - 1; k++) {
                        uint64_t part = path_child[k] - 1;
                        code |= part << (60 - k*3);
                    }
                    MortonCode mc { code };
                    glm::ivec3 cluster_chunk = mc.decode();
                    
                    uint32_t leaf_i = 0;
                    for (int32_t z = 0; z <= 1; z++) {
                    for (int32_t y = 0; y <= 1; y++) {
                    for (int32_t x = 0; x <= 1; x++, leaf_i++) {
                        // signed distance within leaf
                        auto [signed_distance, leaf_exists] = cluster._tsdfs.try_get(leaf_i, trunc_dist);
                        if (!leaf_exists) continue;

                        // leaf position
                        glm::ivec3 leaf_chunk = cluster_chunk + glm::ivec3(x, y, z);
                        glm::vec3 leaf_pos = glm::vec3(leaf_chunk) * m_voxelsize;

                        // signed_distance = glm::length(leaf_pos) - 5.0f;
                        // fmt::println("sd {:.2f}, pos {:.2f} {:.2f} {:.2f}", signed_distance, leaf_pos.x, leaf_pos.y, leaf_pos.z);
                        
                        // create query point
                        size_t querypoint_i = m_queryPoints.size();
                        m_queryPoints.emplace_back(BaseVecT(leaf_pos.x, leaf_pos.y, leaf_pos.z), signed_distance);
                        
                        // 8 cells around the query point
                        std::array<glm::vec3, 8> cell_offsets = {
                            glm::vec3(+0.5, +0.5, +0.5),
                            glm::vec3(-0.5, +0.5, +0.5),
                            glm::vec3(-0.5, -0.5, +0.5),
                            glm::vec3(+0.5, -0.5, +0.5),
                            glm::vec3(+0.5, +0.5, -0.5),
                            glm::vec3(-0.5, +0.5, -0.5),
                            glm::vec3(-0.5, -0.5, -0.5),
                            glm::vec3(+0.5, -0.5, -0.5),
                        };
                        for (size_t i = 0; i < 8; i++) {
                            // create cell
                            glm::vec3 cell_center = leaf_pos + cell_offsets[i] * m_voxelsize;
                            // create morton code of cell
                            // convert position back to chunk index
                            glm::vec3 cell_pos = glm::floor(cell_center * recip);
                            glm::ivec3 cell_chunk = glm::ivec3(cell_pos);
                            // convert to morton code
                            MortonCode mc { cell_chunk };
                            // emplace cell into map, check if it already existed
                            auto [box_it, emplaced] = m_cells.emplace(mc._value, nullptr);
                            if (emplaced) {
                                box_it->second = new BoxT(BaseVecT(cell_center.x, cell_center.y, cell_center.z));
                            }
                            // place query point at the correct cell index
                            box_it->second->setVertex(i, querypoint_i);
                        }
                    }}}
                }
            }

            // cull incomplete cells
            std::vector<uint64_t> incomplete_cells;
            for (auto it = m_cells.cbegin(); it != m_cells.cend(); it++) {
                bool incomplete = false;
                for (auto i = 0; i < 8; i++) {
                    auto vertIndex = it->second->getVertex(i);
                    if (vertIndex == BoxT::INVALID_INDEX) incomplete = true;
                }
                if (incomplete) incomplete_cells.push_back(it->first);
            }
            for (auto it = incomplete_cells.cbegin(); it != incomplete_cells.cend(); it++) {
                auto cell = m_cells.find(*it);
                delete cell->second;
                m_cells.erase(cell);
            }
            std::cout << "ChadGrid created" << std::endl;
        }
        ~ChadGrid() {
            lvr2::GridBase::~GridBase();
        }
        
        auto getNumberOfCells() -> std::size_t { 
            return m_cells.size();
        }
        auto firstCell() -> typename std::unordered_map<size_t, BoxT*>::iterator { 
            return m_cells.begin(); 
        }
        auto lastCell() -> typename unordered_map<size_t, BoxT*>::iterator {
            return m_cells.end();
        }
        auto firstQueryPoint() -> typename std::vector<lvr2::QueryPoint<BaseVecT>>::iterator {
            return m_queryPoints.begin();
        }
        auto lastQueryPoint() -> typename std::vector<lvr2::QueryPoint<BaseVecT>>::iterator {
            return m_queryPoints.end();
        }
        auto getQueryPoints() -> vector<lvr2::QueryPoint<BaseVecT>>& {
            return m_queryPoints;
        }
        auto getCells() -> unordered_map<size_t, BoxT*>& {
            return m_cells;
        }

        // implement pure virtual functions
        void addLatticePoint(int i, int j, int k, float distance = 0.0) override {
            (void)i;
            (void)j;
            (void)k;
            (void)distance;
        }
        void saveGrid(std::string file) override {
            (void)file;
            // // store all the points into .grid file
            // std::ofstream output;
            // output.open(file, std::ofstream::trunc | std::ofstream::binary);
            // // store header data
            // float voxel_res = LEAF_RESOLUTION;
            // output.write(reinterpret_cast<char*>(&voxel_res), sizeof(float));
            // size_t query_points_n = getQueryPoints().size();
            // output.write(reinterpret_cast<char*>(&query_points_n), sizeof(size_t));
            // size_t cells_n = getNumberOfCells();
            // output.write(reinterpret_cast<char*>(&cells_n), sizeof(size_t));
            // // store query points (vec3 + float)
            // auto& query_points = getQueryPoints();
            // for (auto cur = query_points.cbegin(); cur != query_points.cend(); cur++) {
            //     Eigen::Vector3f pos { cur->m_position.x, cur->m_position.y, cur->m_position.z };
            //     float signed_distance = cur->m_distance;
            //     output.write(reinterpret_cast<const char*>(&pos), sizeof(Eigen::Vector3f));
            //     output.write(reinterpret_cast<const char*>(&signed_distance), sizeof(float));
            // }
            // // store cells (8x uint32_t)
            // for (auto cur = firstCell(); cur != lastCell(); cur++) {
            //     auto* cell = cur->second;
            //     for (size_t i = 0; i < 8; i++) {
            //         uint32_t i_query_point = cell->getVertex(i);
            //         output.write(reinterpret_cast<const char*>(&i_query_point), sizeof(uint32_t));
            //     }
            // }
            // output.close();
            // std::cout << "Saved grid as " << file << '\n';
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
        void getMesh(lvr2::BaseMesh<BaseVecT>& mesh, lvr2::BoundingBox<BaseVecT>& bb, vector<unsigned int>& duplicates, float comparePrecision) override {
            (void)mesh;
            (void)bb;
            (void)duplicates;
            (void)comparePrecision;
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

    void reconstruct(const detail::Submap& submap, const NodeLevels& node_levels, float voxel_res, float trunc_dist, std::string_view filename) {
        // begin 3D mesh reconstruction using LVR2
        typedef lvr2::BaseVector<float> VecT;
        typedef lvr2::BilinearFastBox<VecT> BoxT;
        
        // create hash grid from entire tree
        // generate mesh from hash grid
        lvr2::PMPMesh<VecT> mesh{};
        std::string decomp_type = "PMC";
        if (decomp_type == "MC") {
        }
        else if (decomp_type == "PMC") {
            // auto node_levels = chad.get_node_levels();
            // auto leaf_level = chad.get_leaf_level();
            auto grid_p = std::make_shared<ChadGrid<VecT, BoxT>>(node_levels, submap.root_addr_tsdf, voxel_res, trunc_dist);
            // if (save_grid) grid_p->saveGrid("hashgrid.grid");
            
            ChadReconstruction<VecT, BoxT> reconstruction { grid_p };
            reconstruction.getMesh(mesh);
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