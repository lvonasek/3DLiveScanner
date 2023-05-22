#include "tango/scan.h"

#include <ctime>

namespace oc {

    bool GridIndex::operator==(const GridIndex &o) const {
        return indices[0] == o.indices[0] && indices[1] == o.indices[1] && indices[2] == o.indices[2];
    }

    TangoScan::TangoScan() {
        graph = 0;
        compo = 0;
        merge = 0;
    }

    TangoScan::~TangoScan() {
        if (context != nullptr) {
            Tango3DR_ReconstructionContext_destroy(context);
            context = nullptr;
        }
    }

    void TangoScan::Clear() {
        ClearGeometry();
        ClearContext();
    }

    void TangoScan::ClearContext() {
        Tango3DR_clear(context);
        Tango3DR_ReconstructionContext_destroy(context);
        Setup3DR(res_, dmin_, dmax_, noise_, clearing_);
        lastMerged.clear();
    }

    void TangoScan::ClearGeometry() {
        Tango3DR_Status ret;
        for (std::pair<GridIndex, Tango3DR_Mesh*> p : meshes) {
            ret = Tango3DR_Mesh_destroy(p.second);
            if (ret != TANGO_3DR_SUCCESS)
                exit(EXIT_SUCCESS);
            delete p.second;
        }
        meshes.clear();
    }

    std::string TangoScan::DebugInfo() {
        std::string output = "";
        char buffer[4096];
        if (graph > 10) {
            sprintf(buffer, "Generating graph: %dms\n", graph);
            output += buffer;
        }
        if (compo > 10) {
            sprintf(buffer, "Generating components: %dms\n", compo);
            output += buffer;
        }
        if (merge > 10) {
            sprintf(buffer, "Merging components: %dms\n", merge);
            output += buffer;
        }
        return output;
    }

    void TangoScan::Delete(std::vector<GridIndex>& toDelete) {
        Tango3DR_Status ret;
        for (GridIndex gi : toDelete) {
            if (meshes.find(gi) != meshes.end()) {
                ret = Tango3DR_Mesh_destroy(meshes[gi]);
                if (ret != TANGO_3DR_SUCCESS)
                    exit(EXIT_SUCCESS);
                delete meshes[gi];
                meshes.erase(gi);
            }
        }
    }

    std::vector<Mesh> TangoScan::Export() {
        std::vector<Mesh> output;
        for (auto& m : meshes) {
            Mesh mesh;
            for (int i = 0; i < m.second->num_vertices; i++) {
                glm::vec3 v(0);
                v.x = m.second->vertices[i][0];
                v.y = m.second->vertices[i][1];
                v.z = m.second->vertices[i][2];
                mesh.vertices.push_back(v);

                glm::ivec3 c(0);
                c.r = m.second->colors[i][0];
                c.g = m.second->colors[i][1];
                c.b = m.second->colors[i][2];
                mesh.colors.push_back(File3d::CodeColor(c));
            }
            for (int i = 0; i < m.second->num_faces; i++) {
                mesh.indices.push_back(m.second->faces[i][0]);
                mesh.indices.push_back(m.second->faces[i][1]);
                mesh.indices.push_back(m.second->faces[i][2]);
            }
            output.push_back(mesh);
        }
        return output;
    }

    void TangoScan::Merge() {
        lastMerged.clear();
        Tango3DR_Status ret;
        for (std::pair<GridIndex, Tango3DR_Mesh*> p : added) {
            lastMerged.push_back(p.first);
            if (meshes.find(p.first) != meshes.end()) {
                ret = Tango3DR_Mesh_destroy(meshes[p.first]);
                if (ret != TANGO_3DR_SUCCESS)
                    exit(EXIT_SUCCESS);
                delete meshes[p.first];
            }
            meshes[p.first] = p.second;
        }
    }

    void TangoScan::Merge(std::vector<std::pair<GridIndex, Tango3DR_Mesh *>>& data) {
        Tango3DR_Status ret;
        for (std::pair<GridIndex, Tango3DR_Mesh*> p : data) {
            if (meshes.find(p.first) != meshes.end()) {
                ret = Tango3DR_Mesh_destroy(meshes[p.first]);
                if (ret != TANGO_3DR_SUCCESS)
                    exit(EXIT_SUCCESS);
                delete meshes[p.first];
            }
            meshes[p.first] = p.second;
        }
    }

    void TangoScan::Reset3DR(double res, double dmin, double dmax, int noise, bool clearing) {
        Tango3DR_Status ret;
        for (std::pair<GridIndex, Tango3DR_Mesh*> p : meshes) {
            ret = Tango3DR_Mesh_destroy(p.second);
            if (ret != TANGO_3DR_SUCCESS)
                exit(EXIT_SUCCESS);
            delete p.second;
        }
        meshes.clear();
        Tango3DR_clear(context);
        Tango3DR_ReconstructionContext_destroy(context);
        Setup3DR(res, dmin, dmax, noise, clearing);
        lastMerged.clear();
    }

    void TangoScan::Setup3DR(double res, double dmin, double dmax, int noise, bool clearing) {
        Tango3DR_Config t3dr_config = Tango3DR_Config_create(TANGO_3DR_CONFIG_RECONSTRUCTION);
        Tango3DR_Status t3dr_err;
        t3dr_err = Tango3DR_Config_setDouble(t3dr_config, "resolution", res);
        if (t3dr_err != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);

        t3dr_err = Tango3DR_Config_setDouble(t3dr_config, "min_depth", dmin);
        if (t3dr_err != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);

        t3dr_err = Tango3DR_Config_setDouble(t3dr_config, "max_depth", dmax);
        if (t3dr_err != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);

        t3dr_err = Tango3DR_Config_setBool(t3dr_config, "generate_color", true);
        if (t3dr_err != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);

        t3dr_err = Tango3DR_Config_setBool(t3dr_config, "use_space_clearing", clearing);
        if (t3dr_err != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);

        t3dr_err = Tango3DR_Config_setBool(t3dr_config, "use_parallel_integration", true);
        if (t3dr_err != TANGO_3DR_SUCCESS)
            exit(EXIT_SUCCESS);

        Tango3DR_Config_setInt32(t3dr_config, "min_num_vertices", noise);
#ifdef TANGO
        Tango3DR_Config_setInt32(t3dr_config, "update_method", TANGO_3DR_PROJECTIVE_UPDATE);
#endif

        context = Tango3DR_ReconstructionContext_create(t3dr_config);
        if (context == nullptr)
            exit(EXIT_SUCCESS);
        Tango3DR_Config_destroy(t3dr_config);

#ifdef TANGO
        Tango3DR_ReconstructionContext_setColorCalibration(context, &camera);
        Tango3DR_ReconstructionContext_setDepthCalibration(context, &depth);
#endif

        res_ = res;
        dmin_ = dmin;
        dmax_ = dmax;
        noise_ = noise;
        clearing_ = clearing;
    }

    bool TangoScan::Update(Retango* depth, double timestamp, Tango3DR_Pose *t3dr_depth_pose,
                           Tango3DR_ImageBuffer *t3dr_image, Tango3DR_Pose *t3dr_image_pose,
                           bool postprocessing) {
        Tango3DR_Status ret;
        added.clear();
        Tango3DR_GridIndexArray t3dr_updated;
        Tango3DR_PointCloud* pcl = depth->PCL(timestamp);
        ret = Tango3DR_updateFromPointCloud(context, pcl, t3dr_depth_pose, t3dr_image, t3dr_image_pose, &t3dr_updated);
        Tango3DR_PointCloud_destroy(pcl);
        delete[] t3dr_image->data;
        if (ret != TANGO_3DR_SUCCESS) {
            return false;
        }

        unsigned long size = t3dr_updated.num_indices;
        std::pair<GridIndex, Tango3DR_Mesh*> pair;
        for (unsigned long it = 0; it < size; ++it) {
            pair.first.indices[0] = t3dr_updated.indices[it][0];
            pair.first.indices[1] = t3dr_updated.indices[it][1];
            pair.first.indices[2] = t3dr_updated.indices[it][2];

            pair.second = new Tango3DR_Mesh();
            ret = Tango3DR_extractMeshSegment(context, t3dr_updated.indices[it], pair.second);
            if (ret != TANGO_3DR_SUCCESS)
                exit(EXIT_SUCCESS);
            added.push_back(pair);
        }
        Tango3DR_GridIndexArray_destroy(&t3dr_updated);

        if (postprocessing) {
            clock_t step0 = clock();
            GenerateGraph();
            clock_t step1 = clock();
            GenerateComponents();
            clock_t step2 = clock();
            MergeComponents();
            clock_t step3 = clock();
            graph = int(step1 - step0) / 1000;
            compo = int(step2 - step1) / 1000;
            merge = int(step3 - step2) / 1000;
        }
        return size > 0;
    }

    void TangoScan::GenerateComponents() {
        components.clear();
        if (xorEdges.empty())
            return;

        std::map<std::string, std::string> point2edge;
        for (std::map<std::string, Edge>::const_iterator it = xorEdges.begin(); it != xorEdges.end(); ++it)
            point2edge[Mesh::Vector2key(Retango::Vec4ToVec3((*it).second.point[0]))] = (*it).first;

        while (!xorEdges.empty()) {
            Component c;
            std::string key = (*xorEdges.begin()).first;
            while (xorEdges.find(key) != xorEdges.end()) {
                Edge e = xorEdges[key];
                c.edges.push_back(e);
                xorEdges.erase(key);
                key = point2edge[Mesh::Vector2key(Retango::Vec4ToVec3(e.point[1]))];
            }
            if (!c.edges.empty()) {
                c.closed = glm::distance(c.edges[0].point[0], c.edges[c.edges.size() - 1].point[1]) < 0.01f;
                c.valid = true;
                components.push_back(c);
            }
        }
    }

    void TangoScan::GenerateGraph() {
        xorEdges.clear();

        Edge e;
        int ia, ib, ic;
        unsigned long i;
        std::string key;
        bool foundA, foundB;
        glm::vec4 a(1), b(1);
        std::pair<int, int> keyA, keyB;
        std::string vecA, vecB, vecAB, vecBA;
        std::map<std::string, Edge>::const_iterator itEdge;
        std::map<std::pair<int, int>, bool>::const_iterator it;
        for (std::pair<GridIndex, Tango3DR_Mesh *> &node : added) {

            //process xoring on grid level
            std::map<std::pair<int, int>, bool> xoring;
            for (i = 0; i < node.second->num_faces; ++i) {
                ia = node.second->faces[i][0];
                ib = node.second->faces[i][1];
                ic = node.second->faces[i][2];

                //AB
                keyA.first = ia;
                keyA.second = ib;
                keyB.first = ib;
                keyB.second = ia;
                foundA = xoring.find(keyA) != xoring.end();
                foundB = xoring.find(keyB) != xoring.end();
                if (!foundA && !foundB) {
                    xoring[keyA] = true;
                } else {
                    xoring.erase(foundA ? keyA : keyB);
                }

                //BC
                keyA.first = ib;
                keyA.second = ic;
                keyB.first = ic;
                keyB.second = ib;
                foundA = xoring.find(keyA) != xoring.end();
                foundB = xoring.find(keyB) != xoring.end();
                if (!foundA && !foundB) {
                    xoring[keyA] = true;
                } else {
                    xoring.erase(foundA ? keyA : keyB);
                }

                //CA
                keyA.first = ic;
                keyA.second = ia;
                keyB.first = ia;
                keyB.second = ic;
                foundA = xoring.find(keyA) != xoring.end();
                foundB = xoring.find(keyB) != xoring.end();
                if (!foundA && !foundB) {
                    xoring[keyA] = true;
                } else {
                    xoring.erase(foundA ? keyA : keyB);
                }
            }

            //process xoring on active scene level
            for (it = xoring.begin(); it != xoring.end(); ++it) {
                keyA = it->first;
                a.x = node.second->vertices[keyA.first][0];
                a.y = node.second->vertices[keyA.first][1];
                a.z = node.second->vertices[keyA.first][2];
                b.x = node.second->vertices[keyA.second][0];
                b.y = node.second->vertices[keyA.second][1];
                b.z = node.second->vertices[keyA.second][2];

                vecA = Mesh::Vector2key(Retango::Vec4ToVec3(a));
                vecB = Mesh::Vector2key(Retango::Vec4ToVec3(b));
                vecAB = vecA;
                vecAB += " ^ ";
                vecAB += vecB;
                vecBA = vecB;
                vecBA += " ^ ";
                vecBA += vecA;

                bool foundAB = xorEdges.find(vecAB) != xorEdges.end();
                bool foundBA = xorEdges.find(vecBA) != xorEdges.end();
                if (!foundAB && !foundBA) {
                    e.point[0] = a;
                    e.point[1] = b;
                    xorEdges[vecAB] = e;
                } else {
                    xorEdges.erase(foundAB ? vecAB : vecBA);
                }
            }
        }
    }

    void TangoScan::MergeComponents() {
        for (int i = 0; i < components.size(); i++) {
            if (components[i].closed || !components[i].valid)
                continue;

            while (true) {
                int index = -1;
                glm::vec4 point = components[i].edges[components[i].edges.size() - 1].point[1];
                float min = glm::distance(point, components[i].edges[0].point[0]);
                for (int j = components.size() - 1; j > i; j--) {
                    if (components[j].closed || !components[j].valid)
                        continue;
                    float distance = glm::distance(point, components[j].edges[0].point[0]);
                    if (min > distance) {
                        min = distance;
                        index = j;
                    }
                }

                if (index >= 0) {
                    components[index].valid = false;
                    std::vector<Edge>::const_iterator iter = components[i].edges.end();
                    components[i].edges.insert(iter, components[index].edges.begin(), components[index].edges.end());
                    if (glm::distance(components[i].edges[0].point[0], components[i].edges[components[i].edges.size() - 1].point[1]) < 0.01f) {
                        components[i].closed = true;
                        break;
                    }
                } else {
                    components[i].closed = true;
                    break;
                }
            }
        }
    }
}
