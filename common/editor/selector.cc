#include <map>
#include <stack>
#include "editor/selector.h"


char buffer[1024];

namespace oc {

    void Selector::CompleteSelection(std::vector<Mesh> &mesh, bool inverse) {
        for (Mesh& m : mesh)
            for (unsigned int i = 0; i < m.vertices.size(); i++)
                m.colors[i] = inverse ? 0 : DESELECT_COLOR;
    }

    void Selector::DecreaseSelection(std::vector<Mesh> &mesh) {
        //get vertices to deselect
        std::map<std::string, bool> toDeselect;
        for (Mesh& m : mesh)
            for (unsigned int i = 0; i < m.vertices.size(); i += 3)
                if ((m.colors[i + 0] != 0) ||
                    (m.colors[i + 1] != 0) ||
                    (m.colors[i + 2] != 0)) {
                    toDeselect[VertexToKey(m.vertices[i + 0])] = true;
                    toDeselect[VertexToKey(m.vertices[i + 1])] = true;
                    toDeselect[VertexToKey(m.vertices[i + 2])] = true;
                }

        //deselect vertices
        for (Mesh& m : mesh)
            for (unsigned int i = 0; i < m.vertices.size(); i += 3)
                if ((toDeselect.find(VertexToKey(m.vertices[i + 0])) != toDeselect.end()) ||
                    (toDeselect.find(VertexToKey(m.vertices[i + 1])) != toDeselect.end()) ||
                    (toDeselect.find(VertexToKey(m.vertices[i + 2])) != toDeselect.end())) {
                    m.colors[i + 0] = DESELECT_COLOR;
                    m.colors[i + 1] = DESELECT_COLOR;
                    m.colors[i + 2] = DESELECT_COLOR;
                }
    }

    glm::vec3 Selector::GetCenter(std::vector<Mesh> &mesh) {
        glm::vec3 min = glm::vec3(INT_MAX, INT_MAX, INT_MAX);
        glm::vec3 max = glm::vec3(INT_MIN, INT_MIN, INT_MIN);
        glm::vec3 p;

        for (Mesh& m : mesh) {
            for (unsigned int i = 0; i < m.vertices.size(); i++) {
                if (m.colors[i] == 0) {
                    p = m.vertices[i];
                    if (min.x > p.x)
                        min.x = p.x;
                    if (min.z > p.z)
                        min.z = p.z;
                    if (max.x < p.x)
                        max.x = p.x;
                    if (max.z < p.z)
                        max.z = p.z;
                }
            }
        }
        return (min + max) * 0.5f;
    }


    glm::vec4 Selector::GetCircle(std::vector<Mesh> &mesh, glm::mat4 world2screen, float x, float y) {
        glm::vec3 click = Transform(mesh, world2screen, x, y);
        if (hit) {
            for (Mesh& m : mesh) {
                if (m.normals.empty()) {
                    m.GenerateNormals();
                }
            }

            //find center of circle
            std::vector<float> sx, sz;
            for (Mesh& m : mesh) {
                for (glm::vec3& v : m.vertices) {
                    if (glm::distance2(v, click) <= 1) {
                        sx.push_back(v.x);
                        sz.push_back(v.z);
                    }
                }
            }
            std::sort(sx.begin(), sx.end(), std::less<float>());
            std::sort(sz.begin(), sz.end(), std::less<float>());
            glm::vec3 scenter = glm::vec3(sx[sx.size() / 2], click.y, sz[sz.size() / 2]);

            //find radius of circle
            std::vector<float> sdistances;
            sdistances.push_back(0);
            for (Mesh& m : mesh) {
                for (glm::vec3& v : m.vertices) {
                    if (fabs(v.y - click.y) < 0.1f) {
                        float dst = glm::distance(v, scenter);
                        if (dst <= 0.5f) {
                            sdistances.push_back(dst);
                        }
                    }
                }
            }
            std::sort(sdistances.begin(), sdistances.end(), std::less<float>());
            float sradius = sdistances[sdistances.size() / 2];

            CompleteSelection(mesh, true);
            return glm::vec4(scenter, sradius);
        } else {
            return glm::vec4(0);
        }
    }

    void Selector::IncreaseSelection(std::vector<Mesh>& mesh) {

        //get vertices to select
        std::map<std::string, bool> toSelect;
        for (Mesh& m : mesh)
            for (unsigned int i = 0; i < m.vertices.size(); i += 3)
                if ((m.colors[i + 0] == 0) ||
                    (m.colors[i + 1] == 0) ||
                    (m.colors[i + 2] == 0)) {
                    toSelect[VertexToKey(m.vertices[i + 0])] = true;
                    toSelect[VertexToKey(m.vertices[i + 1])] = true;
                    toSelect[VertexToKey(m.vertices[i + 2])] = true;
                }

        //select vertices
        for (Mesh &m : mesh)
            for (unsigned int i = 0; i < m.vertices.size(); i += 3)
                if ((toSelect.find(VertexToKey(m.vertices[i + 0])) != toSelect.end()) ||
                    (toSelect.find(VertexToKey(m.vertices[i + 1])) != toSelect.end()) ||
                    (toSelect.find(VertexToKey(m.vertices[i + 2])) != toSelect.end())) {
                    m.colors[i + 0] = 0;
                    m.colors[i + 1] = 0;
                    m.colors[i + 2] = 0;
                }
    }

    void Selector::Init(int w, int h) {
        SetResolution(w, h);
    }

    void Selector::Process(unsigned long &index, int &x1, int &x2, int &y, glm::dvec3 &z1, glm::dvec3 &z2) {
        switch (mode) {
            case Mode::ARRAY:
                for (unsigned int i = 0; i < points2D.size(); i++) {
                    if (points2D[i].y == y) {
                        if ((x1 <= points2D[i].x) && (points2D[i].x <= x2)) {
                            double z = z1.z + (points2D[i].x - x1) * (z2.z - z1.z) / (double)(x2 - x1);
                            if ((z > 0) && (depths[i] > z)) {
                                depths[i] = z;
                                hits[i].first = selected;
                                hits[i].second = index;
                            }
                        }
                    }
                }
                break;
            case Mode::NORMAL:
                if (pointY == y) {
                    if ((x1 <= pointX) && (pointX <= x2)) {
                        double z = z1.z + (pointX - x1) * (z2.z - z1.z) / (double)(x2 - x1);
                        if ((z > 0) && (depth > z)) {
                            depth = z;
                            selected = (int) index;
                        }
                    }
                }
                break;
            case Mode::CIRCLE:
                for (int x = x1; x <= x2; x++) {
                    float dx = pointX - x;
                    float dy = pointY - y;
                    if (dx * dx + dy * dy < circleRadius * circleRadius) {
                        currentMesh->colors[index + 0] = rangeColor;
                        currentMesh->colors[index + 1] = rangeColor;
                        currentMesh->colors[index + 2] = rangeColor;
                    }
                }
                break;
            case Mode::RECT:
                if ((x1 < pointX) && (x1 < pointX2))
                    return;
                if ((x2 > pointX) && (x2 > pointX2))
                    return;
                if ((y < pointY) && (y < pointY2))
                    return;
                if ((y > pointY) && (y > pointY2))
                    return;
                if ((z1.z < 0) || (z2.z < 0))
                    return;
                currentMesh->colors[index + 0] = rangeColor;
                currentMesh->colors[index + 1] = rangeColor;
                currentMesh->colors[index + 2] = rangeColor;
                break;
        }
    }

    std::string Selector::VertexToKey(glm::vec3& vec) {
        sprintf(buffer, "%.3f,%.3f,%.3f", vec.x, vec.y, vec.z);
        return std::string(buffer);
    }

    glm::vec3 Selector::Raycast(std::vector<Mesh>& mesh, glm::vec3 from, glm::vec3 to) {
        glm::mat4 projection = glm::perspective(60.0f, 1.0f, 0.01f, 1000.0f);
        glm::mat4 view = glm::lookAt(from, to, glm::vec3(0, 1, 0));
        return Transform(mesh, projection * view, viewport_width / 2, viewport_height / 2, false);
    }

    void Selector::SelectObject(std::vector<Mesh> &mesh, glm::mat4 world2screen, float x, float y, bool convexSearch) {

        //define limits
        float convexFactor = 1.4f;
        float normalSmoothing = 0.1f;
        float normalThreshold = 0.05f;

        //prepare model
        mode = Mode::NORMAL;
        depth = INT_MAX;
        pointX = (int) x;
        pointY = (int) y;
        int selectModel = -1;
        int selectFace = -1;
        for (unsigned int i = 0; i < mesh.size(); i++) {
            selected = -1;
            AddVertices(mesh[i].vertices, world2screen, true);
            if (selected >= 0) {
                selectModel = i;
                selectFace = selected;
            }
            if (mesh[i].normals.empty()) {
                mesh[i].GenerateNormals();
            }
        }
        if (selectModel < 0)
            return;
        CreateGraph(mesh);

        //select initial triangle
        mesh[selectModel].colors[selectFace] = 0;
        glm::vec3 v = mesh[selectModel].vertices[selectFace];
        glm::vec3 n = mesh[selectModel].normals[selectFace];

        //process
        float diff, len;
        std::string key;
        glm::vec3 n0, v0, s;
        std::map<std::string, bool> processed;
        std::stack<glm::vec3> normals, smooths, vertices;
        normals.push(n);
        smooths.push(n);
        vertices.push(v);
        while (!vertices.empty()) {
            //update queue
            n = normals.top();
            s = smooths.top();
            v = vertices.top();
            vertices.pop();
            smooths.pop();
            normals.pop();
            key = VertexToKey(v);
            if (processed.find(key) != processed.end())
                continue;
            processed[key] = true;

            //select and add neighbours
            for (std::pair<const std::pair<int, int>, bool>& i : connections[key]) {
                bool selected = false;
                for (int j = 0; j < 3; j++) {
                    v0 = mesh[i.first.first].vertices[i.first.second + j];
                    n0 = mesh[i.first.first].normals[i.first.second + j];
                    diff = glm::dot(s, n0);
                    len = glm::distance(v, v0);
                    bool convex = convexSearch && (glm::distance(v0 + n0 * len, v + n * len) > len * convexFactor);
                    bool flat = (diff > 1.0f - normalThreshold) && (diff < 1.0f + normalThreshold);
                    if (convex || flat) {
                        selected = true;
                        vertices.push(v0);
                        smooths.push(convex ? n0 : n0 * normalSmoothing + s * (1.0f - normalSmoothing));
                        normals.push(n0);
                    }
                }
                if (selected) {
                    for (int j = 0; j < 3; j++) {
                        mesh[i.first.first].colors[i.first.second + j] = 0;
                    }
                }
            }
        }
    }

    void Selector::SelectCircle(std::vector<Mesh>& mesh, glm::mat4 world2screen, float x, float y,
                                float radius, bool invert) {
        mode = Mode::CIRCLE;
        pointX = (int) x;
        pointY = (int) y;
        circleRadius = radius;
        rangeColor = invert ? DESELECT_COLOR : 0;
        for (Mesh &m : mesh) {
            currentMesh = &m;
            AddVertices(m.vertices, world2screen, false);
        }
    }

    void Selector::SelectRect(std::vector<Mesh> &mesh, glm::mat4 world2screen, float x1, float y1,
                              float x2, float y2, bool invert) {
        mode = Mode::RECT;
        pointX = (int) x1;
        pointY = (int) y1;
        pointX2 = (int) x2;
        pointY2 = (int) y2;
        rangeColor = invert ? DESELECT_COLOR : 0;
        for (Mesh &m : mesh) {
            currentMesh = &m;
            AddVertices(m.vertices, world2screen, false);
        }
    }

    void Selector::SelectTriangle(std::vector<Mesh> &mesh, glm::mat4 world2screen, float x, float y) {
        mode = Mode::NORMAL;
        depth = INT_MAX;
        pointX = (int) x;
        pointY = (int) y;
        int selectModel = -1;
        int selectFace = -1;
        for (unsigned int i = 0; i < mesh.size(); i++) {
            selected = -1;
            AddVertices(mesh[i].vertices, world2screen, true);
            if (selected >= 0) {
                selectModel = i;
                selectFace = selected;
            }
        }
        if (selectModel >= 0) {
            mesh[selectModel].colors[selectFace + 0] = 0;
            mesh[selectModel].colors[selectFace + 1] = 0;
            mesh[selectModel].colors[selectFace + 2] = 0;
        }
    }

    glm::vec3 Selector::Transform(std::vector<Mesh> &mesh, glm::mat4 world2screen, float x, float y, bool culling) {
        mode = Mode::NORMAL;
        depth = INT_MAX;
        hit = false;
        pointX = (int) x;
        pointY = (int) (viewport_height - y);
        int selectFace = -1;
        int selectModel = -1;
        for (unsigned int i = 0; i < mesh.size(); i++) {
            selected = -1;
            AddVertices(mesh[i].vertices, world2screen, culling);
            if (selected >= 0) {
                hit = true;
                selectFace = selected;
                selectModel = i;
            }
        }
        if (hit) {
            glm::vec3 a = mesh[selectModel].vertices[selectFace + 0];
            glm::vec3 b = mesh[selectModel].vertices[selectFace + 1];
            glm::vec3 c = mesh[selectModel].vertices[selectFace + 2];
            float vx = 2.0f * x / (float)viewport_width - 1.0f;
            float vy =-2.0f * y / (float)viewport_height + 1.0f;

            //project triangle on the screen
            glm::vec4 a2 = world2screen * glm::vec4(a, 1);
            glm::vec4 b2 = world2screen * glm::vec4(b, 1);
            glm::vec4 c2 = world2screen * glm::vec4(c, 1);
            a2 /= fabs(a2.w);
            b2 /= fabs(b2.w);
            c2 /= fabs(c2.w);

            //barycentric interpolation
            float wa = 1.0f / (glm::length(glm::vec2(a2.x - vx, a2.y - vy)));
            float wb = 1.0f / (glm::length(glm::vec2(b2.x - vx, b2.y - vy)));
            float wc = 1.0f / (glm::length(glm::vec2(c2.x - vx, c2.y - vy)));
            return (wa * a + wb * b + wc * c) / (wa + wb + wc);
        }
        return glm::vec3(99999);
    }

    std::vector<glm::vec3> Selector::Transform(std::vector<Mesh>& mesh, glm::mat4 world2screen, std::vector<glm::vec2>& points, bool culling) {
        depths.clear();
        hits.clear();
        points2D.clear();
        std::pair<int, int> invalid(-1, -1);
        for (glm::vec2& p : points) {
            points2D.emplace_back(glm::ivec2(p.x, viewport_height - p.y));
            hits.push_back(invalid);
            depths.push_back(INT_MAX);
        }

        mode = Mode::ARRAY;
        for (unsigned int i = 0; i < mesh.size(); i++) {
            selected = i;
            AddVertices(mesh[i].vertices, world2screen, culling);
        }

        std::vector<glm::vec3> output;
        for (unsigned int i = 0; i < points.size(); i++) {
            std::pair<int, int> h = hits[i];
            if (h.first >= 0) {
                glm::vec3 a = mesh[h.first].vertices[h.second + 0];
                glm::vec3 b = mesh[h.first].vertices[h.second + 1];
                glm::vec3 c = mesh[h.first].vertices[h.second + 2];
                float vx = 2.0f * points[i].x / (float)viewport_width - 1.0f;
                float vy =-2.0f * points[i].y / (float)viewport_height + 1.0f;

                //project triangle on the screen
                glm::vec4 a2 = world2screen * glm::vec4(a, 1);
                glm::vec4 b2 = world2screen * glm::vec4(b, 1);
                glm::vec4 c2 = world2screen * glm::vec4(c, 1);
                a2 /= fabs(a2.w);
                b2 /= fabs(b2.w);
                c2 /= fabs(c2.w);

                //barycentric interpolation
                float wa = 1.0f / (glm::length(glm::vec2(a2.x - vx, a2.y - vy)));
                float wb = 1.0f / (glm::length(glm::vec2(b2.x - vx, b2.y - vy)));
                float wc = 1.0f / (glm::length(glm::vec2(c2.x - vx, c2.y - vy)));
                output.push_back((wa * a + wb * b + wc * c) / (wa + wb + wc));
            } else {
                output.emplace_back(99999);
            }
        }
        return output;
    }

    void Selector::CreateGraph(std::vector<Mesh>& mesh) {
        std::string a, b, c;
        std::pair<int, int> p;
        if (connections.empty()) {
            for (unsigned int m = 0; m < mesh.size(); m++) {
                for (unsigned int i = 0; i < mesh[m].vertices.size(); i += 3) {
                    a = VertexToKey(mesh[m].vertices[i + 0]);
                    b = VertexToKey(mesh[m].vertices[i + 1]);
                    c = VertexToKey(mesh[m].vertices[i + 2]);
                    p = std::pair<int, int>(m, i);
                    if (connections.find(a) == connections.end())
                        connections[a] = std::map<std::pair<int, int>, bool>();
                    if (connections.find(b) == connections.end())
                        connections[b] = std::map<std::pair<int, int>, bool>();
                    if (connections.find(c) == connections.end())
                        connections[c] = std::map<std::pair<int, int>, bool>();
                    connections[a][p] = true;
                    connections[b][p] = true;
                    connections[c][p] = true;
                }
            }
        }
    }
}