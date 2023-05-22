#include "data/file3d.h"

namespace oc {

    File3d::File3d(std::string filename, bool writeAccess) {
        path = filename;
        writeMode = writeAccess;
        vertexCount = 0;

        if (writeMode) {
            LOGI("Writing into %s", filename.c_str());
        } else {
            LOGI("Loading from %s", filename.c_str());
        }

        std::string ext = filename.substr(filename.size() - 3, filename.size() - 1);
        if (ext.compare("pcl") == 0)
            type = PCL;
        else if (ext.compare("ply") == 0)
            type = PLY;
        else if (ext.compare("obj") == 0)
            type = OBJ;
        else
            assert(false);

        if (writeMode)
            file = fopen(filename.c_str(), "w");
        else
            file = fopen(filename.c_str(), "r");
    }

    File3d::~File3d() {
        fclose(file);
    }

    void File3d::ReadModel(int subdivision, std::vector<Mesh>& output) {
        assert(!writeMode);
        ReadHeader();
        if (type == PCL)
            ParsePCL(subdivision, output);
        else if (type == PLY)
            ParsePLY(subdivision, output);
        else if (type == OBJ)
            ParseOBJ(subdivision, output);
        else
            assert(false);
    }

    void File3d::WriteModel(std::vector<Mesh>& model, bool extra) {
        assert(writeMode);
        //count vertices and faces
        vertexCount = 0;
        std::vector<unsigned long> vectorSize;
        for(unsigned int i = 0; i < model.size(); i++) {
            unsigned long max = model[i].vertices.size() + 1;
            vertexCount += model[i].vertices.size();
            vectorSize.push_back(max);
        }
        //write
        faceCount = extra ? vertexCount / 3 : 0;
        if ((type == PLY) || (type == OBJ)) {
            WriteHeader(model);
            for (unsigned int i = 0; i < model.size(); i++)
                WritePointCloud(model[i]);
            if ((type == OBJ) || extra) {
                int offset = (type == OBJ) ? 1 : 0;
                for (unsigned int i = 0; i < model.size(); i++) {
                    if (model[i].vertices.empty())
                        continue;
                    if (type == OBJ) {
                        if (model[i].image)
                            fprintf(file, "usemtl %d\n", fileToIndex[model[i].image->GetName()]);
                        else
                            fprintf(file, "g %d\n", i);
                    }
                    WriteFaces(model[i], offset);
                    offset += model[i].vertices.size();
                }
            }
        } else
            assert(false);
    }

    void File3d::CleanStr(std::string& str) {
        while(!str.empty()) {
            char c = str[str.size() - 1];
            if ((c == '\r') || (c == '\n') || isspace(c))
                str = str.substr(0, str.size() - 1);
            else
                break;
        }
    }

    unsigned int File3d::CodeColor(glm::ivec3 c) {
        unsigned int output = 0;
        output += glm::clamp(c.r, 0, 255);
        output += glm::clamp(c.g, 0, 255) << 8;
        output += glm::clamp(c.b, 0, 255) << 16;
        return output;
    }

    glm::ivec3 File3d::DecodeColor(unsigned int c) {
        glm::ivec3 output;
        output.r = (c & 0x000000FF);
        output.g = (c & 0x0000FF00) >> 8;
        output.b = (c & 0x00FF0000) >> 16;
        return output;
    }

    void File3d::ParseOBJ(int subdivision, std::vector<Mesh> &output) {
        char buffer[1024];
        unsigned long meshIndex = 0;
        glm::vec3 v;
        glm::vec3 n;
        glm::vec2 t;
        std::string lastKey;
        std::deque<glm::vec3> vertices;
        std::deque<glm::vec3> normals;
        std::deque<glm::vec2> uvs;
        bool hasNormals = false;
        bool hasCoords = false;
        bool hasNumber = false;;
        unsigned int va, vna, vta, vb, vnb, vtb, vc, vnc, vtc, vd, vnd, vtd, count;
        std::map<std::string, Image*> images;

        //dummy material
        std::string key;
        meshIndex = output.size();
        output.push_back(Mesh());
        images[key] = new Image(255, 255, 255, 255);
        output[meshIndex].imageOwner = true;
        output[meshIndex].image = images[key];
        lastKey = key;

        //parse
        while (true) {
            if (!fgets(buffer, 1024, file))
                break;
            std::string sbuf = buffer;
            while(!sbuf.empty() && isspace(sbuf[0])) {
                sbuf = sbuf.substr(1);
            }
            if (sbuf[0] == 'u') {
                key = sbuf.substr(7);
                CleanStr(key);
                if (lastKey.empty() || (lastKey.compare(key) != 0)) {
                    meshIndex = output.size();
                    output.push_back(Mesh());
                    if (images.find(key) == images.end()) {
                        std::string imagefile = keyToFile[key];
                        if (imagefile.empty())
                        {
                            glm::vec3 color = keyToColor[key];
                            unsigned char r = (unsigned char) (255 * color.r);
                            unsigned char g = (unsigned char) (255 * color.g);
                            unsigned char b = (unsigned char) (255 * color.b);
                            images[key] = new Image(r, g, b, 255);
                        }
                        else
                          images[key] = new Image(imagefile);
                        output[meshIndex].imageOwner = true;
                    } else {
                        output[meshIndex].imageOwner = false;
                        images[key]->AddInstance();
                    }
                    output[meshIndex].image = images[key];
                    lastKey = key;
                }
            } else if ((sbuf[0] == 'v') && (sbuf[1] == ' ')) {
                sscanf(sbuf.c_str(), "v %f %f %f", &v.x, &v.y, &v.z);
                vertices.push_back(v);
            } else if ((sbuf[0] == 'v') && (sbuf[1] == 't')) {
                sscanf(sbuf.c_str(), "vt %f %f", &t.x, &t.y);
                uvs.push_back(t);
                hasCoords = true;
            } else if ((sbuf[0] == 'v') && (sbuf[1] == 'n')) {
                sscanf(sbuf.c_str(), "vn %f %f %f", &n.x, &n.y, &n.z);
                normals.push_back(n);
                hasNormals = true;
            } else if ((sbuf[0] == 'f') && (sbuf[1] == ' ')) {
                va = 0;
                vb = 0;
                vc = 0;
                vd = 0;
                count = 0;
                hasNumber = false;
                for (unsigned int i = 1; i < sbuf.size(); i++) {
                    if ((sbuf[i] >= '0') && (sbuf[i] <= '9')) {
                        if (!hasNumber) {
                            hasNumber = true;
                            count++;
                        }
                    } else if (sbuf[i] == ' ') {
                        hasNumber = false;
                    } else if (sbuf[i] == '#') {
                        break;
                    }
                }

                //process triangle
                if (count == 3) {
                    if (!hasCoords && !hasNormals)
                        sscanf(sbuf.c_str(), "f %d %d %d", &va, &vb, &vc);
                    else if (hasCoords && !hasNormals)
                        sscanf(sbuf.c_str(), "f %d/%d %d/%d %d/%d", &va, &vta, &vb, &vtb, &vc, &vtc);
                    else if (hasCoords && hasNormals)
                        sscanf(sbuf.c_str(), "f %d/%d/%d %d/%d/%d %d/%d/%d",
                               &va, &vta, &vna, &vb, &vtb, &vnb, &vc, &vtc, &vnc);
                    else if (!hasCoords && hasNormals)
                        sscanf(buffer, "f %d//%d %d//%d %d//%d", &va, &vna, &vb, &vnb, &vc, &vnc);
                    //broken topology ignored
                    if ((va == vb) || (va == vc) || (vb == vc))
                        continue;
                    //incomplete line ignored
                    if ((va == 0) || (vb == 0) || (vc == 0))
                        continue;
                }

                //process quad
                if (count == 4) {
                    if (!hasCoords && !hasNormals)
                        sscanf(sbuf.c_str(), "f %d %d %d %d", &va, &vb, &vc, &vd);
                    else if (hasCoords && !hasNormals)
                        sscanf(sbuf.c_str(), "f %d/%d %d/%d %d/%d %d/%d", &va, &vta, &vb, &vtb, &vc, &vtc, &vd, &vtd);
                    else if (hasCoords && hasNormals)
                        sscanf(sbuf.c_str(), "f %d/%d/%d %d/%d/%d %d/%d/%d %d/%d/%d",
                               &va, &vta, &vna, &vb, &vtb, &vnb, &vc, &vtc, &vnc, &vd, &vtd, &vnd);
                    else if (!hasCoords && hasNormals)
                        sscanf(buffer, "f %d//%d %d//%d %d//%d %d//%d", &va, &vna, &vb, &vnb, &vc, &vnc, &vd, &vnd);
                    //broken topology ignored
                    if ((va == vb) || (va == vc) || (vb == vc))
                        continue;
                    //incomplete line ignored
                    if ((va == 0) || (vb == 0) || (vc == 0) || (vd == 0))
                        continue;
                }

                //vertices
                output[meshIndex].vertices.push_back(vertices[va - 1]);
                output[meshIndex].vertices.push_back(vertices[vb - 1]);
                output[meshIndex].vertices.push_back(vertices[vc - 1]);
                if (count == 4) {
                    output[meshIndex].vertices.push_back(vertices[vd - 1]);
                    output[meshIndex].vertices.push_back(vertices[va - 1]);
                    output[meshIndex].vertices.push_back(vertices[vc - 1]);
                }
                //selector
                for (int i = 0; i < (count == 3 ? 3 : 6); i++)
                    output[meshIndex].colors.push_back(0);
                //uvs
                if (hasCoords) {
                    output[meshIndex].uv.push_back(uvs[vta - 1]);
                    output[meshIndex].uv.push_back(uvs[vtb - 1]);
                    output[meshIndex].uv.push_back(uvs[vtc - 1]);
                    if (count == 4) {
                        output[meshIndex].uv.push_back(uvs[vtd - 1]);
                        output[meshIndex].uv.push_back(uvs[vta - 1]);
                        output[meshIndex].uv.push_back(uvs[vtc - 1]);
                    }
                } else {
                    for (int i = 0; i < (count == 3 ? 3 : 6); i++)
                        output[meshIndex].uv.push_back(glm::vec2(0, 0));
                }
                //normals
                if (hasNormals) {
                    output[meshIndex].normals.push_back(normals[vna - 1]);
                    output[meshIndex].normals.push_back(normals[vnb - 1]);
                    output[meshIndex].normals.push_back(normals[vnc - 1]);
                    if (count == 4) {
                        output[meshIndex].normals.push_back(normals[vnd - 1]);
                        output[meshIndex].normals.push_back(normals[vna - 1]);
                        output[meshIndex].normals.push_back(normals[vnc - 1]);
                    }
                } else {
                    for (int i = 0; i < (count == 3 ? 3 : 6); i++)
                        output[meshIndex].normals.push_back(glm::vec3(0, 0, 0));
                }

                //create new model if it is already too big
                if (output[meshIndex].vertices.size() >= subdivision * 3) {
                    meshIndex = output.size();
                    output.push_back(Mesh());
                    output[meshIndex].image = images[lastKey];
                    output[meshIndex].image->AddInstance();
                    output[meshIndex].imageOwner = false;
                }
            }
        }
    }

    void File3d::ParsePCL(int subdivision, std::vector<Mesh> &output) {
        assert(!writeMode);
        glm::vec3 a;
        float w;
        //load vertices
        Mesh m;
        for (unsigned int i = 0; i < vertexCount; i++) {
            fscanf(file, "%f %f %f %f\n", &a.x, &a.y, &a.z, &w);
            m.vertices.push_back(a);
            m.colors.push_back(256 * (int)(w * 255));
        }
        output.push_back(m);
    }

    void File3d::ParsePLY(int subdivision, std::vector<Mesh> &output) {
        assert(!writeMode);
        glm::vec3 a, b, c, n;
        glm::vec2 t;
        int i, d, e, f;
        char buffer[1024];
        std::string key;
        std::map<std::string, glm::vec3> vertexNormal;

        //load vertices
        std::vector<glm::ivec3> colors;
        std::vector<glm::vec3> vertices;
        for (i = 0; i < vertexCount; i++) {
            if (hasColors) {
                fscanf(file, "%f %f %f %d %d %d", &a.x, &a.y, &a.z, &d, &e, &f);
                colors.push_back(glm::ivec3(d, e, f));
            } else
                fscanf(file, "%f %f %f", &a.x, &a.y, &a.z);
            vertices.push_back(a);
        }

        //first part
        unsigned long meshIndex = output.size();
        output.push_back(Mesh());

        //special case - do not parse triangles, return just points
        if ((subdivision == -1) || (faceCount == 0)) {
            for (glm::vec3 v : vertices)
                output[meshIndex].vertices.push_back(v);
            for (glm::ivec3& color : colors)
                output[meshIndex].colors.push_back(CodeColor(color));
            return;
        }

        while(true) {
            if (feof(file))
                break;
            if (output[meshIndex].vertices.size() >= subdivision * 3) {
                meshIndex = output.size();
                output.push_back(Mesh());
                output[meshIndex].image = new Image(255, 0, 255, 255);
                output[meshIndex].imageOwner = true;
            }
            d = -1;
            e = -1;
            f = -1;
            fscanf(file, "%d %d %d %d", &i, &d, &e, &f);
            a = vertices[d];
            b = vertices[e];
            c = vertices[f];
            n = glm::vec3();
            t = glm::vec2();
            output[meshIndex].vertices.push_back(a);
            output[meshIndex].vertices.push_back(b);
            output[meshIndex].vertices.push_back(c);
            output[meshIndex].normals.push_back(n);
            output[meshIndex].normals.push_back(n);
            output[meshIndex].normals.push_back(n);
            output[meshIndex].colors.push_back(0);
            output[meshIndex].colors.push_back(0);
            output[meshIndex].colors.push_back(0);
            output[meshIndex].uv.push_back(t);
            output[meshIndex].uv.push_back(t);
            output[meshIndex].uv.push_back(t);
            //calculate vertex normals
            n = glm::normalize(glm::cross(a - b, a - c));
            sprintf(buffer, "%.3f,%.3f,%.3f", a.x, a.y, a.z);
            key = std::string(buffer);
            if (vertexNormal.find(key) == vertexNormal.end())
                vertexNormal[key] = n;
            else
                vertexNormal[key] += n;
            sprintf(buffer, "%.3f,%.3f,%.3f", b.x, b.y, b.z);
            key = std::string(buffer);
            if (vertexNormal.find(key) == vertexNormal.end())
                vertexNormal[key] = n;
            else
                vertexNormal[key] += n;
            sprintf(buffer, "%.3f,%.3f,%.3f", c.x, c.y, c.z);
            key = std::string(buffer);
            if (vertexNormal.find(key) == vertexNormal.end())
                vertexNormal[key] = n;
            else
                vertexNormal[key] += n;
        }
        //apply vertex normals
        for (meshIndex = 0; meshIndex < output.size(); meshIndex++)
        {
            for (i = 0; i < output[meshIndex].vertices.size(); i++)
            {
                a = output[meshIndex].vertices[i];
                sprintf(buffer, "%.3f,%.3f,%.3f", a.x, a.y, a.z);
                key = std::string(buffer);
                output[meshIndex].normals[i] = glm::normalize(vertexNormal[key]);
            }
        }
    }

    void File3d::ReadHeader() {
        char buffer[1024];
        if (type == PCL) {
            fscanf(file, "%d\n", &vertexCount);
        } else if (type == PLY) {
            faceCount = 0;
            hasColors = false;
            while (true) {
                if (!fgets(buffer, 1024, file))
                    break;
                if (StartsWith(buffer, "property uchar blue"))
                    hasColors = true;
                if (StartsWith(buffer, "element face"))
                    faceCount = ScanDec(buffer, 13);
                if (StartsWith(buffer, "element vertex"))
                    vertexCount = ScanDec(buffer, 15);
                else if (StartsWith(buffer, "end_header"))
                    break;
            }
        } else if (type == OBJ) {
            std::string mtlFile;
            while (true) {
                if (!fgets(buffer, 1024, file))
                    break;
                std::string sbuf = buffer;
                while(!sbuf.empty() && isspace(sbuf[0])) {
                    sbuf = sbuf.substr(1);
                }
                if (StartsWith(sbuf, "mtllib")) {
                    mtlFile = sbuf.substr(7);
                    CleanStr(mtlFile);
                    break;
                }
            }
            unsigned int index = 0;
            for (unsigned long i = 0; i < path.size(); i++) {
                if (path[i] == '/')
                    index = (unsigned int) i;
            }
            std::string key, imgFile;
            std::string data = path.substr(0, index + 1);
            std::string filepath = mtlFile;
            if (!mtlFile.empty())
            {
                if (mtlFile[0] != '/') {
                    filepath = data + mtlFile;
                }
                LOGI("Loading material %s", filepath.c_str());
                FILE* mtl = fopen(filepath.c_str(), "r");
                while (true) {
                    if (!fgets(buffer, 1024, mtl))
                        break;
                    std::string sbuf = buffer;
                    while(!sbuf.empty() && isspace(sbuf[0])) {
                        sbuf = sbuf.substr(1);
                    }
                    if (StartsWith(sbuf, "newmtl")) {
                        key = sbuf.substr(7);
                        CleanStr(key);
                    }
                    if (StartsWith(sbuf, "Kd")) {
                        glm::vec3 color;
                        sscanf(sbuf.c_str(), "Kd %f %f %f", &color.r, &color.g, &color.b);
                        keyToColor[key] = color;
                    }
                    if (StartsWith(sbuf, "map_Kd")) {
                        imgFile = sbuf.substr(7);
                        if (imgFile[0] != '/') {
                            imgFile = data + imgFile;
                        }
                        CleanStr(imgFile);
                        keyToFile[key] = imgFile;
                    }
              }
              fclose(mtl);
            } else {
              fclose(file);
              file = fopen(path.c_str(), "r");
            }
        } else
            assert(false);
    }

    unsigned int File3d::ScanDec(char *line, int offset) {
        unsigned int number = 0;
        for (int i = offset; i < 1024; i++) {
            char c = line[i];
            if (c != '\n')
                number = number * 10 + c - '0';
            else
                return number;
        }
        return number;
    }

    bool File3d::StartsWith(std::string s, std::string e) {
        if (s.size() >= e.size())
        if (s.substr(0, e.size()).compare(e) == 0)
            return true;
        return false;
    }

    void File3d::WriteHeader(std::vector<Mesh>& model) {
        bool hasNormals = false;
        bool hasColors = false;
        for (Mesh& mesh : model) {
            if (!mesh.normals.empty())
                hasNormals = true;
            if (!mesh.colors.empty())
                hasColors = true;
        }
        if (type == PLY) {
            fprintf(file, "ply\nformat ascii 1.0\ncomment ---\n");
            fprintf(file, "element vertex %d\n", vertexCount);
            fprintf(file, "property float x\n");
            fprintf(file, "property float y\n");
            fprintf(file, "property float z\n");
            if (hasNormals) {
                fprintf(file, "property float nx\n");
                fprintf(file, "property float ny\n");
                fprintf(file, "property float nz\n");
            }
            if (hasColors) {
                fprintf(file, "property uchar red\n");
                fprintf(file, "property uchar green\n");
                fprintf(file, "property uchar blue\n");
            }
            fprintf(file, "element face %d\n", faceCount);
            fprintf(file, "property list uchar uint vertex_indices\n");
            fprintf(file, "end_header\n");
        } else if (type == OBJ) {
            std::string base = (path.substr(0, path.length() - 4));
            unsigned long index = 0;
            for (unsigned long i = 0; i < base.size(); i++) {
                if (base[i] == '/')
                    index = i;
            }
            std::string shortBase = base.substr(index + 1, base.length());
            fprintf(file, "mtllib %s.mtl\n", shortBase.c_str());
            FILE* mtl = fopen((base + ".mtl").c_str(), "w");
            for (unsigned int i = 0; i < model.size(); i++) {
                if (model[i].vertices.empty())
                    continue;

                fprintf(mtl, "newmtl %d\n", i);
                fprintf(mtl, "Ns 96.078431\n");
                fprintf(mtl, "Ka 1.000000 1.000000 1.000000\n");
                fprintf(mtl, "Kd 0.640000 0.640000 0.640000\n");
                fprintf(mtl, "Ks 0.500000 0.500000 0.500000\n");
                fprintf(mtl, "Ke 0.000000 0.000000 0.000000\n");
                fprintf(mtl, "Ni 1.000000\n");
                fprintf(mtl, "d 1.000000\n");
                fprintf(mtl, "illum 2\n");
                //write texture name
                std::string name = model[i].image ? model[i].image->GetName() : "";
                fileToIndex[name] = i;


                if (!name.empty()) {
                    unsigned long  start = 0;
                    for (unsigned int j = 0; j < name.size(); j++)
                    {
                        char c = name[j];
                        if (c == '/')
                            start = j + 1;
                    }
                    fprintf(mtl, "map_Kd %s\n\n", name.substr(start, name.size()).c_str());
                }
            }
            fclose(mtl);
        }
    }


    void File3d::WritePointCloud(Mesh& mesh) {
        glm::vec3 v;
        glm::vec3 n;
        glm::vec2 t;
        glm::ivec3 c;
        bool hasNormals = !mesh.normals.empty();
        bool hasColors = !mesh.colors.empty();
        for(unsigned int j = 0; j < mesh.vertices.size(); j++) {
            v = mesh.vertices[j];
            if (type == PLY) {
                if (hasNormals) {
                    n = mesh.normals[j];
                    if (hasColors) {
                        c = DecodeColor(mesh.colors[j]);
                        fprintf(file, "%f %f %f %f %f %f %d %d %d\n", v.x, v.y, v.z, n.x, n.y, n.z, c.r, c.g, c.b);
                    } else {
                        fprintf(file, "%f %f %f %f %f %f\n", v.x, v.y, v.z, n.x, n.y, n.z);
                    }
                } else {
                    if (hasColors) {
                        c = DecodeColor(mesh.colors[j]);
                        fprintf(file, "%f %f %f %d %d %d\n", v.x, v.y, v.z, c.r, c.g, c.b);
                    } else {
                        fprintf(file, "%f %f %f\n", v.x, v.y, v.z);
                    }
                }
            } else if (type == OBJ) {
                fprintf(file, "v %f %f %f\n", v.x, v.y, v.z);
                if (!mesh.normals.empty())
                    fprintf(file, "vn %f %f %f\n", mesh.normals[j].x, mesh.normals[j].y, mesh.normals[j].z);
                if (!mesh.uv.empty())
                    fprintf(file, "vt %f %f\n", mesh.uv[j].x, mesh.uv[j].y);
            }
        }
    }


    void File3d::WriteFaces(Mesh& mesh, int offset) {
        glm::ivec3 i;
        bool hasCoords = !mesh.uv.empty();
        bool hasNormals = !mesh.normals.empty();
        if (mesh.indices.empty()) {
            for (unsigned int j = 0; j < mesh.vertices.size(); j+=3) {
                i.x = j + 0 + offset;
                i.y = j + 1 + offset;
                i.z = j + 2 + offset;
                if (type == PLY)
                    fprintf(file, "3 %d %d %d\n", i.x, i.y, i.z);
                else if (type == OBJ) {
                    if (!hasCoords && !hasNormals)
                        fprintf(file, "f %d %d %d\n", i.x, i.y, i.z);
                    else if (hasCoords && !hasNormals)
                        fprintf(file, "f %d/%d %d/%d %d/%d\n", i.x, i.x, i.y, i.y, i.z, i.z);
                    else if (hasCoords && hasNormals)
                        fprintf(file, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
                                i.x, i.x, i.x, i.y, i.y, i.y, i.z, i.z, i.z);
                    else if (!hasCoords && hasNormals)
                        fprintf(file, "f %d//%d %d//%d %d//%d\n", i.x, i.x, i.y, i.y, i.z, i.z);
                }
            }
        } else {
            for (unsigned int j = 0; j < mesh.indices.size(); j+=3) {
                i.x = mesh.indices[j + 0] + offset;
                i.y = mesh.indices[j + 1] + offset;
                i.z = mesh.indices[j + 2] + offset;
                if (type == PLY)
                    fprintf(file, "3 %d %d %d\n", i.x, i.y, i.z);
                else if (type == OBJ) {
                    if (!hasCoords && !hasNormals)
                        fprintf(file, "f %d %d %d\n", i.x, i.y, i.z);
                    else if (hasCoords && !hasNormals)
                        fprintf(file, "f %d/%d %d/%d %d/%d\n", i.x, i.x, i.y, i.y, i.z, i.z);
                    else if (hasCoords && hasNormals)
                        fprintf(file, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
                                i.x, i.x, i.x, i.y, i.y, i.y, i.z, i.z, i.z);
                    else if (!hasCoords && hasNormals)
                        fprintf(file, "f %d//%d %d//%d %d//%d\n", i.x, i.x, i.y, i.y, i.z, i.z);
                }
            }
        }
    }
}
