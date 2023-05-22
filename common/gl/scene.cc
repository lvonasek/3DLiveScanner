#include "gl/scene.h"

#define MAX_VERTICES 50000

bool comparator(const RenderNode& a, const RenderNode& b) {
    return a.distance < b.distance;
}

oc::GLScene::GLScene() : size(0) {
    for (int i = 0; i < 8; i++)
        child[i] = 0;
}

oc::GLScene::~GLScene() {
    for (int i = 0; i < 8; i++)
        if (child[i])
            delete child[i];
}

void oc::GLScene::Clear() {
    models.clear();
}

void oc::GLScene::Load(std::vector<oc::Mesh>& input) {
    for (Mesh& m : input) {
        for (glm::vec3& v : m.vertices) {
            size = glm::max(glm::abs(v.x), size);
            size = glm::max(glm::abs(v.y), size);
            size = glm::max(glm::abs(v.z), size);
        }
        models.push_back(m);
    }
    input.clear();
}

void oc::GLScene::Process() {
    //set AABB
    aabb[0] = -glm::vec3(size, size, size);
    aabb[1] = glm::vec3(size, size, size);

    //build the tree
    ProcessRecursive();
    PatchAABB();
}

void oc::GLScene::Render(GLint position_param, GLint uv_param) {
#ifdef GL_ES_VERSION_3_0
    GLuint gpuMeasuring[1];
    glGenQueries(1, gpuMeasuring);
#endif
    glEnableVertexAttribArray((GLuint) position_param);
    glEnableVertexAttribArray((GLuint) uv_param);
    std::vector<RenderNode> toRender = GetRenderable();
    std::sort(toRender.begin(), toRender.end(), comparator);
    for (RenderNode& n : toRender) {


#ifdef GL_ES_VERSION_3_0
        //occlusion
        if (n.distance > 0) {
            glDepthMask(GL_FALSE);
            glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_TRUE);
            glBindTexture(GL_TEXTURE_2D, 0);
            glBeginQuery(GL_ANY_SAMPLES_PASSED_CONSERVATIVE, gpuMeasuring[0]);
            glVertexAttribPointer((GLuint) position_param, 3, GL_FLOAT, GL_FALSE, 0, n.aabb.data());
            glVertexAttribPointer((GLuint) uv_param, 2, GL_FLOAT, GL_FALSE, 0, n.aabb.data());
            glDrawArrays(GL_TRIANGLES, 0, (GLsizei) n.aabb.size());
            glEndQuery(GL_ANY_SAMPLES_PASSED_CONSERVATIVE);
            glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
            glDepthMask(GL_TRUE);
            GLuint passed = 0;
            glGetQueryObjectuiv(gpuMeasuring[0], GL_QUERY_RESULT, &passed);
            if (passed == 0)
                continue;
        }
#endif

        for (Mesh* m : n.meshes) {
            //upload texture if necessary
            if (m->image && (m->image->GetTexture() == -1)) {
                bool jpg = m->image->GetExtension().compare("jpg") == 0;
                if (jpg) m->image->UpsideDown();

                GLuint textureID;
                glGenTextures(1, &textureID);
                m->image->SetTexture(textureID);
                glBindTexture(GL_TEXTURE_2D, textureID);
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m->image->GetWidth(),
                             m->image->GetHeight(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
                             m->image->GetData());
                glGenerateMipmap(GL_TEXTURE_2D);

                if (jpg) m->image->UpsideDown();
            }
            //render model
            glBindTexture(GL_TEXTURE_2D, (unsigned int)m->image->GetTexture());
            glVertexAttribPointer((GLuint) position_param, 3, GL_FLOAT, GL_FALSE, 0, m->vertices.data());
            glVertexAttribPointer((GLuint) uv_param, 2, GL_FLOAT, GL_FALSE, 0, m->uv.data());
            glDrawArrays(GL_TRIANGLES, 0, (GLsizei) m->vertices.size());
        }
    }
    glDisableVertexAttribArray((GLuint) position_param);
    glDisableVertexAttribArray((GLuint) uv_param);
#ifdef GL_ES_VERSION_3_0
    glDeleteQueries(1, gpuMeasuring);
#endif
}

void oc::GLScene::UpdateVisibility(glm::mat4 mvp, glm::vec4 translate) {
    camera = -translate;

    int bTest = BasicTest(mvp, translate);
    if (bTest == 0)
        SetVisibility(false, true);
    else if (bTest >= 1) {
        SetVisibility(false, false);
        bool ok = false;
        for (int i = 0; i < 8; i++) {
            if (child[i]) {
                child[i]->UpdateVisibility(mvp, translate);
                ok = true;
            }
        }
        if (!ok)
            SetVisibility(true, false);
    }
    else if (bTest == 2)
        SetVisibility(true, true);
    else
        assert(false);
}

/**
 * @brief basicTest tests AABB with frustum planes
 * @param mvp is used model-view-projection matrix
 * @param translate is translation of view
 * @return 0 if it is outside, 1 if it is partitionaly inside, 2 if it is inside
 */
int oc::GLScene::BasicTest(glm::mat4& mvp, glm::vec4& translate) {
    /// Basic Intersection Test
    glm::vec2 min = glm::vec2(1, 1);
    glm::vec2 max = -min;
    int x1 = 0; int x2 = 0; int y1 = 0; int y2 = 0; int z1 = 0; int z2 = 0;
    int x1e = 0; int x2e = 0; int y1e = 0; int y2e = 0; int z1e = 0; int z2e = 0;
    for (int i = 0; i < 8; i++) {
        /// check if it is inside
        glm::vec4 vec = GetTransform(i);
        vec.x += translate.x;
        vec.y += translate.y;
        vec.z += translate.z;
        vec = mvp * vec;
        vec.x /= vec.w;
        vec.y /= vec.w;
        vec.z /= glm::abs(vec.w);
        if (vec.x <= 1) { x1++; } else { x1e++; }
        if (vec.x >= -1) { x2++; } else { x2e++; }
        if (vec.y <= 1) { y1++; } else { y1e++; }
        if (vec.y >= -1) { y2++; } else { y2e++; }
        if (vec.z <= 1) { z1++; } else { z1e++; }
        if (vec.z >= 0) { z2++; } else { z2e++; }
        if (min.x > vec.x) min.x = vec.x;
        if (min.y > vec.y) min.y = vec.y;
        if (max.x < vec.x) max.x = vec.x;
        if (max.y < vec.y) max.y = vec.y;
    }

    /// check if AABB is bigger than frustum
    if ((x1e > 0) && (x2e > 0)) {
        x1 = 8;
        x2 = 8;
    }
    if ((y1e > 0) && (y2e > 0)) {
        y1 = 8;
        y2 = 8;
    }
    if ((z1e > 0) && (z2e > 0)) {
        z1 = 8;
        z2 = 8;
    }

    /// check if AABB is big enough
    if (glm::length(max - min) < 0.03f)
      return 0;

    /// return value
    if (x1 + y1 + x2 + y2 + z1 + z2 == 48) {
        return 2;
    } else if ((x1 > 0) & (x2 > 0) & (y1 > 0) & (y2 > 0) & (z1 > 0) & (z2 > 0)) {
        return 1;
    } else {
        return 0;
    }
}

/*!
 * \brief Counts distance from point to AABB of this node
 * \param p is point to use
 * \return distance as float
 */
float oc::GLScene::DistanceToAABB(glm::vec3 p) {
    glm::vec3 c = (aabb[0] + aabb[1]) * 0.5f;
    float size2d = glm::sqrt(size * size + size * size);
    float size3d = glm::sqrt(size2d * size2d + size * size);
    return glm::length(p - c) - size3d;
}

/*!
 * \brief Gets all meshes which should be rendered
 * \return renderable meshes as vector
 */
std::vector<RenderNode> oc::GLScene::GetRenderable() {
    std::vector<RenderNode> output;
    if (!models.empty()) {
        RenderNode node;

        //left/right occlusion
        for (int i = 0; i <= 1; i++) {
            node.aabb.push_back(glm::vec3(aabb[0].x, aabb[0].y, aabb[i].z));
            node.aabb.push_back(glm::vec3(aabb[1].x, aabb[0].y, aabb[i].z));
            node.aabb.push_back(glm::vec3(aabb[0].x, aabb[1].y, aabb[i].z));
            node.aabb.push_back(glm::vec3(aabb[1].x, aabb[1].y, aabb[i].z));
            node.aabb.push_back(glm::vec3(aabb[0].x, aabb[1].y, aabb[i].z));
            node.aabb.push_back(glm::vec3(aabb[1].x, aabb[0].y, aabb[i].z));
        }
        //top/down occlusion
        for (int i = 0; i <= 1; i++) {
            node.aabb.push_back(glm::vec3(aabb[0].x, aabb[i].y, aabb[0].z));
            node.aabb.push_back(glm::vec3(aabb[1].x, aabb[i].y, aabb[0].z));
            node.aabb.push_back(glm::vec3(aabb[0].x, aabb[i].y, aabb[1].z));
            node.aabb.push_back(glm::vec3(aabb[1].x, aabb[i].y, aabb[1].z));
            node.aabb.push_back(glm::vec3(aabb[0].x, aabb[i].y, aabb[1].z));
            node.aabb.push_back(glm::vec3(aabb[1].x, aabb[i].y, aabb[0].z));
        }
        //front/back occlusion
        for (int i = 0; i <= 1; i++) {
            node.aabb.push_back(glm::vec3(aabb[i].x, aabb[0].y, aabb[0].z));
            node.aabb.push_back(glm::vec3(aabb[i].x, aabb[1].y, aabb[0].z));
            node.aabb.push_back(glm::vec3(aabb[i].x, aabb[0].y, aabb[1].z));
            node.aabb.push_back(glm::vec3(aabb[i].x, aabb[1].y, aabb[1].z));
            node.aabb.push_back(glm::vec3(aabb[i].x, aabb[0].y, aabb[1].z));
            node.aabb.push_back(glm::vec3(aabb[i].x, aabb[1].y, aabb[0].z));
        }

        //process current node
        for (Mesh& m : models)
            if (m.visible && !m.vertices.empty())
                node.meshes.push_back(&m);
        if (!node.meshes.empty()) {
            node.distance = DistanceToAABB(glm::vec3(camera.x, camera.y, camera.z));
            output.push_back(node);
        }
    }

    //process childs
    for (int i = 0; i < 8; i++)
        if (child[i])
            for (RenderNode rn : child[i]->GetRenderable())
                output.push_back(rn);
    return output;
}

/**
 * @brief getTransform gets position of AABB vertex in frustum space
 * @param i is index of vertex
 * @return transformed vertex in frustum space
 */
glm::vec4 oc::GLScene::GetTransform(int i) {
    switch(i) {
        case(0):
            return glm::vec4(aabb[0].x, aabb[0].y, aabb[0].z, 1);
        case(1):
            return glm::vec4(aabb[0].x, aabb[0].y, aabb[1].z, 1);
        case(2):
            return glm::vec4(aabb[0].x, aabb[1].y, aabb[0].z, 1);
        case(3):
            return glm::vec4(aabb[0].x, aabb[1].y, aabb[1].z, 1);
        case(4):
            return glm::vec4(aabb[1].x, aabb[0].y, aabb[0].z, 1);
        case(5):
            return glm::vec4(aabb[1].x, aabb[0].y, aabb[1].z, 1);
        case(6):
            return glm::vec4(aabb[1].x, aabb[1].y, aabb[0].z, 1);
        case(7):
            return glm::vec4(aabb[1].x, aabb[1].y, aabb[1].z, 1);
        default:
            return glm::vec4(0, 0, 0, 1);
    }
}

/*!
 * \brief Patches recursively AABB
 */
void oc::GLScene::PatchAABB() {
    //patch AABB
    for (Mesh& m : models)
        for (glm::vec3& v : m.vertices) {
            if (aabb[0].x > v.x)
                aabb[0].x = v.x;
            if (aabb[1].x < v.x)
                aabb[1].x = v.x;

            if (aabb[0].y > v.y)
                aabb[0].y = v.y;
            if (aabb[1].y < v.y)
                aabb[1].y = v.y;

            if (aabb[0].z > v.z)
                aabb[0].z = v.z;
            if (aabb[1].z < v.z)
                aabb[1].z = v.z;
        }

    //process children
    for (int i = 0; i < 8; i++)
        if (child[i])
            child[i]->PatchAABB();
}

/*!
 * \brief Creates octree data structure
 */
void oc::GLScene::ProcessRecursive() {

    //count vertices in this node
    unsigned long count = 0;
    for (Mesh& m : models)
        count += m.vertices.size();

    //subdivide if there is too many vertices
    if (count > MAX_VERTICES) {

        //create nodes
        LOGI("Subdividing octree node with size %f and %ldK vertices", size, count / 1000);
        for (int i = 0; i < 8; i++) {
            child[i] = new GLScene();
            child[i]->aabb[0] = aabb[0];
            child[i]->aabb[1] = aabb[1];
            child[i]->size = size;
            child[i]->UpdateAABB((bool) (i / 4), (bool) ((i / 2) % 2), (bool) (i % 2));
            for (unsigned int j = 0; j < models.size(); j++) {
                Mesh m;
                m.image = models[j].image;
                m.imageOwner = false;
                child[i]->models.push_back(m);
            }
        }

        //subdivide
        glm::vec3 a, b, c, v;
        for (unsigned int i = 0; i < models.size(); i++) {
            for (unsigned long j = 0; j < models[i].vertices.size(); j += 3) {
                a = models[i].vertices[j + 0];
                b = models[i].vertices[j + 1];
                c = models[i].vertices[j + 2];
                v = (a + b + c) / 3.0f;

                for (int k = 0; k < 8; k++) {
                    bool ok = true;
                    if ((v.x < child[k]->aabb[0].x) || (v.x > child[k]->aabb[1].x))
                        ok = false;
                    if ((v.y < child[k]->aabb[0].y) || (v.y > child[k]->aabb[1].y))
                        ok = false;
                    if ((v.z < child[k]->aabb[0].z) || (v.z > child[k]->aabb[1].z))
                        ok = false;

                    if (ok) {
                        child[k]->models[i].vertices.push_back(a);
                        child[k]->models[i].vertices.push_back(b);
                        child[k]->models[i].vertices.push_back(c);
                        child[k]->models[i].uv.push_back(models[i].uv[j + 0]);
                        child[k]->models[i].uv.push_back(models[i].uv[j + 1]);
                        child[k]->models[i].uv.push_back(models[i].uv[j + 2]);
                        break;
                    }
                }
            }
        }
        models.clear();

        //clean empty and process childs
        for (int i = 0; i < 8; i++) {
            count = 0;
            for (long j = child[i]->models.size() - 1; j >= 0; j--) {
                count += child[i]->models[j].vertices.size();
                if (child[i]->models[j].vertices.empty())
                    child[i]->models.erase(child[i]->models.begin() + j);
            }
            if (count == 0) {
                delete child[i];
                child[i] = 0;
            } else {
                child[i]->ProcessRecursive();
            }
        }
    }
}

/*!
 * \brief sets visibility of models
 * \param on is true to make it visible
 * \param recursive is true to process child nodes
 */
void oc::GLScene::SetVisibility(bool on, bool recursive) {
    for (Mesh& m : models)
        m.visible = on;
    if (recursive)
        for (int i = 0; i < 8; i++)
            if (child[i])
                child[i]->SetVisibility(on, true);
}


/**
 * @brief Updates AABB of child
 * @param x is position coordinate
 * @param y is position coordinate
 * @param z is position coordinate
 * @return instance of region structure
 */
void oc::GLScene::UpdateAABB(bool x, bool y, bool z) {
    if (x) {
        aabb[0].x += size;
    } else {
        aabb[1].x -= size;
    }
    if (y) {
        aabb[0].y += size;
    } else {
        aabb[1].y -= size;
    }
    if (z) {
        aabb[0].z += size;
    } else {
        aabb[1].z -= size;
    }
    size *= 0.5f;
}
