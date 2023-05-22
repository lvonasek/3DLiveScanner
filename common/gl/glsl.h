#ifndef GL_GLSL_H
#define GL_GLSL_H

#include <string>
#include <vector>
#include "data/image.h"
#include "gl/opengl.h"

namespace oc {
    class GLSL {
    public:
        /**
         * @brief Constructor
         * @param vert is vertex shader code
         * @param frag is fragment shader code
         * @param extension is additional GLSL extension request
         */
        GLSL(std::string vert, std::string frag, std::string extension = "");

        ~GLSL();

        /**
         * @brief it sends geometry into GPU
         * @param vertices is vertices
         * @param normals is normals
         * @param coords is texture coords
         * @param colors is vertex colors
         */
        void Attrib(float* vertices, float* normals, float* coords, unsigned int* colors);

        /**
         * @brief it binds shader
         */
        void Bind();

        static GLSL* CurrentShader();

        /**
         * @brief gets last compile error
         * @return error as string or empty string
         */
        static std::string GetError();

        unsigned int GetId() { return id; }

        /**
         * @brief initShader creates shader from code
         * @param vs is vertex shader code
         * @param fs is fragment shader code
         * @return shader program id
         */
        unsigned int InitShader(const char *vs, const char *fs);

        /**
         * @brief it unbinds shader
         */
        void Unbind();

        /**
         * @brief UniformFloat send float into shader
         * @param name is uniform name
         * @param value is uniform value
         */
        void UniformFloat(const char* name, float value);

        /**
         * @brief UniformInt send int into shader
         * @param name is uniform name
         * @param value is uniform value
         */
        void UniformInt(const char* name, int value);

        /**
         * @brief UniformMatrix send matrix into shader
         * @param name is uniform name
         * @param value is uniform value
         */
        void UniformMatrix(const char* name, const float* value);

        /**
         * @brief UniformTexture send int into shader
         * @param name is uniform name
         * @param value is uniform value
         */
        void UniformTexture(const char* name, int value);

        /**
         * @brief UniformVec3 send vec3 into shader
         * @param name is uniform name
         * @param x is uniform value in x coordinate
         * @param y is uniform value in y coordinate
         * @param z is uniform value in z coordinate
         */
        void UniformVec3(const char* name, float x, float y, float z);

        static GLuint Image2GLTexture(Image* img);

    private:
        unsigned int id;          ///< Shader id
        unsigned int shader_vp;   ///< Vertex shader
        unsigned int shader_fp;   ///< Fragment shader
        int attribute_v_vertex;   ///< Pointer to vertices on GPU
        int attribute_v_coord;    ///< Pointer to coords on GPU
        int attribute_v_normal;   ///< Pointer to normals on GPU
        int attribute_v_color;    ///< Pointer to colors on GPU
    };
}
#endif
