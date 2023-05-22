#ifndef GL_RENDERER_H
#define GL_RENDERER_H

#include <stack>
#include <vector>
#include "data/image.h"
#include "data/mesh.h"
#include "gl/camera.h"
#include "gl/glsl.h"
#include "gl/opengl.h"

namespace oc {
    class GLRenderer {

    public:
        /**
         * @brief constructor
         */
        GLRenderer();

        /**
         * @brief destructor
         */
        ~GLRenderer();

        /**
         * @brief Init inits renderer
         * @param w is screen width
         * @param h is screen height
         * @param rw is RTT width
         * @param rh is RTT height
         */
        void Init(int w, int h, int rw, int rh);

        /**
         * @brief Gets CPU access to texture
         * @return RGB Image
         */
        Image* ReadRtt();

        /**
         * @brief Gets CPU access to texture clipped
         * @param x is x offset
         * @param y is y offset
         * @param w is output width
         * @param h is output height
         * @return RGB Image
         */
        Image* ReadRtt(int x, int y, int w, int h);

        /**
         * @brief Render renders model into scene
         * @param vertices is pointer to vertices
         * @param normals is pointer to normals
         * @param uv is pointer to uv
         * @param colors is pointer to colors
         * @param size is of indices array for indexed geometry or size of vertex array for nonindiced
         * @param indices is pointer to indices
         * @param type is a type of the geometry
         */
        void Render(float* vertices, float* normals, float* uv, unsigned int* colors,
                    unsigned long size, unsigned int* indices = 0, int type = GL_TRIANGLES);

        /**
         * @brief Rtt enables rendering into FBO which makes posible to do reflections
         * @param enable is true to start drawing, false to render on screen
         */
        void Rtt(bool enable);

        GLCamera camera;                      ///< Camera object
        int width;                            ///< Screen width
        int height;                           ///< Screen height
        int rWidth;                           ///< RTT width
        int rHeight;                          ///< RTT height

    private:
        void Cleanup();

        GLSL* scene;                          ///< Scene shader
        unsigned int* rendertexture;          ///< Texture for color buffer
        unsigned int* fboID;                  ///< Frame buffer object id
        unsigned int* rboID;                  ///< Render buffer object id
    };
}

#endif // GLES20_H
