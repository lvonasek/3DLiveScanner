#include "gl/renderer.h"

std::string RTTFragmentShader() {
  return "uniform sampler2D color_texture;\n"
         "varying vec2 texture_coordinate;\n"
         "void main() {\n"
         "  gl_FragColor.rgb = texture2D(color_texture, texture_coordinate).rgb;\n"
         "}\n";
}

std::string RTTVertexShader() {
  return "varying vec2 texture_coordinate;\n"
         "attribute vec3 v_vertex;\n"
         "attribute vec2 v_coord;\n"
         "void main()\n"
         "{\n"
         "    texture_coordinate = v_coord.st;\n"
         "    gl_Position = vec4(v_vertex.xy, 0.1001, 1.0);\n"
         "}\n";
}

namespace oc {
    GLRenderer::GLRenderer() {
        fboID = 0;
        rboID = 0;
        rendertexture = 0;
        scene = 0;
    }

    GLRenderer::~GLRenderer() {
        Cleanup();
    }

    void GLRenderer::Cleanup() {
        if (fboID) {
            glDeleteFramebuffers(1, fboID);
            delete[] fboID;
            fboID = 0;
        }
        if (rboID) {
            glDeleteRenderbuffers(1, rboID);
            delete[] rboID;
            rboID = 0;
        }
        if (rendertexture) {
            glDeleteTextures(1, rendertexture);
            delete[] rendertexture;
            rendertexture = 0;
        }
        if (scene) {
            delete scene;
            scene = 0;
        }
    }

    void GLRenderer::Init(int w, int h, int rw, int rh) {
        rWidth = rw;
        rHeight = rh;
        width = w;
        height = h;
        Cleanup();

        //create frame buffer
        fboID = new unsigned int[1];
        rboID = new unsigned int[1];
        rendertexture = new unsigned int[1];

        //framebuffer textures
        glGenTextures(1, rendertexture);
        glBindTexture(GL_TEXTURE_2D, rendertexture[0]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, rWidth, rHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0);

        /// create render buffer for depth buffer
        glGenRenderbuffers(1, rboID);
        glBindRenderbuffer(GL_RENDERBUFFER, rboID[0]);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, rWidth, rHeight);

        //framebuffers
        glGenFramebuffers(1, fboID);
        glBindFramebuffer(GL_FRAMEBUFFER, fboID[0]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, rendertexture[0], 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rboID[0]);

        /// check FBO status
        if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            exit(EXIT_SUCCESS);

        //set viewport
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport (0, 0, (GLsizei) width, (GLsizei) height);
        glClear(GL_COLOR_BUFFER_BIT);
        glActiveTexture( GL_TEXTURE0 );

        //set objects
        camera.projection = glm::perspective(glm::radians(45.0f), w / (float)h, 0.1f, 1000.0f);
        scene = new GLSL(RTTVertexShader(), RTTFragmentShader());
    }

    Image* GLRenderer::ReadRtt() {
        return ReadRtt(0, 0, rWidth, rHeight);
    }

    Image* GLRenderer::ReadRtt(int x, int y, int w, int h) {
        Image* output = new Image(w, h);
        glBindFramebuffer(GL_FRAMEBUFFER, fboID[0]);
        glViewport(x, y, output->GetWidth(), output->GetHeight());
        glReadPixels(x, y, output->GetWidth(), output->GetHeight(), GL_RGBA, GL_UNSIGNED_BYTE, output->GetData());
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, width, height);
        return output;
    }

    void GLRenderer::Render(float* vertices, float* normals, float* uv, unsigned int* colors,
                            unsigned long size, unsigned int* indices, int type) {
        GLSL::CurrentShader()->UniformMatrix("MVP", glm::value_ptr(camera.projection * camera.GetView()));
        GLSL::CurrentShader()->Attrib(vertices, normals, uv, colors);

        if (size > 0) {
            if (indices)
              glDrawElements(type, (GLsizei) size, GL_UNSIGNED_INT, indices);
            else
              glDrawArrays(type, 0, (GLsizei) size);
        }
    }

    void GLRenderer::Rtt(bool enable) {
        if (enable) {
            glBindFramebuffer(GL_FRAMEBUFFER, fboID[0]);
            glViewport(0, 0, rWidth, rHeight);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glEnable(GL_DEPTH_TEST);
            glDepthMask(GL_TRUE);
        } else {
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glViewport(0, 0, width, height);
        }
    }
}
