#ifndef DATA_IMAGE_H
#define DATA_IMAGE_H

#include <string>
#include <vector>
#include "gl/opengl.h"

namespace oc {

    class Image {
    public:
        Image(unsigned char r, unsigned char g, unsigned char b, unsigned char a);
        Image(int w, int h);
        Image(std::string filename);
        ~Image();
        void AddInstance() { instances++; }
        bool CanBeDeleted() { return instances <= 0; }
        void DelInstance() { instances--; }
        unsigned char* ExtractYUV(unsigned int s);
        unsigned char* ExtractYUVDownscaled(unsigned int s);

        Image* Blur(int size);
        Image* Edges();
        Image* Downscale(int scale);

        void Clear();
        void SetName(std::string value) { name = value; }
        void SetTexture(long value) { texture = value; }
        void Turn();
        void UpdateTexture();
        void UpdateYUV(unsigned char* src0, unsigned char* src1, int w, int h, int scale);
        void UpsideDown();
        void Write(std::string filename);

        void DrawCircle(int cx, int cy, int radius, glm::ivec4 color);
        void DrawLine(int x1, int y1, int x2, int y2, glm::ivec4 color);
        void DrawPixel(int x, int y, glm::ivec4& color);

        unsigned int GetColor(int x, int y);
        glm::ivec4 GetColorRGBA(int x, int y, int s = 0);
        std::string GetExtension();
        int GetWidth() { return width; }
        int GetHeight() { return height; }
        unsigned char* GetData() { return data; }
        std::string GetName() { return name; }
        long GetTexture() { return texture; }

        static void JPG2YUV(std::string filename, unsigned char* data, int width, int height);
        static void YUV2JPG(unsigned char* data, int width, int height, std::string filename, bool gray);
        static std::vector<unsigned int> TexturesToDelete();

    private:
        bool ClipTest(double p, double q, double &t1, double &t2);

        void ReadJPG(std::string filename);
        void ReadPNG(std::string filename);
        void WriteJPG(std::string filename);
        void WritePNG(std::string filename);

        int instances;
        int width;
        int height;
        unsigned char* data;
        std::string name;
        long texture;
    };
}

#endif
