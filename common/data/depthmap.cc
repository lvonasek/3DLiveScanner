#include <png.h>
#include <data/depthmap.h>

namespace oc {

Depthmap::Depthmap(int w, int h)
{
    depth = new double[w * h];
    rgb = new Image(w, h);
    width = w;
    height = h;
    SetResolution(width, height);
}

Depthmap::~Depthmap() {
    delete[] depth;
    delete rgb;
}

void Depthmap::BevelMask(int mw, int mh) {
    int count;
    glm::ivec2 p;

    //horizontal masking
    for (p.y = 0; p.y < height; p.y++) {
        count = 0;
        for (p.x = 0; p.x < width; p.x++) {
            BevelShader(count, p, mw);
        }
        count = 0;
        for (p.x = width - 1; p.x >= 0; p.x--) {
            BevelShader(count, p, mw);
        }
    }

    //vertical masking
    for (p.x = 0; p.x < width; p.x++) {
        count = 0;
        for (p.y = 0; p.y < height; p.y++) {
            BevelShader(count, p, mh);
        }
        count = 0;
        for (p.y = height - 1; p.y >= 0; p.y--) {
            BevelShader(count, p, mh);
        }
    }

    //apply masking
    for (glm::ivec2 p : mask) {
        depth[p.y * width + p.x] = 0;
    }
    mask.clear();
}

bool Depthmap::CanPass(int& x, int& y, double& value) {
    if ((x >= 0) && (y >= 0) && (x < width) && (y < height)) {
        int i = (height - y - 1) * width + x;
        if (glm::abs(depth[i] - value) < 0.05) {
            return true;
        }
    }
    return false;
}

bool Depthmap::CanWrite(int& x, int& y, double& value) {
    if ((x >= 0) && (y >= 0) && (x < width) && (y < height)) {
        int i = (height - y - 1) * width + x;
        if ((depth[i] < 0.001) || (glm::abs(depth[i] - value) < 0.05)) {
            return true;
        }
    }
    return false;
}

double Depthmap::CountDiff(Depthmap* data) {
    unsigned int count = 0;
    double diff = 0;
    for (unsigned int i = 0; i < width * height; i++) {
        if ((depth[i] != 0) && (data->depth[i] != 0)) {
            diff += glm::abs(depth[i] - data->depth[i]);
            count++;
        }
    }

    if (count > 0) {
         return diff / count;
    } else {
        return INT_MAX;
    }
}

void Depthmap::FromPCL(Dataset* dataset, int poseIndex, bool fillHoles)
{
    //get point cloud
    int size = 0;
    std::string filename = dataset->GetFileName(poseIndex, ".pcl");
    LOGI("Reading %s", filename.c_str());
    FILE* file = fopen(filename.c_str(), "rb");
    fread(&size, sizeof(int), 1, file);
    Tango3DR_Vector4* points = new Tango3DR_Vector4[size];
    fread(points, sizeof(Tango3DR_Vector4), size, file);
    fclose(file);

    //get matrices
    glm::mat4 depth_mat = dataset->ReadPose(poseIndex)[COLOR_CAMERA];
    glm::mat4 viewproj_mat = dataset->ReadPose(poseIndex)[SCREEN_CAMERA];

    //init variables
    bool valid[width * height];
    for (unsigned int i = 0; i < width * height; i++)
    {
        depth[i] = 0;
        valid[i] = false;
    }

    //convert point cloud into depthmap
    for (unsigned int i = 0; i < size; i++)
    {
        glm::vec4 v = depth_mat * glm::vec4(points[i][0], points[i][1], points[i][2], 1.0f);
        glm::vec4 t = viewproj_mat * glm::vec4(v.x, v.y, v.z, 1.0f);
        t /= fabs(t.w);
        t = t * 0.5f + 0.5f;
        int x = (int)(t.x * width + 0.5f);
        int y = (int)((1.0f - t.y) * height + 0.5f);
        double value = glm::abs(v.y);
        if (fillHoles)
        {
            for (int k = x - 1; k <= x + 1; k++)
            {
                for (int l = y - 1; l <= y + 1; l++)
                {
                    if ((k < 0) || (l < 0) || (k >= width) || (l >= height))
                        continue;
                    if (!valid[l * width + k])
                    {
                        depth[l * width + k] = value;
                        if ((x == k) && (y == l))
                            valid[l * width + k] = true;
                    }
                }
            }
        }
        else
        {
            if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
                continue;
            depth[y * width + x] = value;
        }
    }
    delete[] points;
}

double Depthmap::GetDepth(int x, int y) {
    return depth[(height - y - 1) * width + x];
}

void Depthmap::MaskEdges(int bevel) {

    int count;
    double last;
    glm::ivec2 p;

    //horizontal masking
    for (p.y = 0; p.y < height; p.y++) {
        count = 0;
        last = 0;
        for (p.x = 0; p.x < width; p.x++) {
            EdgeShader(last, count, p, bevel);
        }
        count = 0;
        last = 0;
        for (p.x = width - 1; p.x >= 0; p.x--) {
            EdgeShader(last, count, p, bevel);
        }
    }

    //vertical masking
    for (p.x = 0; p.x < width; p.x++) {
        count = 0;
        last = 0;
        for (p.y = 0; p.y < height; p.y++) {
            EdgeShader(last, count, p, bevel);
        }
        count = 0;
        last = 0;
        for (p.y = height - 1; p.y >= 0; p.y--) {
            EdgeShader(last, count, p, bevel);
        }
    }

    //apply masking
    for (glm::ivec2 p : mask) {
        depth[p.y * width + p.x] = 0;
    }
    mask.clear();
}

void Depthmap::Render(std::vector<Mesh>& geom, glm::mat4& world2screen) {
    std::fill_n(depth, width * height, 0);
    memset(rgb->GetData(), 0, rgb->GetWidth() * rgb->GetHeight() * 4);
    for (int j = 0; j < geom.size(); j++) {
        submodel = j;
        texture = geom[j].image;
        AddUVVertices(geom[j].vertices, geom[j].uv, world2screen);
    }
}

void Depthmap::BevelShader(int& count, glm::ivec2& p, int& bevel) {
    if (count > 0) {
        mask.push_back(p);
        count--;
    } else if (depth[p.y * width + p.x] < 0.001f) {
        count = bevel;
    }
}

void Depthmap::EdgeShader(double& last, int& count, glm::ivec2& p, int& bevel) {
    double current = depth[p.y * width + p.x];
    if (count > 0) {
        mask.push_back(p);
        count--;
    } else if ((current != 0) && (last != 0) && (glm::abs(last - current) > 0.1f)) {
        count = bevel;
    }
    last = current;
}

void Depthmap::Process(unsigned long &index, int &x1, int &x2, int &y, glm::dvec3 &z1, glm::dvec3 &z2) {
    int x, u, v;
    for (x = x1; x <= x2; x++) {
        if ((x >= 0) && (x < width)) {
            int i = (height - y - 1) * width + x;
            glm::vec3 z = z1 + (z2 - z1) * (double)(x - x1) / (double)(x2 - x1);
            if (z.z > 0) {
                if ((depth[i] < 0.001f) || (depth[i] > z.z)) {
                    depth[i] = z.z;
                    u = z.x * texture->GetWidth();
                    v = texture->GetHeight() - z.y * texture->GetHeight() - 1;
                    glm::ivec4 color = texture->GetColorRGBA(u, v);
                    color.a = 255;
                    rgb->DrawPixel(x, y, color);
                }
            }
        }
    }
}

void Depthmap::WritePNG16(std::string filename)
{
    // Open file for writing (binary mode)
    LOGI("Writing depthmap into %s", filename.c_str());
    FILE* file = fopen(filename.c_str(), "wb");

    // init PNG library
    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info_ptr = png_create_info_struct(png_ptr);

    png_init_io(png_ptr, file);
    png_set_IHDR(png_ptr, info_ptr, (png_uint_32) width, (png_uint_32) height,
                 16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png_ptr, info_ptr);
    png_set_swap(png_ptr);

    // write image data
    unsigned short int row[width];
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            row[x] = (unsigned short)glm::clamp((int)(depth[(y * width + x)] * DEPTHMAP_SCALE), 0, USHRT_MAX);
        }
        png_write_row(png_ptr, (png_const_bytep)row);
    }
    png_write_end(png_ptr, NULL);

    /// close all
    if (file != NULL) fclose(file);
    if (info_ptr != NULL) png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
    if (png_ptr != NULL) png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
}
}
