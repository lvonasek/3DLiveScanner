#include <data/file3d.h>
#include <postproc/texturize.h>

#include <chrono>
#include <iostream>

#define FRAME_WIDTH 360
#define FRAME_HEIGHT 640
#define MAXIMAL_DEPTH_IN_METERS 5
#define MISSING_TEXELS_TOLERANCE_IN_PERCENT 0.5f

namespace oc {

void(*g_texturize_callback)(int current, int count) = 0;
std::map<int, double> g_frames;
static bool ComparatorFrames(const int& a, const int& b) {
    return g_frames[a] > g_frames[b];
}

std::vector<int> Texturize::GetFrames() {
    std::vector<int> output;
    for (std::map<int, double>::const_iterator it = frames.begin(); it != frames.end(); ++it) {
        output.push_back(it->first);
    }
    return output;
}

void Texturize::Process(Dataset* dataset, std::string model, bool finalRender) {
    this->dataset = dataset;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    LoadModel(model);
    ProjectFrames(true);
    RemoveBadFrames();
    if (finalRender) {
        ProjectFrames(false);
    }
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int ms = std::chrono::duration_cast<std::chrono::milliseconds> (end - begin).count();
    LOGI("Process done in %.2fs", ms * 0.001f);
}

void Texturize::SetCallback(void(*callback)(int current, int count)) {
    g_texturize_callback = callback;
}

void Texturize::LoadModel(std::string path) {
    int count;
    geom.clear();
    dataset->ReadState(count, width, height, cx, cy, fx, fy);
    cx /= (double)width;
    cy /= (double)height;
    fx /= (double)width;
    fy /= (double)height;
    width = FRAME_WIDTH;
    height = FRAME_HEIGHT;
    cx *= (double)width;
    cy *= (double)height;
    fx *= (double)width;
    fy *= (double)height;
    File3d(path, false).ReadModel(-1, geom);

    frames.clear();
    for (int i = 0; i < count; i++) {
        frames[i] = 0;
    }
}

void Texturize::ProjectFrames(bool edges) {

    //clear texture
    for (Mesh& m : geom) {
        m.GenerateNormals();
        if (m.image && m.imageOwner && !m.vertices.empty()) {
            for (int i = 0; i < m.image->GetWidth() * m.image->GetHeight() * 4; i += 4) {
                m.image->GetData()[i + 0] = 128;
                m.image->GetData()[i + 1] = 128;
                m.image->GetData()[i + 2] = 128;
                m.image->GetData()[i + 3] = 0;
            }
            resolution = m.image->GetWidth();
        }
    }
    SetResolution(resolution, resolution);

    //initialize objects
    glm::ivec4* dx = new glm::ivec4[height];
    glm::ivec4* dy = new glm::ivec4[width];
    glm::ivec4* nx = new glm::ivec4[height];
    glm::ivec4* ny = new glm::ivec4[width];
    depthmap = new Depthmap(width, height);
    mapping = new int[resolution * resolution];
    memset(dx, 0, height * sizeof(glm::ivec4));
    memset(dy, 0, width * sizeof(glm::ivec4));

    //iterate over all frames
    for (std::map<int, double>::const_iterator it = frames.begin(); it != frames.end(); ++it) {

        //get captured photo
        if (g_texturize_callback) g_texturize_callback(it->first + 1, frames.size());
        frame = new Image(dataset->GetFileName(it->first, ".jpg"));
        if ((frame->GetWidth() != FRAME_WIDTH) || (frame->GetHeight() != FRAME_HEIGHT)) {
            Image* downscaled = frame->Downscale(frame->GetWidth() / FRAME_WIDTH);
            delete frame;
            frame = downscaled;
        }
        if (edges) {
            cv::Mat mat(frame->GetHeight(), frame->GetWidth(), CV_8UC1);
            for (int x = 0; x < frame->GetWidth(); x++) {
                for (int y = 0; y < frame->GetHeight(); y++) {
                    glm::ivec4 color = frame->GetColorRGBA(x, y);
                    mat.at<uchar>(y, x) = (color.r + color.g + color.b) / 3;
                }
            }

            cv::Canny(mat, mat, 50, 200, 3);
            for (int x = 0; x < frame->GetWidth(); x++) {
                for (int y = 0; y < frame->GetHeight(); y++) {
                    glm::ivec4 color(mat.at<uchar>(y, x));
                    color = (color + frame->GetColorRGBA(x, y)) / 2;
                    color.a = 255;
                    frame->DrawPixel(x, y, color);
                }
            }
            //frame->UpsideDown();
            //frame->Write(dataset->GetFileName(it->first, ".png"));
            //frame->UpsideDown();
        }

        //generate depthmap
        glm::mat4 world2screen = dataset->ReadPose(it->first)[SCREEN_CAMERA];
        depthmap->Render(geom, world2screen);
        if (edges) {
            //depthmap->GetRGB()->Write(dataset->GetFileName(it->first, ".png"));
            //depthmap->WritePNG16(dataset->GetFileName(it->first, ".png"));
        }

        //get frame to texture mapping (texels)
        for (submodel = 0; submodel < geom.size(); submodel++) {
            if (!geom[submodel].vertices.empty()) {
                memset(mapping, -1, resolution * resolution * sizeof(int));
                AddUVS(geom[submodel].uv, geom[submodel].vertices, world2screen, width, height);
            }
        }

        //texturize
        WriteTexels(dataset->GetFileName(it->first, ".tex"));
        if (!texels.empty()) {
            if (!edges) {
                ColorMapping(dx, dy);
                ColorMappingBlur(dx, dy, nx, ny);
                //ColorMappingDebug(dataset->GetFileName(it->first, ".png"), dx, dy);
            }
            DrawTexels(dx, dy);
            texels.clear();
        }
        delete frame;
    }

    //cleanup
    delete depthmap;
    delete[] mapping;
    delete[] dx;
    delete[] dy;
    delete[] nx;
    delete[] ny;

    if (g_texturize_callback) g_texturize_callback(-1, -1);
}


void Texturize::RemoveBadFrames() {

    //prioritize bad frames
    std::vector<int> toBeProcessed;
    for (std::map<int, double>::const_iterator it = frames.begin(); it != frames.end(); ++it) {
        long count = 0;
        long diff = 0;
        std::vector<TexelBase> data = ReadTexels(dataset->GetFileName(it->first, ".tex"));
        for (TexelBase& texel : data) {
            int index = (geom[texel.submodel].image->GetWidth() * texel.v + texel.u) * 4;
            diff += abs(geom[texel.submodel].image->GetData()[index + 0] - texel.r);
            diff += abs(geom[texel.submodel].image->GetData()[index + 1] - texel.g);
            diff += abs(geom[texel.submodel].image->GetData()[index + 2] - texel.b);
            count += 255 + 255 + 255;
        }
        if (count > 0) {
            frames[it->first] = diff / (double)count;
        }
        toBeProcessed.push_back(it->first);
    }

    //process of removing bad frames
    g_frames = frames;
    std::sort(toBeProcessed.begin(), toBeProcessed.end(), ComparatorFrames);
    for (int& i : toBeProcessed) {

        //check if the frame could be still deleted
        int invalid = 0;
        std::vector<TexelBase> data = ReadTexels(dataset->GetFileName(i, ".tex"));
        for (TexelBase& texel : data) {
            int index = (geom[texel.submodel].image->GetWidth() * texel.v + texel.u) * 4 + 3;
            if (geom[texel.submodel].image->GetData()[index] <= 1) {
                invalid++;
            }
        }

        //delete frame
        float missing = invalid / (float)data.size() * 100.0f;
        if (missing < MISSING_TEXELS_TOLERANCE_IN_PERCENT) {
            int n;
            glm::ivec4 color;
            LOGI("Deleting frame %d with diff %.3lf%%", i, frames[i] * 100.0);
            for (TexelBase& t : data) {
                color = geom[t.submodel].image->GetColorRGBA(t.u, t.v);
                if (color.a > 1) {
                    n = glm::max(color.a - 1, 0);
                    color = ((n + 1) * color - glm::ivec4(t.r, t.g, t.b, 1)) / n;
                    color = glm::clamp(color, glm::ivec4(0), glm::ivec4(255));
                    color.a = n;
                } else {
                    color = glm::ivec4(0);
                }
                geom[t.submodel].image->DrawPixel(t.u, t.v, color);
            }
            frames.erase(i);
        } else {
            LOGI("Frame %d with diff %.3lf%% and %.3f%% missing cannot be deleted", i, frames[i] * 100.0, missing);
        }
    }

    LOGI("%d/%d frames used", frames.size(), g_frames.size());
}

void Texturize::WriteTextures() {
    for (Mesh& m : geom) {
        if (m.image && m.imageOwner && !m.vertices.empty()) {
            AddTextureBevel(m.image);
            for (int i = 3; i < m.image->GetWidth() * m.image->GetHeight() * 4; i += 4) {
                m.image->GetData()[i] = 255;
            }
            m.image->Write(m.image->GetName());
        }
    }
}

void Texturize::AddTextureBevel(Image* image) {

    glm::ivec4 bevel, color;

    //set alpha channel for defined pixel
    for (int i = 3; i < image->GetWidth() * image->GetHeight() * 4; i += 4) {
        if (image->GetData()[i] > 0) {
            image->GetData()[i] = 255;
        }
    }

    //horizontal bevel
    for (int y = 0; y < image->GetHeight(); y++) {
        bevel = glm::ivec4(0);
        for (int x = 0; x < image->GetWidth(); x++) {
            BevelShader(image, bevel, color, x, y);
        }
        bevel = glm::ivec4(0);
        for (int x = image->GetWidth() - 1; x >= 0; x--) {
            BevelShader(image, bevel, color, x, y);
        }
    }

    //vertical bevel
    for (int x = 0; x < image->GetWidth(); x++) {
        bevel = glm::ivec4(0);
        for (int y = 0; y < image->GetHeight(); y++) {
            BevelShader(image, bevel, color, x, y);
        }
        bevel = glm::ivec4(0);
        for (int y = image->GetHeight() - 1; y >= 0; y--) {
            BevelShader(image, bevel, color, x, y);
        }
    }
}

void Texturize::BevelShader(Image* image, glm::ivec4& bevel, glm::ivec4& color, int& x, int& y) {
    color = image->GetColorRGBA(x, y);
    if (color.a < bevel.a) {
        bevel.a--;
        image->DrawPixel(x, y, bevel);
    } else {
        bevel = color;
        bevel.a = glm::min(bevel.a, 50);
    }
}

void Texturize::ColorMapping(glm::ivec4* dx, glm::ivec4* dy) {

    //reset the values
    int* countx = new int[height];
    int* county = new int[width];
    for (int x = 0; x < width; x++) {
        dy[x] = glm::ivec4(0);
        county[x] = 0;
    }
    for (int y = 0; y < height; y++) {
        dx[y] = glm::ivec4(0);
        countx[y] = 0;
    }

    //sum the difference between the texture and current color frame
    for (Texel& t : texels) {
        glm::ivec4 orig = geom[t.submodel].image->GetColorRGBA(t.u, t.v);
        if (orig.a > 0) {
            dx[t.y] += t.color - orig;
            dy[t.x] += t.color - orig;
            countx[t.y]++;
            county[t.x]++;
        }
    }

    //make from the sum an average value
    for (int x = 0; x < width; x++) {
        if (county[x] > 0) {
            dy[x] /= county[x];
        }
    }
    for (int y = 0; y < height; y++) {
        if (countx[y] > 0) {
            dx[y] /= countx[y];
        }
    }
    delete[] countx;
    delete[] county;
}

void Texturize::ColorMappingBlur(glm::ivec4* dx, glm::ivec4* dy, glm::ivec4* nx, glm::ivec4* ny) {
    int count;
    int blur = width / 10;

    for (int x = 0; x < width; x++) {
        count = 0;
        ny[x] = glm::ivec4(0);
        for (int i = glm::max(0, x - blur); i <= glm::min(x + blur, width - 1); i++) {
            ny[x] += dy[i];
            count++;
        }
        ny[x] /= count;
    }
    for (int y = 0; y < height; y++) {
        count = 0;
        nx[y] = glm::ivec4(0);
        for (int i = glm::max(0, y - blur); i <= glm::min(y + blur, height - 1); i++) {
            nx[y] += dx[i];
            count++;
        }
        nx[y] /= count;
    }
    memcpy(dy, ny, width * sizeof(glm::ivec4));
    memcpy(dx, nx, height * sizeof(glm::ivec4));
}

void Texturize::ColorMappingDebug(std::string filename, glm::ivec4* dx, glm::ivec4* dy) {
    Image output(width, height);
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            glm::ivec4 color = (dx[y] + dy[x]) / 2 + 128;
            color.r = glm::clamp(color.r, 0, 255);
            color.g = glm::clamp(color.g, 0, 255);
            color.b = glm::clamp(color.b, 0, 255);
            color.a = 255;
            output.DrawPixel(x, y, color);
        }
    }
    output.Write(filename);
}

void Texturize::DrawTexels(glm::ivec4* dx, glm::ivec4* dy) {
    int n;
    glm::ivec4 color;
    for (Texel& t : texels) {
        t.color -= (dx[t.y] + dy[t.x]) / 2;
        t.color.r = glm::clamp(t.color.r, 0, 255);
        t.color.g = glm::clamp(t.color.g, 0, 255);
        t.color.b = glm::clamp(t.color.b, 0, 255);
        t.color.a = 1;

        color = geom[t.submodel].image->GetColorRGBA(t.u, t.v);
        if (color.a > 0) {
            n = glm::min(color.a + 1, 255);
            color = ((n - 1) * color + t.color) / n;
            color.a = n;
            geom[t.submodel].image->DrawPixel(t.u, t.v, color);
        } else {
            geom[t.submodel].image->DrawPixel(t.u, t.v, t.color);
        }
    }
}

void Texturize::Process(unsigned long &index, int &x1, int &x2, int &y, glm::dvec3 &z1, glm::dvec3 &z2) {
    Texel t;
    for (int x = glm::max(x1, 0); x <= glm::min(x2, viewport_width - 1); x++) {
        glm::vec3 z = z1 + (z2 - z1) * (double)(x - x1) / (double)(x2 - x1);
        if ((z.z > 0) && (z.z < MAXIMAL_DEPTH_IN_METERS)) {
            t.x = z.x;
            t.y = z.y;
            t.depth = z.z;
            t.submodel = submodel;
            if (depthmap->CanPass(t.x, t.y, t.depth)) {
                t.y = height - t.y - 1;
                if (mapping[y * resolution + x] == -1) {
                    mapping[y * resolution + x] = texels.size();
                    t.color = frame->GetColorRGBA(z.x, z.y);
                    t.u = x;
                    t.v = y;
                    texels.push_back(t);
                } else if (texels[mapping[y * resolution + x]].depth > t.depth) {
                    t.color = frame->GetColorRGBA(z.x, z.y);
                    t.u = x;
                    t.v = y;
                    texels[mapping[y * resolution + x]] = t;
                }
            }
        }
    }
}

std::vector<TexelBase> Texturize::ReadTexels(std::string filename) {

    //read data
    unsigned long size = 0;
    FILE* file = fopen(filename.c_str(), "rb");
    fread(&size, sizeof(unsigned long), 1, file);
    TexelBase* data = new TexelBase[size];
    fread(data, sizeof(TexelBase), size, file);
    fclose(file);

    //convert into vector
    std::vector<TexelBase> output;
    for (unsigned long i = 0; i < size; i++) {
        output.push_back(data[i]);
    }
    delete[] data;
    return output;
}

void Texturize::WriteTexels(std::string filename) {

    //convert data to low memory format
    std::vector<TexelBase> data;
    for (Texel& t : texels) {
        TexelBase tb;
        tb.r = t.color.r;
        tb.g = t.color.g;
        tb.b = t.color.b;
        tb.u = t.u;
        tb.v = t.v;
        tb.submodel = t.submodel;
        data.push_back(tb);
    }

    //write the data into a file
    unsigned long size = data.size();
    FILE* file = fopen(filename.c_str(), "wb");
    fwrite(&size, sizeof(unsigned long), 1, file);
    fwrite(&data[0], sizeof(TexelBase), size, file);
    fclose(file);
}
}
