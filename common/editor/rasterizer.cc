#include "editor/rasterizer.h"

namespace oc {

    Rasterizer::Rasterizer() {
        fillCache1 = nullptr;
        fillCache2 = nullptr;
        viewport_width = 0;
        viewport_height = 0;
    }

    Rasterizer::~Rasterizer() {
        if (fillCache1) delete[] fillCache1;
        if (fillCache2) delete[] fillCache2;
    }

    void Rasterizer::AddUVS(std::vector<glm::vec2> uvs, std::vector<unsigned int> selected) {
        glm::vec3 a, b, c;
        for (unsigned long i = 0; i < uvs.size(); i += 3) {
            if (!selected.empty()) {
                if (selected[i + 0] != 0)
                    continue;
                if (selected[i + 1] != 0)
                    continue;
                if (selected[i + 2] != 0)
                    continue;
            }
            //get coordinate
            a = glm::vec3(uvs[i + 0], 0.0f);
            b = glm::vec3(uvs[i + 1], 0.0f);
            c = glm::vec3(uvs[i + 2], 0.0f);
            //mirror y axis
            a.y = 1.0f - a.y;
            b.y = 1.0f - b.y;
            c.y = 1.0f - c.y;
            //scale into raster dimensions
            a.x *= (float)(viewport_width - 1);
            a.y *= (float)(viewport_height - 1);
            b.x *= (float)(viewport_width - 1);
            b.y *= (float)(viewport_height - 1);
            c.x *= (float)(viewport_width - 1);
            c.y *= (float)(viewport_height - 1);
            //process
            Triangle(i, a, b, c);
        }
    }

    void Rasterizer::AddUVS(std::vector<glm::vec2> uvs, std::vector<glm::vec3>& vertices, glm::mat4 world2screen, int frameWidth, int frameHeight) {
        glm::vec3 a, b, c, ba, ca;
        glm::vec4 wa, wb, wc;
        for (unsigned long i = 0; i < uvs.size(); i += 3) {
            //get coordinate
            wa = glm::vec4(vertices[i + 0], 1.0f);
            wb = glm::vec4(vertices[i + 1], 1.0f);
            wc = glm::vec4(vertices[i + 2], 1.0f);
            //transform to 2D
            wa = world2screen * wa;
            wb = world2screen * wb;
            wc = world2screen * wc;
            //perspective division
            wa.x /= glm::abs(wa.w);
            wa.y /= glm::abs(wa.w);
            wb.x /= glm::abs(wb.w);
            wb.y /= glm::abs(wb.w);
            wc.x /= glm::abs(wc.w);
            wc.y /= glm::abs(wc.w);
            //convert it from -1,1 to 0,1
            wa.x = (wa.x + 1.0f) * 0.5f;
            wa.y = (wa.y + 1.0f) * 0.5f;
            wb.x = (wb.x + 1.0f) * 0.5f;
            wb.y = (wb.y + 1.0f) * 0.5f;
            wc.x = (wc.x + 1.0f) * 0.5f;
            wc.y = (wc.y + 1.0f) * 0.5f;
            //scale into raster dimensions
            wa.x *= (float)(frameWidth - 1);
            wa.y *= (float)(frameHeight - 1);
            wb.x *= (float)(frameWidth - 1);
            wb.y *= (float)(frameHeight - 1);
            wc.x *= (float)(frameWidth - 1);
            wc.y *= (float)(frameHeight - 1);
            //get UV coordinate
            a = glm::vec3(uvs[i + 0], wa.z);
            b = glm::vec3(uvs[i + 1], wb.z);
            c = glm::vec3(uvs[i + 2], wc.z);
            //mirror y axis
            a.y = 1.0f - a.y;
            b.y = 1.0f - b.y;
            c.y = 1.0f - c.y;
            //scale into raster dimensions
            a.x *= (float)(viewport_width - 1);
            a.y *= (float)(viewport_height - 1);
            b.x *= (float)(viewport_width - 1);
            b.y *= (float)(viewport_height - 1);
            c.x *= (float)(viewport_width - 1);
            c.y *= (float)(viewport_height - 1);
            //back face culling
            ba = glm::vec3(b.x - a.x, b.y - a.y, 0.0f);
            ca = glm::vec3(c.x - a.x, c.y - a.y, 0.0f);
            if (glm::cross(ba, ca).z < 0)
                continue;
            //process
            Triangle(i, a, b, c, wa, wb, wc);
        }
    }

    void Rasterizer::AddVertices(std::vector<glm::vec3>& vertices, glm::mat4 world2screen, bool culling) {
        glm::vec3 a, b, c, ba, ca;
        glm::vec4 wa, wb, wc;
        for (unsigned long i = 0; i < vertices.size(); i += 3) {
            //get coordinate
            wa = glm::vec4(vertices[i + 0], 1.0f);
            wb = glm::vec4(vertices[i + 1], 1.0f);
            wc = glm::vec4(vertices[i + 2], 1.0f);
            //transform to 2D
            wa = world2screen * wa;
            wb = world2screen * wb;
            wc = world2screen * wc;
            //perspective division
            wa.x /= glm::abs(wa.w);
            wa.y /= glm::abs(wa.w);
            wb.x /= glm::abs(wb.w);
            wb.y /= glm::abs(wb.w);
            wc.x /= glm::abs(wc.w);
            wc.y /= glm::abs(wc.w);
            //convert
            a.x = wa.x;
            a.y = wa.y;
            a.z = wa.z;
            b.x = wb.x;
            b.y = wb.y;
            b.z = wb.z;
            c.x = wc.x;
            c.y = wc.y;
            c.z = wc.z;
            //convert it from -1,1 to 0,1
            a.x = (a.x + 1.0f) * 0.5f;
            a.y = (a.y + 1.0f) * 0.5f;
            b.x = (b.x + 1.0f) * 0.5f;
            b.y = (b.y + 1.0f) * 0.5f;
            c.x = (c.x + 1.0f) * 0.5f;
            c.y = (c.y + 1.0f) * 0.5f;
            //scale into raster dimensions
            a.x *= (float)(viewport_width - 1);
            a.y *= (float)(viewport_height - 1);
            b.x *= (float)(viewport_width - 1);
            b.y *= (float)(viewport_height - 1);
            c.x *= (float)(viewport_width - 1);
            c.y *= (float)(viewport_height - 1);
            //back face culling
            if (culling) {
                ba = glm::vec3(b.x - a.x, b.y - a.y, 0.0f);
                ca = glm::vec3(c.x - a.x, c.y - a.y, 0.0f);
                if (glm::cross(ba, ca).z < 0)
                    continue;
            }
            //process
            Triangle(i, a, b, c);
        }
    }

    void Rasterizer::AddUVVertices(std::vector<glm::vec3>& vertices, std::vector<glm::vec2>& uvs, glm::mat4 world2screen) {
        glm::vec3 a, b, c;
        glm::vec4 wa, wb, wc;
        for (unsigned long i = 0; i < vertices.size(); i += 3) {
            //get coordinate
            wa = glm::vec4(vertices[i + 0], 1.0f);
            wb = glm::vec4(vertices[i + 1], 1.0f);
            wc = glm::vec4(vertices[i + 2], 1.0f);
            //transform to 2D
            wa = world2screen * wa;
            wb = world2screen * wb;
            wc = world2screen * wc;
            //perspective division
            wa.x /= glm::abs(wa.w);
            wa.y /= glm::abs(wa.w);
            wb.x /= glm::abs(wb.w);
            wb.y /= glm::abs(wb.w);
            wc.x /= glm::abs(wc.w);
            wc.y /= glm::abs(wc.w);
            //convert
            a.x = wa.x;
            a.y = wa.y;
            a.z = wa.z;
            b.x = wb.x;
            b.y = wb.y;
            b.z = wb.z;
            c.x = wc.x;
            c.y = wc.y;
            c.z = wc.z;
            //convert it from -1,1 to 0,1
            a.x = (a.x + 1.0f) * 0.5f;
            a.y = (a.y + 1.0f) * 0.5f;
            b.x = (b.x + 1.0f) * 0.5f;
            b.y = (b.y + 1.0f) * 0.5f;
            c.x = (c.x + 1.0f) * 0.5f;
            c.y = (c.y + 1.0f) * 0.5f;
            //scale into raster dimensions
            a.x *= (float)(viewport_width - 1);
            a.y *= (float)(viewport_height - 1);
            b.x *= (float)(viewport_width - 1);
            b.y *= (float)(viewport_height - 1);
            c.x *= (float)(viewport_width - 1);
            c.y *= (float)(viewport_height - 1);
            //process
            Triangle(i, a, b, c, uvs[i + 0], uvs[i + 1], uvs[i + 2]);
        }
    }

    void Rasterizer::SetResolution(int w, int h) {
        if ((viewport_width != w) || (viewport_height != h)) {
            viewport_width = w;
            viewport_height = h;

            unsigned long size = h + 1;
            if (fillCache1) delete[] fillCache1;
            if (fillCache2) delete[] fillCache2;
            fillCache1 = new std::pair<int, glm::dvec3>[size];
            fillCache2 = new std::pair<int, glm::dvec3>[size];
        }
    }

    bool Rasterizer::Line(int x1, int y1, int x2, int y2, glm::dvec3 z1, glm::dvec3 z2,
                          std::pair<int, glm::dvec3>* fillCache) {

        //Liang & Barsky clipping (only top-bottom)
        int h = y2 - y1;
        double t1 = 0, t2 = 1;
        if (Test(-h, y1, t1, t2) && Test(h, viewport_height - 1 - y1, t1, t2) ) {
            glm::dvec3 z;
            int c0, c1, xp0, xp1, yp0, yp1, y, p, w;
            bool wp, hp;

            //clip line
            if (t1 > 0) {
                w = x2 - x1;
                z = z2 - z1;
                x1 += t1 * w;
                y1 += t1 * h;
                z1 += t1 * z;
            } else
                t1 = 0;
            if (t2 < 1) {
                w = x2 - x1;
                z = z2 - z1;
                t2 -= t1;
                x2 = (int) (x1 + t2 * w);
                y2 = (int) (y1 + t2 * h);
                z2 = z1 + t2 * z;
            }

            //count new line dimensions
            wp = x2 >= x1;
            w = wp ? x2 - x1 : x1 - x2;
            hp = y2 >= y1;
            h = hp ? y2 - y1 : y1 - y2;

            //line in x axis nearby
            if (w > h) {
                //direction from left to right
                xp0 = wp ? 1 : -1;
                yp0 = 0;

                //direction from top to bottom
                xp1 = wp ? 1 : -1;
                yp1 = hp ? 1 : -1;

                //line in y axis nearby
            } else {
                //direction from top to bottom
                xp0 = 0;
                yp0 = hp ? 1 : -1;

                //direction from left to right
                xp1 = wp ? 1 : -1;
                yp1 = hp ? 1 : -1;

                //apply line length
                y = w;
                w = h;
                h = y;
            }

            //count z coordinate step
            z = (z2 - z1) / (double)w;

            //Bresenham's algorithm
            c0 = h + h;
            p = c0 - w;
            c1 = p - w;
            y = y1;
            fillCache[y].first = x1;
            fillCache[y].second = z1;
            for (w--; w >= 0; w--) {

                //interpolate
                if (p < 0) {
                    p += c0;
                    x1 += xp0;
                    y1 += yp0;
                } else {
                    p += c1;
                    x1 += xp1;
                    y1 += yp1;
                }
                z1 += z;

                //write cache info
                if (wp || (y != y1)) {
                    y = y1;
                    if ((y >= 0) && (y < viewport_height)) {
                        fillCache[y].first = x1;
                        fillCache[y].second = z1;
                    }
                }
            }
            return true;
        }
        return false;
    }

    bool Rasterizer::Test(double p, double q, double &t1, double &t2) {
        //negative cutting
        if (p < 0) {
            double t = q/p;

            //cut nothing
            if (t > t2)
                return false;
                //cut the first coordinate
            else if (t > t1)
                t1 = t;

            //positive cutting
        } else if (p > 0) {
            double t = q/p;

            //cut nothing
            if (t < t1)
                return false;
                //cut the second coordinate
            else if (t < t2)
                t2 = t;

            //line is right to left(or bottom to top)
        } else if (q < 0)
            return false;
        return true;
    }

    void Rasterizer::Triangle(unsigned long& index, glm::vec3 &a, glm::vec3 &b, glm::vec3 &c, glm::vec2 ta, glm::vec2 tb, glm::vec2 tc) {

        //create markers for filling
        int min, max;
        int ab = (int) glm::abs(a.y - b.y);
        int ac = (int) glm::abs(a.y - c.y);
        int bc = (int) glm::abs(b.y - c.y);
        glm::ivec2 ia = glm::ivec2(a.x + 0.5f, a.y + 0.5f);
        glm::ivec2 ib = glm::ivec2(b.x + 0.5f, b.y + 0.5f);
        glm::ivec2 ic = glm::ivec2(c.x + 0.5f, c.y + 0.5f);
        if ((ab >= ac) && (ab >= bc)) {
            Line(ia.x, ia.y, ib.x, ib.y, glm::dvec3(ta, a.z), glm::dvec3(tb, b.z), &fillCache1[0]);
            Line(ia.x, ia.y, ic.x, ic.y, glm::dvec3(ta, a.z), glm::dvec3(tc, c.z), &fillCache2[0]);
            Line(ib.x, ib.y, ic.x, ic.y, glm::dvec3(tb, b.z), glm::dvec3(tc, c.z), &fillCache2[0]);
            min = glm::min(ia.y, ib.y);
            max = glm::max(ia.y, ib.y);
        } else if ((ac >= ab) && (ac >= bc)) {
            Line(ia.x, ia.y, ic.x, ic.y, glm::dvec3(ta, a.z), glm::dvec3(tc, c.z), &fillCache1[0]);
            Line(ia.x, ia.y, ib.x, ib.y, glm::dvec3(ta, a.z), glm::dvec3(tb, b.z), &fillCache2[0]);
            Line(ib.x, ib.y, ic.x, ic.y, glm::dvec3(tb, b.z), glm::dvec3(tc, c.z), &fillCache2[0]);
            min = glm::min(ia.y, ic.y);
            max = glm::max(ia.y, ic.y);
        } else {
            Line(ib.x, ib.y, ic.x, ic.y, glm::dvec3(tb, b.z), glm::dvec3(tc, c.z), &fillCache1[0]);
            Line(ia.x, ia.y, ib.x, ib.y, glm::dvec3(ta, a.z), glm::dvec3(tb, b.z), &fillCache2[0]);
            Line(ia.x, ia.y, ic.x, ic.y, glm::dvec3(ta, a.z), glm::dvec3(tc, c.z), &fillCache2[0]);
            min = glm::min(ib.y, ic.y);
            max = glm::max(ib.y, ic.y);
        }

        //fill triangle
        max = glm::clamp(max, 0, viewport_height - 1);
        min = glm::clamp(min, 0, viewport_height - 1);
        int centery = (max - min) / 2;
        for (int y = min; y <= max; y++) {
            //int correctedy = y < centery ? y + 1 : y - 1;
            Callback(index, y, y);
        }
        //Callback(index, centery, centery);
    }

    void Rasterizer::Callback(unsigned long& index, int& y, int& correctedy) {
        correctedy = glm::clamp(correctedy, 0, viewport_height - 1);
        int x1 = fillCache1[y].first;
        int x2 = fillCache2[y].first;
        glm::dvec3 z1 = fillCache1[y].second;
        glm::dvec3 z2 = fillCache2[y].second;

        //Liang & Barsky clipping
        double t1 = 0;
        double t2 = 1;
        int x = x2 - x1;
        if (Test(-x, x1, t1, t2) && Test(x, viewport_width - 1 - x1, t1, t2)) {

            //callback for processing
            if (x2 > x1)
                Process(index, x1, x2, correctedy, z1, z2);
            else
                Process(index, x2, x1, correctedy, z2, z1);
        }
    }
}
