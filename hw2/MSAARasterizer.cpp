//
// Created by lxl on 2023/2/3.
//

#include "MSAARasterizer.h"
#include "rasterizer.hpp"

bool insideTriangle(float x, float y, const Vector3f* _v);
std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v);

#define MINMAX_AMONG_THREE(A, B, C, OPERATOR) ( \
    (A) OPERATOR (B) && (A) OPERATOR (C) ? (A) : ( \
        (B) OPERATOR (C) ? (B) : (C) \
    )\
)


rst::MSAARasterizer::MSAARasterizer(int w, int h) : rasterizer(w, h) {
    sub_depth_buf.resize(w * h);
    sub_frame_buf.resize(w * h);
    for (auto & iter : sub_frame_buf) {
        iter.resize(N_SUBPIXEL);
    }
    for (auto & iter : sub_depth_buf) {
        iter.resize(N_SUBPIXEL);
    }
}

void rst::MSAARasterizer::rasterize_triangle(const Triangle &t) {
    auto v = t.toVector4();

    Eigen:: Vector3f A(t.v[0]);
    Eigen:: Vector3f B(t.v[1]);
    Eigen:: Vector3f C(t.v[2]);
    float min_x = MINMAX_AMONG_THREE(A.x(), B.x(), C.x(), <);
    float max_x = MINMAX_AMONG_THREE(A.x(), B.x(), C.x(), >);
    float min_y = MINMAX_AMONG_THREE(A.y(), B.y(), C.y(), <);
    float max_y = MINMAX_AMONG_THREE(A.y(), B.y(), C.y(), >);

    for (int i = (int)min_x; i <= (int)max_x; i++) {
        for (int j = (int)min_y; j <= (int)max_y; j++) {
            for (int ii = 0; ii < N; ii++) {
                for (int jj = 0; jj < N; jj++) {
                    float x = (float)i + ((float)ii + .5f) * (1.0f / N);
                    float y = (float)j + ((float)jj + .5f) * (1.0f / N);

                    if (insideTriangle(x, y, t.v)) {
                        auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                        float w_reciprocal = 1.0f/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        if (z_interpolated < sub_depth_buf[get_index(i, j)][ii * N + jj]) {
                            sub_depth_buf[get_index(i, j)][ii * N + jj] = z_interpolated;
                            sub_frame_buf[get_index(i, j)][ii * N + jj] = t.getColor();
                        }
                    }
                } // subpixel
            }
        } // pixel
    }
}

void rst::MSAARasterizer::clear(rst::Buffers buff) {
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::vector<Eigen::Vector3f> pad(N_SUBPIXEL, Eigen::Vector3f(0, 0, 0));
        std::fill(sub_frame_buf.begin(), sub_frame_buf.end(), pad);
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::vector<float> pad(N_SUBPIXEL, std::numeric_limits<float>::infinity());
        std::fill(sub_depth_buf.begin(), sub_depth_buf.end(), pad);
    }
}

inline Eigen::Vector3f frame_mean(const std::vector<Eigen::Vector3f> &p) {
    Eigen::Vector3f sum(0.0f, 0.0f, 0.0f);
    for (auto & color : p) {
        sum += color;
    }
    return sum / p.size();
}

std::vector<Eigen::Vector3f> &rst::MSAARasterizer::frame_buffer() {
    for (int i = 0; i < frame_buf.size(); i++) {
        frame_buf[i] = frame_mean(sub_frame_buf[i]);
    }
    return frame_buf;
}

