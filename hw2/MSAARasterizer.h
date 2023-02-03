//
// Created by lxl on 2023/2/3.
//

#ifndef RASTERIZER_MSAARASTERIZER_H
#define RASTERIZER_MSAARASTERIZER_H

#include "rasterizer.hpp"
#include <cmath>
#include <vector>

#define N 2
#define N_SUBPIXEL (N * N)

namespace rst {

    class MSAARasterizer: public rasterizer {
    protected:
        std::vector<std::vector<Eigen::Vector3f>> sub_frame_buf;
        std::vector<std::vector<float>> sub_depth_buf;

    public:
        MSAARasterizer(int w, int h);
        void clear(rst::Buffers buff) override;
        std::vector<Eigen::Vector3f>& frame_buffer() override;
    protected:
        void rasterize_triangle(const Triangle& t) override;
    };

} // rst

#endif //RASTERIZER_MSAARASTERIZER_H
