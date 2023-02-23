//
// Created by LEI XU on 4/27/19.
//

#include "Texture.hpp"

Eigen::Vector3f Texture::getColorBilinear(float u, float v) {
    float u_img = u * width;
    float v_img = (1 - v) * height;

    int u00 = std::floor(u_img);
    int v00 = std::floor(v_img);
    float s = u_img - (float)u00;
    float t = v_img - (float)v00;

    auto color00 = image_data.at<cv::Vec3b>(v00, u00);
    auto color10 = color00;
    auto color01 = color00;
    auto color11 = color00;
    if (u00 + 1 < width  && v00 + 1 < height) {
        color10 = image_data.at<cv::Vec3b>(v00, u00 + 1);
        color01 = image_data.at<cv::Vec3b>(v00 + 1, u00);
        color11 = image_data.at<cv::Vec3b>(v00 + 1, u00 + 1);
    } else if (u00 + 1 >= width  && v00 + 1 < height) {
        color01 = image_data.at<cv::Vec3b>(v00 + 1, u00);
        color11 = image_data.at<cv::Vec3b>(v00 + 1, u00);
    } else if (u00 + 1 < width  && v00 + 1 >= height) {
        color10 = image_data.at<cv::Vec3b>(v00, u00 + 1);
        color11 = image_data.at<cv::Vec3b>(v00, u00 + 1);
    }

    cv::Vec<unsigned char, 3> color = (1-s)*(1-t)*color00 + s*(1-t)*color10 + t*(1-s)*color01 + s*t*color11;
    return Eigen::Vector3f(color[0], color[1], color[2]);
}
