#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

//#define EXTRA_PART

constexpr double MY_PI = 3.1415926;

#ifdef EXTRA_PART
const Eigen::Vector3f X_AXIS(1, 0, 0);
const Eigen::Vector3f Y_AXIS(0, 1, 0);
const Eigen::Vector3f Z_AXIS(0, 0, 1);
#endif

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;
    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float rotation_angle_rad = (float)MY_PI / 180 * rotation_angle;
    model << std::cos(rotation_angle_rad), -std::sin(rotation_angle_rad), 0, 0,
             std::sin(rotation_angle_rad), std::cos(rotation_angle_rad), 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // Create the projection matrix for the given parameters.
    // Then return it.

    /* perspective to orthographic */
    Eigen::Matrix4f perspective_to_orthographic;
    perspective_to_orthographic <<
        zNear, 0, 0, 0,
        0, zNear, 0, 0,
        0, 0, (zNear + zFar), -zNear * zFar,
        0, 0, 1, 0;

    /* scale */
    float eye_fov_rad = (float)MY_PI / 180 * eye_fov;
    float t = -zNear * std::tan(eye_fov_rad / 2);
    Eigen::Matrix4f scale = Eigen::Vector4f(
        1.0f / (t * aspect_ratio), 1.0f / t, 2.0f / (zNear - zFar), 1.0f
    ).asDiagonal();

    /* translation */
    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
    translation(2, 3) = -(zNear + zFar) / 2;

    Eigen::Matrix4f orthographic = scale * translation;
    projection = orthographic * perspective_to_orthographic * projection;

    return projection;
}

#ifdef EXTRA_PART
Eigen::Matrix4f get_rotation(const Eigen::Vector3f &axis, float angle) {
    float angle_rad = (float)MY_PI / 180 * angle;

    Eigen::Vector3f axis_normalized = axis / axis.norm();
    float x = axis_normalized.x();
    float y = axis_normalized.y();
    float z = axis_normalized.z();

    Eigen::Matrix3f outer;
    outer << 0, -z, y,
            z, 0, -x,
            -y, x, 0;

    Eigen::Matrix3f rotation = std::cos(angle_rad) * Eigen::Matrix3f::Identity()
                                + (1.0 - std::cos(angle_rad)) * axis_normalized * axis_normalized.transpose()
                                + std::sin(angle_rad) * outer;

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    model.block<3, 3>(0, 0) = rotation;
    return model;
}
#endif

int main(int argc, const char** argv)
{
    float angle = 0;

#ifdef EXTRA_PART
    Eigen::Vector3f axis = Z_AXIS;
#endif

    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

#ifndef EXTRA_PART
        r.set_model(get_model_matrix(angle));
#else
        r.set_model(get_rotation(axis, angle));
#endif
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
#ifdef EXTRA_PART
        else if (key == 'x') {
            angle = 0;
            axis = X_AXIS;
        } else if (key == 'y') {
            angle = 0;
            axis = Y_AXIS;
        } else if (key == 'z') {
            angle = 0;
            axis = Z_AXIS;
        }
#endif
    }
    return 0;
}
