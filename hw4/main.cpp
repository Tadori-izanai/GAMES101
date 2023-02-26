#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <tuple>
#define NUM_CONTROL_POINTS 4

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < NUM_CONTROL_POINTS)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // Implement de Casteljau's algorithm
    int n = (int) control_points.size();
    if (n == 1) {
        return control_points[0];
    }
    std::vector<cv::Point2f> new_control_points(n - 1);
    for (int i = 0; i < n - 1; i++) {
        new_control_points[i] = (1 - t) * control_points[i] + t * control_points[i + 1];
    }
    return recursive_bezier(new_control_points, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.
    double t = 0.0;
    while (t <= 1.0) {
        auto point = recursive_bezier(control_points, (float) t);
        window.at<cv::Vec3b>((int) point.y, (int) point.x)[1] = 255;
        t += 0.001;
    }
}

std::tuple<int, int, int, int> get_bound(const std::vector<cv::Point2f> &control_points, int rows, int cols) {
    int row_min = rows - 1;
    int col_min = cols - 1;
    int row_max = 0;
    int col_max = 0;
    for (auto &p : control_points) {
        float px = p.x;
        float py = p.y;
        if (py < (float) row_min) {
            row_min = (int) py;
        }
        if (py > (float) row_max) {
            row_max = (int) py;
        }
        if (px < (float) col_min) {
            col_min = (int) px;
        }
        if (px > (float) col_max) {
            col_max = (int) px;
        }
    }
    return std::make_tuple(row_min, row_max, col_min, col_max);
}

void msaa_bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) {
    int n = (int) control_points.size();
    int rows = window.rows;
    int cols = window.cols;

    cv::Mat twice_window = cv::Mat(rows * 2, cols * 2, CV_8UC3, cv::Scalar(0));
    std::vector<cv::Point2f> control_points_scaled(n);
    int ind = 0;
    for (auto &point : control_points) {
        control_points_scaled[ind++] = point * 2;
    }

    double t = 0.0;
    while (t <= 1.0) {
        auto point = recursive_bezier(control_points_scaled, (float) t);
        twice_window.at<cv::Vec3b>((int) point.y, (int) point.x)[1] = 1;
        twice_window.at<cv::Vec3b>((int) point.y + 1, (int) point.x)[1] = 1;
        twice_window.at<cv::Vec3b>((int) point.y, (int) point.x + 1)[1] = 1;
        twice_window.at<cv::Vec3b>((int) point.y + 1, (int) point.x + 1)[1] = 1;
        t += 0.0005;
    }

    auto [row_min, row_max, col_min, col_max] = get_bound(control_points, rows, cols);

    for (int i = row_min; i <= row_max; i++) {
        for (int j = col_min; j <= col_max; j++) {
            unsigned char color = 0;
            color += twice_window.at<cv::Vec3b>(i * 2, j * 2)[1];
            color += twice_window.at<cv::Vec3b>(i * 2 + 1, j * 2)[1];
            color += twice_window.at<cv::Vec3b>(i * 2, j * 2 + 1)[1];
            color += twice_window.at<cv::Vec3b>(i * 2 + 1, j * 2 + 1)[1];
            color = color * 255 / 4;
            if (color) {
                window.at<cv::Vec3b>(i, j)[1] = color;
                if (window.at<cv::Vec3b>(i, j)[0]) {
                    window.at<cv::Vec3b>(i, j)[0] = 0;
                    window.at<cv::Vec3b>(i, j)[2] = 0;
                }
            } // ignore the white dots
        }
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == NUM_CONTROL_POINTS)
        {
//            naive_bezier(control_points, window);
//            bezier(control_points, window);
            msaa_bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
//            cv::imwrite("../images/output.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
