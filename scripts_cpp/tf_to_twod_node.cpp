#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <map>
#include <vector>
#include <string>
#include <tf/transform_listener.h>

// Drawing helpers
void drawSquare(cv::Mat& img, int x, int y, cv::Scalar color) {
    int size = 30;
    cv::rectangle(img, cv::Point(x - size, y - size), cv::Point(x + size, y + size), color, -1);
}

void drawTriangle(cv::Mat& img, int x, int y, cv::Scalar color) {
    std::vector<cv::Point> pts = { {x, y - 35}, {x - 35, y + 35}, {x + 35, y + 35} };
    cv::fillConvexPoly(img, pts, color);
}

void drawCircle(cv::Mat& img, int x, int y, cv::Scalar color) {
    cv::circle(img, cv::Point(x, y), 25, color, -1);
}

void drawMap(cv::Mat& map_image, double resolution,
             const std::vector<std::tuple<double, double, double, double>>& obstacles,
             const std::map<int, std::pair<float, float>>& nodes,
             tf::TransformListener& listener)
{
    map_image.setTo(cv::Scalar(210, 210, 210));
    cv::rectangle(map_image, cv::Point(0, 0), map_image.size(), cv::Scalar(0, 0, 0), 2);

    double origin_x = 0.0;
    double origin_y = 0.0;

    for (auto& obs : obstacles) {
        double x, y, w, h;
        std::tie(x, y, w, h) = obs;

        int x1 = static_cast<int>(x / resolution);
        int y1 = static_cast<int>(y / resolution);
        int x2 = static_cast<int>((x + w) / resolution);
        int y2 = static_cast<int>((y + h) / resolution);

        cv::rectangle(map_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 0, 0), 2);
        cv::rectangle(map_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(127, 127, 127), -1);
    }

    for (const auto& entry : nodes) {
        int id = entry.first;
        float y_world = entry.second.first;
        float x_world = entry.second.second;

        int x_px = static_cast<int>(x_world / resolution);
        int y_px = map_image.rows - static_cast<int>(y_world / resolution);

        cv::circle(map_image, cv::Point(x_px, y_px), 25, cv::Scalar(0, 255, 0), -1);
        cv::putText(map_image, std::to_string(id), cv::Point(x_px - 14 , y_px + 5 ), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    }

    std::vector<std::string> frames = {
        "robot1_base", "robot2_base", "robot3_base",
        "humanone_1_frame", "humantwo_1_frame", "humanthree_1_frame",
        "IR_camera_link", "IR_cameratwo_link"
    };

    for (const auto& frame : frames) {
        try {
            tf::StampedTransform transform;
            listener.lookupTransform("map", frame, ros::Time(0), transform);

            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();

            int x_px = static_cast<int>((x - origin_x) / resolution);
            int y_px = static_cast<int>((y - origin_y) / resolution);
            y_px = map_image.rows - y_px;

            if (frame.find("robot1") != std::string::npos) 
            {
                drawSquare(map_image, x_px, y_px, cv::Scalar(0, 165, 255));
                cv::putText(map_image, "R1", cv::Point(x_px-10, y_px+10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 3);
            } 
            else if (frame.find("robot2") != std::string::npos) 
            {
                drawSquare(map_image, x_px, y_px, cv::Scalar(0, 165, 255));
                cv::putText(map_image, "R2", cv::Point(x_px-10, y_px+10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 3);
            } 
            else if (frame.find("robot3") != std::string::npos) 
            {
                drawSquare(map_image, x_px, y_px, cv::Scalar(0, 165, 255));
                cv::putText(map_image, "R3", cv::Point(x_px-10, y_px+10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 3);
            } 
            else if (frame.find("humanone") != std::string::npos) 
            {
                drawTriangle(map_image, x_px, y_px, cv::Scalar(0, 0, 0));
                cv::putText(map_image, "P1", cv::Point(x_px-11, y_px+12), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 3);
            } 
            else if (frame.find("humantwo") != std::string::npos) 
            {
                drawTriangle(map_image, x_px, y_px, cv::Scalar(0, 0, 0));
                cv::putText(map_image, "P2", cv::Point(x_px-11, y_px+12), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 3);
            } 
            else if (frame.find("humanthree") != std::string::npos) 
            {
                drawTriangle(map_image, x_px, y_px, cv::Scalar(0, 0, 0));
                cv::putText(map_image, "P3", cv::Point(x_px-11, y_px+12), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 3);
            } 
            else if (frame.find("camera") != std::string::npos) {
                drawCircle(map_image, x_px, y_px, cv::Scalar(255, 0, 0));
            }
        } catch (tf::TransformException& ex) {
            ROS_WARN_THROTTLE(2.0, "TF error for %s: %s", frame.c_str(), ex.what());
        }
    cv::putText(map_image, "Exit", cv::Point(20 , 250), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 3);
    cv::putText(map_image, "Exit", cv::Point(1650 , 550), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 3);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "maptwo_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("maptwo2", 1);
    tf::TransformListener listener;

    double resolution = 0.01;
    int width_px = static_cast<int>(17.8 / resolution);
    int height_px = static_cast<int>(14.3 / resolution);

    std::vector<std::tuple<double, double, double, double>> obstacles = {
        {0.0, 5.2, 1, 9.1}, {1, 13.3, 2.5, 1}, {3.5, 9.9, 1, 4.4}, {4.5, 9.9, 2.5, 1}, {9.5, 9.9, 5.7, 1},
        {14.2, 10.9, 1, 3.4}, {5.7, 6.6, 9.4, 1.9}, {14.1, 5.3, 1, 1.3},
        {2.2, 0, 2, 4.4}, {4.22, 3.4, 1.3, 1}, {9.0, 3.4, 7.6, 1}, {15.6, 0, 1, 3.4}
    };

    std::map<int, std::pair<float, float>> NODES_POSITIONS = {
        {1, {2.0f, 8.3f}}, {2, {1.0f, 11.0f}}, {22, {12.4f, 6.5f}}, {3, {1.0f, 12.0f}},
        {4, {1.5f, 10.5f}}, {24, {3.0f, 2.5f}}, {5, {1.5f, 12.5f}}, {25, {3.0f, 3.0f}},
        {6, {1.5f, 8.0f}}, {26, {3.4f, 2.5f}}, {7, {0.7f, 8.0f}}, {27, {3.4f, 2.0f}},
        {8, {1.8f, 12.8f}}, {28, {4.0f, 2.5f}}, {9, {2.8f, 8.3f}}, {29, {2.0f, 2.5f}},
        {10, {4.0f, 8.3f}}, {30, {1.8f, 3.0f}}, {11, {5.0f, 8.3f}}, {31, {8.8f, 11.0f}},
        {12, {5.0f, 3.7f}}, {32, {12.5f, 9.5f}}, {13, {5.2f, 3.5f}}, {33, {12.5f, 8.0f}},
        {34, {1.8f, 2.0f}}, {15, {8.8f, 4.5f}}, {35, {12.5f, 12.5f}}, {16, {8.8f, 7.0f}},
        {36, {12.5f, 14.5f}}, {17, {10.1f, 7.0f}}, {37, {12.5f, 10.0f}}, {18, {12.2f, 7.0f}},
        {38, {6.6f, 3.3f}}, {19, {12.5f, 6.0f}}, {39, {13.5f, 7.5f}}, {20, {12.5f, 5.5f}}
    };

    ros::Rate rate(1);
    while (ros::ok()) {
        cv::Mat map_image(height_px, width_px, CV_8UC3);
        drawMap(map_image, resolution, obstacles, NODES_POSITIONS, listener);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", map_image).toImageMsg();
        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}