#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

#define PI 3.1415926
using namespace std;

constexpr double MY_PI = 3.1415926;

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
    model(0,0) = cos(rotation_angle * PI / 180.0f);
    model(0,1) = -sin(rotation_angle * PI / 180.0f);
    model(1,0) = sin(rotation_angle * PI / 180.0f);
    model(1,1) = cos(rotation_angle * PI / 180.0f);

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Identity();
    persp2ortho(0,0) = zNear;
    persp2ortho(1,1) = zNear;
    persp2ortho(3,2) = 1.0f;
    persp2ortho(3,3) = 0.0f;

    persp2ortho(2,2) = zNear + zFar;
    persp2ortho(2,3) = - zNear * zFar;

    Eigen::Matrix4f Mortho_s = Eigen::Matrix4f::Identity();
    //Mortho_t = ;
    float t = -tan(eye_fov * PI/360.0) * zNear; // 1/2 eye_fov
    
    float r = aspect_ratio * t;
    Mortho_s(0,0) = 1.0f/r;
    Mortho_s(1,1) = 1.0f/t;
    Mortho_s(2,2) = 2.0f/(zNear - zFar);

    Eigen::Matrix4f Mortho_t = Eigen::Matrix4f::Identity();
    Mortho_t(2,3) = - 0.5f * (zNear + zFar);

    projection = Mortho_s*Mortho_t*persp2ortho; // first to a cube, then translate, them scale

    return projection;
}
/*
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float eye_angle = eye_fov *PI / 180;
    float t,b,l,r;
    t = zNear * tan(eye_angle /2);
    r = t * aspect_ratio;
    l = -r;
    b = -t;
    Eigen::Matrix4f PersToOrth = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m1;
    Eigen::Matrix4f m2;
    Eigen::Matrix4f m3;
    m1<< zNear,0,0,0,0,zNear,0,0,0,0,zNear + zFar,-zNear*zFar,0,0,1,0;
    m2<<1,0,0,0,0,1,0,0,0,0,1,-(zNear+ zFar)/2 ,0,0,0,1;
    m3<<2/(r-l),0,0,0,0,2/(t-b),0,0,0,0,2/(zNear -zFar),0,0,0,0,1;
    projection = m3 * m2 * m1 * projection;

    return projection;
}
*/
int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    vector<Eigen::Vector3i> ind{{0, 1, 2}};

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

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
