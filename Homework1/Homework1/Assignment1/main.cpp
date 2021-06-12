#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
inline double RadianConvert(float degree){return degree * MY_PI / 180;}
inline float  OrthSFuc(float a, float b){return 2 / (a - b);}

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
    double tempRad = RadianConvert(rotation_angle);
    Eigen::Matrix4f zRotation;
    zRotation << cos(tempRad), -sin(tempRad), 0, 0,
                 sin(tempRad), cos(tempRad),  0, 0,
                 0,                     0,    1, 0,
                 0,                     0,    0, 1; //z axis roation matrix from lec 4 P8

    model = zRotation;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // Create the projection matrix for the given parameters.
    // Then return it.
    
    //P_Note*
    //just notice that the aspect_ratio is differ from the n and f that mentioned in Lec 4 P30
    //Aspect ratio should be in Lec5 P5 which is related to the FOV and convert y to x axis 
        
    Eigen::Matrix4f perspToOrthoMatrix, otrthoMatrix, perspMatrix;
    Eigen::Matrix4f orthScaleMatrix, orthTranMatrix;
    float fovCovert = RadianConvert(eye_fov) / 2;

    //for z axis
    float n = zNear, f = zFar;
    
    //for y axis
    float yNear = zNear * tan(fovCovert);   //same as t
  
    //for x axis 
    float xNear = yNear * aspect_ratio;     //same as r


    float A = n + f;
    float B = -n * f;
    perspToOrthoMatrix<<n,0,0,0,    0,n,0,0,    0,0,A,B,    0,0,1,0;            //ref Lec4 P33~P36

    orthTranMatrix<<1,0,0,0,    0,1,0,0,    0,0,1,-(n+f)/2,     0,0,0,1;        //ref Lec4 P24
    
    orthScaleMatrix<<1/xNear,0,0,0,   0,1/yNear,0,0,    0,0,1/(zNear-zFar),0,    0,0,0,1;   //ref Lec4 P24

    otrthoMatrix = orthScaleMatrix * orthTranMatrix;
    perspMatrix = otrthoMatrix * perspToOrthoMatrix;
    projection = perspMatrix * projection;


    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    //create a function that can return a rotation matrix with any given axis and angle
    //there are two ways to implement this

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    //#1 Rodrigues' rotation formula
    Eigen::Matrix3f rodriguesRotation = Eigen::Matrix3f::Identity();
    float tempRad = RadianConvert(angle);
    Eigen::Matrix3f N;
    N  << 0, -axis.z(), axis.y(),
            axis.z(), 0, -axis.x(),
            -axis.y(), axis.x(), 0;
       
    rodriguesRotation = cos(tempRad)*rodriguesRotation + 
                        (1-cos(tempRad))*axis*axis.transpose() + 
                        sin(tempRad) * N;

    model.block<3, 3>(0, 0) = rodriguesRotation; //transfer 3x3 matrix into I 4x4 matrix
    return model;    

    // NOTE: #2 三个轴相加的方法, 参考,code来自https://github.com/kingiluob/Games101/blob/master/Assignment1/main.cpp
    // float angle_x,angle_y,angle_z;
    // float length = sqrt(axis.x() * axis.x() + axis.y()*axis.y()+axis.z()*axis.z());
    // angle_x = std::acos(axis.x()/length);
    // angle_y = std::acos(axis.y()/length);
    // angle_z = std::acos(axis.z()/length);
    // Eigen::Matrix4f m1,m2,m3  = Eigen::Matrix4f::Identity();
    // m1<<1,0,0,0,0,cos(angle_x),-sin(angle_x),0,0,sin(angle_x),cos(angle_x),0,0,0,0,1;
    // m2<<cos(angle_y),0,sin(angle_y),0,0,1,0,0,-sin(angle_y),0,cos(angle_y),0,0,0,0,1;
    // m3<<cos(angle_z),-sin(angle_z),0,0,sin(angle_z),cos(angle_z),0,0,0,0,1,0,0,0,0,1;

    // Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    // rotation =m3*m2*m1*Eigen::Matrix4f::Identity();
    // return rotation;
}

int main(int argc, const char** argv)
{
    float angle = 0;
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

        r.set_model(get_model_matrix(angle));
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
    }

    return 0;
}
