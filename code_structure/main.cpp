// clang-format off
#include <iostream>
#include <opencv2/opencv.hpp>
#include "rasterizer.hpp"
#include "global.hpp"
#include "Triangle.hpp"
#include<vector>

constexpr double MY_PI = 3.1415926;

//把相机移动到相应位置
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

//平移，旋转，剪切，放缩物体
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    return model;
}

//投影变化
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f P2O = Eigen::Matrix4f::Identity();
    //记得把zNear和zFar改过来，它这个框架比较难绷
    //将透视投影转换为正交投影的矩阵
    printf("%f %f\n", zNear, zFar);
    P2O<<zNear, 0, 0, 0,
    0, zNear, 0, 0,
    0, 0, zNear+zFar,(-1)*zFar*zNear,
    0, 0, 1, 0;
    
    float halfEyeAngelRadian = eye_fov/2.0/180.0*MY_PI;
    //记得给zNear加上负号（取绝对值），因为高和宽肯定是正数
    float t = -zNear*std::tan(halfEyeAngelRadian);//top y轴的最高点
    float r=t*aspect_ratio;//right x轴的最大值
    float l=(-1)*r;//left x轴最小值
    float b=(-1)*t;//bottom y轴的最大值
    
    //进行一定的缩放使之成为一个标准的长度为2的正方体
    Eigen::Matrix4f ortho1=Eigen::Matrix4f::Identity();
    ortho1<<2/(r-l),0,0,0,
    0,2/(t-b),0,0,
    0,0,2/(zNear-zFar),0,
    0,0,0,1;
    // 把一个长方体的中心移动到原点
    Eigen::Matrix4f ortho2 = Eigen::Matrix4f::Identity();
    ortho2<<1,0,0,(-1)*(r+l)/2,
    0,1,0,(-1)*(t+b)/2,
    0,0,1,(-1)*(zNear+zFar)/2,
    0,0,0,1;
    Eigen::Matrix4f Matrix_ortho = ortho1 * ortho2;
    projection = Matrix_ortho * P2O;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc == 2)
    {
        command_line = true;
        filename = std::string(argv[1]);
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0,0,5};
    //相机的g和t向量没有定义，是因为其默认g为(0,0,-1), t为(0,1,0)


    //两个三角形的位置
    std::vector<Eigen::Vector3f> pos
            {
                    {3.5, -1, -5},
                    {2.5, 1.5, -5},
                    {-1, 0.5, -5},
                    {2, 0, -2},
                    {0, 2, -2},
                    {-2, 0, -2}
            };

    //两个三角形的序号，因为两个三角形没有重合的边，所以序号都是不一样的
    std::vector<Eigen::Vector3i> ind
            {
                    {0, 1, 2},
                    {3, 4, 5}
            };

    //三角形三个顶点的颜色
    std::vector<Eigen::Vector3f> cols
            {
                    {217.0, 238.0, 185.0},//黄绿
                    {217.0, 238.0, 185.0},
                    {217.0, 238.0, 185.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0},
                    {185.0, 217.0, 238.0}
            };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);
    auto col_id = r.load_colors(cols);

    int key = 0;
    //帧数统计
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);

        //其实这一整个工程，都是为了得到frame_buff的值
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
    }


    return 0;
}
// clang-format on