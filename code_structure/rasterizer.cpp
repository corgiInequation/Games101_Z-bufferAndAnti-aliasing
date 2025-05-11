// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include<vector>

int count = 0;

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
   //判断是否存在三角形内，即为把(x,y)和三角形三条边进行叉乘，如果叉乘的结果符号都相同，那么就说明在三角形内
   Vector3f des(x,y,0); 
   Vector3f v1 = _v[1] - _v[0];
   Vector3f v0d = des - _v[0];
   
   Vector3f v2 = _v[2] - _v[1];
   Vector3f v1d = des - _v[1];

   Vector3f v3 = _v[0] - _v[2];
   Vector3f v2d = des - _v[2]; 
   //这里有一个容易错的地方，就是叉乘不是des直接和v1,v2,v3相乘，而是还得算出新的向量

   float z1 = v1.cross(v0d).z();
   float z2 = v2.cross(v1d).z();
   float z3 = v3.cross(v2d).z();
   if((z1 > 0 && z2 > 0 && z3 > 0) || (z1 < 0 && z2 < 0 && z3 < 0))
   {
        return true;
   }
   else
   {
        return false;
   }
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (-0.1 - (-50)) * 0.5; // f1=(n - f) / 2.0
    float f2 = (-0.1 + (-50)) * 0.5; // f2=(n + f) / 2.0

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            //这行代码有什么用处吗？屏幕又没有深度？把(-1,1)投影到(far, near)又又什么用处？
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    //v包含三角形的三个顶点，而且是4x4的
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int min_x, max_x, min_y, max_y;
    min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));
    std::vector<Eigen::Vector2f> four_pixels(4); 
    four_pixels = {
        {0.25, 0.25}, {0.25, 0.75},
        {0.75, 0.25}, {0.75, 0.75}
    };

    for(int i = min_x; i <= max_x; i++)
    {
        for(int j = min_y; j <= max_y; j++)
        {
            bool judge = false;
            for(int k = 0; k < 4; k++)
            {
                float x = (float)i + four_pixels[k][0];
                float y = (float)j + four_pixels[k][1];
                if(!insideTriangle(x, y, t.v))
                    continue;
                auto tup = computeBarycentric2D(x, y, t.v);
                float alpha;
                float beta;
                float gamma;
                std::tie(alpha, beta, gamma) = tup;
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                
                //必须得使用get_super_index和super_depth_buf和super_frame_buf，不然的话，该每着色的边界还是没有着色
                if(z_interpolated > super_depth_buf[get_super_index(i,j,k)])
                {
                    judge = true;
                    //记得更新depth_buf
                    super_depth_buf[get_super_index(i,j,k)] = z_interpolated;
                    Eigen::Vector3f color = t.getColor();
                    set_super_pixel(Vector3f(i,j,z_interpolated), color, k);
                }
            }
            if(judge)
            {
                //就是因为这个问题，卡了几乎一整天，color在初始化的时候，记得附上初值，不然就是未定义行为！！！
                Eigen::Vector3f color = {0,0,0};
                int index = get_index(i, j);
                //即使是前一个覆盖的三角形，也会加上后面三角形已经染色的另外的像素
                for(int k = 0; k < 4; k++)
                {
                    color += super_frame_buf[index*4+k];
                }
                color/=4;
                auto tup = computeBarycentric2D((float)i + 0.5, (float)j + 0.5, t.v);
                float alpha;
                float beta;
                float gamma;
                std::tie(alpha, beta, gamma) = tup;
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                set_pixel(Vector3f(i,j,z_interpolated), color);
            }
        }
    }

    for(int i = min_x; i <= max_x; i++)
    {
        for(int j = min_y; j <= max_y; j++)
        {
            int index = get_index(i,j);
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(super_frame_buf.begin(), super_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), -std::numeric_limits<float>::infinity());
        std::fill(super_depth_buf.begin(), super_depth_buf.end(), -std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    super_depth_buf.resize(w*h*4);
    super_frame_buf.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_super_index(int x, int y, int ind)
{
    //ind用来标记，当前的点是哪四个点中的第几个
    int res = ((height - 1-y)*width + x)*4 + ind;
    return res;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

void rst::rasterizer::set_super_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color, int k)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = ((height-1-point.y())*width + point.x())*4 + k;
    super_frame_buf[ind] = color;

}

// clang-format on