#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"


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

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Copy-paste your implementation from the previous assignment.
    // 按顺序乘以这些矩阵得到projection
    Eigen::Matrix4f persp_to_ortho = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float n, f, H, W;
    n = zNear;
    f = zFar;
    H = -zNear * tan(eye_fov / 180 * MY_PI / 2);
    W = H * aspect_ratio;

    persp_to_ortho << n, 0, 0, 0,
                    0, n, 0, 0, 
                    0, 0, n+f, -n*f,
                    0, 0, 1, 0;

    translation << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, -(f+n)/2,
                    0, 0, 0, 1;
        
    scale << 1/W, 0, 0, 0, // project [-W, W]*[-H, H]*[f, n] to [-1, 1]*[-1, 1]*[-1, 1]
            0, 1/H, 0, 0,
            0, 0, 2/(n-f), 0,
            0, 0, 0, 1;
    projection = scale * translation * persp_to_ortho;
    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        float u = payload.tex_coords(0);
        float v = payload.tex_coords(1);
        return_color = payload.texture->getColor(u, v);
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005); // 环境光系数
    Eigen::Vector3f kd = texture_color / 255.f; // 漫反射系数，受材质影响
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937); // 高光系数

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f l, v, h, n, I;
    Eigen::Vector3f la, ls, ld;
    la = ka.cwiseProduct(amb_light_intensity); // 环境光项

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        l = (light.position - point).normalized(); // 点->光源 向量
        v = (eye_pos - point).normalized(); // 点->相机 向量
        float rr = ((light.position - point).array().square()).sum(); // 光源到点的距离的平方
        //         float r2 = (light.position - point).dot(light.position - point);
        h = ((l + v)).normalized(); // 半程向量
        n = normal;
        I = light.intensity;

        ld = kd.cwiseProduct(I / rr) * std::max(0.0f, n.dot(l)); // 漫反射项
        ls = ks.cwiseProduct(I / rr) * std::pow(std::max(0.0f, n.dot(h)), p); // 高光项

        result_color += ld + ls + la;
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005); // 环境光项系数
    Eigen::Vector3f kd = payload.color; // 漫反射系数
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937); // 高光项系数

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10}; // 相机位置

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    Eigen::Vector3f l, v, h, n, I;
    Eigen::Vector3f la, ls, ld;
    la = ka.cwiseProduct(amb_light_intensity); // 环境光项

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        l = (light.position - point).normalized(); // 点->光源 向量
        v = (eye_pos - point).normalized(); // 点->相机 向量
        float rr = ((light.position - point).array().square()).sum(); // 光源到点的距离的平方
        //         float r2 = (light.position - point).dot(light.position - point);
        h = ((l + v)).normalized(); // 半程向量
        n = normal;
        I = light.intensity;

        ld = kd.cwiseProduct(I / rr) * std::max(0.0f, n.dot(l)); // 漫反射项
        ls = ks.cwiseProduct(I / rr) * std::pow(std::max(0.0f, n.dot(h)), p); // 高光项

        result_color += ld + ls + la;
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    Eigen::Vector3f n = normal;
    float x = n(0), y = n(1), z = n(2);
    Eigen::Vector3f t = {x*y/std::sqrt(x*x+z*z), std::sqrt(x*x+z*z), z*y/std::sqrt(x*x+z*z)};
    Eigen::Vector3f b = n.cross(t);
    Eigen::Matrix3f TBN; // t b n对应于TBN的三列
    TBN.col(0) = t;
    TBN.col(1) = b;
    TBN.col(2) = n;
    float u = payload.tex_coords.x(), v = payload.tex_coords.y();
    float w = payload.texture->width, h = payload.texture->height;
    // 该texel在u, v方向的斜率大小（梯度？）
    float dU = kh * kn * (payload.texture->getColor(u + 1.0f/w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f/h).norm() - payload.texture->getColor(u, v).norm());
    Eigen::Vector3f ln = {-dU, -dV, 1};
    // 与凹凸贴图的区别就在于这句话，真实地改变了该点的高度（通过改变相机的位置实现）
    point += (kn * normal * payload.texture->getColor(u, v).norm());
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};

    Eigen::Vector3f l, view, bisector, I;
    Eigen::Vector3f la, ls, ld;
    la = ka.cwiseProduct(amb_light_intensity); // 环境光项

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        l = (light.position - point).normalized(); // 点->光源 向量
        view = (eye_pos - point).normalized(); // 点->相机 向量
        float rr = ((light.position - point).array().square()).sum(); // 光源到点的距离的平方
        //         float r2 = (light.position - point).dot(light.position - point);
        bisector = ((l + view)).normalized(); // 半程向量
        I = light.intensity;

        ld = kd.cwiseProduct(I / rr) * std::max(0.0f, normal.dot(l)); // 漫反射项
        ls = ks.cwiseProduct(I / rr) * std::pow(std::max(0.0f, normal.dot(bisector)), p); // 高光项

        result_color += ld + ls + la;
    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)

    Eigen::Vector3f n = normal;
    float x = n(0), y = n(1), z = n(2);
    Eigen::Vector3f t = {x*y/std::sqrt(x*x+z*z), std::sqrt(x*x+z*z), z*y/std::sqrt(x*x+z*z)};
    Eigen::Vector3f b = n.cross(t);
    Eigen::Matrix3f TBN; // t b n对应于TBN的三列
    TBN.col(0) = t;
    TBN.col(1) = b;
    TBN.col(2) = n;
    float u = payload.tex_coords.x(), v = payload.tex_coords.y();
    float w = payload.texture->width, h = payload.texture->height;
    // 该texel在u, v方向的斜率大小（梯度？）
    float dU = kh * kn * (payload.texture->getColor(u + 1.0f/w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f/h).norm() - payload.texture->getColor(u, v).norm());
    Eigen::Vector3f ln = {-dU, -dV, 1};
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    // 文件目录
    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // 加载.obj文件
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    // 遍历模型的每个面
    for(auto mesh:Loader.LoadedMeshes)
    {
        // 记录图形每个面中连续三个顶点（小三角形）
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            // 把每个小三角形的顶点信息记录在Triangle类t中
            for(int j=0;j<3;j++)
            {
                // 顶点第四维w = 1.0
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));// 记录顶点位置
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));// 记录顶点法线
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));// 记录顶点纹理
            }
            TriangleList.push_back(t);// 三角形信息加入列表
        }
    }

    rst::rasterizer r(700, 700);// 构造光栅化对象

    // 记录纹理到光栅化
    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;// 定义shader的function对象

    // 输入处理 用参数设置shader的function对象或设置纹理
    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the displacement shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);// 设置顶点着色方式
    r.set_fragment_shader(active_shader);// 设置片元着色方式

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);// 清空缓冲区
        // 分别得到MVP变换矩阵
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

		// 应用变换矩阵 并进行光栅化、片元处理、帧缓冲
        r.draw(TriangleList);
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
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
