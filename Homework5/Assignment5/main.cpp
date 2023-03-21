#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "Renderer.hpp"

// In the main function of the program, we create the scene (create objects and lights)
// as well as set the options for the render (image width and height, maximum recursion
// depth, field-of-view, etc.). We then call the render function().
int main()
{
    Scene scene(1280, 960);

    // 添加球体1
    auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2);
    sph1->materialType = DIFFUSE_AND_GLOSSY; // 枚举类型，同时具有漫反射和高光反射的材料类型
    sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8);

    // 添加球体2
    auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5);
    sph2->ior = 1.5;
    sph2->materialType = REFLECTION_AND_REFRACTION; // 反射与折射

    scene.Add(std::move(sph1)); // std::move: 将左值转换为右值引用，于是sph1的所有权也被转移给了scene，sph1会被scene修改
    scene.Add(std::move(sph2));

    Vector3f verts[4] = {{-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16}}; // 定义了空间中的四个点，索引分别是0,1,2,3
    uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3}; // 定义了两个三角形，第一个三角形由索引为0,1,3的点组成；第二个三角形由索引为1,2,3的点组成
    Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}}; // st纹理坐标 = uv纹理坐标
    auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st); // std::make_unique: 智能指针
    mesh->materialType = DIFFUSE_AND_GLOSSY;

    scene.Add(std::move(mesh));
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5));
    scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5));    

    Renderer r;
    r.Render(scene);

    return 0;
}