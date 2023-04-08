//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        // objcets[k]是否发光，算出所有面光源的面积和
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    // get_random_float()生成[0,1)随机数
    // p in [0, emit_area_sum)
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            // 从众多物体中随机选择一个物体，选择的思路是：每一个面光源都在[0, emit_area_sum)中占据了一段大小[A_i, A_j]
            // p落在哪段区间中，对应区间的物体就被选中，然后break
            if (p <= emit_area_sum) {
                // 对被选中的面光源，随机生成一个发射方向，更新pos属性，并设置pdf = 1/A（Sphere和Triangle有些细节上的差异，没仔细看Triangle）
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    static const double eps = 0.00016f;
    // ray is the wo_ray
    // p_inter is the intersection between ray and object plane
    // x_inter is the intersection between ray and light plane
    // ray from p to x is ws_ray
    Vector3f L_dir, L_indir;

    Vector3f wo_dir = -ray.direction; // 为什么pku代码里用的是-ray.direction？
    Intersection p_inter = intersect(ray);
    Vector3f p = p_inter.coords; // 光线wo/ray与场景中的物体相交的坐标
    Vector3f N = p_inter.normal; // p点处的法向量
    if (!p_inter.happened) {
        return Vector3f();
    }
    if (p_inter.m->hasEmission()) {
        return p_inter.m->getEmission();
    }

    Intersection x_inter;
    float pdf = 0.0f;
    sampleLight(x_inter, pdf); // 随机采样得到所有面光源中的一个点，把点和面光源记录在x_inter中，pdf = 1 / Area of 选中的面光源
    Vector3f x = x_inter.coords; // 面光源采样点坐标
    Vector3f ws_dir = (x-p).normalized(); // p到面光源上的采样点的方向向量
    float ws_distance = (x-p).norm(); // <x, p>之间的距离
    Vector3f NN = x_inter.normal; // 面光源法向量
    Vector3f emit = x_inter.emit; // 发光强度，或者说颜色？因为是个向量，所以可能是RGB？

    Ray ws_ray(p, ws_dir);
    Intersection ws_inter = intersect(ws_ray); // 判断ws是否会被遮挡
    if (ws_inter.distance + eps > ws_distance) {
        // ws_ray与场景物体相交的距离就是ws_distance，说明没有遮挡
        // 浮点数很难判断等于

        // 虽然结果对了，但是不理解eval的两个参数的顺序为什么是这样的...(交换后会在长方体侧面出现黑点)
        L_dir = emit * p_inter.m->eval(wo_dir, ws_dir, N) * dotProduct(ws_dir, N) * dotProduct(-ws_dir, NN)
                        / std::pow(ws_distance, 2) / pdf;
    }

    if (get_random_float() < RussianRoulette) {
        Vector3f wi_dir = p_inter.m->sample(-wo_dir, N).normalized();
        Ray wi_ray(p, wi_dir);
        Intersection wi_inter = intersect(wi_ray);
        if (wi_inter.happened && (!wi_inter.m->hasEmission())) {
            L_indir = castRay(wi_ray, depth + 1) * p_inter.m->eval(wo_dir, wi_dir, N) * dotProduct(wi_dir, N)
            / std::max(p_inter.m->pdf(wo_dir, wi_dir, N), 0.0001f) / RussianRoulette;
        }
    }
    return L_dir + Vector3f::Max(L_indir, 0);
}