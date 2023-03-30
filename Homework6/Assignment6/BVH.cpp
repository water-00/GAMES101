#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    if (splitMethod == BVHAccel::SplitMethod::NAIVE)
        root = recursiveBuild(primitives);
    else
        root = recursiveBuildSAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    if (splitMethod == BVHAccel::SplitMethod::NAIVE) {
        printf("\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n", hrs, mins, secs);
    }
    else
        printf("\rSAH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n", hrs, mins, secs);

}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // 这个循环得到覆盖所有物体(primitives)的包围盒
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    // 若果包围盒中只有一个物体，做叶节点
    if (objects.size() == 1) {
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    // 如果有两个物体，分别形成子包围盒，保证每个节点最多只有一个物体
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    // 如果有多个物体，则对包围盒进行BVH划分
    else {
        Bounds3 centroidBounds; // 包含所有物体重心坐标的包围盒
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent(); // 找到覆盖范围最大的坐标轴
        // 对重心坐标在该轴上的值排序，这样下面通过objects.begin() + (objects.size() / 2)得到的middling正好就对应着重心坐标也处于中位数的物体了
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        // 根据middling进行划分
        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));
    
        // 继续划分
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}


BVHBuildNode * BVHAccel::recursiveBuildSAH(std::vector<Object *> objects) {
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    // 以上都和BVH一样，如果包围盒内物体小于等于两个就用BVH
    else {
        auto beginning=objects.begin();
        auto ending=objects.end();
        // 如果物体不超过12个，则认为用BVH比启发式算法省时
        if(objects.size() < 12) {
            auto middling = objects.begin() +objects.size()/2;
            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);
            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
        else {
            int bestChoice = 0;
            double minCost = std::numeric_limits<double >::max();
            int bestDim = 0;
            for(int dim = 0;dim < 3;dim++)
            {
                switch (dim) {
                    case 0:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().x <
                                   f2->getBounds().Centroid().x;
                        });
                        break;
                    case 1:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().y <
                                   f2->getBounds().Centroid().y;
                        });
                        break;
                    case 2:
                        std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                            return f1->getBounds().Centroid().z <
                                   f2->getBounds().Centroid().z;
                        });
                        break;
                }
            auto l = (float)objects.size();
            float nums[] = {1.0/6, 2.0/6, 3.0/6, 4.0/6, 5.0/6};
            // nums是指：在物体按重心坐标排序后的第1/6, 2/6, ...处对物体进行划分。
            // 原来按重心坐标的1/2划分，这里多试几个数，使得cost最小（也即使得其中一块区域既物体密度大又包围盒表面积小）
            for(int i = 0; i < 5; i++)
                nums[i] *= l;
            for(int i = 0; i < 5; i++) {
                auto middling = objects.begin() + (int)nums[i];
                auto leftshapes = std::vector<Object*>(beginning, middling);
                auto rightshapes = std::vector<Object*>(middling, ending);
                double leftBoxSize = computeSize(leftshapes); // 算包围盒表面积
                double rightBoxSize = computeSize(rightshapes);
                // 概率 = 划分后的表面积 / 划分前的表面积
                double cost = 100.f + (leftBoxSize*leftshapes.size() + rightBoxSize*rightshapes.size()) / bounds.SurfaceArea();
                if(cost < minCost)
                {   
                    bestChoice = (int)nums[i];
                    bestDim = dim;
                }
            }
            }
            // 在遍历完所有情况以后整理结果，根据最后的bestDim将物体按照对应的轴排序（不再对z轴排序是因为遍历结束时就是按照z轴排序的）
            if (bestDim == 0)
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().x <
                           f2->getBounds().Centroid().x;
                });
            if (bestDim == 1)
                std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                    return f1->getBounds().Centroid().y <
                           f2->getBounds().Centroid().y;
                });
            auto middling = objects.begin() + bestChoice;
            auto leftshapes = std::vector<Object*>(beginning, middling);
            auto rightshapes = std::vector<Object*>(middling, ending);
            node->left = recursiveBuild(leftshapes);
            node->right = recursiveBuild(rightshapes);
            node->bounds = Union(node->left->bounds, node->right->bounds);
    }
    return node;
    }
}

double BVHAccel::computeSize(std::vector<Object*> objects){
    Bounds3 centroidBounds;
    for (int i = 0; i < objects.size(); ++i)
        centroidBounds =
            Union(centroidBounds, objects[i]->getBounds());
    return centroidBounds.SurfaceArea();
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    if ((node == nullptr) || (node->bounds.IntersectP(ray, ray.direction_inv) == false))
        return Intersection();

    if (node->left == nullptr && node->right == nullptr) {
        // 遍历场景中node下每个物体，与光线求交，返回最近的交点
        // 但在这个项目中由于maxPrimsInNode = 1所以好像不用遍历
        return node->object->getIntersection(ray);
    }

    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);

    return (hit1.distance < hit2.distance) ? hit1 : hit2;
}