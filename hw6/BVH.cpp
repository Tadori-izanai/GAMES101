#include <algorithm>
#include <cassert>
#include "BVH.hpp"

inline int getSplitIndexBVH(const std::vector<Object *> &objects) {
    return (objects.size() / 2);
}

static int getSplitIndexSAH(const std::vector<Object *> &objects) {
    std::vector<double> cost(objects.size() - 1);
    for (int i = 0; i < cost.size(); i++) {
        Bounds3 boundLeft = Bounds3();
        Bounds3 boundRight = Bounds3();
        int countLeft = 0;
        int countRight = 0;

        for (int j = 0; j < i + 1; j++) {
            boundLeft = Union(boundLeft, objects[i]->getBounds());
            countLeft++;
        }
        for (int j = i + 1; j < objects.size(); j++) {
            boundRight = Union(boundRight, objects[i]->getBounds());
            countRight++;
        }
        cost[i] = countLeft * boundLeft.SurfaceArea() + countRight * boundRight.SurfaceArea();
    }

    double minCost = cost[0];
    int bestSplitIndex = 1;
    for (int i = 1; i < cost.size(); i++) {
        if (cost[i] < minCost) {
            minCost = cost[i];
            bestSplitIndex = i + 1;
        }
    }
    return bestSplitIndex;
}

static int getSplitIndexSAH2(const std::vector<Object *> &objects) {
    auto n = objects.size();
    std::vector<Bounds3> mergeFromLeft(n);
    std::vector<Bounds3> mergeFromRight(n);

    Bounds3 tmp = Bounds3();
    for (int k = 0; k < n; k++) {
        tmp = Union(tmp, objects[k]->getBounds());
        mergeFromLeft[k] = tmp;
    }
    tmp = Bounds3();
    for (int k = 0; k < n; k++) {
        tmp = Union(tmp, objects[n - 1 - k]->getBounds());
        mergeFromRight[k] = tmp;
    }

    std::vector<double> cost(n - 1);
    for (int i = 0; i < cost.size(); i++) {
        Bounds3 boundLeft = mergeFromLeft[i];
        Bounds3 boundRight = mergeFromRight[cost.size() - 1 - i];
        int countLeft = i + 1;
        int countRight = n - countLeft;
        cost[i] = countLeft * boundLeft.SurfaceArea() + countRight * boundRight.SurfaceArea();
    }

    double minCost = cost[0];
    int bestSplitIndex = 1;
    for (int i = 1; i < cost.size(); i++) {
        if (cost[i] < minCost) {
            minCost = cost[i];
            bestSplitIndex = i + 1;
        }
    }
    return bestSplitIndex;
}

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    std::string name;
    switch (splitMethod) {
        case SplitMethod::NAIVE:
            root = recursiveBuild(primitives, getSplitIndexBVH);
            name = "BVH";
            break;
        case SplitMethod::SAH:
//            root = recursiveBuild(primitives, getSplitIndexSAH);
            root = recursiveBuild(primitives, getSplitIndexSAH2);
            name = "SAH";
            break;
    }
//    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\r%s Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        name.c_str(), hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects, int f(const std::vector<Object *> &objects))
{
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
        node->left = recursiveBuild(std::vector{objects[0]}, f);
        node->right = recursiveBuild(std::vector{objects[1]}, f);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
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

        int splitIndex = f(objects);

        auto beginning = objects.begin();
        auto middling = objects.begin() + splitIndex;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes, f);
        node->right = recursiveBuild(rightshapes, f);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
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

    // misses node
    assert(node != nullptr);

    // out of bound
    Vector3f dir = ray.direction;
    Vector3f invDir = ray.direction_inv;
    std::array<int, 3> dirIsNeg = {int(dir.x > 0), int(dir.y > 0), int(dir.z > 0)};
    if (!node->bounds.IntersectP(ray, invDir, dirIsNeg)) {
        return {};
    }

    // leaf node
    Object *o = node->object;
    if (o != nullptr) {
        return o->getIntersection(ray);
    }

    // non-leaf node
    Intersection hitLeft = getIntersection(node->left, ray);
    Intersection hitRight = getIntersection(node->right, ray);

    return (hitRight.distance < hitLeft.distance) ? hitRight : hitLeft;
}
