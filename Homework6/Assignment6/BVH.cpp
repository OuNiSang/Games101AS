#include <algorithm>
#include <cassert>
#include <map>
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

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
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
    else if (objects.size() == SAH_MIN_OBJECT_COUNT) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i) //cal the largest BB
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());

        std::vector<Object *> leftshapes, rightshapes;

        switch (splitMethod)
        {
        case SplitMethod::NAIVE:
            {
                /* origin split method */
                int dim = centroidBounds.maxExtent(); //find the largetest steps in x,y or z axis
                //sorting based on largetest steps 
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

                leftshapes = std::vector<Object*>(beginning, middling);
                rightshapes = std::vector<Object*>(middling, ending);
                break;
            }
        
        case SplitMethod::SAH:
            {
                /*SAH method*/ //ref: https://www.cnblogs.com/coolwx/p/14375763.html
                // Bounds3 numBounds;
                // for (int i = 0; i < objects.size(); i++)
                // {
                //     numBounds = Union(numBounds, objects[i]->getBounds());
                // }

                float numArea = centroidBounds.SurfaceArea();
                int minCostCoor = 0;    //record the min cost Axis, x, y, or z
                int minCostIndex = 0; //record the min cost buket index
                float minCost = std::numeric_limits<float>::infinity(); //for record of min Cost of SAH, pick largest at fist 
                std::map<int, std::map<int, int>> indexMap; //record the axis with corresbounding map

                // go through all 3 axis x, y and z
                for (int i = 0; i < 3; i++)
                {
                    std::vector<Bounds3> boundsBukets;
                    std::vector<int> countBuket;
                    for (int j = 0; j < SAH_BUCKET_COUNT; j++)
                    {
                        /* init boundsbukets and counter for bukets */
                        boundsBukets.emplace_back(Bounds3());
                        countBuket.emplace_back(0);
                    }
                    std::map<int, int> objMap; //TODO

                    for (int k = 0; k < objects.size(); k++)
                    {
                        /* cal buketID and start add them to boundsBukets */
                        int _buketID = 0;
                        if(i = 0)_buketID = SAH_BUCKET_COUNT * (centroidBounds.Offset(objects[k]->getBounds().Centroid())).x;
                        if(i = 1)_buketID = SAH_BUCKET_COUNT * (centroidBounds.Offset(objects[k]->getBounds().Centroid())).y;
                        if(i = 2)_buketID = SAH_BUCKET_COUNT * (centroidBounds.Offset(objects[k]->getBounds().Centroid())).z;

                        if(_buketID > SAH_BUCKET_COUNT -1) _buketID = SAH_BUCKET_COUNT - 1;
                        Bounds3 tempBuket = boundsBukets[_buketID];
                        tempBuket = Union(tempBuket, objects[k]->getBounds().Centroid());
                        boundsBukets[_buketID] = tempBuket;
                        countBuket[_buketID] += 1;
                        objMap.insert(std::make_pair(k,_buketID));
                    }

                    indexMap.insert(std::make_pair(i, objMap));

                    for (int l = 1; l < boundsBukets.size(); l++)
                    {
                        /* cal the cost */
                        Bounds3 A, B;
                        int countA = 0;
                        int countB = 0;
                        float invNArea = 1.0f / numArea;
                        for (int j = 0; j < l; j++)
                        {
                            A = Union(A, boundsBukets[j]);
                            countA += countBuket[j];
                        }
                        for (int k = l; k < boundsBukets.size(); k++)
                        {
                            B = Union(B, boundsBukets[k]);
                            countB += countBuket[k];
                        }

                        float pA = A.SurfaceArea() * invNArea;
                        float pB = B.SurfaceArea() * invNArea;
                        float cost = SAH_INTERSECTION_COST +                //NOTE: core calculation function!
                                    countA * pA * SAH_TRAVERSAL_COST +
                                    countB * pB * SAH_TRAVERSAL_COST;
                        
                        if (cost < minCost) //update the min cost
                        {
                            minCost = cost;
                            minCostIndex = l;
                            minCostCoor = i;
                        }
                    } 
                }
                for (int i = 0; i < objects.size(); i++)
                {
                    if(indexMap[minCostCoor][i] < minCostIndex) leftshapes.emplace_back(objects[i]);
                    else rightshapes.emplace_back(objects[i]);
                }
            }
            break;
        }
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

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
    //init param for IntersectP
    Vector3f invDir = Vector3f{1.0f / ray.direction.x, 1.0f / ray.direction.y ,1.0f / ray.direction.z};
    std::array<int,3> dirIsNeg = {ray.direction.x > 0, ray.direction.y > 0, ray.direction.z > 0};
    
    if(!node->bounds.IntersectP(ray,invDir,dirIsNeg))
    {
        Intersection _empty;
        return _empty;
    }

    //hit the leaf node, which is an object
    if(node->left == nullptr && node->right == nullptr) return node->object->getIntersection(ray);

    //not hitting the leaf node, keep recursive on right and left node
    Intersection _r = getIntersection(node->right, ray);
    Intersection _l = getIntersection(node->left, ray);

    return _r.distance < _l.distance ? _r : _l;

}