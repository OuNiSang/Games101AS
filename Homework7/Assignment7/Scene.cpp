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
    //filter out all the self emiting object to obtain sum of emiting area 
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    //Calculate probability density for sample point
    //NOTE: get_random_float() has altered 
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0; //QA: why need to setback to zero again in here? 
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
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
//ref https://blog.csdn.net/qq_36242312/article/details/116307626?utm_term=games101%E4%BD%9C%E4%B8%9A7%E5%A4%9A%E7%BA%BF%E7%A8%8B&utm_medium=distribute.pc_aggpage_search_result.none-task-blog-2~all~sobaiduweb~default-0-116307626&spm=3001.4430
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    //---------Init---------
    Intersection hitObject = Scene::intersect(ray);
    Vector3f L_dir(0,0,0);          //Direct Light 
    Vector3f L_indir(0,0,0);        //Indirect Light

    //---------Inital Check for lightSourceHit condition---------
    if(!hitObject.happened) return Vector3f(0,0,0);
    if(hitObject.m->hasEmission()) return depth == 0 ? hitObject.m->getEmission() : Vector3f(0,0,0);//Check if itself a light, if hit light source in the first time, just return
    
    //---------Light Sampleing---------
    Intersection hitLight;
    float Pdf_l = 0.0f;
    Scene::sampleLight(hitLight, Pdf_l);

    //---------Param Generate---------
    auto& N_O = hitObject.normal;   //normal vector of object surface
    auto& N_L = hitLight.normal;    //normal vector of light surface

    auto& Pos_O = hitObject.coords; //Position of object
    auto& Pos_L = hitLight.coords;  //Position of light

    //Cal basic para 
    auto diff = Pos_L - Pos_O;
    auto Dir_L = diff.normalized();
    float distance = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
    
    //Generate a path from light to object
    Ray path(Pos_O,Dir_L); 
    Intersection light2Object = Scene::intersect(path);

    //---------Direct light Calculation--------- 
    //if the ray is not blocked in the middle
    //NOTE: func of "(light2Object.coords - Pos_L).norm() < 1e-2"
    //norm((a,b)) will return a*a + b*b, magnitude of a vector? 
    //"1e-2" means 1*10^-2
    //Therefore, this statement means filte rout small distance of displace of light  
    if(light2Object.happened && (light2Object.coords - Pos_L).norm() < 1e-2)
    {
        //evaluate the contribution of this ray
        Vector3f f_r = hitObject.m->eval(ray.direction,Dir_L,N_O);
        //NOTE: direct lighting calculation
        //L_dir = L_i * f_r * cosine(theta) * cosine(theta_prime) / absolute(x_prime - p) ^ 2 / pdf_light
        L_dir = hitLight.emit * f_r * dotProduct(Dir_L,N_O) * dotProduct(-Dir_L, N_L) / distance / Pdf_l;
    }
    
    //---------Indirect light Calculation--------- 
    if(get_random_float() < RussianRoulette)
    {
        //init param for next bance cal
        Vector3f Dir_Next = hitObject.m->sample(ray.direction, N_O).normalized();
        Ray path_Next(Pos_O, Dir_Next);
        Intersection hitNext = intersect(path_Next);

        if (hitNext.happened && !hitNext.m->hasEmission())
        {
            float Pdf_Hemi = hitObject.m->pdf(ray.direction,Dir_Next,N_O);
            Vector3f f_r = hitObject.m->eval(ray.direction, Dir_Next,N_O);
            //NOTE: indirect lighting calculation
            //L_indir = shade(q, -wi) * f_r * cosine(theta) / pdf_hemi / P_RR
            L_indir = castRay(path_Next,depth+1) * f_r * dotProduct(Dir_Next, N_O) / Pdf_Hemi / RussianRoulette;
        }
        
    }

    return L_dir + L_indir;
    
}