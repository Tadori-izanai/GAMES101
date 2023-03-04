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
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TODO Implement Path Tracing Algorithm here
    if (depth > maxDepth) {
        return {0.f};
    }

    Intersection interP = intersect(ray);
    if (!interP.happened) {
        return {0.f};
    }

    Vector3f omegaO = -normalize(ray.direction);
    Material *m = interP.m;

    // check whether the intersection is on the light
    if (m->hasEmission()) {
        return {1.f};
    }

    /* Contribute from the light source, directL */
    Vector3f directL(0.f);
    Intersection interLight;
    float pdfLight;
    sampleLight(interLight, pdfLight);

    Vector3f x = interLight.coords;
    Vector3f N = normalize(interP.normal);
    Vector3f p = interP.coords;
    Vector3f omegaS = normalize(x - p);

    // check whether the ray is blocked in the middle
    Intersection interS = intersect(Ray(p, omegaS));
    if (interS.happened && interS.m->hasEmission()) {
        Vector3f NN = normalize(interS.normal);
        Vector3f emit = interS.m->getEmission();
        Vector3f fr = m->eval(omegaS, omegaO, N);
        float d = (x - p).norm();
        directL += emit * fr * dotProduct(omegaS, N) * dotProduct(-omegaS, NN) / (d * d) / pdfLight;
    }

    /* Contribute from other reflection, lIndirect */
    Vector3f indirectL(0.f);
    // test Russian Roulette with probability RussianRoulette
    if (get_random_float() < RussianRoulette) {
        Vector3f omegaI = m->sample(omegaO, N);

        // check whether the ray r hit a non-emitting object q
        Ray r(p, omegaI);
        Intersection interI = intersect(r);
        if (interI.happened && !interI.m->hasEmission()) {
            Vector3f fr = m->eval(omegaI, omegaO, N);
            float pdfQ = m->pdf(omegaI, omegaO, N);
            indirectL += castRay(r, depth) * fr * dotProduct(omegaI, N) / pdfQ / RussianRoulette;
        }
    }

    return directL + indirectL;
}
