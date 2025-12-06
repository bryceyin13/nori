
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class WhittedIntegrator : public Integrator {
public:
    WhittedIntegrator(const PropertyList& props) {}
    
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Intersection its;

        if (!scene->rayIntersect(ray, its)) 
            return Color3f(0.0f);
        
        Color3f Le(0.0f);
        if (its.mesh->isEmitter()) {
            // the mesh is a light source
            EmitterQueryRecord eRec(ray.o, its.p, its.shFrame.n);
            Le = its.mesh->getEmitter()->eval(eRec);
        }

        if (its.mesh->getBSDF()->isDiffuse()) {
            // randomly select an emitter 
            // every light sourse has the same possibility, which means the pdf of choosing every light source is 1 / light_num
            auto light = scene->getRandomEmitter(sampler);
            EmitterQueryRecord eRec(its.p);

            // uniformly sample light source
            Color3f Li = light->getEmitter()->sample(light, eRec, sampler);
            // shadow ray
            if (scene->rayIntersect(eRec.shadowRay)) {
                Li = 0;
            }

            float cosTheta = Frame::cosTheta(its.toLocal(eRec.wi));
            if (cosTheta < 0) {
                cosTheta = 0;
            }
            
            // -ray.d: wi, eRec.wi: wo
            BSDFQueryRecord bRec(its.toLocal(-ray.d), its.toLocal(eRec.wi), ESolidAngle);
            Color3f f = its.mesh->getBSDF()->eval(bRec);

            // pdf : 1/ number of light sources
            // corresponds to line 28: "auto light = scene->getRandomEmitter(sampler);"
            // the pdf of sampling light source surface already contains in Li
            return Le + Li * f * cosTheta / (1.0f / scene->getEmitters().size()); 
        } else {
            // dielectric 
            BSDFQueryRecord bRec(its.toLocal(-ray.d));
            Color3f color = its.mesh->getBSDF()->sample(bRec, sampler->next2D());

            // recursive
            if (sampler->next1D() < 0.95 && color.x() > 0.f) 
                return Li(scene, sampler, Ray3f(its.p, its.toWorld(bRec.wo))) / 0.95 * color;
            else 
                return Color3f(0.0f);
        }
        
        //BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d));
        //Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
        //Color3f Li = 0;
        //Ray3f rayR = Ray3f(its.p, its.shFrame.toWorld(bRec.wo), 0.0001f);
        //Intersection itsR;
        //if (scene->rayIntersect(rayR, itsR)) {
        //  if (itsR.mesh->isEmitter()) {
        //    EmitterQueryRecord lRec = EmitterQueryRecord(its.p, itsR.p, itsR.shFrame.n);
        //    Li = itsR.mesh->getEmitter()->eval(lRec);
        //  }
        // }
        // return Le + Li * f;
    }
    std::string toString() const {
        return "WhittedIntegrator[]";
    }
};
NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END