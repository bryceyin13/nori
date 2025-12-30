#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class PathEmsIntegrator : public Integrator {
public:
    PathEmsIntegrator(const PropertyList& props) {}
    
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        float eta = 1.0f;
        Color3f throughput(1.0f);
        Color3f radiance(0.0f);
        int depth = 1;
        Ray3f currentRay = ray;
        // prevent double counting
        // it means whether the previous bounce is a diffuse surface
        // computed in the last iteration, used in the current iteration
        int isDiffuse = 1;
        
        while (true) {
            Intersection its;
            if (!scene->rayIntersect(currentRay, its)) 
                break;

            // Russian Roulette
            if (depth >= 3) {
                float q = std::min(throughput.maxCoeff() * eta * eta, 0.99f);
                // float q = std::min(throughput.maxCoeff(), 0.99f);
                if (sampler->next1D() > q)
                    break;
                throughput /= q;
            }
            
            if (its.mesh->isEmitter()) {
                // the mesh is a light source
                EmitterQueryRecord eRec(currentRay.o, its.p, its.shFrame.n);
                radiance += throughput * its.mesh->getEmitter()->eval(eRec) * isDiffuse;
            }

            if (its.mesh->getBSDF()->isDiffuse()) {
                // randomly select an emitter
                auto light = scene->getRandomEmitter(sampler);

                // uniformly sample light source
                EmitterQueryRecord lRec(its.p);
                Color3f Li = light->sample(light, lRec, sampler);

                // shadow ray
                // if there is an occluder, Li = 0
                if (scene->rayIntersect(lRec.shadowRay)) {
                    Li = 0;
                }
                // else, no occluder, Li is valid
                // compute the contribution
                float cosTheta = Frame::cosTheta(its.toLocal(lRec.wi));
                // if (cosTheta <= 0) {
                //     continue;
                // }

                // -ray.d: wi, eRec.wi: wo
                BSDFQueryRecord bRec(its.toLocal(-currentRay.d), its.toLocal(lRec.wi), ESolidAngle);
                Color3f f = its.mesh->getBSDF()->eval(bRec);
                radiance += Li * f * cosTheta * throughput / (1.0f / scene->getEmitters().size());

                // prevent double counting
                // should not contain emitter radiance in next bounce
                isDiffuse = 0;
            } else {
                isDiffuse = 1;
            }

            // sample BSDF and get new direction
            BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            throughput *= f;
            currentRay = Ray3f(its.p, its.toWorld(bRec.wo));

            // update eta for RR
            eta *= bRec.eta;

            depth++;
        }

        return radiance;
    }

    std::string toString() const {
        return "PathEmsIntegrator[]";
    }
};
NORI_REGISTER_CLASS(PathEmsIntegrator, "path_ems");
NORI_NAMESPACE_END