#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class PathMatsIntegrator : public Integrator {
public:
    PathMatsIntegrator(const PropertyList& props) {}
    
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        float eta = 1.0f;
        Color3f throughput(1.0f);
        Color3f radiance(0.0f);
        int depth = 1;
        Ray3f currentRay = ray;

        while (true) {
            Intersection its;
            if (!scene->rayIntersect(currentRay, its)) 
                break;

            // Russian roulette
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
                radiance += throughput * its.mesh->getEmitter()->eval(eRec);
            }

            // sample BSDF and get new direction
            BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
            Color3f fr = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            throughput *= fr;
            currentRay = Ray3f(its.p, its.toWorld(bRec.wo));

            // update eta for RR
            eta *= bRec.eta;

            depth++;
        }

        return radiance;
    }

    std::string toString() const {
        return "PathMatsIntegrator[]";
    }
};
NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END