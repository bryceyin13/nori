#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
NORI_NAMESPACE_BEGIN
class PathMisIntegrator : public Integrator {
public:
    PathMisIntegrator(const PropertyList& props) {}

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const override {
        float eta = 1.0f;
        Color3f throughput(1.0f);
        Color3f radiance(0.0f);
        int depth = 1;
        Ray3f currentRay = ray;
        // BSDF weight for MIS
        float weight_brdf = 1.0f;
        
        while (true) {
            Intersection its;
            if (!scene->rayIntersect(currentRay, its)) 
                break;

            // Russian Roulette
            if (depth >= 3) {
                float q = std::min(throughput.maxCoeff() * eta * eta, 0.99f);
                // float q = std::min(throughput.maxCoeff(), 0.99f);
                if (sampler->next1D() > q) {
                    return radiance;
                }
                throughput /= q;
            }

            if (its.mesh->isEmitter()) {
                // the mesh is a light source
                EmitterQueryRecord eRec(currentRay.o, its.p, its.shFrame.n);
                // use the weight from previous BSDF sampling
                radiance += throughput * its.mesh->getEmitter()->eval(eRec) * weight_brdf;
            }

            // emitter importance sampling
            // randomly select an emitter
            auto light = scene->getRandomEmitter(sampler);

            // uniformly sample light source
            EmitterQueryRecord lRec(its.p);
            Color3f Li = light->sample(light, lRec, sampler);
            float pdf_emitter = light->pdf(light, lRec);

            // shadow ray
            // if there is an occluder, Li = 0
            if (scene->rayIntersect(lRec.shadowRay)) {
                Li = 0;
            }
            // else, no occluder, Li is valid
            // compute the contribution
            // float cosTheta = std::max(0.f, Frame::cosTheta(its.shFrame.toLocal(lRec.wi)));
            float cosTheta = Frame::cosTheta(its.toLocal(lRec.wi));

            BSDFQueryRecord blRec(its.toLocal(-currentRay.d), its.toLocal(lRec.wi), ESolidAngle);
            Color3f fl = its.mesh->getBSDF()->eval(blRec);
            float pdf_brdf = its.mesh->getBSDF()->pdf(blRec);
            // balance heuristic
            //float weight_emitter = 0.0f;
            // if (pdf_brdf + pdf_emitter != 0.0f) {
            //     weight_emitter = pdf_emitter / (pdf_brdf + pdf_emitter);
            // }
            float weight_emitter = pdf_brdf + pdf_emitter > 0.0f ? pdf_emitter / (pdf_brdf + pdf_emitter) : pdf_emitter;
            radiance += Li * fl * cosTheta * weight_emitter * throughput / (1.0f / scene->getEmitters().size());

            // BSDF importance sampling
            // sample BSDF and get new direction
            BSDFQueryRecord bRec(its.toLocal(-currentRay.d));
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            throughput *= f;
            currentRay = Ray3f(its.p, its.toWorld(bRec.wo));
            
            float pdf_new_brdf = its.mesh->getBSDF()->pdf(bRec);

            // direct illumination from recursive ray
            Point3f origin = its.p;
            Intersection new_its;
            if (!scene->rayIntersect(currentRay, new_its)) {
                break;
            }

            // new intersection is an emitter
            // update BSDF weight for next bounce
            if (new_its.mesh->isEmitter()) {
                EmitterQueryRecord newbRec = EmitterQueryRecord(origin, new_its.p, new_its.shFrame.n);
                float pdf_new_emitter = new_its.mesh->getEmitter()->pdf(new_its.mesh->getEmitter(), newbRec);
                // balance heuristic
                // if (pdf_new_brdf + pdf_new_emitter != 0.0f) {
                //     weight_brdf = pdf_new_brdf / (pdf_new_brdf + pdf_new_emitter);
                // }
                weight_brdf = pdf_new_brdf + pdf_new_emitter > 0.f ? pdf_new_brdf / (pdf_new_brdf + pdf_new_emitter) : pdf_new_brdf;
                // weight_brdf = pdf_new_brdf / (pdf_new_brdf + pdf_new_emitter);
            }

            // in Nori, if the bsdf is a delta distribution(mirror), pdf() returns 0
            // so we need to manually set the weight to 1.0
            if (bRec.measure == EDiscrete) {
                weight_brdf = 1.0f;
            }

            // update eta for RR
            eta *= bRec.eta;

            depth++;
        }
        return radiance;
    }

    std::string toString() const {
        return "PathMisIntegrator[]";
    }
};
NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END