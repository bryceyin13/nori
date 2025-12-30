#include <nori/emitter.h>
#include <nori/sampler.h>
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter {
private:
    Color3f m_radiance;
    Mesh* mesh;
public:
    AreaLight(const PropertyList& propList) {
        m_radiance = propList.getColor("radiance");
    }

    Color3f eval(const EmitterQueryRecord& eRec) const override {
        // wi is from from shading point to the point on light source, so
        // wi dot n < 0 means the light source is facing the shading point
        // wi dot n > 0 means the light source is facing away from the shading point
        return (eRec.n.dot(eRec.wi) < 0.0f) ? m_radiance : 0.0f;
    }

    float pdf(const Emitter* emitter, const EmitterQueryRecord& eRec) const override {
        float cosTheta = eRec.n.dot(-eRec.wi);
        if (cosTheta > 0.0f) 
            // solid_angle_pdf = area_pdf * dist^2 / cosine theta
            return emitter->mesh->getPDF().getNormalization() * (eRec.light_p - eRec.p).squaredNorm() / cosTheta;
        else
            return 0.0f;
    }

    Color3f sample(const Emitter* emitter, EmitterQueryRecord& eRec, Sampler* sample) const override {
        Point3f light_p;
        Normal3f n;
        float pdf;

        // uniform sampling
        auto sRec = emitter->mesh->sampleUniform(sample, light_p, n, pdf);
        eRec.light_p = light_p;
        eRec.n = n;
        // direction from shading point to light point
        eRec.wi = (eRec.light_p - eRec.p).normalized();
        // setup shadow ray
        // origin, direction, min_t, max_t
        eRec.shadowRay = Ray3f(eRec.p, eRec.wi, Epsilon, (eRec.light_p - eRec.p).norm() - Epsilon);
        eRec.pdf = this->pdf(emitter, eRec);
        
        if (eRec.pdf > 0.0f && !std::isnan(eRec.pdf) && !std::isinf(eRec.pdf)) 
            return eval(eRec) / eRec.pdf;
        else 
            return Color3f(0.0f);
    }

    Color3f getRadiance() const override {
        return m_radiance;
    }

    std::string toString() const override {
        return "Emitter[]";
    }

    void setMesh(Mesh* mesh) {
        this->mesh = mesh;
    }
};

NORI_REGISTER_CLASS(AreaLight, "area")
NORI_NAMESPACE_END