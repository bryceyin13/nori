#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AOIntegrator : public Integrator {
public:
   AOIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
        
        Vector3f v = Warp::squareToCosineHemisphere(sampler->next2D());
        Vector3f wi = its.shFrame.toWorld(v);
        Normal3f n = its.shFrame.n.normalized();
        Ray3f shadowray(its.p, wi);
        if (scene->rayIntersect(shadowray))
            return Color3f(0.0f);

        // According to MC sampling,
        // it should "return Color3f(n.dot(wi) * INV_PI / Warp::squareToCosineHemispherePdf(v));"
        // but since Warp::squareToCosineHemispherePdf(v) == n.dot(wi) / PI, the return value is always 1
        return Color3f(1.0f);
    }

    std::string toString() const {
        return "AOIntegrator[]";
    }
};

NORI_REGISTER_CLASS(AOIntegrator, "ao");
NORI_NAMESPACE_END