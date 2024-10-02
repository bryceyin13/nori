#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class SimpleIntegrator : public Integrator {
public:
    SimpleIntegrator(const PropertyList &props) {
        energy = props.getColor("energy");
        position = props.getPoint("position");
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        Ray3f shadowray(its.p, position - its.p);
        if (scene->rayIntersect(shadowray))
            return Color3f(0.0f);
        /* Return the component-wise absolute
           value of the shading normal as a color */
        Normal3f n = its.shFrame.n.normalized();
        float dist = std::pow(its.p.x() - position.x(), 2) + std::pow(its.p.y() - position.y(), 2) + std::pow(its.p.z() - position.z(), 2);
        Color3f radiance = energy / (4 * M_PI * M_PI) * std::max(0.0f, n.dot((position - its.p).normalized())) / dist; 
        return radiance;
    }

    std::string toString() const {
        return "SimpleIntegrator[]";
    }
private:
    Color3f energy;
    Point3f position;
};

NORI_REGISTER_CLASS(SimpleIntegrator, "simple");
NORI_NAMESPACE_END