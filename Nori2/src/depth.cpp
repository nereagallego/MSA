#include <nori/integrator.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class DepthIntegrator : public Integrator {
public:
    DepthIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        /* Return the component-wise absolute
           value of the shading normal as a color */
        float d = its.t;
        return Color3f(1/d, 1/d, 1/d);
    }

    std::string toString() const {
        return "DepthIntegrator[]";
    }
};

NORI_REGISTER_CLASS(DepthIntegrator, "depth");
NORI_NAMESPACE_END
