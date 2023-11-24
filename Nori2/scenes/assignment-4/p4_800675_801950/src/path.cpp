#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathTracing: public Integrator {
public:
	PathTracing(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

		Color3f Lo(0.);

        // Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

        // Check current its.p is emitter() then distance -> infinite
        if(its.mesh->isEmitter()) {
            EmitterQueryRecord emitterRecord(its.p);
            emitterRecord.ref = ray.o;
			emitterRecord.wi = -ray.d;
			emitterRecord.n = its.shFrame.n;
            // Add the visible radiance of the emitter
            return its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
        }

        // BRDF sampling
        const BSDF *bsdf = its.mesh->getBSDF();

        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
        bsdfRecord.measure = ESolidAngle;

        // Sample with bsdf
        Color3f f = bsdf->sample(bsdfRecord, sampler->next2D());
        
        // Russian Roulette termination
        float q = std::max(0.05f, 1 - f.getLuminance());
        if (sampler->next1D() <= q)
            return Lo;
        
        Ray3f sampleRay(its.p, its.toWorld(bsdfRecord.wo), Epsilon, INFINITY);

        Lo += f * Li(scene, sampler, sampleRay);
        
        return Lo;
	}

	std::string toString() const {
		return "Path Tracing Integrator []";
	}
};

NORI_REGISTER_CLASS(PathTracing, "path");
NORI_NAMESPACE_END