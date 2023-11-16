#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathTracingMIS: public Integrator {
public:
	PathTracingMIS(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

		Color3f Lo(0.);

        // Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);
        
        EmitterQueryRecord emitterRecord(its.p);

        // BRDF sampling
        const BSDF *bsdf = its.mesh->getBSDF();

        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
        // bsdfRecord.measure = ESolidAngle;

        // Check current its.p is emitter() then distance -> infinite
        if(its.mesh->isEmitter()) {
            emitterRecord.ref = ray.o;
			emitterRecord.wi = -ray.d;
			emitterRecord.n = its.shFrame.n;
            // Add the visible radiance of the emitter
            return its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
        }

        // Sample with bsdf
        Color3f f = bsdf->sample(bsdfRecord, sampler->next2D());
        
        // Russian Roulette termination
        float q = std::max(0.05f, 1 - f.getLuminance());
        if (sampler->next1D() <= q)
            return Lo;
        
        // BSDF sampling ray
        Ray3f sampleRay(its.p, its.toWorld(bsdfRecord.wo), Epsilon, INFINITY);

        // Add the direct illumination. NEE

        // Get all lights in the scene
        const std::vector<Emitter*> lights = scene->getLights();

        // Choose a light randomly
        float pdfLight;
        const Emitter* emitter = scene->sampleEmitter(sampler->next1D(), pdfLight);
        
        Color3f Li_contrib = Color3f(0.0f);;// = emitter->sample(emitterRecord, sampler->next2D(), 0.);
        Color3f Li_light = emitter->sample(emitterRecord,sampler->next2D(), 0.);

        float pdfPoint = emitter->pdf(emitterRecord);
        Ray3f shadowRay(its.p, emitterRecord.wi);

        // Check if the light is visible
        Intersection itsLight;
        if (!(scene->rayIntersect(shadowRay, itsLight) && itsLight.t <= (emitterRecord.dist - Epsilon)) && !bsdfRecord.measure == EDiscrete) {
            BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
            float cosTheta = its.shFrame.n.dot(emitterRecord.wi);
            Li_contrib = Li_light * its.mesh->getBSDF()->eval(bsdfRecord) * cosTheta / (pdfLight * pdfPoint);
        }

        Lo += Li_contrib + f * Li(scene, sampler, sampleRay);

        return Lo;


	}

	std::string toString() const {
		return "Path Tracing MIS Integrator []";
	}
};

NORI_REGISTER_CLASS(PathTracingMIS, "path_mis");
NORI_NAMESPACE_END