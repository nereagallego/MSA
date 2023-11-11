#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMaterialSampling: public Integrator {
public:
	DirectMaterialSampling(const PropertyList& props) {
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

        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d));
        bsdfRecord.uv = its.uv;
        bsdfRecord.measure = ESolidAngle;

        // Sample with bsdf
        Color3f f = bsdf->sample(bsdfRecord, sampler->next2D());
        
        // Here perform a visibility query, to check whether the light 
        // source "em" is visible from the intersection point. 
        // For that, we create a ray object (shadow ray),
        // and compute the intersection
        Ray3f sampleRay(its.p, its.toWorld(bsdfRecord.wo), Epsilon, std::numeric_limits<float>::infinity());
        Intersection shadowIts;
        if (scene->rayIntersect(sampleRay, shadowIts)){
            
            // Check if the object intersects is an emitter
            if (shadowIts.mesh->isEmitter()) {
                EmitterQueryRecord emitterRecord(shadowIts.p);
                emitterRecord.ref = sampleRay.o;
                emitterRecord.emitter = shadowIts.mesh->getEmitter();
                emitterRecord.wi = its.toWorld(bsdfRecord.wo); // ?? wi or wo
                emitterRecord.n = shadowIts.shFrame.n;
                emitterRecord.dist = (shadowIts.p - sampleRay.o).norm();

                const Emitter *em = emitterRecord.emitter;

                Color3f Li = em->eval(emitterRecord);
                float cosTheta = its.shFrame.n.dot(emitterRecord.wi);
                Lo += Li * f ;
            }
            
        } else {
            
            // If the shadow ray does not intersect any object, we 
            // add the background color
            Lo += scene->getBackground(sampleRay) * f ;

        }

        return Lo;
	}

	std::string toString() const {
		return "Direct Whitted Integrator []";
	}
};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");
NORI_NAMESPACE_END