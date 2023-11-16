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
            Lo += its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
        }

        // BRDF sampling
        const BSDF *bsdf = its.mesh->getBSDF();

        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
        bsdfRecord.measure = ESolidAngle;

        // Sample with bsdf
        Color3f f = bsdf->sample(bsdfRecord, sampler->next2D());
        
        // Here perform a visibility query, to check whether the light 
        // source "em" is visible from the intersection point. 
        // For that, we create a ray object (shadow ray),
        // and compute the intersection
        Ray3f sampleRay(its.p, its.toWorld(bsdfRecord.wo), Epsilon, INFINITY);
        Intersection shadowIts;
        if (scene->rayIntersect(sampleRay, shadowIts)){
            
            // Check if the object intersects is an emitter
            if (shadowIts.mesh->isEmitter()) {
                const Emitter *em = shadowIts.mesh->getEmitter();
                // EmitterQueryRecord emitterRecord(em,its.p, shadowIts.p, shadowIts.shFrame.n, shadowIts.uv);
                EmitterQueryRecord emitterRecord(em,sampleRay.o, shadowIts.p, shadowIts.shFrame.n, shadowIts.uv);


                Color3f Li = em->eval(emitterRecord);
                
                Lo += Li * f;
            }
            
        } else {
            
            // If the shadow ray does not intersect any object, we 
            // add the background color
            Lo += scene->getBackground(sampleRay) * f ;

        }

        return Lo;
	}

	std::string toString() const {
		return "Direct Material Sampling Integrator []";
	}
};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");
NORI_NAMESPACE_END