#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectEmitterSampling: public Integrator {
public:
	DirectEmitterSampling(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

		Color3f Lo(0.);

		// Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

		float pdflight;
		EmitterQueryRecord emitterRecord(its.p);

		// Get all lights in the scene
		const std::vector<Emitter*> lights = scene->getLights();

        // Choose a light randomly
        int lightIndex = std::min((int)(sampler->next1D() * lights.size()), (int)lights.size() - 1);
        const Emitter* em = lights[lightIndex];

        // Here we sample the point sources, getting its radiance
        // and direction. 
        Color3f Li = em->sample(emitterRecord, sampler->next2D(), 0.);

		//check current its.p is emitter() then distance -> infinite
        if(its.mesh->isEmitter()) {
            // return the power of the emitter
            return its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
            return its.mesh->getEmitter()->eval(emitterRecord);
        }

        // Here perform a visibility query, to check whether the light 
        // source "em" is visible from the intersection point. 
        // For that, we create a ray object (shadow ray),
        // and compute the intersection
        
        Ray3f shadowRay(its.p,emitterRecord.wi);
        Intersection shadowIts;
        if (scene->rayIntersect(shadowRay, shadowIts))
            return Lo;

        // Finally, we evaluate the BSDF. For that, we need to build
        // a BSDFQueryRecord from the outgoing direction (the direction
        // of the primary ray, in ray.d), and the incoming direction 
        // (the direction to the light source, in emitterRecord.wi). 
        // Note that: a) the BSDF assumes directions in the local frame
        // of reference; and b) that both the incoming and outgoing 
        // directions are assumed to start from the intersection point. 	
        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), 
                            its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);

		
        // For each light, we accomulate the incident light times the 
        // foreshortening times the BSDF term (i.e. the render equation).
		float cosTheta = fmaxf(its.shFrame.n.dot(-emitterRecord.wi),0.f);
		Lo = its.mesh->getBSDF()->eval(bsdfRecord) * Li * cosTheta;
		
        // Lo = Le * its.shFrame.n.dot(emitterRecord.wi) * 
        //                     its.mesh->getBSDF()->eval(bsdfRecord);

        // float pdfLight = em->pdf(emitterRecord);
        // float pdfPos = its.mesh->pdf(its.p);
        // Lo /= (pdfLight*pdfPos);
        
		return Lo;
	}

	std::string toString() const {
		return "Direct Whitted Integrator []";
	}
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END
