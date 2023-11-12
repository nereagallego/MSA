#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMIS: public Integrator {
public:
	DirectMIS(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {

		Color3f Lo(0.);

		// Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);

        EmitterQueryRecord emitterRecord(its.p);
		
		// Check current its.p is emitter() then distance -> infinite
        if(its.mesh->isEmitter()) {
            emitterRecord.ref = ray.o;
			emitterRecord.wi = -ray.d;
			emitterRecord.n = its.shFrame.n;
            // Add the visible radiance of the emitter
            Lo += its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
        }

        // Light sampling
		const std::vector<Emitter*> lights = scene->getLights();

        // Choose a light randomly
        float pdfLight;
        const Emitter* em_ems = scene->sampleEmitter(sampler->next1D(), pdfLight);

        // Here we sample the point sources, getting its radiance
        // and direction. 
        Color3f Li_ems = em_ems->sample(emitterRecord, sampler->next2D(), 0.);

        float pdfPoint = em_ems->pdf(emitterRecord);
        float pem = (pdfLight * pdfPoint);


        // BRDF sampling
        const BSDF *bsdf = its.mesh->getBSDF();

        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
        bsdfRecord.measure = ESolidAngle;

        // Sample with bsdf
        Color3f f = bsdf->sample(bsdfRecord, sampler->next2D());

        float pmat = bsdf->pdf(bsdfRecord);

        float we = pem / (pem + pmat);
        float wmat = pmat / (pem + pmat);




        // Here perform a visibility query, to check whether the light 
        // source "em" is visible from the intersection point. 
        // For that, we create a ray object (shadow ray),
        // and compute the intersection
        Ray3f shadowRay_ems(its.p, emitterRecord.wi);
        Intersection shadowIts_ems;
        if (!(scene->rayIntersect(shadowRay_ems, shadowIts_ems) && shadowIts_ems.t <= (emitterRecord.dist - Epsilon))) {

            BSDFQueryRecord bsdfRecord_ems(its.toLocal(-ray.d), its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);

            // For the light chosen, we accomulate the incident light times the 
            // foreshortening times the BSDF term (i.e. the render equation).
            float cosTheta = its.shFrame.n.dot(emitterRecord.wi);
            Lo += Li_ems * its.mesh->getBSDF()->eval(bsdfRecord_ems) * cosTheta;

            Lo = we * Lo/pem;
        }
        
        // Here perform a visibility query, to check whether the light 
        // source "em" is visible from the intersection point. 
        // For that, we create a ray object (shadow ray),
        // and compute the intersection
        Ray3f sampleRay(its.p, its.toWorld(bsdfRecord.wo), Epsilon, INFINITY);
        Intersection shadowIts;
        if (scene->rayIntersect(sampleRay, shadowIts)){
            
            // Check if the object intersects is an emitter
            if (shadowIts.mesh->isEmitter()) {
                const Emitter *em_mats = shadowIts.mesh->getEmitter();
                // EmitterQueryRecord emitterRecord(em,its.p, shadowIts.p, shadowIts.shFrame.n, shadowIts.uv);
                EmitterQueryRecord emitterRecord_mats(em_mats,sampleRay.o, shadowIts.p, shadowIts.shFrame.n, shadowIts.uv);


                Color3f Li = em_mats->eval(emitterRecord_mats);
                
                float cosTheta = its.shFrame.n.dot(emitterRecord_mats.wi);
                Lo += wmat * Li * f;
            }
            
        } else {
            
            // If the shadow ray does not intersect any object, we 
            // add the background color
            Lo += wmat * scene->getBackground(sampleRay) * f ;

        }

        return Lo;
	}

	std::string toString() const {
		return "Direct Whitted Integrator []";
	}
};

NORI_REGISTER_CLASS(DirectMIS, "direct_mis");
NORI_NAMESPACE_END