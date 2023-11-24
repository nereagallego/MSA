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

		// Color3f Lo(0.);

        // // Find the surface that is visible in the requested direction 
		// Intersection its;
		// if (!scene->rayIntersect(ray, its))
		// 	return scene->getBackground(ray);
        
        // EmitterQueryRecord emitterRecord(its.p);

        // // Check current its.p is emitter() then distance -> infinite
        // if(its.mesh->isEmitter()) {
        //     emitterRecord.ref = ray.o;
		// 	emitterRecord.wi = -ray.d;
		// 	emitterRecord.n = its.shFrame.n;
        //     // Add the visible radiance of the emitter
        //     return its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
        // }

        // // Emitters sampling
        // const std::vector<Emitter*> lights = scene->getLights();

        // // Choose a light randomly
        // float pdfLight;
        // const Emitter* em_ems = scene->sampleEmitter(sampler->next1D(), pdfLight);

        // // Here we sample the point sources, getting its radiance
        // // and direction.
        // Color3f Li_ems = em_ems->sample(emitterRecord, sampler->next2D(), 0.);

        // float pdfPoint = em_ems->pdf(emitterRecord);

        // float pem_em = (pdfLight * pdfPoint);


        // // BRDF sampling
        // const BSDF *bsdf = its.mesh->getBSDF();

        // BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
        // // bsdfRecord.measure = ESolidAngle;

        // // Sample with bsdf
        // Color3f f = bsdf->sample(bsdfRecord, sampler->next2D());

        // float pmat_mat = bsdf->pdf(bsdfRecord);
        
        // // Russian Roulette termination
        // float q = std::max(0.05f, 1 - f.getLuminance());
        // if (sampler->next1D() <= q)
        //     return Lo;
        
        // // Here we compute the MIS weight
        // // Accumulate radiance from emitters using MIS
        // Ray3f shadowRay_ems(its.p, emitterRecord.wi);
        // Intersection shadowIts_ems;
        // if (!(scene->rayIntersect(shadowRay_ems, shadowIts_ems) && shadowIts_ems.t <= (emitterRecord.dist - Epsilon))){
        //     BSDFQueryRecord bsdfRecord_ems(its.toLocal(-ray.d), its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
            

        //     // For the light chosen, we accomulate the incident light times the
        //     // foreshortening times the BSDF term (i.e. the render equation).
        //     float cosTheta = its.shFrame.n.dot(emitterRecord.wi);
        //     Li_ems = Li_ems * its.mesh->getBSDF()->eval(bsdfRecord_ems) * cosTheta;

        //     float pmat_em = bsdf->pdf(bsdfRecord_ems);

        //     float we = pem_em / (pem_em + pmat_em);

        //     Li_ems = we * Li_ems/pem_em;
        // }


        // // BSDF sampling ray
        // Ray3f sampleRay(its.p, its.toWorld(bsdfRecord.wo), Epsilon, INFINITY);

        // // Add the direct illumination. NEE

        // // Get all lights in the scene
        // // const std::vector<Emitter*> lights = scene->getLights();

        // // Choose a light randomly
        // const Emitter* emitter = scene->sampleEmitter(sampler->next1D(), pdfLight);
        
        // Color3f Li_contrib = Color3f(0.0f);;// = emitter->sample(emitterRecord, sampler->next2D(), 0.);
        // Color3f Li_light = emitter->sample(emitterRecord,sampler->next2D(), 0.);

        // pdfPoint = emitter->pdf(emitterRecord);
        // Ray3f shadowRay(its.p, emitterRecord.wi);

       
        // // cout << (pdfLight * pdfPoint) << endl;
        // // Check if the light is visible
        // Intersection itsLight;
        // if ((scene->rayIntersect(shadowRay, itsLight) && itsLight.t <= (emitterRecord.dist - Epsilon)) && bsdfRecord.measure != EDiscrete) {
        //     BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
        //     float cosTheta = its.shFrame.n.dot(emitterRecord.wi);

        //     float pem_mat = bsdf->pdf(bsdfRecord);
        //     float wmat = pmat_mat / (pmat_mat + pem_mat);

        //     Li_contrib = Li_light * its.mesh->getBSDF()->eval(bsdfRecord) * cosTheta  / (pdfLight * pdfPoint);
        // }
        
        // // cout << "Li_contrib" << Li_contrib << endl;

        // // Lo += Li_contrib + f * Li(scene, sampler, sampleRay);

        // return Lo + Li_ems + Li_contrib * f * Li(scene, sampler, sampleRay);


        Color3f Lo(0.);

        // Find the surface that is visible in the requested direction 
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);
        
        EmitterQueryRecord emitterRecord(its.p);
        
        if(its.mesh->isEmitter()) {
            emitterRecord.ref = ray.o;
			emitterRecord.wi = -ray.d;
			emitterRecord.n = its.shFrame.n;
            // Add the visible radiance of the emitter
            return its.mesh->getEmitter()->sample(emitterRecord, sampler->next2D(), 0.);
        }

        // Light sampling

        // Get all lights in the scene
        const std::vector<Emitter*> lights = scene->getLights();

        // Choose a light randomly
        float pdfLight;
        const Emitter* emitter = scene->sampleEmitter(sampler->next1D(), pdfLight);
        
        Color3f Li_contrib = Color3f(0.0f);;// = emitter->sample(emitterRecord, sampler->next2D(), 0.);
        Color3f Li_light = emitter->sample(emitterRecord,sampler->next2D(), 0.);

        float pdfPoint = emitter->pdf(emitterRecord);
        float pem_em = (pdfLight * pdfPoint);

        Ray3f shadowRay(its.p, emitterRecord.wi);

        const BSDF *bsdf = its.mesh->getBSDF();
       
        // Check if the light is visible
        Intersection itsLight;
        if (!(scene->rayIntersect(shadowRay, itsLight) && itsLight.t <= (emitterRecord.dist - Epsilon))) {
            BSDFQueryRecord bsdfRecord_ems(its.toLocal(-ray.d), its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
            float cosTheta = its.shFrame.n.dot(emitterRecord.wi);
            float pmat_em = bsdf->pdf(bsdfRecord_ems);
            float we = pem_em / (pem_em + pmat_em);
            Li_contrib = we * (Li_light * its.mesh->getBSDF()->eval(bsdfRecord_ems) * cosTheta)/pem_em;
        }

        // BRDF sampling
        BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
        bsdfRecord.measure = ESolidAngle;

        // Sample with bsdf
        Color3f f = bsdf->sample(bsdfRecord, sampler->next2D());

        float pmat_mat = bsdf->pdf(bsdfRecord);
        
        // Russian Roulette termination
        float q = std::max(0.05f, 1 - f.getLuminance());
        if (sampler->next1D() <= q)
            return Lo;
        
        // BSDF sampling ray
        Ray3f sampleRay(its.p, its.toWorld(bsdfRecord.wo), Epsilon, INFINITY);
        Intersection itsSample;
        if (scene->rayIntersect(sampleRay, itsSample)) {
            if (itsSample.mesh->isEmitter()) {
                const Emitter *em_mats = itsSample.mesh->getEmitter();
                // EmitterQueryRecord emitterRecord(em,its.p, shadowIts.p, shadowIts.shFrame.n, shadowIts.uv);
                EmitterQueryRecord emitterRecord_mats(em_mats,sampleRay.o, itsSample.p, itsSample.shFrame.n, itsSample.uv);
                
                float pem_mat = em_mats->pdf(emitterRecord_mats);

                float wmat = pmat_mat / (pmat_mat + pem_mat);

                Color3f Li_sample = em_mats->eval(emitterRecord_mats);
                
                Li_contrib += wmat * Li_sample;
            }
        } else {
            Li_contrib += scene->getBackground(sampleRay);
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