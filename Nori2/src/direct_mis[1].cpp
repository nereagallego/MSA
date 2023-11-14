#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/proplist.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMIS : public Integrator {
public :

	DirectMIS(const PropertyList &props) {
		// No parameters this time
	}
	
	/// Compute the radiance value for a given ray.
	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		// Initialize variables
		Color3f Lo(0.);
		Color3f Le_o(0.);
		Color3f Lo_ems(0.);
		Color3f Lo_mats(0.);
		float w_ems = 1.f;
		float w_mats = 1.f;
		float pdf_BRDF_mats = 0.0f;			// This is the pdf of the BRDF sampling over the surface area
		float pdf_BRDF_ems = 0.0f;
		float pdf_ems_ems = 0.0f;
		float pdf_ems_mats = 0.0f;

		// Find the surface that is visible in the requested direction
		Intersection its;	// Generate an intersection
		if(!scene->rayIntersect(ray,its))	// Throw a ray to intersect with something and return the intersection
			return scene->getBackground(ray);	// If it doesn't intersect return no light

		// Check if first intersection is an emitter
		if(its.mesh->isEmitter()){
			// Get light emitter
			const Emitter* em_light = its.mesh->getEmitter();

			// Initialize to ray.o as it's where we want to evaluate the radiance
			EmitterQueryRecord em_light_record(em_light, ray.o, its.p, its.shFrame.n, its.uv);
				
			// Get the radiance
			//Le_o =  em_light->sample(em_light_record, sampler->next2D(), 0.);			
			return em_light->eval(em_light_record);
		}

		////// BRDF sampling ///////

		// Sample direction from the material intersection
		BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
		Color3f fr = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D()); // This returns eval * cosTheta / pdf so it's included for the rendering equation afterwards

		// Check if there is a next intersection for the sampled ray from the material intersection
		Intersection next_it;
		Ray3f materialRay(its.p, its.toWorld(bsdfRecord.wo)); // bRec.wo is in the local frame ref but scene->rayIntersect needs world reference
		if(!scene->rayIntersect(materialRay, next_it))
			return scene->getBackground(materialRay);

		// Check if second intersection is a light
		if(next_it.mesh->isEmitter()){
			// Query the emitter and eval its radiance
			const Emitter* em_source = next_it.mesh->getEmitter();
			EmitterQueryRecord em_source_record;
			em_source_record.wi = materialRay.d;
			em_source_record.n = next_it.shFrame.n;
			em_source_record.dist = next_it.t;
			
			Color3f Li_mats = em_source->sample(em_source_record, sampler->next2D(), 0.);
			Lo_mats = Li_mats * fr;
			
			// We calculate the pdf over the surface area
			pdf_ems_mats = next_it.mesh->getEmitter()->pdf(em_source_record); // Where the ray goes to the light --> pdf
			

		}
		
		// We calculate the pdfs over the surface area
		pdf_BRDF_mats = its.mesh->getBSDF()->pdf(bsdfRecord);					// Where the camera ray intersects with the surface --> pdf
		

		////// Emitter sampling ///////

		float pdflight;
		// Initialization of the sample point at our reference
		EmitterQueryRecord emitterRecord(its.p); // EmitterQueryRecord has the information of the emitter that has been sampled from the direct light
		
		// Randomly sample a light source
		const Emitter* em = scene->sampleEmitter(sampler->next1D(),pdflight);
		
		// Here we sample the point sources, getting its radiance and direction
		Color3f Li_ems = em->sample(emitterRecord, sampler->next2D(), 0.);
	
		w_mats = pdf_BRDF_mats/(pdf_ems_mats * pdflight + pdf_BRDF_mats);

		// Visibility check. If emitter sampling evaluates a shadow then we only return the part proportional to BRDF sampling
		Intersection shadow_it;
		Ray3f shadowRay(its.p, emitterRecord.wi);
		if (scene->rayIntersect(shadowRay, shadow_it)){
			if (shadow_it.t < (emitterRecord.dist - 1.e-5))
				return w_mats * Lo_mats + Le_o;
		}
		
		// Evaluate the BSDF and get the final radiance for emitter sampling
		BSDFQueryRecord bsdfRecord_ems(its.toLocal(-ray.d), its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
		
		// We calculate the pdfs over the solid angle
		pdf_BRDF_ems = its.mesh->getBSDF()->pdf(bsdfRecord_ems);
		pdf_ems_ems = em->pdf(emitterRecord) * pdflight;
		
		
		// We obtain the weights
		w_ems = pdf_ems_ems/(pdf_ems_ems + pdf_BRDF_ems);
		
		Lo_ems = Li_ems * its.shFrame.n.dot(emitterRecord.wi) * its.mesh->getBSDF()->eval(bsdfRecord_ems) / (pdf_ems_ems);
		
		// Weighten the total radiance
		return w_ems * Lo_ems + w_mats * Lo_mats;// + Le_o;
		
	}
	
	/// Return a human-readable description for debugging purposes
	std::string toString ( ) const {
		return "DirectMIS[]";
	}
	
protected :
	std::string m_myProperty;
};

NORI_REGISTER_CLASS(DirectMIS, "direct_mis" ) ;
NORI_NAMESPACE_END
