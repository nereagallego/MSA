#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/common.h>

#include <nori/nanoflann.hpp>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <random>

using namespace std;
using namespace nanoflann;
NORI_NAMESPACE_BEGIN

class DispersiveIntegrator : public Integrator {
public:
    enum EPhotonMap{
        ECausticPhotonMap = 0,
        EGlobalPhotonMap
    };

    struct PointCloud {
        struct Point
        {
            float  x,y,z;
            Color3f power;
            Vector3f wi;
        };

        std::vector<Point>  pts;

        // Must return the number of data points
        inline size_t kdtree_get_point_count() const { return pts.size(); }

        // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
        inline float kdtree_distance(const float *p1, const size_t idx_p2,size_t /*size*/) const
        {
            const float d0=p1[0]-pts[idx_p2].x;
            const float d1=p1[1]-pts[idx_p2].y;
            const float d2=p1[2]-pts[idx_p2].z;
            return std::sqrt(d0*d0+d1*d1+d2*d2);
        }

        // Returns the dim'th component of the idx'th point in the class:
        // Since this is inlined and the "dim" argument is typically an immediate value, the
        //  "if/else's" are actually solved at compile time.
        inline float kdtree_get_pt(const size_t idx, int dim) const
        {
            if (dim==0) return pts[idx].x;
            else if (dim==1) return pts[idx].y;
            else return pts[idx].z;
        }

        // Optional bounding-box computation: return false to default to a standard bbox computation loop.
        //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
        //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

    };

    //Define 16 wavelengths values
    const double wavelength[16] = {380, 405, 430, 455, 480, 505, 530, 555, 580, 605,
                              630, 655, 680, 705, 730, 750};

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
            L2_Simple_Adaptor<float, PointCloud> ,
            PointCloud,
            3 /* dim */
    > my_kd_tree_t;

    PointCloud cloud, caustic_cloud;
    my_kd_tree_t  index, caustic_index; //(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );

    Color3f wavelengthToRGB(double wavelength) {
        double gamma = 0.8;
        int intensityMax = 255;
        double factor = 0.0;
        int R = 0, G = 0, B = 0;

        if (wavelength >= 380 && wavelength < 440) {
            R = static_cast<int>((-(wavelength - 440) / (440 - 380)));
            G = 0;
            B = static_cast<int>(1.0);
        } else if (wavelength >= 440 && wavelength < 490) {
            R = 0;
            G = static_cast<int>((wavelength - 440) / (490 - 440));
            B = static_cast<int>(1.0);
        } else if (wavelength >= 490 && wavelength < 510) {
            R = 0;
            G = static_cast<int>(1.0);
            B = static_cast<int>(-(wavelength - 510) / (510 - 490));
        } else if (wavelength >= 510 && wavelength < 580) {
            R = static_cast<int>((wavelength - 510) / (580 - 510));
            G = static_cast<int>(1.0);
            B = 0;
        } else if (wavelength >= 580 && wavelength < 645) {
            R = static_cast<int>(1.0);
            G = static_cast<int>(-(wavelength - 645) / (645 - 580));
            B = 0;
        } else if (wavelength >= 645 && wavelength < 750) {
            R = static_cast<int>(1.0);
            G = 0;
            B = 0;
        }

        // Adjust intensity
        factor = 1.0;

        // Adjust gamma
        R = static_cast<int>(intensityMax * pow((R * factor), gamma));
        G = static_cast<int>(intensityMax * pow((G * factor), gamma));
        B = static_cast<int>(intensityMax * pow((B * factor), gamma));

        return Color3f(R / 255.0f, G / 255.0f, B / 255.0f);
    }

    void preprocess(const Scene *scene) {
        // Randomize Seed
        cout<<"photon_mapping..."<<endl;
        std::cout<<"Generating "<< max_photon_count << " photons for Global Photon Map"<< std::endl;
        std::cout<<"Generating "<< caustic_max_photon_count << " photons for Caustic Photon Map"<< std::endl;

        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<> dis(0, 15);
        std::uniform_int_distribution<> dis3(0, 2);

        std::unique_ptr<Sampler> sampler(scene->getSampler()->clone());

        Color3f totalPower = Color3f(0.0f);
        float totalPowerLuminance = 0.0f;

        for(Emitter *emitter : scene->getLights()){
            totalPowerLuminance += emitter->power().getLuminance();
        }

        // float totalPowerLuminance = 0.27f * totalPower.r() + 0.67f * totalPower.g() + 0.06f * totalPower.b();

        // count number of times each wavelength is chosen
        int wavelengthCount[16] = {0};
        

        // Global Photon Map
        for (Emitter* emitter: scene->getLights()){
            float emitterPowerLuminance = emitter->power().getLuminance();

            int photon_count = max_photon_count * emitterPowerLuminance / totalPowerLuminance;
            int n_emitted = 0;
            float p_absorb = 0.1f;

            cout << "luminance " << emitter->power().getLuminance() << endl;
            Color3f flux = emitter->power() * 4 * M_PI/ photon_count;

            while(n_emitted < photon_count){
                // Generate a ray from the emitter
                Point3f p = emitter->samplePosition(sampler->next2D());
                Vector3f d = emitter->sampleDirection(p, sampler.get());
                
                // cout << "emitterRecord.ref = " << emitterRecord.ref << endl;
                Ray3f ray(p,d);
                Color3f throughput = Color3f(1.0f);
                const BSDF *bsdf = nullptr;

                BSDFQueryRecord bsdfRecord(Vector3f(0.f));
                bsdfRecord.measure = ESolidAngle;
                bool first = false;
                // cout << "Soy tontito y estoy fuera" << endl;



                while (true) {

                    Intersection its;
                    // cout << "Ray origin: " << ray.o.transpose() << ", direction: " << ray.d.transpose() << endl;
                    if(!scene->rayIntersect(ray, its)){
                        // cout<<"No intersection"<<endl;
                        // cout << "n_emitted" << n_emitted << endl;
                        break;
                    }
                    // cout<<"intersection"<<endl;
                    bsdfRecord.wi = its.toLocal(-ray.d);
                    bsdfRecord.uv = its.uv;
                    // BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
                    Color3f color = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());
                    if(its.mesh->getBSDF()->isDiffuse() && !first) {
                        PointCloud::Point photon;
                        photon.x = its.p.x();
                        photon.y = its.p.y();
                        photon.z = its.p.z();
                        photon.wi = ray.d;
                        // cout << "flux: " << flux.r() << " " << flux.g() << " " << flux.b() << endl;
                        // cout << "throughput: " << throughput.r() << " " << throughput.g() << " " << throughput.b() << endl;
                        photon.power = flux * throughput;
                        // cout << "photon power: " << photon.power.r() << " " << photon.power.g() << " " << photon.power.b() << endl;
                       
                    
                        throughput *= color;
                        cloud.pts.push_back(photon);
                        // cout << "Photon emitted" << endl;
                    } else if(its.mesh->getBSDF()->isDiffuse()) {
                        first = false;
                        PointCloud::Point photon;
                        photon.x = its.p.x();
                        photon.y = its.p.y();
                        photon.z = its.p.z();
                        photon.wi = ray.d;
                        // cout << "flux: " << flux.r() << " " << flux.g() << " " << flux.b() << endl;
                        // cout << "throughput: " << throughput.r() << " " << throughput.g() << " " << throughput.b() << endl;
                        photon.power = flux * throughput;
                        // cout << "photon power: " << photon.power.r() << " " << photon.power.g() << " " << photon.power.b() << endl;
                       
                    
                        throughput *= color;
                        // cloud.pts.push_back(photon);
                    }

                    if(sampler->next1D() < p_absorb){
                        // cout << "Photon absorbed" << endl;
                        // cout << "n_emitted" << n_emitted << endl;
                        break;
                    }

                    // Next ray
                  
                    Ray3f nextRay = Ray3f(its.p, its.toWorld(bsdfRecord.wo), Epsilon, INFINITY);
                    // cout << "BSDF record: wi = " << bsdfRecord.wi.transpose() << ", wo = " << bsdfRecord.wo.transpose() << ", uv = " << bsdfRecord.uv.transpose() << endl;
                    // cout << "fucking wo= " << bsdfRecord.wo.transpose() << endl;
                    ray = nextRay;
                    // cout << "asignación " << ray.d.transpose() << endl;
                    // cout << "Next ray" << endl;
                    
                }
                n_emitted++;
            }
        }
        cout << "Global map generated" << endl;
        // Caustic Photon Map
        for (Emitter* emitter: scene->getLights()){
            float emitterPowerLuminance = emitter->power().getLuminance();

            int caustic_photon_count = caustic_max_photon_count * emitterPowerLuminance / totalPowerLuminance;
            int n_emitted = 0;
            float p_absorb = 0.1f;

            Color3f flux = emitter->power() * 4 * M_PI/ caustic_photon_count;

            while(n_emitted < caustic_photon_count){
                // Generate a ray from the emitter

                // Generate a ray from the emitter
                Point3f p = emitter->samplePosition(sampler->next2D());
                Vector3f d = emitter->sampleDirection(p, sampler.get());
                
                // cout << "emitterRecord.ref = " << emitterRecord.ref << endl;
                Ray3f ray(p,d);
                Color3f throughput = Color3f(1.0f);
                const BSDF *bsdf = nullptr;

                
                Intersection its;
                if(!scene->rayIntersect(ray, its)){
                    // cout<<"No intersection"<<endl;
                    continue;
                }
                bsdf = its.mesh->getBSDF();
                if(bsdf->isDiffuse()) {
                    continue; 
                }
                bool causticFound = true;
                while(!bsdf->isDiffuse()){
                    if(!scene->rayIntersect(ray, its)){
                        causticFound = false;
                        break;
                    }
                    BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);
                    bsdf = its.mesh->getBSDF();
                    if(bsdf->isDiffuse()) {
                        break; 
                    }
                    if(!its.mesh->getBSDF()->isDispersive()) {
                            
                        bsdf->sample(bsdfRecord, sampler->next2D());
                    } else {
                        // Choose a random wavelength
                        int index = dis(gen);
                        // cout << "index = " << index << endl;
                        float lambda = wavelength[index];
                        // wavelengthCount[index]++;

                        // // cout << "lambda = " << index << endl;
                        bsdfRecord.wavelength = lambda;
                        bsdf->sample(bsdfRecord, sampler->next2D());
                        Color3f c = wavelengthToRGB(lambda);
                        // Color3f c;
                        // if (index == 0) {
                        //     c = Color3f(1.0f, 0.0f, 0.0f);
                        // } else if (index == 1) {
                        //     c = Color3f(0.0f, 1.0f, 0.0f);
                        // } else if (index == 2) {
                        //     c = Color3f(0.0f, 0.0f, 1.0f);
                        // } else {
                        //     cout << "Error" << endl;
                        // }
                        throughput = c;

                        // cout << "c = " << c.r() << " " << c.g() << " " << c.b() << endl;
                    }
                    if(sampler->next1D() < p_absorb){
                        // cout << "Photon absorbed" << endl;
                        // cout << "n_emitted" << n_emitted << endl;
                        causticFound = false;
                        break;
                    }
                    // bsdf->sample(bsdfRecord, sampler->next2D());
                    ray = Ray3f(its.p, its.toWorld(bsdfRecord.wo), Epsilon, INFINITY);
                    
                }          
                    
                if(causticFound) {
                    PointCloud::Point photon;
                    photon.x = its.p.x();
                    photon.y = its.p.y();
                    photon.z = its.p.z();
                    photon.wi = ray.d;
                    // cout << "flux: " << flux.r() << " " << flux.g() << " " << flux.b() << endl;
                    // cout << "throughput: " << throughput.r() << " " << throughput.g() << " " << throughput.b() << endl;
                    photon.power = flux * throughput;
                    // cout << "photon power: " << photon.power.r() << " " << photon.power.g() << " " << photon.power.b() << endl;
                    
                    caustic_cloud.pts.push_back(photon);
                    // cout << "Photon emitted" << endl;
                    
                    n_emitted++;
                    // cout << "n_emitted" << n_emitted  << "/" << caustic_photon_count << endl;
                }
                
            }
        }
        
        cout<<"cloud's size "<<cloud.pts.size()<<endl;
        cout<<"caustic cloud's size "<<caustic_cloud.pts.size()<<endl;
        std::cout<<"done\n"<<std::endl;

        /* kd-tree */
        index.buildIndex();
        caustic_index.buildIndex();

        // print percentage of each wavelength
        // for(int i = 0; i < 16; i++) {
        //     cout << "wavelength " << wavelength[i] << " nm: " << wavelengthCount[i] * 100 / max_photon_count << "%" << endl;
        // }
    }

    DispersiveIntegrator(const PropertyList &props)
            : index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)),
                caustic_index(3 /*dim*/, caustic_cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)){

        max_photon_count = props.getInteger("photon_count", 10000000);
        rad_estimation_count = props.getInteger("rad_estimation_count", 10000);
        rad_estimation_radius = props.getFloat("rad_estimation_radius", 0.0015);
        caustic_rad_estimation_radius = props.getFloat("caustic_rad_estimation_radius", 0.0015);
        caustic_max_photon_count = props.getInteger("caustic_photon_count", 10000000);
        caustic_rad_estimation_count = props.getInteger("caustic_rad_estimation_count", 10000);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray);
        }
        Color3f Lo(0.0f);
        Color3f Li_(0.0f);
        Color3f Lo_c(0.0f);

        EmitterQueryRecord emitterRecord(its.p);

        const std::vector<Emitter *> lights = scene->getLights();
        for(Emitter* em : lights){
            Color3f Li_light = em->sample(emitterRecord, sampler->next2D(), 0.);
            Ray3f shadowRay(its.p, emitterRecord.wi);
            Intersection shadowIts;
            if (!scene->rayIntersect(shadowRay, shadowIts) && (shadowIts.t - Epsilon) >= (emitterRecord.dist )){
                // cout << "Ligth intersection " << ray.toString() << endl;
                float pdfPoint = em->pdf(emitterRecord);
                float cosTheta = its.shFrame.n.dot(emitterRecord.wi);
                Li_ += Li_light ;
            }
        }

        if(its.mesh->getBSDF()->isDiffuse()) {
            Lo = estimateIrradiance(its, ray.d, EPhotonMap::EGlobalPhotonMap, sampler);
            Lo_c = estimateIrradiance(its, ray.d, EPhotonMap::ECausticPhotonMap, sampler);
        } else {
            bool diffuse = false;
            Ray3f nextRay = ray;
            while (!diffuse)
            {
                BSDFQueryRecord x_bRec = BSDFQueryRecord(its.toLocal(-nextRay.d), its.uv);
                const BSDF *x_BSDF = its.mesh->getBSDF();
                
                x_BSDF->sample(x_bRec, sampler->next2D());
                nextRay = Ray3f(its.p, its.toWorld(x_bRec.wo), Epsilon, INFINITY);
                
                
                
                if(!scene->rayIntersect(nextRay, its)){
                    break;
                }
                if(its.mesh->getBSDF()->isDiffuse()) {
                    diffuse = true;
                }
            }
            if(diffuse) {
                Lo = estimateIrradiance(its, nextRay.d, EPhotonMap::EGlobalPhotonMap, sampler);
                Lo_c = estimateIrradiance(its, nextRay.d, EPhotonMap::ECausticPhotonMap, sampler);
            }
            
        }
        return  Lo + Lo_c;
    }

    Color3f estimateIrradiance(Intersection its, Vector3f wi, EPhotonMap map, Sampler *sampler) const {
        Color3f Lo(0.f);

        Point3f p = its.p;

        float radius_2, search_radius;
        size_t estimateCount, n_numPhotonShot;
        const my_kd_tree_t *m_index = nullptr;
        const PointCloud   *photons = nullptr;

        if(map == EPhotonMap::EGlobalPhotonMap) {
            estimateCount = rad_estimation_count;
            n_numPhotonShot = max_photon_count;
            search_radius = rad_estimation_radius;
            m_index = &index;
            photons = &cloud;
        } else {
            estimateCount = caustic_rad_estimation_count;
            n_numPhotonShot = caustic_max_photon_count;
            search_radius = caustic_rad_estimation_radius;
            m_index = &caustic_index;
            photons = &caustic_cloud;
        }

        size_t num_results = estimateCount;

        /// for N closet points search
        std::vector<std::pair<size_t, float>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;

        num_results = m_index->radiusSearch(&p.x(), search_radius, ret_matches, params);

        if(num_results > estimateCount) {
            num_results = estimateCount;
            radius_2 = ret_matches[num_results-1].second * ret_matches[num_results-1].second;
        } else {
            radius_2 = search_radius * search_radius;
        }

        float alpha = 0.918, beta = 1.953;
        

        BSDFQueryRecord x_bRec = BSDFQueryRecord(its.toLocal(-wi), its.uv);
        const BSDF *x_BSDF = its.mesh->getBSDF();
        Color3f c = x_BSDF->sample(x_bRec, sampler->next2D());
        // cout << "c = " << c.r() << " " << c.g() << " " << c.b() << endl;
        for(std::pair<size_t, float> photon : ret_matches){
            float gaussianKenel = alpha * (1 - ((1- 1 / exp(beta* photon.second * photon.second / (radius_2*2)))/(1-1/exp(beta))));
            Lo += c * photons->pts[photon.first].power * gaussianKenel;
        }
        // cout << "Lo = " << Lo << endl;
        
        return Lo;


    }

    std::string toString() const {
        return tfm::format(
                "PhotonMappingIntegrator[\n"
                "  GlobalPhotonMap:\n photon count = %i,\n"
                "  radiance estimation count = %i,\n"
                "  radiance estimation radius = %f,\n"
                "  CausticPhotonMap:\n photon count = %i,\n"
                "  radiance estimation count = %i,\n"
                "  radiance estimation radius = %f,\n"
                "]",
                max_photon_count,
                rad_estimation_count,
                rad_estimation_radius,
                caustic_max_photon_count,
                caustic_rad_estimation_count,
                caustic_rad_estimation_radius
        );
    }

    private:
        int max_photon_count, caustic_max_photon_count;
        int rad_estimation_count, caustic_rad_estimation_count;
        float rad_estimation_radius, caustic_rad_estimation_radius;


    };

    NORI_REGISTER_CLASS(DispersiveIntegrator, "dispersive_integrator");
NORI_NAMESPACE_END
