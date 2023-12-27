#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/emitter.h>

#include <nori/nanoflann.hpp>
#include <ctime>
#include <cstdlib>
#include <iostream>

using namespace std;
using namespace nanoflann;
NORI_NAMESPACE_BEGIN

class PhotonMappingIntegrator : public Integrator {
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

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
            L2_Simple_Adaptor<float, PointCloud> ,
            PointCloud,
            3 /* dim */
    > my_kd_tree_t;

    PointCloud cloud, caustic_cloud;
    my_kd_tree_t  index, caustic_index; //(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );

    void preprocess(const Scene *scene) {
        // Randomize Seed
        cout<<"photon_mapping..."<<endl;
        std::cout<<"Generating "<< max_photon_count << " photons for Global Photon Map"<< std::endl;
        std::cout<<"Generating "<< caustic_max_photon_count << " photons for Caustic Photon Map"<< std::endl;

        std::unique_ptr<Sampler> sampler(scene->getSampler()->clone());

        Color3f totalPower = Color3f(0.0f);
        float totalPowerLuminance = 0.0f;

        for(Emitter *emitter : scene->getLights()){
            totalPowerLuminance += emitter->power().getLuminance();
        }

        // float totalPowerLuminance = 0.27f * totalPower.r() + 0.67f * totalPower.g() + 0.06f * totalPower.b();
        

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
        
        cout<<"cloud's size "<<cloud.pts.size()<<endl;
        cout<<"caustic cloud's size "<<caustic_cloud.pts.size()<<endl;
        std::cout<<"done\n"<<std::endl;

        /* kd-tree */
        index.buildIndex();
        caustic_index.buildIndex();
    }

    PhotonMappingIntegrator(const PropertyList &props)
            : index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)),
                caustic_index(3 /*dim*/, caustic_cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)){

        max_photon_count = props.getInteger("photon_count", 100000);
        rad_estimation_count = props.getInteger("rad_estimation_count", 10000);
        rad_estimation_radius = props.getFloat("rad_estimation_radius", 0.1);
        caustic_max_photon_count = props.getInteger("caustic_photon_count", 10000);
        caustic_rad_estimation_count = props.getInteger("caustic_rad_estimation_count", 100000);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        
        Intersection its;
        if (!scene->rayIntersect(ray, its)) {
            return scene->getBackground(ray);
        }
        Color3f Lo(0.0f);
        Color3f Li(0.0f);

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
                Li += Li_light ;
            }
        }

        Lo = estimateIrradiance(its, ray.d, EPhotonMap::EGlobalPhotonMap, sampler);
        // if Lo is not black, print a message
        // if(Lo.r() > 0 || Lo.g() > 0 || Lo.b() > 0) {
        //     cout << "Lo: " << Lo.r() << " " << Lo.g() << " " << Lo.b() << endl;
        // }

        // if Li is not black, print a message
        // if(Li.r() > 0 || Li.g() > 0 || Li.b() > 0) {
        //     cout << "Li: " << Li.r() << " " << Li.g() << " " << Li.b() << endl;
        // }


        // print final result
        // cout << "Lo: " << Lo.r() << " " << Lo.g() << " " << Lo.b() << endl;
        return  Lo;
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
            search_radius = rad_estimation_radius;
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

    // Color3f estimateIrradiance(Intersection position, Vector3f wi, EPhotonMap map, Sampler *sampler) const {
    //     Intersection its = position;
    //     Point3f p = its.p;

    //     float radius_2, search_radius;
    //     size_t estimateCount, n_numPhotonShot;
    //     const my_kd_tree_t *m_index = nullptr;
    //     const PointCloud   *photons = nullptr;

    //     if(map == EPhotonMap::EGlobalPhotonMap) {
    //         estimateCount = rad_estimation_count;
    //         n_numPhotonShot = max_photon_count;
    //         search_radius = rad_estimation_radius;
    //         m_index = &index;
    //         photons = &cloud;
    //     } else {
    //         estimateCount = caustic_rad_estimation_count;
    //         n_numPhotonShot = caustic_max_photon_count;
    //         search_radius = rad_estimation_radius;
    //         m_index = &caustic_index;
    //         photons = &caustic_cloud;
    //     }

    //     size_t num_results = estimateCount;

    //     /// for N closet points search
    //     std::vector<std::pair<size_t, float>> ret_matches;
    //     nanoflann::SearchParams params;
    //     params.sorted = true;

    //     num_results = m_index->radiusSearch(&p.x(), search_radius, ret_matches, params);

    //     if(num_results > estimateCount) {
    //         num_results = estimateCount;
    //         radius_2 = ret_matches[num_results-1].second * ret_matches[num_results-1].second;
    //     } else {
    //         radius_2 = search_radius * search_radius;
    //     }

    //     BSDFQueryRecord x_bRec = BSDFQueryRecord(its.shFrame.toLocal(wi));
    //     const BSDF *x_BSDF = its.mesh->getBSDF();
    //     Color3f accflux = Color3f(0.0f);

    //     if(num_results > 0) {

    //         float invRadius_2 = 1/radius_2;

    //         for (size_t i = 0; i < num_results; i++) {

    //             size_t idx = ret_matches[i].first;
    //             PointCloud::Point photon = photons->pts[idx];
    //             Vector3f wi = photon.wi;
    //             Color3f power = photon.power;
    //             // cout << power.r() << " "  << power.g() << " " << power.b() << endl;
    //             x_bRec.wo = its.toLocal(wi);
    //             x_bRec.measure = ESolidAngle;

    //             float sqredDist = ret_matches[i].second * ret_matches[i].second;
    //             float sqrTerm = 1 - sqredDist* invRadius_2;
    //             Color3f c = x_BSDF->sample(x_bRec, sampler->next2D());
    //             // cout << "eval " << c.r() << " " << c.g() << " " << c.b() << endl; 
    //             // cout << "sqrTerm " << sqrTerm << ", power " << power << endl;
    //             accflux += power * (sqrTerm * sqrTerm);
    //         }
    //         // 3 is for regulation due to the weight method
    //         float m_scale = 1 / (float)n_numPhotonShot;
    //         accflux *= m_scale * 3 * M_1_PI * invRadius_2;
    //     }

    //     return accflux;

    //     // return Color3f(0.0f);
    // }

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
                rad_estimation_radius
        );
    }

    private:
        int max_photon_count, caustic_max_photon_count;
        int rad_estimation_count, caustic_rad_estimation_count;
        float rad_estimation_radius;


    };

    NORI_REGISTER_CLASS(PhotonMappingIntegrator, "photon_mapping");
NORI_NAMESPACE_END
