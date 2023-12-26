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

        for(Emitter *emitter : scene->getLights()){
            totalPower += emitter->power();
        }

        float totalPowerLuminance = 0.2126f * totalPower.r() + 0.7152f * totalPower.g() + 0.0722f * totalPower.b();
        

        // Global Photon Map
        for (Emitter* emitter: scene->getLights()){
            float emitterPowerLuminance = 0.2126f * emitter->power().r() + 0.7152f * emitter->power().g() + 0.0722f * emitter->power().b();

            int photon_count = max_photon_count * emitterPowerLuminance / totalPowerLuminance;
            int n_emitted = 0;
            float p_absorb = 0.1f;

            Color3f flux = emitter->power() / photon_count;

            while(n_emitted < photon_count){
                // Generate a ray from the emitter
                EmitterQueryRecord emitterRecord;
                Color3f Li = emitter->sample(emitterRecord, sampler->next2D(), 0.);
                emitterRecord.ref = emitter->samplePosition(sampler->next2D());
                cout << "emitterRecord.ref = " << emitterRecord.ref << endl;
                Ray3f ray(emitterRecord.ref, emitterRecord.wi);
                Color3f throughput = Color3f(1.0f);

                while (true) {

                    Intersection its;
                    if(!scene->rayIntersect(ray, its)){
                        cout<<"No intersection"<<endl;
                        break;
                    }
                    // cout<<"intersection"<<endl;
                    BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.uv);

                    if(its.mesh->getBSDF()->isDiffuse()) {
                        PointCloud::Point photon;
                        photon.x = its.p.x();
                        photon.y = its.p.y();
                        photon.z = its.p.z();
                        photon.wi = ray.d;
                        photon.power = flux * throughput;
                        Color3f color = its.mesh->getBSDF()->eval(bsdfRecord);
                        throughput *= color;
                        cloud.pts.push_back(photon);
                        cout << "Photon emitted" << endl;
                    }
                    // cout << "Photon absorbed" << endl;
                    if(sampler->next1D() < p_absorb){
                        // cout << "Photon absorbed" << endl;
                        break;
                    }

                    // Next ray
                    Ray3f nextRay = Ray3f(its.p, its.toWorld(bsdfRecord.wo));
                    ray = nextRay;
                    // cout << "Next ray" << endl;
                    
                }
                n_emitted++;
            }
        }
        cout<<"cloud's size "<<cloud.pts.size()<<endl;
        cout<<"caustic cloud's size "<<caustic_cloud.pts.size()<<endl;
        std::cout<<"done\n"<<std::endl;

        /* kd-tree */
        // index.buildIndex();
        // caustic_index.buildIndex();
    }

    PhotonMappingIntegrator(const PropertyList &props)
            : index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)),
                caustic_index(3 /*dim*/, caustic_cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */)){

        max_photon_count = props.getInteger("photon_count", 5);
        rad_estimation_count = props.getInteger("rad_estimation_count", 100);
        rad_estimation_radius = props.getFloat("rad_estimation_radius", 0.05);
        caustic_max_photon_count = props.getInteger("caustic_photon_count", 10000);
        caustic_rad_estimation_count = props.getInteger("caustic_rad_estimation_count", 100);
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        return Color3f(0.0f);
    }

     Color3f estimateIrradiance(Intersection position, Vector3f wi, EPhotonMap map) const {}

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
