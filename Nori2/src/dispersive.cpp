#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/reflectance.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dispersive : public BSDF {
public:
    Dispersive(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);
    }

    Color3f eval(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return Color3f(0.0f);
    }

    float pdf(const BSDFQueryRecord &) const {
        /* Discrete BRDFs always evaluate to zero in Nori */
        return 0.0f;
    }

    Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const {
        
        float cosThetaI = Frame::cosTheta(bRec.wi);
        float F = Reflectance::fresnel(cosThetaI, m_extIOR, m_intIOR);

        bRec.measure = EDiscrete;
        
        if (sample[0] < F) // Reflect
        {
            bRec.eta = 1;
            bRec.wo = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
            return 1;
        }
        else
        {
            
            double new_m_extIOR = calculateRefractiveIndex(bRec.wavelength);
            bRec.wo = Reflectance::refract(bRec.wi, Vector3f(0,0,1), m_extIOR, m_intIOR);
            if (cosThetaI < 0.0f) 
                bRec.eta = new_m_extIOR / m_intIOR;
            else
                bRec.eta = m_intIOR / new_m_extIOR;

            return Color3f(bRec.eta);
        }

    }


    bool isDispersive() const {
        return true;
    }

    std::string toString() const {
        return tfm::format(
            "Dispersive[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:

    double calculateRefractiveIndex(double wavelength) const {

        double B1 = 4.658;
        double C1 = 112.5;
        double lambdaSquared = std::pow(wavelength, 2);
        double refractiveIndex = std::sqrt(1 + (B1 * lambdaSquared) / (lambdaSquared - C1));
        return refractiveIndex;
    }

    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dispersive, "dispersive");
NORI_NAMESPACE_END
