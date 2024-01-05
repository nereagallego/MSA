#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/reflectance.h>

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dispersive : public BSDF {
public:
    Dispersive(const PropertyList &propList) {
        /* Interior IOR (default: BK7 borosilicate optical glass) */
        // m_intIOR = propList.getFloat("intIOR", 2.4f);
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
        // float new_m_intIOR = calculateRefractiveIndex(bRec.wavelength);
        float F = Reflectance::fresnel(cosThetaI, m_extIOR, m_intIOR);
        // cout << "F: " << F << endl;

        bRec.measure = EDiscrete;
        
        if (sample[0] < F) // Reflect
        {
            bRec.eta = 1;
            bRec.wo = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
            return 1;
        }
        else
        {
            double new_m_intIOR = calculateRefractiveIndex(bRec.wavelength);
            // cout << "new_m_intIOR: " << new_m_intIOR << endl;
            bRec.wo = Reflectance::refract(bRec.wi, Vector3f(0,0,1), m_extIOR, new_m_intIOR);
            // cout << "bRec.wi: " << bRec.wi << endl;

            // Vector3f wi = {0.470202, 0.352178, 0.803897};
            // double aux_intIOR_1 = calculateRefractiveIndex(380);
            // double aux_intIOR_2 = calculateRefractiveIndex(750);
            // Vector3f wo1 = Reflectance::refract(wi, Vector3f(0,0,1), m_extIOR, aux_intIOR_1);
            // Vector3f wo2 = Reflectance::refract(wi, Vector3f(0,0,1), m_extIOR, aux_intIOR_2);

            // cout << "-------------------------------------" << endl;
            // cout << "aux_intIOR_1: " << aux_intIOR_1 << endl;
            // cout << "aux_intIOR_2: " << aux_intIOR_2 << endl;
            // cout << "wo1: " << wo1[0] << " " << wo1[1] << " " << wo1[2] << endl;
            // cout << "wo2: " << wo2[0] << " " << wo2[1] << " " << wo2[2] << endl;
            // cout << "-------------------------------------" << endl;


            // -------------------------------------
            // aux_intIOR_1: 2.47093
            // aux_intIOR_2: 2.40109
            // wo1: -0.190346 -0.142568 -0.970581
            // wo2: -0.195883 -0.146715 -0.968817
            // -------------------------------------

            
            if (cosThetaI < 0.0f) 
                bRec.eta = m_extIOR / m_intIOR;
            else
                bRec.eta = m_intIOR / m_extIOR;

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

        // DIAMOND
        double lambdaSquared = std::pow(wavelength, 2);
        double B1 = 4.658;
        double C1 = 112.5;
        double refractiveIndex = std::sqrt(1 + (B1 * lambdaSquared) / (lambdaSquared - C1 * C1));

        // BK7
        // double lambdaSquared = std::pow(wavelength, 2);

        // // Constants for BK7 glass
        // double B1 = 1.03961212;
        // double B2 = 0.00600069867;
        // double B3 = 0.231792344;
        // double C1 = 0.00600000000;
        // double C2 = 0.0200179144;
        // double C3 = 103.560653;

        // double refractiveIndex = std::sqrt(1 + (B1 * lambdaSquared) / (lambdaSquared - C1)
        //                                 + (B2 * lambdaSquared) / (lambdaSquared - C2)
        //                                 + (B3 * lambdaSquared) / (lambdaSquared - C3));          


        // double lambdaSquared = std::pow(wavelength, 2);

        // // Corrected coefficients for diamond
        // double A = 5.9762;
        // double B = 0.27754;
        // double C = 9.5403;
        // double D = 79.881;
        // double E = 0.002534;
        // double F = 7.3157;

        // double refractiveIndexSquared = 1 + (A * lambdaSquared) / (lambdaSquared - B)
        //                                 + (C * lambdaSquared) / (lambdaSquared - D)
        //                                 + (E * lambdaSquared) / (lambdaSquared - F);

        // return std::sqrt(refractiveIndexSquared);

        return refractiveIndex;
    }

    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dispersive, "dispersive");
NORI_NAMESPACE_END
