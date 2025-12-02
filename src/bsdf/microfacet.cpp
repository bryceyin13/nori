/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Microfacet : public BSDF {
public:
    Microfacet(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = propList.getFloat("alpha", 0.1f);

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = propList.getColor("kd", Color3f(0.5f));

        /* To ensure energy conservation, we must scale the 
           specular component by 1-kd. 

           While that is not a particularly realistic model of what 
           happens in reality, this will greatly simplify the 
           implementation. Please see the course staff if you're 
           interested in implementing a more realistic version 
           of this BRDF. */
        m_ks = 1 - m_kd.maxCoeff();
    }

    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
    	// throw NoriException("MicrofacetBRDF::eval(): not implemented!");

        // wh
        Vector3f wh = (bRec.wi + bRec.wo).normalized();

        // F (Fresnel) part
        float F = fresnel(bRec.wi.dot(wh), m_extIOR, m_intIOR);

        // G (Geometry) part
        float b_wi = 1 / (m_alpha * Frame::tanTheta(bRec.wi));
        float b_wo = 1 / (m_alpha * Frame::tanTheta(bRec.wo));

        // wi, wo, n are in the local frame
        // n = (0, 0, 1)
        // so wi.dot(n) = Frame::cosTheta(wi), wo.dot(n) = Frame::cosTheta(wo)
        float xc_wi = bRec.wi.dot(wh) / Frame::cosTheta(bRec.wi) > 0 ? 1 : 0;
        float xc_wo = bRec.wo.dot(wh) / Frame::cosTheta(bRec.wo) > 0 ? 1 : 0;
        float G1_wi = xc_wi * (b_wi < 1.6 ? (3.535 * b_wi + 2.181 * b_wi * b_wi) / (1 + 2.276 * b_wi + 2.577 * b_wi * b_wi)
                                      : 1.0f);
        float G1_wo = xc_wo * (b_wo < 1.6 ? (3.535 * b_wo + 2.181 * b_wo * b_wo) / (1 + 2.276 * b_wo + 2.577 * b_wo * b_wo)
                                      : 1.0f);

        float G = G1_wi * G1_wo;

        // D (Distribution) part
        float D = 
                 // azimuthal part
                 0.5 * INV_PI *
                 // longitudinal part
                 2 * std::exp(- Frame::tanTheta(wh) * Frame::tanTheta(wh) / (m_alpha * m_alpha))
                 / (std::pow(Frame::cosTheta(wh), 3) * m_alpha * m_alpha);

        // specular part
        // ks * F * D * G / (4 * cosTheta(wi) * cosTheta(wo) * cosTheta(wh))

        Color3f specular = m_ks * F * D * G / (4 * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo) * Frame::cosTheta(wh));

        // diffuse part
        // kd / pi
        Color3f diffuse = m_kd * INV_PI;

        // fr = diffuse + specular
        return diffuse + specular;

    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
    	// throw NoriException("MicrofacetBRDF::pdf(): not implemented!");

        if (bRec.wo.z() <= 0) {
            return 0;
        }

        // wh
        Vector3f wh = (bRec.wi + bRec.wo).normalized();

        // D (Distribution) part
        float D = 
                 // azimuthal part
                 0.5 * INV_PI *
                 // longitudinal part
                 2 * std::exp(- Frame::tanTheta(wh) * Frame::tanTheta(wh) / (m_alpha * m_alpha))
                 / (std::pow(Frame::cosTheta(wh), 3) * m_alpha * m_alpha);

        // combine diffuse and specular parts
        float diffuse_pdf = (1 - m_ks) * INV_PI * Frame::cosTheta(bRec.wo);
        float specular_pdf = m_ks * D / (4 * bRec.wo.dot(wh));

        return diffuse_pdf + specular_pdf;
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
    	// throw NoriException("MicrofacetBRDF::sample(): not implemented!");

        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);

        if (_sample.x() < m_ks) {
            // ---- specular part ----
            // sample wh
            Point2f sample(_sample.x() / m_ks, _sample.y());
            Vector3f wh = Warp::squareToBeckmann(sample, m_alpha);

            // compute wo
            bRec.wo = (2 * bRec.wi.dot(wh) * wh - bRec.wi).normalized();

            // check if wo is in the same hemisphere as wi
            if (bRec.wo.dot(Vector3f(0.f, 0.f, 1.f)) <= 0.f)
                return Color3f(0.f);

        } else {
            // ---- diffuse part ----
            // sample wo
            Point2f sample((_sample.x() - m_ks) / (1 - m_ks), _sample.y());
            bRec.wo = Warp::squareToCosineHemisphere(sample);
        }

        return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    std::string toString() const {
        return tfm::format(
            "Microfacet[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "  ks = %f\n"
            "]",
            m_alpha,
            m_intIOR,
            m_extIOR,
            m_kd.toString(),
            m_ks
        );
    }
private:
    float m_alpha;
    float m_intIOR, m_extIOR;
    float m_ks;
    Color3f m_kd;
};

NORI_REGISTER_CLASS(Microfacet, "microfacet");
NORI_NAMESPACE_END
