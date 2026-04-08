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

NORI_NAMESPACE_BEGIN

/// Ideal dielectric BSDF
class Dielectric : public BSDF {
public:
    Dielectric(const PropertyList &propList) {
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
        bRec.measure = EDiscrete;

        float cosThetaI = Frame::cosTheta(bRec.wi);
        bool entering = cosThetaI > 0.f; // true: from outside to inside

        float etaI = entering ? m_extIOR : m_intIOR;
        float etaT = entering ? m_intIOR : m_extIOR;
        float eta = etaI / etaT;

        // Fresnel Term -> probability of reflection
        float fr = fresnel(std::abs(cosThetaI), etaI, etaT);

        if (sample.x() < fr) {
            // ---- reflect ----
            bRec.wo = Vector3f(-bRec.wi.x(), -bRec.wi.y(), bRec.wi.z());
            bRec.eta = 1.f;
            return Color3f(1.f);
            // return Color3f(fr);
        } else {
            // ---- refract ----
            Vector3f n(0.f, 0.f, entering ? 1.f : -1.f);

            float cosThetaI = bRec.wi.dot(n);
            float sign = cosThetaI >= 0.f ? 1.f : -1.f;
            cosThetaI = std::abs(cosThetaI);

            // Snell's Law : check Total Internal Reflection 
            float sin2ThetaI = std::max(0.f, 1.f - cosThetaI * cosThetaI);
            float sin2ThetaT = eta * eta * sin2ThetaI;

            float cosThetaT = std::sqrt(std::max(0.f, 1.f - sin2ThetaT));
            // direction：η * (-wi) + (η * cosθi - cosθt) * n
            bRec.wo = (eta * (-bRec.wi) + (eta * cosThetaI - cosThetaT) * sign * n).normalized();

            bRec.eta = eta;
            return Color3f(eta * eta);
        }
        // throw NoriException("Unimplemented!");
    }

    std::string toString() const {
        return tfm::format(
            "Dielectric[\n"
            "  intIOR = %f,\n"
            "  extIOR = %f\n"
            "]",
            m_intIOR, m_extIOR);
    }
private:
    float m_intIOR, m_extIOR;
};

NORI_REGISTER_CLASS(Dielectric, "dielectric");
NORI_NAMESPACE_END
