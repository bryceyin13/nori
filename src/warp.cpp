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

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    float x = sample.x();
    float y = sample.y();
    return Point2f(x < 0.5f ? sqrt(2.0f * x) - 1.0f : 1.0f - sqrt(2.0f - 2.0f * x), 
                y < 0.5f ? sqrt(2.0f * y) - 1.0f : 1.0f - sqrt(2.0f - 2.0f * y));
    // throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
    float x = p.x() >= -1 && p.x() <= 1 ? 1 - abs(p.x()) : 0;
    float y = p.y() >= -1 && p.y() <= 1 ? 1 - abs(p.y()) : 0;
    return x * y;
    // throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float r = std::sqrt(sample.x());
    float theta = sample.y() * (float)M_PI * 2;
    return Point2f(r * cos(theta), r * sin(theta));
    // throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
}

float Warp::squareToUniformDiskPdf(const Point2f &p) {
    return std::sqrt(p.x() * p.x() + p.y() * p.y()) <= 1.0f ? INV_PI : 0.0f;
    // throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    float phi = sample.x() * M_PI * 2;
    float theta = acos(1 - 2 * sample.y());
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
    // throw NoriException("Warp::squareToUniformSphere() is not yet implemented!");
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    return INV_FOURPI;
    // throw NoriException("Warp::squareToUniformSpherePdf() is not yet implemented!");
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    float phi = sample.x() * M_PI * 2;
    float theta = acos(1 - sample.y());
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
    // throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    return v.z() < 0 ? 0 : INV_TWOPI;
    // throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    float phi = sample.x() * M_PI * 2;
    float theta = acos(sqrt(sample.y()));
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
    // throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    return v.z() < 0 ? 0 : v.z() * INV_PI;
    // return v.z() * INV_PI;
    // throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float phi = M_PI * 2 * sample.x();
    float theta = atan(sqrt(-alpha * alpha * log(1 - sample.y())));
    return Vector3f(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
    // throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    if (m.z() <= 0) 
        return 0;
    return INV_PI * exp(-(m.x() * m.x() + m.y() * m.y()) / (m.z() * m.z() * alpha * alpha)) / (alpha * alpha * m.z() * m.z() * m.z());
    // throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
