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

#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

struct EmitterQueryRecord {
    
    
    Point3f p;         // shading point
    Point3f light_p;   // point on light source
    Normal3f n;        // normal vector at shading point
    Vector3f wi;       // direction from shading point to point on light source ( point on light source - shading point)

    float pdf;
    Ray3f shadowRay;

    EmitterQueryRecord(const Point3f& _p) : p(_p) {}

    EmitterQueryRecord(const Point3f& _p, const Point3f& _light_p, const Normal3f& _n) : p(_p), light_p(_light_p), n(_n), wi((light_p - p).normalized()) {}
};

/**
 * \brief Superclass of all emitters
 */
class Emitter : public NoriObject {
public:

    virtual ~Emitter() {}
    virtual Color3f eval(const EmitterQueryRecord& eRec) const = 0;
    virtual Color3f getRadiance() const = 0;
    virtual float pdf(const Mesh* mesh, const EmitterQueryRecord& eRec) const = 0;
    virtual Color3f sample(const Mesh* mesh, EmitterQueryRecord& eRec, Sampler*) const = 0;

    /**
     * \brief Return the type of object (i.e. Mesh/Emitter/etc.) 
     * provided by this instance
     * */
    EClassType getClassType() const { return EEmitter; }
};

NORI_NAMESPACE_END
