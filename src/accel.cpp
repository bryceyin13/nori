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

#include <nori/accel.h>
#include <nori/timer.h>
#include <Eigen/Geometry>
#include <numeric>
#include <stack>
#include <queue>

NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh) {
    if (m_mesh)
        throw NoriException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
    m_bbox = m_mesh->getBoundingBox();
}

void Accel::build() {
    // prepare triangle list
    std::vector<uint32_t> list(m_mesh->getTriangleCount());
    std::iota(list.begin(), list.end(), 0);
    Timer timer;
    cout << "Octree construction .. ";
    octree = build(m_bbox, list, 0);
    cout << "done. (took " << timer.elapsedString() << ")" << endl;
}

Node* Accel::build(BoundingBox3f bbox, std::vector<uint32_t> list, uint32_t maxDepth) {
    /* Nothing to do here for now */
    if (list.empty())
        return nullptr;

    if (list.size() <= 10 || maxDepth > 10)
        return new Node(bbox, list, true);

    std::vector<BoundingBox3f> childBbox;
    for (size_t i = 0; i < 8; ++i) {
        Vector3f minPoint;
        Vector3f maxPoint;
        Vector3f center = bbox.getCenter();
        Vector3f corner = bbox.getCorner(i);
        for (size_t j = 0; j < 3; ++j) {
          minPoint[j] = std::min(center[j], corner[j]);
          maxPoint[j] = std::max(center[j], corner[j]);
        }
        childBbox.emplace_back(minPoint, maxPoint);
    }

    std::vector<uint32_t> newList[8];
    for (auto j : list) {
        for (size_t i = 0; i < 8; ++i) {
            if (m_mesh->getBoundingBox(j).overlaps(childBbox[i]))
                newList[i].push_back(j);
        }
    }

    Node* node = new Node(bbox, false);
    for (size_t i = 0; i < 8; ++i)
        node->child[i] = build(childBbox[i], newList[i], maxDepth + 1);
    return node;
}

bool Accel::rayIntersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const {
    bool foundIntersection = false;  // Was an intersection found so far?
    uint32_t f = (uint32_t) -1;      // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)

    /* priority queue traversal 1.6 - 3.5s*/
    std::priority_queue<PQueNode, std::vector<PQueNode>, CompareNearT> queue;
    // std::stack<Node*> stack;
    // stack.push(octree);
    float nearT;
    float farT;
    if (!octree->bbox.rayIntersect(ray, nearT, farT)) 
        return false;
    queue.push(PQueNode(octree, nearT));
    while (!queue.empty()) {
        PQueNode entry = queue.top();
        queue.pop();
        if (entry.nearT > ray.maxt)
            continue;
        Node* node = entry.node;
        // Node* node = stack.top();
        // stack.pop();
        if (!node->isChild) {
            for (size_t i = 0; i < 8; ++i) {
                float nearT;
                float farT;
                if (node->child[i] != nullptr && node->child[i]->bbox.rayIntersect(ray, nearT, farT)) {
                    if (nearT > ray.maxt)
                        continue;
                    // stack.push(node->child[i]);
                    queue.push(PQueNode(node->child[i], nearT));
                }
            }
        } else {
            for (auto idx : node->list) {
                float u, v, t;
                if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
                    if (shadowRay)
                        return true;
                    ray.maxt = its.t = t;
                    its.uv = Point2f(u, v);
                    its.mesh = m_mesh;
                    f = idx;
                    foundIntersection = true;
                }
            }
        }
    }
    // /* stack traversal 4.7 - 5.0s*/
    // std::stack<Node*> stack;
    // stack.push(octree);
    // if (!m_bbox.rayIntersect(ray)) 
    //     return false;
    // while (!stack.empty()) {
    //     Node* node = stack.top();
    //     stack.pop();
    //     if (!node->isChild) {
    //         for (size_t i = 0; i < 8; ++i) {
    //             float nearT;
    //             float farT;
    //             if (node->child[i] != nullptr && node->child[i]->bbox.rayIntersect(ray, nearT, farT)) {
    //                 if (nearT > ray.maxt)
    //                     continue;
    //                 stack.push(node->child[i]);
    //             }
    //         }
    //     } else {
    //         for (auto idx : node->list) {
    //             float u, v, t;
    //             if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
    //                 if (shadowRay)
    //                     return true;
    //                 ray.maxt = its.t = t;
    //                 its.uv = Point2f(u, v);
    //                 its.mesh = m_mesh;
    //                 f = idx;
    //                 foundIntersection = true;
    //             }
    //         }
    //     }
    // }

    //  && t < ray.maxt
    // /* Brute force search through all triangles */
    // for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
    //     float u, v, t;
    //     if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
    //         /* An intersection was found! Can terminate
    //            immediately if this is a shadow ray query */
    //         if (shadowRay)
    //             return true;
    //         ray.maxt = its.t = t;
    //         its.uv = Point2f(u, v);
    //         its.mesh = m_mesh;
    //         f = idx;
    //         foundIntersection = true;
    //     }
    // }

    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1-its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh *mesh   = its.mesh;
        const MatrixXf &V  = mesh->getVertexPositions();
        const MatrixXf &N  = mesh->getVertexNormals();
        const MatrixXf &UV = mesh->getVertexTexCoords();
        const MatrixXu &F  = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
                bary.y() * UV.col(idx1) +
                bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1-p0).cross(p2-p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                 bary.y() * N.col(idx1) +
                 bary.z() * N.col(idx2)).normalized());
        } else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

NORI_NAMESPACE_END

