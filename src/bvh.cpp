
/*
    pbrt source code is Copyright(c) 1998-2015
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

 // Modified by Madayi Kolangarakath Rohit Shrinath to work with rsmgpt (2016-02-22)

#include "stdafx.h"

// accelerators/bvh.cpp*
#include "bvh.h"
#include "rsmgptResources.h"
//#include "interaction.h"
//#include "paramset.h"
//#include "stats.h"
#include <algorithm>
#include <intrin.h>

//STAT_TIMER("Time/BVH construction", constructionTime);
//STAT_MEMORY_COUNTER("Memory/BVH tree", treeBytes);
//STAT_RATIO("BVH/Primitives per leaf node", totalPrimitives, totalLeafNodes);
//STAT_COUNTER("BVH/Interior nodes", interiorNodes);
//STAT_COUNTER("BVH/Leaf nodes", leafNodes);

namespace rsmgpt
{

// BVHAccel Local Declarations
struct BVHPrimitiveInfo {
    BVHPrimitiveInfo() {}
    BVHPrimitiveInfo(size_t primitiveNumber, const Bounds3f &bounds)
        : primitiveNumber(primitiveNumber),
          bounds(bounds),
          centroid(.5f * bounds.pMin + .5f * bounds.pMax) {}
    size_t primitiveNumber;
    Bounds3f bounds;
    Point3 centroid;
};

struct BVHBuildNode {
    // BVHBuildNode Public Methods
    void InitLeaf(int first, int n, const Bounds3f &b) {
        firstPrimOffset = first;
        nPrimitives = n;
        bounds = b;
        children[0] = children[1] = nullptr;

        // TODO: These are stored in thread-local storage and should be re-instated once we have tbb integrated.
        /*++leafNodes;
        ++totalLeafNodes;
        totalPrimitives += n;*/
    }
    void InitInterior(int axis, BVHBuildNode *c0, BVHBuildNode *c1) {
        children[0] = c0;
        children[1] = c1;
        bounds = Union(c0->bounds, c1->bounds);
        splitAxis = axis;
        nPrimitives = 0;

        // TODO: This is stored in thread-local storage and should be re-instated once we have tbb integrated.
        //++interiorNodes;
    }
    Bounds3f bounds;
    BVHBuildNode *children[2];
    int splitAxis, firstPrimOffset, nPrimitives;
};

struct MortonPrimitive {
    int primitiveIndex;
    uint32_t mortonCode;
};

struct LBVHTreelet {
    int startIndex, nPrimitives;
    BVHBuildNode *buildNodes;
};

struct LinearBVHNode {
    Bounds3f bounds;
    int parentOffset;   // Parent node.
    int siblingOffset;  // Offset of the sibling of this node.
    union {
        int primitivesOffset;   // leaf
        int secondChildOffset;  // interior
    };
    uint16_t nPrimitives;  // 0 -> interior node
    uint8_t axis;          // interior node: xyz
    uint8_t pad[1];        // ensure 32 byte total size
};

// BVHAccel Utility Functions
inline uint32_t LeftShift3(uint32_t x) {
    assert(x <= (1 << 10));
    if (x == (1 << 10)) --x;
    x = (x | (x << 16)) & 0b00000011000000000000000011111111;
    // x = ---- --98 ---- ---- ---- ---- 7654 3210
    x = (x | (x << 8)) & 0b00000011000000001111000000001111;
    // x = ---- --98 ---- ---- 7654 ---- ---- 3210
    x = (x | (x << 4)) & 0b00000011000011000011000011000011;
    // x = ---- --98 ---- 76-- --54 ---- 32-- --10
    x = (x | (x << 2)) & 0b00001001001001001001001001001001;
    // x = ---- 9--8 --7- -6-- 5--4 --3- -2-- 1--0
    return x;
}

inline uint32_t EncodeMorton3(const Vec3 &v) {
    assert(v.x >= 0 && v.x <= (1 << 10));
    assert(v.y >= 0 && v.y <= (1 << 10));
    assert(v.z >= 0 && v.z <= (1 << 10));
    return (LeftShift3(v.z) << 2) | (LeftShift3(v.y) << 1) | LeftShift3(v.x);
}

static void RadixSort(std::vector<MortonPrimitive> *v) {
    std::vector<MortonPrimitive> tempVector(v->size());
    constexpr int bitsPerPass = 6;
    constexpr int nBits = 30;
    assert((nBits % bitsPerPass) == 0);
    constexpr int nPasses = nBits / bitsPerPass;
    for (int pass = 0; pass < nPasses; ++pass) {
        // Perform one pass of radix sort, sorting _bitsPerPass_ bits
        int lowBit = pass * bitsPerPass;

        // Set in and out vector pointers for radix sort pass
        std::vector<MortonPrimitive> &in = (pass & 1) ? tempVector : *v;
        std::vector<MortonPrimitive> &out = (pass & 1) ? *v : tempVector;

        // Count number of zero bits in array for current radix sort bit
        constexpr int nBuckets = 1 << bitsPerPass;
        int bucketCount[nBuckets] = {0};
        constexpr int bitMask = (1 << bitsPerPass) - 1;
        for (const MortonPrimitive &mp : in) {
            int bucket = (mp.mortonCode >> lowBit) & bitMask;
            assert(bucket >= 0 && bucket < nBuckets);
            ++bucketCount[bucket];
        }

        // Compute starting index in output array for each bucket
        int outIndex[nBuckets];
        outIndex[0] = 0;
        for (int i = 1; i < nBuckets; ++i)
            outIndex[i] = outIndex[i - 1] + bucketCount[i - 1];

        // Store sorted values in output array
        for (const MortonPrimitive &mp : in) {
            int bucket = (mp.mortonCode >> lowBit) & bitMask;
            out[outIndex[bucket]++] = mp;
        }
    }
    // Copy final result from _tempVector_, if needed
    if (nPasses & 1) std::swap(*v, tempVector);
}

// Primitive class member defns.

Primitive::Primitive(
    const std::vector<ModelVertex>& vertList,
    const Mat4& trans,
    const unsigned int p0,
    const unsigned int p1,
    const unsigned int p2 ) :
        p0( p0 ), 
        p1( p1 ), 
        p2( p2 ),
        transform( trans ),
        bbox(
            Union(
                Point3( Vec3::Transform( vertList[ p0 ].position, transform ) ),
                Bounds3f(
                    Vec3::Transform( vertList[ p1 ].position, transform ),
                    Vec3::Transform( vertList[ p2 ].position, transform ) ) ) )
{
}

Bounds3f Primitive::WorldBound()
{
    return bbox;
}

bool Primitive::IntersectP( 
    const std::vector<ModelVertex>& vertList, 
    const Ray& ray, 
    float& t,
    float& b1,
    float& b2,
    const bool cullBackFacing /*= true*/ ) const
{
    // Compute the edge vectors for (v1 - v0) and (v2 - v0).
    const Vec3 o = ray.o, d = ray.d, v0 = vertList[ p0 ].position, v1 = vertList[ p1 ].position, v2 = vertList[ p2 ].position;
    const Vec3 e1 = v1 - v0, e2 = v2 - v0;

    // Compute s1 = (d x e2).
    const Vec3 s1 = d.Cross( e2 ); //cross( d, e2 );

    // Compute the det = (e1.s1). Reject if < 0.
    const float det = e1.Dot( s1 ); //dot( e1, s1 );
    if( det < 0.000001f )
    {
        return false;
    }

    // Compute s = o - v0.
    const Vec3 s = o - v0;

    // Compute b1 = s1.s and return if < 0 or > det.
    b1 = s1.Dot( s ); //dot( s1, s );
    if( b1 < 0.f || b1 > det )
    {
        return false;
    }

    // Compute s2 = s x e1 and b2 = s2.d and return if < 0 or b1 + b2 > det.
    const Vec3 s2 = s.Cross( e1 ); //cross( s, e1 );
    b2 = s2.Dot( d ); //dot( s2, d );
    if( b2 < 0.f || ( b1 + b2 ) > det )
    {
        return false;
    }

    // Compute t = s2.e2, scale the parameters by the inv of det and set the intersection to be true.
    const float invDet = 1.f / det;
    t = /*dot( s2, e2 )*/ s2.Dot( e2 ) * invDet;
    b1 *= invDet;
    b2 *= invDet;

    return ( t <= ray.tMax );
    //return true;
}

bool Inside( const Point3 &p, const Bounds3f &b )
{
    return ( p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
        p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z );
}

//template <typename T>
Bounds3f Union( const Bounds3f &b, const Point3 &p )
{
    return Bounds3f(
        Point3( ( std::min )( b.pMin.x, p.x ), ( std::min )( b.pMin.y, p.y ),
            ( std::min )( b.pMin.z, p.z ) ),
        Point3( ( std::max )( b.pMax.x, p.x ), ( std::max )( b.pMax.y, p.y ),
            ( std::max )( b.pMax.z, p.z ) ) );
}

//template <typename T>
Bounds3f Union( const Bounds3f &b1, const Bounds3f &b2 )
{
    return Bounds3f( Point3( ( std::min )( b1.pMin.x, b2.pMin.x ),
        ( std::min )( b1.pMin.y, b2.pMin.y ),
        ( std::min )( b1.pMin.z, b2.pMin.z ) ),
        Point3( ( std::max )( b1.pMax.x, b2.pMax.x ),
            ( std::max )( b1.pMax.y, b2.pMax.y ),
            ( std::max )( b1.pMax.z, b2.pMax.z ) ) );
}

//inline const Point3 &Bounds3f::operator[]( int i ) const
//{
//    assert( i == 0 || i == 1 );
//    return ( i == 0 ) ? pMin : pMax;
//}
//
//inline Point3 &Bounds3f::operator[]( int i )
//{
//    assert( i == 0 || i == 1 );
//    return ( i == 0 ) ? pMin : pMax;
//}

// BVHAccel Method Definitions
BVHAccel::BVHAccel(
    const PrimitiveList &p,
    ID3D12Device* pDevice,
    ID3D12GraphicsCommandList* pCmdList,
    int maxPrimsInNode, 
    SplitMethod splitMethod)
    : maxPrimsInNode((std::min)(255, maxPrimsInNode)),
      primitives(p),
      splitMethod(splitMethod),
      totalNodes(0) {
    //StatTimer buildTime(&constructionTime);   // TODO: Reinstate at some point.
    if (primitives.size() == 0) return;
    // Build BVH from _primitives_

    // Initialize _primitiveInfo_ array for primitives
    std::vector<BVHPrimitiveInfo> primitiveInfo(primitives.size());
    for (size_t i = 0; i < primitives.size(); ++i)
        primitiveInfo[i] = {i, primitives[i].WorldBound()};

    // Build BVH tree for primitives using _primitiveInfo_
    MemoryArena arena(1024 * 1024);
    //int totalNodes = 0;
    PrimitiveList orderedPrims;
    orderedPrims.reserve(primitives.size());
    BVHBuildNode *root;
    if (splitMethod == SplitMethod::HLBVH)
        root = HLBVHBuild(arena, primitiveInfo, &totalNodes, orderedPrims);
    else
        root = recursiveBuild(arena, primitiveInfo, 0, primitives.size(),
                              &totalNodes, orderedPrims);
    primitives.swap(orderedPrims);

    // Create a buffer resource for primitives.
    createCommittedDefaultBuffer(
        pDevice,
        m_pPrimsBuffer,
        D3D12_RESOURCE_STATE_COPY_DEST,                         // Initial resource state is copy dest as the data
        primitives.size() * sizeof( Primitive ),
        D3D12_RESOURCE_FLAG_NONE,
        0,
        nullptr,
        pCmdList,
        m_pPrimsUpload,
        primitives.data() );

    // TODO: totalNodes needs to be in thread local storage.
    /*OutputDebugStringA("BVH created with %d nodes for %d primitives (%.2f MB)", totalNodes,
         (int)primitives.size(),
         float(totalNodes * sizeof(LinearBVHNode)) / (1024.f * 1024.f));*/

    // Compute representation of depth-first traversal of BVH tree
    /*treeBytes += totalNodes * sizeof(LinearBVHNode) + sizeof(*this) +
                 primitives.size() * sizeof(primitives[0]);*/   // TODO: treeBytes needs to be in thread local storage.
    nodes = AllocAligned<LinearBVHNode>(totalNodes);
    ::memset( nodes, 0, totalNodes * sizeof( LinearBVHNode ) );
    int offset = 0, parentOffset = 0;
    flattenBVHTree(parentOffset, root, &offset);
    assert(offset == totalNodes);

    // Create a buffer resource for nodes.
    createCommittedDefaultBuffer(
        pDevice,
        m_pNodesBuffer,
        D3D12_RESOURCE_STATE_COPY_DEST,                         // Initial resource state is copy dest as the data
        totalNodes * sizeof( LinearBVHNode ),
        //D3D12_RESOURCE_FLAG_NONE,
        D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
        0,
        nullptr,
        pCmdList,
        m_pNodesUpload,
        nodes );

    // Transition the nodes resource from copy dest state to unordered access.
    D3D12_RESOURCE_BARRIER nodesBarrier =
        CD3DX12_RESOURCE_BARRIER::Transition(
            m_pNodesBuffer.Get(),
            D3D12_RESOURCE_STATE_COPY_SOURCE,
            D3D12_RESOURCE_STATE_UNORDERED_ACCESS );
    pCmdList->ResourceBarrier( 1, &nodesBarrier );
}

Bounds3f BVHAccel::WorldBound() const {
    return nodes ? nodes[0].bounds : Bounds3f();
}

const UINT BVHAccel::nodeSize() const   { return sizeof( LinearBVHNode ); }

struct BucketInfo {
    int count = 0;
    Bounds3f bounds;
};

BVHBuildNode *BVHAccel::recursiveBuild(
    MemoryArena &arena, std::vector<BVHPrimitiveInfo> &primitiveInfo, int start,
    int end, int *totalNodes,
    PrimitiveList &orderedPrims) {
    assert(start != end);
    BVHBuildNode *node = arena.Alloc<BVHBuildNode>();
    (*totalNodes)++;
    // Compute bounds of all primitives in BVH node
    Bounds3f bounds;
    for (int i = start; i < end; ++i)
        bounds = Union(bounds, primitiveInfo[i].bounds);
    int nPrimitives = end - start;
    if (nPrimitives == 1) {
        // Create leaf _BVHBuildNode_
        int firstPrimOffset = orderedPrims.size();
        for (int i = start; i < end; ++i) {
            int primNum = primitiveInfo[i].primitiveNumber;
            orderedPrims.push_back(primitives[primNum]);
        }
        node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
        return node;
    } else {
        // Compute bound of primitive centroids, choose split dimension _dim_
        Bounds3f centroidBounds;
        for (int i = start; i < end; ++i)
            centroidBounds = Union(centroidBounds, primitiveInfo[i].centroid);
        int dim = centroidBounds.MaximumExtent();

        // Partition primitives into two sets and build children
        int mid = (start + end) / 2;
        if (centroidBounds.pMax[dim] == centroidBounds.pMin[dim]) {
            // Create leaf _BVHBuildNode_
            int firstPrimOffset = orderedPrims.size();
            for (int i = start; i < end; ++i) {
                int primNum = primitiveInfo[i].primitiveNumber;
                orderedPrims.push_back(primitives[primNum]);
            }
            node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
            return node;
        } else {
            // Partition primitives based on _splitMethod_
            switch (splitMethod) {
            case SplitMethod::Middle: {
                // Partition primitives through node's midpoint
                float pmid =
                    (centroidBounds.pMin[dim] + centroidBounds.pMax[dim]) / 2;
                BVHPrimitiveInfo *midPtr = std::partition(
                    &primitiveInfo[start], &primitiveInfo[end - 1] + 1,
                    [dim, pmid](const BVHPrimitiveInfo &pi) {
                        return pi.centroid[dim] < pmid;
                    });
                mid = midPtr - &primitiveInfo[0];
                // For lots of prims with large overlapping bounding boxes, this
                // may fail to partition; in that case don't break and fall
                // through
                // to EqualCounts.
                if (mid != start && mid != end) break;
            }
            case SplitMethod::EqualCounts: {
                // Partition primitives into equally-sized subsets
                mid = (start + end) / 2;
                std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
                                 &primitiveInfo[end - 1] + 1,
                                 [dim](const BVHPrimitiveInfo &a,
                                       const BVHPrimitiveInfo &b) {
                                     return a.centroid[dim] < b.centroid[dim];
                                 });
                break;
            }
            case SplitMethod::SAH:
            default: {
                // Partition primitives using approximate SAH
                if (nPrimitives <= 2) {
                    // Partition primitives into equally-sized subsets
                    mid = (start + end) / 2;
                    std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
                                     &primitiveInfo[end - 1] + 1,
                                     [dim](const BVHPrimitiveInfo &a,
                                           const BVHPrimitiveInfo &b) {
                                         return a.centroid[dim] <
                                                b.centroid[dim];
                                     });
                } else {
                    // Allocate _BucketInfo_ for SAH partition buckets
                    constexpr int nBuckets = 12;
                    BucketInfo buckets[nBuckets];

                    // Initialize _BucketInfo_ for SAH partition buckets
                    for (int i = start; i < end; ++i) {
                        int b = nBuckets *
                                centroidBounds.Offset(
                                    primitiveInfo[i].centroid)[dim];
                        if (b == nBuckets) b = nBuckets - 1;
                        assert(b >= 0 && b < nBuckets);
                        buckets[b].count++;
                        buckets[b].bounds =
                            Union(buckets[b].bounds, primitiveInfo[i].bounds);
                    }

                    // Compute costs for splitting after each bucket
                    float cost[nBuckets - 1];
                    for (int i = 0; i < nBuckets - 1; ++i) {
                        Bounds3f b0, b1;
                        int count0 = 0, count1 = 0;
                        for (int j = 0; j <= i; ++j) {
                            b0 = Union(b0, buckets[j].bounds);
                            count0 += buckets[j].count;
                        }
                        for (int j = i + 1; j < nBuckets; ++j) {
                            b1 = Union(b1, buckets[j].bounds);
                            count1 += buckets[j].count;
                        }
                        cost[i] = 1 +
                                  (count0 * b0.SurfaceArea() +
                                   count1 * b1.SurfaceArea()) /
                                      bounds.SurfaceArea();
                    }

                    // Find bucket to split at that minimizes SAH metric
                    float minCost = cost[0];
                    int minCostSplitBucket = 0;
                    for (int i = 1; i < nBuckets - 1; ++i) {
                        if (cost[i] < minCost) {
                            minCost = cost[i];
                            minCostSplitBucket = i;
                        }
                    }

                    // Either create leaf or split primitives at selected SAH
                    // bucket
                    float leafCost = nPrimitives;
                    if (nPrimitives > maxPrimsInNode || minCost < leafCost) {
                        BVHPrimitiveInfo *pmid = std::partition(
                            &primitiveInfo[start], &primitiveInfo[end - 1] + 1,
                            [=](const BVHPrimitiveInfo &pi) {
                                int b = nBuckets *
                                        centroidBounds.Offset(pi.centroid)[dim];
                                if (b == nBuckets) b = nBuckets - 1;
                                assert(b >= 0 && b < nBuckets);
                                return b <= minCostSplitBucket;
                            });
                        mid = pmid - &primitiveInfo[0];
                    } else {
                        // Create leaf _BVHBuildNode_
                        int firstPrimOffset = orderedPrims.size();
                        for (int i = start; i < end; ++i) {
                            int primNum = primitiveInfo[i].primitiveNumber;
                            orderedPrims.push_back(primitives[primNum]);
                        }
                        node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
                        return node;
                    }
                }
                break;
            }
            }
            node->InitInterior(dim,
                               recursiveBuild(arena, primitiveInfo, start, mid,
                                              totalNodes, orderedPrims),
                               recursiveBuild(arena, primitiveInfo, mid, end,
                                              totalNodes, orderedPrims));
        }
    }
    return node;
}

BVHBuildNode *BVHAccel::HLBVHBuild(
    MemoryArena &arena, const std::vector<BVHPrimitiveInfo> &primitiveInfo,
    int *totalNodes,
    PrimitiveList &orderedPrims) const {
    // Compute bounding box of all primitive centroids
    Bounds3f bounds;
    for (const BVHPrimitiveInfo &pi : primitiveInfo)
        bounds = Union(bounds, pi.centroid);

    // Compute Morton indices of primitives
    std::vector<MortonPrimitive> mortonPrims(primitiveInfo.size());
#ifdef NDEBUG	// NOTE: Added just to keep things simple for debugging. Modify as necessary.
#pragma omp parallel for schedule(dynamic, 512)       // OpenMP
#endif	// NDEBUG
    for( int i = 0; i < primitiveInfo.size(); ++i )    {
        // Initialize _mortonPrims[i]_ for _i_th primitive
        constexpr int mortonBits = 10;
        constexpr int mortonScale = 1 << mortonBits;
        mortonPrims[i].primitiveIndex = primitiveInfo[i].primitiveNumber;
        Vec3 centroidOffset = bounds.Offset(primitiveInfo[i].centroid);
        mortonPrims[i].mortonCode = EncodeMorton3(centroidOffset * mortonScale);
    }//, primitiveInfo.size(), 512);

    // Radix sort primitive Morton indices
    RadixSort(&mortonPrims);

    // Create LBVH treelets at bottom of BVH

    // Find intervals of primitives for each treelet
    std::vector<LBVHTreelet> treeletsToBuild;
    for (int start = 0, end = 1; end <= (int)mortonPrims.size(); ++end) {
        uint32_t mask = 0b00111111111111000000000000000000;
        if (end == (int)mortonPrims.size() ||
            ((mortonPrims[start].mortonCode & mask) !=
             (mortonPrims[end].mortonCode & mask))) {
            // Add entry to _treeletsToBuild_ for this treelet
            int nPrimitives = end - start;
            int maxBVHNodes = 2 * nPrimitives;
            BVHBuildNode *nodes = arena.Alloc<BVHBuildNode>(maxBVHNodes, false);
            treeletsToBuild.push_back({start, nPrimitives, nodes});
            start = end;
        }
    }

    // Create LBVHs for treelets in parallel
    std::atomic<int> atomicTotal(0), orderedPrimsOffset(0);
    orderedPrims.resize(primitives.size());
#ifdef NDEBUG	// NOTE: Added just to keep things simple for debugging. Modify as necessary.
#pragma omp parallel for schedule(dynamic, 1)       // OpenMP
#endif	// NDEBUG
    for( int i = 0; i < treeletsToBuild.size(); ++i )   {
        // Generate _i_th LBVH treelet
        int nodesCreated = 0;
        const int firstBitIndex = 29 - 12;
        LBVHTreelet &tr = treeletsToBuild[i];
        tr.buildNodes =
            emitLBVH(tr.buildNodes, primitiveInfo, &mortonPrims[tr.startIndex],
                     tr.nPrimitives, &nodesCreated, orderedPrims,
                     &orderedPrimsOffset, firstBitIndex);
        atomicTotal += nodesCreated;
    }//, treeletsToBuild.size());
    *totalNodes = atomicTotal;

    // Create and return SAH BVH from LBVH treelets
    std::vector<BVHBuildNode *> finishedTreelets;
    finishedTreelets.reserve(treeletsToBuild.size());
    for (LBVHTreelet &treelet : treeletsToBuild)
        finishedTreelets.push_back(treelet.buildNodes);
    return buildUpperSAH(arena, finishedTreelets, 0, finishedTreelets.size(),
                         totalNodes);
}

BVHBuildNode *BVHAccel::emitLBVH(
    BVHBuildNode *&buildNodes,
    const std::vector<BVHPrimitiveInfo> &primitiveInfo,
    MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
    PrimitiveList &orderedPrims,
    std::atomic<int> *orderedPrimsOffset, int bitIndex) const {
    assert(nPrimitives > 0);
    if (bitIndex == -1 || nPrimitives < maxPrimsInNode) {
        // Create and return leaf node of LBVH treelet
        (*totalNodes)++;
        BVHBuildNode *node = buildNodes++;
        Bounds3f bounds;
        int firstPrimOffset = orderedPrimsOffset->fetch_add(nPrimitives);
        for (int i = 0; i < nPrimitives; ++i) {
            int primitiveIndex = mortonPrims[i].primitiveIndex;
            orderedPrims[firstPrimOffset + i] = primitives[primitiveIndex];
            bounds = Union(bounds, primitiveInfo[primitiveIndex].bounds);
        }
        node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
        return node;
    } else {
        int mask = 1 << bitIndex;
        // Advance to next subtree level if there's no LBVH split for this bit
        if ((mortonPrims[0].mortonCode & mask) ==
            (mortonPrims[nPrimitives - 1].mortonCode & mask))
            return emitLBVH(buildNodes, primitiveInfo, mortonPrims, nPrimitives,
                            totalNodes, orderedPrims, orderedPrimsOffset,
                            bitIndex - 1);

        // Find LBVH split point for this dimension
        int searchStart = 0, searchEnd = nPrimitives - 1;
        while (searchStart + 1 != searchEnd) {
            assert(searchStart != searchEnd);
            int mid = (searchStart + searchEnd) / 2;
            if ((mortonPrims[searchStart].mortonCode & mask) ==
                (mortonPrims[mid].mortonCode & mask))
                searchStart = mid;
            else {
                assert((mortonPrims[mid].mortonCode & mask) ==
                       (mortonPrims[searchEnd].mortonCode & mask));
                searchEnd = mid;
            }
        }
        int splitOffset = searchEnd;
        assert(splitOffset <= nPrimitives - 1);
        assert((mortonPrims[splitOffset - 1].mortonCode & mask) !=
               (mortonPrims[splitOffset].mortonCode & mask));

        // Create and return interior LBVH node
        (*totalNodes)++;
        BVHBuildNode *node = buildNodes++;
        BVHBuildNode *lbvh[2] = {
            emitLBVH(buildNodes, primitiveInfo, mortonPrims, splitOffset,
                     totalNodes, orderedPrims, orderedPrimsOffset,
                     bitIndex - 1),
            emitLBVH(buildNodes, primitiveInfo, &mortonPrims[splitOffset],
                     nPrimitives - splitOffset, totalNodes, orderedPrims,
                     orderedPrimsOffset, bitIndex - 1)};
        int axis = bitIndex % 3;
        node->InitInterior(axis, lbvh[0], lbvh[1]);
        return node;
    }
}

BVHBuildNode *BVHAccel::buildUpperSAH(MemoryArena &arena,
                                      std::vector<BVHBuildNode *> &treeletRoots,
                                      int start, int end,
                                      int *totalNodes) const {
    assert(start < end);
    int nNodes = end - start;
    if (nNodes == 1) return treeletRoots[start];
    (*totalNodes)++;
    BVHBuildNode *node = arena.Alloc<BVHBuildNode>();

    // Compute bounds of all nodes under this HLBVH node
    Bounds3f bounds;
    for (int i = start; i < end; ++i)
        bounds = Union(bounds, treeletRoots[i]->bounds);

    // Compute bound of HLBVH node centroids, choose split dimension _dim_
    Bounds3f centroidBounds;
    for (int i = start; i < end; ++i) {
        Point3 centroid =
            (treeletRoots[i]->bounds.pMin + treeletRoots[i]->bounds.pMax) *
            0.5f;
        centroidBounds = Union(centroidBounds, centroid);
    }
    int dim = centroidBounds.MaximumExtent();
    // FIXME: if this hits, what do we need to do?
    // Make sure the SAH split below does something... ?
    assert(centroidBounds.pMax[dim] != centroidBounds.pMin[dim]);

    // Allocate _BucketInfo_ for SAH partition buckets
    constexpr int nBuckets = 12;
    struct BucketInfo {
        int count = 0;
        Bounds3f bounds;
    };
    BucketInfo buckets[nBuckets];

    // Initialize _BucketInfo_ for HLBVH SAH partition buckets
    for (int i = start; i < end; ++i) {
        float centroid = (treeletRoots[i]->bounds.pMin[dim] +
                          treeletRoots[i]->bounds.pMax[dim]) *
                         0.5f;
        int b =
            nBuckets * ((centroid - centroidBounds.pMin[dim]) /
                        (centroidBounds.pMax[dim] - centroidBounds.pMin[dim]));
        if (b == nBuckets) b = nBuckets - 1;
        assert(b >= 0 && b < nBuckets);
        buckets[b].count++;
        buckets[b].bounds = Union(buckets[b].bounds, treeletRoots[i]->bounds);
    }

    // Compute costs for splitting after each bucket
    float cost[nBuckets - 1];
    for (int i = 0; i < nBuckets - 1; ++i) {
        Bounds3f b0, b1;
        int count0 = 0, count1 = 0;
        for (int j = 0; j <= i; ++j) {
            b0 = Union(b0, buckets[j].bounds);
            count0 += buckets[j].count;
        }
        for (int j = i + 1; j < nBuckets; ++j) {
            b1 = Union(b1, buckets[j].bounds);
            count1 += buckets[j].count;
        }
        cost[i] = .125f +
                  (count0 * b0.SurfaceArea() + count1 * b1.SurfaceArea()) /
                      bounds.SurfaceArea();
    }

    // Find bucket to split at that minimizes SAH metric
    float minCost = cost[0];
    int minCostSplitBucket = 0;
    for (int i = 1; i < nBuckets - 1; ++i) {
        if (cost[i] < minCost) {
            minCost = cost[i];
            minCostSplitBucket = i;
        }
    }

    // Split nodes and create interior HLBVH SAH node
    BVHBuildNode **pmid = std::partition(
        &treeletRoots[start], &treeletRoots[end - 1] + 1,
        [=](const BVHBuildNode *node) {
            float centroid =
                (node->bounds.pMin[dim] + node->bounds.pMax[dim]) * 0.5f;
            int b = nBuckets *
                    ((centroid - centroidBounds.pMin[dim]) /
                     (centroidBounds.pMax[dim] - centroidBounds.pMin[dim]));
            if (b == nBuckets) b = nBuckets - 1;
            assert(b >= 0 && b < nBuckets);
            return b <= minCostSplitBucket;
        });
    int mid = pmid - &treeletRoots[0];
    assert(mid > start && mid < end);
    node->InitInterior(
        dim, buildUpperSAH(arena, treeletRoots, start, mid, totalNodes),
        buildUpperSAH(arena, treeletRoots, mid, end, totalNodes));
    return node;
}

int BVHAccel::flattenBVHTree(const int parentOffset, BVHBuildNode *node, int *offset) {
    LinearBVHNode *linearNode = &nodes[*offset];
    linearNode->bounds = node->bounds;
    linearNode->parentOffset = parentOffset;
    int myOffset = (*offset)++;
    if (node->nPrimitives > 0) {
        assert(!node->children[0] && !node->children[1]);
        assert(node->nPrimitives < 65536);
        linearNode->primitivesOffset = node->firstPrimOffset;
        linearNode->nPrimitives = node->nPrimitives;
    } else {
        // Create interior flattened BVH node
        linearNode->axis = node->splitAxis;
        linearNode->nPrimitives = 0;
        flattenBVHTree(myOffset, node->children[0], offset);
        linearNode->secondChildOffset =
            flattenBVHTree(myOffset, node->children[1], offset);

        // Set the sibling offsets for the child nodes.
        nodes[ *offset ].siblingOffset = linearNode->secondChildOffset;
        nodes[ linearNode->secondChildOffset ].siblingOffset = *offset;
    }
    return myOffset;
}

BVHAccel::~BVHAccel() { FreeAligned(nodes); }

// NOTE: We're going to be doing intersection testing on the GPU. May not need the intersection testing functions right now.
#if 0
bool BVHAccel::Intersect( const Ray &ray, SurfaceInteraction *isect ) const
{
    if( !nodes ) return false;
    ProfilePhase p( Prof::AccelIntersect );
    bool hit = false;
    Vec3 invDir( 1 / ray.d.x, 1 / ray.d.y, 1 / ray.d.z );
    int dirIsNeg[ 3 ] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    // Follow ray through BVH nodes to find primitive intersections
    int toVisitOffset = 0, currentNodeIndex = 0;
    int nodesToVisit[ 64 ];
    while( true )
    {
        const LinearBVHNode *node = &nodes[ currentNodeIndex ];
        // Check ray against BVH node
        if( node->bounds.IntersectP( ray, invDir, dirIsNeg ) )
        {
            if( node->nPrimitives > 0 )
            {
                // Intersect ray with primitives in leaf BVH node
                for( int i = 0; i < node->nPrimitives; ++i )
                    if( primitives[ node->primitivesOffset + i ]->Intersect(
                        ray, isect ) )
                        hit = true;
                if( toVisitOffset == 0 ) break;
                currentNodeIndex = nodesToVisit[ --toVisitOffset ];
            }
            else
            {
                // Put far BVH node on _nodesToVisit_ stack, advance to near
                // node
                if( dirIsNeg[ node->axis ] )
                {
                    nodesToVisit[ toVisitOffset++ ] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                }
                else
                {
                    nodesToVisit[ toVisitOffset++ ] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        }
        else
        {
            if( toVisitOffset == 0 ) break;
            currentNodeIndex = nodesToVisit[ --toVisitOffset ];
        }
    }
    return hit;
}
#endif // 0

bool BVHAccel::IntersectP( const std::vector<ModelVertex>& vertexList, const Ray& ray, Primitive& hitPrim, float& t, float& b1, float& b2 ) const
{
    if( !nodes ) return false;
    //ProfilePhase p( Prof::AccelIntersectP );
    Vec3 invDir( 1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z );
    int dirIsNeg[ 3 ] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    int nodesToVisit[ 64 ];
    int toVisitOffset = 0, currentNodeIndex = 0;
    float boxtMin, boxtMax;
    while( true )
    {
        const LinearBVHNode *node = &nodes[ currentNodeIndex ];
        if( node->bounds.IntersectP( ray, invDir, dirIsNeg, boxtMin ) )
        {
            // Process BVH node _node_ for traversal
            if( node->nPrimitives > 0 )
            {
                for( int i = 0; i < node->nPrimitives; ++i )
                {
                    if( primitives[ node->primitivesOffset + i ].IntersectP( vertexList, ray, t, b1, b2 ) )
                    {
                        hitPrim = primitives[ node->primitivesOffset + i ];
                        return true;
                    }
                }
                if( toVisitOffset == 0 ) break;
                currentNodeIndex = nodesToVisit[ --toVisitOffset ];
            }
            else
            {
                if( dirIsNeg[ node->axis ] )
                {
                    /// second child first
                    nodesToVisit[ toVisitOffset++ ] = currentNodeIndex + 1;
                    currentNodeIndex = node->secondChildOffset;
                }
                else
                {
                    nodesToVisit[ toVisitOffset++ ] = node->secondChildOffset;
                    currentNodeIndex = currentNodeIndex + 1;
                }
            }
        }
        else
        {
            if( toVisitOffset == 0 ) break;
            currentNodeIndex = nodesToVisit[ --toVisitOffset ];
        }
    }
    return false;
}

bool BVHAccel::IntersectStacklessP( const std::vector<ModelVertex>& vertexList, const Ray& ray, Primitive& hitPrim, float& t, float& b1, float& b2 ) const
{
    if( !nodes ) return false;
    //ProfilePhase p( Prof::AccelIntersectP );
    // MBVH2 traversal loop
    const Vec3 invDir = Vec3( 1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z );
    const int dirIsNeg[] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    UINT nodeId = 0, junk, hitPrimIndx;
    UINT bitstack = 0;
    UINT /*parentId = 0,*/ siblingId = 0; // cached node links
    const float oritMax = ray.tMax;
    float tMin = ray.tMax, b1Hit, b2Hit;
    bool stopTraversal = false;

    //for( ; ;)
    while( true )
    {
        // Inner node loop
        while( nodes[ nodeId ].nPrimitives == 0 )
        {
            //// Check if the current node is hit.
            //Bounds3f parentBox = transformBounds( nodes[ nodeId ].bounds, gWorld );
            //if( !boxIntersect( ray, parentBox, invDir, dirIsNeg ) )
            //    break;

            //// Set the parentId to the current nodeId.
            //parentId = nodeId;

            // Check if either of the children will be hit.
            const UINT leftChildIndx = nodeId + 1;
            const UINT rightChildIndx = nodes[ nodeId ].secondChildOffset;
            const Bounds3f leftBox = nodes[ leftChildIndx ].bounds;
            const Bounds3f rightBox = nodes[ rightChildIndx ].bounds;
            float leftBoxt, rightBoxt;
            const bool leftHit = leftBox.IntersectP( ray, invDir, dirIsNeg, leftBoxt ); //boxIntersect( ray, leftBox, invDir, dirIsNeg, leftBoxt );
            const bool rightHit = rightBox.IntersectP( ray, invDir, dirIsNeg, rightBoxt ); //boxIntersect( ray, rightBox, invDir, dirIsNeg, rightBoxt );

            // Terminate this loop if neither of the children were hit.
            if( !leftHit && !rightHit )
                break;

            // Push a 0 onto the bitstack to indicate that we are going to traverse the next level of the tree.
            bitstack <<= 1;

            // If both children are hit, set nodeId to that of the left child.
            if( leftHit && rightHit )
            {
                //nodeId = ( leftBoxt < rightBoxt ) ? leftChildIndx : rightChildIndx; //( near1 < near0 ) ? nid.w : nid.z;

                // Set nodeId to that of the nearest child node.
                if( leftBoxt < rightBoxt )
                {
                    nodeId = leftChildIndx;
                    siblingId = rightChildIndx;
                    nodes[ nodeId ].siblingOffset = rightChildIndx;
                }
                else
                {
                    nodeId = rightChildIndx;
                    siblingId = leftChildIndx;
                    nodes[ nodeId ].siblingOffset = leftChildIndx;
                }

                // Set the lowest bit of the bitstack to 1 to indicate that the near child has been traversed.
                bitstack |= 1;
            }
            else
            {
                //nodeId = hit0 ? nid.z : nid.w;

                // Set nodeId to that of the intersected child.
                //nodeId = leftHit ? leftChildIndx : rightChildIndx;
                if( leftHit )
                {
                    nodeId = leftChildIndx;
                    siblingId = rightChildIndx;
                    nodes[ nodeId ].siblingOffset = rightChildIndx;
                }
                else
                {
                    nodeId = rightChildIndx;
                    siblingId = leftChildIndx;
                    nodes[ nodeId ].siblingOffset = leftChildIndx;
                }
                
            }
        }
        // Leaf node
        if( nodes[ nodeId ].nPrimitives > 0 )
        {
            // Process BVH node _node_ for traversal
            float t, b1, b2;
            for( UINT i = 0; i < nodes[ nodeId ].nPrimitives; ++i )
            {
                // Check if the i'th primitive is hit.
                //Primitive prim = primitives[ nodes[ nodeId ].primitivesOffset + i ];
                /*Triangle tri = {
                    mul( float4( gVertexBuffer[ prim.p0 ].position, 1.0 ), gWorld ).xyz,
                    mul( float4( gVertexBuffer[ prim.p1 ].position, 1.0 ), gWorld ).xyz,
                    mul( float4( gVertexBuffer[ prim.p2 ].position, 1.0 ), gWorld ).xyz };*/
                //if( triangleIntersectWithBackFaceCulling( ray, tri, t, b1, b2 ) )
                if( primitives[ nodes[ nodeId ].primitivesOffset + i ].IntersectP( vertexList, ray, t, b1, b2 ) )
                {
                    // Set tMin to t if it is lesser. This is to ensure that the closest primitive is hit.
                    if( t < tMin )
                    {
                        tMin = t;
                        hitPrimIndx = nodes[ nodeId ].primitivesOffset + i;
                        b1Hit = b1;
                        b2Hit = b2;
                    }
                }
            }
            /*if( toVisitOffset == 0 ) break;
            currentNodeIndex = nodesToVisit[ --toVisitOffset ];*/
        }
        // Backtrack
        while( ( bitstack & 1 ) == 0 )
        {
            // All the nodes have been traversed, we're done.
            if( bitstack == 0 )
            {
                stopTraversal = true;
                break;
            }

            // Set the current nodeId to that of the parent of current node.
            nodeId = nodes[ nodeId ].parentOffset;
            siblingId = nodes[ nodeId ].siblingOffset;
            bitstack >>= 1;
        }

        // End the traversal if necessary.

        if( stopTraversal )
            break;

        if( siblingId != 0 )
            nodeId = siblingId;
        bitstack ^= 1;
    }

    // If tMin is no longer tMax, then we have intersected a primitive.
    if( tMin < oritMax )
    {
        //// Triangle's colour.
        //float4 LColor = gVertexBuffer[ gPrimitives[ hitPrimIndx ].p0 ].color;

        //// Triangle's transformed surface normal.
        //Vec3 N =
        //    b1Hit * gVertexBuffer[ gPrimitives[ hitPrimIndx ].p0 ].normal +
        //    b2Hit * gVertexBuffer[ gPrimitives[ hitPrimIndx ].p1 ].normal +
        //    ( 1 - b1Hit - b2Hit ) * gVertexBuffer[ gPrimitives[ hitPrimIndx ].p2 ].normal;
        //N = mul( N, (float3x3)gWorldInvTranspose );

        //// Half-way vector.
        //Vec3 hitPt = ray.o + tMin * ray.d;
        //Vec3 V = normalize( ray.o - hitPt );
        //Vec3 H = normalize( L + V );

        //// TODO: Remove when done testing.
        ////// Check if the triangle has a diffuse component.
        ////if( dot( -N, -L ) > 0 /*|| dot( N, ray.d ) < 0*/ )
        ////{
        ////    color = float4( 1, 0, 0, 1 );
        ////}

        //// Compute Phong-blinn shading for the intersected triangle.
        //color = calcBlinnPhongLighting( M, LColor, AColor, N, L, H );
        
        return true;
    }

    return false;
}

#if 0
void BVHAccel::PopShortStack( const uint32_t rootLevel, uint32_t& popLevel, uint32_t& level, uint32_t& stackTrail, bool& stopTraversal, int& shortStackOffset, int* pShortStack, LinearBVHNode* pNode )
{
    // Find the first zero bit before the highest one bit in stackTrail.
    constexpr unsigned long msbMask = 0xffffffff;
    unsigned long highestSet;
    _BitScanReverse( &highestSet, msbMask );    // NOTE: The equivalent of this in HLSL is firstbithigh (see https://msdn.microsoft.com/en-us/library/windows/desktop/ff471400%28v=vs.85%29.aspx).
    level = highestSet + 1;

    // Set stackTrail[level] bit.
    stackTrail = ( 1 << level );

    // stopTraversal is true if the rootLevel bit is the only bit set in stackTrail.
    stopTraversal = ( stackTrail & ( 1 << rootLevel ) );

    // Set popLevel to level.
    popLevel = level;

    // If the short stack is empty, set pNode to the root node and level to 0.
    if( !shortStackOffset )
    {
        pNode = nodes;
        level = 0;
    }

    // Set node to whatever is popped from the short stack.
    else
    {
        pNode = &nodes[ shortStackOffset-- ];
    }
}


#endif // 0


void BVHAccel::drawNodes( const Mat4& viewProj, const UINT rootParamIndx, ID3D12GraphicsCommandList* pCmdList ) const
{
    if( !nodes ) return;
    //ProfilePhase p( Prof::AccelIntersectP );
    int nodesToVisit[ 64 ];
    int toVisitOffset = 0, currentNodeIndex = 0;
    DebugBounds dBounds;
    while( true )
    {
        // Draw the bounds for this node.
        const LinearBVHNode *node = &nodes[ currentNodeIndex ];
        dBounds.pMin = node->bounds.pMin;
        dBounds.pMax = node->bounds.pMax;
        dBounds.viewProj = viewProj.Transpose();
        pCmdList->SetGraphicsRoot32BitConstants(
            rootParamIndx,
            sizeof( DebugBounds ) / sizeof( UINT ),
            &dBounds,
            0 );
        pCmdList->DrawInstanced( 2, 1, 0, 0 );

        // Draw the bounds of all the primitives under this node.
        if( node->nPrimitives > 0 )
        {   
            for( int i = 0; i < node->nPrimitives; ++i )
            {
                dBounds.pMin = primitives[ node->primitivesOffset + i ].bbox.pMin;
                dBounds.pMax = primitives[ node->primitivesOffset + i ].bbox.pMax;
                dBounds.viewProj = viewProj.Transpose();
                pCmdList->SetGraphicsRoot32BitConstants(
                    rootParamIndx,
                    sizeof( DebugBounds ) / sizeof( UINT ),
                    &dBounds,
                    0 );
                pCmdList->DrawInstanced( 2, 1, 0, 0 );
            }
            if( toVisitOffset == 0 ) break;
            currentNodeIndex = nodesToVisit[ --toVisitOffset ];
        }
        else
        {
            nodesToVisit[ toVisitOffset++ ] = node->secondChildOffset;
            currentNodeIndex = currentNodeIndex + 1;
        }
    }
}

BVHAccelPtr CreateBVHAccelerator(
    const PrimitiveList &prims,
    const BVHAccel::SplitMethod splitMethod, 
    const int maxPrimsInNode,
    ID3D12Device* pDevice,
    ID3D12GraphicsCommandList* pCmdList ) {
    return std::make_shared<BVHAccel>( prims, pDevice, pCmdList, maxPrimsInNode, splitMethod );

    // TODO: Remove when done testing.
    //std::string splitMethodName = ps.FindOneString("splitmethod", "sah");
    /*BVHAccel::SplitMethod splitMethod;
    if (splitMethodName == "sah")
        splitMethod = BVHAccel::SplitMethod::SAH;
    else if (splitMethodName == "hlbvh")
        splitMethod = BVHAccel::SplitMethod::HLBVH;
    else if (splitMethodName == "middle")
        splitMethod = BVHAccel::SplitMethod::Middle;
    else if (splitMethodName == "equal")
        splitMethod = BVHAccel::SplitMethod::EqualCounts;
    else {
        printf("BVH split method \"%s\" unknown.  Using \"sah\".",
                splitMethodName.c_str());
        splitMethod = BVHAccel::SplitMethod::SAH;
    }*/

    //int maxPrimsInNode = ps.FindOneInt("maxnodeprims", 4);    
}

}   // end of namespace rsmgpt