
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

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_ACCELERATORS_BVH_H
#define PBRT_ACCELERATORS_BVH_H
//#include "stdafx.h"

// accelerators/bvh.h*
#include "memory.h"
#include "rsmgptDefns.h"
#include "rsmgptGeometry.h"
#include <atomic>

namespace rsmgpt
{

struct BVHBuildNode;

// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;
struct MortonPrimitive;
struct LinearBVHNode;

// NOTE: Adapted from pbrt's pbrt.h.
inline float Lerpf( float t, float v1, float v2 ) { return ( 1 - t ) * v1 + t * v2; }

#ifdef _MSC_VER
#define MaxFloat std::numeric_limits<float>::max()
#define Infinity std::numeric_limits<float>::infinity()
#else
static constexpr Float MaxFloat = std::numeric_limits<Float>::max();
static constexpr Float Infinity = std::numeric_limits<Float>::infinity();
#endif
#ifdef _MSC_VER
#define MachineEpsilon (std::numeric_limits<float>::epsilon() * 0.5)
#else
static constexpr Float MachineEpsilon =
std::numeric_limits<Float>::epsilon() * 0.5;
#endif

inline constexpr float gamma( int n )
{
    return ( n * MachineEpsilon ) / ( 1 - n * MachineEpsilon );
}

// NOTE: Adapted from pbrt's geometry.h.

// Ray Declarations

// NOTE: Removing Medium as a member of the Ray classes as we don't really need it right now.

class Ray
{
public:
    // Ray Public Methods
    Ray() : tMax( Infinity ), time( 0.f )/*, medium( nullptr )*/ {}
    Ray( const Point3 &o, const Vec3 &d, float tMax = Infinity,
        float time = 0.f/*, const Medium *medium = nullptr*/ )
        : o( o ), d( d ), tMax( tMax ), time( time )/*, medium( medium )*/
    {
    }
    Point3 operator()( float t ) const { return o + d * t; }
    //bool HasNaNs() const { return ( o.HasNaNs() || d.HasNaNs() || isNaN( tMax ) ); }  // TODO: Re-enable once HasNaNs implementation is available.
    /*friend std::ostream &operator<<( std::ostream &os, const Ray &r )
    {
        os << "[o=" << r.o << ", d=" << r.d << ", tMax=" << r.tMax
            << ", time=" << r.time << "]";
        return os;
    }*/

    // Ray Public Data
    Point3 o;
    Vec3 d;
    mutable float tMax;
    float time;
    //const Medium *medium;
};

class RayDifferential : public Ray
{
public:
    // RayDifferential Public Methods
    RayDifferential() { hasDifferentials = false; }
    RayDifferential( const Point3 &o, const Vec3 &d, float tMax = Infinity,
        float time = 0.f/*, const Medium *medium = nullptr*/ )
        : Ray( o, d, tMax, time/*, medium*/ )
    {
        hasDifferentials = false;
    }
    RayDifferential( const Ray &ray ) : Ray( ray ) { hasDifferentials = false; }

    // TODO: Re-enable once HasNaNs implementation is available.
    /*bool HasNaNs() const
    {
        return Ray::HasNaNs() ||
            ( hasDifferentials &&
                ( rxOrigin.HasNaNs() || ryOrigin.HasNaNs() ||
                    rxDirection.HasNaNs() || ryDirection.HasNaNs() ) );
    }*/
    
    void ScaleDifferentials( float s )
    {
        rxOrigin = o + ( rxOrigin - o ) * s;
        ryOrigin = o + ( ryOrigin - o ) * s;
        rxDirection = d + ( rxDirection - d ) * s;
        ryDirection = d + ( ryDirection - d ) * s;
    }

    // RayDifferential Public Data
    bool hasDifferentials;
    Point3 rxOrigin, ryOrigin;
    Vec3 rxDirection, ryDirection;
};

class Bounds3f;

bool Inside( const Point3& p, const Bounds3f& b );

inline float Distance( const Point3& p1, const Point3& p2 )
{
    return ( p1 - p2 ).Length();
}

Bounds3f Union( const Bounds3f& b, const Point3& p );

Bounds3f Union( const Bounds3f& b1, const Bounds3f& b2 );

//template <typename float>
class Bounds3f
{
public:
    // Bounds3f Public Methods
    Bounds3f()
    {
        float minNum = std::numeric_limits<float>::lowest();
        float maxNum = (std::numeric_limits<float>::max)();
        pMin = Point3( maxNum, maxNum, maxNum );
        pMax = Point3( minNum, minNum, minNum );
    }
    Bounds3f( const Point3 &p ) : pMin( p ), pMax( p ) {}
    Bounds3f( const Point3 &p1, const Point3 &p2 )
        : pMin( ((std::min))( p1.x, p2.x ), ((std::min))( p1.y, p2.y ),
            ((std::min))( p1.z, p2.z ) ),
        pMax( ((std::max))( p1.x, p2.x ), ((std::max))( p1.y, p2.y ),
            ((std::max))( p1.z, p2.z ) )
    {
    }
    inline const Point3 &Bounds3f::operator[]( int i ) const
    {
        assert( i == 0 || i == 1 );
        return ( i == 0 ) ? pMin : pMax;
    }

    inline Point3 &Bounds3f::operator[]( int i )
    {
        assert( i == 0 || i == 1 );
        return ( i == 0 ) ? pMin : pMax;
    }
    bool operator==( const Bounds3f &b ) const
    {
        return b.pMin == pMin && b.pMax == pMax;
    }
    bool operator!=( const Bounds3f &b ) const
    {
        return b.pMin != pMin || b.pMax != pMax;
    }
    Point3 Corner( int corner ) const
    {
        assert( corner >= 0 && corner < 8 );
        return Point3( ( *this )[ ( corner & 1 ) ].x,
            ( *this )[ ( corner & 2 ) ? 1 : 0 ].y,
            ( *this )[ ( corner & 4 ) ? 1 : 0 ].z );
    }
    Vec3 Diagonal() const { return pMax - pMin; }
    float SurfaceArea() const
    {
        Vec3 d = Diagonal();
        return 2 * ( d.x * d.y + d.x * d.z + d.y * d.z );
    }
    float Volume() const
    {
        Vec3 d = Diagonal();
        return d.x * d.y * d.z;
    }
    int MaximumExtent() const
    {
        Vec3 d = Diagonal();
        if( d.x > d.y && d.x > d.z )
            return 0;
        else if( d.y > d.z )
            return 1;
        else
            return 2;
    }
    Point3 Lerp( const Point3 &t ) const
    {
        return Point3( Lerpf( t.x, pMin.x, pMax.x ),
            Lerpf( t.y, pMin.y, pMax.y ),
            Lerpf( t.z, pMin.z, pMax.z ) );
    }
    Vec3 Offset( const Point3 &p ) const
    {
        Vec3 o = p - pMin;
        if( pMax.x > pMin.x ) o.x /= pMax.x - pMin.x;
        if( pMax.y > pMin.y ) o.y /= pMax.y - pMin.y;
        if( pMax.z > pMin.z ) o.z /= pMax.z - pMin.z;
        return o;
    }
    void BoundingSphere( Point3 *center, float *radius ) const
    {
        *center = ( pMin + pMax ) / 2;
        *radius = Inside( *center, *this ) ? Distance( *center, pMax ) : 0;
    }
    //template <typename U>
    explicit operator Bounds3f() const
    {
        return Bounds3f( ( Point3 )pMin, ( Point3 )pMax );
    }

    // NOTE: We're going to be doing intersection testing on the GPU. May not need the intersection testing functions right now.
#if 0
    bool IntersectP( const Ray &ray, float *hitt0 = nullptr,
        float *hitt1 = nullptr ) const;    
#endif // 0

    inline bool IntersectP( 
        const Ray &ray, 
        const Vec3 &invDir,
        const int dirIsNeg[ 3 ],
        float& tMin ) const
    {
        const Bounds3f &bounds = *this;
        // Check for ray intersection against $x$ and $y$ slabs
        tMin = ( bounds[ dirIsNeg[ 0 ] ].x - ray.o.x ) * invDir.x;
        float tMax = ( bounds[ 1 - dirIsNeg[ 0 ] ].x - ray.o.x ) * invDir.x;
        float tyMin = ( bounds[ dirIsNeg[ 1 ] ].y - ray.o.y ) * invDir.y;
        float tyMax = ( bounds[ 1 - dirIsNeg[ 1 ] ].y - ray.o.y ) * invDir.y;

        // Update _tMax_ and _tyMax_ to ensure robust bounds intersection
        tMax *= 1 + 2 * gamma( 3 );
        tyMax *= 1 + 2 * gamma( 3 );
        if( tMin > tyMax || tyMin > tMax ) return false;
        if( tyMin > tMin ) tMin = tyMin;
        if( tyMax < tMax ) tMax = tyMax;

        // Check for ray intersection against $z$ slab
        float tzMin = ( bounds[ dirIsNeg[ 2 ] ].z - ray.o.z ) * invDir.z;
        float tzMax = ( bounds[ 1 - dirIsNeg[ 2 ] ].z - ray.o.z ) * invDir.z;

        // Update _tzMax_ to ensure robust bounds intersection
        tzMax *= 1 + 2 * gamma( 3 );
        if( tMin > tzMax || tzMin > tMax ) return false;
        if( tzMin > tMin ) tMin = tzMin;
        if( tzMax < tMax ) tMax = tzMax;
        return ( tMin < ray.tMax ) && ( tMax > 0 );
    }

    // Bounds3f Public Data
    Vec3 pMin, pMax;
};

//typedef Bounds2<float> Bounds2f;
//typedef Bounds2<int> Bounds2i;
//typedef Bounds3f<float> Bounds3f;
//typedef Bounds3f<int> Bounds3i;

// TODO: Figure out these need to live.
struct Primitive
{
    unsigned int p0, p1, p2;
    Mat4 transform;
    Bounds3f bbox;

    Primitive() = default;
    Primitive(
        const std::vector<ModelVertex>& vertList,
        const Mat4& trans,
        const unsigned int p0, 
        const unsigned int p1, 
        const unsigned int p2 );

    Bounds3f WorldBound();

    bool IntersectP( 
        const std::vector<ModelVertex>& vertList, 
        const Ray& ray, 
        float& t,
        float& b1,
        float& b2,
        const bool cullBackFacing = true ) const;
};
typedef std::vector<Primitive> PrimitiveList;

// Debug info.
struct DebugInfo
{
    Ray ray;
    Primitive hitPrim;
    UINT nTraversedBounds;
    UINT nTotalPrimIntersections;
};

struct DebugBounds
{
    Vec3 pMin;
    UINT pad1;

    Vec3 pMax;
    UINT pad2;

    Mat4 viewProj;
};

// BVHAccel Declarations
class BVHAccel /*: public Aggregate*/ {
  public:
    // BVHAccel Public Types
    enum class SplitMethod { SAH, HLBVH, Middle, EqualCounts };

    // BVHAccel Public Methods
    BVHAccel(const PrimitiveList &p,
             ID3D12Device* pDevice,
             ID3D12GraphicsCommandList* pCmdList,
             int maxPrimsInNode = 1,
             SplitMethod splitMethod = SplitMethod::SAH);
    Bounds3f WorldBound() const;
    ~BVHAccel();

    ID3D12Resource* primitivesResource() const { return m_pPrimsBuffer.Get(); }
    const size_t nPrimitives() const { return primitives.size(); }
    const UINT primitiveSize() const { return sizeof( Primitive ); }
    ID3D12Resource* nodesResource() const { return m_pNodesBuffer.Get(); }
    const int nBVHNodes() const { return totalNodes; }
    const UINT nodeSize() const;    // NOTE: Defining in bvh.cpp because that is where LinearBVHNode is defined.

    bool IntersectP( const std::vector<ModelVertex>& vertexList, const Ray& ray, Primitive& hitPrim, float& t, float& b1, float& b2 ) const;    
    bool IntersectStacklessP( const std::vector<ModelVertex>& vertexList, const Ray& ray, Primitive& hitPrim, float& t, float& b1, float& b2 ) const;
#if 0
    void PopShortStack( const uint32_t rootLevel, uint32_t& popLevel, uint32_t& level, uint32_t& stackTrail, bool& stopTraversal, int& shortStackOffset, int* pShortStack, LinearBVHNode* pNode );

    bool Intersect( const Ray &ray, SurfaceInteraction *isect ) const;    
#endif // 0

    void drawNodes( const Mat4& viewProj, const UINT rootParamIndx, ID3D12GraphicsCommandList* pCmdList ) const;


  private:
    // BVHAccel Private Methods
    BVHBuildNode *recursiveBuild(
        MemoryArena &arena, std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int start, int end, int *totalNodes,
        PrimitiveList &orderedPrims);
    BVHBuildNode *HLBVHBuild(
        MemoryArena &arena, const std::vector<BVHPrimitiveInfo> &primitiveInfo,
        int *totalNodes,
        PrimitiveList &orderedPrims) const;
    BVHBuildNode *emitLBVH(
        BVHBuildNode *&buildNodes,
        const std::vector<BVHPrimitiveInfo> &primitiveInfo,
        MortonPrimitive *mortonPrims, int nPrimitives, int *totalNodes,
        PrimitiveList &orderedPrims,
        std::atomic<int> *orderedPrimsOffset, int bitIndex) const;
    BVHBuildNode *buildUpperSAH(MemoryArena &arena,
                                std::vector<BVHBuildNode *> &treeletRoots,
                                int start, int end, int *totalNodes) const;
    int flattenBVHTree(const int parentOffset, BVHBuildNode *node, int *offset);

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    PrimitiveList primitives;
    LinearBVHNode *nodes = nullptr;
    int totalNodes;
    ComPtr<ID3D12Resource> m_pPrimsBuffer, m_pPrimsUpload, m_pNodesBuffer, m_pNodesUpload;
};
typedef std::shared_ptr<BVHAccel> BVHAccelPtr;

BVHAccelPtr CreateBVHAccelerator(
    const PrimitiveList &prims, 
    const BVHAccel::SplitMethod splitMethod, 
    const int maxPrimsInNode,
    ID3D12Device* pDevice,
    ID3D12GraphicsCommandList* pCmdList );

}   // end of namespace rsmgpt

#endif  // PBRT_ACCELERATORS_BVH_H
