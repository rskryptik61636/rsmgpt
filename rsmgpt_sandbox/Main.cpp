#include <stdafx.h>
#include <rsmgptDefns.h>
#include <iostream>
#include <numeric>

//#pragma push_macro("max")
//#include <algorithm>
//#pragma pop_macro("max")

#pragma message("Linking against rsmgpt.lib")
#pragma comment(lib, "rsmgpt.lib")

#pragma message("Linking against DirectXTK.lib")
#pragma comment(lib, "DirectXTK.lib")

using namespace rsmgpt;

// Ray structure.
struct Ray
{
    Vec3 o;   // Ray origin.
    Vec3 d;   // Ray direction.
    float tMax; // Ray max intersection parameter.
};

// Sphere structure.
struct Sphere
{
    Vec3 o;   // Sphere origin.
    float  r;   // Sphere radius.
};

// Triangle structure.
struct Triangle
{
    Vec3 v0, v1, v2;
};

// Quadratic equation solver.
bool quadratic( const float A, const float B, const float C, float& t1, float& t2 )
{
    // Calculate the discriminant: sqrt(B^2 - 4*A*C).
    float disc = B * B - 4 * A * C;
    if( disc < 0 )
    {
        t1 = t2 = 0;
        return false;
    }        

    // Compute the numerically stable solution of the quadratic equation.
    // b <  0: t1 = 2c / (-b + disc); t2 = (-b + disc) / 2a
    // b >= 0: t1 = (-b - disc) / 2a; t2 = 2c / (-b - disc)
    if( B < 0 )
    {
        float q = -0.5f * ( B + disc );

        t1 = C / q;
        t2 = q / A;
    }
    else
    {
        float q = -0.5f * ( B - disc );

        t1 = q / A;
        t2 = C / q;
    }

    if( t1 > t2 )
    {
        std::swap( t1, t2 );
        /*float t = t1;
        t1 = t2;
        t2 = t;*/
    }

    return true;
}

// Taken from pbrt-v2.
//template <typename T>
int MaxDimension( const Vec3 &v )
{
    return ( v.x > v.y ) ? ( ( v.x > v.z ) ? 0 : 2 ) : ( ( v.y > v.z ) ? 1 : 2 );
}

// Taken from pbrt-v2.
//template <typename T>
Vec3 Abs( const Vec3 &v )
{
    return Vec3( std::abs( v.x ), std::abs( v.y ), std::abs( v.z ) );
}

Vec3 Permute( const Vec3 &v, int x, int y, int z )
{
    return Vec3(
        ( x == 0 ) ? v.x : ( ( x == 1 ) ? v.y : v.z ),
        ( y == 0 ) ? v.x : ( ( y == 1 ) ? v.y : v.z ),
        ( z == 0 ) ? v.x : ( ( z == 1 ) ? v.y : v.z ) );
}

float MaxComponent( const Vec3 &v )
{
    return max( v.x, max( v.y, v.z ) );
}

#define MachineEpsilon std::numeric_limits<float>::epsilon() * 0.5f
inline constexpr float gamma( int n )
{
    return ( n * MachineEpsilon ) / ( 1 - n * MachineEpsilon );
}

// Ray - sphere intersection test.
void sphereIntersect( const Ray& ray, /*in float radius,*/const Sphere& sphere, bool& hit )
{
    // Compute the coefficients for the quadratic equation of the sphere:
    // (sx - cx)^2 + (sy - cy)^2 + (sz - cz)^2 = r^2
    // (ox + t.dx - cx)^2 + (oy + t.dy - cy)^2 + (oz + t.dz - cz)^2 = r^2

    // o(x/y/z) - c(x/y/z)
    Vec3 cd = ray.o - sphere.o;

    // A = dx^2 + dy^2 + dz^2 which can be expressed as a dot product.
    float A = ray.d.Dot( ray.d ); //dot( ray.d, ray.d );

    // B = 2 * ( ( ox - cx ) * dx + ( oy - cy ) * dy + ( oz - cz ) * dz) which can also be expressed as a dot product.
    float B = 2 * cd.Dot( ray.d ); //dot( cd, ray.d );

                                // C = ( ( ox - cx )^2 + ( oy - cy )^2 + ( oz - cz )^2 ) - rad * rad which can be expressed as yet another dot product.
    float C = /*dot( cd, cd )*/ cd.Dot( cd ) - sphere.r * sphere.r;

    // Get the roots of the quadratic equation.
    float t1, t2;
    if( !quadratic( A, B, C, t1, t2 ) )
    {
        hit = false;
        return;
    }        

    // Determine whether the sphere was intersected or not based on the values of the roots.
    hit = true;
    if( t1 > ray.tMax || t2 <= 0 )
        hit = false;

    float thit = t1;
    if( t1 <= 0 )
    {
        thit = t2;
        if( thit > ray.tMax )
            hit = false;
    }

    // TODO: Add intersection point computation here.
}

// Ray - triangle intersection test v0 (Moller-Trumbore algo).
void triangleIntersectV1( const Ray& ray, const Triangle& tri, bool& hit, float& t, float& b1, float& b2, const bool cullBackFacing = true )
{
    // Compute the edge vectors for (v1 - v0) and (v2 - v0).
    static constexpr float eps( std::numeric_limits<float>::epsilon() );
    const auto& o( ray.o ), d( ray.d ), v0( tri.v0 ), v1( tri.v1 ), v2( tri.v2 );
    const Vec3 e1( v1 - v0 ), e2( v2 - v0 );

    // Compute s1 = (d x e2).
    const Vec3 s1( d.Cross( e2 ) );

    // Go one of two ways depending on whether back facing triangles are being culled or not.
    if( cullBackFacing )
    {
        // Compute the det = (e1.s1). Reject if < 0.
        const float det( e1.Dot( s1 ) );
        if( det < eps )
        {
            hit = false;
            return;
        }

        // Compute s = o - v0.
        const Vec3 s( o - v0 );// , s1( d.Cross( e2 ) ), s2( s.Cross( e1 ) );

        // Compute b1 = s1.s and return if < 0 or > det.
        b1 = s1.Dot( s );
        if( b1 < 0 || b1 > det )
        {
            hit = false;
            return;
        }

        // Compute s2 = s x e1 and b2 = s2.d and return if < 0 or b1 + b2 > det.
        const Vec3 s2( s.Cross( e1 ) );
        b2 = s2.Dot( d );
        if( b2 < 0 || ( b1 + b2 ) > det )
        {
            hit = false;
            return;
        }

        // Compute t = s2.e2, scale the parameters by the inv of det and set the intersection to be true.
        const float invDet( 1.f / det );
        t = s2.Dot( e2 ) * invDet;
        b1 *= invDet;
        b2 *= invDet;
        hit = true;
    }
    else
    {

    }    
}

// Ray - triangle intersection test v1 (pbrt-v2 algo)
bool triangleIntersectV2( const Ray& ray, const Triangle& tri, float& t, float& b0, float& b1, float& b2 )
{
    // Transform triangle vertices to ray coordinate space
    const Vec3& p0( tri.v0 ), p1( tri.v1 ), p2( tri.v2 );

    // Translate vertices based on ray origin
    Vec3 p0t = p0 - ray.o;
    Vec3 p1t = p1 - ray.o;
    Vec3 p2t = p2 - ray.o;

    // Permute components of triangle vertices and ray direction
    int kz = MaxDimension( Abs( ray.d ) );
    int kx = kz + 1;
    if( kx == 3 ) kx = 0;
    int ky = kx + 1;
    if( ky == 3 ) ky = 0;
    Vec3 d = Permute( ray.d, kx, ky, kz );
    p0t = Permute( p0t, kx, ky, kz );
    p1t = Permute( p1t, kx, ky, kz );
    p2t = Permute( p2t, kx, ky, kz );

    // Apply shear transformation to translated vertex positions
    float Sx = -d.x / d.z;
    float Sy = -d.y / d.z;
    float Sz = 1.f / d.z;
    p0t.x += Sx * p0t.z;
    p0t.y += Sy * p0t.z;
    p1t.x += Sx * p1t.z;
    p1t.y += Sy * p1t.z;
    p2t.x += Sx * p2t.z;
    p2t.y += Sy * p2t.z;

    // Compute edge function coefficients _e0_, _e1_, and _e2_
    float ea0 = p1t.y - p2t.y;
    float eb0 = p2t.x - p1t.x;
    float e0 = -( ea0*( p1t.x + p2t.x ) + eb0*( p1t.y + p2t.y ) ) / 2.f; //p1t.x * p2t.y - p1t.y * p2t.x;

    float ea1 = p2t.y - p0t.y;
    float eb1 = p0t.x - p2t.x;
    float e1 = -( ea1*( p2t.x + p0t.x ) + eb1*( p2t.y + p0t.y ) ) / 2.f; //p2t.x * p0t.y - p2t.y * p0t.x;

    float ea2 = p0t.y - p1t.y;
    float eb2 = p1t.x - p0t.x;
    float e2 = -( ea2*( p0t.x + p1t.x ) + eb2*( p0t.y + p1t.y ) ) / 2.f; //p0t.x * p1t.y - p0t.y * p1t.x;

    // No double precision for now.
#if 0
    // Compute edge function coefficients _e0_, _e1_, and _e2_
    float e0 = p1t.x * p2t.y - p1t.y * p2t.x;
    float e1 = p2t.x * p0t.y - p2t.y * p0t.x;
    float e2 = p0t.x * p1t.y - p0t.y * p1t.x;

    // Fall back to double precision test at triangle edges
    if( sizeof( float ) == sizeof( float ) &&
        ( e0 == 0.0f || e1 == 0.0f || e2 == 0.0f ) )
    {
        double p2txp1ty = (double)p2t.x * (double)p1t.y;
        double p2typ1tx = (double)p2t.y * (double)p1t.x;
        e0 = (float)( p2typ1tx - p2txp1ty );
        double p0txp2ty = (double)p0t.x * (double)p2t.y;
        double p0typ2tx = (double)p0t.y * (double)p2t.x;
        e1 = (float)( p0typ2tx - p0txp2ty );
        double p1txp0ty = (double)p1t.x * (double)p0t.y;
        double p1typ0tx = (double)p1t.y * (double)p0t.x;
        e2 = (float)( p1typ0tx - p1txp0ty );
    }
#endif // 0


    // Perform triangle edge and determinant tests
    if( ( e0 < 0 || e1 < 0 || e2 < 0 ) && ( e0 > 0 || e1 > 0 || e2 > 0 ) )
        return false;
    float det = e0 + e1 + e2;
    if( det == 0 ) return false;

    // Compute scaled hit distance to triangle and test against ray $t$ range
    p0t.z *= Sz;
    p1t.z *= Sz;
    p2t.z *= Sz;
    float tScaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
    if( det < 0 && ( tScaled >= 0 || tScaled < ray.tMax * det ) )
        return false;
    else if( det > 0 && ( tScaled <= 0 || tScaled > ray.tMax * det ) )
        return false;

    // Compute barycentric coordinates and $t$ value for triangle intersection
    float invDet = 1 / det;
    b0 = e0 * invDet;
    b1 = e1 * invDet;
    b2 = e2 * invDet;
    t = tScaled * invDet;

    // Ensure that computed triangle $t$ is conservatively greater than zero

    // Compute $\delta_z$ term for triangle $t$ error bounds
    float maxZt = MaxComponent( Abs( Vec3( p0t.z, p1t.z, p2t.z ) ) );
    float deltaZ = gamma( 3 ) * maxZt;

    // Compute $\delta_x$ and $\delta_y$ terms for triangle $t$ error bounds
    float maxXt = MaxComponent( Abs( Vec3( p0t.x, p1t.x, p2t.x ) ) );
    float maxYt = MaxComponent( Abs( Vec3( p0t.y, p1t.y, p2t.y ) ) );
    float deltaX = gamma( 5 ) * ( maxXt + maxZt );
    float deltaY = gamma( 5 ) * ( maxYt + maxZt );

    // Compute $\delta_e$ term for triangle $t$ error bounds
    float deltaE =
        2 * ( gamma( 2 ) * maxXt * maxYt + deltaY * maxXt + deltaX * maxYt );

    // Compute $\delta_t$ term for triangle $t$ error bounds and check _t_
    float maxE = MaxComponent( Abs( Vec3( e0, e1, e2 ) ) );
    float deltaT = 3 *
        ( gamma( 3 ) * maxE * maxZt + deltaE * maxZt + deltaZ * maxE ) *
        std::abs( invDet );
    if( t <= deltaT ) return false;

    return true;
}

int main( int argc, char *argv[] )
{
    //// Ray triangle coord system test.
    //const Vec3 p0( 0, 0, 0 ), p1( 2, 5, 0 ), p2( 5, 2, 0 );
    //const float b1( 0.25 ), b2( 0.75 );
    //Vec3 u( b1 * ( p1 - p0 ) ), v( b2 * ( p2 - p0 ) );
    //u.Normalize();
    //v.Normalize();

    // Test ray triangle intersection.
    //const Ray ray = { Vec3( 1, 1, 0 ), Vec3( 0, 0, -1 ), std::numeric_limits<float>::infinity() };
    const Ray ray = { Vec3( 0, 0, 0 ), Vec3( -0.780868800, 0.000000000, 0.624695100 ), std::numeric_limits<float>::infinity() };
    //const Triangle tri1 = { Vec3( 0, 0, -2 ), Vec3( 4, 0, -2 ), Vec3( 2, 2, -2 ) };
    const Triangle tri1 = { Vec3( 6, 0, 3 ), Vec3( 8, 2, 3 ), Vec3( 10, 0, 3 ) };
    const Triangle tri2 = { tri1.v0, tri1.v2, tri1.v1 };
    bool hit;
    float t( 0.f ), b0( 0.f ), b1( 0.f ), b2( 0.f );

    triangleIntersectV1( ray, tri1, hit, t, b1, b2 );
    //hit = triangleIntersectV2( ray, tri1, t, b0, b1, b2 );
    std::cout << "tri1 is " << ( hit ? "hit" : "not hit" ) << std::endl;

    triangleIntersectV1( ray, tri2, hit, t, b1, b2 );
    //hit = triangleIntersectV2( ray, tri2, t, b0, b1, b2 );
    std::cout << "tri2 is " << ( hit ? "hit" : "not hit" ) << std::endl;

#if 0
    // Test ray sphere intersection.
// TODO: Testing out camera transforms. Remove when done testing.
    const float m_width( 1280 ), m_height( 1024 ), fWidth = static_cast<float>( m_width ), fHeight = static_cast<float>( m_height );
    const float
        fov( .5f * XM_PI /*90*/ ),
        aspectRatio( fWidth / fHeight ),
        nearPlane( /*1e-2f*/ 1.f ),
        farPlane( 1000.f ),
        screenxmin( aspectRatio > 1.f ? -aspectRatio : -1 ),    // These seem to be the corners of the window in homogeneous clip space.
        screenxmax( aspectRatio > 1.f ? aspectRatio : 1 ),
        screenymin( aspectRatio < 1.f ? -1.f / aspectRatio : -1 ),
        screenymax( aspectRatio < 1.f ? 1.f / aspectRatio : 1 );

    // Perspective transformation matrix. The canonical form transforms points to homogenous clip space in [-1,1].
    const float zScale( farPlane / ( farPlane - nearPlane ) ), zTrans( -( farPlane * nearPlane ) / ( farPlane - nearPlane ) );
    const float xScale( 1 / tanf( fov / 2 ) ), yScale( xScale );
    const Mat4 worldToCamera( Mat4::Identity );
    const Mat4 cameraToScreen( Mat4::CreateScale( xScale, yScale, zScale ) * Mat4::CreateTranslation( 0, 0, zTrans ) );
    //const Mat4 cameraToScreen(
    //    xScale, 0.f, 0.f, 0.f,
    //    0.f, yScale, 0.f, 0.f,
    //    0.f, 0.f, zScale, 1.f,
    //    0.f, 0.f, zTrans, 0.f );  // Transforms from camera space to D3D style clip space (x,y in [-1,1] and z in [0,1])

    // Transforms from clip space to raster (window) space.
    const Mat4 screenToRaster(
        Mat4::CreateTranslation( -screenxmin, -screenymin, 0.f ) *  // Translate to top left corner of viewport.
        Mat4::CreateScale( 1.f / ( screenxmax - screenxmin ), 1.f / ( screenymax - screenymin ), 1.f ) *    // Transform to NDC [0,1].
        Mat4::CreateScale( fWidth, fHeight, 1.f ) );    // Transforms to raster coords [0,width/height-1]
    const Mat4 rasterToWorld(
        ( worldToCamera * cameraToScreen * screenToRaster ).Invert()
        /*worldToCamera.Invert() * cameraToScreen.Invert() * screenToRaster.Invert()*/ );    // Inverse transform from raster space to camera space.

    // TODO: Remove when done testing.
    //const Mat4 rasterToWorld( /*screenToRaster.Invert()*/ cameraToScreen );    // Transforms to raster coords [0,width/height-1]

    // Test transform a raster space coord to camera space.
    Vec3 pt( 0, 0, 0/*, 1*/ );
    //Vec4 pt( 10, 10, 0, 1 );
    Vec3 res = Vec3::Transform( pt, rasterToWorld );
    res.Normalize();
    //Vec4 res( 0, 0, 1, 1 );

    // A ray from the top-left corner of the viewport to the camera should look like this.
    Vec3 vp( 0, 0, 1 ), cam( 4, 4, 4 );
    Vec3 id( vp - cam );
    id.Normalize();

    // Create a ray and sphere and check if they intersect.
    Ray ray = { Vec3( 0, 0, 0 ), Vec3( res.x, res.y, res.z ), static_cast<float>( ( std::numeric_limits<unsigned>::max )( ) ) };
    Sphere sphere = { Vec3( 0, 0, 3 ), 1 };
    bool hit;
    sphereIntersect( ray, sphere, hit );
    std::cout << ( hit ? "Yay!" : "Boo!" ) << std::endl;
#endif // 0

}