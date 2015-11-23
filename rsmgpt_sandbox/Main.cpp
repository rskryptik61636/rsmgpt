#include <stdafx.h>
#include <rsmgptDefns.h>
#include <iostream>
#include <numeric>

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

#if 1
// Sphere structure.
struct Sphere
{
    Vec3 o;   // Sphere origin.
    float  r;   // Sphere radius.
};
#endif // 0

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
        float q = -0.5 * ( B + disc );

        t1 = C / q;
        t2 = q / A;
    }
    else
    {
        float q = -0.5 * ( B - disc );

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

int main( int argc, char *argv[] )
{
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
    Ray ray = { Vec3( 0, 0, 0 ), Vec3( res.x, res.y, res.z ), static_cast<float>( (std::numeric_limits<unsigned>::max)() ) };
    Sphere sphere = { Vec3( 0, 0, 3 ), 1 };
    bool hit;
    sphereIntersect( ray, sphere, hit );
    std::cout << ( hit ? "Yay!" : "Boo!" ) << std::endl;
}