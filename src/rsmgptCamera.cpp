/******************************************************************************
* Copyright (c) 2015-2016 Madayi Kolangarakath Rohit Shrinath
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
******************************************************************************/

#include "stdafx.h"
#include "rsmgptCamera.h"

namespace rsmgpt
{
    PerspectiveCamera::PerspectiveCamera(
        const Vec3 eye,
        const Vec3 lookAt,
        const Vec3 up,
        const float focalLength,
        const float lensRadius,
        const float rasterWidth,
        const float rasterHeight,
        const float fov,
        const float m_aspectRatio,
        const float nearDist,
        const float farDist,
        const float motionFactor,
        const float rotationFactor ) :
        m_eye(eye),
        m_lookAt(lookAt),
        m_up(up),
        m_focalDist(focalLength),
        m_lensRadius(lensRadius),
        m_rasterWidth(rasterWidth),
        m_rasterHeight(rasterHeight),
        m_fov(fov),
        m_aspectRatio(m_aspectRatio),
        m_nearDist(nearDist),
        m_farDist(farDist),
        m_motionFactor(motionFactor),
        m_rotationFactor(rotationFactor)
    {
        // display a warning that the eye position and look at point cannot be the same and so,
        // the eye position is being pushed -100 units backward to accommodate this case
        if( m_eye == m_lookAt )
        {
            ::MessageBoxA( NULL, "Eye position cannot be the same as the look at point. Pushing the camera 100 units backwards",
                "Camera initialization", MB_ICONWARNING | MB_OK );
            m_eye.z -= 100.0f;
        }

        // hardcode the axes u, v and n if the possibility of gimbal lock exists
        if( m_eye.x == m_lookAt.x && m_eye.z == m_lookAt.z )
        {
            if( m_eye.y < m_lookAt.y )	// look vertically down
            {
                m_u = Vec3( 1, 0, 0 );	// pointing right
                m_v = Vec3( 0, 0, -1 );	// pointing backwards
                m_n = Vec3( 0, 1, 0 );	// pointing up
            }
            else if( m_eye.y > m_lookAt.y )	// look vertically up
            {
                m_u = Vec3( 1, 0, 0 );	// pointing right
                m_v = Vec3( 0, 0, 1 );	// pointing forwards
                m_n = Vec3( 0, -1, 0 );	// pointing down
            }
        }
        else	// compute the axes u, v and n
        {
            m_n = m_lookAt - m_eye;
            m_n.Normalize();
            m_u = m_up.Cross( m_n );
            m_u.Normalize();
            m_v = m_n.Cross( m_u );
        }

        updateView();	// update the view matrix
        updateProj();	// update the projection matrix
    }

    // roll (rotate about n) the camera by angle radians
    void PerspectiveCamera::roll( const float angle )
    {
        // rotate u and v about n by angle radians
        rotateAxes( m_n, angle, m_u, m_v );
    }

    // pitch (rotate about u) the camera by angle radians
    void PerspectiveCamera::pitch( const float angle )
    {
        // rotate v and n about u by angle radians
        rotateAxes( m_u, angle, m_v, m_n );
    }

    // yaw (rotate about v) the camera by angle radians
    void PerspectiveCamera::yaw( const float angle )
    {
        // rotate u and n about v by angle
        rotateAxes( m_v, angle, m_u, m_n );

        // update the view matrix
        updateView();
    }

    // rotate the camera about the up vector by angle radians (much easier to handle than yaw)
    void PerspectiveCamera::rotateY( const float angle )
    {
        // create a trans matrix to rotate by angle about the up vector
        Mat4 trans = Mat4::CreateFromAxisAngle( m_up, angle );
        //D3DXMatrixRotationAxis(&trans, &rotAxis, angle);	// @TODO: remove when done testing

        // rotate m, v and n using trans
        m_u = Vec3::Transform( m_u, trans );
        m_v = Vec3::Transform( m_v, trans );
        m_n = Vec3::Transform( m_n, trans );

        // update the view matrix
        updateView();
    }

    // slide the camera by du, dv and dn along the u, v and n axes respectively
    void PerspectiveCamera::slide( const float du, const float dv, const float dn )
    {
        // CORRECT APPROACH
        // This causes the camera to slide along its u, v and n axes
#if 1
        m_eye.x += du*m_u.x + dv*m_v.x + dn*m_n.x;
        m_eye.y += du*m_u.y + dv*m_v.y + dn*m_n.y;
        m_eye.z += du*m_u.z + dv*m_v.z + dn*m_n.z;
#endif	// 1

        // WRONG APPROACH
        // This causes the camera to slide along the fundamental x (1, 0, 0), y (0, 1, 0) and z (0, 0, 1) axes
        // instead of along its u, v, and n axes.
        // This breaks down rather quickly once the camera has been moved around sufficiently.
#if 0
        m_eye += Vec3( du, dv, dn );
#endif	// 0

        updateView();
    }

    // mutator func to set the aspect ratio (used when the window is resized)
    void PerspectiveCamera::setAspectRatio( const float aspectRatio )
    {
        // update the aspect ratio and the proj matrix
        m_aspectRatio = aspectRatio;
        updateProj();
    }

    // updates the view matrix using eye, lookAt and up
    void PerspectiveCamera::updateView()
    {
        // build the view matrix by hand
        // the camera matrix is defined (in the DirectX documentation) as:
        // note how this is the transpose of the OpenGL camera matrix, must find out why
        // |ux	vx	nx	0|	dx = -eye.u
        // |uy	vy	ny	0|	dy = -eye.v
        // |uz	vz	nz	0|	dz = -eye.n
        // |dx	dy	dz	1|	
        //Vec3 d(-D3DXVec3Dot(&m_eye, &m_u), -D3DXVec3Dot(&m_eye, &m_v), -D3DXVec3Dot(&m_eye, &m_n));	// @TODO: remove when done testing
        Vec3 d( -m_eye.Dot( m_u ), -m_eye.Dot( m_v ), -m_eye.Dot( m_n ) );
        m_view( 0, 0 ) = m_u.x;	m_view( 0, 1 ) = m_v.x;	m_view( 0, 2 ) = m_n.x;	m_view( 0, 3 ) = 0;
        m_view( 1, 0 ) = m_u.y;	m_view( 1, 1 ) = m_v.y;	m_view( 1, 2 ) = m_n.y;	m_view( 1, 3 ) = 0;
        m_view( 2, 0 ) = m_u.z;	m_view( 2, 1 ) = m_v.z;	m_view( 2, 2 ) = m_n.z;	m_view( 2, 3 ) = 0;
        m_view( 3, 0 ) = d.x;		m_view( 3, 1 ) = d.y;		m_view( 3, 2 ) = d.z;		m_view( 3, 3 ) = 1;

        // Implicitly update the projection matrix since it is based off of the view matrix.
        updateProj();
    }

    // updates the projection matrix using fov, m_aspectRatio, nearDist and farDist
    void PerspectiveCamera::updateProj()
    {
        const float
            screenxmin( m_aspectRatio > 1.f ? -m_aspectRatio : -1 ),    // These seem to be the corners of the window in homogeneous clip space.
            screenxmax( m_aspectRatio > 1.f ? m_aspectRatio : 1 ),
            screenymin( m_aspectRatio < 1.f ? -1.f / m_aspectRatio : -1 ),
            screenymax( m_aspectRatio < 1.f ? 1.f / m_aspectRatio : 1 );

        // Perspective transformation matrix. The canonical form transforms points to homogenous clip space in [-1,1].
        const float zScale( m_farDist / ( m_farDist - m_nearDist ) ), zTrans( -( m_farDist * m_nearDist ) / ( m_farDist - m_nearDist ) );
        const float xScale( 1 / tanf( m_fov / 2 ) ), yScale( xScale );
        const Mat4 cameraToScreen( Mat4::CreateScale( xScale, yScale, zScale ) * Mat4::CreateTranslation( 0, 0, zTrans ) );

        // Transforms from clip space to raster (window) space.
        m_proj = 
            Mat4::CreateTranslation( -screenxmin, -screenymin, 0.f ) *  // Translate to top left corner of viewport.
            Mat4::CreateScale( 1.f / ( screenxmax - screenxmin ), 1.f / ( screenymax - screenymin ), 1.f ) *    // Transform to NDC [0,1].
            Mat4::CreateScale( m_rasterWidth, m_rasterHeight, 1.f );    // Transforms to raster coords [0,width/height-1]
        m_rasterToWorld = ( m_view * cameraToScreen * m_proj ).Invert();    // Inverse transform from raster space to camera space.

        // TODO: Remove when done testing.
#if 0
        // build the projection matrix using D3DXPerspectiveFovLH
// @TODO: build the projection matrix by hand as we did in OpenGLCamera
//D3DXMatrixPerspectiveFovLH(&m_proj, m_fov, m_aspectRatio, m_nearDist, m_farDist);

// define the left, right, bottom and top corners of the projection window
        const float top = m_nearDist * float( tan( m_fov / 2 /** XM_PI / 180.0*/ ) );	// no need to convert to radians because m_fov is already in radians
        const float bottom = -top;
        const float left = bottom * m_aspectRatio;
        const float right = -left;

        // the perspective projection matrix is defined as follows
        // transposed from original OpenGL version with some additional modifications for use in Direct3D
        // emulates D3DXMatrixPerspectiveOffCenterLH
        // also note that pseudo-depth is computed differently than it is in OpenGL
        // |		2N/(right-left)						0					0			0 |	N = m_nearDist
        // |				0					2N/(top-bottom)				0			0 | F = m_farDist
        // |	(left+right)/(left-right)	(top+bottom)/(bottom-top)	(F)/(F-N)		1 |
        // |				0							0				FN/(N-F)		0 |
        m_proj( 0, 0 ) = 2 * m_nearDist / ( right - left );	m_proj( 0, 1 ) = 0;							m_proj( 0, 2 ) = 0;												m_proj( 0, 3 ) = 0;
        m_proj( 1, 0 ) = 0;							m_proj( 1, 1 ) = 2 * m_nearDist / ( top - bottom );	m_proj( 1, 2 ) = 0;												m_proj( 1, 3 ) = 0;
        m_proj( 2, 0 ) = ( left + right ) / ( left - right );	m_proj( 2, 1 ) = ( top + bottom ) / ( bottom - top );	m_proj( 2, 2 ) = ( m_farDist ) / ( m_farDist - m_nearDist );				m_proj( 2, 3 ) = 1;
        m_proj( 3, 0 ) = 0;							m_proj( 3, 1 ) = 0;							m_proj( 3, 2 ) = m_farDist*m_nearDist / ( m_nearDist - m_farDist );		m_proj( 3, 3 ) = 0;
#endif // 0


    }

    // helper function to rotate 2 axes around a given axes by a given angle
    void PerspectiveCamera::rotateAxes( const Vec3 rotAxis, const float angle, Vec3 &axis1, Vec3 &axis2 )
    {
        // create a trans matrix to rotate by angle about rotAxis
        Mat4 trans = Mat4::CreateFromAxisAngle( rotAxis, angle );

        // rotate axis1 and axis2 using trans
        axis1 = Vec3::Transform( axis1, trans );
        axis2 = Vec3::Transform( axis2, trans );

        // update the view matrix
        updateView();
    }


    // mutator func to zoom out (increase the fov)
    void PerspectiveCamera::zoomOut()
    {
        // max possible fov (hardcoding to 45 degress (XM_PI/4 radians) for now, shouldn't really have to change though)
        static const float maxFov( XM_PI / 4 );
        static const float inc( XM_PI / 1800 );

        // increase the field-of-view of the camera to simulate a zoom out effect but ensure that its doesn't cross maxFov
        m_fov = ( m_fov + inc > maxFov ) ? maxFov : m_fov + inc;

        // update the projection matrix
        updateProj();
    }

    // mutator func to zoom in (decrease the fov)
    void PerspectiveCamera::zoomIn()
    {
        // min possible fov (hardcoding to 10 degrees for now)
        static const float minFov( 10 * XM_PI / 180.0f );
        static const float dec( -XM_PI / 1800 );

        // decrease the field-of-view of the camera to simulate a zoom in effect but ensure it doesn't cross minFov
        m_fov = ( m_fov + dec < minFov ) ? minFov : m_fov + dec;

        // update the projection matrix
        updateProj();
    }
}   // end of namespace rsmgpt