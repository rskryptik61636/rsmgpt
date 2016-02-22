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

#pragma once

#include "rsmgptDefns.h"

#include <memory>

namespace rsmgpt
{

    class Camera
    {
    public:

        Camera(
            const Vec3 eye,
            const Vec3 lookAt,
            const Vec3 up,
            const float fov,
            const float aspectRatio,
            const float nearDist,
            const float farDist,
            const float motionFactor,
            const float rotationFactor );

        // accessor func for the motion factor
        inline const float motionFactor()	const { return m_motionFactor; }

        // accessor func for the rotation factor
        inline const float rotationFactor()	const { return m_rotationFactor; }

        // Camera world space position accessor.
        inline Vec3 eyePosW() const { return m_eye; }

        // roll (rotate about n) the camera by angle radians
        void roll( const float angle );

        // pitch (rotate about u) the camera by angle radians
        void pitch( const float angle );

        // yaw (rotate about v) the camera by angle radians
        void yaw( const float angle );

        // rotate the camera about the up vector by angle radians (much easier to handle than yaw)
        void rotateY( const float angle );

        // slide the camera by du, dv and dn along the u, v and n axes respectively
        void slide( const float du, const float dv, const float dn );

        // mutator func to set the aspect ratio (used when the window is resized)
        void setAspectRatio( const float aspectRatio );

        // mutator func to zoom out (increase the fov)
        void zoomOut();

        // mutator func to zoom in (decrease the fov)
        void zoomIn();

    protected:

        // NOTE: Default implementations of update(View/Proj) will be available for use by the child classes.

        // Updates the view matrix using eye, lookAt and up. Needs to be implemented by each child class.
        virtual void updateView() = 0;

        // Updates the projection matrix using fov, aspectRatio, nearDist and farDist. Needs to be implemented by each child class.
        virtual void updateProj() = 0;

        // Helper function to rotate 2 axes around a given axes by a given angle
        void rotateAxes( const Vec3 rotAxis, const float angle, Vec3 &axis1, Vec3 &axis2 );

        // View matrix params
        Vec3 m_eye, m_lookAt, m_up;

        // Projection matrix params
        float m_fov, m_aspectRatio, m_nearDist, m_farDist;

        // Camera basis vectors
        Vec3 m_u, m_v, m_n;

        // View, projection and raster to world matrices
        Mat4 m_view, m_proj, m_rasterToWorld;

        // Motion and rotation factors
        float m_motionFactor, m_rotationFactor;
    };

    class PTPerspectiveCamera : public Camera
    {
    public:
        
        PTPerspectiveCamera( 
            const Vec3 eye, 
            const Vec3 lookAt, 
            const Vec3 up,
            const float focalLength,
            const float lensRadius,
            const float rasterWidth,
            const float rasterHeight,
            const float fov,
            const float aspectRatio, 
            const float nearDist, 
            const float farDist,
            const float motionFactor, 
            const float rotationFactor );

        // Raster to world transformation matrix accessor method.
        inline Mat4 rasterToWorld() const { return m_rasterToWorld; }        

    protected:

        // Updates the view matrix using eye, lookAt and up.
        void updateView() override;

        // Updates the projection matrix using fov, aspectRatio, nearDist and farDist
        void updateProj() override;

        // Camera lens params.
        float m_focalDist, m_lensRadius;

        // Raster params.
        float m_rasterWidth, m_rasterHeight;
    };
    typedef std::unique_ptr<PTPerspectiveCamera> PTPerspectiveCameraPtr;

    class DebugPerspectiveCamera : public Camera
    {
    public:

        DebugPerspectiveCamera(
            const Vec3 eye,
            const Vec3 lookAt,
            const Vec3 up,
            const float fov,
            const float aspectRatio,
            const float nearDist,
            const float farDist,
            const float motionFactor,
            const float rotationFactor );

        // Accessor methods to return the view and projection matrices.
        inline Mat4 view() const { return m_view; }
        inline Mat4 proj() const { return m_proj; }

    protected:

        // Updates the view matrix using eye, lookAt and up.
        void updateView() override;

        // Updates the projection matrix using fov, aspectRatio, nearDist and farDist
        void updateProj() override;
    };
    typedef std::unique_ptr<DebugPerspectiveCamera> DebugPerspectiveCameraPtr;

}   // end of namespace rsmgpt
