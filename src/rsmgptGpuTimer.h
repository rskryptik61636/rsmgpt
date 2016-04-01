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

namespace rsmgpt
{
    struct TimeSpan
    {
        MAKE_DEFAULT_CTOR( TimeSpan )
        TimeSpan( const std::size_t indx ) :
            indx( indx ),
            duration( 0 ),
            active( false )
        { }

        //std::string name;
        std::size_t indx;
        UINT64 duration;
        bool active;
    };

    //typedef std::pair<const char*, 
    class GpuTimer
    {
    public:

        GpuTimer(
            const UINT nFrames,
            const std::vector<const char*>& timeSpanNames,
            ID3D12Device* pDevice,
            ID3D12CommandQueue* pCmdQueue,
            const BOOL setStablePowerState = FALSE );

        void startTimer( const UINT frameNum, const char* timeSpanName, ID3D12GraphicsCommandList* pCmdList );
        void stopTimer( const UINT frameNum, const char* timeSpanName, ID3D12GraphicsCommandList* pCmdList );
        UINT64 getTimeSpanInMs( const UINT frameNum, const char* timeSpanName );

    private:

        inline UINT getStartTimerIndx( const UINT frameNum, const char* timeSpanName ) const
        {
            return frameNum * m_timeSpans.size() + 2 * m_timeSpans.at( timeSpanName ).indx;
        }

        inline UINT getStopTimerIndx( const UINT frameNum, const char* timeSpanName ) const
        {
            return getStartTimerIndx( frameNum, timeSpanName ) + 1;
        }
        
        //typedef std::pair<UINT64, UINT64> Duration;
        //typedef std::tuple<const char*, Duration, bool> TimeSpan;   // <Name, Duration, Start/Stop flag>
        typedef std::map<const char*, TimeSpan> TimeSpanMap;

        TimeSpanMap m_timeSpans;

        // TODO: Give these simpler names.
        ComPtr<ID3D12QueryHeap> m_timestampQueryHeap;
        ComPtr<ID3D12Resource> m_timestampResultBuffer;
        UINT64 m_timestampFrequency;
    };
    typedef std::unique_ptr<GpuTimer> GpuTimerPtr;

}   // end of namespace rsmgpt
