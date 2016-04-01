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

#include "rsmgptGpuTimer.h"
#include "rsmgptResources.h"
#include "DXSampleHelper.h"

namespace rsmgpt
{
    GpuTimer::GpuTimer(
        const UINT nFrames,
        const std::vector<const char*>& timeSpanNames,
        ID3D12Device* pDevice,
        ID3D12CommandQueue* pCmdQueue,
        const BOOL setStablePowerState /*= FALSE*/ )
    {
        // Initialize m_timeSpans.
        //for( auto& span : timeSpanNames )
        for( auto i = 0; i < timeSpanNames.size(); ++i )
        {
            m_timeSpans[ timeSpanNames[ i ] ] = TimeSpan( i );
            //m_timeSpans[ i ].name = timeSpanNames[ i ];
        }

        // nFrames timestamps for each frame.
        const UINT resultCount = 2 * timeSpanNames.size() * nFrames;
        const UINT resultBufferSize = resultCount * sizeof( UINT64 );

        D3D12_QUERY_HEAP_DESC timestampHeapDesc = {};
        timestampHeapDesc.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
        timestampHeapDesc.Count = resultCount;

        //for( UINT i = 0; i < GraphicsAdaptersCount; i++ )
        {
            createCommittedReadbackBuffer(
                pDevice,
                m_timestampResultBuffer,
                resultBufferSize );

            ThrowIfFailed( pDevice->CreateQueryHeap( &timestampHeapDesc, IID_PPV_ARGS( &m_timestampQueryHeap ) ) );
        }

        // Set the stable power state on the device as necessary.
        ThrowIfFailed( pDevice->SetStablePowerState( setStablePowerState ) );

        // Get the timestamp frequency of the compute command queue.
        ThrowIfFailed( pCmdQueue->GetTimestampFrequency( &m_timestampFrequency ) );
    }

    void GpuTimer::startTimer( const UINT frameNum, const char* timeSpanName, ID3D12GraphicsCommandList* pCmdList )
    {
        // Activate the given timer.
        assert( m_timeSpans.find( timeSpanName ) != m_timeSpans.end() && !m_timeSpans[ timeSpanName ].active );
        m_timeSpans[ timeSpanName ].active = true;

        // Insert a start timestamp query for the given time span.
        pCmdList->EndQuery( 
            m_timestampQueryHeap.Get(), 
            D3D12_QUERY_TYPE_TIMESTAMP, 
            getStartTimerIndx( frameNum, timeSpanName ) );
    }

    void GpuTimer::stopTimer( const UINT frameNum, const char* timeSpanName, ID3D12GraphicsCommandList* pCmdList )
    {
        // De-activate the given timer.
        assert( m_timeSpans.find( timeSpanName ) != m_timeSpans.end() && m_timeSpans[ timeSpanName ].active );
        m_timeSpans[ timeSpanName ].active = false;

        // Insert an end timestamp query for the given time span and resolve it.
        const auto startIndx( getStartTimerIndx( frameNum, timeSpanName ) ), stopIndx( getStopTimerIndx( frameNum, timeSpanName ) );
        pCmdList->EndQuery(
            m_timestampQueryHeap.Get(),
            D3D12_QUERY_TYPE_TIMESTAMP,
            stopIndx );

        pCmdList->ResolveQueryData(
            m_timestampQueryHeap.Get(),
            D3D12_QUERY_TYPE_TIMESTAMP,
            startIndx,
            2,
            m_timestampResultBuffer.Get(),
            startIndx * sizeof( UINT64 ) );        
    }

    UINT64 GpuTimer::getTimeSpanInMs( const UINT frameNum, const char* timeSpanName )
    {
        // Get the timestamp values for the given frame and time span from the result buffers.
        const D3D12_RANGE emptyRange = {};
        D3D12_RANGE readRange = {};

        readRange.Begin = getStartTimerIndx( frameNum, timeSpanName ) * sizeof( UINT64 );
        readRange.End = readRange.Begin + 2 * sizeof( UINT64 );

        void* pData = nullptr;
        ThrowIfFailed( m_timestampResultBuffer->Map( 0, &readRange, &pData ) );

        const UINT64* pTimestamps = reinterpret_cast<UINT64*>( static_cast<UINT8*>( pData ) + readRange.Begin );
        const UINT64 timeStampDelta = pTimestamps[ 1 ] - pTimestamps[ 0 ];

        // Unmap with an empty range (written range).
        m_timestampResultBuffer->Unmap( 0, &emptyRange );

        // Compute the duration of the given time span in microseconds.
        m_timeSpans[ timeSpanName ].duration = ( timeStampDelta * 1000000 ) / m_timestampFrequency;
        
        return m_timeSpans[ timeSpanName ].duration;
    }

}   // end of namespace rsmgpt