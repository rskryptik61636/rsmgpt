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

// NOTE: This file needs to be included after the shader parametes are declared.

#if 0
bool primHitManual( in Ray ray, out float t, out float b1, out float b2, inout float4 color )
{
    for( uint i = 0; i < gNumFaces; ++i )
    {
        // Check if the i'th primitive is hit.
        Primitive prim = gPrimitives[ i ];
        Triangle tri = {
            mul( float4( gVertexBuffer[ prim.p0 ].position, 1.0 ), gWorld ).xyz,
            mul( float4( gVertexBuffer[ prim.p1 ].position, 1.0 ), gWorld ).xyz,
            mul( float4( gVertexBuffer[ prim.p2 ].position, 1.0 ), gWorld ).xyz };
        if( triangleIntersectWithBackFaceCulling( ray, tri, t, b1, b2 ) )
        {
            // Triangle's colour.
            float4 LColor = gVertexBuffer[ prim.p0 ].color;

            // Triangle's transformed surface normal.
            float3 N = mul( gVertexBuffer[ prim.p2 ].normal, ( float3x3 )gWorldInvTranspose );

            // Half-way vector.
            float3 hitPt = ray.o + t * ray.d;
            float3 V = normalize( ray.o - hitPt );
            float3 H = normalize( L + V );

            // TODO: Remove when done testing.
            //// Check if the triangle has a diffuse component.
            //if( dot( -N, -L ) < 0 /*|| dot( N, ray.d ) < 0*/ )
            //{
            //    color = float4( 1, 0, 0, 1 );
            //}

            // Compute Phong-blinn shading for the intersected triangle.
            color = calcBlinnPhongLighting( M, LColor, AColor, -N, -L, H );

            return true;
        }
    }

    return false;
}
#endif // 0

// NOTE: Adapted from BVHAccel's IntersectP method in bvh.cpp.
bool primHit( in Ray ray, in uint3 dispatchThreadId, inout float4 color )
{
    float3 invDir = float3( 1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z );
    const float3 invDirPad = float3(
        addUlpMag( invDir.x, 2 ),
        addUlpMag( invDir.y, 2 ),
        addUlpMag( invDir.z, 2 ) );
    int3 dirIsNeg = int3( invDir.x < 0, invDir.y < 0, invDir.z < 0 );
    int nodesToVisit[ 64 /*32*/ ];   // NOTE: Reducing nodesToVisit to reduce temp register requirements.
    int toVisitOffset = 0, currentNodeIndex = 0;

    float t, b1, b2;
    float oritMax = ray.tMax, tMin = oritMax, b1Hit, b2Hit, boxtMin;
    uint hitPrimIndx, junk;

#ifdef PATH_TRACING_MODE
    // Initialize the debug info's nTotalPrimIntersections to 0.
    if( gGetDebugInfo && dispatchThreadId.x == 0 && dispatchThreadId.y == 0 )
    {
        InterlockedExchange( gDebugInfo[ 0 ].nTotalPrimIntersections, 0, junk );
    }

    bool isDebugThread = ( gGetDebugInfo && dispatchThreadId.x == gCursorPos.x && dispatchThreadId.y == gCursorPos.y );

    // Initialize the debug info's nTraversedBounds to 0.
    if( isDebugThread )
    {
        gDebugInfo[ 0 ].nTraversedBounds = 0;
    }
#else
    bool isDebugThread = true;
#endif // PATH_TRACING_MODE
        
    while( true )
    {
        Bounds bbox = gBVHNodes[ currentNodeIndex ].bounds;
        if( boxIntersect( ray, bbox, invDir, invDirPad, dirIsNeg, boxtMin ) )
        {
#ifdef PATH_TRACING_MODE
            // Increment the debug info's nTraversedBounds.
            if( isDebugThread )
            {
                ++gDebugInfo[ 0 ].nTraversedBounds;
            }
#endif // PATH_TRACING_MODE

            // Process BVH node _node_ for traversal
            uint nPrims = nPrimitives( gBVHNodes[ currentNodeIndex ] );
            if( nPrims > 0 )
            {
                for( uint i = 0; i < nPrims; ++i )
                {
                    // Check if the i'th primitive is hit.
                    Primitive prim = gPrimitives[ gBVHNodes[ currentNodeIndex ].primitivesOrSecondChildOffset + i ];
                    Triangle tri = {
                        gVertexBuffer[ prim.p0 ].position,
                        gVertexBuffer[ prim.p1 ].position,
                        gVertexBuffer[ prim.p2 ].position };
                    //if( triangleIntersectWithoutBackFaceCulling( ray, tri, t, b1, b2 ) )
                    if( triangleIntersectWithBackFaceCulling( ray, tri, t, b1, b2 ) )
                    {
#ifdef PATH_TRACING_MODE
                        // Increment the debug info's nTotalPrimIntersections.
                        if( gGetDebugInfo )
                        {
                            InterlockedAdd( gDebugInfo[ 0 ].nTotalPrimIntersections, 1 );
                        }
#endif // PATH_TRACING_MODE

                        // Set tMin to t if it is lesser. This is to ensure that the closest primitive is hit.
                        if( t < tMin )
                        {
                            tMin = t;
                            hitPrimIndx = gBVHNodes[ currentNodeIndex ].primitivesOrSecondChildOffset + i;
                            b1Hit = b1;
                            b2Hit = b2;
                        }
                    }
                }
                if( toVisitOffset == 0 ) break;
                currentNodeIndex = nodesToVisit[ --toVisitOffset ];
            }
            else
            {
                if( dirIsNeg[ axis( gBVHNodes[ currentNodeIndex ] ) ] )
                {
                    /// second child first
                    nodesToVisit[ toVisitOffset++ ] = currentNodeIndex + 1;
                    currentNodeIndex = gBVHNodes[ currentNodeIndex ].primitivesOrSecondChildOffset;
                }
                else
                {
                    nodesToVisit[ toVisitOffset++ ] = gBVHNodes[ currentNodeIndex ].primitivesOrSecondChildOffset;
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

    // If tMin is no longer tMax, then we have intersected a primitive.
    if( tMin < oritMax )
    {
#ifdef PATH_TRACING_MODE
        if( isDebugThread )
        {
            gDebugInfo[ 0 ].hitSomething = true;
        }

        // Triangle's colour.
        float4 LColor = gVertexBuffer[ gPrimitives[ hitPrimIndx ].p0 ].color;

        // Triangle's transformed surface normal.
        float3 N =
            b1Hit * gVertexBuffer[ gPrimitives[ hitPrimIndx ].p0 ].normal +
            b2Hit * gVertexBuffer[ gPrimitives[ hitPrimIndx ].p1 ].normal +
            ( 1 - b1Hit - b2Hit ) * gVertexBuffer[ gPrimitives[ hitPrimIndx ].p2 ].normal;
        N = mul( N, ( float3x3 )gWorldInvTranspose );

        // Half-way vector.
        float3 hitPt = ray.o + tMin * ray.d;
        float3 V = normalize( ray.o - hitPt );
        float3 H = normalize( L + V );

        // TODO: Remove when done testing.
        //// Check if the triangle has a diffuse component.
        //if( dot( -N, -L ) > 0 /*|| dot( N, ray.d ) < 0*/ )
        //{
        //    color = float4( 1, 0, 0, 1 );
        //}

        if( isDebugThread )
        {
            // Populate gDebugInfo if the current thread ID corresponds to the cursor pos.
            gDebugInfo[ 0 ].ray = ray;
            gDebugInfo[ 0 ].hitPrim = gPrimitives[ hitPrimIndx ];

            // Set the pixel colour to red.
            color = float4( 1, 0, 0, 1 );
        }
        else
        {
            // Compute Phong-blinn shading for the intersected triangle.
            //color = calcBlinnPhongLighting( M, LColor, AColor, N, L, H );
            color = float4( 1, 1, 0, 1 );
        }
#endif // PATH_TRACING_MODE

        return true;
    }

#ifdef PATH_TRACING_MODE
    if( isDebugThread )
    {
        gDebugInfo[ 0 ].hitSomething = false;
    }
#endif // PATH_TRACING_MODE

    return false;
}

// Helper function to determine if the given node ID corresponds to an inner BVH node or not.
bool isInner( int nodeId )
{
    return ( nPrimitives( gBVHNodes[ nodeId ] ) == 0 );
}

// NOTE: Adapted from this paper: Stackless Multi-BVH Traversal for CPU, MIC and GPU Ray Tracing.
// Link: https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=3&ved=0ahUKEwjjz4m3wN_LAhUP2GMKHceLDxsQFgguMAI&url=http%3A%2F%2Fcg.iit.bme.hu%2F~afra%2Fpublications%2Fafra2013cgf_mbvhsl.pdf&usg=AFQjCNGG63xCj6JEvh2C5vLdYZ0GkwdVDA&sig2=s5UrOwMWigEFI9sw9VKt7w&cad=rja
bool primHitStackless( in Ray ray, in uint3 dispatchThreadId, inout float4 color )
{
    // MBVH2 traversal loop
    const float3 invDir = float3( 1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z );
    const float3 invDirPad = float3(
        addUlpMag( invDir.x, 2 ),
        addUlpMag( invDir.y, 2 ),
        addUlpMag( invDir.z, 2 ) );
    const int3 dirIsNeg = int3( invDir.x < 0, invDir.y < 0, invDir.z < 0 );
    uint nodeId = 0, junk, hitPrimIndx;
    uint bitstack = 0;
    uint siblingId = 0; // cached node links
    float oritMax = ray.tMax, tMin = ray.tMax, b1Hit, b2Hit;
    bool stopTraversal = false;

#ifdef PATH_TRACING_MODE
    // Initialize the debug info's nTotalPrimIntersections to 0.
    if( gGetDebugInfo && dispatchThreadId.x == 0 && dispatchThreadId.y == 0 )
    {
        InterlockedExchange( gDebugInfo[ 0 ].nTotalPrimIntersections, 0, junk );
    }

    // Initialize the debug info's nTraversedBounds to 0.
    bool isDebugThread = ( gGetDebugInfo && dispatchThreadId.x == gCursorPos.x && dispatchThreadId.y == gCursorPos.y );
    if( isDebugThread )
    {
        gDebugInfo[ 0 ].nTraversedBounds = 0;
    }
#else
    bool isDebugThread = true;
#endif // PATH_TRACING_MODE

#ifdef SINGLE_RAY_DEBUG_MODE
    // Accel debug counter.
    uint accelDebugIndx = 0;

    // Tree level counter;
    uint treeLevel = 0;
#endif  // SINGLE_RAY_DEBUG_MODE

    //for( ; ;)
    while( true )
    {
        // Inner node loop
        while( isInner( nodeId ) )
        {
            // Check if either of the children will be hit.
            const uint leftChildIndx = nodeId + 1;
            const uint rightChildIndx = gBVHNodes[ nodeId ].primitivesOrSecondChildOffset;
            float leftBoxt, rightBoxt;
            const bool leftHit =
                boxIntersect(
                    ray,
                    gBVHNodes[ leftChildIndx ].bounds,
                    invDir,
                    invDirPad,
                    dirIsNeg,
                    leftBoxt );
            const bool rightHit =
                boxIntersect(
                    ray,
                    gBVHNodes[ rightChildIndx ].bounds,
                    invDir,
                    invDirPad, 
                    dirIsNeg,
                    rightBoxt );

            // Terminate this loop if neither of the children were hit.
            if( !leftHit && !rightHit )
                break;

            // Push a 0 onto the bitstack to indicate that we are going to traverse the next level of the tree.
            bitstack <<= 1;

            // If both children are hit, set nodeId to that of the left child.
            if( leftHit && rightHit )
            {
                // Set nodeId to that of the nearest child node.
                if( leftBoxt < rightBoxt )
                {
                    nodeId = leftChildIndx;
#ifdef SINGLE_RAY_DEBUG_MODE
                    gAccelDebug[ accelDebugIndx ].accelTraversal = 0;    // HIT_LEFT
#endif  // SINGLE_RAY_DEBUG_MODE
                }
                else
                {
                    nodeId = rightChildIndx;
#ifdef SINGLE_RAY_DEBUG_MODE
                    gAccelDebug[ accelDebugIndx ].accelTraversal = 1;    // HIT_RIGHT
#endif  // SINGLE_RAY_DEBUG_MODE
                }
                siblingId = gBVHNodes[ nodeId ].siblingOffset;

                // Set the lowest bit of the bitstack to 1 to indicate that the near child has been traversed.
                bitstack |= 1;
            }
            else
            {
                //nodeId = hit0 ? nid.z : nid.w;

                // Set nodeId to that of the intersected child.
                if( leftHit )
                {
                    nodeId = leftChildIndx;
#ifdef SINGLE_RAY_DEBUG_MODE
                    gAccelDebug[ accelDebugIndx ].accelTraversal = 2;    // HIT_LEFT_ONLY
#endif  // SINGLE_RAY_DEBUG_MODE
                }
                else
                {
                    nodeId = rightChildIndx;
#ifdef SINGLE_RAY_DEBUG_MODE
                    gAccelDebug[ accelDebugIndx ].accelTraversal = 3;    // HIT_RIGHT_ONLY
#endif  // SINGLE_RAY_DEBUG_MODE
                }
                siblingId = gBVHNodes[ nodeId ].siblingOffset;
            }

#ifdef SINGLE_RAY_DEBUG_MODE
            gAccelDebug[ accelDebugIndx ].treeLevel = treeLevel;
            gAccelDebug[ accelDebugIndx ].nodeId = nodeId;
            gAccelDebug[ accelDebugIndx ].siblingId = siblingId;
            gAccelDebug[ accelDebugIndx ].bitstack = bitstack;
            gAccelDebug.IncrementCounter();
            ++accelDebugIndx;
            
            // Move to the next level of the tree.
            ++treeLevel;
#endif  // SINGLE_RAY_DEBUG_MODE            
        }
        // Leaf node
        if( !isInner( nodeId ) )
        {
            // Process BVH node _node_ for traversal
            float t, b1, b2;
            for( uint i = 0; i < nPrimitives( gBVHNodes[ nodeId ] ); ++i )
            {
                // Check if the i'th primitive is hit.
                Primitive prim = gPrimitives[ gBVHNodes[ nodeId ].primitivesOrSecondChildOffset + i ];
                Triangle tri = {
                    gVertexBuffer[ prim.p0 ].position,
                    gVertexBuffer[ prim.p1 ].position,
                    gVertexBuffer[ prim.p2 ].position };
                if( triangleIntersectWithBackFaceCulling( ray, tri, t, b1, b2 ) )
                {
#ifdef PATH_TRACING_MODE
                    // Increment the debug info's nTotalPrimIntersections.
                    if( gGetDebugInfo )
                    {
                        InterlockedAdd( gDebugInfo[ 0 ].nTotalPrimIntersections, 1 );
                    }
#endif // PATH_TRACING_MODE

                    // Set tMin to t if it is lesser. This is to ensure that the closest primitive is hit.
                    if( t < tMin )
                    {
                        tMin = t;
                        hitPrimIndx = gBVHNodes[ nodeId ].primitivesOrSecondChildOffset + i;
                        b1Hit = b1;
                        b2Hit = b2;

#ifdef SINGLE_RAY_DEBUG_MODE
                        gAccelDebug[ accelDebugIndx ].treeLevel = treeLevel;
                        gAccelDebug[ accelDebugIndx ].accelTraversal = 4;    // HIT_PRIMITIVE
                        gAccelDebug[ accelDebugIndx ].hitPrimId = hitPrimIndx;
                        gAccelDebug[ accelDebugIndx ].tMin = tMin;
                        gAccelDebug[ accelDebugIndx ].bitstack = bitstack;
                        gAccelDebug.IncrementCounter();
                        ++accelDebugIndx;
#endif  // SINGLE_RAY_DEBUG_MODE
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
            nodeId = gBVHNodes[ nodeId ].parentOffset;
            siblingId = gBVHNodes[ nodeId ].siblingOffset;
            bitstack >>= 1;

#ifdef SINGLE_RAY_DEBUG_MODE
            gAccelDebug[ accelDebugIndx ].treeLevel = treeLevel;
            gAccelDebug[ accelDebugIndx ].accelTraversal = 5;    // BACKTRACK
            gAccelDebug[ accelDebugIndx ].nodeId = nodeId;
            gAccelDebug[ accelDebugIndx ].siblingId = siblingId;
            gAccelDebug[ accelDebugIndx ].bitstack = bitstack;
            gAccelDebug.IncrementCounter();
            ++accelDebugIndx;

            // Move to the previous level of the tree.
            --treeLevel;
#endif  // SINGLE_RAY_DEBUG_MODE
        }

        // End the traversal if necessary.
        if( stopTraversal )
            break;

        if( siblingId != 0 )
            nodeId = siblingId;
        bitstack ^= 1;
    }

    if( isDebugThread )
    {
#ifdef PATH_TRACING_MODE
        // Populate gDebugInfo if the current thread ID corresponds to the cursor pos.
        gDebugInfo[ 0 ].ray = ray;
        gDebugInfo[ 0 ].hitPrim = gPrimitives[ hitPrimIndx ];
#endif  // PATH_TRACING_MODE

        // Set the pixel colour to red.
        color = float4( 1, 0, 0, 1 );
    }
    else
    {
        // If tMin is no longer tMax, then we have intersected a primitive.
        if( tMin < oritMax )
        {
#ifdef PATH_TRACING_MODE
            if( isDebugThread )
            {
                gDebugInfo[ 0 ].hitSomething = true;
            }

            // Triangle's colour.
            float4 LColor = gVertexBuffer[ gPrimitives[ hitPrimIndx ].p0 ].color;

            // Triangle's transformed surface normal.
            float3 N =
                b1Hit * gVertexBuffer[ gPrimitives[ hitPrimIndx ].p0 ].normal +
                b2Hit * gVertexBuffer[ gPrimitives[ hitPrimIndx ].p1 ].normal +
                ( 1 - b1Hit - b2Hit ) * gVertexBuffer[ gPrimitives[ hitPrimIndx ].p2 ].normal;
            N = mul( N, ( float3x3 )gWorldInvTranspose );

            // Half-way vector.
            float3 hitPt = ray.o + tMin * ray.d;
            float3 V = normalize( ray.o - hitPt );
            float3 H = normalize( L + V );

            // TODO: Remove when done testing.
            //// Check if the triangle has a diffuse component.
            //if( dot( -N, -L ) > 0 /*|| dot( N, ray.d ) < 0*/ )
            //{
            //    color = float4( 1, 0, 0, 1 );
            //}
            
            // Compute Phong-blinn shading for the intersected triangle.
            color = calcBlinnPhongLighting( M, LColor, AColor, N, L, H );
            //color = float4( 0, 0, 1, 1 ); // NOTE: Uncomment for flat shading to debug lighting issues.
#endif // PATH_TRACING_MODE

            return true;
        }
    }

#ifdef PATH_TRACING_MODE
    if( isDebugThread )
    {
        gDebugInfo[ 0 ].hitSomething = false;
    }
#endif // PATH_TRACING_MODE

    return false;
}