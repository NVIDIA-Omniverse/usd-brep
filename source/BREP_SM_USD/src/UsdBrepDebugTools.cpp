// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepDebugTools.cpp
 * PURPOSE: Debugging tools for
            UsdBrepArrayData and UsdBrepArraySpans classes
 * ******************************************************************************************************************/

#include "UsdBrepDebugTools.h"

#include "UsdBrepArrayData.h"
#include "UsdBrepTokens.h"

#include <filesystem>
#include <fstream>
#include <mutex>

// USD includes
#include "UsdBrepHeaders.h"

/*********************************************************************************************************************
PERSISTENT LOGGING SUPPORT
*********************************************************************************************************************/
namespace
{

std::ofstream& usdBrep_GetLogStream()
{
    static std::ofstream sLogStream;
    static std::once_flag sInitFlag;

    std::call_once(
        sInitFlag,
        []()
        {
            std::error_code ec;
            std::filesystem::create_directories("OutputFiles", ec);
            const std::filesystem::path logPath = std::filesystem::path("OutputFiles") / "usdBrep_debug.log";
            sLogStream.open(logPath, std::ios::out | std::ios::app);
        }
    );

    return sLogStream;
}

} // anonymous namespace

/*********************************************************************************************************************
PURPOSE: Print string to stdout and visual studio

NOTES:  Will eventually add an enum to direct output
        Standard out, visual studio, etc
*********************************************************************************************************************/
void usdBrep_WriteString(const std::string& str)
{
    if (str.empty())
    {
        return;
    }

#ifdef _WIN32
    printf("%s", str.c_str());
    OutputDebugStringA(str.c_str());
#else
    fprintf(stderr, "%s", str.c_str());
#endif

    std::ofstream& logStream = usdBrep_GetLogStream();
    if (logStream.is_open())
    {
        logStream << str;
        if (!str.empty() && str.back() != '\n')
        {
            logStream << '\n';
        }
        logStream.flush();
    }

} // end usdBrep_WriteString

/*********************************************************************************************************************
PURPOSE: standardized debug outputFiles directory

NOTES:
Error handling has been added to handle two failure scenarios:

Directory already exists - in that case it can be ignored and return true.
Directory cannot be created - return false and print error message.
*********************************************************************************************************************/
bool usdBrep_CreateOutputFiles()
{
    std::error_code ec;
    std::filesystem::create_directory("OutputFiles", ec);
    if (ec && ec != std::errc::file_exists)
    {
        std::string sstring = "Failed to create OutputFiles directory: " + ec.message() + "\n";
        usdBrep_WriteString(sstring);
        return false;
    }
    if (ec == std::errc::file_exists && !std::filesystem::is_directory("OutputFiles"))
    {
        std::string sstring = "Path ‘OutputFiles’ exists but is not a directory\n";
        usdBrep_WriteString(sstring);
        return false;
    }

    // Returns true if directory exists or was created successfully
    return true;

} // end usdBrep_CreateOutputFiles

/*********************************************************************************************************************
 PURPOSE: UsdBrepArrayData method implementations

 NOTES:
 ********************************************************************************************************************/
namespace UsdBrepData
{

/*********************************************************************************************************************
PURPOSE: Dump basic info about the Prim.

*********************************************************************************************************************/
void Dump_PrimProperties(const pxr::UsdPrim& crPrim)
{
    std::string sstring;

    // Check if prim is valid
    if (!crPrim)
    {
        sstring = "Error: Invalid UsdPrim provided to Dump_PrimProperties\n";
        usdBrep_WriteString(sstring);
        return;
    }

    // Print Applied Attributes on the Prim
    try
    {
        pxr::TfTokenVector sAppliedSchemas = crPrim.GetAppliedSchemas();
        for (size_t ii = 0; ii < sAppliedSchemas.size(); ++ii)
        {
            sstring = TfStringPrintf("\nAppliedSchema = %s", sAppliedSchemas[ii].GetString().c_str());
            usdBrep_WriteString(sstring);
        }
    }
    catch (const std::exception& e)
    {
        sstring = TfStringPrintf(
            "Error: Exception caught while getting applied schemas for prim [%s]: %s\n",
            crPrim.GetPath().GetString().c_str(),
            e.what()
        );
        usdBrep_WriteString(sstring);
    }

    // Print Properties on the prim
    try
    {
        pxr::TfTokenVector sProperties = crPrim.GetPropertyNames();
        for (size_t ii = 0; ii < sProperties.size(); ++ii)
        {
            sstring = TfStringPrintf("\nProperty = %s", sProperties[ii].GetString().c_str());
            usdBrep_WriteString(sstring);
        }
    }
    catch (const std::exception& e)
    {
        sstring = TfStringPrintf(
            "Error: Exception caught while getting property names for prim [%s]: %s\n",
            crPrim.GetPath().GetString().c_str(),
            e.what()
        );
        usdBrep_WriteString(sstring);
    }

} // end Dump_PrimProperties

/*********************************************************************************************************************
PURPOSE: Dump basic info about the PrimSpec.

NOTES:

*********************************************************************************************************************/
void Dump_PrimSpecProperties(const pxr::SdfPrimSpecHandle& primSpecHandle)
{
    std::string sstring;

    if (!primSpecHandle)
    {
        sstring = TfStringPrintf("Invalid SdfPrimSpecHandle.\n");
        usdBrep_WriteString(sstring);
        return;
    }

    // Check if the primSpec is dormant
    if (primSpecHandle->IsDormant())
    {
        sstring = TfStringPrintf("Warning: SdfPrimSpecHandle is dormant for prim [%s]\n", primSpecHandle->GetPath().GetString().c_str());
        usdBrep_WriteString(sstring);
    }

    // Get the apiSchemas metadata as a SdfTokenListOp
    pxr::VtValue apiSchemasVal = primSpecHandle->GetInfo(pxr::UsdTokens->apiSchemas);
    if (!apiSchemasVal.IsHolding<pxr::SdfTokenListOp>())
    {
        sstring = TfStringPrintf("No applied API schemas found for prim [%s]\n", primSpecHandle->GetPath().GetString().c_str());
        usdBrep_WriteString(sstring);
        return;
    }

    try
    {
        const pxr::SdfTokenListOp& listOp = apiSchemasVal.UncheckedGet<pxr::SdfTokenListOp>();
        pxr::TfTokenVector appliedSchemas = listOp.GetExplicitItems();

        sstring = TfStringPrintf("Applied API Schemas for prim [%s]\n", primSpecHandle->GetPath().GetString().c_str());
        usdBrep_WriteString(sstring);

        for (const auto& schema : appliedSchemas)
        {
            sstring = TfStringPrintf("  %s\n", schema.GetString().c_str());
            usdBrep_WriteString(sstring);
        }
    }
    catch (const std::exception& e)
    {
        sstring = TfStringPrintf(
            "Error: Exception caught while processing API schemas for prim [%s]: %s\n",
            primSpecHandle->GetPath().GetString().c_str(),
            e.what()
        );
        usdBrep_WriteString(sstring);
    }
} // end Dump_PrimSpecProperties

/*********************************************************************************************************************
PURPOSE: Dump basic info about the PrimSpec.

NOTES:
*********************************************************************************************************************/
void Dump_PrimProperties_fromPrimSpecHandle(const pxr::SdfPrimSpecHandle& crPrimSpecHandle, pxr::UsdStageRefPtr& rStage)
{
    // Check if primSpec handle is valid
    if (!crPrimSpecHandle)
    {
        std::string sstring = "Error: Invalid SdfPrimSpecHandle in Dump_PrimProperties_fromPrimSpecHandle\n";
        usdBrep_WriteString(sstring);
        return;
    }

    // Check if stage is valid
    if (!rStage)
    {
        std::string sstring = "Error: Invalid UsdStage provided to Dump_PrimProperties_fromPrimSpecHandle\n";
        usdBrep_WriteString(sstring);
        return;
    }

    // Get the SdfPath from the SdfPrimSpecHandle.
    SdfPath sPrimPathFromSpec = crPrimSpecHandle->GetPath();

    // Check if the path is valid
    if (sPrimPathFromSpec.IsEmpty())
    {
        std::string sstring = "Error: Empty SdfPath in Dump_PrimProperties_fromPrimSpecHandle\n";
        usdBrep_WriteString(sstring);
        return;
    }

    // Use the SdfPath to get the corresponding UsdPrim from the UsdStage.
    UsdPrim sCorrespondingUsdPrim = rStage->GetPrimAtPath(sPrimPathFromSpec);

    if (!sCorrespondingUsdPrim)
    {
        std::string sstring = TfStringPrintf("Error: Could not get UsdPrim from path [%s]\n", sPrimPathFromSpec.GetString().c_str());
        usdBrep_WriteString(sstring);
        return;
    }

    // Print Applied Attributes on the Prim
    Dump_PrimProperties(sCorrespondingUsdPrim);

} // end Dump_PrimProperties_fromPrimSpecHandle

/*********************************************************************************************************************
PURPOSE: pretty print UsdBrepArrayData through usdBrep_WriteString() calls

NOTES: To write to files and output window use:
     usdBrepSet_OutputLong()     - sets b_OutputLong state for usdBrep_WriteString() calls
     usdBrepSet_OutputThin()     - sets b_OutputThin state for usdBrep_WriteString() calls
     Dump_BrepArrayData()    - on windows   in debug - tgt OutputDebugString()
                               on Linux/MAC in debug - tgt stderr
                               if(b_OutputLong)      - tgt LongFile (if pBuff use pBuff else pBuffForFile)
                               if(b_OutputThin)      - tgt ThinFile (if pBuffForFile use pBuffForFile else pBuff)

 PARAMETERS:
    rArrays   : in : Tgt UsdBrepArrayData to pretty print
    bDumpData : in : false = dump only Topology obj counts true  = also dump array data values default:[false]
*********************************************************************************************************************/
void Dump_BrepArrayData(const UsdBrepArrayData& rArrays, bool bDumpData)
{

    // Used by macros but declared here to avoid redefinition
    uint32_t ii, jj;
    constexpr uint32_t i0 = 0, i1 = 1, i2 = 2;

    std::string sstring;

#define MY_ARRAY_int32(ArrayName, ArrayCount, Array, ColSize)                                                                                        \
    if (rArrays.ArrayCount > 0)                                                                                                                      \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rArrays.ArrayCount; ii++)                                                                                                  \
        {                                                                                                                                            \
            sstring = TfStringPrintf("%" #ColSize "d", Array[ii]);                                                                                   \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rArrays.ArrayCount - 1)                                                                                                         \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    } // end MY_ARRAY_int32

#define MY_ARRAY_uint32(ArrayName, ArrayCount, Array, ColSize)                                                                                       \
    if (rArrays.ArrayCount > 0)                                                                                                                      \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rArrays.ArrayCount; ii++)                                                                                                  \
        {                                                                                                                                            \
            sstring = TfStringPrintf("%" #ColSize "u", Array[ii]);                                                                                   \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rArrays.ArrayCount - 1)                                                                                                         \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    } // end MY_ARRAY_uint32

#define MY_INDEX_ARRAY(ArrayName, ArrayCount, ColSize)                                                                                               \
    if (rArrays.ArrayCount > 0)                                                                                                                      \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rArrays.ArrayCount; ii++)                                                                                                  \
        {                                                                                                                                            \
            sstring = TfStringPrintf("%" #ColSize "u", ii);                                                                                          \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rArrays.ArrayCount - 1)                                                                                                         \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    } // end MY_INDEX_ARRAY

#define MY_ARRAY_double(ArrayName, ArrayCount, Array, ColSize, Precision)                                                                            \
    if (rArrays.ArrayCount > 0)                                                                                                                      \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rArrays.ArrayCount; ii++)                                                                                                  \
        {                                                                                                                                            \
            sstring = TfStringPrintf("%" #ColSize "." #Precision "lf", Array[ii]);                                                                   \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rArrays.ArrayCount - 1)                                                                                                         \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    } // MY_ARRAY_double

#define MY_ARRAY_GfVec2d(ArrayName, ArrayCount, Array, ColSize, Precision)                                                                           \
    if (rArrays.ArrayCount > 0)                                                                                                                      \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rArrays.ArrayCount; ii++)                                                                                                  \
        {                                                                                                                                            \
            sstring = TfStringPrintf("{%" #ColSize "." #Precision "lf, %" #ColSize "." #Precision "lf}", Array[ii][0], Array[ii][1]);                \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rArrays.ArrayCount - 1)                                                                                                         \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    } // end MY_ARRAY_GfVec2d

#define MY_ARRAY_GfVec3d(ArrayName, ArrayCount, Array, index, ColSize, Precision)                                                                    \
    if (rArrays.ArrayCount > 0)                                                                                                                      \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rArrays.ArrayCount; ii++)                                                                                                  \
        {                                                                                                                                            \
            if constexpr (index == 0)                                                                                                                \
            {                                                                                                                                        \
                sstring = TfStringPrintf("{%" #ColSize "." #Precision "lf}", Array[ii][0]);                                                          \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else if constexpr (index == 1)                                                                                                           \
            {                                                                                                                                        \
                sstring = TfStringPrintf("{%" #ColSize "." #Precision "lf}", Array[ii][1]);                                                          \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("{%" #ColSize "." #Precision "lf}", Array[ii][2]);                                                          \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
                                                                                                                                                     \
            if (ii < rArrays.ArrayCount - 1)                                                                                                         \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    } // end MY_ARRAY_GfVec3d

#define MY_ARRAY_GfVec2i(ArrayName, ArrayCount, Array, ColSize)                                                                                      \
    if (rArrays.ArrayCount > 0)                                                                                                                      \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rArrays.ArrayCount; ii++)                                                                                                  \
        {                                                                                                                                            \
            sstring = TfStringPrintf("{%" #ColSize "d, %" #ColSize "d}", Array[ii][0], Array[ii][1]);                                                \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rArrays.ArrayCount - 1)                                                                                                         \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    } // end MY_ARRAY_GfVec2i

#define MY_ARRAY_token(ArrayName, ArrayCount, token_map)                                                                                             \
    if (rArrays.ArrayCount > 0)                                                                                                                      \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rArrays.ArrayCount; ii++)                                                                                                  \
        {                                                                                                                                            \
            sstring = TfStringPrintf("%s", token_map);                                                                                               \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rArrays.ArrayCount - 1)                                                                                                         \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    } // MY_ARRAY_token


    sstring = TfStringPrintf(
        "\n\nBegin UsdBrepArrayData Dump, BrepCount:[%4u], SdfPath:[%.900s]",
        rArrays.TotalBrepCount(),
        rArrays.m_sPrimPath.GetAsString().c_str()
    );
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n  Total Topology counts for all %4u Breps in BrepArray", rArrays.TotalBrepCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalBrepCount          :[%4u]", rArrays.TotalBrepCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalRegionCount        :[%4u]", rArrays.TotalRegionCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalShellCount         :[%4u]", rArrays.TotalShellCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n     - TotalShellVertexCount: - [%4u] (number of Shells which are VertexShells)", rArrays.TotalShellVertexCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalFaceuseCount       :[%4u]", rArrays.TotalFaceuseCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalFaceCount          :[%4u]", rArrays.TotalFaceCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalLoopCount          :[%4u]", rArrays.TotalLoopCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n     - TotalLoopVertexCount : - [%4u] (number of Loops which are VertexLoops)", rArrays.TotalLoopVertexCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalEdgeuseCount       :[%4u]", rArrays.TotalEdgeuseCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalEdgeCount          :[%4u]", rArrays.TotalEdgeCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalWireEdgeCount      :[%4u]", rArrays.TotalWireEdgeCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalVertexCount        :[%4u]", rArrays.TotalVertexCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n notes:1. TotalCounts = Sum of BrepCounts for all Breps in a BrepArray.");
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n       2. TotalEdgeCount    includes edges connecting to faces. Excludes WireEdges");
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n          TotalEdgeuseCount includes one edgeuse for ever Edge_to_Face connection.");
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n                              - One Edge can connect to any number of faces.");
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n                              - Manifold: TotalEdgeuseCount == 2 * TotalEdgeCount");
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n       3. TotalWireEdgeCount includes edges not connecting to faces. Excl FaceEdges.");
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n       4. TotalVertexCount excludes shellVertices.");
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n       5. Every Face has 2 Faceuses and TotalFaceuseCount == 2*TotalFaceCoun");
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n       6. EdgeuseCount = Faces w/2 Faceuses & TotalFaceuseCount == 2*TotalFaceCount");
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalShellVertex_PositionCount      :[%4u]", rArrays.TotalShellVertexPositionCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalVertex_PositionCount           :[%4u]", rArrays.TotalVertexPositionCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalEdge_CurveCount                :[%4u]", rArrays.TotalEdgeCurveCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      TotalEdge_ControlVerticesCount    :[%4u]", rArrays.TotalEdgeControlVerticesCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      TotalEdge_KnotCount               :[%4u]", rArrays.TotalEdgeKnotCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalWireEdge_CurveCount            :[%4u]", rArrays.TotalWireEdgeCurveCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      TotalWireEdge_ControlVerticesCount:[%4u]", rArrays.TotalWireEdgeControlVerticesCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      TotalWireEdge_KnotCount           :[%4u]", rArrays.TotalWireEdgeKnotCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalEdgeuse_CurveCount             :[%4u]", rArrays.TotalEdgeuseCurveCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      TotalEdgeuse_NonNullCurveCount    :[%4u]", rArrays.TotalEdgeuseNonNullCurveCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      TotalEdgeuse_ControlVerticesCount :[%4u]", rArrays.TotalEdgeuseControlVerticesCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      TotalEdgeuse_KnotCountCount       :[%4u]", rArrays.TotalEdgeuseKnotCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    TotalFace_SurfaceCount              :[%4u]", rArrays.TotalFaceSurfaceCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      TotalFace_ControlVerticesCount    :[%4u]", rArrays.TotalFaceControlVerticesCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      TotalFace_Knot_UCount             :[%4u]", rArrays.TotalFaceKnot_UCount());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      TotalFace_Knot_VCount             :[%4u]", rArrays.TotalFaceKnot_VCount());
    usdBrep_WriteString(sstring);


    // when asked - dump arrays values
    if (bDumpData)
    {
        // simple header - for tighter logs
        sstring = TfStringPrintf(
            "\n   --- Begin UsdBrepArrayData USD Stage Data, SdfPath:[%.900s], TotalBrepCnt:[%4u] ------",
            rArrays.m_sPrimPath.GetAsString().c_str(),
            rArrays.TotalBrepCount()
        );
        usdBrep_WriteString(sstring);

        // Brep Data
        sstring = TfStringPrintf("\n  Brep USD Stage Metadata");
        usdBrep_WriteString(sstring);

        sstring = TfStringPrintf("\n    BrepArray PrimPath                    :[%.900s]", rArrays.m_sPrimPath.GetAsString().c_str());
        usdBrep_WriteString(sstring);

        sstring = (std::ostringstream() << "\n    BrepArray BBox                        :" << rArrays.m_sBrepArray_BBox).str();
        usdBrep_WriteString(sstring);

        sstring = TfStringPrintf("\n    BrepArray MaterialPath                :[%.900s]", rArrays.m_sBrepArray_MaterialPath.GetAsString().c_str());
        usdBrep_WriteString(sstring);

        if (rArrays.m_sBrepMaterial_BrepPathArray.empty())
        {
            sstring = TfStringPrintf("\n    BrepArray Brep MaterialPathVector size:[0]");
            usdBrep_WriteString(sstring);
        }
        else
        {
            sstring = TfStringPrintf("\n    BrepArray Brep MaterialPathVector size:[%4lu]", rArrays.m_sBrepMaterial_BrepPathArray.size());
            usdBrep_WriteString(sstring);

            for (ii = 0; ii < rArrays.m_sBrepMaterial_BrepPathArray.size(); ii++)
            {
                sstring = TfStringPrintf(
                    "\n      Brep MaterialPath[%d]    :[%.900s]",
                    ii,
                    rArrays.m_sBrepMaterial_BrepPathArray[ii].GetAsString().c_str()
                );
                usdBrep_WriteString(sstring);

                // Check array bounds for BrepIndexArray (only needed if arrays might have different sizes)
                if (ii >= rArrays.m_sBrepMaterial_BrepIndexArray.size())
                {
                    sstring = TfStringPrintf("Error: Array index out of bounds for BrepMaterial_BrepIndexArray at index %d\n", ii);
                    usdBrep_WriteString(sstring);
                    continue;
                }

                sstring = TfStringPrintf("\n        used by %lu Brep Indices:[", rArrays.m_sBrepMaterial_BrepIndexArray[ii].size());
                usdBrep_WriteString(sstring);

                if (!rArrays.m_sBrepMaterial_BrepIndexArray[ii].empty())
                {
                    for (jj = 0; jj < rArrays.m_sBrepMaterial_BrepIndexArray[ii].size(); jj++)
                    {
                        sstring = TfStringPrintf("%3u, ", rArrays.m_sBrepMaterial_BrepIndexArray[ii][jj]);
                        usdBrep_WriteString(sstring);
                    }
                    sstring = TfStringPrintf("]");
                    usdBrep_WriteString(sstring);
                }
            }
        }

        // Brep Topology Data
        sstring = TfStringPrintf("\n   --- Begin Brep Topology Data -----------------------------------");
        usdBrep_WriteString(sstring);

        if (rArrays.TotalBrepCount() > 0)
        {
            MY_INDEX_ARRAY("BrepIndex                ", TotalBrepCount(), 20);
            MY_ARRAY_double("BrepXSectTol3dArray      ", TotalBrepCount(), rArrays.m_sBrepXSectTol3dArray, 20, 6);

            MY_ARRAY_GfVec3d("BrepExtentArray Min/Max X", TotalBrepCount() * 2, (rArrays.m_sBrepExtentArray), i0, 6, 6);
            MY_ARRAY_GfVec3d("                Min/Max Y", TotalBrepCount() * 2, (rArrays.m_sBrepExtentArray), i1, 6, 6);
            MY_ARRAY_GfVec3d("                Min/Max Z", TotalBrepCount() * 2, (rArrays.m_sBrepExtentArray), i2, 6, 6);

            MY_ARRAY_uint32("BrepRegionCountArray     ", TotalBrepCount(), rArrays.m_sBrepRegionCountArray, 20);
        }

        // Regions
        sstring = TfStringPrintf("\n  Region   Data, TotalRegionCount  :[%4u]", rArrays.TotalRegionCount());
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("RegionIndex          ", TotalRegionCount(), 11);
        MY_ARRAY_uint32("RegionShellCountArray", TotalRegionCount(), rArrays.m_sRegionShellCountArray, 11);
        MY_ARRAY_token(
            "RegionTypeArray      ",
            TotalRegionCount(),
            (rArrays.m_sRegionTypeArray.size() == 0                        ? "    NoEntry" :
             rArrays.m_sRegionTypeArray[ii] == UsdBrepSolidTokens->solidRegion ? "solidRegion" :
             rArrays.m_sRegionTypeArray[ii] == UsdBrepSolidTokens->voidRegion  ? " voidRegion" :
                                                                             "      other")
        );

        // Shells
        sstring = TfStringPrintf("\n  Shell    Data, TotalShellCount   :[%4u]", rArrays.TotalShellCount());
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("ShellIndex             ", TotalShellCount(), 12);
        MY_ARRAY_uint32("ShellFaceuseCountArray ", TotalShellCount(), rArrays.m_sShellFaceuseCountArray, 12);
        MY_ARRAY_uint32("ShellWireEdgeCountArray", TotalShellCount(), rArrays.m_sShellWireEdgeCountArray, 12);
        MY_ARRAY_token(
            "ShellPointTypeArray    ",
            TotalShellCount(),
            (rArrays.m_sShellPointTypeArray.size() == 0                         ? "     NoEntry" :
             rArrays.m_sShellPointTypeArray[ii] == UsdBrepSolidTokens->brepPointAPI ? "BrepPointAPI" :
             rArrays.m_sShellPointTypeArray[ii] == UsdBrepSolidTokens->none         ? "        none" :
                                                                                  "       other")
        );

        // Faceuse
        sstring = TfStringPrintf("\n  Faceuse  Data, TotalFaceuseCount :[%4u]", rArrays.TotalFaceuseCount());
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("FaceuseIndex               ", TotalFaceuseCount(), 8);
        MY_ARRAY_uint32("FaceuseFaceIndexArray      ", TotalFaceuseCount(), rArrays.m_sFaceuseFaceIndexArray, 8);
        MY_ARRAY_token(
            "FaceuseOrientationTypeArray",
            TotalFaceuseCount(),
            (rArrays.m_sFaceuseOrientationTypeArray.size() == 0                     ? " NoEntry" :
             rArrays.m_sFaceuseOrientationTypeArray[ii] == UsdBrepSolidTokens->same     ? "    same" :
             rArrays.m_sFaceuseOrientationTypeArray[ii] == UsdBrepSolidTokens->opposite ? "opposite" :
                                                                                      "   other")
        );
        // Faces
        sstring = TfStringPrintf("\n  Face     Data, TotalFaceCount    :[%4u]", rArrays.TotalFaceCount());
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("FaceIndex           ", TotalFaceCount(), 20);
        MY_ARRAY_uint32("FaceLoopCountArray  ", TotalFaceCount(), rArrays.m_sFaceLoopCountArray, 20);
        MY_ARRAY_token(
            "FaceSurfaceTypeArray",
            TotalFaceCount(),
            (rArrays.m_sFaceSurfaceTypeArray.size() == 0                                 ? "             NoEntry" :
             rArrays.m_sFaceSurfaceTypeArray[ii] == UsdBrepSurfaceTokens->brepSurfaceNurbAPI ? "  BrepSurfaceNurbAPI" :
                                                                                           "               other")
        );
        MY_ARRAY_token(
            "FaceTrimTypeArray   ",
            TotalFaceCount(),
            (rArrays.m_sFaceTrimTypeArray.size() == 0                        ? "             NoEntry" :
             rArrays.m_sFaceTrimTypeArray[ii] == UsdBrepSolidTokens->rectangular ? "         rectangular" :
             rArrays.m_sFaceTrimTypeArray[ii] == UsdBrepSolidTokens->general     ? "             general" :
                                                                               "               other")
        );
        MY_ARRAY_GfVec2d("FaceRangeArray     ", TotalFaceCount() * 2, rArrays.m_sFaceRangeArray, 8, 6);

        // Loops
        sstring = TfStringPrintf("\n  Loop     Data, TotalLoopCount    :[%4u]", rArrays.TotalLoopCount());
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("LoopIndex            ", TotalLoopCount(), 7);
        MY_ARRAY_uint32("LoopEdgeuseCountArray", TotalLoopCount(), rArrays.m_sLoopEdgeuseCountArray, 7);
        MY_ARRAY_uint32("LoopVertexIndexArray ", TotalLoopCount(), rArrays.m_sLoopVertexIndexArray, 7);

        // Edgeuse
        sstring = TfStringPrintf("\n  Edgeuse  Data, TotalEdgeuseCount :[%4u]", rArrays.TotalEdgeuseCount());
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("EdgeuseIndex                 ", TotalEdgeuseCount(), 11);
        MY_ARRAY_uint32("EdgeuseEdgeIndexArray        ", TotalEdgeuseCount(), rArrays.m_sEdgeuseEdgeIndexArray, 11);
        MY_ARRAY_token(
            "EdgeuseOrientationTypeArray  ",
            TotalEdgeuseCount(),
            (rArrays.m_sEdgeuseOrientationTypeArray.size() == 0                     ? "    NoEntry" :
             rArrays.m_sEdgeuseOrientationTypeArray[ii] == UsdBrepSolidTokens->same     ? "       same" :
             rArrays.m_sEdgeuseOrientationTypeArray[ii] == UsdBrepSolidTokens->opposite ? "   opposite" :
                                                                                      "      other")
        );
        MY_ARRAY_uint32("EdgeuseNextRadialEUIndexArray", TotalEdgeuseCount(), rArrays.m_sEdgeuseNextRadialEUIndexArray, 11);

        MY_ARRAY_token(
            "EdgeuseOrientationTypeArray  ",
            TotalEdgeuseCount(),
            (rArrays.m_sEdgeuseThisRadialEntryTypeArray.size() == 0                        ? "    NoEntry" :
             rArrays.m_sEdgeuseThisRadialEntryTypeArray[ii] == UsdBrepSolidTokens->topEntry    ? "   topEntry" :
             rArrays.m_sEdgeuseThisRadialEntryTypeArray[ii] == UsdBrepSolidTokens->bottomEntry ? "bottomEntry" :
                                                                                             "      other")
        );
        // Edges
        sstring = TfStringPrintf("\n  Edge     Data, TotalEdgCount     :[%4u]", rArrays.TotalEdgeCount());
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("EdgeIndex             ", TotalEdgeCount(), 18);
        MY_ARRAY_token(
            "EdgeCurveTypeArray    ",
            TotalEdgeCount(),
            (rArrays.m_sEdgeCurveTypeArray.size() == 0                               ? "           NoEntry" :
             rArrays.m_sEdgeCurveTypeArray[ii] == UsdBrepCurveTokens->brepCurve3dNurbAPI ? "BrepCurve3dNurbAPI" :
                                                                                       "             other")
        );
        MY_ARRAY_double("EdgeRangeArray        ", TotalEdgeCount() * 2, rArrays.m_sEdgeRangeArray, 7, 4);
        MY_ARRAY_GfVec2i("EdgeVertexIndicesArray", TotalEdgeCount(), rArrays.m_sEdgeVertexIndicesArray, 7);

        // WireEdges
        sstring = TfStringPrintf("\n  WireEdge Data, TotalWireEdgeCount:[%4u]", rArrays.TotalWireEdgeCount());
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("WireEdgeIndex             ", TotalWireEdgeCount(), 18);
        MY_ARRAY_token(
            "WireEdgeCurveTypeArray    ",
            TotalWireEdgeCount(),
            (rArrays.m_sWireEdgeCurveTypeArray.size() == 0                               ? "           NoEntry" :
             rArrays.m_sWireEdgeCurveTypeArray[ii] == UsdBrepCurveTokens->brepCurve3dNurbAPI ? "BrepCurve3dNurbAPI" :
                                                                                           "             other")
        );
        MY_ARRAY_double("WireEdgeRangeArray        ", TotalWireEdgeCount() * 2, rArrays.m_sWireEdgeRangeArray, 7, 4);
        MY_ARRAY_GfVec2i("WireEdgeVertexIndicesArray", TotalWireEdgeCount(), rArrays.m_sWireEdgeVertexIndicesArray, 7);

        // Vertices
        sstring = TfStringPrintf("\n  Vertex   Data, TotalVertexCount  :[%4u]", rArrays.TotalVertexCount());
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("VertexIndex         ", TotalVertexCount(), 18);
        MY_ARRAY_token(
            "VertexPointTypeArray",
            TotalVertexCount(),
            (rArrays.m_sVertexPointTypeArray.size() == 0                         ? "     NoEntry" :
             rArrays.m_sVertexPointTypeArray[ii] == UsdBrepSolidTokens->brepPointAPI ? "BrepPointAPI" :
             rArrays.m_sVertexPointTypeArray[ii] == UsdBrepSolidTokens->none         ? "        none" :
                                                                                   "       other")
        );

        // Geometry
        sstring = TfStringPrintf("\n   --- Begin Brep Shape Data --------------------------------------");
        usdBrep_WriteString(sstring);

        // ShellVertex Point array
        sstring = TfStringPrintf(
            "\n  ShellVertex Point3dPositions,  ShellVertexCount:[%4u], ShellVertexPositionCount:[%4u]",
            rArrays.TotalShellVertexCount(),
            rArrays.TotalShellVertexPositionCount()
        );
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("ShellVertex index", TotalShellVertexPositionCount(), 12);
        MY_ARRAY_GfVec3d("ShellVertex Pos  ", TotalShellVertexPositionCount(), rArrays.m_sShell_PointPositionArray, i0, 9, 6);
        MY_ARRAY_GfVec3d("                 ", TotalShellVertexPositionCount(), rArrays.m_sShell_PointPositionArray, i1, 9, 6);
        MY_ARRAY_GfVec3d("                 ", TotalShellVertexPositionCount(), rArrays.m_sShell_PointPositionArray, i2, 9, 6);

        // ShellVertex Point array
        sstring = TfStringPrintf(
            "\n  Vertex      Point3dPositions,  VertexCount     :[%4u], VertexPositionCount     :[%4u]",
            rArrays.TotalVertexCount(),
            rArrays.TotalVertexPositionCount()
        );
        usdBrep_WriteString(sstring);


        MY_INDEX_ARRAY("Vertex index", TotalVertexPositionCount(), 12);
        MY_ARRAY_GfVec3d("Vertex Pos  ", TotalVertexPositionCount(), rArrays.m_sVertex_PointPositionArray, i0, 9, 6);
        MY_ARRAY_GfVec3d("            ", TotalVertexPositionCount(), rArrays.m_sVertex_PointPositionArray, i1, 9, 6);
        MY_ARRAY_GfVec3d("            ", TotalVertexPositionCount(), rArrays.m_sVertex_PointPositionArray, i2, 9, 6);

        // Edge curve3d
        sstring = TfStringPrintf(
            "\n  Edge        Curve3dData,       EdgeCount       :[%4u], EdgeCount               :[%4u]",
            rArrays.TotalEdgeCount(),
            rArrays.TotalEdgeCurveCount()
        );
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("Edge index           ", TotalEdgeCurveCount(), 12);

        MY_ARRAY_uint32("EdgeCurve VertexCount", TotalEdgeCurveCount(), rArrays.m_sEdge_CurveNurb_VertexCountArray, 12);

        MY_ARRAY_uint32("EdgeCurve Order      ", TotalEdgeCurveCount(), rArrays.m_sEdge_CurveNurb_OrderArray, 12);

        MY_ARRAY_double("EdgeCurve Knots      ", TotalEdgeKnotCount(), rArrays.m_sEdge_CurveNurb_KnotsArray, 9, 6);

        MY_ARRAY_GfVec3d("EdgeCurve Vertices X ", TotalEdgeControlVerticesCount(), rArrays.m_sEdge_CurveNurb_ControlVerticesArray, i0, 9, 6);

        MY_ARRAY_GfVec3d("                   Y ", TotalEdgeControlVerticesCount(), rArrays.m_sEdge_CurveNurb_ControlVerticesArray, i1, 9, 6);

        MY_ARRAY_GfVec3d("                   Z ", TotalEdgeControlVerticesCount(), rArrays.m_sEdge_CurveNurb_ControlVerticesArray, i2, 9, 6);

        MY_ARRAY_double("                   W ", TotalEdgeControlVerticesCount(), rArrays.m_sEdge_CurveNurb_WeightsArray, 12, 6);

        // WireEdge curve3d
        sstring = TfStringPrintf(
            "\n  WireEdge    Curve3dData,       WireEdgeCount   :[%4u], WireEdgeCurve3dCount    :[%4u]",
            rArrays.TotalWireEdgeCount(),
            rArrays.TotalWireEdgeCurveCount()
        );
        usdBrep_WriteString(sstring);


        MY_INDEX_ARRAY("WireEdge index           ", TotalWireEdgeCount(), 12);
        MY_ARRAY_uint32("WireEdgeCurve VertexCount", TotalWireEdgeCurveCount(), rArrays.m_sWireEdge_CurveNurb_VertexCountArray, 12);
        MY_ARRAY_uint32("WireEdgeCurve Order      ", TotalWireEdgeCurveCount(), rArrays.m_sWireEdge_CurveNurb_OrderArray, 12);
        MY_ARRAY_double("WireEdgeCurve Knots      ", TotalWireEdgeKnotCount(), rArrays.m_sWireEdge_CurveNurb_KnotsArray, 9, 6);
        MY_ARRAY_GfVec3d(
            "WireEdgeCurve Vertices X ",
            TotalWireEdgeControlVerticesCount(),
            rArrays.m_sWireEdge_CurveNurb_ControlVerticesArray,
            i0,
            9,
            6
        );
        MY_ARRAY_GfVec3d(
            "                       Y ",
            TotalWireEdgeControlVerticesCount(),
            rArrays.m_sWireEdge_CurveNurb_ControlVerticesArray,
            i1,
            9,
            6
        );
        MY_ARRAY_GfVec3d(
            "                       Z ",
            TotalWireEdgeControlVerticesCount(),
            rArrays.m_sWireEdge_CurveNurb_ControlVerticesArray,
            i2,
            9,
            6
        );
        MY_ARRAY_double("                       W ", TotalWireEdgeControlVerticesCount(), rArrays.m_sWireEdge_CurveNurb_WeightsArray, 12, 6);

        // Edgeuse curve2d
        sstring = TfStringPrintf(
            "\n  Edgeuse     UVTrimCurve2dData, EdgeuseCount    :[%4u], EdgeuseCurve2dCount     :[%4u], EdgeuseNonNULLCurve2dCount     :[%4u]",
            rArrays.TotalEdgeuseCount(),
            rArrays.TotalEdgeuseCurveCount(),
            rArrays.TotalEdgeuseNonNullCurveCount()
        );
        usdBrep_WriteString(sstring);


        MY_INDEX_ARRAY("Edgeuse index           ", TotalEdgeuseCurveCount(), 12);
        MY_ARRAY_uint32("EdgeuseCurve VertexCount", TotalEdgeuseCurveCount(), rArrays.m_sEdgeuse_CurveNurb_VertexCountArray, 12);
        MY_ARRAY_uint32("EdgeuseCurve Order      ", TotalEdgeuseCurveCount(), rArrays.m_sEdgeuse_CurveNurb_OrderArray, 12);
        MY_ARRAY_double("EdgeuseCurve Knots      ", TotalEdgeuseKnotCount(), rArrays.m_sEdgeuse_CurveNurb_KnotsArray, 9, 6);
        MY_ARRAY_GfVec3d("EdgeuseCurve Vertices X ", TotalEdgeuseControlVerticesCount(), rArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray, i0, 9, 6);
        MY_ARRAY_GfVec3d("                      Y ", TotalEdgeuseControlVerticesCount(), rArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray, i1, 9, 6);
        MY_ARRAY_GfVec3d("                      Z ", TotalEdgeuseControlVerticesCount(), rArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray, i2, 9, 6);
        MY_ARRAY_double("                      W ", TotalEdgeuseControlVerticesCount(), rArrays.m_sEdgeuse_CurveNurb_WeightsArray, 12, 6);

        // Face surface3d
        sstring = TfStringPrintf(
            "\n  Face        Surface3dData,     FaceCount       :[%4u], FaceSurface3dCount      :[%4u]",
            rArrays.TotalFaceCount(),
            rArrays.TotalFaceSurfaceCount()
        );
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("Face index               ", TotalFaceSurfaceCount(), 12);
        MY_ARRAY_uint32("FaceSurface_U VertexCount", TotalFaceSurfaceCount(), rArrays.m_sFace_SurfaceNurb_UVertexCountArray, 12);
        MY_ARRAY_uint32("FaceSurface_V VertexCount", TotalFaceSurfaceCount(), rArrays.m_sFace_SurfaceNurb_VVertexCountArray, 12);
        MY_ARRAY_uint32("FaceSurface_U Order      ", TotalFaceSurfaceCount(), rArrays.m_sFace_SurfaceNurb_UOrderArray, 12);
        MY_ARRAY_uint32("FaceSurface_V Order      ", TotalFaceSurfaceCount(), rArrays.m_sFace_SurfaceNurb_VOrderArray, 12);
        MY_ARRAY_double("FaceSurface_U Knots      ", TotalFaceKnot_UCount(), rArrays.m_sFace_SurfaceNurb_UKnotsArray, 9, 6);
        MY_ARRAY_double("FaceSurface_V Knots      ", TotalFaceKnot_VCount(), rArrays.m_sFace_SurfaceNurb_VKnotsArray, 9, 6);
        MY_ARRAY_GfVec3d("FaceSurface Vertices   X ", TotalFaceControlVerticesCount(), rArrays.m_sFace_SurfaceNurb_ControlVerticesArray, i0, 9, 6);
        MY_ARRAY_GfVec3d("                       Y ", TotalFaceControlVerticesCount(), rArrays.m_sFace_SurfaceNurb_ControlVerticesArray, i1, 9, 6);
        MY_ARRAY_GfVec3d("                       Z ", TotalFaceControlVerticesCount(), rArrays.m_sFace_SurfaceNurb_ControlVerticesArray, i2, 9, 6);
        MY_ARRAY_double("                       W ", TotalFaceControlVerticesCount(), rArrays.m_sFace_SurfaceNurb_WeightsArray, 12, 6);

    } // end asked to DumpData check

#undef MY_ARRAY_int32
#undef MY_ARRAY_uint32
#undef MY_INDEX_ARRAY
#undef MY_ARRAY_double
#undef MY_ARRAY_GfVec2d
#undef MY_ARRAY_GfVec2i
#undef MY_ARRAY_token
#undef MY_ARRAY_GfVec3d

    // locals - rArrays start Indices and counts perBrep
    UsdBrepArraySpans sSpans;
    sSpans.ClearStartsAndCounts();

    // for multiple Breps - output one brep at a time
    if (rArrays.TotalBrepCount() > 1)
    {
        // for every Brep - output perBrep topology obj counts
        for (ii = 0; ii < rArrays.TotalBrepCount(); ii++)
        {
            // set counts for Brep_ii using current start indices
            sSpans.SetStartsAndCountsForBrepIndex(
                rArrays, // in : Tgt Arrays to parse
                ii, //      in : Tgt Brep_ii index
                false, //   in : true = set Brep_ii start Indices by walking rArrays
                       //               data for every Brep prior to Brep_ii
                       //        false= use StartIndices as is assuming they are set
                       //               for Brep_ii.
                true //     in : true = set Brep_ii counts,
            ); //          false= set Brep_ii icounts=0, ready for next move
               //          Brep call.

            // pretty print this Brep
            Dump_BrepArraySpans(rArrays, sSpans, bDumpData);

            // add counts to StartIndices to set up for next iter SetStartsAndCountsForBrepIndex() call
            sSpans.IncrementStartIndices(false); // in : bSaveCounts: false=clear counts, true=don't, default:[false]

        } // end iter every Brep
    } // end multiple Breps check

    // footer
    sstring = TfStringPrintf(
        "\nEnd UsdBrepArrayData Dump, BrepCount:[%4u], SdfPath:[%s]",
        rArrays.TotalBrepCount(),
        rArrays.m_sPrimPath.GetAsString().c_str()
    );
    usdBrep_WriteString(sstring);

} // end Dump_BrepArrayData

/*********************************************************************************************************************
PURPOSE: pretty print UsdBrepArrayData through usdBrep_WriteString() calls

NOTES: To write to files and output window use:
     usdBrepSet_OutputLong()            - sets b_OutputLong state for usdBrep_WriteString() calls
     usdBrepSet_OutputThin()            - sets b_OutputThin state for usdBrep_WriteString() calls
     Dump_BrepArrayData()           - on windows   in debug - tgt OutputDebugString()
                                      on Linux/MAC in debug - tgt stderr

 PARAMETERS:
    rArrays   : in : associated UsdBrepArrayData for this BrepArraySpans
    rSpans    : in : Tgt BrepArraySpans to pretty print
    bDumpData : in : false = dump only Topology obj counts, true = also dump array data values, default:[false]
*********************************************************************************************************************/
void Dump_BrepArraySpans(const UsdBrepArrayData& rArrays, const UsdBrepArraySpans& rSpans, bool bDumpData)
{
    uint32_t ii;
    std::string sstring;

#define MY_ARRAY_int32(ArrayName, ArrayStart, ArrayCount, Array, ColSize)                                                                            \
    if ((rSpans.ArrayCount > 0) && (rArrays.Array.size() >= rSpans.ArrayStart + rSpans.ArrayCount))                                                  \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rSpans.ArrayCount; ii++)                                                                                                   \
        {                                                                                                                                            \
            sstring = TfStringPrintf("%" #ColSize "d", rArrays.Array[rSpans.ArrayStart + ii]);                                                       \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rSpans.ArrayCount - 1)                                                                                                          \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    }

#define MY_ARRAY_uint32(ArrayName, ArrayStart, ArrayCount, Array, ColSize)                                                                           \
    if ((rSpans.ArrayCount > 0) && (rArrays.Array.size() >= rSpans.ArrayStart + rSpans.ArrayCount))                                                  \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rSpans.ArrayCount; ii++)                                                                                                   \
        {                                                                                                                                            \
            sstring = TfStringPrintf("%" #ColSize "u", rArrays.Array[rSpans.ArrayStart + ii]);                                                       \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rSpans.ArrayCount - 1)                                                                                                          \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    }

#define MY_INDEX_ARRAY(ArrayName, ArrayCount, ColSize)                                                                                               \
    if (rSpans.ArrayCount > 0)                                                                                                                       \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rSpans.ArrayCount; ii++)                                                                                                   \
        {                                                                                                                                            \
            sstring = TfStringPrintf("%" #ColSize "u", ii);                                                                                          \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rSpans.ArrayCount - 1)                                                                                                          \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    }

#define MY_ARRAY_double(ArrayName, ArrayStart, ArrayCount, Array, ColSize, Precision)                                                                \
    if ((rSpans.ArrayCount > 0) && (rArrays.Array.size() >= rSpans.ArrayStart + rSpans.ArrayCount))                                                  \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rSpans.ArrayCount; ii++)                                                                                                   \
        {                                                                                                                                            \
            sstring = TfStringPrintf("%" #ColSize "." #Precision "lf", rArrays.Array[rSpans.ArrayStart + ii]);                                       \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rSpans.ArrayCount - 1)                                                                                                          \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    }

#define MY_ARRAY_GfVec2d(ArrayName, ArrayStart, ArrayCount, Array, ColSize, Precision)                                                               \
    if ((rSpans.ArrayCount > 0) && (rArrays.Array.size() >= rSpans.ArrayStart + rSpans.ArrayCount))                                                  \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rSpans.ArrayCount; ii++)                                                                                                   \
        {                                                                                                                                            \
            sstring = TfStringPrintf(                                                                                                                \
                "{%" #ColSize "." #Precision "lf, %" #ColSize "." #Precision "lf}",                                                                  \
                rArrays.Array[rSpans.ArrayStart + ii][0],                                                                                            \
                rArrays.Array[rSpans.ArrayStart + ii][1]                                                                                             \
            );                                                                                                                                       \
            usdBrep_WriteString(sstring);                                                                                                                \
                                                                                                                                                     \
            if (ii < rSpans.ArrayCount - 1)                                                                                                          \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    }

#define MY_ARRAY_GfVec2i(ArrayName, ArrayStart, ArrayCount, Array, ColSize)                                                                          \
    if ((rSpans.ArrayCount > 0) && (rArrays.Array.size() >= rSpans.ArrayStart + rSpans.ArrayCount))                                                  \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rSpans.ArrayCount; ii++)                                                                                                   \
        {                                                                                                                                            \
            sstring = TfStringPrintf(                                                                                                                \
                "{%" #ColSize "d, %" #ColSize "d}",                                                                                                  \
                rArrays.Array[rSpans.ArrayStart + ii][0],                                                                                            \
                rArrays.Array[rSpans.ArrayStart + ii][1]                                                                                             \
            );                                                                                                                                       \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rSpans.ArrayCount - 1)                                                                                                          \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    }

#define MY_ARRAY_token(ArrayName, ArrayStart, ArrayCount, Array, token_map)                                                                          \
    if ((rSpans.ArrayCount > 0) && (rArrays.Array.size() >= rSpans.ArrayStart + rSpans.ArrayCount))                                                  \
    {                                                                                                                                                \
        sstring = TfStringPrintf("\n    %s:[", (ArrayName));                                                                                         \
        usdBrep_WriteString(sstring);                                                                                                                    \
        for (ii = 0; ii < rSpans.ArrayCount; ii++)                                                                                                   \
        {                                                                                                                                            \
            sstring = TfStringPrintf("%s", token_map);                                                                                               \
            usdBrep_WriteString(sstring);                                                                                                                \
            if (ii < rSpans.ArrayCount - 1)                                                                                                          \
            {                                                                                                                                        \
                sstring = TfStringPrintf(", ");                                                                                                      \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
            else                                                                                                                                     \
            {                                                                                                                                        \
                sstring = TfStringPrintf("]");                                                                                                       \
                usdBrep_WriteString(sstring);                                                                                                            \
            }                                                                                                                                        \
        }                                                                                                                                            \
    }

    // simple header - for tighter logs
    sstring = TfStringPrintf("\n\n  BrepIndex:[%4u], BrepSdfPath:[%.900s]", rSpans.m_lBrepIndex, rArrays.m_sPrimPath.GetAsString().c_str());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    Region      Indx:[%4u],  Cnt:[%4u]", rSpans.m_lRegionStartIndex, rSpans.m_lRegionCount);
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    Shell       Indx:[%4u],  Cnt:[%4u]", rSpans.m_lShellStartIndex, rSpans.m_lShellCount);
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      ShellVertices in Shells:[%4u]", rSpans.m_lShellVertexCount);
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    Faceuse     Indx:[%4u],  Cnt:[%4u]", rSpans.m_lFaceuseStartIndex, rSpans.m_lFaceuseCount);
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    Face        Indx:[%4u],  Cnt:[%4u]", rSpans.m_lFaceStartIndex, rSpans.m_lFaceCount);
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    Loop        Indx:[%4u],  Cnt:[%4u]", rSpans.m_lLoopStartIndex, rSpans.m_lLoopCount);
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n      LoopVertices in Loops:[%4u]", rSpans.m_lLoopVertexCount);
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    Edgeuse     Indx:[%4u],  Cnt:[%4u]", rSpans.m_lEdgeuseStartIndex, rSpans.m_lEdgeuseCount);
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    Edge        Indx:[%4u],  Cnt:[%4u]", rSpans.m_lEdgeStartIndex, rSpans.m_lEdgeCount);
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    WireEdge    Indx:[%4u],  Cnt:[%4u]", rSpans.m_lWireEdgeStartIndex, rSpans.m_lWireEdgeCount);
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("\n    Vertex      Indx:[%4u],  Cnt:[%4u]", rSpans.m_lVertexStartIndex, rSpans.m_lVertexCount);
    usdBrep_WriteString(sstring);

    // when asked - dump arrays values
    if (bDumpData)
    {
        sstring = TfStringPrintf(
            "\n\n --- Begin Data Dump for BrepIndex:[%4u], SdfPath:[%.900s] ------------",
            rSpans.m_lBrepIndex,
            rArrays.m_sPrimPath.GetAsString().c_str()
        );
        usdBrep_WriteString(sstring);

        // Brep Data
        // Check array bounds before access
        if (rSpans.m_lBrepIndex >= rArrays.m_sBrepXSectTol3dArray.size())
        {
            sstring = "Error: BrepIndex out of bounds for BrepXSectTol3dArray: " + std::to_string(rSpans.m_lBrepIndex) +
                      " >= " + std::to_string(rArrays.m_sBrepXSectTol3dArray.size()) + "\n";
            usdBrep_WriteString(sstring);
        }
        else
        {
            sstring = TfStringPrintf("\n  XSectTol3d:[%9.6lf]", rArrays.m_sBrepXSectTol3dArray[rSpans.m_lBrepIndex]);
            usdBrep_WriteString(sstring);
        }

        // Check array bounds for extent array access
        size_t extentIndex0 = rSpans.m_lBrepIndex * 2 + 0;
        size_t extentIndex1 = rSpans.m_lBrepIndex * 2 + 1;

        if (extentIndex0 >= rArrays.m_sBrepExtentArray.size() || extentIndex1 >= rArrays.m_sBrepExtentArray.size())
        {
            sstring = "Error: BrepIndex out of bounds for BrepExtentArray: " + std::to_string(rSpans.m_lBrepIndex) + " (indices " +
                      std::to_string(extentIndex0) + ", " + std::to_string(extentIndex1) +
                      ") >= " + std::to_string(rArrays.m_sBrepExtentArray.size()) + "\n";
            usdBrep_WriteString(sstring);
        }
        else
        {
            sstring = TfStringPrintf(
                "\n  Extent X  :[min=%9.6lf, max=%9.6lf]",
                rArrays.m_sBrepExtentArray[extentIndex0][0],
                rArrays.m_sBrepExtentArray[extentIndex1][0]
            );
            usdBrep_WriteString(sstring);
            sstring = TfStringPrintf(
                "\n  Extent Y  :[min=%9.6lf, max=%9.6lf]",
                rArrays.m_sBrepExtentArray[extentIndex0][1],
                rArrays.m_sBrepExtentArray[extentIndex1][1]
            );
            usdBrep_WriteString(sstring);
            sstring = TfStringPrintf(
                "\n  Extent Z  :[min=%9.6lf, max=%9.6lf]",
                rArrays.m_sBrepExtentArray[extentIndex0][2],
                rArrays.m_sBrepExtentArray[extentIndex1][2]
            );
            usdBrep_WriteString(sstring);
        }

        if (rSpans.m_lBrepIndex >= rArrays.m_sBrepRegionCountArray.size())
        {
            sstring = "Error: BrepIndex out of bounds for BrepRegionCountArray: " + std::to_string(rSpans.m_lBrepIndex) +
                      " >= " + std::to_string(rArrays.m_sBrepRegionCountArray.size()) + "\n";
            usdBrep_WriteString(sstring);
        }
        else
        {
            sstring = TfStringPrintf("\n  RegionCnt :[%4u]", rArrays.m_sBrepRegionCountArray[rSpans.m_lBrepIndex]);
            usdBrep_WriteString(sstring);
        }

        // Regions
        sstring = TfStringPrintf("\n  Region   Data, BrepRegionCount    :[%4u]", rSpans.m_lRegionCount);
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("RegionIndex          ", m_lRegionCount, 11);
        MY_ARRAY_uint32("RegionShellCountArray", m_lRegionStartIndex, m_lRegionCount, m_sRegionShellCountArray, 11);
        MY_ARRAY_token(
            "RegionTypeArray      ",
            m_lRegionStartIndex,
            m_lRegionCount,
            m_sRegionTypeArray,
            (rArrays.m_sRegionTypeArray.size() == 0                                                     ? "    NoEntry" :
             rArrays.m_sRegionTypeArray[rSpans.m_lRegionStartIndex + ii] == UsdBrepSolidTokens->solidRegion ? "solidRegion" :
             rArrays.m_sRegionTypeArray[rSpans.m_lRegionStartIndex + ii] == UsdBrepSolidTokens->voidRegion  ? " voidRegion" :
                                                                                                          "      other")
        );
        // Shells
        sstring = TfStringPrintf(
            "\n  Shell    Data, TotalBrepShellCount:[%4u], \n                      BrepVertexShellCount : [%4u]",
            rSpans.m_lShellCount,
            rSpans.m_lShellVertexCount
        );
        usdBrep_WriteString(sstring);


        MY_INDEX_ARRAY("ShellIndex             ", m_lShellCount, 12);
        MY_ARRAY_uint32("ShellFaceuseCountArray ", m_lShellStartIndex, m_lShellCount, m_sShellFaceuseCountArray, 12);
        MY_ARRAY_uint32("ShellWireEdgeCountArray", m_lShellStartIndex, m_lShellCount, m_sShellWireEdgeCountArray, 12);
        MY_ARRAY_token(
            "ShellPointTypeArray    ",
            m_lShellStartIndex,
            m_lShellCount,
            m_sShellPointTypeArray,
            (rArrays.m_sShellPointTypeArray.size() == 0                                                     ? "     NoEntry" :
             rArrays.m_sShellPointTypeArray[rSpans.m_lShellStartIndex + ii] == UsdBrepSolidTokens->brepPointAPI ? "BrepPointAPI" :
             rArrays.m_sShellPointTypeArray[rSpans.m_lShellStartIndex + ii] == UsdBrepSolidTokens->none         ? "        none" :
                                                                                                              "       other")
        );

        // Faceuse
        sstring = TfStringPrintf("\n  Faceuse  Data, BrepFaceuseCount   :[%4u]", rSpans.m_lFaceuseCount);
        usdBrep_WriteString(sstring);


        MY_INDEX_ARRAY("FaceuseIndex               ", m_lFaceuseCount, 8);
        MY_ARRAY_uint32("FaceuseFaceIndexArray      ", m_lFaceuseStartIndex, m_lFaceuseCount, m_sFaceuseFaceIndexArray, 8);
        MY_ARRAY_token(
            "FaceuseOrientationTypeArray",
            m_lFaceuseStartIndex,
            m_lFaceuseCount,
            m_sFaceuseOrientationTypeArray,
            (rArrays.m_sFaceuseOrientationTypeArray.size() == 0                                                   ? " NoEntry" :
             rArrays.m_sFaceuseOrientationTypeArray[rSpans.m_lFaceuseStartIndex + ii] == UsdBrepSolidTokens->same     ? "    same" :
             rArrays.m_sFaceuseOrientationTypeArray[rSpans.m_lFaceuseStartIndex + ii] == UsdBrepSolidTokens->opposite ? "opposite" :
                                                                                                                    "   other")
        );
        // Faces
        sstring = TfStringPrintf("\n  Face     Data, BrepFaceCount      :[%4u]", rSpans.m_lFaceCount);
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("FaceIndex           ", m_lFaceCount, 20);
        MY_ARRAY_uint32("FaceLoopCountArray  ", m_lFaceStartIndex, m_lFaceCount, m_sFaceLoopCountArray, 20);
        MY_ARRAY_token(
            "FaceSurfaceTypeArray",
            m_lFaceStartIndex,
            m_lFaceCount,
            m_sFaceSurfaceTypeArray,
            (rArrays.m_sFaceSurfaceTypeArray.size() == 0                                                            ? "             NoEntry" :
             rArrays.m_sFaceSurfaceTypeArray[rSpans.m_lFaceStartIndex + ii] == UsdBrepSurfaceTokens->brepSurfaceNurbAPI ? "  BrepSurfaceNurbAPI" :
                                                                                                                      "               other")
        );
        MY_ARRAY_token(
            "FaceTrimTypeArray   ",
            m_lFaceStartIndex,
            m_lFaceCount,
            m_sFaceTrimTypeArray,
            (rArrays.m_sFaceTrimTypeArray.size() == 0                                                   ? "             NoEntry" :
             rArrays.m_sFaceTrimTypeArray[rSpans.m_lFaceStartIndex + ii] == UsdBrepSolidTokens->rectangular ? "         rectangular" :
             rArrays.m_sFaceTrimTypeArray[rSpans.m_lFaceStartIndex + ii] == UsdBrepSolidTokens->general     ? "             general" :
                                                                                                          "               other")
        );
        MY_ARRAY_GfVec2d("FaceRangeArray    ", m_lFaceStartIndex, m_lFaceCount * 2, m_sFaceRangeArray, 8, 6);

        // Loops
        sstring = TfStringPrintf(
            "\n  Loop     Data, TotalBrepLoopCount :[%4u], \n                      BrepVertexLoopCount  :[%4u]",
            rSpans.m_lLoopCount,
            rSpans.m_lLoopVertexCount
        );
        usdBrep_WriteString(sstring);


        MY_INDEX_ARRAY("LoopIndex            ", m_lLoopCount, 7);
        MY_ARRAY_uint32("LoopEdgeuseCountArray", m_lLoopStartIndex, m_lLoopCount, m_sLoopEdgeuseCountArray, 7);
        MY_ARRAY_uint32("LoopVertexIndexArray ", m_lLoopStartIndex, m_lLoopCount, m_sLoopVertexIndexArray, 7);

        // Edgeuse
        sstring = TfStringPrintf("\n  Edgeuse  Data, BrepEdgeuseCount   :[%4u]", rSpans.m_lEdgeuseCount);
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("EdgeuseIndex                 ", m_lEdgeuseCount, 11);
        MY_ARRAY_uint32("EdgeuseEdgeIndexArray        ", m_lEdgeuseStartIndex, m_lEdgeuseCount, m_sEdgeuseEdgeIndexArray, 11);
        MY_ARRAY_token(
            "EdgeuseOrientationTypeArray  ",
            m_lEdgeuseStartIndex,
            m_lEdgeuseCount,
            m_sEdgeuseOrientationTypeArray,
            (rArrays.m_sEdgeuseOrientationTypeArray.size() == 0                                                   ? "    NoEntry" :
             rArrays.m_sEdgeuseOrientationTypeArray[rSpans.m_lEdgeuseStartIndex + ii] == UsdBrepSolidTokens->same     ? "       same" :
             rArrays.m_sEdgeuseOrientationTypeArray[rSpans.m_lEdgeuseStartIndex + ii] == UsdBrepSolidTokens->opposite ? "   opposite" :
                                                                                                                    "      other")
        );
        MY_ARRAY_uint32("EdgeuseNextRadialEUIndexArray", m_lEdgeuseStartIndex, m_lEdgeuseCount, m_sEdgeuseNextRadialEUIndexArray, 11);
        MY_ARRAY_token(
            "sEdgeuseThisRadialEntryTypeArray",
            m_lEdgeuseStartIndex,
            m_lEdgeuseCount,
            m_sEdgeuseThisRadialEntryTypeArray,
            (rArrays.m_sEdgeuseThisRadialEntryTypeArray.size() == 0                                                      ? "    NoEntry" :
             rArrays.m_sEdgeuseThisRadialEntryTypeArray[rSpans.m_lEdgeuseStartIndex + ii] == UsdBrepSolidTokens->topEntry    ? "   topEntry" :
             rArrays.m_sEdgeuseThisRadialEntryTypeArray[rSpans.m_lEdgeuseStartIndex + ii] == UsdBrepSolidTokens->bottomEntry ? "bottomEntry" :
                                                                                                                           "      other")
        );

        // Edges
        sstring = TfStringPrintf("\n  Edge     Data, BrepEdgeCount      :[%4u]", rSpans.m_lEdgeCount);
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("EdgeIndex             ", m_lEdgeCount, 18);
        MY_ARRAY_token(
            "EdgeCurveTypeArray    ",
            m_lEdgeStartIndex,
            m_lEdgeCount,
            m_sEdgeCurveTypeArray,
            (rArrays.m_sEdgeCurveTypeArray.size() == 0                                                          ? "           NoEntry" :
             rArrays.m_sEdgeCurveTypeArray[rSpans.m_lEdgeStartIndex + ii] == UsdBrepCurveTokens->brepCurve3dNurbAPI ? "BrepCurve3dNurbAPI" :
                                                                                                                  "             other")
        );
        MY_ARRAY_double("EdgeRangeArray        ", m_lEdgeStartIndex, m_lEdgeCount * 2, m_sEdgeRangeArray, 7, 4);
        MY_ARRAY_GfVec2i("EdgeVertexIndicesArray", m_lEdgeStartIndex, m_lEdgeCount, m_sEdgeVertexIndicesArray, 7);

        // WireEdges
        sstring = TfStringPrintf("\n  WireEdge Data, BrepWireEdgeCount  :[%4u]", rSpans.m_lWireEdgeCount);
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("WireEdgeIndex             ", m_lWireEdgeCount, 18);
        MY_ARRAY_token(
            "WireEdgeCurveTypeArray    ",
            m_lWireEdgeStartIndex,
            m_lWireEdgeCount,
            m_sWireEdgeCurveTypeArray,
            (rArrays.m_sWireEdgeCurveTypeArray.size() == 0                                                              ? "           NoEntry" :
             rArrays.m_sWireEdgeCurveTypeArray[rSpans.m_lWireEdgeStartIndex + ii] == UsdBrepCurveTokens->brepCurve3dNurbAPI ? "BrepCurve3dNurbAPI" :
                                                                                                                          "             other")
        );
        MY_ARRAY_double("WireEdgeRangeArray        ", m_lWireEdgeStartIndex, m_lWireEdgeCount * 2, m_sWireEdgeRangeArray, 7, 4);
        MY_ARRAY_GfVec2i("WireEdgeVertexIndicesArray", m_lWireEdgeStartIndex, m_lWireEdgeCount, m_sWireEdgeVertexIndicesArray, 7);

        // Vertices
        sstring = TfStringPrintf("\n  Vertex   Data, BrepVertexCount    :[%4u]", rSpans.m_lVertexCount);
        usdBrep_WriteString(sstring);

        MY_INDEX_ARRAY("VertexIndex         ", m_lVertexCount, 18);
        MY_ARRAY_token(
            "VertexPointTypeArray",
            m_lVertexStartIndex,
            m_lVertexCount,
            m_sVertexPointTypeArray,
            (rArrays.m_sVertexPointTypeArray.size() == 0                                                      ? "     NoEntry" :
             rArrays.m_sVertexPointTypeArray[rSpans.m_lVertexStartIndex + ii] == UsdBrepSolidTokens->brepPointAPI ? "BrepPointAPI" :
             rArrays.m_sVertexPointTypeArray[rSpans.m_lVertexStartIndex + ii] == UsdBrepSolidTokens->none         ? "        none" :
                                                                                                                "       other")
        );

        sstring = TfStringPrintf(
            "\n  End Data Dump for BrepIndex:[%4u], SdfPath:[%.900s]",
            rSpans.m_lBrepIndex,
            rArrays.m_sPrimPath.GetAsString().c_str()
        );
        usdBrep_WriteString(sstring);

    } // end asked to DumpData check

#undef MY_ARRAY_int32
#undef MY_ARRAY_uint32
#undef MY_INDEX_ARRAY
#undef MY_ARRAY_GfVec2d
#undef MY_ARRAY_GfVec2i
#undef MY_ARRAY_token

    // no footer when bDumpData == false

} // end Dump_BrepArraySpans

} // end namespace UsdBrepData
