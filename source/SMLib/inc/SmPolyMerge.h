// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPolyMerge.h
* PURPOSE: Header file for SmPolyMerge object.
**********************************************************************/

#ifndef __SMPOLYMERGE_H__
#define __SMPOLYMERGE_H__

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMPOLYINTERSECTOR_H__
#include <SmPolyIntersector.h>
#endif

/*******************************************************************//**
PURPOSE: The Boolean Operation Type defines what sort of operation
    is being performed by SmMerge::ManifoldBoolean

NOTES: 
***********************************************************************/
enum SmPolyBooleanOperationType
{
    SM_PBO_UNION,         // Union of solids A and B
    SM_PBO_INTERSECTION,  // Intersection of solids A and B
    SM_PBO_DIFFERENCE,    // Difference - A minus B
    SM_PBO_MERGE,         // Merge Operation
    SM_PBO_PARTIAL_MERGE, // Merge subset of parts of one brep into the other
    SM_PBO_IMPRINT,       // Find the intersections of B with A, and imprint them as Edges
                          // and Vertices in A
    SM_PBO_EXTRACT_SEPARATE  // Extract into separate Poly Breps
};

/*******************************************************************//**
PURPOSE: The Merge object provides the ability to intersect and
    combine the topologies of two Breps.  

NOTES: 
***********************************************************************/
class
SM_EXPORT
SmPolyMerge : public SmObject
{
protected:
    SmBoolean         m_bRemoveCoPlanarEdges;
    SmBoolean         m_bRemoveOnlyInvisibleEdges;
    SmBoolean         m_bManifoldBoolean;
    SmBoolean         m_bIntersectionLoopClosed;
    SmBoolean         m_bImprinting;                 // Just imprint the second brep into the first using intersection.
                                                     // Same as using SM_PBO_IMPRINT operation
    SmBoolean         m_bImprintAndClassify;         // Imprint intersection edges, classify according to operation
                                                     // But do not delete faces from each brep. Instead we simply mark each
                                                     // face that would be deleted with attribute SM_AI_BOOLEAN_DELETE
    SmBoolean         m_bCookieCutter;               // Use the second brep as a cookie cutter.
    SmBoolean         m_bCheckLicense;
    SmPolyIntersector m_vPI;

    // when eOperation == SM_BO_EXTRACT_SEPARATE extract Brep and Other  
    // contiguous face sets to separate Breps and put into following arrays.
    SmTArray<SmPolyBrep*> m_vOneBrepPer_OrigBrepConnectedFaceSet;
    SmTArray<SmPolyBrep*> m_vOneBrepPer_OtherBrepConnectedFaceSet;

public:
    virtual ~SmPolyMerge() { };

    SmPolyMerge
    (
      const SmContext & crContext,
      SmPolyBrep      * pPolyBrep,
      SmPolyBrep      * pPolyOtherBrep,
      double            dApproxTol3d, 
      double            dAngleTolRadians
    );

    SmPolyMerge(const SmContext & crContext);

    SmStatus CleanupCoPlanarFaces();

    SmStatus DoVertexSnappingBeforeBoolean(double dSnappingTol);

    SmStatus MoveFace
    (
      SmPolyFace * & pOFace,                                        ///< [in] : face to move to this m_pBrep    <br>
      SmBoolean bReverseOrientation                                 ///< [in] : TRUE =                          <br>
                                                                    ///<      : FALSE=                          <br>
    );                                                             
                                                                   
    SmStatus CopyFace                                              
    (                                                              
      SmPolyFace * & pOFace,                                        ///< [in] :      <br>
      SmBoolean bReverseOrientation                                 ///< [in] :      <br>
    );

    SmStatus ManifoldBoolean
    (
      SmPolyBooleanOperationType eOperation,                        ///< [in] : one of SM_PBO_UNION,        SM_PBO_IMPRINT                      <br>
                                                                    ///<      :        SM_PBO_MERGE,        SM_PBO_PARTIAL_MERGE                <br>
                                                                    ///<      :        SM_PBO_INTERSECTION, SM_PBO_EXTRACT_SEPARATE             <br>
                                                                    ///<      :        SM_PBO_DIFFERENCE,                                       <br>
      SmPolyBrep              *& rpResult,                          ///< [out]: Resulting PolyBrep                                              <br>
      SmMarkType               * pOptBrepMarkType = NULL,           ///< [in] : Optional Mark used for Brep,  NULL to ignore, default:[NULL]    <br>
      SmMarkType               * pOptOtherMarkType = NULL           ///< [in] : Optional Mark used for Other, NULL to ignore, default:[NULL]    <br>
    );

    SmStatus RemoveCoPlanarEdges();

    void SetRemoveCoPlanarEdges(SmBoolean bRemoveCoPlanarEdges)           { m_bRemoveCoPlanarEdges = bRemoveCoPlanarEdges; }

    void SetRemoveOnlyInvisibleEdges(SmBoolean bRemoveOnlyInvisibleEdges) { m_bRemoveOnlyInvisibleEdges = bRemoveOnlyInvisibleEdges; }

    void SetCookieCutter(SmBoolean bCookieCutter)                         { m_bCookieCutter = bCookieCutter; }
    
    void SetImprinting(SmBoolean bImprinting)                             { m_bImprinting = bImprinting; }

    void SetImprintAndClassify(SmBoolean bImprintAndClassify)             { m_bImprintAndClassify = bImprintAndClassify; }
   
    void SetCheckLicense(SmBoolean bCheckLicense)                         { m_bCheckLicense = bCheckLicense; }
    
    void GetExtractedBrepFaces(SmTArray<SmPolyBrep*> & rPolyBreps)        { rPolyBreps.ReSet(); rPolyBreps.Append(m_vOneBrepPer_OrigBrepConnectedFaceSet); }

    void GetExtractedOtherFaces(SmTArray<SmPolyBrep*> & rPolyBreps)       { rPolyBreps.ReSet(); rPolyBreps.Append(m_vOneBrepPer_OtherBrepConnectedFaceSet); }

    void SetTrimmingPlane(const SmPoint3d & crPlanePoint, const SmVector3d & crPlaneNormal);
};


#endif // !__SMPOLYMERGE_H__
