// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPointClass.h
* PURPOSE: Header file for point classification object.
* Contains
*  class SmPointClassification
**********************************************************************/

#ifndef __SMPOINTCLASSIFICATION_H__
#define __SMPOINTCLASSIFICATION_H__

#ifndef __SMVECTOR2D_H__
#include <SmVector2d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMSOLUTIONARRAY_H__
#include <SmSolutionArray.h>
#endif

#ifndef __SMTOLERANCE_H__
#include <SmTol.h>
#endif

/*******************************************************************//**
PURPOSE: This class represents a classification relationship between
 a point and an object.  Think of classification as intersection, i.e.
 a point clasified to an object lies within tolerance of the object's shape.

 The PointClassification encapsulates 
      the type  of the Classification Object,
      parameters of the closest Object Point to the Classification Point,
      a ClassificationPoint ZoneTol3d value, commonly set to equal the SmCurveClassification SrcZoneTol3d value.
      the Classification/Object Point deviation( i.e. Gap3d)
      and a SSSPoint used only to help the Boolean operation
        to map Points from IntersectionCurves that classify to Edges and Faces
        to actual surf/surf/surf intersections when that's appropriate.

NOTES: PointClassifications are used 
         o to classify a point against a Brep (SmBrep::PointClassify())
         o to classify a ray against a Brep   (SmBrep::RayFire())
         o to partition a curve being classified against a Brep into intervals
            (when owned by a SmCurveClassification object)

       This class cannot have virtual methods since SmCurveClassification
       manages an Array of SmCurveIntervals using smos_MemSet()
       and SmCurveIntervals contain three SmPointClassification objects. 
***********************************************************************/
#define SmPointClass SmPointClassification

class SM_EXPORT SmPointClassification 
{
  friend class SmCurveClassification ; // allows access to private SmCurveClassification::SetCurveZoneTol3d() method
  friend class SmTol ;                 // allows access to private SmTol::SetXSectTol3d() method

protected:
  const SmContext          * m_cpContext = NULL ; // context used for new object constructions

  SmZoneTol3d                m_sSrcZoneTol3d ;  // When classifying a point against a Brep - copy of Point's ZoneTol3d value
                                                // When classifying a ray   against a Brep - copy of Ray's   ZoneTol3d value
                                                // When classifying a curve against a Brep - copy of Curve's ZoneTol3d value
                                                // uninit:[SM_UNINIT_TOL]
                             
  SmPointClassificationType  m_ePointClass = SM_PC_UNKNOWN ; // Type of the Object on which the point lies or is near(became redundant when m_pObject was added - could be removed)
  SmObject                 * m_pObject = NULL ;              // Object on which the point lies or is near

  const SmFace             * m_cpFace = NULL ;               // opt: The face when the point was classified against a face


  double                     m_dGap3d = SM_UNDEF_DOUBLE ;    // Gap3d = ObjPt/ClassifyCrv GapLength = DropToClassifyCrv(PtClassify->ObjPt)->Deviation
                                                             //   (users of this object must check for and handle cases where
                                                             //    the m_dGap3d value exceeds the XSectTol3d(SrcZoneTol3d,pObjectZoneTol3d) value)  
  double                     m_dTParam = SM_UNDEF_DOUBLE ;   // The T parameter of curve/edge m_pObject
  SmPoint2d                  m_vUVParam;                     // The UV parameter of a surface/face m_pObject
  SmPoint3d                  m_vUVWParam;                    // The UVW parameter of a volume m_pObject
  SmBoolean                  m_bParamSet = FALSE   ;         // Is the appropriate T, UV, or UVW Param value set?

// GWC:CHANGE_SSS
  double                     m_dPreSnapParam = SM_UNDEF_DOUBLE ; // When classifying a curve: param value for the SmCurveClassification curve.
                                                                 //  When point comes from 
                                                                 //     an intersection - it's usually the intersection value
                                                                 //     a CurveClassification homogenize operation - it's the param value from the other classification.
                                                                 //     a dropedPoint - it's the closest value on the curve to the target point.
                                                                 //     
                                                                 
                                                                 //  when no snapping was done for classification then
                                                                 //  &(SmCurveInterval.m_vStart) == this && m_dPreSnapParam == SmCurveInterval.m_vInterval.GetMin() or
                                                                 //  &(SmCurveInterval.m_vEnd)   == this && m_dPreSnapParam == SmCurveInterval.m_vInterval.GetMax()
                                                                 // SM_BIG_DOUBLE to ignore
  SmBoolean                  m_bSSSPointFlag = FALSE ;           // TRUE=m_vSSSPoint is set, FALSE=not
  SmPoint3d                  m_vSSSPoint ;                       // Surf/Surf/Surf xsect point for points on curves that classify to edges
// end GWC:CHANGE_SSS

public: 
  // empty constructor
  SmPointClassification(SmZoneTol3d       sSrcZoneTol3d=SM_USE_DEFAULT,
                        const SmContext * cpContext=NULL) : m_cpContext(cpContext) 
                                                          { ReSet(&sSrcZoneTol3d) ; }
  // destructor
 ~SmPointClassification() { m_cpContext = NULL ; ReSet() ; }

  // ReSet all member values to init values
  void ReSet(SmZoneTol3d *pOptZoneTol3d=NULL) ;

  // assignment and equality operators
  SmPointClassification &operator= (SmPointClassification const & crPC) ;  // in : object being copied
  SmBoolean              operator==(SmPointClassification const & crPC) ;  // in : object being compared
                              
  // copy everything but the point data
  SmPointClassification &CopyClassificationOnly
    (SmPointClassification const & crPC) ;  // in : object being copied

  // only copy the SSSPoint and it's storage flag value
  void CopySSSPoint(const SmPointClassification & crPC) { m_bSSSPointFlag = crPC.m_bSSSPointFlag ;
                                                          m_vSSSPoint     = crPC.m_vSSSPoint     ; }

  // simple data access

  SmBrep                  * GetBrep(SmBrep *pSkipBrep=NULL) const ; // return Breps which are not pSkipBrep (pSkipBrep=NULL for any Brep) 
  const SmFace            * GetFace()                       const { return m_cpFace ; }
  SmLoop                  * GetLoop()                       const ; // return NULL when PtClassify->Object not connected to a Loop
  SmVertex                * GetVertexObject()               const { return m_ePointClass == SM_PC_VERTEX ? (SmVertex*)m_pObject : NULL; }
  SmEdge                  * GetEdgeObject()                 const { return m_ePointClass == SM_PC_EDGE   ? (SmEdge*)m_pObject   : NULL; }
  SmFace                  * GetFaceObject()                 const { return m_ePointClass == SM_PC_FACE   ? (SmFace*)m_pObject   : NULL; }

  void SetFace(const SmFace *cpFace)                 { m_cpFace = cpFace ; }

  // merge (insert) pointClassification into classification's Brep - part of the Boolean operation
  SmStatus MergeIntoObject(const SmPoint3d     & sSrcPnt,                  // in : 3D loc on some source object to use for new (or combined) vertex locations
                           SmTArray<SmEdge*>   * pOptNewEdges    = NULL,   // out: 2 edges from splitting existing edge (1 reused, 1 new)
                           SmTArray<SmVertex*> * pOptNewVertices = NULL) ; // out: New Vertex, if any

  const SmContext         * GetContext()       const { return m_cpContext ; }
  SmPointClassificationType GetPointClass()    const { return m_ePointClass ; }

  SmObject                * GetObject()        const { return m_pObject ; }
  SmCurve                 * GetCurveObject()   const ;  // When m_pObject is a curve   else return NULL
  SmSurface               * GetSurfaceObject() const ;  // When m_pObject is a surface else return NULL
  SmVolume                * GetVolumeObject()  const ;  // When m_pObject is a volume  else return NULL

  double                    GetGap3d(const SmSurface * pOptSurface=NULL) const ;
  double                    GetDeviation()     const { return GetGap3d() ; }  // obsolete: for backward compatibility
  double                    GetTParam()        const { return m_dTParam ; }
  SmPoint2d                 GetUVParam()       const { return m_vUVParam ; }
  SmPoint3d                 GetUVWParam()      const { return m_vUVWParam ; }
  SmBoolean                 AreParametersSet() const { return m_bParamSet; } // clear m_bParamSet with UnSetParameters() call.
                                                                             // set   m_bParamSet with ComputePointParameters() call
  double                    GetPreSnapParam()  const { return m_dPreSnapParam ; } // the param value on the curve being classified
  SmBoolean                 GetSSSPointFlag()  const { return m_bSSSPointFlag; }
  SmPoint3d                 GetSSSPoint()      const { return m_vSSSPoint; }

  SmBoolean                 IsConnectedTo(const SmTopology *pConnectedTarget) const ; // rtn: TRUE when ClassifyObject connects to Target
  SmBoolean                 IsConnectedTo(const SmTArray<SmTopology *> &crTopos,      // in : list of TgtTopologies to check for connections with this object
                                          ULONG & rFoundIndx)                         // out: indx in crTopos of  1st Topology found to connect to this object or SM_UNDER_ULONG for not connected
                                                                                      { rFoundIndx = SM_UNDEF_ULONG ;
                                                                                        for(ULONG ii=0;ii<crTopos.GetSize();ii++) 
                                                                                          { if(IsConnectedTo(crTopos[ii]))
                                                                                              { rFoundIndx = ii ; break ; }
                                                                                          }
                                                                                        return(rFoundIndx != SM_UNDEF_ULONG ? TRUE : FALSE) ;
                                                                                      }

  void SetContext     (const SmContext * cpContext)           { m_cpContext = cpContext ; }
  void SetClassObject (SmPointClassificationType ePointClass, 
                       SmObject * pObject);
  void SetGap3d       (double dGap3d)                         { m_dGap3d = dGap3d; }
  void SetDeviation   (double dGap3d)                         { SetGap3d(dGap3d) ; }   // obsolete: for backward compatibility
  void SetTParam      (double dTParam, SmBoolean bSnapToIvl=FALSE) ;
  void SetUVParam     (const SmPoint2d & crUVParam) ;
  void SetUVWParam    (const SmPoint3d & crUVWParam)          { m_vUVWParam     = crUVWParam; m_bParamSet=TRUE; }
  void UnSetParameters()                                      { m_bParamSet = FALSE; } // clears m_bParamSet
    
  void SetPreSnapParam(double dPreSnapParam)                  { m_dPreSnapParam = dPreSnapParam; }  // when classifying a curve = param value on the curve being classified
  void SetSSSPointFlag(SmBoolean bSSSPointFlag)               { m_bSSSPointFlag = bSSSPointFlag ; }
  void SetSSSPoint    (const SmPoint3d & crSSSPoint)          { m_bSSSPointFlag = TRUE ;
                                                                m_vSSSPoint     = crSSSPoint ; }

  // Tolerances: Fetch tolerance values with calls:
  //     SmTol::GetZoneTol3d (pPointClassification)              = PointClass Source's ZoneTol3d Value
  //     SmTol::GetXSectTol3d(pPointClassification)              = PointClass Source/Object intersection 3d tolerance
  //     SmTol::GetZoneTol3d (pPointClassification->GetObject()) = PointClass Object's ZoneTol3d Value
  //   after setting member value m_sSrcZoneTol3d via the constructor or with SetSrcZoneTol3d().

  // GWC: becomes obsolete when uses switch to SmTol::GetXSectTol3d(pPC) and SmTol::GetZoneTol3d(pPC)syntax
  SM_OLDTOL_LINE SmZoneTol3d  GetTolerance()          const { return(m_sSrcZoneTol3d) ; } // obsolete name: now returns this PtClassification's m_sZoneTol3d value.

private: // only for use in SmTol.cpp (supports SmTol::GetZoneTol3d(pPC) and SmTol::GetXSectTol3d(pPC))
  SM_TOL_LINE    SmZoneTol3d  GetObjZoneTol3d()       const { return(SmTol::GetZoneTol3d(m_pObject ) ) ; }
  SM_TOL_LINE    SmZoneTol3d  GetSrcZoneTol3d()       const { SM_ASSERT_TOL(m_sSrcZoneTol3d) ; return(m_sSrcZoneTol3d) ; }              // rtn: ZoneTol3d of curve, pt, or ray being classified
public:
  SM_TOL_LINE void SetSrcZoneTol3d      (SmZoneTol3d sSrcZoneTol3d)        { m_sSrcZoneTol3d = sSrcZoneTol3d ; }

  // return pointClassification with highest precidence
  const SmPointClassification & Combine(const SmPointClassification & crPointClass) ;

  // project crPoint to m_pObject, set m_bParamSet, save m_dGap3d, and m_dTParam, m_vUVParam or m_vUVWParam vals
  SmStatus ComputePointParameters(const SmPoint3d & crPoint,                     // in : Point being classified to object 
                                  double dOptClassifyCrvParam=SM_UNDEF_DOUBLE) ; // NotUsed: in : Optional ClassifyCrvParam when known
                                                                                 //      SM_UNDEF_DOUBLE = not given, default:[SM_UNDEF_DOUBLE]
                                                                                 // 
  // when m_bParamSet==TRUE, set rPoint = m_pObject->Evaluate(ParamValues) and rtn SM_SUCCESS, else return SM_ERR.
  // see ComputePointParameters()
  SmStatus FindObjPoint3d( SmPoint3d & rPoint ) const;

  // load PointClassification with sSolution from a ThisCurve/cpObject intersection
  SmStatus LoadFromIntersection(SmZoneTol3d           sSrcZoneTol3d,    // in : ZoneTol3d of Src in XSect(Src,Obj), e.g. the SmCurveClassification::SrcZoneTol3d val
                                SmObject            * cpObject,         // in : Obj in the XSect(Src,Obj) call that produced crSolutionend
                                const SmSolutionEnd & crSolutionEnd,    // in : contains solution and parameter values for intersection
                                ULONG                 lOffset,          // in : index into crSolutionEnd.m_adParameters array for sol's 1st cpObject param 
                                double dPreSnapParam = SM_BIG_DOUBLE) ; // in : sol val remembered when Point sol comes from a curve/object solution 

  // refine point classification when told underlying curve was modified a small amount
  //   SmStatus AdjustToModifiedCurve(const SmCurve *cpCurve, double dParam, double &dNewParam);

  // The following two methods have been removed.
  // Use SetClassObject() instead, to avoid type/object conflicts.
  //void SetPointClass  (SmPointClassificationType ePointClass);
  //void SetObject      (SmObject *pObject);

  // for debug purposes
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                          //      default:[SM_LEVEL_0] 
                                SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                               const ;
  // obsolete
  //  virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON_ONLY_BASE(SmPointClassification, SmPointClassification_TYPE);
                                                                                      
  void Dump(ULONG            lPointDim,
            double           dClassifyParam=SM_BIG_DOUBLE,    // in : SmCurveInterval Param associated with this point, SM_BIG_DOUBLE to ignore                                          
            const SmPoint3d *pPoint   =NULL, 
            const SmSurface *pSurface1=NULL, 
            const SmSurface *pSurface2=NULL) const;
  SmDisplayList * Draw(void) const;

} ; // end class SmPointClassification

#endif // !__SMPOINTCLASSIFICATION_H__
