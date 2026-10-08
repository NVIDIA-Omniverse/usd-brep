// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPrimitiveCreation.h
* PURPOSE: Header file for the class. 
**********************************************************************/

#ifndef __SMPRIMITIVECREATION_H__
#define __SMPRIMITIVECREATION_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

#ifndef __SMMAPPTRTOPTR_H__
#include <SmMapPtrToPtr.h>
#endif

#ifndef __SMTOPOLOGYSWEEP_H__
#include <SmTopologySweep.h>
#endif

/*******************************************************************//**
PURPOSE: The 2D Boolean Operation Type defines what sort of operation
    is being performed by SmPrimitiveCreation::Boolean2D

NOTES: 
***********************************************************************/
enum Sm2DBooleanOperationType
{
    SM_2D_UNION,        // Union of 2D regions A and B
    SM_2D_INTERSECTION, // Intersection of 2D regions A and B
    SM_2D_DIFFERENCE,   // Difference - A minus B
    SM_2D_EXCLUSIVE_OR, // XOR = (A Union B) - (A intersect B)
    SM_2D_MERGE         // Merge Operation
} ;

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmSweepOptions
{
  public:
      SmBoolean bCapEndsArg = false;                       // if true, cap both ends of the result
                                                           // this can only occur when profile is closed
      SmBoolean bTranslationalSweep = false;               // if true, profile will only translate along path
                                                           // profile will not change orientation as it travels down path
      SmBoolean bMoveProfileCenterToPath = false;          // if true, move profile so that its center is at beginning of path
      SmBoolean bOrientProfilePerpendicularToPath = false; // if true, orient the profile so that it begins perpendicular to the path
      SmBoolean bMoveToProfile = false;                    // if true, move the result back to the original position of the profile
                                                           // otherwise the result will be at the path
} ;

/*******************************************************************//**
PURPOSE: The following class is a container for various primitive
            solid creation functions.

NOTES: 
  1. Create an instance of SmPrimitiveCreation with a pointer to a 
     valid Brep->Region.

  2. Execution of any SmPrimitiveCreation::Create...() function creates
     a primitive in the Brep->Region stored in m_pRegion.

  3. Entity maps are available after most operations, which record which
     new entities are created from which originals.  Topological entities
     are recorded: Vertices, Edges, and Faces.  There are two pairs
     of maps, one pair recording entity pairs of the same dimensionality
     (e.g., an Edge swept into another Edge), and the other recording
     pairs swept into higher dimensionality (e.g., a Face that is created
     by sweeping an Edge).  The easiest ways to use the maps (implemented
     using class SmMapPtrToPtr) is with the methods GetAllKeyValuePairs()
     and the operator[].
***********************************************************************/
class SM_EXPORT SmPrimitiveCreation
{
protected:
  SmRegion    * m_pRegion;          // target region to contain created primitives
                                    // Not owned: not deleted upon destruction.
  long          m_lTagID;           // 0 means no tagging 

  // Topology/geometry maps, for methods that take topology/geometry as input.
  SmMapPtrToPtr<SmTopology, SmTopology> m_vMapToSame;
  SmMapPtrToPtr<SmTopology, SmTopology> m_vMapToHigher;
  SmMapPtrToPtr<SmTopology, SmTopology> m_vMapFromSame;
  SmMapPtrToPtr<SmTopology, SmTopology> m_vMapFromLower;

  // default constructor
  SmPrimitiveCreation()             
    : m_pRegion(NULL), 
      m_lTagID(0)
  { }

  // Protected functions designed to directly create full, analytic solids
  // These may be elevated to public functions if necessary
  SmStatus CreateCone
  (
    double dHeight,                       // in : height of result                                      
    double dBaseRadius,                   // in : radius of base                                        
    double dTopRadius,                    // in : cylinder if same as dBaseRadius. cone if different    
    const SmAxis2Placement & crRefFrame   // in : position and orientation of result                    
  );

  SmStatus CreateSphere
  (
    double dRadiusArg,                    // in : radius of sphere                      
    const SmAxis2Placement & crRefFrame   // in : position and orientation of result    
  );

  SmStatus CreateTorus
  (
    double dMajorRadius,                  // in : major radius of torus                
    double dMinorRadius,                  // in : minor radius of torus                
    const SmAxis2Placement & crRefFrame   // in : position and orientation of result   
  );

  SmStatus CreateLinearSweep
  (
    SmTArray<SmCurve*> & cp3DCurves,      // in : Must be coplanar (consumed)     
    const SmVector3d   & crSweepVec,      // in : The sweep vector                
    double               dSweepDist,      // in : Sweep = SweepDist * SweepVec    
    SmBoolean            bCapEnds         // in : TRUE = build end caps           
  );

  SmStatus CreateLinearSweep
  ( 
    SmTArray<SmFace *>  & aFaces,         // in : Faces to sweep             
    const SmVector3d    & crSweepVec,     // in : The sweep vector           
    double                dSweepDist,     // in : The sweep distance         
    SmBoolean             bCapEnds        // in : Cap ends if true           
  );

  SmStatus CreateRotationalSweep
  (
    SmTArray<SmCurve*> & cp3DCurves,      // in : Curves to revolve. Must be coplanar. They are consumed       
    const SmPoint3d    & crRotAxisBasePt, // in : Position of axis of revolution                               
    const SmVector3d   & crRotAxisDir     // in : Direction of axis of revolution
  );

public:

  // Constructor
  SmPrimitiveCreation
  (
    SmRegion* pRegionArg, 
    long      lTagID = 0,
    SmBoolean bUseAnalytics = false
  )  
    : m_pRegion(pRegionArg), 
      m_lTagID(lTagID)
  { SM_REF1(bUseAnalytics) ; }

  // destructor
  ~SmPrimitiveCreation()
  { 
    m_pRegion = NULL ;
    m_lTagID  = 0 ;
  }
  
  // simple data access
  SmMapPtrToPtr<SmTopology, SmTopology> * GetMapToSame()    { return & m_vMapToSame;    }
  SmMapPtrToPtr<SmTopology, SmTopology> * GetMapToHigher()  { return & m_vMapToHigher;  }
  SmMapPtrToPtr<SmTopology, SmTopology> * GetMapFromSame()  { return & m_vMapFromSame;  }
  SmMapPtrToPtr<SmTopology, SmTopology> * GetMapFromLower() { return & m_vMapFromLower; }

  void InitEntityMaps();
  void CopyEntityMaps  ( SmTopologySweep * pSweep );
  void AppendEntityMaps( SmTopologySweep * pSweep );

  // Change the target region
  void Become(SmRegion* pRegionArg) { m_pRegion = pRegionArg; }

  //***************************************************
  // Common Shapes Rectangle, Circle
  //***************************************************
  SmStatus CreateRectangle
  ( 
    double dWidth,                          // in : Width along X Axis of the reference frame.                        
    double dHeight,                         // in : Height along Y Axis of the reference frame.                       
    const SmAxis2Placement & crRefFrame     // in : origin = bottom right corner of the surface,                      
                                            //    : note: surface made in the RefFram XY plane's positive quadrant.   
  );

  SmStatus CreateCircle
  ( 
    double dRadius,                         // in : Radius of circle                                                   
    const SmAxis2Placement & crRefFrame     // in : Center and orientation of the circle.                              
                                            //    : circle is built centered on the RefFrame's origin in the XY plane  
  );
  
  //***************************************************
  // Common Volumes Box, Cone, Cylinder, Torus, Sphere
  //***************************************************
  SmStatus CreateBox
  (
    double dLength,                         // in : extent along X axis of frame              
    double dWidth,                          // in : extent along Y axis of frame              
    double dHeight,                         // in : extent along Z axis of frame              
    const  SmAxis2Placement & crRefFrame    // in : defines box min corner and orientation    
  ); 

  SmStatus CreateCylindricalBox
  (
    double dLength,                         // in : length measured along Z Axis starting at RefFram Origin, [greater than zero]            
    double dInsideRadius,                   // in : Inside Radius measured from Z Axis, [greater than zero]                                 
    double dOutsideRadius,                  // in : Outside Radius measured from Z Axis, [greater than InsideRadius]                        
    double dStartAngDeg,                    // in : Start Position measured from X Axis in YX plane, [-360 to 360]                          
    double dEndAngDeg,                      // in : Start Position measured from X Axis in YX plane, [EndAng-StartAng in range:(0 to 360]   
    const  SmAxis2Placement & crRefFrame    // in : Z Axis  = Cylinder centers                                                              
                                            //    : Origin  = Bot of CylBox                                                                 
                                            //    : XY Axes = Plane of Start and End angles measured from X Axis                            
  );

  SmStatus CreateCone
  (
    double dHeight,                               // in : height of result                                     
    double dBaseRadius,                           // in : radius of base                                       
    double dTopRadius,                            // in : cylinder if same as dBaseRadius. cone if different   
    double dStartAngleDeg,                        // in : measured in the XY plane from the X axis             
    double dEndAngleDeg,                          // in : measured in the XY plane from the X axis             
    const  SmAxis2Placement & crRefFrame          // in : position and orientation of result                   
  );

  SmStatus CreateSphere
  (
    double dRadiusArg,                            // in : radius of sphere                               
    double dStartAngleDeg,                        // in : measured in the XY plane from the X axis       
    double dEndAngleDeg,                          // in : measured in the XY plane from the X axis       
    const  SmAxis2Placement & crRefFrame          // in : position and orientation of result             
  );

  SmStatus CreateTorus
  (
    double dMajorRadius,                          // in : major radius of torus                                         
    double dMinorRadius,                          // in : minor radius of torus                                         
    double dStartAngleDeg,                        // in : measured in the XY plane from the X axis                      
    double dEndAngleDeg,                          // in : measured in the XY plane from the X axis                      
    const  SmAxis2Placement & crRefFrame          // in : position and orientation of result                            
  );

  //***********************************************
  // Static functions resulting in new brep
  //***********************************************
  static SmStatus CreateSphereNoPole
  (
    const SmContext *pContext,                    // in : context for new object construction                             
    double           dRadius,                     // in : desired sphere radius                                           
    SmVector3d       sCenter,                     // in : desired sphere center                                           
    double           dTolerance,                  // in : max dist between distinct points (set less than Brep->Tol/2.0)  
    SmBrep        *& pSphere                      ///< [in,out]: Empty Brep Target in which to build the Sphere               
  );

  static SmBrep * CreateRectangle
  (
    const SmContext &        crContext,           // in : context for new object construction                             
    double                   dTolerance,          // in : Brep tolerance                                                  
    double                   dWidth,              // in : Width along X Axis of the reference frame.                      
    double                   dHeight,             // in : Height along Y Axis of the reference frame.                     
    const SmAxis2Placement & crRefFrame           // in : origin = bottom right corner of the surface,                    
                                                  //    : note: surface made in the RefFram XY plane's positive quadrant. 
  );
    
  static SmBrep * CreateCircle
  (
    const SmContext &        crContext,           // in : context for new object construction                                 
    double                   dTolerance,          // in : Brep Tolerance                                                      
    double                   dRadius,             // in : Radius of circle                                                    
    const SmAxis2Placement & crRefFrame           // in : Center and orientation of the circle.                               
                                                  //    : circle is built centered on the RefFrame's origin in the XY plane   
  );
    
  //*******************************************************************
  // Extrude and Revolve Primitive Methods
  //*******************************************************************

  SmStatus CreateLinearSweep
  (
    SmTArray<SmCurve*> & cp3DCurves,              // in : Must be coplanar - ObjMem:[owned and managed by new Edges] 
    const SmVector3d   & crSweepVec,              // in : The sweep vector                                          
    double               dSweepDist,              // in : Sweep = SweepDist * SweepVec                              
    ULONG                nRepetitions,            // in : Number of end to end sweeps                               
    SmBoolean            bCapEnds,                // in : TRUE = build end caps                                     
    SmBoolean            bTestContinuity = TRUE   // in : TRUE = split input curves at C1 discontinuties               
  );                                              //      FALSE= use input curves as is. default:[TRUE]
                                       
  SmStatus CreateLinearSweepSimple
  (
    SmTArray<SmCurve*> & cp3DCurves,      // in : Must be coplanar (consumed)     
    const SmVector3d   & crSweepVec,      // in : The sweep vector                
    double               dSweepDist,      // in : Sweep = SweepDist * SweepVec    
    SmBoolean            bCapEnds         // in : TRUE = build end caps           
  );

  SmStatus CreateLinearSweep
  ( 
    SmTArray<SmFace *> & aFaces,                  // in : Faces to sweep                 
    const SmVector3d   & crSweepVec,              // in : The sweep vector               
    double               dSweepDist,              // in : The sweep distance             
    ULONG                nRepetitions,            // in : Number of end to end sweeps    
    SmBoolean            bCapEnds                 // in : Cap ends if true               
  );    


  SmStatus CreateRotationalSweep
  (
    SmTArray<SmCurve*> & cp3DCurves,              // in : Curves to revolve. Must be coplanar. They are consumed   
    const SmPoint3d    & crRotAxisBasePt,         // in : Position of axis of revolution                           
    const SmVector3d   & crRotAxisDir,            // in : Direction of axis of revolution                          
    double               dRotAngleDeg,            // in : Degrees 0->360                                           
    ULONG                nRepetitions,            // in : Number of end to end revolutions                         
    SmBoolean            bCapEnds,                // in : Cap ends if true                                         
    SmBoolean            bTestContinuity = TRUE   // in : Test for tangent continuity if true                      
  ); 
        

  //*******************************************************************
  // Advanced Primitive Methods
  //*******************************************************************

  SmStatus CreateCurveSweep
  (
    SmTArray<SmCurve*> & rProfileCurves,    // in : source curves to sweep.                                       
    SmBSplineCurve     * pPathCurve,        // in : path curve along which to sweep                               
    SmBSplineCurve     * pScaleReference,   // in : used together with pScaleCurve to define change along path    
    SmBSplineCurve     * pScaleCurve,       // in : defines how the profiles can change as it travels down path   
    double               bAppoxTol3d,       // in : 3D distance tolerance                                         
    SmSweepOptions     * pOptions,          // in : several independent boolean flags                             
    SmTArray<SmFace*>  & rStartFaces,       // out: list of new faces for start cap                               
    SmTArray<SmFace*>  & rSideFaces,        // out: list of new faces added to this brep                          
    SmTArray<SmFace*>  & rEndFaces,         // out: list of new faces for end cap                                 
    SmBoolean bTestContinuity = TRUE        // in : test source curves for continuity                             
  );      

  // Extracts curves from the face edges and calls CreateCurveSweep
  SmStatus CreateCurveSweepFromFaces
  (
    SmTArray<SmFace*> & rFaces,               // in : faces to sweep                                                  
    SmBSplineCurve    * pPathCurve,           // in : path curve along which to sweep                                 
    SmBSplineCurve    * pScaleReference,      // in : used together with pScaleCurve to define change along path      
    SmBSplineCurve    * pScaleCurve,          // in : Defines how the profiles can change as it travels down pat      
    double              dAppoxTol3d,          // in : 3D distance tolerance                                           
    SmSweepOptions    * pOptions,             // in : several independent boolean flags                               
    SmTArray<SmFace*> & rStartFaces,          // out: list of new faces for start cap                                 
    SmTArray<SmFace*> & rSideFaces,           // out: list of new faces added to this brep                            
    SmTArray<SmFace*> & rEndFaces             // out: list of new faces for end cap                                   
  );

  // Extracts curves from the edges and calls CreateCurveSweep
  SmStatus CreateCurveSweepFromEdges
  (
    SmTArray<SmEdge*> & rEdges,               // in : curves to sweep                                                
    SmBSplineCurve    * pPathCurve,           // in : path curve along which to sweep                                
    SmBSplineCurve    * pScaleReference,      // in : used together with pScaleCurve to define change along path     
    SmBSplineCurve    * pScaleCurve,          // in : Defines how the profiles can change as it travels down path    
    double              bAppoxTol3d,          // in : 3D distance tolerance                                          
    SmSweepOptions    * pOptions,             // in : several independent boolean flags                              
    SmTArray<SmFace*> & rStartFaces,          // out: list of new faces for start cap                                
    SmTArray<SmFace*> & rSideFaces,           // out: list of new faces for sweep surface                            
    SmTArray<SmFace*> & rEndFaces             // out: list of new faces for end cap                                  
  );

  // Obsolete: replaced by CreateTaperExtrude
  SmStatus CreateDraftSweep
  (
    SmTArray<SmCurve*> & cr3DCurves,          // in : must be coplanar - consumed by this method.    
    SmVector3d         & crSweepVec,          // in :                                                
    double               dHeight,             // in : degrees                                        
    double               dDraftAngleDeg,      // in : 0-90 inner, -90,0=outer                        
    SmBoolean            bCapEnds             // in :                                                
  );

  SmStatus CreateTaperExtrude
  (
    SmTArray <SmCurve*> & cr3DCurves,                             // in : source curves (dir may get reversed if needed and not already head-tail)    
    double                dHeight,                                // in : + = direction of CrvsNormal, - = opp                                        
    double                dDraftAngleDeg,                         // in : 0=along normal  0-90=inner 90-180=outer                                     
    int                   iEndCaps,                               // in : 0=none, 1=at curves end, 2=at offset end, 3=both                            
    SmVector3d          & crCrvsNormal,                           // in : specified normal to plane of curves                                         
                                                                  //    : or enter [0,0,0] for us to compute it for you.                              
    SmOffsetCornerType    eOffsetCorner = SM_OC_LINEAR_EXTENSION  // in : oneof SM_OC_LINEAR_EXTENSION or SM_OC_FILLET_CORNER                         
  );

  SmStatus CreatePipeSweep
  (
    double              dPipeRadius,                 // in :        
    SmBSplineCurve    * pPathCurve,                  // in :        
    double              dThisApproxTol3d,            // NotUsed: in :        
    SmBoolean           bCapEnds,                    // in :        
    SmBoolean           bTranslationalSweep,         // in :        
    SmTArray<SmFace*> & rStartFaces,                 // out:        
    SmTArray<SmFace*> & rSideFaces,                  // out:        
    SmTArray<SmFace*> & rEndFaces                    // out:        
  );

  static SmStatus CreateSweepAlongPlanarPath
  (
    const SmContext    & crContext,                  // in :        
    SmTArray<SmCurve*> & rProfileCurves,             // in :        
    SmTArray<SmCurve*> & rPathCurves,                // in :        
    double               dTol,                       // in :        
    SmBoolean            bCapEndsArg,                // NotUsed: in : Legacy entry point always caps both open ends.
    SmBoolean            bMoveProfileCenterToPath,   // in :        
    SmBrep            *& sweepBrep                   // out:        
  ); 

  static SmStatus CreateSweepAlongPlanarPath
  (
    const SmContext    & crContext,                  // in :
    SmTArray<SmCurve*> & rProfileCurves,             // in :
    SmTArray<SmCurve*> & rPathCurves,                // in :
    double               dTol,                       // in :
    SmBoolean            bCapStart,                  // in : Cap the profile/start end of an open path.
    SmBoolean            bCapEnd,                    // in : Cap the far/path end of an open path.
    SmBoolean            bMoveProfileCenterToPath,   // in :
    SmBrep            *& sweepBrep                   // out:
  );

  // Create a sheet or a solid model within the contained Brep by lofting a set of profiles.
  SmStatus CreateSkinPrimitive
  (                                    
    SmTArray<ULONG>    & rCurvesInEachProfile,    // in : Number of curves in each profile                                               
    SmTArray<SmCurve*> & r3DCurves,               // in : Array of all profile segments                                                  
    double               dThisApproxTol3d,        // in : 0.0 = loft surfaces interpolate loft curves,                                   
                                                  //    : else loft surfaces approximate loft surfaces.                                  
    ULONG                lDegree,                 // in : degree of loft surfaces in loft direction, 1=linear,2=parabolic,3=cubic        
    SmBoolean            bCapEnds,                // in : TRUE = cap first and last profiles, FALSE = no cap faces
    SmTArray<SmFace*>  & rStartFaces,             // out: list of start cap faces
    SmTArray<SmFace*>  & rSideFaces,              // out: list of side faces across all loft spans
    SmTArray<SmFace*>  & rEndFaces                // out: list of end cap faces
  );

  // Create a skin while reporting ownership transferred from r3DCurves.
  // Each output flag corresponds to one input curve. TRUE means the operation
  // consumed that pointer and the caller must not delete it; FALSE means the
  // pointer remains caller-owned, including on failure.
  SmStatus CreateSkinPrimitive
  (
    const SmTArray<ULONG>& rCurvesInEachProfile,
    SmTArray<SmCurve*>  & r3DCurves,
    double                dThisApproxTol3d,
    ULONG                 lDegree,
    SmBoolean             bCapEnds,
    SmTArray<SmFace*>   & rStartFaces,
    SmTArray<SmFace*>   & rSideFaces,
    SmTArray<SmFace*>   & rEndFaces,
    SmTArray<SmBoolean> & rInputOwnershipTransferred
  );

  // build a G1/G2/G3 blend between two surface boundary curves
  SmStatus CreateBlendPrimitive 
  ( 
    SmEdgeuse       * pEU1,                                   // in : target start blend surface                                         
    SmEdgeuse       * pEU2,                                   // in : start surface boundary curve                                       
    SmFace         *& rpBlendFace,                            // out:                                                                    
    SmCurvatureType   eCurvatureType = SM_CT_G2_FROM_SURFACE, // in : blend surface continuity                                           
    SmBlendEndType    eBlendEndType  = SM_BE_NO_CUSP,         // in : blend ends smooth or cusped                                        
    ULONG             lCurvesDirFlag = 0,                     // in : 0 = Guess curve directions based on geometry                       
                                                              //    : 1 = run blend between start of EU1->Curve to start of EU2->Curve   
                                                              //    : 2 = run blend between start of EU1->Curve to end of EU2->Curve     
    double            dStartTanLength = 1.,                   // in : Take-off vector scaling at start of EU1->Curve                     
    double            dEndTanLength   = 1.                    // in : Take-off vector scaling at end of EU1->Curve                       
  );                  

  SmStatus CreateBlendPrimitive
  ( 
    SmFace                * pFace1,           // in : target start blend surface                                                
    SmEdge                * pEdge1,           // in : start surface boundary curve                                              
    SmFace                * pFace2,           // in : target end blend surface                                                  
    SmEdge                * pEdge2,           // in : end surface boundary curve                                                
    SmBoolean               bSameDirCurves,   // in : TRUE = run blend between start of Edge1->Curve to start of Edge2->Curve   
                                              //    : FALSE= run blend between start of edge1->Curve to end of Edge2->Curve     
    SmDerivSurfDefinition * pOptDSDef,        // in : optional blend options, NULL to ignore                                    
                                              //    : NULL = use values hardcoded into this function                            
                                              //    : hardcode's G2 blends                                                      
    SmFace               *& rBlendFace        // out:                                                                           
  );

  SmStatus CreateSwungPrimitive
  (
    SmTArray<SmCurve*> & rXYCurves,           // in :       
    SmTArray<SmCurve*> & rXZCurves,           // in :       
    double               dScale,              // in :                
    double               dThisApproxTol3d     // in :       
  );

  
  // loft the loops on a sequence of faces or loft a sequence of loft curves
  SmStatus CreateSkinFromFaces
  (
    SmTArray<SmFace*>         & rFaces,             // in : When given, create lofts between matching loops on a sequence of faces   
    SmTArray<SmBSplineCurve*> & rCurvesToSkin,      // in : when rFaces is empty, a sequence of profiles to loft                     
                                                    //    : profile curves are always u isoparameter curves in the loft surfaces.    
    double                      dThisApproxTol3d,   // in : 0.0 = loft surface interpolates loft profiles,                           
                                                    //    : else loft surface approximates loft profile.                             
    ULONG                       lDegree,            // in : degree in loft direction, 1=linear, 2=parabolic, 3=cubic                 
    SmTArray<SmFace*>         & rNewFaces           // out: One lofted face for every set of matched loops from rFaces or            
                                                    //    : One lofted face running through rCurvesToLoft                            
  );                                                

  // note: increments unlocked mark value
  static SmStatus Boolean2D
  (
    SmBrep *pBrepA,                           // in : A of A operator B, returned as rpResult                            
    SmBrep *pBrepB,                           // in : B or A operator B, deleted by this operation                       
    Sm2DBooleanOperationType eBooleanType,    // in : oneof: SM_2D_UNION,        = Union of 2D regions A and B           
                                              //    :        SM_2D_INTERSECTION, = Intersection of 2D regions A and B    
                                              //    :        SM_2D_DIFFERENCE,   = Difference - A minus B                
                                              //    :        SM_2D_EXCLUSIVE_OR, = XOR = (A union B) - (A intersect B)   
                                              //    :        SM_2D_MERGE         = Merge Operation                       
    SmBrep *& rpResult                        // out: Modified pBrepA                                                    
  ); 

  static SmStatus OffsetProfile
  (
    const SmContext    & crContext,           // in : context for new object construction
    SmTArray<SmCurve*> & r3DCurves,           // in : Must all lie in a plane and form at least one closed loop.
                                              //    : Curves are consumed individually as construction proceeds.
    double               dThisApproxTol3d,    // in :
    double               dOffsetDistance,     // in : Size of Offset
    ULONG                lOffsetType,         // in : Offset Type  1-LEFT, 2-RIGHT, 3-BOTH
    SmBoolean            bRoundCorners,       // in : TRUE  = Add rounded corners when expanding a corner
                                              //    : FALSE = extend corners
    SmBoolean            bShellResult,        // in : Only works for LEFT or RIGHT offsets
    SmBrep            *& rpResult,            // out: New Brep containing Face between r3DCurves and their offsets
    SmTArray<SmBoolean>& rInputOwnershipTransferred // out: one flag per r3DCurves entry; TRUE once that
                                              //    : curve is no longer caller-owned, including on partial failure.
                                              //    : FALSE entries remain caller-owned and must be cleaned up by caller.
  );

 static SmStatus CreateConeEllipticEnds
 (
   double       front_radius,       // in : applies to cone radius at origin of endcap (before tilt)  
   double       rear_radius,        // in :                                                           
   double       height,             // in : measured from the center(origin) of each end-ellipse      
   double       front_Y_tilt,       // in : tilt about the X axis in degrees                          
   double       rear_Y_tilt,        // in :                                                           
   double       front_X_tilt,       // in : tilt about the Y axis in degrees                          
   double       rear_X_tilt,        // in :                                                           
   SmBoolean    bEndcaps,           // in : if TRUE, close off end caps                               
   SmBrep    *& pCone               // out: external pBrep = new (*pContext) SmBrep;                  
 );

} ; // end class SmPrimitiveCreation

#endif // !__SMPRIMITIVECREATION_H__
