// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmSrfInVolume.h
* PURPOSE: Header file for compound 3D surface defined as a 3D
*    parameter space surface and the volume through which it is projected.
**********************************************************************/

#ifndef __SMSRFINVOLUME_H__
#define __SMSRFINVOLUME_H__

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

class SmVolume ;
class SmGfxArraySet ;

/*******************************************************************//**
PURPOSE: The SmSrfInVolume class defines a 3d Surface which is the
    compound projection of a 3d SmSurface through a SmVolume.

NOTES: 1. Projects the Surface from the Volume's InSpace to its OutSpace
       2. When Volume is a compound Volume, projects the curve
           through the compound sequence of Volume Inspaces to OutSpaces
           ending in the last compounded volume's outspace. 
***********************************************************************/
class SM_EXPORT SmSrfInVolume : public SmSurface
{
protected:
  SmSurface * m_pSurface ;              // Parameter space surface  (When Owned, Owner set to this SmCrvInVolume)
  SmBoolean   m_bInParamSpace ;         // TRUE = m_pSurface is in m_pVolume's ParamSpace
                                        // FALSE= m_pSurface is in m_pVolume's InSpace
  SmVolume  * m_pVolume ;               // Base volume              (When Owned, Owner set to this SmCrvInVolume)
  ULONG       m_lOwnerFlag ;            // Constructor default:[0]
                                        // in : 0 = deletes nothing when destructed      (don't change numbers - used as BitArray)
                                        //      1 = deletes only surface when destructed
                                        //      2 = deletes only volume when destructed
                                        //      3 = deletes both surface and volume when destructed
                                        //      default:[0]
                                        // Note: anything owned by SmSrfInVolume is copied during 
                                        //       construction and deleted during destruction
                                        //       and has its m_pOwner pointer set to this SmSrfInVolume object.
                                        //       Never let anything owned by this SmSrfInVolume have two owners.
public:                              
  // constructor
  SmSrfInVolume
  (
    SmSurface        & rSurface,            // in : projected Surface - When(lOwnerFlag&1) rSurface->m_pOwner = this      
    SmBoolean          bInParamSpace,       // in : TRUE = m_pSurface is in m_pVolume's ParamSpace, FALSE = in InSpace    
    SmVolume         & rVolume,             // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this       
    ULONG              lCopyFlag = 0,       // in : 0 = saves surface and volume without copying                          
                                            //    : 1 = copy surface and save volume orig                                 
                                            //    : 2 = copy volume and save surface orig                                 
                                            //    : 3 = copy both surface and volume                                      
    ULONG              lOwnerFlag = 0,      // in : 0 = deletes nothing when destructed                                   
                                            //    : 1 = delete surface but not volume when destructed                     
                                            //    : 2 = delete volume but not surface when destructed                     
                                            //    : 3 = delete both surface and volume when destructed                    
                                            //    : default:[0]                                                           
    const SmContext  * cpContext = NULL     // in : req for new stack objs, opt for new heap objs.                        
  );

  // empty constructor for I/O
  SmSrfInVolume() { }

  // copy constructor
  SmSrfInVolume(const SmSrfInVolume & crSurfaceToCopy) ;

  // copy
  virtual SmStatus Copy
  (
    const SmContext & crContext,            // in :   
    SmSurface *& crNewSurface               // out:   
  ) const;

  // destructor
  virtual ~SmSrfInVolume() ;

  // equality operator
  virtual SmBoolean operator==(const SmSurface &crOther) const;

  // make exact BSpline equivalent surface mapping m_pSurface to the Volume's final OutSpace if possible
  virtual SmStatus MakeExactBSplineIfPossible(SmBSplineSurface *& rpNewBSplineSurface) const ; 

  // create a mirror copy of this object
  virtual SmStatus CreateMirrorSurface
  (
    const SmContext & crContext,                  // in : context for new object       
    const SmAxis2Placement & crMirrorPlane,       // in : mirror plane                 
    SmSurface *& rpMirrorSurface                  // out: mirrored surface             
  ) const;

  // simple data access
          SmSurface        * GetSurface()               const { return m_pSurface ; }
          SmBoolean          GetInParamSpace()          const { return m_bInParamSpace ; }
  virtual SmBSplineSurface * GetRootSurface()           const { return m_pSurface->GetRootSurface() ; }
          SmVolume         * GetVolume()                const { return m_pVolume ; }
  virtual SmExtent2d         GetNaturalUVDomain()       const { return m_pSurface->GetNaturalUVDomain() ; }
  virtual SmExtent2d         GetSTEPUVDomain()          const { return m_pSurface->GetSTEPUVDomain() ; }
  virtual SmExtent2d         GetMaxAnalyticDomain()     const { return m_pSurface->GetMaxAnalyticDomain() ; }
  virtual ULONG              GetDegree(SmSurfParamType) const { return 3; }

  virtual SmStatus           GetKnots
  (
    SmSurfParamType    eSurfParam,                      // in :                                                               
    SmTArray<double> & rKnots,                          // out:                                                               
    SmTArray<ULONG>  * pKnotMultiplicities = NULL,      // out:                                                               
    const SmExtent1d * pOptIvl = NULL                   // in : interval of interest, NULL=Natural Interval, default:[NULL]   
  ) const;

                             
  virtual SmStatus ConvertUVFromSTEPToNURBS(const SmPoint2d & crSTEPUV, SmPoint2d & rNURBSUV) const 
                                           { return m_pSurface->ConvertUVFromSTEPToNURBS(crSTEPUV, rNURBSUV) ; } 

  virtual SmStatus ConvertUVFromNURBSToSTEP(const SmPoint2d & crNURBSUV, SmPoint2d & rSTEPUV) const 
                                           { return m_pSurface->ConvertUVFromNURBSToSTEP(crNURBSUV, rSTEPUV) ; }

  SmStatus         ConvertDomainFromSTEPToNURBS(const SmExtent2d & crAnalDomain, SmExtent2d & rNurbDomain) const 
                                               { return m_pSurface->ConvertDomainFromSTEPToNURBS(crAnalDomain, rNurbDomain) ; }

  SmStatus         ConvertDomainFromNURBSToSTEP(const SmExtent2d & crNurbDomain, SmExtent2d & rAnalDomain) const 
                                               { return m_pSurface->ConvertDomainFromNURBSToSTEP(crNurbDomain, rAnalDomain) ; }
                             
  void      SetOwnerFlag(ULONG lOwnerFlag) ;                // eff: Set owns contained Surface and Volume flag (sets OwnedObj->m_pOwner == this)

  SmStatus  MakeOwner() ;                                   // eff: ensures contained srf and vol are owned, makes copies as needed

  SmStatus  SetBaseSurface
  (
    SmSurface *pSurface,             // in : new base surface                                                    
    SmBoolean  bInParamSpace,        // in : TRUE = cpCurve is in Volume's ParamSpace, FAlSE=Curve in InSpace    
    SmBoolean  bCopySurface=FALSE,   // in : TRUE = save copy of pSurface, FALSE = save pSurface                 
    SmBoolean  bOwnsSurface=FALSE    // in : TRUE = delete this surface when destructed, FALSE = don't           
  );

  SmStatus  SetVolume
  (
    SmVolume *pVolume,                    // in : new compounding volume                                        
    SmBoolean  bCopySurface=FALSE,        // in : TRUE = save copy of pVolume, FALSE = save pVolume             
    SmBoolean  bOwnsSurface=FALSE         // in : TRUE = delete this volume when destructed, FALSE = don't      
  );

  void      SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                      if (m_lOwnerFlag > 1 ) m_pVolume->SetContext(cpContext);
                                                      if (m_lOwnerFlag == 1 || m_lOwnerFlag == 3 ) m_pSurface->SetContext(cpContext); }

  SmBoolean OwnsSurface() const                             { return ( m_lOwnerFlag == 1 || m_lOwnerFlag == 3 ) ; }
  SmBoolean OwnsVolume()  const                             { return ( m_lOwnerFlag == 2 || m_lOwnerFlag == 3 ) ; }

  // evaluations
  virtual SmStatus EvaluatePoint (const SmPoint2d & crUV, SmPoint3d & rPoint) const;

  virtual SmStatus EvaluateNormal
  (
    const SmPoint2d & crUV,              // in : domain point to investigate                                                   
    SmBoolean         bUFromLeft,        // in : if P is on U interval boundary                                                
                                         //    : TRUE  = evaluate P in upper interval where P is on the left of the interval   
                                         //    : FALSE = evaluate P in lower interval where P is on the right of the interval  
    SmBoolean         bVFromLeft,        // in : if P is on V interval boundary - same as for U                                
    SmVector3d      & rSurfaceNormal     // out: unit-normal                                                                   
  ) const ;

  virtual SmStatus Evaluate
  (
    const SmPoint2d & crUV,         // in : param value to evaluate                                                                  
    ULONG lHighestUDeriv,           // in : number of U derivatives                                                                  
    ULONG lHighestVDeriv,           // in : number of V derivatives to compute                                                       
    SmBoolean bUFromLeft,           // in : if P is on U or V interval boundary                                                      
    SmBoolean bVFromLeft,           //    : TRUE  = evaluate P in upper interval where P is on the left of the interval              
                                    //    : FALSE = evaluate P in lower interval where P is on the right of the interval             
    SmBoolean bOnlyUpperHalf,       // in : TRUE=compute upper half of matrix only                                                   
                                    //    : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value            
                                    //    :           [Dv --]       [Dv   Duv   ---]          (the memory has to be allocated)       
                                    //    :                         [Dvv  ---   ---]                                                 
    SmVector3d *aDerivatives,       // out: matrix of evaluations values                                                             
                                    //    : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                               
                                    //    : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)    
                                    //    :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )    
                                    //    :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                       
                                    //    :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                       
                                    //    : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...] 
   SmBoolean bNonZeroTangents=TRUE, // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors             
                                    //    : FALSE= return exact tangent values                                                       
                                    //    : note: Surprisingly TRUE is the common choice because most tangent uses                   
                                    //    :       are for their direction (Binorm, SurfNorm comps), but when the                     
                                    //    :       tangent is being used for its magnitude (like an arc-length comp)                  
                                    //    :       then set this to FALSE.                                                            
                                    //    : default:[TRUE]                                                                           
   SmBoolean bDoZeroSampling=TRUE   // in : for internal use only, always set to TRUE, default:[TRUE]                                
  ) const ;

  // call Evaluate with bNonZeroTangents = FALSE
  virtual SmStatus EvaluateSimple
  (
    const SmPoint2d & crUV,    // in : same as Evaluate()     
    ULONG lHighestUDeriv,      // in : same as Evaluate()     
    ULONG lHighestVDeriv,      // in : same as Evaluate()     
    SmBoolean bUFromLeft,      // in : same as Evaluate()     
    SmBoolean bVFromLeft,      // in : same as Evaluate()     
    SmBoolean bOnlyUpperHalf,  // in : same as Evaluate()     
    SmVector3d *aDerivatives   // out: same as Evaluate()     
  ) const ; 

  virtual SmStatus STEPInversion
  (
    const SmExtent2d & crAnalUVDomain,        // in :         
    const SmPoint3d  & crPointOnSurf,         // in :         
    double             dDistanceTolerance,    // in :         
    SmPoint2d        & rdAnalUVParameter,     // out:         
    SmLocationType   & reLocation,            // out:         
    SmPoint2d        * pUVGuess = NULL        // NotUsed: in :         
  ) const;

  // surface modifications
  virtual SmStatus Reverse(SmSurfParamType eSurfParam) ;

  virtual SmStatus SplitAt
  (
    const SmContext & crContext,         // in : context for new object construction           
    double            dParam,            // in : split parameter                               
    SmSurfParamType   eSurfParam,        // in : oneof SM_SP_U = split u domain at dParam      
                                         //    :       SM_SP_V = split v domain at dParam      
    SmSurface      *& rpLeftSurface,     // out: Split surface result, Ivl=[MinParam, TgtParam]
    SmSurface      *& rpRightSurface     // out: Split surface result, Ivl=[TgtParam, MaxParam]
  ) ;   

  virtual SmStatus SwapUV() ;

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

  virtual SmStatus TrimWithDomain(SmExtent2d & crTrimDomain) ;

  // surface utilities

  // get memory used for surface but not its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,      // out: bigger size of all allocated memory in bytes   
    SmMarkType eMarkType=SM_MT_NOMARK  // in : uses without increment eMarkType value         
  ) const ; 

  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,             // in : target output stream                                 
    ULONG lDBVersionNumber          // in : database version to get proper sequence of writes    
  ) const ;                                                                       

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,              // NotUsed: in : Object type to be read                                                       
    SmDatabaseIO    & rDB,                // in : target output stream                                                         
    const SmContext & crContext,          // in : context for new object construction                                          
    SmSurface      *& rpNewSurface,       // out: NULL on input = new object allocated in this routine built from stream data  
                                          //    : NotNULL on input = pointer to an empty object to be filled by this routine   
    ULONG             lDBVersionNumber    // in : database version to get proper sequence of writes                            
  );
   
  virtual SmDisplayList * Draw
  (
    SmBoolean       bAddToUIPickList=FALSE,
    SmGfxArraySet * pOptGfxSet=NULL
  ) const;


  virtual SmDisplayList * DrawUV
  (                                                                                                                                       
    ULONG              NumBetweenU=8,                   // in : number of U IsoParameter lines between knots                                
                                                        //    : default:[8]                                                                 
    ULONG              lNumBetweenV = 8,                // in : number of V IsoParameter lines between knots                                
                                                        //    : default:[8]                                                                 
    SmBoolean          bVaryCrossHatchColor = FALSE,    // in : TRUE = Draw U Lines in ObjectColor                                          
                                                        //    :        Draw V lines in m_VaryCrossHatchColor                                
                                                        //    : FALSE= Draw both U and V Lines in ObjectColor                               
                                                        //    : default:[FALSE]                                                             
    const SmExtent2d * pOptUVDomain = NULL,             // in : UVDomain to crossHatch, NULL=use NaturalUVDomain                            
                                                        //    : default:[NULL]                                                              
    SmBoolean          bAddToUIPickList = FALSE,        // in : TRUE=Add to UI pick list, FALSE=don't                                  
                                                        //    : default:[FALSE]                                                             
    SmGfxArraySet    * pOptGfxSet = NULL                // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.  
  ) const;  

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore                                           
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       
                                              //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   
                                              //    : default:[SM_LEVEL_0]                                                                             
    SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
    SmTArray<ULONG>  * pTestRequests=NULL     // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmSrfInVolume, SmSurface, SmSrfInVolume_TYPE) ;

} ; // end class SmSrfInVolume

#endif // !__SMSRFINVOLUME_H__

