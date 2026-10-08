// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmOffsetSurface.h
* PURPOSE: Header file for Offset Surface class.
**********************************************************************/

#ifndef __SMOFFSETSURFACE_H__
#define __SMOFFSETSURFACE_H__

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

/*******************************************************************//**
PURPOSE: This object is a surface which represents an implicit 
   (Evaluator based) offset surface.  

NOTES: Please note that if you attach an offset surface to
   a face, the offset should own the surface.  

  An offset is defined by:

  Offset(uv) = ActingBase(uv) + dOffsetDistance * sNorm
  
    ActingBase = (crUV is in  pBaseSurface Domain)
                 ? m_pSurface
                 : m_pExtendedSurface

  when m_bAllowExtension == FALSE, input UV points beyond
       the BaseSurface domain are clamped to the domain and
       the extendedSurface is never used.

***********************************************************************/
class SM_EXPORT SmOffsetSurface : public SmSurface
{
protected:
  double            m_dOffsetDistance;  // Can be either positive or negative
                                        // positive offsets go toward side of
                                        // natural surface normal.
  SmBoolean         m_bAllowExtension;  // If TRUE will allow extension of surface
                                        // during evaluation.
  SmBoolean         m_bOwnsSurface;     // If TRUE the offset owns its base surface and is
                                        // responsible for deleting it.
  SmSurface       * m_pSurface;         // Base surface 
  SmSurface       * m_pExtendedSurface; // Surface that has been extended for evaluation
                                        // outside of the original domain - used by fillet package.
  SmBoolean         m_bIsPeriodicOnU;   // Indicates if the offset surface is periodic on U parameter, persistent
  SmBoolean         m_bIsPeriodicOnV;   // Indicates if the offset surface is periodic on V parameter, persistent
  ULONG             m_lSingularities;   // SM_SS_NONE or an orof: SM_SS_UMIN, SM_SS_VMIN, SM_SS_UMAX, SM_SS_VMAX, SM_SS_UNKNOWN

public:
  // Constructor
  SmOffsetSurface
  (
    double dOffsetDistance,
    SmSurface & crSurface,
    SmBoolean bOwnsSurface = FALSE,
    ULONG lSingularities = SM_SS_UNKNOWN
  );

  // empty constructor for I/O
  SmOffsetSurface(){};

  // copy constructor
  SmOffsetSurface( const SmOffsetSurface & crSurfaceToCopy );

  // copy
  virtual SmStatus Copy
  (
    const SmContext & crContext,
    SmSurface *& rpNewSurface
  ) const;

  // destructor
  virtual ~SmOffsetSurface();

  // equality operator
  virtual SmBoolean operator==( const SmSurface &crOther ) const;

  // build methods

  // create offset surface - Out Surf->Domain(s) may be trimmed but not scaled
  virtual SmStatus CreateOffsetSurface
  (
    const SmContext      & crContext,               // in : context for new obj construction                                               
    double                 dSignedOffsetDistance,   // in : offset dist, (neg val = Offset dir opposite surface normal)                    
    SmApproxTol3d          sApproxTol3d,            // NotUsed: in : Max Dist between ApproxOffsetSurface and ideal offset shape                    
    SmSurface* &           rOffsetSurface           // out: Offset Surf Approx, may be more than 1 when offsets have self-intersections    
  ) const;

  virtual SmStatus CreateMirrorSurface
  (
    const SmContext & crContext,                    // in : context for new object     
    const SmAxis2Placement & crMirrorPlane,         // in : mirror plane               
    SmSurface *& rpMirrorSurface                    // out: mirrored surface           
  ) const;

  SmStatus CreateExtendedSurface
  (
    const SmContext   & crContext,                  // in : context for new object construction                     
    double              dDist,                      // in : Distance of extension from each side                    
    SmContinuityType    eExtensionContinuity,       // in : OneOf: SM_CT_G1 - linear extension                      
                                                    //    :        SM_CT_G1R -                                      
                                                    //    :        SM_CT_G1_G2 - extension with second derivative   
                                                    //    :        SM_CT_CINFINITY - infinite continuity            
    SmSurface        *& rpExtended                  // out: the newly constructed surface (NULL on input)           
  );

  SmStatus CreateExtendedSurface
  (
    const SmContext   & crContext,                  // in : context for new object construction                     
    SmSurfParamType     eExtDirection,              // in : Either extend in SM_SP_U or SM_SP_V direction           
                                                    //    : or SM_SP_UMIN/VMIN/UMAX/VMAX/BOTH                       
    double              dDist,                      // in : Distance of extension from each side                    
    SmContinuityType    eExtensionContinuity,       // in : oneof: SM_CT_G1 - linear extension                      
                                                    //    :        SM_CT_G1R -                                      
                                                    //    :        SM_CT_G1_G2 - extension with second derivative   
                                                    //    :        SM_CT_CINFINITY - infinite continuity            
    SmSurface        *& rpExtended                  // out: the newly constructed surface (NULL on input)           
  );

  virtual SmStatus CalculateBoundingBox
   (const SmExtent2d  & crUVDomain,                         // NotUsed: in : returns whole surface bounding box
    SmExtent3d        * pNormalBox = NULL,                  // out: Axis aligned box                                                                          
    SmPseudoBox * pPseudoBox = NULL,                    // out: Non-axis aligned box                                                           
    SmPolarBox  * pPolarBox = NULL,                     // out: Surface normal vector field bounding box                                       
    const SmPseudoBox * pOptPseudoBasisGuess = NULL,        // NotUsed: in : guess for basis vectors - used unless another better orientation is found NULL to ignore  
    const SmPolarBox  * pOptPolarBasisGuess = NULL,         // NotUsed: in : guess for basis vectors - used unless another better orientation is found NULL to ignore  
    SmBoolean           bExpandPosBoxesByZoneTol3d = TRUE)  // in : TRUE = returned Normal & Pseudo BBoxes = BBox->ExpandAbsoluate(ZoneTol3d)
   const ;                                                  //    : FALSE= returned Normal & Pseudo BBoxes = BBox with no expansion 
                                                            //      default:[TRUE] = previous behavior

  double                     GetOffsetDistance()        const { return m_dOffsetDistance; }
  SmSurface                * GetBaseSurface()           const { return m_pSurface; }
  virtual SmBSplineSurface * GetRootSurface()           const { return m_pSurface->GetRootSurface(); }
  SmSurface                * GetExtendedBaseSurface()   const { return m_pExtendedSurface; }
  virtual SmExtent2d         GetMaxAnalyticDomain()     const { return m_pSurface->GetMaxAnalyticDomain(); }
  virtual ULONG              GetDegree(SmSurfParamType) const { return 3; }
  ULONG                      GetSingularities()         const { return m_lSingularities ; }

  virtual SmStatus GetKnots
  (
    SmSurfParamType    eSurfParam,                  // in :                                                               
    SmTArray<double> & rKnots,                      // out:                                                               
    SmTArray<ULONG>  * pKnotMultiplicities = NULL,  // out:                                                               
    const SmExtent1d * pOptIvl = NULL               // in : interval of interest, NULL=Natural Interval, default:[NULL]   
  ) const;

  void SetOffsetDistance( double dOffsetDistance ) { m_dOffsetDistance = dOffsetDistance; }
  void SetAllowExtension( SmBoolean bAllowExtension ) { m_bAllowExtension = bAllowExtension; }

  void SetBaseSurface( SmSurface *pSurface, SmBoolean bOwnsSurface = FALSE )
  {
    if(m_bOwnsSurface && m_pSurface)
    {
      delete m_pSurface; m_pSurface = NULL;
    }
    m_pSurface = pSurface;
    m_bOwnsSurface = bOwnsSurface;
  }

  void SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                 if (m_bOwnsSurface && m_pSurface ) m_pSurface->SetContext(cpContext);
                                                 if (m_pExtendedSurface ) m_pExtendedSurface->SetContext(cpContext); }

  virtual SmStatus EvaluatePoint( const SmPoint2d & crUV, SmPoint3d & rPoint ) const;

  virtual SmStatus EvaluateNormal
  (
    const  SmPoint2d & crUV,     // in : Domain point to evaluate                                                      
    SmBoolean bUFromLeft,        // in : if P is on U interval boundary                                                
                                 //    : TRUE  = evaluate P in upper interval where P is on the left of the interval   
                                 //    : FALSE = evaluate P in lower interval where P is on the right of the interval  
    SmBoolean bVFromLeft,        // in : if P is on V interval boundary - same as for U                                
    SmVector3d & rSurfaceNormal  // out: unit-normal
  ) const;

  virtual SmStatus Evaluate
  (
    const SmPoint2d & crUV,           // in : param value to evaluate                                                                  
    ULONG lHighestUDeriv,             // in : number of U derivatives                                                                  
    ULONG lHighestVDeriv,             // in : number of V derivatives to compute                                                       
    SmBoolean bUFromLeft,             // in : if P is on U interval boundary                                                           
                                      //    : TRUE  = evaluate P in upper interval where P is on the left of the interval              
                                      //    : FALSE = evaluate P in lower interval where P is on the right of the interval             
    SmBoolean bVFromLeft,             // in : if P is on V interval boundary                                                           
                                      //    : TRUE  = evaluate P in upper interval where P is on the left of the interval              
                                      //    : FALSE = evaluate P in lower interval where P is on the right of the interval             
    SmBoolean bOnlyUpperHalf,         // in : TRUE=compute upper half of matrix only                                                   
                                      //    : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value            
                                      //    :           [Dv --]       [Dv   Duv   ---]          (the memory has to be allocated)       
                                      //    :                         [Dvv  ---   ---]                                                 
    SmVector3d *aDerivatives,         // out: matrix of evaluations values                                                             
                                      //    : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                               
                                      //    : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)    
                                      //    :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )    
                                      //    :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                       
                                      //    :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                       
                                      //    : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...] 
    SmBoolean bNonZeroTangents = TRUE,// in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors             
                                      //    : FALSE= return exact tangent values                                                       
                                      //    : note: Surprisingly TRUE is the common choice because most tangent uses                   
                                      //    :       are for their direction (Binorm, SurfNorm comps), but when the                     
                                      //    :       tangent is being used for its magnitude (like an arc-length comp)                  
                                      //    :       then set this to FALSE.                                                            
                                      //    : default:[TRUE]                                                                           
    SmBoolean bDoZeroSampling = TRUE  // in : for internal use only, always set to TRUE, default:[TRUE]                                
  ) const;

  virtual SmStatus EvaluateSimple
  (
    const SmPoint2d & crUV,           // in : target surface point                                                                     
    ULONG lHighestUDeriv,             // in : Requested highest U derivative                                                           
    ULONG lHighestVDeriv,             // in : Requested highest V derivative                                                           
    SmBoolean bUFromLeft,             // in : if P is on U interval boundary                                                           
                                      //    : TRUE  = evaluate P in upper interval where P is on the left of the interval              
                                      //    : FALSE = evaluate P in lower interval where P is on the right of the interval             
    SmBoolean bVFromLeft,             // in : if P is on V interval boundary                                                           
                                      //    : TRUE  = evaluate P in upper interval where P is on the left of the interval              
                                      //    : FALSE = evaluate P in lower interval where P is on the right of the interval             
    SmBoolean bOnlyUpperHalf,         // in : TRUE=compute upper half of matrix only                                                   
                                      //    : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value            
                                      //    :           [Dv --]       [Dv   Duv   ---]                                                 
                                      //    :                         [Dvv  ---   ---]                                                 
    SmVector3d *aDerivatives          // out: matrix of evaluations values                                                             
                                      //    : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                               
                                      //    : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]                                       
                                      //    :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]                                       
                                      //    :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                       
                                      //    :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                       
                                      //    : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...] 
  ) const;

  virtual SmExtent2d GetNaturalUVDomain() const;

  virtual SmStatus Reverse( SmSurfParamType eSurfParam );

  virtual SmStatus SplitAt
  (
    const SmContext & crContext,      // in : context for new object construction           
    double dParam,                    // in : split parameter                               
    SmSurfParamType eSurfParam,       // in : oneof SM_SP_U = split u domain at dParam      
                                      //    :       SM_SP_V = split v domain at dParam      
    SmSurface *& rpLeftSurface,       // out: Split surface result, Ivl=[MinParam, TgtParam]
    SmSurface *& rpRightSurface       // out: Split surface result, Ivl=[TgtParam, MaxParam]
  );

  virtual SmStatus STEPInversion
  (
    const SmExtent2d & crAnalUVDomain,        // in :          
    const SmPoint3d  & crPointOnSurf,         // in :          
    double             dDistanceTolerance,    // in :          
    SmPoint2d        & rdAnalUVParameter,     // in :          
    SmLocationType   & reLocation,            // in :          
    SmPoint2d        * pUVGuess = NULL        // in :          
  ) const;

  virtual SmStatus SwapUV();

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

  virtual SmStatus TrimWithDomain( SmExtent2d & crTrimInterval );

  // get memory used for curve but not its attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,      // out: bigger size of all allocated memory in bytes  
    SmMarkType eMarkType = SM_MT_NOMARK  // in : uses without increment eMarkType value      
  ) const;

  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,                 // in : target output stream                                   
    ULONG          lDBVersionNumber     // in : database version to get proper sequence of writes      
  ) const;

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,              // NotUsed: in : Object type to be read                                                          
    SmDatabaseIO    & rDB,                // in : target output stream                                                            
    const SmContext & crContext,          // in : context for new object construction                                             
    SmSurface      *& rpNewSurface,       // out: NULL on input = new object allocated in this routine built from stream data     
                                          //    : NotNULL on input = pointer to an empty object to be filled by this routine      
    ULONG             lDBVersionNumber    // in : database version to get proper sequence of writes                               
  );

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON( SmOffsetSurface, SmSurface, SmOffsetSurface_TYPE );

  virtual SmDisplayList * Draw
  (
    SmBoolean bAddToUIPickList = FALSE,      // in : TRUE = Add this surface to UI pick interface for debugging                                            
    SmGfxArraySet * pOptGfxSet = NULL        // in : When given output GfxVertexArrays not GL calls.// in : When given output GfxVertexArrays not GL calls. 
  ) const;

  virtual SmDisplayList * DrawUV
  (
    ULONG             lNumBetweenU = 8,                // in : draw surface and crossHatch lines                        
    ULONG             lNumBetweenV = 8,                // in : special case: NumBetweenU == NumBetweenV == 999,         
    SmBoolean         bVaryCrossHatchColor = FALSE,    // in :               Draw as 4 corners connected by lines       
    const SmExtent2d *pOptUVDomain = NULL,             // in : UVDomain to crossHatch, NULL=use NaturalUVDomain         
    SmBoolean         bAddToUIPickList = FALSE,        // in : TRUE=Add to UI pick list, FALSE=don't               
    SmGfxArraySet    *pOptGfxSet = NULL                // i/o: When given output GfxVertexArrays not GL calls.      
  ) const;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList = NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore                                          
    SmAssertTestLevel  eTestLevel = SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       
                                                //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   
                                                //    : default:[SM_LEVEL_0]                                                                             
    SmAssertWalking    eWalkTree = SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
    SmTArray<ULONG>  * pTestRequests = NULL     // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 
  ) const;

private:

  // internal methods

  // Common functionality of Evaluate and EvaluateSimple
  SmStatus EvaluatePlainOrSimple
  (
    SmBoolean bDoEvalSimple,   // in : TRUE=EvaluateSimple, FALSE=Evaluate, adjust zero tangents.                       
    const SmPoint2d & crUV,    // in : target surface point                                                             
    ULONG lHighestUDeriv,      // in : Requested highest U derivative                                                   
    ULONG lHighestVDeriv,      // in : Requested highest V derivative                                                   
    SmBoolean bUFromLeft,      // in : if P is on U interval boundary                                                   
                               //    : TRUE  = evaluate P in upper interval where P is on the left of the interval      
                               //    : FALSE = evaluate P in lower interval where P is on the right of the interval     
    SmBoolean bVFromLeft,      // in : if P is on V interval boundary                                                   
                               //    : TRUE  = evaluate P in upper interval where P is on the left of the interval      
                               //    : FALSE = evaluate P in lower interval where P is on the right of the interval     
    SmBoolean bOnlyUpperHalf,  // in : TRUE=compute upper half of matrix only                                           
                               //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value        
                               //    :       [Dv --]       [Dv   Duv   ---]                                             
                               //    :                     [Dvv  ---   ---]                                             
    SmVector3d *aDerivatives   // out: matrix of evaluations values                                                     
                               //    :  sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                      
                               //    :  2d organized: [D     Du     Duu     Duuu     ]                                  
                               //    :                [Dv    Duv    Duuv    ....     ]                                  
                               //    :                [Dvv   Duvv   ....    ....     ]                                  
                               //    :                [Dvvv  ....   ....    ....     ]                                  
                               //    :  1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv,,.. Duu, Duuv,...]             
  ) const;


}; // end class SmOffsetSurface

#endif // !__SMOFFSETSURFACE_H__
