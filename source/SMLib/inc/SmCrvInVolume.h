// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCrvInVolume.h
* PURPOSE: Header file for compound 3D curve defined as a 3D
*    parameter space curve and the volume through which it is projected.
**********************************************************************/

#ifndef __SMCRVINVOLUME_H__
#define __SMCRVINVOLUME_H__

#ifndef __SMCURVE_H__
#include <SmCurve.h>
#endif

class SmVolume ;

/*******************************************************************//**
PURPOSE: The SmCrvInVolume class defines a 3d Curve which is the
    compound projection of a 3d SmCurve through a SmVolume.

NOTES: 1. Projects the Curve from the Volume's InSpace or ParamSpace to its OutSpace
       2. When Volume is a compound Volume, projects the curve
           through the compound sequence of Volume Inspaces to Outspaces
           ending in the last compounded volume's outspace. 
***********************************************************************/
class SM_EXPORT SmCrvInVolume : public SmCurve
{
protected:
  SmCurve                  * m_pCurve = NULL ;          // Parameter space curve  (When owned, Owner set to this SmCrvInVolume)
  SmBoolean                  m_bInParamSpace = UNSURE ; // TRUE = m_pCurve is in m_pVolume's ParamSpace
                                                        // FALSE= m_pCurve is in m_pVolume's InSpace
  SmVolume                 * m_pVolume = NULL ;         // Base volume            (When owned, Owner set to this SmCrvInVolume)
  SmBoolean                  m_bNeedBreaks = UNSURE ;   // TRUE = m_sBreaks not yet computed, FALSE = m_sBreaks is computed
  SmTArray<double>           m_sBreaks ;                // List of curve discontinuities (curve knots + Curve/VolumeDiscontinuity intersections)
  SmTArray<SmContinuityType> m_sConts ;                 // associated list of discontinuities at each break
  ULONG                      m_lOwnerFlag = 0 ;         //   0 = deletes nothing when destructed     (don't change numbers - used as BitArray)          
                                                        //   1 = deletes only curve when destructed 
                                                        //   2 = deletes only volume when destructed 
                                                        //   3 = deletes both curve and volume when destructed
                                                        //   default:[0]
                                                        // Note: anything owned by SmCrvInVolume is copied during 
                                                        //       construction and deleted during destruction
                                                        //       and has its m_pOwner pointer set to this SmCrvInVolume object.
                                                        //       Never let anything owned by this SmCrvInVolume have two owners.
public:                              
  // constructor                                                  
  SmCrvInVolume(SmCurve          & rCurve,              ///< [in] : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this    <br>
                SmBoolean          bInParamSpace,       ///< [in] : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace       <br>
                SmVolume         & rVolume,             ///< [in] : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this   <br>
                ULONG              lCopyFlag = 0,       ///< [in] : 0 = saves curve and volume without copying                        <br>
                                                        ///<        1 = copy curve and save volume orig                               <br>
                                                        ///<        2 = copy volume and save curve orig                               <br>
                                                        ///<        3 = copy both curve and volume                                    <br>
                ULONG              lOwnerFlag = 0,      ///< [in] : 0 = deletes nothing when destructed                               <br>
                                                        ///<        1 = delete curve but not volume when destructed                   <br>
                                                        ///<        2 = delete volume but not curve when destructed                   <br>
                                                        ///<        3 = delete both curve and volume when destructed                  <br>
                const SmContext  * cpContext = NULL) ;  ///< [in] : req for new stack objs, opt for new heap objs                     <br>

  // empty constructor for I/O
  SmCrvInVolume() : SmCurve(3) { m_pCurve        = NULL ; 
                                 m_bInParamSpace = TRUE ;      
                                 m_pVolume       = NULL ;     
                                 m_bNeedBreaks   = TRUE ;
                                 m_lOwnerFlag    = 3 ; 
                               }
  
  // copy constructor
  SmCrvInVolume(const SmCrvInVolume & crCurveToCopy) ;
  
  // copy
  virtual SmStatus Copy(const SmContext & crContext,        ///< [in] :     <br>
                        SmCurve         *& crNewCurve)      ///< [in] :     <br>
                       const ;
  
  // create a mirrored copy
  virtual SmStatus CreateMirrorCurve(const SmContext        & crContext,         ///< [in] : context for new object    <br>
                                     const SmAxis2Placement & crMirrorPlane,     ///< [in] : mirror plane              <br>
                                     SmCurve               *& rpMirrorCurve)     ///< [out]: mirrored curve            <br>
                                    /*const*/;
  
  // destructor
  virtual ~SmCrvInVolume() ;
  
  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const ;
  
  // make exact BSpline equivalent curve mapping m_pCurve to the volume's final OutSpace if possible
  virtual SmStatus MakeExactBSplineIfPossible(SmBSplineCurve *& rpNewBSplineCurve) const ; 
  
  // maps InSpace Curve to Outspace
  virtual SmStatus Evaluate(double     dParameter,              ///< [in] : tgt param                                                                        <br>
                            ULONG      lNumDerivatives,         ///< [in] : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .                                         <br>
                            SmBoolean  bFromLeft,               ///< [in] : if P is on interval boundary                                                     <br>
                                                                ///<        TRUE  = evaluate P in upper interval where P is on the left of the interval      <br>
                                                                ///<        FALSE = evaluate P in lower interval where P is on the right of the interval     <br>
                            SmVector3d aPointAndDerivatives[],  ///< [out]: (pos, tang, 2nd, ...) sized:[lNumDerivatives+1]                                  <br>
                            SmBoolean  bNonZeroTangents=TRUE)   ///< [in] : TRUE = replace zero tangent vectors with properly oriented tol sized vectors     <br>
                                                                ///<        FALSE= return exact tangent values                                               <br>
                                                                ///<        note: Surprisingly TRUE is the common choice because most tangent uses are for   <br>
                                                                ///<        their direction (Binorm, SurfNorm comps), but when the tangent is being          <br>
                                                                ///<        used for its magnitude (like an arc-length comp) then set this to FALSE.         <br>
                           const;
  
  // evaluate volume(curve(dParameter)) for position
  virtual SmStatus EvaluatePoint(double dParameter, SmPoint3d & rPoint) const ;
  
  // evaluate Volume values for given Curve parameter value
  SmStatus EvaluateVolume(double      dParameter,             ///< [in] : tgt param                                                                                        <br>
                          ULONG       lHighestDeriv,          ///< [in] : 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                                         <br>
                          SmBoolean   bFromLeft,              ///< [in] : if P is on interval boundary                                                                     <br>
                                                              ///<        TRUE  = evaluate P in upper interval where P is on the left of the interval                      <br>
                                                              ///<        FALSE = evaluate P in lower interval where P is on the right of the interval                     <br>
                          SmVector3d *aDerivatives,           ///< [out]: matrix of OutSpace evaluations values                                                            <br>
                                                              ///<        sized:[n+1][n+1][n+1], where n=lHighesDeriv                                                      <br>
                                                              ///<        indexing:[1dIndex = xIn*(n+1)*(n+1)+yIn*(n+1)+zIn], where xIn,yIn,Zin=number of partial derivs   <br>
                                                              ///<        3d organized: D[XdirCnt][YDirCnt][ZDirCnt]                                                       <br>
                                                              ///<        1d organized: for lHighestDeriv from 0 to 3,                                                     <br>
                                                              ///<              0,  sized:[1]    order:[D]                                                                 <br>
                                                              ///<              1,  sized:[8]    order:[D    DzIn DyIn ----                                                <br>
                                                              ///<                                      DxIn ---- ---- ----]                                               <br>
                                                              ///<              2,  sized:[27]   order:[D     DzIn  DzzIn DyIn  DyzIn ----- DyyIn ----- -----              <br>
                                                              ///<                                      DxIn  DxzIn ----- DxyIn ----- ----- ----- ----- -----              <br>
                                                              ///<                                      DxxIn ----- ----- ----- ----- ----- ----- ----- -----]             <br>
                                                              ///<              3,  sized:[64]   order: 1dIndex = u*16+v*4+w                                               <br>
                                                              ///<                   [D     DzIn   DzzIn   DzzzIn  DyIn   DyzIn  DyzzIn ...    DyyIn  DyzIn                <br>
                                                              ///<                    ...   ...    DyyyIn  ...     ...    ...    DxIn   DxzIn  DxzzIn ...                  <br>
                                                              ///<                    DxyIn DxyzIn ...     ...     DxyyIn ...    ...    ...    ...    ...                  <br>
                                                              ///<                    ...   ...    DxxIn   DxxzIn  ...    ...    DxxyIn ...    ...    ...                  <br>
                                                              ///<                    ...   ...    ...     ...     ...    ...    ...    ...    DxxxIn ...                  <br>
                                                              ///<                    ...   ...    ...     ...     ...    ...    ...    ...    ....   ...                  <br>
                                                              ///<                    ...   ...    ...     ...     ...    ...    ...    ...    ....   ...                  <br>
                                                              ///<                    ...   ...    ...     ...     ...    ...    ...    ...    ....   ...                  <br>
                                                              ///<                    ...   ... ]                                                                          <br>
                          SmVector3d *pOptBaseCurvePV=NULL,   ///< [out]: Curve position, 1stDeriv, 2ndDeriv, 3rdDeriv                                                     <br>
                                                              ///<        NULL to ignore, Default:[NULL], else sized:[lHighestDeriv]                                       <br>
                          SmBoolean   bNonZeroTangents=TRUE)  ///< [in] : TRUE = replace zero tangent vectors with properly oriented tol sized vectors                     <br>
                         const ;                              ///<        FALSE= return exact tangent values                                                               <br>
                                                              ///<        note: Surprisingly TRUE is the common choice because most tangent uses                           <br>
                                                              ///<              are for their direction (Binorm, SurfNorm comps), but when the                             <br>
                                                              ///<              tangent is being used for its magnitude (like an arc-length comp)                          <br>
                                                              ///<              then set this to FALSE.                                                                    <br>

  // simple data access
  virtual SmBSplineCurve * GetRootCurve()          const { return(m_pCurve->GetRootCurve()) ; }   
  SmCurve                * GetCurve()              const { return m_pCurve ; }
  virtual SmExtent1d       GetNaturalInterval()    const { return m_pCurve->GetNaturalInterval() ; }
  SmBoolean                GetInParamSpace()       const { return m_bInParamSpace ; }
  SmVolume               * GetVolume()             const { return m_pVolume ; }
  
  // report all discontinuities due to the propogation of both the curve and volume discontinuities as knots
  //  on a degree 3 volume with multiplicity 2
  virtual ULONG            GetNumberNaturalKnots() const ;
  
  virtual SmStatus         GetKnots(SmTArray<double> & rKnots,                 ///< [out]:        <br>
                                    SmTArray<ULONG>  * pOptMults = NULL,       ///< [out]:        <br>
                                    const SmExtent1d * pOptIvl = NULL)         ///< [in] :        <br>
                                   const;
                           
  // an internal method used by GetKnots() to generate "effective" knot multiplicities from cached discontinuity data
  SmStatus GenerateMults(SmTArray<double> & rKnots,      ///< [in] : ascending param array of discontinuities of interest (expected to be in m_vKnots   <br>
                         SmTArray<ULONG>  & rMults)      ///< [out]: associated array of "effective" multiplicities.                                    <br>
                        const ;
  
  // report all discontinuities due to the propogation of both the curve and volume discontinuities as SM_CT_C1
  virtual SmStatus CalculateContinuities(SmContinuityType           & reMinContinuityInCurve,    ///< [out]: min continuity of all internal knots       <br>
                                         SmTArray<SmContinuityType> & rContinuitiesAtKnots,      ///< [out]: continuity at every knot value for curve   <br>
                                         double dContinuityAngleTol = SM_CONTINUITY_ANGLE)       ///< [in] :                                            <br>
                                        const ;
  
  SmStatus  SetBaseCurve(SmCurve  * cpCurve,                 ///< [in] : new base curve                                                      <br>
                         SmBoolean  bInParamSpace,           ///< [in] : TRUE = cpCurve is in Volume's ParamSpace, FAlSE=Curve in InSpace    <br>
                         SmBoolean  bCopyCurve=FALSE,        ///< [in] : TRUE = save copy of pCurve, FALSE = save pCurve                     <br>
                         SmBoolean  bOwnsCurve=FALSE) ;      ///< [in] : TRUE = delete this Curve when destructed, FALSE = don't             <br>
  
  SmStatus  SetVolume(SmVolume *pVolume,                     ///< [in] : new compounding volume                                    <br>
                     SmBoolean  bCopyVolume=FALSE,           ///< [in] : TRUE = save copy of pVolume, FALSE = save pVolume         <br>
                     SmBoolean  bOwnsVolume=FALSE) ;         ///< [in] : TRUE = delete this volume when destructed, FALSE = don't  <br>
  
  void      SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                      if (m_lOwnerFlag > 1 ) m_pVolume->SetContext(cpContext);
                                                      if (m_lOwnerFlag == 1 || m_lOwnerFlag == 3 ) m_pCurve->SetContext(cpContext); }

  void              SetOwnerFlag(ULONG lOwnerFlag) ;         // eff: Set owns contained Surface and Volume flag (sets OwnedObj->m_pOwner == this)
  SmStatus          MakeOwner() ;                            // eff: ensures contained srf and vol are owned, makes copies as needed
  
  SmBoolean         OwnsCurve()      const                   { return ( m_lOwnerFlag == 1 || m_lOwnerFlag == 3) ; }
  SmBoolean         OwnsVolume()     const                   { return ( m_lOwnerFlag == 2 || m_lOwnerFlag == 3) ; }
  virtual SmBoolean IsAnalytic()     const                   { return FALSE ; }
  virtual SmBoolean IsBounded()      const                   { return m_pCurve->IsBounded() ; } // TRUE=finite (FALSE=infinite) parameter range
  
  // Change NURB parameterization of curve to given extent range - no Analytic-STEP range change
  virtual SmStatus EditParameterization(const SmExtent1d & crNewParameterization,     ///< [in] : new parameter range for curve                                            <br>
                                        SmBoolean          bNotify = TRUE) ;          ///< [in] : TRUE  = make notify calls (previous behavior) FALSE = Skip notify call   <br>
  
  // reverse curve parameterization while preserving its NaturalInterval range
  // e.g. NewPoint(sNatIvl.Min) == OldPoint(sNatIvl.Max) with negated tangents.
  virtual SmStatus ReverseParameterization(const SmExtent1d & crOldInterval,   ///< [in] : interval of interest, may be a subset of natural interval   <br>
                                           SmExtent1d       & rNewInterval) ;  ///< [out]: new domain for crOldInterval on modified curve              <br>
  
  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL) ; // in : optional scaling about current origin point before RotateNMove
                                                                         //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                                                         //      other geom types only support isoptropic scaling

  virtual SmStatus Trim(SmExtent1d & crTrimInterval,          ///< [i/o] : desired new Trim Ivl - can be snapped by tol to existing knots <br>
                        SmBoolean    bNotify=TRUE,            ///< [in] : internal use: use default value      <br>
                        SmBoolean    bSkipDebugCheck=FALSE) ; ///< [in] : internal use: use default value      <br>
  
  // get Curve Memory size - not its attributes
  virtual ULONG GetMemoryUsed(ULONG    & rlMemoryAllocated,        ///< [out]: bigger size of all allocated memory in bytes    <br>
                              SmMarkType eMarkType=SM_MT_NOMARK)   ///< [in] : uses without increment eMarkType value          <br>
                             const ; 
  
  virtual SmStatus WriteToDB(SmDatabaseIO & rDB,             ///< [in] : target output stream                                 <br>
                             ULONG lDBVersionNumber)         ///< [in] : database version to get proper sequence of writes    <br>
                            const ; 
  
  static  SmStatus ReadFromDB(SM_TYPE           lType,                ///< NotUsed: [in] : Object type to be read                                                              <br>
                               SmDatabaseIO    & rDB,                 ///< [in] : target output stream                                                                <br>
                               ULONG             lDim,                ///< NotUsed: [in] : curve image space dim, 2 or 3                                                       <br>                             
                               const SmContext & crContext,           ///< [in] : context for new object construction                                                 <br>
                               SmCurve         *&rpNewCurve,          ///< [out]: NULL on input = new object allocated in this routine built from stream data         <br>
                                                                      ///<        NotNULL on input = pointer to an empty object to be filled by this routine          <br>
                               ULONG             lDBVersionNumber) ;  ///< [in] : database version to get proper sequence of writes                                   <br>
  
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                             <br>
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                         <br>
                                                                          ///<        SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                     <br>
                                SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't      <br>
                                SmTArray<ULONG>  * pTestRequests=NULL)    ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]   <br>
                               const ;
  
  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
  
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmCrvInVolume,SmCurve,SmCrvInVolume_TYPE);
  
} ; // end class SmCrvInVolume

#endif // !__SMCRVINVOLUME_H__



