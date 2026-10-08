// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmProjectedCurve.cpp
* PURPOSE: Source file for SmProjectedCurve methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmProjectedCurve.h>
#include <SmAxis2Placement.h>
#include <SmDatabaseIO.h>
#include <SmAssertArray.h>

#ifdef SM_DEBUG_CODE
  #include <SmBrep.h>
  #include <SmEdge.h>
// Remove Composites
// #include <SmCEdge.h>
  #include <SmFace.h>
  #include <SmEdgeuse.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: The SmProjectedCurve class defines a projected image of a 3D
    curve onto a view-plane. Currently, only parallel-projection is
    implemented.

NOTES:
***********************************************************************/

/*******************************************************************//**
PURPOSE: Constructor for ProjectedCurve objects.  It loads values
    and checks to make sure that curve is valid.

NOTES: default constructor (also empty constructor for I/O)
***********************************************************************/
SmProjectedCurve::SmProjectedCurve
 (const SmCurve    * cpCurve,        // in : Curve being projected
  const SmPoint3d  * cpProjPoint,    // in : parallel: pt on view plane,  rot: pt on rot axis,      perspective: pt on view plane
  const SmVector3d * cpProjVec,      // in : parallel: view plane normal, rot: pt on rot axis,      perspective: view plane unitNormal
  const SmVector3d * cpAuxData,      // in : parallel: not used,          rot: XAxis of proj plane, perspective: eye point
  SmProjectionType    eProjType,     // in : oneof SM_PT_PARALLEL, SM_PT_PERSPECTIVE, SM_PT_ROTATION
  SmBoolean           bOwnsCurve,    // in : TRUE = delete m_cpCurve in destructor, FALSE = don't
  const SmContext  * cpContext)      // in : must be given for automatic variables, optional for
                                     //      stack variables built with overloaded new. default:[NULL]

: SmCurve(3),
  m_cpCurve(cpCurve),
  m_eProjType(eProjType),
  m_bOwnsCurve(bOwnsCurve)
{
  if(cpProjPoint) { m_sProjPoint = *cpProjPoint; }
  if(cpProjVec)   { m_sProjVec   = *cpProjVec; }
  if(cpAuxData)   { m_sAuxData   = *cpAuxData; }
  if(cpProjVec != NULL)
    { MakeOrthoNormal() ; }

  // when passed a context - use it
  if(cpContext) { SM_ASSERT(   GetContext() == NULL
                            || GetContext() == cpContext) ;
                  SetContext(cpContext) ;
                }

} // end SmProjectedCurve::SmProjectedCurve constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmProjectedCurve object.

NOTES: The constructed object will make deep copies of the target curve's
  m_cpCurve when ever the target curve being copied owns its own m_cpCurve.
  When the target curve just has an unowned reference to the curve
  then an unowned reference will be placed in the constructed object.
***********************************************************************/
SmProjectedCurve::SmProjectedCurve
  (const SmProjectedCurve & crCurveToCopy)
 : SmCurve(crCurveToCopy),
   m_cpCurve(crCurveToCopy.m_cpCurve),
   m_sProjPoint(crCurveToCopy.m_sProjPoint),
   m_sProjVec(crCurveToCopy.m_sProjVec),
   m_sAuxData(crCurveToCopy.m_sAuxData),
   m_eProjType(crCurveToCopy.m_eProjType),
   m_bOwnsCurve(crCurveToCopy.m_bOwnsCurve)
{
  // copy Curve when its owned by this object
  if (m_bOwnsCurve)
    {
      const SmContext * pContext = GetContext();
      if (pContext == NULL)
        { SE(SM_ERR); return; }

      // copy the Curve
      SmCurve *pCopyCurve = NULL ;
      crCurveToCopy.m_cpCurve->Copy(*pContext, pCopyCurve) ;
      m_cpCurve = pCopyCurve ;
    }

} // end SmProjectedCurve::SmProjectedCurve copy constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmProjectedCurve

NOTES: Call base equivalence to check type and then check
       members for equivalence
***********************************************************************/
SmBoolean SmProjectedCurve::operator==
  (const SmCurve& crOther)
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmCurve::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmProjectedCurve &rOther = (SmProjectedCurve &)crOther ;

      // check equivalence of these objects
      bRtn = (   (   (    m_cpCurve == rOther.m_cpCurve)
                  || (    m_cpCurve == NULL && rOther.m_cpCurve == NULL)
                  || (    m_cpCurve != NULL && rOther.m_cpCurve != NULL
                      && *m_cpCurve == *rOther.m_cpCurve))
              && m_sProjPoint  == rOther.m_sProjPoint
              && m_sProjVec    == rOther.m_sProjVec
              && m_sAuxData    == rOther.m_sAuxData
              && m_eProjType   == rOther.m_eProjType) ;
    }

  // all done
  return bRtn ;

} // end SmProjectedCurve::operator==

/*******************************************************************//**
PURPOSE: Copy a projected curve

NOTES:
***********************************************************************/
SmStatus SmProjectedCurve::Copy
  (const SmContext & crContext,
   SmCurve        *& rpNewCurve)
  const
{
  rpNewCurve = new (crContext) SmProjectedCurve(*this);
  NER(rpNewCurve);
  return SM_SUCCESS;

} // end SmProjectedCurve::Copy

/*******************************************************************//**
PURPOSE: Set canonical data fields of projected curve.

NOTES:
***********************************************************************/
SmProjectedCurve::~SmProjectedCurve()
{
  if(m_bOwnsCurve && m_cpCurve) { delete m_cpCurve ; m_cpCurve = NULL ; }
  m_eProjType = SM_PT_UNKNOWN ;

} // end SmProjectedCurve::~SmProjectedCurve destructor

/*******************************************************************//**
PURPOSE: Set canonical data fields of projected curve.

NOTES: Returns SM_ERR     when ProjVec is uninitialized or zero length
               SM_ERR     when SM_PT_ROTATION and AuxData is uninitialized or zero length
       else returns SM_SUCCESS
***********************************************************************/
SmStatus SmProjectedCurve::SetCanonical
 (const SmCurve    & crCurve,         // in : Curve being projected
  const SmPoint3d  & crProjPoint,     // in : parallel: pt on view plane,  rot: pt on rot axis,      perspective: pt on view plane
  const SmVector3d & crProjVec,       // in : parallel: view plane normal, rot: pt on rot axis,      perspective: view plane unitNormal
  const SmVector3d & crAuxData,       // in : parallel: not used,          rot: XAxis of proj plane, perspective: eye point
  SmProjectionType   eProjType,       // in : oneof SM_PT_PARALLEL, SM_PT_PERSPECTIVE, SM_PT_ROTATION
  SmBoolean          bOwnsCurve,      // in : TRUE = delete m_cpCurve in destructor, FALSE = don't, default:[FALSE]
  const SmContext  * cpContext)       // NotUsed: in : must be given for automatic variables, optional for
                                      //      stack variables built with overloaded new.
{
  SM_REF1(cpContext) ;
  if ( !(    m_cpCurve    == &crCurve
          && m_sProjPoint == crProjPoint
          && m_sProjVec   == crProjVec
          && m_sAuxData   == crAuxData
          && m_eProjType  == eProjType
          && m_bOwnsCurve == bOwnsCurve ) )
  { Notify( SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER( this ), NULL ); }

  m_cpCurve     = &crCurve;
  m_sProjPoint  = crProjPoint;
  m_sProjVec    = crProjVec;
  m_sAuxData    = crAuxData;
  m_eProjType   = eProjType;
  m_bOwnsCurve  = bOwnsCurve;

  // unitize and orthoganlize ProjVec and AuxData
  return(MakeOrthoNormal()) ;

  // when passed a context - use it
  // if(cpContext)
  //   { SetContext(cpContext) ; }
  // 
  // SM_ASSERT(GetContext() != NULL) ;

} // end SmProjectedCurve::SetCanonical

/*******************************************************************//**
PURPOSE: unitize and orthoganlize ProjVec and AuxData

NOTES:
***********************************************************************/
SmStatus SmProjectedCurve::MakeOrthoNormal()
{
  // when ProjVec is nonZero - unitize it
  SmBoolean bProjInit = m_sProjVec.IsInitialized() ;
  SmBoolean bProjZero = m_sProjVec.IsZero() ;
  if(bProjInit && bProjZero) { m_sProjVec.Unitize() ; }
  else                       { return( SM_ERR ) ; }

  // for rotations - make the sAuxData vector orthonormal to the ProjVec
  if(m_eProjType == SM_PT_ROTATION)
    {
      // when xAuxData is nonZero - make orthonormal to ProjVec
      SmBoolean bAuxInit = m_sAuxData.IsInitialized() ;
      SmBoolean bAuxZero = m_sAuxData.IsZero() ;
      if(bAuxInit && bAuxZero) { m_sProjVec.NormalInPlane(m_sAuxData) ;
                                 m_sAuxData.Unitize() ;
                               }
      else                     { return( SM_ERR ) ; }
    } // end ProjType is SM_PT_ROTATION check

  // all done
  return(SM_SUCCESS) ;

} // end SmProjectedCurve::MakeOrthoNormal

/*******************************************************************//**
PURPOSE: Edit the parameterization of this curve.

NOTES:
***********************************************************************/
SmStatus SmProjectedCurve::EditParameterization
 (const SmExtent1d & crNewParameterization, // in : new parameter range for curve
  SmBoolean          bNotify)               // in : TRUE  = make notify calls (previous behavior)
                                            //      FALSE = Skip notify call
                                            //      default:[TRUE]
{
  // Don't own curve can not reverse parameterization
  if (m_bOwnsCurve == 0)
    { SER(SM_ERR) ; }

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // pass the call along to m_cpCurve
  SER(((SmCurve *)m_cpCurve)->EditParameterization(crNewParameterization, bNotify));

  // call notify on SmCrvOnSurf container
  if( bNotify == TRUE )
    {
      Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    }

  // all done
  return SM_SUCCESS;

} // end SmProjectedCurve::EditParameterization

/*******************************************************************//**
PURPOSE: Get the interval of the iso curve by looking at the domain
    of its base curve.

NOTES:
***********************************************************************/
SmExtent1d SmProjectedCurve::GetNaturalInterval() const
{
  // pass the call along
  return m_cpCurve->GetNaturalInterval();

} // end SmProjectedCurve::GetNaturalInterval

/*******************************************************************//**
PURPOSE: Get list of unique knots and optionally knot multiplicities
     of a BSpline curve by looking at the base surface

NOTES:
***********************************************************************/
SmStatus SmProjectedCurve::GetKnots
 (SmTArray<double> & rKnots,               // out: Unique knot vector
  SmTArray<ULONG>  * pKnotMultiplicities,  // out: multiplicity value for each knot
  const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL]
 const
{
  // pass the call along
  return m_cpCurve->GetKnots(rKnots, pKnotMultiplicities, pOptIvl);

} // end SmProjectedCurve::GetKnots

/*******************************************************************//**
PURPOSE: Get the knots and optionally the multiplicities of this curve.

NOTES:
***********************************************************************/
ULONG SmProjectedCurve::GetNumberNaturalKnots()
 const
{
  // return value
  ULONG lCnt = 0 ;

  // locals
  ULONG ii ;
  SmTArray<double> sKnots ;
  SmTArray<ULONG>  sMults ;
  GetKnots(sKnots, &sMults) ;

  // count the knots
  for(ii=0;ii<sKnots.GetSize();ii++)
    {
      lCnt += sMults[ii] ;

    } // end iter every knot

  // all done
  return(lCnt) ;

} // end SmProjectedCurve::GetKnots

/*******************************************************************//**
PURPOSE: Calculate the minimum continuity of the curve and the
    continuities at each of the unique knots

NOTES: SmProjectedCurve inherit the continuities from their curve.
***********************************************************************/
SmStatus SmProjectedCurve::CalculateContinuities
 (SmContinuityType           & reMinContinuityInCurve,  // out:
  SmTArray<SmContinuityType> & rContinuitiesAtKnots,    // out:
  double                       dContinuityAngleTol)     // in :
 const
{
  // init output
  rContinuitiesAtKnots.ReSet() ;

  // pass the call along
  SmStatus eRtn = m_cpCurve->CalculateContinuities(reMinContinuityInCurve,
                                                   rContinuitiesAtKnots,
                                                   dContinuityAngleTol) ;

  // all done
  return eRtn ;

} // end SmProjectedCurve::CalculateContinuities

/*******************************************************************//**
PURPOSE: Reverse the parameterization of this curve.

NOTES:
***********************************************************************/
SmStatus SmProjectedCurve::ReverseParameterization
  (const SmExtent1d & crOldInterval,  // in : current curve interval
   SmExtent1d       & rNewInterval)   // out: curve interval after reversal
{
  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // Don't own curve can not reverse parameterization
  if (m_bOwnsCurve == 0)
    { SER(SM_ERR) ; }

  // pass the call along to the cpCurve
  SER(((SmCurve *)m_cpCurve)->ReverseParameterization(crOldInterval,rNewInterval));

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  return SM_SUCCESS;

} // end SmProjectedCurve::ReverseParameterization

/*******************************************************************//**
PURPOSE: Trim this curve.

NOTES:
***********************************************************************/
SmStatus SmProjectedCurve::Trim
 (SmExtent1d & crTrimInterval,  // i/o: desired new interval, set to actual new interval
  SmBoolean    bNotify,         // in : internal use only - use default, default:[TRUE]
                                //      TRUE  = call Notify after trimming (previous behavior)
                                //      FALSE = skip Notify after trimming
                                //      UNSURE= skip notify, skip trimming, just recompute TrimInterval
  SmBoolean    bSkipDebugCheck) // in : internal use only - use default, default:[FALSE]
                                //      FALSE= in debug mode silently run this->AssertValid()
                                //      TRUE = don't run AssertValid() before returning
{
  // Don't own curve can not trim the curve
  if (m_bOwnsCurve == 0)
    { SER(SM_ERR) ; }

  if(bNotify != UNSURE)
    { Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL) ; }

  // pass the call along to the UVCurve
  SER(((SmCurve*)m_cpCurve)->Trim(crTrimInterval, bNotify, bSkipDebugCheck));  // may snap sIvl by tol to existing knots

  if(bNotify == TRUE)
    { Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL) ; }

  return SM_SUCCESS;

} // end SmProjectedCurve::Trim

/*******************************************************************//**
PURPOSE: Transform the SmProjectedCurve object.  Only need to transform it
    if it owns the base curve.

NOTES:
***********************************************************************/
SmStatus SmProjectedCurve::Transform
  (const SmAxis2Placement & crRotateNMove, // in : affine rotate and move transformation      
   const SmVector3d       * cpOptScale)    // in : optional scaling about current origin point before RotateNMove
                                           //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                           //      other geom types only support isoptropic scaling
{
  SmVector3d sIdentityScale(1, 1, 1);

  // no work - identity transform
  if (crRotateNMove.IsIdentity() && (cpOptScale == NULL || *cpOptScale == sIdentityScale))
  {
      return SM_SUCCESS;
  }

  Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // Don't own surface, can not transform the curve
  if (m_bOwnsCurve < 2)
    { SER(SM_ERR) ; }

  // pass the call along to the curve
  SER(((SmCurve*)m_cpCurve)->Transform(crRotateNMove,cpOptScale));

  // apply rotations and moves to points and vecs
  crRotateNMove.TransformPoint(m_sProjPoint, m_sProjPoint) ;
  crRotateNMove.TransformVector(m_sProjVec, m_sProjVec) ;
  if     (m_eProjType == SM_PT_PERSPECTIVE) { crRotateNMove.TransformPoint (m_sAuxData, m_sAuxData) ; }
  else if(m_eProjType == SM_PT_ROTATION)    { crRotateNMove.TransformVector(m_sAuxData, m_sAuxData) ; }

  // apply opt scaling to points
  if(cpOptScale)
    {
      // scale the points
      m_sProjPoint.x *= cpOptScale->x ;
      m_sProjPoint.y *= cpOptScale->y ;
      m_sProjPoint.z *= cpOptScale->z ;

      if(m_eProjType == SM_PT_PERSPECTIVE)
        {
          m_sAuxData.x *= cpOptScale->x ;
          m_sAuxData.y *= cpOptScale->y ;
          m_sAuxData.z *= cpOptScale->z ;
        }

    } // end need to scale check

  Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

  // all done
  return SM_SUCCESS;

} // end SmProjectedCurve::Transform

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optionally derivatives that
    lie on the view plane.

NOTES:
***********************************************************************/
SmStatus SmProjectedCurve::Evaluate
 (double     dParameter,              // in : tgt param
  ULONG      lNumDerivatives,         // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .
  SmBoolean  bFromLeft,               // in : if P is on interval boundary
                                      //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                      //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d aPointAndDerivatives[],  // out: array or (pos, tang, 2nd deriv, ...), sized:[lNumDerivatives+1]
  SmBoolean  bNonZeroTangents)        // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
    const                             //      FALSE= return exact tangent values
                                      //      note: Surprisingly TRUE is the common choice because most tangent uses
                                      //            are for their direction (Binorm, SurfNorm comps), but when the
                                      //            tangent is being used for its magnitude (like an arc-length comp)
                                      //            then set this to FALSE.
                                      //      default:[TRUE]
{
  // pass the call along
  SER(m_cpCurve->Evaluate(dParameter,
                          lNumDerivatives,
                          bFromLeft,
                          aPointAndDerivatives,
                          bNonZeroTangents));

  // project the results
  switch(m_eProjType)
    {
      case SM_PT_PARALLEL:
          {
            // position point
            aPointAndDerivatives[0] = aPointAndDerivatives[0].ProjectPointToPlane(m_sProjPoint,
                                                                                  m_sProjVec);
            // derivative vectors
            for (ULONG i=1; i<=lNumDerivatives; i++)
              {
                aPointAndDerivatives[i] = aPointAndDerivatives[i].ProjectToPlane(m_sProjVec);
              }
          }
        break ; // end case SM_PT_PARALLEL

      case SM_PT_PERSPECTIVE:
          {
            // position point
            aPointAndDerivatives[0] = aPointAndDerivatives[0].PerspectiveProjectPointToPlane(m_sProjPoint,
                                                                                             m_sProjVec,
                                                                                             m_sAuxData);
            // derivative vectors
             for (ULONG i=1; i<=lNumDerivatives; i++)
              {
                aPointAndDerivatives[i] = aPointAndDerivatives[i].PerspectiveProjectToPlane(aPointAndDerivatives[0],
                                                                                            m_sProjPoint,
                                                                                            m_sProjVec,
                                                                                            m_sAuxData);
              }
           }
         break ; // end case SM_PT_PERSPECTIVE

      case SM_PT_ROTATION:
          {
            // position point
            aPointAndDerivatives[0] = aPointAndDerivatives[0].RotateProjectPointToPlane(m_sProjPoint,
                                                                                        m_sProjVec,
                                                                                        m_sAuxData);
            // derivative vectors
            for (ULONG i=1; i<=lNumDerivatives; i++)
              {
                aPointAndDerivatives[i] = aPointAndDerivatives[i].RotateProjectToPlane(m_sProjVec,
                                                                                        m_sAuxData);
              }
          }
        break ; // end case SM_PT_ROTATION

      default:
          break;

    } // end switch on ProjType

  // all done
  return SM_SUCCESS;

} // end SmProjectedCurve::Evaluate

/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding Euclidian point on the curve.

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
***********************************************************************/
SmStatus SmProjectedCurve::EvaluatePoint
  (double      dParameter,    // in : target param
   SmPoint3d & rPoint)        // out: evaluated point
 const
{
  SmPoint3d sPnt;

  // base curve evaluation
  SER(m_cpCurve->EvaluatePoint(dParameter,sPnt)) ;

  rPoint = sPnt.ProjectPointToPlane(m_sProjPoint, m_sProjVec);

  // project the point
  switch(m_eProjType)
    {
      case SM_PT_PARALLEL:    { rPoint = sPnt.ProjectPointToPlane(m_sProjPoint,
                                                                  m_sProjVec);
                              }
                                break ; // end case SM_PT_PARALLEL

      case SM_PT_PERSPECTIVE: { rPoint = sPnt.PerspectiveProjectPointToPlane(m_sProjPoint,
                                                                             m_sProjVec,
                                                                             m_sAuxData);
                              }
                                break ; // end case SM_PT_PERSPECTIVE

      case SM_PT_ROTATION:    { rPoint = sPnt.RotateProjectPointToPlane(m_sProjPoint,
                                                                        m_sProjVec,
                                                                        m_sAuxData);
                              }
                                break ; // end case SM_PT_ROTATION
      default:
          break;

    } // end switch on ProjType

    // all done
    return SM_SUCCESS;

} // end SmProjectedCurve::EvaluatePoint

/*******************************************************************//**
PURPOSE: Write SmProjectedCurve to given output stream.

NOTES:
***********************************************************************/
SmStatus SmProjectedCurve::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes
 const
{
  // file type
  SmFileType       eType    = rDB.GetFileType();
  std::ostream   & rFileOut = *rDB.GetOutStreamPtr();

  // curve locals
  ULONG lProjType = m_eProjType ;

  // version 34 = pre-rotation extension database format
  // version 36 = post-rotation extension database format (added member m_sAuxData)
  if (lDBVersionNumber > 34)
    {
      // write post-rotation extension format
      if (eType == SM_ASCII)
        {
          rFileOut << m_sProjPoint.x  << " " << m_sProjPoint.y  << " " << m_sProjPoint.z << " SmProjectedCurve Point on the view plane or rotation axis\n";
          rFileOut << m_sProjVec.x    << " " << m_sProjVec.y    << " " << m_sProjVec.z   << " SmProjectedCurve ViewPlane Normal or rotation axis vector \n";
          rFileOut << m_sAuxData.x    << " " << m_sAuxData.y    << " " << m_sAuxData.z   << " SmProjectedCurve eye point or rotation base plane XAxis vector \n";
          rFileOut << lProjType                                                          << " SmProjectedCurve Projection Type oneof 0=SM_PT_PARALLEL, 1=SM_PT_PERSPECTIVE, 2=SM_PT_ROTATION \n";
        }
      else
        {
          SER(rDB.WriteDouble(m_sProjPoint.x));
          SER(rDB.WriteDouble(m_sProjPoint.y));
          SER(rDB.WriteDouble(m_sProjPoint.z));

          SER(rDB.WriteDouble(m_sProjVec.x));
          SER(rDB.WriteDouble(m_sProjVec.y));
          SER(rDB.WriteDouble(m_sProjVec.z));

          SER(rDB.WriteDouble(m_sAuxData.x));
          SER(rDB.WriteDouble(m_sAuxData.y));
          SER(rDB.WriteDouble(m_sAuxData.z));

          SER(rDB.WriteLong(lProjType));
        }
    }  // end write post-rotation extension format
  else // write in pre-rotation extension format
    {
      if (eType == SM_ASCII)
        {
          rFileOut << m_sProjPoint.x  << " " << m_sProjPoint.y  << " " << m_sProjPoint.z << " SmProjectedCurve SmProjectedCurve Point on the view plane \n";
          rFileOut << m_sProjVec.x    << " " << m_sProjVec.y    << " " << m_sProjVec.z   << " SmProjectedCurve ViewPlane Normal or Eye location \n";
          rFileOut << lProjType                                                          << " SmProjectedCurve Projection Type \n";
        }
      else
        {
          SER(rDB.WriteDouble(m_sProjPoint.x));
          SER(rDB.WriteDouble(m_sProjPoint.y));
          SER(rDB.WriteDouble(m_sProjPoint.z));

          SER(rDB.WriteDouble(m_sProjVec.x));
          SER(rDB.WriteDouble(m_sProjVec.y));
          SER(rDB.WriteDouble(m_sProjVec.z));

          SER(rDB.WriteLong(lProjType));
        }
    } // end write pre-rotation extension format branch

  // curve being projected
  ULONG lProjectedDim = m_cpCurve->GetDim() ;
  if (eType == SM_ASCII) { rFileOut << " ProjectedCurve->ProjectedCurve \n"; }
  SER(rDB.WriteType(m_cpCurve->GetType(), &lProjectedDim)) ;
  SER(m_cpCurve->WriteToDB(rDB, lDBVersionNumber)) ;

  // all done
  return SM_SUCCESS ;

} // end SmProjectedCurve::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmProjectedCurve from a given stream

NOTES:
***********************************************************************/
SmStatus SmProjectedCurve::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  ULONG             lDim,               // NotUsed: in : curve image space dim, 2 or 3
  const SmContext & crContext,          // in : context for new object construction
  SmCurve        *& rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF2(lType, lDim) ;
  // check input
  SER(  (   rpNewCurve == NULL
         || rpNewCurve->IsKindOf(SmProjectedCurve_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmProjectedCurve *pProjectedCurve =   (rpNewCurve == NULL)
                                      ? new (crContext) SmProjectedCurve()
                                      : (SmProjectedCurve *)rpNewCurve ;

  // file type
  SmFileType       eType   =  rDB.GetFileType();
  std::istream   & rFileIn = *rDB.GetInStreamPtr();

  // locals
  SmVector3d sProjPoint ;
  SmVector3d sProjVec ;
  SmVector3d sAuxData ;
  ULONG      lProjType ;

  // version 34 = pre-rotation extension database format
  // version 36 = post-rotation extension database format (added member m_sAuxData)
  if (lDBVersionNumber > 34)
    {
      // write post-rotation extension format
      if (eType == SM_ASCII)
        {
          rFileIn >> sProjPoint.x >> sProjPoint.y >> sProjPoint.z ;  rDB.GoToNextLine() ;
          rFileIn >> sProjVec.x   >> sProjVec.y   >> sProjVec.z ;    rDB.GoToNextLine() ;
          rFileIn >> sAuxData.x   >> sAuxData.y   >> sAuxData.z ;    rDB.GoToNextLine() ;
          rFileIn >> lProjType ;                                     rDB.GoToNextLine() ;
        }
      else
        {
          SER(rDB.ReadDouble(sProjPoint.x));
          SER(rDB.ReadDouble(sProjPoint.y));
          SER(rDB.ReadDouble(sProjPoint.z));

          SER(rDB.ReadDouble(sProjVec.x));
          SER(rDB.ReadDouble(sProjVec.y));
          SER(rDB.ReadDouble(sProjVec.z));

          SER(rDB.ReadDouble(sAuxData.x));
          SER(rDB.ReadDouble(sAuxData.y));
          SER(rDB.ReadDouble(sAuxData.z));

          SER(rDB.ReadLong(lProjType));
        }
    }  // end write post-rotation extension format
  else // write in pre-rotation extension format
    {
      if (eType == SM_ASCII)
        {
          rFileIn >> sProjPoint.x       >> sProjPoint.y       >> sProjPoint.z ;         rDB.GoToNextLine() ;
          rFileIn >> sProjVec.x >> sProjVec.y >> sProjVec.z ;   rDB.GoToNextLine() ;
          rFileIn >> lProjType ;                                                                       rDB.GoToNextLine() ;
        }
      else
        {
          SER(rDB.ReadDouble(sProjPoint.x));
          SER(rDB.ReadDouble(sProjPoint.y));
          SER(rDB.ReadDouble(sProjPoint.z));

          SER(rDB.ReadDouble(sProjVec.x));
          SER(rDB.ReadDouble(sProjVec.y));
          SER(rDB.ReadDouble(sProjVec.z));

          SER(rDB.ReadLong(lProjType));
        }
    } // end write pre-rotation extension format branch

  // curve locals
  SmCurve *pTgtCurve=NULL ;
  SM_TYPE  lProjectedType ;
  ULONG    lProjectedDim ;

  // base curve
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lProjectedType, &lProjectedDim)) ;
  SER(SmCurve::ReadFromDB(lProjectedType, rDB, lProjectedDim, crContext, pTgtCurve, lDBVersionNumber)) ;

  // load the obj
  pProjectedCurve->m_cpCurve     = pTgtCurve ;
  pProjectedCurve->m_sProjPoint  = sProjPoint ;
  pProjectedCurve->m_sProjVec    = sProjVec ;
  pProjectedCurve->m_sAuxData    = sAuxData ;
  pProjectedCurve->m_eProjType   = (SmProjectionType) lProjType ;
  pProjectedCurve->m_bOwnsCurve  = TRUE ;

  // all done
  rpNewCurve = pProjectedCurve ;
  return SM_SUCCESS;

} // end SmProjectedCurve::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmProjectedCurve::IsKindOf( SM_TYPE t ) const
{
  return ((SmProjectedCurve_TYPE == t) ? TRUE : SmCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmProjectedCurve::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_WriteBuffer(_T("\nBegin SmProjectedCurve::Dump()")) ;

  // header
  smos_sprintf(sBuff,_T("\n%s[0x%p], eProjType:[%s], bOwnsBaseCurve:[%s], BaseCurve:[0x%p], BaseCurveType[%s]"),
             GetClassString(),
             this,
               m_eProjType == SM_PT_PARALLEL    ? _T("SM_PT_PARALLEL")
             : m_eProjType == SM_PT_PERSPECTIVE ? _T("SM_PT_PERSPECTIVE")
             : m_eProjType == SM_PT_ROTATION    ? _T("SM_PT_ROTATION")
             :                                    _T("SM_PT_UNKNOWN"),
               m_bOwnsCurve ? _T("TRUE") : _T("FALSE"),
               m_cpCurve,
               m_cpCurve->GetClassString()) ;

  smos_sprintf(sBuffForFile,_T("\n%s, eProjType:[%s], bOwnsBaseCurve:[%s], BaseCurveType[%s]"),
             GetClassString(),
               m_eProjType == SM_PT_PARALLEL    ? _T("SM_PT_PARALLEL")
             : m_eProjType == SM_PT_PERSPECTIVE ? _T("SM_PT_PERSPECTIVE")
             : m_eProjType == SM_PT_ROTATION    ? _T("SM_PT_ROTATION")
             :                                    _T("SM_PT_UNKNOWN"),
               m_bOwnsCurve ? _T("TRUE") : _T("FALSE"),
               m_cpCurve->GetClassString()) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // ProjPoint
  smos_sprintf(sBuff,_T("%s"),_T("\n  ProjPoint = ")) ; smos_WriteBuffer(sBuff) ; m_sProjPoint.Dump() ;
  smos_WriteBuffer(_T("parallel   : Point on the view plane")) ;
  smos_WriteBuffer(_T("\n                                                                      perspective: Point on the view plane   ")) ;
  smos_WriteBuffer(_T("\n                                                                      rotation   : Point on the rotation axis")) ;

  // ProjVec
  smos_sprintf(sBuff,_T("%s"),_T("\n  ProjVec   = ")) ; smos_WriteBuffer(sBuff) ; m_sProjVec.Dump() ;
  smos_WriteBuffer(_T("parallel   : unit-Normal of the view plane")) ;
  smos_WriteBuffer(_T("\n                                                                      perspective: unit-Normal of the view plane")) ;
  smos_WriteBuffer(_T("\n                                                                      rotation   : rotation axis vector         ")) ;

  // AuxData
  smos_sprintf(sBuff,_T("%s"),_T("\n  AuxData   = ")) ; smos_WriteBuffer(sBuff) ; m_sAuxData.Dump() ;
  smos_WriteBuffer(_T("parallel   : not used")) ;
  smos_WriteBuffer(_T("\n                                                                      perspective: eye point                                                   ")) ;
  smos_WriteBuffer(_T("\n                                                                      rotation   : XAxis vector of plane spanned by vectors [AuxData, sProjVec]")) ;

  // output cache data
  SmCurve::Dump(FALSE) ;

  smos_WriteBuffer(_T(" End SmProjectedCurve::Dump()\n")) ;

} // end SmProjectedCurve::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmDisplayList * SmProjectedCurve::DrawPolygon( SmGfxArraySet *pOptGfxSet )   // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                                                             //      NULL to ignore. default:[NULL]
 const
{
    return(m_cpCurve->DrawPolygon( pOptGfxSet )) ;

} // end SmProjectedCurve::DrawPolygon

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertProjectedCurve_list[] =
{
 /*  0 */ {SM_AT_VALUES,      _T("Uninitialized Object"), _T("Projection Type is uninitialized to unknown") },
 /*  1 */ {SM_AT_NESTED_TEST, _T("Base Curve"),           _T("Base Curve must pass AssertValid") },
 /*  2 */ {SM_AT_VECTOR,      _T("uninit vector"),        _T("vectors defining projection must be initialized") },
 /*  3 */ {SM_AT_VECTOR,      _T("zero vector"),          _T("vectors defining projection must be nonZero") },
 /*  4 */ {SM_AT_VECTOR,      _T("unit vector"),          _T("vectors must be unit-length") },
 /*  5 */ {SM_AT_POINTER,     _T("Base Curve Context"),   _T("Base Curve context must be this->m_pContext") }
} ;

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmProjectedCurve::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(pTestRequests);

  SmBoolean bRtn = TRUE ;

  // test for an initialized object
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, m_eProjType != SM_PT_UNKNOWN, _T("")) ;

  // test initialized objects
  if(bRtn)
    {
      // test contained curve
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, m_cpCurve->AssertValid(pAList, eTestLevel, eWalkTree), _T("")) ;

      // test projection defining geometry for initialization
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0,(   m_sProjPoint.IsInitialized()
                                                      && m_sProjVec.IsInitialized()
                                                      && (m_eProjType == SM_PT_PARALLEL || m_sAuxData.IsInitialized())), 
                                                     _T("") ) ;

      // test projection defining geometry for nonZero vectors
      bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0,(    m_sProjVec.IsZero(SM_EFF_ZERO) == FALSE
                                                    && (m_eProjType != SM_PT_ROTATION || m_sAuxData.IsZero(SM_EFF_ZERO) == FALSE)),
                                    SM_EFF_ZERO, m_sProjVec.Length(), _T("")) ;

      // test projection defining geometry for unit-vectors
      bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0,(   SM_IS_ZERO_TO_TOL(m_sProjVec.Length() - 1.0, SM_EFF_ZERO)
                                                    && (   m_eProjType != SM_PT_ROTATION
                                                        || SM_IS_ZERO_TO_TOL(m_sAuxData.Length() - 1.0, SM_EFF_ZERO))),
                                     SM_EFF_ZERO, m_sProjVec.Length() - 1.0, _T("")) ;

      // Check base curve context
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, !m_bOwnsCurve || m_cpCurve->GetContext() == m_cpContext, _T(""));
    } // end defined projection type check

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      Dump() ;
      SM_ASSERT_VALID(m_cpCurve) ;

      SmObject  *pObj      = GetOwner() ;
      SmEdge    *pEdge     = SM_CAST_PTR(SmEdge, pObj) ;
// Remove Composites
// SmCEdge   *pCEdge    = SM_CAST_PTR(SmCEdge, pObj) ;
      SmFace    *pFace     = SM_CAST_PTR(SmFace, pObj) ;
      SmEdgeuse *pEdgeuse  = SM_CAST_PTR(SmEdgeuse, pObj) ;
      SmSurface *pSurface  = pEdgeuse && pEdgeuse->GetFace() ? pEdgeuse->GetFace()->GetSurface() : NULL ;
      SmBrep    *pBrep     =   pEdge    ? pEdge->GetBrep()
// Remove Composites
//                             : pCEdge   ? pCEdge->GetBrep()
                             : pFace    ? pFace->GetBrep()
                             : pEdgeuse ? pEdgeuse->GetBrep()
                             : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;

      smgfx_SetLook(3,4, 1,0,0) ; m_cpCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; m_cpCurve->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; m_cpCurve->DrawSpeed() ; sm_GraphicsLoop() ;

      smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(bRtn) ;

} // end SmProjectedCurve::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmProjectedCurve::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // in : AssertArray holding rAReport
// {
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//       return ( SmCurve::AssertHeal(rAReport, pAList) ) ;
//     }
// 
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ;
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;
//                rAReport.m_pHealMessage = _T("SmProjectedCurve::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmProjectedCurve::AssertHeal
// end obsolete

