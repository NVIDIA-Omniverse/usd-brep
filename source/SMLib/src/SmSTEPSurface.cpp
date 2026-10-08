// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSTEPSurface.cpp 
* PURPOSE: Implementation of SmSTEPSurface methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmSTEPSurface.h>
#include <SmExtent2d.h>
#include <SmDatabaseIO.h>
#include <SmContext.h>

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmSTEPSurface::SmSTEPSurface
( SmSurface & rSurface,
  SmBoolean bOwnsSurface )
  : m_pSurface( &rSurface ),
  m_bOwnsSurface( bOwnsSurface )
{}

/*******************************************************************//**
PURPOSE: Destructor for STEP surface objects

NOTES: 
***********************************************************************/
SmSTEPSurface::~SmSTEPSurface()
{
  if (m_bOwnsSurface) { delete m_pSurface; }

} // end SmSTEPSurface::~SmSTEPSurface destructor

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmExtent2d SmSTEPSurface::GetNaturalUVDomain() const
{
  return m_pSurface->GetSTEPUVDomain();
}


/*******************************************************************//**
PURPOSE: Equality operator for SmSTEPSurface

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmSTEPSurface::operator== (const SmSurface& crOther) const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmSurface::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmSTEPSurface &rOther = (SmSTEPSurface &)crOther ;

      // check equivalence of these objects
      bRtn =  (   (   ( m_pSurface == rOther.m_pSurface)
                   || ( m_pSurface == NULL && rOther.m_pSurface == NULL)
                   || (   m_pSurface != NULL && rOther.m_pSurface != NULL
                       && *m_pSurface == *rOther.m_pSurface))) ;
    }

  // all done
  return bRtn ;

} // end SmSTEPSurface::operator==

/*******************************************************************//**
PURPOSE: Evaluate a point on the offset surface.

NOTES: 
***********************************************************************/
SmStatus SmSTEPSurface::EvaluatePoint(const SmPoint2d & crUV, SmPoint3d & rPoint) const
{
    SER(m_pSurface->EvaluateSTEPPoint(crUV,rPoint));
    return SM_SUCCESS;

} // end SmSTEPSurface::EvaluatePoint

/*******************************************************************//**
PURPOSE: Evaluate a point and derivatives on the offset surface.

NOTES: 
***********************************************************************/
SmStatus SmSTEPSurface::Evaluate
  (const SmPoint2d & crUV,   // in : param value to evaluate
   ULONG lHighestUDeriv,     // in : number of U derivatives
   ULONG lHighestVDeriv,     // in : number of V derivatives to compute
   SmBoolean bUFromLeft,     // in : if P is on U interval boundary
                             //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                             //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bVFromLeft,     // in : if P is on V interval boundary
                             //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                             //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bOnlyUpperHalf, // in : TRUE=compute upper half of matrix only
                             //      ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value 
                             //                [Dv --]       [Dv   Duv   ---]           (the memory has to be allocated)
                             //                              [Dvv  ---   ---]
   SmVector3d *aDerivatives, // out: matrix of evaluations values
                             //      sized:[lHighestUDeriv+1][lHighestVDeriv+1]
                             //      2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)
                             //                    [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )
                             //                    [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]
                             //                    [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]
                             //      1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]
                             //                     Du, Duv, Duvv, Duvvv,.. 
                             //                     Duu, Duuv, Duuvv, Duuvvv,...]
  SmBoolean,                 // in : bNonZeroTangents: TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                             //                        FALSE= return exact tangent values
                             //      note: Surprisingly TRUE is the common choice because most tangent uses
                             //            are for their direction (Binorm, SurfNorm comps), but when the 
                             //            tangent is being used for its magnitude (like an arc-length comp)
                             //            then set this to FALSE.
                             //      default:[TRUE]
  SmBoolean )                // in : bDoZeroSampling = for internal use only, always set to TRUE, default:[TRUE]
 const  
{
  // pass the call along
  SER(m_pSurface->EvaluateSTEP(crUV,
                               lHighestUDeriv,
                               lHighestVDeriv,
                               bUFromLeft,
                               bVFromLeft,
                               bOnlyUpperHalf,
                               aDerivatives));
  // all done
  return SM_SUCCESS;

} // end SmSTEPSurface::Evaluate

/*******************************************************************//**
PURPOSE: Get the domain of a surface with the minimum continuity
    given that is at the given point and pointed to by the given 
    vector.  If the point lies on a discontinuity then the vector will
    be used to try to determine which side of the discontinuity is
    requested.  There is still one case where there is an ambiguity.
    That is the case where the vector goes along the discontinuity.
    In that case we will take the minimum side.

NOTES: 
***********************************************************************/
SmExtent2d SmSTEPSurface::GetDomainWithContinuity
  (SmContinuityType ,
   const SmPoint2d & ,
   const SmVector2d & ) 
  const
{
    return m_pSurface->GetSTEPUVDomain();

} // end SmSTEPSurface::GetDomainWithContinuity

/*******************************************************************//**
PURPOSE: Get a list of the unique knots and optionally knot multiplicities
    of one of the parameters of the underlying surface of the offset.
    
NOTES: This is the STEP compatible form of the knots not the
    typical knot vector associated with NURBS.
***********************************************************************/
SmStatus SmSTEPSurface::GetKnots
  (SmSurfParamType    eSurfParam,
   SmTArray<double> & rKnots, 
   SmTArray<ULONG>  * pKnotMultiplicities,
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL] 
  const
{
  // init output
  rKnots.ReSet();
  if (pKnotMultiplicities) 
    { pKnotMultiplicities->ReSet() ; }

  // locals
  SmExtent2d sUVDomain = m_pSurface->GetSTEPUVDomain();

  // output single bezier segment for the surface
  if (eSurfParam == SM_SP_U) 
    { if(pOptIvl == NULL) { rKnots.Add(sUVDomain.GetMin().x);
                            rKnots.Add(sUVDomain.GetMax().x);
                          }
      else                { if(pOptIvl->ContainsValue(sUVDomain.GetMin().x))
                              { rKnots.Add(sUVDomain.GetMin().x); }
                            if(pOptIvl->ContainsValue(sUVDomain.GetMax().x))
                              { rKnots.Add(sUVDomain.GetMax().x); }
                          }
    }
  else                       
    { if(pOptIvl == NULL) { rKnots.Add(sUVDomain.GetMin().y);
                            rKnots.Add(sUVDomain.GetMax().y);
                          }
      else                { if(pOptIvl->ContainsValue(sUVDomain.GetMin().y))
                              { rKnots.Add(sUVDomain.GetMin().y); }
                            if(pOptIvl->ContainsValue(sUVDomain.GetMax().y))
                              { rKnots.Add(sUVDomain.GetMax().y); }
                          }
    }

  // when asked - set multiplicities
  if (pKnotMultiplicities)   
    { ULONG lNumKnots = m_pSurface->GetDegree(eSurfParam)+1;
      if(rKnots.GetSize() >= 1) { pKnotMultiplicities->Add(lNumKnots); }
      if(rKnots.GetSize() >= 2) { pKnotMultiplicities->Add(lNumKnots); }
    }

  // all done
  return SM_SUCCESS;

} // end SmSTEPSurface::GetKnots

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmSTEPSurface.

NOTES: Does not add in attribute memory
***********************************************************************/
ULONG SmSTEPSurface::GetMemoryUsed    // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,      // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)              // in : uses without increment eMarkType value
  const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // this + surface memory
  ULONG lThisAllocated = 0 ;
  /* ULONG lUsed        =   sizeof(*this) + */ m_pSurface->GetMemoryUsed(lThisAllocated, eMarkType) ;
  rlMemoryAllocated  =   sizeof(*this) + lThisAllocated ;

  // + attribute memory
  ULONG lUsed       = rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
                                                                       eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;

  // + cache memory
  if ( m_pCacheObj )
  {
      lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
      rlMemoryAllocated += lThisAllocated;
  }

  // all done
  return(lUsed) ;

} // end SmSTEPSurface::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Write SmSTEPSurface to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmSTEPSurface::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type
  SmFileType      eType    = rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();
      
  // base surface
  if (eType == SM_ASCII) { rFileOut << " SmSTEPSurface->baseSurface \n"; }
  SER(rDB.WriteType(m_pSurface->GetType())) ;
  SER(m_pSurface->WriteToDB(rDB, lDBVersionNumber)) ;

  // all done
  return SM_SUCCESS ;

} // end SmSTEPSurface::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmSTEPSurface from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmSTEPSurface::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  const SmContext & crContext,          // in : context for new object construction
  SmSurface      *& rpNewSurface,       // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF1(lType) ;
  // check input
  SER(  (   rpNewSurface == NULL
         || rpNewSurface->IsKindOf(SmSTEPSurface_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmSTEPSurface *pSTEPSurface =   (rpNewSurface == NULL)
                                ? new (crContext) SmSTEPSurface()
                                : (SmSTEPSurface *)rpNewSurface ;

  // file type
  SmFileType eType   =  rDB.GetFileType();
  // std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // surface locals
  SmSurface *pBaseSurface=NULL ;
  SM_TYPE    lBaseType ;  

  // base surface
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lBaseType)) ; 
  SER(SmSurface::ReadFromDB(lBaseType, rDB, crContext, pBaseSurface, lDBVersionNumber)) ; 

  // load object
  pSTEPSurface->m_pSurface     = pBaseSurface ;
  pSTEPSurface->m_bOwnsSurface = TRUE ;
  
  rpNewSurface = pSTEPSurface ; 

  // all done
  return SM_SUCCESS;

} // end SmSTEPSurface::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSTEPSurface::IsKindOf( SM_TYPE t ) const
{
  return ((SmSTEPSurface_TYPE == t) ? TRUE : SmSurface::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmSTEPSurface::Dump
  (void) 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmSTEPSurface::Dump()")) ;

  // report Cache data
  SmSurface::Dump(FALSE) ;

  smos_sprintf(sBuff,       _T("\nSmSTEPSurface = 0x%p, \n"),this);
  smos_sprintf(sBuffForFile,_T("\nSmSTEPSurface = %s, \n"), _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  m_pSurface->Dump();

  smos_WriteBuffer(_T(" End SmSTEPSurface::Dump()\n")) ;

} // end SmSTEPSurface::Dump
