// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmVertexProps.h
* PURPOSE: Header file for SmEdgeProps class.
**********************************************************************/

#ifndef __SMVERTEXPROPS_H__
#define __SMVERTEXPROPS_H__

//#pragma warning(disable : 4291)   // no matching operator delete found; // restored these warnings to clean up errors on linux

#include <SmTypes.h>
#include <SmTArray.h>
#include <SmHealData.h>
#include <SmObjProps.h>

class SmVertex ; 
class SmVertexEdgeGap ;
class SmVertexFaceGap ;

/*******************************************************************//**
PURPOSE: Container class for common Vertex and Vertex AssertValid property values.

NOTES: 1. Initial use: To pass informtion around the Heal Methods
          called by the HealBrep() method.
***********************************************************************/
class SM_EXPORT SmVertexProps: public SmObjProps
{
 public:
  // context pointer (not owned)
  const SmContext       * m_cpContext         = NULL ;            // set : [constructor()]
                                              
  // Simple properties                        
  SmVertex              * m_pVertex           = NULL ;            // set : [SetProps()] - (changes every debug run)
  ULONG                   m_lVertexIndx       = SM_UNDEF_ULONG ;  // set : [SetProps()] associated VertexIndx within managing SmHeadData::m_sTgtVertices List for m_pFace - (constant between debug runs)
  SmZoneTol3d             m_sZoneTol3d        = SM_UNDEF_DOUBLE ; // set : [SetProps() && SmHeal::Fix_TolSizes()]
  SmZoneTol3d             m_sOrigZoneTol3d    = SM_UNDEF_DOUBLE ; // set : [SetProps()]
  SmVertexEdgeGap       * m_pMaxGap3d_VtxEdge = NULL ;            // set : [SetProps()] - NULL for Shell and Loop Vertices
  SmVertexFaceGap       * m_pMaxGap3d_VtxFace = NULL ;            // set : [SetProps()] - NULL for Shell Vertices

  // Problems
  SmBooleanUL             m_bBadSmallZoneTol3d = UNSURE ;         // link: SmHealData::m_sBadVertexProps_SmallZoneTol3d
                                                                  // set : [SetProps()]
                                                                  // use : TRUE = stored ZoneTol3d < Calc ZoneTol3d(MaxGap3ds)
  SmBooleanUL             m_bBadLargeZoneTol3d = UNSURE ;         // link: SmHealData::m_sBadVertexProps_LargeZoneTol3d
                                                                  // set : [SetProps()]
                                                                  // use : TRUE = stored ZoneTol3d > Calc ZoneTol3d(MaxGap3ds)
  SmBooleanUL             m_bWarnLargeGap3d    = UNSURE ;         // link: SmHealData::m_sWarnVertexProps_LargeGap3d
                                                                  // set: [SetProps()]
                                                                  // use: TRUE = any Vertex/Edge or Vertex/Face Gap3d > DefZoneTol3d
  SmBooleanUL             m_bBadCoincidentVertex = UNSURE ;       // link: SmHealData::m_sBadVertexProps_CoinVertices
                                                                  // set: [SmHeal::GetOrDump_CoinVertices()

 public:                                                           

  // constructors and destructor
  SmVertexProps(const SmContext * cpContext) ;                              
  SmVertexProps(SmVertex * pVertex, ULONG lVertexIndx, const SmContext * cpContext) ;
  SmVertexProps(const SmVertexProps & crOther) ;

  virtual ~SmVertexProps() { ReSet(); m_cpContext = NULL; }

  // deep copy operator
 SmStatus Copy( SmVertexProps *& rpNewVertexProps )
 {
   rpNewVertexProps = new SmVertexProps( *this ); NER( rpNewVertexProps );
   return SM_SUCCESS;
 }

  // assignment and equality operators
  SmVertexProps & operator= (const SmVertexProps &crOther) ;
  SmBoolean     operator==(const SmVertexProps &crOther) const;
  void          SetContext(const SmContext * cpContext) { m_cpContext = cpContext ; }

 public:
  
  // set all members to init values
  void ReSet() ;

  // Evaluate and Set all internal member properties for this given Vertex Before Vertexs are split at MissingSeams
  SmStatus SetProps
  (
    SmVertex        * pVertex,                           ///< [in] : tgt Vertex                                                                                  <br>
    ULONG             lVertexIndx,                       ///< [in] : associated Indx of pVertex in managing SmHealData::m_sVerticesList                         <br>
    SmTArray<ULONG> * pOptVertEdgeGap3d_Histogram=NULL,  ///< [in,out] : i/o: accumulating Vertex/Edge Gap3d histogram, NULL to ignore                           <br>
    SmTArray<ULONG> * pOptVertFaceGap3d_Histogram=NULL,  ///< [in,out] : accumulating Vertex/Face Gap3d histogram, NULL to ignore                                <br>
    ULONG             lOptLabel=SM_UNDEF_ULONG           ///< [in] : Optional numeric label for Debug reports, SM_UNDEF_LONG to ignore, default:[SM_UNDEF_LONG]  <br>
  );

  SmBoolean IsSet()       const ; // rtn: TRUE=Obj has no uninit member values
  SmBoolean HasProblems() const ; // rtn: TRUE=Obj has any problems

  // draw
  SmDisplayList * Draw
  (
    SmBoolean bDrawAllCases             = FALSE,         ///< [in] : TRUE = output graphics for all Vertexs (with or without probs)               <br>
                                                         ///<      : default:[FALSE] = output graphics only for Vertexs with probs                <br>
     SmVector3d sHighlightColor         ={ 1, 0, 0},     ///< [in] : def:[ 1, 0, 0]: If VertexHasProblems Draw VertexUV for highlight             <br>
     SmVector3d sBadSmallZoneTol3dColor ={ 0, 1, 1},     ///< [in] : def:[ 0, 1, 1]: Vertices with too small ZoneTol3d Highlight Color            <br>
     SmVector3d sBadLargeZoneTol3dColor ={ 1, 0, 1},     ///< [in] : def:[ 1, 0, 1]: Vertices with too large ZoneTol3d Highlight Color            <br>
     SmVector3d sWarnLargeGap3dColor    ={ 1,.5,.1},     ///< [in] : def:[ 1,.5,.1]: Vertices with Gaps larger than DefZoneTol3d Highlight Color  <br>
     SmGfxArraySet * pOptGfxSet=NULL                     ///< [in] : When given, output GfxVertexArrays not GL calls.                             <br>
  ) const ;
                      
  // pretty print
  void Dump(ULONG iLabel) const ;   // in : optional numeric label, SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_LONG]

  // Common label methods and void Dump() const ;
  SM_COMMON(SmVertexProps, SmObjProps, SmVertexProps_TYPE) ;

} ; // end class SmVertexProps

// add a SmTArray<SmLoopProps> template to the dll interface
// SM_TARRAY_TEMPLATE_PREDECLARATION(SmVertexProps) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmVertexProps*) ;

#endif // !__SMVERTEXPROPS_H__
