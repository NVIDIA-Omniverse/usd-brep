// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmMerge.h
* PURPOSE: Header file for SmMerge object.
**********************************************************************/

#ifndef __SMMERGE_H__
#define __SMMERGE_H__

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif

#ifndef __SMTOPOLOGYINTERSECTOR_H__
#include <SmTopologyIntersector.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#ifdef SM_USE_GLOBAL_CACHE
// Temporary Object Cache MaxCount limits to allow operators the chance to ask for more cache memory.
//    Increased MaxCount limits are restored when the operator completes.
//    Currently only used by SmMerge::ManifoldBoolean().
// To turn this feature off - make sure the extented values are the same as the Default values above.
#define SM_EXTENDED_MAXCOUNT_CURVECACHE    10000
#define SM_EXTENDED_MAXCOUNT_SURFACECACHE   1000
#define SM_EXTENDED_MAXCOUNT_TRIMSRFCACHE   1000
#define SM_EXTENDED_MAXCOUNT_BREPCACHE      1000
#endif // SM_USE_GLOBAL_CACHE

/*******************************************************************//**
PURPOSE: The Boolean flags used by SmMerge::ManifoldBoolean

NOTES: Used by SmMerge static functions to pass flag values along
  1. bCookieCutter       = When TRUE, directs the Boolean operator to only
                           to remove geometry from BrepA based onthe shape of 
                           BrepB.  Imprints (A xsect B) geometry on BrepA,
                           Removes all of BrepA's Delete Faces and skips adding
                           All of the BrepB AddFaces to BrepA.  
                           The output of a cookie cutter opertaion between two
                           manifold objects may not be manifold.
                           default:[FALSE]
  2. bImprinting         = When TRUE, directs the Boolean operator to only
                           add the intersection geometry found between two Breps 
                           onto BrepA by imprinting (A xsect B) geometry onto BrepA
                           and then quits.  This operation will just subdivide the
                           original BrepA geometry showing the locations where
                           it intersects with BrepB.  If BrepA begins as manifold,
                           it will end as manifold.
                           default:[FALSE]
  3. bBooleanPostProcess = When TRUE, directs the Boolean operator to remove
                           all topological vertices and edges from the resulting
                           Brep after the boolean operation is complete.  A topological
                           vertex is a vertex in the middle of a single edge not connected
                           to any other edges or a vertex in the middle of face or region
                           not connected to any edges at all.  A topological
                           edge is an edge in the middle of a single face not connected
                           to any other faces or an edge in the middle or a region not 
                           connected to any faces at all.  Removal of a topological
                           entity does not change the shape of its Brep but may cause
                           a pair of connected topology objects to be merged into one.
                           default:[TRUE]
  4. bImprintAndClassifyFaces = When TRUE, a variation of bImprinting == TRUE.  After
                           directing the Boolean operator to imprint (A xSect B)
                           onto BrepA and before quitting, it marks all the BrepbA
                           faces that would normally have been deleted with an
                           SM_AI_BOOLEAN_DELETED attribute.  This method allows the
                           caller to get the list of faces to be deleted from a Brep
                           by a Boolean operation without having to actually delete those faces.
                           default:[FALSE]
  5. bKeepOtherBrep      = When TRUE, directs the Boolean operator to skip deleting
                           the other Brep when the operation is complete.  Normally
                           this saaves BrepB, but when the operation is SM_BO_EXTRACT_SEPARATE
                           saves BrepA.
                           default:[FALSE]                    
                           
***********************************************************************/
class SM_EXPORT SmMergeOptions
{
  protected:                                        
      
      SmBoolean           m_bCookieCutter;             // [control-param] TRUE=don't add faces from the second Brep.  
                                                       // default:[FALSE]      Just remove faces from the first.
                                                       //   Only used in SmMerge::ManifoldBoolean().
                                                      
      SmBoolean           m_bImprinting;               // [control-param] TRUE= Imprint (A Intersect B) results
                                                       //                       as Edges and Vertices in A
                                                       //   Functionally equivalent to SM_BO_IMPRINT.
                                                       //   Stops the normal deletion of the m_vTI.m_pOther Brep.
                                                       //   Only used in SmMerge::ManifoldBoolean().
                                                       // default:[FALSE]   
                                                      
      SmBoolean           m_bBooleanPostProcess;       // [control-param] TRUE=After Boolean-Try removing extra edges 
                                                       // default:[TRUE]      to merge faces sharing common surfaces.

      SmBoolean           m_bImprintAndClassifyFaces;  // [control-param] TRUE and used with one of the eOperation values of 
                                                       //       SM_BO_UNION,                         
                                                       //       SM_BO_INTERSECTION,                  
                                                       //       SM_BO_DIFFERENCE,                    
                                                       //       SM_BO_EXCLUSIVE_OR,                  
                                                       //       SM_BO_MERGE 
                                                       // changes the SmMerge::ManifoldBoolean behavior. 
                                                       // After imprinting (A intersect B) geometry into the m_pBrep, 
                                                       // keeps and marks faces with an SM_AI_BOOLEAN_DELETED attribute
                                                       // that would have been deleted in the normal course of the operation and
                                                       // stops the normal deletion of the m_vTI.m_pOther Brep.
                                                       // default:[FALSE]

      SmBoolean           m_bKeepOtherBrep;            // [control-param] TRUE= Do not delete other input brep,
                                                       //                       Do not delete the this input brep 
                                                       //                        when eOperation == SM_BO_EXTRACT_SEPARATE.
                                                       // default:[FALSE]

 public:
  
  // default constructor
  SmMergeOptions
  (
    SmBoolean bCookieCutter=FALSE,              // in : TRUE = Skip adding faces (only remove faces) to BrepA, FALSE = don't skip                                 
    SmBoolean bImprinting=FALSE,                // in : TRUE = Imprint (A intersect B) on BrepA and quit (sames SM_BO_IMPRINT), FALSE = don't quit                
    SmBoolean bBooleanPostProcess=TRUE,         // in : TRUE = remove topological edges and vertices after Boolean, FALSE = don't remove                          
    SmBoolean bImprintAndClassifyFaces=FALSE,   // in : TRUE = Imprint (A xsect B) on BrepA, add SM_AI_BOOLEAN_DELETED attrib to del faces, quit. FALSE = don't   
    SmBoolean bKeepOtherBrep=FALSE              // in : TRUE = Do not delete BrepB, Do not delete BrebA with SM_BO_EXTRACT_SEPERATE, FALSE = delete other Brep    
  ) ;           

  // copy constructor
  SmMergeOptions(const SmMergeOptions & crSource) ;

  // virtual destructor
  virtual ~SmMergeOptions() { }

  // simple data access
  SmBoolean GetCookieCutter()                                    { return m_bCookieCutter ; }
  SmBoolean GetImprinting()                                      { return m_bImprinting ; }
  SmBoolean GetBooleanPostProcess()                              { return m_bBooleanPostProcess ; }
  SmBoolean GetImprintAndClassifyFaces()                         { return m_bImprintAndClassifyFaces ; }
  SmBoolean GetKeepOtherBrep()                                   { return m_bKeepOtherBrep ; } 

  void      GetCanonical
  (
    SmBoolean & rbCookieCutter,             // out: TRUE = Skip adding faces (only remove faces) to BrepA, FALSE = don't skip                               
    SmBoolean & rbImprinting,               // out: TRUE = Imprint (A xsect B) on BrepA and quit (sames SM_BO_IMPRINT), FALSE = don't quit                  
    SmBoolean & rbBooleanPostProcess,       // out: TRUE = remove topological edges and vertices after Boolean, FALSE = don't remove                        
    SmBoolean & rbImprintAndClassifyFaces,  // out: TRUE = Imprint (A xsect B) on BrepA, add SM_AI_BOOLEAN_DELETED attrib to del faces, quit. FALSE = don't 
    SmBoolean & rbKeepOtherBrep             // out: TRUE = Do not delete BrepB, Do not delete BrebA with SM_BO_EXTRACT_SEPERATE, FALSE = delete other Brep  
  )
  {
    rbCookieCutter = m_bCookieCutter;
    rbImprinting = m_bImprinting;
    rbBooleanPostProcess = m_bBooleanPostProcess;
    rbImprintAndClassifyFaces = m_bImprintAndClassifyFaces;
    rbKeepOtherBrep = m_bKeepOtherBrep;
  }

  void SetGetCookieCutter           (SmBoolean bCookieCutter)             { m_bCookieCutter            = bCookieCutter            ; }
  void SetGetImprinting             (SmBoolean bImprinting)               { m_bImprinting              = bImprinting              ; }
  void SetGetBooleanPostProcess     (SmBoolean bBooleanPostProcess)       { m_bBooleanPostProcess      = bBooleanPostProcess      ; }
  void SetGetImprintAndClassifyFaces(SmBoolean bImprintAndClassifyFaces)  { m_bImprintAndClassifyFaces = bImprintAndClassifyFaces ; }
  void SetGetKeepOtherBrep          (SmBoolean bKeepOtherBrep)            { m_bKeepOtherBrep           = bKeepOtherBrep           ; }

  void SetCanonical
  (
    SmBoolean bCookieCutter = FALSE,              // in : TRUE = Skip adding faces (only remove faces) to BrepA, FALSE = don't skip                                 
    SmBoolean bImprinting = FALSE,                // in : TRUE = Imprint (A intersect B) on BrepA and quit (sames SM_BO_IMPRINT), FALSE = don't quit                
    SmBoolean bBooleanPostProcess = TRUE,         // in : TRUE = remove topological edges and vertices after Boolean, FALSE = don't remove                          
    SmBoolean bImprintAndClassifyFaces = FALSE,   // in : TRUE = Imprint (A xsect B) on BrepA, add SM_AI_BOOLEAN_DELETED attrib to del faces, quit. FALSE = don't   
    SmBoolean bKeepOtherBrep = FALSE )            // in : TRUE = Do not delete BrepB, Do not delete BrebA with SM_BO_EXTRACT_SEPERATE, FALSE = delete other Brep    
  {
    m_bCookieCutter = bCookieCutter;
    m_bImprinting = bImprinting;
    m_bBooleanPostProcess = bBooleanPostProcess;
    m_bImprintAndClassifyFaces = bImprintAndClassifyFaces;
    m_bKeepOtherBrep = bKeepOtherBrep;
  }

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  //SM_COMMON_BASE(SmMergeOptions, SmMergeOptions_TYPE) ;

};  // end class SmMergeOptions
                                                                         
/*******************************************************************//** 
PURPOSE: The Merge object provides the ability to intersect and          
    combine the topologies of two Breps.

NOTES: 
***********************************************************************/
class SM_EXPORT SmMerge : public SmObject
{
protected:
  SmTopologyIntersector m_vTI;                     // contains target Breps to operate upon,
                                                   // controls intersection/merge phase of boolean operations, and 
                                                   // contains controlling parameters and state needed 
                                                   // for those functions.
                                                  
  SmBoolean           m_bManifoldBoolean;          // [control-param] TRUE=Try fixing non-manifold cases             
                                                   // default:[TRUE]       and treat cases that won't fix as errors.
                                                   //  Set to False in SmMerge::NonManifoldMerge(), used in SmMerge::ManifoldMerge().
                                                  
  SmBoolean           m_bCookieCutter;             // [control-param] TRUE=don't add faces from the second Brep.  
                                                   // default:[FALSE]      Just remove faces from the first.
                                                   //   Only used in SmMerge::ManifoldBoolean().
                                                  
  SmBoolean           m_bImprinting;               // [control-param] TRUE= Imprint (A Intersect B) results
                                                   //                       as Edges and Vertices in A
                                                   //   Functionally equivalent to SM_BO_IMPRINT.
                                                   //   Stops the normal deletion of the m_vTI.m_pOther Brep.
                                                   //   Only used in SmMerge::ManifoldBoolean().
                                                   // default:[FALSE]   
                                                  
  SmBoolean           m_bBooleanPostProcess;       // [control-param] TRUE=After Boolean-Try removing extra edges 
                                                   // default:[FALSE]      to merge faces sharing common surfaces.
                                                  
  SmBoolean           m_bIntersectionLoopClosed;   // [output-status] Set to FALSE if output Brep has any
                                                   //                 unclosed Intersection Loops.
                                                   //    (gaps in intersection loops are detected)

  SmBoolean           m_bImprintAndClassifyFaces;  // [control-param] TRUE and used with one of the eOperation values of 
                                                   //       SM_BO_UNION,                         
                                                   //       SM_BO_INTERSECTION,                  
                                                   //       SM_BO_DIFFERENCE,                    
                                                   //       SM_BO_EXCLUSIVE_OR,                  
                                                   //       SM_BO_MERGE 
                                                   // changes the SmMerge::ManifoldBoolean behavior. 
                                                   // After imprinting (A intersect B) geometry into the m_pBrep, 
                                                   // keeps and marks faces with an SM_AI_BOOLEAN_DELETED attribute
                                                   // that would have been deleted in the normal course of the operation and
                                                   // stops the normal deletion of the m_vTI.m_pOther Brep.
                                                   // default:[FALSE]

  SmBoolean           m_bKeepOtherBrep;            // [control-param] TRUE= Do not delete other input brep,
                                                   //                       Do not delete the this input brep 
                                                   //                        when eOperation == SM_BO_EXTRACT_SEPARATE.
                                                   // default:[FALSE]

  long                m_lTagID;

  SmTArray< SmVertex* > m_sIntersectionVertices;   // Brep vertices and edges that are directly
  SmTArray< SmEdge  * > m_sIntersectionEdges;      //   involved in the intersection.  They will be
                                                   //   checked for removal as topological entities
                                                   //   in BooleanPostProcess().
                                                   //   (No current need for faces.)
public:
  SmTArray<SmVertex*> m_vVertices;                 // Common intersection vertices after Merge on first brep
  SmTArray<SmVertex*> m_vOtherVertices;            // Common intersection vertices after Merge on second brep
  SmTArray<SmEdge*>   m_vEdges;                    // Common intersection edges after Merge on first brep
  SmTArray<SmEdge*>   m_vOtherEdges;               // Common intersection edges after Merge on second brep

  // when eOperation == SM_BO_EXTRACT_SEPARATE extract connected 
  // face sets to separate Breps and put into following arrays.
  // This seperates disjoint geometry into separate Breps
  SmTArray<SmBrep*>   m_vOneBrepPer_OrigBrepConnectedFaceSet ;  // when eOperation == SM_BO_EXTRACT_SEPARATE, 1 Brep from each post merge m_vTI.m_pBrep connected face set (only 1 when all faces connect to one another)
  SmTArray<SmBrep*>   m_vOneBrepPer_OtherBrepConnectedFaceSet ; // when eOperation == SM_BO_EXTRACT_SEPARATE, 1 Brep from each post merge m_vTI.m_pOther connected face set

public:
  // The user can ask us to calculate a default 3d tolerance based on the geometry by 
  // specifying a zero tolerance here. The user can ask us to use a default angle tolerance
  SmMerge
  (
    const SmContext & crContext,                 // in : context for new Object creation                                                                                      
    SmBrep          * pBrep,                     // in : target Brep of future Boolean calls                                                                                  
    SmBrep          * pOtherBrep,                // in : Other Brep used as 2nd argument for future Boolean calls                                                             
    double            dApproxTol3d = 0,          // in : Tolerance used for approximate geometry and intersection distances - gwc: this needs to be split into two arguments  
    double            dAngleTolRadians = 0,      // in : Tolerance used to determine G1 continuity                                                                            
    SmMergeOptions  * pOptMergeOptions = NULL    // in : contains values : m_bCookieCutter                                                                                    
                                                 //                        m_bImprinting                                                                                          
                                                 //                        m_bBooleanPostProcess                                                                                  
                                                 //                        m_bImprintAndClassifyFaces                                                                             
                                                 //                        m_bKeepOtherBrep                                                                                       
  );             
                                                                
  SmMerge(const SmContext & crContext);

  virtual ~SmMerge() { }

  //==============================================================
  // The following methods are considered user-level interface:
  // Users are expected to call these directly in their code.
  //==============================================================

  // General Boolean operator.
  // Combine m_vTI.m_pOther Brep into m_vTI.m_pBrep and delete m_vTI.m_pOther
  //  (exception: don't delete m_vTI.m_pOther when eOperation == SM_BO_PARTIAL_MERGE)
  SmStatus NonManifoldBoolean
  (
    SmBooleanOperationType   eOperation,      // in : oneof SM_BO_UNION                                                       
                                              //            SM_BO_INTERSECTION                                                    
                                              //            SM_BO_DIFFERENCE                                                      
                                              //            SM_BO_EXCLUSIVE_OR                                                    
                                              //            SM_BO_MERGE                                                           
                                              //            SM_BO_PARTIAL_MERGE                                                   
                                              //            SM_BO_IMPRINT                                                         
                                              //            SM_BO_IMPRINT_CLASSIFY  (rather: use Union,XSect,Diff... and          
                                              //                                             m_bImprintAndClassifyFaces == TRUE)  
                                              //            SM_BO_EXTRACT_SEPARATE                                                
                                              //            SM_BO_SLICE                                                           
    SmBrep                *& rpResult         // out: pointer to result or NULL for failure                                   
  );

  // Set m_bManifoldBoolean = TRUE and call NonManifoldBoolean()
  SmStatus ManifoldBoolean
  (
    SmBooleanOperationType   eOperation,           // in : Oneof: SM_BO_UNION        SM_BO_INTERSECTION                                        
                                                   //             SM_BO_DIFFERENCE   SM_BO_EXCLUSIVE_OR                                        
                                                   //             SM_BO_MERGE        SM_BO_PARTIAL_MERGE                                       
                                                   //             SM_BO_IMPRINT      SM_BO_IMPRINT_CLASSIFY                                    
                                                   //                                   (rather: use Union,XSect,Diff... and                   
                                                   //                                            m_bImprintAndClassifyFaces == TRUE)           
                                                   //             SM_BO_SLICE        SM_BO_EXTRACT_SEPARATE                                    
    SmBrep                *& rpResult,             // out: Boolean Result                                                               
    SmNewMarkAndLock       * pOptMarkLock = NULL   // i/o: sets and incs MarkValue, NotNULL=valid marks after call, NULL=not valid   
  ); 
                                                                      

  // Stitch and Merge a Face that is part of m_vTI.m_pBrep.
  // Does stitching, intersections, and all topology connections.
  SmStatus MergeAddedFace
  ( 
    SmFace * pFaceToStitch,                        // in :                      
    ULONG  & rnNumEdgesStitched,                   // out:                      
    double & rdMaxVertGap,                         // out:                      
    double & rdMaxEdgeGap,                         // out:                      
    double & rdMinUnstitchedVertGap,               // out:                      
    double & rdMinUnstitchedEdgeGap,               // out:                      
    SmBoolean bDoGluing=TRUE                       // in : currently unused.    
  );    

  // Stitch and Merge Topology that is part of m_vTI.m_pBrep.
  // Does stitching, intersections, and all topology connections.
  SmStatus MergeAddedTopology
  ( 
    SmTArray< SmFace*   > * pFacesToStitch,             // in :                                                                    
    SmTArray< SmEdge*   > * pEdgesToStitch,             // in :                                                                    
    SmTArray< SmVertex* > * pVertsToStitch,             // in :                                                                    
    ULONG                 & rnNumEdgesStitched,         // out:                                                                    
    double                & rdMaxVertGap,               // out:                                                                    
    double                & rdMaxEdgeGap,               // out:                                                                    
    double                & rdMinUnstitchedVertGap,     // out:                                                                    
    double                & rdMinUnstitchedEdgeGap,     // out:                                                                    
    SmBoolean               bDoGluing=TRUE,             //      Currently unused.                                                  
                                                        //      True: Glue coincident topology. prevents spine edges.  Not advised.
                                                        //      False: just Relate.                                                
    SmBoolean bMakingManifoldSolid = FALSE,              
    SmBoolean bFastEdgeCompare = TRUE,
    SmBoolean bDoRegionNesting = TRUE,
    SmBoolean bIgnoreProblems  = TRUE
  );

  // convenience routine: repeatedly apply the same binary merge 
  //  operator to a list of Breps until one final Brep is made.
  static SmStatus merge_breps
  (
    SmTArray <SmBrep *> & rBreps,                      // i/o: Two or more unique, non-NULL Breps; consumed after validation
    ULONG                 lOperation,                  // in : 0 - union, 1 - intersection, 2 - difference, 3 - merge         
    SmBrep             *& rpResult,                    // out: resulting Brep; NULL on failure
    SmMergeOptions      * pOptMergeOptions = NULL      // in : Optional boolean flags for SmMerge operation (default = NULL)  
  );

  // convenience routine: Build a pair of Breps by merging a list 
  //  of Breps or a list of 2d surface faces and then merge those breps 
  //   - output the pieces as standalone breps.
  static SmStatus BooleanLists 
  (
    SmContext                   * pContext,                 // in : Output Context // in :crements an unlocked mark value                 
    SmTArray <SmBrep *>   const & pBreps1,                  // in : List of input breps must not be modified                                  
    SmTArray <SmSurface*> const & pSurfaces1,               // in : List of input surfaces. All surfaces should be SmPlanes for planar2d        
    SmTArray <SmBrep *>   const & pBreps2,                  // in : List of input breps. input breps must not be modified                     
    SmTArray <SmSurface*> const & pSurfaces2,               // in : List of input surfaces.  All surfaces should be SmPlanes for planar2d     
    int                           operation,                // in : 0,1,2,3,4 = AND, OR, XOR, AND NOT, NOT AND                                
                                                            //      0,1,2,3,4 = intersect,Union,Xor,A-B, B-A                                  
    SmTArray <SmBrep *>         & pBreps3,                  // out: Ouput if breps or non-planar surfs                                        
    SmTArray <SmSurface*>       & pSurfaces3,               // out: Output surfaces if all inputs 2d coplanar                                 
    SmMergeOptions*               pOptMergeOptions = NULL   // in : Optional boolean flags for SmMerge operation (default = NULL)             
  );                                            

  // convenience routine: Perform a set of Boolean operations on a sequence of Breps
  //    using a post fix notation of a CSG tree.
  static SmStatus BooleanTrees
  (
    SmTArray<SmBrep*> & rBreps,                             // i/o: in  - array of Breps to be combined                                     
                                                            //         out - Tree combination result plus any                                  
                                                            //            unused Breps appended to the end                                     
    SmTArray<long>    & rPostFixTrees,                      // in : tree specifying how to combine rBreps                                      
    SmMergeOptions*     pOptMergeOptions = NULL             // in : Optional boolean flags for SmMerge operation (default = NULL)              
  );

  // Merge Brep with otherBrep without deleting the otherBrep object
  SmStatus PiecewiseMerge
  (
    SmRelation<SmSurface,SmSurface> * pSurfacePairsToSkipSSI,      // in : This relation contains pairs of surfaces to skip Surf/Surf intersection.            
    SmBoolean                         bUseExistingTopology,        // in : TRUE = use existing edges between surfaces to eliminate intersections between them. 
                                                                   //           (not currently used in this function)                                          
    SmBoolean                         bCreateNewBrepsForFaces,     // in : TRUE = create a new Brep for each face prior to the merge. This treats each         
                                                                   //          pOther face as a sheet face being merged into pBrep.                            
    SmBrep                         *& rpResult                     // out: Pointer to resulting Brep or NULL for failure                                       
  );

  void GetIntersectionEdges   ( SmTArray< SmEdge*   > &rIntEdges )
    { rIntEdges = m_sIntersectionEdges; }

  void GetIntersectionVertices( SmTArray< SmVertex* > &rIntVerts )
    { rIntVerts = m_sIntersectionVertices; }

  void SetMergeOptions      (SmMergeOptions *pOptions);

  void SetBooleanPostProcess(SmBoolean bBooleanPostProcess)     { m_bBooleanPostProcess = bBooleanPostProcess; }
  void SetCookieCutter      (SmBoolean bCookieCutter)           { m_bCookieCutter = bCookieCutter; }
  void SetImprinting        (SmBoolean bImprinting)             { m_bImprinting = bImprinting; }
  void SetKeepOtherBrep     (SmBoolean bKeepOtherBrep)          { m_bKeepOtherBrep = bKeepOtherBrep; }
  void SetClassifyFaces     (SmBoolean bClassifyFaces)          { m_bImprintAndClassifyFaces = bClassifyFaces; }

  //==============================================================
  // The following methods are considered internal:
  // Users are not expected to call these directly in their code.
  // Calling signatures can change without notice.
  //==============================================================

  // Last step of a Boolean operation.  Mainly, see about removing topological
  // edges and vertices that were created during the operation.
  SmStatus BooleanPostProcess(SmBoolean bCheckAllEdges=TRUE);

  SmStatus FixFoundFaceCoincidence
  (
    SmFace            * pBrepFace,                 // in : Face from Brep coincident with               
    SmFace            * pOtherFace,                // in : Face from other Brep                         
    SmTArray<SmFace*> & rBrepFaces,                // out: faces in rBrep known to be the same as       
    SmTArray<SmFace*> & rOtherFaces,               // out: faces in rOther                              
    SmBoolean         & rbModifiedTopology         // out: TRUE=topology graphs were modified           
  );
                                           
  SmTopologyIntersector & GetTopologyIntersector() { return m_vTI; }

  SmBoolean IsIntersectionLoopClosed() { return m_bIntersectionLoopClosed; }

  // Make a face from the OtherBrep in m_vTI.m_pBrep
  SmStatus MakeFaceuse
  (
    SmFaceuse          * pFaceuseToMakeInBrep,   // in : OtherBrep face to add to Brep                                   
    SmRegion          *& pOldRegion,             // out: Brep Region receiving NewFace                                   
    SmRegion          *& pNewRegion,             // out: New Region made when adding NewFace splits OldRegion            
    SmRegion           * pOptRegion = NULL,      // in : Optional this Brep region known to contain input faceuse->face  
    SmTArray<SmFace*>  * NewFaces = NULL         // out: Newly constructed Face - used for LocalMerge, NULL to ignore    
  ) ;  

  // Merge an edge from the other brep into the brep
  SmStatus MergeEdge
  (
    SmEdgeuse * pEdgeuseInOther,                 // in : Edgeuse from OtherBrep                            
    SmRegion  * pRegionInBrep,                   // in : region in Brep containing otherBrep edge          
    SmEdge   *& rpEdgeInBrep                     // out: new matching edge Brep                            
  );                      

  // Merge a vertex from otherBrep into Brep as a Shell Vertex
  SmStatus MergeVertex
  (
    SmVertex  * pVertexInOther,
    SmRegion  * pRegionInBrep,
    SmVertex *& rpVertexInBrep
  );

  // Do merging and removal of wireframe geometry (wire edges and shell vertices) which do not have corresponding intersections.
  SmStatus ProcessWiresAndShellVertices
  (
    SmBooleanOperationType  eOperation,         // in : one of SM_BO_UNION
                                                //             SM_BO_INTERSECTION                                                                 
                                                //             SM_BO_DIFFERENCE                                                                   
                                                //             SM_BO_EXCLUSIVE_OR                                                                 
                                                //             SM_BO_MERGE                                                                        
                                                //             SM_BO_PARTIAL_MERGE                                                                
                                                //             SM_BO_EXTRACT_SEPARATE                                                             
                                                //             SM_BO_DIFFERENCE                                                                   
    SmTArray<SmEdge*>     & rKeepAsWires,       // out: list of all Brep Edges mated to                                                       
                                                //      Other wires that should be kept in the                                                    
                                                //      final output should all the Brep faces                                                    
                                                //      attached to these edges be deleted.                                                       
    SmMarkType              eBrepMarkType,      // in : // in : MarkType for m_vTI.m_pBrep  from call ManifoldBoolean(), not incremented  
    SmMarkType              eOtherMarkType      // in : // in : MarkType for m_vTI.m_pOther from call ManifoldBoolean(), not incremented  
  ); 

  // for each ResultBrep->Region Copy/Merge Attributes from OtherSrcRegions as needed
  SmStatus PropagateOtherSrcRegionAttribs(SmBooleanOperationType eOperation) ;   // in : current Boolean operation used to determine default Region attribute propagation behavior

} ; // end class SmMerge

#endif // !__SMMERGE_H__
