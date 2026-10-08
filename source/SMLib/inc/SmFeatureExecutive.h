// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFeatureExecutive.h
* PURPOSE: Header file for SmFeatureExecutive object.
**********************************************************************/

#ifndef __SMFEATUREEXECUTIVE_H__
#define __SMFEATUREEXECUTIVE_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

 /*******************************************************************//**
 PURPOSE: This enum sets the type of attempted rebuilding of the base
   and feature breps

 NOTES: The SM_RB_EXTEND_SURFACE method uses an optional extension distance
     set by SmFeatureExecutive::Set***ExtDistance(). If no extension 
     distance is set an estimate is made based on the size of the loop.
 ***********************************************************************/
enum SmRebuildBrepType
{
    SM_RB_NONE,           // Don't rebuild the Brep
    SM_RB_PLANAR_CAP,     // If the loop edges are planar, then insert a planar face
    SM_RB_EXTEND_SURFACE, // Use a G1 extension of surfaces along the loop edges
    SM_RB_FROM_OTHER_BREP,// Rebuild the brep from SM_RB_EXTEND_SURFACE of other Brep (feature from base or vice versa)
};

/*******************************************************************//**
PURPOSE: This class represents a loop of connected edges that act as a closed 
    boundary between a brep 'feature' and the brep's 'base' geometry

NOTES:  1. When complete the list of SmEdge ptrs must form a simple closed loop
        2. Each loop contains two SmRebuildBrepType enum values that specifies 
         how to fill In the hole made when the SmFeatureExecutive::DoDefeature() 
         method is called to remove a Feature from a Brep Base object.
        3. Different patch breps can be built to complete the Feature and
         the Base
***********************************************************************/
class SM_EXPORT SmFeatureLoop
{
    friend class SmFeature;
    friend class SmFeatureExecutive;

private:
    SmFeature        * m_pFeature = NULL; // Back pointer to the owning SmFeature
    SmTArray<SmEdge*>  m_sBaseEdges;      // Edges in the Base    that make up the boundary
    SmTArray<SmEdge*>  m_sFeatEdges;      // Edges in the Feature that make up the boundary.
                                          // Populated by PartitionFeature
    SmTArray<SmFace*>  m_sBSourceFaces;   // Faces used to construct the most recent patch
    SmRebuildBrepType  m_eBaseRBType;     // Define method on how to rebuild a Base brep
                                          //  across this boundary after the feature is partitioned.
                                          //  Used by a call to SmFeatureExecutive::DoDefeature
    SmRebuildBrepType  m_eFeatRBType;     // Define method on how to rebuild the Feature brep
                                          //  across this boundary after the feature is partitioned
                                          //  Used by a call to SmFeatureExecutive::DoDefeature
    SmBrep           * m_pBasePatch;      // Faces to fill the void enclosed by the loop in the base
    SmBrep           * m_pFeatPatch;      // Faces to fill the void enclosed by the loop in the feature
    SmBoolean          m_bUseBPatch;      // FALSE = an invalid patch is being stored for the base,    don't apply
    SmBoolean          m_bUseFPatch;      // FALSE = an invalid patch is being stored for the feature, don't apply
    SmBoolean          m_bTriedExtSurf;   // TRUE = Attempted to rebuild with SM_RB_EXTEND_SURFACE
                                          //  So we don't try twice when paired w/ SM_RB_FROM_OTHER_BREP

    double             m_dBaseExtDist;    // Surface extension distance for SM_RB_EXTEND_SURFACE w/ base brep
                                          //  If <0, then an estimate will be made.
                                          //  Default: [-1]
    double             m_dFeatExtDist;    // Surface extension distance for SM_RB_EXTEND_SURFACE w/ feature brep
                                          //  If <0, then an estimate will be made.
                                          //  Default: [-1]

public:
    // Constructor
    SmFeatureLoop
    (
        SmTArray<SmEdge*> & rBaseEdges,
        SmRebuildBrepType eBaseRBType,
        SmRebuildBrepType eFeatRBType
    ) :
        m_sBaseEdges( rBaseEdges ),
        m_eBaseRBType( eBaseRBType ),
        m_eFeatRBType( eFeatRBType ),
        m_pBasePatch( NULL ),
        m_pFeatPatch( NULL ),
        m_bUseBPatch( TRUE ),
        m_bUseFPatch( TRUE ),
        m_bTriedExtSurf( FALSE ),
        m_dBaseExtDist( -1. ),
        m_dFeatExtDist( -1. )
    {}; // end SmFeatureLoop constructor

private:
    // Copy constructor
    SmFeatureLoop( SmFeatureLoop & rFLoop ) :
        m_sBaseEdges( rFLoop.m_sBaseEdges ),
        m_eBaseRBType( rFLoop.m_eBaseRBType ),
        m_eFeatRBType( rFLoop.m_eFeatRBType ),
        m_pBasePatch( NULL ),
        m_pFeatPatch( NULL ),
        m_bUseBPatch( TRUE ),
        m_bUseFPatch( TRUE ),
        m_bTriedExtSurf( FALSE ),
        m_dBaseExtDist( -1. ),
        m_dFeatExtDist( -1. )
    {}; // end SmFeatureLoop copy constructor
    
public:
    // Destructor
    ~SmFeatureLoop();

    // Accessors
    void   SetBaseExtDistance   ( double dExtDist )           { m_dBaseExtDist = dExtDist; } 
    void   SetFeatureExtDistance( double dExtDist )           { m_dFeatExtDist = dExtDist; } 
    void   SetBaseRBType        ( SmRebuildBrepType eRBType ) { m_eBaseRBType = eRBType; }   
    void   SetFeatRBType        ( SmRebuildBrepType eRBType ) { m_eFeatRBType = eRBType; }   

    ULONG             GetIndexInFeature() const;                        
    double            GetBaseExtDistance()    { return m_dBaseExtDist; }
    double            GetFeatureExtDistance() { return m_dFeatExtDist; }
    SmRebuildBrepType GetBaseRBType()         { return m_eBaseRBType; } 
    SmRebuildBrepType GetFeatureRBType()      { return m_eFeatRBType; } 
    void              GetUniquePatches( SmTArray<SmBrep*> &rPatches );  ///< [out]:

    SmDisplayList * Draw
    (
        SmBoolean bAddToUIPickList = FALSE, ///< [in] :                                                         <br>
        SmGfxArraySet * pOptGfxSet = NULL   ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters() <br>
    ) const;

    SmDisplayList * DrawBase
    (
        SmBoolean bAddToUIPickList = FALSE, ///< [in] :                                                         <br>
        SmGfxArraySet * pOptGfxSet = NULL   ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters() <br>
    ) const;
    
    SmDisplayList * DrawFeature
    (
        SmBoolean bAddToUIPickList = FALSE, ///< [in] :                                                         <br>
        SmGfxArraySet * pOptGfxSet = NULL   ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters() <br>
    ) const;

    // Define  GetType(), IsKindOf(), and Dump() (Dump needs local implementation)
    SM_COMMON_ONLY_BASE( SmFeatureLoop, SmFeatureLoop_TYPE );

    void Dump( SmBrepDumpType eDumpType ) const; ///< [out]:

}; // end class SmFeatureLoop

/*******************************************************************//**
PURPOSE: Feature Executive that is the top level controlling object
  for modifying features of manifold BReps

NOTES: 
    This executive is designed to operate on a manifold brep by modifying
     its features. Examples of features include blind and cut-through features,
     as well as fillets. For a formal definition of a feature, refer to
     the documentation of the SmFeature class.

    SMLib 8.11.0: DoDefeature is the primary method of this class. This method
     will simplify a brep by replacing features with simpler geometry
     inferred from the surrounding geometry.

    DoDefeature:
     This method is designed to separate a feature from a brep and provides
     tools for rebuilding the base and the feature.  The user provides a brep 
     to defeature and identifies the features. The features are defined either
     by their boundary edges or by their faces.

    * Multiple features can be defeatured at one time
    * The sections of the brep are identified as the Base and the Features.
    * The brep is partitioned along the SmFeatures boundaries (SmFeatureLoops) 
    * Each boundary edge must be manifold, with one face as part of the Feature
       and one as part of the Base

    We refer to the process of separating the Feature from the Base as
     partitioning.  To fill the void left by partition we build a Patch
     A few methods for building patches are provided and identified
     by the SmRebuildBrepType.  Both the Base and Feature can be rebuilt
     and are available to the caller.

    Static, high-level access to this class is available through
     SmFeatureExecutive::DefeatureAndRebuild. This method with simplify
     one given Feature on the Brep.

    Alternatively, the user can exert more control and break the process down
     into it's component steps.  An example use case would be:
     {
       SmBrep           * pOrigBrep = (brep to defeature)
       SmTArray<SmFace*>  sFaces    = (all faces in Feature 1)
       SmTArray<SmEdge*>  sEdges    = (boundary edges of Feature 2)
       SmFace           * pFace     = (face in Feature 2)

       // Create an instance of the executive
       SmFeatureExecutive sFeatureExec(*pContext, *pOrigBrep);

       // Add both features
       SmFeature * pFeature1 = sFeatureExec.AddFeature(sFaces);
       SmFeature * pFeature2 = sFeatureExec.AddFeature(pFace, sEdges);

       SmBrep           * pBaseBrep = NULL;
       SmTArray<SmBrep*>  sFeatureBreps;
       sDeExec->DoDefeature(pBaseBrep, &sFeatureBreps);

       // Query if features were simplified
       pFeature1->GetBaseWasApplied()    // TRUE if patch was applied to Base brep
       pFeature1->GetFeatureWasApplied() // TRUE if patch was applied to Feature brep
     }

    * Multiple rebuild methods can be attempted. Ex: After calling DoDefeature, call
       sFeatureExec.SetBaseRBType( ... ) then call DoDefeature again.
    * If a feature can't be rebuilt, call RestoreFeatures() to place the feature
       back into the base.
    * After calling DoDefeature a new SmFeatureExecutive must be created for
       successful calls to AddFeature.

     When a SmFeatureExecutive object goes out of scope, it does not
     delete the Base or Feature breps. It is up to the user to delete
     these objects if desired
     {
       // Prior work...

       SmBrep * pBaseBrep = sDeExec.GetBaseBrep();
       SmTArray<SmBrep*> sFeatBreps; 
       sFeatureExec.GetFeatureBreps(sFeatBreps);

       SmObjDelete           sCleanBase(pBaseBrep);
       SmObjsDelete<SmBrep*> sCleanFeat(&sFeatBrep);
     }

    * Creation of this class iterates and locks a mark
***********************************************************************/
class SM_EXPORT SmFeatureExecutive
{
    friend class SmFeature;
    friend class SmFeatureLoop;

private:
    const SmContext     & m_crContext;
    SmBrep              * m_pBaseBrep;  // The original brep is stashed here, becomes the base
    SmTArray<SmFeature*>  m_sFeatures;  // List of features added to the base brep
    SmMarkType            m_eMarkType;  // Mark type for the loop edges and feature faces

public:
    // Constructor, destructor:
    SmFeatureExecutive
    (
        const SmContext & crContext,
        SmBrep    & rOriginalBrep
    );

    virtual ~SmFeatureExecutive();

    ////////////////////////////////
    //// High-level interface: /////
    ////////////////////////////////

    // Add a feature to the SmFeatureExecutive by boundary edges
    SmFeature* AddFeature
    (
        SmFace            * pFace,                                     ///< [in] : A face in the feature                                         <br>
        SmTArray<SmEdge*> & rLoopEdges,                                ///< [in] : The edges that form the closed loops of the feature boundary  <br>
        SmRebuildBrepType   eOptBaseRBType = SM_RB_EXTEND_SURFACE,     ///< [in] : Rebuild method for each of the loops along the base           <br>
        SmRebuildBrepType   eOptFeatureRBType = SM_RB_FROM_OTHER_BREP  ///< [in] : Rebuild method for each of the loops along the feature        <br>
    );

    // Add a feature to the SmFeatureExecutive by list of faces
    SmFeature* AddFeature
    (
        SmTArray<SmFace*> & sFaces,                                    ///< [in] : The contiguous faces of the feature                     <br>
        SmRebuildBrepType   eOptBaseRBType = SM_RB_EXTEND_SURFACE,     ///< [in] : Rebuild method for each of the loops along the base     <br>
        SmRebuildBrepType   eOptFeatureRBType = SM_RB_FROM_OTHER_BREP  ///< [in] : Rebuild method for each of the loops along the feature  <br>
    );

    // Advanced AddFeature function: Add a feature where each SmFeatureLoop has unique SmRebuildBrepTypes
    SmFeature* AddFeature
    (
        SmFace                      * pFace,   ///< [in] : A face in the feature                                    <br>
        SmTArray<SmFeatureLoop*>    & rFLoops  ///< [in] : Array of the closed loops that form the feature boundary <br>
    );

    // Replace the Features with simpler geometry
    SmStatus DoDefeature
    (
        SmBrep           *& prBaseBrep,            ///< [out]: Rebuilt base brep.        <br>
        SmTArray<SmBrep*> * pFeatureBreps = NULL   ///< [out]: Rebuilt feature brep.     <br>
    );

    // Call after DoDefeature to restore all features that were partitioned but not rebuilt
    SmStatus RestoreFeatures();

    // Convenience function: Replace a single feature with a simpler geometry
    static SmStatus DefeatureAndRebuild
    (
        const SmContext   & crContext,                              ///< [in ]:                                                  <br>
        SmBrep           *& prOriginalBrep,                         ///< [in,out]: Original brep to be defeatured                   <br>
        SmFace            & rFeatureFace,                           ///< [in ]: A face on the feature section of rOriginalBrep   <br>
        SmTArray<SmEdge*> & rLoopEdges,                             ///< [in ]: A closed loop of manifold edges on rOriginalBrep <br>
        SmBrep           *& prFeatureBrep,                          ///< [out]: Feature brep                                     <br>
        SmRebuildBrepType   eRBBaseType = SM_RB_EXTEND_SURFACE,     ///< [in ]: Rebuild method used with BaseBrep                <br>
        SmRebuildBrepType   eRBFeatureType = SM_RB_FROM_OTHER_BREP  ///< [in ]: Rebuild method used with FeatureBrep             <br>
    );

    // Convenience function: Replace a single feature with a simpler geometry
    static SmStatus DefeatureAndRebuild
    (
        const SmContext   & crContext,                              ///< [in ]:                                       <br>
        SmBrep           *& prOriginalBrep,                         ///< [in,out]: Original brep to be defeatured     <br>
        SmTArray<SmFace*> & rFeatureFaces,                          ///< [in ]: The contiguous faces of the feature   <br>
        SmBrep           *& prFeatureBrep,                          ///< [out]: Feature brep                          <br>
        SmRebuildBrepType   eRBBaseType = SM_RB_EXTEND_SURFACE,     ///< [in ]: Rebuild method used with BaseBrep     <br>
        SmRebuildBrepType   eRBFeatureType = SM_RB_FROM_OTHER_BREP  ///< [in ]: Rebuild method used with FeatureBrep  <br>
    );

    // member access:
    SmBrep * GetBaseBrep    ()                                   { return m_pBaseBrep; } 
    void     GetFeatureBreps( SmTArray<SmBrep*>    & rFBreps );  ///< [out]:
    void     GetFeatures    ( SmTArray<SmFeature*> & rFeatures ) { rFeatures = m_sFeatures; }

    // Sets value for all SmFeatureLoops
    void SetBaseExtDist   ( double  dExtDistance );        ///< [in] :     <br>
    void SetFeatureExtDist( double  dExtDistance );        ///< [in] :     <br>
    void SetBaseRBType    ( SmRebuildBrepType  eRBType );  ///< [in] :     <br>
    void SetFeatureRBType ( SmRebuildBrepType  eRBType );  ///< [in] :     <br>

    void Dump() const;
    void Dump(SmBrepDumpType eBDType) const;

    // internal functions
private:

    // Run all AddFeature methods through here, where work is done 
    SmFeature* AddFeatureInternal
    (
        SmFace                   * pFace,
        SmTArray<SmFeatureLoop*> & rFLoops
    );

    // Check that the constructed patch will work to fill void without intersections. Return SM_ERR on failure
    SmStatus ValidatePatch
    (
        const SmBrep            & pTargetBrep, ///< [in] : Base or feature brep being patched     <br>
        const SmBrep            & pPatchBrep,  ///< [in] : Proposed patch for the void            <br>
        const SmTArray<SmEdge*> & rLoopEdges   ///< [in] : Boundary edges for the SmFeatureLoop   <br>
    );

    // Redirect the brep to the right rebuild method
    SmStatus RebuildBrep
    (
        SmBrep           *& prPatchBrep,        ///< [out]: new brep built to patch the SmFeatureLoop             <br>
        SmTArray<SmEdge*> & rLoopEdges,         ///< [in] : Loop edges corresponding to the SourceBrep            <br>
        SmRebuildBrepType   eRebuildType,       ///< [in] : Rebuild method employed                               <br>
        double              dExtDist,           ///< [in] : Extension distance for SM_RB_EXTEND_SURFACE method    <br>
        SmBrep            * pSourceBrep,        ///< [in] : Geometry base for SM_RB_EXTEND_SURFACE                <br>
        SmTArray<SmFace*> & rFaces              ///< [out]: Faces used to construct the patch                     <br>
    );

    // planar cap work done here
    SmStatus PlanarCapRebuild
    (
        SmBrep                  * pPatchBrep,   ///< [in,out]: brep built to patch the SmFeatureLoop      <br>
        const SmTArray<SmEdge*> & rLoopEdges    ///< [in] : Loop edges corresponding to the SourceBrep    <br>
    );

    // surface extension method work done here
    SmStatus ExtSurfaceRebuild
    (
        SmBrep                  & rPatchBrep,   ///< [in,out]: brep built to patch the SmFeatureLoop           <br>
        SmBrep                  & rSourceBrep,  ///< [in] : Geometry base for SM_RB_EXTEND_SURFACE             <br>
        const SmTArray<SmEdge*> & crLoopEdges,  ///< [in] : Loop edges corresponding to the SourceBrep         <br>
        double                    dExtDist,     ///< [in] : Extension distance for SM_RB_EXTEND_SURFACE method <br>
        SmTArray<SmFace*>       & rFaces        ///< NotUsed: [out]: Faces used to construct the patch                  <br>
    );

    // Build nsided patch
    SmStatus NSidedPatchRebuild
    (
        SmBrep            & rPatchBrep,       ///< [in,out]: brep built to patch the SmFeatureLoop    <br>
        SmBrep            & rSourceBrep,      ///< [in] : Geometry base for SM_RB_EXTEND_SURFACE      <br>
        SmTArray<SmEdge*> & rLoopEdges        ///< [in] : Loop edges corresponding to the SourceBrep  <br>
    );

}; // End SmFeatureExecutive

/*******************************************************************//**
PURPOSE: This class represents Brep Feature

NOTES:
 1. A Brep Feature is a list of connected topology objects that can be treated
 as a single unit within a Brep.

 2. Currently, Features are not persistent and they are only used
 to identify a connected topology section that can be removed from the rest of the Brep
 by the SmFeatureExecutive's DoDefeature() method.

 3. A feature is a set of connected Topology Objects that exist within a larger Brep.
 The rest of the Brep's topology that is outside of the Feature is called the Feature's base.

 4. The feature can be identified the in following ways by SmFeatureExecutive's AddFeature() method

 4.a. The Feature is identified by a list of SmEdges (its geometric boundary) and a single
 interior face.  The complete set of feature topology objects is found by topologically
 traversing from the feature's given internal face to the feature's given boundary.

 4.b. The Feature is identified by a list of contiguous SmFaces that comprise the entire Feature.
 The complete set of feature topology objects is found by topologically traversing the faces to
 find the SmEdges that form the features geometric boundary.

 5. An SmFeature contains a list of FeatureLoops.  Each Loop is a list of Edge pointers that form
 a simply connected loop that marks a boundary between the topologically connected
 topology objects forming the Feature and the rest of the Brep 'Base.'

 6. All boundary edges of the Feature must be manifold, with one connecting face in the
 feature and one connecting face in the base.

 Example1: A blind hole Feature Boundary might be represented by 1 Feature loop
 - the circle at the top of the blind hole.
 The blind hole's interior Face might be either the bottom Face or the
 Cylinder face of the blind hole.
 Example2: A through hole Feature Boundary might be represented by 2 Feature loops
 - the circle at the top of the through hole, and
 - the circle at the bottom of the through hole.
 The Through hole's interior Face might be the cylinder Face of the through hole.
***********************************************************************/
class SM_EXPORT SmFeature
{
    friend class SmFeatureLoop;
    friend class SmFeatureExecutive;

private:
    SmTArray<SmFeatureLoop*>  m_sFeatureLoops;  // list of Feature/Base boundary Loops
    SmTArray<SmFace*>         m_sFaces;         // list of Faces in the Feature
    SmBrep                  * m_pFBrep;         // Brep copy of m_sFaces created by PartitionFeature()
    SmBrep                  * m_pBBrep;         // Back pointer to the Base that originated this feature
    SmFeatureExecutive      * m_pFeatureExec;   // Back pointer to the FeatureExecutive

                                 // State variables that indicate what has happened and what can happen
    SmBoolean m_bIsPartitioned;  // TRUE after partitioning the Feature from the Base
    SmBoolean m_bDefeatureReady; // TRUE if the edge loops have been verified
    SmBoolean m_bBaseRebuilt;    // TRUE after a patch for every loop is built for the base
    SmBoolean m_bFeatRebuilt;    // TRUE after a patch for every loop is built for the feature 
    SmBoolean m_bBaseWasApplied; // TRUE after base patches were applied
    SmBoolean m_bFeatWasApplied; // TRUE after feature patches were applied
    SmBoolean m_bCanRestore;     // TRUE after PartitionFeature(), then FALSE after ApplyPatches()
    SmBoolean m_bJoinedPatches;  // TRUE if patches from different FeatureLoops were merged
    SmBoolean m_bUseBPatchForF;  // TRUE if the base patch should be reused for the feature

public:
    // constructor
    SmFeature( SmFeatureExecutive * pFExec );

    virtual ~SmFeature();

    void AddFeatureLoop( SmFeatureLoop * pFLoop ); ///< [in] :

    // Accessors
    void GetFeatureLoops( SmTArray<SmFeatureLoop*> &rLoops ) { rLoops = m_sFeatureLoops; } ///< [out]:
    void GetFaces( SmTArray<SmFace*> &rFaces ) { rFaces = m_sFaces; }///< [out]:

    ULONG     GetIndexInFeatureExec() const;
    SmBrep*   GetBaseBrep()          { return m_pBBrep; }
    SmBrep*   GetFeatureBrep()       { return m_pFBrep; }
    SmBoolean GetIsPartitioned()     { return m_bIsPartitioned; }
    SmBoolean GetDeatureReady()      { return m_bDefeatureReady; }
    SmBoolean GetBaseRebuilt()       { return m_bBaseRebuilt; }
    SmBoolean GetFeatureRebuilt()    { return m_bFeatRebuilt; }
    SmBoolean GetBaseWasApplied()    { return m_bBaseWasApplied; }
    SmBoolean GetFeatureWasApplied() { return m_bFeatWasApplied; }
    SmBoolean GetCanRestore()        { return m_bCanRestore; }

    void     SetBaseExtDist     ( double  dExtDistance );
    SmStatus SetBaseExtDist     ( SmTArray<double> sExtDistances );
    void     SetFeatureExtDist  ( double  dExtDistance );
    SmStatus SetFeatureExtDist  ( SmTArray<double> sExtDistances );
    void     SetBaseRBType      ( SmRebuildBrepType  eRBType );
    SmStatus SetBaseRBType      ( SmTArray<SmRebuildBrepType> sRBTypes );
    void     SetFeatureRBType   ( SmRebuildBrepType  eRBType );
    SmStatus SetFeatureRBType   ( SmTArray<SmRebuildBrepType> sRBTypes );

    SmDisplayList * Draw
    (
        SmBoolean bAddToUIPickList = FALSE, ///< [in] :                                                          <br>
        SmGfxArraySet * pOptGfxSet = NULL   ///< [in] : draw parameters from smgfx_RefGlobalDisplayParameters()  <br>
    ) const;

    // Define  GetType(), IsKindOf(), and Dump() (Dump needs local implementation)
    SM_COMMON_ONLY_BASE( SmFeature, SmFeature_TYPE );

private:
    // Topologocal traversal to populate the feature with faces and check feature validity
    SmStatus CollectFaces
    ( SmFace * pFace );    ///< [in] : Any face in the feature

                           // Find loop edges on the Feature Brep that correspond to loop edges on the Base
    SmStatus CollectFeatureLoopEdges();

    void SetFeatureFaces( SmTArray<SmFace*> sFaces ) { m_sFaces = sFaces; } ///< [in] :

                                                                            // Partition the feature from the base brep
    SmStatus PartitionFeature();

    // Build patches for each FeatureLoop for the base and feature breps
    SmStatus BuildPatches();

    // Build a patch for a FeatureLoop on for either the base or feature brep
    SmStatus BuildPatch
    (
        SmBoolean       bBaseBrep, ///< [in] : TRUE = Build patch for Base    brep     <br>
                                   ///<      : FALSE= Build patch for Feature brep     <br>
        SmFeatureLoop & rFLoop     ///< [in] : SmFeatureLoop having patch built        <br>
    );

    // Apply whichever patches have been built for this feature to the base and feature breps
    SmStatus ApplyPatches();

    // Used after DoDefeature to restore a feature that was partitioned but not rebuilt
    SmStatus RestoreFeature();

    // Join the invalid patches to make a single patch
    SmStatus JoinPatches
    ( SmBoolean bBaseLoops ); ///< [in] : TRUE  = Join loops on base     <br>
                              ///< [in] : FALSE = Join loops on feature  <br>

    void Dump( SmBrepDumpType eBDType ) const;
}; // end class SmFeature

#endif // !__SMFEATUREEXECUTIVE_H__
