// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmVolume.h
* PURPOSE: Header file for Volume class. 
**********************************************************************/

#ifndef __SMVOLUME_H__
#define __SMVOLUME_H__

#ifndef __SMBSPLINECURVE_H__
#include <SmBSplineCurve.h>
#endif

#ifndef __SMBSPLINESURFACE_H__
#include <SmBSplineSurface.h>
#endif

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMVOLUME_TYPES_H__
#include <SmVolumeTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

#ifndef __SMEXTENT2D_H__
#include <SmExtent2d.h>
#endif

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

#ifndef __SMAOBJECT_H__
#include <SmAObject.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#include <SmGraphicsExtern.h>
#include <SmVolumeTypes.h>
#include <SmPseudoBox.h>

enum SmPinCushionType ;
class SmBSplineVolume ; 
class SmTransform ;
class SmGfxArraySet ;
class SmDiscontinuities3d ;

/*******************************************************************//**
PURPOSE: This enum names the Volume Spaces as diagrammed below.

NOTES: Used as method arguments to resolve space ambiguities 
 ***********************************************************************/
enum SmVolumeSpaceTYPE
{
  SM_VS_PARAM_SPACE,
  SM_VS_IN_SPACE,  
  SM_VS_PROJ_SPACE,        
  SM_VS_OUT_SPACE    

} ; // end enum SmVolumeSpaceTYPE

/*******************************************************************//**
PURPOSE: This is the Volume class hierarchy's base class. Volumes can
  be used to model volumetric shapes (ex: deformed trimmed bricks) or 
  to deform 3d Space and the objects within them (ex: sheet metal bending). 
   
NOTES:
   To support both volumetric shape modeling and space deformations, 
   volumes are a pair of mappings used to move points and vectors 
   through 4 different but related spaces as shown below.

     +---------------+                                 +------------------+
     |   InSpace     | -------------'Map'------------> |     OutSpace     |
     | [xIn yIn zIn] | <----------'InvMap'------------ | [xOut yOut zOut] |
     +---------------+                              >  +------------------+     
         ^     |                             >      <        ^     |        
         | 'InvOrient'     'Evaluate' >      <               | 'InvOrient'                    
     'Orient'  |               >      <                  'Orient'  |                       
         |     v        >      < 'InvEvaluate'               |     v             
     +---------------+  <                              +------------------+
     |  ParamSpace   | -------'EvaluateSimple'-------> |    ProjSpace     |   
     |   [u v w]     | <-----'InvEvaluateSimple'------ |                  |     
     +---------------+  [  GlobalPointSolveSimple   ]  +------------------+     
                        [InvEvaluateGuessPointSimple] 
                        [   LocalPointSolveSimple   ]       
                     
  Volume Shape modeling is analagous to SMLib's curve and surface modeling
  in which 1d intervals and 2d rectangular extents of points are mapped 
  from a parameter domain to a shared 3d space image domain.  A volume shape
  is a mapping of a 3d rectilinear extent of points mapped to a common
  3d space.  The Volume's parameter space is called its ParamSpace spanned
  by coordinates UVW. When used for Shape Modeling, the Volume's shape
  is defined by mapping its UVW points into ProjSpace with calls to 
  EvaluateSimple().
  
  Volume Space deformations are analgous to the CrvOnSurf class objects in which
  the shape of a curve is mapped from a 1d interval to a 2d Space curve shape,
  and then that 2d curve shape is mapped to a 3d curve shape through
  a surface mapping. When using a Volume to model a space deformation,
  the shared 3d space of a Brep model is projected to a new 3d space
  in which all the old Brep curves and surfaces look deformed.  In a
  Space deformation, a curve is mapped from its 1d interval of parameter
  points into a common 3d space called the InSpace.  The Volume then
  maps all InSpace points to an OutSpace with calls to Map() which makes
  the original curve look deformed. Since the deformation happens to the
  space, all the objects in the space deform in the same manner.
  
  The classes CrvInVolume and SurfInVolume encapsulate curve and surface
  Space Deformation mapping.  The Evaluate functions of those classes, first
  map points from param space to InSpace using the curve and surface 
  definitions, and then map those points to OutSpace using the Volume's
  Map methods.

  Volumetric deformations are implemented centered on the Volume's
  parameter space coordinate origins.  For example, the bend
  axis for a bend space deformation is aligned with the volume's
  parameter space Z axis.  To allow the bend deformation to be positioned
  anywhere in the shared InSpace, Volumes support an orient transformation.
  The Orient transformation positions the parameter coordinates
  of the space deformation in the desired location within InSpace.
  The orient mapping allows a space bend centered on the Z axis to
  be moved and oriented in any manner within InSpace so that users
  can bend Brep models whereever and however they want.

  One Brep model may be subjected to a series of space deformations just
  like a single piece of sheet metal can be bent many times to fold
  up into a box.  Space deformations compound to support this.  That is,
  the output of one space deformation can be used as the input to 
  another space deformation. This functionality is encapsulated into 
  the Volume, CrvInVolume, and SurfInVolume classes and require
  no extra programming on a user's part to exploit.  Sending any curve
  through a space deformation, outputs a new kind of curve.  That curve
  can be sent through subsequent space deformations ad infinitum.  
  The same applies to surfaces.

  There's a lot to Volumetric modeling for shape and space deformations.
  There are 4 distinct spaces (whereas curves and surfaces only model
  two) and all the evaluate and inverse functions to map between those
  spaces.  What follows is a explanation of all the details and the features
  of volumetric modeling.

  Spaces:
    
  ParamSpace = The volume's 3d parameter space spanned by coordinates [u v w].
               ParamPoints and ParamVecs are points and vectors in this space.
               This space is analogous to the parameter spaces of curves
               and surfaces. IsoParameter curves and planes in ParamSpace
               can be mapped to outspace to see the shape of the volume.
               
  ProjSpace  = The image space of the Volume's SimpleMap map which maps
               ParamPoints to ProjPoints. The image of a Volume in Proj space 
               is the 'shape' of the volume before it is oriented into a 
               common 3d OutSpace. Points and vectors in ProjSpace are called
               ProjPoints and ProjectVecs and the space is spanned by
               the coordinates [xProj, yProj, zProj].

  InSpace    = An intermediate space used for Space Deformations. A
               Space deformation like 'bend' is built centered on its Param
               Space's coordinate system and can be positioned anywhere
               in InSpace using the Volume's Orient transform in preparation
               to deform the curves and surfaces that share that common
               InSpace. Points and Vectors in InSpace are called
               InPoints and InVecs and the space is spanned by
               the coordinates [xIn  yIn  zIn].
               
  OutSpace  =  The image space for both Volumetric Shapes and Deformations.
               InSpace points and vectors are mapped to their deformed shapes
               through the Map() methods to OutSpace.  Outspace points and
               vecs are called OutPoints and OutVecs and the space is spanned by
               the coordinates [xOut yOut zOut].
               
  One interesting note: When a Volume's Orient mapping is the unit mapping, the 
               shape of the Volume in OutSpace and ProjSpace are the same.  This
               gives one interpretation of a volume's ProjSpace shape. It's the
               shape of the Volume mapping as built centered on the ParamSpace 
               coordinate system.                                                         
                            
  Mappings: In functional notation,

    OutPoint   = Map(InPoint)                  // use: Map InSpace points to a Deformed OutSpace space as used for part bending.
    OutPoint   = Evaluate(ParamPoint)          // use: Map ParamSpace shape to an oriented and deformed OutSpace shape.
    InPoint    = Orient(ParamPoint)            // use: Map ParamSpace shape to an oriented shape within InSpace  
    ProjPoint  = EvaluateSimple(ParamPoint)    // use: Map ParamSpace shape to a Volume shape in ProjSpace. - virtual method
                                               
  Inverse Mappings:                            
    InPoint    = InvMapPoint(OutPoint)             // use: InvMap OutSpace Point to an InPoint,
    ParamPoint = InvEvaluatePoint(OutPoint)        // use: InvEvaluate OutSpace Point to a ParamPoint.
    ParamPoint = InvOrientPoint(InPoint)           // use: InvOrient InSpace Point to a ParamPoint.
    ParamPoint = GlobalPointSolveSimple()          // use: InvEvaluateSimple ProjSpace Point to a ParamPoint. - virtual methods
               = LocalPointSolveSimple()
  where:
    OutPoint   = Map(InPoint)           = Orient(EvaluateSimple(InvOrient(InPoint))),
    OutPoint   = Evaluate(ParamPoint)   = Orient(EvaluateSimple(ParamPoint)),
    InPoint    = Orient(ParamPoint)     = Orient(ParamPoint),

    InPoint    = InvMap(OutPoint)       = Orient(InvEvaluateSimple(InvOrient(OutPoint))),
    ParamPoint = InvEvaluate(OutPoint)  = InvEvaluateSimple(InvOrient(OutPoint)),
    ParamPoint = InvOrient(OutPoint)    = InvOrient(ParamPoint),

    ParamPoint   = [u v w],
    InPoint      = [xIn  yIn  zIn],
    OutPoint     = [xOut yOut zOut], and
    ProjPoint    = [xProj, yProj, zProj].
                 
  where: Due to the mappings, the coordinates of a point in one space are functions of the coordinates of other spaces as,
    xIn  = xIn(u,v,w),   xOut = xOut(xIn, yIn, zIn),
    yIn  = yIn(u,v,w),   yOut = yOut(xIn, yIn, zIn),
    zIn  = zIn(u,v,w),   zOut = zOut(xIn, yIn, zIn), etc.

  One Common Special Case: Identity Orient Maps (i.e. m_pOrientMap == NULL), 
     InPoint  = ParamPoint,
     OutPoint = ProjPoint, and
     Evaluate(Point) = Map(Point)

NOTES: 
  1. Forward mappings:
       The 'Orient' method maps ParamSpace points to InSpace points as,
       InPoint = Orient(ParamPoint) = SmVolume::Orient(ParamPoint, ...) ; 

       The 'EvaluateSimple' method maps ParamSpace points to ProjSpace points as,
       ProjPoint = EvaluateSimple(ParamPoint) = SmVolume::EvaluateSimple(ParamPoint, ...) ;

       'Evaluate' For Primitive Shapes  : The 'Evaluate' method combines the Orient and the EvaluateSimple 
                               mappings to map ParamSpace points to OutSpace points as
                               OutPoint = Volume(ParamPoint) = SmVolume::Evaluate(ParamPoint, ...)

       'Map' For Space Deformation : The 'Map' method combines the Orient and the EvaluateSimple 
                               mappings to map InSpace points to OutSpace points as
                               OutPoint = Volume(InPoint) = SmVolume::Map(InPoint, ...)

  2. Inverse mappings: Each of the 4 supported mappings which map points from one of the
     SmVolume's spaces to another of the Volume's sapces, are also supported by
     inverse mappings which project points in the opposite direction of their
     paired mapping function as:
                ParamSpace  ===   'Orient'  ==> InSpace
                ParamSpace  <== 'InvOrient' === InSpace

                ProjSpace  ===   'Orient'  ==> OutSpace
                ProjSpace  <== 'InvOrient' === OutSpace

                ParamSpace  ===   'SimpleEvaluate'  ==> ProjSpace
                ParamSpace  <== 'InvSimpleEvaluate' === ProjSpace

                ParamSpace  ===   'Evaluate'  ==> OutSpace
                ParamSpace  <== 'InvEvaluate' === OutSpace

                InSpace  ===   'Map'  ==> OutSpace
                InSpace  <== 'InvMap' === OutSpace

     These functions compound so:
       Evaluate(ParamPoint) = Orient(SimpleEvaluate(ParamPoint))  and
       Map(InSpacePoint) = Orient(SimpleEvaluate(InvOrient(InSpacePoint)))

     1. Class SmVolume implementes the Orient and InvOrient mappings.
     2. Each derived class implements its own version of SimpleEvaluate and InvSimpleEvaluate.
     3. Class SmVolume uses compounding to implement Evaluate, InvEvaluate, Map, and InvMap,
        from the Orient, InvOrient, SimpleEvaluate, and InvSimpleEvaluate methods.

  3. Naming Spaces, Maps, Points, and Coordinates for curves, surfaces, and volumes.

     Points and Vectors exist in a space and may be mapped from one space to another.
     Primitive shapes and space deformations are all represented as maps between a pair of spaces.

     3a. Spaces are named ParamSpace, InSpace, ProjSpace, and OutSpace.

     3b. Points are named ParamPoint, InPoint, ProjPoint, and OutPoint 
         indicating the space in which the points resides. Point Coordinate
         names vary for each different 'type' of point. All volume point types
         are represented by the class SmVector3d and their coordinates by doubles.
         The 'type' of volume points and coordinates is represented solely by 
         point and coordinate argument names.

         Point coordinate names in different spaces are:
         +---------+------------+------------+----------+----------+-----------+
         | Shape   | ParamSpace | ImageSpace | InSpace  | OutSpace | ProjSpace |
         +---------+------------+------------+----------+----------+-----------+
         | curve   | s          |  xyz       |  ---     |  ---     |  ---      |
         | surface | uv         |  xyz       |  ---     |  ---     |  ---      |
         | Volume  | uvw        |  ---       |  xyzIn   |  xyzOut  |  xyzProj  |
         +---------+------------+------------+----------+----------+-----------+

         InPoint     = InPoint   [xIn   yIn   zIn  ]
         OutPoint    = OutPoint  [xOut  yOut  zOut ]
         ParamPoint  = ParamPoint[u     v     w    ]
         ProjPoint   = ProjPoint [xProj yProj zProj]

     3c. Curve and Surface maps are named after their shape type with a ParamSpace 
         point argument. Volume maps have three different 'map(PointArgumentType)' 
         names indicating the spaces they map between. 

         Curve Evalaute   = Curve(s)        (ParamSpace to OutSpace mapping)
         Surface Evalaute = Surface(u,v)    (ParamSpace to OutSpace mapping) 
         Volume Evaluate  = Evaluate(u,v,w) (Volume ParamSpace to OutSpace mapping - primitive shape)
         Volume Map       = Map(InPoint)    (Volume InSpace    to OutSPace mapping - shape deformation)
         Volume Orient    = Orient(u,v,w)   (Volume ParamSpace to InSpace  mapping) 

         In comments the Volume Map() and the Volume Evaluate() names are often replaced by a VolumeName() as: 
             Map(InPoint)         = VolumeName(InPoint)
             Evaluate(ParamPoint) = VolumeName(ParamPoint)
         where the type of argPoint in VolumeName(argPoint) indicates when VolumeName is
         substituting for Map(argPoint = InPoint) or Evaluate(argPoint = ParamPoint).

         The notation, OutArg = MapName(InArg), is shorthand for implemented method calls.  
         Examples: InPoint  = Orient(ParamPoint)     ==  SmVolume::OrientPoint(ParamPoint, InPoint) ;
                   OutPoint = Map(InPoint)           ==  SmVolume::MapPoint(ParamPoint, InPoint) ;
                   OutPoint = Evaluate(ParamPoint)   ==  SmVolume::EvaluatePoint(InPoint, OutPoint) ;
                   OutPoint = Volume(InPoint)        ==  pVolume->MapPoint(ParamPoint, InPoint) ;    
                   OutPoint = Transform(InPoint)     ==  pTransform->MapPoint(ParamPoint, InPoint) ;  
                   OutPoint = BendVolume(ParamPoint) ==  pBendVolume->MapPoint(ParamPoint, InPoint) ;
                   xyzPoint = SurfaceName(uvPoint)   ==  pSurfaceName->EvaluatePoint(uvPoint, xyzPoint) ;  
                   xyzPoint = CurveName(uPoint)      ==  pCurveName->EvaluatePoint(uPoint, xyzPoint) ;

     3d. A mapping may be commented as:  DomainSpace -- MapName(DomainPoint) -> ImageSpace
         +---------+------------------------------------------------------------+--------------+
         + Type    | DomainSpaceName   -- MapName      -> ImageSpaceName        |  CommentName +
         +---------+------------------------------------------------------------+--------------+
         | Curve   | ParamSpace[s]     -- Curve(u)     -> ImageSpace[x,y,z]     | 'Evaluate'   | 
         | Surface | ParamSpace[u,v]   -- Surface(u,v) -> ImageSpace[x,y,z]     | 'Evaluate'   | 
         | Volume  | ParamSpace[u,v,w] -- Volume(u,v,w) -> OutSpace[xyzOut]     | 'Evaluate'   | 
         |         | InSpace[xyzIn]    -- Volume(xyzIn) -> OutSpace[xyzOut]     | 'Map'        | 
         |         | ParamSpace[u,v,w] -- Orient(u,v,w) -> InSpace[xIn,yIn,zIn] | 'Orient'     | 
         +---------+------------------------------------------------------------+--------------+

  4. Common image space concept.  Shapes interact (e.g. a curve bounds a 
     surface, many curves and surfaces form a Brep, Brep pairs
     can Boolean) when their individual image spaces are treated
     as one common image space. 
  
     Without Volumes, there is only one common 3d image space and the
     distinction between the individual and the common image spaces is 
     rarely made. The common image space has many names including: 
     global modeling space, 3d space, and the common image space.

     When making a model by mixing Curve and Surface objects with 
     Volume objects for both primitive shapes and space deformations, the final model 
     shape may depend on a number of distinct 3d modeling spaces. In such
     situations, care is taken to name each 3d space uniquely.

     Multi-3dImageSpace Model Example: 
       The final model shape is in 3dImageSpace3. The shapes in that 
       image space are the combination of several curve, surface, and 
       volume primitives whose individual shapes are defined by sequences 
       of primitive and space deformation mappings through intermediate 
       3dImageSpaces 1 and 2.
     
 1dParamSpace1 -- Curve1(s)      -+
 1dParamSpace2 -- Curve2(s)       +
 2dParamSpace1 -- Surface1(u,v)   +-> 3dImageSpace1 -- Volume3(xyzIn) -+               
 2dParamSpace2 -- Surface2(u,v)   +                                    +               
 3dParamSpace1 -- Volume1(u,v,w) -+                                    +               
                                                                       +-> 3dImageSpace2 -- Volume4(xyzIn) -> 3dImageSpace3               
 1dParamSpace3 -- Curve3(s)      --------------------------------------+               
 2dParamSpace3 -- Surface3(u,v)                                        +
 3dParamSpace2 -- Volume2(u,v,w) --------------------------------------+

  5. A Volume's parametric shape is a predefined point set within the
     Volume's UVW parameter space.  The parametric shape for the SmVolume derived types are:
  +-----------------+--------------------------------+------------------------------------+----------------------+ 
  | Shape           | ParametricSpace Domain         | InSpace Domain                     | OutSpace Domain      | 
  +-----------------+--------------------------------+------------------------------------+----------------------+ 
  | SmBSplineVolume | axis aligned block             | rotated and translated block       | curvilinear block    | 
  | SmBendVolume    | axis aligned half space, u > 0 | rotated and translated half space  | whole space          | 
  | SmUnbendVolume  | axis aligned whole space       | rotated and translated whole space | oriented half space  | 
  | SmTransform     | axis aligned whole space       | rotated and translated whole space | oriented whole space | 
  +-----------------+--------------------------------+------------------------------------+----------------------+ 
                                                            
  6. Two uses for Volumes: Shape Primitives and Space Deformations.

     6a. A Volume's primitive shape is its set of parameter shape points 
         mapped to OutSpace.  IsoPlanes and Isolines may be extracted from the 
         primitive shape to visualize that shape.

           VolumeMap(uvw) = Evaluate(uvw) = Orient(EvaluateSimple(uvw)) ;

     6b. Volumes deform other shapes by projecting the other shape's ImageSpace
         through the volume mapping into a new ImageSpace as,

           DeformedCurve(s)    = VolumeMap(Curve(s))            = Orient(EvaluateSimple(InvOrient(Curve(s)))) ;     
           DeformedSurface(uv) = VolumeMap(Surface(uv))         = Orient(EvaluateSimple(InvOrient(Surface(uv)))) ;  and
           DeformedVolume(uvw) = VolumeMap(VolumeEvaluate(uvw)) = Orient(EvaluateSimple(InvOrient(VolumeEvaluate(uvw)))) ; 

  7. Volumes Compound.
         Simple Volume evaluations are:   OutPoint = ThisVolume(InPoint)
         Compound volume evaluations are: OutPoint = NextVolume(ThisVolume(InPoint)) ;

         Conceptually, Volumes compound to any depth
         but it's unclear when performance might become a problem. 

         Compounded Volume Example: Mirror a bent curve as,
           a. Construct Curve
           b. Bend Curve with classes SmCrvInVolume and SmBendVolume.
           c. Mirror the bent curve with class SmTransform. 
          OutPoint(s) = SmTransform(SmBendVolume(Curve(s))) 

  8. Affine Outspace SmVolume image changes: 

            Mirror()                    Mirror    Volume Outspace Image about Outspace MirrorPlane   
            Scale()                     Scale     Volume Outspace Image about Outspace origin or opt OutSpace CenterPoint   
            Translate()                 Translate Volume Outspace Image by given OutSpace Vector  
            RotateAboutAxis()           Rotate    Volume Outspace Image about an axis running through OutSpace Origin 
            RotateAboutAxisAtPoint()    Rotate    Volume Outspace Image about an axis running through OutSpace CenterPoint    
            Transform()                 Apply a General Transform to a Volume's Outsapce Image    
  
       Volumes can use compounded functions to apply affine transformations to any derived
       Volume shape at the cost of extra evaluations as:

            Modified Volume Shape in OutSpace = Transform(Orient(SimpleMap(ParamPt)))

       The SmVolume Affine methods all create an SmTransform object to transform the current 
       SmVolume OutSpace image to the next 3d space image and add that to the end of the 
       SmVolume's m_pNextMap list of compounded maps.
    
       For performance, the SmVolume::AffineMethods() give each derived class a chance to be more efficient
       by calling a virtual AffineSimple() method whose job is to combine the effects of the two
       compounded maps (SimpleMap and Transform) into a single modified Volume SimpleMap as:
    
          Transform(Orient(SimpleMap(ParamPt))) = Orient(ModifiedSimpleMap(ParamPt)) 
    
       Users should not call the AffineSimple methods directly.  Leave the efficiency issues
       to the derived implementations by calling the Affine methods directly.

COMPOUND METHOD NOTE:  I did try several design ideas for supporting compound mapping.
         Of those all had various problems, some were more complicated than
         others, but only one design worked consistently for all known requirements.

Compound Evaluation Design:
         1. SmVolume contains a m_pNextMap pointer.  When that is NonNULL
            the mapping is compounded, otherwise it's simple.

            CompoundVolume(InPoint) = m_pNextMap->Evalute(this->Evaluate(InPoint))) or in pseudoCode
            CompoundVolume(InPoint) = m_pNextMap(this(InPoint)) ;

            SimpleVolume(InPoint)   = this->Evaluate(InPoint) or in pseudoCode
            SimpleVolume(InPoint)   = this(InPoint) ;

         2. The base class public evaluation methods manage CompoundMap evaluations which
            in turn call on a set of private virtual "simple" evaluation methods. A "simple"
            evaluation is an evaluation of the form

              ProjPoint = EvaluateSimple(ParamPoint). 

            Derived classes only implement the "simple" virtual evaluate methods leaving 
            the base class with its public evaluate interface to manage compound evaluations.   

         3. Volume evaluations can compound to any depth, but it's unclear when performance issues
            may become a problem.  For example, A curve can be bent, mirrored, bent again,
            and translated.  That would be evaluated as
              ModifiedCurve(s) = Translate(Bent(Mirror(Bent(OrigCurve(s))))) ;
            and Represented as
              SmCrvInVolume::m_pCurve  = OrigCurve ;
              SmCrvInVolume::m_pVolume = SmBendVolume 
                                           ->m_pNextMap = SmTransform(asMirror)
                                             ->m_pNextMap = SmBendVolume
                                               ->m_pNextMap = SmTransform(asTranslate)
                                                 ->m_pNextMap = NULL

         4. All methods which compound are duplicated 
               SmVolume::Method() ;                 // <=== called by users of the SmVolume derived class hierarchy
               virtual SmVolume::MethodSimple() ;   // <=== called only by the SmVolume base class methods.

            4a. the list of compounding methods currently includes most of the
                SmVolume methods.  The thing I don't like about this design
                is the duplication of methods.  There is one public BaseClass interface
                exposed to users of the SmVolume class hierarchy and a second equivalent
                private interface for derived classes to implement Simple methods.

            4b. I tried to make these two interfaces one and the same three different times.
                1. First creating an explicit derived class called SmCompoundVolume to 
                  make sure compounding did not become a part of the other SmVolume derived
                  class implementations.  However, that design did not work because
                  there was no way to take an existing SimpleMap and turn it into
                  a CompoundMap without changing the Derived type of the object
                  being converted.  To be able to add compounding to an existing
                  object without having to reallocate it as a new derived type
                  I had to add compounding as a feature to the SmVolume base
                  class.  That allows users of SmMap to write code without having
                  to know which derived classes can implement things like Transform,
                  Mirror, and Scale, directly and which have to do that indirectly
                  through a compound mapping.
               2. Second by creating a list of overloaded methods.  However, that quickly
                  became confusing.  The overloading did not resolve the duplicated interface
                  problems, the system needs both the base class interface that hides
                  the difference between a simple and a compound map and a simple map interface.
                  And the overloading approach hid that duplication making it hard to figure out
                  which method argument list to use and when.
               3. I Tried adding a 2nd map pointer to the SmVolume base class called
                  something like m_pThisMap.  However, that turned out to have the same
                  basic problems as the first approach, when a SmVolume is promoted
                  from being a simple map to a compound map, then the derived class
                  object had to be reallocated and that became a burden on users of the
                  SmVolume class who would then have to know which derived classes
                  could execute without compounding and which require it.
               After these exercises I converged on the cuurent design for compound evaluations.

***********************************************************************/
class SM_EXPORT SmVolume : public SmAObject
{
public:
  SmObject     *m_pOwner ;           // not yet used
                                     
   SmVolume    *m_pNextMap ;         // Next map of Evaluate(InPoint) = CompoundMap(InPoint) = NextMap(ThisMap(InPoint))
                                     // NULL:       Evaluate(InPoint) = SimpleMap(InPoint)   = ThisMap(InPoint)
                                     //  constructor default:[NULL]
                                     
   ULONG        m_lNextOwnerFlag ;   // constructor and assignment default:[0]
                                     // 0 = don't delete m_pNextMap when destructed
                                     // 1 = copied m_pNextMap when constructed, deletes m_pNextMap when destructed
                                     // 2 = didn't copy m_pNextMap when constructed, deletes m_pNextMap when destructed
                                     
   SmTransform *m_pOrientMap ;       // Map ParamSpace points to InSpace points (m_pOrientMap->m_pOrientMap = NULL and m_pOrientMap->m_pNextMap = NULL)
   SmTransform *m_pInvOrientMap ;    // Map InSpace points back to ParamSpace (a derived object)
                                     //     xyzIn  = Orient(ParamPoint)
                                     //     xyzOut = Orient(EvaluateSimple(ParamPoint)), and combining yields,
                                     //     xyzOut = Orient(EvaluateSimple(InvOrient(xyzIn)))
                                     // Note: m_pOrientMap and m_pInvOrientMap are only used for their EvaluateSimple behaviors,
                                     //       as such, their m_pNextMap, m_pOrientMap, and m_pInvOrient pointers are always set to NULL
                                     // NULL: OrientMap = Identity Where Map(ParamPoint) == Evaluate(ParamPoint) == EvaluateSimple(ParamPoint)
                                     //  constructor default:[NULL]
                                     
   ULONG        m_lOrientOwnerFlag ; // constructor and assignment default:[0]
                                     // 0 = don't delete m_pOrientMap when destructed
                                     // 1 = copied m_pOrientMap when constructed, deletes m_pOrientMap when destructed
                                     // 2 = didn't copy m_pOrientMap when constructed, deletes m_pOrientMap when destructed

  // copy constructor with control of compounded maps - private to force use of virtual Copy() method                          
  SmVolume(const SmVolume & crSourceVolume,     ///< [in ]: SourceVolume to copy
           SmBoolean        bSimpleMapOnly=FALSE) ;   ///< [in ]: TRUE = Copy this Volume omitting any compounding volumes
                                                //      FALSE= Copy this Volumes with any compounding volumes

  // copy constructor - private to force use of virtual Copy() method - written to stop compiler from building one
  // SmVolume(const SmVolume & crSourceVolume)     { SmVolume::SmVolume(crSourceVolume, FALSE) ; }

public:
  // empty constructor needed for I/O
  SmVolume(const SmContext *cpContext=NULL) 
    : m_pOwner(NULL),
      m_pNextMap(NULL),
      m_lNextOwnerFlag(0),
      m_pOrientMap(NULL),
      m_pInvOrientMap(NULL),
      m_lOrientOwnerFlag(0)   
  { 
    if(cpContext) { SetContext(cpContext) ; }
    SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL) ; 
  }

  // constructor
  SmVolume
  (
    const SmContext * cpContext,            ///< [in ]: context for this new object                                                                 <br>
    SmObject        * pOwner,               ///< [in ]: Owner value                                                                                 <br>
    SmVolume        * pNextMap=NULL,        ///< [in ]: Opt NextMap of CompoundMaps. NULL to ignore. default:[NULL]                                 <br>
    ULONG             lNextOwnerFlag=0,     ///< [in ]: 0 = don't copy m_pNextMap when constructed, don't delete m_pNextMap when destructed         <br>
                                            ///<      : 1 = copy m_pNextMap when constructed, delete m_pNextMap when destructed                     <br>
                                            ///<      : 2 = don't copy m_pNextMap when constructed, delete m_pNextMap when destructed               <br>
                                            ///<      : default:[0]                                                                                 <br>
    SmTransform     * pOrientMap=NULL,      ///< [in ]: Opt Orient Map. NULL=IdentityMatrix. default:[NULL]                                         <br>
    ULONG             lOrientOwnerFlag=0    ///< [in ]: 0 = don't copy m_pOrientMap when constructed, don't delete m_pOrientMap when destructed     <br>
                                            ///<      :  1 = copy m_pOrientMap when constructed, delete m_pOrientMap when destructed                <br>
                                            ///<      :  2 = don't copy m_pOrientMap when constructed, delete m_pOrientMap when destructed          <br>
                                            ///<      :  default:[0]                                                                                <br>
  );


  // make an exact copy of any volume with or without any compounding volumes
  virtual SmStatus Copy
  (
    const SmContext & crContext,            ///< [in ]: context for new object construction                          <br>
    SmVolume       *& rpNewVolume,          ///< [out]: The copied Volume                                            <br>
    SmBoolean         bSimpleMapOnly=FALSE  ///< [in ]: TRUE = Copy this Volume omitting any compounding volumes     <br>
                                            ///<      : FALSE= Copy this Volumes with any compounding volumes        <br>
  ) const                                    
  { 
    SM_REF3(crContext, rpNewVolume, bSimpleMapOnly) ; SE(SM_ERR) ; return SM_ERR ; 
    // derived methods need to add
    // Notify(SM_NO_COPY, rpNewVolume, this) ;
  }

  // assignment operator
  SmVolume & operator=(const SmVolume &crVolume) ;

  // equality operator
  virtual SmBoolean operator==(const SmVolume&) const;

  // Notify
  virtual void Notify
  (
    SmNotifyOperation eNotifyOperation,
    SmObject * pData1,
    SmObject * pData2,
    SmObject * pData3
  ) ;
                   
  // destructor                                  
  virtual ~SmVolume() ;

  // simple data access                             
  SmObject     * GetOwner()        const               { return m_pOwner ; }   // not yet used
  SmVolume     * GetNextMap()      const               { return m_pNextMap ; }
  SmVolume     * GetLastMap()      const               { if(m_pNextMap) { if(m_pNextMap->m_pNextMap) { return(m_pNextMap->GetLastMap()) ; }
                                                                          else                       { return(m_pNextMap) ; }
                                                                        }
                                                         else           { return(NULL) ; }
                                                       }

  SmTransform  * GetOrientMap()    const               { return m_pOrientMap ; }
  SmVector3d     GetOrientOrigin() const ;             // rtn : InSpace Orient origin point
  SmVector3d     GetOrientXAxis()  const ;             // rtn : InSpace Orient X Axis vector
  SmVector3d     GetOrientYAxis()  const ;             // rtn : InSpace Orient Y Axis vector
  SmVector3d     GetOrientZAxis()  const ;             // rtn : InSpace Orient Z Axis vector
  SmTransform  * GetInvOrientMap() const               { return m_pInvOrientMap ; }
               
  SmObject     * SetOwner(SmObject *pNewOwner)         // eff: set owner value and return previous value
                                                       { SmObject *pRtn = m_pOwner ;
                                                         m_pOwner = pNewOwner ;
                                                         // Notify(SM_NO_CHANGE_OWNER, pNewOwner, pNewOwner) ;
                                                         return(pRtn) ;             
                                                       }
               
  // Set the compound map (replacing any existing one) so that, OutPoint = NextMap(ThisMapSimple(InPoint)) = ThisMap(InPoint)
  SmStatus SetNextMap
  (
    SmVolume *pNextMap,            ///< [in ]: replacement compounding Map                                                             <br>
    ULONG     lOwnerFlag=0         ///< [in ]: 0 = don't copy m_pNextMap when constructed, don't delete m_pNextMap when destructed     <br>
                                   ///<      : 1 = copy m_pNextMap when constructed, delete m_pNextMap when destructed                 <br>
                                   ///<      : 2 = don't copy m_pNextMap when constructed, delete m_pNextMap when destructed           <br>
                                   ///<      : default:[0]                                                                             <br>
  );
                                                     
  // Add a compound map to current compound map sequence so that, OutPoint = NextMap(ThisMap(InPoint))      
  //   where ThisMap may or may not already be a sequence of compound evaluations.                                                                                                                       
  SmStatus AddNextMap
  (                                                                                                                        
    SmVolume *pNextMap,            ///< [in ]: next level of concatenated-compound Mapping                                               <br>
    ULONG     lOwnerFlag=0         ///< [in ]: 0 = don't copy m_pNextMap when constructed, don't delete m_pNextMap when destructed       <br>
                                   ///<      : 1 = copy m_pNextMap when constructed, delete m_pNextMap when destructed                   <br>
                                   ///<      : 2 = don't copy m_pNextMap when constructed, delete m_pNextMap when destructed             <br>
                                   ///<      : default:[0]                                                                               <br>
  );
                                                     
  // Set the Orient map (replacing any existing one) so that, InPoint = OrientMap(ParamPoint)
  SmStatus SetOrientMap
  (
    SmTransform *pOrientMap,      ///< [in ]: replacement compounding Map                                                                <br>
    ULONG         lOwnerFlag=0    ///< [in ]: 0 = don't copy m_pOrientMap when constructed, don't delete m_pOrientMap when destructed    <br>
                                  ///<      : 1 = copy m_pOrientMap when constructed, delete m_pOrientMap when destructed                <br>
                                  ///<      : 2 = don't copy m_pOrientMap when constructed, delete m_pOrientMap when destructed          <br>
                                  ///<      : default:[0]                                                                                <br>
  );

  // Compute and save the m_pInvOrientMap from the current m_pOrientMap 
  SmStatus RefreshInvOrientMap() ; 

  // Make a SmVolume a simple unoriented map by ensuring its m_pNextMap, m_pOrientMap, and m_pInvOrientMaps are all NULL.
  SmStatus  MakeMapSimple() ;
  SmBoolean IsMapSimple()                            { return(m_pNextMap == NULL && m_pOrientMap == NULL) ; }
                                                     
  // get approximate size of mapping (can be bounded or unbounded)
#define ApproximateUVWSize ApproximateParamSize 

  SmStatus ApproximateParamSize   (SmVector3d & rUVWSize) const ;
  SmStatus ApproximateInSpaceSize (SmVector3d & rInSize)  const ;
  SmStatus ApproximateOutSpaceSize(SmVector3d & rOutSize) const ;

  // gwc: I don't think we'll support these - mapping tolerances from space to space is done in SmTol without these methods
  //      // estimate derivate size average from ParamSpace to InSpace
  //      virtual SmVector3d ApproxInDerivativeLengths( const SmExtent3d & crParamDomain ) const ;
  //      
  //      // estimate derivate size average from InSpace to OutSpace
  //      virtual SmVector3d ApproxOutDerivativeLengths( const SmExtent3d & crInBoundingBox ) const ;

  // valid parameter domains for mappings - many are infinite, SmBSplineVolume is finite, SmBendVolume is a half space.
#define GetNaturalUVWDomain GetNaturalParamDomain 

  // get max sized ParamDomain definition - depending on the derived type may include internal discontinuities (ex: SmUnbendVolume)
  virtual SmExtent3d  GetNaturalParamDomain() const   { SE_MSG(SM_ERR,_T("Called pure virtual method")); return SmExtent3d(SmPoint3d(0,0,0), SmPoint3d(0,0,0)) ; }

  // return a bounded ParamDomain that avoids all internal discontinuities - used for graphics
  virtual SmExtent3d  GetDiscontinuityFreeParamDomain() const { return  GetNaturalParamDomain() ; } 

          SmPseudoBox GetNaturalInSpaceDomain() const ;
          
  // not all parameter domains can be represented by boxes - Test points to see if they are within natural domains
  SmBoolean IsPointInParamDomain   
  (
    const SmPoint3d & crParamPoint,       ///< [in ]: tgt ParamPoint                                  <br>
    SmBoolean bWithCompounding = TRUE     ///< [in ]: TRUE = OutSpacePoint is in LastOutSpace         <br>
                                          ///<      : FALSE= OutSpacePoint is in 1stOutSpace          <br>
  ) const ;

  SmBoolean IsPointInInSpaceDomain 
  (
    const SmPoint3d & crInSpacePoint,      ///< [in ]: target InSpace Point                                                             <br>
    SmPoint3d * pOptParamPoint = NULL,     ///< [out]: InvOriented ParamPoint when return is TRUE, NULL to ignore, default:[NULL]       <br>
    SmBoolean bWithCompounding = TRUE      ///< NotUsed: [in ]: TRUE = OutSpacePoint is in LastOutSpace                                          <br>
                                           ///<      : FALSE= OutSpacePoint is in 1stOutSpace                                           <br>
  ) const ;

  SmBoolean IsPointInOutSpaceDomain
  (
    const SmPoint3d & crOutSpacePoint,     ///< [in ]: OutSpace Point to test                                                           <br>
    SmPoint3d * pOptParamPoint = NULL,     ///< [out]: inverted ParamPoint when return is TRUE, NULL to ignore, default:[NULL]          <br>
    SmBoolean bWithCompounding = TRUE      ///< [in ]: TRUE = OutSpacePoint is in LastOutSpace                                          <br>
                                           ///<      : FALSE= OutSpacePoint is in 1stOutSpace                                           <br>
  ) const ;

  SmBoolean IsLineInParamDomain   
  (
    const SmPoint3d & crStartParamPoint,    ///< [in ]: tgt ParamLine Start Point                 <br>
    const SmPoint3d & crEndParamPoint,      ///< [in ]: tgt ParamLine Start Point                 <br>
    SmBoolean bWithCompounding = TRUE       ///< [in ]: TRUE = OutSpacePoint is in LastOutSpace   <br>
                                            ///<      : FALSE= OutSpacePoint is in 1stOutSpace    <br>
  ) const ;

  SmBoolean IsLineInInSpaceDomain 
  (
    const SmPoint3d & crStartInSpacePoint,  ///< [in ]: target InSpace Line Start Point                                                    <br>
    const SmPoint3d & crEndInSpacePoint,    ///< [in ]: target InSpace Line End Point                                                      <br>
    SmPoint3d * pOptStartParamPoint = NULL, ///< [out]: InvOriented StartParamPoint when return is TRUE, NULL to ignore, default:[NULL]    <br>
    SmPoint3d * pOptEndParamPoint = NULL,   ///< [out]: InvOriented EndParamPoint when return is TRUE, NULL to ignore, default:[NULL]      <br>
    SmBoolean bWithCompounding = TRUE       ///< [in ]: TRUE = OutSpaceLine is in LastOutSpace                                             <br>
                                            ///<      : FALSE= OutSpaceLine is in 1stOutSpace                                              <br>
  ) const ;
  
  // ToBe implemented: base implementations on CrvInVolumes tested with IsCurveInInSpaceDomain 
  //  SmBoolean IsLineInOutSpaceDomain(const SmPoint3d & crStartOutSpacePoint, const SmPoint3d & crEndOutSpacePoint, SmPoint3d * pOptParamPoint=NULL, SmBoolean bWithCompounding = TRUE) const ;

  SmBoolean IsPseudoBoxInParamDomain   
  (
    const SmPseudoBox & crParamPseudoBox,    ///< [in ]: ParamBox to test                               <br>
    SmBoolean bWithCompounding = TRUE        ///< [in ]: TRUE = OutSpacePoint is in LastOutSpace        <br>
                                             ///<      : FALSE= OutSpacePoint is in 1stOutSpace         <br>
  ) const ;

  SmBoolean IsPseudoBoxInInSpaceDomain 
  (
    const SmPseudoBox & crInSpacePseudoBox,   ///< [in ]: InSpace Box to check                             <br>
    SmBoolean bWithCompounding = TRUE         ///< [in ]: TRUE = OutSpacePoint is in LastOutSpace          <br>
                                              ///<      :  FALSE= OutSpacePoint is in 1stOutSpace          <br>
  ) const ;

  SmBoolean IsPseudoBoxInOutSpaceDomain
  (
    const SmPseudoBox & crOutSpacePseudoBox,  ///< [in ]: OutSpace Box to test                             <br>
    SmBoolean bWithCompounding = TRUE         ///< [in ]: TRUE = OutSpacePoint is in LastOutSpace          <br>
                                              ///<      : FALSE= OutSpacePoint is in 1stOutSpace           <br>
  ) const ;

  SmBoolean IsBoxInParamDomain   
  (
    const SmExtent3d & crParamBox,            ///< [in ]: ParamBox to test                                <br>
    SmBoolean bWithCompounding = TRUE         ///< [in ]: TRUE = OutSpacePoint is in LastOutSpace         <br>
                                              ///<      : FALSE= OutSpacePoint is in 1stOutSpace          <br>
  ) const ;

  SmBoolean IsBoxInInSpaceDomain 
  (
    const SmExtent3d & crInSpaceBox,         ///< [in ]: InSpace Box to check                           <br>
    SmBoolean bWithCompounding = TRUE        ///< [in ]: TRUE = OutSpacePoint is in LastOutSpace        <br>
                                             ///<      : FALSE= OutSpacePoint is in 1stOutSpace         <br>
  ) const ;

  SmBoolean IsBoxInOutSpaceDomain
  (
    const SmExtent3d & crOutSpaceBox,       ///< [in ]: OutSpace Box to test                             <br>
    SmBoolean bWithCompounding = TRUE       ///< [in ]: TRUE = OutSpacePoint is in LastOutSpace          <br>
                                            ///<      : FALSE= OutSpacePoint is in 1stOutSpace           <br>
  ) const ;

  SmBoolean IsCurveInParamDomain  
  (
    const SmCurve & crParamCurve,           ///< [in ]: tgt ParamCurve                                           <br>
    SmBoolean bWithCompounding = TRUE       ///< [in ]: TRUE = Check this and all compounded ParamSpaceDomains   <br>
                                            ///<      : FALSE= Check only this ParamSpaceDomain                  <br>
  ) const ;          

  SmBoolean IsCurveInInSpaceDomain
  (
    const SmCurve & crInSpaceCurve,         ///< [in ]: target InSpace Curve                                      <br>
    SmBoolean bWithCompounding = TRUE       ///< [in ]: TRUE = Check this and all compounded ParamSpaceDomains    <br>
                                            ///<      : FALSE= Check only this ParamSpaceDomain                   <br>
  ) const ;      

  SmBoolean IsSurfaceInParamDomain  
  (
    const SmSurface & crParamSurface,       ///< [in ]: tgt ParamSurface                                       <br>
    SmBoolean bWithCompounding = TRUE       ///< [in ]: TRUE = Check this and all compounded ParamSpaceDomains <br>
                                            ///<      : FALSE= Check only this ParamSpaceDomain                <br>
  ) const ;     

  SmBoolean IsSurfaceInInSpaceDomain
  (
    const SmSurface & crInSpaceSurface,     ///< [in ]: target InSpace Surface                                   <br>
    SmBoolean bWithCompounding = TRUE       ///< [in ]: TRUE = Check this and all compounded ParamSpaceDomains   <br>
                                            ///<      : FALSE= Check only this ParamSpaceDomain                  <br>
  ) const ;
                                                        
  // Find portion of InSpace Line interval that maps inside the NaturalParamDomain 
  //   GWC: in the future add compounding which will require a general curve intersection solution
  //        at that time add a bWithCompounding flag to this signature.
  SmStatus FindParamIntervalForInSpaceLine                                     // rtn: SM_ERR for NULL line, else SM_SUCCESS
  (
    const SmPoint3d      & crInSpacePoint,          ///< [in ]: LinePoint         of Line(s) = LinePoint + s * LineVector          <br>     
    const SmVector3d     & crInSpaceVector,         ///< [in ]: scaled LineVector of Line(s) = LinePoint + s * LineVector          <br>     
    const SmExtent1d     & crCurrentIvl,            ///< [in ]: current limits on s interval                                       <br>
    SmTArray<SmExtent1d> & rTrimIvls                ///< [out]: Interval of line that maps legally within the NaturalParamDomains  <br>
  ) const                                            
  { 
    return( FindParamIntervalForInSpaceLineSimple(crInSpacePoint, crInSpaceVector,crCurrentIvl, rTrimIvls) ) ;
  }

  // GWC: exclude FindParamExtentForInSpacePlane from first release. Code is written but I don't
  //      see the bug in SmBendVolume::FindParamExtentForInSpacePlaneSimple().  I'll fix that later.
  //      // Find the largest square extent of an InSpace plane that maps inside the NaturalParamDomain.
  //      //    non-square PlanePoints may also be within the NaturalParamDomain. This method only finds a 
  //      //    plane subset totally inside the NaturalParamDomain, it does not find the set of all inside points.
  //      SmStatus FindParamExtentForInSpacePlane                                      // rtn: SM_ERR for NULL plane, else SM_SUCCESS
  //                                  (const SmPoint3d      & crInSpacePoint,          ///< [in ]: PlanePoint          of Plane(s) = PlanePoint + u * PlaneVecU + v * PlaneVecV
  //                                   const SmVector3d     & crInSpaceVecU,           ///< [in ]: scaled PlaneVectorU of Plane(s) = PlanePoint + u * PlaneVecU + v * PlaneVecV              
  //                                   const SmVector3d     & crInSpaceVecV,           ///< [in ]: scaled PlaneVectorV of Plane(s) = PlanePoint + u * PlaneVecU + v * PlaneVecV              
  //                                   const SmExtent2d     & crCurrentUV,             ///< [in ]: current limits on UV extent
  //                                   SmTArray<SmExtent2d> & rTrimUVs)                ///< [out]: UVExtent of plane that maps legally within the NaturalParamDomains
  //                                  const                                            { return( FindParamExtentForInSpacePlaneSimple              
  //                                                                                                       (crInSpacePoint, crInSpaceVecU, crInSpaceVecV,
  //                                                                                                        crCurrentUV, rTrimUVs) ) ;                      
  //                                                                                   }

  // Get InSpace and/or OutSpace BBoxes for a given ParamSpace BBox 
  SmStatus EvaluateBoundingBox
  (
    const SmExtent3d & crParamBox,                  ///< [in ]: ParamSpace box to proj to OutSpace and/or InSpace                                                                                               <br>
    SmPseudoBox      * pOptParamPseudoBox = NULL,   ///< [in ]: used to project output PseudoBox orientations, NULL to orient on data, default:[NULL]                                                           <br>
    SmExtent3d       * pOutSpaceBox       = NULL,   ///< [out]: OutSpace Box projected from ParamSpace Box, NULL to ignore, default:[NULL]                                                                      <br>
    SmPseudoBox      * pOutSpacePseudoBox = NULL,   ///< [out]: OutSpace PseudoBox circumscribing Box projected from ParamSpace (with OptPseudoParamBox projected orientation), NULL to ignore, default:[NULL]  <br>
    SmExtent3d       * pInSpaceBox        = NULL,   ///< [out]: InSpace Box projected from ParamSpace Box, NULL to ignore, default:[NULL]                                                                       <br>
    SmPseudoBox      * pInSpacePseudoBox  = NULL,   ///< [out]: InSpace PseudoBox (with OptPseudoParamBox projected orientation), NULL to ignore, default:[NULL]                                                <br>
    SmBoolean          bWithCompounding   = TRUE    ///< [in ]: TRUE  = project to last OutSpace                                                                                                                <br>
                                                    ///<      : FALSE = project to first OutSpace                                                                                                               <br>
                                                    ///<      : default:[TRUE]                                                                                                                                  <br>
  ) const;


  // Get InSpace and/or OutSpace PseudoBBoxes for a given ParamSpace PseudoBox 
  SmStatus EvaluatePseudoBox
  (
    const SmPseudoBox & crParamPseudoBox,            ///< [in ]: ParamSpace Pseudo box to proj to OutSpace and/or InSpace                                                                                        <br>
    SmExtent3d        * pOutSpaceBox       = NULL,   ///< [out]: OutSpace Box projected from ParamSpace Box, NULL to ignore, default:[NULL]                                                                      <br>
    SmPseudoBox       * pOutSpacePseudoBox = NULL,   ///< [out]: OutSpace PseudoBox circumscribing Box projected from ParamSpace (with OptPseudoParamBox projected orientation), NULL to ignore, default:[NULL]  <br>
    SmExtent3d        * pInSpaceBox        = NULL,   ///< [out]: InSpace Box projected from ParamSpace Box, NULL to ignore, default:[NULL]                                                                       <br>
    SmPseudoBox       * pInSpacePseudoBox  = NULL,   ///< [out]: InSpace PseudoBox (with OptPseudoParamBox projected orientation), NULL to ignore, default:[NULL]                                                <br>
    SmBoolean           bWithCompounding   = TRUE    ///< [in ]: TRUE  = project to last OutSpace                                                                                                                <br>
                                                     ///<      : FALSE = project to first OutSpace                                                                                                               <br>
                                                     ///<      : default:[TRUE]                                                                                                                                  <br>
  ) const;

  // Get OutSpace BBoxes for a given InSpace BBox
  SmStatus MapBoundingBox
  (
    const SmExtent3d  & crInSpaceBox,                     ///< [in ]: InSpace box to proj to OutSpace                                                                     <br>
    SmExtent3d        * pOutSpaceBox         = NULL,      ///< [out]: OutSpace Box projected from InSpace Box, NULL to ignore, default:[NULL]                             <br>
    SmPseudoBox       * pOutSpacePseudoBox   = NULL,      ///< [out]: OutSpace PseudoBox (with OptPseudoParamBox projected orientation), NULL to ignore, default:[NULL]   <br>
    SmBoolean           bWithCompounding     = TRUE,      ///< [in ]: TRUE = project to last OutSpace, FALSE = project to first OutSpace                                  <br>
                                                          ///<      : default:[TRUE]                                                                                      <br>
    SmBoolean           bTrimToLegalParamSpace = FALSE    ///< [in ]: TRUE  = Trim ParamSpaceBox to LegalParamDomain                                                      <br>
                                                          ///<      : FALSE = Report boxes that need trimming as errors.                                                  <br>
                                                          ///<      : default:[FALSE]                                                                                     <br>
  ) const
  { 
    SmPseudoBox sInSpacePseudoBox(crInSpaceBox) ;
    return(MapPseudoBox(sInSpacePseudoBox, pOutSpaceBox, pOutSpacePseudoBox, bWithCompounding, bTrimToLegalParamSpace)) ;
  }

  // Get OutSpace BBoxes for a given InSpace BBox
  SmStatus MapPseudoBox
  (
    const SmPseudoBox & crInSpaceBox,                     ///< [in ]: InSpace box to proj to OutSpace                                                                    <br>
    SmExtent3d        * pOutSpaceBox         = NULL,      ///< [out]: OutSpace Box projected from InSpace Box, NULL to ignore, default:[NULL]                            <br>
    SmPseudoBox       * pOutSpacePseudoBox   = NULL,      ///< [out]: OutSpace PseudoBox (with OptPseudoParamBox projected orientation), NULL to ignore, default:[NULL]  <br>  
    SmBoolean           bWithCompounding     = TRUE,      ///< [in ]: TRUE = project to last OutSpace, FALSE = project to first OutSpace                                 <br>
                                                          ///<      : default:[TRUE]                                                                                     <br>
    SmBoolean           bTrimToLegalParamSpace = FALSE    ///< [in ]: TRUE  = Trim ParamSpaceBox to LegalParamDomain                                                     <br>
                                                          ///<      : FALSE = Report boxes that need trimming as errors.                                                 <br>
                                                          ///<      : default:[FALSE]                                                                                    <br>
  ) const;

  // Trim a ParamSpace BBox to Natural ParamSpace Domain
  SmStatus TrimParamBoundingBox                                        // rtn: SM_ERR when ParamBox trims to empty set, else SM_SUCCESS
  (
    const SmExtent3d & crParamBox,                       ///< [in ]: Tgt Param Box to trim                           <br>
    SmExtent3d       & rParamTrimBox                     ///< [in ]: Box trimmed to Natural Param Domain             <br>
  ) const                                                 
  { 
    return( TrimParamBoundingBoxSimple(crParamBox, rParamTrimBox) ) ; 
  }

//     // GWC - removed from interface because it made no sense: Even very simple cases of trimming a BBox to a long skinny PseduoBox showed
//     //       that the inscribed Box could be very small and non-unique.  It won't necessarily capture the notion of a good
//     //       bounding box designed to help narrow down the space of options for functions that need to check that.  In general,
//     //       check wether individual InSpace and ProjPace Points map back to legal ParamDomain Points, one point at a time. 
//     // // Trim an InSpace BBox so that it maps totally within the Natural ParamSpace Domain
          //SmStatus TrimInSpaceBoundingBox                                      // rtn: SM_ERR when InSpaceBox trims to empty set, else SM_SUCCESS
          //               (const SmExtent3d & crInSpaceBox,                     ///< [in ]: Tgt InSpace Box to trim              
          //                SmExtent3d       & rInSpaceTrimBox)                  ///< [in ]: Box trimmed so that it maps totally within the Natural ParamSpace Domain
          //               const                                                 { return( TrimInSpaceBoundingBoxSimple(crInSpaceBox, rInSpaceTrimBox) ) ; }

  // Create InSpace and/or OutSpace IsoCurves from a ParamSpace IsoParamLine
  SmStatus EvaluateIsoParametricCurve
  (
    const SmContext     & crContext,                     ///< [in ]: context for new created objects                                           <br>
    SmVolumeParamsType    eConstantParams,               ///< [in ]: specify constant parameters                                               <br>
    double                dIsoParameter1,                ///< [in ]: 1st constant parameter value                                              <br>
    double                dIsoParameter2,                ///< [in ]: 2nd constant parameter value                                              <br>
    double                d3DTolerance,                  ///< [in ]: max allowed 3d approximation distance                                     <br>
    SmCurve            ** ppNewOutSpaceIsoCurve = NULL,  ///< [out]: OutSpace IsoCurve, NULL to ignore, default:[NULL]                         <br>
    SmCurve            ** ppNewInSpaceIsoCurve = NULL,   ///< [out]: InSpace IsoCurve, NULL to ignore, default:[NULL]                          <br>
    const SmExtent3d    * pOptParamTrimBox = NULL,       ///< [in ]: opt ParamSpace TrimBox, NULL=Use NaturalBBox, default:[NULL]              <br>
    SmBoolean             bWithCompounding = TRUE        ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE] <br>
  ) const ;

  // Create OutSpace IsoCurve from a InSpace IsoParamLine
  SmStatus MapIsoParametricCurve
  (
    const SmContext     & crContext,                     ///< [in ]: context for new created objects                                            <br>
    SmVolumeParamsType    eConstantParams,               ///< [in ]: specify constant parameters                                                <br>
    double                dIsoParameter1,                ///< [in ]: 1st constant parameter value                                               <br>
    double                dIsoParameter2,                ///< [in ]: 2nd constant parameter value                                               <br>
    double                d3DTolerance,                  ///< [in ]: max allowed 3d approximation distance                                      <br>
    SmCurve            *& rpNewOutSpaceIsoCurve,         ///< [out]: OutSpace IsoCurve                                                          <br>
    const SmPseudoBox   * pOptInSpaceTrimBox = NULL,     ///< [in ]: opt InSpace trim box for the IsoCurve - used when m_pOrient != NULL        <br>
                                                         ///<      : NULL=natural BoundingBox or default subVolume for infinite ParamDomains    <br>
                                                         ///<      : default:[NULL]                                                             <br>
    const SmExtent3d    * pOptParamTrimBox = NULL,       ///< [in ]: opt ParamSpace trim box for the IsoCurve - used when m_pOrient == NULL     <br>
                                                         ///<      : default:[NULL]                                                             <br>
    SmBoolean             bWithCompounding = TRUE        ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]  <br>
 ) const ;
  
  // Create InSpace and/or OutSpace IsoSurfaces from a ParamSpace IsoParamPlane
  SmStatus EvaluateIsoParametricSurface
  (
    const SmContext    & crContext,                       ///< [in ]: context for created objects                                                <br>
    SmVolumeParamType    eConstantParam,                  ///< [in ]: specify the IsoPlane constant parameter                                    <br>
    double               dIsoParameter,                   ///< [in ]: specify the constant ParamSpace parameter value                            <br>
    double               d3DTolerance,                    ///< [in ]: max allowed 3d approximation distance                                      <br>
    SmSurface         ** ppNewOutSpaceIsoSurface = NULL,  ///< [out]: OutSpace IsoSurface, NULL to ignore, default:[NULL]                        <br>
    SmSurface         ** ppNewInSpaceIsoSurface = NULL,   ///< [out]: InSpace IsoSurface, NULL to ignore, default:[NULL]                         <br>
    const SmExtent3d   * pOptParamTrimBox = NULL,         ///< [in ]: opt ParamSpace TrimBox, NULL=Use NaturalBBox, default:[NULL]               <br>
    SmBoolean            bWithCompounding = TRUE          ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]  <br>
  ) const ;                                                                                                                           

  // for first release - exclude SmVolume::MapIsoParametricSurface method.  All the code is built but there 
  //  are bugs I just don't see in the complicated trimming function, SmBendVolume::FindParamExtentForInSpacePlaneSimple()
  //  that will be looked at later. 
  //      // Create OutSpace IsoSurface from a InSpace IsoParamLine
  //      SmStatus MapIsoParametricSurface
  //                     (const SmContext     & crContext,                     ///< [in ]: context for new created objects
  //                      SmVolumeParamType     eConstantParam,                ///< [in ]: specify the IsoPlane constant parameter
  //                      double                dIsoParameter,                 ///< [in ]: specify the constant InSpace parameter value
  //                      double                d3DTolerance,                  ///< [in ]: max allowed 3d approximation distance
  //                      SmSurface          *& rpNewOutSpaceIsoSurface,       ///< [out]: OutSpace IsoSurface
  //                      const SmPseudoBox   * pOptInSpaceTrimBox = NULL,     ///< [in ]: opt InSpace TrimBox, NULL=Use NaturalBBox, default:[NULL]
  //                      SmBoolean             bWithCompounding = TRUE)       ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]
  //                     const ;
  
  // user interface to Make Exact LastOutSpace(compounding) BSpline geometry from InSpace and ParamSpace geometry when possible
  SmStatus MakeExactBSplineOutCurveFromInSpace     
  (
    const SmCurve   & rInSpaceCurve,     ///< [in ]: Tgt Curve to project to Last OutSpace as an exact curve if possible                              <br>
    SmBSplineCurve *& rpNewCurve )       ///< [in ]: LastOutSpace exact projection of rInSpaceCurve, context:[rInSpaceCurve.GetContext()]             <br>
    const ;

  SmStatus MakeExactBSplineOutCurveFromParamSpace  
  (
    const SmCurve   & rParamSpaceCurve,   ///< [in ]: Tgt Curve to project to Last OutSpace as an exact curve if possible                              <br>
    SmBSplineCurve *& rpNewCurve          ///< [in ]: LastOutSpace exact projection of rParamSpaceCurve, context:[rParamSpaceCurve.GetContext()]       <br>
  ) const ;

  SmStatus MakeExactBSplineOutSurfaceFromInSpace   
  (
    const SmSurface & rInSpaceSurface,    ///< [in ]: Tgt Surface to project to Last OutSpace as an exact curve if possible                             <br>
    SmBSplineSurface *& rpNewSurface      ///< [in ]: LastOutSpace exact projection of rInSpaceSurface, context:[rInSpaceSurface.GetContext()]          <br>
  ) const ;

  SmStatus MakeExactBSplineOutSurfaceFromParamSpace
  (
    const SmSurface & rParamSpaceSurface,  ///< [in ]: Tgt Surface to project to Last OutSpace as an exact curve if possible                             <br>
    SmBSplineSurface *& rpNewSurface       ///< [in ]: LastOutSpace exact projection of rParamSpaceSurface, context:[rParamSpaceSurface.GetContext()]    <br>
  ) const ;

  // virtual methods needed by Derived classes to Make Exact 1st OutSpace BSplineGeometry from InSpace and ParamSpace geometry when possible  
  // make exact BSplineCurve projection of InputSpace Curve to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutCurve
  (
    SmVolumeSpaceTYPE eInputSpace,       ///< [in ]: SM_VS_IN_SPACE   = InputCurve is projected from InSpace                                            <br>
                                         ///<      : SM_VS_PARAM_SPACE= InputCurve is projected from ParamSpace                                         <br>
    const SmCurve   & rInputCurve,       ///< [in ]: Curve to project to Last OutSpace                                                                  <br>
    SmBSplineCurve *& rpNewCurve         ///< [out]: 1st OutSpace projection or NULL for not possible                                                   <br>
  ) const                                 
  { 
    SM_REF2(eInputSpace, rInputCurve) ;
    rpNewCurve = NULL ; 
    return(SM_SUCCESS) ;  
  }
                                                                      
  // make exact BSplineCurve projection of InputSpace Surface to 1st OutSpace if possible
  virtual SmStatus MakeExactBSpline1stOutSurface
  (
    SmVolumeSpaceTYPE   eInputSpace,   ///< [in ]: SM_VS_IN_SPACE   = InputSurface is projected from InSpace            <br>
                                       ///<      : SM_VS_PARAM_SPACE= InputSurface is projected from ParamSpace         <br>
    const SmSurface   & rInputSurface, ///< [in ]: Surface to project to Last OutSpace                                  <br>
    SmBSplineSurface *& rpNewSurface   ///< [out]: 1st OutSpace projection or NULL for not possible                     <br>
  ) const                               
  { 
    SM_REF2(eInputSpace, rInputSurface) ;
    rpNewSurface   = NULL ; 
    return(SM_SUCCESS) ; 
  } 
                                                               
  // Evaluate Section: VOLUME MAPPING DIAGRAM                                          
  //   +---------------+                                 +------------------+
  //   |   InSpace     | -------------'Map'------------> |     OutSpace     |
  //   | [xIn yIn zIn] | <----------'InvMap'------------ | [xOut yOut zOut] |
  //   +---------------+                              >  +------------------+     
  //       ^     |                             >      <        ^     |        
  //       | 'InvOrient'     'Evaluate' >      <               | 'InvOrient'                    
  //   'Orient'  |               >      <                  'Orient'  |                       
  //       |     v        >      < 'InvEvaluate'               |     v             
  //   +---------------+  <                              +------------------+
  //   |  ParamSpace   | -------'EvaluateSimple'-------> |    ProjSpace     |   
  //   |   [u v w]     | <-----'InvEvaluateSimple'------ |                  |     
  //   +---------------+  [  GlobalPointSolveSimple   ]  +------------------+     
  //                      [InvEvaluateGuessPointSimple] 
  //                      [   LocalPointSolveSimple   ]       

  // Sizing and Indexing into Evaluate(), Map(), Orient() output arrays by partial deriv cnts.
  // examples: aDerivs[SM_VSIZE(MaxDeriv)], 
  //           lHighestDeriv=MaxDeriv,    // SM_VI macros hard code name "lHighestDeriv"
  //     sW   = aDerivs[SM_VI(0,0,0)],  - position                            // when MaxDeriv is named 'lHighestDeriv'
  //     sWuu = aDerivs[SM_VI(2,0,0)],  - 2nd deriv in U direction            // 
  //     sWv  = aDerivs[SM_VI(0,1,0)],  - 1st deriv in V direction = TangentV // 
  //     sWuw = aDerivs[SM_VI(1,0,1)],  - cross deriv in UW direction         // 
  //  or sW   = aDerivs[SM_VM(0,0,0,MaxDeriv)],                               // when MaxDeriv is not named 'lHighestDeriv'
  //     sWv  = aDerivs[SM_V1(0,1,0)],                                        // when MaxDeriv is known to equal 1
  //     sWuu = aDerivs[SM_V2(2,0,0)],                                        // when MaxDeriv is known to equal 2
  //     sWuw = aDerivs[SM_V3(1,0,1)],  - cross deriv in UW direction         // when MaxDeriv is known to equal 3

  // Size of output array for Evaluate(), Map(), and Orient() methods
  #define SM_VSIZE(MaxDeriv)    (((MaxDeriv)+1)*((MaxDeriv)+1)*((MaxDeriv)+1))

  // indexing into output arrays for Evaluate(), Map(), and Orient() methods
  #define SM_VI(u,v,w)          (((u)*(lHighestDeriv+1)+(v))*(lHighestDeriv+1)+(w)) // when MaxDeriv is named 'lHighestDeriv'
  #define SM_VM(u,v,w,MaxDeriv) (((u)*((MaxDeriv)+1)+(v))*((MaxDeriv)+1)+(w))       // when MaxDeriv is not named 'lHighestDeriv'
  #define SM_V1(u,v,w)          (((u)*(2)+(v))*(2)+(w))                             // when MaxDeriv = 1
  #define SM_V2(u,v,w)          (((u)*(3)+(v))*(3)+(w))                             // when MaxDeriv = 2
  #define SM_V3(u,v,w)          (((u)*(4)+(v))*(4)+(w))                             // when MaxDeriv = 3
                                                              
  // Compute OutPoint = Evaluate(ParamPoint) (in final outspace with compounding) 
  virtual SmStatus Evaluate
  (
    const SmPoint3d & crParamPoint,     ///< [in ]: ParamSpace point to OutSpace point                                              <br>
    ULONG             lHighestDeriv,    ///< [in ]: 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                        <br>
    SmBoolean         bUFromLeft,       ///< [in ]: if P is on U, V, or W interval boundary                                         <br>
    SmBoolean         bVFromLeft,       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval     <br>
    SmBoolean         bWFromLeft,       ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval    <br>
    SmVector3d      * aDerivatives,     ///< [out]: matrix of OutSpace evaluations values                                           <br>
                                        ///<      : 3d organized: D[u][v][w]                                                        <br>
                                        ///<      : 1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3              <br>
                                        ///<      : sized       : [n+1][n+1][n+1], where n=lHighesDeriv                             <br>
                                        ///<      : lHghDrv = 0,   sized: [1],                                                      <br>
                                        ///<      :   i=0          order: [D]                                                       <br>
                                        ///<      : lHghDrv = 1,   sized: [8]                                                       <br>
                                        ///<      :   i=u*4+v*2+w  order: [D  Dw  Dv  ---                                           <br>
                                        ///<      :                        Du --- --- ---]                                          <br>
                                        ///<      : lHghDrv = 2,   sized: [27]                                                      <br>
                                        ///<      :   i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---                      <br>
                                        ///<      :                        Du  Duw --- Duv --- --- --- --- ---                      <br>
                                        ///<      :                        Duu --- --- --- --- --- --- --- ---]                     <br>
                                        ///<      : lHghDrv = 3,   sized: [81]                                                      <br>
                                        ///<      :   i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw        <br>
                                        ///<      :                        --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---        <br>
                                        ///<      :                        Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---        <br>
                                        ///<      :                        --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---        <br>
                                        ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  Duuu ---        <br>
                                        ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---        <br>
                                        ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---        <br>
                                        ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---        <br>
                                        ///<      :                        --- --- ]                                                <br>
    SmBoolean bNonZeroTangents=TRUE,    ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors    <br>
                                        ///<      : FALSE= return exact tangent values, default:[TRUE]                              <br>
                                        ///<      : note: Surprisingly TRUE is the common choice because most tangent uses          <br>
                                        ///<      :       are for their direction (Binorm, SurfNorm comps), but when the            <br>
                                        ///<      :       tangent is being used for its magnitude (like an arc-length comp)         <br>
                                        ///<      :       then set this to FALSE.                                                   <br>
                                        ///<      : default:[TRUE]                                                                  <br>
    SmBoolean bDoZeroSampling=TRUE,     ///< [in ]: for internal use only, always set to TRUE, default:[TRUE]                       <br>
    SmBoolean bWithCompounding=TRUE     ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]       <br>
  ) const ;
                                                   
  // OutPoint = Evaluate(ParamPoint) (in final OutSpace with compounding)
  SmStatus EvaluatePoint
  (
    const SmPoint3d & crParamPoint,              ///< [in ]: ParamSpace point to map to OutSpace point                                    <br>
    SmPoint3d       & rOutSpacePoint,            ///< [out]: mapped point in outSpace                                                     <br>
    SmBoolean         bWithCompounding = TRUE    ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]    <br>
  )  const ;

  // convenience function using EvaluateDirectionalDerivs to compute just rOutDir.  
  // An EvaluateDirectionalDerivs call is more general. 
  //     Ex: one EvaluateDirectionalDerivs call can replace an EvaluatePoint() and EvaluateVector() pair of calls for half the cost.
  SmStatus EvaluateVector
  (
    const SmPoint3d & crParamPoint,    ///< [in ]: Param Point to evaluate                                                           <br>
    SmVector3d      & rParamDir,       ///< [in ]: Param direction for derivatives, gets unitized                                    <br>
    SmVector3d      & rOutDir,         ///< [out]: OutDir = 1st directional derivative for given unit-vector ParamDir                <br>
    SmBoolean         bUFromLeft=TRUE, ///< [in ]: if P is on U, V, or W interval boundary                                           <br>
    SmBoolean         bVFromLeft=TRUE, ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval       <br>
    SmBoolean         bWFromLeft=TRUE  ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval      <br>
  ) const                               
  { 
    SmVector3d aDirDerivs[2] ;
    SER(EvaluateDirectionalDerivs(crParamPoint, rParamDir, 1, aDirDerivs, bUFromLeft, bVFromLeft, bWFromLeft)) ;
    rOutDir = aDirDerivs[1] ;
    return(SM_SUCCESS) ; 
  }

  // 1st derivs for OutPoint = Evaluate(ParamPoint) (in final OutSpace with compounding)
  SmStatus Evaluate1stDerivatives
  (
    const SmPoint3d & crParamPoint,    ///< [in ]: ParamSpace point to map to OutSpace point                                         <br>
    SmBoolean         bUFromLeft,      ///< [in ]: if P is on U, V, or W interval boundary                                           <br>
    SmBoolean         bVFromLeft,      ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval       <br>
    SmBoolean         bWFromLeft,      ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval      <br>
    SmPoint3d       & rOutSpacePoint,  ///< [out]: OutSpace Point                                                                    <br>
    SmVector3d      & rDUout,          ///< [out]: OutSpace U direction tangent                                                      <br>
    SmVector3d      & rDVout,          ///< [out]: OutSpace V direction tangent                                                      <br>
    SmVector3d      & rDWout,          ///< [out]: OutSpace W direction tangent                                                      <br>
    SmBoolean bNonZeroTangents=TRUE,   ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors      <br>
                                       ///<      : FALSE= return exact tangent values, default:[TRUE]                                <br>
    SmBoolean bWithCompounding=TRUE    ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]         <br>
  ) const ;                

  // 2nd derivs for OutPoint = Evaluate(ParamPoint) (in final OutSpace with compounding)
  SmStatus Evaluate2ndDerivatives
  (
    const SmPoint3d & crParamPoint,    ///< [in ]: ParamSpace point to map to OutSpace Point                                        <br>
    SmBoolean         bUFromLeft,      ///< [in ]: if P is on U, V, or W interval boundary                                          <br>
    SmBoolean         bVFromLeft,      ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval      <br>
    SmBoolean         bWFromLeft,      ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval     <br>
    SmPoint3d       & rOutSpacePoint,  ///< [out]: OutSpace position                                                                <br>
    SmVector3d      & rDUout,          ///< [out]: OutSpace U direction tangent                                                     <br>
    SmVector3d      & rDVout,          ///< [out]: OutSpace V direction tangent                                                     <br>
    SmVector3d      & rDWout,          ///< [out]: OutSpace W direction tangent                                                     <br>
    SmVector3d      & rDUUout,         ///< [out]: OutSpace UU direction 2nd derivative                                             <br>
    SmVector3d      & rDVVout,         ///< [out]: OutSpace VV direction 2nd derivative                                             <br>
    SmVector3d      & rDWWout,         ///< [out]: OutSpace WW direction 2nd derivative                                             <br>
    SmVector3d      & rDUVout,         ///< [out]: OutSpace UV 2nd order cross derivative                                           <br>
    SmVector3d      & rDUWout,         ///< [out]: OutSpace UW 2nd order cross derivative                                           <br>
    SmVector3d      & rDVWout,         ///< [out]: OutSpace VW 2nd order cross derivative                                           <br>
    SmBoolean bNonZeroTangents=TRUE,   ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors     <br>
                                       ///<      : FALSE= return exact tangent values, default:[TRUE]                               <br>
    SmBoolean bWithCompounding=TRUE    ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]        <br>
  ) const ;                                                                                                                 

  // directional derives for OutPoint = Evaluate(ParamPoint) (in final OutSpace with compounding)
  SmStatus EvaluateDirectionalDerivs
  (
    const SmPoint3d & crParamPoint,    ///< [in ]: Param Point to evaluate                                                          <br>
    SmVector3d      & rParamDir,       ///< [in ]: Param direction for derivatives, gets unitized                                   <br>
    ULONG             lHighestDeriv,   ///< [in ]: 1=1st deriv, 2=1st and 2nd derivs, 3=1st, 2nd, and 3rd derivs                    <br>
    SmVector3d        aDirDerivs[],    ///< [out]: OutSpace aDirDerivs[0] = position                                                <br>
                                       ///<      :      aDirDerivs[1] = 1st directional derivative                                  <br>
                                       ///<      :      aDirDerivs[2] = 2nd directional derivative                                  <br>
                                       ///<      :      aDirDerivs[3] = 3rd directional derivative                                  <br>
                                       //      sized:[lHighestDeriv+1]                                                              <br>
    SmBoolean         bUFromLeft=TRUE, ///< [in ]: if P is on U, V, or W interval boundary                                          <br>
    SmBoolean         bVFromLeft=TRUE, ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval      <br>
    SmBoolean         bWFromLeft=TRUE  ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval     <br>
  ) const ;

  // InvertMap point from OutSpace back to ParamSpace (from final OutSpace with compounding)
  SmStatus InvEvaluatePoint
  (
    const SmPoint3d     & crOutSpacePoint,      ///< [in ]: OutSpace Point to map back to ParamSpace Point                                <br>
    const SmPoint3d     * pOptParamGuess,       ///< [in ]: last intermediate ParamSpace guess location at which to start the search      <br>
    SmBoolean           & rbSuccess,            ///< [out]: TRUE = inverse was found, else FALSE                                          <br>
    SmTArray<SmPoint3d> & rParamPoints,         ///< [out]: the ParamSpace inverse mapping                                                <br>
    SmTArray<double>    & rdGaps,               ///< [out]: max distance between found result (in case of bounding) and target point      <br>
    const SmExtent3d    * pOptParamDomain=NULL, ///< [in ]: ParamSpace domain over which to search for the inverse point                  <br>
                                                ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                <br>
    SmBoolean             bWithCompounding=TRUE ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]     <br>
  ) const ;

  // InvertMap vector from OutSpace back to ParamSpace (from final OutSpace with compounding)
  SmStatus InvEvaluateVector
  (
    const SmVector3d     & crOutSpaceVec,          ///< [in ]: OutSpace Vector to map back to InSpace Vector                                        <br>
    const SmPoint3d      & crParamPoint,           ///< [in ]: ParamSpace Point specifying where the Drop will take place (see InvEvaluatePoint())  <br>
    SmTArray<SmVector3d> & rParamVecs,             ///< [out]: Dropped ParamSpace vecs (2 if crParamPoint maps through seam)                        <br>
                                                   ///<      : note: with rParamVecs[ii] = [ParamVecU, ParamVecV, ParamVecW]                        <br>
                                                   ///<      :            rOutSpaceVec   = ParamVecU*rDX + ParamVecV*rDY + ParamVecW*rDZ            <br>
                                                   ///<      :       where rDX, rDY, rDZ are the outspace 1stDeriv vectors at crParamPoint          <br>
                                                   ///<      :       as computed by Evaluate1stDerivatives()                                        <br>
    SmBoolean              bWithCompounding=TRUE   ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]            <br>
  ) const ;  
                                  
  // get approximate ParamSpace location for given OutSpace Point (from final OutSpace with compounding)
  SmStatus InvEvaluateGuessPoint
  (
    const SmPoint3d     & crOutSpacePoint,        ///< [in ]: OutSpace Point to map back to InSpace Point                                          <br>
    SmTArray<SmPoint3d> & rGuessParamPoints,      ///< [out]: ParamSpace Point near Target Point actual map back to ParamSpace                     <br>
    SmBoolean             bWithCompounding=TRUE   ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]            <br>
  ) const ;

  // Map Section: VOLUME MAPPING DIAGRAM
  //   +---------------+                                 +------------------+
  //   |   InSpace     | -------------'Map'------------> |     OutSpace     |
  //   | [xIn yIn zIn] | <----------'InvMap'------------ | [xOut yOut zOut] |
  //   +---------------+                              >  +------------------+     
  //       ^     |                             >      <        ^     |        
  //       | 'InvOrient'     'Evaluate' >      <               | 'InvOrient'                    
  //   'Orient'  |               >      <                  'Orient'  |                       
  //       |     v        >      < 'InvEvaluate'               |     v             
  //   +---------------+  <                              +------------------+
  //   |  ParamSpace   | -------'EvaluateSimple'-------> |    ProjSpace     |   
  //   |   [u v w]     | <-----'InvEvaluateSimple'------ |                  |     
  //   +---------------+  [  GlobalPointSolveSimple   ]  +------------------+     
  //                      [InvEvaluateGuessPointSimple] 
  //                      [   LocalPointSolveSimple   ]       

  // Index into Evaluate(), Map(), Orient() output arrays by partial deriv cnts.
  // examples: aDerivs[SM_VSIZE(MaxDeriv)], lHighestDeriv=MaxDeriv, 
  //     sW   = aDerivs[SM_VI(0,0,0)],  - position                            // when MaxDeriv is named 'lHighestDeriv'
  //     sWuu = aDerivs[SM_VI(2,0,0)],  - 2nd deriv in U direction            // 
  //     sWv  = aDerivs[SM_VI(0,1,0)],  - 1st deriv in V direction = TangentV // 
  //     sWuw = aDerivs[SM_VI(1,0,1)],  - cross deriv in UW direction         // 
  //  or sW   = aDerivs[SM_VM(0,0,0,MaxDeriv)],                               // when MaxDeriv is not named 'lHighestDeriv'
  //     sWv  = aDerivs[SM_V1(0,1,0)],                                        // when MaxDeriv is known to equal 1
  //     sWuu = aDerivs[SM_V2(2,0,0)],                                        // when MaxDeriv is known to equal 2
  //     sWuw = aDerivs[SM_V3(1,0,1)],  - cross deriv in UW direction         // when MaxDeriv is known to equal 3
   
  // Compute OutPoint = Map(InPoint) (in final OutSpace with compounding)
  SmStatus Map
  (
    const SmPoint3d & crInSpacePoint,       ///< [in ]: InSpace point to map to OutSpace point                                                        <br>
    ULONG             lHighestDeriv,        ///< [in ]: 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                                      <br>
    SmBoolean         bXinFromLeft,         ///< [in ]: if P is on Xin, Yin, or Zin interval boundary                                                 <br>
    SmBoolean         bYinFromLeft,         ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval                   <br>
    SmBoolean         bZinFromLeft,         ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval                  <br>
    SmVector3d      * aDerivatives,         ///< [out]: matrix of OutSpace evaluations values                                                         <br>
                                            ///<      : sized:[n+1][n+1][n+1], where n=lHighesDeriv                                                   <br>
                                            ///<      : indexing:[1dIndex = xIn*(n+1)*(n+1)+yIn*(n+1)+zIn], where xIn,yIn,Zin=number of partial derivs<br>
                                            ///<      : 3d organized: D[XdirCnt][YDirCnt][ZDirCnt]                                                    <br>
                                            ///<      : 1d organized: for lHighestDeriv from 0 to 3,                                                  <br>
                                            ///<      :       0,  sized:[1]    order:[D]                                                              <br>
                                            ///<      :       1,  sized:[8]    order:[D    DzIn DyIn ----                                             <br>
                                            ///<      :                               DxIn ---- ---- ----]                                            <br>
                                            ///<      :       2,  sized:[27]   order:[D     DzIn  DzzIn DyIn  DyzIn ----- DyyIn ----- -----           <br>
                                            ///<      :                               DxIn  DxzIn ----- DxyIn ----- ----- ----- ----- -----           <br>
                                            ///<      :                               DxxIn ----- ----- ----- ----- ----- ----- ----- -----]          <br>
                                            ///<      :       3,  sized:[64]   order: 1dIndex = u*16+v*4+w                                            <br>
                                            ///<      :            [D     DzIn   DzzIn   DzzzIn  DyIn   DyzIn  DyzzIn ...    DyyIn  DyzIn             <br>
                                            ///<      :             ...   ...    DyyyIn  ...     ...    ...    DxIn   DxzIn  DxzzIn ...               <br>
                                            ///<      :             DxyIn DxyzIn ...     ...     DxyyIn ...    ...    ...    ...    ...               <br>
                                            ///<      :             ...   ...    DxxIn   DxxzIn  ...    ...    DxxyIn ...    ...    ...               <br>
                                            ///<      :             ...   ...    ...     ...     ...    ...    ...    ...    DxxxIn ...               <br>
                                            ///<      :             ...   ...    ...     ...     ...    ...    ...    ...    ....   ...               <br>
                                            ///<      :             ...   ...    ...     ...     ...    ...    ...    ...    ....   ...               <br>
                                            ///<      :             ...   ...    ...     ...     ...    ...    ...    ...    ....   ...               <br>
                                            ///<      :             ...   ... ]                                                                       <br>
    SmBoolean bNonZeroTangents=TRUE,        ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors                  <br>
                                            ///<      :  FALSE= return exact tangent values, default:[TRUE]                                           <br>
                                            ///<      :  note: Surprisingly TRUE is the common choice because most tangent uses                       <br>
                                            ///<      :        are for their direction (Binorm, SurfNorm comps), but when the                         <br>
                                            ///<      :        tangent is being used for its magnitude (like an arc-length comp)                      <br>
                                            ///<      :        then set this to FALSE.                                                                <br>
                                            ///<      :  default:[TRUE]                                                                               <br>
    SmBoolean bDoZeroSampling=TRUE,         ///< [in ]: for internal use only, always set to TRUE, default:[TRUE]                                     <br>
    SmBoolean bWithCompounding=TRUE         ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]                     <br>
  ) const ;
                                                   
  // OutPoint = Map(InPoint) (in final OutSpace with compounding)
  SmStatus MapPoint
  (
    const SmPoint3d & crInSpacePoint,          ///< [in ]: point to map from inSpace                                                       <br>
    SmPoint3d       & rOutSpacePoint,          ///< [out]: mapped point in outSpace                                                        <br>
    SmBoolean         bWithCompounding = TRUE  ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]       <br>
  ) const ;

  // convenience function using MapDirectionalDerivs to compute just rOutDir.  
  // A MapDirectionalDerivs call is more general. 
  //   Ex: one MapDirectionalDerivs() call can replace a MapPoint() and MapVector() pair of calls for half the cost.
  SmStatus MapVector
  (
    const SmPoint3d & crInSpacePoint,  ///< [in ]: InSpace Point to evaluate                                                      <br>
    SmVector3d      & rInSpaceDir,     ///< [in ]: InSpace direction for derivatives, gets unitized                               <br>
    SmVector3d      & rOutDir,         ///< [out]: OutDir = 1st directional derivative for given unit-vector InSpaceDir           <br>
    SmBoolean         bUFromLeft=TRUE, ///< [in ]: if P is on U, V, or W interval boundary                                        <br>
    SmBoolean         bVFromLeft=TRUE, ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval    <br>
    SmBoolean         bWFromLeft=TRUE, ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval   <br>
    SmBoolean   bWithCompounding=TRUE  ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]      <br>
  ) const                               
  { 
    SmVector3d aDirDerivs[2] ;
    SER(MapDirectionalDerivs(crInSpacePoint, rInSpaceDir, 1, aDirDerivs, 
                             bUFromLeft, bVFromLeft, bWFromLeft, 
                             bWithCompounding)) ;
    rOutDir = aDirDerivs[1] ;
    return(SM_SUCCESS) ; 
  }

  // 1st derivs for OutPoint = Map(InPoint) (in final OutSpace with compounding)
  SmStatus Map1stDerivatives
  (
    const SmPoint3d & crInSpacePoint, ///< [in ]: InSpace point to map to OutSpace point                                          <br>
    SmBoolean         bXinFromLeft,   ///< [in ]: if P is on Xin, Yin, or Zin interval boundary                                   <br>
    SmBoolean         bYinFromLeft,   ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval     <br>
    SmBoolean         bZinFromLeft,   ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval    <br>
    SmPoint3d       & rOutSpacePoint, ///< [out]: OutSpace Point                                                                  <br>
    SmVector3d      & rDXin,          ///< [out]: OutSpace Xin direction tangent                                                  <br>
    SmVector3d      & rDYin,          ///< [out]: OutSpace Yin direction tangent                                                  <br>
    SmVector3d      & rDZin,          ///< [out]: OutSpace Zin direction tangent                                                  <br>
    SmBoolean bNonZeroTangents=TRUE,  ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors    <br>
                                      ///<      : FALSE= return exact tangent values, default:[TRUE]                              <br>
    SmBoolean bWithCompounding=TRUE   ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]       <br>
  ) const ;

  // 2nd derivs for OutPoint = Map(InPoint) (in final OutSpace with compounding)
  SmStatus Map2ndDerivatives
  (
    const SmPoint3d & crInSpacePoint, ///< [in ]: InSpace point to map to OutSpace Point                                           <br>
    SmBoolean         bXinFromLeft,   ///< [in ]: if P is on Xin, Yin, or Zin interval boundary                                    <br>
    SmBoolean         bYinFromLeft,   ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval      <br>
    SmBoolean         bZinFromLeft,   ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval     <br>
    SmPoint3d       & rOutSpacePoint, ///< [out]: OutSpace position                                                                <br>
    SmVector3d      & rDX,            ///< [out]: OutSpace Xin direction tangent                                                   <br>
    SmVector3d      & rDY,            ///< [out]: OutSpace Yin direction tangent                                                   <br>
    SmVector3d      & rDZ,            ///< [out]: OutSpace Zin direction tangent                                                   <br>
    SmVector3d      & rDXX,           ///< [out]: OutSpace Xin direction 2nd derivative                                            <br>
    SmVector3d      & rDYY,           ///< [out]: OutSpace Yin direction 2nd derivative                                            <br>
    SmVector3d      & rDZZ,           ///< [out]: OutSpace Zin direction 2nd derivative                                            <br>
    SmVector3d      & rDXY,           ///< [out]: OutSpace XYin 2nd order cross derivative                                         <br>
    SmVector3d      & rDXZ,           ///< [out]: OutSpace XZin 2nd order cross derivative                                         <br>
    SmVector3d      & rDYZ,           ///< [out]: OutSpace YZin 2nd order cross derivative                                         <br>
    SmBoolean bNonZeroTangents=TRUE,  ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors     <br>
                                      ///<      : FALSE= return exact tangent values, default:[TRUE]                               <br>
    SmBoolean bWithCompounding=TRUE   ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]        <br>
  ) const ;

  // directional derives for OutPoint = Map(InPoint) (in final OutSpace with compounding)
  SmStatus MapDirectionalDerivs
  (
    const SmPoint3d & crInSpacePoint,        ///< [in ]: InSpace Point to evaluate                                                     <br>
    SmVector3d      & rInSpaceDir,           ///< [in ]: InSpace direction for derivatives, gets unitized                              <br>
    ULONG             lHighestDeriv,         ///< [in ]: 1=1st deriv, 2=1st and 2nd derivs, 3=1st, 2nd, and 3rd derivs                 <br>
    SmVector3d        aDirDerivs[],          ///< [out]: OutSpace aDirDerivs[0] = position                                             <br>
                                             ///<      :      aDirDerivs[1] = 1st directional derivative                               <br>
                                             ///<      :      aDirDerivs[2] = 2nd directional derivative                               <br>
                                             ///<      :      aDirDerivs[3] = 3rd directional derivative                               <br>
                                             ///<      : sized:[lHighestDeriv+1]                                                       <br>
    SmBoolean         bUFromLeft=TRUE,       ///< [in ]: if P is on U, V, or W interval boundary                                       <br>
    SmBoolean         bVFromLeft=TRUE,       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval   <br>
    SmBoolean         bWFromLeft=TRUE,       ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval  <br>
    SmBoolean         bWithCompounding=TRUE  ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]     <br>
  ) const ;

  // InvertMap point from OutSpace back to InSpace (from final OutSpace with compounding)
  SmStatus InvMapPoint
  (
    const SmPoint3d     & crOutSpacePoint,        ///< [in ]: OutSpace Point to map back to InSpace Point                                  <br>
    const SmPoint3d     * pOptInPointGuess,       ///< [in ]: last intermediate InSpace guess location at which to start the search        <br>
    SmBoolean           & rbSuccess,              ///< [out]: TRUE = inverse was found, else FALSE                                         <br>
    SmTArray<SmPoint3d> & rInSpacePoints,         ///< [out]: the InSpace inverse mapping                                                  <br>
    SmTArray<double>    & rdGaps,                 ///< [out]: max distance between found result (in case of bounding) and target point     <br>
    const SmPseudoBox   * pOptInSpaceDomain=NULL, ///< [in ]: InSpace domain over which to search for the inverse point                    <br>
                                                  ///<      : NULL = use Map's NaturalDomain, default:[NULL]                               <br>
    SmBoolean             bWithCompounding=TRUE   ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]    <br>
  ) const ;                                    
                                                                 
  // InvertMap vector from OutSpace back to InSpace (from final OutSpace with compounding)             
  SmStatus InvMapVector
  (
    const SmVector3d     & crOutSpaceVec,        ///< [in ]: OutSpace Vector to map back to InSpace Vector                                     <br>
    const SmPoint3d      & crInSpacePoint,       ///< [in ]: InSpace Point specifying where the Drop will take place (see InvMapPoint())       <br>
    SmTArray<SmVector3d> & rInSpaceVecs,         ///< [out]: Dropped InSpace vecs (2 if crParamPoint maps through seam)                        <br>
                                                 ///<      : note: with rInSpaceVecs[ii] = [InSpaceVecU, InSpaceVecV, InSpaceVecW]             <br>
                                                 ///<      :            rOutSpaceVec   = InSpaceVecU*rDX + InSpaceVecV*rDY + InSpaceVecW*rDZ   <br>
                                                 ///<      :       where rDX, rDY, rDZ are the outpspace 1stDeriv vectors at crInSpacePoint    <br>
    SmBoolean              bWithCompounding=TRUE ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]         <br>
  ) const ; 
                                   
  // get approximate InSpace location for given OutSpace Point (from final OutSpace with compounding)
  SmStatus InvMapGuessPoint
  (
    const SmPoint3d     & crOutSpacePoint,        ///< [in ]: OutSpace Point to map back to InSpace Point                                   <br>
    SmTArray<SmPoint3d> & rGuessInSpacePoints,    ///< [out]: InSpace Point near Target Point actual map back to InSpace                    <br>
    SmBoolean             bWithCompounding        ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]     <br>
  ) const ;

  // Orient Section: VOLUME MAPPING DIAGRAM
  //   +---------------+                                 +------------------+
  //   |   InSpace     | -------------'Map'------------> |     OutSpace     |
  //   | [xIn yIn zIn] | <----------'InvMap'------------ | [xOut yOut zOut] |
  //   +---------------+                              >  +------------------+     
  //       ^     |                             >      <        ^     |        
  //       | 'InvOrient'     'Evaluate' >      <               | 'InvOrient'                    
  //   'Orient'  |               >      <                  'Orient'  |                       
  //       |     v        >      < 'InvEvaluate'               |     v             
  //   +---------------+  <                              +------------------+
  //   |  ParamSpace   | -------'EvaluateSimple'-------> |    ProjSpace     |   
  //   |   [u v w]     | <-----'InvEvaluateSimple'------ |                  |     
  //   +---------------+  [  GlobalPointSolveSimple   ]  +------------------+     
  //                      [InvEvaluateGuessPointSimple] 
  //                      [   LocalPointSolveSimple   ]       

  // Index into Evaluate(), Map(), Orient() output arrays by partial deriv cnts.
  // examples: aDerivs[SM_VSIZE(MaxDeriv)], lHighestDeriv=MaxDeriv, 
  //     sW   = aDerivs[SM_VI(0,0,0)],  - position                            // when MaxDeriv is named 'lHighestDeriv'
  //     sWuu = aDerivs[SM_VI(2,0,0)],  - 2nd deriv in U direction            // 
  //     sWv  = aDerivs[SM_VI(0,1,0)],  - 1st deriv in V direction = TangentV // 
  //     sWuw = aDerivs[SM_VI(1,0,1)],  - cross deriv in UW direction         // 
  //  or sW   = aDerivs[SM_VM(0,0,0,MaxDeriv)],                               // when MaxDeriv is not named 'lHighestDeriv'
  //     sWv  = aDerivs[SM_V1(0,1,0)],                                        // when MaxDeriv is known to equal 1
  //     sWuu = aDerivs[SM_V2(2,0,0)],                                        // when MaxDeriv is known to equal 2
  //     sWuw = aDerivs[SM_V3(1,0,1)],  - cross deriv in UW direction         // when MaxDeriv is known to equal 3

  // Compute InPoint = Orient(ParamPoint)  
  SmStatus Orient
  (
    const SmPoint3d & crParamPoint,         ///< [in ]: ParamSpace point to map to InSpace point                                        <br>
    ULONG             lHighestDeriv,        ///< [in ]: 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                        <br>
    SmBoolean         bUFromLeft,           ///< [in ]: if P is on U, V, or W interval boundary                                         <br>
    SmBoolean         bVFromLeft,           ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval     <br>
    SmBoolean         bWFromLeft,           ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval    <br>
    SmVector3d      * aDerivatives,         ///< [out]: matrix of InSpace evaluations values                                            <br>
                                            ///<      : 3d organized: D[u][v][w]                                                        <br>
                                            ///<      : 1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3              <br>
                                            ///<      : sized       : [n+1][n+1][n+1], where n=lHighesDeriv                             <br>
                                            ///<      : lHghDrv = 0,   sized: [1],                                                      <br>
                                            ///<      :   i=0          order: [D]                                                       <br>
                                            ///<      : lHghDrv = 1,   sized: [8]                                                       <br>
                                            ///<      :   i=u*4+v*2+w  order: [D  Dw  Dv  ---                                           <br>
                                            ///<      :                        Du --- --- ---]                                          <br>
                                            ///<      : lHghDrv = 2,   sized: [27]                                                      <br>
                                            ///<      :   i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---                      <br>
                                            ///<      :                        Du  Duw --- Duv --- --- --- --- ---                      <br>
                                            ///<      :                        Duu --- --- --- --- --- --- --- ---]                     <br>
                                            ///<      : lHghDrv = 3,   sized: [81]                                                      <br>
                                            ///<      :   i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw        <br>
                                            ///<      :                        --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---        <br>
                                            ///<      :                        Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---        <br>
                                            ///<      :                        --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---        <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  Duuu ---        <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---        <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---        <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---        <br>
                                            ///<      :                        --- --- ]                                                <br>
    SmBoolean bNonZeroTangents=TRUE,        ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors    <br>
                                            ///<      : FALSE= return exact tangent values, default:[TRUE]                              <br>
                                            ///<      : note: Surprisingly TRUE is the common choice because most tangent uses          <br>
                                            ///<      :       are for their direction (Binorm, SurfNorm comps), but when the            <br>
                                            ///<      :       tangent is being used for its magnitude (like an arc-length comp)         <br>
                                            ///<      :       then set this to FALSE.                                                   <br>
                                            ///<      : default:[TRUE]                                                                  <br>
    SmBoolean bDoZeroSampling=TRUE          ///< [in ]: for internal use only, always set to TRUE, default:[TRUE]                       <br>
  ) const ;
                                                   
  // InPoint = Orient(ParamPoint)
  SmStatus OrientPoint
  (
    const SmPoint3d & crParamPoint,         ///< [in ]: ParamSpace point to map to InSpace point     <br>
    SmPoint3d & rInSpacePoint               ///< [out]: InSpace point mapped from ParamSpace point   <br>
  ) const ;

  // convenience function using OrientDirectionalDerivs to compute just rInDir.  
  // An OrientDirectionalDerivs call is more general. 
  //   Ex: one OrientDirectionalDerivs() call can replace an OrientPoint() and OrientVector() pair of calls for half the cost.
  SmStatus OrientVector
  (
    const SmPoint3d & crParamPoint,    ///< [in ]: Param Point to evaluate                                                         <br>
    SmVector3d      & rParamDir,       ///< [in ]: Param direction for derivatives, gets unitized                                  <br>
    SmVector3d      & rInDir,          ///< [out]: InDir = 1st directional derivative for given unit-vector ParamDir               <br>
    SmBoolean         bUFromLeft=TRUE, ///< [in ]: if P is on U, V, or W interval boundary                                         <br>
    SmBoolean         bVFromLeft=TRUE, ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval     <br>
    SmBoolean         bWFromLeft=TRUE  ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval    <br>
  ) const                               
  { 
    SmVector3d aDirDerivs[2] ;
    SER(OrientDirectionalDerivs(crParamPoint, rParamDir, 1, aDirDerivs, bUFromLeft, bVFromLeft, bWFromLeft)) ;
    rInDir = aDirDerivs[1] ;
    return(SM_SUCCESS) ;
  }

  // 1st derivs for InPoint = Orient(ParamPoint) 
  SmStatus Orient1stDerivatives
  (
    const SmPoint3d & crParamPoint,     ///< [in ]: ParamSpace point to map to InSpace point                                          <br>
    SmBoolean         bUFromLeft,       ///< [in ]: if P is on U, V,or W interval boundary                                            <br>
    SmBoolean         bVFromLeft,       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval       <br>
    SmBoolean         bWFromLeft,       ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval      <br>
    SmPoint3d       & rInSpacePoint,    ///< [out]: InSpace Point                                                                     <br>
    SmVector3d      & rDUin,            ///< [out]: InSpace U direction tangent                                                       <br>
    SmVector3d      & rDVin,            ///< [out]: InSpace V direction tangent                                                       <br>
    SmVector3d      & rDWin,            ///< [out]: InSpace W direction tangent                                                       <br>
    SmBoolean bNonZeroTangents=TRUE     ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors      <br>
                                        ///<      : FALSE= return exact tangent values, default:[TRUE]                                <br>
  ) const ;                             

  // 2nd derivs for InPoint = Orient(ParamPoint) 
  SmStatus Orient2ndDerivatives
  (
    const SmPoint3d & crParamPoint,     ///< [in ]: ParamSpace point to map to InSpace Point                                        <br>
    SmBoolean         bUFromLeft,       ///< [in ]: if P is on U, V, or W interval boundary                                         <br>
    SmBoolean         bVFromLeft,       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval     <br>
    SmBoolean         bWFromLeft,       ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval    <br>
    SmPoint3d       & rInSpacePoint,    ///< [out]: InSpace position                                                                <br>
    SmVector3d      & rDUin,            ///< [out]: InSpace U direction tangent                                                     <br>
    SmVector3d      & rDVin,            ///< [out]: InSpace V direction tangent                                                     <br>
    SmVector3d      & rDWin,            ///< [out]: InSpace W direction tangent                                                     <br>
    SmVector3d      & rDUUin,           ///< [out]: InSpace Uin direction 2nd derivative                                            <br>
    SmVector3d      & rDVVin,           ///< [out]: InSpace Vin direction 2nd derivative                                            <br>
    SmVector3d      & rDWWin,           ///< [out]: InSpace Win direction 2nd derivative                                            <br>
    SmVector3d      & rDUVin,           ///< [out]: InSpace UVin 2nd order cross derivative                                         <br>
    SmVector3d      & rDUWin,           ///< [out]: InSpace UWin 2nd order cross derivative                                         <br>
    SmVector3d      & rDVWin,           ///< [out]: InSpace VWin 2nd order cross derivative                                         <br>
    SmBoolean bNonZeroTangents=TRUE     ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors    <br>
                                        ///<      : FALSE= return exact tangent values, default:[TRUE]                              <br>
  ) const ;                             

  // directional derives for InPoint = Orient(ParamPoint) 
  SmStatus OrientDirectionalDerivs
  (
    const SmPoint3d & crParamPoint,    ///< [in ]: ParamSpace Point to evaluate                                                     <br>
    SmVector3d      & rParamDir,       ///< [in ]: ParamSpace direction for derivatives, gets unitized                              <br>
    ULONG             lHighestDeriv,   ///< [in ]: 1=1st deriv, 2=1st and 2nd derivs, 3=1st, 2nd, and 3rd derivs                    <br>
    SmVector3d        aDirDerivs[],    ///< [out]: InSpace aDirDerivs[0] = position                                                 <br>
                                       ///<      :         aDirDerivs[1] = 1st directional derivative                               <br>
                                       ///<      :         aDirDerivs[2] = 2nd directional derivative                               <br>
                                       ///<      :         aDirDerivs[3] = 3rd directional derivative                               <br>
                                       ///<      : sized:[lHighestDeriv+1]                                                          <br>
    SmBoolean         bUFromLeft=TRUE, ///< [in ]: if P is on U, V, or W interval boundary                                          <br>
    SmBoolean         bVFromLeft=TRUE, ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval      <br>
    SmBoolean         bWFromLeft=TRUE  ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval     <br>
  ) const ;

  // InvertMap point from InSpace back to ParamSpace
  SmStatus InvOrientPoint
  (
    const SmPoint3d  & crInSpacePoint,   ///< [in ]: InSpace Point to map back to ParamSpace Point                                 <br>
    SmBoolean        & rbSuccess,        ///< [out]: TRUE = inverse was found, else FALSE                                          <br>
    SmPoint3d        & rParamPoint,      ///< [out]: the parameter space inverse mapping                                           <br>
    double           & rdGap             ///< [out]: max distance between found result (in case of bounding) and target point      <br>
  ) const ;

  // InvertMap vector from InSpace back to ParamSpace
  SmStatus InvOrientVector
  (
    const SmVector3d & crInSpaceVec,     ///< [in ]: InSpace Vector to map back to ParamSpace Vector                                    <br>
     const SmPoint3d  & crParamPoint,    ///< [in ]: ParamSpace Point specifying where the Drop will take place (see InvOrientPoint())  <br>
     SmVector3d       & rParamVec        ///< [out]: Dropped ParamSpace vecs                                                            <br>
                                         ///<      : note: with rParamVec = [ParamVecU, ParamVecV, ParamVecW]                           <br>
                                         ///<      :            rOutSpaceVec   = ParamVecU*rDX + ParamVecV*rDY + ParamVecW*rDZ          <br>
                                         ///<      :       where rDX, rDY, rDZ are the InSpace 1stDeriv vectors at crParamPoint         <br>
  ) const;
                                                                
  // Global and Local Point solvers map from OutSpace back to ParamSpace.
  //   The methods InvEvaluatePoint and InvEvaluateVector do the same thing.
  // 
  // Map from OutSpace back to InSpace with InvMapPoint and InvMapVector. 
  //   No Global or Local Point solver methods exist for OutSpace back to InSpace mappings.
  // 
  // Map from ParamSpace back to InSpace with InvOrientPoint and InvOrientVector.
  //   No Global or Local Point solver methods exist for ParamSpace back to InSpace mappings

  // InvertMap OutSpace Point back to ParamSpace Point. (from final OutSpace with compounding)
  SmStatus GlobalPointSolve
  (
    const SmPoint3d  & crOutSpacePoint,       ///< [in ]: OutSpace Point to map back to ParamSpace                                                <br>
    SmBoolean        & rbFoundAnswer,         ///< [out]: TRUE = successfully mapped OutSpace Point to a ParamSpace Point                         <br>
    SmSolutionArray  & rSolutions,            ///< [out]: Contains ParamSpace Found Points                                                        <br>
                                              ///<      : rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                             <br>                  
                                              ///<      : rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crOutSpacePoint);<br>                      
                                              ///<      : rSolution[i].m_vStart[0] =  u of [u,v,w] the found ParamSpace point location            <br>        
                                              ///<      : rSolution[i].m_vStart[1] =  v of [u,v,w] the found ParamSpace point location            <br>        
                                              ///<      : rSolution[i].m_vStart[2] =  w of [u,v,w] the found ParamSpace point location            <br>        
    const SmExtent3d * pOptParamDomain=NULL,  ///< [in ]: ParamSpace domain over which to search for the inverse point                            <br>
                                              ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                          <br>
    SmBoolean          bWithCompounding=TRUE  ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]               <br>
  ) const ;
  
  // Inverse Map OutSpace Point back to ParamSpace Point using a local calculation
  SmStatus LocalPointSolve
  (
    const SmPoint3d  & crOutSpacePoint,        ///< [in ]: OutSpace point to map back to ParamSpace                                                <br>
    const SmPoint3d  & crParamPointGuess,      ///< [in ]: ParamSpace point guess, the closer to the actual ParamSpace point the better            <br>
    SmBoolean        & rbFoundAnswer,          ///< [out]: TRUE = successfully mapped OutSpace Point to an ParamSpace Point                        <br>
    SmSolutionArray  & rSolutions,             ///< [out]: Found ParamSpace Point coordinates and accuracy of inverse mapping                      <br>
                                               ///<      : rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                             <br>                  
                                               ///<      : rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crOutSpacePoint);<br>                      
                                               ///<      : rSolution[i].m_vStart[0] =  u of [u,v,w] the found ParamSpace point location            <br>        
                                               ///<      : rSolution[i].m_vStart[1] =  v of [u,v,w] the found ParamSpace point location            <br>        
                                               ///<      : rSolution[i].m_vStart[2] =  w of [u,v,w] the found ParamSpace point location            <br>        
    const SmExtent3d * pOptParamDomain=NULL,   ///< [in ]: ParamSpace domain over which to search for the inverse point                            <br>
                                               ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                          <br>
    SmBoolean          bWithCompounding=TRUE   ///< [in ]: TRUE = Evaluate compounds, FALSE = Evaluate just this map, default:[TRUE]               <br>
  ) const ;

  // create a new mirror volume of this Volume
  SmStatus CreateMirrorVolume
  (
    const SmContext        & crContext,          ///< [in ]: context for new object construction                        <br>
    const SmAxis2Placement & crMirrorPlane,      ///< [in ]: volume is mirrored about the MirrorPlane's XY plane        <br>
    SmVolume              *& rpMirrorVolume      ///< [out]: newly allocated volume                                     <br>
  );

  //  Affine Outspace SmVolume image changes: 
  //
  //       Mirror()                    Mirror    Volume Outspace Image about Outspace MirrorPlane   
  //       Scale()                     Scale     Volume Outspace Image about Outspace origin or opt OutSpace CenterPoint   
  //       Translate()                 Translate Volume Outspace Image by given OutSpace Vector  
  //       RotateAboutAxis()           Rotate    Volume Outspace Image about an axis running through OutSpace Origin 
  //       RotateAboutAxisAtPoint()    Rotate    Volume Outspace Image about an axis running through OutSpace CenterPoint    
  //       Transform()                 Apply a General Transform to a Volume's Outsapce Image    
  //
  //  Volumes can use compounded functions to apply affine transformations to any derived
  //  Volume shape at the cost of extra evaluations as:
  //
  //       Modified Volume Shape in OutSpace = Transform(Orient(SimpleMap(ParamPt)))
  //
  //  The SmVolume Affine methods all create an SmTransform object to transform the current 
  //  SmVolume OutSpace image to the next 3d space image and add that to the end of the 
  //  SmVolume's m_pNextMap list of compounded maps.
  //
  //  For performance, the SmVolume::AffineMethods() give each derived class a chance to be more efficient
  //  by calling a virtual AffineSimple() method whose job is to combine the effects of the two
  //  compounded maps (SimpleMap and Transform) into a single modified Volume SimpleMap as:
  //
  //     Transform(Orient(SimpleMap(ParamPt))) = Orient(ModifiedSimpleMap(ParamPt)) 
  //
  //  Users should not call the AffineSimple methods directly.  Leave the efficiency issues
  //  to the derived implementations by calling the Affine methods directly.
  
  
  // Apply a General rotate, move, and scale Transform to Volume's Outspace Image                                                                          
  SmStatus Transform(const SmAxis2Placement & crOutRotateNMove,     // in : Map Pts from current OutSpace to the next OutSpace. i.e. a (0,0,0)
                                                                    //    : OutSpacePoint projects to the RotatNMove NextOutSpace origin.     
                     const SmVector3d       * cpOptOutScale=NULL) ; // in : Opt OutSpace Origin Scaling done after RotateNMove mapping.       
                                                                    //    : NULL to ignore. default:[NULL]                                    

  // Apply Specific Affine transformations to Volume's Outspace Image

  // Translate Volume Outspace Image by given OutSpace Vector
  SmStatus Translate (const SmVector3d & crOutTranslate) ;   

  // Mirror Volume Outspace Image about Outspace MirrorPlane
  SmStatus Mirror (const SmAxis2Placement & crOutMirrorPlane) ; 

  // Scale Volume Outspace Image about Outspace origin or opt OutSpace CenterPoint
  SmStatus Scale (SmVector3d & crOutScaleVec,  SmPoint3d * cpOptOutCenter=NULL) ;

  // Rotate Volume Outspace Image about an axis running through OutSpace Origin
  SmStatus RotateAboutAxis( double dAngRad, const SmVector3d        & crOutAxis) ;

  // Rotate Volume Outspace Image about an axis running through OutSpace CenterPoint
  SmStatus RotateAboutAxisAtPoint(double dAngRad, const SmPoint3d  & crOutOrigin, const SmVector3d & crOutAxis) ;
     
  // rtn: TRUE if any of the compound mappings are bounded
  SmBoolean        IsBounded         () const ;  

  // rtn: TRUE if Volume's Simple map has internal discontinuities
  SmBoolean        HasDiscontinuities
  (
    SmDiscontinuities3d *pOptDisconts=NULL,  
    SmBoolean            bCalcGeometric=TRUE  ///< [in ]: TRUE = expensive - use geometric testing to compute actal geometric discontinuities   <br>
                                              ///<      : FALSE= cheap - report representational discontinuites                                 <br>
                                              ///<      : default:[TRUE]                                                                        <br>
  )  const ;                                   
                                                                               

  SmBoolean IsClosed          
  (
    SmBoolean        & rbClosedU,                ///< [out]: TRUE = closed in U direction, [check Pos[Umin,v,w] == Pos[Umax,v,w] for v,w samples     <br>
    SmBoolean        & rbClosedV,                ///< [out]: TRUE = closed in V direction, [check Pos[u,Vmin,w] == Pos[u,Vmax,w] for w,u samples     <br>
    SmBoolean        & rbClosedW,                ///< [out]: TRUE = closed in W direction, [check Pos[u,v,Wmin] == Pos[u,v,Wmax] for u,v samples     <br>
    double           * pdOptTolerance = NULL,    ///< [out]: pdOptTolerance = Tolerance to allow for check                                           <br>
    const SmExtent3d * pOptParamDomain = NULL,   ///< [in ]: ParamSpace domain over which to search for the inverse point                            <br>
                                                 ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                          <br>
    SmContinuityType * peOptContinuityU = NULL,  ///< [out]: U dir Continuity when closed                                                            <br>
    SmContinuityType * peOptContinuityV = NULL,  ///< [out]: V dir Continuity when closed                                                            <br>
    SmContinuityType * peOptContinuityW = NULL   ///< [out]: W dir Continuity when closed                                                            <br>
                                                 ///<      : oneof SM_CT_DISCONTINUOUS                                                               <br>
                                                 ///<      :       SM_CT_C0                                                                          <br>
                                                 ///<      :       SM_CT_G1                                                                          <br>
                                                 ///<      :       SM_CT_G1_G2                                                                       <br>
                                                 ///<      :       SM_CT_G1_G2_G3                                                                    <br>
                                                 ///<      :       SM_CT_C1                                                                          <br>
                                                 ///<      :       SM_CT_C1_G2                                                                       <br>
                                                 ///<      :       SM_CT_C1_G2_G3                                                                    <br>
                                                 ///<      :       SM_CT_C1_C2                                                                       <br>
                                                 ///<      :       SM_CT_C1_G3                                                                       <br>
                                                 ///<      :       SM_CT_C1_C3                                                                       <br>
  ) const ;

  SmBoolean IsPeriodic        
  (
    SmBoolean        & rbPeriodicU,              ///< [out]: TRUE = G1 or better in U Dir                                     <br>
    SmBoolean        & rbPeriodicV,              ///< [out]: TRUE = G1 or better in V Dir                                     <br>
    SmBoolean        & rbPeriodicW,              ///< [out]: TRUE = G1 or better in W Dir                                     <br>
    const SmExtent3d * pOptParamDomain = NULL    ///< [in ]: ParamSpace domain over which to search for the inverse point     <br>
                                                 ///<      : NULL = use Map's NaturalDomain, default:[NULL]                   <br>
  )  const;

  // return TRUE if point evaluation is singular in any parametric direction
  SmBoolean IsSingularity    
  (
    const SmPoint3d  & crParamPoint,                         
    SmBoolean        & rbSingularU,          ///< [out]: TRUE = [Wu=0]                                          <br>
    SmBoolean        & rbSingularV,          ///< [out]: TRUE = [Wv=0]                                          <br>
    SmBoolean        & rbSingularW,          ///< [out]: TRUE = [Ww=0]                                          <br>
    double             d3dTol=SM_EFF_ZERO    ///< NotUsed: [in ]: min OutSpace distance between distinct points          <br>
  )  const ;

  SmBoolean IsOnBoundary      
  (
    const SmPoint3d  & crParamPoint,            ///< [in ]:                                                      <br>
    SmBoolean        & rbOnU,                   ///< [out]:                                                      <br>
    SmBoolean        & rbOnV,                   ///< [out]:                                                      <br>
    SmBoolean        & rbOnW,                   ///< [out]:                                                      <br>
    double           * pdOptTolerance = NULL,   ///< [in ]: default:[NULL]                                       <br>
    const SmExtent3d * pOptParamDomain = NULL   ///< [in ]: ParamSpace domain to limit search                    <br>
                                                ///<      : NULL = use Map's NaturalDomain, default:[NULL]       <br>
  ) const ;

  // gwc: a function we might add later
  //      virtual SmStatus SplitAt(const SmContext    & crContext,     // in : context for new object construction           
  //                               double               dParam,        // in : split parameter                               
  //                               SmVolumeParamType    eVolumeParam,  // in : oneof SM_SP_U = split u domain at dParam      
  //                                                                   //    :       SM_SP_V = split v domain at dParam      
  //                               SmVolume          *& rpLeftVolume,  // out: Split surface result, Ivl=[MinParam, TgtParam]
  //                               SmVolume          *& rpRightVolume) // out: Split surface result, Ivl=[TgtParam, MaxParam]              
  //                                 { SE(SM_ERR) ; return SM_ERR ; }

  // gwc: a function we might add later
  //      virtual SmStatus TrimWithDomain(SmExtent3d & crTrimParamDomain)        
  //                                 { SE(SM_ERR) ; return SM_ERR ; }

  //      // look like a BSplineVolume
  //      gwc change - not all volumes are BSplines - these don't need to be here just in SmBSplineVolume
  //      
  //      // convenience functions for SmBSplineVolume - other classes use base default behaviors
  //      virtual gw_VOLUME * GetGwNurbPointer()  const { return NULL ; }           // retrieve the NLIB NURB struct
  //
  //      virtual SmStatus    Reparameterize(const SmExtent3d & crNewDomain);
  //                          
  //      virtual ULONG       GetDegree(SmVolumeParamType eParam) const              { SM_REF1(eParam) ; SE_MSG(SM_ERR,_T("Called pure virtual method")) ; return 3 ; }
  //                                                                                 
  //      virtual SmStatus    GetKnots(SmVolumeParamType  eParam,              
  //                                   SmTArray<double> & rKnots,                    
  //                                   SmTArray<ULONG>  * pKnotMults = NULL,         
  //                                   const SmExtent1d * pOptIvl = NULL)            ///< [in ]: interval of interest, NULL=Natural Interval, default:[NULL]
  //                                  const                                          { SM_REF2(eParam, pOptIvl) ; 
  //                                                                                   rKnots.ReSet() ; 
  //                                                                                   if(pKnotMults) pKnotMults->ReSet() ;
  //                                                                                   SE_MSG(SM_ERR,_T("Called pure virtual method")) ; 
  //                                                                                   return SM_ERR ;
  //                                                                                 }
  //                                                                                 
  //      virtual ULONG       GetNumberControlPoints(SmVolumeParamType eParam) const { SM_REF1(eParam) ; SE_MSG(SM_ERR,_T("Called pure virtual method")) ; return 0 ; }
  //

  // Snap ParamPoint to discontinuities within tolerance      
  SmStatus SnapToDiscontinuity
  (
    const SmPoint3d & crParamPoint,          ///< [in ]:           <br>
    double            dParamSnapTol,         ///< [in ]:           <br>
    SmPoint3d       & rSnappedParamPoint     ///< [out]:           <br>
  ) const ;


  // SIMPLE MAPPING INTERFACE - used to support the above compound mapping and oriented mapping Volume interface.
  //   These virtual methods are implemented by derived classes computing only the EvaluateSimple mapping.  
  //   Derived classes are isolated from the complications of both compounding and orientation.
  //     Methods to be implemented for derived Classes
  //
  //      // standard structure
  //        constructors   [as needed]           // req: make sure to call base constructors
  //        Copy()                               // req: deep virtual copy operator
  //        operator=()    [not virtual]         // req: deep copy operator
  //        operator==()                         // req: deep compare predicate 
  //        Notify()                             // opt: Notify mechanism. Default:[pass Notify to compounded volumes and base class]. Implement if:[class caches data]
  //        Destructor()                         // req: derived class destructors must be virtual
  //        GetNaturalParamDomain()              // req: return SmExtent3d ParamSpace domain
  //                                             
  //     // obsolete - look like a BSplineVolume
  //     // gwc change: Volumes are no longer required to look like BSplines - these only need to be in SmBSplineVolume
  //        GetGwNurbPointer()                   // obsolete opt: retrieve the NLIB NURB struct           
  //        Reparameterize()                     // obsolete opt: reparameterize the param space domain intervals
  //        GetDegree()                          // obsolete opt: return paramDirection degree. Default:[signal error and return 3]
  //        GetKnots()                           // obsolete opt: return paramDirection knot vector. Default:[init output and return SM_ERR]
  //        GetNumberControlPoints()             // obsolete opt: return paramDirection CPoint cnt. Default:[signal error and return 0]  
  //                                             
  //     // EvaluateSimple Interface to fit within the Oriented and compounded structure of the SmVolume base class
  //           
  //        EvaluateBoundingBoxSimple()             // opt: map ParamBoundingBoxes to ProjBoundingBoxes, Default:[Map and Box ParamBox corners to ProjSpace], implement if:[map is not Affine] 
  //        EvaluateIsoParametricCurveSimple()      // req: create ProjSpace IsoCurve for ParamSpace IsoLine
  //        EvaluateIsoParametricSurfaceSimple()    // req: create ProjSpace IsoSurface for ParamSpace IsoPlane
  //        EvaluateSimple()                        // req: compute ProjPoint = EvaluateSimple(ParamPoint)
  //        InvEvaluateGuessPointSimple()           // req: get approximate ParamSpace location for given ProjSpace point
  //        GlobalPointSolveSimple()                // opt: ProjSpace to ParamSpace Point Solver without a start guess. Default:[InvEvaluateGuessPointSimple() then NewtonRaphson], implement if:[avoid NR with exact inversion]
  //        LocalPointSolveSimple()                 // opt: ProjSpace to ParamSpace Point Solver with a start guess. Default:[NewtonRaphson], implement if:[avoid NR with exact inversion]
  //        FindParamIntervalForInSpaceLineSimple() // opt: Find InSpaceSegment portion that maps inside the NaturalParamDomain, implement:[when ParamNaturalDomain is not rectilinear], default:[XSect ParamLine with NaturalParamDomain]
  //        TrimParamBoundingBoxSimple()            // opt: Trim ParamBBox to natural ParamDomain, default:[pTrimBox = XSect(ParamBBox,NaturalParamDomain)], implement if:[MapParamDomain not rectilinear]
//     // FindParamExtentForInSpacePlaneSimple()  // opt: Find InSpace Plane extent that's within the NaturalParamDomain, default rTrimIvl = crCurrentIvl
//     // GWC - excluded from first release - has bug that needs debugging in SmSpaceBend:  
//     // GWC - removed from interface because it made no sense: Even very simple cases of trimming a BBox to a long skinny PseduoBox showed
//     //       that the inscribed Box could be very small and non-unique.  It won't necessarily capture the notion of a good
//     //       bounding box designed to help narrow down the space of options for functions that need to check that.  In general,
//     //       check wether individual InSpace and ProjPace Points map back to legal ParamDomain Points, one point at a time. 
//     // TrimInSpaceBoundingBoxSimple()          // opt: Trim InSpaceBBox to map within the NaturalParamSpaceDomain, default:[pTrimBox = pGivenBox], implement if:[map is bounded]
  //
  //     // Derived class EvaluateSimple map modifications
  //        // design issue - derived and default behaviors are not exactly the same.  
  //        //                Default behavior modifies the volume maps after being oriented, i.e. concatenates another NextMap object.
  //        //                Derived behavior modifies the volume map before being oriented, i.e. modifies EvaluateSimple() by modifying derived map parameters.
  //        //                If at all possible, derived classes should implement this set of virtual methods.
  //        MirrorSimple()                 // opt: Mirror EvaluateSimple map.                 Default:[concat Mirror        Transform as NextMap], implement if:[map to be used without a Orient Map like BSplineVolumes]
  //        ScaleSimple()                  // opt: Scale EvaluateSimple map.                  Default:[concat Scale         Transform as NextMap], implement if:[map to be used without a Orient Map like BSplineVolumes]
  //        TranslateSimple()              // opt: Translate EvaluateSimple map.              Default:[concat Translate     Transform as NextMap], implement if:[map to be used without a Orient Map like BSplineVolumes]
  //        RotateAboutAxisSimple()        // opt: RotateAboutAxis EvaluateSimple map.        Default:[concat Rotate        Transform as NextMap], implement if:[map to be used without a Orient Map like BSplineVolumes]
  //        RotateAboutAxisAtPointSimple() // opt: RotateAboutAxisAtPoint EvaluateSimple map. Default:[concat RotateAtPoint Transform as NextMap], implement if:[map to be used without a Orient Map like BSplineVolumes]  
  //        TransformSimple()              // opt: Transform EvaluateSimple map.              Default:[concat General       Transform as NextMap], implement if:[map to be used without a Orient Map like BSplineVolumes]
  //                                       
  //     // predicates                     
  //        IsBoundedSimple()              // opt: Bounded in ParamSpace Predicate. Default:[return FALSE],                    implement if:[bounded map] 
  //        IsClosedSimple()               // opt: Closed in ProjSpace Predicate.   Default:[OutSpace PointSample tests],      implement if:[closed and continuities are known without sampling]
  //        IsPeriodicSimple()             // opt: Periodic ParamSpace Predicate.   Default:[calls IsClosedSimple()],          implement if:[cheaper than IsClosedSimple()] 
  //        IsSingularitySimple()          // opt: EvaluateSingularity Predicate.   Default:[OutSpace ZeroSizedTangent test],  implement if:[singularity is known without sampling] 
  //        IsOnBoundarySimple()           // opt: ParamPt on ParamBnd Predicate.   Default:[GetNaturalDomain().IsPoint3dOnBoundary], implement if:[ParamDomain is not rectilinear], 
  //        IsPointInParamDomainSimple()   // opt: rtn TRUE if ParamPoint is in map's NaturalParamDomain. default:[if bnded(GetNaturalDomain().Contains(ParamPt)) else TRUE], implement if:[ParamDomain is bounded not rectilinear]                                   
  //        IsPointInProjDomainSimple()    // opt: rtn TRUE if ProjPoint drops within map's NaturalParamDomain. default:[if bnded(InvEvaluatePointSimple()) else TRUE], implement if:[ParamDomain is bounded not rectilinear]
  //        IsLineInParamDomainSimple()    // opt: rtn TRUE if ParamLine is in map's NaturalParamDomain. default:[if bnded(GetNaturalDomain().Contains(ParamPt)) else TRUE], implement if:[ParamDomain is bounded not rectilinear]                                   
  //   ToBe IsLineInProjDomainSimple()     // opt: rtn TRUE if ProjLine drops within map's NaturalParamDomain. default:[if bnded(InvEvaluatePointSimple()) else TRUE], implement if:[ParamDomain is bounded not rectilinear]
  //               
  //        IsCurveInParamDomainSimple()   // opt: return TRUE when TgtCurve does NOT cross any Volume external or internal discontinuity boundary
  //        IsSurfaceInParamDomainSimple() // opt: return TRUE when TgtSurface does NOT cross any Volume external or internal discontinuity boundary
  //        HasDiscontinuitiesSimple()     // opt: Return TRUE if Simple map has internal C1 discontinuities, implement if:[TRUE], default:[return FALSE]
  //        CalculateContinuitiesSimple()  // opt: Return list of all Simple map internal discontinuities,    implement if:[map has discontinuities], default:[none]
  //                   
  //     // Persistence and reporting      
  //        WriteToDB()                    // req: virtual nested write i/o
  //        static ReadFromDB()            // req: static nested read i/o
  //        GetMemoryUsed()                // opt: required when derived class has any members. Default:[this + NextMap + OrientMap + InvOrientMap + attributes]
  //        Dump()                         // req: nested pretty print
  //        Draw()                         // opt: 
  //        DrawMesh()                     // opt: 
  //        DrawControlPoints()            // opt: 
  //        DrawUVW()                      // opt: 
  //        AssertValid()                  // req: 
  
  // compute ProjSpace BBoxes for given ParamSpace Box
  virtual SmStatus EvaluateBoundingBoxSimple
  (
    const SmExtent3d  & crParamBox,                       ///< [in ]: ParamSpace BBox to project to Project Space                                                           <br>
    SmPseudoBox       * pOptParamPseudoBox = NULL,        ///< [in ]: optional ParamSpace PseudoBox used to set output PseudoBox orientations,                              <br>
                                                          ///<      : NULL   : ProjPseudoBox Basis = Project X Y Z ParamVecs to ProjSpace at crParamBox Center              <br>
                                                          ///<      : NotNULL: ProjPseudoBox Basis = Project pOptParamPseudoBox BasisVecs to ProjSpace at crParamBox Center <br>
                                                          ///<      : default:[NULL]                                                                                        <br>
    SmExtent3d        * pProjBox = NULL,                  ///< [out]: ProjectSpace Axis aligned box                                                                         <br>
    SmPseudoBox       * pProjPseudoBox = NULL             ///< [out]: ProjectSpace Non-axis aligned box
  )  const ; 

  // compute ProjSpace BBoxes for given ParamSpace PseudoBox
  virtual SmStatus EvaluatePseudoBoxSimple
  (
    const SmPseudoBox & crParamPseudoBox,                 ///< [in ]: ParamSpace BBox to project to Project Space    <br>
    SmExtent3d        * pProjBox = NULL,                  ///< [out]: ProjectSpace Axis aligned box                  <br>
    SmPseudoBox       * pProjPseudoBox = NULL             ///< [out]: ProjectSpace Non-axis aligned box              <br>
  ) const ; 

  // Create ProjSpace IsoCurve from a ParamSpace IsoParamLine - requires derived implementation
  virtual SmStatus EvaluateIsoParametricCurveSimple
  (
    const SmContext     & crContext,            ///< [in ]: context for created objects                                <br>
    SmVolumeParamsType    eConstantParams,      ///< [in ]: oneof: SM_VPS_UV_IN,                                       <br>
                                                ///<      :        SM_VPS_UW_IN,                                       <br>
                                                ///<      :        SM_VPS_VW_IN.                                       <br>
    double                dIsoParam1,           ///< [in ]: 1st constant SM_VP_U_IN or SM_VP_V_IN parameter value      <br>
    double                dIsoParam2,           ///< [in ]: 2nd constant SM_VP_V_IN or SM_VP_W_IN parameter value      <br>
    double                d3DTolerance,         ///< NotUsed: [in ]: Max ApproxCurve to IdealCurve deviation                    <br>
    SmCurve            *& rpNewIsoCurve,        ///< [out]: the ProjSpace IsoCurve                                     <br>
    const SmExtent3d    * pOptParamDomain=NULL  ///< [in ]: limiting domain, NULL to ignore. default:[NULL]            <br>
  ) const ;

  // Create ProjSpace IsoSurface from a ParamSpace IsoParamPlane - requires derived implementation
  virtual SmStatus EvaluateIsoParametricSurfaceSimple
  (
    const SmContext   & crContext,              ///< [in ]: context for created objects                               <br>
    SmVolumeParamType   eConstantParam,         ///< [in ]: oneof: SM_VP_U_IN,                                        <br>
                                                ///<      :        SM_VP_V_IN,                                        <br>
                                                ///<      :        SM_VP_W_IN                                         <br>
    double              dIsoParam,              ///< [in ]: constant param value                                      <br>
    double              d3DTolerance,           ///< NotUsed: [in ]: Max ApproxSurface to IdealSurface deviation               <br>
    SmSurface        *& rpNewIsoSurface,        ///< [out]: the ProjSpace IsoSurface                                  <br>
    const SmExtent3d  * pOptParamDomain=NULL    ///< [in ]: limiting domain, NULL to ignore. default:[NULL]           <br>
  ) const ;

  // compute ProjPoint = EvaluateSimple(ParamPoint) - requires derived implementation
  virtual SmStatus EvaluateSimple 
  (
    const SmPoint3d & crParamPoint,         ///< [in ]: ParamSpace point to map to ProjSpace point                                     <br>
    ULONG             lHighestDeriv,        ///< [in ]: 0-Pos Only, 1=Pos+1st Derivs, ..., max 3                                       <br>
    SmBoolean         bUFromLeft,           ///< [in ]: if P is on U, V, or w interval boundary                                        <br>
    SmBoolean         bVFromLeft,           ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval    <br>
    SmBoolean         bWFromLeft,           ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval   <br>
    SmVector3d      * aDerivatives,         ///< [out]: matrix of ParamSpace evaluations values                                        <br>
                                            ///<      : 3d organized: D[u][v][w]                                                       <br>
                                            ///<      : 1d organized: D[i], i = u*n*n+v*n+w, for lHighestDeriv from 0 to 3             <br>
                                            ///<      : sized       : [n+1][n+1][n+1], where n=lHighesDeriv                            <br>
                                            ///<      : lHghDrv = 0,   sized: [1],                                                     <br>
                                            ///<      :   i=0          order: [D]                                                      <br>
                                            ///<      : lHghDrv = 1,   sized: [8]                                                      <br>
                                            ///<      :   i=u*4+v*2+w  order: [D  Dw  Dv  ---                                          <br>
                                            ///<      :                        Du --- --- ---]                                         <br>
                                            ///<      : lHghDrv = 2,   sized: [27]                                                     <br>
                                            ///<      :   i=u*9+v*3+w  order: [D   Dw  Dww Dv  Dvw --- Dvv --- ---                     <br>
                                            ///<      :                        Du  Duw --- Duv --- --- --- --- ---                     <br>
                                            ///<      :                        Duu --- --- --- --- --- --- --- ---]                    <br>
                                            ///<      : lHghDrv = 3,   sized: [81]                                                     <br>
                                            ///<      :   i=u*16+v*4+w order: [D   Dw   Dww   Dwww  Dv   Dvw  Dvww ---  Dvv  Dvw       <br>
                                            ///<      :                        --- ---  Dvvv  ---   ---  ---  Du   Duw  Duww ---       <br>
                                            ///<      :                        Duv Duvw ---   ---   Duvv ---  ---  ---  ---  ---       <br>
                                            ///<      :                        --- ---  Duu   Duuw  ---  ---  Duuv ---  ---  ---       <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  Duuu ---       <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---       <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---       <br>
                                            ///<      :                        --- ---  ---   ---   ---  ---  ---  ---  ---- ---       <br>
                                            ///<      :                        --- --- ]                                               <br>
    SmBoolean bNonZeroTangents=TRUE,        ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors   <br>
                                            ///<      : FALSE= return exact tangent values, default:[TRUE]                             <br>
                                            ///<      : note: Surprisingly TRUE is the common choice because most tangent uses         <br>
                                            ///<      :       are for their direction (Binorm, SurfNorm comps), but when the           <br>
                                            ///<      :       tangent is being used for its magnitude (like an arc-length comp)        <br>
                                            ///<      :       then set this to FALSE.                                                  <br>
                                            ///<      : default:[TRUE]                                                                 <br>
    SmBoolean bDoZeroSampling=TRUE          ///< [in ]: for internal use only, always set to TRUE, default:[TRUE]                      <br>
  ) const 
  { 
    SM_REF5(crParamPoint, lHighestDeriv, bUFromLeft, bVFromLeft, bWFromLeft) ; 
    SM_REF3(aDerivatives, bNonZeroTangents, bDoZeroSampling) ; 
    SE(SM_ERR) ; return(SM_ERR) ;  
  }

  // position of ProjPoint = EvaluateSimple(ParamPoint) 
  SmStatus EvaluatePointSimple
  (
    const SmPoint3d & crParamPoint,        ///< [in ]: point to map from ParamSpace           <br>
    SmPoint3d & rProjPoint                 ///< [out]: mapped point in ProjSpace              <br>
  ) const ; 

  // convenience function using EvaluateDirectionalDerivsSimple to compute just rInDir.  
  // An EvaluateDirectionalDerivs call is more general. 
  //   Ex: one EvaluateDirectionalDerivs() call can replace an EvaluatePoint() and EvaluateVector() pair of calls for half the cost.
  SmStatus EvaluateVectorSimple
  (
    const SmPoint3d & crParamPoint,    ///< [in ]: Param Point to evaluate                                                         <br>
    SmVector3d      & rParamDir,       ///< [in ]: Param direction for derivatives, gets unitized                                  <br>
    SmVector3d      & rProjDir,        ///< [out]: ProjDir = 1st directional derivative for given unit-vector ParamDir             <br>
    SmBoolean         bUFromLeft=TRUE, ///< [in ]: if P is on U, V, or W interval boundary                                         <br>
    SmBoolean         bVFromLeft=TRUE, ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval     <br>
    SmBoolean         bWFromLeft=TRUE  ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval    <br>
  ) const                               
  { 
    SmVector3d aDirDerivs[2] ;
    SER(EvaluateDirectionalDerivsSimple(crParamPoint, rParamDir, 1, aDirDerivs, bUFromLeft, bVFromLeft, bWFromLeft)) ;
    rProjDir = aDirDerivs[1] ;
    return(SM_SUCCESS) ;
  }

  // 1st derivs of ProjPoint = EvaluateSimple(ParamPoint) 
  SmStatus Evaluate1stDerivativesSimple
  (
    const SmPoint3d & crParamPoint,   ///< [in ]: ParamSpace point to map to ProjSpace point                                      <br>
    SmBoolean         bUFromLeft,     ///< [in ]: if P is on U, V, or W interval boundary                                         <br>
    SmBoolean         bVFromLeft,     ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval     <br>
    SmBoolean         bWFromLeft,     ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval    <br>
    SmPoint3d       & rProjPoint,     ///< [out]: ProjSpace Point                                                                 <br>
    SmVector3d      & rDU,            ///< [out]: U direction tangent in ProjSpace                                                <br>
    SmVector3d      & rDV,            ///< [out]: V direction tangent in ProjSpace                                                <br>
    SmVector3d      & rDW,            ///< [out]: W direction tangent in ProjSpace                                                <br>
    SmBoolean bNonZeroTangents=TRUE   ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors    <br>
                                      ///<      : FALSE= return exact tangent values, default:[TRUE]                              <br>
  ) const ;                           

  // 2nd derivs of ProjPoint = EvaluateSimple(ParamPoint)
  SmStatus Evaluate2ndDerivativesSimple
  (
    const SmPoint3d & crParamPoint,   ///< [in ]: ParamSpace point to map to OutSpace Point                                       <br>
    SmBoolean         bUFromLeft,     ///< [in ]: if P is on U, V, or W interval boundary                                         <br>
    SmBoolean         bVFromLeft,     ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval     <br>
    SmBoolean         bWFromLeft,     ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval    <br>
    SmPoint3d       & rProjPoint,     ///< [out]: position in ProjSpace                                                           <br>
    SmVector3d      & rDU,            ///< [out]: U direction tangent in ProjSpace                                                <br>
    SmVector3d      & rDV,            ///< [out]: V direction tangent in ProjSpace                                                <br>
    SmVector3d      & rDW,            ///< [out]: W direction tangent in ProjSpace                                                <br>
    SmVector3d      & rDUU,           ///< [out]: U direction 2nd derivative in ProjSpace                                         <br>
    SmVector3d      & rDVV,           ///< [out]: V direction 2nd derivative in ProjSpace                                         <br>
    SmVector3d      & rDWW,           ///< [out]: W direction 2nd derivative in ProjSpace                                         <br>
    SmVector3d      & rDUV,           ///< [out]: UV 2nd order cross derivative in ProjSpace                                      <br>
    SmVector3d      & rDUW,           ///< [out]: UW 2nd order cross derivative in ProjSpace                                      <br>
    SmVector3d      & rDVW,           ///< [out]: VW 2nd order cross derivative in ProjSpace                                      <br>
    SmBoolean bNonZeroTangents=TRUE   ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors    <br>
                                      ///<      : FALSE= return exact tangent values, default:[TRUE]                              <br>
  ) const;

  // directional derives for OutPoint = EvaluateSimple(ParamPoint) (in ProjSpace)
  SmStatus EvaluateDirectionalDerivsSimple
  (
    const SmPoint3d & crParamPoint,    ///< [in ]: Param Point to evaluate                                                        <br>
    SmVector3d      & rParamDir,       ///< [in ]: direction for derivatives, gets unitized                                       <br>
    ULONG             lHighestDeriv,   ///< [in ]: 1=1st deriv, 2=1st and 2nd derivs, 3=1st, 2nd, and 3rd derivs                  <br>
    SmVector3d        aDirDerivs[],    ///< [out]: aDirDerivs[0] = position                                                       <br>
                                       ///<      : aDirDerivs[1] = 1st directional derivative                                     <br>
                                       ///<      : aDirDerivs[2] = 2nd directional derivative                                     <br>
                                       ///<      : aDirDerivs[3] = 3rd directional derivative                                     <br>
                                       ///<      : sized:[lHighestDeriv+1]                                                        <br>
    SmBoolean         bUFromLeft=TRUE, ///< [in ]: if P is on U, V, or W interval boundary                                        <br>
    SmBoolean         bVFromLeft=TRUE, ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval    <br>
    SmBoolean         bWFromLeft=TRUE  ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval   <br>
  ) const ;

  // InvertMap point from ProjSpace back to ParamSpace
  SmStatus InvEvaluatePointSimple
  (
    const SmPoint3d     & crProjPoint,          ///< [in ]: ProjSpace Point to map back to InSpace Point                                <br>
    const SmPoint3d     * pOptParamPointGuess,  ///< [in ]: last intermediate ParamSpace guess location at which to start the search    <br>
    SmBoolean           & rbSuccess,            ///< [out]: TRUE = inverse was found, else FALSE                                        <br>
    SmTArray<SmPoint3d> & rParamSpacePoints,    ///< [out]: the ParamSpace inverse mapping                                              <br>
    SmTArray<double>    & rdGaps,               ///< [out]: max distance between found result (in case of bounding) and target point    <br>
    const SmExtent3d    * pOptParamDomain=NULL  ///< [in ]: ParamSpace domain over which to search for the inverse point                <br>
                                                ///<      : NULL = use Map's NaturalDomain, default:[NULL]                              <br>
  ) const ;                                                                                                                   
                                     

  // InvertMap vector from OutSpace back to ParamSpace (from final OutSpace with compounding)
  SmStatus InvEvaluateVectorSimple
  (
    const SmVector3d     & crProjSpaceVec,     ///< [in ]: ProjSpace Vector to map back to InSpace Vector                                         <br>
    const SmPoint3d      & crParamPoint,       ///< [in ]: ParamSpace Point specifying where the Drop will take place (see InvEvaluatePoint())    <br>
    SmTArray<SmVector3d> & rParamVecs          ///< [out]: Dropped ParamSpace vecs (2 if crParamPoint maps through seam)                          <br>
                                               ///<      : note: with rParamVecs[ii] = [ParamVecU, ParamVecV, ParamVecW]                          <br>
                                               ///<      : rOutSpaceVec   = ParamVecU*rDX + ParamVecV*rDY + ParamVecW*rDZ                         <br>
                                               ///<      : where rDX, rDY, rDZ are the outpspace 1stDeriv vectors at crParamPoint                 <br>
  ) const;

  // get approximate ParamSpace location for given ProjSpace point for upcoming newton raphson search
  virtual SmStatus InvEvaluateGuessPointSimple
  (
    const SmPoint3d     & crProjPoint,            ///< [in ]: ProjSpace Point to map back to ParamSpace Point                       <br>
    SmTArray<SmPoint3d> & rGuessParamPoints       ///< [out]: ParamSpace Point near Target Point actual map back to ParamSpace      <br>
  ) const   
  { 
    SM_REF2(crProjPoint, rGuessParamPoints) ; 
    SE(SM_ERR) ; 
    return(SM_ERR) ;  
  }
  
  // General Point Solver without a start guess from ProjSpace to ParamSpace based on NewtonRaphson. implement if:[NR can be avoided with an exact inverse mapping]
#define InvEvaluateSimple GlobalPointSolveSimple

  virtual SmStatus GlobalPointSolveSimple
  (
    const SmPoint3d  & crProjPoint,          ///< [in ]: ProjSpace Point to map back to ParamSpace                                            <br>
    SmBoolean        & rbFoundAnswer,        ///< [out]: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point                     <br>
    SmSolutionArray  & rSolutions,           ///< [out]: Contains ParamSpace Found Point                                                      <br>
                                             ///<      : rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                          <br>                   
                                             ///<      : rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint); <br>                   
                                             ///<      : rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location         <br>         
                                             ///<      : rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location         <br>         
                                             ///<      : rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location         <br>         
    const SmExtent3d * pOptParamDomain=NULL  ///< [in ]: ParamSpace domain over which to search for the inverse point                         <br>
                                             ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                       <br>
  ) const;

  // Local Point Solver with a start guess from ProjSpace to ParamSpace based on NewtonRaphson or exact inversion.
  virtual SmStatus LocalPointSolveSimple
  (
    const SmPoint3d  & crProjPoint,          ///< [in ]: ProjSpace Point to map back to ParamSpace                                            <br>
    const SmPoint3d  & crParamPointGuess,    ///< [in ]: ParamSpace point guess, the closer to the actual ParamSpace point the better         <br>
    SmBoolean        & rbFoundAnswer,        ///< [out]: TRUE = successfully mapped ProjSpace Point to a ParamSpace Point                     <br>
    SmSolution       & rSolution,            ///< [out]: Contains ParamSpace Found Point                                                      <br>
                                             ///<      : rSolution[i].m_eSolutionType           = SM_ST_SINGLE_VALUE                          <br>                    
                                             ///<      : rSolution[i].m_vStart.m_dSolutionValue = PointOnVolume.DistanceBetween(crProjPoint); <br>                    
                                             ///<      : rSolution[i].m_vStart[0] =  U of [U,V,W] the found ParamSpace point location         <br>          
                                             ///<      : rSolution[i].m_vStart[1] =  V of [U,V,W] the found ParamSpace point location         <br>          
                                             ///<      : rSolution[i].m_vStart[2] =  W of [U,V,W] the found ParamSpace point location         <br>          
    const SmExtent3d * pOptParamDomain=NULL  ///< [in ]: ParamSpace domain over which to search for the inverse point                         <br>
                                             ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                       <br>
  ) const;

  // Find InSpace Line interval portion that maps to the NaturalParamDomain 
  virtual SmStatus FindParamIntervalForInSpaceLineSimple
  (
    const SmPoint3d      & crInSpacePoint,   ///< [in ]: LinePoint         of Line(s) = LinePoint + s * LineVector         <br>      
    const SmVector3d     & crInSpaceVector,  ///< [in ]: scaled LineVector of Line(s) = LinePoint + s * LineVector         <br>      
    const SmExtent1d     & crCurrentIvl,     ///< [in ]: current limits on s interval                                      <br>
    SmTArray<SmExtent1d> & rTrimIvls         ///< [out]: Interval of line that maps legally within the NaturalParamDomains <br>
  ) const ;

  // GWC: exclude FindParamExtentForInSpacePlaneSimple from first release. Code is written but I don't
  //      see the bug in SmBendVolume::FindParamExtentForInSpacePlaneSimple().  I'll fix that later.
  //      // Find the largest square extent of an InSpace plane that maps inside the NaturalParamDomain.
  //      virtual SmStatus FindParamExtentForInSpacePlaneSimple                 // rtn: SM_ERR for NULL plane, else SM_SUCCESS
  //                                  (const SmPoint3d      & crInSpacePoint,   ///< [in ]: LinePoint          of Line(s) = LinePoint + u * LineVecU + v * LineVecV              
  //                                   const SmVector3d     & crInSpaceVecU,    ///< [in ]: scaled LineVectorU of Line(s) = LinePoint + u * LineVecU + v * LineVecV              
  //                                   const SmVector3d     & crInSpaceVecV,    ///< [in ]: scaled LineVectorV of Line(s) = LinePoint + u * LineVecU + v * LineVecV              
  //                                   const SmExtent2d     & crCurrentUV,      ///< [in ]: current limits on UV extent
  //                                   SmTArray<SmExtent2d> & rTrimUVs)         ///< [out]: UVExtent of plane that maps legally within the NaturalParamDomains
  //                                  const                                     { SM_REF3(crInSpacePoint, crInSpaceVecU, crInSpaceVecV) ;
  //                                                                              rTrimUVs.SetSize(1) ;
  //                                                                              if(&crCurrentUV != &rTrimUVs[0]) { rTrimUVs[0] = crCurrentUV ; }
  //                                                                              return(SM_SUCCESS) ;
  //                                                                            }              

  // Trim ParamSpace BBox to Natural Parameter Domain - implement if:[DerivedMap ParamDomain not rectilinear)
  // rtn: SM_ERR when ParamBox trims to empty set, else SM_SUCCESS
  virtual SmStatus TrimParamBoundingBoxSimple                     
 (
    const SmExtent3d & crParamBox,              ///< [in ]: Tgt Param Box to trim                       <br>
    SmExtent3d       & rParamTrimBox            ///< [out]: Box trimmed to Natural Param Domain         <br>
 ) const ;

//     // GWC - removed from interface because it made no sense: Even very simple cases of trimming a BBox to a long skinny PseduoBox showed
//     //       that the inscribed Box could be very small and non-unique.  It won't necessarily capture the notion of a good
//     //       bounding box designed to help narrow down the space of options for functions that need to check that.  In general,
//     //       check wether individual InSpace and ProjPace Points map back to legal ParamDomain Points, one point at a time. 
//     // Trim an InSpace BBox so that it maps totally within the Natural ParamSpace Domain
       // virtual SmStatus TrimInSpaceBoundingBoxSimple                   // rtn: SM_ERR when InSpaceBox trims to empty set, else SM_SUCCESS
       //                (const SmExtent3d & crInSpaceBox,                ///< [in ]: Tgt InSpace Box to trim              
       //                 SmExtent3d       & rInSpaceTrimBox)             ///< [in ]: Box trimmed so that it maps totally within the Natural ParamSpace Domain
       //                const                                            { if(&crInSpaceBox != &rInSpaceTrimBox) 
       //                                                                     { rInSpaceTrimBox = crInSpaceBox ; }
       //                                                                   return(SM_SUCCESS) ;
       //                                                                 }


  // Predicate Methods

  // return TRUE if this simple volume's ParamSpace is bounded
  virtual SmBoolean IsBoundedSimple    () const { return(FALSE) ; }               

  virtual SmBoolean IsClosedSimple     
  (
    SmBoolean        & rbClosedU,             ///< [out]: TRUE = closed in U direction, [check Pos[Umin,v,w] == Pos[Umax,v,w] for v,w samples         <br>
    SmBoolean        & rbClosedV,             ///< [out]: TRUE = closed in V direction, [check Pos[u,Vmin,w] == Pos[u,Vmax,w] for w,u samples         <br>
    SmBoolean        & rbClosedW,             ///< [out]: TRUE = closed in W direction, [check Pos[u,v,Wmin] == Pos[u,v,Wmax] for u,v samples         <br>
    double           * pdOptTolerance=NULL,   ///< [out]: pdOptTolerance = Tolerance to allow for check, NULL=dScaledZero(Volume), default:[NULL]     <br>
    const SmExtent3d * pOptParamDomain=NULL,  ///< [in ]: ParamSpace domain over which to search for the inverse point                                <br>
                                              ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                              <br>
    SmContinuityType * peOptContinuityU=NULL, ///< [out]: U dir Continuity when closed, NULL to igmore, default:[NULL]                                <br>
    SmContinuityType * peOptContinuityV=NULL, ///< [out]: V dir Continuity when closed, NULL to igmore, default:[NULL]                                <br>
    SmContinuityType * peOptContinuityW=NULL  ///< [out]: W dir Continuity when closed, NULL to igmore, default:[NULL]                                <br>
  ) const ;

  virtual SmBoolean IsPeriodicSimple   
  (
    SmBoolean        & rbPeriodicU,           ///< [out]: TRUE = G1 or better in U Dir                                                               <br>
    SmBoolean        & rbPeriodicV,           ///< [out]: TRUE = G1 or better in V Dir                                                               <br>
    SmBoolean        & rbPeriodicW,           ///< [out]: TRUE = G1 or better in W Dir                                                               <br>
    const SmExtent3d * pOptParamDomain=NULL   ///< [in ]: ParamSpace domain over which to search for the inverse point                               <br>
                                              ///<      : NULL = use Map's NaturalDomain, default:[NULL]                                             <br>
  ) const;

  virtual SmBoolean IsSingularitySimple
  (
    const SmPoint3d  & crParamPoint,         ///< [in ]: ParamSpace Point to test                             <br>
    SmBoolean        & rbSingularU,          ///< [out]: TRUE = [Wu=0]                                        <br>
    SmBoolean        & rbSingularV,          ///< [out]: TRUE = [Wv=0]                                        <br>
    SmBoolean        & rbSingularW,          ///< [out]: TRUE = [Ww=0]                                        <br>
    double             d3dTol=SM_EFF_ZERO    ///< NotUsed: [in ]: min ProjSpace distance between distinct points       <br>
  ) const ;

  virtual SmBoolean IsOnBoundarySimple 
  (
    const SmPoint3d  & crParamPoint,         ///< [in ]: Volume UVWPoint to test                                          <br>
    SmBoolean        & rbOnU,                ///< [out]: TRUE = TargetPoint on UMin or UMax                               <br>
    SmBoolean        & rbOnV,                ///< [out]: TRUE = TargetPoint on VMin or VMax                               <br>
    SmBoolean        & rbOnW,                ///< [out]: TRUE = TargetPoint on WMin or WMax                               <br>
    double           * pdOptTolerance=NULL,  ///< [in ]: max deviation allowed for point on seam                          <br>
    const SmExtent3d * pOptParamDomain=NULL  ///<      : NULL = use SM_EFF_ZERO * 1000 * (1 + maxDimension())             <br>
                                             ///< [in ]: ParamSpace domain over which to search for the inverse point     <br>
                                             ///<      : NULL = use Map's NaturalDomain, default:[NULL]                   <br>
  ) const;

  // rtn: TRUE if ParamPoint is within the maps paramSpace Natural domain 
  virtual SmBoolean IsPointInParamDomainSimple(const SmPoint3d  & crParamPoint) const                              
  { 
    if(IsBoundedSimple()) { return(GetNaturalParamDomain().ContainsPoint3d(crParamPoint) ) ; } 
    return(TRUE) ;
  } 
                                                                                 
  // rtn: TRUE if projPoint can be inverted uniquely to ParamSpace
  virtual SmBoolean IsPointInProjDomainSimple
  (
    const SmPoint3d  & crProjPoint,    ///< [in ]:                                                              <br>
    SmPoint3d   * pOptParamPoint=NULL  ///< [out]: if the point is invertible, go ahead and get the inverse     <br>
  ) const                               
  { 
    if(IsBoundedSimple() || pOptParamPoint) 
    { 
      SmBoolean bSuccess = false;
      SmTArray<SmPoint3d> sParamPoints ;
      SmTArray<double>    sGaps ; 
      InvEvaluatePointSimple(crProjPoint, NULL, bSuccess, sParamPoints, sGaps) ;
      if(bSuccess && pOptParamPoint) { *pOptParamPoint = sParamPoints[0] ; }
      return(bSuccess) ;
    }
    return(TRUE) ;
  }

  // rtn: TRUE if ParamPoint is within the maps paramSpace Natural domain 
  virtual SmBoolean IsLineInParamDomainSimple
  (
    const SmPoint3d  & crStartParamPoint,                                   
    const SmPoint3d  & crEndParamPoint
  ) const                               
  { 
    if(IsBoundedSimple()) 
    { 
      return(GetNaturalParamDomain().ContainsLineSeg3d(crStartParamPoint, crEndParamPoint) ) ; 
    } 
    return(TRUE) ; 
  } 
                                                                                 
  // ToBe implemented: base implementations on projecting line to ParamSpace and testing IsCurveInParamDomainSimple 
  //  virtual SmBoolean IsLineInProjDomainSimple(const SmPoint3d  & crStartProjPoint,    // rtn: TRUE if projPoint can be inverted uniquely to ParamSpace                                   
  //                                             const SmPoint3d  & crEndProjPoint,      ///< [in ]: LineSeg End projPoint
  //                                             SmPoint3d        * pOptStartParamPoint, ///< [out]: if the point is invertible, go ahead and get the inverse
  //                                             SmPoint3d        * pOptEndParamPoint)   ///< [out]: if the point is invertible, go ahead and get the inverse
  //                                            const                                    { // GWC: following code needs to be replaced
  //                                                                                       // if(IsBoundedSimple() || pOptStartParamPoint || pOptEndParamPoint) 
  //                                                                                       //   { SmBoolean bStartSuccess, bEndSuccess ;
  //                                                                                       //     SmTArray<SmPoint3d> sStartParamPoints, sEndParamPoints ;
  //                                                                                       //     SmTArray<double>    sGaps ; 
  //                                                                                       //     InvEvaluatePointSimple(crStartProjPoint, NULL, bStartSuccess, sStartParamPoints, sGaps) ;
  //                                                                                       //     InvEvaluatePointSimple(crEndProjPoint, NULL, bEndSuccess, sEndParamPoints, sGaps) ;
  //                                                                                       //     if(bStartSuccess && pOptStartParamPoint) { *pOptStartParamPoint = sStartParamPoints[0] ; }
  //                                                                                       //     if(bEndSuccess   && pOptEndParamPoint)   { *pOptEndParamPoint   = sEndParamPoints[0] ; }
  //                                                                                       //     return(bStartSuccess && bEndSuccess) ;
  //                                                                                       //   }
  //                                                                                       // return(TRUE) ; 
  //                                                                                     }

  virtual SmBoolean IsCurveInParamDomainSimple(const SmCurve & crParamCurve) const       
  {
    SM_REF1(crParamCurve) ;
    SE_MSG(SM_ERR,_T("")) ; 
    return FALSE ; 
  }

  virtual SmBoolean IsSurfaceInParamDomainSimple(const SmSurface & crParamSurface) const 
  { 
    SM_REF1(crParamSurface) ;
    SE_MSG(SM_ERR,_T("")) ; 
    return FALSE ; 
  } 

  // rtn: TRUE if Simple map has internal C1 discontinuities
  virtual SmBoolean HasDiscontinuitiesSimple                                    
  (
    SmDiscontinuities3d * pOptDisconts=NULL,                 ///< [out]: list of ParamSpace discontinuities and summary data                                    <br>
    SmBoolean             bCalcGeometric=TRUE                ///< NotUsed: [in ]: TRUE = expensive - use geometric testing to compute actal geometric discontinuities    <br>
                                                             ///<      : FALSE= cheap - report representational discontinuites                                  <br>
  )  const ;                                                   

  virtual SmStatus  CalculateContinuitiesSimple
  (
    SmVolumeParamType            eVolumeParam,               ///< [in ]: SM_VP_U, SM_VP_V, or SM_VP_W                                                          <br>
    SmContinuityType           & reMinContinuity,            ///< [out]: minimum continuity over all interior knots                                            <br>
    SmTArray<double>           & rParams,                    ///< [out]: param values marking discontinuity                                                    <br>
    SmTArray<SmContinuityType> & rConts,                     ///< [out]: assocaited continuity type for each rParams value                                     <br>
                                                             ///<      : end param continuities = SM_CT_DISCONTINUOUS                                          <br>
    SmBoolean                    bCalcGeometric=TRUE         ///< [in ]: TRUE = expensive - use geometric testing to compute actal geometric discontinuities   <br>
                                                             ///<      : FALSE= cheap - report representational discontinuites                                 <br>
                                                             ///<      : default:[TRUE]                                                                        <br>
  )  const
  { 
    SM_REF2(eVolumeParam, bCalcGeometric) ;
    reMinContinuity = SM_CT_CINFINITY ;
    rParams.ReSet() ;
    rConts.ReSet() ;
    return(SM_SUCCESS) ;
  }

  //  AffineSimple virtual methods: 
  //   The default behavior for all AffineSimple methods is to build and append an  
  //   SmTransform mapping to the end of the Volume's m_pNextMap compounded map list.
  //   When possible, derived classes should improve evaluation performance by
  //   implementing these virtual methods to modify their SimpleMap parameters so that
  //
  //     Transform(Orient(SimpleMap(ParamPt))) = Orient(ModifiedSimpleMap(ParamPt)) 
  //     InvOrient(Transform(Orient(SimpleMap(ParamPt) = ModifiedSimpleMap(ParamPt)
  //
  //   Users should not call the AffineSimple methods directly.  Leave the efficiency issues
  //   to the derived implementations by calling the Affine methods directly.

  // derived classes modify SimpleMap params so: Orient(ModifiedSimpleMap(uvw)) = Transform(Orient(SimpleMap(uvw))) 

  // Orient(ModSimpleMap(uvw)) = Transform(Orient(SimpleMap(uvw)))
  virtual SmStatus  TransformSimple
  (
    const SmAxis2Placement & crOutRotateNMove,       ///< [in ]: OutSpace Rotate and Move Transform                                <br>
    const SmVector3d       * cpOptOutScale = NULL    ///< [in ]: Optional OutSpace scale factors applied after Rotate and Move     <br>
  ) ;

 
  // Orient(ModSimpleMap(uvw)) = Translate(Orient(SimpleMap(uvw)))
  virtual SmStatus  TranslateSimple(const SmVector3d & crOutTranslate) ;           

  // Orient(ModSimpleMap(uvw)) = Mirror   (Orient(SimpleMap(uvw)))
  virtual SmStatus  MirrorSimple   
  (
    const SmPoint3d  & crPlaneOutPt,          ///< [in ]: OutSpace Pt on Mirror Plane           <br>
    const SmVector3d & crPlaneOutNormal       ///< [in ]: OutSpace Normal to Mirror Plane       <br>
  ) ;        

  // Orient(ModSimpleMap(uvw)) = Scale    (Orient(SimpleMap(uvw)))
  virtual SmStatus  ScaleSimple    
  (
    const SmVector3d & crScaleOutVec,          ///< [in ]: OutSpace scale factors                    <br>
    const SmPoint3d  * cpOptOutCenter = NULL   ///< [in ]: Optional OutSpace scaling center point    <br>
  ) ; 

  // Orient(ModSimpleMap(uvw)) = RotateAboutAxis(Orient(SimpleMap(uvw)))
  virtual SmStatus  RotateAboutAxisSimple
  (
    double             dAngRad,                ///< [in ]: OutSpace rotation AngRad                        <br>
    const SmVector3d & crOutAxis               ///< [in ]: OutSpace rotation axis through OutSpace origin  <br>
  ) ;  

  // Orient(ModSimpleMap(uvw)) = RotateAboutAxisAtPoint(Orient(SimpleMap(uvw)))
  virtual SmStatus  RotateAboutAxisAtPointSimple 
  (
    double             dAngRad,                ///< [in ]:       <br>
    const SmPoint3d  & crOutOrigin,            ///< [out]:       <br>
    const SmVector3d & crOutAxis               ///< [out]:       <br>
  ) ;

  // utilities

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmVolume, SmAObject, SmVolume_TYPE);

  // write volume to file
  SmStatus WriteToFile
  (
    const TCHAR * cOutputFileName,                ///< [in ]: target file name                                <br>
    SmBoolean     bSkipHeaderWrite = FALSE,       ///< NotUsed: [in ]: TRUE = Omit "Volume Type: TYPE" header label    <br>
    SmBoolean     bNewFile = FALSE,               ///< [in ]: TRUE = open file and rewrite contents           <br>
                                                  ///<      : FALSE= open file and append to end              <br>
                                                  ///<      : default:[FALSE]                                 <br>
    SmBoolean     bWriteAttributes = FALSE        ///< [in ]: TRUE=Write attributes, FALSE=don't              <br>
                                                  ///<      : default:[FALSE]                                 <br>
  ) const ;   
  
  // write array of volumes to file                                          
  static SmStatus WriteArrayToFile
  (
    const TCHAR                * cOutputFileName,           ///< [in ]: target file name                     <br>      
    const SmTArray<SmVolume *> & rVolArr,                   ///< [in ]: array of volumes to write            <br>
    SmBoolean                    bWriteAttributes=FALSE     ///< [in ]: TRUE=Write attributes, FALSE=don't   <br>
  );

  // read a volume from file
  static SmStatus ReadFromFile
  (
    const SmContext & crContext,                           ///< [in ]: context for new object construction                                          <br>
    const TCHAR     * cInputFileName,                      ///< [in ]: target file                                                                  <br>
    SmVolume       *& rpNewVolume,                         ///< [out]: NULL on input = new object allocated in this routine built from stream data  <br>
                                                           ///<      : NotNULL on input = pointer to an empty object to be filled by this routine   <br>
    const ULONG       lFileOffsetInBytes = 0               ///< NotUsed: [in ]: Number of characters in file to skip before reading                          <br>
                                                           ///<      : default:[0]                                                                  <br>
  );

  // read array of volumes from file
  static SmStatus ReadArrayFromFile
  (
    const SmContext      & crContext,                     ///< [in ]: context for new object construction                       <br>
    const TCHAR          * cInputFileName,                ///< [in ]: target file                                               <br>
    SmTArray<SmVolume *> & rNewVolumes,                   ///< [out]: the read volume with newly allocated memory               <br>
    const ULONG            lFileOffsetInBytes = 0,        ///< NotUsed: [in ]: Number of characters in file to skip before reading)      <br>
                                                          ///<      : default:[0]                                               <br>
    SmTArray<SmVolume *> * pTestVolumes = NULL            ///< [in ]: for debug only - the array of curves expected to be read  <br>
                                                          ///<      : NULL to ignore. Default:[NULL]                            <br>
  );

  // I/O assist methods
  virtual SmStatus WriteToDB
  (
    SmDatabaseIO & rDB,                                   ///< [in ]: target output stream                                   <br>
    ULONG          lDBVersionNumber                       ///< [in ]: database version to get proper sequence of writes      <br>                                                                 
  ) const;

  static  SmStatus ReadFromDB
  (
    SM_TYPE           lType,                ///< [in ]: Object type to be read                                                       <br>
    SmDatabaseIO    & rDB,                  ///< [in ]: target output stream                                                         <br>
    const SmContext & crContext,            ///< [in ]: context for new object construction                                          <br>
    SmVolume       *& rpNewVolume,          ///< [out]: NULL on input = new object allocated in this routine built from stream data  <br>
                                            ///<      : NotNULL on input = pointer to an empty object to be filled by this routine   <br>
    ULONG             lDBVersionNumber      ///< [in ]: database version to get proper sequence of writes                            <br>
  );

  // get memory used for volume and its attributes
  virtual ULONG    GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,           ///< [out]: bigger size of all allocated memory in bytes         <br>
    SmMarkType eMarkType=SM_MT_NOMARK       ///< [in ]: uses without increment eMarkType value               <br>
  ) const ; 

  // pretty print SmVolume
  virtual void     Dump(SmBoolean bAbbrev, ULONG lIndentCnt) const ;

  // add volume graphics to display state
  // Draw Volume icon of IsoPlanes and IsoLines
  virtual SmDisplayList * Draw       
  (
    SmBoolean       bShowOutSpace=TRUE,       ///< [in ]: TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]         <br>
    SmBoolean       bShowInSpace = FALSE,     ///< [in ]: TRUE = Draw InSpace Volume graphics, FALSE=don't ,default:[FALSE]         <br>
    SmExtent3d    * pOptParamDomain = NULL,   ///< [in ]: optional limiting param domain, NULL to ignore, default:[NULL]            <br>
    SmGfxArraySet * pOptGfxSet = NULL         ///< [in,out]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.<br>
                                              ///<      : NULL to ignore. default:[NULL]                                            <br>
  ) const;

  // Draw InSpace and/or OutSpace Point for Param Point
  SmDisplayList * DrawAtParamPoint
  (
    const SmPoint3d & sParamPoint,             ///< [in ]: target ParamSpace Point                                                     <br>
    ULONG             lNumDeriv = 0,           ///< [in ]: optional derivative count                                                   <br>
                                               ///<      : 0 = only draw point                                                         <br>
                                               ///<      : 1 = draw point, 1st derivatives (dW/du, dW/dv, dW/dw)                       <br>
    SmBoolean         bShowOutSpace = TRUE,    ///< [in ]: TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]           <br>
    SmBoolean         bShowInSpace = FALSE,    ///< [in ]: TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]           <br>
    SmGfxArraySet   * pOptGfxSet = NULL        ///< [in,out]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.  <br>
                                               ///<      : NULL to ignore. default:[NULL]                                              <br>
  ) const;

  // Draw OutSpace Point for InSpace Point
  SmDisplayList * DrawAtInSpacePoint
  (
    const SmPoint3d & sInSpacePoint,           ///< [in ]: target InSpace Point                                                         <br>
    ULONG             lNumDeriv = 0,           ///< [in ]: optional derivative count                                                    <br>
                                               ///<      : 0 = only draw point                                                          <br>
                                               ///<      : 1 = draw point, 1st derivatives (dW/du, dW/dv, dW/dw)                        <br>
    SmGfxArraySet   * pOptGfxSet = NULL        ///< [in,out]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.   <br>
                                               ///<      : NULL to ignore. default:[NULL]                                               <br>
  ) const;

  // Draw OutSpace ControlMesh and/or InSpace IsoLine Grid
  virtual SmDisplayList * DrawMesh
  (
    SmBoolean       bShowOutSpace=TRUE,        ///< [in ]: TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]           <br>
    SmBoolean       bShowInSpace = FALSE,      ///< [in ]: TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]           <br>
    SmGfxArraySet * pOptGfxSet = NULL          ///< [in,out]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.  <br>
                                               ///<      : NULL to ignore. default:[NULL]                                              <br>
  ) const;          

  // Draw OutSpace ControlPoints and/or InSpace IsoLine Grid
  virtual SmDisplayList * DrawControlPoints
  (
    SmBoolean       bShowOutSpace=TRUE,       ///< [in ]: TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]            <br>
    SmBoolean       bShowInSpace = FALSE,     ///< [in ]: TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]            <br>
    SmGfxArraySet *pOptGfxSet = NULL          ///< [in,out]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.   <br>
                                              ///<      : NULL to ignore. default:[NULL]                                               <br>
  ) const; 

  // Draw Grid of IsoCurves in InSpace and/or OutSpace
  virtual SmDisplayList * DrawUVW
  (
    ULONG             lNumBetweenU=3,                ///< [in ]: number of U IsoParameter lines between knots                               <br>
    ULONG             lNumBetweenV = 3,              ///< [in ]: number of V IsoParameter lines between knots                               <br>
    ULONG             lNumBetweenW = 3,              ///< [in ]: number of W IsoParameter lines between knots                               <br>
    SmBoolean         bVaryCrossHatchColor = FALSE,  ///< [in ]: TRUE = Draw U Lines in ObjectColor                                         <br>
                                                     ///<      :        Draw V lines in m_VaryCrossHatchColor                               <br>
                                                     ///<      : FALSE= Draw both U and V Lines in ObjectColor                              <br>
                                                     ///<      : default:[FALSE]                                                            <br>
    const SmExtent3d *pOptParamDomain = NULL,        ///< [in ]: ParamDomain to crossHatch, NULL=NaturalParamDomain, default:[NULL]         <br>
    SmBoolean         bShowOutSpace = TRUE,          ///< [in ]: TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]          <br>
    SmBoolean         bShowInSpace = FALSE,          ///< [in ]: TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]          <br>
    SmGfxArraySet    *pOptGfxSet = NULL              ///< [in,out]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls. <br>
                                                     ///<      : NULL to ignore. default:[NULL]                                             <br>
  ) const; 
                                                      
  SmDisplayList * DrawVectorField
  (
    const SmExtent3d &crParamDomain,                ///< NotUsed: [in ]: volume domain                                                                 <br>
    ULONG lNumUPoints,                              ///< [in ]: number of U points                                                            <br>
    ULONG lNumVPoints,                              ///< [in ]: number of V point                                                             <br>
    ULONG lNumWPoints,                              ///< [in ]: number of W points                                                            <br>
    SmPinCushionType eType,                         ///< [in ]: oneof: SM_DM_POINTS                                                           <br>
                                                    ///<      :        SM_DM_U_NATURAL                                                        <br>
                                                    ///<      :        SM_DM_V_NATURAL                                                        <br>
                                                    ///<      :        SM_DM_W_NATURAL                                                        <br>
                                                    ///<      :        SM_DM_UVW_NATURAL                                                      <br>
                                                    ///<      :        SM_DM_U_SCALED                                                         <br>
                                                    ///<      :        SM_DM_V_SCALED                                                         <br>
                                                    ///<      :        SM_DM_W_SCALED                                                         <br>
                                                    ///<      :        SM_DM_UVW_SCALED                                                       <br>
    SmBoolean        bShowOutSpace = TRUE,          ///< [in ]: TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]             <br>
    SmBoolean        bShowInSpace = FALSE,          ///< [in ]: TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]             <br>
    SmGfxArraySet   *pOptGfxSet = NULL              ///< [in,out]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.    <br>
                                                    ///<      : NULL to ignore. default:[NULL]
  ) const;

  SmStatus OutputGraphics
  (
    const SmDisplayParameters & crDisp,                   ///< [in ]: display control parameters                                                        <br>
    const SmExtent3d          * pOptParamDomain=NULL,     ///< NotUsed: [in ]: ParamDomain to crossHatch, NULL=NaturalParamDomain                                <br>
    SmBoolean                   bShowOutSpace=TRUE,       ///< [in ]: TRUE = Draw OutSpace Volume graphics, FALSE=don't, default:[TRUE]                 <br>
    SmBoolean                   bShowInSpace=FALSE,       ///< [in ]: TRUE = Draw InSpace Volume graphics, FALSE=don't, default:[FALSE]                 <br>
    SmPoint3d                 * pOptUnboundedCenter=NULL, ///< [in ]: Center of Interest for Unbounded Domains, NUll to ignore,                         <br>
    double                      dUnboundedHalfSize=33,    ///< [in ]: Unbounded half space display size. Unbounded volumes displayed                    <br>
                                                          ///<      : over a domain cube twice this size, default:[33]                                  <br>
    SmGfxArraySet             * pOptGfxSet=NULL           ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.   <br>
                                                          ///<      : NULL to ignore. default:[NULL]                                                    <br>
  ) const;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                             <br>
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in ]: SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                         <br>
                                              ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                     <br>
                                              ///<      : default:[SM_LEVEL_0]                                                                               <br>
    SmAssertWalking    eWalkTree=SM_WALK,     ///< [in ]: SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]   <br>
    SmTArray<ULONG>  * pTestRequests=NULL     ///< [in ]: when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                   <br>
  ) const ;                                                                                                                                               

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

} ; // end class SmVolume

/*******************************************************************//**
PURPOSE: I/O object for SmVolume::HasDiscontinities

NOTES: this is a container object only
***********************************************************************/
class SM_EXPORT SmDiscontinuities3d
{
public:
  SmBoolean          m_bHasDiscontinuities = FALSE ;         // TRUE  = Has discontinuties
                                                             // FALSE = Doesn't
  SmContinuityType   m_eMinContinuity = SM_CT_UNDEFINED ;    // min continuity seen in entire mapping
                                                             
  SmBoolean                  m_bUnalignedU = FALSE ;         // TRUE=has discontinuities not aligned with a ParamPlane, 
                                                             // FALSE=has all discontinuities are aligned with VW ParamPlanes.
  SmContinuityType           m_eMinContU = SM_CT_UNDEFINED ; // when m_bUnalignedU == FALSE, minimum continuity over all interior U knots
  SmTArray<double>           m_sParamsU = {0} ;              // when m_bUnalignedU == FALSE, param locations for U dir discontinuities
  SmTArray<SmContinuityType> m_sContsU ;                     // when m_bUnalignedU == FALSE, Continuity value for each unique U param value
                                                             //                              end domain continuities = SM_CT_DISCONTINUOUS
                                                             
  SmBoolean                  m_bUnalignedV = FALSE ;         // TRUE=has discontinuities not aligned with a ParamPlane, 
                                                             // FALSE=has all discontinuities are aligned with UW ParamPlanes.
  SmContinuityType           m_eMinContV = SM_CT_UNDEFINED ; // when m_bUnalignedV == FALSE, minimum continuity over all interior V knots
  SmTArray<double>           m_sParamsV ;                    // when m_bUnalignedV == FALSE, param locations for V dir discontinuities
  SmTArray<SmContinuityType> m_sContsV ;                     // when m_bUnalignedV == FALSE, Continuity value for each unique U param value
                                                             //                              end domain continuities = SM_CT_DISCONTINUOUS
                                                             
  SmBoolean                  m_bUnalignedW = FALSE ;         // TRUE=has discontinuities not aligned with a ParamPlane, 
                                                             // FALSE=has all discontinuities are aligned with UV ParamPlanes.
  SmContinuityType           m_eMinContW = SM_CT_UNDEFINED ; // when m_bUnalignedW == FALSE, minimum continuity over all interior W knots
  SmTArray<double>           m_sParamsW ;                    // when m_bUnalignedW == FALSE, param locations for W dir discontinuities
  SmTArray<SmContinuityType> m_sContsW ;                     // when m_bUnalignedW == FALSE, Continuity value for each unique U param value
                                                             //                              end domain continuities = SM_CT_DISCONTINUOUS
public:

  // constructor
  SmDiscontinuities3d() { Init() ; }

  // copy constructor
  SmDiscontinuities3d(const SmDiscontinuities3d &crOther) ;

  // assignment operator
  SmDiscontinuities3d & operator=(const SmDiscontinuities3d &crOther) ;

  // init - assume no discontinuities
  void Init() { m_bHasDiscontinuities = FALSE ;
                m_eMinContinuity      = SM_CT_CINFINITY ;
                m_bUnalignedU = FALSE ;  m_eMinContU = SM_CT_CINFINITY ;  m_sParamsU.ReSet() ;  m_sContsU.ReSet() ;          
                m_bUnalignedV = FALSE ;  m_eMinContV = SM_CT_CINFINITY ;  m_sParamsV.ReSet() ;  m_sContsV.ReSet() ;          
                m_bUnalignedW = FALSE ;  m_eMinContW = SM_CT_CINFINITY ;  m_sParamsW.ReSet() ;  m_sContsW.ReSet() ;         
              } 

  // set discontinuous
  void SetUnaligned() { m_bHasDiscontinuities = TRUE ;
                        m_eMinContinuity      = SM_CT_C0 ;
                        m_bUnalignedU = TRUE ;  m_eMinContU = SM_CT_C0 ;  m_sParamsU.ReSet() ;  m_sContsU.ReSet() ;          
                        m_bUnalignedV = TRUE ;  m_eMinContV = SM_CT_C0 ;  m_sParamsV.ReSet() ;  m_sContsV.ReSet() ;          
                        m_bUnalignedW = TRUE ;  m_eMinContW = SM_CT_C0 ;  m_sParamsW.ReSet() ;  m_sContsW.ReSet() ;         
                      } 

  // destructor
 ~SmDiscontinuities3d() { Init() ; }
  
} ; // end class SmDiscontinuities3d

/*******************************************************************//**
PURPOSE: Calc pConcatMap(uvw) = pSecondMap(pFirstMap(uvw)) 

NOTES: a chain rule implementation
***********************************************************************/
SmStatus smvol_ConcatEvals
( 
  ULONG        lHighestDeriv,  ///< [in ]: highest desired derivative,                                                       <br>
                               ///<      : 0 = position                                                                      <br>
                               ///<      : 1 = position & 1st Derivs                                                         <br>
                               ///<      : 2 = position, 1st Derivs, 2nd Derivs                                              <br>
                               ///<      : 3 = position, 1st Derivs, 2nd Derivs, 3rd Derivs                                  <br>
  SmVector3d * pFirstMap,      ///< [in ]: FirstMap(UVW) evals,           of pConcatMap(uvw) = pSecondMap(pFirstMap(uvw))    <br>
  SmVector3d * pSecondMap,     ///< [in ]: SecondMap(pFirstMap[0]) evals, of pConcatMap(uvw) = pSecondMap(pFirstMap(uvw))    <br>
  SmVector3d * pConcatMap      ///< [in ]: ConcatMap(UVW) evals,          of pConcatMap(uvw) = pSecondMap(pFirstMap(uvw))    <br>
);


// Declare Compound derivative functions for curves = D(C(s)), surfaces D(S(u,v)) and volumes D(F(u,v,w))
// Vs   = smvol_LiftFirstDerivative(Cs, Vol 1st Derivs)
//        smvol_LiftFirstDerivative(Srf 1st uv Derivs, Vol 1st xyz Derivs,      out: Vol u,v 1st derivs)
//        smvol_LiftFirstDerivative(BaseVol 1st uvw Derivs, Vol 1st xyz Derivs, out: BaseVol u,v,w 1st derivs)

// Vss  = smvol_LiftSecondDerivative(Cs, Css, Vol 1st & 2nd Derivs)
//        smvol_LiftSecondDerivative(Srf 1st & 2nd uv Derivs, Vol 1st & 2nd xyz Derivs,      out: Vol u,v 1st & 2nd derivs)
//        smvol_LiftSecondDerivative(BaseVol 1st & 2nd uvw Derivs, Vol 1st & 2nd xyz Derivs, out: BaseVol u,vw 1st & 2nd derivs)

// Vsss = smvol_LiftThirdDerivative(Cs, Css, Csss, Vol 1st, 2nd, & 3rd Derivs)
//        smvol_LiftThirdDerivative(Srf 1st, 2nd & 3rd uv Derivs, Vol 1st, 2nd & 3rd xyz Derivs,      out: Vol u,v 1st, 2nd & 3rd derivs)
//        smvol_LiftThirdDerivative(BaseVol 1st, 2nd & 3rd uvw Derivs, Vol 1st, 2nd & 3rd xyz Derivs, out: BaseVol u,v,w 1st, 2nd & 3rd derivs)

/*******************************************************************//**
PURPOSE: Compute 1st derivative of CrvInVol(s) = D(C(s)) 
         using the chain rule given C'(s) 
         and all the D(u,v,w) 1st derivatives .

NOTES:  D = Volume(u,v,w) - called D (not V) to not be confused with parameter 'v'
***********************************************************************/
SmVector3d smvol_LiftFirstDerivative
(
  const SmVector3d & crCs,     ///< [in ]: Curve 1st deriv               <br>

  const SmVector3d & crDu,     ///< [in ]: Volume 1st U deriv            <br>
  const SmVector3d & crDv,     ///< [in ]: Volume 1st V deriv            <br>
  const SmVector3d & crDw      ///< [in ]: Volume 1st W deriv            <br>
);

/*******************************************************************//**
PURPOSE: Compute 2nd derivative of CrvInVol(s) = D(C(s)) 
         using the chain rule given C'(s) and C''(s) 
         and all the D(u,v,w) 1st and 2nd derivatives .

NOTES:  D = Volume(u,v,w) - called D (not V) to not be confused with parameter 'v'
***********************************************************************/
SM_EXPORT SmVector3d smvol_LiftSecondDerivative  // rtn: directional 2nd derivative 3d vector
(
  const SmVector3d & crCs,                      ///< [in ]: Curve 1st deriv          <br>
  const SmVector3d & crCss,                     ///< [in ]: Curve 2nd deriv          <br>

  const SmVector3d & crDu,                      ///< [in ]: Volume 1st U deriv       <br>
  const SmVector3d & crDv,                      ///< [in ]: Volume 1st V deriv       <br>
  const SmVector3d & crDw,                      ///< [in ]: Volume 1st W deriv       <br>

  const SmVector3d & crDuu,                     ///< [in ]: Volume 2nd UU deriv      <br>
  const SmVector3d & crDuv,                     ///< [in ]: Volume 2nd UV deriv      <br>
  const SmVector3d & crDuw,                     ///< [in ]: Volume 2nd UW deriv      <br>
  const SmVector3d & crDvv,                     ///< [in ]: Volume 2nd VV deriv      <br>
  const SmVector3d & crDvw,                     ///< [in ]: Volume 2nd VW deriv      <br>
  const SmVector3d & crDww                      ///< [in ]: Volume 2nd WW deriv      <br>
);

/*******************************************************************//**
PURPOSE: Compute 3rd derivative of CrvInVol(s) = D(C(s)) 
         using the chain rule given C'(s), C''(s), and C'''(s) 
         and all the D(u,v,w) 1st, 2nd, and 3rd derivatives .

NOTES:  D = Volume(u,v,w) - called D (not V) to not be confused with parameter 'v'
        rtn: directional 2nd derivative 3d vector
***********************************************************************/
SM_EXPORT SmVector3d smvol_LiftThirdDerivative   
(
  const SmVector3d & crCs,                      ///< [in ]: Curve 1st deriv            <br>
  const SmVector3d & crCss,                     ///< [in ]: Curve 2nd deriv            <br>
  const SmVector3d & crCsss,                    ///< [in ]: Curve 3rd deriv            <br>

  const SmVector3d & crDu,                      ///< [in ]: Volume 1st U deriv         <br>
  const SmVector3d & crDv,                      ///< [in ]: Volume 1st V deriv         <br>
  const SmVector3d & crDw,                      ///< [in ]: Volume 1st W deriv         <br>

  const SmVector3d & crDuu,                     ///< [in ]: Volume 2nd UU deriv        <br>
  const SmVector3d & crDuv,                     ///< [in ]: Volume 2nd UV deriv        <br>
  const SmVector3d & crDuw,                     ///< [in ]: Volume 2nd UW deriv        <br>
  const SmVector3d & crDvv,                     ///< [in ]: Volume 2nd VV deriv        <br>
  const SmVector3d & crDvw,                     ///< [in ]: Volume 2nd VW deriv        <br>
  const SmVector3d & crDww,                     ///< [in ]: Volume 2nd WW deriv        <br>
  
  const SmVector3d & crDuuu,                    ///< [in ]: Volume 3rd UUU deriv       <br>
  const SmVector3d & crDuuv,                    ///< [in ]: Volume 3rd UUV deriv       <br>
  const SmVector3d & crDuuw,                    ///< [in ]: Volume 3rd UUW deriv       <br>
  const SmVector3d & crDuvv,                    ///< [in ]: Volume 3rd UVV deriv       <br>
  const SmVector3d & crDuvw,                    ///< [in ]: Volume 3rd UVW deriv       <br>
  const SmVector3d & crDuww,                    ///< [in ]: Volume 3rd UWW deriv       <br>
  const SmVector3d & crDvvv,                    ///< [in ]: Volume 3rd VVV deriv       <br>
  const SmVector3d & crDvvw,                    ///< [in ]: Volume 3rd VVW deriv       <br>
  const SmVector3d & crDvww,                    ///< [in ]: Volume 3rd VWW deriv       <br>
  const SmVector3d & crDwww                     ///< [in ]: Volume 3rd WWW deriv       <br>
);

/*******************************************************************//**
PURPOSE: Compute 1st derivative of SrfInVol(u,v) = D(S(u,v)) 
         using the chain rule given Su(u,v) and Sv(u,v)
         and all the D(x,y,z) 1st derivatives .

NOTES:  D = Volume(x,y,z) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v'
***********************************************************************/
void smvol_LiftFirstDerivative
(
  const SmVector3d & crSu,     ///< [in ]: surface 1st u deriv                <br>
  const SmVector3d & crSv,     ///< [in ]: surface 1st v deriv                <br>

  const SmVector3d & crDx,     ///< [in ]: Volume 1st X deriv                 <br>
  const SmVector3d & crDy,     ///< [in ]: Volume 1st Y deriv                 <br>
  const SmVector3d & crDz,     ///< [in ]: Volume 1st Z deriv

  SmVector3d       & rDu,      ///< [out]: Volume 1st u deriv                 <br>
  SmVector3d       & rDv       ///< [out]: Volume 1st v deriv                 <br>
);

/*******************************************************************//**
PURPOSE: Compute 2nd derivative of SrfInVol(u,v) = D(S(u,v)) 
         using the chain rule given Su, Sv, Suu, Suv, and Svv
         and all the D(x,y,z) 1st and 2nd derivatives .

NOTES:  D = Volume(x,y,z) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v'
***********************************************************************/
void smvol_LiftSecondDerivative
(
  const SmVector3d & crSu,     ///< [in ]: surface 1st u deriv              <br>
  const SmVector3d & crSv,     ///< [in ]: surface 1st v deriv              <br>

  const SmVector3d & crSuu,    ///< [in ]: surface 2nd UU deriv             <br>
  const SmVector3d & crSuv,    ///< [in ]: surface 2nd UV deriv             <br>
  const SmVector3d & crSvv,    ///< [in ]: surface 2nd VV deriv             <br>

  const SmVector3d & crDx,     ///< [in ]: Volume 1st X deriv               <br>
  const SmVector3d & crDy,     ///< [in ]: Volume 1st Y deriv               <br>
  const SmVector3d & crDz,     ///< [in ]: Volume 1st Z deriv               <br>

  const SmVector3d & crDxx,    ///< [in ]: Volume 2nd XX deriv              <br>
  const SmVector3d & crDxy,    ///< [in ]: Volume 2nd XY deriv              <br>
  const SmVector3d & crDxz,    ///< [in ]: Volume 2nd XZ deriv              <br>
  const SmVector3d & crDyy,    ///< [in ]: Volume 2nd YY deriv              <br>
  const SmVector3d & crDyz,    ///< [in ]: Volume 2nd YZ deriv              <br>
  const SmVector3d & crDzz,    ///< [in ]: Volume 2nd ZZ deriv              <br>

  SmVector3d       & rDuu,     ///< [out]: volume 2nd UU deriv              <br>
  SmVector3d       & rDuv,     ///< [out]: volume 2nd UV deriv              <br>
  SmVector3d       & rDvv      ///< [out]: volume 2nd VV deriv              <br>
);

/*******************************************************************//**
PURPOSE: Compute 3rd derivative of SrfInVol(u,v) = D(S(u,v)) 
         using the chain rule given Su, Sv, Suu, Suv, Svv,
         Suuu, Suuv, Suvv, Svvv
         and all the D(x,y,z) 1st, 2nd, and 3rd derivatives .

NOTES:  D = Volume(x,y,z) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v'
***********************************************************************/
void smvol_LiftThirdDerivative
(
  const SmVector3d & crSu,     ///< [in ]: surface 1st u deriv           <br>
  const SmVector3d & crSv,     ///< [in ]: surface 1st v deriv           <br>

  const SmVector3d & crSuu,    ///< [in ]: surface 2nd XX deriv           <br>
  const SmVector3d & crSuv,    ///< [in ]: surface 2nd XY deriv           <br>
  const SmVector3d & crSvv,    ///< [in ]: surface 2nd YY deriv           <br>
                                                             
  const SmVector3d & crSuuu,   ///< [in ]: surface 3rd XXX deriv          <br>
  const SmVector3d & crSuuv,   ///< [in ]: surface 3rd XXY deriv          <br>
  const SmVector3d & crSuvv,   ///< [in ]: surface 3rd XYY deriv          <br>
  const SmVector3d & crSvvv,   ///< [in ]: surface 3rd YYY deriv          <br>
                                                             
  const SmVector3d & crDx,     ///< [in ]: Volume 1st X deriv             <br>
  const SmVector3d & crDy,     ///< [in ]: Volume 1st Y deriv             <br>
  const SmVector3d & crDz,     ///< [in ]: Volume 1st Z deriv             <br>
                                                             
  const SmVector3d & crDxx,    ///< [in ]: Volume 2nd XX deriv            <br>
  const SmVector3d & crDxy,    ///< [in ]: Volume 2nd XY deriv            <br>
  const SmVector3d & crDxz,    ///< [in ]: Volume 2nd XZ deriv            <br>
  const SmVector3d & crDyy,    ///< [in ]: Volume 2nd YY deriv            <br>
  const SmVector3d & crDyz,    ///< [in ]: Volume 2nd YZ deriv            <br>
  const SmVector3d & crDzz,    ///< [in ]: Volume 2nd ZZ deriv            <br>
                                                             
  const SmVector3d & crDxxx,   ///< [in ]: Volume 3rd XXX deriv          <br>
  const SmVector3d & crDxxy,   ///< [in ]: Volume 3rd XXY deriv          <br>
  const SmVector3d & crDxxz,   ///< [in ]: Volume 3rd XXZ deriv          <br>
  const SmVector3d & crDxyy,   ///< [in ]: Volume 3rd XYY deriv          <br>
  const SmVector3d & crDxyz,   ///< [in ]: Volume 3rd XYZ deriv          <br>
  const SmVector3d & crDxzz,   ///< [in ]: Volume 3rd XZZ deriv          <br>
  const SmVector3d & crDyyy,   ///< [in ]: Volume 3rd YYY deriv          <br>
  const SmVector3d & crDyyz,   ///< [in ]: Volume 3rd YYZ deriv          <br>
  const SmVector3d & crDyzz,   ///< [in ]: Volume 3rd YZZ deriv          <br>
  const SmVector3d & crDzzz,   ///< [in ]: Volume 3rd ZZZ deriv          <br>

  SmVector3d       & rDuuu,    ///< [out]: Volume 3rd UUU deriv          <br>
  SmVector3d       & rDuuv,    ///< [out]: Volume 3rd UUV deriv          <br>
  SmVector3d       & rDuvv,    ///< [out]: Volume 3rd UVV deriv          <br>
  SmVector3d       & rDvvv     ///< [out]: Volume 3rd VVV deriv          <br>
);

/*******************************************************************//**
PURPOSE: Compute 1st derivative of VolInVol(u,v,w) = D(F(u,v,w)) 
         using the chain rule given Fu(u,v,w), Fv(u,v,w), and Fw(u,v,w)
         and all the D(x,y,z) 1st derivatives.

NOTES:  D = Volume(xyz) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v' and
        F = Volume(uvw) 
***********************************************************************/
void smvol_LiftFirstDerivative
(
  const SmVector3d & crFu,     ///< [in ]: BaseVolume 1st u deriv           <br>
  const SmVector3d & crFv,     ///< [in ]: BaseVolume 1st v deriv           <br>
  const SmVector3d & crFw,     ///< [in ]: BaseVolume 1st w deriv           <br>

  const SmVector3d & crDx,     ///< [in ]: CompoundingVolume 1st X deriv    <br>
  const SmVector3d & crDy,     ///< [in ]: CompoundingVolume 1st Y deriv    <br>
  const SmVector3d & crDz,     ///< [in ]: CompoundingVolume 1st Z deriv    <br>

  SmVector3d       & rDu,      ///< [out]: CompoundingVolume 1st u deriv    <br>
  SmVector3d       & rDv,      ///< [out]: CompoundingVolume 1st v deriv    <br>
  SmVector3d       & rDw       ///< [out]: CompoundingVolume 1st v deriv    <br>
);

/*******************************************************************//**
PURPOSE: Compute 2nd derivative of VolInVol(u,v,w) = D(F(u,v,w)) 
         using the chain rule given Fu(u,v,w), Fv(u,v,w), Fw(u,v,w), and
         F 2nd derivs and all the D(x,y,z) 1st and 2nd derivatives.

NOTES:  D = Volume(xyz) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v' and
        F = Volume(uvw)
***********************************************************************/
void smvol_LiftSecondDerivative
(
  const SmVector3d & crFu,     ///< [in ]: BaseVolume 1st u deriv          <br>
  const SmVector3d & crFv,     ///< [in ]: BaseVolume 1st v deriv          <br>
  const SmVector3d & crFw,     ///< [in ]: BaseVolume 1st w deriv          <br>

  const SmVector3d & crFuu,    ///< [in ]: BaseVolume 2nd XX deriv          <br>
  const SmVector3d & crFuv,    ///< [in ]: BaseVolume 2nd XY deriv          <br>
  const SmVector3d & crFuw,    ///< [in ]: BaseVolume 2nd XZ deriv          <br>
  const SmVector3d & crFvv,    ///< [in ]: BaseVolume 2nd YY deriv          <br>
  const SmVector3d & crFvw,    ///< [in ]: BaseVolume 2nd YZ deriv          <br>
  const SmVector3d & crFww,    ///< [in ]: BaseVolume 2nd ZZ deriv          <br>

  const SmVector3d & crDx,     ///< [in ]: CompoundingVolume 1st X deriv    <br>
  const SmVector3d & crDy,     ///< [in ]: CompoundingVolume 1st Y deriv    <br>
  const SmVector3d & crDz,     ///< [in ]: CompoundingVolume 1st Z deriv    <br>

  const SmVector3d & crDxx,    ///< [in ]: CompoundingVolume 2nd XX deriv     <br>
  const SmVector3d & crDxy,    ///< [in ]: CompoundingVolume 2nd XY deriv     <br>
  const SmVector3d & crDxz,    ///< [in ]: CompoundingVolume 2nd XZ deriv     <br>
  const SmVector3d & crDyy,    ///< [in ]: CompoundingVolume 2nd YY deriv     <br>
  const SmVector3d & crDyz,    ///< [in ]: CompoundingVolume 2nd YZ deriv     <br>
  const SmVector3d & crDzz,    ///< [in ]: CompoundingVolume 2nd ZZ deriv     <br>

  SmVector3d       & rDuu,     ///< [out]: CompoundingVolume 2nd UU deriv    <br>
  SmVector3d       & rDuv,     ///< [out]: CompoundingVolume 2nd UV deriv    <br>
  SmVector3d       & rDuw,     ///< [out]: CompoundingVolume 2nd UW deriv    <br>
  SmVector3d       & rDvv,     ///< [out]: CompoundingVolume 2nd VV deriv    <br>
  SmVector3d       & rDvw,     ///< [out]: CompoundingVolume 2nd VW deriv    <br>
  SmVector3d       & rDww      ///< [out]: CompoundingVolume 2nd WW deriv    <br>
);


/*******************************************************************//**
PURPOSE: Compute 3rd derivative of VolInVol(u,v,w) = D(F(u,v,w)) 
         using the chain rule given Fu(u,v,w), Fv(u,v,w), Fw(u,v,w), and
         F 2nd and 3rd derivs and all the D(x,y,z) 1st, 2nd, and 3rd derivatives.

NOTES:  D = Volume(xyz) - called D (not V) to not be confused with parameter 'v'
        and parameterized in x,y,z rather than u,v,w to not be confused with
        parameters 'u' and 'v' and
        F = Volume(uvw)
***********************************************************************/
void smvol_LiftThirdDerivative
(
  const SmVector3d & crFu,     ///< [in ]: BaseVolume 1st U deriv          <br>
  const SmVector3d & crFv,     ///< [in ]: BaseVolume 1st V deriv          <br>
  const SmVector3d & crFw,     ///< [in ]: BaseVolume 1st W deriv          <br>
                                                             
  const SmVector3d & crFuu,    ///< [in ]: BaseVolume 2nd UU deriv         <br>
  const SmVector3d & crFuv,    ///< [in ]: BaseVolume 2nd UV deriv         <br>
  const SmVector3d & crFuw,    ///< [in ]: BaseVolume 2nd UW deriv         <br>
  const SmVector3d & crFvv,    ///< [in ]: BaseVolume 2nd VV deriv         <br>
  const SmVector3d & crFvw,    ///< [in ]: BaseVolume 2nd VW deriv         <br>
  const SmVector3d & crFww,    ///< [in ]: BaseVolume 2nd WW deriv         <br>
                                                             
  const SmVector3d & crFuuu,   ///< [in ]: BaseVolume 3rd UUU deriv        <br>
  const SmVector3d & crFuuv,   ///< [in ]: BaseVolume 3rd UUV deriv        <br>
  const SmVector3d & crFuuw,   ///< [in ]: BaseVolume 3rd UUW deriv        <br>
  const SmVector3d & crFuvv,   ///< [in ]: BaseVolume 3rd UVV deriv        <br>
  const SmVector3d & crFuvw,   ///< [in ]: BaseVolume 3rd UVW deriv        <br>
  const SmVector3d & crFuww,   ///< [in ]: BaseVolume 3rd UWW deriv        <br>
  const SmVector3d & crFvvv,   ///< [in ]: BaseVolume 3rd VVV deriv        <br>
  const SmVector3d & crFvvw,   ///< [in ]: BaseVolume 3rd VVW deriv        <br>
  const SmVector3d & crFvww,   ///< [in ]: BaseVolume 3rd VWW deriv        <br>
  const SmVector3d & crFwww,   ///< [in ]: BaseVolume 3rd WWW deriv        <br>
                                                             
  const SmVector3d & crDx,     ///< [in ]: CompoundingVolume 1st X deriv   <br>
  const SmVector3d & crDy,     ///< [in ]: CompoundingVolume 1st Y deriv   <br>
  const SmVector3d & crDz,     ///< [in ]: CompoundingVolume 1st Z deriv   <br>
                                                             
  const SmVector3d & crDxx,    ///< [in ]: CompoundingVolume 2nd XX deriv   <br>
  const SmVector3d & crDxy,    ///< [in ]: CompoundingVolume 2nd XY deriv   <br>
  const SmVector3d & crDxz,    ///< [in ]: CompoundingVolume 2nd XZ deriv   <br>
  const SmVector3d & crDyy,    ///< [in ]: CompoundingVolume 2nd YY deriv   <br>
  const SmVector3d & crDyz,    ///< [in ]: CompoundingVolume 2nd YZ deriv   <br>
  const SmVector3d & crDzz,    ///< [in ]: CompoundingVolume 2nd ZZ deriv   <br>
                                                             
  const SmVector3d & crDxxx,   ///< [in ]: CompoundingVolume 3rd XXX deriv   <br>
  const SmVector3d & crDxxy,   ///< [in ]: CompoundingVolume 3rd XXY deriv   <br>
  const SmVector3d & crDxxz,   ///< [in ]: CompoundingVolume 3rd XXZ deriv   <br>
  const SmVector3d & crDxyy,   ///< [in ]: CompoundingVolume 3rd XYY deriv   <br>
  const SmVector3d & crDxyz,   ///< [in ]: CompoundingVolume 3rd XYZ deriv   <br>
  const SmVector3d & crDxzz,   ///< [in ]: CompoundingVolume 3rd XZZ deriv   <br>
  const SmVector3d & crDyyy,   ///< [in ]: CompoundingVolume 3rd YYY deriv   <br>
  const SmVector3d & crDyyz,   ///< [in ]: CompoundingVolume 3rd YYZ deriv   <br>
  const SmVector3d & crDyzz,   ///< [in ]: CompoundingVolume 3rd YZZ deriv   <br>
  const SmVector3d & crDzzz,   ///< [in ]: CompoundingVolume 3rd ZZZ deriv   <br>

  SmVector3d       & rDuuu,    ///< [out]: Volume 3rd UUU deriv         <br>
  SmVector3d       & rDuuv,    ///< [out]: Volume 3rd UUV deriv         <br>
  SmVector3d       & rDuuw,    ///< [out]: Volume 3rd UUW deriv         <br>
  SmVector3d       & rDuvv,    ///< [out]: Volume 3rd UVV deriv         <br>
  SmVector3d       & rDuvw,    ///< [out]: Volume 3rd UVW deriv         <br>
  SmVector3d       & rDuww,    ///< [out]: Volume 3rd UWW deriv         <br>
  SmVector3d       & rDvvv,    ///< [out]: Volume 3rd VVV deriv         <br>
  SmVector3d       & rDvvw,    ///< [out]: Volume 3rd VVW deriv         <br>
  SmVector3d       & rDvww,    ///< [out]: Volume 3rd VWW deriv         <br>
  SmVector3d       & rDwww     ///< [out]: Volume 3rd WWW deriv         <br>
);

/*******************************************************************//**
PURPOSE: MACRO to temporarily allow a volume of type
  SmBSplineVolume to make out-of-bounds evaluations.

NOTES: 
  1. pVolume is a pointer to a SmVolume which is checked for type
  2. n is a unique integer needed to create a tmp variable to allow
      the SmTemporaryChangeValue mechanism to be used for more than
      one SmVolume in a single name scope without generating a name
      clash
  3. Files using this macro must 
     #include <SmBSplineVolume.h>
***********************************************************************/

#define SM_VOLUME_ENABLE_OUTOFBOUNDS(pVolume,n)                  \
  SmBoolean tmpS_##n = 0 ;                                       \
  SmTemporaryChangeValue<SmBoolean> sSStack_##n                  \
       (   (pVolume)->IsKindOf(SmBSplineVolume_TYPE)             \
        ? ((SmBSplineVolume *)pVolume)->GetOutOfBoundsEnabled()  \
        : tmpS_##n,                                              \
        TRUE) ;
       
#endif // !__SMVOLUME_H__


