// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmAttribute.h
* PURPOSE: Header file for Attribute object
**********************************************************************/

#ifndef __SMATTRIBUTE_H__
#define __SMATTRIBUTE_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMTARRAH_H__
#include <SmTArray.h>
#endif

#ifndef __SMAOBJECT_H__
#include <SmAObject.h>
#endif

#ifndef __SMDATABASEIO_H__
#include <SmDatabaseIO.h>
#endif

#ifndef __SMMAPTYPETOTYPE_H__
#include <SmMapTypeToType.h>
#endif

#ifndef __SMCONTEXT_H__
#include <SmContext.h>
#endif

#ifndef __MUTEX__
#define __MUTEX__
#include <mutex>
#endif

#ifndef __ATOMIC__
#define __ATOMIC__
#include <atomic>
#endif

class SmRegion ;
enum SmBooleanOperationType ;

// GWC:BIND_TEMPLATES_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmAObject*) ;


// Currently supported attribute ID's.  
#define SM_AI_COLOR              100   
#define SM_AI_MARK               101
#define SM_AI_OFFSETMAP          102
#define SM_AI_GENERIC            103
#define SM_AI_UNKNOWN            104
#define SM_AI_SURFACEMAP         105
#define SM_AI_TAG                106
#define SM_AI_FILLET_EDGEMAP     107
#define SM_AI_FILLET_VERTMAP     108
#define SM_AI_FILLET_REGIONMAP   109
#define SM_AI_FILLET_EDGES       110
#define SM_AI_ALL_FILLET_EDGES   111
#define SM_AI_SSSPOINT           112
#define SM_AI_MICRO_COLOR        113
#define SM_AI_OTHER_REGION_ID    114  // supports Region attribute propagation through Boolean operations
#define SM_AI_FACE_ID            116  // only for testing attribute propagation from Brep to PolyBrep.
#define SM_AI_EDGE_ID            118  // only for testing attribute propagation from Brep to PolyBrep.
#define SM_AI_VERTEX_ID          120  // only for testing attribute propagation from Brep to PolyBrep.
#define SM_AI_REUSE_UVCURVES_ID  122  // JLMCC per FS, modified for consistency
#define SM_AI_TESSELLATING       124  // used as part of tessellator
#define SM_AI_REPLACE_COMPOSITES 126  // NonPersistent attribute used to replace CompositeFace tracking with Attribute tracking
                                      //   after a boolean every AObject with this attribute is a child of the AObject originally marked
                                      //   When tracking one AObject (a face) use SmAttribute, when tracking many faces use 
                                      //   SmLongAttribute or SmPointerAttribute so that children of different parents can be identified.

// The range 150-199 are used by SmMetric Derived classes
//  (SM_AI_METRIC_BASE == METRIC_BASE_TYPE)
#define SM_AI_METRIC_BASE      150

// Following are used by mass properties
// The properties attribute is saved as a SmVector3dAttribute
// with the following information embedded in the vector.
// vector.X - Density (Density * Volume = Mass)
// vector.Y - Thickness of a Face, Cross Section Radius of Edge Tube, Radius of a Vertex Sphere
// vectoy.Z - Not used
#define SM_AI_MASS_PROPERTIES  201

// Following are used by polygon's error-bounds
// The error-bounds attribute is saved as a SmVector3dAttribute
// with the following information embedded in the vector.
// vector.X - Lower(-) bound
// vector.Y - Upper(+) bound
// vectoy.Z - Not used
#define SM_AI_POLY_ERRORBOUNDS 301

// The following is used to contain data used in decimation
// and attached to edges or vertices.
#define SM_AI_VERTEX_DECIMATION_VALUES 302
#define SM_AI_QUADRIC_VERTEX_VALUES    303
#define SM_AI_QUADRIC_EDGE_VALUES      304
#define SM_AI_QUADRIC_VERTEX_N_VALUES  305

// used by SmMerge::ManifoldBoolean when SmMerge::m_bImprintAndClassifyFaces == TRUE.
// Faces normally saved are marked with an attribute with id SM_AI_BOOLEAN_SAVE
// Faces normally deleted are marked with an attribute with this Id rather than being deleted.
#define SM_AI_BOOLEAN_INTERSECT        307
#define SM_AI_BOOLEAN_SAVE             308
#define SM_AI_BOOLEAN_DELETE           309

// The following is used in tessellation to hold back pointers
// to the Brep Edgeuse and Vertexuse of a Polygon
#define SM_AI_POLY_TO_EUVU                     310
#define SM_AI_MESH_ATTRIBUTE                   311
#define SM_AI_TESSELATION_POLY_EDGE_VERTEX_USE 312

#define SM_AI_USER_1  10000
#define SM_AI_USER_2  20000
#define SM_AI_USER_3  30000

// These classes are primarily designed to provide an indexed representation
// for a SmPolyBrep class.  They are used in IGES translation and also used in
// I/O to ASCII files.  They are not documented and not considered public
// objects.  

/*******************************************************************//**
PURPOSE: 

NOTES:
 static class object - don't add virtual methods to SmAttributeData.  
 Virtual methods conflict with the methods in SmTArray<SmAttributeData>
 that use memset() to clear memory - with SmAttributeData virtual methods the
 virtual pointer tables get corrupted by SmTArray<SmAttributeData>::ReSet() 
 calls and the like.

***********************************************************************/
class SM_EXPORT SmAttributeData
{
public:
  SmTArray<ULONG> m_sAttributes;     // - Each topology object can have many attributes
                                     // - SmAttributeData is the base for the SmBrepData topology classes
                                     // - When data is moved between SmBrep and SmBrepData one global array
                                     //   of attributes is created containing all the attributes in the Brep model.
                                     // - Each SmTypeData object uses this list to store the indices
                                     //   of the attributes that belong to it that are stored in that 
                                     //   one global array of attributes.

public:
  // constructor, destructor
   SmAttributeData() { }
  ~SmAttributeData() { ReSet() ; } // clear m_sAttributes array

  // I/O
  static SmStatus WriteIndexedAttributesToDB
  (
    SmTArray<ULONG> & rAttributes,  // in : list of attribute indices   
    SmDatabaseIO    & rDB           // in : contains target stream      
  );

  static SmStatus ReadIndexedAttributesFromDB
  (
    const SmContext        & crContext,       // in : context for new object construction                 
    SmTArray<SmAttribute*> & rAllAttributes,  // out: accumulation array of all attribute objects         
                                              //    : initialized with a dummy attribute object for every 
                                              //    : attribute index value seen.                         
    SmTArray<ULONG>        & rAttributes,     // out: array of newly allocated attribute index values     
    SmDatabaseIO           & rDB              // in : contains target I/O stream                          
  );

  // simple access
  const TCHAR * GetTypeString() const { return _T("SmAttributeData") ; }

  // added ReSet - so that classes derived from SmAttributeData can be static classes
  void ReSet()   { m_sAttributes.SetSize(0) ; }

} ; // end class SmAttributeData

/*******************************************************************//**
PURPOSE: gather pObject's list of attributes into an indexed array
         
NOTES: The indexed array can be written to file with 
       SmAttributeData::WriteIndexedAttributesToDB()
***********************************************************************/
SM_EXPORT SmStatus sm_ExtractAttributes
(
  const SmContext                     & crContext,          // NotUsed: in : current context for object creation                      
  const SmAObject                     * pObject,            // in : target object potentially containing attributes                   
  SmTArray<ULONG>                     & rIndexedAttributes, // out: rAttributes index array for this object's attributes              
  SmTArray<SmAttribute*>              & rAttributes,        // i/o: accumulation of all attributes on all entities                 
  SmMapTypeToType<SmAttribute*,ULONG> & rAttrMap            // i/o: accumulation of [attribute,ObjectAttributeIndexValue+1] pairs  
) ;          

/*******************************************************************//**
PURPOSE: returns TRUE when the pAttributeToFind is in the rAttrMap
         
NOTES: 
***********************************************************************/
SM_EXPORT SmBoolean sm_FindAttribute
(
  SmAttribute                          * pAttributeToFind,  // in : target attribute pointer                          
  const SmTArray<SmAttribute*>         & rAttributes,       // in : array of attributes to search                     
  SmMapTypeToType<SmAttribute*, ULONG> & rAttrMap,          // in : list of [attribute,index] pairs to search         
  ULONG                                & rlFoundIndex       // out: index of pAttributeToFind in rAttributes array.   
) ;

/*******************************************************************//**
PURPOSE: This enum defines the different attribute behavior types
    that are available.

NOTES: 
***********************************************************************/
enum SmAttributeBehaviorType 
{
  SM_AB_COPY,                 // This is the default behavior.  Copy the attribute
                              // and maintain a 1-1 relationship between attribute and
                              // owner.
  SM_AB_REFERENCE,            // Don't copy the attribute just reference existing attribute
                              // and add it to the list of owners.
  SM_AB_STANDALONE_COPY,      // Copy      behavior without auto deletion within RemoveUser.
  SM_AB_STANDALONE_REFERENCE, // Reference behavior without auto deletion within RemoveUser.
  SM_AB_TEMP                  // Not Persistent, Not Copied - useful for single session entity marking
}  ;

/*******************************************************************//**
PURPOSE: This object is designed as an Attribute which can be
   placed on SmAObject and its subclasses.  There is a default set
   of attribute management techniques that come with the object to
   handle attribute propagation for operations applied to the attribute
   owner like splitting, copying, merging, deleting.

NOTES: 

   Rules and Hints:
   1. Attributes are created using their constructors then added, 
      removed, and retrieved from user-objects with calls to the SmAObject's methods as:

      SmAObject::AddAttribute(),
      SmAObject::RemoveAttribute(),
      SmAObject::FindAttribute(), and
      SmAObject::GetAttributes().

       Hint: attributes are given their derived type, AttributeId, 
             and SmAttributeBehaviorType values when constructed.
              
       1a. Once an attribute is constructed, it can be added to any
          number of user-objects.  Attributes are restricted at 
          construction-time so that they can either be associated 
          to just one user-object or to many.  
          The Attribute behavior allowing an attribute to
          connect to any number of user-objects is called 'REFERENCE'.  
          The behavior limiting an attribute to just one user-object is
          called 'COPY'.  This behavior is recorded explictly in the  
          SmAttribute's SmAttributeBehaviorType enum value as:
          
           SM_AB_COPY                 - one user-object per attribute
           SM_AB_STANDALONE_COPY      - one user-object per attribute
           SM_AB_REFERENCE            - many user-objects per attribute
           SM_AB_STANDALONE_REFERENCE - many user-objects per attribute

          The specification of the STANDALONE behavior controls when the attribute
          is deleted.  The default behavior is to delete an attribute when its
          user-list length goes to zero.  Adding the STANDALONE behavior by
          using the SM_AB_STANDALONE_COPY or SM_AB_STANDALONE_REFERENCE
          SmAttributeBehaviorType when constructing an attribute directs
          the system to not delete that attribute when its user-list length 
          goes to zero.  The calling application must delete STANDALONE  
          attribute objects when appropriate to avoid memory leaks.

           Hint: REFERENCE attributes are used to limit model size when
                 one attribute value can be used by a large number 
                 geometry objects.  COPY attributes are used when
                 each geometry object with that attribute will need
                 to store its own unique value.  Use COPY attributes
                 for things like giving each geometry object a unique
                 name.  Use REFERENCE attributes for things like assigning
                 the color BLUE to a large number of geometry objects.

       1b. Any number of attributes can be associated with one user-object
          as long as every attribute associated with one user-object has
          a unique attributeId number.
           Hint: attributeId numbers are used to represent different types
                 of attributes.  In this way a single derived Attribute
                 type can be used for several purposes by assigning each
                 use a unique attributeId value.
           Hint: The list of SMLib AttributeIds is maintained in file 
                 SmAttribute.h.
           Hint: System attributeIds are integer values less than 10000.  
                 To prevent conflicts between application and 
                 system attributeIds, applications should use applicationIds 
                 above 10000 and check in the SmAttribute.h file to 
                 make sure their ids are unique.

        1c. Example: add one-to-one user-to-attribute relationships
            for(ii=0;ii<sFaces.GetSize();ii++)
              {
                SmFace *pFace = sFaces[ii] ;

                // construct the derived attribute with desired data storage, proper Attribute ID and value.
                SmLongAttribute *pLongAttribute = new (crContext) SmLongAttribute(SM_AI_FACE_ID, // Attribute ID - unique per object
                                                                                  ii,            // Attribute value 
                                                                                  SM_AB_COPY) ;  // one user per attribute 

                // connect the attribute to the object
                pFace->AddAttribute(pLongAttribute) ;
              }

         1d. Example: add many-to-one users-to-attribute relationships
              
              // local
              SmVector3d sRedColor (1,0,0) ;
              SmVector3d sBlueColor(0,0,1) ;

              // construct the derived attributes with desired data storage, proper Attribute ID and value.
              SmVector3dAttribute *pRedAttribute  = new (crContext) SmVector3dAttribute(SM_AI_MICRO_COLOR, // Attribute ID - unique per object
                                                                                        sRedColor,         // Attribute value 
                                                                                        SM_AB_REFERENCE) ; // many users per attribute 
              SmVector3dAttribute *pBlueAttribute = new (crContext) SmVector3dAttribute(SM_AI_MICRO_COLOR, // Attribute ID - unique per object
                                                                                        sBlueColor,        // Attribute value 
                                                                                        SM_AB_REFERENCE) ; // many users per attribute 

              // connect the attributes to many objects
              for(i0=0,i1=1;i1<sFaces.GetSize();i0++,i1++)
                {
                  sFaces[i0]->AddAttribute(pRedAttribute) ;
                  sFaces[i1]->AddAttribute(pBlueAttribute) ;
                }

   2. An attribute/user-object relationship is stored as a pair of
      pointers; one pointer from the user-object to the attribute and
      another from the attribute back to the user-object.
       Hint: A single call to AddAttribute() creates and sets both of
             these pointer values.
       Hint: A single call to RemoveAttribute() removes both of these
             pointer values.

   3. Attributes with data are defined by deriving from the SmAttribute 
      class.  Attributes are distinguished by their attributeId values
      and not their derived type. 
        Hint: Every attribute in a single use-object's attribute list
              must have a unique attributeId value.
        Hint: If your application needs two different attributes that 
              both store a pointer and a long, define a derived type 
              of attribute that contains both a pointer and a long, 
              and define two different attributeID values that are 
              both intended to be used with this derived type.
        Hint: when deriving a type from SmAttribute, review all virtual 
              SmAttribute functions and decide which will need to be written
              in the derived class.
        Hint: make sure that all the virtual Get functions() are implemented
              properly.  When they are, derived SmAttribute classes 
              will enjoy inherited persistence (they can be written to and
              read from file).

       3a. Simple attributes classes are already defined which can be used 
          for most attribute applications or as templates for creating
          more complicated attributes. These include:
            1. SmAttribute            - contains no data
            2. SmLongAttribute        - contains one long value
            3. SmPointerAttribute     - contains one pointer value. (Persistence not supported).
            5. SmVector3dAttribute    - contains one Vector3d value
            6. SmGenericAttribute     - contains 3 list objects:[longs, doubles, chars]. Good for mixed storage needs.
            4. SmPointerListAttribute - contains one list of SmObject pointers stored in a SmTArray<SmObject *> object. (Persistence not supported).
            Hint: to use these general attribute classes for specific purposes
                  the application only needs to define a unique attributeId 
                  value.  For example, SMLib implements a color attribute
                  using the SmVector3dAttribute attribute class with the 
                  SM_AI_COLOR attributeId value.
            Hint: attributes with no data can be used as geometry labels.
                  For example, To divide all the geometry in a model
                  into a group to be rendered and another group not to
                  be rendered, create an AttributeId value called 
                  SM_AI_RENDER.  At run-time add an SmAttribute with its
                  AttributeId = SM_AI_RENDER to every piece of geometry
                  to be rendered.  In the render function, render only
                  those geometry objects labeled for rendering as,
                    for(ii=0;ii<GeometryObjectList.GetSize();ii++)
                      {
                        if(GeometryObjectList[ii]->FindAttribute(SM_AI_RENDER))
                          {
                            A_render_function(GeometryObjectList[ii]) ; 
                          }
                      }
                  To save on model size, make sure the SM_AI_RENDER
                  attribute has a REFERENCE behavior.

   4. Attribute Inheritance. 
      attributes are inherited from their attribute parents when
      searching for an attribute with the FindInheritedAttribute() 
      function in the following order:

          brep<-+-Region<---Shell
                |
                +-Face  <-+-Surface
                |         |
                |         +-Faceuse
                +-Loop  <---Loopuse
                |
                +-Edge  <-+-Curve
                |         |
                |         +-Edgeuse<-TrimCurve
                +-Vertex<---Vertexuse
      
      There is no attribute inheritance when searching for an attribute
      with the FindAttribute() function.
              SmAttribute *MyFindInheritedAttribute(SmTopology *Object, ULONG lAttributeID)
                {
                  SmAttribute *pAttr = Object->FindAttribute(lAttributeID) ;
                  if(pAttr) return(pAttr) ;
                  SmTopology *pOwner = object->GetOwner()
                  if(pOwner) return(MyFindInheritedAttribute(pOwner, lAttributeID) ;
                  else       return(NULL) ;
                }

   5. Attribute propagation.  
      Attributes are connected to topology
      objects that are part of a geometry model's topology graph.
      Edit-functions, such as booleans, blends, offsets, and the like,
      modify the topology graph by modifying individual topology 
      objects.  Attribute propagation is an automated mechanism that
      decides how to propagate attributes from the initial topology
      graph to the final topology graph when the user-object to which
      the attribute is attached is modified.

      Attribute propagation is managed by a callback mechanism
      implemented within the SmAObject::Notify() method and
      the callback methods of the SmAttribute class.

       5a. Edit-functions call the Notify() method on each 
          object that they modify along with an input
          SmNotifyOperation enum input value describing the
          change being made to the object and an optional
          set of object pointers depending on type of change 
          being made.  The current list of SmNotifyOperation values
          includes:
            1. SM_NO_ADD_TO_BREP           Topology obj added to Brep
            2. SM_NO_SPLIT_IN_BREP         Topology obj split into 2 children in a Brep
            3. SM_NO_MERGE_IN_BREP         Two Topology objs merged to 1 in a Brep
            4. SM_NO_TRIM_NO_SPLIT_IN_BREP A bndry edge or vertex is added to face or edge without splitting the parent obj
            5. SM_NO_COINCIDENT            Two Topology objs in different Breps are found coincident during Boolean operation
            6. SM_NO_RM_FROM_BREP          Topology obj removed from a Brep
            7. SM_NO_CHANGE_GEOMETRY       Topology obj's geometry is changed
            8. SM_NO_CHANGE_OWNER          Geometry obj's owner obj is changed
            9. SM_NO_CONSTRUCTION          Called by constructors
           10. SM_NO_COPY                  Topology or Geometry obj is copied (attribs not copied if target already has attrib of same type)
           11. SM_NO_PRE_EDIT              Topology or Geometry obj about to be changed - typically used to remove object caches
           12. SM_NO_POST_EDIT             Topology or Geometry obj done being changed
           13. SM_NO_SPLIT                 Geometry obj split into 2 children
           14. SM_NO_MERGE                 Two Geometry objs merged into 1
           15. SM_NO_REG_PROPAGATION       OtherBrep->SrcRegions merged into ThisBrep->SrcRegions to make one ThisBrep->Merged Region
           16. SM_NO_DESTRUCTION           Called by Destructors - typically used to remove object cashes
           17. SM_NO_UNKNOWN               Uninitialized value. typically used by switch statements to detect unsupported Notify values
           Hint: Attribute propagation succeeds only if the 
                 edit-functions call Notify() at the proper times.
                 Currently, the edit-functions only make Split and 
                 Merge Notification calls on Regions, Faces, Edges, 
                 and Vertices and not on ObjectUses, Curves, or 
                 Surfaces.  A missing call to Notify() is an easily 
                 fixed bug not a feature.  Contact SMS support if 
                 you find such a case or need to extend attribute 
                 propagation to new Object types.

   expected calls: caller->Notify(Event, pData1, pData2, pData2)                                    
 
 |       event                | caller      | pData1   | pData2                 | pData3                  |
 +----------------------------+-------------+----------+------------------------+-------------------------+
 | SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                   | AddObj->GeomPtr or NULL |
 | SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| SplitObj | Child1                 | Child2                  |
 | SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SrvObj   | DelObj                 | Brep                    |
 | SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj          | NULL                    | 
 | SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj               | BrepB                   |
 | SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                   | RmObj->GeomPtr or NULL  |
 | SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL           | OldGeom or NULL         |
 | SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner->Brep or NULL | OldOwner or NULL        |
 | SM_NO_CONSTRUCTION         | NewObj      | NewObj   | CopyFromObj or NULL    | NULL                    |
 | SM_NO_COPY                 | FromObj     | ToObj    | ToObj->Owner or NULL   | FromObj->Owner or NULL  |
 | SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj->Owner or NULL | NULL                    |
 | SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj->Owner or NULL | NULL                    |
 | SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                 | SplitObj->Owner or NULL |
 | SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2               | MergeObj->Owner or NULL |
 | SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs     | ThisBrep->MergeReg      |
 | SM_NO_DESTRUCTION          | DelObj      | DelObj   | NULL                   |  NULL                   |

       5b. Notify() is a virtual method of the SmObject base
          class.  All derived Notify() functions, after they 
          complete their notify responses, always pass the  
          Notify() call onto their immediate base class. 
          so one edit-function's call to an object's 
          Notify() method can result in a Notify() method 
          being executed for each level in the object's 
          inheritance hierarchy. Notify() methods are used for 
          a variety of purposes including cache management, SmBrep
          object list management and attribute propagation. 
          The SmAObject::Notify() method is used to implement 
          attribute propagation.

        5c. The SmAObject::Notify() method calls an SmAttribute
          virtual response method on each attribute within
          the object's attribute list.  Which SmAttribute
          response method called is determined by the SmNotifyOperation
          enum value as:
            1. SM_NO_SPLIT_IN_BREP        ->  SmAttribute::Split(OrigObject,child1,child2)
            2. SM_NO_MERGE_IN_BREP        ->  SmAttribute::Merge(OrigObject1,OrigObject2,MergeResult)
            3. SM_NO_COINCIDENT           ->  SmAttribute::Coincident(FromObj, ToObj,BrepContainingFromObj, BrepContainingToObj)
            4. SM_NO_CONSTRUCTION,        ->  SmAttribute::Copy(NotUsedObjBeingCopiedFrom, ObjBeingCopiedTo)
            5. SM_NO_COPY                 ->  SmAttribute::Copy(OrigObject,NewCopyObject)
            6. SM_NO_PRE_EDIT             ->  SmAttribute::Edit(OrigObject)
            7. SM_NO_SPLIT                ->  SmAttribute::Split(ObjBeingSplit, Child1, Child2)
            8. SM_NO_MERGE                ->  SmAttribute::Merge(OrigObject1,OrigObject2,MergeResult)
            9. SM_NO_REG_PROPAGATION      ->  if(pAttribute->IsPropagatedThroughBooleans()) SmAttribute::PropagateRegionsThroughBooleans()
           10. SM_NO_DESTRUCTION          ->  SmAttribute::Destruction(OrigObject)
           11. SM_NO_ADD_TO_BREP          ->  No method call
           12. SM_NO_TRIM_NO_SPLIT_IN_BREP->  No method call
           13. SM_NO_RM_FROM_BREP         ->  No method call
           14. SM_NO_CHANGE_GEOMETRY      ->  No method call
           15. SM_NO_CHANGE_OWNER         ->  No method call
           16. SM_NO_POST_EDIT            ->  No method call
           17. SM_NO_UNKNOWN              ->  SE error message
           18. default                    ->  SE error message

        5d. The actions taken by the SmAObject response methods are
          controlled by the attribute's SmAttributeBehaviorType value 
          to preserve the attribute's 'COPY/REFERENCE' behavior
          and to implement a 'STANDALONE/DEPENDENT' behavior.  
          A DEPENDENT attribute is deleted whenever the number of 
          objects in its user list goes to zero. STANDALONE attributes  
          are not deleted when the length of their user list 
          goes to zero.  The combination of these two behaviors
          creates four different behavior states for an attribute
          represented in its SmAttributeBehaviorType value as:
            1. SM_AB_COPY,                // attribute connects to one object 
                                          //   and is deleted when its user-list length is 0.
            2. SM_AB_REFERENCE,           // attribute connects to many objects
                                          //   and is deleted when its user-list length is 0.
            3. SM_AB_STANDALONE_COPY,     // attribute connects to one object 
                                          //   and is not deleted when its user-list length is 0.
            4. SM_AB_STANDALONE_REFERENCE // attribute connects to many objects
                                          //   and is not deleted when its user-list length is 0.
            
          The default behaviors for the SmAObject::Callback functions are:
            1. SmAttribute::Copy(OrigObject,NewCopyObject)
                 Place a copy of this attribute on NewCopyObject.
            2. SmAttribute::Split(OrigObject,child1,child2) 
                 Place a copy of this attribute on child1 and child2.
            3. SmAttribute::Merge(OrigObject1,OrigObject2,MergeResult)
                 Place a copy of this attribute on MergeResult.
            4. SmAttribute::Destruction(OrigObject) - No Action.                           
            5. SmAttribute::Edit(OrigObject)        - No Action.                           
            Hint: All SmAObject::Callback functions are virtual.  
                  Derive from SmAttribute and define new Callback
                  functions to create attributes with new and more
                  interesting propagation behaviors.

   6. Attribute persistence.
      Geometry models along with their attributes are written
      to and read from files with the following SmBrepData 
      object methods:
        SmBrepData::WritePartToFile(),     (ASCII)
        SmBrepData::ReadPartFromFile(),    (ASCII)
        SmBrepData::WritePartToDB(), and   (binary)
        SmBrepData::ReadPartFromDB().      (binary)

      The persistence mechanism writes and reads attributes with
      any number of long, double, char values along with the
      attribute's AttributeId and Behavior values.  By default
      all attributes, except for those with AttributeIds of 
      SM_AI_COLOR and SM_AI_TAG, are read in with a 
      SmGenericAttribute derived type.  Attributes
      with an attributeID value of SM_AI_COLOR are read in
      as SmVector3dAttribute objects and SM_AI_TAG attributes are read
      in as SmTagAttribute objects.
      
      A callback mechanism, not used by SMLib but available for
      application use, is available for reading in attributes
      of different derived types.  To read in an attribute and
      assign it a derived type other than SmGenericAttribute,
      first write a callback function of the form,

      SmAttribute *MyCreateDerivedTypeAttribute
        (const SmContext & crContext,        // in : context for constructor
         ULONG lAttributeID,                 // in : identifier value
         SmAttributeBehaviorType eBehavior,  // in : behavior value
         const SmTArray<long> & rLongs,      // in : array of longs to store
         const SmTArray<double> & rDoubles,  // in : array of doubles to store
         const SmTArray<char> & rChars)      // in : array of characters to store
        {
          if(lAttributeID != SM_AI_MYDERIVEDTYPE) return(NULL) ;
          return(new (crContext) SmMyDerivedAttribute(. . .) ;
        }

      Then create an SmTArray<void*> array containing pointers to
      all such specialized attribute creating functions you may
      have written.  Register this array with the SmContext using
      the functions

      SmContext::SetAttributeCallbacks() and
      SmContext::GetAttributeCallbacks().

      When attributes are read from file or DB, all the functions 
      registered with the SmContext object will be given a chance 
      to create an attribute of appropriate derived type.  If all 
      of those functions return NULL, then that attribute will be
      read in with a derived type of SmGenericAttribute.
        Hint: If your derive an attribute type that overloads
              any of the SmAttribute virtual functions, you
              will need to use the persistence callback mechanism
              for reading those attributes from file or DB. 

***********************************************************************/
class SM_EXPORT SmAttribute : public SmObject
{
  friend class SmAObject;
protected:
  ULONG                   m_lAttributeID ;  // unique id for each attribute type - not each attribute.
  SmAttributeBehaviorType m_eBehavior ;     // oneof SM_AB_COPY                  - one geom obj per attrib obj   - auto deleted
                                            //       SM_AB_REFERENCE             - many geom objs per attrib obj - auto deleted
                                            //       SM_AB_STANDALONE_COPY       - same as COPY      - not deleted if user_count goes to zero
                                            //       SM_AB_STANDALONE_REFERENCE  - same as REFERENCE - not deleted if user_count goes to zero
                                            //       SM_AB_TEMP                  - Not Copied, Not Persistent
  ULONG                   m_lMark ;         // Used to mark attributes during various traversal operations.
                                            //   [not persistent - not written to and read from file]
  SmTArray<SmAObject*>    m_vUsers ;        // list of all geometry objects connected to this attribute
  // mutable std::mutex   mUsersMutex;       // Mutex for tessellation parallelization // JLMCC removed to address compiler warnings. Revisit.

  public:
  // SmAttribute class constructors and destructor
  // constructor
  SmAttribute() : m_lAttributeID(SM_UNDEF_ULONG), 
                  m_eBehavior(SM_AB_COPY),
                  m_lMark(SM_UNDEF_ULONG)
                { }

  // constructor
  SmAttribute(ULONG lAttributeID,                             // oneof of the SM_AI_ macro type names. A unique id for each attribute type - not each attribute. 
              SmAttributeBehaviorType eBehavior = SM_AB_COPY) // oneof SM_AB_COPY                  - one geom obj per attrib obj   - auto deleted 
                                                              //       SM_AB_REFERENCE             - many geom objs per attrib obj - auto deleted 
                                                              //       SM_AB_STANDALONE_COPY       - same as COPY      - not deleted if user_count goes to zero 
                                                              //       SM_AB_STANDALONE_REFERENCE  - same as REFERENCE - not deleted if user_count goes to zero 
                                                              //       SM_AB_TEMP                  - Not Copied, Not Persistent 
             : m_lAttributeID(lAttributeID),
               m_eBehavior(eBehavior), 
               m_lMark(SM_UNDEF_ULONG)  
             { m_vUsers.SetContext(GetContext()); }

  // copy constructor
  SmAttribute(const SmAttribute & crOriginal)               
             : m_lAttributeID(crOriginal.m_lAttributeID),
               m_eBehavior(crOriginal.m_eBehavior),
               m_lMark(SM_UNDEF_ULONG)
             { m_vUsers.SetContext(GetContext()); }

  // destructor
  virtual ~SmAttribute()
    {
      m_lAttributeID = SM_UNDEF_ULONG;
      m_lMark = SM_UNDEF_ULONG;

      // Note, RemoveAttribute() reduces the size of m_vUsers, so don't just iterate on GetSize().
      // But put in a check, in case it doesn't.
      while(m_vUsers.GetSize() > 0)
        {
          ULONG lSizeCheck = m_vUsers.GetSize();
          if(m_vUsers[0] != NULL)
            {
              m_vUsers[0]->RemoveAttribute(this);
            }
          // Check:
          if ( m_vUsers.GetSize() >= lSizeCheck )
            { break; }  // Possible memory leak, better than infinite loop.
        } // end while users
    } // end SmAttribute destructor

  // virtual Copy function - required implementation for derived objects
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const 
                                { return( new (crContext) SmAttribute(m_lAttributeID,m_eBehavior)) ; }

  // for persistence: create SmAttribute derived objects from known AttributeIDs 
  static SmAttribute * CreateAttribute
  (
    const SmContext        & crContext,       // in :   
    ULONG                    m_lAttributeID,  // in :   
    SmAttributeBehaviorType  eBehavior,       // in :   
    const SmTArray<long>   & rLongs,          // in :   
    const SmTArray<double> & rDoubles,        // in :   
    const SmTArray<char>   & rChars           // in :   
  ) ;

  // query functions
  ULONG                          GetAttributeID()                        const { return m_lAttributeID; }
  SmAttributeBehaviorType        GetBehavior()                           const { return m_eBehavior; }
  virtual ULONG                  GetNumLongElements()                    const { return 0; }
  virtual ULONG                  GetNumDoubleElements()                  const { return 0; }
  virtual ULONG                  GetNumCharacterElements()               const { return 0; }
  virtual const long           * GetLongElementsAddress()                const { return NULL; }
  virtual const double         * GetDoubleElementsAddress()              const { return NULL; }
  virtual const char           * GetCharacterElementsAddress()           const { return NULL; } 
  virtual void                   GetUsers(SmTArray<SmAObject*> & rUsers) const ; // eff: copies m_vUsers array into rUsers
  virtual SmTArray<SmAObject*> * GetUsersRef()                                 { return &m_vUsers ; }

  // propagation handlers - each branches on m_eBehavior value

  // called : when one topology object is split into two topology objects within one topology graph.
  // default: if children do not have same attribute type, add deep copy of or reference to this attrib to children
  // assumes: this attribute belongs to pSplitObject 
  virtual void Split 
  (
    SmAObject * pSplitObject,    // in : Object being split (the object who owns this attribute)    
    SmAObject * pChild1,         // in : child1, gets copy of this attribute when needed            
    SmAObject * pChild2          // in : child2, gets copy of this attribute when needed            
  ) ;       
                                    
  // called : when a new topology object is created as a copy of some other topology object 
  // default: if pToObj does not have same attribute type, place deep copy of or reference to this attrib on new one 
  // assumes: this attribute belongs to pFromObj
  virtual void Copy
  (
    const SmAObject * pFromObj,    // in : not used by default behavior - object being copied from    
    SmAObject       * pToObj       // in : object being copied into                                   
  ) ;    

  // called : when two topology objects in one topology graph are being merged into a single topology object
  // default: if pMergedResult does not have same attribute type, place deep copy of or reference to this attrib on new one 
  // assumes: this attribute belongs to either pOrigObject1 or pOrigObject2, i.e this = pOrigObject1->Attribute[i] or pOrigObject2->Attribute[i] 
  virtual void Merge
  (
    SmAObject * pOrigObject1,    // in : not used by default behavior - object1 being merged         
    SmAObject * pOrigObject2,    // in : not used by default behavior - object2 being merged         
    SmAObject * pMergedResult    // in : merged object - gets copy of this attribute when needed     
  ) ; 

  // called: By SmMerge::PropagateOtherSrcRegionAttribs() when an Attribute being propagated from a Boolean OtherBrep needs to be merged with a ThisBrep attribute 
  // default: if pMergedResult does not have same attribute type, place deep copy of or reference to this attrib on new one
  // assumes: this attribute belongs to either pOrigObject1 or pOrigObject2, i.e this = pOrigObject1->Attribute[i] or pOrigObject2->Attribute[i] 
  // rtn: Only attributes on Regions returning true to this call 
  //      are propagated through booleans from OtherBrep to ThisBrep
  virtual SmBoolean IsPropagatedThroughBooleans() 
  { return FALSE ; } 
  
  virtual void PropagateRegionsThroughBooleans
  (
    SmAObject * pOrigObject1,    // in : not used by default behavior - object1 being merged       
    SmAObject * pOrigObject2,    // in : not used by default behavior - object2 being merged       
    SmAObject * pMergedResult    // in : merged object - gets copy of this attribute when needed   
  )  
  { SmAttribute::Merge(pOrigObject1, pOrigObject2, pMergedResult) ; }

  // called when two topology objects from two different Breps are found coincident during a boolean operation
  // let this = pFromObj->Attribute, When pToObj does not have same attribute type, place deep copy of or reference to this attrib on pToObj
  // assumes: this attribute belongs to the pThisObject
  virtual void Coincident
  (
    SmAObject * pFromObj,    // in : FromObj of the FromObj-ToObj Coincident Topology pair   
    SmAObject * pToObj,      // NotUsed: in : ToObj   of the FromObj-ToObj Coincident Topology pair   
    SmAObject * pFromBrep,   // NotUsed: in : Brep containing pFromObj                                
    SmAObject * pToBrep      // in : Brep containing pToObj                                  
  ) ;   

  // base class behavior: Do Nothing                         
  virtual void Edit(SmAObject * pEditedObject);

  // base class behavior: Do Nothing
  virtual void Destruction(SmAObject * pToBeDeleted);

  // user list management

  // remove pUser and destroy if unused
  virtual void RemoveUser(SmAObject * pUser, SmBoolean bDoNotDelete = FALSE);

  // FS fast attribute destructor
  virtual void fromClearAttributes(SmAObject* pThis);

  // add pUser to user's list
  virtual void AddUser(SmAObject * pUser);

  // increment the mark/mark2 on the context by 1
  void      Mark(SmMarkType eMarkType)                    { m_lMark = GetContext()->GetCurrentMark(eMarkType); }
  void      UnMark(SmMarkType eMarkType)                  { if(m_lMark > 0 && m_lMark == GetContext()->GetCurrentMark(eMarkType)) m_lMark--; }
  SmBoolean IsMarked(SmMarkType eMarkType) const          { return (m_lMark == GetContext()->GetCurrentMark(eMarkType)) ; }
                                
  // get memory used and allocated
  virtual ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const ;

  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore                                            
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        
                                              //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    
    SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  
    SmTArray<ULONG>  * pTestRequests=NULL     // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  
  ) const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmAttribute,SmObject,SmAttribute_TYPE);

} ; // end class SmAttribute

/*******************************************************************//**
PURPOSE: This object is a generic attribute.  It allows reading and
    manipulation of user defined attribute types in executables where
    the user attribute is not defined.  In this case the default
    behavior will occur when editing operations happen.

NOTES: The virtual methods, Split(),       Copy(),    Merge(), 
                            Coincident(),  Edit(),    Destruction(), 
                            RemoveUser(),  AddUser(), 
                            AssertValid()
       are not implemented because their default base class behaviors
       are fine for this attribute and likely fine for all other derived attributes.

       The virtual methods, GetNumLongElements(),      GetLongElementsAddress(),
                            GetNumDoubleElements()     GetDoubleElementsAddress(),
                            GetNumCharacterElements(), GetCharacterElementsAddress(),
       are implemented to support attribute persistence.
***********************************************************************/
class SM_EXPORT SmGenericAttribute : public SmAttribute
{
protected:
  SmTArray<long>   m_vLongElements;
  SmTArray<double> m_vDoubleElements;
  SmTArray<char>   m_vCharacterElements;
    
public:
  SmGenericAttribute
  (
    ULONG lAttributeID,
    SmAttributeBehaviorType  eBehavior,
    const SmTArray<long>   & rLongElements,
    const SmTArray<double> & rDoubleElements,
    const SmTArray<char>   & rCharacterElements
  );

  SmGenericAttribute(const SmGenericAttribute & crOriginal);

  // virtual destructor - required implementation for derived objects
  virtual ~SmGenericAttribute() { }
    
  // virtual Copy function - required implementation for derived objects
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const 
  { 
    return( new (crContext) SmGenericAttribute(m_lAttributeID, m_eBehavior, m_vLongElements,
                                               m_vDoubleElements, m_vCharacterElements)) ;
  }

  // simple data access
  SmTArray<long>   & GetLongTArray()   { return m_vLongElements ; }
  SmTArray<double> & GetDoubleTArray() { return m_vDoubleElements ; }
  SmTArray<char>   & GetCharTArray()   { return m_vCharacterElements ; }

  // virtual GetMethods to support persistance
  virtual ULONG          GetNumLongElements()          const { return m_vLongElements.GetSize(); }
  virtual ULONG          GetNumDoubleElements()        const { return m_vDoubleElements.GetSize(); }
  virtual ULONG          GetNumCharacterElements()     const { return m_vCharacterElements.GetSize(); }
  virtual const long   * GetLongElementsAddress()      const { return m_vLongElements.GetDataArray(); }
  virtual const double * GetDoubleElementsAddress()    const { return m_vDoubleElements.GetDataArray(); }
  virtual const char   * GetCharacterElementsAddress() const { return m_vCharacterElements.GetDataArray(); }

  SmStatus SetLongElement(ULONG iNum, long lNewValue)      
  { 
    if (iNum >= m_vLongElements.GetSize()) return SM_ERR;
    m_vLongElements[iNum] = lNewValue;
    return SM_SUCCESS;
  }

  SmStatus SetDoubleElement(ULONG iNum, double lNewValue)  
  { 
    if (iNum >= m_vDoubleElements.GetSize()) return SM_ERR;
    m_vDoubleElements[iNum] = lNewValue;
    return SM_SUCCESS;
  }

  SmStatus SetCharacterElement(ULONG iNum, char lNewValue) 
  { 
    if (iNum >= m_vCharacterElements.GetSize()) return SM_ERR;
    m_vCharacterElements[iNum] = lNewValue;
    return SM_SUCCESS;
  }
       
  // get memory used and allocated
  virtual ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const ; 

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmGenericAttribute,SmAttribute,SmGenericAttribute_TYPE);

} ; // end class SmGenericAttribute

/*******************************************************************//**
PURPOSE: This object is an attribute which will contain a long value.

NOTES: 
***********************************************************************/
class SM_EXPORT SmLongAttribute : public SmAttribute
{
protected:
  long m_lValue;
public:
  // constructor 
  SmLongAttribute(ULONG lAttributeID, long lValue, SmAttributeBehaviorType eBehavior=SM_AB_COPY) 
    : SmAttribute(lAttributeID,eBehavior),
      m_lValue(lValue) 
  { }

  // copy constructor
  SmLongAttribute(const SmLongAttribute & crOriginal)           
    : SmAttribute(crOriginal), 
      m_lValue(crOriginal.m_lValue)  
  { }

  // virtual destructor - required implementation for derived objects
  virtual ~SmLongAttribute() { }

  // virtual Copy function - required implementation for derived objects
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const  
  { return(new (crContext) SmLongAttribute(m_lAttributeID,m_lValue)) ; }

  // simple data access
  virtual ULONG          GetNumLongElements()          const    { return 1; }
  virtual ULONG          GetNumDoubleElements()        const    { return 0; }
  virtual ULONG          GetNumCharacterElements()     const    { return 0; }
  virtual const long   * GetLongElementsAddress()      const    { return &m_lValue; }
  virtual const double * GetDoubleElementsAddress()    const    { return NULL; }
  virtual const char   * GetCharacterElementsAddress() const    { return NULL; }
  long                   GetValue()                    const    { return m_lValue; }

  void SetValue(long lNewValue)                                 { m_lValue = lNewValue; }

  // get memory used and allocated
  virtual ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const   
  { 
    ULONG lAllocated, lUsed = m_vUsers.GetMemoryUsed(lAllocated) ;
    rlMemoryAllocated = sizeof(this) + lAllocated - sizeof(SmTArray<SmAObject*>) ;
    return(             sizeof(this) + lUsed      - sizeof(SmTArray<SmAObject*>)) ;
  }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmLongAttribute, SmAttribute, SmLongAttribute_TYPE);

} ; // end class SmLongAttribute

/*******************************************************************//**
PURPOSE: This object is an attribute which will contain a pointer value.

NOTES: This is a temporary attribute built, used, and freed
 all in one working session.  This attribute does not support persistence
 since the memory address of the objects identified by these stored 
 address pointers will change when a database is written and read back
 into working memory.  Any SmPointerAttribute read from file will have
 a stale pointer value that should not be used.
***********************************************************************/
class SM_EXPORT SmPointerAttribute : public SmAttribute
{
protected:
  void * m_pPointer;

public:
  SmPointerAttribute( ULONG lAttributeID, void * pPointer, SmAttributeBehaviorType eBehavior=SM_AB_COPY)              
    : SmAttribute(lAttributeID,eBehavior), 
      m_pPointer(pPointer) 
  { }

  SmPointerAttribute( const SmPointerAttribute & crOriginal) 
    : SmAttribute(crOriginal), 
      m_pPointer(crOriginal.m_pPointer) 
  { }

  // virtual destructor - required implementation for derived objects
  virtual ~SmPointerAttribute() { }

  // virtual Copy function - required implementation for derived objects
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const 
  { return( new (crContext) SmPointerAttribute(*this)) ; }

  void SetValue(void * pPointer) { m_pPointer = pPointer; }

  void * GetValue() const { return m_pPointer; }

  // get memory used and allocated
  virtual ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const 
  { 
    ULONG lAllocated, lUsed = m_vUsers.GetMemoryUsed(lAllocated) ;
    rlMemoryAllocated = sizeof(this) + lAllocated - sizeof(SmTArray<SmAObject*>) ;
    return(             sizeof(this) + lUsed      - sizeof(SmTArray<SmAObject*>)) ;
  }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPointerAttribute,SmAttribute,SmPointerAttribute_TYPE);

} ; // end class SmPointerAttribute

/*******************************************************************//**
PURPOSE: This is an attribute object containing a list of pointer values.

NOTES: This is a temporary attribute built, used, and freed
 all in one working session.  This attribute does not support persistence.  
 Any SmPointerAttribute read from file will have stale pointer values that 
 should not be used. 
***********************************************************************/
class SM_EXPORT SmPointerListAttribute : public SmAttribute
{
protected:
  SmTArray<SmObject *> m_vList ;

public:
  SmPointerListAttribute( ULONG lAttributeID, const SmTArray<SmObject*> * pOptListToCopy=NULL, SmAttributeBehaviorType eBehavior = SM_AB_COPY)
    : SmAttribute(lAttributeID,eBehavior)  
  { if(pOptListToCopy) { m_vList.Append(*pOptListToCopy) ; } }

  SmPointerListAttribute(const SmPointerListAttribute & crOriginal) 
    : SmAttribute(crOriginal)
  { m_vList.Append(crOriginal.m_vList) ; }

  // virtual destructor - required implementation for derived objects
  virtual ~SmPointerListAttribute() { m_vList.ReSet() ; }

  // virtual Copy function - required implementation for derived objects
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const 
  { return( new (crContext) SmPointerListAttribute(*this)) ; }

  const SmTArray<SmObject *> & GetValue() const                             { return m_vList ; }
  void                         SetValue(SmTArray<SmObject *> & rListToCopy) { m_vList = rListToCopy ; }

  // get memory used and allocated
  virtual ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const 
     { ULONG lUsersAllocated, lUsersUsed = m_vUsers.GetMemoryUsed(lUsersAllocated) - sizeof(SmTArray<SmAObject*>) ;
       ULONG lListAllocated,  lListUsed  = m_vList.GetMemoryUsed(lListAllocated)   - sizeof(SmTArray<SmAObject*>) ;
       rlMemoryAllocated = sizeof(this) + lUsersAllocated + lListAllocated ;
       return(             sizeof(this) + lUsersUsed      + lListUsed     ) ;
     }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmPointerListAttribute,SmAttribute,SmPointerListAttribute_TYPE);

} ; // end class SmPointerListAttribute

/*******************************************************************//**
PURPOSE: This object is an attribute which will contain a vector value.
    The vector may be used to represent anything which can be represented
    with three doubles such as a color value (lAttributeID = SM_AI_COLOR).

NOTES: 
***********************************************************************/
class SM_EXPORT SmVector3dAttribute : public SmAttribute
{
protected:
  SmVector3d m_vValue;
public:
  SmVector3dAttribute(ULONG lAttributeID, const SmVector3d & rValue, SmAttributeBehaviorType eBehavior = SM_AB_COPY)
    : SmAttribute(lAttributeID,eBehavior), 
      m_vValue(rValue) 
  { }

  SmVector3dAttribute(const SmVector3dAttribute & crOriginal) 
    : SmAttribute(crOriginal), 
      m_vValue(crOriginal.m_vValue) 
  { }

  // virtual destructor - required implementation for derived objects
  virtual ~SmVector3dAttribute() { }

  // virtual Copy function - required implementation for derived objects
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const 
  { return(new (crContext) SmVector3dAttribute(m_lAttributeID, m_vValue)) ; }

  virtual ULONG          GetNumLongElements()          const { return 0; }
  virtual ULONG          GetNumDoubleElements()        const { return 3; }
  virtual ULONG          GetNumCharacterElements()     const { return 0; }
  virtual const long   * GetLongElementsAddress()      const { return NULL; }
  virtual const double * GetDoubleElementsAddress()    const { return (double*)&m_vValue.x; }
  virtual const char   * GetCharacterElementsAddress() const { return NULL; }

  void SetValue(SmVector3d & rNewValue) { m_vValue = rNewValue; }

  const SmVector3d & GetValue() const { return m_vValue; }

  // get memory used and allocated
  virtual ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const 
  { 
    ULONG lAllocated, lUsed = m_vUsers.GetMemoryUsed(lAllocated) ;
    rlMemoryAllocated = sizeof(this) + lAllocated - sizeof(SmTArray<SmAObject*>) ;
    return(             sizeof(this) + lUsed      - sizeof(SmTArray<SmAObject*>)) ;
  }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmVector3dAttribute,SmAttribute,SmVector3dAttribute_TYPE);

} ; // end class SmVector3dAttribute

/*******************************************************************//**
PURPOSE: This object is an attribute built to support region attribute 
 propagation through the SmMerge Boolean operation.

NOTES: This is a temporary attribute built, used, and freed
 all in one working session. It contains 
 a pointer value marking a split ThisBrep's region's OtherBrep->Region.
 a Boolean tracking Which regions were split from the Infinite Region.
 
 Assigned m_lAttributeID = SM_AI_OTHER_REGION_ID 
***********************************************************************/
class SM_EXPORT SmMergeRegionAttribute : public SmAttribute
{
protected:
  SmBooleanOperationType m_eOperation ;          // assigned by the caller of pRegion->Notify(SM_NO_REG_PROPAGATION,NULL,NULL,pResultRegion)

  // SourceRegions are maintained as unique lists
  SmTArray<SmRegion *>   m_vThisSourceRegions ;  // input region source (these are used as Ids - they may become stale pointers)
                                                 // This is a list of original input ThisBrep regions that share points with the
                                                 // OutputRegion containing this attribute.
  SmTArray<long>         m_vThisSourceType ;     // associated Region types: 0 = Internal solid, 1 = Internal void, 2 = Infinite Region
                         
  SmTArray<SmRegion *>   m_vOtherSourceRegions ; // input region source (these are used as Ids - they won't become stale pointers)
                                                 // This is a list of original input OtherBrep regions that share points with the
                                                 // OutputRegion containing this attribute.
  SmTArray<long>         m_vOtherSourceType ;    // associated 0 = Internal solid, 1 = Internal void, 2 = Infinite Region
                         
public:
  // constructor - hard codes AttributeID:[SM_AI_OTHER_REGION_ID] and Behavior:[SM_AB_COPY]
  SmMergeRegionAttribute
  ( 
    SmRegion * pThisSourceRegion,      // in : This Brep source Region (as input - before any splits)   
    ULONG      lThisSourceType,        // in : Associated This Source Region type                       
                                       //    : magic numbers: 0 = Internal Solid                        
                                       //    :                1 = Internal Void                         
                                       //    :                2 = Infinite Region                       
    SmRegion * pOtherSourceRegion,     // in : OtherBrep source Region (as input)                       
    ULONG      lOtherSourceType) ;     // in : Associated Other Source Region type                      
                                       //    : magic numbers: 0 = Internal Solid                        
                                       //    :                1 = Internal Void                         
                                       //    :                2 = Infinite Region                       
  // copy constructor
  SmMergeRegionAttribute( const SmMergeRegionAttribute & crOriginal) 
    : SmAttribute(crOriginal) 
  { 
    m_eOperation          =      crOriginal.m_eOperation ; 
    m_vThisSourceRegions. Append(crOriginal.m_vThisSourceRegions) ;
    m_vThisSourceType.    Append(crOriginal.m_vThisSourceType) ;
    m_vOtherSourceRegions.Append(crOriginal.m_vOtherSourceRegions) ;
    m_vOtherSourceType.   Append(crOriginal.m_vOtherSourceType) ;
  }

  // virtual destructor - required implementation for derived objects
  virtual ~SmMergeRegionAttribute() { }

  // virtual Copy function - required implementation for derived objects
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const
  { return( new (crContext) SmMergeRegionAttribute(*this)) ; }  

  // propagation behavior for Merge events. SmMergeRegionAttribute is a special use Merge::ManifoldBoolean() helper
  // attribute. Because of the way the Boolean splits and merges TheBrep input regions to OutputBrep regions
  // SmMergeRegionAttribute needs a nonDefault Merge operator behavior which accumulates ThisBrep and OtherBrep
  // region data in each merge. Fortunately, SmMergeRegionAttribute can happily use the default Split() behavior.     
  virtual void Merge
  (
    SmAObject * pOrigObject1,    // in : not used by default behavior - object1 being merged        
    SmAObject * pOrigObject2,    // in : not used by default behavior - object2 being merged        
    SmAObject * pMergedResult    // in : merged object - gets copy of this attribute when needed    
  );

  // simple data input
  void AddThisOriginRegion (SmRegion * pThisBrepRegion,  ULONG RegionType)     
  { if(m_vThisSourceRegions.AddUnique(pThisBrepRegion)) { m_vThisSourceType.Add(RegionType) ; } }  

  void AddOtherOriginRegion(SmRegion * pOtherBrepRegion, ULONG RegionType)     
  { if(m_vThisSourceRegions.AddUnique(pOtherBrepRegion)) { m_vThisSourceType.Add(RegionType) ; } }  


  // simnple data access
  ULONG                 GetThisRegionsSize()             const { return m_vThisSourceRegions.GetSize() ; }
  SmTArray<SmRegion*> & GetThisRegions    ()                   { return m_vThisSourceRegions ; }
  SmTArray<long>      & GetThisRegionTypes()                   { return m_vThisSourceType ; }
  SmRegion            * GetThisRegion     (ULONG lIndx)  const { SM_ASSERT_BREAK(lIndx < m_vThisSourceRegions.GetSize() ) ; return m_vThisSourceRegions. GetAt(lIndx) ; }
  ULONG                 GetThisRegionType (ULONG lIndx)  const { SM_ASSERT_BREAK(lIndx < m_vThisSourceType.GetSize() ) ;    return m_vThisSourceType. GetAt(lIndx) ; }
                                                         
  ULONG                 GetOtherRegionsSize()            const { return m_vOtherSourceRegions.GetSize() ; }
  SmTArray<SmRegion*> & GetOtherRegions    ()                  { return m_vOtherSourceRegions ; }
  SmTArray<long>      & GetOtherRegionTypes()                  { return m_vOtherSourceType ; }
  SmRegion            * GetOtherRegion     (ULONG lIndx) const { SM_ASSERT_BREAK(lIndx < m_vOtherSourceRegions.GetSize()) ; return m_vOtherSourceRegions.GetAt(lIndx) ; }
  ULONG                 GetOtherRegionType (ULONG lIndx) const { SM_ASSERT_BREAK(lIndx < m_vOtherSourceType.GetSize()) ;    return m_vOtherSourceType.GetAt(lIndx) ; }

  SmBooleanOperationType GetOperation()                                  { return m_eOperation ; }
  void                   SetOperation(SmBooleanOperationType eOperation) { m_eOperation = eOperation ; }
  
  // set SmMergeRegionAtribute stored values for a given existing index, rtns error when lIndx is not valid
  SmStatus   SetRegions(ULONG lIndx, SmRegion * pThisSourceRegion, ULONG lThisSourceType, SmRegion * pOtherSourceRegion, ULONG lOtherSourceType) ;
  
  // predicates

  // return TRUE = ThisSrcRegions include the original ThisBrep InfiniteRegion
  SmBoolean  HasThisInfiniteRegion() const
  {
    for(ULONG ii = 0; ii < m_vThisSourceType.GetSize(); ii++)
    {
      if(m_vThisSourceType.GetAt( ii ) == 2) 
        return TRUE;
    }
    return FALSE;
  }

  // return TRUE = OtherSrcRegions includes the original OtherBrep InfiniteRegion                                                      
  SmBoolean  HasOtherInfiniteRegion() const
  {
    for(ULONG ii = 0; ii < m_vOtherSourceType.GetSize(); ii++)
    {
      if(m_vOtherSourceType.GetAt( ii ) == 2) 
        return TRUE;
    }
    return FALSE;
  }
                                                   
  // return TRUE=At least one Solid Source Region in the requested set,FALSE=No Solid regions                                                                                        
  SmBoolean  HasAnySolid(ULONG lThisOtherFlag) const ; // in : 1=ThisSrcs, 2=OtherSrcs, 3=Both This & Other Sources
  
  // return TRUE=At least one Void Source Region in the requested set,FALSE=No Void regions                                                                                        
  SmBoolean  HasAnyVoid (ULONG lThisOtherFlag) const ; // in : 1=ThisSrcs, 2=OtherSrcs, 3=Both This & Other Sources

  // return TRUE=has SrcRegion->attributes which return TRUE for IsPropagatedThroughBooleans() call
  SmBoolean  HasAnyOtherPropagatedAttribs() const ; 
  
  // return first found matching attribute in the OtherSrcRegions list
  SmAttribute * HasOtherTgtAttribs
  (
    ULONG       lTgtAttribID,   // in : Tgt AttributeID
    SmRegion *& pOtherRegion    // out: pRegion owner of the returned Attribute
   ) const ; 

  // get memory used and allocated
  virtual ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const 
  { 
    ULONG lUsersAllocated, lUsersUsed = m_vUsers.GetMemoryUsed(lUsersAllocated) - sizeof(SmTArray<SmAObject*>) ;
    ULONG lThisSourceAllocated,  lThisSourceUsed  = m_vThisSourceRegions .GetMemoryUsed(lThisSourceAllocated)  - sizeof(SmTArray<SmAObject*>) ;
    ULONG lThisTypeAllocated,    lThisTypeUsed    = m_vThisSourceType    .GetMemoryUsed(lThisTypeAllocated)    - sizeof(SmTArray<SmAObject*>) ;
    ULONG lOtherSourceAllocated, lOtherSourceUsed = m_vOtherSourceRegions.GetMemoryUsed(lOtherSourceAllocated) - sizeof(SmTArray<SmAObject*>) ;
    ULONG lOtherTypeAllocated,   lOtherTypeUsed   = m_vOtherSourceType   .GetMemoryUsed(lOtherTypeAllocated)   - sizeof(SmTArray<SmAObject*>) ;
    rlMemoryAllocated = sizeof(this) + lUsersAllocated + lThisSourceAllocated + lThisTypeAllocated + lOtherSourceAllocated + lOtherTypeAllocated ;
    return(             sizeof(this) + lUsersUsed      + lThisSourceUsed      + lThisTypeUsed      + lOtherSourceUsed      + lOtherTypeUsed) ;
  }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmMergeRegionAttribute,SmAttribute,SmPointerAttribute_TYPE);

} ; // end class SmMergeRegionAttribute


/*******************************************************************//**
PURPOSE: Describes the type of the TAG ID.

NOTES: 
***********************************************************************/
                                        
#define SM_TI_UNDEFINED            0      // ID is not defined                         
#define SM_TI_GENERATOR_CURVE      1      // ID is from generator curve                       
#define SM_TI_SWEEP_STEP_DELTA 100000     // ID is some sort of copy of generator curve       
#define SM_TI_ORDINAL              2      // ID is some sort of ordinal number                
#define SM_TI_START_OF_SWEEP  -100000     // ID is from start of sweep                        
#define SM_TI_END_OF_SWEEP    -200000     // ID is from endof sweep                           
                                                                                              
                                          // ID is from a combination of two entities         
#define SM_TI_COMBO_OPERATION  100000     // in some sort of combination operation.           
#define SM_TI_INTERSECTION_EDGES   3      // ID of newly created intersection edges.          
                                                                  
                                                                  
/*******************************************************************//**
PURPOSE: This object is an attribute which will contain a tag value.
     A TAG is something that is used to uniquely identify a piece of 
     geometry or topology during a reexecution of a parametric script
                                                                  
NOTES: Note that a 0 means that the particular ID or ordinal
     is undefined.  Ordinals start at 1.                          
***********************************************************************/
class SM_EXPORT SmTagAttribute : public SmAttribute               
{                                                                 
public:                                                           
  long         m_lPrimaryID;         // ID of the object which is created during the generation
                                     // of this entity.  The primary ID of a face belonging to a
                                     // box would be the ID of the box.   
  long         m_lPrimaryOrdinal;    // Ordinal number within the PrimaryID if needed - 0 means
                                     // no ordinal needed
  long         m_lSecondaryIDType;   // Tells what operation sort of operation created the
                                     // Secondary ID.  For example: it could be the ID of 
                                     // a curve that was swept.  It could be the ID of the top
                                     // or bottom face of a sweep.
  long         m_lSecondaryID;       // ID of the secondary generating item or some sort of ordinal number
  long         m_lSecondaryOrdinal;  // If 0 no secondary ordinal needed
  long         m_lThirdIDType;       // What sort of operation contributed to the 3rd ID - for example
                                     // it may be the spine curve in a sweep.
  long         m_lThirdID;           // ID of the thrid generating item or some sort of ordinal number.
                                     // If there is a multi step sweep this is the step number starting at 0.
  long         m_lThirdOrdinal;      // Third ordinal number if non zero.
public:
  SmTagAttribute(const SmTagAttribute & crOriginal);

  SmTagAttribute(ULONG                   lAttributeID     =SM_AI_TAG,
                 long                    lPrimaryID       =0, 
                 long                    lPrimaryOrdinal  =0, 
                 long                    lSecondaryIDType =0, 
                 long                    lSecondaryID     =0, 
                 long                    lSecondaryOrdinal=0, 
                 long                    lThirdIDType     =0, 
                 long                    lThirdID         =0, 
                 long                    lThirdOrdinal    =0,
                 SmAttributeBehaviorType eBehavior        =SM_AB_COPY);

  // virtual destructor - required implementation for derived objects
  virtual ~SmTagAttribute() { } ;

  // virtual Copy function - required implementation for derived objects
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const 
  { return(  new(crContext) SmTagAttribute(*this)) ; }

  virtual ULONG GetNumLongElements()                const { return 8; }
  virtual ULONG GetNumDoubleElements()              const { return 0; }
  virtual ULONG GetNumCharacterElements()           const { return 0; }
  virtual const long * GetLongElementsAddress()     const { return &m_lPrimaryID; }
  virtual const double * GetDoubleElementsAddress() const { return NULL; }
  virtual const char *GetCharacterElementsAddress() const { return NULL; }

  SmStatus RangeCheck
  (
    const SmTagAttribute & rStart,  // in :                                                       
    const SmTagAttribute & rEnd,    // in :                                                       
    SmBoolean & rbIsInRange,        // out: True if Tag is                                        
                                    //    : within the range between rStart and rEnd.             
    ULONG & rlSelectionStatus       // out: What is status of selection                           
                                    //    : right now it is 0 always.  It will be used in future  
                                    //    : to indicate quality of tag on ambiguous selections.   
  ) const;

  SmStatus MatchCheck
  (
    const SmTagAttribute & rTag,    // in :                                                       
    SmBoolean & rbIsMatch,          // out: True if Tag is matching                               
    ULONG & rlSelectionStatus       // out: What is status of selection                           
                                    //    : right now it is 0 always.  It will be used in future  
                                    //    : to indicate quality of tag on ambiguous selections.   
  ) const;

  // get memory used and allocated
  virtual ULONG GetMemoryUsed( ULONG &rlMemoryAllocated ) const
  {
    ULONG lAllocated, lUsed = m_vUsers.GetMemoryUsed( lAllocated );
    rlMemoryAllocated = sizeof( this ) + lAllocated - sizeof( SmTArray<SmAObject*> );
    return(sizeof( this ) + lUsed - sizeof( SmTArray<SmAObject*> ));
  }

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmTagAttribute,SmAttribute,SmTagAttribute_TYPE);

} ; // end class SmTagAttribute

/*******************************************************************//**
PURPOSE: An example attribute implementation to show how default attribute
     propagation behaviors of Split/Copy/Merge/Coincident/Edit/Destruction
     can be overwritten.
    
NOTES: 
 1. This example only overwrites the Merge() propagation behavior,
    but can be used to show how any of the propagation behaviors can
    be overwritten.
    
 2. This Attirbute contains a 3dVector vector value used to hold RGB 
    color values derived from SmVector3dAttribute so that it can 
    overload the SmAttribute::Merge() default behavior to do something 
    more appropriate for color propagation.

 3. Uses lAttributeID = SM_AI_COLOR

 4. To place an attribute on any object derived from SmAObject
      SmVector3d sColor(0,0,1) ;
      SmColorAttribute *pColorAttribute = new (crContext) SmColorAttribute(SM_AI_COLOR, sColor, SmAttributeBehaviorType) ;
      pTgtObj->AddAttribute(pColorAttribute) ;

 5. To fetch an attribute from any object derived from SmAObject
      SmColorAttribute *pFoundColorAttribute = pTgtObj->FindAttribute(SM_AI_COLOR) ;
***********************************************************************/
class SM_EXPORT SmColorAttribute : public SmVector3dAttribute
{
  // from SmAttribute

    // ULONG                   m_lAttributeID; // unique id for each attribute type - not each attribute.
    // SmAttributeBehaviorType m_eBehavior;    // oneof SM_AB_COPY                  - one geom obj per attrib obj   - auto deleted
    //                                         //       SM_AB_REFERENCE             - many geom objs per attrib obj - auto deleted
    //                                         //       SM_AB_STANDALONE_COPY       - same as COPY      - not deleted if user_count goes to zero
    //                                         //       SM_AB_STANDALONE_REFERENCE  - same as REFERENCE - not deleted if user_count goes to zero
    //                                         //       SM_AB_TEMP                  - Not Copied, Not Persistent
    // ULONG                   m_lMark;        // Used to mark attributes during various traversal operations.
    //                                         //   [not persistent - not written to and read from file]
    // SmTArray<SmAObject*>    m_vUsers;       // list of all geometry objects connected to this attribute

  // from SmVector3dAttribute

    // SmVector3d              m_vValue;       // used here as [Red Green Blue] values ranging from 0.0 to 1.0
    //
    // void                    SetValue(SmVector3d & rNewValue) { m_vValue = rNewValue; }
    // const SmVector3d &      GetValue() const                 { return m_vValue; }

public:
  SmColorAttribute(ULONG                   lAttributeID, 
                   const SmVector3d      & rValue,
                   SmAttributeBehaviorType eBehavior = SM_AB_COPY)
    : SmVector3dAttribute(lAttributeID, rValue, eBehavior) 
    { }

  SmColorAttribute(const SmVector3dAttribute & crOriginal) 
    : SmVector3dAttribute(crOriginal) 
    { }

  // virtual destructor - required implementation for derived objects
  virtual ~SmColorAttribute() { }

  // virtual Copy function - required implementation for derived objects
  virtual SmAttribute * MakeCopy(const SmContext & crContext) const
  { return( new (crContext) SmColorAttribute(m_lAttributeID, m_vValue)) ; }

  // rtn: Only attributes on Regions returning true to this call 
  //      are propagated through booleans from OtherBrep to ThisBrep
  virtual SmBoolean IsPropagatedThroughBooleans() { return TRUE ; }  

  // local implementation for virtual PropagateRegionsThroughBooleans - averages two color values when two are present - else propagates color when one is present
  // called : when one resultBrep->Region is built from ThisBrep and OtherBrep regions where the attribute value has to be merged.
  virtual void PropagateRegionsThroughBooleans
  (
    SmAObject * pOrigObject1,    // in : not used by default behavior - object1 being merged
    SmAObject * pOrigObject2,    // in : not used by default behavior - object2 being merged
    SmAObject * pMergedResult    // in : merged object - gets copy of this attribute when needed
  ) ; 

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmColorAttribute, SmVector3dAttribute, SmVector3dAttribute_TYPE);

} ; // end class SmVector3dAttribute


#endif // !__SMATTRIBUTE_H__


