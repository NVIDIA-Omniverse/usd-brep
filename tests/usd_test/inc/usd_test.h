// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

 /******************************************************************/ /**
* FILE NAME --- usd_test.h
* PURPOSE: Header file for testing of SMLib / USD interface
**********************************************************************/

#ifndef _USD_TEST_H_
#define _USD_TEST_H_

#include "SmTypes.h"
#include "SmTArray.h"

#ifdef _WIN32
#define UT_EXPORT __declspec( dllexport )
#else
#define UT_EXPORT
#endif

// character buffer size
#define USD_TBLOCK_SIZE 1024

struct SmUsdConversionOptions;

// Main entry point to run test suite
UT_EXPORT double run_usd_tests(SmBoolean* bAllTestsOk = NULL);

// brep USD<->UsdBrepArrayData<->SMLib translation tests - called by run_usd_tests()
UT_EXPORT SmStatus my_test_usd();

// SMLib Brep<->USD translation
UT_EXPORT SmStatus ImportFromUsd(const std::string usd_Filename, const SmContext& crContext, SmTArray<SmBrep*>& crBreps);
UT_EXPORT SmStatus ExportToUsd(const std::string& sOutputFilename, const SmTArray<SmBrep*>& crBreps, const SmTArray<SmPolyBrep*> & crPolyBreps);

// SMLib Assembly<->USD translation - called from my_test_assembly_roundtrip()
UT_EXPORT SmStatus ImportFromUsd(const std::string& usd_Filename, const SmContext& crContext, SmTArray<SmAssembly*>& rAssemblies, SmTArray<SmBrep*> & rBreps);
UT_EXPORT SmStatus ExportToUsd(const std::string& sOutputFilename, SmAssembly& rAssembly, SmBoolean* pbExportMeshVariant = NULL);

// Tessellate an SmBrep for SMU_BrepConvert::PopulateMeshAttr, copying the Brep's attributes to the PolyBrep
UT_EXPORT SmStatus TessellateBrepForMesh(SmBrep& rBrep, SmPolyBrep*& rpPolyBrep);

// USD file -> SMLib Breps
UT_EXPORT SmStatus TranslateUsdToSmlib(const std::string sFilename, const SmContext& crContext, SmTArray<SmBrep*>& rBreps);

// components of the my_test_usd() test suite - called by run_usd_tests()

SmStatus my_test_brep_IO           (ULONG lTest); // Brep     SmLib/USD roundtrips for: 0=box, 1=mouse, 2=2boxes, 3=sphere in box, 4=cone, 5=cylinder, 6=torus, 7=plane, 8=cube
SmStatus my_test_brep_array        (ULONG lTest); // Brep     SmLib/USD roundtrips for: 0=2 boxes with materials
SmStatus my_test_brep_attributes   ();            // Brep     SmLib/USD roundtrip for box with attributes
SmStatus my_test_brep_materials    (ULONG lTest); // Brep     SmLib/USD roundtrip for Brep with materials: 0=Brep, 1= brepFaces, 2=Brep and brepFaces
SmStatus my_test_polybrep_IO       ();            // PolyBrep SmLib/USD roundtrip for test case = mouse
SmStatus my_test_assembly_roundtrip(ULONG lTest); // Exercise SmAssembly read fromfile to UsdMesh set conversion
SmStatus my_test_optional_wireedge_topology_read(); // Regression: absent wire-edge topology must not fail BrepRead
SmStatus my_test_healer_no_crash   ();            // Regression: B-rep healer must not crash importing a poor-geometry BrepArray

//UT_EXPORT SmStatus usdPatchToUsdMesh();

#endif // no _USD_TEST_H_
