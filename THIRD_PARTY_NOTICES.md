# Third-Party Notices

**USD BRep** is Copyright (c) 2020-2026, NVIDIA CORPORATION & AFFILIATES. All
rights reserved. NVIDIA-authored portions of this product are licensed under the
Apache License, Version 2.0, except where otherwise noted. See the `LICENSE` file
at the root of this repository for the full product license text.

This product includes software developed by third parties and/or by NVIDIA. The required
copyright notices, attribution statements, and license texts for those components are
provided below.

The list of components below is generated from the build-time and run-time dependencies
declared in `deps/target-deps.packman.xml` and `deps/target-deps-python.packman.xml`. The release
package also ships, in `PACKAGE-LICENSES/OpenUSD/`, the license texts that the OpenUSD package
provides for the third-party components it bundles.

---

# 3rd Party Open Source Components

## Pixar Animation Studios - OpenUSD - Tomorrow Open Source Technology License 1.0

Component: `usd-${config}` (version 25.11)

Attribution Statements: USD BRep links against OpenUSD shared libraries and uses
its C++ and Python APIs for USD import/export, traversal, schema integration, and
mesh/BRep data exchange. Source code is available at https://github.com/PixarAnimationStudios/OpenUSD.
The full upstream `LICENSE.txt` (which also enumerates the OSS components bundled inside
OpenUSD itself, e.g. RapidJSON, double-conversion, OpenEXR/Half, libdeflate, LZ4, stb,
pugixml, pbrt, Draco, Roboto fonts, Spirv Reflect, khrplatform.h, Tessil robin-map,
CLI11, PEGTL, LibAvif, libaom, boost, etc.) is the authoritative source for those
embedded notices.

License Text(https://github.com/PixarAnimationStudios/OpenUSD/blob/release/LICENSE.txt)

```
Note: The Tomorrow Open Source Technology License 1.0 differs from the
original Apache License 2.0 in the following manner. Section 6 ("Trademarks")
is different.

TOMORROW OPEN SOURCE TECHNOLOGY LICENSE 1.0

   TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION

   1. Definitions. [...]
   2. Grant of Copyright License. [...]
   3. Grant of Patent License. [...]
   4. Redistribution. [...]
   5. Submission of Contributions. [...]

   6. Trademarks. This License does not grant permission to use the trade
      names, trademarks, service marks, or product names of the Licensor
      and its affiliates, except as required to comply with Section 4(c) of
      the License and to reproduce the content of the NOTICE file.

   7. Disclaimer of Warranty. [...]
   8. Limitation of Liability. [...]
   9. Accepting Warranty or Additional Liability. [...]

(See the upstream LICENSE.txt linked above for the full, unabridged text.)
```

NOTICE (https://github.com/PixarAnimationStudios/OpenUSD/blob/v25.11/NOTICE.txt)

```
Universal Scene Description
Copyright 2016 Pixar

All rights reserved.

This product includes software developed at:

Pixar (http://www.pixar.com/).
```

---

## Intel Corporation / UXL Foundation - oneTBB (Threading Building Blocks) - Apache License 2.0

Component: `oneTBB` (transitive, bundled with OpenUSD)

Attribution Statements: USD BRep links against TBB and calls its parallel
algorithms (e.g. `tbb::parallel_for`, `tbb::parallel_reduce`) directly from C++ source to
parallelize tessellation and other compute-heavy operations. TBB is
also a transitive runtime dependency of OpenUSD itself. Source code is available at
https://github.com/uxlfoundation/oneTBB.

License Text(https://github.com/uxlfoundation/oneTBB/blob/master/LICENSE.txt)

```
                                 Apache License
                           Version 2.0, January 2004
                        http://www.apache.org/licenses/

Copyright (c) 2005-2024 Intel Corporation

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

---

## Inria, Université Bordeaux, Cisco Systems and other contributors - hwloc (Portable Hardware Locality) - BSD 3-Clause License

Component: hwloc (version 2.9.3, built into oneTBB's `tbbbind_2_5.dll` on Windows)

Attribution Statements: oneTBB's `tbbbind_2_5` library, which the Windows release package
ships in `extraLibs/` and the Windows wheel ships beside its other libraries, has hwloc built
in. TBB loads it only to bind work to NUMA nodes or core types, which USD BRep does not do.
On Linux, `libtbbbind_2_5.so` links hwloc dynamically and USD BRep does not ship hwloc.
Source code is available at https://github.com/open-mpi/hwloc. The full license text is in
`PACKAGE-LICENSES/OpenUSD/hwloc-COPYING.txt`.

License Text(https://github.com/open-mpi/hwloc/blob/hwloc-2.9.3/COPYING)

```
Copyright © 2004-2006 The Trustees of Indiana University and Indiana University Research and Technology Corporation.  All rights reserved.
Copyright © 2004-2005 The University of Tennessee and The University of Tennessee Research Foundation.  All rights reserved.
Copyright © 2004-2005 High Performance Computing Center Stuttgart, University of Stuttgart.  All rights reserved.
Copyright © 2004-2005 The Regents of the University of California. All rights reserved.
Copyright © 2009      CNRS
Copyright © 2009-2016 Inria.  All rights reserved.
Copyright © 2009-2015 Université Bordeaux
Copyright © 2009-2015 Cisco Systems, Inc.  All rights reserved.
Copyright © 2009-2012 Oracle and/or its affiliates.  All rights reserved.
Copyright © 2010      IBM
Copyright © 2010      Jirka Hladky
Copyright © 2012      Aleksej Saushev, The NetBSD Foundation
Copyright © 2012      Blue Brain Project, EPFL. All rights reserved.
Copyright © 2013-2014 University of Wisconsin-La Crosse. All rights reserved.
Copyright © 2015      Research Organization for Information Science and Technology (RIST). All rights reserved.
Copyright © 2015-2016 Intel, Inc.  All rights reserved.
See COPYING in top-level directory.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

---

## Contributors to the MaterialX Project / Academy Software Foundation - MaterialX - Apache License 2.0

Component: MaterialX (version 1.39.3, bundled with OpenUSD)

Attribution Statements: OpenUSD's `usdMtlx` library (MaterialX file and shader support)
links against the MaterialX shared libraries, so the release package ships them in
`extraLibs/` with the rest of the OpenUSD runtime. USD BRep does not call MaterialX
directly. MaterialX's upstream `THIRD-PARTY.md` lists the small libraries it embeds.
Source code is available at https://github.com/AcademySoftwareFoundation/MaterialX. The
full license text is in `PACKAGE-LICENSES/OpenUSD/materialx-LICENSE.txt`.

License Text(https://github.com/AcademySoftwareFoundation/MaterialX/blob/main/LICENSE)

```
                                 Apache License
                           Version 2.0, January 2004
                        http://www.apache.org/licenses/

Copyright Contributors to the MaterialX Project

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

---

## Pixar Animation Studios - OpenSubdiv - Modified Apache 2.0 License

Component: OpenSubdiv (version 3.6.0, bundled with OpenUSD)

Attribution Statements: The Linux release package ships OpenSubdiv's `libosdCPU` and
`libosdGPU`, which come with the OpenUSD runtime, in `extraLibs/`. The Windows release package
and the wheels do not ship them. Source code is available at
https://github.com/PixarAnimationStudios/OpenSubdiv. The full license text is in
`PACKAGE-LICENSES/OpenUSD/opensubdiv-LICENSE.txt`.

NOTICE (https://github.com/PixarAnimationStudios/OpenSubdiv/blob/v3_6_0/NOTICE.txt)

```
OpenSubdiv
Copyright 2013 Pixar
All rights reserved.

This product includes software developed at:
    Pixar (http://www.pixar.com/).
    Dreamworks Animation (http://www.dreamworksanimation.com/)
    Autodesk, Inc. (http://www.autodesk.com/).
    Google, Inc. (http://www.google.com/).
    DigitalFish (http://digitalfish.com/).
```

---

## Python Software Foundation - CPython runtime library - Python Software Foundation License Version 2

Component: `python` (version 3.12.16), the runtime library only

Attribution Statements: The bundled OpenUSD libraries link against the CPython runtime
library, so the release package ships it in `extraLibs/` (`python3.dll` and
`python312.dll` on Windows, `libpython3.12.so*` and `libpython3.so` on Linux). The package
does not ship the Python interpreter or the standard library; consumers supply their own
Python 3.12. The wheel does not ship any part of CPython. CPython incorporates third-party code; its "Licenses and Acknowledgements for
Incorporated Software" (https://docs.python.org/3.12/license.html) is the authoritative
source for those notices. On Windows, `python312.dll` also contains HACL*, which that page
does not list for 3.12; its notice follows this entry. `python312.dll` also has zlib 1.3.2
built in. On Linux, `libpython3.12.so` also has zlib 1.3.2, bzip2 1.0.8 and liblzma 5.8.3
(XZ Utils) built in. Their license texts are `zlib-LICENSE.txt`, `bzip2-LICENSE.txt` and
`xz-LICENSE.txt` in `PACKAGE-LICENSES/Python/`.
Source code is available at https://github.com/python/cpython.
The full license text is in `PACKAGE-LICENSES/OpenUSD/cpython-LICENSE.txt`.

License Text (https://docs.python.org/3.12/license.html; CPython's full `LICENSE` file also
gives the license history and the earlier BeOpen, CNRI and CWI agreements)

```
PYTHON SOFTWARE FOUNDATION LICENSE VERSION 2
--------------------------------------------

1. This LICENSE AGREEMENT is between the Python Software Foundation
("PSF"), and the Individual or Organization ("Licensee") accessing and
otherwise using this software ("Python") in source or binary form and
its associated documentation.

2. Subject to the terms and conditions of this License Agreement, PSF hereby
grants Licensee a nonexclusive, royalty-free, world-wide license to reproduce,
analyze, test, perform and/or display publicly, prepare derivative works,
distribute, and otherwise use Python alone or in any derivative version,
provided, however, that PSF's License Agreement and PSF's notice of copyright,
i.e., "Copyright (c) 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009, 2010,
2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019, 2020, 2021, 2022, 2023 Python Software Foundation;
All Rights Reserved" are retained in Python alone or in any derivative version
prepared by Licensee.

3. In the event Licensee prepares a derivative work that is based on
or incorporates Python or any part thereof, and wants to make
the derivative work available to others as provided herein, then
Licensee hereby agrees to include in any such work a brief summary of
the changes made to Python.

4. PSF is making Python available to Licensee on an "AS IS"
basis.  PSF MAKES NO REPRESENTATIONS OR WARRANTIES, EXPRESS OR
IMPLIED.  BY WAY OF EXAMPLE, BUT NOT LIMITATION, PSF MAKES NO AND
DISCLAIMS ANY REPRESENTATION OR WARRANTY OF MERCHANTABILITY OR FITNESS
FOR ANY PARTICULAR PURPOSE OR THAT THE USE OF PYTHON WILL NOT
INFRINGE ANY THIRD PARTY RIGHTS.

5. PSF SHALL NOT BE LIABLE TO LICENSEE OR ANY OTHER USERS OF PYTHON
FOR ANY INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES OR LOSS AS
A RESULT OF MODIFYING, DISTRIBUTING, OR OTHERWISE USING PYTHON,
OR ANY DERIVATIVE THEREOF, EVEN IF ADVISED OF THE POSSIBILITY THEREOF.

6. This License Agreement will automatically terminate upon a material
breach of its terms and conditions.

7. Nothing in this License Agreement shall be deemed to create any
relationship of agency, partnership, or joint venture between PSF and
Licensee.  This License Agreement does not grant permission to use PSF
trademarks or trade name in a trademark sense to endorse or promote
products or services of Licensee, or any third party.

8. By copying, installing or otherwise using Python, Licensee
agrees to be bound by the terms and conditions of this License
Agreement.
```

---

## INRIA, CMU, Microsoft Corporation and HACL* Contributors - HACL* - MIT License

Component: HACL* (CPython 3.12.16's `Modules/_hacl`, built into `python312.dll` on Windows)

Attribution Statements: CPython's Windows build compiles HACL*'s MD5, SHA-1, SHA-2 and SHA-3
code into `python312.dll`, which the Windows release package ships in `extraLibs/`. The
Linux `libpython3.12.so` does not contain it. Source code is available at
https://github.com/hacl-star/hacl-star.

License Text(https://github.com/python/cpython/blob/v3.12.16/Modules/_hacl/Hacl_Hash_SHA3.c)

```
MIT License

Copyright (c) 2016-2022 INRIA, CMU and Microsoft Corporation
Copyright (c) 2022-2023 HACL* Contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## Alliance for Open Media and Joe Drago - libaom and libavif (OpenUSD's hioAvif plugin) - BSD 2-Clause License and Alliance for Open Media Patent License 1.0

Component: libaom (version 3.12.1, AV1 decoder only) and libavif (version 1.0.1), built into
OpenUSD's `hioAvif` image plugin (`plugin/usd/hioAvif.so`, `hioAvif.dll`)

Attribution Statements: OpenUSD reads AVIF image files through its `hioAvif` plugin, which
the release packages ship; the wheel does not. OpenUSD compiles only the decoding parts of
libaom and libavif into it. Their BSD 2-Clause license texts are in OpenUSD's `LICENSE.txt`
(see the OpenUSD entry above). libaom also comes with the Alliance for Open Media Patent
License 1.0 (its `PATENTS` file), which OpenUSD's `LICENSE.txt` does not include. Source code
is available at https://aomedia.googlesource.com/aom/ and https://github.com/AOMediaCodec/libavif.
OpenUSD's `pxr/imaging/plugin/hioAvif/README.md` names libavif v1.0.4 and libaom v3.8.3, but
the vendored sources are libaom 3.12.1 (`aom_version.h`, the version the plugin reports) and
libavif code from after its v1.0.1 release (`avif.h`: 1.0.1 with `AVIF_VERSION_DEVEL 1`).

License Text(https://aomedia.googlesource.com/aom/+/refs/tags/v3.12.1/PATENTS)

```
Alliance for Open Media Patent License 1.0

1. License Terms.

1.1. Patent License. Subject to the terms and conditions of this License, each
     Licensor, on behalf of itself and successors in interest and assigns,
     grants Licensee a non-sublicensable, perpetual, worldwide, non-exclusive,
     no-charge, royalty-free, irrevocable (except as expressly stated in this
     License) patent license to its Necessary Claims to make, use, sell, offer
     for sale, import or distribute any Implementation.

1.2. Conditions.

1.2.1. Availability. As a condition to the grant of rights to Licensee to make,
       sell, offer for sale, import or distribute an Implementation under
       Section 1.1, Licensee must make its Necessary Claims available under
       this License, and must reproduce this License with any Implementation
       as follows:

       a. For distribution in source code, by including this License in the
          root directory of the source code with its Implementation.

       b. For distribution in any other form (including binary, object form,
          and/or hardware description code (e.g., HDL, RTL, Gate Level Netlist,
          GDSII, etc.)), by including this License in the documentation, legal
          notices, and/or other written materials provided with the
          Implementation.

1.2.2. Additional Conditions. This license is directly from Licensor to
       Licensee.  Licensee acknowledges as a condition of benefiting from it
       that no rights from Licensor are received from suppliers, distributors,
       or otherwise in connection with this License.

1.3. Defensive Termination. If any Licensee, its Affiliates, or its agents
     initiates patent litigation or files, maintains, or voluntarily
     participates in a lawsuit against another entity or any person asserting
     that any Implementation infringes Necessary Claims, any patent licenses
     granted under this License directly to the Licensee are immediately
     terminated as of the date of the initiation of action unless 1) that suit
     was in response to a corresponding suit regarding an Implementation first
     brought against an initiating entity, or 2) that suit was brought to
     enforce the terms of this License (including intervention in a third-party
     action by a Licensee).

1.4. Disclaimers. The Reference Implementation and Specification are provided
     "AS IS" and without warranty. The entire risk as to implementing or
     otherwise using the Reference Implementation or Specification is assumed
     by the implementer and user. Licensor expressly disclaims any warranties
     (express, implied, or otherwise), including implied warranties of
     merchantability, non-infringement, fitness for a particular purpose, or
     title, related to the material. IN NO EVENT WILL LICENSOR BE LIABLE TO
     ANY OTHER PARTY FOR LOST PROFITS OR ANY FORM OF INDIRECT, SPECIAL,
     INCIDENTAL, OR CONSEQUENTIAL DAMAGES OF ANY CHARACTER FROM ANY CAUSES OF
     ACTION OF ANY KIND WITH RESPECT TO THIS LICENSE, WHETHER BASED ON BREACH
     OF CONTRACT, TORT (INCLUDING NEGLIGENCE), OR OTHERWISE, AND WHETHER OR
     NOT THE OTHER PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

2. Definitions.

2.1. Affiliate.  "Affiliate" means an entity that directly or indirectly
     Controls, is Controlled by, or is under common Control of that party.

2.2. Control. "Control" means direct or indirect control of more than 50% of
     the voting power to elect directors of that corporation, or for any other
     entity, the power to direct management of such entity.

2.3. Decoder.  "Decoder" means any decoder that conforms fully with all
     non-optional portions of the Specification.

2.4. Encoder.  "Encoder" means any encoder that produces a bitstream that can
     be decoded by a Decoder only to the extent it produces such a bitstream.

2.5. Final Deliverable.  "Final Deliverable" means the final version of a
     deliverable approved by the Alliance for Open Media as a Final
     Deliverable.

2.6. Implementation.  "Implementation" means any implementation, including the
     Reference Implementation, that is an Encoder and/or a Decoder. An
     Implementation also includes components of an Implementation only to the
     extent they are used as part of an Implementation.

2.7. License. "License" means this license.

2.8. Licensee. "Licensee" means any person or entity who exercises patent
     rights granted under this License.

2.9. Licensor.  "Licensor" means (i) any Licensee that makes, sells, offers
     for sale, imports or distributes any Implementation, or (ii) a person
     or entity that has a licensing obligation to the Implementation as a
     result of its membership and/or participation in the Alliance for Open
     Media working group that developed the Specification.

2.10. Necessary Claims.  "Necessary Claims" means all claims of patents or
      patent applications, (a) that currently or at any time in the future,
      are owned or controlled by the Licensor, and (b) (i) would be an
      Essential Claim as defined by the W3C Policy as of February 5, 2004
      (https://www.w3.org/Consortium/Patent-Policy-20040205/#def-essential)
      as if the Specification was a W3C Recommendation; or (ii) are infringed
      by the Reference Implementation.

2.11. Reference Implementation. "Reference Implementation" means an Encoder
      and/or Decoder released by the Alliance for Open Media as a Final
      Deliverable.

2.12. Specification. "Specification" means the specification designated by
      the Alliance for Open Media as a Final Deliverable for which this
      License was issued.
```

---

## Wenzel Jakob - pybind11 - BSD 3-Clause License

Component: `pybind11` (version 2.11.1)

Attribution Statements: USD BRep includes pybind11 headers and uses its
macros/types to define Python binding modules that expose the C++ and C APIs to
Python. This is a compile-time, header-only interaction. Source code is available at
https://github.com/pybind/pybind11.

License Text(https://github.com/pybind/pybind11/blob/master/LICENSE)

```
Copyright (c) 2016 Wenzel Jakob <wenzel.jakob@epfl.ch>, All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
```

---

## Alliance for OpenUSD (AOUSD) Geometry Working Group - UsdSolid and cad_geometry OpenUSD proposals - OpenUSD Technical Proposal Supplemental Terms

Component: `proposals/UsdSolid` and `proposals/cad_geometry` (proposal documents and a
preliminary `schema.usda`), snapshot of commit `811f03c603b06d3b175515a4bf991dee8f42f651`

Attribution Statements: Unmodified snapshots of two AOUSD Geometry Working Group proposals
submitted to Pixar's OpenUSD-proposals repository: "Boundary Representation Geometry in
OpenUSD - Problem Statement" (`cad_geometry`, pull request #108,
https://github.com/PixarAnimationStudios/OpenUSD-proposals/pull/108; Copyright (c) 2026,
Alliance for OpenUSD (AOUSD), Geometry Working Group) and "Solid Models in USD"
(`UsdSolid`, pull request #109,
https://github.com/PixarAnimationStudios/OpenUSD-proposals/pull/109). The source repository
and the release package carry them as design context; the wheel does not. They are not
compiled, registered or loaded. `proposals/NOTICE` and `proposals/README.md` record their
provenance.

Terms: https://openusd.org/release/contributing_supplemental.html

---

## NVIDIA Corporation - usd-validation-nvidia (Asset Validator) - Apache License 2.0 (source code) and CC BY 4.0 (documentation/skills)

Component: `usd-validation-nvidia` pip package (provides `usd_validation_nvidia`,
`omni.asset_validator`, and `omni.capabilities`)

Attribution Statements: An optional dependency that the user installs from PyPI with
`pip install usd-brep[validation]`; it is not redistributed with the USD BRep product. Only
the wheel's `brep_usdschema_validator` plugin uses it, to run USD asset-validation rules.
Distribution: https://pypi.org/project/usd-validation-nvidia/.

License: the package is dual-licensed by content type, as stated in the `LICENSE` file it
ships: its source code is under the Apache License, Version 2.0 and its
documentation/skills are under the Creative Commons Attribution 4.0 International Public
License (https://creativecommons.org/licenses/by/4.0/legalcode). The complete texts are in
that package's own `LICENSE` file.

License Text (source code; https://www.apache.org/licenses/LICENSE-2.0)

```
Copyright (c) 2020-2026, NVIDIA CORPORATION & AFFILIATES.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

---

## NVIDIA CORPORATION - Repository Bootstrap Scripts - MIT License

Component: `repo.sh`, `repo.bat`, `tools/repoman/repoman_bootstrapper.py`

Attribution Statements: These three files bootstrap the `repo` build-tool launcher
shared across NVIDIA Omniverse repositories and are vendored/updated from that shared
tooling. NVIDIA actively chooses the MIT license for these specific files; the rest of
the NVIDIA-authored source in this repository is licensed as stated at the top of this
document and in `LICENSE`.

License Text

```
MIT License

Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
```

## Various - Optional GUI developer tool (uv.lock dependency group `gui`) - LGPL v3, MIT, BSD and others

Component: Python packages declared in `pyproject.toml` (dependency group `gui`) and
pinned, with their transitive dependencies, in `uv.lock`. They are used only by the
optional SMLib GUI developer tool (`tools/smlib_gui`, started with
`uv run --locked --group gui python tools/scripts/smlib_gui.py`).

Attribution Statements: These packages are obtained from PyPI by the user when the GUI is
run. They are not packman dependencies and are not redistributed in the USD BRep wheel or
release package, which contain none of the GUI code. They are listed for completeness
because the source repository carries `pyproject.toml`, `uv.lock` and the GUI. The licenses
below are the ones each package declares in its metadata; the authoritative license texts
are the ones shipped inside each package. `uv.lock` is the authoritative full list (35
packages for the locked Linux and Windows environments). The PySide6-Essentials wheel also
contains Qt modules that Qt licenses only under the GPL (Qt Qml Compiler, Qt Lottie
Animation, Qt Quick Timeline, Qt Wayland Compositor and the Qt Virtual Keyboard input
plugin); the GUI uses only QtCore, QtGui and QtWidgets.

Direct dependencies:

| Package | Version | License | Use |
|---|---|---|---|
| PySide6-Essentials | 6.11.1 | LGPL v3 OR GPL v2 OR GPL v3 | GUI toolkit (includes the Qt 6 libraries) |
| numpy | 2.4.6 | BSD-3-Clause AND 0BSD AND MIT AND Zlib AND CC0-1.0 | array handling |
| pyvista | 0.48.4 | MIT | 3D plotting |
| pyvistaqt | 0.11.4 | MIT | embeds pyvista in a Qt window |
| vtk | 9.6.2 | BSD | rendering |

Transitive dependencies, by declared license:

* LGPL v3 OR GPL v2 OR GPL v3: shiboken6 (the binding runtime used by PySide6)
* BSD-2-Clause: Pygments
* BSD / BSD-3-Clause: contourpy, cycler, kiwisolver, idna, pooch
* MIT: attrs, charset-normalizer, docstring-parser, fonttools, markdown-it-py, mdurl, platformdirs, pyparsing, QtPy, rich, rich-rst, scooby, six, urllib3
* MIT-CMU: pillow
* Apache-2.0: cyclopts, requests
* Apache-2.0 OR BSD-2-Clause: packaging
* Apache-2.0 / BSD dual license: python-dateutil
* MPL-2.0: certifi
* PSF license: matplotlib, typing-extensions (PSF-2.0)

---

# Notes

* This file enumerates the packages declared in `deps/target-deps.packman.xml` and
  `deps/target-deps-python.packman.xml`.
  Build-host-only and developer tooling dependencies (`deps/host-deps.packman.xml`,
  `deps/repo-deps.packman.xml`, and the optional `deps/repo-deps-nv.packman.xml`
  side-car) are not included here because they are not redistributed with the product.
* The `repo.sh` / `repo.bat` / `tools/repoman/repoman_bootstrapper.py` entry above is a
  first-party exception, not a packman dependency: these three files are licensed MIT
  rather than under this repository's primary license. See `LICENSE` and the entry above
  for the full text.
* The `usd-validation-nvidia` entry above is another exception: it is not a packman
  dependency but an optional pip dependency of the wheel that the user installs. It is not
  redistributed with the product.
* The optional GUI developer-tool entry above is a further exception of the same kind: the
  packages come from PyPI via `uv.lock`, not from packman, and are not redistributed with
  the product.
* The OpenUSD package bundles further third-party libraries that the release package ships
  in `extraLibs/` (on Linux, for example, Alembic, Imath, OpenSubdiv and zlib). Their license
  texts are in `PACKAGE-LICENSES/OpenUSD/`, copied from the OpenUSD package.
* For each NVIDIA-distributed package above, the canonical and most up-to-date
  copyright, license, and embedded third-party notices are the ones shipped
  inside that package (typically as `LICENSE`, `LICENSE.txt`, or `THIRD_PARTY_NOTICES`).
  The entries here are a product-level summary intended to satisfy the OSS attribution
  requirement at the level of the USD BRep distribution.
