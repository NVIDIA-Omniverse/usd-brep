// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Python bindings for native SMLib file I/O through SM_API.

#include "SmPyCommon.h"

void bind_file_io(BoundModule& m)
{
    m.def("read_brep_from_file",
          [](const std::string& filename, py::object ascii, bool rebuild_uv_trim_curves) {
              return smpy_read_brep_from_file(filename, ascii, rebuild_uv_trim_curves);
          },
          py::return_value_policy::reference,
          py::arg("filename"), py::arg("ascii") = py::none(),
          py::arg("rebuild_uv_trim_curves") = false,
          "Pure: returns new Brep; inputs unchanged.\n"
          "\n"
          "Deserialize a native SMLib Brep file from disk.\n"
          "\n"
          "Args:\n"
          "    filename: Path to an SMLib native Brep file.\n"
          "    ascii: Format selector. Use ``None`` to auto-detect,\n"
          "        ``True`` for ASCII, or ``False`` for binary.\n"
          "    rebuild_uv_trim_curves: Rebuild UV trim curves after loading.\n"
          "\n"
          "Returns:\n"
          "    Brep: loaded model owned by the module context.\n"
          "\n"
          "Notes:\n"
          "    Recognized SMLib part containers are rejected; load them with\n"
          "    ``read_part_from_file`` and the matching ASCII/binary selector.\n"
          "\n"
          "See Also:\n"
          "    Brep.read_from_file: Class-method spelling.\n"
          "    Brep.write_to_file: Serialize a Brep to native SMLib format.\n"
          "    read_part_from_file: Deserialize an SMLib part container.\n"
          "\n"
          "Wraps: SmApiReadBrepFromFile");

    m.def("read_part_from_file",
          [](const std::string& filename, bool ascii) {
              return smpy_read_part_from_file(filename, ascii);
          },
          py::arg("filename"), py::arg("ascii") = true,
          "Pure: returns new objects; inputs unchanged.\n"
          "\n"
          "Deserialize an SMLib part file containing curves, surfaces, Boolean\n"
          "tree nodes, and Breps.\n"
          "\n"
          "Args:\n"
          "    filename: Path to an SMLib part file, conventionally ``.smp``.\n"
          "    ascii: Read the ASCII format when true, or binary when false.\n"
          "\n"
          "Returns:\n"
          "    tuple[list[Curve], list[Surface], list[int], list[Brep]]: Standalone\n"
          "        curves, standalone surfaces, Boolean tree nodes, and Brep models.\n"
          "\n"
          "See Also:\n"
          "    read_brep_from_file: Read a single-Brep ``.smb`` file.\n"
          "\n"
          "Wraps: SmApiReadPartFromFile");
}
