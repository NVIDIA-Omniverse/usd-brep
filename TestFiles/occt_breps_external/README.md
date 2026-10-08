# External (third-party) OCCT `.brep` test samples

This directory is for **third-party** `.brep` files used to exercise the OCCT importer and
exporter against real-world parts, such as the sample data shipped with the OpenCASCADE
distribution (under its `data/occ` directory).

**These files must never be committed.** Their data license is incompatible with this
repository, so everything here except this `README.md` and the `.gitignore` is ignored by git
(see `.gitignore`). Keeping them in a dedicated directory keeps them separate from the
self-authored, shareable corpus in `../occt_breps`.

## How it is tested

The external-sample tests in `tools/occt_to_usd_test/test_occt_to_usd.py` and
`tools/usd_to_occt_test/test_usd_to_occt.py` discover every `*.brep` under this directory
(recursively) and run each one through the converter. They **skip** when the directory contains
no `.brep` files, so a fresh clone or CI without this data still passes.

## Populating it

First set `OCCT_ROOT` to your OpenCASCADE source or installation directory:

    export OCCT_ROOT=/path/to/opencascade

Then copy (or symlink) sample files here, for example:

    cp "$OCCT_ROOT"/data/occ/*.brep TestFiles/occt_breps_external/

Or point the tests at an out-of-tree directory without copying anything:

    export OCCT_BREP_EXTERNAL_DIR="$OCCT_ROOT/data/occ"
