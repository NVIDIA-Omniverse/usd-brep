USD BRep Python API
===================

The ``usd_brep`` package exposes the ``SM_API`` and ``SM_API_USD`` layers to
Python through pybind11.

Installing
----------

.. code-block:: bash

   pip install usd-brep

CPython 3.12 on Windows or Linux (x86_64, aarch64). The wheel bundles the OpenUSD
runtime it was built against, so no separate OpenUSD installation is needed and
nothing is added to the ``pxr`` namespace. On Windows it also needs the
`Microsoft Visual C++ Redistributable
<https://learn.microsoft.com/cpp/windows/latest-supported-vc-redist>`_.

The prebuilt package is an alternative. Like the wheel, it bundles its OpenUSD
runtime, including OpenUSD's ``pxr`` modules, so you supply only a CPython 3.12
interpreter. It is importable once its ``python`` and ``usdpy`` directories are on
``PYTHONPATH`` and its ``lib`` and ``extraLibs`` directories are on the platform's
library search path (``LD_LIBRARY_PATH`` on Linux, ``PATH`` on Windows), set before
Python starts.

.. _using-it:

Using the Python API
--------------------

Functions are also grouped into submodule attributes of ``usd_brep``
(``primitives``, ``curves``, ``surfaces``, ``booleans``, ``sweeps``, ``fillets``,
``offset``, ``io``, ``brep_ops``, ``stitching``, ``usd``, …). These are pybind11
submodules rather than importable Python modules, so they are reached as
attributes — ``usd_brep.usd.export_brep(...)`` — and the functions they hold
are the same objects documented below.

The compiled module underneath is named ``_omni_solid`` and importing it directly
still works, but ``usd_brep`` is the supported name: it also registers the library
search path on Windows and points ``OMNISOLID_PLUGIN_PATH`` at the packaged
omniSolid schema, both of which a bare ``import _omni_solid`` leaves to the caller.

Import ``usd_brep`` before opening any stage with ``pxr``: it registers the
omniSolid schema plugin, and OpenUSD ignores schema plugins registered after it
has loaded its schema types. Otherwise, set ``PXR_PLUGINPATH_NAME`` to the
omniSolid ``resources`` directory.

Reference
---------

.. automodule:: usd_brep
    :platform: Windows-x86_64, Linux-x86_64, Linux-aarch64
    :members:
    :imported-members:
