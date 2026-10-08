:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
@echo on

set "start_dir=%cd%"
set rtn=0

ECHO Run usd_test tests
cd "%start_dir%"
call "tests\usd_test\usd_test.bat"
ECHO Error level after usd_test.bat:  %ERRORLEVEL%
if %ERRORLEVEL% == 0 (
ECHO usd_test passed
) else (
ECHO usd_test failed
set rtn=1
)

:: Unit tests for the kernel-free USD_BREP_UTILS math (knot conversions + analytic/NURBS
:: evaluators + inverse projection). Pure functions, no USD stage required.
ECHO Run usd_brep_utils_test tests
cd "%start_dir%"
call "tests\usd_brep_utils_test\usd_brep_utils_test.bat"
ECHO Error level after usd_brep_utils_test.bat:  %ERRORLEVEL%
if %ERRORLEVEL% == 0 (
ECHO usd_brep_utils_test passed
) else (
ECHO usd_brep_utils_test failed
set rtn=1
)



ECHO Run prog_test tests
cd "%start_dir%"
call "tests\prog_test\prog_test.bat"
ECHO Error level after prog_test.bat:  %ERRORLEVEL%
if %ERRORLEVEL% == 0 (
ECHO prog tests passed
) else (
ECHO prog tests failed
set rtn=1
)

ECHO Run SM_API_test tests
cd "%start_dir%"
call "tests\SM_API_test\SM_API_test.bat"
ECHO Error level after SM_API_test.bat:  %ERRORLEVEL%
if %ERRORLEVEL% == 0 (
ECHO SM_API tests passed
) else (
ECHO SM_API tests failed
set rtn=1
)

ECHO Run SM_API_USD_test tests
cd "%start_dir%"
call "tests\SM_API_USD_test\SM_API_USD_test.bat"
ECHO Error level after SM_API_USD_test.bat:  %ERRORLEVEL%
if %ERRORLEVEL% == 0 (
ECHO SM_API_USD tests passed
) else (
ECHO SM_API_USD tests failed
set rtn=1
)

:: The hdUsdBrep plugin is built only against an OpenUSD with imaging (usd_has_imaging() in premake5-libraries.lua).
if not exist "%start_dir%\_build\target-deps\usd\release\include\pxr\imaging\hd\" (
ECHO Skipping HYDRA_USD_BREP_test: OpenUSD built without imaging
goto hydra_done
)
ECHO Run HYDRA_USD_BREP_test tests
cd "%start_dir%"
call "tests\HYDRA_USD_BREP_test\HYDRA_USD_BREP_test.bat"
if not "%ERRORLEVEL%" == "0" set rtn=1
:hydra_done

ECHO Run brep_geometry_validator Python tests
cd "%start_dir%"
set "BREP_GEOMETRY_VALIDATOR=%start_dir%\_build\windows-x86_64\release\brep_geometry_validator.exe"
set "OMNISOLID_PLUGIN_PATH=%start_dir%\_build\schema\omniSolid\resources"
set "PYTHONPATH=%start_dir%\_build\target-deps\usd\release\lib\python"
set "PATH=%start_dir%\_build\windows-x86_64\release;%start_dir%\_build\target-deps\usd\release\bin;%start_dir%\_build\target-deps\usd\release\lib;%start_dir%\_build\target-deps\python;%PATH%"
call "_build\target-deps\python\python.exe" -m unittest discover -s tools\brep_geometry_validator\tests -p "test_*.py"
if not "%ERRORLEVEL%" == "0" set rtn=1

ECHO Run heal_file Python tests
cd "%start_dir%"
set "OMNISOLID_PLUGIN_PATH=%start_dir%\_build\schema\omniSolid\resources"
set "PYTHONPATH=%start_dir%\_build\target-deps\usd\release\lib\python;%start_dir%\_build\windows-x86_64\release"
set "PATH=%start_dir%\_build\windows-x86_64\release;%start_dir%\_build\target-deps\usd\release\bin;%start_dir%\_build\target-deps\usd\release\lib;%start_dir%\_build\target-deps\python;%PATH%"
call "_build\target-deps\python\python.exe" -m unittest discover -s source\SmPyLib\tests_usd -p "test_*.py"
if not "%ERRORLEVEL%" == "0" set rtn=1

ECHO Run SmPyLib (Python bindings) tests
cd "%start_dir%"
set "OMNISOLID_PLUGIN_PATH=%start_dir%\_build\schema\omniSolid\resources"
set "PYTHONPATH=%start_dir%\_build\windows-x86_64\release"
set "PATH=%start_dir%\_build\windows-x86_64\release;%start_dir%\_build\target-deps\usd\release\lib;%start_dir%\_build\target-deps\python;%PATH%"
call "_build\target-deps\python\python.exe" -m unittest discover -s source\SmPyLib\tests -p "test_*.py" -v
ECHO Error level after SmPyLib tests:  %ERRORLEVEL%
if %ERRORLEVEL% == 0 (
ECHO SmPyLib tests passed
) else (
ECHO SmPyLib tests failed
set rtn=1
)

if exist "%start_dir%\_build\windows-x86_64\release\_smlib_dev*.pyd" (
    ECHO Run SmPyDevLib tests
    call "_build\target-deps\python\python.exe" -m unittest discover -s source\SmPyDevLib\tests -p "test_*.py" -v
    if errorlevel 1 set rtn=1
) else (
    ECHO Skipping SmPyDevLib tests (_smlib_dev module not built^)
)

:: OCCT occt_to_usd importer tests run everywhere (including CI) as a hard gate:
:: they convert the committed .brep corpus to USD, validate the schema, and run a
:: SMLib AssertValid round-trip. usd_test_app (spawned by --assert-valid) links the
:: SMLib shared libs in _build\windows-x86_64\release and needs the omniSolid plugin;
:: the SmPyLib block above already set OMNISOLID_PLUGIN_PATH / PYTHONPATH / PATH to
:: the same values, so reuse them here.
ECHO Run OCCT occt_to_usd importer tests
cd "%start_dir%"
set "OCCT_TO_USD=%start_dir%\_build\windows-x86_64\release\occt_to_usd.exe"
set "USD_TEST_APP=%start_dir%\_build\windows-x86_64\release\usd_test_app.exe"
set "USD_TO_OCCT=%start_dir%\_build\windows-x86_64\release\usd_to_occt.exe"
:: Hard gate: fail fast if a required binary is missing so the Python suite cannot
:: silently skip (or auto-discover a stale artifact for the wrong platform).
if not exist "%OCCT_TO_USD%" (ECHO OCCT occt_to_usd importer tests failed: required binary missing %OCCT_TO_USD% & set rtn=1 & goto occt_exporter)
if not exist "%USD_TEST_APP%" (ECHO OCCT occt_to_usd importer tests failed: required binary missing %USD_TEST_APP% & set rtn=1 & goto occt_exporter)
call "_build\target-deps\python\python.exe" -m unittest discover -s tools\occt_to_usd_test -p "test_*.py"
ECHO Error level after OCCT occt_to_usd importer tests:  %ERRORLEVEL%
if %ERRORLEVEL% == 0 (
ECHO OCCT occt_to_usd importer tests passed
) else (
ECHO OCCT occt_to_usd importer tests failed
set rtn=1
)

:: OCCT usd_to_occt exporter tests: round-trip the committed corpus through the exporter
:: (occt_to_usd seed import -> usd_to_occt export -> occt_to_usd re-import -> validate).
:: Reuses the importer binary and usd_test_app, so pin all three and fail fast if any is missing.
:occt_exporter
ECHO Run OCCT usd_to_occt exporter tests
cd "%start_dir%"
if not exist "%USD_TO_OCCT%" (ECHO OCCT usd_to_occt exporter tests failed: required binary missing %USD_TO_OCCT% & set rtn=1 & goto occt_done)
if not exist "%OCCT_TO_USD%" (ECHO OCCT usd_to_occt exporter tests failed: required binary missing %OCCT_TO_USD% & set rtn=1 & goto occt_done)
if not exist "%USD_TEST_APP%" (ECHO OCCT usd_to_occt exporter tests failed: required binary missing %USD_TEST_APP% & set rtn=1 & goto occt_done)
call "_build\target-deps\python\python.exe" -m unittest discover -s tools\usd_to_occt_test -p "test_*.py"
ECHO Error level after OCCT usd_to_occt exporter tests:  %ERRORLEVEL%
if %ERRORLEVEL% == 0 (
ECHO OCCT usd_to_occt exporter tests passed
) else (
ECHO OCCT usd_to_occt exporter tests failed
set rtn=1
)
:occt_done

ECHO Run brep_validator Python unittests
cd "%start_dir%"
:: usd-validation-nvidia is pure Python. Install it into _build\pydeps, not into the
:: interpreter, which can be a link into a shared packman cache. The folder is named
:: after the version range, so changing the range installs afresh. Change both lines.
set "validator_requirement=usd-validation-nvidia>=1.22.0,<2"
set "pydeps=%start_dir%\_build\pydeps\usd-validation-nvidia-1.22-2"
if not exist "%pydeps%\usd_validation_nvidia\" (
call "_build\target-deps\python\python.exe" -m pip install --quiet --disable-pip-version-check --target "%pydeps%" "%validator_requirement%"
if errorlevel 1 (
ECHO Installing usd-validation-nvidia failed
set rtn=1
goto brep_validator_done
)
)
set "PYTHONPATH=%pydeps%;%PYTHONPATH%"
:: Kept out of ( ) blocks: there cmd would read ERRORLEVEL before the call runs.
call "_build\target-deps\python\python.exe" -m unittest discover -s tools\brep_validator_test -p "test_*.py"
ECHO Error level after brep_validator Python unittests:  %ERRORLEVEL%
if %ERRORLEVEL% == 0 (
ECHO brep_validator Python unittests passed
) else (
ECHO brep_validator Python unittests failed
set rtn=1
)
:brep_validator_done

exit /B %rtn%
