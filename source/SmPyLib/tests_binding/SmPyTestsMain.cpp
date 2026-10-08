// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Dev-only Python bindings for in-process SMLib regression tests.  This module
// intentionally stays separate from the stable _omni_solid API surface.

#include <prog_test.h>
#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#ifndef _WIN32
#include <dlfcn.h> // dladdr() in symbol_info(); Windows has no equivalent here
#endif

namespace py = pybind11;

namespace
{
    constexpr size_t kMaxCapturedDrawBatches = 20000;
    constexpr size_t kMaxCapturedDrawPositions = 250000;

    struct CapturedDrawBatch
    {
        std::string primitive;
        std::vector<std::array<double, 3>> positions;
        std::array<double, 3> color { 0.0, 0.0, 0.0 };
        double pointSize = 10.0;
        double lineWidth = 2.0;
        bool dashed = false;
        bool connected = true;
    };

    struct DrawCaptureState
    {
        std::vector<CapturedDrawBatch> batches;
        size_t batchesSeen = 0;
        size_t positionsSeen = 0;
        size_t positionsCaptured = 0;
        bool truncated = false;
    };

    std::mutex g_drawCaptureMutex;
    std::mutex g_progTestMutex;
    DrawCaptureState* g_drawCaptureState = nullptr;

    bool has_same_draw_style(const CapturedDrawBatch& lhs, const CapturedDrawBatch& rhs)
    {
        return lhs.primitive == rhs.primitive &&
               lhs.color == rhs.color &&
               lhs.pointSize == rhs.pointSize &&
               lhs.lineWidth == rhs.lineWidth &&
               lhs.dashed == rhs.dashed &&
               lhs.connected == rhs.connected;
    }

    CapturedDrawBatch* find_merge_target(DrawCaptureState& state, const CapturedDrawBatch& batch)
    {
        for (CapturedDrawBatch& existing : state.batches)
        {
            if (has_same_draw_style(existing, batch))
            {
                return &existing;
            }
        }
        return nullptr;
    }

    const char* sm_status_name_for_tests(SmStatus status)
    {
        switch (status)
        {
            case SM_SUCCESS:                     return "SM_SUCCESS";
            case SM_ERR:                         return "SM_ERR";
            case SM_ERR_OUT_OF_MEMORY:           return "SM_ERR_OUT_OF_MEMORY";
            case SM_ERR_UNKNOWN:                 return "SM_ERR_UNKNOWN";
            case SM_ERR_FATAL:                   return "SM_ERR_FATAL";
            case SM_ERR_ASSERT_FAILURE:          return "SM_ERR_ASSERT_FAILURE";
            case SM_ERR_NULL_POINTER:            return "SM_ERR_NULL_POINTER";
            case SM_ERR_INVALID_INPUT:           return "SM_ERR_INVALID_INPUT";
            case SM_ERR_NON_NULL_OUTPUT_POINTER: return "SM_ERR_NON_NULL_OUTPUT_POINTER";
            case SM_ERR_ASSERTVALID_FAILURE:     return "SM_ERR_ASSERTVALID_FAILURE";
            case SM_ERR_OUTSIDE_OF_DOMAIN:       return "SM_ERR_OUTSIDE_OF_DOMAIN";
            case SM_ERR_NOT_WITHIN_TOLERANCE:    return "SM_ERR_NOT_WITHIN_TOLERANCE";
            case SM_ERR_NOT_CONVERGING:          return "SM_ERR_NOT_CONVERGING";
            case SM_ERR_WARNING:                 return "SM_ERR_WARNING";
            case SM_ERR_MESSAGE:                 return "SM_ERR_MESSAGE";
            case SM_ERR_AXIS_INSIDE_FACE:        return "SM_ERR_AXIS_INSIDE_FACE";
            case SM_ERR_LICENSE_EXPIRED:         return "SM_ERR_LICENSE_EXPIRED";
            case SM_ERR_BAD_INTERSECTIONS:       return "SM_ERR_BAD_INTERSECTIONS";
            case SM_ERR_BAD_COINCIDENT_VERTICES: return "SM_ERR_BAD_COINCIDENT_VERTICES";
            case SM_ERR_BAD_SURFACE_POINT:       return "SM_ERR_BAD_SURFACE_POINT";
            case SM_ERR_BAD_TANGENT_DROP:        return "SM_ERR_BAD_TANGENT_DROP";
            case SM_ERR_DEGENERATE_SURFACE:      return "SM_ERR_DEGENERATE_SURFACE";
            case SM_ERR_BAD_FIND_DEGEN_PARAM:    return "SM_ERR_BAD_FIND_DEGEN_PARAM";
            case SM_ERR_METHOD_FAILURE_QUITING:  return "SM_ERR_METHOD_FAILURE_QUITING";
            case SM_ERR_NOTYET_HEAL_FACE:        return "SM_ERR_NOTYET_HEAL_FACE";
            case SM_ERR_TESS_FAIL_FACE:          return "SM_ERR_TESS_FAIL_FACE";
            default:                             return "SM_ERR_???";
        }
    }

    std::array<double, 3> current_draw_color(SmGfxArraySet* pOptGfxSet)
    {
        SmVector3d color = smgfx_GetOutputColor(pOptGfxSet);
        return { color.x, color.y, color.z };
    }

    double current_point_size(SmGfxArraySet* pOptGfxSet)
    {
        return smgfx_GetOutputPointSize(pOptGfxSet);
    }

    double current_line_width(SmGfxArraySet* pOptGfxSet)
    {
        return smgfx_GetOutputLineWidth(pOptGfxSet);
    }

    bool current_dashed_lines(SmGfxArraySet* pOptGfxSet)
    {
        return smgfx_GetOutputDashedLines(pOptGfxSet) == TRUE;
    }

    void append_draw_batch(CapturedDrawBatch&& batch)
    {
        std::lock_guard<std::mutex> lock(g_drawCaptureMutex);
        DrawCaptureState* state = g_drawCaptureState;
        if (!state)
        {
            return;
        }

        ++state->batchesSeen;
        state->positionsSeen += batch.positions.size();
        const size_t positionCount = batch.positions.size();
        CapturedDrawBatch* mergeTarget = find_merge_target(*state, batch);
        if ((!mergeTarget && state->batches.size() >= kMaxCapturedDrawBatches) ||
            state->positionsCaptured + positionCount > kMaxCapturedDrawPositions)
        {
            state->truncated = true;
            return;
        }

        state->positionsCaptured += positionCount;
        if (mergeTarget)
        {
            mergeTarget->positions.insert(mergeTarget->positions.end(),
                                          batch.positions.begin(),
                                          batch.positions.end());
        }
        else
        {
            state->batches.emplace_back(std::move(batch));
        }
    }

    void capture_draw_point(double x, double y, double z, SmGfxArraySet* pOptGfxSet)
    {
        CapturedDrawBatch batch;
        batch.primitive = "point";
        batch.positions.push_back({ x, y, z });
        batch.color = current_draw_color(pOptGfxSet);
        batch.pointSize = current_point_size(pOptGfxSet);
        batch.lineWidth = current_line_width(pOptGfxSet);
        batch.dashed = current_dashed_lines(pOptGfxSet);
        batch.connected = false;
        append_draw_batch(std::move(batch));
    }

    void capture_draw_line(double x1,
                           double y1,
                           double z1,
                           double x2,
                           double y2,
                           double z2,
                           SmGfxArraySet* pOptGfxSet)
    {
        CapturedDrawBatch batch;
        batch.primitive = "line";
        batch.positions.push_back({ x1, y1, z1 });
        batch.positions.push_back({ x2, y2, z2 });
        batch.color = current_draw_color(pOptGfxSet);
        batch.pointSize = current_point_size(pOptGfxSet);
        batch.lineWidth = current_line_width(pOptGfxSet);
        batch.dashed = current_dashed_lines(pOptGfxSet);
        batch.connected = false;
        append_draw_batch(std::move(batch));
    }

    void capture_draw_polyline(double* pts, long npts, SmGfxArraySet* pOptGfxSet)
    {
        if (!pts || npts < 2)
        {
            return;
        }

        CapturedDrawBatch batch;
        batch.primitive = "line";
        batch.positions.reserve(static_cast<size_t>(2 * (npts - 1)));
        for (long ii = 0; ii < npts - 1; ++ii)
        {
            batch.positions.push_back({ pts[3 * ii], pts[3 * ii + 1], pts[3 * ii + 2] });
            batch.positions.push_back({ pts[3 * (ii + 1)], pts[3 * (ii + 1) + 1], pts[3 * (ii + 1) + 2] });
        }
        batch.color = current_draw_color(pOptGfxSet);
        batch.pointSize = current_point_size(pOptGfxSet);
        batch.lineWidth = current_line_width(pOptGfxSet);
        batch.dashed = current_dashed_lines(pOptGfxSet);
        batch.connected = false;
        append_draw_batch(std::move(batch));
    }

    class ScopedWorkingDirectory
    {
    public:
        explicit ScopedWorkingDirectory(const std::string& path)
        {
            if (path.empty())
            {
                return;
            }

            std::error_code error;
            m_oldPath = std::filesystem::current_path(error);
            if (error)
            {
                throw std::runtime_error("Could not read current working directory: " + error.message());
            }

            std::filesystem::current_path(path, error);
            if (error)
            {
                throw std::runtime_error("Could not change to prog_test working directory: " + path);
            }
            m_changed = true;
        }

        ~ScopedWorkingDirectory()
        {
            if (m_changed)
            {
                std::error_code ignored;
                std::filesystem::current_path(m_oldPath, ignored);
            }
        }

        ScopedWorkingDirectory(const ScopedWorkingDirectory&) = delete;
        ScopedWorkingDirectory& operator=(const ScopedWorkingDirectory&) = delete;

    private:
        std::filesystem::path m_oldPath;
        bool m_changed = false;
    };

    class ScopedProgTestRunFlags
    {
    public:
        explicit ScopedProgTestRunFlags(bool doGraphics)
            : m_doGraphics(smGet_DoGraphics())
            , m_outputLong(smGet_OutputLong())
            , m_outputThin(smGet_OutputThin())
            , m_outputDebugLog(smGet_OutputDebugLog())
        {
            smSet_DoGraphics(doGraphics ? TRUE : FALSE);
            smSet_OutputLong(FALSE);
            smSet_OutputThin(FALSE);
            smSet_OutputDebugLog(FALSE);
        }

        ~ScopedProgTestRunFlags()
        {
            smSet_DoGraphics(m_doGraphics);
            smSet_OutputLong(m_outputLong);
            smSet_OutputThin(m_outputThin);
            smSet_OutputDebugLog(m_outputDebugLog);
        }

        ScopedProgTestRunFlags(const ScopedProgTestRunFlags&) = delete;
        ScopedProgTestRunFlags& operator=(const ScopedProgTestRunFlags&) = delete;

    private:
        SmBoolean m_doGraphics;
        SmBoolean m_outputLong;
        SmBoolean m_outputThin;
        SmBoolean m_outputDebugLog;
    };

    class ScopedDrawCapture
    {
    public:
        explicit ScopedDrawCapture(DrawCaptureState& state)
        {
            {
                std::lock_guard<std::mutex> lock(g_drawCaptureMutex);
                g_drawCaptureState = &state;
            }
            smgfx_SetDrawPointCallBack(&capture_draw_point);
            smgfx_SetDrawLineCallBack(&capture_draw_line);
            smgfx_SetDrawPolyLineCallBack(&capture_draw_polyline);
        }

        ~ScopedDrawCapture()
        {
            smgfx_SetDrawPointCallBack(nullptr);
            smgfx_SetDrawLineCallBack(nullptr);
            smgfx_SetDrawPolyLineCallBack(nullptr);
            std::lock_guard<std::mutex> lock(g_drawCaptureMutex);
            g_drawCaptureState = nullptr;
        }

        ScopedDrawCapture(const ScopedDrawCapture&) = delete;
        ScopedDrawCapture& operator=(const ScopedDrawCapture&) = delete;
    };

    void ensure_prog_test_output_dir()
    {
        std::error_code error;
        std::filesystem::create_directory("OutputFiles", error);
        if (error)
        {
            throw std::runtime_error("Could not create OutputFiles: " + error.message());
        }
    }

    py::tuple point_tuple(const std::array<double, 3>& point)
    {
        return py::make_tuple(point[0], point[1], point[2]);
    }

    py::dict captured_batch_to_dict(const CapturedDrawBatch& batch, size_t index)
    {
        py::dict out;
        py::list positions;
        for (const auto& point : batch.positions)
        {
            positions.append(point_tuple(point));
        }

        py::dict metadata;
        metadata["source"] = "prog_test";
        metadata["batch_index"] = index;

        out["primitive"] = batch.primitive;
        out["draw_type"] = batch.primitive;
        out["positions"] = positions;
        out["color"] = point_tuple(batch.color);
        out["point_size"] = batch.pointSize;
        out["line_width"] = batch.lineWidth;
        out["dashed"] = batch.dashed;
        out["connected"] = batch.connected;
        out["metadata"] = metadata;
        return out;
    }

    py::list captured_draw_events_to_list(const DrawCaptureState& capture)
    {
        py::list events;
        if (capture.batches.empty())
        {
            return events;
        }

        py::list batches;
        for (size_t ii = 0; ii < capture.batches.size(); ++ii)
        {
            batches.append(captured_batch_to_dict(capture.batches[ii], ii));
        }

        py::dict metadata;
        metadata["source"] = "prog_test";
        metadata["batch_count"] = capture.batches.size();
        metadata["raw_batch_count"] = capture.batchesSeen;
        metadata["batches_seen"] = capture.batchesSeen;
        metadata["positions_captured"] = capture.positionsCaptured;
        metadata["positions_seen"] = capture.positionsSeen;
        metadata["coalesced"] = true;
        metadata["truncated"] = capture.truncated;

        py::dict event;
        event["type"] = "display_list";
        event["name"] = "prog_test_draw";
        event["batches"] = batches;
        event["metadata"] = metadata;
        events.append(event);
        return events;
    }

    py::dict result_to_dict(const ProgTestRunResult& result,
                            const std::string& logText,
                            const DrawCaptureState& capture)
    {
        py::dict out;
        out["suite_name"] = result.pSuiteName ? result.pSuiteName : "";
        out["status"] = static_cast<long>(result.eStatus);
        out["status_name"] = sm_status_name_for_tests(result.eStatus);
        out["ok"] = result.bOk == TRUE;
        out["elapsed_seconds"] = result.dElapsedSeconds;
        out["log"] = logText;
        out["draw_events"] = captured_draw_events_to_list(capture);
        return out;
    }

    py::dict symbol_info(const char* name, void* symbol)
    {
        py::dict out;
        out["name"] = name;
        out["address"] = py::int_(reinterpret_cast<std::uintptr_t>(symbol));
#ifndef _WIN32
        Dl_info info {};
        if (dladdr(symbol, &info) != 0)
        {
            out["path"] = info.dli_fname ? info.dli_fname : "";
            out["symbol"] = info.dli_sname ? info.dli_sname : "";
        }
        else
#endif
        {
            out["path"] = "";
            out["symbol"] = "";
        }
        return out;
    }

    py::dict compile_flags()
    {
        py::dict flags;
#ifdef SM_DEBUG_CODE
        flags["SM_DEBUG_CODE"] = true;
#else
        flags["SM_DEBUG_CODE"] = false;
#endif
#ifdef SM_GFX_CODE
        flags["SM_GFX_CODE"] = true;
#else
        flags["SM_GFX_CODE"] = false;
#endif
#ifdef SM_GFX_OUTPUT_CODE
        flags["SM_GFX_OUTPUT_CODE"] = true;
#else
        flags["SM_GFX_OUTPUT_CODE"] = false;
#endif
#if defined(SM_GRAPHICS_CALLBACKS) && SM_GRAPHICS_CALLBACKS
        flags["SM_GRAPHICS_CALLBACKS"] = true;
#else
        flags["SM_GRAPHICS_CALLBACKS"] = false;
#endif
        return flags;
    }
}

PYBIND11_MODULE(_smlib_tests, m)
{
    m.doc() = "Developer-only in-process bindings for selected SMLib regression tests.";

    m.def("list_prog_test_suites", []() {
        std::vector<std::string> names;
        int count = prog_test_suite_count();
        names.reserve(static_cast<size_t>(count));
        for (int ii = 0; ii < count; ++ii)
        {
            const char* name = prog_test_suite_name(ii);
            if (name)
            {
                names.emplace_back(name);
            }
        }
        return names;
    });

    m.def("debug_runtime_info", []() {
        py::dict libraries;
        libraries["prog_test"] = symbol_info(
            "prog_test_run_suite_by_name",
            reinterpret_cast<void*>(&prog_test_run_suite_by_name));
        libraries["smlib"] = symbol_info("smGet_DoGraphics", reinterpret_cast<void*>(&smGet_DoGraphics));

        py::dict out;
        out["compile_flags"] = compile_flags();
        out["libraries"] = libraries;
        out["do_graphics"] = smGet_DoGraphics() == TRUE;
        return out;
    });

    m.def("run_prog_test_suite",
          [](const std::string& name, const std::string& workingDirectory, bool doGraphics) {
              if (!prog_test_suite_exists(name.c_str()))
              {
                  throw py::value_error("Unknown prog_test suite: " + name);
              }

              ProgTestRunResult result {};
              // Reserved for captured stdout/stderr from the suite run. Capture is
              // not wired up yet, so the result dict's "log" key is currently always
              // empty; left in place so callers can rely on the key existing.
              std::string logText;
              DrawCaptureState drawCapture;
              {
                  std::lock_guard<std::mutex> progTestLock(g_progTestMutex);
                  ScopedWorkingDirectory cwd(workingDirectory);
                  ensure_prog_test_output_dir();
                  ScopedProgTestRunFlags runFlags(doGraphics);
                  ScopedDrawCapture captureDraw(drawCapture);
                  {
                      py::gil_scoped_release release;
                      prog_test_run_suite_by_name(name.c_str(), &result);
                  }
              }
              return result_to_dict(result, logText, drawCapture);
          },
          py::arg("name"),
          py::arg("working_directory") = "",
          py::arg("do_graphics") = false);
}
