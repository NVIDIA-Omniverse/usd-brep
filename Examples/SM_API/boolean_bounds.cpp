// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "boolean_bounds.h"

#include <SmApiBrep.h>
#include <SmApiGeneral.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmBrep.h> // Complete type for caller-owned Brep destruction.
#include <SmVector3d.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace
{
using Clock = std::chrono::steady_clock;
using BrepOwner = std::unique_ptr<SmBrep>;

constexpr int kRows = 4;
constexpr double kSpacing = 3.0;
constexpr double kSize = kRows * kSpacing;
constexpr double kHeight = 1.5;
constexpr double kRadius = 0.75;
constexpr double kBoundsTolerance = 1.0e-4; // Allows modeling-tolerance padding on this fixture.
constexpr double kVolumeTolerance = 1.0e-6; // Relative tolerance for this analytic fixture.

enum class BoundsMode
{
    None,
    Ordinary,
    Tight
};

void Require(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void Check(SmApiStatus status, const char* operation)
{
    if (status != SM_SUCCESS)
    {
        throw std::runtime_error(std::string(operation) + " returned " + std::to_string(status));
    }
}

double MillisecondsSince(Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

// Through-holes do not change the plate's outer bounds. Ordinary boxes only
// need to enclose them; tight boxes should fit within the fixture tolerance.
void CheckPlateBounds(const SmPoint3d& minimum, const SmPoint3d& maximum, bool tight)
{
    const double expectedMax[] = { kSize, kSize, kHeight };
    for (int axis = 0; axis != 3; ++axis)
    {
        Require(std::isfinite(minimum[axis]) && std::isfinite(maximum[axis]), "Non-finite bounds");
        Require(minimum[axis] <= maximum[axis], "Reversed bounds");
        Require(minimum[axis] <= kBoundsTolerance && maximum[axis] >= expectedMax[axis] - kBoundsTolerance, "Bounds do not enclose plate");
        if (tight)
        {
            Require(
                std::abs(minimum[axis]) <= kBoundsTolerance && std::abs(maximum[axis] - expectedMax[axis]) <= kBoundsTolerance,
                "Unexpected tight plate bounds"
            );
        }
    }
}

struct Snapshot
{
    double volume;
    ULONG faces;
    ULONG edges;
};

Snapshot CheckSolid(SmBrep* brep)
{
    SmBoolean manifold = FALSE;
    Check(SmApiBrepIsManifoldSolid(brep, manifold), "SmApiBrepIsManifoldSolid");
    Require(manifold == TRUE, "Plate is not a manifold solid");

    double volume = 0.0;
    Check(SmApiBrepComputeVolume(brep, 1.0e-7, volume), "SmApiBrepComputeVolume");
    const double expected = kSize * kSize * kHeight - kRows * kRows * std::acos(-1.0) * kRadius * kRadius * kHeight;
    Require(std::isfinite(volume) && std::abs(volume - expected) <= expected * kVolumeTolerance, "Unexpected plate volume");

    SmTArray<SmFace*> faces;
    SmTArray<SmEdge*> edges;
    Check(SmApiGetFaces(brep, faces), "SmApiGetFaces");
    Check(SmApiGetEdges(brep, edges), "SmApiGetEdges");
    return { volume, faces.GetSize(), edges.GetSize() };
}

void RunCase(BoundsMode mode, const char* label, std::ostream& output)
{
    SmBrep* rawPlate = nullptr;
    SmApiStatus status = SmApiCreateBox(SmVector3d(0, 0, 0), kSize, kSize, kHeight, rawPlate);
    BrepOwner plate(rawPlate);
    Check(status, "SmApiCreateBox");
    Require(plate != nullptr, "Null plate");

    double booleanMs = 0.0;
    double boundsMs = 0.0;
    int boundsCalls = 0;
    for (int row = 0; row != kRows; ++row)
    {
        for (int column = 0; column != kRows; ++column)
        {
            SmBrep* rawTool = nullptr;
            const SmVector3d origin((column + 0.5) * kSpacing, (row + 0.5) * kSpacing, -1.0);
            status = SmApiCreateCylinder(origin, kRadius, kHeight + 2.0, rawTool);
            BrepOwner tool(rawTool);
            Check(status, "SmApiCreateCylinder");
            Require(tool != nullptr, "Null cutter");

            SmBrep* result = nullptr;
            const auto booleanStart = Clock::now();
            status = SmApiBooleanDifference(plate.get(), tool.get(), result);
            booleanMs += MillisecondsSince(booleanStart);

            // On success the API deletes B and returns A in place. Release
            // our stale B pointer before any later check can throw. Do not
            // reset plate to result: it already owns that same object.
            // On failure both operands remain allocated (possibly modified),
            // so their owners clean up as Check throws; do not retry them.
            if (status == SM_SUCCESS)
            {
                tool.release();
            }
            Check(status, "SmApiBooleanDifference");
            Require(result == plate.get(), "Boolean result must alias the primary operand");

            // These queries are optional caller work, not a Boolean input or
            // prerequisite. Use ordinary bounds unless tighter bounds matter.
            if (mode != BoundsMode::None)
            {
                SmPoint3d minimum, maximum;
                const auto boundsStart = Clock::now();
                status = SmApiBrepBoundingBox(result, mode == BoundsMode::Tight ? TRUE : FALSE, minimum, maximum);
                boundsMs += MillisecondsSince(boundsStart);
                ++boundsCalls;
                Check(status, "SmApiBrepBoundingBox");
                CheckPlateBounds(minimum, maximum, mode == BoundsMode::Tight);
            }
        }
    }

    // Qualification is outside the timed calls. Also query both modes on the
    // SAME completed Brep and check its observables before/after. No fixed
    // topology counts: representation changes need not invalidate this demo.
    const Snapshot before = CheckSolid(plate.get());
    for (SmBoolean tight : { FALSE, TRUE })
    {
        SmPoint3d minimum, maximum;
        Check(SmApiBrepBoundingBox(plate.get(), tight, minimum, maximum), "SmApiBrepBoundingBox");
        CheckPlateBounds(minimum, maximum, tight == TRUE);
    }
    const Snapshot after = CheckSolid(plate.get());
    Require(
        before.faces == after.faces && before.edges == after.edges && std::abs(before.volume - after.volume) <= before.volume * kVolumeTolerance,
        "Bounds queries changed solid observables"
    );

    output << label << ": " << kRows * kRows << " differences, " << boundsCalls << " bounds calls; Boolean " << booleanMs << " ms, bounds "
           << boundsMs << " ms; volume " << after.volume << "; checks passed\n";
}
} // namespace

void RunBooleanBoundsExample(std::ostream& output)
{
    SmApiCreateContext();
    output << "4 x 4 through-holes in a 12 x 12 x 1.5 plate, radius 0.75.\n"
              "Timings exclude construction, qualification, and cleanup; they are not speed guarantees.\n";
    RunCase(BoundsMode::None, "no per-cut bounds", output);
    RunCase(BoundsMode::Ordinary, "ordinary bounds", output);
    RunCase(BoundsMode::Tight, "tight bounds", output);
}

#ifdef SM_BOOLEAN_BOUNDS_EXAMPLE_STANDALONE
int main()
{
    try
    {
        RunBooleanBoundsExample(std::cout);
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Boolean/bounds example failed: " << error.what() << '\n';
        return 1;
    }
}
#endif
