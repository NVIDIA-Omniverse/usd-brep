<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

<a name="context--concurrency"></a>

# Context and Concurrency

**Source:** `SmApiGeneral.h` (`SmApiCreateContext`, `SmApiGetOrCreateContext`), `source/SMLib/inc/SmContext.h`, `source/SMLib/inc/SmConfig.h`, `source/SMLib/inc/SmCacheMgr.h`

## Rules

- **The concurrency unit is the process.** Run one worker process per concurrent job. Do **not** call `SmApi*` / `_omni_solid` operations concurrently from multiple threads in one process. A shared-memory WASM worker counts as a thread (see [WASM workers](#wasm-workers)).
  - **Exception:** `SmApiUsdTessellateBrepArray` builds its Breps in a context private to the call and never uses the default context, so it may be called from several threads at once. Do not modify the stage while calls are running.
  - **Exception:** `SmApiUsdHealFile` heals each BrepArray in its own context the same way, on worker threads of its own (`threads`); it never uses the default context.
- **Operations run single-threaded** in the shipped build; scale throughput with more worker processes, not threads.
- **Keep one long-lived default `SmContext` per worker** across the edit loop. In the shipped build, caches belong to objects; keeping the context alive alone does not preserve caches when those objects are replaced.
- **Create the context once at startup** (`SmApiCreateContext()`), before spawning threads.

## Why in-process threading is unsafe

The `SM_API` layer provides one process-global default `SmContext`
(`SmApiGetOrCreateContext()`). Many entry points use it for allocation and traversal.
It holds **unlocked mutable shared state**, including traversal mark counters
(`m_lCurrentMark*`), so concurrent calls sharing it can race and corrupt traversal
state. Selected operations use an input object's context or a dedicated output
context, but that does not establish thread safety for the API as a whole. Use
separate worker processes for concurrent jobs rather than relying on explicit
contexts to isolate arbitrary `SmApi*` calls.

## Operations are single-threaded

Individual operations do not use internal thread parallelism in the shipped build:
a single call (tessellation included) runs on one thread. Some parallel code paths
exist in the source but are gated behind build options that are not enabled, so
they are compiled out. Scale throughput by running more worker processes, each on
its own context.

## Cache lifetime

In the shipped build, `SM_USE_GLOBAL_CACHE` is disabled. Object caches are stored
on individual `SmAObject` instances; their lifetime and reuse depend on those
objects and cache invalidation. Keeping the context alive alone does not retain
caches for objects that are deleted or replaced, and edits may invalidate caches
even when an object survives. The same distinction applies in a browser/WASM worker.

When the optional `SM_USE_GLOBAL_CACHE` configuration is enabled, the `SmContext`
instead owns a `SmGlobalCache` containing curve, surface, trimmed-surface, and BRep
cache queues. That configuration exposes cache-size parameters on the context
constructor. Destroying the context discards those queues; a long-lived context
retains them subject to cache invalidation and eviction. This optional shared cache
also has unlocked mutable state and must not be treated as thread-safe.

## Startup

`SmApiGetOrCreateContext()` lazily creates the global context and is **not
synchronized**, so call `SmApiCreateContext()` once from the main thread at startup
(`_omni_solid` does this on import).

## WASM workers

A browser/WASM worker counts as a separate worker only when it runs its own
independently instantiated module with its own, unshared `WebAssembly.Memory`. It
then has its own global context, and the one-context-per-worker rule applies.
Emscripten pthreads and shared-memory Wasm Workers share the module's linear
memory, and with it ordinary globals such as the default context. Treat them as
threads: do not make concurrent `SmApi*` calls from them.

## See Also

- **[tessellation](tessellation.md)** — the heaviest single operation per worker.
