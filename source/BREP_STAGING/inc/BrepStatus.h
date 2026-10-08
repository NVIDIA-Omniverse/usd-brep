// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef BREP_STATUS_H
#define BREP_STATUS_H

enum BrepStatus
{
    BrepStatusSuccess = 0,
    BrepStatusError = 1
};

inline bool IsBrepStatusSuccess(BrepStatus status)
{
    return status == BrepStatusSuccess;
}

#endif // BREP_STATUS_H
