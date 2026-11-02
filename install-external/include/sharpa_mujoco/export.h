// Copyright 2026 Sharpa
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(SHARPA_MUJOCO_STATIC)
#define SHARPA_MUJOCO_API
#else
#define SHARPA_MUJOCO_API __attribute__((visibility("default")))
#endif
