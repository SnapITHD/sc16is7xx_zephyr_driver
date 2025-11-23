# SPDX-License-Identifier: Apache-2.0

#include_directories(.)
target_include_directories(app PRIVATE .)
target_sources(app PRIVATE sc16is7xx.c)