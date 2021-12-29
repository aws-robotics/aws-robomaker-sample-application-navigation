#!/usr/bin/env bash

# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
COLCON_LOG_PATH="$@/logs" colcon bundle --base-paths "$@" --build-base "$@/build" --install-base "$@/install" --bundle-base "$@/bundle"
