# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Open3D wrapper that suppresses memory statistics warnings at exit.

Open3D's MemoryManagerStatistic prints debug output during Python shutdown.
The C++ SetPrintAtProgramEnd() method isn't exposed to Python, so we suppress
stderr at exit when this module is imported.

Usage:
    from dimos.utils import o3d
    # Use o3d.geometry, o3d.core, etc. as normal
"""

import atexit
import os
import sys

import open3d as _o3d
from open3d import *  # noqa: F403 - re-export everything

# Suppress info/warning messages during normal operation
_o3d.utility.set_verbosity_level(_o3d.utility.VerbosityLevel.Error)


# Suppress memory statistics printed at program exit
def _suppress_open3d_exit_spam() -> None:
    sys.stderr = open(os.devnull, "w")


atexit.register(_suppress_open3d_exit_spam)
