#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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

"""Create the pyz installer and bundle the correct files (pyproject.toml etc)"""

from __future__ import annotations

import asyncio

from support.build_help import (
    OUT_PATH,
    call_build_pyz,
    copy_app_sources,
    install_dependencies_into_pyz,
    reset_build_dir,
)


async def main() -> None:
    await reset_build_dir()

    # Run copy and pip install concurrently to speed up the build.
    await asyncio.gather(
        copy_app_sources(),
        install_dependencies_into_pyz(),
    )

    await call_build_pyz()
    print(f"Built: {OUT_PATH}")
    print(f"Run:   python3 {OUT_PATH}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
