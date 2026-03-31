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

import pickle
from typing import TypeVar

from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.memory2.store.sqlite import SqliteStore

# from dimos.memory2.transform import normalize, smooth, speed
from dimos.memory2.vis.drawing2d.drawing2d import Drawing2D
from dimos.models.embedding.clip import CLIPModel
from dimos.utils.data import get_data

T = TypeVar("T")

store = SqliteStore(path=get_data("go2_bigoffice.db"))
global_map = pickle.loads(get_data("unitree_go2_bigoffice_map.pickle").read_bytes())
costmap = simple_inflate(general_occupancy(global_map), 0.05)

clip = CLIPModel()

embedded = store.streams.color_image_embedded

drawing = Drawing2D()
drawing.add(costmap)

search_vector = clip.embed_text("bottle")


embedded.search(search_vector, k=10).tap(drawing.add).drain()

drawing.to_rerun()
