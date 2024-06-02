/*
   Copyright 2022 Spectacular AI Ltd

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef ISO_OCTREE_LOGGING_INCLUDED
#define ISO_OCTREE_LOGGING_INCLUDED

#include <cstdio>

#ifndef NDEBUG
#define ISO_OCTREE_DEBUG
#endif

#ifdef ISO_OCTREE_DEBUG
#define log_debug(fmt, ...) ((void)std::fprintf(stdout, "IsoOctree DEBUG: " fmt "\n", ## __VA_ARGS__))
#else
#define log_debug(fmt, ...)
#endif
#define log_error(fmt, ...) ((void)std::fprintf(stderr, "IsoOctree ERROR: " fmt "\n", ## __VA_ARGS__))
#define log_warn(fmt, ...) ((void)std::fprintf(stderr, "IsoOctree WARNING: " fmt "\n", ## __VA_ARGS__))

#endif