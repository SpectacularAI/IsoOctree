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