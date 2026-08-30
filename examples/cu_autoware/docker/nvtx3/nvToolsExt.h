#pragma once

// LaME uses NVTX only for profiler annotations. The benchmark does not require CUDA,
// so these ABI-free no-ops keep the measurement build portable.
#include <stdint.h>

#define NVTX_VERSION 2
#define NVTX_EVENT_ATTRIB_STRUCT_SIZE sizeof(nvtxEventAttributes_t)
#define NVTX_COLOR_ARGB 1
#define NVTX_MESSAGE_TYPE_ASCII 1

typedef uint64_t nvtxRangeId_t;
typedef union { const char *ascii; } nvtxMessageValue_t;
typedef struct {
  uint16_t version;
  uint16_t size;
  uint32_t category;
  int32_t colorType;
  uint32_t color;
  int32_t payloadType;
  uint32_t reserved0;
  uint64_t payload;
  int32_t messageType;
  nvtxMessageValue_t message;
} nvtxEventAttributes_t;

static inline nvtxRangeId_t nvtxRangeStartA(const char *) { return 0; }
static inline nvtxRangeId_t nvtxRangeStartEx(const nvtxEventAttributes_t *) { return 0; }
static inline void nvtxRangeEnd(nvtxRangeId_t) {}
static inline int nvtxRangePushEx(const nvtxEventAttributes_t *) { return 0; }
static inline int nvtxRangePop(void) { return 0; }
static inline void nvtxNameOsThread(uint32_t, const char *) {}
