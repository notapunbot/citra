#pragma once

#include "AL/al.h"
#include "AL/alc.h"
#include "AL/alext.h"

#include "common/common_types.h"

#include <tuple>

namespace Audio {

void Init();
void Shutdown();

enum Format : u16 {
    FORMAT_PCM8 = 0,
    FORMAT_PCM16 = 1,
    FORMAT_ADPCM = 2
};

void UpdateFormat(int chanid, int mono_or_stereo, Format format);
void UpdateAdpcm(int chanid, s16 coeffs[16]);

void Play(int chanid, bool play);

void EnqueueBuffer(int chanid, u16 buffer_id, void* data, int sample_count, bool is_looping);

void Tick(int chanid);

// Return values:
// <1>: is_enabled
// <2>: prev buffer_id
// <3>: current sample position in current buffer
std::tuple<bool, u16, u32> GetStatus(int chanid);

};