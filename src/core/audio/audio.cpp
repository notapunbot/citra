
#include "AL/al.h"
#include "AL/alc.h"
#include "AL/alext.h"

#include "common/logging/log.h"

#include "core/audio/audio.h"

#include <algorithm>
#include <array>
#include <queue>

namespace Audio {

static const int BASE_SAMPLE_RATE = 22050;

struct Buffer {
    u16 id; ///< buffer_id that userland gives us
    ALuint buffer;
    bool is_looping;

    bool operator < (const Buffer& other) const {
        // We want things with lower id to appear first, unless we have wraparound.
        // priority_queue puts a before b when b < a.
        // Should perhaps be a instead.
        if ((other.id - id) > 1000) return true;
        if ((id - other.id) > 1000) return false;
        return id > other.id;
    }
};

struct AdpcmState {
    // Two historical samples from previous processed buffer
    s16 yn1; ///< y[n-1]
    s16 yn2; ///< y[n-2]
};

// GC-ADPCM with scale factor and variable coefficients.
// Frames are 8 bytes long containing 14 samples each.
// Samples are 4 bits (one nybble) long.
std::vector<s16> DecodeADPCM(const u8 * const data, const size_t sample_count, const std::array<s16, 16>& adpcm_coeff, AdpcmState& state) {
    const size_t FRAME_LEN = 8;
    const size_t SAMPLES_PER_FRAME = 14;
    const static int SIGNED_NYBBLES[16] = { 0,1,2,3,4,5,6,7,-8,-7,-6,-5,-4,-3,-2,-1 };

    std::vector<s16> ret(sample_count);
    int yn1 = 0, yn2 = 0;// state.yn1, yn2 = state.yn2;

    const int NUM_FRAMES = (sample_count + (SAMPLES_PER_FRAME-1)) / SAMPLES_PER_FRAME; // Round up.
    for (int frameno = 0; frameno < NUM_FRAMES; frameno++) {
        int frame_header = data[frameno * FRAME_LEN];
        int scale = 1 << (frame_header & 0xF);
        int idx = (frame_header >> 4) & 0x7;

        // Coefficients are fixed point with 11 bits fractional part.
        int coef1 = adpcm_coeff[idx * 2 + 0];
        int coef2 = adpcm_coeff[idx * 2 + 1];

        auto process_nybble = [&](int nybble) -> s16 {
            int xn = nybble * scale;
            // We first transform everything into 11 bit fixed point, perform the second order digital filter, then transform back.
            // 0x400 == 0.5 in 11 bit fixed point.
            // Filter: y[n] = x[n] + 0.5 + c1 * y[n-1] + c2 * y[n-2]
            int val = ((xn << 11) + 0x400 + coef1 * yn1 + coef2 * yn2) >> 11;
            // Clamp to output range.
            if (val >= 32767) val = 32767;
            if (val <= -32768) val = -32768;
            // Advance output feedback.
            yn2 = yn1;
            yn1 = val;
            return (s16)val;
        };

        int outputi = frameno * SAMPLES_PER_FRAME;
        int datai = frameno * FRAME_LEN + 1;
        for (int i = 0; i < SAMPLES_PER_FRAME && outputi < sample_count; i += 2) {
            ret[outputi++] = process_nybble(SIGNED_NYBBLES[data[datai] & 0xF]);
            ret[outputi++] = process_nybble(SIGNED_NYBBLES[data[datai] >> 4]);
            datai++;
        }
    }

    state.yn1 = yn1;
    state.yn2 = yn2;

    return ret;
}

struct OutputChannel {
    ALuint source;                     ///< Each channel has it's own output, we lean on OpenAL to do our mixing.

    // Configuration
    int mono_or_stereo;                ///< Value from userland. 1 == mono, 2 == stereo, other == ???
    Format format;
    bool enabled;                      ///< Userland wants us to remind them we have enabled this channel.

    // Buffer management
    std::priority_queue<Buffer> queue; ///< Things we have gotten from userland we haven't queued onto `source` yet.
    std::queue<Buffer> playing;        ///< Things we have queued onto `source`.
    u16 last_bufid;                    ///< Userland wants us to report back what was the thing we last played.

    // For ADPCM decoding use.
    std::array<s16, 16> adpcm_coeffs;
    AdpcmState adpcm_state;
};

OutputChannel chans[24];

int InitAL() {
    ALCdevice *device = alcOpenDevice(nullptr);
    if (!device) {
        LOG_CRITICAL(Audio, "Could not open a device!");
        return 1;
    }

    ALCcontext *ctx = alcCreateContext(device, nullptr);
    if (ctx == nullptr || alcMakeContextCurrent(ctx) == ALC_FALSE) {
        if (ctx != nullptr) {
            alcDestroyContext(ctx);
        }
        alcCloseDevice(device);
        LOG_CRITICAL(Audio, "Could not set a context!");
        return 1;
    }

    LOG_INFO(Audio, "Audio output is on \"%s\"", alcGetString(device, ALC_DEVICE_SPECIFIER));
    return 0;
}

ALCint dev_rate;                ///< Native sample rate of our output device
std::array<u8, 10000> silence;  ///< Some silence, used if an audio error occurs

void Init() {
    InitAL();

    ALCdevice *device = alcGetContextsDevice(alcGetCurrentContext());
    alcGetIntegerv(device, ALC_FREQUENCY, 1, &dev_rate);
    if (alcGetError(device) != ALC_NO_ERROR) {
        LOG_CRITICAL(Audio, "Failed to get device sample rate");
    }
    LOG_INFO(Audio, "Device Frequency: %i", dev_rate);

    for (int i = 0; i < 24; i++) {
        alGenSources(1, &chans[i].source);
        if (alGetError() != AL_NO_ERROR) {
            LOG_CRITICAL(Audio, "Channel %i: Failed to setup sound source", i);
        }
    }

    silence.fill(0);
}

void Shutdown() {
    ALCcontext *ctx = alcGetCurrentContext();
    if (ctx == nullptr) {
        return;
    }

    ALCdevice* dev = alcGetContextsDevice(ctx);

    for (int i = 0; i < 24; i++) {
        alDeleteSources(1, &chans[i].source);
        while (!chans[i].queue.empty()) {
            alDeleteBuffers(1, &chans[i].queue.top().buffer);
            chans[i].queue.pop();
        }
        while (!chans[i].playing.empty()) {
            alDeleteBuffers(1, &chans[i].playing.front().buffer);
            chans[i].playing.pop();
        }
    }

    alcMakeContextCurrent(nullptr);
    alcDestroyContext(ctx);
    alcCloseDevice(dev);
}

void UpdateFormat(int chanid, int mono_or_stereo, Format format) {
    chans[chanid].mono_or_stereo = mono_or_stereo;
    chans[chanid].format = format;
}

void UpdateAdpcm(int chanid, s16 coeffs[16]) {
    LOG_DEBUG(Audio, "Channel %i: ADPCM Coeffs updated", chanid);
    std::copy(coeffs, coeffs+16, std::begin(chans[chanid].adpcm_coeffs));
}

void EnqueueBuffer(int chanid, u16 buffer_id, void* data, int sample_count, bool is_looping) {
    LOG_DEBUG(Audio, "Channel %i: Buffer %i: Enqueued (size %i)", chanid, buffer_id, sample_count);

    if (is_looping) {
        LOG_WARNING(Audio, "Channel %i: Buffer %i: Looped buffers are unimplemented", chanid, buffer_id);
    }

    ALuint b;
    alGenBuffers(1, &b);

    switch(chans[chanid].format) {
    case FORMAT_PCM16:
        switch (chans[chanid].mono_or_stereo) {
        case 2:
            alBufferData(b, AL_FORMAT_STEREO16, data, sample_count * 4, BASE_SAMPLE_RATE);
            break;
        case 1:
        default:
            alBufferData(b, AL_FORMAT_MONO16, data, sample_count * 2, BASE_SAMPLE_RATE);
            break;
        }

        if (alGetError() != AL_NO_ERROR) goto do_silence;

        break;
    case FORMAT_PCM8:
        switch (chans[chanid].mono_or_stereo) {
        case 2:
            alBufferData(b, AL_FORMAT_STEREO8, data, sample_count * 2, BASE_SAMPLE_RATE);
            break;
        case 1:
        default:
            alBufferData(b, AL_FORMAT_MONO8, data, sample_count * 1, BASE_SAMPLE_RATE);
            break;
        }

        if (alGetError() != AL_NO_ERROR) goto do_silence;

        break;
    case FORMAT_ADPCM: {
        if (chans[chanid].mono_or_stereo != 1) {
            LOG_ERROR(Audio, "Channel %i: Buffer %i: Being fed non-mono ADPCM (size: %i samples)", chanid, buffer_id, sample_count);
        }

        std::vector<s16> decoded = DecodeADPCM((u8*)data, sample_count, chans[chanid].adpcm_coeffs, chans[chanid].adpcm_state);
        alBufferData(b, AL_FORMAT_STEREO16, decoded.data(), decoded.size() * 2, BASE_SAMPLE_RATE);

        if (alGetError() != AL_NO_ERROR) goto do_silence;

        break;
    }
    default:
        LOG_ERROR(Audio, "Channel %i: Buffer %i: Unrecognised audio format (size: %i samples)", chanid, buffer_id, sample_count);
    do_silence:
        if (alGetError() != AL_NO_ERROR) {
            LOG_CRITICAL(Audio, "Channel %i: Buffer %i: OpenAL says \"%s\"", chanid, buffer_id, alGetString(alGetError()));
        }
        alBufferData(b, AL_FORMAT_MONO8, silence.data(), silence.size(), BASE_SAMPLE_RATE);
        if (alGetError() != AL_NO_ERROR) {
            LOG_CRITICAL(Audio, "Channel %i: Failed to init silence buffer!!! (%s)", chanid, alGetString(alGetError()));
        }
        break;
    }

    chans[chanid].queue.emplace( Buffer { buffer_id, b, is_looping });

    if (chans[chanid].queue.size() > 10) {
        LOG_ERROR(Audio, "We have far far too many buffers enqueued on channel %i (%i of them)", chanid, chans[chanid].queue.size());
    }
}

void Play(int chanid, bool play) {
    if (play) {
        LOG_INFO(Audio, "Channel %i: Enabled", chanid);
    } else {
        LOG_INFO(Audio, "Channel %i: Disabled", chanid);
    }
    chans[chanid].enabled = play;
}

void Tick(int chanid) {
    auto& c = chans[chanid];

    if (!c.queue.empty()) {
        while (!c.queue.empty()) {
            alSourceQueueBuffers(c.source, 1, &c.queue.top().buffer);
            if (alGetError() != AL_NO_ERROR) {
                alDeleteBuffers(1, &c.queue.top().buffer);
                LOG_CRITICAL(Audio, "Channel %i: Buffer %i: Failed to enqueue : %s", chanid, c.queue.top().id, alGetString(alGetError()));
                c.queue.pop();
                continue;
            }
            c.playing.emplace(c.queue.top());
            c.queue.pop();
        }
        if (c.enabled) {
            ALint state;
            alGetSourcei(c.source, AL_SOURCE_STATE, &state);
            if (state != AL_PLAYING) {
                alSourcePlay(c.source);
            }
        }
    }

    if (chans[chanid].playing.size() > 10) {
        LOG_ERROR(Audio, "Channel %i: We have far far too many buffers enqueued (%i of them)", chanid, chans[chanid].playing.size());
    }

    ALint processed;
    alGetSourcei(c.source, AL_BUFFERS_PROCESSED, &processed);
    while (processed > 0) {
        ALuint buf;
        alSourceUnqueueBuffers(c.source, 1, &buf);
        processed--;

        if (!c.playing.empty()) {
            if (c.playing.front().buffer != buf) {
                LOG_CRITICAL(Audio, "Channel %i: Play queue desynced with OpenAL queue. (buf???)", chanid);
            } else {
                LOG_DEBUG(Audio, "Channel %i: Buffer %i: Finished playing", chanid, c.playing.front().id);
            }
            c.last_bufid = c.playing.front().id;
            c.playing.pop();
        } else {
            LOG_CRITICAL(Audio, "Channel %i: Play queue desynced with OpenAL queue. (empty)", chanid);
        }

        alDeleteBuffers(1, &buf);
    }

    if (!c.playing.empty()) {
        c.last_bufid = c.playing.front().id;
    }
}

std::tuple<bool, u16, u32> GetStatus(int chanid) {
    auto& c = chans[chanid];

    bool isplaying = c.enabled;
    u16 bufid = c.last_bufid;
    u32 pos;

    ALint state, samples;
    alGetSourcei(c.source, AL_SOURCE_STATE, &state);
    alGetSourcei(c.source, AL_SAMPLE_OFFSET, &samples);
    pos = samples;

    return std::make_tuple(isplaying, bufid, pos);
}

};