
#include "AL/al.h"
#include "AL/alc.h"
#include "AL/alext.h"

#include "common/logging/log.h"

#include "core/audio/stream.h"

#include <array>
#include <queue>

namespace Audio {
    std::vector<u16> DecodeADPCM(u8* data, size_t sample_count, u16 adpcm_ps, s16 adpcm_yn[2], std::array<u8, 16> adpcm_coeff);

    static const int BASE_SAMPLE_RATE = 22050;

    struct Buffer {
        u16 id;
        ALuint buffer;
        bool is_looping;

        bool operator < (const Buffer& other) const {
            if ((other.id - id) > 1000) return true;
            if ((id - other.id) > 1000) return false;
            return id > other.id;
        }
    };

    struct OutputChannel {
        ALuint source;
        int mono_or_stereo;
        Format format;
        std::priority_queue<Buffer> queue;
        std::queue<Buffer> playing;
        u16 last_bufid;
    };

    OutputChannel chans[24];

    int InitAL(void)
    {
        ALCdevice *device;
        ALCcontext *ctx;

        /* Open and initialize a device with default settings */
        device = alcOpenDevice(NULL);
        if (!device)
        {
            LOG_CRITICAL(Audio, "Could not open a device!");
            return 1;
        }

        ctx = alcCreateContext(device, NULL);
        if (ctx == NULL || alcMakeContextCurrent(ctx) == ALC_FALSE)
        {
            if (ctx != NULL)
                alcDestroyContext(ctx);
            alcCloseDevice(device);
            LOG_CRITICAL(Audio, "Could not set a context!");
            return 1;
        }

        LOG_INFO(Audio, "Opened \"%s\"", alcGetString(device, ALC_DEVICE_SPECIFIER));
        return 0;
    }

    ALuint silencebuffer;
    ALCint dev_rate;
    std::array<u8, 10000> silence;

    void Init() {
        InitAL();

        {
            ALCdevice *device = alcGetContextsDevice(alcGetCurrentContext());
            alcGetIntegerv(device, ALC_FREQUENCY, 1, &dev_rate);
            if (alcGetError(device) != ALC_NO_ERROR) LOG_CRITICAL(Audio, "Failed to get device sample rate");
            LOG_INFO(Audio, "Device Frequency: %i", dev_rate);
        }

        for (int i = 0; i < 24; i++) {
            alGenSources(1, &chans[i].source);
            if (alGetError() != AL_NO_ERROR) LOG_CRITICAL(Audio, "Failed to setup sound source");
        }
    }

    void Shutdown() {}

    void UpdateFormat(int chanid, int mono_or_stereo, Format format) {
        chans[chanid].mono_or_stereo = mono_or_stereo;
        chans[chanid].format = format;

        LOG_WARNING(Audio, "(STUB)");
    }

    void EnqueueBuffer(int chanid, u16 buffer_id,
            void* data, int sample_count,
            bool has_adpcm, u16 adpcm_ps, s16 adpcm_yn[2],
            bool is_looping) {

        if (is_looping) {
            LOG_WARNING(Audio, "Looped buffers are unimplemented");
        }

        ALuint b;
        alGenBuffers(1, &b);

        if (chans[chanid].format == FORMAT_PCM16) {
            switch (chans[chanid].mono_or_stereo) {
            case 2:
                alBufferData(b, AL_FORMAT_STEREO16, data, sample_count * 4, BASE_SAMPLE_RATE);
                break;
            case 1:
            default:
                alBufferData(b, AL_FORMAT_MONO16, data, sample_count * 2, BASE_SAMPLE_RATE);
                break;
            }
            if (alGetError() != AL_NO_ERROR) LOG_CRITICAL(Audio, "Failed to init buffer");
        } else if (chans[chanid].format == FORMAT_PCM8) {
            switch (chans[chanid].mono_or_stereo) {
            case 2:
                alBufferData(b, AL_FORMAT_STEREO8, data, sample_count * 2, BASE_SAMPLE_RATE);
                break;
            case 1:
            default:
                alBufferData(b, AL_FORMAT_MONO8, data, sample_count * 1, BASE_SAMPLE_RATE);
                break;
            }
            if (alGetError() != AL_NO_ERROR) LOG_CRITICAL(Audio, "Failed to init buffer");
        } /*else if (chans[chanid].format == FORMAT_ADPCM) {
            if (chans[chanid].mono_or_stereo != 1) {
                LOG_ERROR(Audio, "Being fed non-mono ADPCM");
            }
            std::vector<u16> decoded = DecodeADPCM(data, sample_count, adpcm_ps, adpcm_yn, chans[chanid].adpcm_coeff);
            alBufferData(b, AL_FORMAT_MONO16, decoded.data(), decoded.size() * 2, BASE_SAMPLE_RATE);
            if (alGetError() != AL_NO_ERROR) LOG_CRITICAL(Audio, "Failed to init buffer");
        }*/ else {
            LOG_ERROR(Audio, "Unrecognised audio format in buffer 0x%04x (size: %i samples)", buffer_id, sample_count);
            alBufferData(b, AL_FORMAT_MONO8, silence.data(), silence.size(), BASE_SAMPLE_RATE);
            if (alGetError() != AL_NO_ERROR) LOG_CRITICAL(Audio, "Failed to init buffer");
        }

        chans[chanid].queue.emplace( Buffer { buffer_id, b, is_looping });
    }

    void Tick(int chanid) {
        auto& c = chans[chanid];

        if (!c.queue.empty()) {
            while (!c.queue.empty()) {
                alSourceQueueBuffers(c.source, 1, &c.queue.top().buffer);
                if (alGetError() != AL_NO_ERROR) LOG_CRITICAL(Audio, "Failed to enqueue buffer");
                c.playing.emplace(c.queue.top());
                LOG_INFO(Audio, "Enqueued buffer id 0x%04x", c.queue.top().id);
                c.queue.pop();
            }

            ALint state;
            alGetSourcei(c.source, AL_SOURCE_STATE, &state);
            if (state != AL_PLAYING) {
                alSourcePlay(c.source);
            }
        }

        if (!c.playing.empty()) {
            c.last_bufid = c.playing.front().id;
        }

        ALint processed;
        alGetSourcei(c.source, AL_BUFFERS_PROCESSED, &processed);
        while (processed > 0) {
            ALuint buf;
            alSourceUnqueueBuffers(c.source, 1, &buf);
            processed--;

            LOG_INFO(Audio, "Finished buffer id 0x%04x", c.playing.front().id);

            while (!c.playing.empty() && c.playing.front().buffer != buf) {
                c.playing.pop();
                LOG_ERROR(Audio, "Audio is extremely funky. Should abort. (Desynced queue.)");
            }

            if (!c.playing.empty()) {
                c.last_bufid = c.playing.front().id;
                c.playing.pop();
            } else {
                LOG_ERROR(Audio, "Audio is extremely funky. Should abort. (Empty queue.)");
            }

            alDeleteBuffers(1, &buf);
        }

        if (!c.playing.empty()) {
            c.last_bufid = c.playing.front().id;
        }
    }

    std::tuple<bool, u16, u32> GetStatus(int chanid) {
        auto& c = chans[chanid];

        bool isplaying = false;
        u16 bufid = 0;
        u32 pos = 0;

        ALint state, samples;
        alGetSourcei(c.source, AL_SOURCE_STATE, &state);
        alGetSourcei(c.source, AL_SAMPLE_OFFSET, &samples);

        if (state == AL_PLAYING) isplaying = true;

        bufid = c.last_bufid;

        pos = samples;

        return std::make_tuple(isplaying, bufid, pos);
    }

};