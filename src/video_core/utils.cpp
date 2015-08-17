// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include <cstdio>
#include <cstring>

#include "video_core/utils.h"

namespace VideoCore {

/**
 * Dumps a texture to TGA
 * @param filename String filename to dump texture to
 * @param width Width of texture in pixels
 * @param height Height of texture in pixels
 * @param raw_data Raw RGBA8 texture data to dump
 * @todo This should be moved to some general purpose/common code
 */
void DumpTGA(std::string filename, short width, short height, u8* raw_data) {
    TGAHeader hdr = {0, 0, 2, 0, 0, 0, 0, width, height, 24, 0};
    FILE* fout = fopen(filename.c_str(), "wb");

    fwrite(&hdr, sizeof(TGAHeader), 1, fout);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            putc(raw_data[(3 * (y * width)) + (3 * x) + 0], fout); // b
            putc(raw_data[(3 * (y * width)) + (3 * x) + 1], fout); // g
            putc(raw_data[(3 * (y * width)) + (3 * x) + 2], fout); // r
        }
    }

    fclose(fout);
}

template<typename T>
void CopyTextureAndTile(const T* src, T* dst, unsigned int width, unsigned int height) {
    for (unsigned int y = 0; y + 8 <= height; y += 8) {
        for (unsigned int x = 0; x + 8 <= width; x += 8) {
            const T* line = &src[y * width + x];

            for (unsigned int yy = 0; yy < 8; ++yy) {
                for (unsigned int xx = 0; xx < 8; ++xx) {
                    dst[morton_lut[yy * 8 + xx]] = line[xx];
                }
                line += width;
            }

            dst += 8 * 8;
        }
    }
}

template void CopyTextureAndTile<u16>(const u16* src, u16* dst, unsigned int width, unsigned int height);
template void CopyTextureAndTile<u24_be>(const u24_be* src, u24_be* dst, unsigned int width, unsigned int height);
template void CopyTextureAndTile<u32>(const u32* src, u32* dst, unsigned int width, unsigned int height);

template<typename T>
void CopyTextureAndUntile(const T* src, T* dst, unsigned int width, unsigned int height) {
    for (unsigned int y = 0; y + 8 <= height; y += 8) {
        for (unsigned int x = 0; x + 8 <= width; x += 8) {
            T* line = &dst[y * width + x];

            for (unsigned int yy = 0; yy < 8; ++yy) {
                for (unsigned int xx = 0; xx < 8; ++xx) {
                    line[xx] = src[morton_lut[yy * 8 + xx]];
                }
                line += width;
            }

            src += 8 * 8;
        }
    }
}

template void CopyTextureAndUntile<u16>(const u16* src, u16* dst, unsigned int width, unsigned int height);
template void CopyTextureAndUntile<u24_be>(const u24_be* src, u24_be* dst, unsigned int width, unsigned int height);
template void CopyTextureAndUntile<u32>(const u32* src, u32* dst, unsigned int width, unsigned int height);
} // namespace
