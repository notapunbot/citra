// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#pragma once

#include <string>

#include "common/common_types.h"

namespace VideoCore {

/// Structure for the TGA texture format (for dumping)
struct TGAHeader {
    char  idlength;
    char  colormaptype;
    char  datatypecode;
    short int colormaporigin;
    short int colormaplength;
    short int x_origin;
    short int y_origin;
    short width;
    short height;
    char  bitsperpixel;
    char  imagedescriptor;
};

/**
 * Dumps a texture to TGA
 * @param filename String filename to dump texture to
 * @param width Width of texture in pixels
 * @param height Height of texture in pixels
 * @param raw_data Raw RGBA8 texture data to dump
 * @todo This should be moved to some general purpose/common code
 */
void DumpTGA(std::string filename, short width, short height, u8* raw_data);

/// Lookup table for the offsets used to convert an image to Morton order.
static const u8 morton_lut[64] = {
     0,  1,  4,  5, 16, 17, 20, 21,
     2,  3,  6,  7, 18, 19, 22, 23,
     8,  9, 12, 13, 24, 25, 28, 29,
    10, 11, 14, 15, 26, 27, 30, 31,
    32, 33, 36, 37, 48, 49, 52, 53,
    34, 35, 38, 39, 50, 51, 54, 55,
    40, 41, 44, 45, 56, 57, 60, 61,
    42, 43, 46, 47, 58, 59, 62, 63,
};

/**
 * Lookup the intra-block offset for the specified coordinates in the Morton order (Z-order) lookup table.
 * @param x X coordinate, must be [0, 7]
 * @param y Y coordinate, must be [0, 7]
 */
static inline u32 MortonInterleave(u32 x, u32 y) {
    return morton_lut[y * 8 + x];
}

/**
 * Copies the texture data from the source address to the destination address, applying a
 * Morton-order transformation while copying.
 *
 * @param T Byte size of a pixel in the source and destination pointers.
 * @param src Pointer to the source texture data.
 * @param dst Pointer to which the texture will be copied.
 * @param width Width of the texture, should be a multiple of 8.
 * @param height Height of the texture, should be a multiple of 8.
 */
template<int size>
void CopyTextureAndTile(const u8* src, u8* dst, unsigned int width, unsigned int height);

/**
 * Copies texture data while undoing the transformation applied by `CopyTextureAndTile`.
 *
 * @param T Byte size of a pixel in the source and destination pointers.
 * @param src Pointer to the source texture data.
 * @param dst Pointer to which the texture will be copied.
 * @param width Width of the texture, should be a multiple of 8.
 * @param height Height of the texture, should be a multiple of 8.
 */
template<int size>
void CopyTextureAndUntile(const u8* src, u8* dst, unsigned int width, unsigned int height);

/**
 * Calculates the offset of the position of the pixel in Morton order
 */
static inline u32 GetMortonOffset(u32 x, u32 y, u32 bytes_per_pixel) {
    // Images are split into 8x8 tiles. Each tile is composed of four 4x4 subtiles each
    // of which is composed of four 2x2 subtiles each of which is composed of four texels.
    // Each structure is embedded into the next-bigger one in a diagonal pattern, e.g.
    // texels are laid out in a 2x2 subtile like this:
    // 2 3
    // 0 1
    //
    // The full 8x8 tile has the texels arranged like this:
    //
    // 42 43 46 47 58 59 62 63
    // 40 41 44 45 56 57 60 61
    // 34 35 38 39 50 51 54 55
    // 32 33 36 37 48 49 52 53
    // 10 11 14 15 26 27 30 31
    // 08 09 12 13 24 25 28 29
    // 02 03 06 07 18 19 22 23
    // 00 01 04 05 16 17 20 21
    //
    // This pattern is what's called Z-order curve, or Morton order.

    const unsigned int block_height = 8;
    const unsigned int coarse_x = x & ~7;

    u32 i = VideoCore::MortonInterleave(x & 7, y & 7);

    const unsigned int offset = coarse_x * block_height;

    return (i + offset) * bytes_per_pixel;
}

} // namespace
