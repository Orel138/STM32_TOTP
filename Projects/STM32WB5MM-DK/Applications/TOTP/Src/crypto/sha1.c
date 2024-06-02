/*
 * Copyright (c) 2024 Orel138
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "sha1.h"
#include <string.h>

// SHA1 transformation constants
#define SHA1_BLOCK_SIZE 64

static void SHA1_Transform(uint32_t state[5], const uint8_t buffer[64]);

// Rotate left operation
#define ROL(value, bits) (((value) << (bits)) | ((value) >> (32 - (bits))))

// SHA1 basic functions
#define F0(b, c, d) (((b) & (c)) | ((~(b)) & (d)))
#define F1(b, c, d) ((b) ^ (c) ^ (d))
#define F2(b, c, d) (((b) & (c)) | ((b) & (d)) | ((c) & (d)))

// SHA1 initialization
void SHA1_Init(SHA1_CTX *context) {
    context->state[0] = 0x67452301;
    context->state[1] = 0xEFCDAB89;
    context->state[2] = 0x98BADCFE;
    context->state[3] = 0x10325476;
    context->state[4] = 0xC3D2E1F0;
    context->count[0] = context->count[1] = 0;
}

// SHA1 update with data
void SHA1_Update(SHA1_CTX *context, const uint8_t *data, uint32_t len) {
    uint32_t i, j;

    j = (context->count[0] >> 3) & 63;
    if ((context->count[0] += len << 3) < (len << 3)) context->count[1]++;
    context->count[1] += (len >> 29);

    if ((j + len) > 63) {
        memcpy(&context->buffer[j], data, (i = 64 - j));
        SHA1_Transform(context->state, context->buffer);
        for (; i + 63 < len; i += 64) {
            SHA1_Transform(context->state, &data[i]);
        }
        j = 0;
    } else i = 0;

    memcpy(&context->buffer[j], &data[i], len - i);
}

// SHA1 finalization
void SHA1_Final(uint8_t digest[20], SHA1_CTX *context) {
    uint8_t finalcount[8];
    uint8_t c;

    for (uint32_t i = 0; i < 8; i++) {
        finalcount[i] = (uint8_t)((context->count[(i >= 4 ? 0 : 1)] >>
                                   ((3 - (i & 3)) * 8)) & 255);
    }

    c = 0x80;
    SHA1_Update(context, &c, 1);
    while ((context->count[0] & 504) != 448) {
        c = 0x00;
        SHA1_Update(context, &c, 1);
    }
    SHA1_Update(context, finalcount, 8);
    for (uint32_t i = 0; i < 20; i++) {
        digest[i] = (uint8_t)
                ((context->state[i >> 2] >> ((3 - (i & 3)) * 8)) & 255);
    }

    // Wipe variables for security reasons
    memset(context, 0, sizeof(*context));
    memset(finalcount, 0, sizeof(finalcount));
}

// SHA1 transformation
static void SHA1_Transform(uint32_t state[5], const uint8_t buffer[64]) {
    uint32_t a, b, c, d, e;
    uint32_t block[80];

    for (uint32_t i = 0; i < 16; i++) {
        block[i] = ((uint32_t) buffer[i * 4 + 3]) |
                   (((uint32_t) buffer[i * 4 + 2]) << 8) |
                   (((uint32_t) buffer[i * 4 + 1]) << 16) |
                   (((uint32_t) buffer[i * 4 + 0]) << 24);
    }
    for (uint32_t i = 16; i < 80; i++) {
        block[i] = ROL(block[i - 3] ^ block[i - 8] ^ block[i - 14] ^ block[i - 16], 1);
    }

    a = state[0];
    b = state[1];
    c = state[2];
    d = state[3];
    e = state[4];

    for (uint32_t i = 0; i < 20; i++) {
        uint32_t temp = ROL(a, 5) + F0(b, c, d) + e + block[i] + 0x5A827999;
        e = d;
        d = c;
        c = ROL(b, 30);
        b = a;
        a = temp;
    }

    for (uint32_t i = 20; i < 40; i++) {
        uint32_t temp = ROL(a, 5) + F1(b, c, d) + e + block[i] + 0x6ED9EBA1;
        e = d;
        d = c;
        c = ROL(b, 30);
        b = a;
        a = temp;
    }

    for (uint32_t i = 40; i < 60; i++) {
        uint32_t temp = ROL(a, 5) + F2(b, c, d) + e + block[i] + 0x8F1BBCDC;
        e = d;
        d = c;
        c = ROL(b, 30);
        b = a;
        a = temp;
    }

    for (uint32_t i = 60; i < 80; i++) {
        uint32_t temp = ROL(a, 5) + F1(b, c, d) + e + block[i] + 0xCA62C1D6;
        e = d;
        d = c;
        c = ROL(b, 30);
        b = a;
        a = temp;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;

    // Wipe variables for security reasons
    memset(block, 0, sizeof(block));
}
