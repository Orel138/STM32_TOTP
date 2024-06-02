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

#include "totp.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#if USE_MBEDTLS
#include "mbedtls/sha1.h"
#else
#include "sha1.h"
#endif

#if DEBUG_LOGS
#include "logging_levels.h"
#define LOG_LEVEL LOG_DEBUG
#include "logging.h"
#include "cli.h"
#endif

static const uint8_t *totp_key;
static size_t totp_key_length;
static uint32_t totp_time_step;

void totp_init(const uint8_t *key, size_t key_length, uint32_t time_step) {
    totp_key = key;
    totp_key_length = key_length;
    totp_time_step = time_step;
}

uint32_t generate_totp(uint64_t time) {
    uint64_t time_counter = time / totp_time_step;
    uint8_t time_bytes[8];
    for (int i = 7; i >= 0; --i) {
        time_bytes[i] = time_counter & 0xFF;
        time_counter >>= 8;
    }

#if DEBUG_LOGS
    LogInfo("Time bytes: ");
    for (int i = 0; i < 8; i++) {
        LogInfo("%02x ", time_bytes[i]);
    }
    LogInfo("\n");
#endif

    uint8_t hash[20]; // SHA1 produces a 20-byte hash

#if USE_MBEDTLS
    mbedtls_sha1_context ctx;
    mbedtls_sha1_init(&ctx);
    mbedtls_sha1_starts(&ctx);
    mbedtls_sha1_update(&ctx, totp_key, totp_key_length);
    mbedtls_sha1_update(&ctx, time_bytes, sizeof(time_bytes));
    mbedtls_sha1_finish(&ctx, hash);
    mbedtls_sha1_free(&ctx);
#else
    SHA1_CTX ctx;
    SHA1_Init(&ctx);
    SHA1_Update(&ctx, totp_key, totp_key_length);
    SHA1_Update(&ctx, time_bytes, sizeof(time_bytes));
    SHA1_Final(hash, &ctx);
#endif

#if DEBUG_LOGS
    LogInfo("HMAC-SHA1 hash: ");
    for (int i = 0; i < 20; i++) {
        LogInfo("%02x ", hash[i]);
    }
    LogInfo("\n");
#endif

    int offset = hash[19] & 0x0F;
#if DEBUG_LOGS
    LogInfo("Offset: %d\n", offset);
#endif

    uint32_t binary_code = (hash[offset] & 0x7F) << 24 |
                           (hash[offset + 1] & 0xFF) << 16 |
                           (hash[offset + 2] & 0xFF) << 8 |
                           (hash[offset + 3] & 0xFF);

#if DEBUG_LOGS
    LogInfo("Binary code: %u\n", binary_code);
#endif

    uint32_t totp = binary_code % (uint32_t) pow(10, TOTP_DIGITS);
    return totp;
}

