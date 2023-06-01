/*
 * Strider Compression Algorithm v0.4
 * Copyright (c) 2022 Carlos de Diego
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef __STRIDER__

#define __STRIDER__

#include <cstdint>
#include <stdio.h> //size_t

namespace strider {
	//Base class that allows to track progress of compression and decompression of
	// a Strider stream. You will have to create a child class which implements the functions.
	// - progress(): the algorithm will pass the number of bytes that have been compressed/decompressed, 
	//				 and its current compressed size. The function will return whether to stop encoding/decoding.
	class ProgressCallback {
	public:
		virtual bool progress(size_t processedBytes, size_t compressedSize) {
			return false;
		}
	};

	//Compresses "size" bytes of data present in "input", and stores it in "output".
	//"Level" specifies a tradeoff between compressed size and speed, and must be in range [0, 10].
	//You may pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns the size of the compressed stream or -1 on failure.
	size_t compress(const uint8_t* input, const size_t size, uint8_t* output, int level = 6,
		ProgressCallback* progress = nullptr);
	//Decompresses contents in "compressed" to "decompressed".
	//You may also pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns 0 on success or -1 on failure or corrupted data.
	int decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed,
		const size_t decompressedSize, ProgressCallback* progress = nullptr);

	//For a given input size, returns a size for the output buffer that is big enough to
	// contain the compressed stream even if it expands.
	size_t compress_bound(const size_t size);
	//Returns the amount of memory the algorithm will consume on compression.
	size_t estimate_memory(const size_t size, int level = 6);
}

#ifdef STRIDER_IMPLEMENTATION

#include <algorithm>
#include <cstring>
#include <thread>
#include <chrono>
#include <cmath>

#if defined(_MSC_VER)
  #define FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
  #define FORCE_INLINE inline __attribute__((always_inline))
#else
  #define FORCE_INLINE inline
#endif

#if defined(__GNUC__) || defined(__clang__)
  #define expect(expr,value)    (__builtin_expect ((expr),(value)) )
  #define likely(expr)     expect((expr) != 0, 1)
  #define unlikely(expr)   expect((expr) != 0, 0)
#else
  #define likely(expr)     (expr)
  #define unlikely(expr)   (expr)
#endif

//Probably not the correct way to do it but bleh
#define IS_64BIT (UINTPTR_MAX > UINT32_MAX)

#if defined(_MSC_VER)
  #if defined(_M_AMD64)
    #include <intrin.h>
	#define x64
  #elif defined(_M_IX86)
    #include <intrin.h>
	#define x86
  #endif
#elif defined(__GNUC__) || defined(__clang__)
  #if defined(__amd64__)
	#include <x86intrin.h>
	#define x64
  #elif defined(__i386__)
	#include <x86intrin.h>
	#define x86
  #endif
#endif

namespace strider {

#if defined(x64)
	#define HAS_SSE2 true
	#define COMPILE_WITH_SSE2
	//SSE2 detection for 32 bit x86 platforms
#elif defined(x86) && !((defined(__GNUC__) || defined(__clang__)) && !defined(__SSE2__))
	//Detection based on https://learn.microsoft.com/en-us/cpp/intrinsics/cpuid-cpuidex?view=msvc-170
	bool has_sse2() {
		// Calling __cpuid with 0x0 as the function_id argument
		// gets the number of the highest valid function ID.
		int cpuData[4];
		__cpuid(cpuData, 0);
		int nIds = cpuData[0];

		if (nIds < 1)
			return false;

		__cpuidex(cpuData, 1, 0);
		return (cpuData[3] >> 26) & 1;
	}
	const bool HAS_SSE2 = has_sse2();
	#define COMPILE_WITH_SSE2
#endif

	bool is_little_endian() {
		const union { uint16_t u; uint8_t c[2]; } LITTLE_ENDIAN_CHECK = { 1 };
		return LITTLE_ENDIAN_CHECK.c[0];
	}

//Undefined behaviour if value == 0
	FORCE_INLINE size_t unsafe_int_log2(size_t value) {
#if defined(_MSC_VER)
		unsigned long result;
	#if IS_64BIT
		_BitScanReverse64(&result, value);
	#else
		_BitScanReverse(&result, value);
	#endif
		return result;
#elif defined(__GNUC__) || defined(__clang__)
	#if IS_64BIT
		return 63 - __builtin_clzll(value);
	#else
		return 31 - __builtin_clz(value);
	#endif
#else
	#if IS_64BIT
		const uint8_t tab64[64] = {
			 0, 58,  1, 59, 47, 53,  2, 60,
			39, 48, 27, 54, 33, 42,  3, 61,
			51, 37, 40, 49, 18, 28, 20, 55,
			30, 34, 11, 43, 14, 22,  4, 62,
			57, 46, 52, 38, 26, 32, 41, 50,
			36, 17, 19, 29, 10, 13, 21, 56,
			45, 25, 31, 35, 16,  9, 12, 44,
			24, 15,  8, 23,  7,  6,  5, 63
		};
		value |= value >> 1;
		value |= value >> 2;
		value |= value >> 4;
		value |= value >> 8;
		value |= value >> 16;
		value |= value >> 32;
		return tab64[value * 0x03f6eaf2cd271461 >> 58];
	#else
		const uint8_t tab32[32] = {
			 0,  9,  1, 10, 13, 21,  2, 29,
			11, 14, 16, 18, 22, 25,  3, 30,
			 8, 12, 20, 28, 15, 17, 24,  7,
			19, 27, 23,  6, 26,  5,  4, 31
		};
		value |= value >> 1;
		value |= value >> 2;
		value |= value >> 4;
		value |= value >> 8;
		value |= value >> 16;
		return tab32[value * 0x07C4ACDDU >> 27];
	#endif
#endif
	}

	//Way faster than using log2(double), also returns 0 for a value of 0
	FORCE_INLINE size_t int_log2(const size_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_int_log2(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_int_log2(value);
	}

	//Undefined behaviour when value == 0
	FORCE_INLINE size_t unsafe_bit_scan_forward(const size_t value) {
#if defined(_MSC_VER)
		unsigned long result;
	#if IS_64BIT
		_BitScanForward64(&result, value);
	#else
		_BitScanForward(&result, value);
	#endif
		return result;
#elif defined(__GNUC__) || defined(__clang__)
	#if IS_64BIT
		return __builtin_ctzll(value);
	#else
		return __builtin_ctz(value);
	#endif
#else
	#if IS_64BIT
		const unsigned int tab64[64] = {
			 0,  1,  2, 53,  3,  7, 54, 27,
			 4, 38, 41,  8, 34, 55, 48, 28,
			62,  5, 39, 46, 44, 42, 22,  9,
			24, 35, 59, 56, 49, 18, 29, 11,
			63, 52,  6, 26, 37, 40, 33, 47,
			61, 45, 43, 21, 23, 58, 17, 10,
			51, 25, 36, 32, 60, 20, 57, 16,
			50, 31, 19, 15, 30, 14, 13, 12
		};
		return tab64[(value & (0 - value)) * 0x022FDD63CC95386D >> 58];
	#else
		static uint8_t tab32[32] = {
			 0,  1, 28,  2, 29, 14, 24,  3,
			30, 22, 20, 15, 25, 17,  4,  8,
			31, 27, 13, 23, 21, 19, 16,  7,
			26, 12, 18,  6, 11,  5, 10,  9
		};
		return tab32[(value & (0 - value)) * 0x077CB531U >> 27];
	#endif
#endif
	}

	//Returns the index of the first set bit, starting from the least significant, or 0 if the input is null
	FORCE_INLINE size_t bit_scan_forward(const size_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_bit_scan_forward(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_bit_scan_forward(value);
	}

	struct FastIntHash {
		//Use top bits
		size_t operator()(const size_t value) {
#if IS_64BIT
			return value * 0xff51afd7ed558ccd;
#else
			return value * 0x27d4eb2d;
#endif
		}
	};

	//These functions are used to obtain the data for the hash. If the number of bytes is higher than the word size,
	//they will be mixed. Also note that these might read more bytes than necessary.
	FORCE_INLINE size_t read_hash4(const uint8_t* const ptr) {
		uint32_t value;
		memcpy(&value, ptr, 4);
		return value;
	}
	FORCE_INLINE size_t read_hash8(const uint8_t* const ptr) {
#if IS_64BIT
		uint64_t value;
		memcpy(&value, ptr, 8);
#else
		uint32_t value = read_hash4(ptr) ^ read_hash4(ptr + 4);
#endif
		return value;
	}
	FORCE_INLINE size_t read_hash16(const uint8_t* const ptr) {
#if IS_64BIT
		return read_hash8(ptr) ^ read_hash8(ptr + 8);
#else
		return read_hash4(ptr) ^ read_hash4(ptr + 4) ^ read_hash4(ptr + 8) ^ read_hash4(ptr + 12);
#endif
	}
	FORCE_INLINE size_t read_hash6(const uint8_t* const ptr) {
#if IS_64BIT
		if (is_little_endian())
			return read_hash8(ptr) << 16;
		return read_hash8(ptr) >> 16;
#else
		uint16_t b;
		memcpy(&b, ptr + 4, 2);
		return read_hash4(ptr) ^ b;
#endif
	}
	FORCE_INLINE size_t read_hash3(const uint8_t* const ptr) {

		if (is_little_endian())
#if IS_64BIT
			return read_hash4(ptr) << 40;
#else
			return read_hash4(ptr) << 8;
#endif
		return read_hash4(ptr) >> 8;
	}
	FORCE_INLINE size_t read_hash2(const uint8_t* const ptr) {
		uint16_t value;
		memcpy(&value, ptr, 2);
		return value;
	}

	FORCE_INLINE uint32_t read_uint32le(const uint8_t* const ptr) {
		if (is_little_endian()) {
			uint32_t value;
			memcpy(&value, ptr, 4);
			return value;
		}
		uint32_t value = 0;
		for (int i = 0; i < 4; i++)
			value |= (uint32_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void write_uint32le(uint8_t* const ptr, const uint32_t value) {
		if (is_little_endian())
			memcpy(ptr, &value, 4);
		else {
			for (int i = 0; i < 4; i++)
				ptr[i] = value >> i * 8;
		}
	}
	FORCE_INLINE uint16_t read_uint16le(const uint8_t* const ptr) {
		uint16_t value;
		if (is_little_endian())
			memcpy(&value, ptr, 2);
		else
			value = ptr[0] | (ptr[1] << 8);
		return value;
	}
	FORCE_INLINE void write_uint16le(uint8_t* const ptr, const uint16_t value) {
		if (is_little_endian())
			memcpy(ptr, &value, 2);
		else {
			for (int i = 0; i < 2; i++)
				ptr[i] = value >> i * 8;
		}
	}

	enum {
		LAZY1,
		LAZY2,
		LAZY3,
		OPTIMAL1,
		OPTIMAL2,
		OPTIMAL3,
		OPTIMAL4,
	};

	struct CompressorOptions {
		int parser;
		int maxHashLog;
		int maxElementsLog;
		int niceLength;
		int optimalBlockSize;        // (Optimal)
		int maxArrivals;             // (Optimal)
	};

	//Tries to find a match between the two locations, and returns the length
	//Note that this function should be called with at least MIN_LENGTH + 8 bytes of buffer after limit
	FORCE_INLINE size_t test_match(const uint8_t* front, const uint8_t* back,
		const uint8_t* const limit, const size_t minLength)
	{
		//Test first bytes
		//Usually compilers will optimize std::equal as 2 comparisons for minLength 5, 6, etc
		//We can only use one comparison to make it faster. For powers of 2, std::equal should be good
		if (minLength == 3) {
			uint32_t a, b;
			memcpy(&a, front, sizeof(uint32_t));
			memcpy(&b, back, sizeof(uint32_t));
			if ((is_little_endian() && (a << 8) != (b << 8)) || (!is_little_endian() && (a >> 8) != (b >> 8)))
				return 0;
		}
#if IS_64BIT
		else if (minLength == 5) {
			uint64_t a, b;
			memcpy(&a, front, sizeof(uint64_t));
			memcpy(&b, back, sizeof(uint64_t));
			if ((is_little_endian() && (a << 24) != (b << 24)) || (!is_little_endian() && (a >> 24) != (b >> 24)))
				return 0;
		}
		else if (minLength == 6) {
			uint64_t a, b;
			memcpy(&a, front, sizeof(uint64_t));
			memcpy(&b, back, sizeof(uint64_t));
			if ((is_little_endian() && (a << 16) != (b << 16)) || (!is_little_endian() && (a >> 16) != (b >> 16)))
				return 0;
		}
#endif
		else {
			if (!std::equal(back, back + minLength, front))
				return 0;
		}

		const uint8_t* const matchOrigin = front;
		front += minLength;
		back += minLength;

		while (true) {
			if (unlikely(front + sizeof(size_t) > limit)) {
				if (front > limit)
					return 0;

				while (*front == *back && front < limit) {
					front++;
					back++;
				}
				return front - matchOrigin;
			}

			//Compare 4 or 8 bytes at a time using xor. It has the property of returning 0 if the two values are equal.
			//In case they differ, we can get the first byte that differs using a bit scan.
			size_t a, b;
			memcpy(&a, front, sizeof(size_t));
			memcpy(&b, back, sizeof(size_t));
			const size_t xorVal = a ^ b;

			if (xorVal) {
				if (is_little_endian())
					front += unsafe_bit_scan_forward(xorVal) >> 3;
				else
					front += unsafe_int_log2(xorVal) >> 3;
				return front - matchOrigin;
			}

			front += sizeof(size_t);
			back += sizeof(size_t);
		}
	}

	//A hash table which does not check for collisions
	template<class Value, class Hash>
	class HashTable {
		Value* arr = nullptr;
		int hashShift;
	public:
		//Use 2^x sizes to avoid the use of modulo operator
		HashTable() {}
		~HashTable() {
			delete[] arr;
		}
		void init(const int logSize) {
			arr = new Value[(size_t)1 << logSize]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
		}
		Value& operator[](const size_t value) {
			return arr[Hash{}(value) >> hashShift];
		}
	};

	template<class Value>
	class LZ2WayCacheBucket {
		Value* data;
	public:
		LZ2WayCacheBucket() {}
		LZ2WayCacheBucket(Value* _data) {
			data = _data;
		}
		//Loads the first value, and at the same time pushes a value into that slot.
		void first(Value* value) {
			const Value tmp = data[0];
			data[0] = *value;
			*value = tmp;
		}
		//Loads the second value, and at the same time pushes a value into that slot.
		//Should be used after loading the first value.
		void second(Value* value) {
			const Value tmp = data[1];
			data[1] = *value;
			*value = tmp;
		}
		//Inserts a value into the first slot.
		//Used when skipping bytes.
		void push_in_first(const Value value) {
			const Value tmp = data[0];
			data[0] = value;
			data[1] = tmp;
		}
		//Inserts a value into the second slot.
		//Used when we have checked the first entry, but we wont check the second.
		void push_in_second(const size_t value) {
			data[1] = value;
		}
	};

	//Like a hash table, but stores 2 elements per entry
	template<class Value, class Hash>
	class LZ2WayCacheTable {
		Value* arr = nullptr;
		int hashShift;
	public:
		LZ2WayCacheTable() {}
		~LZ2WayCacheTable() {
			delete[] arr;
		}
		void init(const int logSize) {
			arr = new Value[(size_t)2 << logSize]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
		}
		LZ2WayCacheBucket<Value> operator[](const size_t value) {
			return LZ2WayCacheBucket<Value>(arr + (Hash{}(value) >> hashShift) * 2);
		}
	};

	//Used for easier implementation
	template<class Value>
	class LZCacheBucket {
		Value* it;
		Value* last;
	public:
		LZCacheBucket() {}
		LZCacheBucket(Value* _begin, Value* _end) {
			it = _begin;
			last = _end;
		}
		//Pushes a new value into the bucket. Used when skipping bytes
		void push(const Value newValue) {
			//An std::move can be faster, but only for big buckets (8 elements or more)
			for (last--; last != it; last--)
				last[0] = last[-1];
			it[0] = newValue;
		}
		//Whether all positions in the bucket have been loaded
		bool ended() {
			return it == last;
		}
		//Loads the next position, and at the same time, stores the previous position in the same spot
		//The first time value must have the current position (so it basically pushes the new value into the bucket)
		void next(Value* value) {
			const Value tmp = *it;
			*it = *value;
			*value = tmp;
			it++;
		}
		//Loads the next position, but without updating the bucket as we go
		Value next() {
			return *it++;
		}
	};

	//This works like the basic hash table, except that it stores N positions per bucket
	template<class Value, class Hash>
	class LZCacheTable {
		Value* arr = nullptr;
		int hashShift;
		int elementsLog;
	public:
		//Use 2^x sizes to avoid the use of modulo and multiplication
		LZCacheTable() {}
		LZCacheTable(const int logSize, const int numElementsLog) {
			init(logSize, numElementsLog);
		}
		~LZCacheTable() {
			delete[] arr;
		}
		void init(const int logSize, const int numElementsLog) {
			arr = new Value[(size_t)1 << logSize << numElementsLog]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
			elementsLog = numElementsLog;
		}
		LZCacheBucket<Value> operator[](size_t value) {
			value = Hash{}(value) >> hashShift;
			Value* bucket = arr + (value << elementsLog);
			return LZCacheBucket<Value>(bucket, bucket + ((size_t)1 << elementsLog));
		}
	};

	struct LZStructure {
		uint32_t matchLength;
		uint32_t matchDistance;
		uint32_t literalRunLength;
	};

	struct LZMatch {
		uint32_t length;
		uint32_t distance;
		uint32_t pos;
	};

	//Simple and fast
	class HashTableMatchFinder {
		HashTable<uint32_t, FastIntHash> lzdict3;
		LZCacheTable<uint32_t, FastIntHash> lzdict4;
		LZCacheTable<uint32_t, FastIntHash> lzdict8;

	public:
		void init(const int windowLog, const CompressorOptions& compressorOptions) {
			const int hashLog = std::min(compressorOptions.maxHashLog, windowLog - 3);
			lzdict3.init(std::min(hashLog, 12));
			lzdict4.init(hashLog, compressorOptions.maxElementsLog);
			lzdict8.init(hashLog, compressorOptions.maxElementsLog);
		}

		LZMatch* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart,
			const uint8_t* const limit, LZMatch* matches, size_t minLength, const CompressorOptions& compressorOptions)
		{
			uint32_t& chain3 = lzdict3[read_hash3(input)];
			size_t nextExpectedLength = minLength;

			if (nextExpectedLength <= 3) {
				const uint8_t* where = inputStart + chain3;
				size_t length = test_match(input, where, limit, 3);

				if (length >= nextExpectedLength) {
					matches->distance = input - where;
					matches->length = length;
					matches++;

					if (length >= compressorOptions.niceLength) {
						chain3 = input - inputStart;
						return matches;
					}
					nextExpectedLength = length + 1;
				}
			}
			chain3 = input - inputStart;

			if (nextExpectedLength <= 7) {
				LZCacheBucket<uint32_t> chain4 = lzdict4[read_hash4(input)];
				uint32_t pos = input - inputStart;
				while (!chain4.ended()) {
					chain4.next(&pos);

					const uint8_t* where = inputStart + pos;

					if (*(input + nextExpectedLength - 1) != *(where + nextExpectedLength - 1))
						continue;

					const size_t length = test_match(input, where, limit, 4);

					if (length >= nextExpectedLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						nextExpectedLength = length + 1;
						if (length >= 7) {
							while (!chain4.ended())
								chain4.next(&pos);
							break;
						}
					}
				}
			}

			if (nextExpectedLength >= 4 && nextExpectedLength <= compressorOptions.niceLength) {
				LZCacheBucket<uint32_t> chain8 = lzdict8[read_hash8(input)];
				uint32_t pos = input - inputStart;
				while (!chain8.ended()) {
					chain8.next(&pos);

					const uint8_t* where = inputStart + pos;
					if (*(input + nextExpectedLength - 1) != *(where + nextExpectedLength - 1))
						continue;

					const size_t length = test_match(input, where, limit, 8);

					if (length >= nextExpectedLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						nextExpectedLength = length + 1;
						if (length >= compressorOptions.niceLength) {
							while (!chain8.ended())
								chain8.next(&pos);
							break;
						}
					}
				}
			}

			return matches;
		}

		void update_position(const uint8_t* const input, const uint8_t* const inputStart) {
			const size_t pos = input - inputStart;
			lzdict3[read_hash3(input)] = pos;
			lzdict4[read_hash4(input)].push(pos);
			lzdict8[read_hash8(input)].push(pos);
		}
	};

	const int NO_MATCH_POS = 0;
	const int LOOKAHEAD_MATCHES = 16;
	//Original match finder implementation from BriefLZ
	class BinaryMatchFinder {

		HashTable<uint32_t, FastIntHash> lzdict23;
		HashTable<uint32_t, FastIntHash> nodeLookup;
		uint32_t* nodes = nullptr;
		size_t nodeListSize;
		size_t nodeListMask;
		
		//Long distance matcher. Searching and storing several positions at a time
		// seems faster, maybe because of better cache use?
		HashTable<uint32_t, FastIntHash> lzdict16;
		LZMatch ldmAheadMatches[LOOKAHEAD_MATCHES];

	public:

		~BinaryMatchFinder() {
			delete[] nodes;
		}
		BinaryMatchFinder() {}

		void init(const size_t inputSize, const int windowLog, const CompressorOptions& compressorOptions) {

			const size_t binaryTreeWindow = std::min(compressorOptions.maxHashLog, windowLog);

			//Input size is smaller than maximum binary tree size
			if (inputSize < ((size_t)1 << binaryTreeWindow)) {
				nodes = new uint32_t[(size_t)2 * inputSize];
				nodeListSize = inputSize;
				nodeListMask = -1;
			}
			else {
				nodes = new uint32_t[(size_t)2 << binaryTreeWindow];
				nodeListSize = (size_t)1 << binaryTreeWindow;
				nodeListMask = nodeListSize - 1;

				if (windowLog > compressorOptions.maxHashLog) {
					lzdict16.init(std::max(1, (int)int_log2(std::min(inputSize, (size_t)1 << windowLog) - nodeListSize) - 3));
					memset(&ldmAheadMatches, 0, LOOKAHEAD_MATCHES * sizeof(LZMatch));
				}
			}
			lzdict23.init(std::min(12, windowLog - 3));
			nodeLookup.init(std::min(20, windowLog - 3));
		}

		LZMatch* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart,
			const uint8_t* const compressionLimit, const uint8_t* const blockLimit, LZMatch* matches,
			size_t minLength, const CompressorOptions& compressorOptions)
		{
			const size_t inputPosition = input - inputStart;
			size_t nextExpectedLength = minLength;

			//Length 2/3 match
			uint32_t& chain23 = lzdict23[compressorOptions.parser == OPTIMAL4 ? read_hash2(input) : read_hash3(input)];
			if (nextExpectedLength <= 3) {
				const uint8_t* where = inputStart + chain23;
				const size_t length = test_match(input, where, blockLimit, 2);

				if (length >= nextExpectedLength) {
					matches->distance = input - where;
					matches->length = length;
					matches++;

					if (length >= compressorOptions.niceLength) {
						update_position(input, inputStart, compressionLimit, compressorOptions);
						return matches;
					}
					nextExpectedLength = length;
					if (compressorOptions.parser != OPTIMAL4)
						nextExpectedLength++;
				}
			}
			chain23 = inputPosition;

			//If we reach this position stop the search
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;

			uint32_t& lookupEntry = nodeLookup[compressorOptions.parser == OPTIMAL4 ? read_hash3(input) : read_hash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			uint32_t* lesserNode = &nodes[2 * (lookupEntry & nodeListMask)];
			uint32_t* greaterNode = lesserNode + 1;
			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;

			size_t depth = (size_t)1 << compressorOptions.maxElementsLog;

			// Check matches
			while (true) {

				if (backPosition <= btEnd || depth-- == 0) {
					*lesserNode = NO_MATCH_POS;
					*greaterNode = NO_MATCH_POS;
					break;
				}

				const uint8_t* front = std::min(lesserFront, greaterFront);
				const uint8_t* back = front - (inputPosition - backPosition);

				const size_t extraLength = test_match(front, back, compressionLimit, 0);
				front += extraLength;
				back += extraLength;

				size_t length = front - input;
				//Match cant go outside of block boundaries
				const size_t effectiveLength = std::min(length, (size_t)(blockLimit - input));
				uint32_t* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
				if (effectiveLength >= nextExpectedLength) {
					nextExpectedLength = effectiveLength;
					if (compressorOptions.parser != OPTIMAL4)
						nextExpectedLength++;
					matches->distance = front - back;
					matches->length = effectiveLength;
					matches++;
				}

				if (length >= compressorOptions.niceLength) {
					*lesserNode = nextNode[0];
					*greaterNode = nextNode[1];
					if (inputPosition > nodeListSize)
						lzdict16[read_hash16(input - nodeListSize)] = (inputPosition - nodeListSize);
					return matches;
				}

				if (*back < *front) {
					*lesserNode = backPosition;
					lesserNode = &nextNode[1];
					backPosition = *lesserNode;
					lesserFront = front;
				}
				else {
					*greaterNode = backPosition;
					greaterNode = &nextNode[0];
					backPosition = *greaterNode;
					greaterFront = front;
				}
			}

			if (inputPosition > nodeListSize) {

				const size_t positionModulo = inputPosition % LOOKAHEAD_MATCHES;
				if (ldmAheadMatches[positionModulo].pos != inputPosition) {
					for (int i = 0; i < LOOKAHEAD_MATCHES; i++) {
						const uint8_t* const where = inputStart + lzdict16[read_hash16(input + i)];
						lzdict16[read_hash16(input + i - nodeListSize)] = inputPosition + i - nodeListSize;
						const size_t length = test_match(input + i, where, blockLimit, 16);

						size_t index = (positionModulo + i) % LOOKAHEAD_MATCHES;
						ldmAheadMatches[index].length = length;
						ldmAheadMatches[index].distance = input + i - where;
						ldmAheadMatches[index].pos = inputPosition + i;
						if (length >= compressorOptions.niceLength)
							break;
					}
				}
				if (ldmAheadMatches[positionModulo].length >= nextExpectedLength) {
					matches->distance = ldmAheadMatches[positionModulo].distance;
					matches->length = ldmAheadMatches[positionModulo].length;
					matches++;
				}
			}

			return matches;
		}

		void update_position(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			const CompressorOptions& compressorOptions) {

			const size_t inputPosition = input - inputStart;
			lzdict23[compressorOptions.parser == OPTIMAL4 ? read_hash2(input) : read_hash3(input)] = inputPosition;
			if (inputPosition > nodeListSize)
				lzdict16[read_hash16(input - nodeListSize)] = (inputPosition - nodeListSize);

			//If we reach this position on the front stop the update
			const uint8_t* positionSkip = std::min(limit, input + compressorOptions.niceLength);
			//If we reach this position on the back stop the update
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;
			uint32_t& lookupEntry = nodeLookup[compressorOptions.parser == OPTIMAL4 ? read_hash3(input) : read_hash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			uint32_t* lesserNode = &nodes[2 * (inputPosition & nodeListMask)];
			uint32_t* greaterNode = &nodes[2 * (inputPosition & nodeListMask) + 1];

			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;
			size_t depth = (size_t)1 << compressorOptions.maxElementsLog;

			// Check matches
			while (true) {
				if (backPosition <= btEnd || depth-- == 0) {
					*lesserNode = NO_MATCH_POS;
					*greaterNode = NO_MATCH_POS;
					return;
				}

				const uint8_t* front = std::min(lesserFront, greaterFront);
				const uint8_t* back = front - (inputPosition - backPosition);

				const size_t length = test_match(front, back, positionSkip, 0);
				front += length;
				back += length;

				uint32_t* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
				if (front >= positionSkip) {
					*lesserNode = nextNode[0];
					*greaterNode = nextNode[1];
					return;
				}

				if (*back < *front) {
					*lesserNode = backPosition;
					lesserNode = &nextNode[1];
					backPosition = *lesserNode;
					lesserFront = front;
				}
				else {
					*greaterNode = backPosition;
					greaterNode = &nextNode[0];
					backPosition = *greaterNode;
					greaterFront = front;
				}
			}
		}
	};

	FORCE_INLINE uint32_t mulHigh32(const uint32_t a, const uint32_t b) {
		return (uint64_t)a * b >> 32;
	}

	//https://github.com/romeric/fastapprox/blob/master/fastapprox/src/fastlog.h
	FORCE_INLINE float fast_log2(float x) {
		union { float f; uint32_t i; } vx = { x };
		float y = vx.i;
		y *= 1.1920928955078125e-7f;
		return y - 126.94269504f;
	}

	const int STRIDER_MAX_BLOCK_SIZE = 65536;
	const int STRIDER_MIN_LENGTH = 1;
	const int STRIDER_LAST_BYTES = 63;
	const int STRIDER_COST_PRECISION = 32;

	const int MODEL_PRECISION_BITS = 14;
	const int MODEL_SCALE = (1 << MODEL_PRECISION_BITS);
	const int MODEL_BIT_MASK = ((1 << MODEL_PRECISION_BITS) - 1);
	const int MODEL_UPDATE_SPEED = 7;

	const uint16_t NIBBLE_INITIAL[16] = {
		0, MODEL_SCALE / 16, 2 * MODEL_SCALE / 16, 3 * MODEL_SCALE / 16,
		4 * MODEL_SCALE / 16, 5 * MODEL_SCALE / 16, 6 * MODEL_SCALE / 16, 7 * MODEL_SCALE / 16,
		8 * MODEL_SCALE / 16, 9 * MODEL_SCALE / 16, 10 * MODEL_SCALE / 16, 11 * MODEL_SCALE / 16,
		12 * MODEL_SCALE / 16, 13 * MODEL_SCALE / 16, 14 * MODEL_SCALE / 16, 15 * MODEL_SCALE / 16
	};

	/* HOW TO GENERATE THE MIXIN
	for (int y = 0; y < 16; y++) {
		printf("{ ");
		for (int x = 0; x <= y; x++)
			printf("%d, ", x * 16);
		for (int x = y + 1; x < 16; x++)
			printf("%d, ", MODEL_SCALE + (1 << MODEL_UPDATE_SPEED) - (16 * 16) - 1 + x * 16);
		printf("},\n");
	} 
	*/

	const static union {
#if defined(x64) || defined(x86)
		__m128i NIBBLE_MIXIN_SSE[16][2];
#endif
		uint16_t NIBBLE_MIXIN_SCALAR[16][16] = {
			{ 0, 16271, 16287, 16303, 16319, 16335, 16351, 16367, 16383, 16399, 16415, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 16287, 16303, 16319, 16335, 16351, 16367, 16383, 16399, 16415, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 16303, 16319, 16335, 16351, 16367, 16383, 16399, 16415, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 16319, 16335, 16351, 16367, 16383, 16399, 16415, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 16335, 16351, 16367, 16383, 16399, 16415, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 80, 16351, 16367, 16383, 16399, 16415, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 80, 96, 16367, 16383, 16399, 16415, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 80, 96, 112, 16383, 16399, 16415, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 80, 96, 112, 128, 16399, 16415, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 16415, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 16431, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 16447, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 16463, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 16479, 16495, },
			{ 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 16495, },
			{ 0, 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, },
		};
	};

	//alignment on 32 byte boundary improves decode speed
	struct alignas(32) NibbleModel {

		union {
			uint16_t scalar[16];
#if defined(x64) || defined(x86)
			__m128i sse[2];  
#endif
		};

		NibbleModel() {}
		FORCE_INLINE void init() {
			memcpy(scalar, NIBBLE_INITIAL, 32);
		}

		FORCE_INLINE size_t get_freq(const size_t symbol) {
			const size_t high = scalar[(symbol + 1) & 0xF];
			const size_t low = scalar[symbol];
			return (high - low) & MODEL_BIT_MASK;
		}

#if defined(COMPILE_WITH_SSE2) 
		FORCE_INLINE void update_sse(const size_t symbol) {
			__m128i upd0 = _mm_srai_epi16(_mm_sub_epi16(NIBBLE_MIXIN_SSE[symbol][0], sse[0]), MODEL_UPDATE_SPEED);
			__m128i upd1 = _mm_srai_epi16(_mm_sub_epi16(NIBBLE_MIXIN_SSE[symbol][1], sse[1]), MODEL_UPDATE_SPEED);
			sse[0] = _mm_add_epi16(sse[0], upd0);
			sse[1] = _mm_add_epi16(sse[1], upd1);
		}
#endif

		FORCE_INLINE void update_scalar(const size_t symbol) {
			for (int i = 1; i <= 15; i++)
				scalar[i] += (NIBBLE_MIXIN_SCALAR[symbol][i] - scalar[i]) >> MODEL_UPDATE_SPEED;
		}

		FORCE_INLINE void update(const size_t symbol) {
#if defined(x64)
			update_sse(symbol);
#elif defined(COMPILE_WITH_SSE2)
			if (HAS_SSE2)
				update_sse(symbol);
			else
				update_scalar(symbol);
#else
			update_scalar(symbol);
#endif
		}

		//The work of the encoder
		FORCE_INLINE void encode_symbol(const size_t symbol, size_t* low, size_t* freq) {
			const size_t high = scalar[(symbol + 1) & 0xF];
			*low = scalar[symbol];
			*freq = (high - *low) & MODEL_BIT_MASK;
			update(symbol);
		}

#if defined(COMPILE_WITH_SSE2)
		FORCE_INLINE size_t decode_symbol_sse(const size_t value, size_t* low, size_t* freq) {
			__m128i cmp = _mm_cvtsi32_si128(value);
			cmp = _mm_shuffle_epi32(_mm_unpacklo_epi16(cmp, cmp), 0);
			__m128i tmp0 = _mm_cmpgt_epi16(sse[0], cmp);
			__m128i tmp1 = _mm_cmpgt_epi16(sse[1], cmp);
			const size_t mask = _mm_movemask_epi8(_mm_packs_epi16(tmp0, tmp1)) | 0x10000;
			size_t symbol = unsafe_bit_scan_forward(mask);

			const size_t high = scalar[symbol & 0xF];
			symbol -= 1;
			*low = scalar[symbol];
			*freq = (high - *low) & MODEL_BIT_MASK;

			tmp0 = _mm_and_si128(_mm_set1_epi16(16255), tmp0);
			tmp1 = _mm_and_si128(_mm_set1_epi16(16255), tmp1);
			tmp0 = _mm_add_epi16(tmp0, _mm_set_epi16(112, 96, 80, 64, 48, 32, 16, 0));
			tmp1 = _mm_add_epi16(tmp1, _mm_set_epi16(240, 224, 208, 192, 176, 160, 144, 128));
			sse[0] = _mm_add_epi16(_mm_srai_epi16(_mm_sub_epi16(tmp0, sse[0]), MODEL_UPDATE_SPEED), sse[0]);
			sse[1] = _mm_add_epi16(_mm_srai_epi16(_mm_sub_epi16(tmp1, sse[1]), MODEL_UPDATE_SPEED), sse[1]);
			return symbol;
		}
#endif

		FORCE_INLINE size_t decode_symbol_scalar(const size_t value, size_t* low, size_t* freq) {
			size_t symbol = 8 * (value >= scalar[8]);
			symbol += 4 * (value >= scalar[4 + symbol]);
			symbol += 2 * (value >= scalar[2 + symbol]);
			symbol += value >= scalar[1 + symbol];

			const size_t high = scalar[(symbol + 1) & 0xF];
			*low = scalar[symbol];
			*freq = (high - *low) & MODEL_BIT_MASK;

			update_scalar(symbol);
			return symbol;
		}

		//The work of the decoder
		FORCE_INLINE size_t decode_symbol(const size_t value, size_t* low, size_t* freq) {
#if defined(x64)
			return decode_symbol_sse(value, low, freq);
#elif defined(x86) && defined(COMPILE_WITH_SSE2)
			if (HAS_SSE2)
				return decode_symbol_sse(value, low, freq);
			else
				return decode_symbol_scalar(value, low, freq);
#else
			return decode_symbol_scalar(value, low, freq);
#endif
		}
	};

	const uint32_t RANS_BIT_PRECISION = 32;
	const uint32_t RANS_NORMALIZATION_INTERVAL = 1 << (RANS_BIT_PRECISION - 16);

	class RansEncoder {

		uint32_t stateA;
		uint32_t stateB;
		//rans stream has to be written backwards, store it first in a buffer
		uint8_t* streamBufferEnd = nullptr;
		uint8_t* streamBufferBegin;
		uint8_t* streamBufferIt;
		//rans symbols also have to be written backwards, so store them in this buffer
		uint32_t* symbolBufferBegin = nullptr;
		uint32_t* symbolBufferEnd;
		uint32_t* symbolBufferIt;

	public:
		RansEncoder() {}
		~RansEncoder() {
			delete[] streamBufferEnd;
			delete[] symbolBufferBegin;
		}

		//for symbolBufferSize:
		//nibble model takes 1 slot, raw bits take 1 slot for every 15 bits
		//returns whether it fails to initialize
		bool initialize_rans_encoder(const size_t streamBufferSize, const size_t symbolBufferSize) {

			try {
				streamBufferEnd = new uint8_t[streamBufferSize];
				symbolBufferBegin = new uint32_t[symbolBufferSize];
			}
			catch (const std::bad_alloc& e) {
				delete[] streamBufferEnd;
				delete[] symbolBufferBegin;
				return true;
			}

			streamBufferBegin = streamBufferEnd + streamBufferSize;
			streamBufferIt = streamBufferBegin;
			symbolBufferEnd = symbolBufferBegin + symbolBufferSize;
			symbolBufferIt = symbolBufferBegin;
			return false;
		}
		void start_rans() {
			symbolBufferIt = symbolBufferBegin;
			streamBufferIt = streamBufferBegin;
			stateA = RANS_NORMALIZATION_INTERVAL;
			stateB = RANS_NORMALIZATION_INTERVAL;
		}
		FORCE_INLINE void encode_nibble(const size_t symbol, NibbleModel* model) {
			//Not enough space available
			if (symbolBufferIt == symbolBufferEnd)
				return;
			size_t low, freq;
			model->encode_symbol(symbol, &low, &freq);
			*symbolBufferIt++ = (low << 15) | freq;
		}
		FORCE_INLINE void encode_raw_bits(size_t symbol, size_t nBits) {

			while (nBits) {
				//Not enough space available
				if (symbolBufferIt == symbolBufferEnd)
					return;
				size_t outBits = std::min(nBits, (size_t)16);
				*symbolBufferIt++ = 0x80000000 | (outBits << 16) | ((symbol >> (nBits - outBits)) & (1 << outBits) - 1);
				nBits -= outBits;
			}
		}
		//For nBits <= 16, it runs faster
		//Used for literal run length and match length
		FORCE_INLINE void encode_raw_bits_short(const size_t symbol, const size_t nBits) {
			if (nBits) {
				//Not enough space available
				if (symbolBufferIt == symbolBufferEnd)
					return;
				*symbolBufferIt++ = 0x80000000 | (nBits << 16) | symbol;
			}
		}
		FORCE_INLINE void renormalize(const size_t interval) {
			bool renormalize = stateB >= interval;
			write_uint16le(streamBufferIt - 2, stateB);
			streamBufferIt -= renormalize << 1;
			stateB = (0 - renormalize & ((stateB >> 16) ^ stateB)) ^ stateB;
		}

		//Encodes all the symbols stored into a rans stream and returns the encoded size.
		//In case it cant be safely encoded it will return -1.
		size_t end_rans(uint8_t* output) {

			//Very likely we are missing symbols
			if (symbolBufferIt == symbolBufferEnd)
				return -1;

			while (symbolBufferIt != symbolBufferBegin) {

				//Not enough space available
				if (streamBufferIt - streamBufferEnd < 2)
					return -1;

				std::swap(stateA, stateB);
				symbolBufferIt--;
				size_t data = *symbolBufferIt;

				//raw
				if (data >> 31) {
					const size_t nBits = (data >> 16) & 0xFF;
					const size_t symbol = data & 0xFFFF;

					const size_t interval = RANS_NORMALIZATION_INTERVAL << (16 - nBits);
					renormalize(interval);
					stateB = (stateB << nBits) + symbol;
				}
				//nibble
				else {
					const size_t freq = data & 0x7FFF;
					const size_t low = data >> 15;

					const size_t interval = (RANS_NORMALIZATION_INTERVAL << (16 - MODEL_PRECISION_BITS)) * freq;
					renormalize(interval);
					stateB = ((stateB / freq) << MODEL_PRECISION_BITS) + (stateB % freq) + low;
				}
			}

			//Not enough space available
			if (streamBufferIt - streamBufferEnd < 8)
				return -1;
			streamBufferIt -= 8;
			write_uint32le(streamBufferIt + 0, stateB);
			write_uint32le(streamBufferIt + 4, stateA);

			size_t finalBlockSize = streamBufferBegin - streamBufferIt;
			memcpy(output, streamBufferIt, finalBlockSize);

			return finalBlockSize;
		}
	};

	void get_histogram16(const uint8_t* buf, size_t bufSize, uint32_t histogram[16][256]) {

		std::fill_n(&histogram[0][0], 256 * 16, 0);
		const uint8_t* fastLoopEnd = buf + (bufSize & ~0xF);
		const uint8_t* bufEnd = buf + bufSize;
		for (; buf < fastLoopEnd; buf += 16) {
			if (IS_64BIT) {
				uint64_t w;
				memcpy(&w, buf, 8);
				histogram[0][w >> 0 & 0xFF]++;
				histogram[1][w >> 8 & 0xFF]++;
				histogram[2][w >> 16 & 0xFF]++;
				histogram[3][w >> 24 & 0xFF]++;
				histogram[4][w >> 32 & 0xFF]++;
				histogram[5][w >> 40 & 0xFF]++;
				histogram[6][w >> 48 & 0xFF]++;
				histogram[7][w >> 56 & 0xFF]++;
				memcpy(&w, buf + 8, 8);
				histogram[8][w >> 0 & 0xFF]++;
				histogram[9][w >> 8 & 0xFF]++;
				histogram[10][w >> 16 & 0xFF]++;
				histogram[11][w >> 24 & 0xFF]++;
				histogram[12][w >> 32 & 0xFF]++;
				histogram[13][w >> 40 & 0xFF]++;
				histogram[14][w >> 48 & 0xFF]++;
				histogram[15][w >> 56 & 0xFF]++;
			}
			else {
				uint32_t w;
				memcpy(&w, buf, 4);
				histogram[0][w >> 0 & 0xFF]++;
				histogram[1][w >> 8 & 0xFF]++;
				histogram[2][w >> 16 & 0xFF]++;
				histogram[3][w >> 24 & 0xFF]++;
				memcpy(&w, buf + 4, 4);
				histogram[4][w >> 0 & 0xFF]++;
				histogram[5][w >> 8 & 0xFF]++;
				histogram[6][w >> 16 & 0xFF]++;
				histogram[7][w >> 24 & 0xFF]++;
				memcpy(&w, buf + 8, 4);
				histogram[8][w >> 0 & 0xFF]++;
				histogram[9][w >> 8 & 0xFF]++;
				histogram[10][w >> 16 & 0xFF]++;
				histogram[11][w >> 24 & 0xFF]++;
				memcpy(&w, buf + 12, 4);
				histogram[12][w >> 0 & 0xFF]++;
				histogram[13][w >> 8 & 0xFF]++;
				histogram[14][w >> 16 & 0xFF]++;
				histogram[15][w >> 24 & 0xFF]++;
			}
		}
		for (int i = 0; buf < bufEnd; buf++, i++)
			histogram[i % 16][*buf]++;
	}

	float calculate_entropy16(uint32_t histogram[16][256], const size_t bufsize, const size_t alignment) {

		const float probDiv = 1 / float(bufsize / alignment);
		float entropy = 0;
		
		for (int a = 0; a < alignment; a++) {

			for (int b = 0; b < 256; b++) {

				uint32_t count;
				switch (alignment) {
				case 1:
					count = histogram[0][b] + histogram[1][b] + histogram[2][b] + histogram[3][b] +
						histogram[4][b] + histogram[5][b] + histogram[6][b] + histogram[7][b] +
						histogram[8][b] + histogram[9][b] + histogram[10][b] + histogram[11][b] +
						histogram[12][b] + histogram[13][b] + histogram[14][b] + histogram[15][b];
					break;
				case 2:
					count = histogram[a + 0][b] + histogram[a + 2][b] + histogram[a + 4][b] + histogram[a + 6][b] +
						histogram[a + 8][b] + histogram[a + 10][b] + histogram[a + 12][b] + histogram[a + 14][b];
					break;
				case 4:
					count = histogram[a + 0][b] + histogram[a + 4][b] + histogram[a + 8][b] + histogram[a + 12][b];
					break;
				case 8:
					count = histogram[a + 0][b] + histogram[a + 8][b];
					break;
				case 16:
					count = histogram[a][b];
					break;
				}

				if (count) {
					const float prob = count * probDiv;
					entropy += -(prob * fast_log2(prob));
				}
			}
		}
		entropy /= alignment;

		return entropy;
	}

	struct DataSector {
		const uint8_t* start;
		int pbMask;
		int lcShift;
		bool rawLiterals;
	};

	struct LiteralData {
		uint8_t literal;
		uint8_t positionContext;
		uint8_t previousLiteral;
		uint8_t exclude;
	};

	class DataDetector {
		DataSector* dataSectors = nullptr;
		size_t numberSectors;
		size_t currentSector;

	public:
		~DataDetector() {
			delete[] dataSectors;
		}
		DataDetector() {}

		int calculate_pb(const uint8_t* data, size_t inSize) {

			const size_t PREDICTOR_BLOCK_SIZE = 262144;
			const size_t numberBlocks = inSize / PREDICTOR_BLOCK_SIZE + (inSize % PREDICTOR_BLOCK_SIZE > 0);
			numberSectors = 0;
			currentSector = 0;

			try {
				dataSectors = new DataSector[numberBlocks + 1];
			}
			catch (const std::bad_alloc& e) {
				return -1;
			}

			for (size_t i = 0; i < numberBlocks; i++) {
				const size_t thisBlockStart = i * PREDICTOR_BLOCK_SIZE;
				const size_t thisBlockSize = std::min(inSize - thisBlockStart, PREDICTOR_BLOCK_SIZE);

				uint32_t blockHistogram[16][256];
				get_histogram16(data + thisBlockStart, thisBlockSize, blockHistogram);
				float bestEntropy = calculate_entropy16(blockHistogram, thisBlockSize, 1);
				int predictedPb = 0; //default pb = 0

				if (thisBlockSize > 65536) {

					float entropyPosSize2 = calculate_entropy16(blockHistogram, thisBlockSize, 2) + 0.05;
					if (entropyPosSize2 < bestEntropy) {
						bestEntropy = entropyPosSize2;
						predictedPb = 1;
					}
					float entropyPosSize4 = calculate_entropy16(blockHistogram, thisBlockSize, 4) + 0.10;
					if (entropyPosSize4 < bestEntropy) {
						bestEntropy = entropyPosSize4;
						predictedPb = 3;
					}
					float entropyPosSize8 = calculate_entropy16(blockHistogram, thisBlockSize, 8) + 0.30;
					if (entropyPosSize8 < bestEntropy) {
						bestEntropy = entropyPosSize8;
						predictedPb = 7;
					}
					float entropyPosSize16 = calculate_entropy16(blockHistogram, thisBlockSize, 16) + 0.60;
					if (entropyPosSize16 < bestEntropy) {
						bestEntropy = entropyPosSize16;
						predictedPb = 15;
					}
				}
				//If last block is small, reuse pb mask
				else if (numberSectors != 0)
					predictedPb = dataSectors[numberSectors - 1].pbMask;

				bool rawLiterals = false;
				if (bestEntropy > 7.9 || inSize < 128) {
					rawLiterals = true;
					predictedPb = 0;
				}
				if (numberSectors == 0 || dataSectors[numberSectors - 1].pbMask != predictedPb
					|| dataSectors[numberSectors - 1].rawLiterals != rawLiterals)
				{
					dataSectors[numberSectors] = { data + i * PREDICTOR_BLOCK_SIZE, predictedPb, 4, rawLiterals };
					numberSectors++;
				}
			}
			dataSectors[numberSectors] = { data + inSize, 0, 0 };

			//Set lower lc for small blocks
			for (size_t i = 0; i < numberSectors; i++) {
				size_t blockSize = dataSectors[i + 1].start - dataSectors[i].start;
				if (blockSize <= 524288 * (dataSectors[i].pbMask + 1) || dataSectors[i].rawLiterals)
					dataSectors[i].lcShift = 8;
			}

			return 0;
		}

		double test_literal_size(const LiteralData* literalBuffer, const size_t literalCount, NibbleModel* literalModel,
			uint8_t literalContextBitShift, uint8_t positionContextBitMask, const uint8_t* costTable) {

			size_t compressedSize = 0;

			for (size_t i = 0; i < (48 << (8 - literalContextBitShift)) * (positionContextBitMask + 1); i++)
				literalModel[i].init();

			for (size_t i = 0; i < literalCount; i++) {

				const uint8_t literal = literalBuffer[i].literal;
				const uint8_t positionContext = literalBuffer[i].positionContext;
				const uint8_t previousLiteral = literalBuffer[i].previousLiteral;
				const uint8_t exclude = literalBuffer[i].exclude;

				NibbleModel* const modelTree = &literalModel[(((positionContext << 8) | previousLiteral) >> literalContextBitShift) * 48];
				size_t low, freq;

				modelTree[exclude >> 4].encode_symbol(literal >> 4, &low, &freq);
				compressedSize += costTable[freq];
				modelTree[(literal >> 4) == (exclude >> 4) ? 16 | (exclude & 0xF) : 32 | (literal >> 4)].encode_symbol(literal & 0xF, &low, &freq);
				compressedSize += costTable[freq];
			}

			return (double)compressedSize / std::max(literalCount, (size_t)1) / STRIDER_COST_PRECISION;
		}

		int calculate_lc(const uint8_t* input, const size_t size, const uint8_t* costTable) {

			//Dont bother calculating lc for small files, 99.9% of the time no context is best
			if (size < 131072) {
				dataSectors[0].lcShift = 8;
				return 0;
			}

			HashTable<uint32_t, FastIntHash> lzdict;
			lzdict.init(std::min(int_log2(size) - 3, (size_t)18));

			const size_t maxLiteralCount = std::min(size / 4, (size_t)1 << 20);
			LiteralData* literalBuffer = nullptr;
			NibbleModel* literalModel = nullptr;
			try {
				literalBuffer = new LiteralData[maxLiteralCount];
				literalModel = new NibbleModel[48 * 256 * 16];
			}
			catch (std::bad_alloc& e) {
				delete[] literalBuffer;
				delete[] literalModel;
				return -1;
			}

			const uint8_t* const inputStart = input;
			input++;  //As always skip first byte so that it does not find a match with distance = 0 at the begining

			for (size_t i = 0; i < numberSectors; i++) {

				//Skip high entropy sectors
				if (dataSectors[i].rawLiterals) {
					dataSectors[i].lcShift = 8;
					continue;
				}

				size_t literalCount = 0;
				size_t lastDistance = 1;
				const size_t positionContextBitMask = dataSectors[i].pbMask;
				const uint8_t* const matchLimit = dataSectors[i + 1].start;

				for (; input < matchLimit; ) {

					uint32_t* dictEntry = &lzdict[read_hash4(input)];
					size_t matchLength = test_match(input, inputStart + *dictEntry, matchLimit, 4);
					size_t distance = input - (inputStart + *dictEntry);

					//We have found a match
					if (matchLength) {
						lastDistance = input - (inputStart + *dictEntry);
						*dictEntry = input - inputStart;
						const uint8_t* const matchEnd = input + matchLength;
						for (input++; input < matchEnd; input++)
							lzdict[read_hash4(input)] = input - inputStart;
						continue;
					}
					else {
						*dictEntry = input - inputStart;
						LiteralData literalData = { *input, (uint8_t)(reinterpret_cast<size_t>(input) & positionContextBitMask), input[-1], *(input - lastDistance) };
						literalBuffer[literalCount] = literalData;
						literalCount++;
						input++;

						if (literalCount == maxLiteralCount) {
							input = matchLimit;
							break;
						}
					}
				}

				int bestLc = 8;
				//Skip on low literal count
				if (literalCount >= 65536) {
					double bestSize = test_literal_size(literalBuffer, literalCount,
						literalModel, 8, positionContextBitMask, costTable);

					double sizeLc4 = test_literal_size(literalBuffer, literalCount,
						literalModel, 4, positionContextBitMask, costTable);
					if (sizeLc4 < bestSize) {
						bestLc = 4;
						bestSize = sizeLc4;
					}

					if (positionContextBitMask == 0 && bestLc == 4) {
						double sizeLc0 = test_literal_size(literalBuffer, literalCount,
							literalModel, 0, positionContextBitMask, costTable);
						if (sizeLc0 < bestSize)
							bestLc = 0;
					}
				}

				dataSectors[i].lcShift = bestLc;
			}

			delete[] literalBuffer;
			delete[] literalModel;
			return 0;
		}

		void get_block_info(const uint8_t* const input, size_t* blockSize, size_t* pb, size_t* lc, bool* rawLiterals) {
			if (input >= dataSectors[currentSector + 1].start)
				currentSector++;

			*blockSize = std::min((size_t)(dataSectors[currentSector + 1].start - input), (size_t)STRIDER_MAX_BLOCK_SIZE);
			*rawLiterals = dataSectors[currentSector].rawLiterals;
			if (dataSectors[currentSector].rawLiterals) {
				*pb = 0;
				*lc = 8;
			}
			else {
				*pb = dataSectors[currentSector].pbMask;
				*lc = dataSectors[currentSector].lcShift;
			}
		}
	};

	// too many models
	void reset_models(NibbleModel* literalRunLengthModel, size_t* matchContext, NibbleModel* literalModel,
		size_t literalContextBitsShift, size_t positionContextBitMask, bool rawLiterals, size_t* lastDistance, 
		size_t* repOffsets, NibbleModel* distanceModel, NibbleModel* distanceLow, NibbleModel* matchLengthModel,
		const bool allReset, const bool literalModelReset, const bool lengthModelReset) {

		if ((allReset || literalModelReset) && !rawLiterals) {
			for (size_t i = 0; i < ((64 << 8) >> literalContextBitsShift) * (positionContextBitMask + 1); i++) { literalModel[i].init(); }
		}
		if (allReset || lengthModelReset) {
			for (size_t i = 0; i < 17; i++) { literalRunLengthModel[i].init(); }
			for (size_t i = 0; i < 194; i++) { matchLengthModel[i].init(); }
		}
		if (allReset) {
			*matchContext = 0b1000;  //Match, literal run
			*lastDistance = 1;
			for (size_t i = 0; i < 8; i++) { repOffsets[i] = 1; }
			for (size_t i = 0; i < 24; i++) { distanceModel[i].init(); }
			distanceLow->init();
			distanceLow[1].init();
		}
	}

	void store_block_header(size_t blockSize, uint8_t positionContextBitMask, uint8_t literalContextBitShift,
		bool rawLiterals, bool resetModels, bool uncompressedBlock, uint8_t*& output)
	{
		write_uint16le(output, blockSize - 1);
		uint8_t metadata;
		if (uncompressedBlock)
			metadata = 1 << 7;
		else {
			metadata = resetModels << 6;
			if (!rawLiterals) 
				metadata |= int_log2(positionContextBitMask + 1) * 3 + literalContextBitShift / 4 + 1;
		}
		output[2] = metadata;
		output += 3;
	}

	FORCE_INLINE void encode_literal_run(RansEncoder* encoder, const uint8_t* literalRun,
		NibbleModel* literalRunLengthModel, size_t literalRunLength, size_t* matchContext,
		NibbleModel* literalModel, const size_t lastDistance, const size_t literalContextBitsShift,
		const size_t positionContextBitMask, size_t* positionContext, bool rawLiterals) {

		*positionContext = reinterpret_cast<size_t>(literalRun) & positionContextBitMask;

		encoder->encode_nibble(std::min(literalRunLength, (size_t)15), &literalRunLengthModel[*positionContext]);
		if (literalRunLength >= 15) {
			literalRunLength -= 14;  //Allows for some simplifications
			size_t symbol = int_log2(literalRunLength);
			encoder->encode_nibble(symbol, &literalRunLengthModel[16]);
			encoder->encode_raw_bits_short(literalRunLength & (1 << symbol) - 1, symbol);
			literalRunLength += 14;
		}

		if (literalRunLength) {
			 
			*matchContext >>= 2;
			if (rawLiterals) {
				while (literalRunLength > 1) {
					encoder->encode_raw_bits_short(read_uint16le(literalRun), 16);
					literalRun += 2;
					literalRunLength -= 2;
				} 
				if (literalRunLength == 1)
					encoder->encode_raw_bits_short(*literalRun, 8);
			}
			else {
				//Also used as the previous byte to reduce memory access
				size_t literal = literalRun[-1];
				size_t lowExclusionModel = 32;

				do {
					const size_t exclude = *(literalRun - lastDistance);
					const size_t excludeHigh = exclude >> 4;
					const size_t excludeLow = exclude & 0xF;

					NibbleModel* const literalModelTree =
						&literalModel[(((*positionContext << 8) | literal) >> literalContextBitsShift) * 64];

					literal = *literalRun;
					const size_t literalHigh = literal >> 4;
					const size_t literalLow = literal & 0xF;

					encoder->encode_nibble(literalHigh, &literalModelTree[excludeHigh]);
					encoder->encode_nibble(literalLow,
						&literalModelTree[excludeHigh == literalHigh ? lowExclusionModel | excludeLow : 48 | literalHigh]);

					literalRun++;
					literalRunLength--;
					*positionContext = reinterpret_cast<size_t>(literalRun) & positionContextBitMask;
					lowExclusionModel = 16;
				} while (literalRunLength);
			}
		}
	}

	FORCE_INLINE void encode_match_length(RansEncoder* encoder, const size_t matchLengthContext,
		const size_t positionContext, NibbleModel* matchLengthModel, size_t matchLength)
	{
		matchLength -= STRIDER_MIN_LENGTH;
		encoder->encode_nibble(std::min(matchLength, (size_t)15), &matchLengthModel[matchLengthContext * 16 | positionContext]);
		if (matchLength >= 15) {
			matchLength -= 15;
			encoder->encode_nibble(std::min(matchLength, (size_t)15), &matchLengthModel[144 + (matchLengthContext != 0) * 16 + positionContext]);
			if (matchLength >= 15) {
				matchLength -= 15;
				encoder->encode_nibble(matchLength % 16, &matchLengthModel[176 + positionContext]);
				matchLength >>= 4;

				if (matchLength >= 4) {
					matchLength -= 3;
					size_t rawBits = int_log2(matchLength);
					size_t symbol = rawBits + 4;
					encoder->encode_nibble(symbol, &matchLengthModel[192 + (matchLengthContext != 0)]);
					encoder->encode_raw_bits_short(matchLength & ((size_t)1 << rawBits) - 1, rawBits);
				}
				else {
					encoder->encode_nibble(matchLength, &matchLengthModel[192 + (matchLengthContext != 0)]);
				}
			}
		}
	}

	FORCE_INLINE void encode_match(RansEncoder* encoder, size_t* matchContext,
		const size_t positionContext, size_t* repOffsets, NibbleModel* distanceModel, NibbleModel* distanceLow,
		NibbleModel* matchLengthModel, size_t matchLength, size_t distance)
	{
		size_t rep = std::find(repOffsets, repOffsets + 8, distance) - repOffsets;
		size_t matchLengthContext;

		if (rep < 8) {

			encoder->encode_nibble(rep, &distanceModel[*matchContext]);
			*matchContext = (*matchContext >> 2) | 4;
			matchLengthContext = 0;

			for (; rep > 0; rep--)
				repOffsets[rep] = repOffsets[rep - 1];
			repOffsets[0] = distance;
		}
		else {

			repOffsets[7] = repOffsets[6];
			repOffsets[6] = distance;
			distance--;

			if (distance < 256) {
				size_t symbol = distance / 16;
				encoder->encode_nibble(8, &distanceModel[*matchContext]);
				encoder->encode_nibble(symbol, &distanceModel[16]);
				encoder->encode_nibble(distance & 0xF, &distanceLow[symbol != 0]);
				matchLengthContext = 1;
			}
			else {
				size_t logarithm = int_log2(distance);
				size_t rawBits = logarithm - 5;
				size_t symbol = logarithm * 2 + ((distance >> (logarithm - 1)) & 1);
				size_t symbolHigh = symbol >> 4;
				encoder->encode_nibble(8 | symbolHigh, &distanceModel[*matchContext]);
				size_t symbolLow = symbol & 0xF;
				encoder->encode_nibble(symbolLow, &distanceModel[16 | symbolHigh]);
				encoder->encode_raw_bits((distance >> 4) & (((size_t)1 << rawBits) - 1), rawBits);
				encoder->encode_nibble(distance & 0xF, &distanceLow[1]);
				matchLengthContext = 1 + logarithm / 8;
			}
			*matchContext = (*matchContext >> 2) | 8;
		}

		encode_match_length(encoder, matchLengthContext, positionContext, matchLengthModel, matchLength);
	}

	FORCE_INLINE void encode_last_distance_match(RansEncoder* encoder, size_t* matchContext,
		const size_t positionContext, size_t* repOffsets, NibbleModel* distanceModel,
		NibbleModel* matchLengthModel, size_t matchLength, size_t distance)
	{
		size_t rep = distance == repOffsets[0] ? 0 : 6;
		encoder->encode_nibble(rep, &distanceModel[*matchContext]);
		*matchContext = (*matchContext >> 2) | 4;

		for (; rep > 0; rep--)
			repOffsets[rep] = repOffsets[rep - 1];
		repOffsets[0] = distance;

		encode_match_length(encoder, 0, positionContext, matchLengthModel, matchLength);
	}

	size_t compress_greedy(const uint8_t* input, const size_t size, uint8_t* output, RansEncoder& encoder,
		const int windowLog, const CompressorOptions& compressorOptions, ProgressCallback* progress)
	{
		const size_t accelerationThreshold = 6;
		const size_t accelerationMax = 63;

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. 
		*output++ = *input++;

		DataDetector dataDetector;
		if (dataDetector.calculate_pb(input, size - STRIDER_LAST_BYTES - 1))
			return -1;

		//Model stuff
		bool resetAllModels = true;
		NibbleModel literalRunLengthModel[17];
		NibbleModel* literalModel = nullptr;
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthModel[194];

		size_t matchContext = 0;
		size_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		size_t positionContextBitMask = -1;
		bool rawLiterals = false;
		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };
		size_t acceleration = 1 << accelerationThreshold;

		const size_t hashLog = std::min(compressorOptions.maxHashLog, windowLog - 3);
		HashTable<uint32_t, FastIntHash> lzdict;
		try {
			lzdict.init(hashLog);
		}
		catch (std::bad_alloc& e) {
			return -1;
		}

		for (; input < compressionLimit; ) {

			uint8_t* const compressedBlockStart = output;
			const uint8_t* const thisBlockStart = input;

			bool newRawLiterals;
			size_t thisBlockSize, newPositionContextBitMask, newLiteralContextBitShift;
			dataDetector.get_block_info(input, &thisBlockSize, &newPositionContextBitMask, 
				&newLiteralContextBitShift, &newRawLiterals);
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//The size of the literal model has been modified
			if (literalContextBitsShift != newLiteralContextBitShift ||
				positionContextBitMask != newPositionContextBitMask ||
				rawLiterals != newRawLiterals)
			{
				delete[] literalModel;
				literalModel = nullptr;
				if (!newRawLiterals) {
					try {
						literalModel = new NibbleModel[((64 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
					}
					catch (const std::bad_alloc& e) {
						return -1;
					}
				}
			}

			bool resetLiteralModel = literalContextBitsShift != newLiteralContextBitShift ||
				positionContextBitMask != newPositionContextBitMask || rawLiterals != newRawLiterals;
			bool resetLengthModel = positionContextBitMask != newPositionContextBitMask;
			reset_models(literalRunLengthModel, &matchContext, literalModel, newLiteralContextBitShift,
				newPositionContextBitMask, newRawLiterals, &distance, repOffsets, distanceModel,
				distanceLow, matchLengthModel, resetAllModels, resetLiteralModel, resetLengthModel);
			literalContextBitsShift = newLiteralContextBitShift;
			positionContextBitMask = newPositionContextBitMask;
			rawLiterals = newRawLiterals;

			store_block_header(thisBlockSize, positionContextBitMask, literalContextBitsShift, rawLiterals, resetAllModels, 0, output);
			encoder.start_rans();

			resetAllModels = false;
			const uint8_t* literalRunStart = input;
			size_t literalCount = 0;

			for (; input < thisBlockEnd; ) {

				//First try to get a rep match
				size_t matchLength = test_match(input + 1, input + 1 - distance, thisBlockEnd, 4);
				if (matchLength) {

					input++;
					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;
					literalCount += literalRunLength;

					encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext, rawLiterals);

					//Only update 1 position. This is enough
					lzdict[read_hash6(input)] = input - inputStart;
					input += matchLength;

					literalRunStart = input;
					encode_last_distance_match(&encoder, &matchContext, positionContext, repOffsets,
						distanceModel, matchLengthModel, matchLength, distance);

					acceleration = 1 << accelerationThreshold;
					continue;
				}

				//If no rep, try a normal match
				uint32_t* dictEntry = &lzdict[read_hash6(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				matchLength = test_match(input, match, thisBlockEnd, 6);

				if (matchLength) {

					//Lazy match
					dictEntry = &lzdict[read_hash6(input + 1)];
					const uint8_t* lazyMatch = inputStart + *dictEntry;
					*dictEntry = input + 1 - inputStart;
					size_t lazyLength = test_match(input + 1, lazyMatch, thisBlockEnd, 6);
					if (lazyLength > matchLength) {
						match = lazyMatch;
						matchLength = lazyLength;
						input++;
					}

					//Update up to position 6
					lzdict[read_hash6(input + 2)] = input + 2 - inputStart;
					lzdict[read_hash6(input + 3)] = input + 3 - inputStart;
					lzdict[read_hash6(input + 4)] = input + 4 - inputStart;
					lzdict[read_hash6(input + 5)] = input + 5 - inputStart;

					//Expand match to the left
					while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
						match--;
						input--;
						matchLength++;
					}

					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;
					literalCount += literalRunLength;

					encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext, rawLiterals);

					distance = input - match;
					input += matchLength;

					literalRunStart = input;
					encode_match(&encoder, &matchContext, positionContext, repOffsets,
						distanceModel, distanceLow, matchLengthModel, matchLength, distance);

					acceleration = 1 << accelerationThreshold;
				}
				else {
					input += acceleration >> accelerationThreshold;
					if (acceleration < (accelerationMax << accelerationThreshold))
						acceleration++;
				}
			}

			//We might have surpassed block end while skiping bytes
			input = std::min(input, thisBlockEnd);
			//Last literal run
			size_t literalRunLength = input - literalRunStart;
			literalCount += literalRunLength;

			bool sendUncompressed = true;
			//Do not attempt encoding if the entropy is high and not many matches are found
			if (!(rawLiterals && thisBlockSize - literalCount < thisBlockSize / 32)) {
				//Last literal run
				size_t positionContext;
				encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchContext,
					literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext, rawLiterals);

				size_t compressedBlockSize = encoder.end_rans(output);
				output += compressedBlockSize;
				sendUncompressed = compressedBlockSize >= thisBlockSize;
			}

			//If the algorithm ends up expanding the data, store it uncompressed and reset all models
			if (sendUncompressed) {
				resetAllModels = true;
				output = compressedBlockStart;
				store_block_header(thisBlockSize, 0, 0, 0, 0, 1, output);
				memcpy(output, thisBlockStart, thisBlockSize);
				input = thisBlockStart + thisBlockSize;
				output += thisBlockSize;
			}

			if (progress->progress(input - inputStart, output - outputStart)) {
				delete[] literalModel;
				return 0;
			}
		}

		memcpy(output, input, STRIDER_LAST_BYTES);
		progress->progress(input - inputStart + STRIDER_LAST_BYTES, output - outputStart + STRIDER_LAST_BYTES);
		delete[] literalModel;

		return output - outputStart + STRIDER_LAST_BYTES;
	}

	//Try to find the longest match for a given position. Then try to find an even longer one in the next.
	//If it is found, code a literal, and the longer one found. If not, code the original.
	FORCE_INLINE void fast_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const blockEnd,
		LZ2WayCacheTable<uint32_t, FastIntHash>* lzdict, size_t* bestMatchLength, size_t* bestMatchDistance,
		int* lazySteps, int* testedPositions, const size_t repOffset, const CompressorOptions& compressorOptions) {

		*testedPositions = 1;
		LZ2WayCacheBucket<uint32_t> dictEntry = (*lzdict)[read_hash6(input)];

		//Test first entry
		uint32_t pos = input - inputStart;
		dictEntry.first(&pos);
		const uint8_t* where = inputStart + pos;
		*bestMatchLength = test_match(input, where, blockEnd, 6);
		*bestMatchDistance = input - where;
		if (*bestMatchLength >= compressorOptions.niceLength) {
			dictEntry.push_in_second(pos);
			*lazySteps = 0;
			return;
		}

		//Test second entry
		dictEntry.second(&pos);
		where = inputStart + pos;
		//Simple heuristic: as we are looking for a longer match, we can first
		//test the byte that would make this match longer. There is a high
		//chance it will differ, so the rest of the match wont need to be tested
		if (*(input + *bestMatchLength) == *(where + *bestMatchLength)) {
			const size_t length = test_match(input, where, blockEnd, 6);
			if (length > *bestMatchLength) {
				*bestMatchDistance = input - where;
				*bestMatchLength = length;

				if (*bestMatchLength >= compressorOptions.niceLength) {
					*lazySteps = 0;
					return;
				}
			}
		}

		//Nothing was found, code a literal and try again from the begining
		if (*bestMatchLength < 6) {
			*lazySteps = 1;
			return;
		}

		//Now try to find a longer match at next position
		input++;
		*lazySteps = 0;
		*testedPositions = 2;
		dictEntry = (*lzdict)[read_hash6(input)];

		//Test first entry
		pos = input - inputStart;
		dictEntry.first(&pos);
		where = inputStart + pos;
		if (*(input + *bestMatchLength) == *(where + *bestMatchLength)) {
			const size_t length = test_match(input, where, blockEnd, 6);
			if (length > *bestMatchLength) {
				*bestMatchDistance = input - where;
				*bestMatchLength = length;
				*lazySteps = 1;

				if (*bestMatchLength >= compressorOptions.niceLength) {
					dictEntry.push_in_second(pos);
					return;
				}
			}
		}
		//Test second entry
		dictEntry.second(&pos);
		where = inputStart + pos;
		if (*(input + *bestMatchLength) == *(where + *bestMatchLength)) {
			const size_t length = test_match(input, where, blockEnd, 6);
			if (length > *bestMatchLength) {
				*bestMatchDistance = input - where;
				*bestMatchLength = length;
				*lazySteps = 1;
			}
		}
	}

	//Same principle as the last function, but with additional heuristics to improve ratio
	FORCE_INLINE void lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LZCacheTable<uint32_t, FastIntHash>* lzdict4, LZCacheTable<uint32_t, FastIntHash>* lzdict8, size_t* bestLength,
		size_t* bestDistance, const size_t repOffset, int* lazySteps, int* testedPositions, const CompressorOptions& compressorOptions) {

		LZCacheBucket<uint32_t> chain4 = (*lzdict4)[read_hash4(input)];
		LZCacheBucket<uint32_t> chain8 = (*lzdict8)[read_hash8(input)];
		uint32_t pos = input - inputStart;
		*testedPositions = 1;
		*lazySteps = 0;
		size_t bestMatchCost = 0;
		size_t length;

		//Try to get a length 8
		pos = input - inputStart;
		while (!chain8.ended()) {
			chain8.next(&pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + *bestLength) != *(where + *bestLength))
				continue;

			length = test_match(input, where, limit, 8);

			if (length > *bestLength) {
				*bestDistance = input - where;
				*bestLength = length;

				if (*bestLength >= compressorOptions.niceLength) {
					chain4.push(input - inputStart);
					while (!chain8.ended())
						chain8.next(&pos);
					return;
				}
			}
		}

		//If we did not found a length 8 try a length 4
		if (*bestLength < 8) {

			pos = input - inputStart;
			while (!chain4.ended()) {
				chain4.next(&pos);

				const uint8_t* where = inputStart + pos;

				if (*(input + *bestLength) != *(where + *bestLength))
					continue;

				length = test_match(input, where, limit, 4);

				size_t distance = input - where;
				size_t matchCost = 2 + unsafe_int_log2(distance) / 4;
				if (length + bestMatchCost > matchCost + *bestLength) {
					*bestDistance = distance;
					*bestLength = length;
					bestMatchCost = matchCost;
					//Next match length is 8, so stop search
					if (*bestLength >= 7) {
						while (!chain4.ended())
							chain4.next(&pos);
						if (*bestLength >= compressorOptions.niceLength)
							return;
						break;
					}
				}
			}
		}

		//No match found, code a literal and retry
		if (*bestLength == 0) {
			*lazySteps = 1;
			return;
		}

		//Now try to get a better match at pos + 1
		input++;
		pos = input - inputStart;
		(*lzdict4)[read_hash4(input)].push(pos);  //We wont search for length < 8
		chain8 = (*lzdict8)[read_hash8(input)];
		*testedPositions = 2;

		//Only try to find matches of length at least 8 at pos + 1
		while (!chain8.ended()) {
			chain8.next(&pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + *bestLength) != *(where + *bestLength))
				continue;

			length = test_match(input, where, limit, 8);

			if (length > *bestLength) {
				*bestDistance = input - where;
				*bestLength = length;
				*lazySteps = 1;

				if (*bestLength >= compressorOptions.niceLength) {
					while (!chain8.ended())
						chain8.next(&pos);
					return;
				}
			}
		}
	}

	size_t compress_lazy(const uint8_t* input, const size_t size, uint8_t* output, RansEncoder& encoder,
		const int windowLog, const CompressorOptions& compressorOptions, ProgressCallback* progress)
	{
		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. Its progress will be reported at the end
		*output++ = *input++;

		DataDetector dataDetector;
		if (dataDetector.calculate_pb(input, size - STRIDER_LAST_BYTES - 1))
			return -1;

		//Model stuff
		bool resetAllModels = true;
		NibbleModel literalRunLengthModel[17];
		NibbleModel* literalModel = nullptr;
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthModel[194];

		size_t matchContext = 0;
		size_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		size_t positionContextBitMask = -1;
		bool rawLiterals = false;
		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

		const size_t hashLog = std::min(compressorOptions.maxHashLog, windowLog - 3);
		LZ2WayCacheTable<uint32_t, FastIntHash> lzdict6;
		LZCacheTable<uint32_t, FastIntHash> lzdict4;
		LZCacheTable<uint32_t, FastIntHash> lzdict8;
		try {
			if (compressorOptions.parser == LAZY2)
				lzdict6.init(hashLog);
			else {
				lzdict4.init(hashLog, compressorOptions.maxElementsLog);
				lzdict8.init(hashLog, compressorOptions.maxElementsLog);
			}
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		for (; input < compressionLimit; ) {

			uint8_t* const compressedBlockStart = output;
			const uint8_t* const thisBlockStart = input;

			bool newRawLiterals;
			size_t thisBlockSize, newPositionContextBitMask, newLiteralContextBitShift;
			dataDetector.get_block_info(input, &thisBlockSize, &newPositionContextBitMask,
				&newLiteralContextBitShift, &newRawLiterals);
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//The size of the literal model has been modified
			if (literalContextBitsShift != newLiteralContextBitShift ||
				positionContextBitMask != newPositionContextBitMask ||
				rawLiterals != newRawLiterals)
			{
				delete[] literalModel;
				literalModel = nullptr;
				if (!newRawLiterals) {
					try {
						literalModel = new NibbleModel[((64 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
					}
					catch (const std::bad_alloc& e) {
						return -1;
					}
				}
			}

			bool resetLiteralModel = literalContextBitsShift != newLiteralContextBitShift ||
				positionContextBitMask != newPositionContextBitMask || rawLiterals != newRawLiterals;
			bool resetLengthModel = positionContextBitMask != newPositionContextBitMask;
			reset_models(literalRunLengthModel, &matchContext, literalModel, newLiteralContextBitShift,
				newPositionContextBitMask, newRawLiterals, &distance, repOffsets, distanceModel,
				distanceLow, matchLengthModel, resetAllModels, resetLiteralModel, resetLengthModel);
			literalContextBitsShift = newLiteralContextBitShift;
			positionContextBitMask = newPositionContextBitMask;
			rawLiterals = newRawLiterals;

			store_block_header(thisBlockSize, positionContextBitMask, literalContextBitsShift, rawLiterals, resetAllModels, 0, output);
			encoder.start_rans();

			resetAllModels = false;
			const uint8_t* literalRunStart = input;
			size_t literalCount = 0;

			for (; input < thisBlockEnd; ) {

				//First try to get a rep match
				size_t matchLength = test_match(input + 1, input + 1 - distance, thisBlockEnd, 4);
				if (matchLength) {

					input++;
					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;
					literalCount += literalRunLength;

					encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext, rawLiterals);

					if (compressorOptions.parser == LAZY2) {
						lzdict6[read_hash6(input - 1)].push_in_first(input - 1 - inputStart);
						lzdict6[read_hash6(input + 0)].push_in_first(input + 0 - inputStart);
						lzdict6[read_hash6(input + 1)].push_in_first(input + 1 - inputStart);
						lzdict6[read_hash6(input + 2)].push_in_first(input + 2 - inputStart);
						lzdict6[read_hash6(input + 3)].push_in_first(input + 3 - inputStart);
					}
					else {
						lzdict4[read_hash4(input - 1)].push(input - 1 - inputStart);
						lzdict8[read_hash8(input - 1)].push(input - 1 - inputStart);
						lzdict4[read_hash4(input + 0)].push(input + 0 - inputStart);
						lzdict8[read_hash8(input + 0)].push(input + 0 - inputStart);
						lzdict4[read_hash4(input + 1)].push(input + 1 - inputStart);
						lzdict8[read_hash8(input + 1)].push(input + 1 - inputStart);
						lzdict4[read_hash4(input + 2)].push(input + 2 - inputStart);
						lzdict8[read_hash8(input + 2)].push(input + 2 - inputStart);
						lzdict4[read_hash4(input + 3)].push(input + 3 - inputStart);
						lzdict8[read_hash8(input + 3)].push(input + 3 - inputStart);
					}
					input += matchLength;

					literalRunStart = input;
					encode_last_distance_match(&encoder, &matchContext, positionContext, repOffsets,
						distanceModel, matchLengthModel, matchLength, distance);
					continue;
				}

				size_t newDistance;
				int lazySteps;   //bytes to skip because of lazy matching
				int testedPositions;   //number of positions that have been added to hash table

				if (compressorOptions.parser == LAZY2) {
					fast_lazy_search(input, inputStart, thisBlockEnd, &lzdict6,
						&matchLength, &newDistance, &lazySteps, &testedPositions, distance, compressorOptions);
				}
				else {
					lazy_search(input, inputStart, thisBlockEnd, &lzdict4, &lzdict8, &matchLength,
						&newDistance, distance, &lazySteps, &testedPositions, compressorOptions);
				}

				input += lazySteps;
				//We have found a match
				if (matchLength) {

					//If distance < length, update the hash table only on the first "distance" bytes
					const uint8_t* const updateEnd = input + std::min(distance, matchLength);
					const uint8_t* matchPos = input + testedPositions - lazySteps;
					if (compressorOptions.parser == LAZY2) {
						for (; matchPos < updateEnd; matchPos++)
							lzdict6[read_hash6(matchPos)].push_in_first(matchPos - inputStart);
					}
					else {
						for (; matchPos < updateEnd; matchPos++) {
							lzdict4[read_hash4(matchPos)].push(matchPos - inputStart);
							lzdict8[read_hash8(matchPos)].push(matchPos - inputStart);
						}
					}

					//Expand match to the left
					const uint8_t* match = input - newDistance;
					while (input > literalRunStart && match > inputStart && input[-1] == match[-1]) {
						matchLength++;
						input--;
						match--;
					}

					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;
					literalCount += literalRunLength;

					encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext, rawLiterals);

					distance = newDistance;
					input += matchLength;
					literalRunStart = input;
					//Output the match
					encode_match(&encoder, &matchContext, positionContext, repOffsets,
						distanceModel, distanceLow, matchLengthModel, matchLength, distance);
				}
			}

			//Last literal run
			size_t literalRunLength = input - literalRunStart;
			literalCount += literalRunLength;

			bool sendUncompressed = true;
			//Do not attempt encoding if the entropy is high and not many matches are found
			if (!(rawLiterals && thisBlockSize - literalCount < thisBlockSize / 32)) {
				//Last literal run
				size_t positionContext;
				encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchContext,
					literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext, rawLiterals);

				size_t compressedBlockSize = encoder.end_rans(output);
				output += compressedBlockSize;
				sendUncompressed = compressedBlockSize >= thisBlockSize;
			}

			//If the algorithm ends up expanding the data, store it uncompressed and reset all models
			if (sendUncompressed) {
				resetAllModels = true;
				output = compressedBlockStart;
				store_block_header(thisBlockSize, 0, 0, 0, 0, 1, output);
				memcpy(output, thisBlockStart, thisBlockSize);
				input = thisBlockStart + thisBlockSize;
				output += thisBlockSize;
			}

			if (progress->progress(input - inputStart, output - outputStart)) {
				delete[] literalModel;
				return 0;
			}
		}

		memcpy(output, input, STRIDER_LAST_BYTES);
		progress->progress(input - inputStart + STRIDER_LAST_BYTES, output - outputStart + STRIDER_LAST_BYTES);
		delete[] literalModel;

		return output - outputStart + STRIDER_LAST_BYTES;
	}

	struct StriderFastOptimalState {
		uint32_t sizeCost;
		uint16_t literalRunLength;
		uint16_t matchLength;
		uint32_t repOffsets[2];   //Match distance will always be the first offset stored here
	};

	LZStructure* forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const blockLimit,
		HashTableMatchFinder* matchFinder, StriderFastOptimalState* parser, LZStructure* stream,
		size_t* repOffsets, const CompressorOptions& compressorOptions) {

		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		//Initialize positions cost to maximum. We dont need to initialize ALL, only enough ahead
		// to cover the maximum match length we can write, which is niceLength - 1, otherwise we would simply skip.
		//This speeds up compression on data with a lot of long matches.
		for (size_t i = 1; i < compressorOptions.niceLength; i++)
			parser[i].sizeCost = UINT32_MAX;
		parser[0].sizeCost = 0;
		parser[0].repOffsets[0] = repOffsets[0];
		parser[0].repOffsets[1] = repOffsets[6];
		parser[0].literalRunLength = 0;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			StriderFastOptimalState* parserPosition = parser + position;
			//Make sure we have enough positions ahead initialized
			parserPosition[compressorOptions.niceLength].sizeCost = UINT32_MAX;

			//Unpromising position
			if (parserPosition[0].sizeCost + 4 > parserPosition[1].sizeCost) {
				matchFinder->update_position(inputPosition, inputStart);
				continue;
			}

			size_t literalSizeCost = parserPosition->sizeCost + 6;
			StriderFastOptimalState* nextPosition = parserPosition + 1;
			if (literalSizeCost < nextPosition->sizeCost) {
				nextPosition->sizeCost = literalSizeCost;
				memcpy(nextPosition->repOffsets, parserPosition->repOffsets, 2 * sizeof(uint32_t));
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
			}

			size_t repMatchLength = 0;
			//Try to find rep offsets
			size_t rep = parserPosition->literalRunLength == 0;
			for (; rep < 2; rep++) {
				size_t length = test_match(inputPosition, 
					inputPosition - parserPosition->repOffsets[rep], blockLimit, 2);
				if (length) {
					repMatchLength = length;
					break;
				}
			}

			if (repMatchLength) {
				size_t repDistance = parserPosition->repOffsets[rep];
				if (repMatchLength >= compressorOptions.niceLength / 2) {
					matchFinder->update_position(inputPosition, inputStart);
					lastMatchLength = repMatchLength;
					lastMatchDistance = repDistance;
					goto doBackwardParse;
				}

				const size_t matchSizeCost = parserPosition->sizeCost + 9 + rep;  //Increase cost with each successive offset

				nextPosition = parserPosition + repMatchLength;
				if (matchSizeCost < nextPosition->sizeCost) {
					nextPosition->sizeCost = matchSizeCost;
					nextPosition->matchLength = repMatchLength;
					nextPosition->repOffsets[0] = repDistance;
					memcpy(&nextPosition->repOffsets[1], parserPosition->repOffsets, rep * sizeof(uint32_t));
					memcpy(&nextPosition->repOffsets[rep + 1], &parserPosition->repOffsets[rep + 1], (1 - rep) * sizeof(uint32_t));
					nextPosition->literalRunLength = 0;
				}
			}

			LZMatch matches[17];
			const LZMatch* matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart,
				blockLimit, matches, repMatchLength + 1, compressorOptions);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZMatch* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					break;
				}

				for (const LZMatch* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					const size_t matchSizeCost = parserPosition->sizeCost + 12 + int_log2((size_t)matchIt->distance - 1);

					nextPosition = parserPosition + matchIt->length;
					if (matchSizeCost < nextPosition->sizeCost) {
						nextPosition->sizeCost = matchSizeCost;
						nextPosition->matchLength = matchIt->length;
						nextPosition->repOffsets[0] = matchIt->distance;
						memcpy(&nextPosition->repOffsets[1], parserPosition->repOffsets, 1 * sizeof(uint32_t));
						nextPosition->literalRunLength = 0;
					}
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const StriderFastOptimalState* backwardParse;
		const StriderFastOptimalState* const parseEnd = parser;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			backwardParse = parser + position;
			stream->literalRunLength = backwardParse->literalRunLength;
			backwardParse -= backwardParse->literalRunLength;
			stream++;

			//Update only first positions if distance < length
			const uint8_t* const updateEnd = input + position + std::min(lastMatchDistance, lastMatchLength);
			const uint8_t* inputPosition = input + position + 1;
			for (; inputPosition < updateEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart);
		}
		else {
			size_t bestPos = blockLength;
			size_t bestSize = parser[blockLength].sizeCost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				//Bias the end of parse towards longer lengths
				if (parser[blockLength + i].sizeCost <= bestSize + (blockLength + i - bestPos) * STRIDER_COST_PRECISION * 2) 
				{
					bestPos = blockLength + i;
					bestSize = parser[blockLength + i].sizeCost;
				}
			}
			backwardParse = parser + bestPos;
			stream->literalRunLength = backwardParse->literalRunLength;
			backwardParse -= backwardParse->literalRunLength;
			stream++;
		}

		while (backwardParse > parseEnd) {

			stream->matchDistance = backwardParse->repOffsets[0];
			stream->matchLength = backwardParse->matchLength;
			backwardParse -= backwardParse->matchLength;
			stream->literalRunLength = backwardParse->literalRunLength;
			backwardParse -= backwardParse->literalRunLength;
			stream++;
		}

		return stream - 1;
	}

	struct StriderOptimalParserState {
		uint32_t sizeCost;
		//The maximum literal run length is 65536, so it can overflow, but as it is 
		// very rare it does not matter much. Also the backwards parse does not depend on this variable
		uint16_t literalRunLength;
		uint16_t matchLength;
		uint16_t currentLiteralRunLengthCost;  //Cache so it does not get computed twice
		uint8_t prevPath;
		uint8_t matchContext;
		uint32_t distance;   //Also serves as last used distance
		uint32_t repOffsets[8];
	};

	FORCE_INLINE size_t get_literal_run_cost(size_t literalRunLength, const size_t positionContext,
		NibbleModel* literalRunLengthModel, const uint8_t* costTable) {

		size_t cost = costTable[literalRunLengthModel[positionContext].get_freq(std::min((size_t)15, literalRunLength))];
		if (literalRunLength >= 15) {
			literalRunLength -= 14;
			size_t symbol = int_log2(literalRunLength);
			cost += costTable[literalRunLengthModel[16].get_freq(symbol)] + symbol * STRIDER_COST_PRECISION;
		}
		return cost;
	}

	FORCE_INLINE size_t get_match_length_cost(size_t length, const size_t positionContext,
		const size_t matchLengthContext, NibbleModel* matchLengthModel, const uint8_t* costTable) {
		
		length -= STRIDER_MIN_LENGTH;
		size_t cost = costTable[matchLengthModel[matchLengthContext * 16 | positionContext].get_freq(std::min((size_t)15, length))];
		if (length >= 15) {
			length -= 15;
			cost += costTable[matchLengthModel[144 + (matchLengthContext != 0) * 16 + positionContext].get_freq(std::min((size_t)15, length))];
			if (length >= 15) {
				length -= 15;
				cost += costTable[matchLengthModel[176 + positionContext].get_freq(length % 16)];
				length >>= 4;

				size_t symbol, rawBits;
				if (length >= 4) {
					rawBits = int_log2(length - 3);
					symbol = rawBits + 4;
				}
				else {
					symbol = length;
					rawBits = 0;
				}
				cost += costTable[matchLengthModel[192 + (matchLengthContext != 0)].get_freq(symbol)];
				cost += STRIDER_COST_PRECISION * rawBits;
			}
		}
		return cost;
	}

	FORCE_INLINE size_t get_distance_cost(size_t distance, NibbleModel* distanceModel,
		NibbleModel* distanceLow, const size_t matchContext, const uint8_t* costTable) {

		size_t cost = 0;

		distance--;
		if (distance < 256) {
			size_t symbol = distance / 16;
			cost += costTable[distanceModel[matchContext].get_freq(8)];
			cost += costTable[distanceModel[16].get_freq(symbol)];
			cost += costTable[distanceLow[symbol != 0].get_freq(distance & 0xF)];
		}
		else {
			size_t logarithm = int_log2(distance);
			size_t rawBits = logarithm - 5;
			size_t symbol = logarithm * 2 + ((distance >> (logarithm - 1)) & 1);

			size_t symbolHigh = symbol >> 4;
			cost += costTable[distanceModel[matchContext].get_freq(8 | symbolHigh)];
			size_t symbolLow = symbol & 0xF;
			cost += costTable[distanceModel[16 | symbolHigh].get_freq(symbolLow)];
			cost += costTable[distanceLow[1].get_freq(distance & 0xF)];
			cost += rawBits * STRIDER_COST_PRECISION;
		}

		return cost;
	}

	LZStructure* priced_forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, const uint8_t* const blockLimit, BinaryMatchFinder* matchFinder,
		StriderOptimalParserState* parser, LZStructure* stream, const CompressorOptions& compressorOptions,
		const uint8_t* costTable, NibbleModel* literalRunLengthModel, size_t startingLiteralRunLength, size_t matchContext,
		NibbleModel* literalModel, size_t literalContextBitsShift, size_t positionContextBitMask, bool rawLiterals,
		size_t lastDistance, size_t* repOffsets, NibbleModel* distanceModel, NibbleModel* distanceLow, NibbleModel* matchLengthModel)  
	{
		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		for (size_t i = 1; i < compressorOptions.niceLength; i++)
			parser[i].sizeCost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;

		// While computing the cost of a new literal it is possible that due to substracting the previous run length
		// we get a negative number. To avoid that simply add some starting cost, instead of 0. This has no other effect on the parse
		parser[0].sizeCost = 256 * STRIDER_COST_PRECISION;
		parser[0].distance = lastDistance;
		for (size_t i = 0; i < 8; i++) { parser[0].repOffsets[i] = repOffsets[i]; }
		parser[0].literalRunLength = startingLiteralRunLength;
		parser[0].currentLiteralRunLengthCost =
			get_literal_run_cost(startingLiteralRunLength, reinterpret_cast<size_t>(input - startingLiteralRunLength) & positionContextBitMask,
				literalRunLengthModel, costTable);
		parser[0].matchContext = matchContext;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			StriderOptimalParserState* parserPosition = parser + position;
			parserPosition[compressorOptions.niceLength].sizeCost = UINT32_MAX;

			//Unpromising position
			if (parserPosition[0].sizeCost + 1 * STRIDER_COST_PRECISION > parserPosition[1].sizeCost) {
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions);
				continue;
			}

			const size_t positionContext = reinterpret_cast<size_t>(inputPosition) & positionContextBitMask;

			size_t literalCost;
			if (rawLiterals) 
				literalCost = 8 * STRIDER_COST_PRECISION;
			else {
				const size_t literal = *inputPosition;
				const size_t exclude = *(inputPosition - parserPosition->distance);
				NibbleModel* const literalModelTree = &literalModel[(((positionContext << 8) | inputPosition[-1]) >> literalContextBitsShift) * 64];
				size_t lowExclusionModel = parserPosition->literalRunLength == 0 ? 32 : 16;
				literalCost = costTable[literalModelTree[exclude >> 4].get_freq(literal >> 4)] +
					costTable[literalModelTree[(exclude >> 4) == (literal >> 4) ? lowExclusionModel | (exclude & 0xF) : 48 | (literal >> 4)].get_freq(literal & 0xF)];
			}

			size_t newLiteralRunLengthCost =
				get_literal_run_cost(parserPosition->literalRunLength + 1,
					reinterpret_cast<size_t>(inputPosition - parserPosition->literalRunLength) & positionContextBitMask,
					literalRunLengthModel, costTable);
			size_t thisLiteralSize = parserPosition->sizeCost + literalCost + newLiteralRunLengthCost - parserPosition->currentLiteralRunLengthCost;

			StriderOptimalParserState* nextPosition = parserPosition + 1;
			if (thisLiteralSize < nextPosition->sizeCost) {
				nextPosition->sizeCost = thisLiteralSize;
				nextPosition->distance = parserPosition->distance;
				nextPosition->matchLength = 0;
				memcpy(nextPosition->repOffsets, parserPosition->repOffsets, 8 * sizeof(uint32_t));
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
				nextPosition->currentLiteralRunLengthCost = newLiteralRunLengthCost;
				nextPosition->matchContext = parserPosition->matchContext;
			}

			size_t nextExpectedLength = 2;
			const size_t currentMatchContext = parserPosition->matchContext >> (parserPosition->literalRunLength > 0) * 2;  //After literal run;

			for (size_t i = 0; i < 8; i++) {
				size_t repMatchLength = test_match(inputPosition, inputPosition - parserPosition->repOffsets[i], blockLimit, 2);
				if (repMatchLength >= nextExpectedLength) {
					if (repMatchLength >= compressorOptions.niceLength / 2) {
						matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions);
						lastMatchLength = repMatchLength;
						lastMatchDistance = parserPosition->repOffsets[i];
						goto doBackwardParse;
					}

					nextExpectedLength = repMatchLength;
					const size_t offsetCost = costTable[distanceModel[currentMatchContext].get_freq(i)];
					const size_t lengthCost = get_match_length_cost(repMatchLength, positionContext, 0, matchLengthModel, costTable);
					const size_t nextLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + repMatchLength) & positionContextBitMask;
					const size_t literalRunLengthCost = costTable[literalRunLengthModel[nextLiteralRunPositionContext].get_freq(0)];
					const size_t matchSizeCost = parserPosition->sizeCost + lengthCost + offsetCost + literalRunLengthCost;

					nextPosition = parserPosition + repMatchLength;
					if (matchSizeCost < nextPosition->sizeCost) {
						size_t repDistance = parserPosition->repOffsets[i];
						nextPosition->sizeCost = matchSizeCost;
						nextPosition->matchLength = repMatchLength;
						nextPosition->distance = repDistance;
						nextPosition->repOffsets[0] = repDistance;
						memcpy(&nextPosition->repOffsets[1], parserPosition->repOffsets, i * sizeof(uint32_t));
						memcpy(&nextPosition->repOffsets[i + 1], &parserPosition->repOffsets[i + 1], (7 - i) * sizeof(uint32_t));
						nextPosition->literalRunLength = 0;
						nextPosition->currentLiteralRunLengthCost = literalRunLengthCost;
						nextPosition->matchContext = (currentMatchContext >> 2) | 4;  //After rep match
					}
				}
			}

			LZMatch matches[260];
			const LZMatch* matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart, limit,
				blockLimit, matches, nextExpectedLength + 1, compressorOptions);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZMatch* const longestMatch = matchesEnd - 1;
				//Match goes outside the buffer or is very long
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					goto doBackwardParse;
				}

				const size_t nextMatchContext = (currentMatchContext >> 2) | 8;  //After match

				for (const LZMatch* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					const size_t matchLengthContext = 1 + int_log2((size_t)matchIt->distance) / 8;
					const size_t lengthCost = get_match_length_cost(matchIt->length, positionContext, matchLengthContext, matchLengthModel, costTable);
					const size_t distanceCost = get_distance_cost(matchIt->distance, distanceModel, distanceLow, currentMatchContext, costTable);
					const size_t nextLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + matchIt->length) & positionContextBitMask;
					const size_t literalRunLengthCost = costTable[literalRunLengthModel[nextLiteralRunPositionContext].get_freq(0)];
					const size_t matchSizeCost = parserPosition->sizeCost + distanceCost + lengthCost + literalRunLengthCost;

					nextPosition = parserPosition + matchIt->length;
					if (matchSizeCost < nextPosition->sizeCost) {
						nextPosition->sizeCost = matchSizeCost;
						nextPosition->matchLength = matchIt->length;
						nextPosition->distance = matchIt->distance;
						memcpy(nextPosition->repOffsets, parserPosition->repOffsets, 6 * sizeof(uint32_t));
						nextPosition->repOffsets[6] = matchIt->distance;
						nextPosition->repOffsets[7] = parserPosition->repOffsets[6];
						nextPosition->literalRunLength = 0;
						nextPosition->currentLiteralRunLengthCost = literalRunLengthCost;
						nextPosition->matchContext = nextMatchContext;
					}
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const StriderOptimalParserState* backwardParse;
		const StriderOptimalParserState* const parseEnd = parser;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			stream->literalRunLength = 0;
			stream++;
			backwardParse = parser + position;

			const uint8_t* const updateEnd = input + position + std::min(lastMatchDistance, lastMatchLength);
			const uint8_t* inputPosition = input + position + 1;
			for (; inputPosition < updateEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions);
		}
		else {
			size_t bestPos = blockLength;
			size_t bestSize = parser[blockLength].sizeCost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				if (parser[blockLength + i].sizeCost <= bestSize + (blockLength + i - bestPos) * STRIDER_COST_PRECISION * 2) 
				{
					bestPos = blockLength + i;
					bestSize = parser[blockLength + i].sizeCost;
				}
			}
			backwardParse = parser + bestPos;
			stream->literalRunLength = 0;
			stream++;
		}

		while (backwardParse > parseEnd) {
			if (backwardParse->matchLength >= 1) {
				stream->matchDistance = backwardParse->distance;
				stream->matchLength = backwardParse->matchLength;
				stream->literalRunLength = 0;
				backwardParse -= stream->matchLength;
				stream++;
			}
			else {
				(stream - 1)->literalRunLength++;
				backwardParse--;
			}
		}

		return stream - 1;
	}

	LZStructure* multi_arrivals_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, const uint8_t* const blockLimit, BinaryMatchFinder* matchFinder,
		StriderOptimalParserState* parser, LZStructure* stream, const CompressorOptions& compressorOptions,
		const uint8_t* costTable, NibbleModel* literalRunLengthModel, size_t startingLiteralRunLength, size_t matchContext,
		NibbleModel* literalModel, size_t literalContextBitsShift, size_t positionContextBitMask, bool rawLiterals,
		size_t lastDistance, size_t* repOffsets, NibbleModel* distanceModel, NibbleModel* distanceLow, NibbleModel* matchLengthModel) 
	{
		//Limit the amount a match length can be reduced. Improves speed when there are long matches
		const int MAX_LENGTH_REDUCTION = 15; 
		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		for (size_t i = 0; i < (blockLength + compressorOptions.niceLength) * compressorOptions.maxArrivals; i++)
			parser[i].sizeCost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastMatchPath;

		// Fill first arrival of first position
		// While computing the cost of a new literal it is possible that due to substracting the previous run length
		// we get a negative number. To avoid that simply add some starting cost, instead of 0. This has no other effect on the parse
		parser[0].sizeCost = 256 * STRIDER_COST_PRECISION;
		for (size_t j = 0; j < 8; j++) { parser[0].repOffsets[j] = repOffsets[j]; }
		parser[0].literalRunLength = startingLiteralRunLength;
		parser[0].currentLiteralRunLengthCost =
			get_literal_run_cost(startingLiteralRunLength, reinterpret_cast<size_t>(input - startingLiteralRunLength) & positionContextBitMask,
				literalRunLengthModel, costTable);
		parser[0].distance = lastDistance;
		parser[0].matchContext = matchContext;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			StriderOptimalParserState* parserPosition = parser + position * compressorOptions.maxArrivals;
			const size_t positionContext = reinterpret_cast<size_t>(inputPosition) & positionContextBitMask;

			NibbleModel* const literalModelTree = &literalModel[(((positionContext << 8) | inputPosition[-1]) >> literalContextBitsShift) * 64];
			const size_t literal = *inputPosition;

			//Literal parsing
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				StriderOptimalParserState* currentArrival = parserPosition + i;

				size_t thisLiteralSize;
				if (!rawLiterals) {
					//Break literal parsing if current arrival has a very high cost. VERY noticeable speed improvement
					//Will also break if there are no more arrivals for this position
					if (currentArrival->sizeCost - compressorOptions.maxArrivals * STRIDER_COST_PRECISION * 3 / 2 >
						parserPosition[compressorOptions.maxArrivals].sizeCost)
						break;

					const size_t exclude = *(inputPosition - currentArrival->distance);
					//Force the literal after match to be different than the rep0 literal, so that the parse
					// only sends max lengths. The exception to this is the beginning of a block, since a match might have been cut.
					if (literal == exclude && currentArrival->literalRunLength == 0 && position != 0)
						continue;

					size_t lowExclusionModel = currentArrival->literalRunLength == 0 ? 32 : 16;
					thisLiteralSize = costTable[literalModelTree[exclude >> 4].get_freq(literal >> 4)] +
						costTable[literalModelTree[(exclude >> 4) == (literal >> 4) ? lowExclusionModel | (exclude & 0xF) : 48 | (literal >> 4)].get_freq(literal & 0xF)];
				}
				else {
					//Break literal parsing earlier with raw literals
					if (currentArrival->sizeCost - compressorOptions.maxArrivals * STRIDER_COST_PRECISION / 2 >
						parserPosition[compressorOptions.maxArrivals].sizeCost)
						break;
					thisLiteralSize = 8 * STRIDER_COST_PRECISION;
				}
				
				const size_t newLiteralRunLengthCost =
					get_literal_run_cost(currentArrival->literalRunLength + 1,
						reinterpret_cast<size_t>(inputPosition - currentArrival->literalRunLength) & positionContextBitMask,
						literalRunLengthModel, costTable);
				thisLiteralSize += currentArrival->sizeCost + newLiteralRunLengthCost - currentArrival->currentLiteralRunLengthCost;

				StriderOptimalParserState* arrivalIt = parserPosition + compressorOptions.maxArrivals;
				StriderOptimalParserState* const lastArrival = arrivalIt + compressorOptions.maxArrivals;

				for (; arrivalIt < lastArrival; arrivalIt++) {

					if (thisLiteralSize < arrivalIt->sizeCost) {

						std::copy_backward(arrivalIt, lastArrival - 1, lastArrival);

						arrivalIt->sizeCost = thisLiteralSize;
						arrivalIt->matchLength = 0;
						memcpy(arrivalIt->repOffsets, currentArrival->repOffsets, 8 * sizeof(uint32_t));
						arrivalIt->literalRunLength = currentArrival->literalRunLength + 1;
						arrivalIt->currentLiteralRunLengthCost = newLiteralRunLengthCost;
						arrivalIt->distance = currentArrival->distance;
						arrivalIt->matchContext = currentArrival->matchContext;
						arrivalIt->prevPath = i;
						break;
					}
				}
			}

			//Only try lengths that are at least this long
			size_t nextExpectedLength = 1;
			//Rep match parsing
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				StriderOptimalParserState* currentArrival = parserPosition + i;
				if (currentArrival->sizeCost - compressorOptions.maxArrivals * STRIDER_COST_PRECISION / 2 > 
					parserPosition[compressorOptions.maxArrivals].sizeCost)
					break;

				const size_t currentMatchContext = currentArrival->matchContext >> (currentArrival->literalRunLength > 0) * 2;
				const size_t nextMatchContext = (currentMatchContext >> 2) | 4;

				//Try to find rep offsets
				for (size_t j = 0; j < 8; j++) {

					size_t repMatchLength = test_match(inputPosition, inputPosition - currentArrival->repOffsets[j], blockLimit, 1);
					if (repMatchLength >= nextExpectedLength) {
						size_t repDistance = currentArrival->repOffsets[j];

						if (repMatchLength >= compressorOptions.niceLength / 2) {
							matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions);
							lastMatchLength = repMatchLength;
							lastMatchDistance = repDistance;
							lastMatchPath = i;
							goto doBackwardParse;
						}

						const size_t baseMatchCost = currentArrival->sizeCost + costTable[distanceModel[currentMatchContext].get_freq(j)];
						const size_t maxLengthReduction = repMatchLength - nextExpectedLength > MAX_LENGTH_REDUCTION ?
							repMatchLength - MAX_LENGTH_REDUCTION : nextExpectedLength;
						for (size_t length = repMatchLength; length >= maxLengthReduction; length--) {

							//Dont send rep0len1 like LZMA. The exclusion literal models this really well, so it is redundant.
							//Though there are some cases where it is better to send rep0len1. Needs more testing
							if (length == 1 && currentArrival->distance == repDistance)
								continue;

							const size_t lengthCost = get_match_length_cost(length, positionContext, 0, matchLengthModel, costTable);
							const size_t newLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + length) & positionContextBitMask;
							const size_t literalRunLengthCost = costTable[literalRunLengthModel[newLiteralRunPositionContext].get_freq(0)];
							const size_t matchSizeCost = baseMatchCost + lengthCost + literalRunLengthCost;

							StriderOptimalParserState* arrivalIt = parserPosition + length * compressorOptions.maxArrivals;
							StriderOptimalParserState* const lastArrival = arrivalIt + compressorOptions.maxArrivals;

							for (; arrivalIt < lastArrival; arrivalIt++) {

								if (matchSizeCost < arrivalIt->sizeCost) {

									//Find if an arrival with the same last offset as this already existed,
									// and if thats the case, remove it. This keeps more interesting offsets in the arrivals.
									StriderOptimalParserState* backwardCopyEnd = arrivalIt;
									for (; backwardCopyEnd < lastArrival; backwardCopyEnd++) {
										if (backwardCopyEnd->distance == repDistance) {
											backwardCopyEnd++;
											break;
										}
									}
									std::copy_backward(arrivalIt, backwardCopyEnd - 1, backwardCopyEnd);

									arrivalIt->sizeCost = matchSizeCost;
									arrivalIt->matchLength = length;
									arrivalIt->distance = repDistance;
									arrivalIt->repOffsets[0] = repDistance;
									memcpy(&arrivalIt->repOffsets[1], currentArrival->repOffsets, j * sizeof(uint32_t));
									memcpy(&arrivalIt->repOffsets[j + 1], &currentArrival->repOffsets[j + 1], (7 - j) * sizeof(uint32_t));
									arrivalIt->literalRunLength = 0;
									arrivalIt->currentLiteralRunLengthCost = literalRunLengthCost;
									arrivalIt->matchContext = nextMatchContext;
									arrivalIt->prevPath = i;
									break;
								}

								//If an arrival exists with the same distance as this one AND it has lower cost, simply skip
								//This will keep the arrivals filled with more interesting choices
								if (arrivalIt->distance == repDistance)
									break;
							}
						}

						nextExpectedLength = repMatchLength;
					}
				}
			}

			if (parserPosition[0].sizeCost - compressorOptions.maxArrivals * STRIDER_COST_PRECISION / 8 > parserPosition[compressorOptions.maxArrivals].sizeCost) {
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions);
				continue;
			}

			LZMatch matches[260];
			//Only try length 2+ for normal matches
			nextExpectedLength = std::max(nextExpectedLength, (size_t)2);
			const LZMatch* matchesEnd =
				matchFinder->find_matches_and_update(inputPosition, inputStart, limit, blockLimit,
					matches, nextExpectedLength, compressorOptions);

			//At least one match was found
			if (matchesEnd != matches) {

				//The last match should be the longest
				const LZMatch* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastMatchPath = 0;
					goto doBackwardParse;
				}

				size_t previousMatchLength = nextExpectedLength;
				const size_t currentMatchContext = parserPosition->matchContext >> (parserPosition->literalRunLength > 0) * 2;  //After literal run
				const size_t nextMatchContext = (currentMatchContext >> 2) | 8;  //After match

				for (const LZMatch* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->repOffsets[0] || matchIt->distance == parserPosition->repOffsets[1] ||
						matchIt->distance == parserPosition->repOffsets[2] || matchIt->distance == parserPosition->repOffsets[3] ||
						matchIt->distance == parserPosition->repOffsets[4] || matchIt->distance == parserPosition->repOffsets[5] ||
						matchIt->distance == parserPosition->repOffsets[6] || matchIt->distance == parserPosition->repOffsets[7])
						continue;

					const size_t matchLengthContext = 1 + int_log2((size_t)matchIt->distance) / 8;
					const size_t baseMatchCost = parserPosition->sizeCost + get_distance_cost(matchIt->distance, distanceModel, distanceLow, currentMatchContext, costTable);

					size_t matchLengthReductionLimit = matchIt->length == previousMatchLength ?
						previousMatchLength : previousMatchLength + 1;
					if (matchIt->length - matchLengthReductionLimit > MAX_LENGTH_REDUCTION)
						matchLengthReductionLimit = matchIt->length - MAX_LENGTH_REDUCTION;
					previousMatchLength = matchIt->length;

					//Start search at the highest length. Stop when we reach the limit,
					for (size_t length = matchIt->length; length >= matchLengthReductionLimit; length--) {

						const size_t lengthCost = get_match_length_cost(length, positionContext, matchLengthContext, matchLengthModel, costTable);
						const size_t newLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + length) & positionContextBitMask;
						const size_t literalRunLengthCost = costTable[literalRunLengthModel[newLiteralRunPositionContext].get_freq(0)];
						const size_t matchSizeCost = baseMatchCost + lengthCost + literalRunLengthCost;

						StriderOptimalParserState* arrivalIt = parserPosition + length * compressorOptions.maxArrivals;
						StriderOptimalParserState* const lastArrival = arrivalIt + compressorOptions.maxArrivals;

						for (; arrivalIt < lastArrival; arrivalIt++) {

							if (matchSizeCost < arrivalIt->sizeCost) {

								StriderOptimalParserState* backwardCopyEnd = arrivalIt;
								for (; backwardCopyEnd < lastArrival; backwardCopyEnd++) {
									if (backwardCopyEnd->distance == matchIt->distance) {
										backwardCopyEnd++;
										break;
									}
								}
								std::copy_backward(arrivalIt, backwardCopyEnd - 1, backwardCopyEnd);

								arrivalIt->sizeCost = matchSizeCost;
								arrivalIt->matchLength = length;
								arrivalIt->distance = matchIt->distance;
								memcpy(arrivalIt->repOffsets, parserPosition->repOffsets, 6 * sizeof(uint32_t));
								arrivalIt->repOffsets[6] = matchIt->distance;
								arrivalIt->repOffsets[7] = parserPosition->repOffsets[6];
								arrivalIt->literalRunLength = 0;
								arrivalIt->currentLiteralRunLengthCost = literalRunLengthCost;
								arrivalIt->matchContext = nextMatchContext;
								arrivalIt->prevPath = 0;
								break;
							}

							if (arrivalIt->distance == matchIt->distance)
								break;
						}
					}
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const StriderOptimalParserState* backwardParse;
		const StriderOptimalParserState* const parseEnd = parser;
		size_t path = 0;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			stream->literalRunLength = 0;
			stream++;
			backwardParse = parser + position * compressorOptions.maxArrivals;
			path = lastMatchPath;

			const uint8_t* const updateEnd = input + position + std::min(lastMatchDistance, lastMatchLength);
			const uint8_t* inputPosition = input + position + 1;
			for (; inputPosition < updateEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions);
		}
		else {
			size_t bestPos = blockLength;
			size_t bestSize = parser[blockLength * compressorOptions.maxArrivals].sizeCost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				if (parser[(blockLength + i) * compressorOptions.maxArrivals].sizeCost <= 
					bestSize + (blockLength + i - bestPos) * STRIDER_COST_PRECISION * 2)
				{
					bestPos = blockLength + i;
					bestSize = parser[(blockLength + i) * compressorOptions.maxArrivals].sizeCost;
				}
			}
			backwardParse = parser + bestPos * compressorOptions.maxArrivals;
			stream->literalRunLength = 0;
			stream++;
		}

		while (backwardParse > parseEnd) {
			if (backwardParse[path].matchLength >= 1) {
				stream->matchDistance = backwardParse[path].distance;
				stream->matchLength = backwardParse[path].matchLength;
				stream->literalRunLength = 0;
				path = backwardParse[path].prevPath;
				backwardParse -= stream->matchLength * compressorOptions.maxArrivals;
				stream++;
			}
			else {
				(stream - 1)->literalRunLength++;
				path = backwardParse[path].prevPath;
				backwardParse -= compressorOptions.maxArrivals;
			}
		}
		return stream - 1;
	}

	void generate_cost_table(uint8_t* costTable) {
		/*
		We calculate the cost with this
		int values[MODEL_SCALE];
		for (int freq = 0; freq < MODEL_SCALE; freq++) {
			if (freq == 0) { values[0] = 255; }
			else
				values[freq] = std::min(int(1 + (MODEL_PRECISION_BITS - log2((float)freq)) * 32), 255);
		}
		This will have a lot of repeated values, which we can encode using run length encoding
		int pos = MODEL_SCALE - 1;
		int length = 0;
		for (int i = 1; i <= 255; ) {
			if (pos < 0 || values[pos] != i) {
				std::cout << length << "," << (i % 16 == 0 ? "\n" : "");
				length = 0;
				i++;
			}
			else {
				length++;
				pos--;
			}
		}
		*/
		/*Capping the cost to 8 bits actually improves the parsing a bit*/
		const uint16_t RLE[256] = {
			0,351,343,336,329,322,315,309,301,295,289,283,277,270,265,260,253,
			249,242,238,233,227,223,218,214,208,205,199,196,191,188,183,179,
			176,172,168,164,161,158,154,151,147,145,141,139,135,132,130,127,
			124,121,119,117,113,112,109,107,104,102,100,98,95,94,92,89,
			88,86,84,82,81,79,77,75,74,72,71,69,68,66,65,63,
			62,61,59,59,56,56,55,53,52,51,50,49,48,47,46,44,
			44,43,42,41,41,39,39,37,37,36,36,34,34,33,33,31,
			31,31,29,30,28,28,27,27,26,25,25,25,24,23,23,22,
			22,22,21,20,21,19,20,18,19,18,18,17,17,16,17,15,
			16,15,15,15,14,14,13,14,13,12,13,12,12,12,11,11,
			11,11,11,10,10,10,10,9,9,9,9,9,8,8,9,7,
			8,8,7,8,7,7,6,7,7,6,6,6,6,6,6,5,
			6,5,6,5,5,5,5,4,5,4,5,4,4,4,5,3,
			4,4,4,4,3,4,3,3,4,3,3,3,3,3,3,2,
			3,3,3,2,3,2,3,2,2,2,3,2,2,2,2,2,
			2,2,2,2,1,2,2,1,2,2,1,2,1,2,67
		};
		int pos = 0;
		for (int i = 254; i >= 0; i--) {
			memset(costTable + pos, i, RLE[i]);
			pos += RLE[i];
		}
	}

	size_t compress_optimal(const uint8_t* input, const size_t size, uint8_t* output, RansEncoder& encoder,
		const int windowLog, const CompressorOptions& compressorOptions, ProgressCallback* progress) 
	{
		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. Its progress will be reported at the end
		*output++ = *input++;

		DataDetector dataDetector;
		if (dataDetector.calculate_pb(input, size - STRIDER_LAST_BYTES - 1))
			return -1;

		//Model stuff
		bool resetAllModels = true;
		NibbleModel literalRunLengthModel[17];
		NibbleModel* literalModel = nullptr;
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthModel[194];

		size_t matchContext = 0;
		size_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		size_t positionContextBitMask = -1;
		bool rawLiterals = false;
		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

		HashTableMatchFinder hashMatchFinder;
		BinaryMatchFinder binaryMatchFinder;
		char* parser = nullptr;
		LZStructure* stream = nullptr;
		uint8_t* costTable = nullptr;
		try {
			if (compressorOptions.parser == OPTIMAL1) {
				hashMatchFinder.init(windowLog, compressorOptions);
				parser = (char*)(new StriderFastOptimalState[compressorOptions.optimalBlockSize + compressorOptions.niceLength]);
			}
			else {
				binaryMatchFinder.init(size, windowLog, compressorOptions);
				if (compressorOptions.parser == OPTIMAL2)
					parser = (char*)(new StriderOptimalParserState[compressorOptions.optimalBlockSize + compressorOptions.niceLength]);
				else
					parser = (char*)(new StriderOptimalParserState[(compressorOptions.optimalBlockSize + compressorOptions.niceLength) * compressorOptions.maxArrivals]);
			}
			stream = new LZStructure[compressorOptions.optimalBlockSize];
			costTable = new uint8_t[MODEL_SCALE];
		}
		catch (const std::bad_alloc& e) {
			delete[] parser;
			delete[] stream;
			delete[] costTable;
			return -1;
		}

		generate_cost_table(costTable);
		if (dataDetector.calculate_lc(input, size, costTable)) {
			delete[] parser;
			delete[] stream;
			delete[] costTable;
			return -1;
		}

		for (; input < compressionLimit; ) {

			uint8_t* const compressedBlockStart = output;
			const uint8_t* const thisBlockStart = input;

			bool newRawLiterals;
			size_t thisBlockSize, newPositionContextBitMask, newLiteralContextBitShift;
			dataDetector.get_block_info(input, &thisBlockSize, &newPositionContextBitMask,
				&newLiteralContextBitShift, &newRawLiterals);
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//The size of the literal model has been modified
			if (literalContextBitsShift != newLiteralContextBitShift ||
				positionContextBitMask != newPositionContextBitMask ||
				rawLiterals != newRawLiterals)
			{
				delete[] literalModel;
				literalModel = nullptr;
				if (!newRawLiterals) {
					try {
						literalModel = new NibbleModel[((64 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
					}
					catch (const std::bad_alloc& e) {
						return -1;
					}
				}
			}

			bool resetLiteralModel = literalContextBitsShift != newLiteralContextBitShift ||
				positionContextBitMask != newPositionContextBitMask || rawLiterals != newRawLiterals;
			bool resetLengthModel = positionContextBitMask != newPositionContextBitMask;
			reset_models(literalRunLengthModel, &matchContext, literalModel, newLiteralContextBitShift,
				newPositionContextBitMask, newRawLiterals, &distance, repOffsets, distanceModel,
				distanceLow, matchLengthModel, resetAllModels, resetLiteralModel, resetLengthModel);
			literalContextBitsShift = newLiteralContextBitShift;
			positionContextBitMask = newPositionContextBitMask;
			rawLiterals = newRawLiterals;

			store_block_header(thisBlockSize, positionContextBitMask, literalContextBitsShift, rawLiterals, resetAllModels, 0, output);
			encoder.start_rans();

			resetAllModels = false;
			const uint8_t* literalRunStart = input;

			for (; input < thisBlockEnd; ) {

				LZStructure* streamIt;
				if (compressorOptions.parser >= OPTIMAL3) {
					streamIt = multi_arrivals_parse(input, inputStart, compressionLimit, thisBlockEnd, &binaryMatchFinder,
						(StriderOptimalParserState*)parser, stream, compressorOptions, costTable, literalRunLengthModel,
						input - literalRunStart, matchContext, literalModel, literalContextBitsShift, positionContextBitMask,
						rawLiterals, distance, repOffsets, distanceModel, distanceLow, matchLengthModel);
				}
				else if (compressorOptions.parser >= OPTIMAL2) {
					streamIt = priced_forward_optimal_parse(input, inputStart, compressionLimit, thisBlockEnd, &binaryMatchFinder,
						(StriderOptimalParserState*)parser, stream, compressorOptions, costTable, literalRunLengthModel,
						input - literalRunStart, matchContext, literalModel, literalContextBitsShift, positionContextBitMask,
						rawLiterals, distance, repOffsets, distanceModel, distanceLow, matchLengthModel);
				}
				else {
					streamIt = forward_optimal_parse(input, inputStart, thisBlockEnd, &hashMatchFinder,
						(StriderFastOptimalState*)parser, stream, repOffsets, compressorOptions);
				}

				//Main compression loop
				while (true) {
					input += streamIt->literalRunLength;

					if (streamIt == stream)
						break;

					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;

					encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext, rawLiterals);

					size_t matchLength = streamIt->matchLength;
					distance = streamIt->matchDistance;
					input += matchLength;
					literalRunStart = input;
					//Output the match
					encode_match(&encoder, &matchContext, positionContext, repOffsets,
						distanceModel, distanceLow, matchLengthModel, matchLength, distance);

					streamIt--;
				}
			}

			size_t positionContext;
			size_t literalRunLength = input - literalRunStart;
			encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchContext,
				literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext, rawLiterals);

			size_t compressedBlockSize = encoder.end_rans(output);
			output += compressedBlockSize;

			//If the algorithm ends up expanding the data, store it uncompressed and reset all models
			if (compressedBlockSize >= thisBlockSize) {
				resetAllModels = true;
				output = compressedBlockStart;
				store_block_header(thisBlockSize, 0, 0, 0, 0, 1, output);
				memcpy(output, thisBlockStart, thisBlockSize);
				input = thisBlockStart + thisBlockSize;
				output += thisBlockSize;
			}

			if (progress->progress(input - inputStart, output - outputStart)) {
				delete[] parser;
				delete[] stream;
				delete[] costTable;
				delete[] literalModel;
				return 0;
			}
		}

		memcpy(output, input, STRIDER_LAST_BYTES);
		progress->progress(input - inputStart + STRIDER_LAST_BYTES, output - outputStart + STRIDER_LAST_BYTES);
		delete[] parser;
		delete[] stream;
		delete[] costTable;
		delete[] literalModel;

		return output - outputStart + STRIDER_LAST_BYTES;
	}

	const int NOT_USED = -1;
	const CompressorOptions striderCompressorLevels[] = {
		//      Parser        Hash log     Elements per hash     Nice length     Block size     Max arrivals
			{ LAZY1        ,     17     ,      NOT_USED       ,   NOT_USED   ,    NOT_USED    ,   NOT_USED   },
			{ LAZY2        ,     17     ,      NOT_USED       ,      32      ,    NOT_USED    ,   NOT_USED   },
			{ LAZY3        ,     17     ,          1          ,      32      ,    NOT_USED    ,   NOT_USED   },
			{ OPTIMAL1     ,     17     ,          1          ,      32      ,      1024      ,   NOT_USED   },
			{ OPTIMAL1     ,     18     ,          2          ,      32      ,      1024      ,   NOT_USED   },
			{ OPTIMAL1     ,     19     ,          3          ,      64      ,      1024      ,   NOT_USED   },
			{ OPTIMAL2     ,     22     ,          5          ,      128     ,      2048      ,   NOT_USED   },
			{ OPTIMAL3     ,     23     ,          5          ,      128     ,      2048      ,       2      },
			{ OPTIMAL3     ,     25     ,          6          ,      256     ,      2048      ,       4      },
			{ OPTIMAL4     ,     27     ,          7          ,      512     ,      2048      ,       8      },
			{ OPTIMAL4     ,     29     ,          8          ,      1024    ,      2048      ,       16     },
	};

	size_t compress(const uint8_t* input, const size_t size, uint8_t* output, int level, ProgressCallback* progress) {

		if (level < 0)
			level = 0;
		if (level > 10)
			level = 10;
		int windowLog = int_log2(size - 1) + 1;
		if (windowLog < 6)
			windowLog = 6;

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		if (size <= STRIDER_LAST_BYTES + 1) {
			memcpy(output, input, size);
			progress->progress(size, size);
			return size;
		}

		RansEncoder ransEncoder;
		if (ransEncoder.initialize_rans_encoder(std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size),
			std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 2.5 + 16))
			return -1;

		if (striderCompressorLevels[level].parser == LAZY1)
			return compress_greedy(input, size, output, ransEncoder, windowLog, striderCompressorLevels[level], progress);
		if (striderCompressorLevels[level].parser <= LAZY3)
			return compress_lazy(input, size, output, ransEncoder, windowLog, striderCompressorLevels[level], progress);
		return compress_optimal(input, size, output, ransEncoder, windowLog, striderCompressorLevels[level], progress);
	}

	size_t compress_bound(const size_t size) {
		return size + size / 1024 + 8;
	}

	size_t parser_memory_estimator(const size_t size, const int level, const int windowLog) {
		if (striderCompressorLevels[level].parser == LAZY1)
			return sizeof(uint32_t) << std::min(striderCompressorLevels[level].maxHashLog, windowLog - 3);
		if (striderCompressorLevels[level].parser == LAZY2)
			return sizeof(uint32_t) << std::min(striderCompressorLevels[level].maxHashLog, windowLog - 3) << 1;
		if (striderCompressorLevels[level].parser == LAZY3)
			//Lazy extra uses 2 tables
			return sizeof(uint32_t) << std::min(striderCompressorLevels[level].maxHashLog, windowLog - 3)
			<< striderCompressorLevels[level].maxElementsLog << 1;
		if (striderCompressorLevels[level].parser == OPTIMAL1) {
			const int hashLog = std::min(striderCompressorLevels[level].maxHashLog, windowLog - 3);
			size_t memory = sizeof(uint32_t) << std::min(hashLog, 12);  //hash 3 table
			memory += sizeof(uint32_t) << hashLog << striderCompressorLevels[level].maxElementsLog << 1;  //hash 4 and hash 8 tables
			memory += sizeof(StriderFastOptimalState) * (striderCompressorLevels[level].optimalBlockSize + striderCompressorLevels[level].niceLength);
			memory += sizeof(LZStructure) * striderCompressorLevels[level].optimalBlockSize;
			return memory;
		}
		size_t memory = sizeof(uint32_t) << std::min(20, windowLog - 3);  //binary node lookup
		memory += sizeof(uint32_t) << std::min(12, windowLog - 3);  //hash 2/3 table
		const size_t binaryTreeWindow = (size_t)1 << std::min(striderCompressorLevels[level].maxHashLog, windowLog);
		if (size < binaryTreeWindow)
			memory += sizeof(uint32_t) * 2 * size;  //binary tree
		else {
			memory += sizeof(uint32_t) * 2 * binaryTreeWindow;  //binary tree
			if (windowLog > striderCompressorLevels[level].maxHashLog)
				memory += sizeof(uint32_t) << std::max(1, (int)int_log2(std::min(size, (size_t)1 << windowLog) - binaryTreeWindow) - 3);  //hash 16 table
		}
		if (striderCompressorLevels[level].parser == OPTIMAL2)
			memory += sizeof(StriderOptimalParserState) * (striderCompressorLevels[level].optimalBlockSize + striderCompressorLevels[level].niceLength);
		else
			memory += sizeof(StriderOptimalParserState) * (striderCompressorLevels[level].optimalBlockSize + striderCompressorLevels[level].niceLength) * striderCompressorLevels[level].maxArrivals;
		memory += sizeof(LZStructure) * striderCompressorLevels[level].optimalBlockSize;
		return memory;
	}

	size_t estimate_memory(const size_t size, int level) {

		if (level < 0)
			level = 0;
		if (level > 10)
			level = 10;
		int windowLog = int_log2(size - 1) + 1;
		if (windowLog < 6)
			windowLog = 6;

		if (size <= STRIDER_LAST_BYTES + 1)
			return 0;

		size_t blockSize = std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size);
		size_t memory = blockSize;   //stream buffer
		//symbol buffer
		memory += (blockSize * 2.5 + 16) * sizeof(uint32_t);
		memory += sizeof(NibbleModel) * 64 * 256;  //literal model, the encoder will not use lc + pb > 8
		if (striderCompressorLevels[level].parser >= OPTIMAL1)
			memory += MODEL_SCALE * sizeof(uint8_t);  //cost table

		return memory + parser_memory_estimator(size, level, windowLog);
	}

	class RansDecoder {
		uint32_t stateA;
		uint32_t stateB;

	public:
		RansDecoder() {}
		~RansDecoder() {}

		//rans block functions
		void start_rans(const uint8_t*& compressed) {
			stateA = read_uint32le(compressed + 0);
			stateB = read_uint32le(compressed + 4);
			compressed += 8;
		}
		FORCE_INLINE void normalize(const uint8_t*& compressed) {
			bool renormalize = stateB < RANS_NORMALIZATION_INTERVAL;
			uint32_t newStateB = (stateB << 16) | read_uint16le(compressed);
			stateB = (0 - renormalize & (newStateB ^ stateB)) ^ stateB;  //Hope the compiler replaces this with a cmov or equivalent
			compressed += renormalize << 1;
		}
		FORCE_INLINE size_t decode_raw_bits(size_t nBits, const uint8_t*& compressed) {

			size_t symbol = 0;
			while (nBits) {
				size_t inBits = std::min(nBits, (size_t)16);
				symbol <<= inBits;
				std::swap(stateA, stateB);
				symbol |= stateB & (1 << inBits) - 1;
				stateB >>= inBits;
				normalize(compressed);
				nBits -= inBits;
			}
			return symbol;
		}
		//Only for nBits <= 16, but runs faster  
		//Used in literal run length and match length
		FORCE_INLINE size_t decode_raw_bits_short(const size_t nBits, const uint8_t*& compressed) {

			if (nBits == 0)
				return 0;
			std::swap(stateA, stateB);
			size_t symbol = stateB & (1 << nBits) - 1;
			stateB >>= nBits;
			normalize(compressed);
			return symbol;
		}
		//Decodes a 16 bit number. Very fast. Used for raw literal decoding.
		FORCE_INLINE size_t decode_short(const uint8_t*& compressed) {

			std::swap(stateA, stateB);
			size_t symbol = stateB & 0xFFFF;
			//Unconditional renormalize
			stateB = (stateB & 0xFFFF0000) | read_uint16le(compressed);
			compressed += 2;
			return symbol;
		}
		FORCE_INLINE size_t decode_nibble(NibbleModel* model, const uint8_t*& compressed) {
			std::swap(stateA, stateB);
			const size_t stateLow = stateB & MODEL_BIT_MASK;
			size_t freq, low;
			size_t symbol = model->decode_symbol(stateLow, &low, &freq);
			stateB = freq * (stateB >> MODEL_PRECISION_BITS) + stateLow - low;
			normalize(compressed);
			return symbol;
		}
	};

	const int8_t inc32table[16] = { 0, 1, 2, 1, 0,  4,  4,  4, 4, 4, 4, 4, 4, 4, 4, 4 };
	const int8_t inc64table[16] = { 0, 0, 0, 1, 4, -1, -2, -3, 4, 4, 4, 4, 4, 4, 4, 4 };

	//Optimized for long matches
	FORCE_INLINE void copy_match(uint8_t*& dst, const uint8_t*& match, const uint8_t* const copyEnd, const size_t offset) {

		//If the offset is big enough we can perform a faster copy
		if (offset >= 16) {
			do {
				memcpy(dst, match, 16);
				memcpy(dst + 16, match + 16, 16);
				match += 32;
				dst += 32;
			} while (dst < copyEnd);
		}
		//Else it is a run-length type match
		else {

			dst[0] = match[0];
			dst[1] = match[1];
			dst[2] = match[2];
			dst[3] = match[3];
			match += inc32table[offset];
			memcpy(dst + 4, match, 4);
			match += inc64table[offset];
			memcpy(dst + 8, match, 8);

			dst += 16;
			match += 8;
			do {
				memcpy(dst, match, 8);
				memcpy(dst + 8, match + 8, 8);
				match += 16;
				dst += 16;
			} while (dst < copyEnd);
		}
	}

	int decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed,
		const size_t decompressedSize, ProgressCallback* progress) {

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		if (decompressedSize <= STRIDER_LAST_BYTES + 1) {
			if (compressedSize < decompressedSize)
				return -1;
			memcpy(decompressed, compressed, decompressedSize);
			progress->progress(decompressedSize, decompressedSize);
			return 0;
		}

		if (compressedSize < STRIDER_LAST_BYTES + 1)
			return -1;

		RansDecoder ransDecoder;
		const uint8_t* const decompressedStart = decompressed;
		const uint8_t* const decompressedEnd = decompressed + decompressedSize - STRIDER_LAST_BYTES;
		const uint8_t* const compressedEnd = compressed + compressedSize - STRIDER_LAST_BYTES;
		//First byte is uncompressed
		*decompressed++ = *compressed++;

		//Model stuff
		NibbleModel literalRunLengthModel[17];
		NibbleModel* literalModel = nullptr;  //Also indicates whether this is the first rans block
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthModel[194];

		size_t matchContext;
		size_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		size_t positionContextBitMask = -1;
		bool rawLiterals = false;
		size_t distance = 1;  //Also stores the last used distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };
		//Optimization for many offsets from https://fgiesen.wordpress.com/2016/03/07/repeated-match-offsets-in-bitknit/
		size_t repState = 0x76543210;

		while (decompressed < decompressedEnd) {

			const size_t thisBlockSize = read_uint16le(compressed) + 1;
			const size_t blockMetadata = compressed[2];
			compressed += 3;
			const uint8_t* const thisBlockEnd = decompressed + thisBlockSize;

			if (compressed > compressedEnd || decompressedEnd - decompressed < thisBlockSize) {
				delete[] literalModel;
				return -1;
			}

			//Uncompressed data
			if (blockMetadata & 0x80) {

				if (compressedEnd - compressed < thisBlockSize) {
					delete[] literalModel;
					return -1;
				}
				memcpy(decompressed, compressed, thisBlockSize);
				decompressed += thisBlockSize;
				compressed += thisBlockSize;
			}
			//Standard LZ+rANS block
			else {
				//Either we explicitly have to reset models, or this is the first block and they must be initialized
				bool resetAllModels = (blockMetadata >> 6) || (decompressed == decompressedStart + 1);
				bool newRawLiterals = (blockMetadata & 0xF) == 0;
				size_t newLiteralContextBitShift, newPositionContextBitMask;
				if (newRawLiterals) {
					newLiteralContextBitShift = 8;
					newPositionContextBitMask = 0;
				}
				else {
					newLiteralContextBitShift = (blockMetadata % 16 - 1) % 3 * 4;
					newPositionContextBitMask = (1 << (blockMetadata % 16 - 1) / 3) - 1;
				}

				//The size of the literal model has been modified
				if (literalContextBitsShift != newLiteralContextBitShift || 
					positionContextBitMask != newPositionContextBitMask ||
					rawLiterals != newRawLiterals) 
				{
					delete[] literalModel;
					literalModel = nullptr;
					if (!newRawLiterals) {
						try {
							literalModel = new NibbleModel[((64 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
						}
						catch (const std::bad_alloc& e) {
							return -1;
						}
					}
				}

				bool resetLiteralModel = literalContextBitsShift != newLiteralContextBitShift ||
					positionContextBitMask != newPositionContextBitMask || rawLiterals != newRawLiterals;
				bool resetLengthModel = positionContextBitMask != newPositionContextBitMask;
				reset_models(literalRunLengthModel, &matchContext, literalModel, newLiteralContextBitShift,
					newPositionContextBitMask, newRawLiterals, &distance, repOffsets, distanceModel,
					distanceLow, matchLengthModel, resetAllModels, resetLiteralModel, resetLengthModel);
				literalContextBitsShift = newLiteralContextBitShift;
				positionContextBitMask = newPositionContextBitMask;
				rawLiterals = newRawLiterals;

				ransDecoder.start_rans(compressed);
				if (unlikely(compressed > compressedEnd)) {
					delete[] literalModel;
					return -1;
				}

				while (true) {

					size_t positionContext = reinterpret_cast<size_t>(decompressed) & positionContextBitMask;

					size_t literalRunLength = ransDecoder.decode_nibble(&literalRunLengthModel[positionContext], compressed);
					if (literalRunLength == 15) {
						size_t symbol = ransDecoder.decode_nibble(&literalRunLengthModel[16], compressed);
						literalRunLength = (1 << symbol) | ransDecoder.decode_raw_bits_short(symbol, compressed);
						literalRunLength += 14;

						if (unlikely(thisBlockEnd - decompressed < literalRunLength)) {
							delete[] literalModel;
							return -1;
						}
					}

					//A symbol can have a minimum frequency of 32/32768, that is, its decoding will consume at most 10 bits.
					//As both states might be just before renormalization interval, we should be able to read at least 448 bits
					// before having to check again for the buffer overflow.
					//A match can consume, at most, 137 bits plus 20 for a long literal run, 
					// plus 10 for the first literal (total = 167)
					if (unlikely(compressed > compressedEnd)) {
						delete[] literalModel;
						return -1;
					}

					if (literalRunLength) {
						matchContext >>= 2;
						const uint8_t* literalRunEnd = decompressed + literalRunLength;
						if (rawLiterals) {
							while (decompressed + 1 < literalRunEnd) {
								write_uint16le(decompressed, ransDecoder.decode_short(compressed));
								decompressed += 2;
								if (compressed > compressedEnd) {
									delete[] literalModel;
									return -1;
								}
							}
							if (decompressed != literalRunEnd)
								*decompressed++ = ransDecoder.decode_raw_bits_short(8, compressed);
						}
						else {
							if (literalContextBitsShift == 8) {
								
								size_t exclude = *(decompressed - distance);
								size_t excludeHigh = exclude >> 4;
								size_t excludeLow = exclude & 0xF;

								NibbleModel* literalModelTree =
									&literalModel[positionContext * 64];

								size_t literal = ransDecoder.decode_nibble(&literalModelTree[excludeHigh], compressed);
								literal = (literal << 4) | ransDecoder.decode_nibble
									(&literalModelTree[literal == excludeHigh ? 32 | excludeLow : 48 | literal], compressed);

								*decompressed++ = literal;
								positionContext = reinterpret_cast<size_t>(decompressed) & positionContextBitMask;

								while (decompressed != literalRunEnd) {
									exclude = *(decompressed - distance);
									excludeHigh = exclude >> 4;
									excludeLow = exclude & 0xF;

									literalModelTree =
										&literalModel[positionContext * 64];

									literal = ransDecoder.decode_nibble(&literalModelTree[excludeHigh], compressed);
									literal = (literal << 4) | ransDecoder.decode_nibble
										(&literalModelTree[literal == excludeHigh ? 16 | excludeLow : 48 | literal], compressed);

									*decompressed++ = literal;
									positionContext = reinterpret_cast<size_t>(decompressed) & positionContextBitMask;

									//Check overflow for every decoded literal
									if (unlikely(compressed > compressedEnd)) {
										delete[] literalModel;
										return -1;
									}
								}
							}
							else {
								//The literal we just have decoded is the previous byte
								size_t literal = decompressed[-1];

								size_t exclude = *(decompressed - distance);
								size_t excludeHigh = exclude >> 4;
								size_t excludeLow = exclude & 0xF;

								NibbleModel* literalModelTree =
									&literalModel[(((positionContext << 8) | literal) >> literalContextBitsShift) * 64];

								literal = ransDecoder.decode_nibble(&literalModelTree[excludeHigh], compressed);
								literal = (literal << 4) | ransDecoder.decode_nibble
									(&literalModelTree[literal == excludeHigh ? 32 | excludeLow : 48 | literal], compressed);

								*decompressed++ = literal;
								positionContext = reinterpret_cast<size_t>(decompressed) & positionContextBitMask;

								while (decompressed != literalRunEnd) {
									exclude = *(decompressed - distance);
									excludeHigh = exclude >> 4;
									excludeLow = exclude & 0xF;

									literalModelTree =
										&literalModel[(((positionContext << 8) | literal) >> literalContextBitsShift) * 64];

									literal = ransDecoder.decode_nibble(&literalModelTree[excludeHigh], compressed);
									literal = (literal << 4) | ransDecoder.decode_nibble
										(&literalModelTree[literal == excludeHigh ? 16 | excludeLow : 48 | literal], compressed);

									*decompressed++ = literal;
									positionContext = reinterpret_cast<size_t>(decompressed) & positionContextBitMask;

									//Check overflow for every decoded literal
									if (unlikely(compressed > compressedEnd)) {
										delete[] literalModel;
										return -1;
									}
								}
							}
						}
					}

					//This is for the usual case where literal run length < 15 and match length < 32. 
					//Together, they do not overflow the buffer, so a single check here is enough.
					if (unlikely(decompressed >= thisBlockEnd)) {
						if (decompressed > thisBlockEnd) {
							delete[] literalModel;
							return -1;
						}
						break;
					}

					size_t distanceToken = ransDecoder.decode_nibble(&distanceModel[matchContext], compressed);
					matchContext >>= 2;  //Remove the last event

					size_t matchLengthContext;

					if (distanceToken < 8) {

						size_t repIndex = distanceToken * 4;
						size_t slot = (repState >> repIndex) & 0xF;
						distance = repOffsets[slot];

						size_t movedRepState = (repState << 4) + slot;
						size_t mask = ~0xF << repIndex;
						repState = (repState & mask) | (movedRepState & ~mask);

						matchContext |= 4;
						matchLengthContext = 0;
					}
					else {
						if (distanceToken == 8) {
							distanceToken = ransDecoder.decode_nibble(&distanceModel[16], compressed);
							distance = (distanceToken << 4) | ransDecoder.decode_nibble(&distanceLow[distanceToken != 0], compressed);
							matchLengthContext = 1;
						}
						else {
							distanceToken -= 8;
							matchLengthContext = 1 + distanceToken;
							distanceToken = (distanceToken << 4) | ransDecoder.decode_nibble(&distanceModel[16 | distanceToken], compressed);
							const size_t rawBits = distanceToken / 2 - 5;
							distance = ((size_t)2 | (distanceToken & 1)) << rawBits;
							distance = (distance | ransDecoder.decode_raw_bits(rawBits, compressed)) << 4;
							distance |= ransDecoder.decode_nibble(&distanceLow[1], compressed);
						}
						matchContext |= 8;
						distance++;

						if (unlikely(decompressed - decompressedStart < distance)) {
							delete[] literalModel;
							return -1;
						}

						size_t last = (repState >> 28) & 0xF;
						size_t secondLast = (repState >> 24) & 0xF;
						repOffsets[last] = repOffsets[secondLast];
						repOffsets[secondLast] = distance;
					}
					const uint8_t* match = decompressed - distance;

					size_t matchLength = STRIDER_MIN_LENGTH
						+ ransDecoder.decode_nibble(&matchLengthModel[matchLengthContext * 16 | positionContext], compressed);
					if (matchLength == 16) {
						matchLengthContext = matchLengthContext != 0;
						matchLength +=
							ransDecoder.decode_nibble(&matchLengthModel[144 + matchLengthContext * 16 | positionContext], compressed);
						if (matchLength == 31) {
							size_t lowBits = ransDecoder.decode_nibble(&matchLengthModel[176 | positionContext], compressed);
							size_t highBits = ransDecoder.decode_nibble(&matchLengthModel[192 | matchLengthContext], compressed);

							if (highBits >= 4)
								highBits = (1 << (highBits - 4)) + ransDecoder.decode_raw_bits_short(highBits - 4, compressed) + 3;
							matchLength += highBits * 16 | lowBits;

							//Check overflow
							if (unlikely(thisBlockEnd - decompressed < matchLength)) {
								delete[] literalModel;
								return -1;
							}
						}

						uint8_t* const copyEnd = decompressed + matchLength;
						copy_match(decompressed, match, copyEnd, distance);
						decompressed = copyEnd;
					}
					//Faster copy
					else {
						if (distance >= 8) {
							memcpy(decompressed, match, 8);
							memcpy(decompressed + 8, match + 8, 8);
						}
						else {
							decompressed[0] = match[0];
							decompressed[1] = match[1];
							decompressed[2] = match[2];
							decompressed[3] = match[3];
							match += inc32table[distance];
							memcpy(decompressed + 4, match, 4);
							match += inc64table[distance];
							memcpy(decompressed + 8, match, 8);
						}
						decompressed += matchLength;
					}
				}
			}

			if (progress->progress(decompressed - decompressedStart, compressed - (compressedEnd - compressedSize))) {
				delete[] literalModel;
				return 0;
			}
		}

		memcpy(decompressed, compressed, STRIDER_LAST_BYTES);
		progress->progress(decompressedSize, compressed - (compressedEnd - compressedSize));

		delete[] literalModel;
		return 0;
	} 
}

#endif  //STRIDER_IMPLEMENTATION

#endif  //__STRIDER__
