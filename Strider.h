/*
 * Strider Compression Algorithm v1.0.0
 * Copyright (c) 2022 Carlos de Diego
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

 //#define STRIDER_IMPLEMENTATION

#ifndef __STRIDER__

#define __STRIDER__

#include <cstdint>
#include <stdio.h> //size_t

namespace strider {
	//Base class that allows to track progress of compression and decompression of
	// a Strider stream. You will have to create a child class which implements the functions.
	// - progress(bytes): the algorithm will pass the number of bytes that have been compressed/decompressed
	// - abort(): if this returns true, the compression or decompression will stop with a return code of 0
	class ProgressCallback {
	public:
		virtual void progress(size_t bytes) {
			return;
		}
		virtual int abort() {
			return false;
		}
	};

	//Compresses "size" bytes of data present in "input", and stores it in "output".
	//"Level" specifies a tradeoff between compressed size and speed, and must be in range [0, 10].
	//You may pass a pointer to an object with base class ProgressCallback, to track progress.
	//window determines the maximum backwards distance the matches can have, as a power of 2.
	//Larger values can improve compression, but use more memory.
	//Returns the size of the compressed stream or -1 on failure.
	size_t strider_compress(const uint8_t* input, const size_t size, uint8_t* output, const int level,
		ProgressCallback* progress = nullptr, const int window = 20);
	//Decompresses contents in "compressed" to "decompressed".
	//You may also pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns 0 on success or -1 on failure or corrupted data.
	int strider_decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed,
		const size_t uncompressedSize, ProgressCallback* progress = nullptr);

	//For a given input size, returns a size for the output buffer that is big enough to
	// contain the compressed stream even if it expands.
	size_t strider_compress_bound(const size_t size);
	//Returns the amount of memory the algorithm will consume on compression.
	size_t strider_estimate_memory(const size_t size, const int level, const int window = 20);
}

#ifdef STRIDER_IMPLEMENTATION

#include <algorithm>
#include <cstring>
#include <cmath>

using namespace std;

namespace strider {

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
#if UINTPTR_MAX > UINT32_MAX
#define IS_64BIT 1
#else
#define IS_64BIT 0
#endif

#if defined(_MSC_VER)
#if defined(_M_AMD64)
#include <intrin.h>
#define x64
#endif
#elif defined(__GNUC__) || defined(__clang__)
#if defined(__amd64__)
#include <x86intrin.h>
#define x64
#endif
#endif

	const union { uint16_t u; uint8_t c[2]; } LITTLE_ENDIAN_CHECK = { 1 };
#define IS_LITTLE_ENDIAN LITTLE_ENDIAN_CHECK.c[0]

#define MIN3(a, b, c) (std::min(a, std::min(b, c)))

#if IS_64BIT

	//Undefined behaviour if value == 0
	FORCE_INLINE uint64_t unsafe_int_log2(const uint64_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanReverse64(&result, value);
		return result;
#elif defined(__GNUC__) || defined(__clang__)
		return 63 - __builtin_clzll(value);
#else
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

		uint64_t index = value;
		index |= index >> 1;
		index |= index >> 2;
		index |= index >> 4;
		index |= index >> 8;
		index |= index >> 16;
		index |= index >> 32;
		return tab64[index * 0x03f6eaf2cd271461 >> 58];
#endif
	}

	//Way faster than using log2(double), also returns 0 for a value of 0
	FORCE_INLINE uint64_t int_log2(const uint64_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_int_log2(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_int_log2(value);
	}

	//Undefined behaviour when value == 0
	FORCE_INLINE uint64_t unsafeBitScanForward(const uint64_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanForward64(&result, value);
		return result;
#elif defined(__GNUC__) || defined(__clang__)
		return __builtin_ctzll(value);
#else
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
#endif
	}

	//Returns the index of the first set bit, starting from the least significant, or 0 if the input is null
	FORCE_INLINE uint64_t bitScanForward(const uint64_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafeBitScanForward(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafeBitScanForward(value);
	}

	struct FastIntHash {
		//Use top bits
		uint64_t operator()(const uint64_t value) {
			return value * 0xff51afd7ed558ccd;
		}
	};

	//These functions are used to obtain the data for the hash. If the number of bytes is higher than the word size,
	//they will be mixed. Also note that these might read more bytes than necessary.
	FORCE_INLINE uint64_t readHash8(const uint8_t* const ptr) {
		uint64_t value;
		memcpy(&value, ptr, 8);
		return value;
	}
	FORCE_INLINE uint64_t readHash4(const uint8_t* const ptr) {
		uint32_t value;
		memcpy(&value, ptr, 4);
		return value;
	}
	FORCE_INLINE uint64_t readHash2(const uint8_t* const ptr) {
		uint16_t value;
		memcpy(&value, ptr, 2);
		return value;
	}
	FORCE_INLINE uint64_t readHash12(const uint8_t* const ptr) {
		return readHash8(ptr) ^ readHash4(ptr + 8);
	}
	FORCE_INLINE uint64_t readHash6(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN)
			return readHash8(ptr) << 16;
		return readHash8(ptr) >> 16;  //Assumes big endian
	}
	FORCE_INLINE uint64_t readHash5(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN)
			return readHash8(ptr) << 24;
		return readHash8(ptr) >> 24;  //Assumes big endian
	}
	FORCE_INLINE uint64_t readHash3(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN)
			return readHash4(ptr) << 40;
		return readHash4(ptr) >> 8;  //Assumes big endian
	}

#else  //32 bit functions

	FORCE_INLINE uint32_t unsafe_int_log2(uint32_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanReverse(&result, value);
		return result;
#elif defined(__GNUC__) || defined(__clang__)
		return 31 - __builtin_clz(value);
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
	}

	FORCE_INLINE uint32_t int_log2(uint32_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_int_log2(value);
		return 0;
#endif
		return unsafe_int_log2(value);
	}

	FORCE_INLINE uint32_t unsafeBitScanForward(uint32_t value) {
#if defined(_MSC_VER)
		unsigned long result;
		_BitScanForward(&result, value);
		return result;
#elif defined(__GNUC__) || defined(__clang__)
		return __builtin_ctz(value);
#else
		static uint8_t tab32[32] = {
			 0,  1, 28,  2, 29, 14, 24,  3,
			30, 22, 20, 15, 25, 17,  4,  8,
			31, 27, 13, 23, 21, 19, 16,  7,
			26, 12, 18,  6, 11,  5, 10,  9
		};
		return tab32[(value & (0 - value)) * 0x077CB531U >> 27];
#endif
	}

	FORCE_INLINE uint32_t bitScanForward(uint32_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafeBitScanForward(value);
		return 0;
#endif
		return unsafeBitScanForward(value);
	}

	struct FastIntHash {
		uint32_t operator()(const uint32_t value) {
			return value * 0x27d4eb2d;
		}
	};

	FORCE_INLINE uint32_t readHash4(const uint8_t* const ptr) {
		uint32_t value;
		memcpy(&value, ptr, 4);
		return value;
	}
	FORCE_INLINE uint32_t readHash2(const uint8_t* const ptr) {
		uint16_t value;
		memcpy(&value, ptr, 2);
		return value;
	}
	FORCE_INLINE uint32_t readHash12(const uint8_t* const ptr) {
		return readHash4(ptr) ^ readHash4(ptr + 4) ^ readHash4(ptr + 8);
	}
	FORCE_INLINE uint32_t readHash8(const uint8_t* const ptr) {
		return readHash4(ptr) ^ readHash4(ptr + 4);
	}
	FORCE_INLINE uint32_t readHash6(const uint8_t* const ptr) {
		uint16_t a;
		memcpy(&a, ptr + 0, 2);
		return readHash4(ptr + 2) ^ a;
	}
	FORCE_INLINE uint32_t readHash5(const uint8_t* const ptr) {
		return readHash4(ptr) ^ ptr[4];
	}
	FORCE_INLINE uint32_t readHash3(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN)
			return readHash4(ptr) << 8;
		return readHash4(ptr) >> 8;  //Assumes big endian
	}

#endif

	FORCE_INLINE uint64_t readUint64LE(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN) {
			uint64_t value;
			memcpy(&value, ptr, 8);
			return value;
		}
		uint64_t value = 0;
		for (int i = 0; i < 8; i++)
			value |= (uint64_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void writeUint64LE(uint8_t* const ptr, const uint64_t value) {
		if (IS_LITTLE_ENDIAN)
			memcpy(ptr, &value, 8);
		else {
			for (int i = 0; i < 8; i++)
				ptr[i] = value >> i * 8;
		}
	}
	FORCE_INLINE uint32_t readUint32LE(const uint8_t* const ptr) {
		if (IS_LITTLE_ENDIAN) {
			uint32_t value;
			memcpy(&value, ptr, 4);
			return value;
		}
		uint32_t value = 0;
		for (int i = 0; i < 4; i++)
			value |= (uint64_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void writeUint32LE(uint8_t* const ptr, const uint32_t value) {
		if (IS_LITTLE_ENDIAN)
			memcpy(ptr, &value, 4);
		else {
			for (int i = 0; i < 4; i++)
				ptr[i] = value >> i * 8;
		}
	}
	FORCE_INLINE uint16_t readUint16LE(const uint8_t* const ptr) {
		uint16_t value;
		if (IS_LITTLE_ENDIAN)
			memcpy(&value, ptr, 2);
		else
			value = ptr[0] | (ptr[1] << 8);
		return value;
	}
	FORCE_INLINE void writeUint16LE(uint8_t* const ptr, const uint16_t value) {
		if (IS_LITTLE_ENDIAN)
			memcpy(ptr, &value, 2);
		else {
			for (int i = 0; i < 2; i++)
				ptr[i] = value >> i * 8;
		}
	}

	//Tries to find a match between the two locations, and returns the length
	//Note that this function should be called with at least MIN_LENGTH + 8 bytes of buffer after limit
	FORCE_INLINE size_t test_match(const uint8_t* front, const uint8_t* back, const uint8_t* const limit,
		const size_t minLength, const int window) {

		//Test first bytes
		//Usually compilers will optimize std::equal as 2 comparisons for minLength 5, 6, etc
		//We can only use one comparison to make it faster. For powers of 2, std::equal should be good
		if (IS_64BIT && IS_LITTLE_ENDIAN) {
			switch (minLength) {
			case 6:
				if ((readUint64LE(front) << 16) != (readUint64LE(back) << 16) || ((size_t)(front - back) >> window))
					return 0;
				break;
			case 5:
				if ((readUint64LE(front) << 24) != (readUint64LE(back) << 24) || ((size_t)(front - back) >> window))
					return 0;
				break;
			case 3:
				if ((readUint32LE(front) << 8) != (readUint32LE(back) << 8) || ((size_t)(front - back) >> window))
					return 0;
				break;
			default:
				if (!std::equal(back, back + minLength, front) || ((size_t)(front - back) >> window))
					return 0;
				break;
			}
		}
		else {
			if (!std::equal(back, back + minLength, front) || ((size_t)(front - back) >> window))
				return 0;
		}

		const uint8_t* const matchOrigin = front;
		front += minLength;
		back += minLength;

		if (IS_64BIT && IS_LITTLE_ENDIAN) {
			while (true) {
				if (unlikely(front + 8 > limit)) {
					if (front > limit)
						return 0;

					while (*front == *back && front < limit) {
						front++;
						back++;
					}
					return front - matchOrigin;
				}

				//Compare 8 bytes at a time using xor. It has the property of returning 0 if the two values are equal.
				//In case they differ, we can get the first byte that differs using a bit scan.
				const uint64_t xorVal = readUint64LE(front) ^ readUint64LE(back);

				if (xorVal) {
					front += unsafeBitScanForward(xorVal) >> 3;
					return front - matchOrigin;
				}

				front += 8;
				back += 8;
			}
		}
		else {
			if (front > limit)
				return 0;
			while (*front == *back && front < limit) {
				front++;
				back++;
			}
			return front - matchOrigin;
		}
	}

	const int GREEDY = 0;
	const int LAZY_NORMAL = 1;
	const int LAZY_EXTRA = 2;
	const int OPTIMAL_FAST = 3;
	const int OPTIMAL = 4;
	const int OPTIMAL_BRUTE = 5;
	const int OPTIMAL_ULTRA = 6;

	struct CompressorOptions {
		int parserFunction;
		int maxHashTableSize;
		int maxElementsPerHash;
		int standardNiceLength;
		int repNiceLength;
		int optimalBlockSize;        // (Optimal)
		int maxArrivals;             // (Optimal)
	};

	template<class IntType>
	struct LZ_Structure {
		IntType matchLength;
		IntType matchDistance;
		IntType literalRunLength;
	};

	template<class IntType>
	struct LZ_Match {
		IntType length;
		IntType distance;
	};

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

	template<class IntType>
	class LZ2WayCacheBucket {
		IntType* data;
	public:
		LZ2WayCacheBucket() {}
		LZ2WayCacheBucket(IntType* _data) {
			data = _data;
		}
		//Loads the first value, and at the same time pushes a value into that slot.
		void first(size_t* value) {
			const IntType tmp = data[0];
			data[0] = *value;
			*value = tmp;
		}
		//Loads the second value, and at the same time pushes a value into that slot.
		//Should be used after loading the first value.
		void second(size_t* value) {
			const IntType tmp = data[1];
			data[1] = *value;
			*value = tmp;
		}
		//Inserts a value into the first slot.
		//Used when skipping bytes.
		void push_in_first(const size_t value) {
			const IntType tmp = data[0];
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
	template<class IntType, class Hash>
	class LZ2WayCacheTable {
		IntType* arr = nullptr;
		size_t hashShift;
	public:
		LZ2WayCacheTable() {}
		~LZ2WayCacheTable() {
			delete[] arr;
		}
		void init(const size_t logSize) {
			arr = new IntType[(size_t)2 << logSize]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
		}
		LZ2WayCacheBucket<IntType> operator[](const size_t value) {
			return LZ2WayCacheBucket<IntType>(arr + (Hash{}(value) >> hashShift) * 2);
		}
	};

	//Used for easier implementation
	//Please do not use push() and next() on the same bucket
	template<class IntType>
	class LzCacheBucket {
		IntType* it;
		IntType* last;
	public:
		LzCacheBucket() {}
		LzCacheBucket(IntType* _begin, IntType* _end) {
			it = _begin;
			last = _end;
		}
		//Pushes a new value into the bucket. Used when skipping bytes
		void push(const size_t newValue) {
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
		void next(size_t* value) {
			const IntType tmp = *it;
			*it = *value;
			*value = tmp;
			it++;
		}
		//Loads the next position, but without updating the bucket as we go
		IntType next() {
			return *it++;
		}
	};

	//This works like the basic hash table, except that it stores N positions per bucket
	template<class IntType, class Hash>
	class LzCacheTable {
		IntType* arr = nullptr;
		size_t hashShift;
		size_t elementsPerBucket;  //log2
	public:
		//Use 2^x sizes to avoid the use of modulo and multiplication
		LzCacheTable() {}
		LzCacheTable(const size_t logSize, const size_t numElements) {
			init(logSize, numElements);
		}
		~LzCacheTable() {
			delete[] arr;
		}
		void init(const size_t logSize, const size_t numElements) {
			arr = new IntType[(size_t)1 << logSize << numElements]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
			elementsPerBucket = numElements;
		}
		LzCacheBucket<IntType> operator[](size_t value) {
			value = Hash{}(value) >> hashShift;
			IntType* bucket = arr + (value << elementsPerBucket);
			return LzCacheBucket<IntType>(bucket, bucket + ((size_t)1 << elementsPerBucket));
		}
	};

	//Simple and fast
	template<class IntType>
	class HashTableMatchFinder {
		HashTable<IntType, FastIntHash> lzdict3;
		LzCacheTable<IntType, FastIntHash> lzdict4;
		LzCacheTable<IntType, FastIntHash> lzdict8;

	public:
		void init(const size_t size, const CompressorOptions compressorOptions, const int window) {
			const int log2size = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
			lzdict3.init(std::min(log2size, 14));
			lzdict4.init(log2size, compressorOptions.maxElementsPerHash);
			lzdict8.init(log2size, compressorOptions.maxElementsPerHash);
		}

		LZ_Match<IntType>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			LZ_Match<IntType>* matches, size_t highestLength, const CompressorOptions compressorOptions, const int window) {

			IntType& chain3 = lzdict3[readHash3(input)];
			LzCacheBucket<IntType> chain4 = lzdict4[readHash4(input)];
			LzCacheBucket<IntType> chain8 = lzdict8[readHash8(input)];

			if (highestLength < 3) {
				const uint8_t* where = inputStart + chain3;
				size_t length = test_match(input, where, limit, 3, window);

				if (length > highestLength) {
					matches->distance = input - where;
					matches->length = length;
					matches++;

					highestLength = length;
					if (highestLength > compressorOptions.standardNiceLength) {
						chain3 = input - inputStart;
						chain4.push(input - inputStart);
						chain8.push(input - inputStart);
						return matches;
					}
				}
			}
			chain3 = input - inputStart;

			if (highestLength < 7) {

				size_t pos = input - inputStart;
				while (!chain4.ended()) {
					chain4.next(&pos);

					const uint8_t* where = inputStart + pos;

					if (*(input + highestLength) != *(where + highestLength))
						continue;

					const size_t length = test_match(input, where, limit, 4, window);

					if (length > highestLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						highestLength = length;
						if (highestLength >= 7) {
							while (!chain4.ended())
								chain4.next(&pos);
							break;
						}
					}
				}
			}
			else {
				chain4.push(input - inputStart);
			}

			if (highestLength >= 4 && highestLength < compressorOptions.standardNiceLength) {

				size_t pos = input - inputStart;
				while (!chain8.ended()) {
					chain8.next(&pos);

					const uint8_t* where = inputStart + pos;
					if (*(input + highestLength) != *(where + highestLength))
						continue;

					const size_t length = test_match(input, where, limit, 8, window);

					if (length > highestLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						highestLength = length;
						if (highestLength > compressorOptions.standardNiceLength) {
							while (!chain8.ended())
								chain8.next(&pos);
							break;
						}
					}
				}
			}
			else {
				chain8.push(input - inputStart);
			}

			return matches;
		}

		void update_position(const uint8_t* const input, const uint8_t* const inputStart) {
			const size_t pos = input - inputStart;
			lzdict3[readHash3(input)] = pos;
			lzdict4[readHash4(input)].push(pos);
			lzdict8[readHash8(input)].push(pos);
		}
	};

	const int NO_MATCH_POS = 0;

	//Original match finder implementation from BriefLZ
	template<class IntType>
	class BinaryMatchFinder {
	public:

		IntType* nodes = nullptr;
		size_t nodeListMask;
		size_t nodeListSize;

		HashTable<IntType, FastIntHash> nodeLookup;
		HashTable<IntType, FastIntHash> lzdict3;
		LzCacheTable<IntType, FastIntHash> lzdict12;

		~BinaryMatchFinder() {
			delete[] nodes;
		}
		BinaryMatchFinder() {}

		void init(const size_t size, const CompressorOptions compressorOptions, const int window) {

			const size_t binaryTreeSize = (size_t)1 << std::min(compressorOptions.maxHashTableSize, window);
			const size_t totalWindowSize = std::min(size, (size_t)1 << window);
			nodeListSize = std::min(binaryTreeSize, size);
			nodeListMask = binaryTreeSize - 1;
			//It is not necessary to initialize to 0
			nodes = new IntType[2 * nodeListSize];

			nodeLookup.init(MIN3((int)int_log2(size) - 3, 20, window - 3));

			lzdict3.init(MIN3((int)int_log2(size) - 3, 16, window - 3));
			if (totalWindowSize > nodeListSize)
				lzdict12.init(std::max(4, (int)int_log2(totalWindowSize - nodeListSize) - 4), compressorOptions.maxElementsPerHash - 4);
		}

		LZ_Match<IntType>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			LZ_Match<IntType>* matches, size_t highestLength, const CompressorOptions compressorOptions, const int window) {

			const size_t inputPosition = input - inputStart;

			// First try to get a length 3 match
			IntType& chain3 = lzdict3[readHash3(input)];
			const uint8_t* where = inputStart + chain3;
			size_t length = test_match(input, where, limit, 3, window);

			if (length > highestLength || (compressorOptions.parserFunction == OPTIMAL_ULTRA && length)) {
				matches->distance = input - where;
				matches->length = length;
				highestLength = length;
				matches++;

				if (highestLength >= compressorOptions.standardNiceLength) {
					update_position(input, inputStart, limit, compressorOptions, window);
					return matches;
				}
			}
			chain3 = inputPosition;

			//If we reach this position on the back stop the search
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;

			IntType& lookupEntry = nodeLookup[readHash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			IntType* lesserNode = &nodes[2 * (lookupEntry & nodeListMask)];
			IntType* greaterNode = &nodes[2 * (lookupEntry & nodeListMask) + 1];
			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;

			size_t depth = (size_t)1 << compressorOptions.maxElementsPerHash;

			// Check matches
			while (true) {

				if (backPosition <= btEnd || depth-- == 0) {
					*lesserNode = NO_MATCH_POS;
					*greaterNode = NO_MATCH_POS;
					break;
				}

				const uint8_t* front = std::min(lesserFront, greaterFront);
				const uint8_t* back = front - (inputPosition - backPosition);

				const size_t extraLength = test_match(front, back, limit, 0, window);
				front += extraLength;
				back += extraLength;

				length = front - input;
				IntType* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
				if (length > highestLength || (compressorOptions.parserFunction == OPTIMAL_ULTRA && length > 3)) {
					matches->distance = front - back;
					matches->length = length;
					matches++;

					if (length >= compressorOptions.standardNiceLength) {
						*lesserNode = nextNode[0];
						*greaterNode = nextNode[1];
						if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize)
							lzdict12[readHash12(input - nodeListSize)].push(inputPosition - nodeListSize);
						return matches;
					}
					highestLength = length;
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

			if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize) {

				LzCacheBucket<IntType> cacheBucket = lzdict12[readHash12(input)];

				while (!cacheBucket.ended()) {
					const IntType pos = cacheBucket.next();

					const uint8_t* const where = inputStart + pos;
					if (*(input + highestLength) != *(where + highestLength))
						continue;

					const size_t length = test_match(input, where, limit, 12, window);

					if (length > highestLength) {
						matches->distance = input - where;
						matches->length = length;
						matches++;

						highestLength = length;
						if (highestLength >= compressorOptions.standardNiceLength)
							break;
					}
				}

				lzdict12[readHash12(input - nodeListSize)].push(inputPosition - nodeListSize);
			}

			return matches;
		}

		void update_position(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			const CompressorOptions& compressorOptions, const int window) {

			const size_t inputPosition = input - inputStart;
			lzdict3[readHash3(input)] = inputPosition;
			if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize)
				lzdict12[readHash12(input - nodeListSize)].push(inputPosition - nodeListSize);

			//If we reach this position on the front stop the update
			const uint8_t* positionSkip = std::min(limit, input + compressorOptions.standardNiceLength);
			//If we reach this position on the back stop the update
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;
			IntType& lookupEntry = nodeLookup[readHash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			IntType* lesserNode = &nodes[2 * (inputPosition & nodeListMask)];
			IntType* greaterNode = &nodes[2 * (inputPosition & nodeListMask) + 1];

			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;
			size_t depth = (size_t)1 << compressorOptions.maxElementsPerHash;

			// Check matches
			while (true) {
				if (backPosition <= btEnd || depth-- == 0) {
					*lesserNode = NO_MATCH_POS;
					*greaterNode = NO_MATCH_POS;
					return;
				}

				const uint8_t* front = std::min(lesserFront, greaterFront);
				const uint8_t* back = front - (inputPosition - backPosition);

				const size_t length = test_match(front, back, positionSkip, 0, window);
				front += length;
				back += length;

				IntType* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
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

	FORCE_INLINE uint64_t mulHigh64(const uint64_t a, const uint64_t b) {
#if defined(_MSC_VER) && defined(x64)
		return __umulh(a, b);
#elif defined(__GNUC__) && IS_64BIT
		unsigned __int128 prod = a * (unsigned __int128)b;
		return prod >> 64;
#else
		uint64_t a_lo = (uint32_t)a;
		uint64_t a_hi = a >> 32;
		uint64_t b_lo = (uint32_t)b;
		uint64_t b_hi = b >> 32;

		uint64_t a_x_b_hi = a_hi * b_hi;
		uint64_t a_x_b_mid = a_hi * b_lo;
		uint64_t b_x_a_mid = b_hi * a_lo;
		uint64_t a_x_b_lo = a_lo * b_lo;

		uint64_t carry_bit = ((uint64_t)(uint32_t)a_x_b_mid +
			(uint64_t)(uint32_t)b_x_a_mid +
			(a_x_b_lo >> 32)) >> 32;

		uint64_t multhi = a_x_b_hi +
			(a_x_b_mid >> 32) + (b_x_a_mid >> 32) +
			carry_bit;

		return multhi;
#endif
	}

	//https://github.com/romeric/fastapprox/blob/master/fastapprox/src/fastlog.h
	FORCE_INLINE float fast_log2(float x) {
		union { float f; uint32_t i; } vx = { x };
		union { uint32_t i; float f; } mx = { (vx.i & 0x007FFFFF) | 0x3f000000 };
		float y = vx.i;
		y *= 1.1920928955078125e-7f;

		return y - 124.22551499f
			- 1.498030302f * mx.f
			- 1.72587999f / (0.3520887068f + mx.f);
	}

	const int MODEL_PRECISION_BITS = 14;
	const int MODEL_SCALE = (1 << MODEL_PRECISION_BITS);
	const int MODEL_BIT_MASK = (((size_t)1 << MODEL_PRECISION_BITS) - 1);
	const int MODEL_UPDATE_SPEED = 7;

	const int STRIDER_MAX_BLOCK_SIZE = 65536;
	const int STRIDER_MIN_LENGTH = 2;
	const int STRIDER_LAST_BYTES = 31;
	const int STRIDER_COST_PRECISION = 4096;

	const int STRIDER_RAW_FLAG = 0;
	const int HIGH_ENTROPY = 255;
	const int UNCOMPRESSED_LITERALS = 9;

	const uint16_t NIBBLE_INITIAL[16] = {
		0, MODEL_SCALE / 16, 2 * MODEL_SCALE / 16, 3 * MODEL_SCALE / 16,
		4 * MODEL_SCALE / 16, 5 * MODEL_SCALE / 16, 6 * MODEL_SCALE / 16, 7 * MODEL_SCALE / 16,
		8 * MODEL_SCALE / 16, 9 * MODEL_SCALE / 16, 10 * MODEL_SCALE / 16, 11 * MODEL_SCALE / 16,
		12 * MODEL_SCALE / 16, 13 * MODEL_SCALE / 16, 14 * MODEL_SCALE / 16, 15 * MODEL_SCALE / 16
	};

	/* HOW TO GENERATE THE MIXIN
	*  n = number of elements in the model
	*  d = update speed
	*
	*  const int bias = d - n - 1;
	*
	*  for (int y = 0; y < n; y++) {
	*      for (int x = 0; x <= y; x++)
	*	       mixin[y][x] = x;
	*	   for (int x = y + 1; x < n; x++)
	*		   mixin[y][x] = FULL_RANGE + x + bias;
	*  }
	*
	*  In this case
	*
	*  for (int y = 0; y < 16; y++) {
	*      printf("{ ");
	*	   for (int x = 0; x <= y; x++)
	*	       printf("%d, ", x * 16);
	*	   for (int x = y + 1; x < 16; x++)
	*	       printf("%d, ", MODEL_SCALE + (1 << MODEL_UPDATE_SPEED) - (16 * 16) - 1 + x * 16);
	*	   printf("},\n");
	*  }
	*/

	const static union {
#ifdef x64
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

	//Tried to speed up with AVX, and got the oposite. Nice
	struct nibble_model {

		union {
			uint16_t scalar[16];
#ifdef x64
			__m128i sse[2];
#endif
		};

		nibble_model() {}
		FORCE_INLINE void init() {
			memcpy(scalar, NIBBLE_INITIAL, 32);
		}

		FORCE_INLINE uint16_t get_freq(const size_t symbol) {
			const uint16_t high = scalar[(symbol + 1) & 0xF];
			const uint16_t low = scalar[symbol];
			return (high - low) & MODEL_BIT_MASK;;
		}

		FORCE_INLINE void update(const size_t symbol) {
#ifdef x64
			__m128i upd0 = _mm_srai_epi16(_mm_sub_epi16(NIBBLE_MIXIN_SSE[symbol][0], sse[0]), MODEL_UPDATE_SPEED);
			__m128i upd1 = _mm_srai_epi16(_mm_sub_epi16(NIBBLE_MIXIN_SSE[symbol][1], sse[1]), MODEL_UPDATE_SPEED);
			sse[0] = _mm_add_epi16(sse[0], upd0);
			sse[1] = _mm_add_epi16(sse[1], upd1);
#else
			for (int i = 1; i < 16; i++)
				scalar[i] += (NIBBLE_MIXIN_SCALAR[symbol][i] - scalar[i]) >> MODEL_UPDATE_SPEED;
#endif
		}

		//The work of the encoder
		FORCE_INLINE void get_range_and_update(const size_t symbol, uint16_t* low, uint16_t* freq) {
			const uint16_t high = scalar[(symbol + 1) & 0xF];
			*low = scalar[symbol];
			*freq = (high - *low) & MODEL_BIT_MASK;
			update(symbol);
		}

		//The work of the decoder
		FORCE_INLINE size_t lookup_and_update(const uint16_t value, uint16_t* low, uint16_t* freq) {
#ifdef x64
			__m128i cmp = _mm_set1_epi16(value);
			__m128i tmp0 = _mm_cmpgt_epi16(sse[0], cmp);
			__m128i tmp1 = _mm_cmpgt_epi16(sse[1], cmp);
			const size_t mask = _mm_movemask_epi8(_mm_packs_epi16(tmp0, tmp1)) | 0x10000;
			size_t symbol = bitScanForward(mask) - 1;
#else
			size_t symbol = 8 * (value >= scalar[8]);
			symbol += 4 * (value >= scalar[4 + symbol]);
			symbol += 2 * (value >= scalar[2 + symbol]);
			symbol += value >= scalar[1 + symbol];
#endif
			get_range_and_update(symbol, low, freq);
			return symbol;
		}
	};

	const uint64_t RANS_BIT_PRECISION = 63;
	const uint64_t RANS_NORMALIZATION_INTERVAL = ((uint64_t)1 << (RANS_BIT_PRECISION - 32));

	//64 bit dual state rans encoder
	class RansEncoder {

		uint8_t* streamBegin;
		uint8_t* streamIt;

		uint64_t stateA;
		uint64_t stateB;
		//rans stream has to be written backwards, store it first in a buffer
		uint8_t* blockBufferBegin = nullptr;
		uint8_t* blockBufferIt;
		size_t blockBufferSize;
		//rans symbols also have to be written backwards, so store them in this buffer
		uint32_t* dataBufferBegin = nullptr;
		uint32_t* dataBufferIt;
		uint64_t* rcpFreq = nullptr;

	public:
		RansEncoder() {}
		~RansEncoder() {
			delete[] blockBufferBegin;
			delete[] dataBufferBegin;
			delete[] rcpFreq;
		}

		//for dataBufferSize:
		//nibble model takes 1 slot, raw bits take 2 slots for every 31 bits
		void initialize_rans_encoder(const size_t _blockBufferSize, const size_t dataBufferSize) {
			blockBufferSize = _blockBufferSize;
			blockBufferBegin = new uint8_t[blockBufferSize];
			blockBufferIt = blockBufferBegin + blockBufferSize;
			dataBufferBegin = new uint32_t[dataBufferSize];
			dataBufferIt = dataBufferBegin;
		}
		//this is about twice as fast, but has some initialization overhead, so it is not recommended for small files
		void initialize_fast_rans_encoder() {
			rcpFreq = new uint64_t[MODEL_SCALE];

			for (size_t freq = 2; freq < MODEL_SCALE; freq++) {
				// Say M := 1 << scale_bits.
				//
				// The original encoder does:
				//   x_new = (x/freq)*M + start + (x%freq)
				//
				// The fast encoder does (schematically):
				//   q     = mul_hi(x, rcp_freq) >> rcp_shift   (division)
				//   r     = x - q*freq                         (remainder)
				//   x_new = q*M + bias + r                     (new x)
				// plugging in r into x_new yields:
				//   x_new = bias + x + q*(M - freq)
				//        =: bias + x + q*cmpl_freq             (*)
				//
				// and we can just precompute cmpl_freq. Now we just need to
				// set up our parameters such that the original encoder and
				// the fast encoder agree.
				size_t shift = int_log2(freq - 1) + 1;
				uint64_t x0, x1, t0, t1;

				// long divide ((uint128) (1 << (shift + 63)) + freq-1) / freq
				// by splitting it into two 64:64 bit divides (this works because
				// the dividend has a simple form.)
				x0 = freq - 1;
				x1 = (uint64_t)1 << (shift + 31);

				t1 = x1 / freq;
				x0 += (x1 % freq) << 32;
				t0 = x0 / freq;

				rcpFreq[freq] = t0 + (t1 << 32);
			}
		}
		void start_rans(uint8_t* output) {
			streamIt = output;
			stateA = RANS_NORMALIZATION_INTERVAL;
			stateB = RANS_NORMALIZATION_INTERVAL;
		}
		FORCE_INLINE void encode_nibble(const size_t symbol, nibble_model* model) {
			uint16_t low, freq;
			model->get_range_and_update(symbol, &low, &freq);
			*dataBufferIt++ = (low << 15) | freq;
		}
		//Maximum of 62 bits
		FORCE_INLINE void encode_raw_bits(size_t symbol, size_t nBits) {

			if (nBits > 31) {
				*dataBufferIt++ = symbol & 0x7FFFFFFF;
				*dataBufferIt++ = 0x80000000 | 31;
				symbol >>= 31;
				nBits -= 31;
			}
			*dataBufferIt++ = symbol;
			*dataBufferIt++ = 0x80000000 | nBits;
		}

		size_t end_rans() {

			do {
				std::swap(stateA, stateB);
				dataBufferIt--;
				size_t data = *dataBufferIt;

				//raw
				if (data >> 31) {
					const size_t nBits = data & 0x1F;
					dataBufferIt--;
					data = *dataBufferIt;

					const uint64_t interval = RANS_NORMALIZATION_INTERVAL << (32 - nBits);
					if (stateB >= interval) {
						blockBufferIt -= 4;
						writeUint32LE(blockBufferIt, stateB);
						stateB >>= 32;
					}
					stateB = (stateB << nBits) + data;
				}
				//nibble
				else {
					const size_t freq = data & 0x7FFF;
					const size_t low = data >> 15;

					const uint64_t interval = (RANS_NORMALIZATION_INTERVAL << (32 - MODEL_PRECISION_BITS)) * freq;
					if (stateB >= interval) {
						blockBufferIt -= 4;
						writeUint32LE(blockBufferIt, stateB);
						stateB >>= 32;
					}
					//Fast encoder is enabled. The model will never have a freq of 1
					if (rcpFreq)
						stateB = stateB + low + (mulHigh64(stateB, rcpFreq[freq]) >> int_log2(freq - 1)) * ((1 << MODEL_PRECISION_BITS) - freq);
					else
						stateB = ((stateB / freq) << MODEL_PRECISION_BITS) + (stateB % freq) + low;
				}
			} while (dataBufferIt != dataBufferBegin);

			blockBufferIt -= 16;
			writeUint64LE(blockBufferIt + 0, stateB);
			writeUint64LE(blockBufferIt + 8, stateA);

			size_t finalBlockSize = blockBufferBegin + blockBufferSize - blockBufferIt;
			memcpy(streamIt, blockBufferIt, finalBlockSize);
			blockBufferIt = blockBufferBegin + blockBufferSize;

			return finalBlockSize;
		}
	};

	void get_histogram16(const uint8_t* buf, size_t bufsize, uint32_t* hist) {

		std::fill(hist, hist + 256 * 16, 0);
		size_t pos = 0;
		const size_t fastLoopEnd = bufsize & ~0xF;
		for (; pos < fastLoopEnd; pos += 16) {
			if (IS_64BIT) {
				uint64_t w;
				memcpy(&w, buf + pos, 8);
				hist[0 + (w >> 0 & 0xFF)]++;
				hist[256 + (w >> 8 & 0xFF)]++;
				hist[512 + (w >> 16 & 0xFF)]++;
				hist[768 + (w >> 24 & 0xFF)]++;
				hist[1024 + (w >> 32 & 0xFF)]++;
				hist[1280 + (w >> 40 & 0xFF)]++;
				hist[1536 + (w >> 48 & 0xFF)]++;
				hist[1792 + (w >> 56 & 0xFF)]++;
				memcpy(&w, buf + pos + 8, 8);
				hist[2048 + (w >> 0 & 0xFF)]++;
				hist[2304 + (w >> 8 & 0xFF)]++;
				hist[2560 + (w >> 16 & 0xFF)]++;
				hist[2816 + (w >> 24 & 0xFF)]++;
				hist[3072 + (w >> 32 & 0xFF)]++;
				hist[3328 + (w >> 40 & 0xFF)]++;
				hist[3584 + (w >> 48 & 0xFF)]++;
				hist[3840 + (w >> 56 & 0xFF)]++;
			}
			else {
				uint32_t w;
				memcpy(&w, buf + pos, 4);
				hist[0 + (w >> 0 & 0xFF)]++;
				hist[256 + (w >> 8 & 0xFF)]++;
				hist[512 + (w >> 16 & 0xFF)]++;
				hist[768 + (w >> 24 & 0xFF)]++;
				memcpy(&w, buf + pos + 4, 4);
				hist[1024 + (w >> 0 & 0xFF)]++;
				hist[1280 + (w >> 8 & 0xFF)]++;
				hist[1536 + (w >> 16 & 0xFF)]++;
				hist[1792 + (w >> 24 & 0xFF)]++;
				memcpy(&w, buf + pos + 8, 4);
				hist[2048 + (w >> 0 & 0xFF)]++;
				hist[2304 + (w >> 8 & 0xFF)]++;
				hist[2560 + (w >> 16 & 0xFF)]++;
				hist[2816 + (w >> 24 & 0xFF)]++;
				memcpy(&w, buf + pos + 12, 4);
				hist[3072 + (w >> 0 & 0xFF)]++;
				hist[3328 + (w >> 8 & 0xFF)]++;
				hist[3584 + (w >> 16 & 0xFF)]++;
				hist[3840 + (w >> 24 & 0xFF)]++;
			}
		}
		for (; pos < bufsize; pos++)
			hist[256 * (pos & 0xF) + buf[pos]]++;
	}

	float calculate_entropy16(uint32_t* hist, const size_t bufsize, const size_t alignment) {

		const float probDiv = 1 / float(bufsize / alignment);

		float entropy = 0;
		for (size_t a = 0; a < alignment; a++) {

			for (size_t b = 0; b < 256; b++) {

				size_t count;
				switch (alignment) {
				case 1:
					count = hist[0 + b] + hist[256 + b] + hist[512 + b] + hist[768 + b] +
						hist[1024 + b] + hist[1280 + b] + hist[1536 + b] + hist[1792 + b] +
						hist[2048 + b] + hist[2304 + b] + hist[2560 + b] + hist[2816 + b] +
						hist[3072 + b] + hist[3328 + b] + hist[3584 + b] + hist[3840 + b];
					break;
				case 2:
				{
					const size_t base = 256 * a + b;
					count = hist[0 + base] + hist[512 + base] + hist[1024 + base] + hist[1536 + base] +
						hist[2048 + base] + hist[2560 + base] + hist[3072 + base] + hist[3584 + base];
					break;
				}
				case 4:
				{
					const size_t base = 256 * a + b;
					count = hist[0 + base] + hist[1024 + base] + hist[2048 + base] + hist[3072 + base];
					break;
				}
				case 8:
				{
					const size_t base = 256 * a + b;
					count = hist[0 + base] + hist[2048 + base];
					break;
				}
				case 16:
					count = hist[256 * a + b];
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

	struct PbLcSector {
		const uint8_t* start;
		size_t pbMask;
		size_t lcValue;
	};

	class PbLcDetector {
		PbLcSector* pbLcSectors = nullptr;
		size_t numberSectors;
		size_t currentSector;

	public:
		~PbLcDetector() {
			delete[] pbLcSectors;
		}
		PbLcDetector() {}

		int init(const uint8_t* data, size_t size) {

			const size_t PREDICTOR_BLOCK_SIZE = 65536;
			const size_t numberBlocks = size / PREDICTOR_BLOCK_SIZE + (size % PREDICTOR_BLOCK_SIZE > 0);

			uint8_t* predictedBlocks = nullptr;
			try {
				predictedBlocks = new uint8_t[numberBlocks];
				pbLcSectors = new PbLcSector[numberBlocks + 1];
			}
			catch (const std::bad_alloc& e) {
				delete[] predictedBlocks;
				delete[] pbLcSectors;
				return -1;
			}

			for (size_t i = 0; i < numberBlocks; i++) {
				const size_t thisBlockStart = i * PREDICTOR_BLOCK_SIZE;
				const size_t thisBlockSize = std::min(size - thisBlockStart, PREDICTOR_BLOCK_SIZE);

				uint32_t blockHistogram[256 * 16];
				get_histogram16(data + thisBlockStart, thisBlockSize, blockHistogram);
				float bestEntropy = calculate_entropy16(blockHistogram, thisBlockSize, 1);

				uint8_t predictedPb = 0; //default pb = 0

				if (size >= 196608) {
					float entropyPosSize2 = calculate_entropy16(blockHistogram, thisBlockSize, 2) + 0.03;
					if (entropyPosSize2 < bestEntropy) {
						bestEntropy = entropyPosSize2;
						predictedPb = 1;
					}
				}
				if (size >= 262144 && (predictedPb == 1)) {
					float entropyPosSize4 = calculate_entropy16(blockHistogram, thisBlockSize, 4) + 0.07;
					if (entropyPosSize4 < bestEntropy) {
						bestEntropy = entropyPosSize4;
						predictedPb = 3;
					}
				}
				//Only try higher pb if the lower ones already reduce entropy
				if (size >= 393216 && (predictedPb == 3)) {
					float entropyPosSize8 = calculate_entropy16(blockHistogram, thisBlockSize, 8) + 0.25;
					if (entropyPosSize8 < bestEntropy) {
						bestEntropy = entropyPosSize8;
						predictedPb = 7;
					}
				}
				if (size >= 720896 && (predictedPb == 7)) {
					float entropyPosSize16 = calculate_entropy16(blockHistogram, thisBlockSize, 16) + 0.50;
					if (entropyPosSize16 < bestEntropy) {
						bestEntropy = entropyPosSize16;
						predictedPb = 15;
					}
				}

				predictedBlocks[i] = predictedPb;
			}

			//Now get where each sector of pb begins
			pbLcSectors[0] = { data, predictedBlocks[0], 4 };
			numberSectors = 1;
			for (size_t i = 1; i < numberBlocks; i++) {
				if (predictedBlocks[i] != predictedBlocks[i - 1]) {
					pbLcSectors[numberSectors] = { data + i * PREDICTOR_BLOCK_SIZE, predictedBlocks[i], 4 };
					numberSectors++;
				}
			}
			pbLcSectors[numberSectors] = { data + size, 0, 0 };
			currentSector = 0;

			// The most problematic aspect of this method is that it tends to introduce a lot
			// of noise in the prediction. That is, if the predicted pb is supposed to be 3,
			// there will be blocks with pb = 1 or 7 sparsed throughout the file.
			// This second stage will try to remove those "random" blocks

			//Now remove that noise
			for (size_t sector = 0; sector < numberSectors; ) {
				const size_t sectorSize = pbLcSectors[sector + 1].start - pbLcSectors[sector].start;
				PbLcSector sectorData = pbLcSectors[sector];
				//Pb is high and the sector small, we have to fix that
				if ((sectorData.pbMask == 0 && sectorSize < 196608) || (sectorData.pbMask == 1 && sectorSize < 262144) ||
					(sectorData.pbMask == 3 && sectorSize < 327680) || (sectorData.pbMask == 7 && sectorSize < 458752) ||
					(sectorData.pbMask == 15 && sectorSize < 786432)) {

					//First sector
					if (sector == 0) {
						//Only one sector? Reduce pb
						if (numberSectors == 1) {
							size_t expectedMaxPb = sectorSize < 262144 ? 0 :
								sectorSize < 327680 ? 1 :
								sectorSize < 458752 ? 3 :
								sectorSize < 786432 ? 7 : 15;
							if (sectorData.pbMask > expectedMaxPb)
								pbLcSectors[sector].pbMask = expectedMaxPb;
							break;
						}
						else {
							//Normally we would take the previous run, in this case take the following
							pbLcSectors[0].pbMask = pbLcSectors[1].pbMask;
							memmove(&pbLcSectors[1], &pbLcSectors[2], sizeof(PbLcSector) * numberSectors - 2);
							numberSectors--;
						}
					}
					else {
						//Copy the pb of the previous run
						memmove(&pbLcSectors[sector], &pbLcSectors[sector + 1], sizeof(PbLcSector) * numberSectors - sector - 1);
						numberSectors--;
					}
				}
				//Everything is nice
				else
					sector++;
			}

			delete[] predictedBlocks;

			return 0;
		}

		uint64_t test_literal_size(const uint32_t* literalBuffer, const size_t literalCount, nibble_model* literalModel,
			uint8_t literalContextBitShift, uint8_t positionContextBitMask, const uint16_t* freqCostTable) {

			uint64_t compressedSize = 0;

			for (size_t i = 0; i < (48 << (8 - literalContextBitShift)) * (positionContextBitMask + 1); i++) { literalModel[i].init(); }
			positionContextBitMask = ((positionContextBitMask + 1) << (8 - literalContextBitShift)) - 1;

			for (size_t i = 0; i < literalCount; i++) {

				const uint32_t literalData = literalBuffer[i];
				const uint8_t symbol = literalData >> 24;
				const uint8_t exclude = (literalData >> 16) & 0xFF;
				const size_t context = literalData & 0xFFF;
				nibble_model* const modelTree = literalModel + ((context >> literalContextBitShift) & positionContextBitMask) * 48;

				uint16_t low, freq;

				const uint8_t symbolHigh = symbol >> 4;
				const uint8_t excludeHigh = exclude >> 4;
				const uint8_t excludeLow = exclude & 0xF;
				modelTree[excludeHigh].get_range_and_update(symbolHigh, &low, &freq);
				compressedSize += freqCostTable[freq];
				modelTree[symbolHigh == excludeHigh ? 16 | excludeLow : 32 | symbolHigh].get_range_and_update(symbol & 0xF, &low, &freq);
				compressedSize += freqCostTable[freq];
			}

			return compressedSize;
		}

		template<class IntType>
		int calculate_lc(const uint8_t* input, const size_t size, const uint8_t lcStep, const uint16_t* freqCostTable) {

			HashTable<IntType, FastIntHash> lzdict;
			lzdict.init(std::min(int_log2(size) - 3, (size_t)18));

			const size_t maxLiteralCount = std::max(std::min((uint64_t)1 << 48, (uint64_t)size / 4), (uint64_t)1048576);
			uint32_t* literalBuffer = nullptr;
			nibble_model* literalModel = nullptr;
			try {
				literalBuffer = new uint32_t[maxLiteralCount];
				literalModel = new nibble_model[48 * 256 * 16];
			}
			catch (std::bad_alloc& e) {
				delete[] literalBuffer;
				delete[] literalModel;
				return -1;
			}

			const uint8_t* const inputStart = input;
			input++;  //As always skip first byte so that it does not find a match with distance = 0 at the begining

			for (size_t i = 0; i < numberSectors; i++) {

				size_t literalCount = 0;
				size_t lastDistance = 1;
				const size_t positionContextBitMask = pbLcSectors[i].pbMask;
				const uint8_t* const matchLimit = pbLcSectors[i + 1].start;

				for (; input < matchLimit; ) {

					IntType* dictEntry = &lzdict[readHash4(input)];
					const size_t matchLength = test_match(input, inputStart + *dictEntry, matchLimit, 4);

					//We have found a match
					if (matchLength) {
						lastDistance = input - (inputStart + *dictEntry);
						*dictEntry = input - inputStart;
						const uint8_t* const matchEnd = input + matchLength;
						for (input++; input < matchEnd; input++)
							lzdict[readHash4(input)] = input - inputStart;
					}
					else {
						*dictEntry = input - inputStart;
						const uint32_t literalData = (*input << 24) | (*(input - lastDistance) << 16) | ((reinterpret_cast<size_t>(input) & positionContextBitMask) << 8) | input[-1];
						literalBuffer[literalCount] = literalData;
						literalCount++;
						input++;

						if (literalCount == maxLiteralCount) {
							input = matchLimit;
							break;
						}
					}
				}

				uint8_t bestLcValue = 4;
				uint64_t bestSize = test_literal_size(literalBuffer, literalCount, literalModel, 4, positionContextBitMask, freqCostTable);

				for (int8_t lcValue = 4 + lcStep; lcValue <= 8; lcValue += lcStep) {
					uint64_t size = test_literal_size(literalBuffer, literalCount, literalModel, lcValue, positionContextBitMask, freqCostTable);
					if (size >= bestSize) {
						break;
					}
					bestLcValue = lcValue;
					bestSize = size;
				}
				if (bestLcValue == 4) {
					for (int8_t lcValue = 4 - lcStep; lcValue >= 0; lcValue -= lcStep) {
						uint64_t size = test_literal_size(literalBuffer, literalCount, literalModel, lcValue, positionContextBitMask, freqCostTable);
						if (size >= bestSize) {
							break;
						}
						bestLcValue = lcValue;
						bestSize = size;
					}
				}

				pbLcSectors[i].lcValue = bestLcValue;
			}

			delete[] literalBuffer;
			delete[] literalModel;
			return 0;
		}

		void get_block_info(const uint8_t* const input, uint8_t* pb, uint8_t* lc, size_t* size) {
			if (input >= pbLcSectors[currentSector + 1].start)
				currentSector++;

			*pb = pbLcSectors[currentSector].pbMask;
			*lc = pbLcSectors[currentSector].lcValue;
			*size = std::min((size_t)(pbLcSectors[currentSector + 1].start - input), (size_t)STRIDER_MAX_BLOCK_SIZE);
		}
	};

	// too many models
	void reset_models(nibble_model* literalRunLengthHigh, uint8_t* matchLiteralContext, nibble_model* literalModel,
		uint8_t literalContextBitsShift, uint8_t positionContextBitMask, size_t* lastDistance, size_t* repOffsets,
		nibble_model* distanceModel, nibble_model* distanceLow, nibble_model* matchLengthHigh) {

		for (size_t i = 0; i < 193; i++) { literalRunLengthHigh[i].init(); }
		*matchLiteralContext = 0b1000;  //Match, literal run
		for (size_t i = 0; i < ((48 << 8) >> literalContextBitsShift) * (positionContextBitMask + 1); i++) { literalModel[i].init(); }
		*lastDistance = 1;
		for (size_t i = 0; i < 8; i++) { repOffsets[i] = 1; }
		for (size_t i = 0; i < 24; i++) { distanceModel[i].init(); }
		distanceLow->init();
		for (size_t i = 0; i < 291; i++) { matchLengthHigh[i].init(); }
	}

	void store_block_header(size_t blockSize, uint8_t positionContextBitMask, uint8_t literalContextBitShift,
		bool resetModels, bool uncompressedBlock, uint8_t*& output)
	{
		writeUint16LE(output, blockSize - 1);
		uint8_t metadata = uncompressedBlock | (resetModels << 1);
		metadata |= (int_log2(positionContextBitMask + 1) * 9 + literalContextBitShift) << 2;
		output[2] = metadata;
		output += 3;
	}

	FORCE_INLINE void strider_encode_literal_run(RansEncoder* encoder, const uint8_t* input, 
		nibble_model* literalRunLengthHigh, size_t literalRunLength, uint8_t* matchLiteralContext, 
		nibble_model* literalModel, const size_t lastDistance, const uint8_t literalContextBitsShift, 
		const uint8_t positionContextBitMask, size_t* positionContext) {

		input -= literalRunLength;
		*positionContext = reinterpret_cast<size_t>(input) & positionContextBitMask;

		if (literalRunLength >= 15) {
			literalRunLength -= 14;  //Allows for some simplifications
			size_t symbol = int_log2(literalRunLength);
			encoder->encode_nibble(15, &literalRunLengthHigh[*matchLiteralContext * 16 | *positionContext]);
			encoder->encode_nibble(symbol, &literalRunLengthHigh[192]);
			encoder->encode_raw_bits(literalRunLength & ((size_t)1 << symbol) - 1, symbol);
			literalRunLength += 14;
		}
		else {
			encoder->encode_nibble(literalRunLength, &literalRunLengthHigh[*matchLiteralContext * 16 | *positionContext]);
		}

		if (literalRunLength) {

			*matchLiteralContext >>= 2;
			//Also used as the previous byte to reduce memory access
			uint8_t literal = input[-1];

			do {
				const uint8_t exclude = *(input - lastDistance);
				nibble_model* const literalModelTree = 
					&literalModel[(((*positionContext << 8) | literal) >> literalContextBitsShift) * 48];
				literal = *input;

				encoder->encode_nibble(literal >> 4, &literalModelTree[exclude >> 4]);
				encoder->encode_nibble(literal & 0xF, 
					&literalModelTree[(exclude >> 4) == (literal >> 4) ? 32 | (exclude & 0xF) : 16 | (literal >> 4)]);

				input++;
				*positionContext = reinterpret_cast<size_t>(input) & positionContextBitMask;
				literalRunLength--;
			} while (literalRunLength);
		}
	}

	FORCE_INLINE void strider_encode_match(RansEncoder* encoder, const uint8_t* const input, uint8_t* matchLiteralContext, 
		const uint8_t positionContext, size_t* repOffsets, nibble_model* distanceModel, nibble_model* distanceLow, 
		nibble_model* matchLengthHigh, size_t matchLength, size_t distance)
	{

		size_t rep = std::find(repOffsets, repOffsets + 8, distance) - repOffsets;
		size_t matchLengthContext;

		if (rep < 8) {

			encoder->encode_nibble(rep, &distanceModel[*matchLiteralContext]);
			*matchLiteralContext = (*matchLiteralContext >> 2) | 4;
			matchLengthContext = 0;

			for (; rep > 0; rep--)
				repOffsets[rep] = repOffsets[rep - 1];
			repOffsets[0] = distance;
		}
		else {

			std::copy_backward(repOffsets + 5, repOffsets + 7, repOffsets + 8);
			repOffsets[5] = distance;

			distance--;
			if (distance < 256) {
				size_t symbol = distance / 16;
				encoder->encode_nibble(8, &distanceModel[*matchLiteralContext]);
				encoder->encode_nibble(symbol, &distanceModel[16]);
				matchLengthContext = 1;
			}
			else {
				size_t logarithm = int_log2(distance);
				size_t rawBits = logarithm - 5;
				size_t symbol = logarithm * 2 + ((distance >> logarithm - 1) & 1);
				size_t symbolHigh = symbol >> 4;
				encoder->encode_nibble(8 | symbolHigh, &distanceModel[*matchLiteralContext]);
				size_t symbolLow = symbol & 0xF;
				encoder->encode_nibble(symbolLow, &distanceModel[16 | symbolHigh]);
				encoder->encode_raw_bits((distance >> 4) & (((size_t)1 << rawBits) - 1), rawBits);
				matchLengthContext = 1 + logarithm / 8;
			}

			encoder->encode_nibble(distance & 0xF, distanceLow);
			*matchLiteralContext = (*matchLiteralContext >> 2) | 8;
		}

		const size_t shortLengthContext = matchLengthContext * 16 | positionContext;

		if (matchLength > 16) {
			encoder->encode_nibble(15, &matchLengthHigh[shortLengthContext]);
			if (matchLength > 31) {
				encoder->encode_nibble(15, &matchLengthHigh[144 + shortLengthContext]);
				matchLength -= 31;
				size_t symbol = int_log2(matchLength);
				encoder->encode_nibble(symbol, &matchLengthHigh[288 + std::min(matchLengthContext, (size_t)2)]);
				encoder->encode_raw_bits(matchLength & ((size_t)1 << symbol) - 1, symbol);
			}
			else {
				matchLength -= 17;
				encoder->encode_nibble(matchLength, &matchLengthHigh[144 + shortLengthContext]);
			}
		}
		else {
			matchLength -= STRIDER_MIN_LENGTH;
			encoder->encode_nibble(matchLength, &matchLengthHigh[shortLengthContext]);
		}
	}

	template<class IntType>
	size_t strider_compress_greedy(const uint8_t* input, const size_t size, uint8_t* output, 
		const CompressorOptions& compressorOptions, ProgressCallback* progress, const int window) {

		RansEncoder encoder;
		encoder.initialize_rans_encoder(std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 1.1 + 32,
			std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 2.5 + 64);
		if (size > 32768)
			encoder.initialize_fast_rans_encoder();

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. Its progress will be reported at the end
		*output++ = *input++;

		PbLcDetector pbLcDetector;
		if (pbLcDetector.init(input, size - STRIDER_LAST_BYTES - 1))
			return 0;

		//Model stuff
		bool doModelReset = true;
		nibble_model literalRunLengthHigh[193];
		nibble_model* literalModel = nullptr;
		nibble_model distanceModel[24];
		nibble_model distanceLow;
		nibble_model matchLengthHigh[291];

		uint8_t matchLiteralContext = 0;
		uint8_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		uint8_t positionContextBitMask = -1;
		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

		const size_t log2Size = std::min((int)int_log2(size) - 3, compressorOptions.maxHashTableSize);
		HashTable<IntType, FastIntHash> lzdict;
		try {
			lzdict.init(log2Size);
		}
		catch (std::bad_alloc& e) {
			return -1;
		}

		for (; input < compressionLimit; ) {

			uint8_t* const compressedBlockStart = output;
			const uint8_t* const thisBlockStart = input;

			size_t thisBlockSize;
			uint8_t newLiteralContextBitShift, newPositionContextBitMask;
			pbLcDetector.get_block_info(input, &newPositionContextBitMask, &newLiteralContextBitShift, &thisBlockSize);
			thisBlockSize = std::min(thisBlockSize, (size_t)STRIDER_MAX_BLOCK_SIZE);
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//The size of the literal model has been modified
			if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {

				delete[] literalModel;
				try {
					literalModel = new nibble_model[((48 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
				}
				catch (const std::bad_alloc& e) {
					return -1;
				}
			}

			//We have to reset everything
			if (doModelReset) {
				reset_models(literalRunLengthHigh, &matchLiteralContext, literalModel, newLiteralContextBitShift,
					newPositionContextBitMask, &distance, repOffsets, distanceModel, &distanceLow, matchLengthHigh);
			}
			//Only reset certain models if needed
			else {
				//These only require position bit mask
				if (positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < 192; i++) 
						literalRunLengthHigh[i].init();
					for (size_t i = 0; i < 288; i++) 
						matchLengthHigh[i].init();
				}
				if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < ((48 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1); i++) 
						literalModel[i].init();
				}
			}

			literalContextBitsShift = newLiteralContextBitShift;
			positionContextBitMask = newPositionContextBitMask;

			store_block_header(thisBlockSize, positionContextBitMask, literalContextBitsShift, doModelReset, 0, output);
			encoder.start_rans(output);

			doModelReset = false;
			const uint8_t* literalRunStart = input;

			for (; input < thisBlockEnd; ) {

				IntType* const dictEntry = &lzdict[readHash6(input)];
				const uint8_t* match = inputStart + *dictEntry;
				size_t matchLength = test_match(input, match, thisBlockEnd, 6, window);

				//We have found a match
				if (matchLength) {

					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;

					strider_encode_literal_run(&encoder, input, literalRunLengthHigh, literalRunLength, &matchLiteralContext, 
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

					distance = input - match;
					//Update the hash table for every position
					const uint8_t* const matchEnd = input + matchLength;
					input = distance < matchLength ? matchEnd - distance : input;
					for (; input < matchEnd; input++)
						lzdict[readHash6(input)] = input - inputStart;

					input = matchEnd;
					literalRunStart = input;
					//Output the match
					strider_encode_match(&encoder, input, &matchLiteralContext, positionContext, repOffsets, 
						distanceModel, &distanceLow, matchLengthHigh, matchLength, distance);
				}
				else {
					*dictEntry = input - inputStart;
					input++;
				}
			}

			size_t positionContext;
			size_t literalRunLength = input - literalRunStart;
			strider_encode_literal_run(&encoder, input, literalRunLengthHigh, literalRunLength, &matchLiteralContext,
				literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

			size_t compressedBlockSize = encoder.end_rans();
			output += compressedBlockSize;

			/*If the algorithm ends up expanding the data, store it uncompressed and reset all models*/
			if (input - thisBlockStart <= compressedBlockSize) {
				doModelReset = true;
				output = compressedBlockStart;
				store_block_header(thisBlockSize, 0, 0, 0, 1, output);
				memcpy(output, thisBlockStart, thisBlockSize);
				input = thisBlockStart + thisBlockSize;
				output += thisBlockSize;
			}

			if (progress->abort()) {
				delete[] literalModel;
				return 0;
			}
			progress->progress(input - inputStart);
		}

		memcpy(output, input, STRIDER_LAST_BYTES);
		progress->progress(input - inputStart + STRIDER_LAST_BYTES);
		delete[] literalModel;

		return output - outputStart + STRIDER_LAST_BYTES;
	}

	//Try to find the longest match for a given position. Then try to find an even longer one in the next.
	//If it is found, code a literal, and the longer one found. If not, code the original.
	//Note: limit defines the end of the input stream, blockLimit simply this block
	/*template<class IntType>
	FORCE_INLINE void strider_fast_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		const uint8_t* const blockLimit, LzCacheTable<IntType, FastIntHash>& lzdict6, size_t& bestLength, size_t& bestDistance,
		size_t& lazySteps, size_t& testedPositions, const CompressorOptions compressorOptions) {

		bestLength = 0;

		LzCacheBucket<IntType> dictEntry6 = lzdict6[readHash6(input)];

		size_t pos = input - inputStart;
		while (!dictEntry6.ended()) {
			dictEntry6.next(pos);

			const uint8_t* where = inputStart + pos;

			//Simple heuristic: as we are looking for a longer match, we can first
			//test the byte that would make this match longer. There is a high
			//chance it will differ, so the rest of the match wont need to be tested
			if (*(input + bestLength) != *(where + bestLength))
				continue;

			const size_t length = test_match(input, where, limit, 6);

			if (length > bestLength) {
				bestDistance = input - where;
				bestLength = length;

				//If we reach a certain length, simply code that match
				if (length >= compressorOptions.standardNiceLength) {
					while (!dictEntry6.ended())
						dictEntry6.next(pos);
					lazySteps = 0;
					testedPositions = 1;
					return;
				}
			}
		}

		//Nothing was found, code a literal and try again from the begining
		if (bestLength < 6) {
			testedPositions = 1;
			lazySteps = 1;
			return;
		}

		input++;
		//Block end reached, search must be stopped
		if (input == blockLimit) {
			lazySteps = 0;
			testedPositions = 1;
			return;
		}
		//Now try to find a longer match
		lazySteps = 0;
		testedPositions = 2;
		dictEntry6 = lzdict6[readHash6(input)];
		pos = input - inputStart;
		while (!dictEntry6.ended()) {
			dictEntry6.next(pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + bestLength) != *(where + bestLength))
				continue;

			const size_t length = test_match(input, where, limit, 6);

			if (length > bestLength) {
				lazySteps = 1;
				bestDistance = input - where;
				bestLength = length;

				if (length >= compressorOptions.standardNiceLength) {
					while (!dictEntry6.ended())
						dictEntry6.next(pos);
					break;
				}
			}
		}
		return;
	}

	//Same principle as the last function, but with additional heuristics to improve ratio
	template<class IntType>
	FORCE_INLINE void strider_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit, const uint8_t* const blockLimit,
		LzCacheTable<IntType, FastIntHash>& lzdict5, LzCacheTable<IntType, FastIntHash>& lzdict8,
		size_t& bestLength, size_t& bestDistance, size_t repOffsetA, size_t repOffsetB,
		size_t& lazySteps, size_t& testedPositions, const CompressorOptions compressorOptions) {

		bestLength = 0;
		size_t bestMatchCost = 0;

		for (testedPositions = 0; testedPositions < 2 && input < blockLimit; testedPositions++, input++) {
			LzCacheBucket<IntType> chain5 = lzdict5[readHash5(input)];
			LzCacheBucket<IntType> chain8 = lzdict8[readHash8(input)];

			//First try to find a match in any of the rep offsets. If it is found simply take it
			//Note: it should only skip after reaching a minimum length, but this doesnt seem to affect ratio...
			size_t length = test_match(input, input - repOffsetA, limit, 4);
			size_t matchCost = 2 + testedPositions;
			if (length + bestMatchCost > matchCost + bestLength) {
				bestDistance = repOffsetA;
				bestLength = length;
				chain5.push(input - inputStart);
				chain8.push(input - inputStart);
				lazySteps = testedPositions;
				bestMatchCost = matchCost;
				testedPositions++;
				return;
			}

			length = test_match(input, input - repOffsetB, limit, 4);
			if (length + bestMatchCost > matchCost + bestLength) {
				bestDistance = repOffsetB;
				bestLength = length;
				chain5.push(input - inputStart);
				chain8.push(input - inputStart);
				lazySteps = testedPositions;
				testedPositions++;
				return;
			}

			//If no rep offset was found try to get a length 8 match
			size_t pos = input - inputStart;
			while (!chain8.ended()) {
				chain8.next(pos);

				const uint8_t* where = inputStart + pos;

				if (*(input + bestLength) != *(where + bestLength))
					continue;

				length = test_match(input, where, limit, 8);

				size_t distance = input - where;
				matchCost = 3 + testedPositions + int_log2(distance) / 4;
				if (length + bestMatchCost > matchCost + bestLength) {
					bestDistance = distance;
					bestLength = length;
					bestMatchCost = matchCost;
					lazySteps = testedPositions;

					if (bestLength > compressorOptions.standardNiceLength) {
						while (!chain8.ended())
							chain8.next(pos);
						chain5.push(input - inputStart);
						testedPositions++;
						return;
					}
				}
			}

			//If still nothing was found, try a length 5
			if (bestLength < 8) {

				pos = input - inputStart;
				while (!chain5.ended()) {
					chain5.next(pos);

					const uint8_t* where = inputStart + pos;

					if (*(input + bestLength) != *(where + bestLength))
						continue;

					size_t length = test_match(input, where, limit, 5);

					size_t distance = input - where;
					matchCost = 3 + testedPositions + int_log2(distance) / 4;
					if (length + bestMatchCost > matchCost + bestLength) {
						bestDistance = distance;
						bestLength = length;
						bestMatchCost = matchCost;
						lazySteps = testedPositions;

						if (bestLength >= 7) {
							if (bestLength > compressorOptions.standardNiceLength) {
								while (!chain5.ended())
									chain5.next(pos);
								//chain8 has already been updated
								testedPositions++;
								return;
							}
							break;
						}
					}
				}

				//No match found, code a literal and retry
				if (bestLength < 5) {
					testedPositions++;
					lazySteps = 1;
					return;
				}
			}
			else {
				chain5.push(input - inputStart);
			}
		}
	}

	template<class IntType>
	struct StriderFastOptimalState {
		uint16_t sizeCost;
		uint16_t literalRunLength;
		uint32_t matchLength;
		IntType repOffsets[3];   //Match distance will always be the first offset stored here
	};

	template<class IntType>
	LZ_Structure<IntType>* strider_forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit, const uint8_t* const blockLimit,
		HashTableMatchFinder<IntType>& matchFinder, StriderFastOptimalState<IntType>* parser, LZ_Structure<IntType>* stream,
		size_t lastDistance, size_t* repOffsets, const CompressorOptions compressorOptions) {

		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		for (size_t i = 1; i <= blockLength; i++)
			parser[i].sizeCost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastMatchStart;

		parser[0].sizeCost = 0;
		//They might not be the last 3 rep offsets but whatever
		parser[0].repOffsets[0] = lastDistance;
		parser[0].repOffsets[1] = repOffsets[0];
		parser[0].repOffsets[2] = repOffsets[4];
		parser[0].literalRunLength = 0;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			StriderFastOptimalState<IntType>* parserPosition = parser + position;

			size_t literalSizeCost = parserPosition->sizeCost + 6;
			StriderFastOptimalState<IntType>* nextPosition = parserPosition + 1;
			if (literalSizeCost < nextPosition->sizeCost) {
				nextPosition->matchLength = 0;
				nextPosition->sizeCost = literalSizeCost;
				memcpy(nextPosition->repOffsets, parserPosition->repOffsets, 3 * sizeof(IntType));
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
			}

			size_t highestLength = 0;
			size_t whichRep;
			//Try to find rep offsets
			for (size_t i = 0; i < 3; i++) {
				size_t repMatchLength = test_match(inputPosition, inputPosition - parserPosition->repOffsets[i], limit, 2);
				if (repMatchLength > highestLength) {
					highestLength = repMatchLength;
					whichRep = i;
				}
			}

			if (highestLength) {
				size_t repDistance = parserPosition->repOffsets[whichRep];
				if (position + highestLength >= blockLength || highestLength >= compressorOptions.repNiceLength) {
					lastMatchLength = highestLength;
					lastMatchDistance = repDistance;
					lastMatchStart = position;
					goto doBackwardParse;
				}

				const size_t matchSizeCost = parserPosition->sizeCost + 9 + whichRep;  //Increase cost depending with each successive offset

				nextPosition = parserPosition + highestLength;
				if (matchSizeCost < nextPosition->sizeCost) {
					nextPosition->sizeCost = matchSizeCost;
					nextPosition->matchLength = highestLength;
					nextPosition->repOffsets[0] = repDistance;
					memcpy(&nextPosition->repOffsets[1], parserPosition->repOffsets, whichRep * sizeof(IntType));
					memcpy(&nextPosition->repOffsets[whichRep + 1], &parserPosition->repOffsets[whichRep + 1], (2 - whichRep) * sizeof(IntType));
					nextPosition->literalRunLength = 0;
				}
			}

			LZ_Match<IntType> matches[32];
			const LZ_Match<IntType>* matchesEnd = matchFinder.find_matches_and_update(inputPosition, inputStart, limit, matches, highestLength, compressorOptions);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZ_Match<IntType>* const longestMatch = matchesEnd - 1;
				//Match goes outside the buffer or is very long
				if (position + longestMatch->length >= blockLength || longestMatch->length >= compressorOptions.standardNiceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastMatchStart = position;
					break;
				}

				for (const LZ_Match<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					const size_t matchSizeCost = parserPosition->sizeCost + 11 + int_log2((size_t)matchIt->distance - 1);

					nextPosition = parserPosition + matchIt->length;
					if (matchSizeCost < nextPosition->sizeCost) {
						nextPosition->sizeCost = matchSizeCost;
						nextPosition->matchLength = matchIt->length;
						nextPosition->repOffsets[0] = matchIt->distance;
						memcpy(&nextPosition->repOffsets[1], parserPosition->repOffsets, 2 * sizeof(IntType));
						nextPosition->literalRunLength = 0;
					}
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const StriderFastOptimalState<IntType>* backwardParse = parser + position;
		const StriderFastOptimalState<IntType>* const parseEnd = parser;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			stream->literalRunLength = 0;
			stream++;
			backwardParse = parser + lastMatchStart;

			const uint8_t* inputPosition = input + lastMatchStart;
			const uint8_t* const matchEnd = inputPosition + lastMatchLength;
			for (inputPosition++; inputPosition < std::min(blockLimit, matchEnd); inputPosition++)
				matchFinder.update_position(inputPosition, inputStart);
		}
		else if (backwardParse->matchLength < 1) {
			stream->matchDistance = 0;
			stream->matchLength = 0;
			stream->literalRunLength = 0;
			stream++;
		}
		else {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = backwardParse->repOffsets[0];
			stream->matchLength = backwardParse->matchLength;
			stream->literalRunLength = 0;
			stream++;
			backwardParse -= backwardParse->matchLength;
		}

		while (backwardParse > parseEnd) {
			if (backwardParse->matchLength >= 1) {
				stream->matchDistance = backwardParse->repOffsets[0];
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

	template<class IntType>
	struct StriderOptimalParserState {
		uint32_t sizeCost;
		uint32_t matchLength;
		IntType distance;           //Also used as last distance
		uint32_t literalRunLength;
		uint32_t currentLiteralRunLengthCost;  //Cache so it does not get computed twice
		uint32_t matchLiteralContext;
		uint32_t prevPath;
		IntType repOffsets[8];
	};

	FORCE_INLINE size_t get_literal_run_cost(size_t literalRunLength, const size_t positionContext, const size_t matchLiteralContext,
		nibble_model* literalRunLengthHigh, const uint16_t* freqCostTable) {

		if (literalRunLength >= 15) {
			literalRunLength -= 11;  //Allows for some simplifications
			size_t symbol = int_log2(literalRunLength) - 2;
			size_t rawBits = symbol + 2;

			return freqCostTable[literalRunLengthHigh[matchLiteralContext * 16 | positionContext].get_freq(15)]
				+ freqCostTable[literalRunLengthHigh[192].get_freq(symbol)] + STRIDER_COST_PRECISION * rawBits;
		}
		else
			return freqCostTable[literalRunLengthHigh[matchLiteralContext * 16 | positionContext].get_freq(literalRunLength)];
	}

	FORCE_INLINE size_t get_match_length_cost(size_t length, const size_t positionContext, const size_t matchLengthContext,
		nibble_model* matchLengthHigh, const uint16_t* freqCostTable) {

		const size_t shortLengthContext = matchLengthContext * 16 | positionContext;

		if (length > 16) {
			size_t cost = freqCostTable[matchLengthHigh[shortLengthContext].get_freq(15)];
			if (length > 31) {
				cost += freqCostTable[matchLengthHigh[144 + shortLengthContext].get_freq(15)];
				if (length > 61) {
					cost += freqCostTable[matchLengthHigh[288 + matchLengthContext].get_freq(15)];
					if (length > 65595) {
						cost += freqCostTable[matchLengthHigh[297 + std::min(matchLengthContext, (size_t)2)].get_freq(15)];

						length -= 65595;
						cost += STRIDER_COST_PRECISION * 17;
						length >>= 16;

						while (length) {
							cost += STRIDER_COST_PRECISION * 4;
							length >>= 3;
						}
						return cost;
					}
					else {
						length -= 60;
						const size_t symbol = int_log2(length) - 1;
						const size_t rawBits = symbol + 1;

						return STRIDER_COST_PRECISION * rawBits + cost
							+ freqCostTable[matchLengthHigh[297 + std::min(matchLengthContext, (size_t)2)].get_freq(symbol)];
					}
				}
				else {
					length -= 32;
					return STRIDER_COST_PRECISION + cost
						+ freqCostTable[matchLengthHigh[288 + matchLengthContext].get_freq(length / 2)];
				}
			}
			else {
				length -= 17;
				return cost + freqCostTable[matchLengthHigh[144 + shortLengthContext].get_freq(length)];
			}
		}
		else {
			length -= STRIDER_MIN_LENGTH;
			return freqCostTable[matchLengthHigh[shortLengthContext].get_freq(length)];
		}
	}

	FORCE_INLINE size_t get_distance_cost(size_t distance, nibble_model* distanceModel,
		nibble_model* distanceLow, const size_t matchLiteralContext, uint16_t* freqCostTable) {

		size_t cost = 0;

		distance--;
		if (distance < 256) {
			size_t symbol = distance / 16;
			cost += freqCostTable[distanceModel[matchLiteralContext].get_freq(8)];
			cost += freqCostTable[distanceModel[16].get_freq(symbol)];
			cost += freqCostTable[distanceLow[std::min(symbol, (size_t)2)].get_freq(distance & 0xF)];
		}
		else {
			size_t logarithm = int_log2(distance);
			size_t rawBits = logarithm - 5;
			size_t symbol = logarithm * 2 + ((distance >> logarithm - 1) & 1);

			size_t symbolHigh = symbol >> 4;
			cost += freqCostTable[distanceModel[matchLiteralContext].get_freq(8 | symbolHigh)];
			size_t symbolLow = symbol & 0xF;
			cost += freqCostTable[distanceModel[16 | symbolHigh].get_freq(symbolLow)];
			cost += rawBits * STRIDER_COST_PRECISION;
			cost += freqCostTable[distanceLow[2].get_freq(distance & 0xF)];
		}

		return cost;
	}

	template<class IntType>
	LZ_Structure<IntType>* strider_priced_forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit, const uint8_t* const blockLimit,
		BinaryMatchFinder<IntType>& matchFinder, StriderOptimalParserState<IntType>* parser, LZ_Structure<IntType>* stream, const CompressorOptions compressorOptions, uint16_t* freqCostTable,
		nibble_model* literalRunLengthHigh, size_t startingLiteralRunLength, size_t matchLiteralContext, nibble_model* literalModel,
		size_t literalContextBitsShift, size_t positionContextBitMask, size_t lastDistance, size_t* repOffsets, nibble_model* distanceModel,
		nibble_model* distanceLow, nibble_model* matchLengthHigh) {

		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		for (size_t i = 1; i <= blockLength; i++)
			parser[i].sizeCost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastMatchStart;

		// While computing the cost of a new literal it is possible that due to substracting the previous run length
		// we get a negative number. To avoid that simply add some starting cost, instead of 0. This has no other effect on the parse
		parser[0].sizeCost = 256 * STRIDER_COST_PRECISION;
		parser[0].distance = lastDistance;
		for (size_t i = 0; i < 8; i++) { parser[0].repOffsets[i] = repOffsets[i]; }
		parser[0].literalRunLength = startingLiteralRunLength;
		parser[0].currentLiteralRunLengthCost = get_literal_run_cost(startingLiteralRunLength, reinterpret_cast<size_t>(input) & positionContextBitMask, matchLiteralContext, literalRunLengthHigh, freqCostTable);
		parser[0].matchLiteralContext = matchLiteralContext;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			StriderOptimalParserState<IntType>* parserPosition = parser + position;
			const size_t positionContext = reinterpret_cast<size_t>(inputPosition) & positionContextBitMask;

			const size_t literal = *inputPosition;
			const size_t literalHigh = literal >> 4;
			const size_t exclude = *(inputPosition - parserPosition->distance);
			const size_t excludeHigh = exclude >> 4;
			const size_t excludeLow = exclude & 0xF;

			nibble_model* const literalModelTree = &literalModel[(((positionContext << 8) | inputPosition[-1]) >> literalContextBitsShift) * 48];
			size_t thisLiteralSize = freqCostTable[literalModelTree[excludeHigh].get_freq(literalHigh)] +
				freqCostTable[literalModelTree[excludeHigh == literalHigh ? 32 | excludeLow : 16 | literalHigh].get_freq(literal & 0xF)];

			const size_t literalRunLengthStartPositionContext = reinterpret_cast<size_t>(inputPosition - parserPosition->literalRunLength) & positionContextBitMask;
			const size_t newLiteralRunLengthCost = get_literal_run_cost(parserPosition->literalRunLength + 1, literalRunLengthStartPositionContext, parserPosition->matchLiteralContext, literalRunLengthHigh, freqCostTable);

			thisLiteralSize += parserPosition->sizeCost + newLiteralRunLengthCost - parserPosition->currentLiteralRunLengthCost;

			StriderOptimalParserState<IntType>* nextPosition = parserPosition + 1;
			if (thisLiteralSize < nextPosition->sizeCost) {
				nextPosition->sizeCost = thisLiteralSize;
				nextPosition->distance = parserPosition->distance;  //Remember this is also the last used distance
				nextPosition->matchLength = 0;
				memcpy(nextPosition->repOffsets, parserPosition->repOffsets, 8 * sizeof(IntType));
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
				nextPosition->currentLiteralRunLengthCost = newLiteralRunLengthCost;
				nextPosition->matchLiteralContext = parserPosition->matchLiteralContext;
			}

			size_t highestLength = 1;
			size_t whichRep;

			for (size_t i = 0; i < 8; i++) {
				size_t repMatchLength = test_match(inputPosition, inputPosition - parserPosition->repOffsets[i], limit, 2);
				if (repMatchLength > highestLength) {
					highestLength = repMatchLength;
					whichRep = i;
				}
			}

			if (highestLength >= 2) {
				size_t repDistance = parserPosition->repOffsets[whichRep];

				if (highestLength >= compressorOptions.repNiceLength || position + highestLength >= blockLength) {
					lastMatchLength = highestLength;
					lastMatchDistance = repDistance;
					lastMatchStart = position;
					goto doBackwardParse;
				}

				size_t currentMatchLiteralContext = parserPosition->matchLiteralContext >> (parserPosition->literalRunLength > 0) * 2;  //After literal run
				const size_t offsetCost = freqCostTable[distanceModel[currentMatchLiteralContext].get_freq(whichRep)];
				currentMatchLiteralContext = (currentMatchLiteralContext >> 2) | 4;  //After rep match

				const size_t lengthCost = get_match_length_cost(highestLength, positionContext, 0, matchLengthHigh, freqCostTable);
				const size_t nextLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + highestLength) & positionContextBitMask;
				const size_t literalRunLengthCost = freqCostTable[literalRunLengthHigh[currentMatchLiteralContext * 16 | nextLiteralRunPositionContext].get_freq(0)];
				const size_t matchSizeCost = parserPosition->sizeCost + lengthCost + offsetCost + literalRunLengthCost;

				nextPosition = parserPosition + highestLength;
				if (matchSizeCost < nextPosition->sizeCost) {
					nextPosition->sizeCost = matchSizeCost;
					nextPosition->matchLength = highestLength;
					nextPosition->distance = repDistance;
					nextPosition->repOffsets[0] = repDistance;
					memcpy(&nextPosition->repOffsets[1], parserPosition->repOffsets, whichRep * sizeof(IntType));
					memcpy(&nextPosition->repOffsets[whichRep + 1], &parserPosition->repOffsets[whichRep + 1], (7 - whichRep) * sizeof(IntType));
					nextPosition->literalRunLength = 0;
					nextPosition->currentLiteralRunLengthCost = literalRunLengthCost;
					nextPosition->matchLiteralContext = currentMatchLiteralContext;
				}
			}

			LZ_Match<IntType> matches[144];
			const LZ_Match<IntType>* matchesEnd = matchFinder.find_matches_and_update(inputPosition, inputStart, limit, matches,
				highestLength, compressorOptions);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZ_Match<IntType>* const longestMatch = matchesEnd - 1;
				//Match goes outside the buffer or is very long
				if (position + longestMatch->length >= blockLength || longestMatch->length >= compressorOptions.standardNiceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastMatchStart = position;
					goto doBackwardParse;
				}

				const size_t currentMatchLiteralContext = parserPosition->matchLiteralContext >> (parserPosition->literalRunLength > 0) * 2;  //After literal run
				const size_t nextMatchLiteralContext = (currentMatchLiteralContext >> 2) | 8;  //After match

				for (const LZ_Match<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					const size_t matchLengthContext = 1 + int_log2((size_t)matchIt->distance) / 8;
					const size_t lengthCost = get_match_length_cost(matchIt->length, positionContext, matchLengthContext, matchLengthHigh, freqCostTable);
					const size_t distanceCost = get_distance_cost(matchIt->distance, distanceModel, distanceLow, currentMatchLiteralContext, freqCostTable);
					const size_t nextLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + matchIt->length) & positionContextBitMask;
					const size_t literalRunLengthCost = freqCostTable[literalRunLengthHigh[nextMatchLiteralContext * 16 | nextLiteralRunPositionContext].get_freq(0)];
					const size_t matchSizeCost = parserPosition->sizeCost + distanceCost + lengthCost + literalRunLengthCost;

					nextPosition = parserPosition + matchIt->length;
					if (matchSizeCost < nextPosition->sizeCost) {
						nextPosition->sizeCost = matchSizeCost;
						nextPosition->matchLength = matchIt->length;
						nextPosition->distance = matchIt->distance;
						memcpy(nextPosition->repOffsets, parserPosition->repOffsets, 5 * sizeof(IntType));
						nextPosition->repOffsets[5] = matchIt->distance;
						memcpy(&nextPosition->repOffsets[6], &parserPosition->repOffsets[5], 2 * sizeof(IntType));
						nextPosition->literalRunLength = 0;
						nextPosition->currentLiteralRunLengthCost = literalRunLengthCost;
						nextPosition->matchLiteralContext = nextMatchLiteralContext;
					}
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const StriderOptimalParserState<IntType>* backwardParse = parser + position;
		const StriderOptimalParserState<IntType>* const parseEnd = parser;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			stream->literalRunLength = 0;
			stream++;
			backwardParse = parser + lastMatchStart;

			const uint8_t* inputPosition = input + lastMatchStart;
			const uint8_t* const matchEnd = inputPosition + lastMatchLength;
			for (inputPosition++; inputPosition < std::min(blockLimit, matchEnd); inputPosition++) {
				matchFinder.update_position(inputPosition, inputStart, limit, compressorOptions);
			}
		}
		else if (backwardParse->matchLength < 1) {
			stream->matchDistance = 0;
			stream->matchLength = 0;
			stream->literalRunLength = 0;
			stream++;
		}
		else {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = backwardParse->distance;
			stream->matchLength = backwardParse->matchLength;
			stream->literalRunLength = 0;
			stream++;
			backwardParse -= backwardParse->matchLength;
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

	template<class IntType>
	LZ_Structure<IntType>* strider_multi_arrivals_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, const uint8_t* const blockLimit, BinaryMatchFinder<IntType>& matchFinder,
		StriderOptimalParserState<IntType>* parser, LZ_Structure<IntType>* stream, const CompressorOptions compressorOptions,
		uint16_t* freqCostTable, nibble_model* literalRunLengthHigh, size_t startingLiteralRunLength, size_t matchLiteralContext,
		nibble_model* literalModel, size_t literalContextBitsShift, size_t positionContextBitMask, size_t lastDistance,
		size_t* repOffsets, nibble_model* distanceModel, nibble_model* distanceLow, nibble_model* matchLengthHigh) {

		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		for (size_t i = 1; i <= blockLength * compressorOptions.maxArrivals; i++)
			parser[i].sizeCost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastMatchStart;
		size_t lastPath;

		for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {
			// While computing the cost of a new literal it is possible that due to substracting the previous run length
			// we get a negative number. To avoid that simply add some starting cost, instead of 0. This has no other effect on the parse
			parser[i].sizeCost = 256 * STRIDER_COST_PRECISION;
			for (size_t j = 0; j < 8; j++) { parser[i].repOffsets[j] = repOffsets[j]; }
			parser[i].literalRunLength = startingLiteralRunLength;
			parser[i].currentLiteralRunLengthCost = get_literal_run_cost(startingLiteralRunLength, reinterpret_cast<size_t>(input) & positionContextBitMask, matchLiteralContext, literalRunLengthHigh, freqCostTable);
			parser[i].distance = lastDistance;
			parser[i].matchLiteralContext = matchLiteralContext;
		}

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			const size_t positionContext = reinterpret_cast<size_t>(inputPosition) & positionContextBitMask;
			StriderOptimalParserState<IntType>* parserPosition = parser + position * compressorOptions.maxArrivals;

			//Only try lengths that are at least this long for rep matches
			size_t acceptableRepMatchLength = 2;
			nibble_model* const literalModelTree = &literalModel[(((positionContext << 8) | inputPosition[-1]) >> literalContextBitsShift) * 48];

			//Literal and rep match parsing can be done at the same time
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				StriderOptimalParserState<IntType>* currentArrival = parserPosition + i;

				const size_t literal = *inputPosition;
				const size_t literalHigh = literal >> 4;
				const size_t exclude = *(inputPosition - currentArrival->distance);
				const size_t excludeHigh = exclude >> 4;
				const size_t excludeLow = exclude & 0xF;

				size_t thisLiteralSize = freqCostTable[literalModelTree[excludeHigh].get_freq(literalHigh)] +
					freqCostTable[literalModelTree[excludeHigh == literalHigh ? 32 | excludeLow : 16 | literalHigh].get_freq(literal & 0xF)];

				const size_t literalRunLengthStartPositionContext = reinterpret_cast<size_t>(inputPosition - currentArrival->literalRunLength) & positionContextBitMask;
				const size_t newLiteralRunLengthCost = get_literal_run_cost(currentArrival->literalRunLength + 1, literalRunLengthStartPositionContext, currentArrival->matchLiteralContext, literalRunLengthHigh, freqCostTable);

				thisLiteralSize += currentArrival->sizeCost + newLiteralRunLengthCost - currentArrival->currentLiteralRunLengthCost;

				StriderOptimalParserState<IntType>* arrivalIt = parserPosition + compressorOptions.maxArrivals;
				StriderOptimalParserState<IntType>* const lastArrival = arrivalIt + compressorOptions.maxArrivals;

				for (; arrivalIt < lastArrival; arrivalIt++) {

					if (thisLiteralSize < arrivalIt->sizeCost) {

						std::copy_backward(arrivalIt, lastArrival - 1, lastArrival);

						arrivalIt->sizeCost = thisLiteralSize;
						arrivalIt->matchLength = 0;
						memcpy(arrivalIt->repOffsets, currentArrival->repOffsets, 8 * sizeof(IntType));
						arrivalIt->literalRunLength = currentArrival->literalRunLength + 1;
						arrivalIt->currentLiteralRunLengthCost = newLiteralRunLengthCost;
						arrivalIt->distance = currentArrival->distance;
						arrivalIt->matchLiteralContext = currentArrival->matchLiteralContext;
						arrivalIt->prevPath = i;
						break;
					}
				}

				//Try to find rep offsets
				for (size_t j = 0; j < 8; j++) {
					size_t repMatchLength = test_match(inputPosition, inputPosition - currentArrival->repOffsets[j], limit, 2);
					if (repMatchLength >= acceptableRepMatchLength) {
						size_t repDistance = currentArrival->repOffsets[j];

						if (repMatchLength >= compressorOptions.repNiceLength || position + repMatchLength >= blockLength) {
							lastMatchLength = repMatchLength;
							lastMatchDistance = repDistance;
							lastMatchStart = position;
							lastPath = i;
							goto doBackwardParse;
						}

						size_t currentMatchLiteralContext = currentArrival->matchLiteralContext >> (currentArrival->literalRunLength > 0) * 2;
						const size_t baseMatchCost = currentArrival->sizeCost + freqCostTable[distanceModel[currentMatchLiteralContext].get_freq(j)];
						currentMatchLiteralContext = (currentMatchLiteralContext >> 2) | 4;

						size_t matchLengthReductionLimit = repMatchLength - std::min((size_t)3, repMatchLength - acceptableRepMatchLength);
						for (size_t length = repMatchLength; length >= matchLengthReductionLimit; length--) {

							const size_t lengthCost = get_match_length_cost(length, positionContext, 0, matchLengthHigh, freqCostTable);
							const size_t newLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + length) & positionContextBitMask;
							const size_t literalRunLengthCost = freqCostTable[literalRunLengthHigh[currentMatchLiteralContext * 16 | newLiteralRunPositionContext].get_freq(0)];
							const size_t matchSizeCost = baseMatchCost + lengthCost + literalRunLengthCost;

							StriderOptimalParserState<IntType>* arrivalIt = parserPosition + length * compressorOptions.maxArrivals;
							StriderOptimalParserState<IntType>* const lastArrival = arrivalIt + compressorOptions.maxArrivals;

							bool betterArrival = false;

							for (; arrivalIt < lastArrival; arrivalIt++) {

								if (matchSizeCost < arrivalIt->sizeCost) {

									betterArrival = true;

									std::copy_backward(arrivalIt, lastArrival - 1, lastArrival);

									arrivalIt->sizeCost = matchSizeCost;
									arrivalIt->matchLength = length;
									arrivalIt->distance = repDistance;
									arrivalIt->repOffsets[0] = repDistance;
									memcpy(&arrivalIt->repOffsets[1], currentArrival->repOffsets, j * sizeof(IntType));
									memcpy(&arrivalIt->repOffsets[j + 1], &currentArrival->repOffsets[j + 1], (7 - j) * sizeof(IntType));
									arrivalIt->literalRunLength = 0;
									arrivalIt->currentLiteralRunLengthCost = literalRunLengthCost;
									arrivalIt->matchLiteralContext = currentMatchLiteralContext;
									arrivalIt->prevPath = i;
									break;
								}
							}

							if (!betterArrival)
								break;
						}

						acceptableRepMatchLength = repMatchLength;
					}
				}
			}

			LZ_Match<IntType> matches[144];
			const LZ_Match<IntType>* matchesEnd =
				matchFinder.find_matches_and_update(inputPosition, inputStart, limit, matches, 1, compressorOptions);

			//At least one match was found
			if (matchesEnd != matches) {

				if (compressorOptions.parserFunction == OPTIMAL_ULTRA) {
					//Longest length can be anywhere
					size_t longestLength = 0;
					size_t distance;
					for (LZ_Match<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {
						if (matchIt->length > longestLength) {
							longestLength = matchIt->length;
							distance = matchIt->distance;
						}
					}
					if (position + longestLength >= blockLength || longestLength >= compressorOptions.standardNiceLength) {
						lastMatchLength = longestLength;
						lastMatchDistance = distance;
						lastMatchStart = position;
						lastPath = 0;
						break;
					}
				}
				else {
					//The last match should be the longest
					const LZ_Match<IntType>* const longestMatch = matchesEnd - 1;
					//Match goes outside the buffer or is very long
					if (position + longestMatch->length >= blockLength || longestMatch->length >= compressorOptions.standardNiceLength) {
						lastMatchLength = longestMatch->length;
						lastMatchDistance = longestMatch->distance;
						lastMatchStart = position;
						lastPath = 0;
						goto doBackwardParse;
					}
				}

				size_t matchLengthReductionLimit = 1;
				const size_t currentMatchLiteralContext = parserPosition->matchLiteralContext >> (parserPosition->literalRunLength > 0) * 2;  //After literal run
				const size_t nextMatchLiteralContext = (currentMatchLiteralContext >> 2) | 8;  //After match

				for (const LZ_Match<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->repOffsets[0] || matchIt->distance == parserPosition->repOffsets[1] ||
						matchIt->distance == parserPosition->repOffsets[2] || matchIt->distance == parserPosition->repOffsets[3] ||
						matchIt->distance == parserPosition->repOffsets[4] || matchIt->distance == parserPosition->repOffsets[5] ||
						matchIt->distance == parserPosition->repOffsets[6] || matchIt->distance == parserPosition->repOffsets[7])
						continue;

					const size_t matchLengthContext = 1 + int_log2((size_t)matchIt->distance) / 8;
					const size_t baseMatchCost = parserPosition->sizeCost + get_distance_cost(matchIt->distance, distanceModel, distanceLow, currentMatchLiteralContext, freqCostTable);

					if (compressorOptions.parserFunction == OPTIMAL_ULTRA)
						matchLengthReductionLimit = matchIt->length - std::min((size_t)matchIt->length - 1, (size_t)4);
					else
						matchLengthReductionLimit = matchIt->length - std::min((size_t)4, matchIt->length - matchLengthReductionLimit);
					//Start search at the highest length. Stop when we reach the previous length or
					// the current match length does not improve any arrival.
					for (size_t length = matchIt->length; length > matchLengthReductionLimit; length--) {

						const size_t lengthCost = get_match_length_cost(length, positionContext, matchLengthContext, matchLengthHigh, freqCostTable);
						const size_t newLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + length) & positionContextBitMask;
						const size_t literalRunLengthCost = freqCostTable[literalRunLengthHigh[nextMatchLiteralContext * 16 | newLiteralRunPositionContext].get_freq(0)];
						const size_t matchSizeCost = baseMatchCost + lengthCost + literalRunLengthCost;

						StriderOptimalParserState<IntType>* arrivalIt = parserPosition + length * compressorOptions.maxArrivals;
						StriderOptimalParserState<IntType>* const lastArrival = arrivalIt + compressorOptions.maxArrivals;

						bool betterArrival = false;

						for (; arrivalIt < lastArrival; arrivalIt++) {

							if (matchSizeCost < arrivalIt->sizeCost) {

								betterArrival = true;

								std::copy_backward(arrivalIt, lastArrival - 1, lastArrival);

								arrivalIt->sizeCost = matchSizeCost;
								arrivalIt->matchLength = length;
								arrivalIt->distance = matchIt->distance;
								memcpy(arrivalIt->repOffsets, parserPosition->repOffsets, 5 * sizeof(IntType));
								arrivalIt->repOffsets[5] = matchIt->distance;
								memcpy(&arrivalIt->repOffsets[6], &parserPosition->repOffsets[5], 2 * sizeof(IntType));
								arrivalIt->literalRunLength = 0;
								arrivalIt->currentLiteralRunLengthCost = literalRunLengthCost;
								arrivalIt->matchLiteralContext = nextMatchLiteralContext;
								arrivalIt->prevPath = 0;
								break;
							}
						}

						if (!betterArrival)
							break;
					}
					matchLengthReductionLimit = matchIt->length;
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const StriderOptimalParserState<IntType>* backwardParse = parser + position * compressorOptions.maxArrivals;
		const StriderOptimalParserState<IntType>* const parseEnd = parser;

		size_t path = 0;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			stream->literalRunLength = 0;
			stream++;
			backwardParse = parser + lastMatchStart * compressorOptions.maxArrivals;
			path = lastPath;

			const uint8_t* inputPosition = input + lastMatchStart;
			const uint8_t* const matchEnd = inputPosition + lastMatchLength;
			for (inputPosition++; inputPosition < std::min(blockLimit, matchEnd); inputPosition++)
				matchFinder.update_position(inputPosition, inputStart, limit, compressorOptions);
		}
		else if (backwardParse[path].matchLength < 1) {
			stream->matchDistance = 0;
			stream->matchLength = 0;
			stream->literalRunLength = 0;
			stream++;
			path = backwardParse[path].prevPath;
		}
		else {
			stream->literalRunLength = 0;
			stream++;
			size_t matchLength = backwardParse[path].matchLength;
			stream->matchDistance = backwardParse[path].distance;
			stream->matchLength = matchLength;
			stream->literalRunLength = 0;
			stream++;
			path = backwardParse[path].prevPath;
			backwardParse -= matchLength * compressorOptions.maxArrivals;
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

	template<class IntType>
	size_t strider_compress_generic(const uint8_t* input, const size_t size, uint8_t* output, const CompressorOptions compressorOptions, ProgressCallback* progress) {

		if (size <= STRIDER_LAST_BYTES + 1) {
			memcpy(output, input, size);
			progress->progress(size);
			return size;
		}

		RansEncoder encoder;
		encoder.initialize_rans_encoder(std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 1.1 + 32,
			std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 2.5 + 64);
		if (size > 32768)
			encoder.initialize_fast_rans_encoder();

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. Its progress will be reported at the end
		*output++ = *input++;

		PbLcDetector pbLcDetector;
		if (pbLcDetector.init(input, size - STRIDER_LAST_BYTES - 1))
			return 0;

		//Model stuff
		bool doModelReset = true;

		nibble_model literalRunLengthHigh[193];
		size_t matchLiteralContext = 0;

		nibble_model* literalModel = nullptr;
		size_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		size_t positionContextBitMask = -1;

		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };
		nibble_model distanceModel[24];
		nibble_model distanceLow;

		nibble_model matchLengthHigh[291];

		// * / * // * / * // * / * // * / * // * / * // * / * //
		//                   LAZY PARSER                      //
		// * / * // * / * // * / * // * / * // * / * // * / * //
		if (compressorOptions.parserFunction == LAZY_NORMAL || compressorOptions.parserFunction == LAZY_EXTRA) {

			const size_t log2Size = std::min((int)int_log2(size) - 3, compressorOptions.maxHashTableSize);
			LzCacheTable<IntType, FastIntHash> lzdict6;
			LzCacheTable<IntType, FastIntHash> lzdict5;
			LzCacheTable<IntType, FastIntHash> lzdict8;

			if (compressorOptions.parserFunction == LAZY_NORMAL) {
				lzdict6.init(log2Size, compressorOptions.maxElementsPerHash);
			}
			else {
				lzdict5.init(log2Size, compressorOptions.maxElementsPerHash);
				lzdict8.init(log2Size, compressorOptions.maxElementsPerHash);
			}

			for (; input < compressionLimit; ) {

				uint8_t* const compressedBlockStart = output;
				const uint8_t* rawBlockStart = input;

				size_t newLiteralContextBitShift, newPositionContextBitMask, thisBlockSize;
				pbLcDetector.get_current_pb_lc(input, newPositionContextBitMask, newLiteralContextBitShift, thisBlockSize);
				thisBlockSize = std::min(thisBlockSize, (size_t)STRIDER_MAX_BLOCK_SIZE);

				const uint8_t* rawBlockEnd = compressionLimit - input < STRIDER_MAX_BLOCK_SIZE ? compressionLimit : input + STRIDER_MAX_BLOCK_SIZE;
				rawBlockEnd = input + thisBlockSize;
				size_t compressedBlockSize;

				//The size of the literal model has been modified
				if ((literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) &&
					newLiteralContextBitShift != UNCOMPRESSED_LITERALS) {

					delete[] literalModel;
					try {
						literalModel = new nibble_model[((48 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
					}
					catch (const std::bad_alloc& e) {
						return -1;
					}
				}

				//We have to reset everything
				if (doModelReset) {
					//reset_models(literalRunLengthHigh, matchLiteralContext, literalModel, newLiteralContextBitShift,
					//	newPositionContextBitMask, distance, repOffsets, distanceModel, distanceLow, matchLengthHigh);
				}
				//Only reset certain models if needed
				else {
					//These only require position bit mask
					if (positionContextBitMask != newPositionContextBitMask) {
						for (size_t i = 0; i < 192; i++) { literalRunLengthHigh[i].init(); }
						for (size_t i = 0; i < 288; i++) { matchLengthHigh[i].init(); }
					}
					if ((literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) &&
						newLiteralContextBitShift != UNCOMPRESSED_LITERALS) {

						for (size_t i = 0; i < ((48 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1); i++) { literalModel[i].init(); }
					}
				}

				literalContextBitsShift = newLiteralContextBitShift;
				positionContextBitMask = newPositionContextBitMask;

				store_block_header(thisBlockSize, positionContextBitMask, literalContextBitsShift, doModelReset, 0, output);
				encoder.start_rans(output);

				doModelReset = false;
				const uint8_t* literalRunStart = input;

				//Last 2 offsets, used by the heavy lazy parser
				size_t repOffsetA = repOffsets[0];
				size_t repOffsetB = repOffsets[1];

				for (; input < rawBlockEnd; ) {

					size_t matchLength;
					size_t newDistance;
					size_t lazySteps;
					size_t testedPositions;

					if (compressorOptions.parserFunction == LAZY_NORMAL)
						strider_fast_lazy_search<IntType>(input, inputStart, compressionLimit, rawBlockEnd, lzdict6, matchLength, newDistance, lazySteps, testedPositions, compressorOptions);
					else
						strider_lazy_search<IntType>(input, inputStart, compressionLimit, rawBlockEnd, lzdict5, lzdict8, matchLength, newDistance, repOffsetA, repOffsetB, lazySteps, testedPositions, compressorOptions);

					input += lazySteps;

					//We have found a match
					if (matchLength) {

						size_t positionContext;
						size_t literalRunLength = input - literalRunStart;

						//strider_encode_literal_run(encoder, input, literalRunLengthHigh, literalRunLength, matchLiteralContext, literalModel, distance, literalContextBitsShift,
						//	positionContextBitMask, positionContext);

						distance = newDistance;
						//Update the hash table for every position, but only up to block end
						const uint8_t* const matchEnd = input + matchLength;
						if (matchEnd > rawBlockEnd) {
							if (compressorOptions.parserFunction == LAZY_NORMAL) {
								for (input++; input < rawBlockEnd; input++)
									lzdict6[readHash6(input)].push(input - inputStart);
							}
							else {
								for (input++; input < rawBlockEnd; input++) {
									lzdict5[readHash5(input)].push(input - inputStart);
									lzdict8[readHash8(input)].push(input - inputStart);
								}
							}
							input = matchEnd;
						}
						else {
							if (compressorOptions.parserFunction == LAZY_NORMAL) {
								for (input++; input < matchEnd; input++)
									lzdict6[readHash6(input)].push(input - inputStart);
							}
							else {
								for (input++; input < matchEnd; input++) {
									lzdict5[readHash5(input)].push(input - inputStart);
									lzdict8[readHash8(input)].push(input - inputStart);
								}
							}
						}

						repOffsetB = repOffsetA;
						repOffsetA = distance;

						literalRunStart = input;
						//Output the match
						//strider_encode_match(encoder, input, matchLiteralContext, positionContext, repOffsets, 
						//	distanceModel, distanceLow, matchLengthHigh, matchLength, distance);
					}
				}

				size_t positionContext;
				size_t literalRunLength = input - literalRunStart;
				//strider_encode_literal_run(encoder, input, literalRunLengthHigh, literalRunLength, matchLiteralContext, literalModel, distance, literalContextBitsShift,
				//	positionContextBitMask, positionContext);

				compressedBlockSize = encoder.end_rans();
				output += compressedBlockSize;

				//If the algorithm ends up expanding the data, store it uncompressed and reset all models
				if (input - rawBlockStart <= compressedBlockSize) {
					doModelReset = true;
					output = compressedBlockStart;
					store_block_header(thisBlockSize, 0, 0, 0, 1, output);
					input = rawBlockStart;
					memcpy(output, input, thisBlockSize);
					output += thisBlockSize;
					input += thisBlockSize;
				}
				else {
					if (compressorOptions.parserFunction == LAZY_NORMAL) {
						for (; rawBlockEnd < input; rawBlockEnd++)
							lzdict6[readHash6(rawBlockEnd)].push(rawBlockEnd - inputStart);
					}
					else {
						for (; rawBlockEnd < input; rawBlockEnd++) {
							lzdict5[readHash5(rawBlockEnd)].push(rawBlockEnd - inputStart);
							lzdict8[readHash8(rawBlockEnd)].push(rawBlockEnd - inputStart);
						}
					}
				}

				if (progress->abort()) {
					delete[] literalModel;
					return 0;
				}
				progress->progress(input - inputStart);
			}
		}
		// * / * // * / * // * / * // * / * // * / * // * / * //
		//                   OPTIMAL PARSER                   //
		// * / * // * / * // * / * // * / * // * / * // * / * //
		else if (compressorOptions.parserFunction >= OPTIMAL_FAST) {

			//Cost of encoding any symbol with a given frequency
			uint16_t* freqCostTable = nullptr;
			if (compressorOptions.parserFunction > OPTIMAL_FAST) {
				try {
					freqCostTable = new uint16_t[1 << MODEL_PRECISION_BITS];
				}
				catch (std::bad_alloc& e) {
					return 0;
				}
				if (size > 16384) {
					for (uint16_t i = 1; i < MODEL_SCALE; i++)
						freqCostTable[i] = (MODEL_PRECISION_BITS - std::log2((float)i)) * STRIDER_COST_PRECISION;
				}
				//Use a faster log so that this does not become the bottleneck
				else {
					for (uint16_t i = 1; i < MODEL_SCALE; i++)
						freqCostTable[i] = (MODEL_PRECISION_BITS - fast_log2((float)i)) * STRIDER_COST_PRECISION;
				}

				if (pbLcDetector.calculate_lc<IntType>(input, size, compressorOptions.parserFunction == OPTIMAL ? 4 : 1, freqCostTable)) {

					delete[] freqCostTable;
					return 0;
				}
			}

			HashTableMatchFinder<IntType> hashMatchFinder;
			BinaryMatchFinder<IntType> binaryMatchFinder;

			void* parser = nullptr;
			LZ_Structure<IntType>* stream = nullptr;

			try {
				if (compressorOptions.parserFunction == OPTIMAL_FAST) {
					hashMatchFinder.init(size, compressorOptions);
					parser = new StriderFastOptimalState<IntType>[compressorOptions.optimalBlockSize + 1];
				}
				else {
					binaryMatchFinder.init(size, compressorOptions);
					parser = new StriderOptimalParserState<IntType>[(compressorOptions.optimalBlockSize + 1) * compressorOptions.maxArrivals];
				}
				stream = new LZ_Structure<IntType>[compressorOptions.optimalBlockSize];
			}
			catch (const std::bad_alloc& e) {
				delete[] freqCostTable;
				delete[] parser;
				delete[] stream;
				return 0;
			}

			for (; input < compressionLimit; ) {

				uint8_t* const compressedBlockStart = output;
				const uint8_t* rawBlockStart = input;

				size_t newLiteralContextBitShift, newPositionContextBitMask, thisBlockSize;
				pbLcDetector.get_current_pb_lc(input, newPositionContextBitMask, newLiteralContextBitShift, thisBlockSize);
				thisBlockSize = std::min(thisBlockSize, (size_t)STRIDER_MAX_BLOCK_SIZE);

				const uint8_t* rawBlockEnd = compressionLimit - input < STRIDER_MAX_BLOCK_SIZE ? compressionLimit : input + STRIDER_MAX_BLOCK_SIZE;
				rawBlockEnd = input + thisBlockSize;
				size_t compressedBlockSize;

				//The size of the literal model has been modified
				if ((literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) &&
					newLiteralContextBitShift != UNCOMPRESSED_LITERALS) {

					delete[] literalModel;
					try {
						literalModel = new nibble_model[((48 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
					}
					catch (const std::bad_alloc& e) {
						return -1;
					}
				}

				//We have to reset everything
				if (doModelReset) {
					//reset_models(literalRunLengthHigh, matchLiteralContext, literalModel, newLiteralContextBitShift,
					//	newPositionContextBitMask, distance, repOffsets, distanceModel, distanceLow, matchLengthHigh);
				}
				//Only reset certain models if needed
				else {
					//These only require position bit mask
					if (positionContextBitMask != newPositionContextBitMask) {
						for (size_t i = 0; i < 192; i++) { literalRunLengthHigh[i].init(); }
						for (size_t i = 0; i < 288; i++) { matchLengthHigh[i].init(); }
					}
					if ((literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) &&
						newLiteralContextBitShift != UNCOMPRESSED_LITERALS) {

						for (size_t i = 0; i < ((48 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1); i++) { literalModel[i].init(); }
					}
				}

				literalContextBitsShift = newLiteralContextBitShift;
				positionContextBitMask = newPositionContextBitMask;

				store_block_header(thisBlockSize, positionContextBitMask, literalContextBitsShift, doModelReset, 0, output);
				encoder.start_rans(output);

				doModelReset = false;
				const uint8_t* literalRunStart = input;

				for (; input < rawBlockEnd; ) {

					LZ_Structure<IntType>* streamIt;
					if (compressorOptions.parserFunction == OPTIMAL_FAST) {
						streamIt = strider_forward_optimal_parse<IntType>(input, inputStart, compressionLimit, rawBlockEnd, hashMatchFinder,
							(StriderFastOptimalState<IntType>*)parser, stream, distance, repOffsets, compressorOptions);
					}
					else if (compressorOptions.parserFunction == OPTIMAL) {
						streamIt = strider_priced_forward_optimal_parse<IntType>(input, inputStart, compressionLimit, rawBlockEnd, binaryMatchFinder,
							(StriderOptimalParserState<IntType>*)parser, stream, compressorOptions, freqCostTable, literalRunLengthHigh, input - literalRunStart,
							matchLiteralContext, literalModel, literalContextBitsShift, positionContextBitMask, distance, repOffsets,
							distanceModel, &distanceLow, matchLengthHigh);
					}
					else {
						streamIt = strider_multi_arrivals_parse<IntType>(input, inputStart, compressionLimit, rawBlockEnd, binaryMatchFinder,
							(StriderOptimalParserState<IntType>*)parser, stream, compressorOptions, freqCostTable, literalRunLengthHigh, input - literalRunStart,
							matchLiteralContext, literalModel, literalContextBitsShift, positionContextBitMask, distance, repOffsets,
							distanceModel, &distanceLow, matchLengthHigh);
					}

					//Main compression loop
					while (true) {
						input += streamIt->literalRunLength;

						if (streamIt == stream)
							break;

						size_t positionContext;
						size_t literalRunLength = input - literalRunStart;

						//strider_encode_literal_run(encoder, input, literalRunLengthHigh, literalRunLength, matchLiteralContext, literalModel, distance, literalContextBitsShift,
						//	positionContextBitMask, positionContext);

						size_t matchLength = streamIt->matchLength;
						distance = streamIt->matchDistance;
						input += matchLength;
						literalRunStart = input;
						//Output the match
						//strider_encode_match(encoder, input, matchLiteralContext, positionContext, repOffsets, distanceModel,
						//	distanceLow, matchLengthHigh, matchLength, distance);

						streamIt--;
					}
				}

				size_t positionContext;
				size_t literalRunLength = input - literalRunStart;
				//strider_encode_literal_run(encoder, input, literalRunLengthHigh, literalRunLength, matchLiteralContext, literalModel, distance, literalContextBitsShift,
				//	positionContextBitMask, positionContext);

				compressedBlockSize = encoder.end_rans();
				output += compressedBlockSize;

				//If the algorithm ends up expanding the data, store it uncompressed and reset all models
				if (input - rawBlockStart <= compressedBlockSize) {
					doModelReset = true;
					output = compressedBlockStart;
					store_block_header(thisBlockSize, 0, 0, 0, 1, output);
					input = rawBlockStart;
					memcpy(output, input, thisBlockSize);
					output += thisBlockSize;
					input += thisBlockSize;
				}
				else {
					if (compressorOptions.parserFunction == OPTIMAL_FAST) {
						for (; rawBlockEnd < input; rawBlockEnd++)
							hashMatchFinder.update_position(rawBlockEnd, inputStart);
					}
					else {
						for (; rawBlockEnd < input; rawBlockEnd++)
							binaryMatchFinder.update_position(rawBlockEnd, inputStart, compressionLimit, compressorOptions);
					}
				}

				if (progress->abort()) {
					delete[] freqCostTable;
					delete[] parser;
					delete[] stream;
					delete[] literalModel;
					return 0;
				}
				progress->progress(input - inputStart);
			}

			delete[] freqCostTable;
			delete[] parser;
			delete[] stream;
		}

		memcpy(output, input, STRIDER_LAST_BYTES);
		progress->progress(input - inputStart + STRIDER_LAST_BYTES); //Report the first byte

		delete[] literalModel;

		return output - outputStart + STRIDER_LAST_BYTES;
	}*/

	const int NOT_USED = -1;
	const CompressorOptions striderCompressorLevels[] = {
		//      Parser        Hash log     Elements per hash     Nice length     Rep nice length      Block size      Max arrivals
			{ GREEDY       ,     17     ,      NOT_USED       ,    NOT_USED   ,      NOT_USED     ,     NOT_USED    ,   NOT_USED   },
			{ LAZY_NORMAL  ,     17     ,          1          ,       16      ,      NOT_USED     ,     NOT_USED    ,   NOT_USED   },
			{ LAZY_EXTRA   ,     17     ,          1          ,       16      ,      NOT_USED     ,     NOT_USED    ,   NOT_USED   },
			{ LAZY_EXTRA   ,     18     ,          2          ,       24      ,      NOT_USED     ,     NOT_USED    ,   NOT_USED   },
			{ OPTIMAL_FAST ,     18     ,          2          ,       24      ,         8         ,       1024      ,   NOT_USED   },
			{ OPTIMAL_FAST ,     19     ,          3          ,       32      ,         16        ,       1024      ,   NOT_USED   },
			{ OPTIMAL      ,     22     ,          5          ,       32      ,         16        ,       2048      ,   NOT_USED   },
			{ OPTIMAL      ,     24     ,          6          ,       64      ,         32        ,       2048      ,   NOT_USED   },
			{ OPTIMAL_BRUTE,     24     ,          6          ,       64      ,         32        ,       4096      ,       3      },
			{ OPTIMAL_BRUTE,     26     ,          7          ,       256     ,         128       ,       4096      ,       8      },
			{ OPTIMAL_ULTRA,     31     ,          7          ,       1024    ,         1024      ,       4096      ,       24     },
	};

	size_t strider_compress(const uint8_t* input, const size_t size, uint8_t* output, const int level, 
		ProgressCallback* progress, const int window) {

		if (level > 10 || level < 0 || window > (IS_64BIT ? 63 : 31) || window < 0)
			return 0;

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		if (size <= STRIDER_LAST_BYTES + 1) {
			memcpy(output, input, size);
			progress->progress(size);
			return size;
		}

#ifdef IS_64BIT
		if (size > ((uint64_t)1 << 32)) {
			if (striderCompressorLevels[level].parserFunction == GREEDY)
				return strider_compress_greedy<uint64_t>(input, size, output, striderCompressorLevels[level], progress, window);
		}
#endif
		if (striderCompressorLevels[level].parserFunction == GREEDY)
			return strider_compress_greedy<uint32_t>(input, size, output, striderCompressorLevels[level], progress, window);
	}

	size_t strider_compress_bound(const size_t size) {
		return size + size / 128 + 32;
	}

	template<class IntType>
	size_t strider_parser_memory_estimator(const size_t size, const int level, const int window) {
		if (striderCompressorLevels[level].parserFunction == GREEDY)
			return sizeof(IntType) << MIN3((int)int_log2(size) - 3, striderCompressorLevels[level].maxHashTableSize, window - 3);
	}

	size_t strider_estimate_memory(const size_t size, const int level, const int window) {

		if (level > 10 || level < 0 || window > (IS_64BIT ? 63 : 31) || window < 0)
			return 0;

		if (size <= STRIDER_LAST_BYTES + 1)
			return 0;

		size_t blockSize = std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size);
		size_t memory = blockSize * 1.1 + 32;   //stream buffer
		memory += (blockSize * 2.5 + 64) * sizeof(uint32_t);  //model buffer
		memory += sizeof(nibble_model) * 48 * (striderCompressorLevels[level].parserFunction > OPTIMAL_FAST ? 256 : 16) * 16;  //literal model, lc on non optimal levels is set to 4
		if (size > MODEL_SCALE * 3)
			memory += sizeof(uint64_t) * MODEL_SCALE;  //fast rans encoder

#ifdef IS_64BIT
		if (size > ((uint64_t)1 << 32)) {
			if (striderCompressorLevels[level].parserFunction == GREEDY)
				return strider_parser_memory_estimator<uint64_t>(size, level, window);
		}
#endif
		if (striderCompressorLevels[level].parserFunction == GREEDY)
			return strider_parser_memory_estimator<uint32_t>(size, level, window);
		/*if (striderCompressorLevels[level].parserFunction < OPTIMAL_FAST)
			//Lazy extra uses 2 tables
			return memory + (sizeof(uint32_t) << std::min((size_t)int_log2((size_t)size) - 3, (size_t)striderCompressorLevels[level].maxHashTableSize)
				<< striderCompressorLevels[level].maxElementsPerHash << (striderCompressorLevels[level].parserFunction == LAZY_EXTRA));
		if (striderCompressorLevels[level].parserFunction < OPTIMAL) {
			const size_t log2size = std::min((size_t)int_log2((size_t)size) - 3, (size_t)striderCompressorLevels[level].maxHashTableSize);
			memory += sizeof(uint32_t) << std::min(log2size, (size_t)14);  //hash 3 table
			memory += sizeof(uint32_t) << log2size << striderCompressorLevels[level].maxElementsPerHash << 1;  //One table for hash 4 and one for hash 8
			memory += sizeof(StriderFastOptimalState<uint32_t>) * striderCompressorLevels[level].optimalBlockSize + 1;
			memory += sizeof(LZ_Structure<uint32_t>) * striderCompressorLevels[level].optimalBlockSize;
			return memory;
		}
		const size_t binaryTreeSize = std::min((size_t)size, (size_t)1 << striderCompressorLevels[level].maxHashTableSize);
		memory += sizeof(uint32_t) * 2 * binaryTreeSize; //binary tree
		memory += sizeof(uint32_t) << std::min((size_t)int_log2((size_t)size) - 3, (size_t)striderCompressorLevels[level].maxHashTableSize - 3);  //binary node lookup
		memory += sizeof(uint32_t) << std::min((size_t)int_log2((size_t)size) - 3, (size_t)16);  //hash 3 table
		memory += sizeof(uint32_t) << std::min((size_t)int_log2((size_t)size) - 3, (size_t)12);  //hash 2 table
		if (size > binaryTreeSize + 32)
			memory += sizeof(uint32_t) << std::max(4, (int)int_log2(size - binaryTreeSize) - 4) <<
			(striderCompressorLevels[level].maxElementsPerHash - 4);  //extra hash 12 table
		memory += sizeof(StriderOptimalParserState<uint32_t>) * striderCompressorLevels[level].optimalBlockSize * striderCompressorLevels[level].maxArrivals + striderCompressorLevels[level].maxArrivals;
		memory += sizeof(LZ_Structure<uint32_t>) * striderCompressorLevels[level].optimalBlockSize;
		return memory;*/
	}

	class RansDecoder {
		uint64_t stateA;
		uint64_t stateB;
		const uint8_t* compressedStreamIt;
		const uint8_t* compressedStreamEnd;

	public:
		RansDecoder() {}
		~RansDecoder() {}

		//rans block functions
		void start_rans(const uint8_t* compressed, const uint8_t* compressedEnd) {
			compressedStreamIt = compressed;
			compressedStreamEnd = compressedEnd;

			stateA = readUint64LE(compressedStreamIt + 0);
			stateB = readUint64LE(compressedStreamIt + 8);
			compressedStreamIt += 16;
		}
		FORCE_INLINE void normalize() {
			if (stateB < RANS_NORMALIZATION_INTERVAL) {
				stateB <<= 32;
				stateB |= readUint32LE(compressedStreamIt);
				compressedStreamIt += 4;
			}
		}
		//Maximum of 62 bits
		FORCE_INLINE size_t decode_raw_bits(size_t nBits) {

			if (nBits > 31) {

				std::swap(stateA, stateB);
				size_t symbol = stateB & (((size_t)1 << 31) - 1);
				stateB >>= 31;
				normalize();
				nBits -= 31;

				std::swap(stateA, stateB);
				symbol |= (stateB & (((size_t)1 << nBits) - 1)) << 31;
				stateB >>= nBits;
				normalize();
				return symbol;
			}

			std::swap(stateA, stateB);
			size_t symbol = stateB & (((size_t)1 << nBits) - 1);
			stateB >>= nBits;
			normalize();
			return symbol;
		}
		FORCE_INLINE size_t decode_nibble(nibble_model* model) {
			std::swap(stateA, stateB);
			const uint16_t stateLow = stateB & MODEL_BIT_MASK;
			uint16_t freq, low;
			size_t symbol = model->lookup_and_update(stateLow, &low, &freq);
			stateB = freq * (stateB >> MODEL_PRECISION_BITS) + stateLow - low;
			normalize();
			return symbol;
		}
		FORCE_INLINE bool check_stream_overflow() {
			return compressedStreamIt > compressedStreamEnd;
		}
		const uint8_t* end_rans() {
			return compressedStreamIt;
		}
	};

	const int8_t inc32table[16] = { 0, 1, 2, 1, 0,  4,  4,  4, 4, 4, 4, 4, 4, 4, 4, 4 };
	const int8_t inc64table[16] = { 0, 0, 0, 1, 4, -1, -2, -3, 4, 4, 4, 4, 4, 4, 4, 4 };

	//Note that this function will write beyond the end of the match. It is important to keep
	//a buffer of at least 32 bytes after the end of match. But it is much faster than a memcpy()
	FORCE_INLINE void copy_match(uint8_t* dst, const size_t offset, const size_t length) {

		uint8_t* const end = dst + length;
		const uint8_t* src = dst - offset;

		//If the offset is big enough we can perform a faster copy
		if (offset >= 16) {
			//Should be translated to some sse/neon or whatever-that-loads-stores-16-bytes instructions
			do {
				memcpy(dst, src, 16);
				memcpy(dst + 16, src + 16, 16);
				src += 32;
				dst += 32;
			} while (dst < end);
		}
		//Else it is a run-length type match
		else {
			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];
			dst[3] = src[3];
			src += inc32table[offset];
			memcpy(dst + 4, src, 4);
			src += inc64table[offset];
			memcpy(dst + 8, src, 8);

			dst += 16;
			src += 8;
			do {
				memcpy(dst, src, 8);
				memcpy(dst + 8, src + 8, 8);
				src += 16;
				dst += 16;
			} while (dst < end);
		}
	}

	int strider_decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed, 
		const size_t uncompressedSize, ProgressCallback* progress) {

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		if (uncompressedSize <= STRIDER_LAST_BYTES + 1) {
			if (compressedSize < uncompressedSize)
				return -1;
			memcpy(decompressed, compressed, uncompressedSize);
			progress->progress(uncompressedSize);
			return 0;
		}

		if (compressedSize < STRIDER_LAST_BYTES + 1)
			return -1;

		RansDecoder ransDecoder;
		const uint8_t* const decompressedStart = decompressed;
		const uint8_t* const decompressedEnd = decompressed + uncompressedSize - STRIDER_LAST_BYTES;
		const uint8_t* const compressedEnd = compressed + compressedSize - STRIDER_LAST_BYTES;
		//First byte is uncompressed
		*decompressed++ = *compressed++;

		//Model stuff
		nibble_model literalRunLengthHigh[193];
		nibble_model* literalModel = nullptr;  //Also indicates whether this is the first rans block
		nibble_model distanceModel[24];
		nibble_model distanceLow;
		nibble_model matchLengthHigh[291];

		uint8_t matchLiteralContext;
		uint8_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		uint8_t positionContextBitMask = -1;
		size_t distance = 1;  //Also stores the last used distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

		while (decompressed < decompressedEnd) {

			const size_t thisBlockSize = readUint16LE(compressed) + 1;
			const uint8_t blockMetadata = compressed[2];
			compressed += 3;
			const uint8_t* const thisBlockEnd = decompressed + thisBlockSize;

			if (compressed > compressedEnd || decompressedEnd - decompressed < thisBlockSize) {
				delete[] literalModel;
				return -1;
			}

			//Uncompressed data
			if (blockMetadata & 1) {

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
				bool doModelReset = (blockMetadata & 2) || (!literalModel);
				size_t newLiteralContextBitsShift = (blockMetadata >> 2) % 9;
				size_t newPositionContextBitMask = (1 << ((blockMetadata >> 2) / 9)) - 1;

				if (newLiteralContextBitsShift > 8 || newPositionContextBitMask > 15) {
					delete[] literalModel;
					return -1;
				}

				//The size of the literal model has been modified
				if (literalContextBitsShift != newLiteralContextBitsShift || positionContextBitMask != newPositionContextBitMask) {

					delete[] literalModel;
					try {
						literalModel = new nibble_model[((48 << 8) >> newLiteralContextBitsShift) * (newPositionContextBitMask + 1)];
					}
					catch (const std::bad_alloc& e) {
						return -1;
					}
				}

				//We have to reset everything
				if (doModelReset) {
					reset_models(literalRunLengthHigh, &matchLiteralContext, literalModel, newLiteralContextBitsShift,
						newPositionContextBitMask, &distance, repOffsets, distanceModel, &distanceLow, matchLengthHigh);
				}
				//Only reset certain models if needed
				else {
					//These only require position bit mask
					if (positionContextBitMask != newPositionContextBitMask) {
						for (size_t i = 0; i < 192; i++) 
							literalRunLengthHigh[i].init(); 
						for (size_t i = 0; i < 288; i++) 
							matchLengthHigh[i].init(); 
					}
					if (literalContextBitsShift != newLiteralContextBitsShift || positionContextBitMask != newPositionContextBitMask) {
						for (size_t i = 0; i < ((48 << 8) >> newLiteralContextBitsShift) * (newPositionContextBitMask + 1); i++) 
							literalModel[i].init(); 
					}
				}

				literalContextBitsShift = newLiteralContextBitsShift;
				positionContextBitMask = newPositionContextBitMask;

				ransDecoder.start_rans(compressed, compressedEnd);
				if (ransDecoder.check_stream_overflow()) {
					delete[] literalModel;
					return -1;
				}

				while (true) {

					size_t positionContext = reinterpret_cast<size_t>(decompressed) & positionContextBitMask;
					const size_t literalRunContext = matchLiteralContext * 16 | positionContext;

					size_t literalRunLength = ransDecoder.decode_nibble(&literalRunLengthHigh[literalRunContext]);
					if (literalRunLength == 15) {
						size_t symbol = ransDecoder.decode_nibble(&literalRunLengthHigh[192]);
						literalRunLength = ((size_t)1 << symbol) + ransDecoder.decode_raw_bits(symbol);
						literalRunLength += 14;

						if (unlikely(thisBlockEnd - decompressed < literalRunLength)) {
							delete[] literalModel;
							return -1;
						}
					}

					if (literalRunLength) {

						matchLiteralContext >>= 2;
						//The literal we just have decoded is the previous byte
						uint8_t literal = decompressed[-1];

						do {
							const uint8_t exclude = *(decompressed - distance);
							nibble_model* const literalModelTree = 
								&literalModel[(((positionContext << 8) | literal) >> literalContextBitsShift) * 48];

							literal = ransDecoder.decode_nibble(&literalModelTree[exclude >> 4]);
							literal = (literal << 4) | ransDecoder.decode_nibble(
								&literalModelTree[literal == (exclude >> 4) ? 32 | (exclude & 0xF) : 16 | literal]);

							*decompressed++ = literal;
							positionContext = reinterpret_cast<size_t>(decompressed) & positionContextBitMask;
							literalRunLength--;

							//Check overflow for every decoded literal
							if (unlikely(ransDecoder.check_stream_overflow())) {
								delete[] literalModel;
								return -1;
							}
						} while (literalRunLength);
					}

					//A symbol can have a minimum frequency of 16/16384, that is, its decoding will consume at most 10 bits.
					//As both states might be just before renormalization interval, we should be able to read at least 192 bits
					// before having to check again for the buffer overflow.
					//A match (with 64 bit decode limit) can consume, at most, 131 bits plus 10 for the literal run 
					// (assumes that literal run length was 0, as stream overflow should be checked after every literal)
					if (unlikely(ransDecoder.check_stream_overflow())) {
						delete[] literalModel;
						return -1;
					}

					//This is for the usual case where literal run length < 15 and match length <= 16. 
					//Together, they do not overflow the buffer, so a single check here is enough.
					if (unlikely(decompressed >= thisBlockEnd)) {
						if (decompressed > thisBlockEnd) {
							delete[] literalModel;
							return -1;
						}
						break;
					}

					size_t distanceToken = ransDecoder.decode_nibble(&distanceModel[matchLiteralContext]);
					matchLiteralContext >>= 2;  //Remove the last event

					size_t matchLengthContext;

					if (distanceToken < 8) {
						distance = repOffsets[distanceToken];

						for (; distanceToken > 0; distanceToken--)
							repOffsets[distanceToken] = repOffsets[distanceToken - 1];
						repOffsets[0] = distance;

						matchLiteralContext |= 4;
						matchLengthContext = 0;
					}
					else {
						distanceToken -= 8;
						distanceToken = (distanceToken << 4) | ransDecoder.decode_nibble(&distanceModel[16 | distanceToken]);

						if (distanceToken < 16) {
							distance = 16 * distanceToken;
							matchLengthContext = 1;
						}
						else {
							const size_t rawBits = distanceToken / 2 - 5;
							distance = ((size_t)2 | (distanceToken & 1)) << rawBits;
							distance = (distance | ransDecoder.decode_raw_bits(rawBits)) << 4;
							matchLengthContext = 1 + distanceToken / 16;
						}
						distance |= ransDecoder.decode_nibble(&distanceLow);
						distance++;

						if (unlikely(decompressed - decompressedStart < distance)) {
							delete[] literalModel;
							return -1;
						}

						std::copy_backward(repOffsets + 5, repOffsets + 7, repOffsets + 8);
						repOffsets[5] = distance;

						matchLiteralContext |= 8;
					}

					const size_t shortLengthContext = matchLengthContext * 16 | positionContext;
					size_t matchLength = ransDecoder.decode_nibble(&matchLengthHigh[shortLengthContext]);

					if (matchLength == 15) {
						matchLength = ransDecoder.decode_nibble(&matchLengthHigh[144 + shortLengthContext]) + 17;
						if (matchLength == 32) {
							size_t symbol = ransDecoder.decode_nibble(&matchLengthHigh[288 + std::min(matchLengthContext, (size_t)2)]);
							matchLength = ((size_t)1 << symbol) + ransDecoder.decode_raw_bits(symbol) + 31;
						}
					}
					else {
						matchLength += STRIDER_MIN_LENGTH;
					}

					//Check overflow
					if (unlikely(thisBlockEnd - decompressed < matchLength)) {
						delete[] literalModel;
						return -1;
					}
					copy_match(decompressed, distance, matchLength);
					decompressed += matchLength;
				}

				compressed = ransDecoder.end_rans();
			}

			if (progress->abort()) {
				delete[] literalModel;
				return 0;
			}
			progress->progress(decompressed - decompressedStart);
		}

		memcpy(decompressed, compressed, STRIDER_LAST_BYTES);
		progress->progress(decompressed - decompressedStart + STRIDER_LAST_BYTES);

		delete[] literalModel;
		return 0;
	}
}

#endif  //STRIDER_IMPLEMENTATION

#endif  //__STRIDER__
