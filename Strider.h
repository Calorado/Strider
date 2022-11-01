/*
 * Strider Compression Algorithm v1.2.1
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
	//window determines the maximum backwards distance the matches can have, as a power of 2.
	//Larger values can improve compression, but use more memory.
	//Returns the size of the compressed stream or -1 on failure.
	size_t compress(const uint8_t* input, const size_t size, uint8_t* output, int level = 6,
		int window = 999, ProgressCallback* progress = nullptr);
	//Decompresses contents in "compressed" to "decompressed".
	//You may also pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns 0 on success or -1 on failure or corrupted data.
	int decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed,
		const size_t decompressedSize, ProgressCallback* progress = nullptr);

	//For a given input size, returns a size for the output buffer that is big enough to
	// contain the compressed stream even if it expands.
	size_t compress_bound(const size_t size);
	//Returns the amount of memory the algorithm will consume on compression.
	size_t estimate_memory(const size_t size, int level = 6, int window = 26);
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

#define MIN3(a, b, c) (std::min(a, std::min(b, c)))

namespace strider {

	bool is_little_endian() {
		const union { uint16_t u; uint8_t c[2]; } LITTLE_ENDIAN_CHECK = { 1 };
		return LITTLE_ENDIAN_CHECK.c[0];
	}

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
	FORCE_INLINE uint64_t unsafe_bit_scan_forward(const uint64_t value) {
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
	FORCE_INLINE uint64_t bit_scan_forward(const uint64_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_bit_scan_forward(value);
		return 0;
#endif  //Fallback already returns 0 when value == 0
		return unsafe_bit_scan_forward(value);
	}

	struct FastIntHash {
		//Use top bits
		uint64_t operator()(const uint64_t value) {
			return value * 0xff51afd7ed558ccd;
		}
	};

	//These functions are used to obtain the data for the hash. If the number of bytes is higher than the word size,
	//they will be mixed. Also note that these might read more bytes than necessary.
	FORCE_INLINE uint64_t read_hash8(const uint8_t* const ptr) {
		uint64_t value;
		memcpy(&value, ptr, 8);
		return value;
	}
	FORCE_INLINE uint64_t read_hash4(const uint8_t* const ptr) {
		uint32_t value;
		memcpy(&value, ptr, 4);
		return value;
	}
	FORCE_INLINE uint64_t read_hash2(const uint8_t* const ptr) {
		uint16_t value;
		memcpy(&value, ptr, 2);
		return value;
	}
	FORCE_INLINE uint64_t read_hash16(const uint8_t* const ptr) {
		return read_hash8(ptr) ^ read_hash8(ptr + 8);
	}
	FORCE_INLINE uint64_t read_hash6(const uint8_t* const ptr) {
		if (is_little_endian())
			return read_hash8(ptr) << 16;
		return read_hash8(ptr) >> 16;  //Assumes big endian
	}
	FORCE_INLINE uint64_t read_hash5(const uint8_t* const ptr) {
		if (is_little_endian())
			return read_hash8(ptr) << 24;
		return read_hash8(ptr) >> 24;  //Assumes big endian
	}
	FORCE_INLINE uint64_t read_hash3(const uint8_t* const ptr) {
		if (is_little_endian())
			return read_hash4(ptr) << 40;
		return read_hash4(ptr) >> 8;  //Assumes big endian
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

	FORCE_INLINE uint32_t unsafe_bit_scan_forward(uint32_t value) {
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

	FORCE_INLINE uint32_t bit_scan_forward(uint32_t value) {
#if defined(_MSC_VER) || defined(__GNUC__) || defined(__clang__)
		if (likely(value != 0))
			return unsafe_bit_scan_forward(value);
		return 0;
#endif
		return unsafe_bit_scan_forward(value);
	}

	struct FastIntHash {
		uint32_t operator()(const uint32_t value) {
			return value * 0x27d4eb2d;
		}
	};

	FORCE_INLINE uint32_t read_hash4(const uint8_t* const ptr) {
		uint32_t value;
		memcpy(&value, ptr, 4);
		return value;
	}
	FORCE_INLINE uint32_t read_hash2(const uint8_t* const ptr) {
		uint16_t value;
		memcpy(&value, ptr, 2);
		return value;
	}
	FORCE_INLINE uint32_t read_hash8(const uint8_t* const ptr) {
		return read_hash4(ptr) ^ read_hash4(ptr + 4);
	}
	FORCE_INLINE uint32_t read_hash16(const uint8_t* const ptr) {
		return read_hash8(ptr) ^ read_hash8(ptr + 8);
	}
	FORCE_INLINE uint32_t read_hash6(const uint8_t* const ptr) {
		uint16_t a;
		memcpy(&a, ptr + 0, 2);
		return read_hash4(ptr + 2) ^ a;
	}
	FORCE_INLINE uint32_t read_hash5(const uint8_t* const ptr) {
		return read_hash4(ptr) ^ ptr[4];
	}
	FORCE_INLINE uint32_t read_hash3(const uint8_t* const ptr) {
		if (is_little_endian())
			return read_hash4(ptr) << 8;
		return read_hash4(ptr) >> 8;  //Assumes big endian
	}

#endif

	FORCE_INLINE uint64_t read_uint64le(const uint8_t* const ptr) {
		if (is_little_endian()) {
			uint64_t value;
			memcpy(&value, ptr, 8);
			return value;
		}
		uint64_t value = 0;
		for (int i = 0; i < 8; i++)
			value |= (uint64_t)ptr[i] << i * 8;
		return value;
	}
	FORCE_INLINE void write_uint64le(uint8_t* const ptr, const uint64_t value) {
		if (is_little_endian())
			memcpy(ptr, &value, 8);
		else {
			for (int i = 0; i < 8; i++)
				ptr[i] = value >> i * 8;
		}
	}
	FORCE_INLINE uint32_t read_uint32le(const uint8_t* const ptr) {
		if (is_little_endian()) {
			uint32_t value;
			memcpy(&value, ptr, 4);
			return value;
		}
		uint32_t value = 0;
		for (int i = 0; i < 4; i++)
			value |= (uint64_t)ptr[i] << i * 8;
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

	//Tries to find a match between the two locations, and returns the length
	//Note that this function should be called with at least MIN_LENGTH + 8 bytes of buffer after limit
	FORCE_INLINE size_t test_match(const uint8_t* front, const uint8_t* back, const uint8_t* const limit,
		const size_t minLength, const int window) {

		//Test first bytes
		//Usually compilers will optimize std::equal as 2 comparisons for minLength 5, 6, etc
		//We can only use one comparison to make it faster. For powers of 2, std::equal should be good
		if (IS_64BIT && is_little_endian()) {
			switch (minLength) {
			case 6:
				if ((read_uint64le(front) << 16) != (read_uint64le(back) << 16) || ((size_t)(front - back) >> window))
					return 0;
				break;
			case 5:
				if ((read_uint64le(front) << 24) != (read_uint64le(back) << 24) || ((size_t)(front - back) >> window))
					return 0;
				break;
			case 3:
				if ((read_uint32le(front) << 8) != (read_uint32le(back) << 8) || ((size_t)(front - back) >> window))
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

		if (IS_64BIT && is_little_endian()) {
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
				const uint64_t xorVal = read_uint64le(front) ^ read_uint64le(back);

				if (xorVal) {
					front += unsafe_bit_scan_forward(xorVal) >> 3;
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

	const int LAZY1 = 0;
	const int LAZY2 = 1;
	const int LAZY3 = 2;
	const int OPTIMAL1 = 3;
	const int OPTIMAL2 = 4;
	const int OPTIMAL3 = 5;
	const int OPTIMAL4 = 6;

	struct CompressorOptions {
		int parserFunction;
		int maxHashTableSize;
		int maxElementsPerHash;
		int niceLength;
		int optimalBlockSize;        // (Optimal)
		int maxArrivals;             // (Optimal)
	};

	template<class IntType>
	struct LZStructure {
		IntType matchLength;
		IntType matchDistance;
		IntType literalRunLength;
	};

	template<class IntType>
	struct LZMatch {
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
	class LZCacheBucket {
		IntType* it;
		IntType* last;
	public:
		LZCacheBucket() {}
		LZCacheBucket(IntType* _begin, IntType* _end) {
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
	class LZCacheTable {
		IntType* arr = nullptr;
		size_t hashShift;
		size_t elementsPerBucket;  //log2
	public:
		//Use 2^x sizes to avoid the use of modulo and multiplication
		LZCacheTable() {}
		LZCacheTable(const size_t logSize, const size_t numElements) {
			init(logSize, numElements);
		}
		~LZCacheTable() {
			delete[] arr;
		}
		void init(const size_t logSize, const size_t numElements) {
			arr = new IntType[(size_t)1 << logSize << numElements]();
			hashShift = (IS_64BIT ? 64 : 32) - logSize;
			elementsPerBucket = numElements;
		}
		LZCacheBucket<IntType> operator[](size_t value) {
			value = Hash{}(value) >> hashShift;
			IntType* bucket = arr + (value << elementsPerBucket);
			return LZCacheBucket<IntType>(bucket, bucket + ((size_t)1 << elementsPerBucket));
		}
	};

	//Simple and fast
	template<class IntType>
	class HashTableMatchFinder {
		HashTable<IntType, FastIntHash> lzdict3;
		LZCacheTable<IntType, FastIntHash> lzdict4;
		LZCacheTable<IntType, FastIntHash> lzdict8;

	public:
		void init(const size_t size, const CompressorOptions& compressorOptions, const int window) {
			const int hashSize = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
			lzdict3.init(std::min(hashSize, 14));
			lzdict4.init(hashSize, compressorOptions.maxElementsPerHash);
			lzdict8.init(hashSize, compressorOptions.maxElementsPerHash);
		}

		LZMatch<IntType>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			LZMatch<IntType>* matches, size_t highestLength, const CompressorOptions& compressorOptions, const int window) {

			IntType& chain3 = lzdict3[read_hash3(input)];
			LZCacheBucket<IntType> chain4 = lzdict4[read_hash4(input)];
			LZCacheBucket<IntType> chain8 = lzdict8[read_hash8(input)];

			if (highestLength < 3) {
				const uint8_t* where = inputStart + chain3;
				size_t length = test_match(input, where, limit, 3, window);

				if (length > highestLength) {
					matches->distance = input - where;
					matches->length = length;
					matches++;

					highestLength = length;
					if (highestLength > compressorOptions.niceLength) {
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

			if (highestLength >= 4 && highestLength < compressorOptions.niceLength) {

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
						if (highestLength > compressorOptions.niceLength) {
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
			lzdict3[read_hash3(input)] = pos;
			lzdict4[read_hash4(input)].push(pos);
			lzdict8[read_hash8(input)].push(pos);
		}
	};

	const int NO_MATCH_POS = 0;

	//Original match finder implementation from BriefLZ
	template<class IntType>
	class BinaryMatchFinder {

		HashTable<IntType, FastIntHash> lzdict16;
		HashTable<IntType, FastIntHash> lzdict3;
		HashTable<IntType, FastIntHash> lzdict2;
		HashTable<IntType, FastIntHash> nodeLookup;
		IntType* nodes = nullptr;
		size_t nodeListSize;
		size_t nodeListMask;

	public:

		~BinaryMatchFinder() {
			delete[] nodes;
		}
		BinaryMatchFinder() {}

		void init(const size_t inputSize, const CompressorOptions& compressorOptions, const int window) {

			const size_t binaryTreeWindow = std::min(compressorOptions.maxHashTableSize, window);

			//Input size is smaller than maximum binary tree size
			if (inputSize < ((size_t)1 << binaryTreeWindow)) {
				nodes = new IntType[(size_t)2 * inputSize];
				nodeListSize = inputSize;
				nodeListMask = -1;
			}
			else {
				nodes = new IntType[(size_t)2 << binaryTreeWindow];
				nodeListSize = (size_t)1 << binaryTreeWindow;
				nodeListMask = nodeListSize - 1;

				if (window > compressorOptions.maxHashTableSize)
					lzdict16.init(std::max(1, (int)int_log2(std::min(inputSize, (size_t)1 << window) - nodeListSize) - 3));
			}
			lzdict3.init(MIN3((int)int_log2(inputSize) - 3, 14, window - 3));
			if (compressorOptions.parserFunction >= OPTIMAL3)
				lzdict2.init(MIN3((int)int_log2(inputSize) - 3, 12, window - 3));
			nodeLookup.init(MIN3((int)int_log2(inputSize) - 3, 20, window - 3));
		}

		LZMatch<IntType>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const compressionLimit,
			const uint8_t* const blockLimit, LZMatch<IntType>* matches, size_t highestLength, const CompressorOptions& compressorOptions, const int window) {

			const size_t inputPosition = input - inputStart;

			// First try to get a length 2 match
			if (compressorOptions.parserFunction >= OPTIMAL3 && highestLength < 2) {
				IntType& chain2 = lzdict2[read_hash2(input)];
				if (highestLength < 2 || compressorOptions.parserFunction == OPTIMAL4) {
					const uint8_t* where = inputStart + chain2;
					const size_t length = test_match(input, where, blockLimit, 2, window);

					if (length >= 2) {
						matches->distance = input - where;
						matches->length = length;
						highestLength = length;
						matches++;

						if (highestLength >= compressorOptions.niceLength) {
							update_position(input, inputStart, compressionLimit, compressorOptions, window);
							return matches;
						}
					}
				}
				chain2 = inputPosition;
			}

			// Then a length 3 match
			IntType& chain3 = lzdict3[read_hash3(input)];
			if (highestLength < 3) {
				const uint8_t* where = inputStart + chain3;
				const size_t length = test_match(input, where, blockLimit, 3, window);

				if (length >= 3) {
					matches->distance = input - where;
					matches->length = length;
					highestLength = length;
					matches++;

					if (highestLength >= compressorOptions.niceLength) {
						update_position(input, inputStart, compressionLimit, compressorOptions, window);
						return matches;
					}
				}
			}
			chain3 = inputPosition;

			//If we reach this position stop the search
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;

			IntType& lookupEntry = nodeLookup[read_hash4(input)];
			size_t backPosition = lookupEntry;
			lookupEntry = inputPosition;

			IntType* lesserNode = &nodes[2 * (lookupEntry & nodeListMask)];
			IntType* greaterNode = &nodes[2 * (lookupEntry & nodeListMask) + 1];
			const uint8_t* lesserFront = input;
			const uint8_t* greaterFront = input;

			size_t depth = (size_t)1 << compressorOptions.maxElementsPerHash;
			highestLength = std::max(highestLength, (size_t)3); //Avoid reading lengths < 4 in the binary tree
			//optimal ultra reads lengths >= highest length, but we want 
			// lengths strictly longer than what highest length is right now
			highestLength += compressorOptions.parserFunction == OPTIMAL4;

			// Check matches
			while (true) {

				if (backPosition <= btEnd || depth-- == 0) {
					*lesserNode = NO_MATCH_POS;
					*greaterNode = NO_MATCH_POS;
					break;
				}

				const uint8_t* front = std::min(lesserFront, greaterFront);
				const uint8_t* back = front - (inputPosition - backPosition);

				const size_t extraLength = test_match(front, back, compressionLimit, 0, window);
				front += extraLength;
				back += extraLength;

				size_t length = front - input;
				//Match cant go outside of block boundaries
				const size_t effectiveLength = std::min(length, (size_t)(blockLimit - input));
				IntType* const nextNode = &nodes[2 * (backPosition & nodeListMask)];
				if (effectiveLength > highestLength || (compressorOptions.parserFunction == OPTIMAL4 && effectiveLength >= highestLength)) {
					highestLength = effectiveLength;
					matches->distance = front - back;
					matches->length = effectiveLength;
					matches++;
				}

				if (length >= compressorOptions.niceLength) {
					*lesserNode = nextNode[0];
					*greaterNode = nextNode[1];
					if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize)
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

			if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize) {

				const uint8_t* const where = inputStart + lzdict16[read_hash16(input)];
				const size_t length = test_match(input, where, blockLimit, 16, window);

				if (length > highestLength) {
					matches->distance = input - where;
					matches->length = length;
					matches++;
				}

				lzdict16[read_hash16(input - nodeListSize)] = (inputPosition - nodeListSize);
			}

			return matches;
		}

		void update_position(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const limit,
			const CompressorOptions& compressorOptions, const int window) {

			const size_t inputPosition = input - inputStart;
			lzdict3[read_hash3(input)] = inputPosition;
			if (compressorOptions.parserFunction >= OPTIMAL3)
				lzdict2[read_hash2(input)] = inputPosition;
			if (inputPosition > nodeListSize && ((size_t)1 << window) > nodeListSize)
				lzdict16[read_hash16(input - nodeListSize)] = (inputPosition - nodeListSize);

			//If we reach this position on the front stop the update
			const uint8_t* positionSkip = std::min(limit, input + compressorOptions.niceLength);
			//If we reach this position on the back stop the update
			const size_t btEnd = inputPosition < nodeListSize ? 0 : inputPosition - nodeListSize;
			IntType& lookupEntry = nodeLookup[read_hash4(input)];
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

	FORCE_INLINE uint32_t mulHigh32(const uint32_t a, const uint32_t b) {
		return (uint64_t)a * b >> 32;
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
	const int STRIDER_LAST_BYTES = 63;
	const int STRIDER_COST_PRECISION = 32;

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

	struct NibbleModel {

		union {
			uint16_t scalar[16];
#ifdef x64
			__m128i sse[2];
#endif
		};

		NibbleModel() {}
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
			scalar[1] += (NIBBLE_MIXIN_SCALAR[symbol][1] - scalar[1]) >> MODEL_UPDATE_SPEED;
			scalar[2] += (NIBBLE_MIXIN_SCALAR[symbol][2] - scalar[2]) >> MODEL_UPDATE_SPEED;
			scalar[3] += (NIBBLE_MIXIN_SCALAR[symbol][3] - scalar[3]) >> MODEL_UPDATE_SPEED;
			scalar[4] += (NIBBLE_MIXIN_SCALAR[symbol][4] - scalar[4]) >> MODEL_UPDATE_SPEED;
			scalar[5] += (NIBBLE_MIXIN_SCALAR[symbol][5] - scalar[5]) >> MODEL_UPDATE_SPEED;
			scalar[6] += (NIBBLE_MIXIN_SCALAR[symbol][6] - scalar[6]) >> MODEL_UPDATE_SPEED;
			scalar[7] += (NIBBLE_MIXIN_SCALAR[symbol][7] - scalar[7]) >> MODEL_UPDATE_SPEED;
			scalar[8] += (NIBBLE_MIXIN_SCALAR[symbol][8] - scalar[8]) >> MODEL_UPDATE_SPEED;
			scalar[9] += (NIBBLE_MIXIN_SCALAR[symbol][9] - scalar[9]) >> MODEL_UPDATE_SPEED;
			scalar[10] += (NIBBLE_MIXIN_SCALAR[symbol][10] - scalar[10]) >> MODEL_UPDATE_SPEED;
			scalar[11] += (NIBBLE_MIXIN_SCALAR[symbol][11] - scalar[11]) >> MODEL_UPDATE_SPEED;
			scalar[12] += (NIBBLE_MIXIN_SCALAR[symbol][12] - scalar[12]) >> MODEL_UPDATE_SPEED;
			scalar[13] += (NIBBLE_MIXIN_SCALAR[symbol][13] - scalar[13]) >> MODEL_UPDATE_SPEED;
			scalar[14] += (NIBBLE_MIXIN_SCALAR[symbol][14] - scalar[14]) >> MODEL_UPDATE_SPEED;
			scalar[15] += (NIBBLE_MIXIN_SCALAR[symbol][15] - scalar[15]) >> MODEL_UPDATE_SPEED;
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
			size_t symbol = unsafe_bit_scan_forward(mask) - 1;
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

	const uint32_t RANS_BIT_PRECISION = 31;
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
		//To speed up encoding
		uint32_t* reciprocalFreqs = nullptr;

	public:
		RansEncoder() {}
		~RansEncoder() {
			delete[] streamBufferEnd;
			delete[] symbolBufferBegin;
			delete[] reciprocalFreqs;
		}
        
        void initialize_reciprocal_freqs() {
            reciprocalFreqs[2] = 2147483648;
            for (int freq = 3; freq < 16384; freq += 2) {
				// Alverson, "Integer Division using reciprocals". For a divider "x", with a dividend of up to "n" bits
				// shift = ceil(log2(x)) + n
				// mult = (((1 << shift) + x - 1) / x);
				size_t shift = unsafe_int_log2(freq - 1) + 1;
				size_t mult = (((uint64_t)1 << (shift + 31)) + freq - 1) / freq;
				reciprocalFreqs[freq] = mult;
			}
            //Even numbers share some other value, so we can reduce overhead by half
            for (int freq = 4; freq < 16384; freq += 2) 
		        reciprocalFreqs[freq] = reciprocalFreqs[freq / 2];
        }
		//for symbolBufferSize:
		//nibble model takes 1 slot, raw bits take 1 slot for every 15 bits
		//returns whether it fails to initialize
		bool initialize_rans_encoder(const size_t _streamBufferSize, const size_t _symbolBufferSize) {

			try {
				streamBufferEnd = new uint8_t[_streamBufferSize];
				symbolBufferBegin = new uint32_t[_symbolBufferSize];
				//This means file is big enough. For small files this overhead might be worse than using divisions
				if (_streamBufferSize > 32768) { 
					reciprocalFreqs = new uint32_t[MODEL_SCALE];
					initialize_reciprocal_freqs();
				}
			}
			catch (const std::bad_alloc& e) {
				delete[] streamBufferEnd;
				delete[] symbolBufferBegin;
				delete[] reciprocalFreqs;
				return true;
			}
			
			streamBufferBegin = streamBufferEnd + _streamBufferSize;
			streamBufferIt = streamBufferBegin;
			symbolBufferEnd = symbolBufferBegin + _symbolBufferSize;
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
			uint16_t low, freq;
			model->get_range_and_update(symbol, &low, &freq);
			*symbolBufferIt++ = (low << 15) | freq;
		}
		FORCE_INLINE void encode_raw_bits(size_t symbol, size_t nBits) {

			while (nBits) {
				//Not enough space available
				if (symbolBufferIt == symbolBufferEnd)
					return;
				size_t outBits = std::min(nBits, (size_t)15);
				*symbolBufferIt++ = 0x80000000 | (outBits << 15) | ((symbol >> nBits - outBits) & (1 << outBits) - 1);
				nBits -= outBits;
			}
		}

		//Encodes all the symbols stored into a rans stream and returns the encoded size.
		//In case it cant be safely encoded it will return -1.
		size_t end_rans(uint8_t* output) {

			//Very posibly we are missing symbols
			if (symbolBufferIt == symbolBufferEnd)
				return -1;

			while (symbolBufferIt != symbolBufferBegin) {
				std::swap(stateA, stateB);
				symbolBufferIt--;
				size_t data = *symbolBufferIt;

				//raw
				if (data >> 31) {
					const size_t nBits = (data >> 15) & 0xF;
					const size_t symbol = data & 0x7FFF;

					const size_t interval = RANS_NORMALIZATION_INTERVAL << (16 - nBits);
					//Not enough space available
					if (streamBufferIt - streamBufferEnd < 2)
						return -1;
					const bool renormalize = stateB >= interval;
					write_uint16le(streamBufferIt - 2, stateB);
					streamBufferIt -= renormalize * 2;
					stateB >>= renormalize * 16;

					stateB = (stateB << nBits) + symbol;
				}
				//nibble
				else {
					const size_t freq = data & 0x7FFF;
					const size_t low = data >> 15;

					const size_t interval = (RANS_NORMALIZATION_INTERVAL << (16 - MODEL_PRECISION_BITS)) * freq;
					//Not enough space available
					if (streamBufferIt - streamBufferEnd < 2)
						return -1;
					const bool renormalize = stateB >= interval;
					write_uint16le(streamBufferIt - 2, stateB);
					streamBufferIt -= renormalize * 2;
					stateB >>= renormalize * 16;

					if (reciprocalFreqs)
						stateB = stateB + low + (mulHigh32(stateB, reciprocalFreqs[freq]) >> int_log2(freq - 1)) * ((1 << MODEL_PRECISION_BITS) - freq);
					else
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

	struct DataSector {
		const uint8_t* start;
		int pbMask;
		int lcValue;
		bool highEntropy;
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

		int calculate_pb(const uint8_t* data, size_t size, bool bruteForce) {

			const size_t PREDICTOR_BLOCK_SIZE = 262144;
			const size_t numberBlocks = size / PREDICTOR_BLOCK_SIZE + (size % PREDICTOR_BLOCK_SIZE > 0);
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
				const size_t thisBlockSize = std::min(size - thisBlockStart, PREDICTOR_BLOCK_SIZE);

				uint32_t blockHistogram[256 * 16];
				get_histogram16(data + thisBlockStart, thisBlockSize, blockHistogram);
				float bestEntropy = calculate_entropy16(blockHistogram, thisBlockSize, 1);
				int predictedPb = 0; //default pb = 0

				if (size > 65536) {

					if ((bestEntropy < 7.95 && bestEntropy > 1.0) || bruteForce) {
						float entropyPosSize2 = calculate_entropy16(blockHistogram, thisBlockSize, 2) + 0.05;
						if (entropyPosSize2 < bestEntropy) {
							bestEntropy = entropyPosSize2;
							predictedPb = 1;
						}
					}
					if ((predictedPb == 1 && bestEntropy > 1.0) || bruteForce) {
						float entropyPosSize4 = calculate_entropy16(blockHistogram, thisBlockSize, 4) + 0.10;
						if (entropyPosSize4 < bestEntropy) {
							bestEntropy = entropyPosSize4;
							predictedPb = 3;
						}
					}
					if (size > 262144) {
						if ((predictedPb == 3 && bestEntropy > 1.0) || bruteForce) {
							float entropyPosSize8 = calculate_entropy16(blockHistogram, thisBlockSize, 8) + 0.30;
							if (entropyPosSize8 < bestEntropy) {
								bestEntropy = entropyPosSize8;
								predictedPb = 7;
							}
						}
						if ((predictedPb == 7 && bestEntropy > 1.0) || bruteForce) {
							float entropyPosSize16 = calculate_entropy16(blockHistogram, thisBlockSize, 16) + 0.60;
							if (entropyPosSize16 < bestEntropy) {
								bestEntropy = entropyPosSize16;
								predictedPb = 15;
							}
						}
					}
				}

				bool highEntropy = bestEntropy > 7.9;
				if (numberSectors == 0 || dataSectors[numberSectors - 1].pbMask != predictedPb
					|| dataSectors[numberSectors - 1].highEntropy != highEntropy)
				{
					dataSectors[numberSectors] = { data + i * PREDICTOR_BLOCK_SIZE, predictedPb, 4, highEntropy };
					numberSectors++;
				}
			}
			dataSectors[numberSectors] = { data + size, 0, 0 };

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
				uint16_t low, freq;

				modelTree[exclude >> 4].get_range_and_update(literal >> 4, &low, &freq);
				compressedSize += costTable[freq];
				modelTree[(literal >> 4) == (exclude >> 4) ? 16 | (exclude & 0xF) : 32 | (literal >> 4)].get_range_and_update(literal & 0xF, &low, &freq);
				compressedSize += costTable[freq];
			}

			return (double)compressedSize / literalCount / STRIDER_COST_PRECISION;
		}

		template<class IntType>
		int calculate_lc(const uint8_t* input, const size_t size, const uint8_t* costTable) {

			HashTable<IntType, FastIntHash> lzdict;
			lzdict.init(std::min(int_log2(size) - 3, (size_t)18));

			const size_t maxLiteralCount = std::min(size / 4, (size_t)2097152);
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

				size_t literalCount = 0;
				size_t lastDistance = 1;
				const size_t positionContextBitMask = dataSectors[i].pbMask;
				const uint8_t* const matchLimit = dataSectors[i + 1].start;

				for (; input < matchLimit; ) {

					IntType* dictEntry = &lzdict[read_hash4(input)];
					size_t matchLength = test_match(input, inputStart + *dictEntry, matchLimit, 4, 31);
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

				int bestLc = 0;
				double bestSize = 1e100;

				for (int k = 8; k >= 0; k -= 4) {
					//Usually bad
					if ((8 - k) + int_log2(positionContextBitMask + 1) > 8)
						break;

					double size = test_literal_size(literalBuffer, literalCount, 
						literalModel, k, positionContextBitMask, costTable);
					if (size < bestSize) {
						bestLc = k;
						bestSize = size;
					}
					else 
						break;
				}
				dataSectors[i].lcValue = bestLc;
			}

			delete[] literalBuffer;
			delete[] literalModel;
			return 0;
		}

		void get_block_info(const uint8_t* const input, uint8_t* pb, uint8_t* lc, size_t* size, bool* highEntropy) {
			if (input >= dataSectors[currentSector + 1].start)
				currentSector++;

			*pb = dataSectors[currentSector].pbMask;
			*lc = dataSectors[currentSector].lcValue;
			*size = std::min((size_t)(dataSectors[currentSector + 1].start - input), (size_t)STRIDER_MAX_BLOCK_SIZE);
			*highEntropy = dataSectors[currentSector].highEntropy;
		}
	};

	// too many models
	void reset_models(NibbleModel* literalRunLengthModel, uint8_t* matchLiteralContext, NibbleModel* literalModel,
		uint8_t literalContextBitsShift, uint8_t positionContextBitMask, size_t* lastDistance, size_t* repOffsets,
		NibbleModel* distanceModel, NibbleModel* distanceLow, NibbleModel* matchLengthModel) {

		for (size_t i = 0; i < 17; i++) { literalRunLengthModel[i].init(); }
		*matchLiteralContext = 0b1000;  //Match, literal run
		for (size_t i = 0; i < ((64 << 8) >> literalContextBitsShift) * (positionContextBitMask + 1); i++) { literalModel[i].init(); }
		*lastDistance = 1;
		for (size_t i = 0; i < 8; i++) { repOffsets[i] = 1; }
		for (size_t i = 0; i < 24; i++) { distanceModel[i].init(); }
		for (size_t i = 0; i < 2; i++) { distanceLow[i].init(); }
		for (size_t i = 0; i < 194; i++) { matchLengthModel[i].init(); }
	}

	void store_block_header(size_t blockSize, uint8_t positionContextBitMask, uint8_t literalContextBitShift,
		bool resetModels, bool uncompressedBlock, uint8_t*& output)
	{
		write_uint16le(output, blockSize - 1);
		uint8_t metadata = 0;
		if (!uncompressedBlock) {
			metadata |= int_log2(positionContextBitMask + 1) * 3 + literalContextBitShift / 4 + 1;
			metadata |= resetModels << 7;
		}
		output[2] = metadata;
		output += 3;
	}

	FORCE_INLINE void encode_literal_run(RansEncoder* encoder, const uint8_t* literalRun,
		NibbleModel* literalRunLengthModel, size_t literalRunLength, uint8_t* matchLiteralContext,
		NibbleModel* literalModel, const size_t lastDistance, const uint8_t literalContextBitsShift,
		const uint8_t positionContextBitMask, size_t* positionContext) {

		*positionContext = reinterpret_cast<size_t>(literalRun) & positionContextBitMask;

		encoder->encode_nibble(std::min(literalRunLength, (size_t)15), &literalRunLengthModel[*positionContext]);
		if (literalRunLength >= 15) {
			literalRunLength -= 14;  //Allows for some simplifications
			size_t symbol = int_log2(literalRunLength);
			encoder->encode_nibble(symbol, &literalRunLengthModel[16]);
			encoder->encode_raw_bits(literalRunLength & ((size_t)1 << symbol) - 1, symbol);
			literalRunLength += 14;
		}

		if (literalRunLength) {

			*matchLiteralContext >>= 2;
			//Also used as the previous byte to reduce memory access
			uint8_t literal = literalRun[-1];
			size_t lowExclusionModel = 32;

			do {
				const uint8_t exclude = *(literalRun - lastDistance);
				NibbleModel* const literalModelTree =
					&literalModel[(((*positionContext << 8) | literal) >> literalContextBitsShift) * 64];
				literal = *literalRun;

				encoder->encode_nibble(literal >> 4, &literalModelTree[exclude >> 4]);
				encoder->encode_nibble(literal & 0xF,
					&literalModelTree[(exclude >> 4) == (literal >> 4) ? lowExclusionModel | (exclude & 0xF) : 48 | (literal >> 4)]);

				literalRun++;
				literalRunLength--;
				*positionContext = reinterpret_cast<size_t>(literalRun) & positionContextBitMask;
				lowExclusionModel = 16;
			} while (literalRunLength);
		}
	}

	FORCE_INLINE void encode_match_length(RansEncoder* encoder, const size_t matchLengthContext,
		const uint8_t positionContext, NibbleModel* matchLengthModel, size_t matchLength)
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
					encoder->encode_raw_bits(matchLength & ((size_t)1 << rawBits) - 1, rawBits);
				}
				else {
					encoder->encode_nibble(matchLength, &matchLengthModel[192 + (matchLengthContext != 0)]);
				}
			}
		}
	}

	FORCE_INLINE void encode_match(RansEncoder* encoder, uint8_t* matchLiteralContext,
		const uint8_t positionContext, size_t* repOffsets, NibbleModel* distanceModel, NibbleModel* distanceLow,
		NibbleModel* matchLengthModel, size_t matchLength, size_t distance)
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

			repOffsets[7] = repOffsets[6];
			repOffsets[6] = distance;

			distance--;
			if (distance < 256) {
				size_t symbol = distance / 16;
				encoder->encode_nibble(8, &distanceModel[*matchLiteralContext]);
				encoder->encode_nibble(symbol, &distanceModel[16]);
				encoder->encode_nibble(distance & 0xF, &distanceLow[symbol != 0]);
				matchLengthContext = 1;
			}
			else {
				size_t logarithm = int_log2(distance);
				size_t rawBits = logarithm - 5;
				size_t symbol = logarithm * 2 + ((distance >> (logarithm - 1)) & 1);
				size_t symbolHigh = symbol >> 4;
				encoder->encode_nibble(8 | symbolHigh, &distanceModel[*matchLiteralContext]);
				size_t symbolLow = symbol & 0xF;
				encoder->encode_nibble(symbolLow, &distanceModel[16 | symbolHigh]);
				encoder->encode_raw_bits((distance >> 4) & (((size_t)1 << rawBits) - 1), rawBits);
				encoder->encode_nibble(distance & 0xF, &distanceLow[1]);
				matchLengthContext = 1 + logarithm / 8;
			}

			*matchLiteralContext = (*matchLiteralContext >> 2) | 8;
		}

		encode_match_length(encoder, matchLengthContext, positionContext, matchLengthModel, matchLength);
	}

	FORCE_INLINE void encode_last_distance_match(RansEncoder* encoder, uint8_t* matchLiteralContext,
		const uint8_t positionContext, size_t* repOffsets, NibbleModel* distanceModel,
		NibbleModel* matchLengthModel, size_t matchLength, size_t distance)
	{

		size_t rep = distance == repOffsets[0] ? 0 : 6;
		encoder->encode_nibble(rep, &distanceModel[*matchLiteralContext]);
		*matchLiteralContext = (*matchLiteralContext >> 2) | 4;

		for (; rep > 0; rep--)
			repOffsets[rep] = repOffsets[rep - 1];
		repOffsets[0] = distance;

		encode_match_length(encoder, 0, positionContext, matchLengthModel, matchLength);
	}

	template<class IntType>
	size_t compress_greedy(const uint8_t* input, const size_t size, uint8_t* output,
		const CompressorOptions& compressorOptions, ProgressCallback* progress, 
		const int window, RansEncoder& encoder) {

		const size_t accelerationThreshold = 6;
		const size_t accelerationMax = 64;

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. 
		*output++ = *input++;

		DataDetector dataDetector;
		if (dataDetector.calculate_pb(input, size - STRIDER_LAST_BYTES - 1, false))
			return -1;

		//Model stuff
		bool doModelReset = true;
		NibbleModel literalRunLengthModel[17];
		NibbleModel* literalModel = nullptr;
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthModel[194];

		uint8_t matchLiteralContext = 0;
		uint8_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		uint8_t positionContextBitMask = -1;
		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };
		size_t acceleration = 1 << accelerationThreshold;

		const size_t hashSize = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
		HashTable<IntType, FastIntHash> lzdict;
		try {
			lzdict.init(hashSize);
		}
		catch (std::bad_alloc& e) {
			return -1;
		}

		for (; input < compressionLimit; ) {

			uint8_t* const compressedBlockStart = output;
			const uint8_t* const thisBlockStart = input;

			size_t thisBlockSize;
			bool highEntropy = false;
			uint8_t newLiteralContextBitShift, newPositionContextBitMask;
			dataDetector.get_block_info(input, &newPositionContextBitMask, &newLiteralContextBitShift, &thisBlockSize, &highEntropy);
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//The size of the literal model has been modified
			if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {

				delete[] literalModel;
				try {
					literalModel = new NibbleModel[((64 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
				}
				catch (const std::bad_alloc& e) {
					return -1;
				}
			}

			//We have to reset everything
			if (doModelReset) {
				reset_models(literalRunLengthModel, &matchLiteralContext, literalModel, newLiteralContextBitShift,
					newPositionContextBitMask, &distance, repOffsets, distanceModel, distanceLow, matchLengthModel);
			}
			//Only reset certain models if needed
			else {
				//These only require position bit mask
				if (positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < 16; i++)
						literalRunLengthModel[i].init();
					for (size_t i = 0; i < 192; i++)
						matchLengthModel[i].init();
				}
				if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < ((64 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1); i++)
						literalModel[i].init();
				}
			}

			literalContextBitsShift = newLiteralContextBitShift;
			positionContextBitMask = newPositionContextBitMask;

			store_block_header(thisBlockSize, positionContextBitMask, literalContextBitsShift, doModelReset, 0, output);
			encoder.start_rans();

			doModelReset = false;
			const uint8_t* literalRunStart = input;

			for (; input < thisBlockEnd; ) {

				//First try to get a rep match
				size_t matchLength = test_match(input + 1, input + 1 - distance, thisBlockEnd, 4, window);
				if (matchLength) {

					input++;
					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;

					encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchLiteralContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

					//Only update 1 position. This is enough
					lzdict[read_hash6(input)] = input - inputStart;
					input += matchLength;

					literalRunStart = input;
					encode_last_distance_match(&encoder, &matchLiteralContext, positionContext, repOffsets,
						distanceModel, matchLengthModel, matchLength, distance);
					acceleration = 1 << accelerationThreshold;
					continue;
				}

				//If no rep, try a normal match
				IntType* dictEntry = &lzdict[read_hash6(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				matchLength = test_match(input, match, thisBlockEnd, 6, window);

				if (matchLength) {

					//Lazy match
					dictEntry = &lzdict[read_hash6(input + 1)];
					const uint8_t* lazyMatch = inputStart + *dictEntry;
					*dictEntry = input + 1 - inputStart;
					size_t lazyLength = test_match(input + 1, lazyMatch, thisBlockEnd, 6, window);
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

					encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchLiteralContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

					distance = input - match;
					input += matchLength;

					literalRunStart = input;
					encode_match(&encoder, &matchLiteralContext, positionContext, repOffsets,
						distanceModel, distanceLow, matchLengthModel, matchLength, distance);

					acceleration = 1 << accelerationThreshold;
				}
				else {
					input += acceleration >> accelerationThreshold;
					acceleration += acceleration < (accelerationMax << accelerationThreshold);
				}
			}

			bool sendUncompressed = true;
			//Do not attempt encoding if the entropy is high and no match is found
			if (!(highEntropy && literalRunStart == thisBlockStart)) {
				//We might have surpassed block end while skiping bytes
				input = std::min(input, thisBlockEnd);
				//Last literal run
				size_t positionContext;
				size_t literalRunLength = input - literalRunStart;
				encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchLiteralContext,
					literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

				size_t compressedBlockSize = encoder.end_rans(output);
				output += compressedBlockSize;
				sendUncompressed = input - thisBlockStart <= compressedBlockSize;
			}

			//If the algorithm ends up expanding the data, store it uncompressed and reset all models
			if (sendUncompressed) {
				doModelReset = true;
				output = compressedBlockStart;
				store_block_header(thisBlockSize, 0, 0, 0, 1, output);
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
	template<class IntType>
	FORCE_INLINE void fast_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const blockEnd,
		LZ2WayCacheTable<IntType, FastIntHash>* lzdict, size_t* bestMatchLength, size_t* bestMatchDistance,
		int* lazySteps, int* testedPositions, const size_t repOffset, const int window, const CompressorOptions& compressorOptions) {

		*testedPositions = 1;
		LZ2WayCacheBucket<IntType> dictEntry = (*lzdict)[read_hash6(input)];

		//Test first entry
		size_t pos = input - inputStart;
		dictEntry.first(&pos);
		const uint8_t* where = inputStart + pos;
		*bestMatchLength = test_match(input, where, blockEnd, 6, window);
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
			const size_t length = test_match(input, where, blockEnd, 6, window);
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
			const size_t length = test_match(input, where, blockEnd, 6, window);
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
			const size_t length = test_match(input, where, blockEnd, 6, window);
			if (length > *bestMatchLength) {
				*bestMatchDistance = input - where;
				*bestMatchLength = length;
				*lazySteps = 1;
			}
		}
	}

	//Same principle as the last function, but with additional heuristics to improve ratio
	template<class IntType>
	FORCE_INLINE void lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LZCacheTable<IntType, FastIntHash>* lzdict4, LZCacheTable<IntType, FastIntHash>* lzdict8, size_t* bestLength,
		size_t* bestDistance, const size_t repOffset, int* lazySteps, int* testedPositions,
		const CompressorOptions& compressorOptions, const int window) {

		LZCacheBucket<IntType> chain4 = (*lzdict4)[read_hash4(input)];
		LZCacheBucket<IntType> chain8 = (*lzdict8)[read_hash8(input)];
		size_t pos = input - inputStart;
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

			length = test_match(input, where, limit, 8, window);

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

				length = test_match(input, where, limit, 4, window);

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

			length = test_match(input, where, limit, 8, window);

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

		//Now try to get an even better match at pos + 2
		input++;
		pos = input - inputStart;
		(*lzdict4)[read_hash4(input)].push(pos);
		chain8 = (*lzdict8)[read_hash8(input)];
		*testedPositions = 3;

		while (!chain8.ended()) {
			chain8.next(&pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + *bestLength) != *(where + *bestLength))
				continue;

			length = test_match(input, where, limit, 8, window);

			if (length > *bestLength) {
				*bestDistance = input - where;
				*bestLength = length;
				*lazySteps = 2;

				if (*bestLength >= compressorOptions.niceLength) {
					while (!chain8.ended())
						chain8.next(&pos);
					return;
				}
			}
		}
	}

	template<class IntType>
	size_t compress_lazy(const uint8_t* input, const size_t size, uint8_t* output,
		const CompressorOptions& compressorOptions, ProgressCallback* progress, 
		const int window, RansEncoder& encoder) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. Its progress will be reported at the end
		*output++ = *input++;

		DataDetector dataDetector;
		if (dataDetector.calculate_pb(input, size - STRIDER_LAST_BYTES - 1, false))
			return -1;

		//Model stuff
		bool doModelReset = true;
		NibbleModel literalRunLengthModel[17];
		NibbleModel* literalModel = nullptr;
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthModel[194];

		uint8_t matchLiteralContext = 0;
		uint8_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		uint8_t positionContextBitMask = -1;
		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

		const size_t hashSize = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
		LZ2WayCacheTable<IntType, FastIntHash> lzdict6;
		LZCacheTable<IntType, FastIntHash> lzdict4;
		LZCacheTable<IntType, FastIntHash> lzdict8;
		try {
			if (compressorOptions.parserFunction == LAZY2)
				lzdict6.init(hashSize);
			else {
				lzdict4.init(hashSize, compressorOptions.maxElementsPerHash);
				lzdict8.init(hashSize, compressorOptions.maxElementsPerHash);
			}
		}
		catch (const std::bad_alloc& e) {
			return -1;
		}

		for (; input < compressionLimit; ) {

			uint8_t* const compressedBlockStart = output;
			const uint8_t* const thisBlockStart = input;

			size_t thisBlockSize;
			bool highEntropy = false;
			uint8_t newLiteralContextBitShift, newPositionContextBitMask;
			dataDetector.get_block_info(input, &newPositionContextBitMask, &newLiteralContextBitShift, &thisBlockSize, &highEntropy);
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//The size of the literal model has been modified
			if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {

				delete[] literalModel;
				try {
					literalModel = new NibbleModel[((64 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
				}
				catch (const std::bad_alloc& e) {
					return -1;
				}
			}

			//We have to reset everything
			if (doModelReset) {
				reset_models(literalRunLengthModel, &matchLiteralContext, literalModel, newLiteralContextBitShift,
					newPositionContextBitMask, &distance, repOffsets, distanceModel, distanceLow, matchLengthModel);
			}
			//Only reset certain models if needed
			else {
				//These only require position bit mask
				if (positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < 16; i++)
						literalRunLengthModel[i].init();
					for (size_t i = 0; i < 192; i++)
						matchLengthModel[i].init();
				}
				if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < ((64 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1); i++)
						literalModel[i].init();
				}
			}

			literalContextBitsShift = newLiteralContextBitShift;
			positionContextBitMask = newPositionContextBitMask;

			store_block_header(thisBlockSize, positionContextBitMask, literalContextBitsShift, doModelReset, 0, output);
			encoder.start_rans();

			doModelReset = false;
			const uint8_t* literalRunStart = input;

			for (; input < thisBlockEnd; ) {

				//First try to get a rep match
				size_t matchLength = test_match(input + 1, input + 1 - distance, thisBlockEnd, 4, window);
				if (matchLength) {

					input++;
					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;

					encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchLiteralContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

					if (compressorOptions.parserFunction == LAZY2) {
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
					encode_last_distance_match(&encoder, &matchLiteralContext, positionContext, repOffsets,
						distanceModel, matchLengthModel, matchLength, distance);
					continue;
				}

				size_t newDistance;
				int lazySteps;   //bytes to skip because of lazy matching
				int testedPositions;   //number of positions that have been added to hash table

				if (compressorOptions.parserFunction == LAZY2) {
					fast_lazy_search<IntType>(input, inputStart, thisBlockEnd, &lzdict6,
						&matchLength, &newDistance, &lazySteps, &testedPositions, distance, window, compressorOptions);
				}
				else {
					lazy_search<IntType>(input, inputStart, thisBlockEnd, &lzdict4, &lzdict8, &matchLength,
						&newDistance, distance, &lazySteps, &testedPositions, compressorOptions, window);
				}

				input += lazySteps;
				//We have found a match
				if (matchLength) {

					//Update the hash table for every position
					const uint8_t* const matchEnd = input + matchLength;
					const uint8_t* matchPos = input + testedPositions - lazySteps;
					if (compressorOptions.parserFunction == LAZY2) {
						for (; matchPos < matchEnd; matchPos++)
							lzdict6[read_hash6(matchPos)].push_in_first(matchPos - inputStart);
					}
					else {
						for (; matchPos < matchEnd; matchPos++) {
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

					encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchLiteralContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

					distance = newDistance;
					input = matchEnd;
					literalRunStart = input;
					//Output the match
					encode_match(&encoder, &matchLiteralContext, positionContext, repOffsets,
						distanceModel, distanceLow, matchLengthModel, matchLength, distance);
				}
			}

			//Last literal run
			size_t positionContext;
			size_t literalRunLength = input - literalRunStart;
			encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchLiteralContext,
				literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

			size_t compressedBlockSize = encoder.end_rans(output);
			output += compressedBlockSize;

			//If the algorithm ends up expanding the data, store it uncompressed and reset all models
			if (input - thisBlockStart <= compressedBlockSize) {
				doModelReset = true;
				output = compressedBlockStart;
				store_block_header(thisBlockSize, 0, 0, 0, 1, output);
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

	template<class IntType>
	struct StriderFastOptimalState {
		uint32_t sizeCost;
		uint16_t literalRunLength;
		uint16_t matchLength;
		IntType repOffsets[2];   //Match distance will always be the first offset stored here
	};

	template<class IntType>
	LZStructure<IntType>* forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const blockLimit,
		HashTableMatchFinder<IntType>* matchFinder, StriderFastOptimalState<IntType>* parser, LZStructure<IntType>* stream,
		size_t* repOffsets, const CompressorOptions& compressorOptions, const int window) {

		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		for (size_t i = 1; i < blockLength + compressorOptions.niceLength + 1; i++)
			parser[i].sizeCost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastMatchStart;

		parser[0].sizeCost = 0;
		parser[0].repOffsets[0] = repOffsets[0];
		parser[0].repOffsets[1] = repOffsets[6];
		parser[0].literalRunLength = 0;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			StriderFastOptimalState<IntType>* parserPosition = parser + position;

			size_t literalSizeCost = parserPosition->sizeCost + 6;
			StriderFastOptimalState<IntType>* nextPosition = parserPosition + 1;
			if (literalSizeCost < nextPosition->sizeCost) {
				nextPosition->sizeCost = literalSizeCost;
				memcpy(nextPosition->repOffsets, parserPosition->repOffsets, 2 * sizeof(IntType));
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
			}

			size_t highestLength = 0;
			size_t whichRep;
			//Try to find rep offsets
			for (size_t i = (parserPosition->literalRunLength == 0); i < 2; i++) {
				size_t repMatchLength = test_match(inputPosition, inputPosition - parserPosition->repOffsets[i], blockLimit, 2, window);
				if (repMatchLength > highestLength) {
					highestLength = repMatchLength;
					whichRep = i;
					break;
				}
			}

			if (highestLength) {
				size_t repDistance = parserPosition->repOffsets[whichRep];
				if (highestLength >= compressorOptions.niceLength / 2) {
					lastMatchLength = highestLength;
					lastMatchDistance = repDistance;
					lastMatchStart = position;
					goto doBackwardParse;
				}

				const size_t matchSizeCost = parserPosition->sizeCost + 9 + whichRep;  //Increase cost with each successive offset

				nextPosition = parserPosition + highestLength;
				if (matchSizeCost < nextPosition->sizeCost) {
					nextPosition->sizeCost = matchSizeCost;
					nextPosition->matchLength = highestLength;
					nextPosition->repOffsets[0] = repDistance;
					memcpy(&nextPosition->repOffsets[1], parserPosition->repOffsets, whichRep * sizeof(IntType));
					memcpy(&nextPosition->repOffsets[whichRep + 1], &parserPosition->repOffsets[whichRep + 1], (1 - whichRep) * sizeof(IntType));
					nextPosition->literalRunLength = 0;
				}
			}

			LZMatch<IntType> matches[32];
			const LZMatch<IntType>* matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart,
				blockLimit, matches, highestLength, compressorOptions, window);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZMatch<IntType>* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastMatchStart = position;
					break;
				}

				for (const LZMatch<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					const size_t matchSizeCost = parserPosition->sizeCost + 12 + int_log2((size_t)matchIt->distance - 1);

					nextPosition = parserPosition + matchIt->length;
					if (matchSizeCost < nextPosition->sizeCost) {
						nextPosition->sizeCost = matchSizeCost;
						nextPosition->matchLength = matchIt->length;
						nextPosition->repOffsets[0] = matchIt->distance;
						memcpy(&nextPosition->repOffsets[1], parserPosition->repOffsets, 1 * sizeof(IntType));
						nextPosition->literalRunLength = 0;
					}
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const StriderFastOptimalState<IntType>* backwardParse;
		const StriderFastOptimalState<IntType>* const parseEnd = parser;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			backwardParse = parser + position;
			stream->literalRunLength = backwardParse->literalRunLength;
			backwardParse -= backwardParse->literalRunLength;
			stream++;

			const uint8_t* const matchEnd = input + position + lastMatchLength;
			const uint8_t* inputPosition = input + position + 1;
			for (; inputPosition < matchEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart);
		}
		else {
			size_t bestPos = blockLength;
			size_t bestSize = parser[blockLength].sizeCost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				if (parser[blockLength + i].sizeCost <= bestSize) {
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

	template<class IntType>
	struct StriderOptimalParserState {
		uint32_t sizeCost;
		uint32_t literalRunLength;
		uint16_t matchLength;
		uint16_t currentLiteralRunLengthCost;  //Cache so it does not get computed twice
		uint16_t prevPath;
		uint8_t matchLiteralContext;
		uint8_t lastDistanceIndex;   //Which rep is the last used distance
		IntType repOffsets[8];
	};

	FORCE_INLINE size_t get_literal_run_cost(size_t literalRunLength, const size_t positionContext, 
		NibbleModel* literalRunLengthModel, const uint8_t* costTable) {

		if (literalRunLength >= 15) {
			literalRunLength -= 14;
			size_t symbol = int_log2(literalRunLength);
			return costTable[literalRunLengthModel[positionContext].get_freq(15)]
				+ costTable[literalRunLengthModel[16].get_freq(symbol)] + symbol * STRIDER_COST_PRECISION;
		}
		else
			return costTable[literalRunLengthModel[positionContext].get_freq(literalRunLength)];
	}

	FORCE_INLINE size_t get_match_length_cost(size_t length, const size_t positionContext, 
		const size_t matchLengthContext, NibbleModel* matchLengthModel, const uint8_t* costTable) {

		if (length > 16) {
			size_t cost = costTable[matchLengthModel[matchLengthContext * 16 | positionContext].get_freq(15)];
			if (length > 31) {
				cost += costTable[matchLengthModel[144 + (matchLengthContext != 0) * 16 + positionContext].get_freq(15)];
				cost += costTable[matchLengthModel[176 + positionContext].get_freq(length % 16)];
				length -= 32;
				length >>= 4;

				if (length >= 4) {
					length -= 3;
					size_t rawBits = int_log2(length);
					size_t symbol = rawBits + 4;
					cost += costTable[matchLengthModel[192 + (matchLengthContext != 0)].get_freq(symbol)];
					cost += STRIDER_COST_PRECISION * rawBits;
					return cost;
				}
				else {
					return cost + costTable[matchLengthModel[192 + (matchLengthContext != 0)].get_freq(length)];
				}
			}
			else {
				length -= 17;
				return cost + costTable[matchLengthModel[144 + (matchLengthContext != 0) * 16 + positionContext].get_freq(length)];
			}
		}
		else {
			length -= STRIDER_MIN_LENGTH;
			return costTable[matchLengthModel[matchLengthContext * 16 | positionContext].get_freq(length)];
		}
	}

	FORCE_INLINE size_t get_distance_cost(size_t distance, NibbleModel* distanceModel,
		NibbleModel* distanceLow, const size_t matchLiteralContext, const uint8_t* costTable) {

		size_t cost = 0;

		distance--;
		if (distance < 256) {
			size_t symbol = distance / 16;
			cost += costTable[distanceModel[matchLiteralContext].get_freq(8)];
			cost += costTable[distanceModel[16].get_freq(symbol)];
			cost += costTable[distanceLow[symbol != 0].get_freq(distance & 0xF)];
		}
		else {
			size_t logarithm = int_log2(distance);
			size_t rawBits = logarithm - 5;
			size_t symbol = logarithm * 2 + ((distance >> (logarithm - 1)) & 1);

			size_t symbolHigh = symbol >> 4;
			cost += costTable[distanceModel[matchLiteralContext].get_freq(8 | symbolHigh)];
			size_t symbolLow = symbol & 0xF;
			cost += costTable[distanceModel[16 | symbolHigh].get_freq(symbolLow)];
			cost += costTable[distanceLow[1].get_freq(distance & 0xF)];
			cost += rawBits * STRIDER_COST_PRECISION;
		}

		return cost;
	}

	template<class IntType>
	LZStructure<IntType>* priced_forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, const uint8_t* const blockLimit, BinaryMatchFinder<IntType>* matchFinder,
		StriderOptimalParserState<IntType>* parser, LZStructure<IntType>* stream, const CompressorOptions& compressorOptions,
		const uint8_t* costTable, NibbleModel* literalRunLengthModel, size_t startingLiteralRunLength, size_t matchLiteralContext,
		NibbleModel* literalModel, uint8_t literalContextBitsShift, uint8_t positionContextBitMask, size_t lastDistance,
		size_t* repOffsets, NibbleModel* distanceModel, NibbleModel* distanceLow, NibbleModel* matchLengthModel, const int window) {

		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		for (size_t i = 1; i < blockLength + compressorOptions.niceLength + 1; i++)
			parser[i].sizeCost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;

		// While computing the cost of a new literal it is possible that due to substracting the previous run length
		// we get a negative number. To avoid that simply add some starting cost, instead of 0. This has no other effect on the parse
		parser[0].sizeCost = 256 * STRIDER_COST_PRECISION;
		parser[0].lastDistanceIndex = std::find(repOffsets, repOffsets + 8, lastDistance) - repOffsets;
		for (size_t i = 0; i < 8; i++) { parser[0].repOffsets[i] = repOffsets[i]; }
		parser[0].literalRunLength = startingLiteralRunLength;
		parser[0].currentLiteralRunLengthCost =
			get_literal_run_cost(startingLiteralRunLength, reinterpret_cast<size_t>(input - startingLiteralRunLength) & positionContextBitMask,
				literalRunLengthModel, costTable);
		parser[0].matchLiteralContext = matchLiteralContext;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			StriderOptimalParserState<IntType>* parserPosition = parser + position;
			//Unpromising position
			if (parserPosition[0].sizeCost + 1 * STRIDER_COST_PRECISION > parserPosition[1].sizeCost) {
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions, window);
				continue;
			}

			const size_t positionContext = reinterpret_cast<size_t>(inputPosition) & positionContextBitMask;

			const size_t literal = *inputPosition;
			const size_t exclude = *(inputPosition - parserPosition->repOffsets[parserPosition->lastDistanceIndex]);

			NibbleModel* const literalModelTree = &literalModel[(((positionContext << 8) | inputPosition[-1]) >> literalContextBitsShift) * 64];
			bool strongExclusion = parserPosition->literalRunLength == 0;
			size_t thisLiteralSize = costTable[literalModelTree[exclude >> 4].get_freq(literal >> 4)];
			thisLiteralSize += costTable[literalModelTree[(exclude >> 4) == (literal >> 4) ? (16 << strongExclusion) | (exclude & 0xF) : 48 | (literal >> 4)].get_freq(literal & 0xF)];

			const size_t newLiteralRunLengthCost =
				get_literal_run_cost(parserPosition->literalRunLength + 1,
					reinterpret_cast<size_t>(inputPosition - parserPosition->literalRunLength) & positionContextBitMask,
					literalRunLengthModel, costTable);

			thisLiteralSize += parserPosition->sizeCost + newLiteralRunLengthCost - parserPosition->currentLiteralRunLengthCost;

			StriderOptimalParserState<IntType>* nextPosition = parserPosition + 1;
			if (thisLiteralSize < nextPosition->sizeCost) {
				nextPosition->sizeCost = thisLiteralSize;
				nextPosition->lastDistanceIndex = parserPosition->lastDistanceIndex;
				nextPosition->matchLength = 0;
				memcpy(nextPosition->repOffsets, parserPosition->repOffsets, 8 * sizeof(IntType));
				nextPosition->literalRunLength = parserPosition->literalRunLength + 1;
				nextPosition->currentLiteralRunLengthCost = newLiteralRunLengthCost;
				nextPosition->matchLiteralContext = parserPosition->matchLiteralContext;
			}

			size_t highestLength = 1;
			size_t whichRep;

			// Test all reps, but only take the longest one
			for (size_t i = 0; i < 8; i++) {
				size_t repMatchLength = test_match(inputPosition, inputPosition - parserPosition->repOffsets[i], blockLimit, 2, window);
				if (repMatchLength > highestLength) {
					if (repMatchLength >= compressorOptions.niceLength / 2) {
						lastMatchLength = repMatchLength;
						lastMatchDistance = parserPosition->repOffsets[i];
						goto doBackwardParse;
					}
					highestLength = repMatchLength;
					whichRep = i;
				}
			}

			if (highestLength >= 2) {
				size_t repDistance = parserPosition->repOffsets[whichRep];

				size_t currentMatchLiteralContext = parserPosition->matchLiteralContext >> (parserPosition->literalRunLength > 0) * 2;  //After literal run
				const size_t offsetCost = costTable[distanceModel[currentMatchLiteralContext].get_freq(whichRep)];
				currentMatchLiteralContext = (currentMatchLiteralContext >> 2) | 4;  //After rep match

				const size_t lengthCost = get_match_length_cost(highestLength, positionContext, 0, matchLengthModel, costTable);
				const size_t nextLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + highestLength) & positionContextBitMask;
				const size_t literalRunLengthCost = costTable[literalRunLengthModel[nextLiteralRunPositionContext].get_freq(0)];
				const size_t matchSizeCost = parserPosition->sizeCost + lengthCost + offsetCost + literalRunLengthCost;

				nextPosition = parserPosition + highestLength;
				if (matchSizeCost < nextPosition->sizeCost) {
					nextPosition->sizeCost = matchSizeCost;
					nextPosition->matchLength = highestLength;
					nextPosition->lastDistanceIndex = 0;
					nextPosition->repOffsets[0] = repDistance;
					memcpy(&nextPosition->repOffsets[1], parserPosition->repOffsets, whichRep * sizeof(IntType));
					memcpy(&nextPosition->repOffsets[whichRep + 1], &parserPosition->repOffsets[whichRep + 1], (7 - whichRep) * sizeof(IntType));
					nextPosition->literalRunLength = 0;
					nextPosition->currentLiteralRunLengthCost = literalRunLengthCost;
					nextPosition->matchLiteralContext = currentMatchLiteralContext;
				}
			}

			LZMatch<IntType> matches[258];
			const LZMatch<IntType>* matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart, limit,
				blockLimit, matches, highestLength, compressorOptions, window);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZMatch<IntType>* const longestMatch = matchesEnd - 1;
				//Match goes outside the buffer or is very long
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					goto doBackwardParse;
				}

				const size_t currentMatchLiteralContext = parserPosition->matchLiteralContext >> (parserPosition->literalRunLength > 0) * 2;  //After literal run
				const size_t nextMatchLiteralContext = (currentMatchLiteralContext >> 2) | 8;  //After match

				for (const LZMatch<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					const size_t matchLengthContext = 1 + int_log2((size_t)matchIt->distance) / 8;
					const size_t lengthCost = get_match_length_cost(matchIt->length, positionContext, matchLengthContext, matchLengthModel, costTable);
					const size_t distanceCost = get_distance_cost(matchIt->distance, distanceModel, distanceLow, currentMatchLiteralContext, costTable);
					const size_t nextLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + matchIt->length) & positionContextBitMask;
					const size_t literalRunLengthCost = costTable[literalRunLengthModel[nextLiteralRunPositionContext].get_freq(0)];
					const size_t matchSizeCost = parserPosition->sizeCost + distanceCost + lengthCost + literalRunLengthCost;

					nextPosition = parserPosition + matchIt->length;
					if (matchSizeCost < nextPosition->sizeCost) {
						nextPosition->sizeCost = matchSizeCost;
						nextPosition->matchLength = matchIt->length;
						nextPosition->lastDistanceIndex = 6;
						memcpy(nextPosition->repOffsets, parserPosition->repOffsets, 6 * sizeof(IntType));
						nextPosition->repOffsets[6] = matchIt->distance;
						nextPosition->repOffsets[7] = parserPosition->repOffsets[6];
						nextPosition->literalRunLength = 0;
						nextPosition->currentLiteralRunLengthCost = literalRunLengthCost;
						nextPosition->matchLiteralContext = nextMatchLiteralContext;
					}
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const StriderOptimalParserState<IntType>* backwardParse;
		const StriderOptimalParserState<IntType>* const parseEnd = parser;

		if (lastMatchLength) {
			stream->literalRunLength = 0;
			stream++;
			stream->matchDistance = lastMatchDistance;
			stream->matchLength = lastMatchLength;
			stream->literalRunLength = 0;
			stream++;
			backwardParse = parser + position;

			const uint8_t* inputPosition = input + position + 1;
			const uint8_t* const matchEnd = input + position + lastMatchLength;
			for (; inputPosition < matchEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions, window);
		}
		else {
			size_t bestPos = blockLength;
			size_t bestSize = parser[blockLength].sizeCost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				if (parser[blockLength + i].sizeCost <= bestSize) {
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
				stream->matchDistance = backwardParse->repOffsets[backwardParse->lastDistanceIndex];
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
	LZStructure<IntType>* multi_arrivals_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, const uint8_t* const blockLimit, BinaryMatchFinder<IntType>* matchFinder,
		StriderOptimalParserState<IntType>* parser, LZStructure<IntType>* stream, const CompressorOptions& compressorOptions,
		const uint8_t* costTable, NibbleModel* literalRunLengthModel, size_t startingLiteralRunLength, size_t matchLiteralContext,
		NibbleModel* literalModel, size_t literalContextBitsShift, size_t positionContextBitMask, size_t lastDistance,
		size_t* repOffsets, NibbleModel* distanceModel, NibbleModel* distanceLow, NibbleModel* matchLengthModel, const int window) {

		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		for (size_t i = 0; i < (blockLength + compressorOptions.niceLength + 1) * compressorOptions.maxArrivals; i++)
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
		parser[0].lastDistanceIndex = std::find(repOffsets, repOffsets + 8, lastDistance) - repOffsets;
		parser[0].matchLiteralContext = matchLiteralContext;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			StriderOptimalParserState<IntType>* parserPosition = parser + position * compressorOptions.maxArrivals;

			const size_t positionContext = reinterpret_cast<size_t>(inputPosition) & positionContextBitMask;

			//Only try lengths that are at least this long for rep matches
			size_t acceptableRepMatchLength = 2;
			NibbleModel* const literalModelTree = &literalModel[(((positionContext << 8) | inputPosition[-1]) >> literalContextBitsShift) * 64];

			//Literal and rep match parsing can be done at the same time
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				StriderOptimalParserState<IntType>* currentArrival = parserPosition + i;
				//There might be less than max arrivals for this position. Note that there will always be at least one
				if (currentArrival->sizeCost == UINT32_MAX)
					break;

				const size_t literal = *inputPosition;
				const size_t exclude = *(inputPosition - currentArrival->repOffsets[currentArrival->lastDistanceIndex]);

				bool strongExclusion = currentArrival->literalRunLength == 0;
				size_t thisLiteralSize = costTable[literalModelTree[exclude >> 4].get_freq(literal >> 4)];
				thisLiteralSize += costTable[literalModelTree[(exclude >> 4) == (literal >> 4) ? (16 << strongExclusion) | (exclude & 0xF) : 48 | (literal >> 4)].get_freq(literal & 0xF)];

				const size_t newLiteralRunLengthCost =
					get_literal_run_cost(currentArrival->literalRunLength + 1,
						reinterpret_cast<size_t>(inputPosition - currentArrival->literalRunLength) & positionContextBitMask,
						literalRunLengthModel, costTable);

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
						arrivalIt->lastDistanceIndex = currentArrival->lastDistanceIndex;
						arrivalIt->matchLiteralContext = currentArrival->matchLiteralContext;
						arrivalIt->prevPath = i;
						break;
					}
				}

				//Try to find rep offsets
				for (size_t j = 0; j < 8; j++) {

					size_t repMatchLength = test_match(inputPosition, inputPosition - currentArrival->repOffsets[j], blockLimit, 2, window);
					if (repMatchLength >= acceptableRepMatchLength) {
						size_t repDistance = currentArrival->repOffsets[j];

						if (repMatchLength >= compressorOptions.niceLength / 2) {
							lastMatchLength = repMatchLength;
							lastMatchDistance = repDistance;
							lastMatchPath = i;
							goto doBackwardParse;
						}

						size_t currentMatchLiteralContext = currentArrival->matchLiteralContext >> (currentArrival->literalRunLength > 0) * 2;
						const size_t baseMatchCost = currentArrival->sizeCost + costTable[distanceModel[currentMatchLiteralContext].get_freq(j)];
						currentMatchLiteralContext = (currentMatchLiteralContext >> 2) | 4;

						size_t matchLengthReductionLimit = repMatchLength - std::min((size_t)3, repMatchLength - acceptableRepMatchLength);
						for (size_t length = repMatchLength; length >= matchLengthReductionLimit; length--) {

							const size_t lengthCost = get_match_length_cost(length, positionContext, 0, matchLengthModel, costTable);
							const size_t newLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + length) & positionContextBitMask;
							const size_t literalRunLengthCost = costTable[literalRunLengthModel[newLiteralRunPositionContext].get_freq(0)];
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
									arrivalIt->lastDistanceIndex = 0;
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

			LZMatch<IntType> matches[258];
			const LZMatch<IntType>* matchesEnd =
				matchFinder->find_matches_and_update(inputPosition, inputStart, limit, blockLimit,
					matches, acceptableRepMatchLength - 1, compressorOptions, window);

			//At least one match was found
			if (matchesEnd != matches) {

				//The last match should be the longest
				const LZMatch<IntType>* const longestMatch = matchesEnd - 1;
				if (longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastMatchPath = 0;
					goto doBackwardParse;
				}

				size_t previousMatchLength = acceptableRepMatchLength;
				const size_t currentMatchLiteralContext = parserPosition->matchLiteralContext >> (parserPosition->literalRunLength > 0) * 2;  //After literal run
				const size_t nextMatchLiteralContext = (currentMatchLiteralContext >> 2) | 8;  //After match

				for (const LZMatch<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->repOffsets[0] || matchIt->distance == parserPosition->repOffsets[1] ||
						matchIt->distance == parserPosition->repOffsets[2] || matchIt->distance == parserPosition->repOffsets[3] ||
						matchIt->distance == parserPosition->repOffsets[4] || matchIt->distance == parserPosition->repOffsets[5] ||
						matchIt->distance == parserPosition->repOffsets[6] || matchIt->distance == parserPosition->repOffsets[7])
						continue;

					const size_t matchLengthContext = 1 + int_log2((size_t)matchIt->distance) / 8;
					const size_t baseMatchCost = parserPosition->sizeCost + get_distance_cost(matchIt->distance, distanceModel, distanceLow, currentMatchLiteralContext, costTable);

					size_t matchLengthReductionLimit;
					if (previousMatchLength == matchIt->length)
						matchLengthReductionLimit = previousMatchLength - 1;
					else
						matchLengthReductionLimit = matchIt->length - std::min((size_t)4, matchIt->length - previousMatchLength);
					previousMatchLength = matchIt->length;

					//Start search at the highest length. Stop when we reach the limit or
					// the current match length does not improve any arrival.
					for (size_t length = matchIt->length; length > matchLengthReductionLimit; length--) {

						const size_t lengthCost = get_match_length_cost(length, positionContext, matchLengthContext, matchLengthModel, costTable);
						const size_t newLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + length) & positionContextBitMask;
						const size_t literalRunLengthCost = costTable[literalRunLengthModel[newLiteralRunPositionContext].get_freq(0)];
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
								arrivalIt->lastDistanceIndex = 6;
								memcpy(arrivalIt->repOffsets, parserPosition->repOffsets, 6 * sizeof(IntType));
								arrivalIt->repOffsets[6] = matchIt->distance;
								arrivalIt->repOffsets[7] = parserPosition->repOffsets[6];
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
				}
			}
		}

	doBackwardParse:

		// Backward pass, pick best option at each step.
		const StriderOptimalParserState<IntType>* backwardParse;
		const StriderOptimalParserState<IntType>* const parseEnd = parser;
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

			const uint8_t* inputPosition = input + position + 1;
			const uint8_t* const matchEnd = input + position + lastMatchLength;
			for (; inputPosition < matchEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions, window);
		}
		else {
			size_t bestPos = blockLength;
			size_t bestSize = parser[blockLength * compressorOptions.maxArrivals].sizeCost;
			for (size_t i = 1; i < compressorOptions.niceLength; i++) {
				if (parser[(blockLength + i) * compressorOptions.maxArrivals].sizeCost <= bestSize) {
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
				stream->matchDistance = backwardParse[path].repOffsets[backwardParse[path].lastDistanceIndex];
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
        /*We calculate the cost with this*/
        //for (int freq = 0; freq < 16384; freq++) {
		//    if (freq == 0) { values[0] = 255; }
		//    else 
		//	      values[freq] = std::min(int(1 + (14 - log2((float)freq)) * 32), 255);
	    //}
        /*This will have a lot of repeated values, which we can encode using run length encoding*/
        //int pos = 16383;
	    //int length = 0;
	    //for (int i = 1; i <= 255; ) {
		//    if (pos < 0 || values[pos] != i) {
		//	    cout << length << "," << (i % 16 == 0 ? "\n" : "");
		//	    length = 0;
		//	    i++;
		//    }
		//    else {
		//	    rle++;
		//	    pos--;
		//    }
	    //}
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
	    for (int i = 255; i > 0; i--) {
		    memset(costTable + pos, i, RLE[i]);
		    pos += RLE[i];
	    }
    }

	template<class IntType>
	size_t compress_optimal(const uint8_t* input, const size_t size, uint8_t* output,
		const CompressorOptions& compressorOptions, ProgressCallback* progress,
		const int window, RansEncoder& encoder) {

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. Its progress will be reported at the end
		*output++ = *input++;

		DataDetector dataDetector;
		if (dataDetector.calculate_pb(input, size - STRIDER_LAST_BYTES - 1, true))
			return -1;

		//Model stuff
		bool doModelReset = true;
		NibbleModel literalRunLengthModel[17];
		NibbleModel* literalModel = nullptr;
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthModel[194];

		uint8_t matchLiteralContext = 0;
		uint8_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		uint8_t positionContextBitMask = -1;
		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

		HashTableMatchFinder<IntType> hashMatchFinder;
		BinaryMatchFinder<IntType> binaryMatchFinder;
		char* parser = nullptr;
		LZStructure<IntType>* stream = nullptr;
		uint8_t* costTable = nullptr;
		try {
			if (compressorOptions.parserFunction == OPTIMAL1) {
				hashMatchFinder.init(size, compressorOptions, window);
				parser = (char*)(new StriderFastOptimalState<IntType>[compressorOptions.optimalBlockSize + compressorOptions.niceLength + 1]);
			}
			else {
				binaryMatchFinder.init(size, compressorOptions, window);
				if (compressorOptions.parserFunction == OPTIMAL2)
					parser = (char*)(new StriderOptimalParserState<IntType>[compressorOptions.optimalBlockSize + compressorOptions.niceLength + 1]);
				else
					parser = (char*)(new StriderOptimalParserState<IntType>[(compressorOptions.optimalBlockSize + compressorOptions.niceLength + 1) * compressorOptions.maxArrivals]);
			}
			stream = new LZStructure<IntType>[compressorOptions.optimalBlockSize];
			costTable = new uint8_t[MODEL_SCALE];
		}
		catch (const std::bad_alloc& e) {
			delete[] parser;
			delete[] stream;
			delete[] costTable;
			return -1;
		}

        generate_cost_table(costTable);
		if (dataDetector.calculate_lc<IntType>(input, size, costTable)) {
			delete[] parser;
			delete[] stream;
			delete[] costTable;
			return -1;
		}

		for (; input < compressionLimit; ) {

			uint8_t* const compressedBlockStart = output;
			const uint8_t* const thisBlockStart = input;

			size_t thisBlockSize;
			bool highEntropy = false;
			uint8_t newLiteralContextBitShift, newPositionContextBitMask;
			dataDetector.get_block_info(input, &newPositionContextBitMask, &newLiteralContextBitShift, &thisBlockSize, &highEntropy);
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//The size of the literal model has been modified
			if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {

				delete[] literalModel;
				try {
					literalModel = new NibbleModel[((64 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
				}
				catch (const std::bad_alloc& e) {
					delete[] parser;
					delete[] stream;
					delete[] costTable;
					return -1;
				}
			}

			//We have to reset everything
			if (doModelReset) {
				reset_models(literalRunLengthModel, &matchLiteralContext, literalModel, newLiteralContextBitShift,
					newPositionContextBitMask, &distance, repOffsets, distanceModel, distanceLow, matchLengthModel);
			}
			//Only reset certain models if needed
			else {
				//These only require position bit mask
				if (positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < 16; i++)
						literalRunLengthModel[i].init();
					for (size_t i = 0; i < 192; i++)
						matchLengthModel[i].init();
				}
				if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < ((64 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1); i++)
						literalModel[i].init();
				}
			}

			literalContextBitsShift = newLiteralContextBitShift;
			positionContextBitMask = newPositionContextBitMask;

			store_block_header(thisBlockSize, positionContextBitMask, literalContextBitsShift, doModelReset, 0, output);
			encoder.start_rans();

			doModelReset = false;
			const uint8_t* literalRunStart = input;

			for (; input < thisBlockEnd; ) {

				LZStructure<IntType>* streamIt;
				if (compressorOptions.parserFunction == OPTIMAL1) {
					streamIt = forward_optimal_parse<IntType>(input, inputStart, thisBlockEnd, &hashMatchFinder,
						(StriderFastOptimalState<IntType>*)parser, stream, repOffsets, compressorOptions, window);
				}
				else if (compressorOptions.parserFunction == OPTIMAL2) {
					streamIt = priced_forward_optimal_parse<IntType>(input, inputStart, compressionLimit, thisBlockEnd, &binaryMatchFinder,
						(StriderOptimalParserState<IntType>*)parser, stream, compressorOptions, costTable, literalRunLengthModel,
						input - literalRunStart, matchLiteralContext, literalModel, literalContextBitsShift, positionContextBitMask, 
						distance, repOffsets, distanceModel, distanceLow, matchLengthModel, window);
				}
				else {
					streamIt = multi_arrivals_parse<IntType>(input, inputStart, compressionLimit, thisBlockEnd, &binaryMatchFinder,
						(StriderOptimalParserState<IntType>*)parser, stream, compressorOptions, costTable, literalRunLengthModel, 
						input - literalRunStart, matchLiteralContext, literalModel, literalContextBitsShift, positionContextBitMask,
						distance, repOffsets, distanceModel, distanceLow, matchLengthModel, window);
				}

				//Main compression loop
				while (true) {
					input += streamIt->literalRunLength;

					if (streamIt == stream)
						break;

					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;

					encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchLiteralContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

					size_t matchLength = streamIt->matchLength;
					distance = streamIt->matchDistance;
					input += matchLength;
					literalRunStart = input;
					//Output the match
					encode_match(&encoder, &matchLiteralContext, positionContext, repOffsets,
						distanceModel, distanceLow, matchLengthModel, matchLength, distance);

					streamIt--;
				}
			}

			size_t positionContext;
			size_t literalRunLength = input - literalRunStart;
			encode_literal_run(&encoder, literalRunStart, literalRunLengthModel, literalRunLength, &matchLiteralContext,
				literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

			size_t compressedBlockSize = encoder.end_rans(output);
			output += compressedBlockSize;

			//If the algorithm ends up expanding the data, store it uncompressed and reset all models
			if (input - thisBlockStart <= compressedBlockSize) {
				doModelReset = true;
				output = compressedBlockStart;
				store_block_header(thisBlockSize, 0, 0, 0, 1, output);
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
			{ LAZY2        ,     17     ,      NOT_USED       ,      16      ,    NOT_USED    ,   NOT_USED   },
			{ LAZY3        ,     17     ,          1          ,      16      ,    NOT_USED    ,   NOT_USED   },
			{ LAZY3        ,     18     ,          2          ,      24      ,    NOT_USED    ,   NOT_USED   },
			{ OPTIMAL1     ,     18     ,          2          ,      24      ,      1024      ,   NOT_USED   },
			{ OPTIMAL1     ,     19     ,          3          ,      32      ,      1024      ,   NOT_USED   },
			{ OPTIMAL2     ,     22     ,          5          ,      32      ,      2048      ,   NOT_USED   },
			{ OPTIMAL2     ,     24     ,          6          ,      64      ,      2048      ,   NOT_USED   },
			{ OPTIMAL3     ,     25     ,          6          ,      64      ,      4096      ,       3      },
			{ OPTIMAL4     ,     27     ,          7          ,      256     ,      4096      ,       8      },
			{ OPTIMAL4     ,     31     ,          8          ,      1024    ,      4096      ,       24     },
	};

	size_t compress(const uint8_t* input, const size_t size, uint8_t* output, int level,
		int window, ProgressCallback* progress) {

		if (level < 0)
			level = 0;
		if (level > 10)
			level = 10;
		if (window > (IS_64BIT ? 63 : 31))
			window = (IS_64BIT ? 63 : 31);
		if (window < 1)
			window = 1;

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
			std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 2.5 + 64))
			return -1;

#ifdef IS_64BIT
		if (size > ((uint64_t)1 << 32)) {
			if (striderCompressorLevels[level].parserFunction == LAZY1)
				return compress_greedy<uint64_t>(input, size, output, striderCompressorLevels[level], progress, window, ransEncoder);
			if (striderCompressorLevels[level].parserFunction <= LAZY3)
				return compress_lazy<uint64_t>(input, size, output, striderCompressorLevels[level], progress, window, ransEncoder);
			return compress_optimal<uint64_t>(input, size, output, striderCompressorLevels[level], progress, window, ransEncoder);
		}
#endif
		if (striderCompressorLevels[level].parserFunction == LAZY1)
			return compress_greedy<uint32_t>(input, size, output, striderCompressorLevels[level], progress, window, ransEncoder);
		if (striderCompressorLevels[level].parserFunction <= LAZY3)
			return compress_lazy<uint32_t>(input, size, output, striderCompressorLevels[level], progress, window, ransEncoder);
		return compress_optimal<uint32_t>(input, size, output, striderCompressorLevels[level], progress, window, ransEncoder);
	}

	size_t compress_bound(const size_t size) {
		return size + size / 128 + 32;
	}

	template<class IntType>
	size_t parser_memory_estimator(const size_t size, const int level, const int window) {
		if (striderCompressorLevels[level].parserFunction == LAZY1)
			return sizeof(IntType) << MIN3((int)int_log2(size) - 3, striderCompressorLevels[level].maxHashTableSize, window - 3);
		if (striderCompressorLevels[level].parserFunction == LAZY2)
			return sizeof(IntType) << MIN3((int)int_log2(size) - 3, striderCompressorLevels[level].maxHashTableSize, window - 3) << 1;
		if (striderCompressorLevels[level].parserFunction == LAZY3)
			//Lazy extra uses 2 tables
			return sizeof(IntType) << MIN3((int)int_log2(size) - 3, striderCompressorLevels[level].maxHashTableSize, window - 3)
			<< striderCompressorLevels[level].maxElementsPerHash << 1;
		if (striderCompressorLevels[level].parserFunction == OPTIMAL1) {
			const int hashSize = MIN3((int)int_log2(size) - 3, striderCompressorLevels[level].maxHashTableSize, window - 3);
			size_t memory = sizeof(IntType) << std::min(hashSize, 14);  //hash 3 table
			memory += sizeof(IntType) << hashSize << striderCompressorLevels[level].maxElementsPerHash << 1;  //hash 4 and hash 8 tables
			memory += sizeof(StriderFastOptimalState<IntType>) * (striderCompressorLevels[level].optimalBlockSize + striderCompressorLevels[level].niceLength + 1);
			memory += sizeof(LZStructure<IntType>) * striderCompressorLevels[level].optimalBlockSize;
			return memory;
		}
		size_t memory = sizeof(IntType) << MIN3((int)int_log2(size) - 3, 20, window - 3);  //binary node lookup
		memory += sizeof(IntType) << MIN3((int)int_log2(size) - 3, 14, window - 3);  //hash 3 table
		if (striderCompressorLevels[level].parserFunction >= OPTIMAL3)
			memory += sizeof(IntType) << MIN3((int)int_log2(size) - 3, 12, window - 3);  //hash 2 table
		const size_t binaryTreeWindow = (size_t)1 << std::min(striderCompressorLevels[level].maxHashTableSize, window);
		if (size < binaryTreeWindow)
			memory += sizeof(IntType) * 2 * size;  //binary tree
		else {
			memory += sizeof(IntType) * 2 * binaryTreeWindow;  //binary tree
			if (window > striderCompressorLevels[level].maxHashTableSize)
				memory += sizeof(IntType) << std::max(1, (int)int_log2(std::min(size, (size_t)1 << window) - binaryTreeWindow) - 3);  //hash 16 table
		}
		if (striderCompressorLevels[level].parserFunction == OPTIMAL2)
			memory += sizeof(StriderOptimalParserState<IntType>) * (striderCompressorLevels[level].optimalBlockSize + striderCompressorLevels[level].niceLength + 1);
		else
			memory += sizeof(StriderOptimalParserState<IntType>) * (striderCompressorLevels[level].optimalBlockSize + striderCompressorLevels[level].niceLength + 1) * striderCompressorLevels[level].maxArrivals;
		memory += sizeof(LZStructure<IntType>) * striderCompressorLevels[level].optimalBlockSize;
		return memory;
	}

	size_t estimate_memory(const size_t size, int level, int window) {

		if (level < 0)
			level = 0;
		if (level > 10)
			level = 10;
		if (window > (IS_64BIT ? 63 : 31))
			window = (IS_64BIT ? 63 : 31);
		if (window < 1)
			window = 1;

		if (size <= STRIDER_LAST_BYTES + 1)
			return 0;

		size_t blockSize = std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size);
		size_t memory = blockSize;   //stream buffer
		//symbol buffer
		memory += (blockSize * 2.5 + 64) * sizeof(uint32_t);
		memory += sizeof(NibbleModel) * 64 * 256;  //literal model, the encoder will not use lc + pb > 8
		if (size > 32768)
			memory += MODEL_SCALE * sizeof(uint32_t);  //reciprocal frequencies for faster encoding
		if (striderCompressorLevels[level].parserFunction >= OPTIMAL1)
			memory += MODEL_SCALE * sizeof(uint8_t);  //cost table

#ifdef IS_64BIT
		if (size > ((uint64_t)1 << 32))
			return memory + parser_memory_estimator<uint64_t>(size, level, window);
#endif
		return memory + parser_memory_estimator<uint32_t>(size, level, window);
	}

	class RansDecoder {
		uint32_t stateA;
		uint32_t stateB;
		const uint8_t* compressedStreamIt;
		const uint8_t* compressedStreamEnd;

	public:
		RansDecoder() {}
		~RansDecoder() {}

		//rans block functions
		void start_rans(const uint8_t* compressed, const uint8_t* compressedEnd) {
			compressedStreamIt = compressed;
			compressedStreamEnd = compressedEnd;

			stateA = read_uint32le(compressedStreamIt + 0);
			stateB = read_uint32le(compressedStreamIt + 4);
			compressedStreamIt += 8;
		}
		FORCE_INLINE void normalize() {
			const size_t renormalize = stateB < RANS_NORMALIZATION_INTERVAL;
			stateB <<= renormalize * 16;
			stateB |= read_uint16le(compressedStreamIt) & (0 - renormalize);
			compressedStreamIt += renormalize * 2;
		}
		FORCE_INLINE size_t decode_raw_bits(size_t nBits) {

			size_t symbol = 0;
			while (nBits) {
				size_t inBits = std::min(nBits, (size_t)15);
				symbol <<= inBits;
				std::swap(stateA, stateB);
				symbol |= stateB & (1 << inBits) - 1;
				stateB >>= inBits;
				normalize();
				nBits -= inBits;
			}
			return symbol;
		}
		FORCE_INLINE size_t decode_nibble(NibbleModel* model) {
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

		uint8_t matchLiteralContext;
		uint8_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		uint8_t positionContextBitMask = -1;
		size_t distance = 1;  //Also stores the last used distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

		while (decompressed < decompressedEnd) {

			const size_t thisBlockSize = read_uint16le(compressed) + 1;
			const uint8_t blockMetadata = compressed[2];
			compressed += 3;
			const uint8_t* const thisBlockEnd = decompressed + thisBlockSize;

			if (compressed > compressedEnd || decompressedEnd - decompressed < thisBlockSize) {
				delete[] literalModel;
				return -1;
			}

			//Uncompressed data
			if (blockMetadata == 0) {

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
				bool doModelReset = (blockMetadata >> 7) || (!literalModel);
				size_t newLiteralContextBitsShift = (blockMetadata % 16 - 1) % 3 * 4;
				size_t newPositionContextBitMask = (1 << (blockMetadata % 16 - 1) / 3) - 1;

				if (newLiteralContextBitsShift > 8 || newPositionContextBitMask > 15) {
					delete[] literalModel;
					return -1;
				}

				//The size of the literal model has been modified
				if (literalContextBitsShift != newLiteralContextBitsShift || positionContextBitMask != newPositionContextBitMask) {

					delete[] literalModel;
					try {
						literalModel = new NibbleModel[((64 << 8) >> newLiteralContextBitsShift) * (newPositionContextBitMask + 1)];
					}
					catch (const std::bad_alloc& e) {
						return -1;
					}
				}

				//We have to reset everything
				if (doModelReset) {
					reset_models(literalRunLengthModel, &matchLiteralContext, literalModel, newLiteralContextBitsShift,
						newPositionContextBitMask, &distance, repOffsets, distanceModel, distanceLow, matchLengthModel);
				}
				//Only reset certain models if needed
				else {
					//These only require position bit mask
					if (positionContextBitMask != newPositionContextBitMask) {
						for (size_t i = 0; i < 16; i++)
							literalRunLengthModel[i].init();
						for (size_t i = 0; i < 192; i++)
							matchLengthModel[i].init();
					}
					if (literalContextBitsShift != newLiteralContextBitsShift || positionContextBitMask != newPositionContextBitMask) {
						for (size_t i = 0; i < ((64 << 8) >> newLiteralContextBitsShift) * (newPositionContextBitMask + 1); i++)
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

					size_t literalRunLength = ransDecoder.decode_nibble(&literalRunLengthModel[positionContext]);
					if (literalRunLength == 15) {
						size_t symbol = ransDecoder.decode_nibble(&literalRunLengthModel[16]);
						literalRunLength = ((size_t)1 << symbol) + ransDecoder.decode_raw_bits(symbol);
						literalRunLength += 14;

						if (unlikely(thisBlockEnd - decompressed < literalRunLength)) {
							delete[] literalModel;
							return -1;
						}
					}

					//A symbol can have a minimum frequency of 16/16384, that is, its decoding will consume at most 10 bits.
					//As both states might be just before renormalization interval, we should be able to read at least 448 bits
					// before having to check again for the buffer overflow.
					//A match can consume, at most, 137 bits plus 20 for a long literal run, 
					// plus 10 for the first literal (total = 167)
					if (unlikely(ransDecoder.check_stream_overflow())) {
						delete[] literalModel;
						return -1;
					}

					if (literalRunLength) {
						matchLiteralContext >>= 2;
						//The literal we just have decoded is the previous byte
						uint8_t literal = decompressed[-1];

						uint8_t exclude = *(decompressed - distance);
						NibbleModel* literalModelTree =
							&literalModel[(((positionContext << 8) | literal) >> literalContextBitsShift) * 64];

						literal = ransDecoder.decode_nibble(&literalModelTree[exclude >> 4]);
						//Note that in the first literal the low bits are coded with a 
						// different model when (literal >> 4) == (exclude >> 4)
						literal = (literal << 4) | ransDecoder.decode_nibble(
							&literalModelTree[literal == (exclude >> 4) ? 32 | (exclude & 0xF) : 48 | literal]);

						*decompressed++ = literal;
						positionContext = reinterpret_cast<size_t>(decompressed) & positionContextBitMask;
						literalRunLength--;

						while (literalRunLength) {
							exclude = *(decompressed - distance);
							literalModelTree =
								&literalModel[(((positionContext << 8) | literal) >> literalContextBitsShift) * 64];

							literal = ransDecoder.decode_nibble(&literalModelTree[exclude >> 4]);
							literal = (literal << 4) | ransDecoder.decode_nibble(
								&literalModelTree[literal == (exclude >> 4) ? 16 | (exclude & 0xF) : 48 | literal]);

							*decompressed++ = literal;
							positionContext = reinterpret_cast<size_t>(decompressed) & positionContextBitMask;
							literalRunLength--;

							//Check overflow for every decoded literal
							if (unlikely(ransDecoder.check_stream_overflow())) {
								delete[] literalModel;
								return -1;
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
						if (distanceToken == 8) {
							distanceToken = ransDecoder.decode_nibble(&distanceModel[16]);
							distance = (distanceToken << 4) | ransDecoder.decode_nibble(&distanceLow[distanceToken != 0]);
							matchLengthContext = 1;
						}
						else {
							distanceToken -= 8;
							distanceToken = (distanceToken << 4) | ransDecoder.decode_nibble(&distanceModel[16 | distanceToken]);
							const size_t rawBits = distanceToken / 2 - 5;
							distance = ((size_t)2 | (distanceToken & 1)) << rawBits;
							distance = (distance | ransDecoder.decode_raw_bits(rawBits)) << 4;
							distance |= ransDecoder.decode_nibble(&distanceLow[1]);
							matchLengthContext = 1 + distanceToken / 16;
						}
						distance++;

						if (unlikely(decompressed - decompressedStart < distance)) {
							delete[] literalModel;
							return -1;
						}

						repOffsets[7] = repOffsets[6];
						repOffsets[6] = distance;

						matchLiteralContext |= 8;
					}

					size_t matchLength = STRIDER_MIN_LENGTH
						+ ransDecoder.decode_nibble(&matchLengthModel[matchLengthContext * 16 | positionContext]);
					if (matchLength == 17) {
						matchLength +=
							ransDecoder.decode_nibble(&matchLengthModel[144 + (matchLengthContext != 0) * 16 + positionContext]);
						if (matchLength == 32) {
							size_t lowBits = ransDecoder.decode_nibble(&matchLengthModel[176 + positionContext]);
							size_t highBits = ransDecoder.decode_nibble(&matchLengthModel[192 + (matchLengthContext != 0)]);

							if (highBits >= 4)
								highBits = (1 << highBits - 4) + ransDecoder.decode_raw_bits(highBits - 4) + 3;
							matchLength += highBits * 16 + lowBits;

							//Check overflow
							if (unlikely(thisBlockEnd - decompressed < matchLength)) {
								delete[] literalModel;
								return -1;
							}
						}
					}

					copy_match(decompressed, distance, matchLength);
					decompressed += matchLength;
				}

				compressed = ransDecoder.end_rans();
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
