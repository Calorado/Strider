/*
 * Strider Compression Algorithm v1.1.0
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
	size_t strider_compress(const uint8_t* input, const size_t size, uint8_t* output, int level = 6,
		int window = 999, ProgressCallback* progress = nullptr);
	//Decompresses contents in "compressed" to "decompressed".
	//You may also pass a pointer to an object with base class ProgressCallback, to track progress.
	//Returns 0 on success or -1 on failure or corrupted data.
	int strider_decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed,
		const size_t decompressedSize, ProgressCallback* progress = nullptr);

	//For a given input size, returns a size for the output buffer that is big enough to
	// contain the compressed stream even if it expands.
	size_t strider_compress_bound(const size_t size);
	//Returns the amount of memory the algorithm will consume on compression.
	size_t strider_estimate_memory(const size_t size, int level = 6, int window = 26);
}

#ifdef STRIDER_IMPLEMENTATION

#include <algorithm>
#include <cstring>

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
			const int log2size = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
			lzdict3.init(std::min(log2size, 14));
			lzdict4.init(log2size, compressorOptions.maxElementsPerHash);
			lzdict8.init(log2size, compressorOptions.maxElementsPerHash);
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
			if (compressorOptions.parserFunction >= OPTIMAL_BRUTE)
				lzdict2.init(MIN3((int)int_log2(inputSize) - 3, 12, window - 3));
			nodeLookup.init(MIN3((int)int_log2(inputSize) - 3, 20, window - 3));
		}

		LZMatch<IntType>* find_matches_and_update(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const compressionLimit,
			const uint8_t* const blockLimit, LZMatch<IntType>* matches, size_t highestLength, const CompressorOptions& compressorOptions, const int window) {

			const size_t inputPosition = input - inputStart;

			// First try to get a length 2 match
			if (compressorOptions.parserFunction >= OPTIMAL_BRUTE) {
				IntType& chain2 = lzdict2[read_hash2(input)];
				if (highestLength < 2 || compressorOptions.parserFunction == OPTIMAL_ULTRA) {
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
			if (highestLength < 3 || compressorOptions.parserFunction == OPTIMAL_ULTRA) {
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
				if (effectiveLength > highestLength || (compressorOptions.parserFunction == OPTIMAL_ULTRA && effectiveLength > 3)) {
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
			if (compressorOptions.parserFunction >= OPTIMAL_BRUTE)
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
	const int STRIDER_LAST_BYTES = 31;
	const int STRIDER_COST_PRECISION = 16;

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

	//Precalculated numbers to speed up rans encoding
	/*
	for (int freq = 0; freq < 16384; freq++) {
		if (freq == 0) { cout << "0,"; }
		else {
			// Alverson, "Integer Division using reciprocals". For a divider "x", with a dividend of up to "n" bits
			// shift = ceil(log2(x)) + n
			// mult = (((1 << shift) + x - 1) / x);
			uint64_t shift = ceil(log2(freq));
			uint64_t mult = (((uint64_t)1 << (shift + 31)) + freq - 1) / freq;
			cout << mult << "," << (freq % 32 == 0 ? "\n" : "");
		}
	}
	*/
	const uint32_t RCP_FREQS[MODEL_SCALE] = {
		0,2147483648,2147483648,2863311531,2147483648,3435973837,2863311531,2454267027,2147483648,3817748708,3435973837,3123612579,2863311531,2643056798,2454267027,2290649225,2147483648,4042322161,3817748708,3616814566,3435973837,3272356036,3123612579,2987803337,2863311531,2748779070,2643056798,2545165806,2454267027,2369637129,2290649225,2216757315,2147483648,
		4164816772,4042322161,3926827243,3817748708,3714566311,3616814566,3524075731,3435973837,3352169597,3272356036,3196254732,3123612579,3054198967,2987803337,2924233053,2863311531,2804876602,2748779070,2694881441,2643056798,2593187802,2545165806,2498890064,2454267027,2411209711,2369637129,2329473788,2290649225,2253097598,2216757315,2181570691,2147483648,
		4228890877,4164816772,4102655328,4042322161,3983737782,3926827243,3871519817,3817748708,3765450781,3714566311,3665038760,3616814566,3569842948,3524075731,3479467177,3435973837,3393554407,3352169597,3311782012,3272356036,3233857729,3196254732,3159516172,3123612579,3088515809,3054198967,3020636341,2987803337,2955676419,2924233053,2893451653,2863311531,
		2833792856,2804876602,2776544515,2748779070,2721563436,2694881441,2668717544,2643056798,2617884829,2593187802,2568952402,2545165806,2521815661,2498890064,2476377541,2454267027,2432547850,2411209711,2390242670,2369637129,2349383821,2329473788,2309898378,2290649225,2271718240,2253097598,2234779732,2216757315,2199023256,2181570691,2164392969,2147483648,
		4261672976,4228890877,4196609267,4164816772,4133502361,4102655328,4072265289,4042322161,4012816160,3983737782,3955077798,3926827243,3898977404,3871519817,3844446251,3817748708,3791419407,3765450781,3739835469,3714566311,3689636335,3665038760,3640766980,3616814566,3593175255,3569842948,3546811703,3524075731,3501629388,3479467177,3457583736,3435973837,
		3414632385,3393554407,3372735055,3352169597,3331853418,3311782012,3291950982,3272356036,3252992982,3233857729,3214946281,3196254732,3177779272,3159516172,3141461794,3123612579,3105965051,3088515809,3071261531,3054198967,3037324939,3020636341,3004130131,2987803337,2971653049,2955676419,2939870663,2924233053,2908760921,2893451653,2878302691,2863311531,
		2848475720,2833792856,2819260585,2804876602,2790638650,2776544515,2762592030,2748779070,2735103552,2721563436,2708156719,2694881441,2681735678,2668717544,2655825188,2643056798,2630410593,2617884829,2605477791,2593187802,2581013211,2568952402,2557003786,2545165806,2533436931,2521815661,2510300521,2498890064,2487582869,2476377541,2465272709,2454267027,
		2443359173,2432547850,2421831780,2411209711,2400680410,2390242670,2379895299,2369637129,2359467013,2349383821,2339386443,2329473788,2319644785,2309898378,2300233531,2290649225,2281144456,2271718240,2262369605,2253097598,2243901282,2234779732,2225732041,2216757315,2207854675,2199023256,2190262207,2181570691,2172947881,2164392969,2155905153,2147483648,
		4278255361,4261672976,4245218641,4228890877,4212688230,4196609267,4180652578,4164816772,4149100483,4133502361,4118021078,4102655328,4087403821,4072265289,4057238479,4042322161,4027515121,4012816160,3998224102,3983737782,3969356057,3955077798,3940901892,3926827243,3912852768,3898977404,3885200099,3871519817,3857935537,3844446251,3831050968,3817748708,
		3804538505,3791419407,3778390474,3765450781,3752599413,3739835469,3727158061,3714566311,3702059353,3689636335,3677296414,3665038760,3652862551,3640766980,3628751247,3616814566,3604956157,3593175255,3581471101,3569842948,3558290058,3546811703,3535407164,3524075731,3512816703,3501629388,3490513105,3479467177,3468490940,3457583736,3446744915,3435973837,
		3425269869,3414632385,3404060768,3393554407,3383112701,3372735055,3362420881,3352169597,3341980632,3331853418,3321787396,3311782012,3301836721,3291950982,3282124263,3272356036,3262645780,3252992982,3243397133,3233857729,3224374276,3214946281,3205573259,3196254732,3186990226,3177779272,3168621406,3159516172,3150463117,3141461794,3132511761,3123612579,
		3114763819,3105965051,3097215853,3088515809,3079864504,3071261531,3062706485,3054198967,3045738582,3037324939,3028957653,3020636341,3012360625,3004130131,2995944490,2987803337,2979706309,2971653049,2963643202,2955676419,2947752354,2939870663,2932031008,2924233053,2916476467,2908760921,2901086090,2893451653,2885857291,2878302691,2870787540,2863311531,
		2855874358,2848475720,2841115318,2833792856,2826508041,2819260585,2812050199,2804876602,2797739511,2790638650,2783573742,2776544515,2769550700,2762592030,2755668241,2748779070,2741924259,2735103552,2728316695,2721563436,2714843526,2708156719,2701502771,2694881441,2688292489,2681735678,2675210774,2668717544,2662255758,2655825188,2649425610,2643056798,
		2636718532,2630410593,2624132764,2617884829,2611666575,2605477791,2599318269,2593187802,2587086184,2581013211,2574968684,2568952402,2562964168,2557003786,2551071063,2545165806,2539287824,2533436931,2527612938,2521815661,2516044915,2510300521,2504582296,2498890064,2493223646,2487582869,2481967558,2476377541,2470812647,2465272709,2459757557,2454267027,
		2448800953,2443359173,2437941526,2432547850,2427177987,2421831780,2416509073,2411209711,2405933540,2400680410,2395450170,2390242670,2385057761,2379895299,2374755136,2369637129,2364541136,2359467013,2354414621,2349383821,2344374473,2339386443,2334419592,2329473788,2324548896,2319644785,2314761322,2309898378,2305055824,2300233531,2295431374,2290649225,
		2285886961,2281144456,2276421590,2271718240,2267034285,2262369605,2257724082,2253097598,2248490037,2243901282,2239331218,2234779732,2230246710,2225732041,2221235612,2216757315,2212297038,2207854675,2203430116,2199023256,2194633988,2190262207,2185907809,2181570691,2177250749,2172947881,2168661988,2164392969,2160140723,2155905153,2151686161,2147483648,
		4286595041,4278255361,4269948070,4261672976,4253429895,4245218641,4237039029,4228890877,4220774003,4212688230,4204633376,4196609267,4188615725,4180652578,4172719651,4164816772,4156943773,4149100483,4141286734,4133502361,4125747197,4118021078,4110323843,4102655328,4095015374,4087403821,4079820512,4072265289,4064737996,4057238479,4049766585,4042322161,
		4034905057,4027515121,4020152204,4012816160,4005506841,3998224102,3990967796,3983737782,3976533917,3969356057,3962204065,3955077798,3947977120,3940901892,3933851978,3926827243,3919827551,3912852768,3905902763,3898977404,3892076559,3885200099,3878347894,3871519817,3864715740,3857935537,3851179082,3844446251,3837736921,3831050968,3824388271,3817748708,
		3811132159,3804538505,3797967627,3791419407,3784893728,3778390474,3771909530,3765450781,3759014113,3752599413,3746206569,3739835469,3733486003,3727158061,3720851533,3714566311,3708302286,3702059353,3695837405,3689636335,3683456040,3677296414,3671157355,3665038760,3658940526,3652862551,3646804736,3640766980,3634749183,3628751247,3622773074,3616814566,
		3610875625,3604956157,3599056065,3593175255,3587313631,3581471101,3575647571,3569842948,3564057141,3558290058,3552541609,3546811703,3541100251,3535407164,3529732353,3524075731,3518437209,3512816703,3507214124,3501629388,3496062410,3490513105,3484981388,3479467177,3473970388,3468490940,3463028749,3457583736,3452155818,3446744915,3441350948,3435973837,
		3430613504,3425269869,3419942855,3414632385,3409338381,3404060768,3398799468,3393554407,3388325510,3383112701,3377915908,3372735055,3367570070,3362420881,3357287414,3352169597,3347067361,3341980632,3336909341,3331853418,3326812793,3321787396,3316777158,3311782012,3306801889,3301836721,3296886441,3291950982,3287030278,3282124263,3277232870,3272356036,
		3267493694,3262645780,3257812231,3252992982,3248187970,3243397133,3238620406,3233857729,3229109039,3224374276,3219653376,3214946281,3210252928,3205573259,3200907214,3196254732,3191615756,3186990226,3182378084,3177779272,3173193731,3168621406,3164062239,3159516172,3154983151,3150463117,3145956017,3141461794,3136980394,3132511761,3128055841,3123612579,
		3119181923,3114763819,3110358212,3105965051,3101584282,3097215853,3092859713,3088515809,3084184090,3079864504,3075557001,3071261531,3066978042,3062706485,3058446809,3054198967,3049962907,3045738582,3041525942,3037324939,3033135525,3028957653,3024791274,3020636341,3016492806,3012360625,3008239748,3004130131,3000031727,2995944490,2991868375,2987803337,
		2983749330,2979706309,2975674230,2971653049,2967642721,2963643202,2959654449,2955676419,2951709068,2947752354,2943806233,2939870663,2935945602,2932031008,2928126839,2924233053,2920349609,2916476467,2912613584,2908760921,2904918436,2901086090,2897263842,2893451653,2889649482,2885857291,2882075041,2878302691,2874540204,2870787540,2867044662,2863311531,
		2859588109,2855874358,2852170241,2848475720,2844790758,2841115318,2837449363,2833792856,2830145761,2826508041,2822879661,2819260585,2815650776,2812050199,2808458820,2804876602,2801303511,2797739511,2794184569,2790638650,2787101719,2783573742,2780054685,2776544515,2773043198,2769550700,2766066989,2762592030,2759125792,2755668241,2752219344,2748779070,
		2745347386,2741924259,2738509659,2735103552,2731705908,2728316695,2724935881,2721563436,2718199328,2714843526,2711496000,2708156719,2704825653,2701502771,2698188044,2694881441,2691582933,2688292489,2685010081,2681735678,2678469252,2675210774,2671960214,2668717544,2665482735,2662255758,2659036585,2655825188,2652621539,2649425610,2646237372,2643056798,
		2639883861,2636718532,2633560786,2630410593,2627267928,2624132764,2621005073,2617884829,2614772005,2611666575,2608568512,2605477791,2602394386,2599318269,2596249417,2593187802,2590133399,2587086184,2584046129,2581013211,2577987404,2574968684,2571957025,2568952402,2565954791,2562964168,2559980508,2557003786,2554033979,2551071063,2548115013,2545165806,
		2542223417,2539287824,2536359003,2533436931,2530521583,2527612938,2524710971,2521815661,2518926983,2516044915,2513169435,2510300521,2507438148,2504582296,2501732942,2498890064,2496053639,2493223646,2490400063,2487582869,2484772041,2481967558,2479169398,2476377541,2473591964,2470812647,2468039569,2465272709,2462512045,2459757557,2457009225,2454267027,
		2451530943,2448800953,2446077037,2443359173,2440647343,2437941526,2435241701,2432547850,2429859951,2427177987,2424501936,2421831780,2419167498,2416509073,2413856483,2411209711,2408568736,2405933540,2403304105,2400680410,2398062438,2395450170,2392843587,2390242670,2387647401,2385057761,2382473733,2379895299,2377322439,2374755136,2372193372,2369637129,
		2367086390,2364541136,2362001349,2359467013,2356938109,2354414621,2351896531,2349383821,2346876474,2344374473,2341877802,2339386443,2336900378,2334419592,2331944068,2329473788,2327008737,2324548896,2322094251,2319644785,2317200481,2314761322,2312327293,2309898378,2307474560,2305055824,2302642153,2300233531,2297829944,2295431374,2293037806,2290649225,
		2288265615,2285886961,2283513246,2281144456,2278780576,2276421590,2274067483,2271718240,2269373845,2267034285,2264699543,2262369605,2260044456,2257724082,2255408468,2253097598,2250791460,2248490037,2246193316,2243901282,2241613921,2239331218,2237053160,2234779732,2232510920,2230246710,2227987088,2225732041,2223481553,2221235612,2218994204,2216757315,
		2214524931,2212297038,2210073624,2207854675,2205640177,2203430116,2201224481,2199023256,2196826430,2194633988,2192445918,2190262207,2188082842,2185907809,2183737096,2181570691,2179408579,2177250749,2175097187,2172947881,2170802819,2168661988,2166525375,2164392969,2162264755,2160140723,2158020860,2155905153,2153793591,2151686161,2149582851,2147483648,
		4290777085,4286595041,4282421141,4278255361,4274097679,4269948070,4265806510,4261672976,4257547446,4253429895,4249320301,4245218641,4241124891,4237039029,4232961031,4228890877,4224828541,4220774003,4216727240,4212688230,4208656949,4204633376,4200617490,4196609267,4192608686,4188615725,4184630363,4180652578,4176682347,4172719651,4168764466,4164816772,
		4160876548,4156943773,4153018425,4149100483,4145189926,4141286734,4137390886,4133502361,4129621138,4125747197,4121880517,4118021078,4114168860,4110323843,4106486005,4102655328,4098831791,4095015374,4091206057,4087403821,4083608646,4079820512,4076039399,4072265289,4068498161,4064737996,4060984775,4057238479,4053499089,4049766585,4046040949,4042322161,
		4038610204,4034905057,4031206702,4027515121,4023830294,4020152204,4016480833,4012816160,4009158169,4005506841,4001862158,3998224102,3994592654,3990967796,3987349512,3983737782,3980132590,3976533917,3972941745,3969356057,3965776836,3962204065,3958637724,3955077798,3951524269,3947977120,3944436333,3940901892,3937373779,3933851978,3930336472,3926827243,
		3923324274,3919827551,3916337054,3912852768,3909374677,3905902763,3902437011,3898977404,3895523925,3892076559,3888635289,3885200099,3881770972,3878347894,3874930847,3871519817,3868114786,3864715740,3861322662,3857935537,3854554349,3851179082,3847809722,3844446251,3841088656,3837736921,3834391030,3831050968,3827716720,3824388271,3821065605,3817748708,
		3814437564,3811132159,3807832478,3804538505,3801250226,3797967627,3794690692,3791419407,3788153757,3784893728,3781639305,3778390474,3775147220,3771909530,3768677388,3765450781,3762229693,3759014113,3755804024,3752599413,3749400266,3746206569,3743018308,3739835469,3736658039,3733486003,3730319348,3727158061,3724002127,3720851533,3717706265,3714566311,
		3711431655,3708302286,3705178190,3702059353,3698945763,3695837405,3692734267,3689636335,3686543597,3683456040,3680373650,3677296414,3674224321,3671157355,3668095506,3665038760,3661987104,3658940526,3655899012,3652862551,3649831130,3646804736,3643783357,3640766980,3637755593,3634749183,3631747739,3628751247,3625759696,3622773074,3619791368,3616814566,
		3613842655,3610875625,3607913463,3604956157,3602003695,3599056065,3596113256,3593175255,3590242050,3587313631,3584389985,3581471101,3578556966,3575647571,3572742901,3569842948,3566947698,3564057141,3561171265,3558290058,3555413510,3552541609,3549674344,3546811703,3543953676,3541100251,3538251417,3535407164,3532567479,3529732353,3526901774,3524075731,
		3521254213,3518437209,3515624710,3512816703,3510013178,3507214124,3504419531,3501629388,3498843685,3496062410,3493285553,3490513105,3487745053,3484981388,3482222100,3479467177,3476716610,3473970388,3471228502,3468490940,3465757692,3463028749,3460304101,3457583736,3454867645,3452155818,3449448245,3446744915,3444045820,3441350948,3438660291,3435973837,
		3433291578,3430613504,3427939604,3425269869,3422604289,3419942855,3417285557,3414632385,3411983330,3409338381,3406697530,3404060768,3401428083,3398799468,3396174913,3393554407,3390937943,3388325510,3385717099,3383112701,3380512307,3377915908,3375323493,3372735055,3370150584,3367570070,3364993506,3362420881,3359852186,3357287414,3354726554,3352169597,
		3349616536,3347067361,3344522062,3341980632,3339443061,3336909341,3334379463,3331853418,3329331197,3326812793,3324298195,3321787396,3319280386,3316777158,3314277703,3311782012,3309290077,3306801889,3304317439,3301836721,3299359724,3296886441,3294416863,3291950982,3289488790,3287030278,3284575438,3282124263,3279676743,3277232870,3274792637,3272356036,
		3269923057,3267493694,3265067937,3262645780,3260227214,3257812231,3255400823,3252992982,3250588701,3248187970,3245790784,3243397133,3241007009,3238620406,3236237316,3233857729,3231481640,3229109039,3226739921,3224374276,3222012097,3219653376,3217298107,3214946281,3212597890,3210252928,3207911387,3205573259,3203238537,3200907214,3198579281,3196254732,
		3193933560,3191615756,3189301314,3186990226,3184682485,3182378084,3180077015,3177779272,3175484846,3173193731,3170905921,3168621406,3166340181,3164062239,3161787571,3159516172,3157248034,3154983151,3152721514,3150463117,3148207954,3145956017,3143707299,3141461794,3139219495,3136980394,3134744485,3132511761,3130282215,3128055841,3125832631,3123612579,
		3121395679,3119181923,3116971305,3114763819,3112559456,3110358212,3108160079,3105965051,3103773121,3101584282,3099398528,3097215853,3095036250,3092859713,3090686235,3088515809,3086348429,3084184090,3082022783,3079864504,3077709245,3075557001,3073407765,3071261531,3069118292,3066978042,3064840775,3062706485,3060575165,3058446809,3056321412,3054198967,
		3052079467,3049962907,3047849281,3045738582,3043630804,3041525942,3039423989,3037324939,3035228787,3033135525,3031045149,3028957653,3026873029,3024791274,3022712379,3020636341,3018563152,3016492806,3014425299,3012360625,3010298776,3008239748,3006183535,3004130131,3002079530,3000031727,2997986716,2995944490,2993905045,2991868375,2989834474,2987803337,
		2985774957,2983749330,2981726449,2979706309,2977688904,2975674230,2973662280,2971653049,2969646531,2967642721,2965641613,2963643202,2961647483,2959654449,2957664097,2955676419,2953691412,2951709068,2949729384,2947752354,2945777972,2943806233,2941837132,2939870663,2937906822,2935945602,2933986999,2932031008,2930077623,2928126839,2926178651,2924233053,
		2922290041,2920349609,2918411753,2916476467,2914543745,2912613584,2910685977,2908760921,2906838408,2904918436,2903000998,2901086090,2899173706,2897263842,2895356492,2893451653,2891549317,2889649482,2887752142,2885857291,2883964926,2882075041,2880187631,2878302691,2876420217,2874540204,2872662647,2870787540,2868914881,2867044662,2865176881,2863311531,
		2861448609,2859588109,2857730027,2855874358,2854021098,2852170241,2850321783,2848475720,2846632047,2844790758,2842951850,2841115318,2839281157,2837449363,2835619930,2833792856,2831968134,2830145761,2828325731,2826508041,2824692686,2822879661,2821068962,2819260585,2817454524,2815650776,2813849336,2812050199,2810253362,2808458820,2806666568,2804876602,
		2803088918,2801303511,2799520377,2797739511,2795960910,2794184569,2792410484,2790638650,2788869063,2787101719,2785336613,2783573742,2781813101,2780054685,2778298491,2776544515,2774792752,2773043198,2771295849,2769550700,2767807748,2766066989,2764328417,2762592030,2760857823,2759125792,2757395932,2755668241,2753942713,2752219344,2750498131,2748779070,
		2747062156,2745347386,2743634755,2741924259,2740215895,2738509659,2736805546,2735103552,2733403674,2731705908,2730010249,2728316695,2726625240,2724935881,2723248614,2721563436,2719880341,2718199328,2716520390,2714843526,2713168730,2711496000,2709825331,2708156719,2706490161,2704825653,2703163191,2701502771,2699844390,2698188044,2696533729,2694881441,
		2693231177,2691582933,2689936705,2688292489,2686650282,2685010081,2683371880,2681735678,2680101470,2678469252,2676839021,2675210774,2673584506,2671960214,2670337894,2668717544,2667099158,2665482735,2663868269,2662255758,2660645198,2659036585,2657429917,2655825188,2654222397,2652621539,2651022611,2649425610,2647830531,2646237372,2644646129,2643056798,
		2641469377,2639883861,2638300247,2636718532,2635138713,2633560786,2631984747,2630410593,2628838322,2627267928,2625699410,2624132764,2622567986,2621005073,2619444021,2617884829,2616327491,2614772005,2613218367,2611666575,2610116624,2608568512,2607022236,2605477791,2603935176,2602394386,2600855418,2599318269,2597782937,2596249417,2594717706,2593187802,
		2591659701,2590133399,2588608895,2587086184,2585565263,2584046129,2582528780,2581013211,2579499421,2577987404,2576477160,2574968684,2573461973,2571957025,2570453835,2568952402,2567452722,2565954791,2564458608,2562964168,2561471469,2559980508,2558491281,2557003786,2555518020,2554033979,2552551661,2551071063,2549592181,2548115013,2546639555,2545165806,
		2543693761,2542223417,2540754773,2539287824,2537822569,2536359003,2534897125,2533436931,2531978418,2530521583,2529066424,2527612938,2526161121,2524710971,2523262485,2521815661,2520370494,2518926983,2517485124,2516044915,2514606353,2513169435,2511734159,2510300521,2508868518,2507438148,2506009409,2504582296,2503156808,2501732942,2500310695,2498890064,
		2497471046,2496053639,2494637840,2493223646,2491811055,2490400063,2488990669,2487582869,2486176660,2484772041,2483369007,2481967558,2480567689,2479169398,2477772683,2476377541,2474983968,2473591964,2472201524,2470812647,2469425330,2468039569,2466655363,2465272709,2463891603,2462512045,2461134030,2459757557,2458382623,2457009225,2455637360,2454267027,
		2452898222,2451530943,2450165188,2448800953,2447438237,2446077037,2444717350,2443359173,2442002505,2440647343,2439293684,2437941526,2436590865,2435241701,2433894030,2432547850,2431203158,2429859951,2428518229,2427177987,2425839223,2424501936,2423166122,2421831780,2420498906,2419167498,2417837555,2416509073,2415182049,2413856483,2412532371,2411209711,
		2409888500,2408568736,2407250417,2405933540,2404618104,2403304105,2401991541,2400680410,2399370710,2398062438,2396755592,2395450170,2394146169,2392843587,2391542421,2390242670,2388944330,2387647401,2386351878,2385057761,2383765047,2382473733,2381183818,2379895299,2378608173,2377322439,2376038094,2374755136,2373473563,2372193372,2370914562,2369637129,
		2368361073,2367086390,2365813078,2364541136,2363270560,2362001349,2360733501,2359467013,2358201883,2356938109,2355675689,2354414621,2353154902,2351896531,2350639504,2349383821,2348129478,2346876474,2345624806,2344374473,2343125473,2341877802,2340631459,2339386443,2338142750,2336900378,2335659327,2334419592,2333181173,2331944068,2330708273,2329473788,
		2328240610,2327008737,2325778166,2324548896,2323320926,2322094251,2320868872,2319644785,2318421988,2317200481,2315980259,2314761322,2313543668,2312327293,2311112198,2309898378,2308685833,2307474560,2306264558,2305055824,2303848356,2302642153,2301437212,2300233531,2299031109,2297829944,2296630032,2295431374,2294233966,2293037806,2291842893,2290649225,
		2289456800,2288265615,2287075669,2285886961,2284699487,2283513246,2282328237,2281144456,2279961904,2278780576,2277600472,2276421590,2275243928,2274067483,2272892254,2271718240,2270545437,2269373845,2268203462,2267034285,2265866312,2264699543,2263533974,2262369605,2261206433,2260044456,2258883673,2257724082,2256565681,2255408468,2254252441,2253097598,
		2251943939,2250791460,2249640160,2248490037,2247341089,2246193316,2245046714,2243901282,2242757018,2241613921,2240471988,2239331218,2238191609,2237053160,2235915868,2234779732,2233644750,2232510920,2231378241,2230246710,2229116326,2227987088,2226858993,2225732041,2224606228,2223481553,2222358015,2221235612,2220114342,2218994204,2217875195,2216757315,
		2215640560,2214524931,2213410424,2212297038,2211184772,2210073624,2208963592,2207854675,2206746870,2205640177,2204534593,2203430116,2202326746,2201224481,2200123318,2199023256,2197924294,2196826430,2195729662,2194633988,2193539408,2192445918,2191353519,2190262207,2189171982,2188082842,2186994785,2185907809,2184821914,2183737096,2182653356,2181570691,
		2180489099,2179408579,2178329129,2177250749,2176173435,2175097187,2174022003,2172947881,2171874821,2170802819,2169731876,2168661988,2167593155,2166525375,2165458647,2164392969,2163328338,2162264755,2161202217,2160140723,2159080271,2158020860,2156962488,2155905153,2154848855,2153793591,2152739360,2151686161,2150633991,2149582851,2148532737,2147483648,
		4292871168,4290777085,4288685043,4286595041,4284507074,4282421141,4280337238,4278255361,4276175510,4274097679,4272021867,4269948070,4267876285,4265806510,4263738741,4261672976,4259609212,4257547446,4255487675,4253429895,4251374105,4249320301,4247268481,4245218641,4243170778,4241124891,4239080975,4237039029,4234999048,4232961031,4230924975,4228890877,
		4226858733,4224828541,4222800299,4220774003,4218749651,4216727240,4214706767,4212688230,4210671624,4208656949,4206644201,4204633376,4202624474,4200617490,4198612422,4196609267,4194608023,4192608686,4190611255,4188615725,4186622096,4184630363,4182640525,4180652578,4178666519,4176682347,4174700059,4172719651,4170741121,4168764466,4166789684,4164816772,
		4162845728,4160876548,4158909231,4156943773,4154980172,4153018425,4151058529,4149100483,4147144283,4145189926,4143237411,4141286734,4139337893,4137390886,4135445709,4133502361,4131560838,4129621138,4127683258,4125747197,4123812950,4121880517,4119949894,4118021078,4116094068,4114168860,4112245453,4110323843,4108404028,4106486005,4104569773,4102655328,
		4100742668,4098831791,4096922694,4095015374,4093109829,4091206057,4089304055,4087403821,4085505352,4083608646,4081713700,4079820512,4077929079,4076039399,4074151470,4072265289,4070380853,4068498161,4066617209,4064737996,4062860519,4060984775,4059110763,4057238479,4055367922,4053499089,4051631978,4049766585,4047902910,4046040949,4044180700,4042322161,
		4040465330,4038610204,4036756780,4034905057,4033055031,4031206702,4029360066,4027515121,4025671864,4023830294,4021990408,4020152204,4018315680,4016480833,4014647660,4012816160,4010986331,4009158169,4007331673,4005506841,4003683670,4001862158,4000042303,3998224102,3996407553,3994592654,3992779402,3990967796,3989157834,3987349512,3985542829,3983737782,
		3981934370,3980132590,3978332439,3976533917,3974737019,3972941745,3971148092,3969356057,3967565640,3965776836,3963989645,3962204065,3960420092,3958637724,3956856961,3955077798,3953300235,3951524269,3949749898,3947977120,3946205932,3944436333,3942668321,3940901892,3939137046,3937373779,3935612091,3933851978,3932093439,3930336472,3928581073,3926827243,
		3925074977,3923324274,3921575133,3919827551,3918081525,3916337054,3914594136,3912852768,3911112949,3909374677,3907637949,3905902763,3904169118,3902437011,3900706441,3898977404,3897249900,3895523925,3893799479,3892076559,3890355163,3888635289,3886916935,3885200099,3883484779,3881770972,3880058678,3878347894,3876638618,3874930847,3873224581,3871519817,
		3869816552,3868114786,3866414516,3864715740,3863018456,3861322662,3859628356,3857935537,3856244201,3854554349,3852865976,3851179082,3849493664,3847809722,3846127251,3844446251,3842766721,3841088656,3839412057,3837736921,3836063246,3834391030,3832720272,3831050968,3829383119,3827716720,3826051772,3824388271,3822726216,3821065605,3819406437,3817748708,
		3816092418,3814437564,3812784145,3811132159,3809481604,3807832478,3806184779,3804538505,3802893655,3801250226,3799608217,3797967627,3796328452,3794690692,3793054344,3791419407,3789785878,3788153757,3786523041,3784893728,3783265817,3781639305,3780014191,3778390474,3776768151,3775147220,3773527681,3771909530,3770292766,3768677388,3767063393,3765450781,
		3763839548,3762229693,3760621216,3759014113,3757408382,3755804024,3754201034,3752599413,3750999157,3749400266,3747802737,3746206569,3744611760,3743018308,3741426212,3739835469,3738246079,3736658039,3735071348,3733486003,3731902004,3730319348,3728738035,3727158061,3725579425,3724002127,3722426163,3720851533,3719278234,3717706265,3716135625,3714566311,
		3712998321,3711431655,3709866311,3708302286,3706739580,3705178190,3703618115,3702059353,3700501903,3698945763,3697390930,3695837405,3694285184,3692734267,3691184651,3689636335,3688089318,3686543597,3684999172,3683456040,3681914200,3680373650,3678834389,3677296414,3675759726,3674224321,3672690198,3671157355,3669625792,3668095506,3666566496,3665038760,
		3663512296,3661987104,3660463181,3658940526,3657419137,3655899012,3654380151,3652862551,3651346211,3649831130,3648317305,3646804736,3645293420,3643783357,3642274544,3640766980,3639260663,3637755593,3636251767,3634749183,3633247841,3631747739,3630248875,3628751247,3627254855,3625759696,3624265770,3622773074,3621281607,3619791368,3618302354,3616814566,
		3615328000,3613842655,3612358531,3610875625,3609393937,3607913463,3606434204,3604956157,3603479321,3602003695,3600529277,3599056065,3597584059,3596113256,3594643655,3593175255,3591708054,3590242050,3588777243,3587313631,3585851212,3584389985,3582929948,3581471101,3580013441,3578556966,3577101677,3575647571,3574194646,3572742901,3571292336,3569842948,
		3568394736,3566947698,3565501834,3564057141,3562613618,3561171265,3559730078,3558290058,3556851202,3555413510,3553976979,3552541609,3551107397,3549674344,3548242446,3546811703,3545382113,3543953676,3542526389,3541100251,3539675261,3538251417,3536828719,3535407164,3533986751,3532567479,3531149347,3529732353,3528316496,3526901774,3525488186,3524075731,
		3522664407,3521254213,3519845147,3518437209,3517030397,3515624710,3514220145,3512816703,3511414381,3510013178,3508613093,3507214124,3505816271,3504419531,3503023904,3501629388,3500235982,3498843685,3497452494,3496062410,3494673430,3493285553,3491898779,3490513105,3489128530,3487745053,3486362673,3484981388,3483601197,3482222100,3480844093,3479467177,
		3478091350,3476716610,3475342957,3473970388,3472598904,3471228502,3469859181,3468490940,3467123777,3465757692,3464392684,3463028749,3461665889,3460304101,3458943383,3457583736,3456225157,3454867645,3453511199,3452155818,3450801500,3449448245,3448096050,3446744915,3445394839,3444045820,3442697857,3441350948,3440005093,3438660291,3437316539,3435973837,
		3434632184,3433291578,3431952019,3430613504,3429276033,3427939604,3426604216,3425269869,3423936560,3422604289,3421273055,3419942855,3418613690,3417285557,3415958456,3414632385,3413307343,3411983330,3410660343,3409338381,3408017444,3406697530,3405378639,3404060768,3402743916,3401428083,3400113268,3398799468,3397486683,3396174913,3394864154,3393554407,
		3392245670,3390937943,3389631223,3388325510,3387020802,3385717099,3384414399,3383112701,3381812004,3380512307,3379213609,3377915908,3376619203,3375323493,3374028778,3372735055,3371442324,3370150584,3368859833,3367570070,3366281295,3364993506,3363706701,3362420881,3361136043,3359852186,3358569310,3357287414,3356006495,3354726554,3353447588,3352169597,
		3350892580,3349616536,3348341463,3347067361,3345794227,3344522062,3343250864,3341980632,3340711365,3339443061,3338175721,3336909341,3335643922,3334379463,3333115962,3331853418,3330591830,3329331197,3328071519,3326812793,3325555018,3324298195,3323042321,3321787396,3320533418,3319280386,3318028300,3316777158,3315526959,3314277703,3313029387,3311782012,
		3310535575,3309290077,3308045515,3306801889,3305559197,3304317439,3303076614,3301836721,3300597757,3299359724,3298122619,3296886441,3295651189,3294416863,3293183461,3291950982,3290719425,3289488790,3288259074,3287030278,3285802400,3284575438,3283349393,3282124263,3280900046,3279676743,3278454351,3277232870,3276012299,3274792637,3273573883,3272356036,
		3271139094,3269923057,3268707924,3267493694,3266280365,3265067937,3263856409,3262645780,3261436049,3260227214,3259019275,3257812231,3256606081,3255400823,3254196457,3252992982,3251790397,3250588701,3249387892,3248187970,3246988935,3245790784,3244593517,3243397133,3242201631,3241007009,3239813268,3238620406,3237428422,3236237316,3235047085,3233857729,
		3232669248,3231481640,3230294904,3229109039,3227924045,3226739921,3225556664,3224374276,3223192753,3222012097,3220832304,3219653376,3218475310,3217298107,3216121764,3214946281,3213771656,3212597890,3211424981,3210252928,3209081731,3207911387,3206741897,3205573259,3204405473,3203238537,3202072451,3200907214,3199742824,3198579281,3197416584,3196254732,
		3195093725,3193933560,3192774237,3191615756,3190458115,3189301314,3188145351,3186990226,3185835938,3184682485,3183529867,3182378084,3181227133,3180077015,3178927728,3177779272,3176631644,3175484846,3174338875,3173193731,3172049413,3170905921,3169763252,3168621406,3167480383,3166340181,3165200800,3164062239,3162924496,3161787571,3160651464,3159516172,
		3158381696,3157248034,3156115186,3154983151,3153851927,3152721514,3151591911,3150463117,3149335132,3148207954,3147081583,3145956017,3144831256,3143707299,3142584146,3141461794,3140340244,3139219495,3138099545,3136980394,3135862041,3134744485,3133627725,3132511761,3131396591,3130282215,3129168632,3128055841,3126943841,3125832631,3124722211,3123612579,
		3122503736,3121395679,3120288409,3119181923,3118076222,3116971305,3115867171,3114763819,3113661247,3112559456,3111458445,3110358212,3109258757,3108160079,3107062177,3105965051,3104868699,3103773121,3102678315,3101584282,3100491020,3099398528,3098306806,3097215853,3096125668,3095036250,3093947599,3092859713,3091772592,3090686235,3089600641,3088515809,
		3087431739,3086348429,3085265880,3084184090,3083103058,3082022783,3080943266,3079864504,3078786498,3077709245,3076632747,3075557001,3074482008,3073407765,3072334273,3071261531,3070189537,3069118292,3068047793,3066978042,3065909036,3064840775,3063773258,3062706485,3061640454,3060575165,3059510617,3058446809,3057383741,3056321412,3055259821,3054198967,
		3053138849,3052079467,3051020820,3049962907,3048905727,3047849281,3046793566,3045738582,3044684328,3043630804,3042578009,3041525942,3040474602,3039423989,3038374101,3037324939,3036276501,3035228787,3034181795,3033135525,3032089977,3031045149,3030001042,3028957653,3027914982,3026873029,3025831793,3024791274,3023751469,3022712379,3021674003,3020636341,
		3019599390,3018563152,3017527624,3016492806,3015458699,3014425299,3013392608,3012360625,3011329347,3010298776,3009268910,3008239748,3007211290,3006183535,3005156482,3004130131,3003104481,3002079530,3001055279,3000031727,2999008873,2997986716,2996965255,2995944490,2994924421,2993905045,2992886364,2991868375,2990851079,2989834474,2988818561,2987803337,
		2986788803,2985774957,2984761800,2983749330,2982737546,2981726449,2980716036,2979706309,2978697265,2977688904,2976681226,2975674230,2974667915,2973662280,2972657325,2971653049,2970649451,2969646531,2968644287,2967642721,2966641829,2965641613,2964642071,2963643202,2962645006,2961647483,2960650631,2959654449,2958658938,2957664097,2956669924,2955676419,
		2954683582,2953691412,2952699907,2951709068,2950718894,2949729384,2948740538,2947752354,2946764832,2945777972,2944791772,2943806233,2942821353,2941837132,2940853569,2939870663,2938888414,2937906822,2936925884,2935945602,2934965974,2933986999,2933008677,2932031008,2931053990,2930077623,2929101906,2928126839,2927152421,2926178651,2925205528,2924233053,
		2923261224,2922290041,2921319503,2920349609,2919380360,2918411753,2917443789,2916476467,2915509786,2914543745,2913578345,2912613584,2911649462,2910685977,2909723131,2908760921,2907799347,2906838408,2905878105,2904918436,2903959400,2903000998,2902043228,2901086090,2900129583,2899173706,2898218459,2897263842,2896309853,2895356492,2894403759,2893451653,
		2892500172,2891549317,2890599088,2889649482,2888700500,2887752142,2886804406,2885857291,2884910798,2883964926,2883019673,2882075041,2881131026,2880187631,2879244852,2878302691,2877361146,2876420217,2875479903,2874540204,2873601119,2872662647,2871724787,2870787540,2869850905,2868914881,2867979466,2867044662,2866110467,2865176881,2864243902,2863311531,
		2862379767,2861448609,2860518057,2859588109,2858658766,2857730027,2856801891,2855874358,2854947427,2854021098,2853095369,2852170241,2851245713,2850321783,2849398453,2848475720,2847553585,2846632047,2845711104,2844790758,2843871007,2842951850,2842033287,2841115318,2840197941,2839281157,2838364964,2837449363,2836534351,2835619930,2834706099,2833792856,
		2832880201,2831968134,2831056654,2830145761,2829235453,2828325731,2827416594,2826508041,2825600072,2824692686,2823785882,2822879661,2821974021,2821068962,2820164483,2819260585,2818357265,2817454524,2816552361,2815650776,2814749768,2813849336,2812949480,2812050199,2811151494,2810253362,2809355804,2808458820,2807562408,2806666568,2805771299,2804876602,
		2803982475,2803088918,2802195930,2801303511,2800411660,2799520377,2798629661,2797739511,2796849928,2795960910,2795072458,2794184569,2793297245,2792410484,2791524286,2790638650,2789753576,2788869063,2787985111,2787101719,2786218886,2785336613,2784454898,2783573742,2782693143,2781813101,2780933615,2780054685,2779176311,2778298491,2777421226,2776544515,
		2775668357,2774792752,2773917699,2773043198,2772169248,2771295849,2770423000,2769550700,2768678950,2767807748,2766937095,2766066989,2765197430,2764328417,2763459951,2762592030,2761724654,2760857823,2759991536,2759125792,2758260591,2757395932,2756531816,2755668241,2754805206,2753942713,2753080759,2752219344,2751358469,2750498131,2749638332,2748779070,
		2747920345,2747062156,2746204503,2745347386,2744490803,2743634755,2742779240,2741924259,2741069811,2740215895,2739362511,2738509659,2737657337,2736805546,2735954284,2735103552,2734253349,2733403674,2732554527,2731705908,2730857816,2730010249,2729163209,2728316695,2727470705,2726625240,2725780299,2724935881,2724091986,2723248614,2722405764,2721563436,
		2720721628,2719880341,2719039575,2718199328,2717359600,2716520390,2715681699,2714843526,2714005870,2713168730,2712332107,2711496000,2710660408,2709825331,2708990768,2708156719,2707323184,2706490161,2705657651,2704825653,2703994167,2703163191,2702332726,2701502771,2700673326,2699844390,2699015963,2698188044,2697360633,2696533729,2695707332,2694881441,
		2694056056,2693231177,2692406803,2691582933,2690759567,2689936705,2689114345,2688292489,2687471135,2686650282,2685829931,2685010081,2684190730,2683371880,2682553530,2681735678,2680918325,2680101470,2679285112,2678469252,2677653889,2676839021,2676024650,2675210774,2674397392,2673584506,2672772113,2671960214,2671148808,2670337894,2669527473,2668717544,
		2667908106,2667099158,2666290701,2665482735,2664675257,2663868269,2663061769,2662255758,2661450234,2660645198,2659840648,2659036585,2658233008,2657429917,2656627310,2655825188,2655023551,2654222397,2653421727,2652621539,2651821834,2651022611,2650223870,2649425610,2648627830,2647830531,2647033712,2646237372,2645441511,2644646129,2643851224,2643056798,
		2642262849,2641469377,2640676381,2639883861,2639091816,2638300247,2637509153,2636718532,2635928386,2635138713,2634349513,2633560786,2632772530,2631984747,2631197435,2630410593,2629624222,2628838322,2628052890,2627267928,2626483435,2625699410,2624915853,2624132764,2623350141,2622567986,2621786296,2621005073,2620224315,2619444021,2618664193,2617884829,
		2617105928,2616327491,2615549516,2614772005,2613994955,2613218367,2612442241,2611666575,2610891370,2610116624,2609342339,2608568512,2607795145,2607022236,2606249785,2605477791,2604706255,2603935176,2603164553,2602394386,2601624674,2600855418,2600086617,2599318269,2598550376,2597782937,2597015950,2596249417,2595483335,2594717706,2593952528,2593187802,
		2592423526,2591659701,2590896325,2590133399,2589370923,2588608895,2587847315,2587086184,2586325500,2585565263,2584805473,2584046129,2583287232,2582528780,2581770773,2581013211,2580256094,2579499421,2578743191,2577987404,2577232061,2576477160,2575722701,2574968684,2574215108,2573461973,2572709279,2571957025,2571205210,2570453835,2569702899,2568952402,
		2568202343,2567452722,2566703538,2565954791,2565206481,2564458608,2563711170,2562964168,2562217601,2561471469,2560725771,2559980508,2559235678,2558491281,2557747317,2557003786,2556260687,2555518020,2554775784,2554033979,2553292605,2552551661,2551811147,2551071063,2550331407,2549592181,2548853383,2548115013,2547377070,2546639555,2545902467,2545165806,
		2544429570,2543693761,2542958376,2542223417,2541488883,2540754773,2540021087,2539287824,2538554985,2537822569,2537090575,2536359003,2535627854,2534897125,2534166818,2533436931,2532707464,2531978418,2531249791,2530521583,2529793795,2529066424,2528339472,2527612938,2526886821,2526161121,2525435838,2524710971,2523986521,2523262485,2522538865,2521815661,
		2521092870,2520370494,2519648532,2518926983,2518205847,2517485124,2516764814,2516044915,2515325429,2514606353,2513887689,2513169435,2512451592,2511734159,2511017135,2510300521,2509584315,2508868518,2508153129,2507438148,2506723575,2506009409,2505295649,2504582296,2503869349,2503156808,2502444673,2501732942,2501021616,2500310695,2499600177,2498890064,
		2498180353,2497471046,2496762141,2496053639,2495345539,2494637840,2493930543,2493223646,2492517150,2491811055,2491105359,2490400063,2489695167,2488990669,2488286570,2487582869,2486879566,2486176660,2485474152,2484772041,2484070326,2483369007,2482668085,2481967558,2481267426,2480567689,2479868346,2479169398,2478470844,2477772683,2477074915,2476377541,
		2475680558,2474983968,2474287770,2473591964,2472896549,2472201524,2471506891,2470812647,2470118794,2469425330,2468732255,2468039569,2467347272,2466655363,2465963842,2465272709,2464581962,2463891603,2463201631,2462512045,2461822845,2461134030,2460445601,2459757557,2459069898,2458382623,2457695732,2457009225,2456323101,2455637360,2454952002,2454267027,
		2453582433,2452898222,2452214392,2451530943,2450847875,2450165188,2449482881,2448800953,2448119406,2447438237,2446757448,2446077037,2445397004,2444717350,2444038073,2443359173,2442680651,2442002505,2441324736,2440647343,2439970326,2439293684,2438617417,2437941526,2437266008,2436590865,2435916096,2435241701,2434567679,2433894030,2433220754,2432547850,
		2431875318,2431203158,2430531369,2429859951,2429188905,2428518229,2427847923,2427177987,2426508420,2425839223,2425170395,2424501936,2423833845,2423166122,2422498767,2421831780,2421165159,2420498906,2419833019,2419167498,2418502344,2417837555,2417173131,2416509073,2415845379,2415182049,2414519084,2413856483,2413194245,2412532371,2411870859,2411209711,
		2410548924,2409888500,2409228437,2408568736,2407909396,2407250417,2406591799,2405933540,2405275642,2404618104,2403960925,2403304105,2402647644,2401991541,2401335797,2400680410,2400025382,2399370710,2398716396,2398062438,2397408837,2396755592,2396102703,2395450170,2394797992,2394146169,2393494700,2392843587,2392192827,2391542421,2390892369,2390242670,
		2389593324,2388944330,2388295689,2387647401,2386999464,2386351878,2385704644,2385057761,2384411229,2383765047,2383119215,2382473733,2381828601,2381183818,2380539384,2379895299,2379251562,2378608173,2377965132,2377322439,2376680093,2376038094,2375396442,2374755136,2374114177,2373473563,2372833295,2372193372,2371553795,2370914562,2370275673,2369637129,
		2368998929,2368361073,2367723560,2367086390,2366449563,2365813078,2365176936,2364541136,2363905677,2363270560,2362635784,2362001349,2361367255,2360733501,2360100087,2359467013,2358834278,2358201883,2357569827,2356938109,2356306730,2355675689,2355044986,2354414621,2353784593,2353154902,2352525548,2351896531,2351267849,2350639504,2350011495,2349383821,
		2348756482,2348129478,2347502809,2346876474,2346250473,2345624806,2344999473,2344374473,2343749807,2343125473,2342501471,2341877802,2341254465,2340631459,2340008785,2339386443,2338764431,2338142750,2337521399,2336900378,2336279688,2335659327,2335039295,2334419592,2333800219,2333181173,2332562457,2331944068,2331326007,2330708273,2330090867,2329473788,
		2328857036,2328240610,2327624510,2327008737,2326393289,2325778166,2325163369,2324548896,2323934749,2323320926,2322707426,2322094251,2321481400,2320868872,2320256667,2319644785,2319033225,2318421988,2317811074,2317200481,2316590209,2315980259,2315370630,2314761322,2314152335,2313543668,2312935321,2312327293,2311719586,2311112198,2310505128,2309898378,
		2309291946,2308685833,2308080038,2307474560,2306869401,2306264558,2305660033,2305055824,2304451932,2303848356,2303245097,2302642153,2302039525,2301437212,2300835214,2300233531,2299632163,2299031109,2298430370,2297829944,2297229831,2296630032,2296030547,2295431374,2294832513,2294233966,2293635730,2293037806,2292440194,2291842893,2291245904,2290649225,
		2290052857,2289456800,2288861052,2288265615,2287670487,2287075669,2286481160,2285886961,2285293069,2284699487,2284106212,2283513246,2282920588,2282328237,2281736193,2281144456,2280553027,2279961904,2279371087,2278780576,2278190371,2277600472,2277010879,2276421590,2275832607,2275243928,2274655553,2274067483,2273479717,2272892254,2272305095,2271718240,
		2271131687,2270545437,2269959490,2269373845,2268788502,2268203462,2267618722,2267034285,2266450148,2265866312,2265282777,2264699543,2264116609,2263533974,2262951640,2262369605,2261787869,2261206433,2260625295,2260044456,2259463916,2258883673,2258303729,2257724082,2257144733,2256565681,2255986926,2255408468,2254830306,2254252441,2253674872,2253097598,
		2252520621,2251943939,2251367552,2250791460,2250215662,2249640160,2249064951,2248490037,2247915416,2247341089,2246767056,2246193316,2245619868,2245046714,2244473852,2243901282,2243329004,2242757018,2242185324,2241613921,2241042809,2240471988,2239901458,2239331218,2238761269,2238191609,2237622240,2237053160,2236484369,2235915868,2235347655,2234779732,
		2234212097,2233644750,2233077691,2232510920,2231944436,2231378241,2230812332,2230246710,2229681375,2229116326,2228551564,2227987088,2227422898,2226858993,2226295374,2225732041,2225168992,2224606228,2224043748,2223481553,2222919642,2222358015,2221796672,2221235612,2220674836,2220114342,2219554132,2218994204,2218434558,2217875195,2217316114,2216757315,
		2216198797,2215640560,2215082605,2214524931,2213967537,2213410424,2212853591,2212297038,2211740765,2211184772,2210629059,2210073624,2209518469,2208963592,2208408994,2207854675,2207300633,2206746870,2206193385,2205640177,2205087246,2204534593,2203982216,2203430116,2202878293,2202326746,2201775475,2201224481,2200673761,2200123318,2199573149,2199023256,
		2198473638,2197924294,2197375225,2196826430,2196277909,2195729662,2195181688,2194633988,2194086561,2193539408,2192992527,2192445918,2191899582,2191353519,2190807727,2190262207,2189716959,2189171982,2188627276,2188082842,2187538678,2186994785,2186451162,2185907809,2185364727,2184821914,2184279370,2183737096,2183195092,2182653356,2182111889,2181570691,
		2181029761,2180489099,2179948705,2179408579,2178868720,2178329129,2177789805,2177250749,2176711958,2176173435,2175635178,2175097187,2174559462,2174022003,2173484810,2172947881,2172411219,2171874821,2171338688,2170802819,2170267215,2169731876,2169196800,2168661988,2168127440,2167593155,2167059134,2166525375,2165991880,2165458647,2164925677,2164392969,
		2163860523,2163328338,2162796416,2162264755,2161733356,2161202217,2160671340,2160140723,2159610367,2159080271,2158550435,2158020860,2157491544,2156962488,2156433691,2155905153,2155376874,2154848855,2154321093,2153793591,2153266346,2152739360,2152212631,2151686161,2151159947,2150633991,2150108292,2149582851,2149057665,2148532737,2148008065,2147483648,
		4293918976,4292871168,4291823871,4290777085,4289730809,4288685043,4287639787,4286595041,4285550803,4284507074,4283463854,4282421141,4281378936,4280337238,4279296046,4278255361,4277215183,4276175510,4275136342,4274097679,4273059521,4272021867,4270984716,4269948070,4268911926,4267876285,4266841146,4265806510,4264772375,4263738741,4262705609,4261672976,
		4260640844,4259609212,4258578080,4257547446,4256517311,4255487675,4254458536,4253429895,4252401752,4251374105,4250346955,4249320301,4248294143,4247268481,4246243313,4245218641,4244194462,4243170778,4242147588,4241124891,4240102687,4239080975,4238059756,4237039029,4236018793,4234999048,4233979795,4232961031,4231942758,4230924975,4229907681,4228890877,
		4227874561,4226858733,4225843393,4224828541,4223814177,4222800299,4221786908,4220774003,4219761585,4218749651,4217738203,4216727240,4215716762,4214706767,4213697257,4212688230,4211679686,4210671624,4209664046,4208656949,4207650334,4206644201,4205638548,4204633376,4203628685,4202624474,4201620742,4200617490,4199614716,4198612422,4197610605,4196609267,
		4195608406,4194608023,4193608116,4192608686,4191609732,4190611255,4189613252,4188615725,4187618673,4186622096,4185625993,4184630363,4183635207,4182640525,4181646315,4180652578,4179659313,4178666519,4177674198,4176682347,4175690968,4174700059,4173709620,4172719651,4171730151,4170741121,4169752559,4168764466,4167776841,4166789684,4165802995,4164816772,
		4163831017,4162845728,4161860905,4160876548,4159892657,4158909231,4157926270,4156943773,4155961740,4154980172,4153999066,4153018425,4152038246,4151058529,4150079275,4149100483,4148122152,4147144283,4146166874,4145189926,4144213439,4143237411,4142261843,4141286734,4140312084,4139337893,4138364161,4137390886,4136418069,4135445709,4134473806,4133502361,
		4132531371,4131560838,4130590760,4129621138,4128651971,4127683258,4126715000,4125747197,4124779847,4123812950,4122846507,4121880517,4120914979,4119949894,4118985260,4118021078,4117057348,4116094068,4115131239,4114168860,4113206932,4112245453,4111284423,4110323843,4109363711,4108404028,4107444793,4106486005,4105527665,4104569773,4103612327,4102655328,
		4101698775,4100742668,4099787007,4098831791,4097877020,4096922694,4095968812,4095015374,4094062380,4093109829,4092157722,4091206057,4090254835,4089304055,4088353718,4087403821,4086454366,4085505352,4084556779,4083608646,4082660953,4081713700,4080766886,4079820512,4078874576,4077929079,4076984020,4076039399,4075095216,4074151470,4073208161,4072265289,
		4071322853,4070380853,4069439289,4068498161,4067557467,4066617209,4065677385,4064737996,4063799041,4062860519,4061922431,4060984775,4060047553,4059110763,4058174405,4057238479,4056302985,4055367922,4054433290,4053499089,4052565318,4051631978,4050699067,4049766585,4048834533,4047902910,4046971715,4046040949,4045110611,4044180700,4043251217,4042322161,
		4041393532,4040465330,4039537554,4038610204,4037683279,4036756780,4035830706,4034905057,4033979832,4033055031,4032130655,4031206702,4030283172,4029360066,4028437382,4027515121,4026593281,4025671864,4024750869,4023830294,4022910141,4021990408,4021071096,4020152204,4019233732,4018315680,4017398047,4016480833,4015564037,4014647660,4013731701,4012816160,
		4011901037,4010986331,4010072042,4009158169,4008244713,4007331673,4006419050,4005506841,4004595048,4003683670,4002772707,4001862158,4000952023,4000042303,3999132995,3998224102,3997315621,3996407553,3995499897,3994592654,3993685822,3992779402,3991873394,3990967796,3990062610,3989157834,3988253468,3987349512,3986445966,3985542829,3984640101,3983737782,
		3982835872,3981934370,3981033276,3980132590,3979232311,3978332439,3977432975,3976533917,3975635265,3974737019,3973839179,3972941745,3972044716,3971148092,3970251872,3969356057,3968460647,3967565640,3966671036,3965776836,3964883040,3963989645,3963096654,3962204065,3961311877,3960420092,3959528707,3958637724,3957747142,3956856961,3955967179,3955077798,
		3954188817,3953300235,3952412053,3951524269,3950636884,3949749898,3948863310,3947977120,3947091327,3946205932,3945320934,3944436333,3943552129,3942668321,3941784909,3940901892,3940019271,3939137046,3938255215,3937373779,3936492738,3935612091,3934731838,3933851978,3932972512,3932093439,3931214759,3930336472,3929458576,3928581073,3927703962,3926827243,
		3925950914,3925074977,3924199430,3923324274,3922449509,3921575133,3920701147,3919827551,3918954343,3918081525,3917209095,3916337054,3915465401,3914594136,3913723258,3912852768,3911982665,3911112949,3910243620,3909374677,3908506120,3907637949,3906770164,3905902763,3905035748,3904169118,3903302873,3902437011,3901571534,3900706441,3899841731,3898977404,
		3898113460,3897249900,3896386721,3895523925,3894661511,3893799479,3892937828,3892076559,3891215671,3890355163,3889495036,3888635289,3887775922,3886916935,3886058327,3885200099,3884342249,3883484779,3882627686,3881770972,3880914636,3880058678,3879203097,3878347894,3877493067,3876638618,3875784544,3874930847,3874077526,3873224581,3872372011,3871519817,
		3870667997,3869816552,3868965482,3868114786,3867264464,3866414516,3865564941,3864715740,3863866911,3863018456,3862170372,3861322662,3860475323,3859628356,3858781761,3857935537,3857089684,3856244201,3855399090,3854554349,3853709977,3852865976,3852022344,3851179082,3850336189,3849493664,3848651509,3847809722,3846968302,3846127251,3845286568,3844446251,
		3843606303,3842766721,3841927505,3841088656,3840250174,3839412057,3838574307,3837736921,3836899901,3836063246,3835226956,3834391030,3833555469,3832720272,3831885438,3831050968,3830216862,3829383119,3828549738,3827716720,3826884065,3826051772,3825219841,3824388271,3823557063,3822726216,3821895730,3821065605,3820235841,3819406437,3818577392,3817748708,
		3816920383,3816092418,3815264812,3814437564,3813610676,3812784145,3811957973,3811132159,3810306703,3809481604,3808656862,3807832478,3807008450,3806184779,3805361464,3804538505,3803715902,3802893655,3802071763,3801250226,3800429044,3799608217,3798787745,3797967627,3797147862,3796328452,3795509395,3794690692,3793872341,3793054344,3792236699,3791419407,
		3790602466,3789785878,3788969642,3788153757,3787338223,3786523041,3785708209,3784893728,3784079597,3783265817,3782452386,3781639305,3780826574,3780014191,3779202158,3778390474,3777579138,3776768151,3775957512,3775147220,3774337277,3773527681,3772718432,3771909530,3771100975,3770292766,3769484904,3768677388,3767870218,3767063393,3766256914,3765450781,
		3764644992,3763839548,3763034449,3762229693,3761425283,3760621216,3759817492,3759014113,3758211076,3757408382,3756606032,3755804024,3755002358,3754201034,3753400053,3752599413,3751799114,3750999157,3750199541,3749400266,3748601331,3747802737,3747004483,3746206569,3745408994,3744611760,3743814864,3743018308,3742222090,3741426212,3740630671,3739835469,
		3739040605,3738246079,3737451890,3736658039,3735864525,3735071348,3734278507,3733486003,3732693836,3731902004,3731110508,3730319348,3729528524,3728738035,3727947880,3727158061,3726368576,3725579425,3724790609,3724002127,3723213978,3722426163,3721638681,3720851533,3720064717,3719278234,3718492083,3717706265,3716920779,3716135625,3715350802,3714566311,
		3713782150,3712998321,3712214823,3711431655,3710648818,3709866311,3709084134,3708302286,3707520769,3706739580,3705958721,3705178190,3704397988,3703618115,3702838570,3702059353,3701280464,3700501903,3699723669,3698945763,3698168183,3697390930,3696614004,3695837405,3695061131,3694285184,3693509563,3692734267,3691959296,3691184651,3690410331,3689636335,
		3688862664,3688089318,3687316296,3686543597,3685771223,3684999172,3684227444,3683456040,3682684959,3681914200,3681143764,3680373650,3679603858,3678834389,3678065241,3677296414,3676527909,3675759726,3674991863,3674224321,3673457099,3672690198,3671923617,3671157355,3670391414,3669625792,3668860489,3668095506,3667330842,3666566496,3665802469,3665038760,
		3664275369,3663512296,3662749541,3661987104,3661224984,3660463181,3659701695,3658940526,3658179673,3657419137,3656658916,3655899012,3655139424,3654380151,3653621194,3652862551,3652104224,3651346211,3650588514,3649831130,3649074061,3648317305,3647560864,3646804736,3646048922,3645293420,3644538232,3643783357,3643028794,3642274544,3641520606,3640766980,
		3640013666,3639260663,3638507972,3637755593,3637003524,3636251767,3635500320,3634749183,3633998357,3633247841,3632497635,3631747739,3630998152,3630248875,3629499907,3628751247,3628002897,3627254855,3626507122,3625759696,3625012579,3624265770,3623519268,3622773074,3622027187,3621281607,3620536334,3619791368,3619046708,3618302354,3617558307,3616814566,
		3616071130,3615328000,3614585175,3613842655,3613100441,3612358531,3611616926,3610875625,3610134629,3609393937,3608653548,3607913463,3607173682,3606434204,3605695029,3604956157,3604217588,3603479321,3602741357,3602003695,3601266335,3600529277,3599792520,3599056065,3598319911,3597584059,3596848507,3596113256,3595378305,3594643655,3593909305,3593175255,
		3592441504,3591708054,3590974902,3590242050,3589509497,3588777243,3588045288,3587313631,3586582273,3585851212,3585120450,3584389985,3583659818,3582929948,3582200376,3581471101,3580742122,3580013441,3579285055,3578556966,3577829174,3577101677,3576374476,3575647571,3574920961,3574194646,3573468626,3572742901,3572017471,3571292336,3570567495,3569842948,
		3569118695,3568394736,3567671070,3566947698,3566224619,3565501834,3564779341,3564057141,3563335233,3562613618,3561892295,3561171265,3560450526,3559730078,3559009922,3558290058,3557570485,3556851202,3556132211,3555413510,3554695099,3553976979,3553259149,3552541609,3551824358,3551107397,3550390726,3549674344,3548958250,3548242446,3547526930,3546811703,
		3546096764,3545382113,3544667751,3543953676,3543239889,3542526389,3541813176,3541100251,3540387613,3539675261,3538963196,3538251417,3537539925,3536828719,3536117798,3535407164,3534696815,3533986751,3533276973,3532567479,3531858271,3531149347,3530440708,3529732353,3529024282,3528316496,3527608993,3526901774,3526194838,3525488186,3524781817,3524075731,
		3523369927,3522664407,3521959169,3521254213,3520549539,3519845147,3519141038,3518437209,3517733663,3517030397,3516327413,3515624710,3514922287,3514220145,3513518284,3512816703,3512115402,3511414381,3510713639,3510013178,3509312996,3508613093,3507913469,3507214124,3506515058,3505816271,3505117762,3504419531,3503721579,3503023904,3502326507,3501629388,
		3500932547,3500235982,3499539695,3498843685,3498147951,3497452494,3496757314,3496062410,3495367782,3494673430,3493979354,3493285553,3492592028,3491898779,3491205804,3490513105,3489820680,3489128530,3488436654,3487745053,3487053726,3486362673,3485671894,3484981388,3484291156,3483601197,3482911512,3482222100,3481532960,3480844093,3480155499,3479467177,
		3478779127,3478091350,3477403844,3476716610,3476029648,3475342957,3474656537,3473970388,3473284511,3472598904,3471913568,3471228502,3470543706,3469859181,3469174925,3468490940,3467807224,3467123777,3466440600,3465757692,3465075054,3464392684,3463710582,3463028749,3462347185,3461665889,3460984861,3460304101,3459623608,3458943383,3458263426,3457583736,
		3456904313,3456225157,3455546267,3454867645,3454189289,3453511199,3452833375,3452155818,3451478526,3450801500,3450124740,3449448245,3448772015,3448096050,3447420350,3446744915,3446069745,3445394839,3444720197,3444045820,3443371706,3442697857,3442024271,3441350948,3440677889,3440005093,3439332561,3438660291,3437988284,3437316539,3436645057,3435973837,
		3435302880,3434632184,3433961750,3433291578,3432621668,3431952019,3431282631,3430613504,3429944638,3429276033,3428607688,3427939604,3427271780,3426604216,3425936913,3425269869,3424603085,3423936560,3423270295,3422604289,3421938542,3421273055,3420607826,3419942855,3419278143,3418613690,3417949494,3417285557,3416621877,3415958456,3415295292,3414632385,
		3413969735,3413307343,3412645208,3411983330,3411321708,3410660343,3409999234,3409338381,3408677785,3408017444,3407357359,3406697530,3406037957,3405378639,3404719576,3404060768,3403402215,3402743916,3402085873,3401428083,3400770548,3400113268,3399456241,3398799468,3398142949,3397486683,3396830671,3396174913,3395519407,3394864154,3394209154,3393554407,
		3392899913,3392245670,3391591681,3390937943,3390284457,3389631223,3388978241,3388325510,3387673030,3387020802,3386368825,3385717099,3385065624,3384414399,3383763425,3383112701,3382462228,3381812004,3381162031,3380512307,3379862833,3379213609,3378564634,3377915908,3377267431,3376619203,3375971224,3375323493,3374676011,3374028778,3373381792,3372735055,
		3372088566,3371442324,3370796330,3370150584,3369505085,3368859833,3368214828,3367570070,3366925559,3366281295,3365637277,3364993506,3364349980,3363706701,3363063668,3362420881,3361778339,3361136043,3360493992,3359852186,3359210626,3358569310,3357928240,3357287414,3356646832,3356006495,3355366402,3354726554,3354086949,3353447588,3352808471,3352169597,
		3351530967,3350892580,3350254437,3349616536,3348978878,3348341463,3347704291,3347067361,3346430673,3345794227,3345158024,3344522062,3343886342,3343250864,3342615627,3341980632,3341345878,3340711365,3340077093,3339443061,3338809271,3338175721,3337542411,3336909341,3336276512,3335643922,3335011573,3334379463,3333747593,3333115962,3332484570,3331853418,
		3331222505,3330591830,3329961394,3329331197,3328701239,3328071519,3327442037,3326812793,3326183787,3325555018,3324926488,3324298195,3323670139,3323042321,3322414740,3321787396,3321160288,3320533418,3319906784,3319280386,3318654225,3318028300,3317402611,3316777158,3316151941,3315526959,3314902214,3314277703,3313653428,3313029387,3312405582,3311782012,
		3311158676,3310535575,3309912709,3309290077,3308667679,3308045515,3307423585,3306801889,3306180426,3305559197,3304938202,3304317439,3303696910,3303076614,3302456551,3301836721,3301217123,3300597757,3299978624,3299359724,3298741055,3298122619,3297504414,3296886441,3296268699,3295651189,3295033910,3294416863,3293800046,3293183461,3292567106,3291950982,
		3291335088,3290719425,3290103992,3289488790,3288873817,3288259074,3287644561,3287030278,3286416224,3285802400,3285188804,3284575438,3283962301,3283349393,3282736713,3282124263,3281512040,3280900046,3280288280,3279676743,3279065433,3278454351,3277843497,3277232870,3276622471,3276012299,3275402355,3274792637,3274183147,3273573883,3272964846,3272356036,
		3271747452,3271139094,3270530962,3269923057,3269315378,3268707924,3268100696,3267493694,3266886917,3266280365,3265674039,3265067937,3264462061,3263856409,3263250983,3262645780,3262040802,3261436049,3260831519,3260227214,3259623133,3259019275,3258415641,3257812231,3257209044,3256606081,3256003340,3255400823,3254798529,3254196457,3253594608,3252992982,
		3252391578,3251790397,3251189438,3250588701,3249988185,3249387892,3248787820,3248187970,3247588342,3246988935,3246389749,3245790784,3245192040,3244593517,3243995214,3243397133,3242799271,3242201631,3241604210,3241007009,3240410029,3239813268,3239216728,3238620406,3238024305,3237428422,3236832759,3236237316,3235642091,3235047085,3234452298,3233857729,
		3233263379,3232669248,3232075335,3231481640,3230888163,3230294904,3229701863,3229109039,3228516434,3227924045,3227331874,3226739921,3226148184,3225556664,3224965362,3224374276,3223783406,3223192753,3222602317,3222012097,3221422093,3220832304,3220242732,3219653376,3219064236,3218475310,3217886601,3217298107,3216709828,3216121764,3215533915,3214946281,
		3214358861,3213771656,3213184666,3212597890,3212011329,3211424981,3210838848,3210252928,3209667223,3209081731,3208496452,3207911387,3207326535,3206741897,3206157472,3205573259,3204989260,3204405473,3203821899,3203238537,3202655388,3202072451,3201489727,3200907214,3200324913,3199742824,3199160947,3198579281,3197997827,3197416584,3196835553,3196254732,
		3195674123,3195093725,3194513537,3193933560,3193353793,3192774237,3192194892,3191615756,3191036831,3190458115,3189879610,3189301314,3188723228,3188145351,3187567684,3186990226,3186412977,3185835938,3185259107,3184682485,3184106072,3183529867,3182953871,3182378084,3181802504,3181227133,3180651970,3180077015,3179502268,3178927728,3178353396,3177779272,
		3177205354,3176631644,3176058142,3175484846,3174911757,3174338875,3173766200,3173193731,3172621469,3172049413,3171477564,3170905921,3170334483,3169763252,3169192226,3168621406,3168050792,3167480383,3166910180,3166340181,3165770388,3165200800,3164631417,3164062239,3163493265,3162924496,3162355932,3161787571,3161219415,3160651464,3160083716,3159516172,
		3158948832,3158381696,3157814763,3157248034,3156681509,3156115186,3155549067,3154983151,3154417437,3153851927,3153286619,3152721514,3152156611,3151591911,3151027413,3150463117,3149899024,3149335132,3148771442,3148207954,3147644668,3147081583,3146518699,3145956017,3145393536,3144831256,3144269177,3143707299,3143145622,3142584146,3142022870,3141461794,
		3140900919,3140340244,3139779769,3139219495,3138659420,3138099545,3137539869,3136980394,3136421117,3135862041,3135303163,3134744485,3134186005,3133627725,3133069643,3132511761,3131954076,3131396591,3130839304,3130282215,3129725324,3129168632,3128612137,3128055841,3127499742,3126943841,3126388137,3125832631,3125277322,3124722211,3124167297,3123612579,
		3123058059,3122503736,3121949609,3121395679,3120841946,3120288409,3119735068,3119181923,3118628975,3118076222,3117523666,3116971305,3116419140,3115867171,3115315397,3114763819,3114212435,3113661247,3113110254,3112559456,3112008853,3111458445,3110908231,3110358212,3109808387,3109258757,3108709321,3108160079,3107611031,3107062177,3106513517,3105965051,
		3105416778,3104868699,3104320813,3103773121,3103225621,3102678315,3102131202,3101584282,3101037555,3100491020,3099944678,3099398528,3098852571,3098306806,3097761234,3097215853,3096670665,3096125668,3095580863,3095036250,3094491829,3093947599,3093403560,3092859713,3092316057,3091772592,3091229318,3090686235,3090143342,3089600641,3089058129,3088515809,
		3087973679,3087431739,3086889989,3086348429,3085807060,3085265880,3084724890,3084184090,3083643479,3083103058,3082562826,3082022783,3081482930,3080943266,3080403790,3079864504,3079325406,3078786498,3078247777,3077709245,3077170902,3076632747,3076094780,3075557001,3075019410,3074482008,3073944792,3073407765,3072870925,3072334273,3071797808,3071261531,
		3070725440,3070189537,3069653821,3069118292,3068582949,3068047793,3067512824,3066978042,3066443446,3065909036,3065374812,3064840775,3064306923,3063773258,3063239778,3062706485,3062173376,3061640454,3061107717,3060575165,3060042798,3059510617,3058978621,3058446809,3057915183,3057383741,3056852484,3056321412,3055790524,3055259821,3054729301,3054198967,
		3053668816,3053138849,3052609066,3052079467,3051550052,3051020820,3050491772,3049962907,3049434226,3048905727,3048377412,3047849281,3047321332,3046793566,3046265982,3045738582,3045211364,3044684328,3044157475,3043630804,3043104315,3042578009,3042051884,3041525942,3041000181,3040474602,3039949205,3039423989,3038898954,3038374101,3037849430,3037324939,
		3036800630,3036276501,3035752553,3035228787,3034705201,3034181795,3033658570,3033135525,3032612661,3032089977,3031567473,3031045149,3030523006,3030001042,3029479257,3028957653,3028436228,3027914982,3027393916,3026873029,3026352322,3025831793,3025311444,3024791274,3024271282,3023751469,3023231835,3022712379,3022193102,3021674003,3021155083,3020636341,
		3020117776,3019599390,3019081182,3018563152,3018045299,3017527624,3017010126,3016492806,3015975664,3015458699,3014941910,3014425299,3013908865,3013392608,3012876528,3012360625,3011844898,3011329347,3010813974,3010298776,3009783755,3009268910,3008754241,3008239748,3007725431,3007211290,3006697325,3006183535,3005669921,3005156482,3004643219,3004130131,
		3003617218,3003104481,3002591918,3002079530,3001567317,3001055279,3000543416,3000031727,2999520213,2999008873,2998497707,2997986716,2997475898,2996965255,2996454786,2995944490,2995434369,2994924421,2994414646,2993905045,2993395618,2992886364,2992377283,2991868375,2991359641,2990851079,2990342690,2989834474,2989326431,2988818561,2988310862,2987803337,
		2987295984,2986788803,2986281794,2985774957,2985268292,2984761800,2984255479,2983749330,2983243352,2982737546,2982231912,2981726449,2981221157,2980716036,2980211087,2979706309,2979201702,2978697265,2978192999,2977688904,2977184980,2976681226,2976177643,2975674230,2975170987,2974667915,2974165012,2973662280,2973159717,2972657325,2972155102,2971653049,
		2971151165,2970649451,2970147906,2969646531,2969145324,2968644287,2968143420,2967642721,2967142190,2966641829,2966141637,2965641613,2965141758,2964642071,2964142552,2963643202,2963144020,2962645006,2962146161,2961647483,2961148973,2960650631,2960152456,2959654449,2959156610,2958658938,2958161434,2957664097,2957166927,2956669924,2956173088,2955676419,
		2955179917,2954683582,2954187414,2953691412,2953195576,2952699907,2952204405,2951709068,2951213898,2950718894,2950224056,2949729384,2949234878,2948740538,2948246363,2947752354,2947258510,2946764832,2946271319,2945777972,2945284789,2944791772,2944298920,2943806233,2943313710,2942821353,2942329160,2941837132,2941345268,2940853569,2940362034,2939870663,
		2939379457,2938888414,2938397536,2937906822,2937416271,2936925884,2936435661,2935945602,2935455706,2934965974,2934476405,2933986999,2933497757,2933008677,2932519761,2932031008,2931542417,2931053990,2930565725,2930077623,2929589683,2929101906,2928614291,2928126839,2927639549,2927152421,2926665455,2926178651,2925692009,2925205528,2924719210,2924233053,
		2923747058,2923261224,2922775552,2922290041,2921804691,2921319503,2920834476,2920349609,2919864904,2919380360,2918895976,2918411753,2917927691,2917443789,2916960048,2916476467,2915993046,2915509786,2915026686,2914543745,2914060965,2913578345,2913095885,2912613584,2912131443,2911649462,2911167640,2910685977,2910204474,2909723131,2909241946,2908760921,
		2908280054,2907799347,2907318798,2906838408,2906358177,2905878105,2905398191,2904918436,2904438839,2903959400,2903480120,2903000998,2902522034,2902043228,2901564580,2901086090,2900607757,2900129583,2899651565,2899173706,2898696004,2898218459,2897741072,2897263842,2896786769,2896309853,2895833094,2895356492,2894880047,2894403759,2893927627,2893451653,
		2892975834,2892500172,2892024667,2891549317,2891074124,2890599088,2890124207,2889649482,2889174913,2888700500,2888226243,2887752142,2887278196,2886804406,2886330771,2885857291,2885383967,2884910798,2884437784,2883964926,2883492222,2883019673,2882547280,2882075041,2881602956,2881131026,2880659251,2880187631,2879716164,2879244852,2878773695,2878302691,
		2877831842,2877361146,2876890605,2876420217,2875949983,2875479903,2875009977,2874540204,2874070584,2873601119,2873131806,2872662647,2872193640,2871724787,2871256087,2870787540,2870319146,2869850905,2869382816,2868914881,2868447097,2867979466,2867511988,2867044662,2866577489,2866110467,2865643598,2865176881,2864710316,2864243902,2863777641,2863311531,
		2862845573,2862379767,2861914112,2861448609,2860983257,2860518057,2860053007,2859588109,2859123362,2858658766,2858194321,2857730027,2857265884,2856801891,2856338050,2855874358,2855410818,2854947427,2854484187,2854021098,2853558159,2853095369,2852632730,2852170241,2851707902,2851245713,2850783673,2850321783,2849860043,2849398453,2848937012,2848475720,
		2848014578,2847553585,2847092741,2846632047,2846171501,2845711104,2845250857,2844790758,2844330808,2843871007,2843411354,2842951850,2842492494,2842033287,2841574228,2841115318,2840656555,2840197941,2839739475,2839281157,2838822987,2838364964,2837907089,2837449363,2836991783,2836534351,2836077067,2835619930,2835162941,2834706099,2834249403,2833792856,
		2833336455,2832880201,2832424094,2831968134,2831512321,2831056654,2830601134,2830145761,2829690534,2829235453,2828780519,2828325731,2827871089,2827416594,2826962245,2826508041,2826053984,2825600072,2825146306,2824692686,2824239211,2823785882,2823332699,2822879661,2822426768,2821974021,2821521419,2821068962,2820616650,2820164483,2819712462,2819260585,
		2818808852,2818357265,2817905822,2817454524,2817003370,2816552361,2816101496,2815650776,2815200200,2814749768,2814299480,2813849336,2813399336,2812949480,2812499768,2812050199,2811600775,2811151494,2810702356,2810253362,2809804512,2809355804,2808907241,2808458820,2808010542,2807562408,2807114416,2806666568,2806218862,2805771299,2805323879,2804876602,
		2804429467,2803982475,2803535625,2803088918,2802642353,2802195930,2801749649,2801303511,2800857514,2800411660,2799965947,2799520377,2799074948,2798629661,2798184515,2797739511,2797294649,2796849928,2796405349,2795960910,2795516613,2795072458,2794628443,2794184569,2793740837,2793297245,2792853794,2792410484,2791967314,2791524286,2791081397,2790638650,
		2790196042,2789753576,2789311249,2788869063,2788427017,2787985111,2787543345,2787101719,2786660232,2786218886,2785777680,2785336613,2784895686,2784454898,2784014250,2783573742,2783133373,2782693143,2782253052,2781813101,2781373288,2780933615,2780494081,2780054685,2779615429,2779176311,2778737332,2778298491,2777859790,2777421226,2776982802,2776544515,
		2776106367,2775668357,2775230486,2774792752,2774355157,2773917699,2773480380,2773043198,2772606154,2772169248,2771732480,2771295849,2770859355,2770423000,2769986781,2769550700,2769114756,2768678950,2768243281,2767807748,2767372353,2766937095,2766501973,2766066989,2765632141,2765197430,2764762855,2764328417,2763894116,2763459951,2763025922,2762592030,
		2762158274,2761724654,2761291171,2760857823,2760424611,2759991536,2759558596,2759125792,2758693123,2758260591,2757828194,2757395932,2756963806,2756531816,2756099960,2755668241,2755236656,2754805206,2754373892,2753942713,2753511668,2753080759,2752649984,2752219344,2751788839,2751358469,2750928233,2750498131,2750068164,2749638332,2749208634,2748779070,
		2748349640,2747920345,2747491183,2747062156,2746633263,2746204503,2745775878,2745347386,2744919028,2744490803,2744062712,2743634755,2743206931,2742779240,2742351683,2741924259,2741496969,2741069811,2740642787,2740215895,2739789137,2739362511,2738936019,2738509659,2738083432,2737657337,2737231375,2736805546,2736379849,2735954284,2735528852,2735103552,
		2734678385,2734253349,2733828446,2733403674,2732979035,2732554527,2732130152,2731705908,2731281796,2730857816,2730433967,2730010249,2729586664,2729163209,2728739886,2728316695,2727893634,2727470705,2727047907,2726625240,2726202704,2725780299,2725358024,2724935881,2724513868,2724091986,2723670235,2723248614,2722827124,2722405764,2721984535,2721563436,
		2721142467,2720721628,2720300920,2719880341,2719459893,2719039575,2718619386,2718199328,2717779399,2717359600,2716939930,2716520390,2716100980,2715681699,2715262548,2714843526,2714424633,2714005870,2713587235,2713168730,2712750354,2712332107,2711913989,2711496000,2711078140,2710660408,2710242805,2709825331,2709407985,2708990768,2708573679,2708156719,
		2707739887,2707323184,2706906608,2706490161,2706073842,2705657651,2705241588,2704825653,2704409846,2703994167,2703578615,2703163191,2702747895,2702332726,2701917685,2701502771,2701087985,2700673326,2700258795,2699844390,2699430113,2699015963,2698601940,2698188044,2697774275,2697360633,2696947118,2696533729,2696120467,2695707332,2695294323,2694881441,
		2694468686,2694056056,2693643553,2693231177,2692818927,2692406803,2691994805,2691582933,2691171187,2690759567,2690348073,2689936705,2689525462,2689114345,2688703354,2688292489,2687881749,2687471135,2687060646,2686650282,2686240044,2685829931,2685419943,2685010081,2684600343,2684190730,2683781243,2683371880,2682962643,2682553530,2682144541,2681735678,
		2681326939,2680918325,2680509835,2680101470,2679693229,2679285112,2678877120,2678469252,2678061508,2677653889,2677246393,2676839021,2676431774,2676024650,2675617650,2675210774,2674804021,2674397392,2673990887,2673584506,2673178248,2672772113,2672366102,2671960214,2671554449,2671148808,2670743289,2670337894,2669932622,2669527473,2669122447,2668717544,
		2668312763,2667908106,2667503571,2667099158,2666694869,2666290701,2665886657,2665482735,2665078935,2664675257,2664271702,2663868269,2663464958,2663061769,2662658703,2662255758,2661852935,2661450234,2661047655,2660645198,2660242862,2659840648,2659438556,2659036585,2658634736,2658233008,2657831402,2657429917,2657028553,2656627310,2656226189,2655825188,
		2655424309,2655023551,2654622913,2654222397,2653822001,2653421727,2653021573,2652621539,2652221626,2651821834,2651422162,2651022611,2650623180,2650223870,2649824680,2649425610,2649026660,2648627830,2648229121,2647830531,2647432061,2647033712,2646635482,2646237372,2645839382,2645441511,2645043760,2644646129,2644248617,2643851224,2643453952,2643056798,
		2642659764,2642262849,2641866053,2641469377,2641072819,2640676381,2640280061,2639883861,2639487779,2639091816,2638695972,2638300247,2637904641,2637509153,2637113783,2636718532,2636323400,2635928386,2635533490,2635138713,2634744054,2634349513,2633955090,2633560786,2633166599,2632772530,2632378580,2631984747,2631591032,2631197435,2630803955,2630410593,
		2630017349,2629624222,2629231213,2628838322,2628445547,2628052890,2627660351,2627267928,2626875623,2626483435,2626091364,2625699410,2625307573,2624915853,2624524250,2624132764,2623741394,2623350141,2622959005,2622567986,2622177083,2621786296,2621395626,2621005073,2620614635,2620224315,2619834110,2619444021,2619054049,2618664193,2618274453,2617884829,
		2617495320,2617105928,2616716651,2616327491,2615938446,2615549516,2615160703,2614772005,2614383422,2613994955,2613606603,2613218367,2612830246,2612442241,2612054350,2611666575,2611278915,2610891370,2610503939,2610116624,2609729424,2609342339,2608955368,2608568512,2608181771,2607795145,2607408633,2607022236,2606635953,2606249785,2605863731,2605477791,
		2605091966,2604706255,2604320658,2603935176,2603549807,2603164553,2602779412,2602394386,2602009473,2601624674,2601239989,2600855418,2600470961,2600086617,2599702386,2599318269,2598934266,2598550376,2598166600,2597782937,2597399387,2597015950,2596632627,2596249417,2595866320,2595483335,2595100464,2594717706,2594335061,2593952528,2593570109,2593187802,
		2592805608,2592423526,2592041557,2591659701,2591277957,2590896325,2590514806,2590133399,2589752105,2589370923,2588989853,2588608895,2588228049,2587847315,2587466693,2587086184,2586705786,2586325500,2585945325,2585565263,2585185312,2584805473,2584425745,2584046129,2583666625,2583287232,2582907950,2582528780,2582149721,2581770773,2581391937,2581013211,
		2580634597,2580256094,2579877702,2579499421,2579121250,2578743191,2578365242,2577987404,2577609677,2577232061,2576854555,2576477160,2576099875,2575722701,2575345637,2574968684,2574591841,2574215108,2573838485,2573461973,2573085571,2572709279,2572333097,2571957025,2571581062,2571205210,2570829468,2570453835,2570078312,2569702899,2569327596,2568952402,
		2568577318,2568202343,2567827477,2567452722,2567078075,2566703538,2566329110,2565954791,2565580582,2565206481,2564832490,2564458608,2564084834,2563711170,2563337614,2562964168,2562590830,2562217601,2561844481,2561471469,2561098566,2560725771,2560353085,2559980508,2559608038,2559235678,2558863425,2558491281,2558119245,2557747317,2557375498,2557003786,
		2556632183,2556260687,2555889299,2555518020,2555146848,2554775784,2554404828,2554033979,2553663238,2553292605,2552922079,2552551661,2552181350,2551811147,2551441051,2551071063,2550701181,2550331407,2549961741,2549592181,2549222728,2548853383,2548484144,2548115013,2547745988,2547377070,2547008259,2546639555,2546270958,2545902467,2545534083,2545165806,
		2544797635,2544429570,2544061612,2543693761,2543326015,2542958376,2542590844,2542223417,2541856097,2541488883,2541121775,2540754773,2540387877,2540021087,2539654403,2539287824,2538921352,2538554985,2538188724,2537822569,2537456519,2537090575,2536724737,2536359003,2535993376,2535627854,2535262437,2534897125,2534531919,2534166818,2533801822,2533436931,
		2533072145,2532707464,2532342889,2531978418,2531614052,2531249791,2530885635,2530521583,2530157637,2529793795,2529430057,2529066424,2528702896,2528339472,2527976153,2527612938,2527249827,2526886821,2526523919,2526161121,2525798428,2525435838,2525073353,2524710971,2524348694,2523986521,2523624451,2523262485,2522900624,2522538865,2522177211,2521815661,
		2521454214,2521092870,2520731630,2520370494,2520009461,2519648532,2519287706,2518926983,2518566363,2518205847,2517845434,2517485124,2517124917,2516764814,2516404813,2516044915,2515685121,2515325429,2514965840,2514606353,2514246970,2513887689,2513528511,2513169435,2512810462,2512451592,2512092824,2511734159,2511375596,2511017135,2510658777,2510300521,
		2509942367,2509584315,2509226365,2508868518,2508510773,2508153129,2507795588,2507438148,2507080811,2506723575,2506366441,2506009409,2505652478,2505295649,2504938922,2504582296,2504225772,2503869349,2503513028,2503156808,2502800690,2502444673,2502088757,2501732942,2501377228,2501021616,2500666105,2500310695,2499955386,2499600177,2499245070,2498890064,
		2498535158,2498180353,2497825649,2497471046,2497116543,2496762141,2496407840,2496053639,2495699539,2495345539,2494991639,2494637840,2494284141,2493930543,2493577044,2493223646,2492870348,2492517150,2492164053,2491811055,2491458157,2491105359,2490752661,2490400063,2490047565,2489695167,2489342868,2488990669,2488638570,2488286570,2487934669,2487582869,
		2487231168,2486879566,2486528063,2486176660,2485825357,2485474152,2485123047,2484772041,2484421134,2484070326,2483719617,2483369007,2483018497,2482668085,2482317772,2481967558,2481617442,2481267426,2480917508,2480567689,2480217968,2479868346,2479518823,2479169398,2478820072,2478470844,2478121714,2477772683,2477423750,2477074915,2476726179,2476377541,
		2476029000,2475680558,2475332214,2474983968,2474635821,2474287770,2473939818,2473591964,2473244208,2472896549,2472548988,2472201524,2471854159,2471506891,2471159720,2470812647,2470465672,2470118794,2469772013,2469425330,2469078744,2468732255,2468385863,2468039569,2467693372,2467347272,2467001269,2466655363,2466309554,2465963842,2465618227,2465272709,
		2464927287,2464581962,2464236735,2463891603,2463546569,2463201631,2462856790,2462512045,2462167396,2461822845,2461478389,2461134030,2460789768,2460445601,2460101531,2459757557,2459413679,2459069898,2458726212,2458382623,2458039129,2457695732,2457352430,2457009225,2456666115,2456323101,2455980183,2455637360,2455294633,2454952002,2454609467,2454267027,
		2453924682,2453582433,2453240280,2452898222,2452556259,2452214392,2451872620,2451530943,2451189362,2450847875,2450506484,2450165188,2449823987,2449482881,2449141869,2448800953,2448460132,2448119406,2447778774,2447438237,2447097795,2446757448,2446417195,2446077037,2445736973,2445397004,2445057130,2444717350,2444377664,2444038073,2443698576,2443359173,
		2443019865,2442680651,2442341531,2442002505,2441663574,2441324736,2440985993,2440647343,2440308787,2439970326,2439631958,2439293684,2438955504,2438617417,2438279425,2437941526,2437603720,2437266008,2436928390,2436590865,2436253434,2435916096,2435578852,2435241701,2434904643,2434567679,2434230808,2433894030,2433557345,2433220754,2432884255,2432547850,
		2432211537,2431875318,2431539191,2431203158,2430867217,2430531369,2430195614,2429859951,2429524382,2429188905,2428853520,2428518229,2428183029,2427847923,2427512909,2427177987,2426843157,2426508420,2426173776,2425839223,2425504763,2425170395,2424836120,2424501936,2424167845,2423833845,2423499938,2423166122,2422832399,2422498767,2422165228,2421831780,
		2421498424,2421165159,2420831987,2420498906,2420165917,2419833019,2419500213,2419167498,2418834875,2418502344,2418169904,2417837555,2417505297,2417173131,2416841056,2416509073,2416177180,2415845379,2415513669,2415182049,2414850521,2414519084,2414187738,2413856483,2413525319,2413194245,2412863263,2412532371,2412201570,2411870859,2411540240,2411209711,
		2410879272,2410548924,2410218667,2409888500,2409558423,2409228437,2408898541,2408568736,2408239021,2407909396,2407579862,2407250417,2406921063,2406591799,2406262625,2405933540,2405604546,2405275642,2404946828,2404618104,2404289469,2403960925,2403632470,2403304105,2402975830,2402647644,2402319548,2401991541,2401663624,2401335797,2401008059,2400680410,
		2400352851,2400025382,2399698001,2399370710,2399043509,2398716396,2398389373,2398062438,2397735593,2397408837,2397082170,2396755592,2396429103,2396102703,2395776392,2395450170,2395124036,2394797992,2394472036,2394146169,2393820390,2393494700,2393169099,2392843587,2392518162,2392192827,2391867580,2391542421,2391217351,2390892369,2390567475,2390242670,
		2389917952,2389593324,2389268783,2388944330,2388619966,2388295689,2387971501,2387647401,2387323388,2386999464,2386675627,2386351878,2386028218,2385704644,2385381159,2385057761,2384734451,2384411229,2384088094,2383765047,2383442088,2383119215,2382796431,2382473733,2382151124,2381828601,2381506166,2381183818,2380861557,2380539384,2380217298,2379895299,
		2379573387,2379251562,2378929824,2378608173,2378286609,2377965132,2377643742,2377322439,2377001223,2376680093,2376359050,2376038094,2375717225,2375396442,2375075746,2374755136,2374434613,2374114177,2373793826,2373473563,2373153386,2372833295,2372513290,2372193372,2371873540,2371553795,2371234135,2370914562,2370595075,2370275673,2369956358,2369637129,
		2369317986,2368998929,2368679958,2368361073,2368042273,2367723560,2367404932,2367086390,2366767933,2366449563,2366131277,2365813078,2365494964,2365176936,2364858993,2364541136,2364223364,2363905677,2363588076,2363270560,2362953129,2362635784,2362318524,2362001349,2361684259,2361367255,2361050335,2360733501,2360416751,2360100087,2359783507,2359467013,
		2359150603,2358834278,2358518038,2358201883,2357885813,2357569827,2357253926,2356938109,2356622378,2356306730,2355991168,2355675689,2355360296,2355044986,2354729761,2354414621,2354099565,2353784593,2353469705,2353154902,2352840183,2352525548,2352210997,2351896531,2351582148,2351267849,2350953635,2350639504,2350325457,2350011495,2349697616,2349383821,
		2349070109,2348756482,2348442938,2348129478,2347816101,2347502809,2347189599,2346876474,2346563432,2346250473,2345937598,2345624806,2345312098,2344999473,2344686932,2344374473,2344062098,2343749807,2343437598,2343125473,2342813430,2342501471,2342189595,2341877802,2341566092,2341254465,2340942921,2340631459,2340320081,2340008785,2339697573,2339386443,
		2339075395,2338764431,2338453549,2338142750,2337832033,2337521399,2337210847,2336900378,2336589992,2336279688,2335969466,2335659327,2335349270,2335039295,2334729403,2334419592,2334109864,2333800219,2333490655,2333181173,2332871774,2332562457,2332253221,2331944068,2331634996,2331326007,2331017099,2330708273,2330399529,2330090867,2329782287,2329473788,
		2329165371,2328857036,2328548782,2328240610,2327932519,2327624510,2327316583,2327008737,2326700972,2326393289,2326085687,2325778166,2325470727,2325163369,2324856092,2324548896,2324241782,2323934749,2323627797,2323320926,2323014136,2322707426,2322400798,2322094251,2321787785,2321481400,2321175095,2320868872,2320562729,2320256667,2319950686,2319644785,
		2319338965,2319033225,2318727567,2318421988,2318116491,2317811074,2317505737,2317200481,2316895305,2316590209,2316285194,2315980259,2315675405,2315370630,2315065936,2314761322,2314456788,2314152335,2313847961,2313543668,2313239454,2312935321,2312631267,2312327293,2312023400,2311719586,2311415852,2311112198,2310808623,2310505128,2310201713,2309898378,
		2309595123,2309291946,2308988850,2308685833,2308382896,2308080038,2307777259,2307474560,2307171941,2306869401,2306566940,2306264558,2305962256,2305660033,2305357889,2305055824,2304753838,2304451932,2304150105,2303848356,2303546687,2303245097,2302943585,2302642153,2302340799,2302039525,2301738329,2301437212,2301136174,2300835214,2300534333,2300233531,
		2299932808,2299632163,2299331597,2299031109,2298730700,2298430370,2298130117,2297829944,2297529848,2297229831,2296929893,2296630032,2296330250,2296030547,2295730921,2295431374,2295131905,2294832513,2294533201,2294233966,2293934809,2293635730,2293336729,2293037806,2292738961,2292440194,2292141505,2291842893,2291544360,2291245904,2290947525,2290649225,
		2290351002,2290052857,2289754790,2289456800,2289158887,2288861052,2288563295,2288265615,2287968013,2287670487,2287373040,2287075669,2286778376,2286481160,2286184022,2285886961,2285589976,2285293069,2284996240,2284699487,2284402811,2284106212,2283809691,2283513246,2283216878,2282920588,2282624374,2282328237,2282032176,2281736193,2281440286,2281144456,
		2280848703,2280553027,2280257427,2279961904,2279666457,2279371087,2279075793,2278780576,2278485436,2278190371,2277895384,2277600472,2277305637,2277010879,2276716196,2276421590,2276127060,2275832607,2275538229,2275243928,2274949702,2274655553,2274361480,2274067483,2273773562,2273479717,2273185948,2272892254,2272598637,2272305095,2272011630,2271718240,
		2271424926,2271131687,2270838524,2270545437,2270252426,2269959490,2269666630,2269373845,2269081136,2268788502,2268495944,2268203462,2267911054,2267618722,2267326466,2267034285,2266742179,2266450148,2266158192,2265866312,2265574507,2265282777,2264991122,2264699543,2264408038,2264116609,2263825254,2263533974,2263242770,2262951640,2262660585,2262369605,
		2262078700,2261787869,2261497114,2261206433,2260915827,2260625295,2260334839,2260044456,2259754149,2259463916,2259173757,2258883673,2258593664,2258303729,2258013868,2257724082,2257434370,2257144733,2256855170,2256565681,2256276266,2255986926,2255697660,2255408468,2255119350,2254830306,2254541336,2254252441,2253963619,2253674872,2253386198,2253097598,
		2252809073,2252520621,2252232243,2251943939,2251655708,2251367552,2251079469,2250791460,2250503524,2250215662,2249927874,2249640160,2249352519,2249064951,2248777457,2248490037,2248202690,2247915416,2247628216,2247341089,2247054036,2246767056,2246480149,2246193316,2245906555,2245619868,2245333254,2245046714,2244760246,2244473852,2244187530,2243901282,
		2243615106,2243329004,2243042974,2242757018,2242471134,2242185324,2241899586,2241613921,2241328328,2241042809,2240757362,2240471988,2240186686,2239901458,2239616302,2239331218,2239046207,2238761269,2238476403,2238191609,2237906888,2237622240,2237337664,2237053160,2236768728,2236484369,2236200082,2235915868,2235631726,2235347655,2235063658,2234779732,
		2234495878,2234212097,2233928387,2233644750,2233361184,2233077691,2232794269,2232510920,2232227642,2231944436,2231661303,2231378241,2231095250,2230812332,2230529485,2230246710,2229964007,2229681375,2229398815,2229116326,2228833910,2228551564,2228269290,2227987088,2227704957,2227422898,2227140910,2226858993,2226577148,2226295374,2226013672,2225732041,
		2225450481,2225168992,2224887574,2224606228,2224324952,2224043748,2223762615,2223481553,2223200562,2222919642,2222638793,2222358015,2222077308,2221796672,2221516107,2221235612,2220955189,2220674836,2220394554,2220114342,2219834202,2219554132,2219274133,2218994204,2218714346,2218434558,2218154842,2217875195,2217595619,2217316114,2217036679,2216757315,
		2216478020,2216198797,2215919643,2215640560,2215361547,2215082605,2214803733,2214524931,2214246199,2213967537,2213688945,2213410424,2213131972,2212853591,2212575280,2212297038,2212018867,2211740765,2211462734,2211184772,2210906881,2210629059,2210351306,2210073624,2209796012,2209518469,2209240996,2208963592,2208686258,2208408994,2208131800,2207854675,
		2207577619,2207300633,2207023717,2206746870,2206470093,2206193385,2205916746,2205640177,2205363677,2205087246,2204810885,2204534593,2204258370,2203982216,2203706132,2203430116,2203154170,2202878293,2202602485,2202326746,2202051076,2201775475,2201499943,2201224481,2200949087,2200673761,2200398505,2200123318,2199848199,2199573149,2199298168,2199023256,
		2198748413,2198473638,2198198931,2197924294,2197649725,2197375225,2197100793,2196826430,2196552135,2196277909,2196003751,2195729662,2195455641,2195181688,2194907804,2194633988,2194360241,2194086561,2193812950,2193539408,2193265933,2192992527,2192719188,2192445918,2192172716,2191899582,2191626517,2191353519,2191080589,2190807727,2190534933,2190262207,
		2189989549,2189716959,2189444437,2189171982,2188899595,2188627276,2188355025,2188082842,2187810726,2187538678,2187266698,2186994785,2186722940,2186451162,2186179452,2185907809,2185636234,2185364727,2185093286,2184821914,2184550608,2184279370,2184008200,2183737096,2183466060,2183195092,2182924190,2182653356,2182382589,2182111889,2181841256,2181570691,
		2181300192,2181029761,2180759396,2180489099,2180218868,2179948705,2179678608,2179408579,2179138616,2178868720,2178598891,2178329129,2178059434,2177789805,2177520244,2177250749,2176981320,2176711958,2176442663,2176173435,2175904273,2175635178,2175366149,2175097187,2174828291,2174559462,2174290699,2174022003,2173753373,2173484810,2173216312,2172947881,
		2172679517,2172411219,2172142987,2171874821,2171606721,2171338688,2171070721,2170802819,2170534984,2170267215,2169999513,2169731876,2169464305,2169196800,2168929361,2168661988,2168394681,2168127440,2167860265,2167593155,2167326112,2167059134,2166792222,2166525375,2166258595,2165991880,2165725231,2165458647,2165192129,2164925677,2164659290,2164392969,
		2164126713,2163860523,2163594398,2163328338,2163062345,2162796416,2162530553,2162264755,2161999023,2161733356,2161467754,2161202217,2160936746,2160671340,2160405999,2160140723,2159875512,2159610367,2159345286,2159080271,2158815321,2158550435,2158285615,2158020860,2157756169,2157491544,2157226983,2156962488,2156698057,2156433691,2156169389,2155905153,
		2155640981,2155376874,2155112832,2154848855,2154584942,2154321093,2154057310,2153793591,2153529936,2153266346,2153002821,2152739360,2152475963,2152212631,2151949364,2151686161,2151423022,2151159947,2150896937,2150633991,2150371110,2150108292,2149845539,2149582851,2149320226,2149057665,2148795169,2148532737,2148270369,2148008065,2147745825,2147483648,
		4294443072,4293918976,4293395008,4292871168,4292347456,4291823871,4291300414,4290777085,4290253883,4289730809,4289207862,4288685043,4288162351,4287639787,4287117350,4286595041,4286072858,4285550803,4285028875,4284507074,4283985400,4283463854,4282942434,4282421141,4281899975,4281378936,4280858023,4280337238,4279816579,4279296046,4278775641,4278255361,
		4277735209,4277215183,4276695283,4276175510,4275655863,4275136342,4274616947,4274097679,4273578537,4273059521,4272540631,4272021867,4271503228,4270984716,4270466330,4269948070,4269429935,4268911926,4268394043,4267876285,4267358653,4266841146,4266323765,4265806510,4265289380,4264772375,4264255495,4263738741,4263222112,4262705609,4262189230,4261672976,
		4261156848,4260640844,4260124966,4259609212,4259093583,4258578080,4258062700,4257547446,4257032316,4256517311,4256002431,4255487675,4254973043,4254458536,4253944153,4253429895,4252915761,4252401752,4251887866,4251374105,4250860468,4250346955,4249833566,4249320301,4248807160,4248294143,4247781250,4247268481,4246755835,4246243313,4245730915,4245218641,
		4244706490,4244194462,4243682559,4243170778,4242659121,4242147588,4241636178,4241124891,4240613727,4240102687,4239591769,4239080975,4238570304,4238059756,4237549331,4237039029,4236528849,4236018793,4235508859,4234999048,4234489360,4233979795,4233470352,4232961031,4232451834,4231942758,4231433806,4230924975,4230416267,4229907681,4229399218,4228890877,
		4228382657,4227874561,4227366586,4226858733,4226351002,4225843393,4225335906,4224828541,4224321298,4223814177,4223307177,4222800299,4222293543,4221786908,4221280395,4220774003,4220267733,4219761585,4219255557,4218749651,4218243867,4217738203,4217232661,4216727240,4216221940,4215716762,4215211704,4214706767,4214201951,4213697257,4213192683,4212688230,
		4212183897,4211679686,4211175595,4210671624,4210167775,4209664046,4209160437,4208656949,4208153581,4207650334,4207147207,4206644201,4206141314,4205638548,4205135902,4204633376,4204130971,4203628685,4203126519,4202624474,4202122548,4201620742,4201119056,4200617490,4200116043,4199614716,4199113509,4198612422,4198111454,4197610605,4197109876,4196609267,
		4196108777,4195608406,4195108155,4194608023,4194108010,4193608116,4193108341,4192608686,4192109150,4191609732,4191110434,4190611255,4190112194,4189613252,4189114429,4188615725,4188117140,4187618673,4187120325,4186622096,4186123985,4185625993,4185128119,4184630363,4184132726,4183635207,4183137807,4182640525,4182143361,4181646315,4181149387,4180652578,
		4180155886,4179659313,4179162857,4178666519,4178170300,4177674198,4177178214,4176682347,4176186599,4175690968,4175195454,4174700059,4174204780,4173709620,4173214576,4172719651,4172224842,4171730151,4171235577,4170741121,4170246781,4169752559,4169258454,4168764466,4168270595,4167776841,4167283204,4166789684,4166296281,4165802995,4165309825,4164816772,
		4164323836,4163831017,4163338314,4162845728,4162353258,4161860905,4161368669,4160876548,4160384545,4159892657,4159400886,4158909231,4158417692,4157926270,4157434963,4156943773,4156452699,4155961740,4155470898,4154980172,4154489561,4153999066,4153508688,4153018425,4152528277,4152038246,4151548330,4151058529,4150568844,4150079275,4149589821,4149100483,
		4148611260,4148122152,4147633160,4147144283,4146655521,4146166874,4145678343,4145189926,4144701625,4144213439,4143725367,4143237411,4142749569,4142261843,4141774231,4141286734,4140799352,4140312084,4139824932,4139337893,4138850970,4138364161,4137877466,4137390886,4136904420,4136418069,4135931832,4135445709,4134959701,4134473806,4133988026,4133502361,
		4133016809,4132531371,4132046047,4131560838,4131075742,4130590760,4130105892,4129621138,4129136497,4128651971,4128167558,4127683258,4127199073,4126715000,4126231042,4125747197,4125263465,4124779847,4124296342,4123812950,4123329672,4122846507,4122363456,4121880517,4121397692,4120914979,4120432380,4119949894,4119467521,4118985260,4118503113,4118021078,
		4117539157,4117057348,4116575651,4116094068,4115612597,4115131239,4114649993,4114168860,4113687840,4113206932,4112726136,4112245453,4111764882,4111284423,4110804077,4110323843,4109843721,4109363711,4108883813,4108404028,4107924354,4107444793,4106965343,4106486005,4106006779,4105527665,4105048663,4104569773,4104090994,4103612327,4103133772,4102655328,
		4102176996,4101698775,4101220666,4100742668,4100264782,4099787007,4099309343,4098831791,4098354350,4097877020,4097399801,4096922694,4096445697,4095968812,4095492037,4095015374,4094538821,4094062380,4093586049,4093109829,4092633720,4092157722,4091681834,4091206057,4090730391,4090254835,4089779390,4089304055,4088828831,4088353718,4087878714,4087403821,
		4086929039,4086454366,4085979804,4085505352,4085031010,4084556779,4084082657,4083608646,4083134744,4082660953,4082187272,4081713700,4081240238,4080766886,4080293644,4079820512,4079347489,4078874576,4078401773,4077929079,4077456495,4076984020,4076511655,4076039399,4075567253,4075095216,4074623288,4074151470,4073679761,4073208161,4072736670,4072265289,
		4071794016,4071322853,4070851798,4070380853,4069910017,4069439289,4068968670,4068498161,4068027760,4067557467,4067087284,4066617209,4066147243,4065677385,4065207636,4064737996,4064268464,4063799041,4063329726,4062860519,4062391421,4061922431,4061453549,4060984775,4060516110,4060047553,4059579104,4059110763,4058642530,4058174405,4057706388,4057238479,
		4056770678,4056302985,4055835400,4055367922,4054900552,4054433290,4053966136,4053499089,4053032150,4052565318,4052098594,4051631978,4051165468,4050699067,4050232772,4049766585,4049300506,4048834533,4048368668,4047902910,4047437259,4046971715,4046506279,4046040949,4045575726,4045110611,4044645602,4044180700,4043715905,4043251217,4042786636,4042322161,
		4041857794,4041393532,4040929378,4040465330,4040001389,4039537554,4039073825,4038610204,4038146688,4037683279,4037219976,4036756780,4036293690,4035830706,4035367828,4034905057,4034442391,4033979832,4033517379,4033055031,4032592790,4032130655,4031668625,4031206702,4030744884,4030283172,4029821566,4029360066,4028898671,4028437382,4027976198,4027515121,
		4027054148,4026593281,4026132520,4025671864,4025211314,4024750869,4024290529,4023830294,4023370165,4022910141,4022450222,4021990408,4021530700,4021071096,4020611598,4020152204,4019692916,4019233732,4018774654,4018315680,4017856811,4017398047,4016939387,4016480833,4016022383,4015564037,4015105796,4014647660,4014189628,4013731701,4013273879,4012816160,
		4012358546,4011901037,4011443632,4010986331,4010529134,4010072042,4009615053,4009158169,4008701389,4008244713,4007788141,4007331673,4006875310,4006419050,4005962893,4005506841,4005050893,4004595048,4004139307,4003683670,4003228137,4002772707,4002317381,4001862158,4001407039,4000952023,4000497111,4000042303,3999587597,3999132995,3998678497,3998224102,
		3997769809,3997315621,3996861535,3996407553,3995953673,3995499897,3995046224,3994592654,3994139186,3993685822,3993232561,3992779402,3992326347,3991873394,3991420544,3990967796,3990515152,3990062610,3989610170,3989157834,3988705600,3988253468,3987801439,3987349512,3986897688,3986445966,3985994346,3985542829,3985091414,3984640101,3984188891,3983737782,
		3983286776,3982835872,3982385070,3981934370,3981483772,3981033276,3980582882,3980132590,3979682399,3979232311,3978782324,3978332439,3977882656,3977432975,3976983395,3976533917,3976084540,3975635265,3975186091,3974737019,3974288048,3973839179,3973390411,3972941745,3972493180,3972044716,3971596353,3971148092,3970699932,3970251872,3969803914,3969356057,
		3968908302,3968460647,3968013093,3967565640,3967118288,3966671036,3966223886,3965776836,3965329888,3964883040,3964436292,3963989645,3963543099,3963096654,3962650309,3962204065,3961757921,3961311877,3960865934,3960420092,3959974349,3959528707,3959083166,3958637724,3958192383,3957747142,3957302001,3956856961,3956412020,3955967179,3955522439,3955077798,
		3954633258,3954188817,3953744476,3953300235,3952856094,3952412053,3951968111,3951524269,3951080527,3950636884,3950193342,3949749898,3949306554,3948863310,3948420165,3947977120,3947534174,3947091327,3946648580,3946205932,3945763384,3945320934,3944878584,3944436333,3943994182,3943552129,3943110175,3942668321,3942226565,3941784909,3941343351,3940901892,
		3940460532,3940019271,3939578109,3939137046,3938696081,3938255215,3937814448,3937373779,3936933209,3936492738,3936052365,3935612091,3935171915,3934731838,3934291859,3933851978,3933412196,3932972512,3932532926,3932093439,3931654050,3931214759,3930775566,3930336472,3929897475,3929458576,3929019776,3928581073,3928142469,3927703962,3927265553,3926827243,
		3926389029,3925950914,3925512897,3925074977,3924637155,3924199430,3923761804,3923324274,3922886843,3922449509,3922012272,3921575133,3921138091,3920701147,3920264300,3919827551,3919390898,3918954343,3918517885,3918081525,3917645261,3917209095,3916773026,3916337054,3915901179,3915465401,3915029720,3914594136,3914158649,3913723258,3913287965,3912852768,
		3912417669,3911982665,3911547759,3911112949,3910678236,3910243620,3909809100,3909374677,3908940350,3908506120,3908071986,3907637949,3907204008,3906770164,3906336415,3905902763,3905469208,3905035748,3904602385,3904169118,3903735947,3903302873,3902869894,3902437011,3902004225,3901571534,3901138939,3900706441,3900274038,3899841731,3899409519,3898977404,
		3898545384,3898113460,3897681632,3897249900,3896818263,3896386721,3895955276,3895523925,3895092671,3894661511,3894230448,3893799479,3893368606,3892937828,3892507146,3892076559,3891646067,3891215671,3890785369,3890355163,3889925052,3889495036,3889065115,3888635289,3888205558,3887775922,3887346381,3886916935,3886487583,3886058327,3885629165,3885200099,
		3884771127,3884342249,3883913467,3883484779,3883056185,3882627686,3882199282,3881770972,3881342757,3880914636,3880486610,3880058678,3879630841,3879203097,3878775449,3878347894,3877920434,3877493067,3877065795,3876638618,3876211534,3875784544,3875357649,3874930847,3874504140,3874077526,3873651007,3873224581,3872798249,3872372011,3871945867,3871519817,
		3871093860,3870667997,3870242228,3869816552,3869390970,3868965482,3868540087,3868114786,3867689578,3867264464,3866839443,3866414516,3865989682,3865564941,3865140294,3864715740,3864291279,3863866911,3863442637,3863018456,3862594367,3862170372,3861746471,3861322662,3860898946,3860475323,3860051793,3859628356,3859205012,3858781761,3858358602,3857935537,
		3857512564,3857089684,3856666896,3856244201,3855821599,3855399090,3854976673,3854554349,3854132117,3853709977,3853287931,3852865976,3852444114,3852022344,3851600667,3851179082,3850757589,3850336189,3849914881,3849493664,3849072541,3848651509,3848230569,3847809722,3847388966,3846968302,3846547731,3846127251,3845706863,3845286568,3844866364,3844446251,
		3844026231,3843606303,3843186466,3842766721,3842347067,3841927505,3841508035,3841088656,3840669369,3840250174,3839831070,3839412057,3838993136,3838574307,3838155568,3837736921,3837318366,3836899901,3836481528,3836063246,3835645055,3835226956,3834808948,3834391030,3833973204,3833555469,3833137825,3832720272,3832302809,3831885438,3831468158,3831050968,
		3830633870,3830216862,3829799945,3829383119,3828966383,3828549738,3828133184,3827716720,3827300347,3826884065,3826467873,3826051772,3825635761,3825219841,3824804011,3824388271,3823972622,3823557063,3823141594,3822726216,3822310928,3821895730,3821480623,3821065605,3820650678,3820235841,3819821094,3819406437,3818991869,3818577392,3818163005,3817748708,
		3817334501,3816920383,3816506356,3816092418,3815678570,3815264812,3814851143,3814437564,3814024075,3813610676,3813197366,3812784145,3812371015,3811957973,3811545021,3811132159,3810719386,3810306703,3809894109,3809481604,3809069188,3808656862,3808244625,3807832478,3807420419,3807008450,3806596570,3806184779,3805773077,3805361464,3804949940,3804538505,
		3804127159,3803715902,3803304734,3802893655,3802482664,3802071763,3801660950,3801250226,3800839591,3800429044,3800018587,3799608217,3799197937,3798787745,3798377642,3797967627,3797557700,3797147862,3796738113,3796328452,3795918879,3795509395,3795099999,3794690692,3794281472,3793872341,3793463298,3793054344,3792645477,3792236699,3791828009,3791419407,
		3791010893,3790602466,3790194128,3789785878,3789377716,3788969642,3788561655,3788153757,3787745946,3787338223,3786930588,3786523041,3786115581,3785708209,3785300925,3784893728,3784486619,3784079597,3783672663,3783265817,3782859057,3782452386,3782045802,3781639305,3781232896,3780826574,3780420339,3780014191,3779608131,3779202158,3778796273,3778390474,
		3777984763,3777579138,3777173601,3776768151,3776362788,3775957512,3775552323,3775147220,3774742205,3774337277,3773932435,3773527681,3773123013,3772718432,3772313937,3771909530,3771505209,3771100975,3770696827,3770292766,3769888792,3769484904,3769081103,3768677388,3768273760,3767870218,3767466762,3767063393,3766660111,3766256914,3765853804,3765450781,
		3765047843,3764644992,3764242227,3763839548,3763436955,3763034449,3762632028,3762229693,3761827445,3761425283,3761023206,3760621216,3760219311,3759817492,3759415760,3759014113,3758612551,3758211076,3757809686,3757408382,3757007164,3756606032,3756204985,3755804024,3755403148,3755002358,3754601653,3754201034,3753800501,3753400053,3752999690,3752599413,
		3752199221,3751799114,3751399093,3750999157,3750599306,3750199541,3749799861,3749400266,3749000756,3748601331,3748201991,3747802737,3747403567,3747004483,3746605483,3746206569,3745807739,3745408994,3745010335,3744611760,3744213270,3743814864,3743416544,3743018308,3742620157,3742222090,3741824109,3741426212,3741028399,3740630671,3740233028,3739835469,
		3739437995,3739040605,3738643300,3738246079,3737848942,3737451890,3737054922,3736658039,3736261240,3735864525,3735467894,3735071348,3734674885,3734278507,3733882213,3733486003,3733089877,3732693836,3732297878,3731902004,3731506214,3731110508,3730714886,3730319348,3729923894,3729528524,3729133237,3728738035,3728342915,3727947880,3727552929,3727158061,
		3726763277,3726368576,3725973959,3725579425,3725184975,3724790609,3724396326,3724002127,3723608011,3723213978,3722820029,3722426163,3722032381,3721638681,3721245065,3720851533,3720458083,3720064717,3719671434,3719278234,3718885117,3718492083,3718099133,3717706265,3717313481,3716920779,3716528160,3716135625,3715743172,3715350802,3714958515,3714566311,
		3714174189,3713782150,3713390195,3712998321,3712606531,3712214823,3711823198,3711431655,3711040196,3710648818,3710257523,3709866311,3709475181,3709084134,3708693169,3708302286,3707911486,3707520769,3707130133,3706739580,3706349109,3705958721,3705568414,3705178190,3704788048,3704397988,3704008011,3703618115,3703228302,3702838570,3702448921,3702059353,
		3701669868,3701280464,3700891143,3700501903,3700112745,3699723669,3699334675,3698945763,3698556932,3698168183,3697779516,3697390930,3697002427,3696614004,3696225664,3695837405,3695449227,3695061131,3694673117,3694285184,3693897333,3693509563,3693121874,3692734267,3692346741,3691959296,3691571933,3691184651,3690797450,3690410331,3690023292,3689636335,
		3689249459,3688862664,3688475951,3688089318,3687702766,3687316296,3686929906,3686543597,3686157370,3685771223,3685385157,3684999172,3684613268,3684227444,3683841702,3683456040,3683070459,3682684959,3682299539,3681914200,3681528941,3681143764,3680758667,3680373650,3679988714,3679603858,3679219083,3678834389,3678449775,3678065241,3677680787,3677296414,
		3676912122,3676527909,3676143777,3675759726,3675375754,3674991863,3674608052,3674224321,3673840670,3673457099,3673073608,3672690198,3672306867,3671923617,3671540446,3671157355,3670774345,3670391414,3670008563,3669625792,3669243101,3668860489,3668477958,3668095506,3667713134,3667330842,3666948629,3666566496,3666184442,3665802469,3665420574,3665038760,
		3664657025,3664275369,3663893793,3663512296,3663130879,3662749541,3662368283,3661987104,3661606004,3661224984,3660844043,3660463181,3660082398,3659701695,3659321071,3658940526,3658560060,3658179673,3657799365,3657419137,3657038987,3656658916,3656278925,3655899012,3655519179,3655139424,3654759748,3654380151,3654000633,3653621194,3653241833,3652862551,
		3652483348,3652104224,3651725178,3651346211,3650967323,3650588514,3650209783,3649831130,3649452556,3649074061,3648695644,3648317305,3647939045,3647560864,3647182761,3646804736,3646426790,3646048922,3645671132,3645293420,3644915787,3644538232,3644160755,3643783357,3643406036,3643028794,3642651630,3642274544,3641897536,3641520606,3641143754,3640766980,
		3640390284,3640013666,3639637126,3639260663,3638884279,3638507972,3638131744,3637755593,3637379520,3637003524,3636627607,3636251767,3635876004,3635500320,3635124713,3634749183,3634373731,3633998357,3633623060,3633247841,3632872699,3632497635,3632122648,3631747739,3631372907,3630998152,3630623475,3630248875,3629874352,3629499907,3629125538,3628751247,
		3628377033,3628002897,3627628837,3627254855,3626880950,3626507122,3626133370,3625759696,3625386099,3625012579,3624639136,3624265770,3623892481,3623519268,3623146133,3622773074,3622400092,3622027187,3621654359,3621281607,3620908932,3620536334,3620163813,3619791368,3619418999,3619046708,3618674493,3618302354,3617930292,3617558307,3617186398,3616814566,
		3616442810,3616071130,3615699527,3615328000,3614956549,3614585175,3614213877,3613842655,3613471510,3613100441,3612729448,3612358531,3611987691,3611616926,3611246238,3610875625,3610505089,3610134629,3609764245,3609393937,3609023704,3608653548,3608283468,3607913463,3607543535,3607173682,3606803905,3606434204,3606064579,3605695029,3605325555,3604956157,
		3604586835,3604217588,3603848417,3603479321,3603110301,3602741357,3602372488,3602003695,3601634977,3601266335,3600897768,3600529277,3600160861,3599792520,3599424255,3599056065,3598687951,3598319911,3597951947,3597584059,3597216245,3596848507,3596480844,3596113256,3595745743,3595378305,3595010942,3594643655,3594276442,3593909305,3593542242,3593175255,
		3592808342,3592441504,3592074742,3591708054,3591341441,3590974902,3590608439,3590242050,3589875737,3589509497,3589143333,3588777243,3588411228,3588045288,3587679422,3587313631,3586947915,3586582273,3586216705,3585851212,3585485794,3585120450,3584755180,3584389985,3584024864,3583659818,3583294846,3582929948,3582565125,3582200376,3581835701,3581471101,
		3581106574,3580742122,3580377744,3580013441,3579649211,3579285055,3578920974,3578556966,3578193033,3577829174,3577465388,3577101677,3576738039,3576374476,3576010986,3575647571,3575284229,3574920961,3574557766,3574194646,3573831599,3573468626,3573105727,3572742901,3572380150,3572017471,3571654867,3571292336,3570929879,3570567495,3570205185,3569842948,
		3569480785,3569118695,3568756679,3568394736,3568032866,3567671070,3567309347,3566947698,3566586122,3566224619,3565863190,3565501834,3565140551,3564779341,3564418204,3564057141,3563696150,3563335233,3562974389,3562613618,3562252920,3561892295,3561531743,3561171265,3560810859,3560450526,3560090265,3559730078,3559369964,3559009922,3558649954,3558290058,
		3557930235,3557570485,3557210807,3556851202,3556491670,3556132211,3555772824,3555413510,3555054268,3554695099,3554336003,3553976979,3553618028,3553259149,3552900343,3552541609,3552182947,3551824358,3551465842,3551107397,3550749026,3550390726,3550032499,3549674344,3549316261,3548958250,3548600312,3548242446,3547884652,3547526930,3547169281,3546811703,
		3546454198,3546096764,3545739403,3545382113,3545024896,3544667751,3544310677,3543953676,3543596746,3543239889,3542883103,3542526389,3542169747,3541813176,3541456678,3541100251,3540743896,3540387613,3540031401,3539675261,3539319193,3538963196,3538607271,3538251417,3537895635,3537539925,3537184286,3536828719,3536473223,3536117798,3535762445,3535407164,
		3535051954,3534696815,3534341747,3533986751,3533631826,3533276973,3532922190,3532567479,3532212839,3531858271,3531503773,3531149347,3530794992,3530440708,3530086495,3529732353,3529378282,3529024282,3528670353,3528316496,3527962709,3527608993,3527255348,3526901774,3526548270,3526194838,3525841476,3525488186,3525134966,3524781817,3524428738,3524075731,
		3523722794,3523369927,3523017132,3522664407,3522311752,3521959169,3521606655,3521254213,3520901841,3520549539,3520197308,3519845147,3519493057,3519141038,3518789088,3518437209,3518085401,3517733663,3517381995,3517030397,3516678870,3516327413,3515976026,3515624710,3515273463,3514922287,3514571181,3514220145,3513869179,3513518284,3513167458,3512816703,
		3512466017,3512115402,3511764856,3511414381,3511063975,3510713639,3510363374,3510013178,3509663052,3509312996,3508963009,3508613093,3508263246,3507913469,3507563762,3507214124,3506864556,3506515058,3506165630,3505816271,3505466982,3505117762,3504768612,3504419531,3504070520,3503721579,3503372707,3503023904,3502675171,3502326507,3501977913,3501629388,
		3501280933,3500932547,3500584230,3500235982,3499887804,3499539695,3499191655,3498843685,3498495783,3498147951,3497800188,3497452494,3497104870,3496757314,3496409827,3496062410,3495715061,3495367782,3495020572,3494673430,3494326358,3493979354,3493632419,3493285553,3492938756,3492592028,3492245369,3491898779,3491552257,3491205804,3490859420,3490513105,
		3490166858,3489820680,3489474570,3489128530,3488782558,3488436654,3488090819,3487745053,3487399355,3487053726,3486708165,3486362673,3486017249,3485671894,3485326607,3484981388,3484636238,3484291156,3483946143,3483601197,3483256321,3482911512,3482566772,3482222100,3481877496,3481532960,3481188493,3480844093,3480499762,3480155499,3479811304,3479467177,
		3479123118,3478779127,3478435205,3478091350,3477747563,3477403844,3477060193,3476716610,3476373095,3476029648,3475686268,3475342957,3474999713,3474656537,3474313429,3473970388,3473627416,3473284511,3472941673,3472598904,3472256202,3471913568,3471571001,3471228502,3470886070,3470543706,3470201410,3469859181,3469517019,3469174925,3468832899,3468490940,
		3468149048,3467807224,3467465467,3467123777,3466782155,3466440600,3466099113,3465757692,3465416339,3465075054,3464733835,3464392684,3464051599,3463710582,3463369632,3463028749,3462687934,3462347185,3462006503,3461665889,3461325341,3460984861,3460644447,3460304101,3459963821,3459623608,3459283462,3458943383,3458603371,3458263426,3457923547,3457583736,
		3457243991,3456904313,3456564701,3456225157,3455885679,3455546267,3455206923,3454867645,3454528433,3454189289,3453850210,3453511199,3453172254,3452833375,3452494563,3452155818,3451817139,3451478526,3451139980,3450801500,3450463087,3450124740,3449786459,3449448245,3449110096,3448772015,3448433999,3448096050,3447758167,3447420350,3447082600,3446744915,
		3446407297,3446069745,3445732259,3445394839,3445057485,3444720197,3444382975,3444045820,3443708730,3443371706,3443034748,3442697857,3442361031,3442024271,3441687576,3441350948,3441014386,3440677889,3440341458,3440005093,3439668794,3439332561,3438996393,3438660291,3438324254,3437988284,3437652379,3437316539,3436980765,3436645057,3436309414,3435973837,
		3435638326,3435302880,3434967499,3434632184,3434296935,3433961750,3433626632,3433291578,3432956590,3432621668,3432286810,3431952019,3431617292,3431282631,3430948035,3430613504,3430279038,3429944638,3429610303,3429276033,3428941828,3428607688,3428273613,3427939604,3427605659,3427271780,3426937966,3426604216,3426270532,3425936913,3425603358,3425269869,
		3424936444,3424603085,3424269790,3423936560,3423603395,3423270295,3422937260,3422604289,3422271384,3421938542,3421605766,3421273055,3420940408,3420607826,3420275308,3419942855,3419610467,3419278143,3418945884,3418613690,3418281560,3417949494,3417617493,3417285557,3416953685,3416621877,3416290134,3415958456,3415626841,3415295292,3414963806,3414632385,
		3414301028,3413969735,3413638507,3413307343,3412976243,3412645208,3412314237,3411983330,3411652487,3411321708,3410990993,3410660343,3410329756,3409999234,3409668775,3409338381,3409008051,3408677785,3408347582,3408017444,3407687370,3407357359,3407027413,3406697530,3406367712,3406037957,3405708266,3405378639,3405049075,3404719576,3404390140,3404060768,
		3403731459,3403402215,3403073034,3402743916,3402414863,3402085873,3401756946,3401428083,3401099284,3400770548,3400441876,3400113268,3399784723,3399456241,3399127823,3398799468,3398471177,3398142949,3397814785,3397486683,3397158646,3396830671,3396502760,3396174913,3395847128,3395519407,3395191749,3394864154,3394536623,3394209154,3393881749,3393554407,
		3393227128,3392899913,3392572760,3392245670,3391918644,3391591681,3391264780,3390937943,3390611168,3390284457,3389957808,3389631223,3389304700,3388978241,3388651844,3388325510,3387999239,3387673030,3387346885,3387020802,3386694782,3386368825,3386042931,3385717099,3385391330,3385065624,3384739980,3384414399,3384088881,3383763425,3383438032,3383112701,
		3382787433,3382462228,3382137085,3381812004,3381486986,3381162031,3380837138,3380512307,3380187539,3379862833,3379538190,3379213609,3378889090,3378564634,3378240239,3377915908,3377591638,3377267431,3376943286,3376619203,3376295182,3375971224,3375647327,3375323493,3374999721,3374676011,3374352363,3374028778,3373705254,3373381792,3373058393,3372735055,
		3372411779,3372088566,3371765414,3371442324,3371119296,3370796330,3370473426,3370150584,3369827803,3369505085,3369182428,3368859833,3368537300,3368214828,3367892418,3367570070,3367247784,3366925559,3366603396,3366281295,3365959255,3365637277,3365315361,3364993506,3364671712,3364349980,3364028310,3363706701,3363385154,3363063668,3362742244,3362420881,
		3362099579,3361778339,3361457160,3361136043,3360814987,3360493992,3360173058,3359852186,3359531375,3359210626,3358889937,3358569310,3358248744,3357928240,3357607796,3357287414,3356967092,3356646832,3356326633,3356006495,3355686418,3355366402,3355046447,3354726554,3354406721,3354086949,3353767238,3353447588,3353127999,3352808471,3352489004,3352169597,
		3351850252,3351530967,3351211743,3350892580,3350573478,3350254437,3349935456,3349616536,3349297677,3348978878,3348660140,3348341463,3348022847,3347704291,3347385795,3347067361,3346748986,3346430673,3346112420,3345794227,3345476095,3345158024,3344840013,3344522062,3344204172,3343886342,3343568573,3343250864,3342933216,3342615627,3342298100,3341980632,
		3341663225,3341345878,3341028591,3340711365,3340394199,3340077093,3339760047,3339443061,3339126136,3338809271,3338492466,3338175721,3337859036,3337542411,3337225846,3336909341,3336592897,3336276512,3335960187,3335643922,3335327718,3335011573,3334695488,3334379463,3334063498,3333747593,3333431747,3333115962,3332800236,3332484570,3332168964,3331853418,
		3331537931,3331222505,3330907138,3330591830,3330276583,3329961394,3329646266,3329331197,3329016188,3328701239,3328386349,3328071519,3327756748,3327442037,3327127385,3326812793,3326498260,3326183787,3325869373,3325555018,3325240723,3324926488,3324612312,3324298195,3323984137,3323670139,3323356200,3323042321,3322728501,3322414740,3322101038,3321787396,
		3321473812,3321160288,3320846823,3320533418,3320220071,3319906784,3319593555,3319280386,3318967276,3318654225,3318341233,3318028300,3317715426,3317402611,3317089855,3316777158,3316464520,3316151941,3315839421,3315526959,3315214557,3314902214,3314589929,3314277703,3313965536,3313653428,3313341378,3313029387,3312717455,3312405582,3312093768,3311782012,
		3311470315,3311158676,3310847096,3310535575,3310224113,3309912709,3309601363,3309290077,3308978848,3308667679,3308356567,3308045515,3307734520,3307423585,3307112707,3306801889,3306491128,3306180426,3305869782,3305559197,3305248670,3304938202,3304627791,3304317439,3304007146,3303696910,3303386733,3303076614,3302766553,3302456551,3302146607,3301836721,
		3301526893,3301217123,3300907411,3300597757,3300288162,3299978624,3299669145,3299359724,3299050360,3298741055,3298431808,3298122619,3297813487,3297504414,3297195398,3296886441,3296577541,3296268699,3295959915,3295651189,3295342521,3295033910,3294725358,3294416863,3294108426,3293800046,3293491725,3293183461,3292875254,3292567106,3292259015,3291950982,
		3291643006,3291335088,3291027228,3290719425,3290411680,3290103992,3289796362,3289488790,3289181275,3288873817,3288566417,3288259074,3287951789,3287644561,3287337391,3287030278,3286723222,3286416224,3286109283,3285802400,3285495573,3285188804,3284882093,3284575438,3284268841,3283962301,3283655818,3283349393,3283043025,3282736713,3282430459,3282124263,
		3281818123,3281512040,3281206015,3280900046,3280594135,3280288280,3279982483,3279676743,3279371059,3279065433,3278759863,3278454351,3278148895,3277843497,3277538155,3277232870,3276927642,3276622471,3276317357,3276012299,3275707299,3275402355,3275097468,3274792637,3274487864,3274183147,3273878486,3273573883,3273269336,3272964846,3272660412,3272356036,
		3272051715,3271747452,3271443244,3271139094,3270835000,3270530962,3270226982,3269923057,3269619189,3269315378,3269011623,3268707924,3268404282,3268100696,3267797167,3267493694,3267190277,3266886917,3266583613,3266280365,3265977174,3265674039,3265370960,3265067937,3264764971,3264462061,3264159207,3263856409,3263553668,3263250983,3262948353,3262645780,
		3262343263,3262040802,3261738398,3261436049,3261133756,3260831519,3260529339,3260227214,3259925145,3259623133,3259321176,3259019275,3258717430,3258415641,3258113908,3257812231,3257510610,3257209044,3256907534,3256606081,3256304682,3256003340,3255702054,3255400823,3255099648,3254798529,3254497465,3254196457,3253895505,3253594608,3253293767,3252992982,
		3252692252,3252391578,3252090960,3251790397,3251489889,3251189438,3250889041,3250588701,3250288415,3249988185,3249688011,3249387892,3249087828,3248787820,3248487868,3248187970,3247888128,3247588342,3247288610,3246988935,3246689314,3246389749,3246090238,3245790784,3245491384,3245192040,3244892751,3244593517,3244294338,3243995214,3243696146,3243397133,
		3243098174,3242799271,3242500423,3242201631,3241902893,3241604210,3241305582,3241007009,3240708492,3240410029,3240111621,3239813268,3239514970,3239216728,3238918539,3238620406,3238322328,3238024305,3237726336,3237428422,3237130563,3236832759,3236535010,3236237316,3235939676,3235642091,3235344560,3235047085,3234749664,3234452298,3234154986,3233857729,
		3233560527,3233263379,3232966286,3232669248,3232372264,3232075335,3231778460,3231481640,3231184874,3230888163,3230591506,3230294904,3229998356,3229701863,3229405424,3229109039,3228812709,3228516434,3228220212,3227924045,3227627933,3227331874,3227035870,3226739921,3226444025,3226148184,3225852397,3225556664,3225260986,3224965362,3224669791,3224374276,
		3224078814,3223783406,3223488053,3223192753,3222897508,3222602317,3222307180,3222012097,3221717068,3221422093,3221127171,3220832304,3220537491,3220242732,3219948027,3219653376,3219358779,3219064236,3218769746,3218475310,3218180929,3217886601,3217592327,3217298107,3217003940,3216709828,3216415769,3216121764,3215827812,3215533915,3215240071,3214946281,
		3214652544,3214358861,3214065232,3213771656,3213478134,3213184666,3212891251,3212597890,3212304583,3212011329,3211718128,3211424981,3211131888,3210838848,3210545861,3210252928,3209960049,3209667223,3209374450,3209081731,3208789065,3208496452,3208203893,3207911387,3207618935,3207326535,3207034190,3206741897,3206449658,3206157472,3205865339,3205573259,
		3205281233,3204989260,3204697340,3204405473,3204113659,3203821899,3203530192,3203238537,3202946936,3202655388,3202363893,3202072451,3201781062,3201489727,3201198444,3200907214,3200616037,3200324913,3200033842,3199742824,3199451859,3199160947,3198870088,3198579281,3198288528,3197997827,3197707179,3197416584,3197126042,3196835553,3196545116,3196254732,
		3195964401,3195674123,3195383897,3195093725,3194803604,3194513537,3194223522,3193933560,3193643650,3193353793,3193063989,3192774237,3192484538,3192194892,3191905298,3191615756,3191326267,3191036831,3190747447,3190458115,3190168836,3189879610,3189590436,3189301314,3189012245,3188723228,3188434263,3188145351,3187856491,3187567684,3187278929,3186990226,
		3186701575,3186412977,3186124431,3185835938,3185547496,3185259107,3184970770,3184682485,3184394252,3184106072,3183817944,3183529867,3183241843,3182953871,3182665952,3182378084,3182090268,3181802504,3181514793,3181227133,3180939526,3180651970,3180364467,3180077015,3179789615,3179502268,3179214972,3178927728,3178640536,3178353396,3178066308,3177779272,
		3177492287,3177205354,3176918474,3176631644,3176344867,3176058142,3175771468,3175484846,3175198276,3174911757,3174625290,3174338875,3174052512,3173766200,3173479940,3173193731,3172907575,3172621469,3172335416,3172049413,3171763463,3171477564,3171191716,3170905921,3170620176,3170334483,3170048842,3169763252,3169477713,3169192226,3168906790,3168621406,
		3168336073,3168050792,3167765562,3167480383,3167195256,3166910180,3166625155,3166340181,3166055259,3165770388,3165485569,3165200800,3164916083,3164631417,3164346802,3164062239,3163777726,3163493265,3163208855,3162924496,3162640188,3162355932,3162071726,3161787571,3161503468,3161219415,3160935414,3160651464,3160367564,3160083716,3159799919,3159516172,
		3159232477,3158948832,3158665239,3158381696,3158098204,3157814763,3157531373,3157248034,3156964746,3156681509,3156398322,3156115186,3155832101,3155549067,3155266083,3154983151,3154700268,3154417437,3154134657,3153851927,3153569248,3153286619,3153004041,3152721514,3152439037,3152156611,3151874236,3151591911,3151309637,3151027413,3150745240,3150463117,
		3150181045,3149899024,3149617053,3149335132,3149053262,3148771442,3148489673,3148207954,3147926286,3147644668,3147363100,3147081583,3146800116,3146518699,3146237333,3145956017,3145674751,3145393536,3145112371,3144831256,3144550192,3144269177,3143988213,3143707299,3143426436,3143145622,3142864859,3142584146,3142303483,3142022870,3141742307,3141461794,
		3141181332,3140900919,3140620556,3140340244,3140059982,3139779769,3139499607,3139219495,3138939432,3138659420,3138379457,3138099545,3137819682,3137539869,3137260107,3136980394,3136700731,3136421117,3136141554,3135862041,3135582577,3135303163,3135023799,3134744485,3134465220,3134186005,3133906840,3133627725,3133348659,3133069643,3132790677,3132511761,
		3132232894,3131954076,3131675309,3131396591,3131117922,3130839304,3130560734,3130282215,3130003745,3129725324,3129446953,3129168632,3128890360,3128612137,3128333964,3128055841,3127777766,3127499742,3127221766,3126943841,3126665964,3126388137,3126110359,3125832631,3125554952,3125277322,3124999742,3124722211,3124444729,3124167297,3123889913,3123612579,
		3123335295,3123058059,3122780873,3122503736,3122226648,3121949609,3121672620,3121395679,3121118788,3120841946,3120565153,3120288409,3120011714,3119735068,3119458471,3119181923,3118905425,3118628975,3118352574,3118076222,3117799920,3117523666,3117247461,3116971305,3116695198,3116419140,3116143131,3115867171,3115591260,3115315397,3115039583,3114763819,
		3114488103,3114212435,3113936817,3113661247,3113385726,3113110254,3112834831,3112559456,3112284130,3112008853,3111733625,3111458445,3111183314,3110908231,3110633197,3110358212,3110083275,3109808387,3109533548,3109258757,3108984015,3108709321,3108434676,3108160079,3107885531,3107611031,3107336580,3107062177,3106787823,3106513517,3106239260,3105965051,
		3105690890,3105416778,3105142714,3104868699,3104594732,3104320813,3104046943,3103773121,3103499347,3103225621,3102951944,3102678315,3102404735,3102131202,3101857718,3101584282,3101310894,3101037555,3100764263,3100491020,3100217825,3099944678,3099671579,3099398528,3099125526,3098852571,3098579665,3098306806,3098033996,3097761234,3097488520,3097215853,
		3096943235,3096670665,3096398143,3096125668,3095853242,3095580863,3095308533,3095036250,3094764016,3094491829,3094219690,3093947599,3093675556,3093403560,3093131613,3092859713,3092587861,3092316057,3092044300,3091772592,3091500931,3091229318,3090957752,3090686235,3090414765,3090143342,3089871968,3089600641,3089329361,3089058129,3088786945,3088515809,
		3088244720,3087973679,3087702685,3087431739,3087160840,3086889989,3086619185,3086348429,3086077721,3085807060,3085536446,3085265880,3084995361,3084724890,3084454466,3084184090,3083913761,3083643479,3083373245,3083103058,3082832918,3082562826,3082292781,3082022783,3081752833,3081482930,3081213074,3080943266,3080673504,3080403790,3080134124,3079864504,
		3079594932,3079325406,3079055928,3078786498,3078517114,3078247777,3077978488,3077709245,3077440050,3077170902,3076901801,3076632747,3076363740,3076094780,3075825867,3075557001,3075288182,3075019410,3074750686,3074482008,3074213377,3073944792,3073676255,3073407765,3073139322,3072870925,3072602576,3072334273,3072066017,3071797808,3071529646,3071261531,
		3070993462,3070725440,3070457465,3070189537,3069921656,3069653821,3069386033,3069118292,3068850597,3068582949,3068315348,3068047793,3067780286,3067512824,3067245410,3066978042,3066710720,3066443446,3066176217,3065909036,3065641901,3065374812,3065107770,3064840775,3064573826,3064306923,3064040067,3063773258,3063506495,3063239778,3062973108,3062706485,
		3062439907,3062173376,3061906892,3061640454,3061374062,3061107717,3060841418,3060575165,3060308958,3060042798,3059776684,3059510617,3059244596,3058978621,3058712692,3058446809,3058180973,3057915183,3057649439,3057383741,3057118090,3056852484,3056586925,3056321412,3056055945,3055790524,3055525149,3055259821,3054994538,3054729301,3054464111,3054198967,
		3053933868,3053668816,3053403809,3053138849,3052873934,3052609066,3052344243,3052079467,3051814736,3051550052,3051285413,3051020820,3050756273,3050491772,3050227316,3049962907,3049698543,3049434226,3049169954,3048905727,3048641547,3048377412,3048113324,3047849281,3047585283,3047321332,3047057426,3046793566,3046529751,3046265982,3046002259,3045738582,
		3045474950,3045211364,3044947823,3044684328,3044420879,3044157475,3043894117,3043630804,3043367537,3043104315,3042841139,3042578009,3042314924,3042051884,3041788890,3041525942,3041263039,3041000181,3040737369,3040474602,3040211881,3039949205,3039686574,3039423989,3039161449,3038898954,3038636505,3038374101,3038111743,3037849430,3037587162,3037324939,
		3037062762,3036800630,3036538543,3036276501,3036014505,3035752553,3035490647,3035228787,3034966971,3034705201,3034443475,3034181795,3033920160,3033658570,3033397025,3033135525,3032874071,3032612661,3032351297,3032089977,3031828703,3031567473,3031306289,3031045149,3030784055,3030523006,3030262001,3030001042,3029740127,3029479257,3029218433,3028957653,
		3028696918,3028436228,3028175583,3027914982,3027654427,3027393916,3027133450,3026873029,3026612653,3026352322,3026092035,3025831793,3025571596,3025311444,3025051336,3024791274,3024531255,3024271282,3024011353,3023751469,3023491630,3023231835,3022972085,3022712379,3022452718,3022193102,3021933530,3021674003,3021414521,3021155083,3020895689,3020636341,
		3020377036,3020117776,3019858561,3019599390,3019340264,3019081182,3018822145,3018563152,3018304203,3018045299,3017786439,3017527624,3017268853,3017010126,3016751444,3016492806,3016234213,3015975664,3015717159,3015458699,3015200282,3014941910,3014683583,3014425299,3014167060,3013908865,3013650715,3013392608,3013134546,3012876528,3012618554,3012360625,
		3012102739,3011844898,3011587100,3011329347,3011071638,3010813974,3010556353,3010298776,3010041243,3009783755,3009526310,3009268910,3009011553,3008754241,3008496973,3008239748,3007982568,3007725431,3007468339,3007211290,3006954286,3006697325,3006440408,3006183535,3005926706,3005669921,3005413180,3005156482,3004899829,3004643219,3004386653,3004130131,
		3003873653,3003617218,3003360828,3003104481,3002848177,3002591918,3002335702,3002079530,3001823402,3001567317,3001311277,3001055279,3000799326,3000543416,3000287550,3000031727,2999775948,2999520213,2999264521,2999008873,2998753268,2998497707,2998242190,2997986716,2997731285,2997475898,2997220555,2996965255,2996709999,2996454786,2996199616,2995944490,
		2995689408,2995434369,2995179373,2994924421,2994669512,2994414646,2994159824,2993905045,2993650310,2993395618,2993140969,2992886364,2992631802,2992377283,2992122808,2991868375,2991613986,2991359641,2991105338,2990851079,2990596863,2990342690,2990088561,2989834474,2989580431,2989326431,2989072474,2988818561,2988564690,2988310862,2988057078,2987803337,
		2987549639,2987295984,2987042372,2986788803,2986535277,2986281794,2986028354,2985774957,2985521603,2985268292,2985015025,2984761800,2984508618,2984255479,2984002383,2983749330,2983496319,2983243352,2982990428,2982737546,2982484708,2982231912,2981979159,2981726449,2981473781,2981221157,2980968575,2980716036,2980463540,2980211087,2979958677,2979706309,
		2979453984,2979201702,2978949462,2978697265,2978445111,2978192999,2977940931,2977688904,2977436921,2977184980,2976933082,2976681226,2976429413,2976177643,2975925915,2975674230,2975422587,2975170987,2974919430,2974667915,2974416442,2974165012,2973913625,2973662280,2973410977,2973159717,2972908500,2972657325,2972406192,2972155102,2971904054,2971653049,
		2971402086,2971151165,2970900287,2970649451,2970398657,2970147906,2969897197,2969646531,2969395906,2969145324,2968894785,2968644287,2968393832,2968143420,2967893049,2967642721,2967392434,2967142190,2966891989,2966641829,2966391712,2966141637,2965891604,2965641613,2965391664,2965141758,2964891893,2964642071,2964392290,2964142552,2963892856,2963643202,
		2963393590,2963144020,2962894492,2962645006,2962395562,2962146161,2961896801,2961647483,2961398207,2961148973,2960899781,2960650631,2960401522,2960152456,2959903432,2959654449,2959405509,2959156610,2958907753,2958658938,2958410165,2958161434,2957912744,2957664097,2957415491,2957166927,2956918404,2956669924,2956421485,2956173088,2955924733,2955676419,
		2955428147,2955179917,2954931729,2954683582,2954435477,2954187414,2953939392,2953691412,2953443473,2953195576,2952947721,2952699907,2952452135,2952204405,2951956716,2951709068,2951461463,2951213898,2950966375,2950718894,2950471454,2950224056,2949976699,2949729384,2949482110,2949234878,2948987687,2948740538,2948493430,2948246363,2947999338,2947752354,
		2947505411,2947258510,2947011650,2946764832,2946518055,2946271319,2946024625,2945777972,2945531360,2945284789,2945038260,2944791772,2944545326,2944298920,2944052556,2943806233,2943559951,2943313710,2943067511,2942821353,2942575236,2942329160,2942083125,2941837132,2941591179,2941345268,2941099398,2940853569,2940607781,2940362034,2940116328,2939870663,
		2939625039,2939379457,2939133915,2938888414,2938642955,2938397536,2938152158,2937906822,2937661526,2937416271,2937171057,2936925884,2936680752,2936435661,2936190611,2935945602,2935700634,2935455706,2935210820,2934965974,2934721169,2934476405,2934231682,2933986999,2933742358,2933497757,2933253197,2933008677,2932764199,2932519761,2932275364,2932031008,
		2931786692,2931542417,2931298183,2931053990,2930809837,2930565725,2930321654,2930077623,2929833633,2929589683,2929345774,2929101906,2928858078,2928614291,2928370545,2928126839,2927883173,2927639549,2927395964,2927152421,2926908917,2926665455,2926422032,2926178651,2925935309,2925692009,2925448748,2925205528,2924962349,2924719210,2924476111,2924233053,
		2923990035,2923747058,2923504121,2923261224,2923018368,2922775552,2922532776,2922290041,2922047346,2921804691,2921562077,2921319503,2921076969,2920834476,2920592022,2920349609,2920107237,2919864904,2919622612,2919380360,2919138148,2918895976,2918653844,2918411753,2918169702,2917927691,2917685720,2917443789,2917201898,2916960048,2916718237,2916476467,
		2916234736,2915993046,2915751396,2915509786,2915268216,2915026686,2914785195,2914543745,2914302335,2914060965,2913819635,2913578345,2913337095,2913095885,2912854714,2912613584,2912372494,2912131443,2911890432,2911649462,2911408531,2911167640,2910926789,2910685977,2910445206,2910204474,2909963783,2909723131,2909482518,2909241946,2909001413,2908760921,
		2908520467,2908280054,2908039681,2907799347,2907559053,2907318798,2907078583,2906838408,2906598273,2906358177,2906118121,2905878105,2905638128,2905398191,2905158294,2904918436,2904678618,2904438839,2904199100,2903959400,2903719740,2903480120,2903240539,2903000998,2902761496,2902522034,2902282611,2902043228,2901803884,2901564580,2901325315,2901086090,
		2900846904,2900607757,2900368650,2900129583,2899890554,2899651565,2899412616,2899173706,2898934835,2898696004,2898457212,2898218459,2897979746,2897741072,2897502437,2897263842,2897025286,2896786769,2896548291,2896309853,2896071454,2895833094,2895594774,2895356492,2895118250,2894880047,2894641884,2894403759,2894165674,2893927627,2893689620,2893451653,
		2893213724,2892975834,2892737984,2892500172,2892262400,2892024667,2891786973,2891549317,2891311701,2891074124,2890836587,2890599088,2890361628,2890124207,2889886825,2889649482,2889412178,2889174913,2888937687,2888700500,2888463352,2888226243,2887989173,2887752142,2887515149,2887278196,2887041281,2886804406,2886567569,2886330771,2886094012,2885857291,
		2885620610,2885383967,2885147363,2884910798,2884674272,2884437784,2884201336,2883964926,2883728555,2883492222,2883255928,2883019673,2882783457,2882547280,2882311141,2882075041,2881838979,2881602956,2881366972,2881131026,2880895120,2880659251,2880423422,2880187631,2879951878,2879716164,2879480489,2879244852,2879009254,2878773695,2878538174,2878302691,
		2878067247,2877831842,2877596475,2877361146,2877125856,2876890605,2876655392,2876420217,2876185081,2875949983,2875714924,2875479903,2875244921,2875009977,2874775071,2874540204,2874305375,2874070584,2873835832,2873601119,2873366443,2873131806,2872897207,2872662647,2872428124,2872193640,2871959195,2871724787,2871490418,2871256087,2871021795,2870787540,
		2870553324,2870319146,2870085007,2869850905,2869616842,2869382816,2869148829,2868914881,2868680970,2868447097,2868213263,2867979466,2867745708,2867511988,2867278306,2867044662,2866811056,2866577489,2866343959,2866110467,2865877014,2865643598,2865410220,2865176881,2864943579,2864710316,2864477090,2864243902,2864010753,2863777641,2863544567,2863311531,
		2863078533,2862845573,2862612651,2862379767,2862146921,2861914112,2861681342,2861448609,2861215914,2860983257,2860750638,2860518057,2860285513,2860053007,2859820539,2859588109,2859355717,2859123362,2858891045,2858658766,2858426525,2858194321,2857962155,2857730027,2857497937,2857265884,2857033869,2856801891,2856569952,2856338050,2856106185,2855874358,
		2855642569,2855410818,2855179104,2854947427,2854715789,2854484187,2854252624,2854021098,2853789609,2853558159,2853326745,2853095369,2852864031,2852632730,2852401467,2852170241,2851939053,2851707902,2851476789,2851245713,2851014674,2850783673,2850552710,2850321783,2850090895,2849860043,2849629229,2849398453,2849167714,2848937012,2848706347,2848475720,
		2848245130,2848014578,2847784063,2847553585,2847323144,2847092741,2846862375,2846632047,2846401755,2846171501,2845941284,2845711104,2845480962,2845250857,2845020789,2844790758,2844560764,2844330808,2844100889,2843871007,2843641162,2843411354,2843181583,2842951850,2842722154,2842492494,2842262872,2842033287,2841803739,2841574228,2841344755,2841115318,
		2840885918,2840656555,2840427230,2840197941,2839968690,2839739475,2839510297,2839281157,2839052053,2838822987,2838593957,2838364964,2838136008,2837907089,2837678208,2837449363,2837220554,2836991783,2836763049,2836534351,2836305691,2836077067,2835848480,2835619930,2835391417,2835162941,2834934501,2834706099,2834477733,2834249403,2834021111,2833792856,
		2833564637,2833336455,2833108309,2832880201,2832652129,2832424094,2832196096,2831968134,2831740209,2831512321,2831284469,2831056654,2830828876,2830601134,2830373429,2830145761,2829918129,2829690534,2829462975,2829235453,2829007968,2828780519,2828553107,2828325731,2828098392,2827871089,2827643823,2827416594,2827189401,2826962245,2826735125,2826508041,
		2826280994,2826053984,2825827009,2825600072,2825373171,2825146306,2824919478,2824692686,2824465930,2824239211,2824012529,2823785882,2823559273,2823332699,2823106162,2822879661,2822653197,2822426768,2822200377,2821974021,2821747702,2821521419,2821295172,2821068962,2820842788,2820616650,2820390549,2820164483,2819938454,2819712462,2819486505,2819260585,
		2819034700,2818808852,2818583041,2818357265,2818131525,2817905822,2817680155,2817454524,2817228929,2817003370,2816777848,2816552361,2816326911,2816101496,2815876118,2815650776,2815425470,2815200200,2814974966,2814749768,2814524606,2814299480,2814074390,2813849336,2813624318,2813399336,2813174390,2812949480,2812724606,2812499768,2812274966,2812050199,
		2811825469,2811600775,2811376116,2811151494,2810926907,2810702356,2810477841,2810253362,2810028919,2809804512,2809580140,2809355804,2809131505,2808907241,2808683012,2808458820,2808234663,2808010542,2807786457,2807562408,2807338394,2807114416,2806890474,2806666568,2806442697,2806218862,2805995063,2805771299,2805547572,2805323879,2805100223,2804876602,
		2804653017,2804429467,2804205953,2803982475,2803759032,2803535625,2803312254,2803088918,2802865617,2802642353,2802419123,2802195930,2801972772,2801749649,2801526562,2801303511,2801080495,2800857514,2800634569,2800411660,2800188786,2799965947,2799743144,2799520377,2799297645,2799074948,2798852287,2798629661,2798407070,2798184515,2797961996,2797739511,
		2797517063,2797294649,2797072271,2796849928,2796627621,2796405349,2796183112,2795960910,2795738744,2795516613,2795294518,2795072458,2794850433,2794628443,2794406488,2794184569,2793962685,2793740837,2793519023,2793297245,2793075502,2792853794,2792632121,2792410484,2792188881,2791967314,2791745782,2791524286,2791302824,2791081397,2790860006,2790638650,
		2790417329,2790196042,2789974791,2789753576,2789532395,2789311249,2789090138,2788869063,2788648022,2788427017,2788206046,2787985111,2787764210,2787543345,2787322514,2787101719,2786880958,2786660232,2786439542,2786218886,2785998266,2785777680,2785557129,2785336613,2785116132,2784895686,2784675275,2784454898,2784234557,2784014250,2783793979,2783573742,
		2783353540,2783133373,2782913240,2782693143,2782473080,2782253052,2782033059,2781813101,2781593177,2781373288,2781153434,2780933615,2780713830,2780494081,2780274365,2780054685,2779835040,2779615429,2779395852,2779176311,2778956804,2778737332,2778517894,2778298491,2778079123,2777859790,2777640491,2777421226,2777201997,2776982802,2776763641,2776544515,
		2776325424,2776106367,2775887345,2775668357,2775449404,2775230486,2775011601,2774792752,2774573937,2774355157,2774136411,2773917699,2773699022,2773480380,2773261772,2773043198,2772824659,2772606154,2772387684,2772169248,2771950847,2771732480,2771514147,2771295849,2771077585,2770859355,2770641160,2770423000,2770204873,2769986781,2769768724,2769550700,
		2769332711,2769114756,2768896836,2768678950,2768461098,2768243281,2768025497,2767807748,2767590033,2767372353,2767154707,2766937095,2766719517,2766501973,2766284464,2766066989,2765849548,2765632141,2765414768,2765197430,2764980125,2764762855,2764545619,2764328417,2764111250,2763894116,2763677016,2763459951,2763242920,2763025922,2762808959,2762592030,
		2762375135,2762158274,2761941447,2761724654,2761507895,2761291171,2761074480,2760857823,2760641200,2760424611,2760208056,2759991536,2759775049,2759558596,2759342177,2759125792,2758909441,2758693123,2758476840,2758260591,2758044375,2757828194,2757612046,2757395932,2757179852,2756963806,2756747794,2756531816,2756315871,2756099960,2755884084,2755668241,
		2755452431,2755236656,2755020914,2754805206,2754589532,2754373892,2754158285,2753942713,2753727173,2753511668,2753296197,2753080759,2752865354,2752649984,2752434647,2752219344,2752004075,2751788839,2751573637,2751358469,2751143334,2750928233,2750713165,2750498131,2750283131,2750068164,2749853231,2749638332,2749423466,2749208634,2748993835,2748779070,
		2748564338,2748349640,2748134976,2747920345,2747705747,2747491183,2747276653,2747062156,2746847693,2746633263,2746418866,2746204503,2745990174,2745775878,2745561615,2745347386,2745133190,2744919028,2744704899,2744490803,2744276741,2744062712,2743848717,2743634755,2743420826,2743206931,2742993069,2742779240,2742565445,2742351683,2742137955,2741924259,
		2741710597,2741496969,2741283373,2741069811,2740856282,2740642787,2740429324,2740215895,2740002499,2739789137,2739575807,2739362511,2739149248,2738936019,2738722822,2738509659,2738296529,2738083432,2737870368,2737657337,2737444339,2737231375,2737018444,2736805546,2736592681,2736379849,2736167050,2735954284,2735741552,2735528852,2735316186,2735103552,
		2734890952,2734678385,2734465850,2734253349,2734040881,2733828446,2733616043,2733403674,2733191338,2732979035,2732766765,2732554527,2732342323,2732130152,2731918013,2731705908,2731493836,2731281796,2731069789,2730857816,2730645875,2730433967,2730222092,2730010249,2729798440,2729586664,2729374920,2729163209,2728951531,2728739886,2728528274,2728316695,
		2728105148,2727893634,2727682153,2727470705,2727259290,2727047907,2726836557,2726625240,2726413955,2726202704,2725991485,2725780299,2725569145,2725358024,2725146936,2724935881,2724724858,2724513868,2724302911,2724091986,2723881094,2723670235,2723459408,2723248614,2723037853,2722827124,2722616428,2722405764,2722195133,2721984535,2721773969,2721563436,
		2721352935,2721142467,2720932031,2720721628,2720511258,2720300920,2720090614,2719880341,2719670101,2719459893,2719249718,2719039575,2718829464,2718619386,2718409341,2718199328,2717989347,2717779399,2717569483,2717359600,2717149749,2716939930,2716730144,2716520390,2716310669,2716100980,2715891323,2715681699,2715472107,2715262548,2715053021,2714843526,
		2714634063,2714424633,2714215235,2714005870,2713796536,2713587235,2713377967,2713168730,2712959526,2712750354,2712541215,2712332107,2712123032,2711913989,2711704978,2711496000,2711287054,2711078140,2710869258,2710660408,2710451590,2710242805,2710034052,2709825331,2709616642,2709407985,2709199361,2708990768,2708782208,2708573679,2708365183,2708156719,
		2707948287,2707739887,2707531519,2707323184,2707114880,2706906608,2706698369,2706490161,2706281986,2706073842,2705865731,2705657651,2705449604,2705241588,2705033605,2704825653,2704617734,2704409846,2704201990,2703994167,2703786375,2703578615,2703370887,2703163191,2702955527,2702747895,2702540295,2702332726,2702125190,2701917685,2701710212,2701502771,
		2701295362,2701087985,2700880640,2700673326,2700466045,2700258795,2700051577,2699844390,2699637236,2699430113,2699223022,2699015963,2698808936,2698601940,2698394976,2698188044,2697981144,2697774275,2697567438,2697360633,2697153859,2696947118,2696740407,2696533729,2696327082,2696120467,2695913884,2695707332,2695500812,2695294323,2695087866,2694881441,
		2694675048,2694468686,2694262355,2694056056,2693849789,2693643553,2693437349,2693231177,2693025036,2692818927,2692612849,2692406803,2692200788,2691994805,2691788853,2691582933,2691377044,2691171187,2690965361,2690759567,2690553804,2690348073,2690142373,2689936705,2689731068,2689525462,2689319888,2689114345,2688908834,2688703354,2688497906,2688292489,
		2688087103,2687881749,2687676426,2687471135,2687265875,2687060646,2686855448,2686650282,2686445147,2686240044,2686034972,2685829931,2685624921,2685419943,2685214996,2685010081,2684805196,2684600343,2684395521,2684190730,2683985971,2683781243,2683576546,2683371880,2683167246,2682962643,2682758071,2682553530,2682349020,2682144541,2681940094,2681735678,
		2681531293,2681326939,2681122616,2680918325,2680714064,2680509835,2680305637,2680101470,2679897334,2679693229,2679489155,2679285112,2679081101,2678877120,2678673171,2678469252,2678265365,2678061508,2677857683,2677653889,2677450125,2677246393,2677042692,2676839021,2676635382,2676431774,2676228196,2676024650,2675821134,2675617650,2675414196,2675210774,
		2675007382,2674804021,2674600691,2674397392,2674194124,2673990887,2673787681,2673584506,2673381361,2673178248,2672975165,2672772113,2672569092,2672366102,2672163142,2671960214,2671757316,2671554449,2671351613,2671148808,2670946033,2670743289,2670540577,2670337894,2670135243,2669932622,2669730032,2669527473,2669324945,2669122447,2668919980,2668717544,
		2668515138,2668312763,2668110419,2667908106,2667705823,2667503571,2667301349,2667099158,2666896998,2666694869,2666492770,2666290701,2666088664,2665886657,2665684680,2665482735,2665280819,2665078935,2664877081,2664675257,2664473464,2664271702,2664069970,2663868269,2663666598,2663464958,2663263348,2663061769,2662860221,2662658703,2662457215,2662255758,
		2662054331,2661852935,2661651569,2661450234,2661248929,2661047655,2660846411,2660645198,2660444015,2660242862,2660041740,2659840648,2659639587,2659438556,2659237555,2659036585,2658835645,2658634736,2658433857,2658233008,2658032190,2657831402,2657630644,2657429917,2657229220,2657028553,2656827916,2656627310,2656426734,2656226189,2656025673,2655825188,
		2655624734,2655424309,2655223915,2655023551,2654823217,2654622913,2654422640,2654222397,2654022184,2653822001,2653621849,2653421727,2653221635,2653021573,2652821541,2652621539,2652421568,2652221626,2652021715,2651821834,2651621983,2651422162,2651222372,2651022611,2650822881,2650623180,2650423510,2650223870,2650024260,2649824680,2649625130,2649425610,
		2649226120,2649026660,2648827230,2648627830,2648428460,2648229121,2648029811,2647830531,2647631281,2647432061,2647232872,2647033712,2646834582,2646635482,2646436412,2646237372,2646038362,2645839382,2645640431,2645441511,2645242621,2645043760,2644844929,2644646129,2644447358,2644248617,2644049906,2643851224,2643652573,2643453952,2643255360,2643056798,
		2642858266,2642659764,2642461291,2642262849,2642064436,2641866053,2641667700,2641469377,2641271083,2641072819,2640874585,2640676381,2640478206,2640280061,2640081946,2639883861,2639685805,2639487779,2639289783,2639091816,2638893880,2638695972,2638498095,2638300247,2638102429,2637904641,2637706882,2637509153,2637311453,2637113783,2636916143,2636718532,
		2636520951,2636323400,2636125878,2635928386,2635730923,2635533490,2635336087,2635138713,2634941369,2634744054,2634546769,2634349513,2634152287,2633955090,2633757923,2633560786,2633363678,2633166599,2632969550,2632772530,2632575540,2632378580,2632181649,2631984747,2631787875,2631591032,2631394218,2631197435,2631000680,2630803955,2630607259,2630410593,
		2630213956,2630017349,2629820771,2629624222,2629427703,2629231213,2629034753,2628838322,2628641920,2628445547,2628249204,2628052890,2627856606,2627660351,2627464125,2627267928,2627071761,2626875623,2626679514,2626483435,2626287385,2626091364,2625895372,2625699410,2625503477,2625307573,2625111699,2624915853,2624720037,2624524250,2624328492,2624132764,
		2623937064,2623741394,2623545753,2623350141,2623154559,2622959005,2622763481,2622567986,2622372520,2622177083,2621981675,2621786296,2621590947,2621395626,2621200335,2621005073,2620809840,2620614635,2620419460,2620224315,2620029198,2619834110,2619639051,2619444021,2619249021,2619054049,2618859106,2618664193,2618469308,2618274453,2618079626,2617884829,
		2617690060,2617495320,2617300610,2617105928,2616911275,2616716651,2616522057,2616327491,2616132954,2615938446,2615743967,2615549516,2615355095,2615160703,2614966339,2614772005,2614577699,2614383422,2614189174,2613994955,2613800765,2613606603,2613412471,2613218367,2613024292,2612830246,2612636229,2612442241,2612248281,2612054350,2611860448,2611666575,
		2611472730,2611278915,2611085128,2610891370,2610697640,2610503939,2610310267,2610116624,2609923010,2609729424,2609535867,2609342339,2609148839,2608955368,2608761926,2608568512,2608375128,2608181771,2607988444,2607795145,2607601875,2607408633,2607215420,2607022236,2606829080,2606635953,2606442855,2606249785,2606056744,2605863731,2605670747,2605477791,
		2605284865,2605091966,2604899096,2604706255,2604513443,2604320658,2604127903,2603935176,2603742477,2603549807,2603357166,2603164553,2602971968,2602779412,2602586885,2602394386,2602201915,2602009473,2601817059,2601624674,2601432318,2601239989,2601047689,2600855418,2600663175,2600470961,2600278774,2600086617,2599894487,2599702386,2599510314,2599318269,
		2599126254,2598934266,2598742307,2598550376,2598358474,2598166600,2597974754,2597782937,2597591148,2597399387,2597207655,2597015950,2596824275,2596632627,2596441008,2596249417,2596057854,2595866320,2595674813,2595483335,2595291886,2595100464,2594909071,2594717706,2594526369,2594335061,2594143781,2593952528,2593761305,2593570109,2593378941,2593187802,
		2592996691,2592805608,2592614553,2592423526,2592232527,2592041557,2591850615,2591659701,2591468815,2591277957,2591087127,2590896325,2590705552,2590514806,2590324089,2590133399,2589942738,2589752105,2589561500,2589370923,2589180374,2588989853,2588799360,2588608895,2588418458,2588228049,2588037668,2587847315,2587656990,2587466693,2587276424,2587086184,
		2586895971,2586705786,2586515629,2586325500,2586135398,2585945325,2585755280,2585565263,2585375273,2585185312,2584995378,2584805473,2584615595,2584425745,2584235923,2584046129,2583856363,2583666625,2583476914,2583287232,2583097577,2582907950,2582718351,2582528780,2582339236,2582149721,2581960233,2581770773,2581581341,2581391937,2581202560,2581013211,
		2580823890,2580634597,2580445332,2580256094,2580066884,2579877702,2579688547,2579499421,2579310322,2579121250,2578932207,2578743191,2578554203,2578365242,2578176309,2577987404,2577798527,2577609677,2577420855,2577232061,2577043294,2576854555,2576665844,2576477160,2576288504,2576099875,2575911274,2575722701,2575534155,2575345637,2575157147,2574968684,
		2574780248,2574591841,2574403461,2574215108,2574026783,2573838485,2573650215,2573461973,2573273758,2573085571,2572897411,2572709279,2572521174,2572333097,2572145047,2571957025,2571769030,2571581062,2571393123,2571205210,2571017325,2570829468,2570641638,2570453835,2570266060,2570078312,2569890592,2569702899,2569515234,2569327596,2569139985,2568952402,
		2568764846,2568577318,2568389816,2568202343,2568014896,2567827477,2567640086,2567452722,2567265385,2567078075,2566890793,2566703538,2566516310,2566329110,2566141937,2565954791,2565767673,2565580582,2565393518,2565206481,2565019472,2564832490,2564645535,2564458608,2564271707,2564084834,2563897988,2563711170,2563524379,2563337614,2563150878,2562964168,
		2562777485,2562590830,2562404202,2562217601,2562031027,2561844481,2561657961,2561471469,2561285004,2561098566,2560912155,2560725771,2560539415,2560353085,2560166783,2559980508,2559794259,2559608038,2559421844,2559235678,2559049538,2558863425,2558677340,2558491281,2558305249,2558119245,2557933268,2557747317,2557561394,2557375498,2557189628,2557003786,
		2556817971,2556632183,2556446421,2556260687,2556074980,2555889299,2555703646,2555518020,2555332420,2555146848,2554961302,2554775784,2554590292,2554404828,2554219390,2554033979,2553848595,2553663238,2553477908,2553292605,2553107329,2552922079,2552736857,2552551661,2552366492,2552181350,2551996235,2551811147,2551626086,2551441051,2551256043,2551071063,
		2550886109,2550701181,2550516281,2550331407,2550146561,2549961741,2549776947,2549592181,2549407441,2549222728,2549038042,2548853383,2548668750,2548484144,2548299565,2548115013,2547930487,2547745988,2547561516,2547377070,2547192652,2547008259,2546823894,2546639555,2546455243,2546270958,2546086699,2545902467,2545718262,2545534083,2545349931,2545165806,
		2544981707,2544797635,2544613589,2544429570,2544245578,2544061612,2543877673,2543693761,2543509875,2543326015,2543142183,2542958376,2542774597,2542590844,2542407117,2542223417,2542039744,2541856097,2541672477,2541488883,2541305316,2541121775,2540938261,2540754773,2540571312,2540387877,2540204469,2540021087,2539837732,2539654403,2539471100,2539287824,
		2539104575,2538921352,2538738155,2538554985,2538371842,2538188724,2538005633,2537822569,2537639531,2537456519,2537273534,2537090575,2536907643,2536724737,2536541857,2536359003,2536176176,2535993376,2535810601,2535627854,2535445132,2535262437,2535079768,2534897125,2534714509,2534531919,2534349355,2534166818,2533984307,2533801822,2533619363,2533436931,
		2533254525,2533072145,2532889792,2532707464,2532525163,2532342889,2532160640,2531978418,2531796222,2531614052,2531431908,2531249791,2531067700,2530885635,2530703596,2530521583,2530339597,2530157637,2529975703,2529793795,2529611913,2529430057,2529248228,2529066424,2528884647,2528702896,2528521171,2528339472,2528157800,2527976153,2527794532,2527612938,
		2527431370,2527249827,2527068311,2526886821,2526705357,2526523919,2526342507,2526161121,2525979761,2525798428,2525617120,2525435838,2525254582,2525073353,2524892149,2524710971,2524529820,2524348694,2524167594,2523986521,2523805473,2523624451,2523443455,2523262485,2523081541,2522900624,2522719732,2522538865,2522358025,2522177211,2521996423,2521815661,
		2521634924,2521454214,2521273529,2521092870,2520912237,2520731630,2520551049,2520370494,2520189965,2520009461,2519828983,2519648532,2519468106,2519287706,2519107331,2518926983,2518746660,2518566363,2518386092,2518205847,2518025628,2517845434,2517665266,2517485124,2517305008,2517124917,2516944853,2516764814,2516584801,2516404813,2516224851,2516044915,
		2515865005,2515685121,2515505262,2515325429,2515145621,2514965840,2514786084,2514606353,2514426649,2514246970,2514067317,2513887689,2513708087,2513528511,2513348960,2513169435,2512989936,2512810462,2512631014,2512451592,2512272195,2512092824,2511913479,2511734159,2511554864,2511375596,2511196353,2511017135,2510837943,2510658777,2510479636,2510300521,
		2510121431,2509942367,2509763328,2509584315,2509405327,2509226365,2509047429,2508868518,2508689633,2508510773,2508331938,2508153129,2507974346,2507795588,2507616855,2507438148,2507259467,2507080811,2506902180,2506723575,2506544995,2506366441,2506187912,2506009409,2505830931,2505652478,2505474051,2505295649,2505117273,2504938922,2504760596,2504582296,
		2504404021,2504225772,2504047548,2503869349,2503691176,2503513028,2503334905,2503156808,2502978736,2502800690,2502622668,2502444673,2502266702,2502088757,2501910837,2501732942,2501555073,2501377228,2501199410,2501021616,2500843848,2500666105,2500488387,2500310695,2500133027,2499955386,2499777769,2499600177,2499422611,2499245070,2499067554,2498890064,
		2498712598,2498535158,2498357743,2498180353,2498002989,2497825649,2497648335,2497471046,2497293782,2497116543,2496939330,2496762141,2496584978,2496407840,2496230727,2496053639,2495876576,2495699539,2495522526,2495345539,2495168576,2494991639,2494814727,2494637840,2494460978,2494284141,2494107329,2493930543,2493753781,2493577044,2493400333,2493223646,
		2493046985,2492870348,2492693737,2492517150,2492340589,2492164053,2491987541,2491811055,2491634594,2491458157,2491281746,2491105359,2490928998,2490752661,2490576350,2490400063,2490223802,2490047565,2489871353,2489695167,2489519005,2489342868,2489166756,2488990669,2488814607,2488638570,2488462557,2488286570,2488110607,2487934669,2487758757,2487582869,
		2487407006,2487231168,2487055354,2486879566,2486703802,2486528063,2486352349,2486176660,2486000996,2485825357,2485649742,2485474152,2485298587,2485123047,2484947531,2484772041,2484596575,2484421134,2484245717,2484070326,2483894959,2483719617,2483544300,2483369007,2483193740,2483018497,2482843278,2482668085,2482492916,2482317772,2482142652,2481967558,
		2481792488,2481617442,2481442422,2481267426,2481092454,2480917508,2480742586,2480567689,2480392816,2480217968,2480043145,2479868346,2479693572,2479518823,2479344098,2479169398,2478994722,2478820072,2478645445,2478470844,2478296267,2478121714,2477947186,2477772683,2477598204,2477423750,2477249320,2477074915,2476900535,2476726179,2476551847,2476377541,
		2476203258,2476029000,2475854767,2475680558,2475506374,2475332214,2475158079,2474983968,2474809882,2474635821,2474461783,2474287770,2474113782,2473939818,2473765879,2473591964,2473418074,2473244208,2473070366,2472896549,2472722756,2472548988,2472375244,2472201524,2472027829,2471854159,2471680513,2471506891,2471333293,2471159720,2470986171,2470812647,
		2470639147,2470465672,2470292220,2470118794,2469945391,2469772013,2469598659,2469425330,2469252024,2469078744,2468905487,2468732255,2468559047,2468385863,2468212704,2468039569,2467866458,2467693372,2467520310,2467347272,2467174258,2467001269,2466828304,2466655363,2466482446,2466309554,2466136686,2465963842,2465791022,2465618227,2465445456,2465272709,
		2465099986,2464927287,2464754613,2464581962,2464409336,2464236735,2464064157,2463891603,2463719074,2463546569,2463374088,2463201631,2463029198,2462856790,2462684405,2462512045,2462339709,2462167396,2461995109,2461822845,2461650605,2461478389,2461306198,2461134030,2460961887,2460789768,2460617672,2460445601,2460273554,2460101531,2459929532,2459757557,
		2459585606,2459413679,2459241776,2459069898,2458898043,2458726212,2458554405,2458382623,2458210864,2458039129,2457867419,2457695732,2457524069,2457352430,2457180815,2457009225,2456837658,2456666115,2456494596,2456323101,2456151630,2455980183,2455808759,2455637360,2455465985,2455294633,2455123306,2454952002,2454780723,2454609467,2454438235,2454267027,
		2454095843,2453924682,2453753546,2453582433,2453411345,2453240280,2453069239,2452898222,2452727229,2452556259,2452385314,2452214392,2452043494,2451872620,2451701770,2451530943,2451360140,2451189362,2451018606,2450847875,2450677168,2450506484,2450335824,2450165188,2449994575,2449823987,2449653422,2449482881,2449312363,2449141869,2448971399,2448800953,
		2448630531,2448460132,2448289757,2448119406,2447949078,2447778774,2447608494,2447438237,2447268004,2447097795,2446927609,2446757448,2446587309,2446417195,2446247104,2446077037,2445906993,2445736973,2445566977,2445397004,2445227055,2445057130,2444887228,2444717350,2444547495,2444377664,2444207857,2444038073,2443868313,2443698576,2443528863,2443359173,
		2443189507,2443019865,2442850246,2442680651,2442511079,2442341531,2442172006,2442002505,2441833028,2441663574,2441494143,2441324736,2441155353,2440985993,2440816656,2440647343,2440478053,2440308787,2440139545,2439970326,2439801130,2439631958,2439462809,2439293684,2439124582,2438955504,2438786449,2438617417,2438448409,2438279425,2438110463,2437941526,
		2437772611,2437603720,2437434853,2437266008,2437097188,2436928390,2436759616,2436590865,2436422138,2436253434,2436084754,2435916096,2435747463,2435578852,2435410265,2435241701,2435073160,2434904643,2434736149,2434567679,2434399232,2434230808,2434062407,2433894030,2433725676,2433557345,2433389038,2433220754,2433052493,2432884255,2432716041,2432547850,
		2432379682,2432211537,2432043416,2431875318,2431707243,2431539191,2431371163,2431203158,2431035176,2430867217,2430699281,2430531369,2430363480,2430195614,2430027771,2429859951,2429692155,2429524382,2429356632,2429188905,2429021201,2428853520,2428685863,2428518229,2428350617,2428183029,2428015465,2427847923,2427680404,2427512909,2427345436,2427177987,
		2427010561,2426843157,2426675777,2426508420,2426341087,2426173776,2426006488,2425839223,2425671982,2425504763,2425337568,2425170395,2425003246,2424836120,2424669016,2424501936,2424334879,2424167845,2424000833,2423833845,2423666880,2423499938,2423333019,2423166122,2422999249,2422832399,2422665572,2422498767,2422331986,2422165228,2421998492,2421831780,
		2421665090,2421498424,2421331780,2421165159,2420998562,2420831987,2420665435,2420498906,2420332400,2420165917,2419999456,2419833019,2419666605,2419500213,2419333844,2419167498,2419001175,2418834875,2418668598,2418502344,2418336112,2418169904,2418003718,2417837555,2417671415,2417505297,2417339203,2417173131,2417007082,2416841056,2416675053,2416509073,
		2416343115,2416177180,2416011268,2415845379,2415679512,2415513669,2415347848,2415182049,2415016274,2414850521,2414684791,2414519084,2414353400,2414187738,2414022099,2413856483,2413690889,2413525319,2413359771,2413194245,2413028743,2412863263,2412697805,2412532371,2412366959,2412201570,2412036203,2411870859,2411705538,2411540240,2411374964,2411209711,
		2411044480,2410879272,2410714087,2410548924,2410383784,2410218667,2410053572,2409888500,2409723450,2409558423,2409393419,2409228437,2409063478,2408898541,2408733627,2408568736,2408403867,2408239021,2408074197,2407909396,2407744618,2407579862,2407415128,2407250417,2407085729,2406921063,2406756419,2406591799,2406427200,2406262625,2406098071,2405933540,
		2405769032,2405604546,2405440083,2405275642,2405111224,2404946828,2404782455,2404618104,2404453775,2404289469,2404125186,2403960925,2403796686,2403632470,2403468276,2403304105,2403139956,2402975830,2402811725,2402647644,2402483585,2402319548,2402155533,2401991541,2401827572,2401663624,2401499699,2401335797,2401171917,2401008059,2400844224,2400680410,
		2400516620,2400352851,2400189105,2400025382,2399861680,2399698001,2399534345,2399370710,2399207098,2399043509,2398879941,2398716396,2398552873,2398389373,2398225894,2398062438,2397899005,2397735593,2397572204,2397408837,2397245493,2397082170,2396918870,2396755592,2396592337,2396429103,2396265892,2396102703,2395939537,2395776392,2395613270,2395450170,
		2395287092,2395124036,2394961003,2394797992,2394635003,2394472036,2394309091,2394146169,2393983268,2393820390,2393657534,2393494700,2393331889,2393169099,2393006332,2392843587,2392680863,2392518162,2392355484,2392192827,2392030192,2391867580,2391704989,2391542421,2391379875,2391217351,2391054849,2390892369,2390729911,2390567475,2390405061,2390242670,
		2390080300,2389917952,2389755627,2389593324,2389431042,2389268783,2389106546,2388944330,2388782137,2388619966,2388457817,2388295689,2388133584,2387971501,2387809440,2387647401,2387485383,2387323388,2387161415,2386999464,2386837535,2386675627,2386513742,2386351878,2386190037,2386028218,2385866420,2385704644,2385542891,2385381159,2385219449,2385057761,
		2384896095,2384734451,2384572829,2384411229,2384249651,2384088094,2383926560,2383765047,2383603556,2383442088,2383280641,2383119215,2382957812,2382796431,2382635071,2382473733,2382312418,2382151124,2381989851,2381828601,2381667373,2381506166,2381344981,2381183818,2381022677,2380861557,2380700460,2380539384,2380378330,2380217298,2380056287,2379895299,
		2379734332,2379573387,2379412463,2379251562,2379090682,2378929824,2378768988,2378608173,2378447380,2378286609,2378125860,2377965132,2377804426,2377643742,2377483080,2377322439,2377161820,2377001223,2376840647,2376680093,2376519561,2376359050,2376198561,2376038094,2375877648,2375717225,2375556822,2375396442,2375236083,2375075746,2374915430,2374755136,
		2374594864,2374434613,2374274384,2374114177,2373953991,2373793826,2373633684,2373473563,2373313463,2373153386,2372993329,2372833295,2372673282,2372513290,2372353320,2372193372,2372033445,2371873540,2371713657,2371553795,2371393954,2371234135,2371074338,2370914562,2370754807,2370595075,2370435363,2370275673,2370116005,2369956358,2369796733,2369637129,
		2369477547,2369317986,2369158447,2368998929,2368839433,2368679958,2368520505,2368361073,2368201662,2368042273,2367882906,2367723560,2367564235,2367404932,2367245650,2367086390,2366927151,2366767933,2366608737,2366449563,2366290409,2366131277,2365972167,2365813078,2365654010,2365494964,2365335939,2365176936,2365017954,2364858993,2364700054,2364541136,
		2364382239,2364223364,2364064510,2363905677,2363746866,2363588076,2363429307,2363270560,2363111834,2362953129,2362794446,2362635784,2362477143,2362318524,2362159926,2362001349,2361842794,2361684259,2361525746,2361367255,2361208784,2361050335,2360891907,2360733501,2360575115,2360416751,2360258409,2360100087,2359941787,2359783507,2359625250,2359467013,
		2359308797,2359150603,2358992430,2358834278,2358676148,2358518038,2358359950,2358201883,2358043837,2357885813,2357727809,2357569827,2357411866,2357253926,2357096007,2356938109,2356780233,2356622378,2356464543,2356306730,2356148938,2355991168,2355833418,2355675689,2355517982,2355360296,2355202630,2355044986,2354887363,2354729761,2354572181,2354414621,
		2354257082,2354099565,2353942068,2353784593,2353627139,2353469705,2353312293,2353154902,2352997532,2352840183,2352682855,2352525548,2352368262,2352210997,2352053753,2351896531,2351739329,2351582148,2351424988,2351267849,2351110731,2350953635,2350796559,2350639504,2350482470,2350325457,2350168465,2350011495,2349854545,2349697616,2349540708,2349383821,
		2349226954,2349070109,2348913285,2348756482,2348599699,2348442938,2348286197,2348129478,2347972779,2347816101,2347659445,2347502809,2347346194,2347189599,2347033026,2346876474,2346719942,2346563432,2346406942,2346250473,2346094025,2345937598,2345781192,2345624806,2345468442,2345312098,2345155775,2344999473,2344843192,2344686932,2344530692,2344374473,
		2344218275,2344062098,2343905942,2343749807,2343593692,2343437598,2343281525,2343125473,2342969441,2342813430,2342657440,2342501471,2342345523,2342189595,2342033688,2341877802,2341721937,2341566092,2341410268,2341254465,2341098682,2340942921,2340787180,2340631459,2340475760,2340320081,2340164423,2340008785,2339853169,2339697573,2339541997,2339386443,
		2339230909,2339075395,2338919903,2338764431,2338608979,2338453549,2338298139,2338142750,2337987381,2337832033,2337676706,2337521399,2337366113,2337210847,2337055603,2336900378,2336745175,2336589992,2336434829,2336279688,2336124567,2335969466,2335814386,2335659327,2335504288,2335349270,2335194272,2335039295,2334884338,2334729403,2334574487,2334419592,
		2334264718,2334109864,2333955031,2333800219,2333645427,2333490655,2333335904,2333181173,2333026464,2332871774,2332717105,2332562457,2332407829,2332253221,2332098634,2331944068,2331789522,2331634996,2331480491,2331326007,2331171543,2331017099,2330862676,2330708273,2330553891,2330399529,2330245188,2330090867,2329936567,2329782287,2329628027,2329473788,
		2329319569,2329165371,2329011193,2328857036,2328702899,2328548782,2328394686,2328240610,2328086554,2327932519,2327778505,2327624510,2327470536,2327316583,2327162649,2327008737,2326854844,2326700972,2326547120,2326393289,2326239477,2326085687,2325931916,2325778166,2325624436,2325470727,2325317038,2325163369,2325009720,2324856092,2324702484,2324548896,
		2324395329,2324241782,2324088255,2323934749,2323781263,2323627797,2323474351,2323320926,2323167520,2323014136,2322860771,2322707426,2322554102,2322400798,2322247515,2322094251,2321941008,2321787785,2321634582,2321481400,2321328238,2321175095,2321021974,2320868872,2320715790,2320562729,2320409688,2320256667,2320103666,2319950686,2319797725,2319644785,
		2319491865,2319338965,2319186085,2319033225,2318880386,2318727567,2318574767,2318421988,2318269230,2318116491,2317963772,2317811074,2317658395,2317505737,2317353099,2317200481,2317047883,2316895305,2316742747,2316590209,2316437692,2316285194,2316132717,2315980259,2315827822,2315675405,2315523007,2315370630,2315218273,2315065936,2314913619,2314761322,
		2314609045,2314456788,2314304551,2314152335,2314000138,2313847961,2313695804,2313543668,2313391551,2313239454,2313087377,2312935321,2312783284,2312631267,2312479270,2312327293,2312175337,2312023400,2311871483,2311719586,2311567709,2311415852,2311264015,2311112198,2310960400,2310808623,2310656866,2310505128,2310353411,2310201713,2310050036,2309898378,
		2309746740,2309595123,2309443525,2309291946,2309140388,2308988850,2308837332,2308685833,2308534354,2308382896,2308231457,2308080038,2307928639,2307777259,2307625900,2307474560,2307323241,2307171941,2307020661,2306869401,2306718160,2306566940,2306415739,2306264558,2306113397,2305962256,2305811134,2305660033,2305508951,2305357889,2305206846,2305055824,
		2304904821,2304753838,2304602875,2304451932,2304301008,2304150105,2303999221,2303848356,2303697512,2303546687,2303395882,2303245097,2303094331,2302943585,2302792859,2302642153,2302491466,2302340799,2302190152,2302039525,2301888917,2301738329,2301587761,2301437212,2301286683,2301136174,2300985684,2300835214,2300684764,2300534333,2300383923,2300233531,
		2300083160,2299932808,2299782476,2299632163,2299481870,2299331597,2299181343,2299031109,2298880895,2298730700,2298580525,2298430370,2298280234,2298130117,2297980021,2297829944,2297679886,2297529848,2297379830,2297229831,2297079852,2296929893,2296779953,2296630032,2296480132,2296330250,2296180389,2296030547,2295880724,2295730921,2295581138,2295431374,
		2295281629,2295131905,2294982199,2294832513,2294682847,2294533201,2294383573,2294233966,2294084377,2293934809,2293785260,2293635730,2293486220,2293336729,2293187258,2293037806,2292888374,2292738961,2292589568,2292440194,2292290840,2292141505,2291992189,2291842893,2291693617,2291544360,2291395122,2291245904,2291096705,2290947525,2290798366,2290649225,
		2290500104,2290351002,2290201920,2290052857,2289903814,2289754790,2289605785,2289456800,2289307834,2289158887,2289009960,2288861052,2288712164,2288563295,2288414445,2288265615,2288116804,2287968013,2287819240,2287670487,2287521754,2287373040,2287224345,2287075669,2286927013,2286778376,2286629759,2286481160,2286332581,2286184022,2286035482,2285886961,
		2285738459,2285589976,2285441513,2285293069,2285144645,2284996240,2284847854,2284699487,2284551139,2284402811,2284254502,2284106212,2283957942,2283809691,2283661459,2283513246,2283365053,2283216878,2283068723,2282920588,2282772471,2282624374,2282476296,2282328237,2282180197,2282032176,2281884175,2281736193,2281588230,2281440286,2281292362,2281144456,
		2280996570,2280848703,2280700855,2280553027,2280405217,2280257427,2280109656,2279961904,2279814171,2279666457,2279518762,2279371087,2279223430,2279075793,2278928175,2278780576,2278632996,2278485436,2278337894,2278190371,2278042868,2277895384,2277747918,2277600472,2277453045,2277305637,2277158248,2277010879,2276863528,2276716196,2276568884,2276421590,
		2276274316,2276127060,2275979824,2275832607,2275685408,2275538229,2275391069,2275243928,2275096805,2274949702,2274802618,2274655553,2274508507,2274361480,2274214472,2274067483,2273920513,2273773562,2273626630,2273479717,2273332823,2273185948,2273039091,2272892254,2272745436,2272598637,2272451857,2272305095,2272158353,2272011630,2271864925,2271718240,
		2271571573,2271424926,2271278297,2271131687,2270985096,2270838524,2270691971,2270545437,2270398922,2270252426,2270105949,2269959490,2269813051,2269666630,2269520228,2269373845,2269227481,2269081136,2268934810,2268788502,2268642214,2268495944,2268349694,2268203462,2268057249,2267911054,2267764879,2267618722,2267472585,2267326466,2267180366,2267034285,
		2266888222,2266742179,2266596154,2266450148,2266304161,2266158192,2266012243,2265866312,2265720400,2265574507,2265428633,2265282777,2265136940,2264991122,2264845323,2264699543,2264553781,2264408038,2264262314,2264116609,2263970922,2263825254,2263679605,2263533974,2263388363,2263242770,2263097195,2262951640,2262806103,2262660585,2262515086,2262369605,
		2262224143,2262078700,2261933275,2261787869,2261642482,2261497114,2261351764,2261206433,2261061121,2260915827,2260770552,2260625295,2260480058,2260334839,2260189638,2260044456,2259899293,2259754149,2259609023,2259463916,2259318827,2259173757,2259028706,2258883673,2258738659,2258593664,2258448687,2258303729,2258158789,2258013868,2257868966,2257724082,
		2257579217,2257434370,2257289542,2257144733,2256999942,2256855170,2256710416,2256565681,2256420964,2256276266,2256131587,2255986926,2255842284,2255697660,2255553054,2255408468,2255263900,2255119350,2254974819,2254830306,2254685812,2254541336,2254396879,2254252441,2254108021,2253963619,2253819236,2253674872,2253530526,2253386198,2253241889,2253097598,
		2252953326,2252809073,2252664837,2252520621,2252376423,2252232243,2252088081,2251943939,2251799814,2251655708,2251511621,2251367552,2251223501,2251079469,2250935455,2250791460,2250647483,2250503524,2250359584,2250215662,2250071759,2249927874,2249784008,2249640160,2249496330,2249352519,2249208726,2249064951,2248921195,2248777457,2248633738,2248490037,
		2248346354,2248202690,2248059044,2247915416,2247771807,2247628216,2247484644,2247341089,2247197554,2247054036,2246910537,2246767056,2246623593,2246480149,2246336723,2246193316,2246049926,2245906555,2245763203,2245619868,2245476552,2245333254,2245189975,2245046714,2244903471,2244760246,2244617040,2244473852,2244330682,2244187530,2244044397,2243901282,
		2243758185,2243615106,2243472046,2243329004,2243185980,2243042974,2242899987,2242757018,2242614067,2242471134,2242328220,2242185324,2242042445,2241899586,2241756744,2241613921,2241471115,2241328328,2241185559,2241042809,2240900076,2240757362,2240614666,2240471988,2240329328,2240186686,2240044063,2239901458,2239758871,2239616302,2239473751,2239331218,
		2239188703,2239046207,2238903729,2238761269,2238618827,2238476403,2238333997,2238191609,2238049240,2237906888,2237764555,2237622240,2237479943,2237337664,2237195403,2237053160,2236910935,2236768728,2236626540,2236484369,2236342217,2236200082,2236057966,2235915868,2235773788,2235631726,2235489681,2235347655,2235205647,2235063658,2234921686,2234779732,
		2234637796,2234495878,2234353978,2234212097,2234070233,2233928387,2233786559,2233644750,2233502958,2233361184,2233219429,2233077691,2232935971,2232794269,2232652586,2232510920,2232369272,2232227642,2232086030,2231944436,2231802861,2231661303,2231519763,2231378241,2231236736,2231095250,2230953782,2230812332,2230670899,2230529485,2230388089,2230246710,
		2230105349,2229964007,2229822682,2229681375,2229540086,2229398815,2229257562,2229116326,2228975109,2228833910,2228692728,2228551564,2228410418,2228269290,2228128180,2227987088,2227846014,2227704957,2227563919,2227422898,2227281895,2227140910,2226999943,2226858993,2226718062,2226577148,2226436252,2226295374,2226154514,2226013672,2225872847,2225732041,
		2225591252,2225450481,2225309727,2225168992,2225028274,2224887574,2224746892,2224606228,2224465581,2224324952,2224184341,2224043748,2223903173,2223762615,2223622075,2223481553,2223341049,2223200562,2223060093,2222919642,2222779209,2222638793,2222498395,2222358015,2222217653,2222077308,2221936981,2221796672,2221656380,2221516107,2221375851,2221235612,
		2221095392,2220955189,2220815003,2220674836,2220534686,2220394554,2220254439,2220114342,2219974263,2219834202,2219694158,2219554132,2219414123,2219274133,2219134159,2218994204,2218854266,2218714346,2218574443,2218434558,2218294691,2218154842,2218015010,2217875195,2217735398,2217595619,2217455858,2217316114,2217176388,2217036679,2216896988,2216757315,
		2216617659,2216478020,2216338400,2216198797,2216059211,2215919643,2215780093,2215640560,2215501045,2215361547,2215222067,2215082605,2214943160,2214803733,2214664323,2214524931,2214385556,2214246199,2214106859,2213967537,2213828232,2213688945,2213549676,2213410424,2213271189,2213131972,2212992773,2212853591,2212714427,2212575280,2212436150,2212297038,
		2212157944,2212018867,2211879807,2211740765,2211601741,2211462734,2211323744,2211184772,2211045818,2210906881,2210767961,2210629059,2210490174,2210351306,2210212457,2210073624,2209934809,2209796012,2209657231,2209518469,2209379724,2209240996,2209102285,2208963592,2208824917,2208686258,2208547618,2208408994,2208270388,2208131800,2207993229,2207854675,
		2207716138,2207577619,2207439118,2207300633,2207162167,2207023717,2206885285,2206746870,2206608473,2206470093,2206331730,2206193385,2206055057,2205916746,2205778453,2205640177,2205501918,2205363677,2205225453,2205087246,2204949057,2204810885,2204672730,2204534593,2204396472,2204258370,2204120284,2203982216,2203844165,2203706132,2203568115,2203430116,
		2203292135,2203154170,2203016223,2202878293,2202740380,2202602485,2202464607,2202326746,2202188903,2202051076,2201913267,2201775475,2201637701,2201499943,2201362203,2201224481,2201086775,2200949087,2200811415,2200673761,2200536125,2200398505,2200260903,2200123318,2199985750,2199848199,2199710666,2199573149,2199435650,2199298168,2199160704,2199023256,
		2198885826,2198748413,2198611016,2198473638,2198336276,2198198931,2198061604,2197924294,2197787001,2197649725,2197512466,2197375225,2197238000,2197100793,2196963603,2196826430,2196689274,2196552135,2196415013,2196277909,2196140821,2196003751,2195866698,2195729662,2195592643,2195455641,2195318656,2195181688,2195044737,2194907804,2194770887,2194633988,
		2194497106,2194360241,2194223392,2194086561,2193949747,2193812950,2193676170,2193539408,2193402662,2193265933,2193129221,2192992527,2192855849,2192719188,2192582545,2192445918,2192309309,2192172716,2192036141,2191899582,2191763041,2191626517,2191490009,2191353519,2191217045,2191080589,2190944150,2190807727,2190671322,2190534933,2190398562,2190262207,
		2190125870,2189989549,2189853246,2189716959,2189580689,2189444437,2189308201,2189171982,2189035780,2188899595,2188763427,2188627276,2188491142,2188355025,2188218925,2188082842,2187946776,2187810726,2187674694,2187538678,2187402679,2187266698,2187130733,2186994785,2186858854,2186722940,2186587042,2186451162,2186315298,2186179452,2186043622,2185907809,
		2185772013,2185636234,2185500472,2185364727,2185228998,2185093286,2184957592,2184821914,2184686253,2184550608,2184414981,2184279370,2184143777,2184008200,2183872640,2183737096,2183601570,2183466060,2183330568,2183195092,2183059632,2182924190,2182788765,2182653356,2182517964,2182382589,2182247230,2182111889,2181976564,2181841256,2181705965,2181570691,
		2181435433,2181300192,2181164968,2181029761,2180894570,2180759396,2180624239,2180489099,2180353975,2180218868,2180083778,2179948705,2179813648,2179678608,2179543585,2179408579,2179273589,2179138616,2179003660,2178868720,2178733798,2178598891,2178464002,2178329129,2178194273,2178059434,2177924611,2177789805,2177655016,2177520244,2177385488,2177250749,
		2177116026,2176981320,2176846631,2176711958,2176577303,2176442663,2176308041,2176173435,2176038846,2175904273,2175769717,2175635178,2175500655,2175366149,2175231660,2175097187,2174962731,2174828291,2174693868,2174559462,2174425072,2174290699,2174156343,2174022003,2173887680,2173753373,2173619083,2173484810,2173350553,2173216312,2173082089,2172947881,
		2172813691,2172679517,2172545359,2172411219,2172277094,2172142987,2172008895,2171874821,2171740763,2171606721,2171472696,2171338688,2171204696,2171070721,2170936762,2170802819,2170668894,2170534984,2170401092,2170267215,2170133356,2169999513,2169865686,2169731876,2169598082,2169464305,2169330544,2169196800,2169063072,2168929361,2168795666,2168661988,
		2168528326,2168394681,2168261052,2168127440,2167993844,2167860265,2167726702,2167593155,2167459625,2167326112,2167192615,2167059134,2166925670,2166792222,2166658790,2166525375,2166391977,2166258595,2166125229,2165991880,2165858547,2165725231,2165591931,2165458647,2165325380,2165192129,2165058895,2164925677,2164792475,2164659290,2164526121,2164392969,
		2164259833,2164126713,2163993610,2163860523,2163727452,2163594398,2163461360,2163328338,2163195333,2163062345,2162929372,2162796416,2162663476,2162530553,2162397646,2162264755,2162131881,2161999023,2161866181,2161733356,2161600547,2161467754,2161334977,2161202217,2161069473,2160936746,2160804035,2160671340,2160538661,2160405999,2160273353,2160140723,
		2160008110,2159875512,2159742931,2159610367,2159477819,2159345286,2159212771,2159080271,2158947788,2158815321,2158682870,2158550435,2158418017,2158285615,2158153229,2158020860,2157888506,2157756169,2157623848,2157491544,2157359255,2157226983,2157094727,2156962488,2156830264,2156698057,2156565866,2156433691,2156301532,2156169389,2156037263,2155905153,
		2155773059,2155640981,2155508920,2155376874,2155244845,2155112832,2154980835,2154848855,2154716890,2154584942,2154453009,2154321093,2154189193,2154057310,2153925442,2153793591,2153661755,2153529936,2153398133,2153266346,2153134575,2153002821,2152871082,2152739360,2152607654,2152475963,2152344289,2152212631,2152080989,2151949364,2151817754,2151686161,
		2151554583,2151423022,2151291476,2151159947,2151028434,2150896937,2150765456,2150633991,2150502542,2150371110,2150239693,2150108292,2149976908,2149845539,2149714187,2149582851,2149451530,2149320226,2149188938,2149057665,2148926409,2148795169,2148663945,2148532737,2148401545,2148270369,2148139209,2148008065,2147876937,2147745825,2147614729,
	};
	/*
	for (int freq = 0; freq < 16384; freq++) {
		if (freq == 0) { cout << "255,"; }
		else {
			cout << int(1 + (14 - log2((float)freq)) * 16) << "," << (freq % 32 == 0 ? "\n" : "");
		}
	}
	*/
	const uint8_t FREQ_COST[MODEL_SCALE] = {
		255,225,209,199,193,187,183,180,177,174,171,169,167,165,164,162,161,159,158,157,155,154,153,152,151,150,149,148,148,147,146,145,145,
		144,143,142,142,141,141,140,139,139,138,138,137,137,136,136,135,135,134,134,133,133,132,132,132,131,131,130,130,130,129,129,129,
		128,128,127,127,127,126,126,126,125,125,125,125,124,124,124,123,123,123,122,122,122,122,121,121,121,121,120,120,120,120,119,119,
		119,119,118,118,118,118,118,117,117,117,117,116,116,116,116,116,115,115,115,115,115,114,114,114,114,114,113,113,113,113,113,113,
		112,112,112,112,112,111,111,111,111,111,111,110,110,110,110,110,110,109,109,109,109,109,109,109,108,108,108,108,108,108,107,107,
		107,107,107,107,107,106,106,106,106,106,106,106,106,105,105,105,105,105,105,105,105,104,104,104,104,104,104,104,104,103,103,103,
		103,103,103,103,103,102,102,102,102,102,102,102,102,102,101,101,101,101,101,101,101,101,101,100,100,100,100,100,100,100,100,100,
		99,99,99,99,99,99,99,99,99,99,98,98,98,98,98,98,98,98,98,98,98,97,97,97,97,97,97,97,97,97,97,97,
		96,96,96,96,96,96,96,96,96,96,96,95,95,95,95,95,95,95,95,95,95,95,95,94,94,94,94,94,94,94,94,94,
		94,94,94,93,93,93,93,93,93,93,93,93,93,93,93,93,92,92,92,92,92,92,92,92,92,92,92,92,92,91,91,91,
		91,91,91,91,91,91,91,91,91,91,91,90,90,90,90,90,90,90,90,90,90,90,90,90,90,90,89,89,89,89,89,89,
		89,89,89,89,89,89,89,89,89,89,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,88,87,87,87,87,87,87,
		87,87,87,87,87,87,87,87,87,87,86,86,86,86,86,86,86,86,86,86,86,86,86,86,86,86,86,86,85,85,85,85,
		85,85,85,85,85,85,85,85,85,85,85,85,85,85,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,84,
		84,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,83,82,82,82,82,82,82,82,82,82,82,82,
		82,82,82,82,82,82,82,82,82,82,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,81,
		80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,79,79,79,79,79,79,79,79,79,79,
		79,79,79,79,79,79,79,79,79,79,79,79,79,79,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,78,
		78,78,78,78,78,78,78,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,77,
		76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,76,75,75,75,75,75,
		75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,74,74,74,74,74,74,74,74,74,
		74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,74,73,73,73,73,73,73,73,73,73,73,73,
		73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,73,72,72,72,72,72,72,72,72,72,72,72,72,
		72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,72,71,71,71,71,71,71,71,71,71,71,71,71,
		71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,71,70,70,70,70,70,70,70,70,70,70,70,
		70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,70,69,69,69,69,69,69,69,69,
		69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,69,68,68,68,
		68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,68,
		68,68,68,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,67,
		67,67,67,67,67,67,67,67,67,67,67,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,
		66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,66,65,65,65,65,65,65,65,65,65,65,65,65,
		65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,65,
		64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
		64,64,64,64,64,64,64,64,64,64,64,64,64,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,
		63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,63,62,62,62,62,
		62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,62,
		62,62,62,62,62,62,62,62,62,62,62,62,62,62,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,
		61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,61,
		61,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,
		60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,59,59,59,59,59,59,59,59,59,
		59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,
		59,59,59,59,59,59,59,59,59,59,59,59,59,59,59,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,
		58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,58,
		58,58,58,58,58,58,58,58,58,58,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,
		57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,57,
		57,57,57,57,57,57,57,57,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,
		56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,56,
		56,56,56,56,56,56,56,56,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,
		55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,
		55,55,55,55,55,55,55,55,55,55,55,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,
		54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,
		54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,54,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,
		53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,
		53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,53,52,52,52,52,52,52,
		52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,
		52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,52,
		52,52,52,52,52,52,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,
		51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,
		51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,51,50,50,50,50,50,50,50,50,50,50,
		50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,
		50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,
		50,50,50,50,50,50,50,50,50,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,
		49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,
		49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,49,
		48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,
		48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,
		48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,47,47,47,47,47,47,
		47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,
		47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,
		47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,47,46,46,46,46,46,46,46,
		46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,
		46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,
		46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,46,45,45,45,45,
		45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,
		45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,
		45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,45,
		45,45,45,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,
		44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,
		44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,
		44,44,44,44,44,44,44,44,44,44,44,44,44,44,44,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,
		43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,
		43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,
		43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,43,42,
		42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,
		42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,
		42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,
		42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,42,41,41,41,41,41,41,41,41,41,41,41,
		41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,
		41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,
		41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,
		41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,41,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,
		40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,
		40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,
		40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,
		40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,40,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,
		39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,
		39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,
		39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,
		39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,38,38,38,38,38,38,38,38,38,38,
		38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
		38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
		38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
		38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
		38,38,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,
		37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,
		37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,
		37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,
		37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,37,36,36,36,36,36,36,36,36,36,36,36,36,
		36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,
		36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,
		36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,
		36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,36,
		36,36,36,36,36,36,36,36,36,36,36,36,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,
		35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,
		35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,
		35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,
		35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,35,
		35,35,35,35,35,35,35,35,35,35,35,35,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,
		34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,
		34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,
		34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,
		34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,
		34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,34,33,33,33,33,33,33,33,33,33,33,33,33,33,33,
		33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,
		33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,
		33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,
		33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,
		33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,33,
		32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
		32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
		32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
		32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
		32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
		32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,31,31,31,31,31,31,31,31,31,31,31,
		31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,
		31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,
		31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,
		31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,
		31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,
		31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,30,30,30,30,30,30,30,30,30,30,30,30,30,30,
		30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,
		30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,
		30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,
		30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,
		30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,
		30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,29,29,29,29,29,29,29,29,
		29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,
		29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,
		29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,
		29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,
		29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,
		29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,
		29,29,29,29,29,29,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,
		28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,
		28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,
		28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,
		28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,
		28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,
		28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,27,27,
		27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,
		27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,
		27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,
		27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,
		27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,
		27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,
		27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,26,
		26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,
		26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,
		26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,
		26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,
		26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,
		26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,
		26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,
		26,26,26,26,26,26,26,26,26,26,26,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
		25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
		25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
		25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
		25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
		25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
		25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
		25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
		24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,
		24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,
		24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,
		24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,
		24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,
		24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,
		24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,
		24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,
		24,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
		23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
		23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
		23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
		23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
		23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
		23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
		23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
		23,23,23,23,23,23,23,23,23,23,23,23,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
		22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
		22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
		22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
		22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
		22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
		22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
		22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
		22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
		22,22,22,22,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
		21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
		21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
		21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
		21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
		21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
		21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
		21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
		21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,
		21,21,21,21,21,21,21,21,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
		20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
		20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
		20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
		20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
		20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
		20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
		20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
		20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
		20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,19,19,19,19,19,19,19,
		19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
		19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
		19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
		19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
		19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
		19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
		19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
		19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
		19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
		19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,18,18,18,18,18,18,18,18,
		18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
		18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
		18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
		18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
		18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
		18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
		18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
		18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
		18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
		18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,
		18,18,18,18,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
		16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,
		15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
		14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
		13,13,13,13,13,13,13,13,13,13,13,13,13,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,
		12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
		11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
		10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
		9,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,
		8,8,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
		7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,
		6,6,6,6,6,6,6,6,6,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,
		5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,
		4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,
		3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
		2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
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
		uint8_t* streamBufferBegin = nullptr;
		uint8_t* streamBufferIt;
		size_t streamBufferSize;
		//rans symbols also have to be written backwards, so store them in this buffer
		uint32_t* symbolBufferBegin = nullptr;
		uint32_t* symbolBufferIt;

	public:
		RansEncoder() {}
		~RansEncoder() {
			delete[] streamBufferBegin;
			delete[] symbolBufferBegin;
		}

		//for symbolBufferSize:
		//nibble model takes 1 slot, raw bits take 1 slot for every 15 bits
		void initialize_rans_encoder(const size_t _streamBufferSize, const size_t _symbolBufferSize) {
			streamBufferSize = _streamBufferSize;
			streamBufferBegin = new uint8_t[streamBufferSize];
			streamBufferIt = streamBufferBegin + streamBufferSize;
			symbolBufferBegin = new uint32_t[_symbolBufferSize];
			symbolBufferIt = symbolBufferBegin;
		}
		void start_rans() {
			stateA = RANS_NORMALIZATION_INTERVAL;
			stateB = RANS_NORMALIZATION_INTERVAL;
		}
		FORCE_INLINE void encode_nibble(const size_t symbol, NibbleModel* model) {
			uint16_t low, freq;
			model->get_range_and_update(symbol, &low, &freq);
			*symbolBufferIt++ = (low << 15) | freq;
		}
		FORCE_INLINE void encode_raw_bits(size_t symbol, size_t nBits) {

			while (nBits > 15) {
				*symbolBufferIt++ = 0x80000000 | (15 << 15) | ((symbol >> (nBits - 15)) & 0x7FFF);
				nBits -= 15;
			}
			*symbolBufferIt++ = 0x80000000 | (nBits << 15) | (symbol & (1 << nBits) - 1);
		}
		FORCE_INLINE void normalize(const size_t interval) {
			const bool renormalize = stateB >= interval;
			write_uint16le(streamBufferIt - 2, stateB);
			streamBufferIt -= renormalize * 2;
			stateB >>= renormalize * 16;
		}

		size_t end_rans(uint8_t* output) {

			do {
				std::swap(stateA, stateB);
				symbolBufferIt--;
				size_t data = *symbolBufferIt;

				//raw
				if (data >> 31) {
					const size_t nBits = (data >> 15) & 0xF;
					const size_t symbol = data & 0x7FFF;

					const size_t interval = RANS_NORMALIZATION_INTERVAL << (16 - nBits);
					normalize(interval);
					stateB = (stateB << nBits) + symbol;
				}
				//nibble
				else {
					const size_t freq = data & 0x7FFF;
					const size_t low = data >> 15;

					const size_t interval = (RANS_NORMALIZATION_INTERVAL << (16 - MODEL_PRECISION_BITS)) * freq;
					normalize(interval);
					stateB = stateB + low + (mulHigh32(stateB, RCP_FREQS[freq]) >> int_log2(freq - 1)) * ((1 << MODEL_PRECISION_BITS) - freq);
					//stateB = ((stateB / freq) << MODEL_PRECISION_BITS) + (stateB % freq) + low;
				}
			} while (symbolBufferIt != symbolBufferBegin);

			streamBufferIt -= 8;
			write_uint32le(streamBufferIt + 0, stateB);
			write_uint32le(streamBufferIt + 4, stateA);

			size_t finalBlockSize = streamBufferBegin + streamBufferSize - streamBufferIt;
			memcpy(output, streamBufferIt, finalBlockSize);
			streamBufferIt = streamBufferBegin + streamBufferSize;

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

			const size_t PREDICTOR_BLOCK_SIZE = 131072;
			const size_t numberBlocks = size / PREDICTOR_BLOCK_SIZE + (size % PREDICTOR_BLOCK_SIZE > 0);

			uint8_t* predictedBlocks = nullptr;
			try {
				predictedBlocks = new uint8_t[numberBlocks];
				dataSectors = new DataSector[numberBlocks + 1];
			}
			catch (const std::bad_alloc& e) {
				delete[] predictedBlocks;
				delete[] dataSectors;
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
					if (bestEntropy < 7.95 || bruteForce) {
						float entropyPosSize2 = calculate_entropy16(blockHistogram, thisBlockSize, 2) + 0.05;
						if (entropyPosSize2 < bestEntropy) {
							bestEntropy = entropyPosSize2;
							predictedPb = 1;
						}
					}
					if (predictedPb == 1 || bruteForce) {
						float entropyPosSize4 = calculate_entropy16(blockHistogram, thisBlockSize, 4) + 0.10;
						if (entropyPosSize4 < bestEntropy) {
							bestEntropy = entropyPosSize4;
							predictedPb = 3;
						}
					}
					if (size > 262144) {
						if (predictedPb == 3 || bruteForce) {
							float entropyPosSize8 = calculate_entropy16(blockHistogram, thisBlockSize, 8) + 0.30;
							if (entropyPosSize8 < bestEntropy) {
								bestEntropy = entropyPosSize8;
								predictedPb = 7;
							}
						}
						if (predictedPb == 7 || bruteForce) {
							float entropyPosSize16 = calculate_entropy16(blockHistogram, thisBlockSize, 16) + 0.60;
							if (entropyPosSize16 < bestEntropy) {
								bestEntropy = entropyPosSize16;
								predictedPb = 15;
							}
						}
					}
				}

				predictedBlocks[i] = predictedPb;
			}

			//Now get where each sector of pb begins
			dataSectors[0] = { data, predictedBlocks[0], 4 };
			numberSectors = 1;
			for (size_t i = 1; i < numberBlocks; i++) {
				if (predictedBlocks[i] != predictedBlocks[i - 1]) {
					dataSectors[numberSectors] = { data + i * PREDICTOR_BLOCK_SIZE, predictedBlocks[i], 4 };
					numberSectors++;
				}
			}
			dataSectors[numberSectors] = { data + size, 0, 0 };
			currentSector = 0;

			// The most problematic aspect of this method is that it tends to introduce a lot
			// of noise in the prediction. That is, if the predicted pb is supposed to be 3,
			// there will be blocks with pb = 1 or 7 sparsed throughout the file.
			// This second stage will try to remove those "random" blocks

			//Now remove that noise
			for (size_t sector = 0; sector < numberSectors; ) {
				const size_t sectorSize = dataSectors[sector + 1].start - dataSectors[sector].start;
				DataSector sectorData = dataSectors[sector];
				//Small sector
				if (sectorSize <= 262144) {

					//First sector
					if (sector == 0) {
						//Only one sector? Reduce pb
						if (numberSectors == 1) {
							if (sectorSize <= 65536)
								dataSectors[sector].pbMask = 0;
							else
								dataSectors[sector].pbMask = std::min(dataSectors[sector].pbMask, 3);
							break;
						}
						else {
							//Normally we would take the previous run, in this case take the following
							dataSectors[0].pbMask = dataSectors[1].pbMask;
							memmove(&dataSectors[1], &dataSectors[2], sizeof(DataSector) * numberSectors - 2);
							numberSectors--;
						}
					}
					else {
						//Copy the pb of the previous run
						memmove(&dataSectors[sector], &dataSectors[sector + 1], sizeof(DataSector) * numberSectors - sector - 1);
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

		double test_literal_size(const LiteralData* literalBuffer, const size_t literalCount, NibbleModel* literalModel,
			uint8_t literalContextBitShift, uint8_t positionContextBitMask) {

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
				compressedSize += FREQ_COST[freq];
				modelTree[(literal >> 4) == (exclude >> 4) ? 16 | (exclude & 0xF) : 32 | (literal >> 4)].get_range_and_update(literal & 0xF, &low, &freq);
				compressedSize += FREQ_COST[freq];
			}

			return (double)compressedSize / literalCount / STRIDER_COST_PRECISION;
		}

		template<class IntType>
		int calculate_lc(const uint8_t* input, const size_t size) {

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

					double size = test_literal_size(literalBuffer, literalCount, literalModel, k, positionContextBitMask);
					if (size < bestSize) {
						bestLc = k;
						bestSize = size;
					}
					else {
						break;
					}
				}
				dataSectors[i].lcValue = bestLc;
			}

			delete[] literalBuffer;
			delete[] literalModel;
			return 0;
		}

		void get_block_info(const uint8_t* const input, uint8_t* pb, uint8_t* lc, size_t* size) {
			if (input >= dataSectors[currentSector + 1].start)
				currentSector++;

			*pb = dataSectors[currentSector].pbMask;
			*lc = dataSectors[currentSector].lcValue;
			*size = std::min((size_t)(dataSectors[currentSector + 1].start - input), (size_t)STRIDER_MAX_BLOCK_SIZE);
		}
	};

	// too many models
	void reset_models(NibbleModel* literalRunLengthHigh, uint8_t* matchLiteralContext, NibbleModel* literalModel,
		uint8_t literalContextBitsShift, uint8_t positionContextBitMask, size_t* lastDistance, size_t* repOffsets,
		NibbleModel* distanceModel, NibbleModel* distanceLow, NibbleModel* matchLengthHigh) {

		for (size_t i = 0; i < 193; i++) { literalRunLengthHigh[i].init(); }
		*matchLiteralContext = 0b1000;  //Match, literal run
		for (size_t i = 0; i < ((48 << 8) >> literalContextBitsShift) * (positionContextBitMask + 1); i++) { literalModel[i].init(); }
		*lastDistance = 1;
		for (size_t i = 0; i < 8; i++) { repOffsets[i] = 1; }
		for (size_t i = 0; i < 24; i++) { distanceModel[i].init(); }
		for (size_t i = 0; i < 2; i++) { distanceLow[i].init(); }
		for (size_t i = 0; i < 148; i++) { matchLengthHigh[i].init(); }
	}

	void store_block_header(size_t blockSize, uint8_t positionContextBitMask, uint8_t literalContextBitShift,
		bool resetModels, bool uncompressedBlock, uint8_t*& output)
	{
		write_uint16le(output, blockSize - 1);
		uint8_t metadata = uncompressedBlock | (resetModels << 1);
		metadata |= (int_log2(positionContextBitMask + 1) * 9 + literalContextBitShift) << 2;
		output[2] = metadata;
		output += 3;
	}

	FORCE_INLINE void strider_encode_literal_run(RansEncoder* encoder, const uint8_t* literalRun,
		NibbleModel* literalRunLengthHigh, size_t literalRunLength, uint8_t* matchLiteralContext,
		NibbleModel* literalModel, const size_t lastDistance, const uint8_t literalContextBitsShift,
		const uint8_t positionContextBitMask, size_t* positionContext) {

		*positionContext = reinterpret_cast<size_t>(literalRun) & positionContextBitMask;

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
			uint8_t literal = literalRun[-1];

			do {
				uint8_t exclude = *(literalRun - lastDistance);
				NibbleModel* literalModelTree =
					&literalModel[(((*positionContext << 8) | literal) >> literalContextBitsShift) * 48];
				literal = *literalRun;

				encoder->encode_nibble(literal >> 4, &literalModelTree[exclude >> 4]);
				encoder->encode_nibble(literal & 0xF,
					&literalModelTree[(exclude >> 4) == (literal >> 4) ? 32 | (exclude & 0xF) : 16 | (literal >> 4)]);

				literalRun++;
				*positionContext = reinterpret_cast<size_t>(literalRun) & positionContextBitMask;
				literalRunLength--;
			} while (literalRunLength);
		}
	}

	FORCE_INLINE void encode_match_length(RansEncoder* encoder, const size_t matchLengthContext,
		const uint8_t positionContext, NibbleModel* matchLengthHigh, size_t matchLength)
	{
		if (matchLength > 16) {
			encoder->encode_nibble(15, &matchLengthHigh[matchLengthContext * 16 | positionContext]);
			if (matchLength > 31) {
				encoder->encode_nibble(15, &matchLengthHigh[144 + (matchLengthContext != 0)]);
				matchLength -= 31;
				size_t symbol = int_log2(matchLength);
				encoder->encode_nibble(symbol, &matchLengthHigh[146 + (matchLengthContext != 0)]);
				encoder->encode_raw_bits(matchLength & ((size_t)1 << symbol) - 1, symbol);
			}
			else {
				matchLength -= 17;
				encoder->encode_nibble(matchLength, &matchLengthHigh[144 + (matchLengthContext != 0)]);
			}
		}
		else {
			matchLength -= STRIDER_MIN_LENGTH;
			encoder->encode_nibble(matchLength, &matchLengthHigh[matchLengthContext * 16 | positionContext]);
		}
	}

	FORCE_INLINE void strider_encode_match(RansEncoder* encoder, uint8_t* matchLiteralContext,
		const uint8_t positionContext, size_t* repOffsets, NibbleModel* distanceModel, NibbleModel* distanceLow,
		NibbleModel* matchLengthHigh, size_t matchLength, size_t distance)
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

		encode_match_length(encoder, matchLengthContext, positionContext, matchLengthHigh, matchLength);
	}

	FORCE_INLINE void strider_encode_last_distance_match(RansEncoder* encoder, uint8_t* matchLiteralContext,
		const uint8_t positionContext, size_t* repOffsets, NibbleModel* distanceModel,
		NibbleModel* matchLengthHigh, size_t matchLength, size_t distance)
	{

		size_t rep = distance == repOffsets[0] ? 0 : 5;
		encoder->encode_nibble(rep, &distanceModel[*matchLiteralContext]);
		*matchLiteralContext = (*matchLiteralContext >> 2) | 4;

		for (; rep > 0; rep--)
			repOffsets[rep] = repOffsets[rep - 1];
		repOffsets[0] = distance;

		encode_match_length(encoder, 0, positionContext, matchLengthHigh, matchLength);
	}

	template<class IntType>
	size_t strider_compress_greedy(const uint8_t* input, const size_t size, uint8_t* output,
		const CompressorOptions& compressorOptions, ProgressCallback* progress, const int window) {

		RansEncoder encoder;
		encoder.initialize_rans_encoder(std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 1.25 + 32,
			std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 3.0 + 64);

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. 
		*output++ = *input++;

		DataDetector dataDetector;
		if (dataDetector.calculate_pb(input, size - STRIDER_LAST_BYTES - 1, false))
			return 0;

		//Model stuff
		bool doModelReset = true;
		NibbleModel literalRunLengthHigh[193];
		NibbleModel* literalModel = nullptr;
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthHigh[148];

		uint8_t matchLiteralContext = 0;
		uint8_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		uint8_t positionContextBitMask = -1;
		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

		const size_t hashLog = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
		HashTable<IntType, FastIntHash> lzdict;
		try {
			lzdict.init(hashLog);
		}
		catch (std::bad_alloc& e) {
			return -1;
		}

		for (; input < compressionLimit; ) {

			uint8_t* const compressedBlockStart = output;
			const uint8_t* const thisBlockStart = input;

			size_t thisBlockSize;
			uint8_t newLiteralContextBitShift, newPositionContextBitMask;
			dataDetector.get_block_info(input, &newPositionContextBitMask, &newLiteralContextBitShift, &thisBlockSize);
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//The size of the literal model has been modified
			if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {

				delete[] literalModel;
				try {
					literalModel = new NibbleModel[((48 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
				}
				catch (const std::bad_alloc& e) {
					return -1;
				}
			}

			//We have to reset everything
			if (doModelReset) {
				reset_models(literalRunLengthHigh, &matchLiteralContext, literalModel, newLiteralContextBitShift,
					newPositionContextBitMask, &distance, repOffsets, distanceModel, distanceLow, matchLengthHigh);
			}
			//Only reset certain models if needed
			else {
				//These only require position bit mask
				if (positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < 192; i++)
						literalRunLengthHigh[i].init();
					for (size_t i = 0; i < 144; i++)
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

					strider_encode_literal_run(&encoder, literalRunStart, literalRunLengthHigh, literalRunLength, &matchLiteralContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

					//Only update 1 position. This is enough
					lzdict[read_hash6(input)] = input - inputStart;
					input += matchLength;

					literalRunStart = input;
					strider_encode_last_distance_match(&encoder, &matchLiteralContext, positionContext, repOffsets,
						distanceModel, matchLengthHigh, matchLength, distance);
					continue;
				}

				//If no rep, try a normal match
				IntType* const dictEntry = &lzdict[read_hash6(input)];
				const uint8_t* match = inputStart + *dictEntry;
				*dictEntry = input - inputStart;
				matchLength = test_match(input, match, thisBlockEnd, 6, window);

				if (matchLength) {

					//Update 6 positions.
					lzdict[read_hash6(input + 1)] = input + 1 - inputStart;
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

					strider_encode_literal_run(&encoder, literalRunStart, literalRunLengthHigh, literalRunLength, &matchLiteralContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

					distance = input - match;
					input += matchLength;

					literalRunStart = input;
					strider_encode_match(&encoder, &matchLiteralContext, positionContext, repOffsets,
						distanceModel, distanceLow, matchLengthHigh, matchLength, distance);
				}
				else {
					input++;
				}
			}

			size_t positionContext;
			size_t literalRunLength = input - literalRunStart;
			strider_encode_literal_run(&encoder, literalRunStart, literalRunLengthHigh, literalRunLength, &matchLiteralContext,
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
	template<class IntType>
	FORCE_INLINE void strider_fast_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const blockEnd,
		LZ2WayCacheTable<IntType, FastIntHash>* lzdict, size_t* bestMatchLength, size_t* bestMatchDistance,
		int* lazySteps, int* testedPositions, const size_t repOffset, const int window, const CompressorOptions& compressorOptions) {

		//Check for a rep match at the next position. Gives better results than searching at current
		*bestMatchLength = test_match(input + 1, input + 1 - repOffset, blockEnd, 3, window);
		if (*bestMatchLength) {
			*bestMatchDistance = repOffset;
			*testedPositions = 0;
			*lazySteps = 1;
			return;
		}

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
	FORCE_INLINE void strider_lazy_search(const uint8_t* input, const uint8_t* const inputStart, const uint8_t* const limit,
		LZCacheTable<IntType, FastIntHash>* lzdict4, LZCacheTable<IntType, FastIntHash>* lzdict8, size_t* bestLength,
		size_t* bestDistance, const size_t repOffset, int* lazySteps, int* testedPositions,
		const CompressorOptions& compressorOptions, const int window) {

		//First try to find a match in the rep offset. If it is found simply take it
		*bestLength = test_match(input + 1, input + 1 - repOffset, limit, 3, window);
		if (*bestLength) {
			*bestDistance = repOffset;
			*testedPositions = 0;
			*lazySteps = 1;
			return;
		}

		LZCacheBucket<IntType> chain4 = (*lzdict4)[read_hash4(input)];
		LZCacheBucket<IntType> chain8 = (*lzdict8)[read_hash8(input)];
		size_t pos = input - inputStart;
		*testedPositions = 1;
		*lazySteps = 0;
		size_t bestMatchCost = 0;
		size_t length;

		//If no rep offset was found try to get a length 8 match
		while (!chain8.ended()) {
			chain8.next(&pos);

			const uint8_t* where = inputStart + pos;

			if (*(input + *bestLength) != *(where + *bestLength))
				continue;

			length = test_match(input, where, limit, 8, window);

			size_t distance = input - where;
			size_t matchCost = 2 + unsafe_int_log2(distance) / 4;
			if (length + bestMatchCost > matchCost + *bestLength) {
				*bestDistance = distance;
				*bestLength = length;
				bestMatchCost = matchCost;

				if (*bestLength >= compressorOptions.niceLength) {
					while (!chain8.ended())
						chain8.next(&pos);
					chain4.push(input - inputStart);
					return;
				}
			}
		}

		//If still nothing was found, try a length 4
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
					//We did not found any length 8 earlier, so we wont find anything better than 7 now
					if (*bestLength >= 7) {
						if (*bestLength >= compressorOptions.niceLength) {
							while (!chain4.ended())
								chain4.next(&pos);
							return;
						}
						break;
					}
				}
			}

			//No match found, code a literal and retry
			if (*bestLength == 0) {
				*lazySteps = 1;
				return;
			}
		}
		else {
			chain4.push(input - inputStart);
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

			size_t distance = input - where;
			size_t matchCost = 2 + unsafe_int_log2(distance) / 4;
			if (length + bestMatchCost > matchCost + *bestLength) {
				*bestDistance = distance;
				*bestLength = length;
				bestMatchCost = matchCost;
				*lazySteps = 1;

				if (*bestLength >= compressorOptions.niceLength) {
					while (!chain8.ended())
						chain8.next(&pos);
					return;
				}
			}
		}
	}

	template<class IntType>
	size_t strider_compress_lazy(const uint8_t* input, const size_t size, uint8_t* output,
		const CompressorOptions& compressorOptions, ProgressCallback* progress, const int window) {

		RansEncoder encoder;
		encoder.initialize_rans_encoder(std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 1.25 + 32,
			std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 3.0 + 64);

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. Its progress will be reported at the end
		*output++ = *input++;

		DataDetector dataDetector;
		if (dataDetector.calculate_pb(input, size - STRIDER_LAST_BYTES - 1, false))
			return 0;

		//Model stuff
		bool doModelReset = true;
		NibbleModel literalRunLengthHigh[193];
		NibbleModel* literalModel = nullptr;
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthHigh[148];

		uint8_t matchLiteralContext = 0;
		uint8_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		uint8_t positionContextBitMask = -1;
		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

		const size_t hashSize = MIN3((int)int_log2(size) - 3, compressorOptions.maxHashTableSize, window - 3);
		LZ2WayCacheTable<IntType, FastIntHash> way2lzdict;
		LZCacheTable<IntType, FastIntHash> lzdict4;
		LZCacheTable<IntType, FastIntHash> lzdict8;
		try {
			if (compressorOptions.parserFunction == LAZY_NORMAL) {
				way2lzdict.init(hashSize);
			}
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
			uint8_t newLiteralContextBitShift, newPositionContextBitMask;
			dataDetector.get_block_info(input, &newPositionContextBitMask, &newLiteralContextBitShift, &thisBlockSize);
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//The size of the literal model has been modified
			if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {

				delete[] literalModel;
				try {
					literalModel = new NibbleModel[((48 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
				}
				catch (const std::bad_alloc& e) {
					return -1;
				}
			}

			//We have to reset everything
			if (doModelReset) {
				reset_models(literalRunLengthHigh, &matchLiteralContext, literalModel, newLiteralContextBitShift,
					newPositionContextBitMask, &distance, repOffsets, distanceModel, distanceLow, matchLengthHigh);
			}
			//Only reset certain models if needed
			else {
				//These only require position bit mask
				if (positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < 192; i++)
						literalRunLengthHigh[i].init();
					for (size_t i = 0; i < 144; i++)
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
			encoder.start_rans();

			doModelReset = false;
			const uint8_t* literalRunStart = input;

			for (; input < thisBlockEnd; ) {

				size_t matchLength;
				size_t newDistance;
				int lazySteps;   //bytes to skip because of lazy matching
				int testedPositions;   //number of positions that have been added to hash table

				if (compressorOptions.parserFunction == LAZY_NORMAL) {
					strider_fast_lazy_search<IntType>(input, inputStart, thisBlockEnd, &way2lzdict,
						&matchLength, &newDistance, &lazySteps, &testedPositions, distance, window, compressorOptions);
				}
				else {
					strider_lazy_search<IntType>(input, inputStart, thisBlockEnd, &lzdict4, &lzdict8, &matchLength,
						&newDistance, distance, &lazySteps, &testedPositions, compressorOptions, window);
				}

				input += lazySteps;

				//We have found a match
				if (matchLength) {

					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;

					strider_encode_literal_run(&encoder, literalRunStart, literalRunLengthHigh, literalRunLength, &matchLiteralContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

					//Update the hash table for every position
					const uint8_t* const matchEnd = input + matchLength;
					input = distance < matchLength ? matchEnd - distance : input + testedPositions - lazySteps;
					if (compressorOptions.parserFunction == LAZY_NORMAL) {
						for (; input < matchEnd; input++)
							way2lzdict[read_hash6(input)].push_in_first(input - inputStart);
					}
					else {
						for (; input < matchEnd; input++) {
							lzdict4[read_hash4(input)].push(input - inputStart);
							lzdict8[read_hash8(input)].push(input - inputStart);
						}
					}

					distance = newDistance;
					literalRunStart = input;
					//Output the match
					strider_encode_match(&encoder, &matchLiteralContext, positionContext, repOffsets,
						distanceModel, distanceLow, matchLengthHigh, matchLength, distance);
				}
			}

			size_t positionContext;
			size_t literalRunLength = input - literalRunStart;
			strider_encode_literal_run(&encoder, literalRunStart, literalRunLengthHigh, literalRunLength, &matchLiteralContext,
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

	template<class IntType>
	struct StriderFastOptimalState {
		uint32_t sizeCost;
		uint16_t literalRunLength;
		uint16_t matchLength;
		IntType repOffsets[2];   //Match distance will always be the first offset stored here
	};

	template<class IntType>
	LZStructure<IntType>* strider_forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart, const uint8_t* const blockLimit,
		HashTableMatchFinder<IntType>* matchFinder, StriderFastOptimalState<IntType>* parser, LZStructure<IntType>* stream,
		size_t* repOffsets, const CompressorOptions& compressorOptions, const int window) {

		const size_t blockLength = std::min((size_t)(blockLimit - input), (size_t)compressorOptions.optimalBlockSize);
		for (size_t i = 1; i <= blockLength; i++)
			parser[i].sizeCost = UINT32_MAX;

		size_t lastMatchLength = 0;
		size_t lastMatchDistance;
		size_t lastMatchStart;

		parser[0].sizeCost = 0;
		parser[0].repOffsets[0] = repOffsets[0];
		parser[0].repOffsets[1] = repOffsets[5];
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
				if (position + highestLength >= blockLength || highestLength >= compressorOptions.niceLength / 2) {
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
				//Match goes outside the buffer or is very long
				if (position + longestMatch->length >= blockLength || longestMatch->length >= compressorOptions.niceLength) {
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
			for (inputPosition++; inputPosition < matchEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart);
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
		uint16_t matchLength;
		uint16_t literalRunLength;
		uint16_t currentLiteralRunLengthCost;  //Cache so it does not get computed twice
		uint8_t matchLiteralContext;
		uint8_t prevPath;
		IntType distance;           //Also used as last distance
		IntType repOffsets[8];
	};

	FORCE_INLINE size_t get_literal_run_cost(size_t literalRunLength, const size_t positionContext, const size_t matchLiteralContext,
		NibbleModel* literalRunLengthHigh) {

		if (literalRunLength >= 15) {
			literalRunLength -= 14;
			size_t symbol = int_log2(literalRunLength);
			return FREQ_COST[literalRunLengthHigh[matchLiteralContext * 16 | positionContext].get_freq(15)]
				+ FREQ_COST[literalRunLengthHigh[192].get_freq(symbol)] + symbol * STRIDER_COST_PRECISION;
		}
		else
			return FREQ_COST[literalRunLengthHigh[matchLiteralContext * 16 | positionContext].get_freq(literalRunLength)];
	}

	FORCE_INLINE size_t get_match_length_cost(size_t length, const size_t positionContext, const size_t matchLengthContext,
		NibbleModel* matchLengthHigh) {

		if (length > 16) {
			size_t cost = FREQ_COST[matchLengthHigh[matchLengthContext * 16 | positionContext].get_freq(15)];
			if (length > 31) {
				length -= 31;
				size_t logarithm = int_log2(length);
				cost += FREQ_COST[matchLengthHigh[144 + (matchLengthContext != 0)].get_freq(15)];
				cost += FREQ_COST[matchLengthHigh[146 + (matchLengthContext != 0)].get_freq(logarithm)];
				cost += STRIDER_COST_PRECISION * logarithm;
				return cost;
			}
			else {
				length -= 17;
				return cost + FREQ_COST[matchLengthHigh[144 + (matchLengthContext != 0)].get_freq(length)];
			}
		}
		else {
			length -= STRIDER_MIN_LENGTH;
			return FREQ_COST[matchLengthHigh[matchLengthContext * 16 | positionContext].get_freq(length)];
		}
	}

	FORCE_INLINE size_t get_distance_cost(size_t distance, NibbleModel* distanceModel,
		NibbleModel* distanceLow, const size_t matchLiteralContext) {

		size_t cost = 0;

		distance--;
		if (distance < 256) {
			size_t symbol = distance / 16;
			cost += FREQ_COST[distanceModel[matchLiteralContext].get_freq(8)];
			cost += FREQ_COST[distanceModel[16].get_freq(symbol)];
			cost += FREQ_COST[distanceLow[symbol != 0].get_freq(distance & 0xF)];
		}
		else {
			size_t logarithm = int_log2(distance);
			size_t rawBits = logarithm - 5;
			size_t symbol = logarithm * 2 + ((distance >> (logarithm - 1)) & 1);

			size_t symbolHigh = symbol >> 4;
			cost += FREQ_COST[distanceModel[matchLiteralContext].get_freq(8 | symbolHigh)];
			size_t symbolLow = symbol & 0xF;
			cost += FREQ_COST[distanceModel[16 | symbolHigh].get_freq(symbolLow)];
			cost += FREQ_COST[distanceLow[1].get_freq(distance & 0xF)];
			cost += rawBits * STRIDER_COST_PRECISION;
		}

		return cost;
	}

	template<class IntType>
	LZStructure<IntType>* strider_priced_forward_optimal_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, const uint8_t* const blockLimit, BinaryMatchFinder<IntType>* matchFinder,
		StriderOptimalParserState<IntType>* parser, LZStructure<IntType>* stream, const CompressorOptions& compressorOptions,
		NibbleModel* literalRunLengthHigh, size_t startingLiteralRunLength, size_t matchLiteralContext,
		NibbleModel* literalModel, uint8_t literalContextBitsShift, uint8_t positionContextBitMask, size_t lastDistance,
		size_t* repOffsets, NibbleModel* distanceModel, NibbleModel* distanceLow, NibbleModel* matchLengthHigh, const int window) {

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
		parser[0].currentLiteralRunLengthCost =
			get_literal_run_cost(startingLiteralRunLength, reinterpret_cast<size_t>(input - startingLiteralRunLength) & positionContextBitMask,
				matchLiteralContext, literalRunLengthHigh);
		parser[0].matchLiteralContext = matchLiteralContext;

		size_t position = 0;
		for (; position < blockLength; position++) {

			const uint8_t* inputPosition = input + position;
			StriderOptimalParserState<IntType>* parserPosition = parser + position;
			const size_t positionContext = reinterpret_cast<size_t>(inputPosition) & positionContextBitMask;

			const size_t literal = *inputPosition;
			const size_t exclude = *(inputPosition - parserPosition->distance);

			NibbleModel* const literalModelTree = &literalModel[(((positionContext << 8) | inputPosition[-1]) >> literalContextBitsShift) * 48];
			size_t thisLiteralSize = FREQ_COST[literalModelTree[exclude >> 4].get_freq(literal >> 4)];
			thisLiteralSize += FREQ_COST[literalModelTree[(exclude >> 4) == (literal >> 4) ? 32 | (exclude & 0xF) : 16 | (literal >> 4)].get_freq(literal & 0xF)];

			const size_t newLiteralRunLengthCost =
				get_literal_run_cost(parserPosition->literalRunLength + 1,
					reinterpret_cast<size_t>(inputPosition - parserPosition->literalRunLength) & positionContextBitMask,
					parserPosition->matchLiteralContext, literalRunLengthHigh);

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

			// Test all reps, but only take the longest one
			for (size_t i = 0; i < 8; i++) {
				size_t repMatchLength = test_match(inputPosition, inputPosition - parserPosition->repOffsets[i], blockLimit, 2, window);
				if (repMatchLength > highestLength) {
					if (repMatchLength >= compressorOptions.niceLength / 2 || position + repMatchLength >= blockLength) {
						lastMatchLength = repMatchLength;
						lastMatchDistance = parserPosition->repOffsets[i];
						lastMatchStart = position;
						goto doBackwardParse;
					}
					highestLength = repMatchLength;
					whichRep = i;
				}
			}

			if (highestLength >= 2) {
				size_t repDistance = parserPosition->repOffsets[whichRep];

				size_t currentMatchLiteralContext = parserPosition->matchLiteralContext >> (parserPosition->literalRunLength > 0) * 2;  //After literal run
				const size_t offsetCost = FREQ_COST[distanceModel[currentMatchLiteralContext].get_freq(whichRep)];
				currentMatchLiteralContext = (currentMatchLiteralContext >> 2) | 4;  //After rep match

				const size_t lengthCost = get_match_length_cost(highestLength, positionContext, 0, matchLengthHigh);
				const size_t nextLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + highestLength) & positionContextBitMask;
				const size_t literalRunLengthCost = FREQ_COST[literalRunLengthHigh[currentMatchLiteralContext * 16 | nextLiteralRunPositionContext].get_freq(0)];
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

			LZMatch<IntType> matches[258];
			const LZMatch<IntType>* matchesEnd = matchFinder->find_matches_and_update(inputPosition, inputStart, limit,
				blockLimit, matches, highestLength, compressorOptions, window);

			//At least one match was found
			if (matchesEnd != matches) {
				//The last match should be the longest
				const LZMatch<IntType>* const longestMatch = matchesEnd - 1;
				//Match goes outside the buffer or is very long
				if (position + longestMatch->length >= blockLength || longestMatch->length >= compressorOptions.niceLength) {
					lastMatchLength = longestMatch->length;
					lastMatchDistance = longestMatch->distance;
					lastMatchStart = position;
					goto doBackwardParse;
				}

				const size_t currentMatchLiteralContext = parserPosition->matchLiteralContext >> (parserPosition->literalRunLength > 0) * 2;  //After literal run
				const size_t nextMatchLiteralContext = (currentMatchLiteralContext >> 2) | 8;  //After match

				for (const LZMatch<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					const size_t matchLengthContext = 1 + int_log2((size_t)matchIt->distance) / 8;
					const size_t lengthCost = get_match_length_cost(matchIt->length, positionContext, matchLengthContext, matchLengthHigh);
					const size_t distanceCost = get_distance_cost(matchIt->distance, distanceModel, distanceLow, currentMatchLiteralContext);
					const size_t nextLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + matchIt->length) & positionContextBitMask;
					const size_t literalRunLengthCost = FREQ_COST[literalRunLengthHigh[nextMatchLiteralContext * 16 | nextLiteralRunPositionContext].get_freq(0)];
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
			for (inputPosition++; inputPosition < matchEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions, window);
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
	LZStructure<IntType>* strider_multi_arrivals_parse(const uint8_t* const input, const uint8_t* const inputStart,
		const uint8_t* const limit, const uint8_t* const blockLimit, BinaryMatchFinder<IntType>* matchFinder,
		StriderOptimalParserState<IntType>* parser, LZStructure<IntType>* stream, const CompressorOptions& compressorOptions,
		NibbleModel* literalRunLengthHigh, size_t startingLiteralRunLength, size_t matchLiteralContext,
		NibbleModel* literalModel, size_t literalContextBitsShift, size_t positionContextBitMask, size_t lastDistance,
		size_t* repOffsets, NibbleModel* distanceModel, NibbleModel* distanceLow, NibbleModel* matchLengthHigh, const int window) {

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
			parser[i].currentLiteralRunLengthCost =
				get_literal_run_cost(startingLiteralRunLength, reinterpret_cast<size_t>(input - startingLiteralRunLength) & positionContextBitMask,
					matchLiteralContext, literalRunLengthHigh);
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
			NibbleModel* const literalModelTree = &literalModel[(((positionContext << 8) | inputPosition[-1]) >> literalContextBitsShift) * 48];

			//Literal and rep match parsing can be done at the same time
			for (size_t i = 0; i < compressorOptions.maxArrivals; i++) {

				StriderOptimalParserState<IntType>* currentArrival = parserPosition + i;

				const size_t literal = *inputPosition;
				const size_t exclude = *(inputPosition - currentArrival->distance);

				size_t thisLiteralSize = FREQ_COST[literalModelTree[exclude >> 4].get_freq(literal >> 4)];
				thisLiteralSize += FREQ_COST[literalModelTree[(exclude >> 4) == (literal >> 4) ? 32 | (exclude & 0xF) : 16 | (literal >> 4)].get_freq(literal & 0xF)];

				const size_t newLiteralRunLengthCost =
					get_literal_run_cost(currentArrival->literalRunLength + 1,
						reinterpret_cast<size_t>(inputPosition - currentArrival->literalRunLength) & positionContextBitMask,
						currentArrival->matchLiteralContext, literalRunLengthHigh);

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

					size_t repMatchLength = test_match(inputPosition, inputPosition - currentArrival->repOffsets[j], blockLimit, 2, window);
					if (repMatchLength >= acceptableRepMatchLength) {
						size_t repDistance = currentArrival->repOffsets[j];

						if (repMatchLength >= compressorOptions.niceLength / 2 || position + repMatchLength >= blockLength) {
							lastMatchLength = repMatchLength;
							lastMatchDistance = repDistance;
							lastMatchStart = position;
							lastPath = i;
							goto doBackwardParse;
						}

						size_t currentMatchLiteralContext = currentArrival->matchLiteralContext >> (currentArrival->literalRunLength > 0) * 2;
						const size_t baseMatchCost = currentArrival->sizeCost + FREQ_COST[distanceModel[currentMatchLiteralContext].get_freq(j)];
						currentMatchLiteralContext = (currentMatchLiteralContext >> 2) | 4;

						size_t matchLengthReductionLimit = repMatchLength - std::min((size_t)3, repMatchLength - acceptableRepMatchLength);
						for (size_t length = repMatchLength; length >= matchLengthReductionLimit; length--) {

							const size_t lengthCost = get_match_length_cost(length, positionContext, 0, matchLengthHigh);
							const size_t newLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + length) & positionContextBitMask;
							const size_t literalRunLengthCost = FREQ_COST[literalRunLengthHigh[currentMatchLiteralContext * 16 | newLiteralRunPositionContext].get_freq(0)];
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

			LZMatch<IntType> matches[258];
			const LZMatch<IntType>* matchesEnd =
				matchFinder->find_matches_and_update(inputPosition, inputStart, limit, blockLimit, matches, 1, compressorOptions, window);

			//At least one match was found
			if (matchesEnd != matches) {

				if (compressorOptions.parserFunction == OPTIMAL_ULTRA) {
					//Longest length can be anywhere
					size_t longestLength = 0;
					size_t distance;
					for (LZMatch<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {
						if (matchIt->length > longestLength) {
							longestLength = matchIt->length;
							distance = matchIt->distance;
						}
					}
					if (position + longestLength >= blockLength || longestLength >= compressorOptions.niceLength) {
						lastMatchLength = longestLength;
						lastMatchDistance = distance;
						lastMatchStart = position;
						lastPath = 0;
						goto doBackwardParse;
					}
				}
				else {
					//The last match should be the longest
					const LZMatch<IntType>* const longestMatch = matchesEnd - 1;
					//Match goes outside the buffer or is very long
					if (position + longestMatch->length >= blockLength || longestMatch->length >= compressorOptions.niceLength) {
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

				for (const LZMatch<IntType>* matchIt = matches; matchIt != matchesEnd; matchIt++) {

					if (matchIt->distance == parserPosition->repOffsets[0] || matchIt->distance == parserPosition->repOffsets[1] ||
						matchIt->distance == parserPosition->repOffsets[2] || matchIt->distance == parserPosition->repOffsets[3] ||
						matchIt->distance == parserPosition->repOffsets[4] || matchIt->distance == parserPosition->repOffsets[5] ||
						matchIt->distance == parserPosition->repOffsets[6] || matchIt->distance == parserPosition->repOffsets[7])
						continue;

					const size_t matchLengthContext = 1 + int_log2((size_t)matchIt->distance) / 8;
					const size_t baseMatchCost = parserPosition->sizeCost + get_distance_cost(matchIt->distance, distanceModel, distanceLow, currentMatchLiteralContext);

					if (compressorOptions.parserFunction == OPTIMAL_ULTRA)
						matchLengthReductionLimit = matchIt->length - std::min((size_t)4, (size_t)matchIt->length - 1);
					else
						matchLengthReductionLimit = matchIt->length - std::min((size_t)4, matchIt->length - matchLengthReductionLimit);
					//Start search at the highest length. Stop when we reach the previous length or
					// the current match length does not improve any arrival.
					for (size_t length = matchIt->length; length > matchLengthReductionLimit; length--) {

						const size_t lengthCost = get_match_length_cost(length, positionContext, matchLengthContext, matchLengthHigh);
						const size_t newLiteralRunPositionContext = reinterpret_cast<size_t>(inputPosition + length) & positionContextBitMask;
						const size_t literalRunLengthCost = FREQ_COST[literalRunLengthHigh[nextMatchLiteralContext * 16 | newLiteralRunPositionContext].get_freq(0)];
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
			for (inputPosition++; inputPosition < matchEnd; inputPosition++)
				matchFinder->update_position(inputPosition, inputStart, limit, compressorOptions, window);
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
	size_t strider_compress_optimal(const uint8_t* input, const size_t size, uint8_t* output,
		const CompressorOptions& compressorOptions, ProgressCallback* progress, const int window) {

		RansEncoder encoder;
		encoder.initialize_rans_encoder(std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 1.25 + 32,
			std::min((size_t)STRIDER_MAX_BLOCK_SIZE, size) * 3.0 + 64);

		const uint8_t* const inputStart = input;
		const uint8_t* const outputStart = output;
		const uint8_t* const compressionLimit = input + size - STRIDER_LAST_BYTES;
		//Store first byte uncompressed. Its progress will be reported at the end
		*output++ = *input++;

		DataDetector dataDetector;
		if (dataDetector.calculate_pb(input, size - STRIDER_LAST_BYTES - 1, true))
			return 0;
		if (dataDetector.calculate_lc<IntType>(input, size))
			return 0;

		//Model stuff
		bool doModelReset = true;
		NibbleModel literalRunLengthHigh[193];
		NibbleModel* literalModel = nullptr;
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthHigh[148];

		uint8_t matchLiteralContext = 0;
		uint8_t literalContextBitsShift = -1;   //The right shift on the previous byte context for literals
		uint8_t positionContextBitMask = -1;
		size_t distance = 1;  //Also used to store the last distance
		size_t repOffsets[8] = { 1, 1, 1, 1, 1, 1, 1, 1 };

		HashTableMatchFinder<IntType> hashMatchFinder;
		BinaryMatchFinder<IntType> binaryMatchFinder;
		char* parser = nullptr;
		LZStructure<IntType>* stream = nullptr;
		try {
			if (compressorOptions.parserFunction == OPTIMAL_FAST) {
				hashMatchFinder.init(size, compressorOptions, window);
				parser = (char*)(new StriderFastOptimalState<IntType>[compressorOptions.optimalBlockSize + 1]);
			}
			else {
				binaryMatchFinder.init(size, compressorOptions, window);
				if (compressorOptions.parserFunction == OPTIMAL)
					parser = (char*)(new StriderOptimalParserState<IntType>[compressorOptions.optimalBlockSize + 1]);
				else
					parser = (char*)(new StriderOptimalParserState<IntType>[(compressorOptions.optimalBlockSize + 1) * compressorOptions.maxArrivals]);
			}
			stream = new LZStructure<IntType>[compressorOptions.optimalBlockSize];
		}
		catch (const std::bad_alloc& e) {
			delete[] parser;
			delete[] stream;
			return -1;
		}

		for (; input < compressionLimit; ) {

			uint8_t* const compressedBlockStart = output;
			const uint8_t* const thisBlockStart = input;

			size_t thisBlockSize;
			uint8_t newLiteralContextBitShift, newPositionContextBitMask;
			dataDetector.get_block_info(input, &newPositionContextBitMask, &newLiteralContextBitShift, &thisBlockSize);
			const uint8_t* const thisBlockEnd = input + thisBlockSize;

			//The size of the literal model has been modified
			if (literalContextBitsShift != newLiteralContextBitShift || positionContextBitMask != newPositionContextBitMask) {

				delete[] literalModel;
				try {
					literalModel = new NibbleModel[((48 << 8) >> newLiteralContextBitShift) * (newPositionContextBitMask + 1)];
				}
				catch (const std::bad_alloc& e) {
					delete[] parser;
					delete[] stream;
					return -1;
				}
			}

			//We have to reset everything
			if (doModelReset) {
				reset_models(literalRunLengthHigh, &matchLiteralContext, literalModel, newLiteralContextBitShift,
					newPositionContextBitMask, &distance, repOffsets, distanceModel, distanceLow, matchLengthHigh);
			}
			//Only reset certain models if needed
			else {
				//These only require position bit mask
				if (positionContextBitMask != newPositionContextBitMask) {
					for (size_t i = 0; i < 192; i++)
						literalRunLengthHigh[i].init();
					for (size_t i = 0; i < 144; i++)
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
			encoder.start_rans();

			doModelReset = false;
			const uint8_t* literalRunStart = input;

			for (; input < thisBlockEnd; ) {

				LZStructure<IntType>* streamIt;
				if (compressorOptions.parserFunction == OPTIMAL_FAST) {
					streamIt = strider_forward_optimal_parse<IntType>(input, inputStart, thisBlockEnd, &hashMatchFinder,
						(StriderFastOptimalState<IntType>*)parser, stream, repOffsets, compressorOptions, window);
				}
				else if (compressorOptions.parserFunction == OPTIMAL) {
					streamIt = strider_priced_forward_optimal_parse<IntType>(input, inputStart, compressionLimit, thisBlockEnd, &binaryMatchFinder,
						(StriderOptimalParserState<IntType>*)parser, stream, compressorOptions, literalRunLengthHigh, input - literalRunStart,
						matchLiteralContext, literalModel, literalContextBitsShift, positionContextBitMask, distance, repOffsets,
						distanceModel, distanceLow, matchLengthHigh, window);
				}
				else {
					streamIt = strider_multi_arrivals_parse<IntType>(input, inputStart, compressionLimit, thisBlockEnd, &binaryMatchFinder,
						(StriderOptimalParserState<IntType>*)parser, stream, compressorOptions, literalRunLengthHigh, input - literalRunStart,
						matchLiteralContext, literalModel, literalContextBitsShift, positionContextBitMask, distance, repOffsets,
						distanceModel, distanceLow, matchLengthHigh, window);
				}

				//Main compression loop
				while (true) {
					input += streamIt->literalRunLength;

					if (streamIt == stream)
						break;

					size_t positionContext;
					size_t literalRunLength = input - literalRunStart;

					strider_encode_literal_run(&encoder, literalRunStart, literalRunLengthHigh, literalRunLength, &matchLiteralContext,
						literalModel, distance, literalContextBitsShift, positionContextBitMask, &positionContext);

					size_t matchLength = streamIt->matchLength;
					distance = streamIt->matchDistance;
					input += matchLength;
					literalRunStart = input;
					//Output the match
					strider_encode_match(&encoder, &matchLiteralContext, positionContext, repOffsets,
						distanceModel, distanceLow, matchLengthHigh, matchLength, distance);

					streamIt--;
				}
			}

			size_t positionContext;
			size_t literalRunLength = input - literalRunStart;
			strider_encode_literal_run(&encoder, literalRunStart, literalRunLengthHigh, literalRunLength, &matchLiteralContext,
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

			if (progress->abort()) {
				delete[] parser;
				delete[] stream;
				delete[] literalModel;
				return 0;
			}
			progress->progress(input - inputStart);
		}

		memcpy(output, input, STRIDER_LAST_BYTES);
		progress->progress(input - inputStart + STRIDER_LAST_BYTES);
		delete[] parser;
		delete[] stream;
		delete[] literalModel;

		return output - outputStart + STRIDER_LAST_BYTES;
	}

	const int NOT_USED = -1;
	const CompressorOptions striderCompressorLevels[] = {
		//      Parser        Hash log     Elements per hash     Nice length     Block size     Max arrivals
			{ GREEDY       ,     17     ,      NOT_USED       ,   NOT_USED   ,    NOT_USED    ,   NOT_USED   },
			{ LAZY_NORMAL  ,     17     ,      NOT_USED       ,      16      ,    NOT_USED    ,   NOT_USED   },
			{ LAZY_EXTRA   ,     17     ,          1          ,      16      ,    NOT_USED    ,   NOT_USED   },
			{ LAZY_EXTRA   ,     18     ,          2          ,      24      ,    NOT_USED    ,   NOT_USED   },
			{ OPTIMAL_FAST ,     18     ,          2          ,      24      ,      1024      ,   NOT_USED   },
			{ OPTIMAL_FAST ,     19     ,          3          ,      32      ,      1024      ,   NOT_USED   },
			{ OPTIMAL      ,     22     ,          5          ,      32      ,      2048      ,   NOT_USED   },
			{ OPTIMAL      ,     24     ,          6          ,      64      ,      2048      ,   NOT_USED   },
			{ OPTIMAL_BRUTE,     25     ,          6          ,      64      ,      4096      ,       3      },
			{ OPTIMAL_BRUTE,     27     ,          7          ,      256     ,      4096      ,       8      },
			{ OPTIMAL_ULTRA,     31     ,          8          ,      1024    ,      4096      ,       24     },
	};

	size_t strider_compress(const uint8_t* input, const size_t size, uint8_t* output, int level,
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
			progress->progress(size);
			return size;
		}

#ifdef IS_64BIT
		if (size > ((uint64_t)1 << 32)) {
			if (striderCompressorLevels[level].parserFunction == GREEDY)
				return strider_compress_greedy<uint64_t>(input, size, output, striderCompressorLevels[level], progress, window);
			if (striderCompressorLevels[level].parserFunction <= LAZY_EXTRA)
				return strider_compress_lazy<uint64_t>(input, size, output, striderCompressorLevels[level], progress, window);
			return strider_compress_optimal<uint64_t>(input, size, output, striderCompressorLevels[level], progress, window);
		}
#endif
		if (striderCompressorLevels[level].parserFunction == GREEDY)
			return strider_compress_greedy<uint32_t>(input, size, output, striderCompressorLevels[level], progress, window);
		if (striderCompressorLevels[level].parserFunction <= LAZY_EXTRA)
			return strider_compress_lazy<uint32_t>(input, size, output, striderCompressorLevels[level], progress, window);
		return strider_compress_optimal<uint32_t>(input, size, output, striderCompressorLevels[level], progress, window);
	}

	size_t strider_compress_bound(const size_t size) {
		return size + size / 128 + 32;
	}

	template<class IntType>
	size_t strider_parser_memory_estimator(const size_t size, const int level, const int window) {
		if (striderCompressorLevels[level].parserFunction == GREEDY)
			return sizeof(IntType) << MIN3((int)int_log2(size) - 3, striderCompressorLevels[level].maxHashTableSize, window - 3);
		if (striderCompressorLevels[level].parserFunction == LAZY_NORMAL)
			return sizeof(IntType) << MIN3((int)int_log2(size) - 3, striderCompressorLevels[level].maxHashTableSize, window - 3) << 1;
		if (striderCompressorLevels[level].parserFunction == LAZY_EXTRA)
			//Lazy extra uses 2 tables
			return sizeof(IntType) << MIN3((int)int_log2(size) - 3, striderCompressorLevels[level].maxHashTableSize, window - 3)
			<< striderCompressorLevels[level].maxElementsPerHash << 1;
		if (striderCompressorLevels[level].parserFunction == OPTIMAL_FAST) {
			const int log2size = MIN3((int)int_log2(size) - 3, striderCompressorLevels[level].maxHashTableSize, window - 3);
			size_t memory = sizeof(IntType) << std::min(log2size, 14);  //hash 3 table
			memory += sizeof(IntType) << log2size << striderCompressorLevels[level].maxElementsPerHash << 1;  //hash 4 and hash 8 tables
			memory += sizeof(StriderFastOptimalState<IntType>) * striderCompressorLevels[level].optimalBlockSize + 1;
			memory += sizeof(LZStructure<IntType>) * striderCompressorLevels[level].optimalBlockSize;
			return memory;
		}
		size_t memory = sizeof(IntType) << MIN3((int)int_log2(size) - 3, 20, window - 3);  //binary node lookup
		memory += sizeof(IntType) << MIN3((int)int_log2(size) - 3, 14, window - 3);  //hash 3 table
		if (striderCompressorLevels[level].parserFunction >= OPTIMAL_BRUTE)
			memory += sizeof(IntType) << MIN3((int)int_log2(size) - 3, 12, window - 3);  //hash 2 table
		const size_t binaryTreeWindow = (size_t)1 << std::min(striderCompressorLevels[level].maxHashTableSize, window);
		if (size < binaryTreeWindow)
			memory += sizeof(IntType) * 2 * size;  //binary tree
		else {
			memory += sizeof(IntType) * 2 * binaryTreeWindow;  //binary tree
			if (window > striderCompressorLevels[level].maxHashTableSize)
				memory += sizeof(IntType) << std::max(1, (int)int_log2(std::min(size, (size_t)1 << window) - binaryTreeWindow) - 3);  //hash 16 table
		}
		if (striderCompressorLevels[level].parserFunction == OPTIMAL)
			memory += sizeof(StriderOptimalParserState<IntType>) * (striderCompressorLevels[level].optimalBlockSize + 1);
		else
			memory += sizeof(StriderOptimalParserState<IntType>) * (striderCompressorLevels[level].optimalBlockSize + 1) * striderCompressorLevels[level].maxArrivals;
		memory += sizeof(LZStructure<IntType>) * striderCompressorLevels[level].optimalBlockSize;
		return memory;
	}

	size_t strider_estimate_memory(const size_t size, int level, int window) {

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
		size_t memory = blockSize * 1.25 + 32;   //stream buffer
		memory += (blockSize * 3.0 + 64) * sizeof(uint32_t);  //symbol buffer
		memory += sizeof(NibbleModel) * 64 * 256;  //literal model, the encoder will not use lc + pb > 8

#ifdef IS_64BIT
		if (size > ((uint64_t)1 << 32))
			return memory + strider_parser_memory_estimator<uint64_t>(size, level, window);
#endif
		return memory + strider_parser_memory_estimator<uint32_t>(size, level, window);
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
			while (nBits > 15) {
				symbol <<= 15;
				std::swap(stateA, stateB);
				symbol |= stateB & (1 << 15) - 1;
				stateB >>= 15;
				normalize();
				nBits -= 15;
			}

			symbol <<= nBits;
			std::swap(stateA, stateB);
			symbol |= stateB & (1 << nBits) - 1;
			stateB >>= nBits;
			normalize();
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

	int strider_decompress(const uint8_t* compressed, const size_t compressedSize, uint8_t* decompressed,
		const size_t decompressedSize, ProgressCallback* progress) {

		ProgressCallback defaultProgress;
		if (!progress)
			progress = &defaultProgress;

		if (decompressedSize <= STRIDER_LAST_BYTES + 1) {
			if (compressedSize < decompressedSize)
				return -1;
			memcpy(decompressed, compressed, decompressedSize);
			progress->progress(decompressedSize);
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
		NibbleModel literalRunLengthHigh[193];
		NibbleModel* literalModel = nullptr;  //Also indicates whether this is the first rans block
		NibbleModel distanceModel[24];
		NibbleModel distanceLow[2];
		NibbleModel matchLengthHigh[148];

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
						literalModel = new NibbleModel[((48 << 8) >> newLiteralContextBitsShift) * (newPositionContextBitMask + 1)];
					}
					catch (const std::bad_alloc& e) {
						return -1;
					}
				}

				//We have to reset everything
				if (doModelReset) {
					reset_models(literalRunLengthHigh, &matchLiteralContext, literalModel, newLiteralContextBitsShift,
						newPositionContextBitMask, &distance, repOffsets, distanceModel, distanceLow, matchLengthHigh);
				}
				//Only reset certain models if needed
				else {
					//These only require position bit mask
					if (positionContextBitMask != newPositionContextBitMask) {
						for (size_t i = 0; i < 192; i++)
							literalRunLengthHigh[i].init();
						for (size_t i = 0; i < 144; i++)
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

					//A symbol can have a minimum frequency of 16/16384, that is, its decoding will consume at most 10 bits.
					//As both states might be just before renormalization interval, we should be able to read at least 192 bits
					// before having to check again for the buffer overflow.
					//A match can consume, at most, 131 bits plus 20 for a long literal run (total = 151)
					if (unlikely(ransDecoder.check_stream_overflow())) {
						delete[] literalModel;
						return -1;
					}

					if (literalRunLength) {
						matchLiteralContext >>= 2;
						//The literal we just have decoded is the previous byte
						uint8_t literal = decompressed[-1];

						do {
							uint8_t exclude = *(decompressed - distance);
							NibbleModel* literalModelTree =
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

						std::copy_backward(repOffsets + 5, repOffsets + 7, repOffsets + 8);
						repOffsets[5] = distance;

						matchLiteralContext |= 8;
					}

					size_t matchLength = ransDecoder.decode_nibble(&matchLengthHigh[matchLengthContext * 16 | positionContext]);
					if (matchLength == 15) {
						matchLength = ransDecoder.decode_nibble(&matchLengthHigh[144 + (matchLengthContext != 0)]) + 17;
						if (matchLength == 32) {
							size_t symbol = ransDecoder.decode_nibble(&matchLengthHigh[146 + (matchLengthContext != 0)]);
							matchLength = ((size_t)1 << symbol) + ransDecoder.decode_raw_bits(symbol) + 31;
						}

						//Check overflow
						if (unlikely(thisBlockEnd - decompressed < matchLength)) {
							delete[] literalModel;
							return -1;
						}
					}
					else {
						matchLength += STRIDER_MIN_LENGTH;
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
