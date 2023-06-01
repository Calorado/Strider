# Strider
An algorithm that aims at reaching the same ratios as LZMA, but with 2x faster decompression. This is mainly achieved through the use of interleaved rANS and the use of a nibble based model, which can be accelerated with SIMD.
# How to use
It's basically the same as my other compressor Skanda, you can check it here: https://github.com/Calorado/Skanda
# Benchmark
The algorithm was benchmarked on Windows 11, on a Ryzen 6900HX@3.3GHz and compiled with Visual Studio 2022. The file used was produced by tarring the [Silesia corpus](http://sun.aei.polsl.pl/~sdeor/index.php?page=silesia).
| Compressor name         | Compression| Decompress.| Compr. size | Ratio |
| ---------------         | -----------| -----------| ----------- | ----- | 
| zstd 1.5.5 -1 | 349.89MiB/s | 860.82MiB/s | 73423309 | 34.64 |
| zstd 1.5.5 -6 | 76.19MiB/s | 795.90MiB/s | 61481995 | 29.01 |
| zstd 1.5.5 -12 | 21.84MiB/s | 837.13MiB/s | 58196278 | 27.46 |
| zstd 1.5.5 -17 | 3.91MiB/s | 776.80MiB/s | 54284479 | 25.61 |
| zstd 1.5.5 -22 | 1.54MiB/s | 725.87MiB/s | 52473367 | 24.76 |
| **strider 0.4 -0** | 112.04MiB/s | 188.83MiB/s | 60610789 | 28.60 |
| **strider 0.4 -2** | 58.50MiB/s | 195.42MiB/s | 57595529 | 27.17 |
| **strider 0.4 -4** | 16.13MiB/s | 216.11MiB/s | 54322313 | 25.63 |
| **strider 0.4 -6** | 2.87MiB/s | 221.97MiB/s | 49044571 | 23.14 |
| **strider 0.4 -9** | 0.99MiB/s | 207.69MiB/s | 46416263 | 21.90 |
| lzma 22.01 -1 | 23.79MiB/s | 70.62MiB/s | 58443115 | 27.57 |
| lzma 22.01 -3 | 13.60MiB/s | 76.12MiB/s | 56213980 | 26.52 |
| lzma 22.01 -5 | 4.35MiB/s | 81.54MiB/s | 49716983 | 23.46 |
| lzma 22.01 -7 | 3.52MiB/s | 82.22MiB/s | 48726295 | 22.99 |
| lzma 22.01 -9 | 3.47MiB/s | 82.38MiB/s | 48709675 | 22.98 |
| lzham 1.0 -1 | 2.73MiB/s | 268.40MiB/s | 54750002 | 25.83 |
| lzham 1.0 -2 | 2.14MiB/s | 280.17MiB/s | 53070327 | 25.04 |
| lzham 1.0 -3 | 1.56MiB/s | 288.16MiB/s | 51357555 | 24.23 |
| lzham 1.0 -4 | 1.37MiB/s | 289.39MiB/s | 51088981 | 24.10 |
# Thanks
To Igor Pavlov for making the LZMA algorithm open source, the Oodle team for sharing some of their research and the encode.su community.
