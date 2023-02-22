# Strider
An algorithm that aims at reaching the same ratios as LZMA, but with 2x faster decompression. This is mainly achieved through the use of interleaved rANS and the use of a nibble based model, which can be accelerated with SIMD.
# How to use
It's basically the same as my other compressor Skanda, you can check it here: https://github.com/Calorado/Skanda
# Benchmark
The algorithm was benchmarked on Windows 11, on a Ryzen 6900HX@3.3GHz and compiled with Visual Studio 2022. The file used was produced by tarring the [Silesia corpus](http://sun.aei.polsl.pl/~sdeor/index.php?page=silesia).
| Compressor name         | Compression| Decompress.| Compr. size | Ratio |
| ---------------         | -----------| -----------| ----------- | ----- | 
| **strider 0.3 -0** | 104.82MiB/s | 177.69MiB/s | 60620211 | 28.60 |
| **strider 0.3 -2** | 53.35MiB/s | 183.17MiB/s | 57244311 | 27.01 |
| **strider 0.3 -4** | 17.75MiB/s | 201.47MiB/s | 54355547 | 25.65 |
| **strider 0.3 -6** | 2.96MiB/s | 203.52MiB/s | 49083277 | 23.16 |
| **strider 0.3 -9** | 1.05MiB/s | 192.86MiB/s | 46484093 | 21.93 |
| zstd 1.5.4 -1 | 361.02MiB/s | 903.88MiB/s | 73423309 | 34.64 |
| zstd 1.5.4 -6 | 82.46MiB/s | 807.23MiB/s | 61481995 | 29.01 |
| zstd 1.5.4 -12 | 22.76MiB/s | 870.69MiB/s | 58196278 | 27.46 |
| zstd 1.5.4 -17 | 4.28MiB/s | 798.03MiB/s | 54284479 | 25.61 |
| zstd 1.5.4 -22 | 1.95MiB/s | 767.13MiB/s | 52473367 | 24.76 |
| lzham 1.0 -1 | 2.79MiB/s | 271.96MiB/s | 54750002 | 25.83 |
| lzham 1.0 -2 | 2.21MiB/s | 285.19MiB/s | 53070327 | 25.04 |
| lzham 1.0 -3 | 1.60MiB/s | 296.55MiB/s | 51357555 | 24.23 |
| lzham 1.0 -4 | 1.41MiB/s | 296.42MiB/s | 51088981 | 24.10 |
| lzma 22.01 -1 | 23.65MiB/s | 71.35MiB/s | 58443115 | 27.57 |
| lzma 22.01 -3 | 14.23MiB/s | 77.12MiB/s | 56213980 | 26.52 |
| lzma 22.01 -5 | 4.44MiB/s | 82.76MiB/s | 49716983 | 23.46 |
| lzma 22.01 -7 | 3.62MiB/s | 83.69MiB/s | 48726295 | 22.99 |
| lzma 22.01 -9 | 3.55MiB/s | 83.67MiB/s | 48709675 | 22.98 |
# Thanks
To Igor Pavlov for making the LZMA algorithm open source, the Oodle team for sharing some of their research and the encode.su community.
