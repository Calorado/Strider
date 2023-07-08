# Strider
An algorithm that aims at reaching the same ratios as LZMA, but with 2x faster decompression. This is mainly achieved through the use of interleaved rANS and the use of a nibble based model, which can be accelerated with SIMD.
# How to use
It's basically the same as my other compressor Skanda, you can check it here: https://github.com/Calorado/Skanda
# Benchmark
The algorithm was benchmarked on Windows 11, on a Ryzen 6900HX@3.3GHz and compiled with Visual Studio 2022. The file used was produced by tarring the [Silesia corpus](http://sun.aei.polsl.pl/~sdeor/index.php?page=silesia).
| Compressor name         | Compression| Decompress.| Compr. size | Ratio |
| ---------------         | -----------| -----------| ----------- | ----- | 
| zstd 1.5.5 -1 | 360.13MiB/s | 861.60MiB/s | 73423309 | 34.64 |
| zstd 1.5.5 -6 | 74.28MiB/s | 796.59MiB/s | 61543204 | 29.04 |
| zstd 1.5.5 -12 | 19.94MiB/s | 812.60MiB/s | 58211131 | 27.46 |
| zstd 1.5.5 -17 | 4.09MiB/s | 757.63MiB/s | 54284479 | 25.61 |
| zstd 1.5.5 -22 | 1.69MiB/s | 724.07MiB/s | 52473367 | 24.76 |
| **strider 0.5 -0** | 113.70MiB/s | 188.33MiB/s | 60683105 | 28.63 |
| **strider 0.5 -3** | 23.33MiB/s | 208.06MiB/s | 55905255 | 26.38 |
| **strider 0.5 -5** | 7.95MiB/s | 207.33MiB/s | 52235531 | 24.65 |
| **strider 0.5 -7** | 2.05MiB/s | 225.53MiB/s | 47748313 | 22.53 |
| **strider 0.5 -9** | 0.91MiB/s | 212.30MiB/s | 46384225 | 21.88 |
| lzma 22.01 -1 | 24.02MiB/s | 72.98MiB/s | 58443115 | 27.57 |
| lzma 22.01 -3 | 13.32MiB/s | 79.13MiB/s | 56213980 | 26.52 |
| lzma 22.01 -5 | 4.50MiB/s | 84.69MiB/s | 49716983 | 23.46 |
| lzma 22.01 -7 | 3.67MiB/s | 85.43MiB/s | 48726295 | 22.99 |
| lzma 22.01 -9 | 3.55MiB/s | 85.50MiB/s | 48709675 | 22.98 |
| lzham 1.0 -1 | 2.75MiB/s | 277.86MiB/s | 54750002 | 25.83 |
| lzham 1.0 -2 | 2.17MiB/s | 290.15MiB/s | 53070327 | 25.04 |
| lzham 1.0 -3 | 1.60MiB/s | 302.36MiB/s | 51357555 | 24.23 |
| lzham 1.0 -4 | 1.41MiB/s | 301.76MiB/s | 51088981 | 24.10 |
# Thanks
To Igor Pavlov for making the LZMA algorithm open source, the Oodle team for sharing some of their research and the encode.su community.
