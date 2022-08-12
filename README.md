# Strider
An algorithm that aims at reaching the same ratios as LZMA, but with 2.5x faster decompression. This is mainly achieved through the use of interleaved rANS and the use of a nibble based model, which can be accelerated with SIMD.
# Benchmark
The algorithm was benchmarked using [lzbench](https://github.com/inikep/lzbench) on Windows 10, on an i5-6300HQ and compiled with Visual Studio 2022. The file used was produced by tarring the [Silesia corpus](http://sun.aei.polsl.pl/~sdeor/index.php?page=silesia). The only additional parameter was -t16,16.
| Compressor name         | Compression| Decompress.| Compr. size | Ratio |
| ---------------         | -----------| -----------| ----------- | ----- | 
| memcpy                  | 11769 MB/s | 11526 MB/s |   211948544 |100.00 |  
| **strider 1.1.0 -0**    |    88 MB/s |   148 MB/s |    61609295 | 29.07 |
| **strider 1.1.0 -1**    |    70 MB/s |   154 MB/s |    59616463 | 28.13 |
| **strider 1.1.0 -3**    |    16 MB/s |   163 MB/s |    56387479 | 26.60 |
| **strider 1.1.0 -5**    |  6.22 MB/s |   168 MB/s |    53448711 | 25.22 |
| **strider 1.1.0 -7**    |  2.03 MB/s |   170 MB/s |    49223009 | 23.22 | 
| **strider 1.1.0 -9**    |  0.90 MB/s |   161 MB/s |    47000961 | 22.18 | 
| lzma 21.03 -1           |    21 MB/s |    61 MB/s |    60694124 | 28.64 |  
| lzma 21.03 -3           |    15 MB/s |    68 MB/s |    57809272 | 27.28 |
| lzma 21.03 -5           |  2.76 MB/s |    77 MB/s |    49716983 | 23.46 |
| lzma 21.03 -7           |  2.28 MB/s |    78 MB/s |    48726228 | 22.99 |
| lzma 21.03 -9           |  2.23 MB/s |    78 MB/s |    48711149 | 22.98 |
# Thanks
To Igor Pavlov for making the LZMA algorithm open source, the Oodle team for sharing some of their research and the encode.su community.
