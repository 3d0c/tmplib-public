#ifndef ZLIB_UTILS_H
#define ZLIB_UTILS_H

#include "zlib.h"
#ifdef _WIN32
#pragma comment(lib, "zdll.lib")
#else
#endif

int GetMaxCompressedLen( int nLenSrc ) 
{
    int n16kBlocks = (nLenSrc+16383) / 16384; // round up any fraction of a block
    return ( nLenSrc + 6 + (n16kBlocks*5) );
}

int CompressData( const uint8_t* Src, int nLenSrc, uint8_t* Dst, int nLenDst, int level )
{
    z_stream zInfo = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    zInfo.total_in = zInfo.avail_in = nLenSrc;
    zInfo.total_out = zInfo.avail_out = nLenDst;
    zInfo.next_in = (uint8_t*)Src;
    zInfo.next_out = Dst;

    int nErr, nRet= -1;
    nErr = deflateInit( &zInfo, level );
    if ( nErr == Z_OK ) {
        nErr= deflate( &zInfo, Z_FINISH );
        if ( nErr == Z_STREAM_END ) {
            nRet = zInfo.total_out;
        }
    }
    deflateEnd( &zInfo );
    return( nRet );
}

int UncompressData( const uint8_t* Src, int nLenSrc, uint8_t* Dst, int nLenDst )
{
    z_stream zInfo = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    zInfo.total_in = zInfo.avail_in = nLenSrc;
    zInfo.total_out = zInfo.avail_out = nLenDst;
    zInfo.next_in = (uint8_t*)Src;
    zInfo.next_out = Dst;

    int nErr, nRet = -1;
    nErr = inflateInit( &zInfo );
    if ( nErr == Z_OK ) {
        nErr = inflate( &zInfo, Z_FINISH );
        if ( nErr == Z_STREAM_END ) {
            nRet = zInfo.total_out;
        }
    }
    inflateEnd( &zInfo );
    return( nRet );
}
#endif // !ZLIB_UTILS_H
