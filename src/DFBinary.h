#ifndef DFBINARY_H
#define DFBINARY_H

#include <cstdint>

enum MeshOutputMode
{
    OUT_VERTICES = 0x1,
    OUT_TEXTURES = 0x2,
    OUT_NORMALS = 0x4,
    OUT_FACES = 0x8,
    OUT_GROUPS = 0x10,
    OUT_INCEDENCE_STRUCT = 0x20
};

struct BinaryHeader001
{
    uint64_t file_type_sign;            // internal type/version signature
    uint64_t header_size;               // size of header (bytes)
    uint64_t block_position_vv;         // start position of vertices block
    uint64_t block_size_vv;             // size of vertices block (bytes)
    uint64_t block_position_vt;         // start position of texture coors block
    uint64_t block_size_vt;             // size of texture coors block (bytes)
    uint64_t block_position_vn;         // start position of normal vectors block
    uint64_t block_size_vn;             // size of normal vectors block (bytes)
    uint64_t block_position_vf;         // start position of faces block
    uint64_t block_size_vf;             // size of faces block (bytes)
                                        // --- start of faces block structure ---
                                        // 32bit: faces quantity
                                        // 32bit: face item 1 elements quantity
                                        // 32bit: element 1 (v/t/n)
                                        // 32bit: element 2 (v/t/n)
                                        // . . .
                                        // 32bit: face item # elements quantity
                                        // 32bit: element 1 (v/t/n)
                                        // 32bit: element 2 (v/t/n)
                                        // . . .
                                        // --- end of faces block structure ---
    uint64_t block_position_groups;     // start position of groups block
    uint64_t block_size_groups;         // size of groups block (bytes)
    uint64_t block_position_incidence;  // start position of incedence struct block
    uint64_t block_size_incidence;      // size of incedence struct block (bytes)

    BinaryHeader001() : file_type_sign (0x3130304e49424644LL /* "DFBIN001" */), header_size(sizeof(BinaryHeader001)) { }
};

typedef BinaryHeader001 BinaryHeader;

#endif // !DFBINARY_H
