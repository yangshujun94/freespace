/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __Lidar_FreeSpace_v2_hpp__
#define __Lidar_FreeSpace_v2_hpp__

#include <vector>
#include "HEADER.hpp"
#include "Lidar_FreeSpace_Point_v2.hpp"


class Lidar_FreeSpace_v2
{
    public:
        HEADER     stHeader;

        int32_t    nPointNum;

        std::vector< Lidar_FreeSpace_Point_v2 > gstPoints;

        int32_t    nVehicleOriginRows;

        int32_t    nVehicleOriginCols;

        float      fResolution;

        float      fReserved[10];

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "Lidar_FreeSpace_v2"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int Lidar_FreeSpace_v2::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int Lidar_FreeSpace_v2::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int Lidar_FreeSpace_v2::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t Lidar_FreeSpace_v2::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* Lidar_FreeSpace_v2::getTypeName()
{
    return "Lidar_FreeSpace_v2";
}

int Lidar_FreeSpace_v2::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = this->stHeader._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->nPointNum, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->nPointNum; a0++) {
        tlen = this->gstPoints[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->nVehicleOriginRows, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->nVehicleOriginCols, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fResolution, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fReserved[0], 10);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int Lidar_FreeSpace_v2::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = this->stHeader._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->nPointNum, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->gstPoints.resize(this->nPointNum);
    for (int a0 = 0; a0 < this->nPointNum; a0++) {
        tlen = this->gstPoints[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->nVehicleOriginRows, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->nVehicleOriginCols, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fResolution, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fReserved[0], 10);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int Lidar_FreeSpace_v2::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->stHeader._getEncodedSizeNoHash();
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->nPointNum; a0++) {
        enc_size += this->gstPoints[a0]._getEncodedSizeNoHash();
    }
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 10);
    return enc_size;
}

uint64_t Lidar_FreeSpace_v2::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == Lidar_FreeSpace_v2::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)Lidar_FreeSpace_v2::getHash };

    uint64_t hash = 0xb417ad9b66c3d267LL +
         HEADER::_computeHash(&cp) +
         Lidar_FreeSpace_Point_v2::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
