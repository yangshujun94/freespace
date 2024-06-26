/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __LIDAR_CNN_OBJECTS_hpp__
#define __LIDAR_CNN_OBJECTS_hpp__

#include "HEADER.hpp"


class LIDAR_CNN_OBJECTS
{
    public:
        HEADER     stHeader;

        int16_t    nObjectsNum;

        int16_t    nNumOfPoints[256];

        int16_t    nNumOfGrids[256];

        uint8_t    nClasType[256];

        float      gfObjectBoxOrientation[256];

        float      gfObjectBoxCenterX[256];

        float      gfObjectBoxCenterY[256];

        float      gfObjectBoxCenterZ[256];

        float      gfObjectBoxLength[256];

        float      gfObjectBoxWidth[256];

        float      gfObjectBoxHeigth[256];

        float      fTypeConfidence[256];

        uint8_t    nLostLidarStatus;

        float      Reserved[4];

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
         * Returns "LIDAR_CNN_OBJECTS"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int LIDAR_CNN_OBJECTS::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LIDAR_CNN_OBJECTS::decode(const void *buf, int offset, int maxlen)
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

int LIDAR_CNN_OBJECTS::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t LIDAR_CNN_OBJECTS::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* LIDAR_CNN_OBJECTS::getTypeName()
{
    return "LIDAR_CNN_OBJECTS";
}

int LIDAR_CNN_OBJECTS::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = this->stHeader._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->nObjectsNum, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->nNumOfPoints[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->nNumOfGrids[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->nClasType[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxOrientation[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxCenterX[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxCenterY[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxCenterZ[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxLength[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxWidth[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxHeigth[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fTypeConfidence[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->nLostLidarStatus, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->Reserved[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LIDAR_CNN_OBJECTS::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = this->stHeader._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->nObjectsNum, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->nNumOfPoints[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->nNumOfGrids[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->nClasType[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxOrientation[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxCenterX[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxCenterY[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxCenterZ[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxLength[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxWidth[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->gfObjectBoxHeigth[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fTypeConfidence[0], 256);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->nLostLidarStatus, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->Reserved[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LIDAR_CNN_OBJECTS::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->stHeader._getEncodedSizeNoHash();
    enc_size += __int16_t_encoded_array_size(NULL, 1);
    enc_size += __int16_t_encoded_array_size(NULL, 256);
    enc_size += __int16_t_encoded_array_size(NULL, 256);
    enc_size += __byte_encoded_array_size(NULL, 256);
    enc_size += __float_encoded_array_size(NULL, 256);
    enc_size += __float_encoded_array_size(NULL, 256);
    enc_size += __float_encoded_array_size(NULL, 256);
    enc_size += __float_encoded_array_size(NULL, 256);
    enc_size += __float_encoded_array_size(NULL, 256);
    enc_size += __float_encoded_array_size(NULL, 256);
    enc_size += __float_encoded_array_size(NULL, 256);
    enc_size += __float_encoded_array_size(NULL, 256);
    enc_size += __byte_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 4);
    return enc_size;
}

uint64_t LIDAR_CNN_OBJECTS::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == LIDAR_CNN_OBJECTS::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)LIDAR_CNN_OBJECTS::getHash };

    uint64_t hash = 0xc4b275eef52edfb6LL +
         HEADER::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
