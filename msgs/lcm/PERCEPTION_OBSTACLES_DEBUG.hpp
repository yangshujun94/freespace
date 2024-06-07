/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __PERCEPTION_OBSTACLES_DEBUG_hpp__
#define __PERCEPTION_OBSTACLES_DEBUG_hpp__

#include <vector>
#include "HEADER.hpp"
#include "PERCEPTION_OBSTACLE_DEBUG.hpp"


class PERCEPTION_OBSTACLES_DEBUG
{
    public:
        HEADER     stHeader;

        int8_t     obj_num;

        std::vector< PERCEPTION_OBSTACLE_DEBUG > gstObstacles;

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
         * Returns "PERCEPTION_OBSTACLES_DEBUG"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int PERCEPTION_OBSTACLES_DEBUG::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int PERCEPTION_OBSTACLES_DEBUG::decode(const void *buf, int offset, int maxlen)
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

int PERCEPTION_OBSTACLES_DEBUG::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t PERCEPTION_OBSTACLES_DEBUG::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* PERCEPTION_OBSTACLES_DEBUG::getTypeName()
{
    return "PERCEPTION_OBSTACLES_DEBUG";
}

int PERCEPTION_OBSTACLES_DEBUG::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = this->stHeader._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->obj_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->obj_num; a0++) {
        tlen = this->gstObstacles[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int PERCEPTION_OBSTACLES_DEBUG::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = this->stHeader._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->obj_num, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->gstObstacles.resize(this->obj_num);
    for (int a0 = 0; a0 < this->obj_num; a0++) {
        tlen = this->gstObstacles[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int PERCEPTION_OBSTACLES_DEBUG::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += this->stHeader._getEncodedSizeNoHash();
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->obj_num; a0++) {
        enc_size += this->gstObstacles[a0]._getEncodedSizeNoHash();
    }
    return enc_size;
}

uint64_t PERCEPTION_OBSTACLES_DEBUG::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == PERCEPTION_OBSTACLES_DEBUG::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)PERCEPTION_OBSTACLES_DEBUG::getHash };

    uint64_t hash = 0xe8498c7de4d782aaLL +
         HEADER::_computeHash(&cp) +
         PERCEPTION_OBSTACLE_DEBUG::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
