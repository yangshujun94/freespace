/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __LocFusion_hpp__
#define __LocFusion_hpp__



class LocFusion
{
    public:
        int64_t    loc_fusion_timestamp;

        int64_t    odo_timestamp;

        double     odo_x;

        double     odo_y;

        double     odo_z;

        double     odo_roll;

        double     odo_pitch;

        double     odo_yaw;

        double     utm_x;

        double     utm_y;

        double     heading;

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
         * Returns "LocFusion"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int LocFusion::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LocFusion::decode(const void *buf, int offset, int maxlen)
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

int LocFusion::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t LocFusion::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* LocFusion::getTypeName()
{
    return "LocFusion";
}

int LocFusion::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->loc_fusion_timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->odo_timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->odo_x, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->odo_y, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->odo_z, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->odo_roll, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->odo_pitch, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->odo_yaw, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->utm_x, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->utm_y, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->heading, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LocFusion::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->loc_fusion_timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->odo_timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->odo_x, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->odo_y, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->odo_z, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->odo_roll, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->odo_pitch, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->odo_yaw, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->utm_x, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->utm_y, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->heading, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int LocFusion::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    enc_size += __double_encoded_array_size(NULL, 1);
    return enc_size;
}

uint64_t LocFusion::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0x6f1977b69d2446dfLL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
