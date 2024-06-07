/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __FS_BEV_SEG_CameraFreespaceSeg_hpp__
#define __FS_BEV_SEG_CameraFreespaceSeg_hpp__

#include <vector>

namespace FS_BEV_SEG
{

class CameraFreespaceSeg
{
    public:
        int64_t    measure_timestamp;

        int64_t    publish_timestamp;

        int32_t    bev_fs_seg_image_height;

        int32_t    bev_fs_seg_image_width;

        float      bev_fs_seg_width_resolution;

        float      bev_fs_seg_height_resolution;

        int32_t    bev_fs_seg_image_size;

        std::vector< uint8_t > bev_fs_seg_image_data_label;

        std::vector< uint8_t > bev_fs_seg_image_data_score;

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
         * Returns "CameraFreespaceSeg"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int CameraFreespaceSeg::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int CameraFreespaceSeg::decode(const void *buf, int offset, int maxlen)
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

int CameraFreespaceSeg::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t CameraFreespaceSeg::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* CameraFreespaceSeg::getTypeName()
{
    return "CameraFreespaceSeg";
}

int CameraFreespaceSeg::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->measure_timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->publish_timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_image_height, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_image_width, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_width_resolution, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_height_resolution, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_image_size, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->bev_fs_seg_image_size > 0) {
        tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_image_data_label[0], this->bev_fs_seg_image_size);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->bev_fs_seg_image_size > 0) {
        tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_image_data_score[0], this->bev_fs_seg_image_size);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int CameraFreespaceSeg::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->measure_timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->publish_timestamp, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_image_height, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_image_width, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_width_resolution, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_height_resolution, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_image_size, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    if(this->bev_fs_seg_image_size) {
        this->bev_fs_seg_image_data_label.resize(this->bev_fs_seg_image_size);
        tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_image_data_label[0], this->bev_fs_seg_image_size);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->bev_fs_seg_image_size) {
        this->bev_fs_seg_image_data_score.resize(this->bev_fs_seg_image_size);
        tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->bev_fs_seg_image_data_score[0], this->bev_fs_seg_image_size);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int CameraFreespaceSeg::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __byte_encoded_array_size(NULL, this->bev_fs_seg_image_size);
    enc_size += __byte_encoded_array_size(NULL, this->bev_fs_seg_image_size);
    return enc_size;
}

uint64_t CameraFreespaceSeg::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0x7999d3f06136c0ebLL;
    return (hash<<1) + ((hash>>63)&1);
}

}

#endif
