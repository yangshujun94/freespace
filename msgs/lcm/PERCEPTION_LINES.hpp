/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __PERCEPTION_LINES_hpp__
#define __PERCEPTION_LINES_hpp__

#include "HEADER.hpp"

class PERCEPTION_LINES {
public:
  HEADER stHeader;

  int8_t bLeftLeftLine;

  int8_t nLeftLeftLineType;

  int16_t nLeftLeftArrayLength;

  float gfLeftLeftLineX[300];

  float gfLeftLeftLineY[300];

  float fLeftLeftLineConf;

  int8_t bLeftLine;

  int8_t nLeftLineType;

  int16_t nLeftArrayLength;

  float gfLeftLineX[300];

  float gfLeftLineY[300];

  float fLeftLineConf;

  int8_t bRightLine;

  int8_t nRightLineType;

  int16_t nRightArrayLength;

  float gfRightLineX[300];

  float gfRightLineY[300];

  float fRightLineConf;

  int8_t bRightRightLine;

  int8_t nRightRightLineType;

  int16_t nRightRightArrayLength;

  float gfRightRightLineX[300];

  float gfRightRightLineY[300];

  float fRightRightLineConf;

  int8_t bLeftCentralLine;

  int16_t nLeftCentralArrayLength;

  float gfLeftCentralLineX[300];

  float gfLeftCentralLineY[300];

  float fLeftCentralLineConf;

  int8_t bHostCentralLine;

  int16_t nHostCentralArrayLength;

  float gfHostCentralLineX[300];

  float gfHostCentralLineY[300];

  float fHostCentralLineConf;

  int8_t bRightCentralLine;

  int16_t nRightCentralArrayLength;

  float gfRightCentralLineX[300];

  float gfRightCentralLineY[300];

  float fRightCentralLineConf;

  int8_t bIntersectionFlag;

  float gfLineCurvature[300];

  int8_t gnLaneProperty[3];

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
   * Returns "PERCEPTION_LINES"
   */
  inline static const char *getTypeName();

  // LCM support functions. Users should not call these
  inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
  inline int _getEncodedSizeNoHash() const;
  inline int _decodeNoHash(const void *buf, int offset, int maxlen);
  inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int PERCEPTION_LINES::encode(void *buf, int offset, int maxlen) const {
  int pos = 0, tlen;
  int64_t hash = (int64_t)getHash();

  tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int PERCEPTION_LINES::decode(const void *buf, int offset, int maxlen) {
  int pos = 0, thislen;

  int64_t msg_hash;
  thislen =
      __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
  if (thislen < 0)
    return thislen;
  else
    pos += thislen;
  if (msg_hash != getHash())
    return -1;

  thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
  if (thislen < 0)
    return thislen;
  else
    pos += thislen;

  return pos;
}

int PERCEPTION_LINES::getEncodedSize() const {
  return 8 + _getEncodedSizeNoHash();
}

int64_t PERCEPTION_LINES::getHash() {
  static int64_t hash = _computeHash(NULL);
  return hash;
}

const char *PERCEPTION_LINES::getTypeName() { return "PERCEPTION_LINES"; }

int PERCEPTION_LINES::_encodeNoHash(void *buf, int offset, int maxlen) const {
  int pos = 0, tlen;

  tlen = this->stHeader._encodeNoHash(buf, offset + pos, maxlen - pos);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->bLeftLeftLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->nLeftLeftLineType, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos,
                                &this->nLeftLeftArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftLeftLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftLeftLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->fLeftLeftLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->bLeftLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->nLeftLineType, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos,
                                &this->nLeftArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->fLeftLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->bRightLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->nRightLineType, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos,
                                &this->nRightArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->fRightLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->bRightRightLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->nRightRightLineType, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos,
                                &this->nRightRightArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightRightLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightRightLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->fRightRightLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->bLeftCentralLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos,
                                &this->nLeftCentralArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftCentralLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftCentralLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->fLeftCentralLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->bHostCentralLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos,
                                &this->nHostCentralArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfHostCentralLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfHostCentralLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->fHostCentralLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->bRightCentralLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos,
                                &this->nRightCentralArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightCentralLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightCentralLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->fRightCentralLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->bIntersectionFlag, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLineCurvature[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos,
                               &this->gnLaneProperty[0], 3);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int PERCEPTION_LINES::_decodeNoHash(const void *buf, int offset, int maxlen) {
  int pos = 0, tlen;

  tlen = this->stHeader._decodeNoHash(buf, offset + pos, maxlen - pos);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->bLeftLeftLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->nLeftLeftLineType, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos,
                                &this->nLeftLeftArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftLeftLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftLeftLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->fLeftLeftLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->bLeftLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->nLeftLineType, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos,
                                &this->nLeftArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->fLeftLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->bRightLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->nRightLineType, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos,
                                &this->nRightArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->fRightLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->bRightRightLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->nRightRightLineType, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos,
                                &this->nRightRightArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightRightLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightRightLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->fRightRightLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->bLeftCentralLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos,
                                &this->nLeftCentralArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftCentralLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLeftCentralLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->fLeftCentralLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->bHostCentralLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos,
                                &this->nHostCentralArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfHostCentralLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfHostCentralLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->fHostCentralLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->bRightCentralLine, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos,
                                &this->nRightCentralArrayLength, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightCentralLineX[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfRightCentralLineY[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->fRightCentralLineConf, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->bIntersectionFlag, 1);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos,
                              &this->gfLineCurvature[0], 300);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos,
                               &this->gnLaneProperty[0], 3);
  if (tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int PERCEPTION_LINES::_getEncodedSizeNoHash() const {
  int enc_size = 0;
  enc_size += this->stHeader._getEncodedSizeNoHash();
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int16_t_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int16_t_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int16_t_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int16_t_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int16_t_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int16_t_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int16_t_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 300);
  enc_size += __int8_t_encoded_array_size(NULL, 3);
  return enc_size;
}

uint64_t PERCEPTION_LINES::_computeHash(const __lcm_hash_ptr *p) {
  const __lcm_hash_ptr *fp;
  for (fp = p; fp != NULL; fp = fp->parent)
    if (fp->v == PERCEPTION_LINES::getHash)
      return 0;
  const __lcm_hash_ptr cp = {p, (void *)PERCEPTION_LINES::getHash};

  uint64_t hash = 0x4777b8bdf0ffe449LL + HEADER::_computeHash(&cp);

  return (hash << 1) + ((hash >> 63) & 1);
}

#endif
