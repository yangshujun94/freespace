/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __PERCEPTION_OBSTACLE_FMS_hpp__
#define __PERCEPTION_OBSTACLE_FMS_hpp__

#include <vector>
#include "OBSTACLE_HISTORY_TRAJECTORY.hpp"
#include "ASSOCIATED_FMS_INFO.hpp"

/**
 * fmsValid  感知障碍物与fms系统是否存在关系
 * associated_fms_info 感知障碍物与fms系统的关联关系
 */
class PERCEPTION_OBSTACLE_FMS
{
public:
  int8_t bValid;

  int16_t nObjectID;

  uint8_t nType;

  float fHeading;

  float fOrient;

  float fRelX;

  float fRelY;

  float fLength;

  float fWidth;

  float fHeight;

  float fAbsVelX;

  float fAbsVelY;

  float fAbsAccelX;

  float fAbsAccelY;

  float fAbsSpeed;

  float fYawRate;

  uint8_t nMoveStatus;

  uint8_t nLaneLabel;

  uint8_t nMotionCategory;

  uint8_t nMotionOrientation;

  uint8_t nBrakeLights;

  uint8_t nRightTurnLights;

  uint8_t nLeftTurnLights;

  float fRelX_STD;

  float fRelY_STD;

  float fHeight_STD;

  float fWidth_STD;

  float fLength_STD;

  float fAbsVelX_STD;

  float fAbsVelY_STD;

  int8_t nTrajectoryLength;

  std::vector<OBSTACLE_HISTORY_TRAJECTORY> gstHistoryTrajectory;

  int8_t reserved_1;

  int8_t reserved_2;

  int8_t reserved_3;

  int8_t fmsValid;

  ASSOCIATED_FMS_INFO associated_fms_info;

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
         * Returns "PERCEPTION_OBSTACLE_FMS"
         */
  inline static const char *getTypeName();

  // LCM support functions. Users should not call these
  inline int             _encodeNoHash(void *buf, int offset, int maxlen) const;
  inline int             _getEncodedSizeNoHash() const;
  inline int             _decodeNoHash(const void *buf, int offset, int maxlen);
  inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int PERCEPTION_OBSTACLE_FMS::encode(void *buf, int offset, int maxlen) const
{
  int     pos  = 0, tlen;
  int64_t hash = (int64_t)getHash();

  tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int PERCEPTION_OBSTACLE_FMS::decode(const void *buf, int offset, int maxlen)
{
  int pos = 0, thislen;

  int64_t msg_hash;
  thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
  if(thislen < 0)
    return thislen;
  else
    pos += thislen;
  if(msg_hash != getHash())
    return -1;

  thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
  if(thislen < 0)
    return thislen;
  else
    pos += thislen;

  return pos;
}

int PERCEPTION_OBSTACLE_FMS::getEncodedSize() const
{
  return 8 + _getEncodedSizeNoHash();
}

int64_t PERCEPTION_OBSTACLE_FMS::getHash()
{
  static int64_t hash = _computeHash(NULL);
  return hash;
}

const char *PERCEPTION_OBSTACLE_FMS::getTypeName()
{
  return "PERCEPTION_OBSTACLE_FMS";
}

int PERCEPTION_OBSTACLE_FMS::_encodeNoHash(void *buf, int offset, int maxlen) const
{
  int pos = 0, tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->bValid, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->nObjectID, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->nType, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fHeading, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fOrient, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fRelX, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fRelY, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fLength, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fWidth, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fHeight, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fAbsVelX, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fAbsVelY, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fAbsAccelX, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fAbsAccelY, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fAbsSpeed, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fYawRate, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->nMoveStatus, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->nLaneLabel, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->nMotionCategory, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->nMotionOrientation, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->nBrakeLights, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->nRightTurnLights, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_encode_array(buf, offset + pos, maxlen - pos, &this->nLeftTurnLights, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fRelX_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fRelY_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fHeight_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fWidth_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fLength_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fAbsVelX_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fAbsVelY_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->nTrajectoryLength, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  for(int a0 = 0; a0 < this->nTrajectoryLength; a0++)
  {
    tlen = this->gstHistoryTrajectory[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0)
      return tlen;
    else
      pos += tlen;
  }

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->reserved_1, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->reserved_2, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->reserved_3, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->fmsValid, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = this->associated_fms_info._encodeNoHash(buf, offset + pos, maxlen - pos);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int PERCEPTION_OBSTACLE_FMS::_decodeNoHash(const void *buf, int offset, int maxlen)
{
  int pos = 0, tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->bValid, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->nObjectID, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->nType, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fHeading, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fOrient, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fRelX, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fRelY, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fLength, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fWidth, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fHeight, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fAbsVelX, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fAbsVelY, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fAbsAccelX, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fAbsAccelY, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fAbsSpeed, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fYawRate, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->nMoveStatus, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->nLaneLabel, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->nMotionCategory, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->nMotionOrientation, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->nBrakeLights, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->nRightTurnLights, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __byte_decode_array(buf, offset + pos, maxlen - pos, &this->nLeftTurnLights, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fRelX_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fRelY_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fHeight_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fWidth_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fLength_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fAbsVelX_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fAbsVelY_STD, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->nTrajectoryLength, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  this->gstHistoryTrajectory.resize(this->nTrajectoryLength);
  for(int a0 = 0; a0 < this->nTrajectoryLength; a0++)
  {
    tlen = this->gstHistoryTrajectory[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(tlen < 0)
      return tlen;
    else
      pos += tlen;
  }

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->reserved_1, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->reserved_2, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->reserved_3, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->fmsValid, 1);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  tlen = this->associated_fms_info._decodeNoHash(buf, offset + pos, maxlen - pos);
  if(tlen < 0)
    return tlen;
  else
    pos += tlen;

  return pos;
}

int PERCEPTION_OBSTACLE_FMS::_getEncodedSizeNoHash() const
{
  int enc_size = 0;
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int16_t_encoded_array_size(NULL, 1);
  enc_size += __byte_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __byte_encoded_array_size(NULL, 1);
  enc_size += __byte_encoded_array_size(NULL, 1);
  enc_size += __byte_encoded_array_size(NULL, 1);
  enc_size += __byte_encoded_array_size(NULL, 1);
  enc_size += __byte_encoded_array_size(NULL, 1);
  enc_size += __byte_encoded_array_size(NULL, 1);
  enc_size += __byte_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __float_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  for(int a0 = 0; a0 < this->nTrajectoryLength; a0++)
  {
    enc_size += this->gstHistoryTrajectory[a0]._getEncodedSizeNoHash();
  }
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += __int8_t_encoded_array_size(NULL, 1);
  enc_size += this->associated_fms_info._getEncodedSizeNoHash();
  return enc_size;
}

uint64_t PERCEPTION_OBSTACLE_FMS::_computeHash(const __lcm_hash_ptr *p)
{
  const __lcm_hash_ptr *fp;
  for(fp = p; fp != NULL; fp = fp->parent)
    if(fp->v == PERCEPTION_OBSTACLE_FMS::getHash)
      return 0;
  const __lcm_hash_ptr cp = {p, (void *)PERCEPTION_OBSTACLE_FMS::getHash};

  uint64_t hash = 0x4783d95d42a9c52dLL +
                  OBSTACLE_HISTORY_TRAJECTORY::_computeHash(&cp) +
                  ASSOCIATED_FMS_INFO::_computeHash(&cp);

  return (hash << 1) + ((hash >> 63) & 1);
}

#endif