// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: monitor_errorCode.proto

#ifndef PROTOBUF_monitor_5ferrorCode_2eproto__INCLUDED
#define PROTOBUF_monitor_5ferrorCode_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3003000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3003000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/message_lite.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
// @@protoc_insertion_point(includes)
namespace atd {
namespace monitor {
class ErrorCode2Monitor;
class ErrorCode2MonitorDefaultTypeInternal;
extern ErrorCode2MonitorDefaultTypeInternal _ErrorCode2Monitor_default_instance_;
}  // namespace monitor
}  // namespace atd

namespace atd {
namespace monitor {

namespace protobuf_monitor_5ferrorCode_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[];
  static const ::google::protobuf::uint32 offsets[];
  static void InitDefaultsImpl();
  static void Shutdown();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_monitor_5ferrorCode_2eproto

// ===================================================================

class ErrorCode2Monitor : public ::google::protobuf::MessageLite /* @@protoc_insertion_point(class_definition:atd.monitor.ErrorCode2Monitor) */ {
 public:
  ErrorCode2Monitor();
  virtual ~ErrorCode2Monitor();

  ErrorCode2Monitor(const ErrorCode2Monitor& from);

  inline ErrorCode2Monitor& operator=(const ErrorCode2Monitor& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::std::string& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }

  inline ::std::string* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ErrorCode2Monitor& default_instance();

  static inline const ErrorCode2Monitor* internal_default_instance() {
    return reinterpret_cast<const ErrorCode2Monitor*>(
               &_ErrorCode2Monitor_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(ErrorCode2Monitor* other);

  // implements Message ----------------------------------------------

  inline ErrorCode2Monitor* New() const PROTOBUF_FINAL { return New(NULL); }

  ErrorCode2Monitor* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CheckTypeAndMergeFrom(const ::google::protobuf::MessageLite& from)
    PROTOBUF_FINAL;
  void CopyFrom(const ErrorCode2Monitor& from);
  void MergeFrom(const ErrorCode2Monitor& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  void DiscardUnknownFields();
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  void InternalSwap(ErrorCode2Monitor* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::std::string GetTypeName() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated int32 errorCode = 1;
  int errorcode_size() const;
  void clear_errorcode();
  static const int kErrorCodeFieldNumber = 1;
  ::google::protobuf::int32 errorcode(int index) const;
  void set_errorcode(int index, ::google::protobuf::int32 value);
  void add_errorcode(::google::protobuf::int32 value);
  const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
      errorcode() const;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
      mutable_errorcode();

  // optional uint64 timestamp = 2;
  bool has_timestamp() const;
  void clear_timestamp();
  static const int kTimestampFieldNumber = 2;
  ::google::protobuf::uint64 timestamp() const;
  void set_timestamp(::google::protobuf::uint64 value);

  // @@protoc_insertion_point(class_scope:atd.monitor.ErrorCode2Monitor)
 private:
  void set_has_timestamp();
  void clear_has_timestamp();

  ::google::protobuf::internal::InternalMetadataWithArenaLite _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 > errorcode_;
  ::google::protobuf::uint64 timestamp_;
  friend struct protobuf_monitor_5ferrorCode_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// ErrorCode2Monitor

// repeated int32 errorCode = 1;
inline int ErrorCode2Monitor::errorcode_size() const {
  return errorcode_.size();
}
inline void ErrorCode2Monitor::clear_errorcode() {
  errorcode_.Clear();
}
inline ::google::protobuf::int32 ErrorCode2Monitor::errorcode(int index) const {
  // @@protoc_insertion_point(field_get:atd.monitor.ErrorCode2Monitor.errorCode)
  return errorcode_.Get(index);
}
inline void ErrorCode2Monitor::set_errorcode(int index, ::google::protobuf::int32 value) {
  errorcode_.Set(index, value);
  // @@protoc_insertion_point(field_set:atd.monitor.ErrorCode2Monitor.errorCode)
}
inline void ErrorCode2Monitor::add_errorcode(::google::protobuf::int32 value) {
  errorcode_.Add(value);
  // @@protoc_insertion_point(field_add:atd.monitor.ErrorCode2Monitor.errorCode)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
ErrorCode2Monitor::errorcode() const {
  // @@protoc_insertion_point(field_list:atd.monitor.ErrorCode2Monitor.errorCode)
  return errorcode_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
ErrorCode2Monitor::mutable_errorcode() {
  // @@protoc_insertion_point(field_mutable_list:atd.monitor.ErrorCode2Monitor.errorCode)
  return &errorcode_;
}

// optional uint64 timestamp = 2;
inline bool ErrorCode2Monitor::has_timestamp() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ErrorCode2Monitor::set_has_timestamp() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ErrorCode2Monitor::clear_has_timestamp() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ErrorCode2Monitor::clear_timestamp() {
  timestamp_ = GOOGLE_ULONGLONG(0);
  clear_has_timestamp();
}
inline ::google::protobuf::uint64 ErrorCode2Monitor::timestamp() const {
  // @@protoc_insertion_point(field_get:atd.monitor.ErrorCode2Monitor.timestamp)
  return timestamp_;
}
inline void ErrorCode2Monitor::set_timestamp(::google::protobuf::uint64 value) {
  set_has_timestamp();
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:atd.monitor.ErrorCode2Monitor.timestamp)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace monitor
}  // namespace atd

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_monitor_5ferrorCode_2eproto__INCLUDED
