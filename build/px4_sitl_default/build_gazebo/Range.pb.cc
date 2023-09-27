// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Range.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "Range.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace sensor_msgs {
namespace msgs {

namespace {

const ::google::protobuf::Descriptor* Range_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Range_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_Range_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AssignDesc_Range_2eproto() {
  protobuf_AddDesc_Range_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "Range.proto");
  GOOGLE_CHECK(file != NULL);
  Range_descriptor_ = file->message_type(0);
  static const int Range_offsets_[8] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Range, time_usec_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Range, min_distance_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Range, max_distance_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Range, current_distance_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Range, h_fov_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Range, v_fov_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Range, orientation_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Range, signal_quality_),
  };
  Range_reflection_ =
    ::google::protobuf::internal::GeneratedMessageReflection::NewGeneratedMessageReflection(
      Range_descriptor_,
      Range::default_instance_,
      Range_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Range, _has_bits_[0]),
      -1,
      -1,
      sizeof(Range),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Range, _internal_metadata_),
      -1);
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_Range_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
      Range_descriptor_, &Range::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_Range_2eproto() {
  delete Range::default_instance_;
  delete Range_reflection_;
}

void protobuf_AddDesc_Range_2eproto() GOOGLE_ATTRIBUTE_COLD;
void protobuf_AddDesc_Range_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::gazebo::msgs::protobuf_AddDesc_quaternion_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\013Range.proto\022\020sensor_msgs.msgs\032\020quatern"
    "ion.proto\"\304\001\n\005Range\022\021\n\ttime_usec\030\001 \002(\003\022\024"
    "\n\014min_distance\030\002 \002(\002\022\024\n\014max_distance\030\003 \002"
    "(\002\022\030\n\020current_distance\030\004 \002(\002\022\r\n\005h_fov\030\005 "
    "\001(\002\022\r\n\005v_fov\030\006 \001(\002\022,\n\013orientation\030\007 \001(\0132"
    "\027.gazebo.msgs.Quaternion\022\026\n\016signal_quali"
    "ty\030\010 \001(\005", 248);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "Range.proto", &protobuf_RegisterTypes);
  Range::default_instance_ = new Range();
  Range::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_Range_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_Range_2eproto {
  StaticDescriptorInitializer_Range_2eproto() {
    protobuf_AddDesc_Range_2eproto();
  }
} static_descriptor_initializer_Range_2eproto_;

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Range::kTimeUsecFieldNumber;
const int Range::kMinDistanceFieldNumber;
const int Range::kMaxDistanceFieldNumber;
const int Range::kCurrentDistanceFieldNumber;
const int Range::kHFovFieldNumber;
const int Range::kVFovFieldNumber;
const int Range::kOrientationFieldNumber;
const int Range::kSignalQualityFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Range::Range()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:sensor_msgs.msgs.Range)
}

void Range::InitAsDefaultInstance() {
  orientation_ = const_cast< ::gazebo::msgs::Quaternion*>(&::gazebo::msgs::Quaternion::default_instance());
}

Range::Range(const Range& from)
  : ::google::protobuf::Message(),
    _internal_metadata_(NULL) {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:sensor_msgs.msgs.Range)
}

void Range::SharedCtor() {
  _cached_size_ = 0;
  time_usec_ = GOOGLE_LONGLONG(0);
  min_distance_ = 0;
  max_distance_ = 0;
  current_distance_ = 0;
  h_fov_ = 0;
  v_fov_ = 0;
  orientation_ = NULL;
  signal_quality_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Range::~Range() {
  // @@protoc_insertion_point(destructor:sensor_msgs.msgs.Range)
  SharedDtor();
}

void Range::SharedDtor() {
  if (this != default_instance_) {
    delete orientation_;
  }
}

void Range::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Range::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Range_descriptor_;
}

const Range& Range::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_Range_2eproto();
  return *default_instance_;
}

Range* Range::default_instance_ = NULL;

Range* Range::New(::google::protobuf::Arena* arena) const {
  Range* n = new Range;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Range::Clear() {
// @@protoc_insertion_point(message_clear_start:sensor_msgs.msgs.Range)
#if defined(__clang__)
#define ZR_HELPER_(f) \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Winvalid-offsetof\"") \
  __builtin_offsetof(Range, f) \
  _Pragma("clang diagnostic pop")
#else
#define ZR_HELPER_(f) reinterpret_cast<char*>(\
  &reinterpret_cast<Range*>(16)->f)
#endif

#define ZR_(first, last) do {\
  ::memset(&first, 0,\
           ZR_HELPER_(last) - ZR_HELPER_(first) + sizeof(last));\
} while (0)

  if (_has_bits_[0 / 32] & 255u) {
    ZR_(time_usec_, h_fov_);
    ZR_(v_fov_, signal_quality_);
    if (has_orientation()) {
      if (orientation_ != NULL) orientation_->::gazebo::msgs::Quaternion::Clear();
    }
  }

#undef ZR_HELPER_
#undef ZR_

  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  if (_internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->Clear();
  }
}

bool Range::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:sensor_msgs.msgs.Range)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // required int64 time_usec = 1;
      case 1: {
        if (tag == 8) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int64, ::google::protobuf::internal::WireFormatLite::TYPE_INT64>(
                 input, &time_usec_)));
          set_has_time_usec();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(21)) goto parse_min_distance;
        break;
      }

      // required float min_distance = 2;
      case 2: {
        if (tag == 21) {
         parse_min_distance:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &min_distance_)));
          set_has_min_distance();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(29)) goto parse_max_distance;
        break;
      }

      // required float max_distance = 3;
      case 3: {
        if (tag == 29) {
         parse_max_distance:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &max_distance_)));
          set_has_max_distance();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(37)) goto parse_current_distance;
        break;
      }

      // required float current_distance = 4;
      case 4: {
        if (tag == 37) {
         parse_current_distance:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &current_distance_)));
          set_has_current_distance();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(45)) goto parse_h_fov;
        break;
      }

      // optional float h_fov = 5;
      case 5: {
        if (tag == 45) {
         parse_h_fov:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &h_fov_)));
          set_has_h_fov();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(53)) goto parse_v_fov;
        break;
      }

      // optional float v_fov = 6;
      case 6: {
        if (tag == 53) {
         parse_v_fov:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &v_fov_)));
          set_has_v_fov();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(58)) goto parse_orientation;
        break;
      }

      // optional .gazebo.msgs.Quaternion orientation = 7;
      case 7: {
        if (tag == 58) {
         parse_orientation:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_orientation()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(64)) goto parse_signal_quality;
        break;
      }

      // optional int32 signal_quality = 8;
      case 8: {
        if (tag == 64) {
         parse_signal_quality:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &signal_quality_)));
          set_has_signal_quality();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:sensor_msgs.msgs.Range)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:sensor_msgs.msgs.Range)
  return false;
#undef DO_
}

void Range::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:sensor_msgs.msgs.Range)
  // required int64 time_usec = 1;
  if (has_time_usec()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt64(1, this->time_usec(), output);
  }

  // required float min_distance = 2;
  if (has_min_distance()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->min_distance(), output);
  }

  // required float max_distance = 3;
  if (has_max_distance()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->max_distance(), output);
  }

  // required float current_distance = 4;
  if (has_current_distance()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(4, this->current_distance(), output);
  }

  // optional float h_fov = 5;
  if (has_h_fov()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(5, this->h_fov(), output);
  }

  // optional float v_fov = 6;
  if (has_v_fov()) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(6, this->v_fov(), output);
  }

  // optional .gazebo.msgs.Quaternion orientation = 7;
  if (has_orientation()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      7, *this->orientation_, output);
  }

  // optional int32 signal_quality = 8;
  if (has_signal_quality()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(8, this->signal_quality(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:sensor_msgs.msgs.Range)
}

::google::protobuf::uint8* Range::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:sensor_msgs.msgs.Range)
  // required int64 time_usec = 1;
  if (has_time_usec()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt64ToArray(1, this->time_usec(), target);
  }

  // required float min_distance = 2;
  if (has_min_distance()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->min_distance(), target);
  }

  // required float max_distance = 3;
  if (has_max_distance()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->max_distance(), target);
  }

  // required float current_distance = 4;
  if (has_current_distance()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(4, this->current_distance(), target);
  }

  // optional float h_fov = 5;
  if (has_h_fov()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(5, this->h_fov(), target);
  }

  // optional float v_fov = 6;
  if (has_v_fov()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(6, this->v_fov(), target);
  }

  // optional .gazebo.msgs.Quaternion orientation = 7;
  if (has_orientation()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        7, *this->orientation_, false, target);
  }

  // optional int32 signal_quality = 8;
  if (has_signal_quality()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(8, this->signal_quality(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:sensor_msgs.msgs.Range)
  return target;
}

int Range::RequiredFieldsByteSizeFallback() const {
// @@protoc_insertion_point(required_fields_byte_size_fallback_start:sensor_msgs.msgs.Range)
  int total_size = 0;

  if (has_time_usec()) {
    // required int64 time_usec = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int64Size(
        this->time_usec());
  }

  if (has_min_distance()) {
    // required float min_distance = 2;
    total_size += 1 + 4;
  }

  if (has_max_distance()) {
    // required float max_distance = 3;
    total_size += 1 + 4;
  }

  if (has_current_distance()) {
    // required float current_distance = 4;
    total_size += 1 + 4;
  }

  return total_size;
}
int Range::ByteSize() const {
// @@protoc_insertion_point(message_byte_size_start:sensor_msgs.msgs.Range)
  int total_size = 0;

  if (((_has_bits_[0] & 0x0000000f) ^ 0x0000000f) == 0) {  // All required fields are present.
    // required int64 time_usec = 1;
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int64Size(
        this->time_usec());

    // required float min_distance = 2;
    total_size += 1 + 4;

    // required float max_distance = 3;
    total_size += 1 + 4;

    // required float current_distance = 4;
    total_size += 1 + 4;

  } else {
    total_size += RequiredFieldsByteSizeFallback();
  }
  if (_has_bits_[4 / 32] & 240u) {
    // optional float h_fov = 5;
    if (has_h_fov()) {
      total_size += 1 + 4;
    }

    // optional float v_fov = 6;
    if (has_v_fov()) {
      total_size += 1 + 4;
    }

    // optional .gazebo.msgs.Quaternion orientation = 7;
    if (has_orientation()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          *this->orientation_);
    }

    // optional int32 signal_quality = 8;
    if (has_signal_quality()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->signal_quality());
    }

  }
  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Range::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:sensor_msgs.msgs.Range)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  const Range* source = 
      ::google::protobuf::internal::DynamicCastToGenerated<const Range>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:sensor_msgs.msgs.Range)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:sensor_msgs.msgs.Range)
    MergeFrom(*source);
  }
}

void Range::MergeFrom(const Range& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:sensor_msgs.msgs.Range)
  if (GOOGLE_PREDICT_FALSE(&from == this)) {
    ::google::protobuf::internal::MergeFromFail(__FILE__, __LINE__);
  }
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_time_usec()) {
      set_time_usec(from.time_usec());
    }
    if (from.has_min_distance()) {
      set_min_distance(from.min_distance());
    }
    if (from.has_max_distance()) {
      set_max_distance(from.max_distance());
    }
    if (from.has_current_distance()) {
      set_current_distance(from.current_distance());
    }
    if (from.has_h_fov()) {
      set_h_fov(from.h_fov());
    }
    if (from.has_v_fov()) {
      set_v_fov(from.v_fov());
    }
    if (from.has_orientation()) {
      mutable_orientation()->::gazebo::msgs::Quaternion::MergeFrom(from.orientation());
    }
    if (from.has_signal_quality()) {
      set_signal_quality(from.signal_quality());
    }
  }
  if (from._internal_metadata_.have_unknown_fields()) {
    mutable_unknown_fields()->MergeFrom(from.unknown_fields());
  }
}

void Range::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:sensor_msgs.msgs.Range)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Range::CopyFrom(const Range& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:sensor_msgs.msgs.Range)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Range::IsInitialized() const {
  if ((_has_bits_[0] & 0x0000000f) != 0x0000000f) return false;

  if (has_orientation()) {
    if (!this->orientation_->IsInitialized()) return false;
  }
  return true;
}

void Range::Swap(Range* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Range::InternalSwap(Range* other) {
  std::swap(time_usec_, other->time_usec_);
  std::swap(min_distance_, other->min_distance_);
  std::swap(max_distance_, other->max_distance_);
  std::swap(current_distance_, other->current_distance_);
  std::swap(h_fov_, other->h_fov_);
  std::swap(v_fov_, other->v_fov_);
  std::swap(orientation_, other->orientation_);
  std::swap(signal_quality_, other->signal_quality_);
  std::swap(_has_bits_[0], other->_has_bits_[0]);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Range::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Range_descriptor_;
  metadata.reflection = Range_reflection_;
  return metadata;
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// Range

// required int64 time_usec = 1;
bool Range::has_time_usec() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
void Range::set_has_time_usec() {
  _has_bits_[0] |= 0x00000001u;
}
void Range::clear_has_time_usec() {
  _has_bits_[0] &= ~0x00000001u;
}
void Range::clear_time_usec() {
  time_usec_ = GOOGLE_LONGLONG(0);
  clear_has_time_usec();
}
 ::google::protobuf::int64 Range::time_usec() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.Range.time_usec)
  return time_usec_;
}
 void Range::set_time_usec(::google::protobuf::int64 value) {
  set_has_time_usec();
  time_usec_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.Range.time_usec)
}

// required float min_distance = 2;
bool Range::has_min_distance() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
void Range::set_has_min_distance() {
  _has_bits_[0] |= 0x00000002u;
}
void Range::clear_has_min_distance() {
  _has_bits_[0] &= ~0x00000002u;
}
void Range::clear_min_distance() {
  min_distance_ = 0;
  clear_has_min_distance();
}
 float Range::min_distance() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.Range.min_distance)
  return min_distance_;
}
 void Range::set_min_distance(float value) {
  set_has_min_distance();
  min_distance_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.Range.min_distance)
}

// required float max_distance = 3;
bool Range::has_max_distance() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
void Range::set_has_max_distance() {
  _has_bits_[0] |= 0x00000004u;
}
void Range::clear_has_max_distance() {
  _has_bits_[0] &= ~0x00000004u;
}
void Range::clear_max_distance() {
  max_distance_ = 0;
  clear_has_max_distance();
}
 float Range::max_distance() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.Range.max_distance)
  return max_distance_;
}
 void Range::set_max_distance(float value) {
  set_has_max_distance();
  max_distance_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.Range.max_distance)
}

// required float current_distance = 4;
bool Range::has_current_distance() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
void Range::set_has_current_distance() {
  _has_bits_[0] |= 0x00000008u;
}
void Range::clear_has_current_distance() {
  _has_bits_[0] &= ~0x00000008u;
}
void Range::clear_current_distance() {
  current_distance_ = 0;
  clear_has_current_distance();
}
 float Range::current_distance() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.Range.current_distance)
  return current_distance_;
}
 void Range::set_current_distance(float value) {
  set_has_current_distance();
  current_distance_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.Range.current_distance)
}

// optional float h_fov = 5;
bool Range::has_h_fov() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
void Range::set_has_h_fov() {
  _has_bits_[0] |= 0x00000010u;
}
void Range::clear_has_h_fov() {
  _has_bits_[0] &= ~0x00000010u;
}
void Range::clear_h_fov() {
  h_fov_ = 0;
  clear_has_h_fov();
}
 float Range::h_fov() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.Range.h_fov)
  return h_fov_;
}
 void Range::set_h_fov(float value) {
  set_has_h_fov();
  h_fov_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.Range.h_fov)
}

// optional float v_fov = 6;
bool Range::has_v_fov() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
void Range::set_has_v_fov() {
  _has_bits_[0] |= 0x00000020u;
}
void Range::clear_has_v_fov() {
  _has_bits_[0] &= ~0x00000020u;
}
void Range::clear_v_fov() {
  v_fov_ = 0;
  clear_has_v_fov();
}
 float Range::v_fov() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.Range.v_fov)
  return v_fov_;
}
 void Range::set_v_fov(float value) {
  set_has_v_fov();
  v_fov_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.Range.v_fov)
}

// optional .gazebo.msgs.Quaternion orientation = 7;
bool Range::has_orientation() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
void Range::set_has_orientation() {
  _has_bits_[0] |= 0x00000040u;
}
void Range::clear_has_orientation() {
  _has_bits_[0] &= ~0x00000040u;
}
void Range::clear_orientation() {
  if (orientation_ != NULL) orientation_->::gazebo::msgs::Quaternion::Clear();
  clear_has_orientation();
}
const ::gazebo::msgs::Quaternion& Range::orientation() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.Range.orientation)
  return orientation_ != NULL ? *orientation_ : *default_instance_->orientation_;
}
::gazebo::msgs::Quaternion* Range::mutable_orientation() {
  set_has_orientation();
  if (orientation_ == NULL) {
    orientation_ = new ::gazebo::msgs::Quaternion;
  }
  // @@protoc_insertion_point(field_mutable:sensor_msgs.msgs.Range.orientation)
  return orientation_;
}
::gazebo::msgs::Quaternion* Range::release_orientation() {
  // @@protoc_insertion_point(field_release:sensor_msgs.msgs.Range.orientation)
  clear_has_orientation();
  ::gazebo::msgs::Quaternion* temp = orientation_;
  orientation_ = NULL;
  return temp;
}
void Range::set_allocated_orientation(::gazebo::msgs::Quaternion* orientation) {
  delete orientation_;
  orientation_ = orientation;
  if (orientation) {
    set_has_orientation();
  } else {
    clear_has_orientation();
  }
  // @@protoc_insertion_point(field_set_allocated:sensor_msgs.msgs.Range.orientation)
}

// optional int32 signal_quality = 8;
bool Range::has_signal_quality() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
void Range::set_has_signal_quality() {
  _has_bits_[0] |= 0x00000080u;
}
void Range::clear_has_signal_quality() {
  _has_bits_[0] &= ~0x00000080u;
}
void Range::clear_signal_quality() {
  signal_quality_ = 0;
  clear_has_signal_quality();
}
 ::google::protobuf::int32 Range::signal_quality() const {
  // @@protoc_insertion_point(field_get:sensor_msgs.msgs.Range.signal_quality)
  return signal_quality_;
}
 void Range::set_signal_quality(::google::protobuf::int32 value) {
  set_has_signal_quality();
  signal_quality_ = value;
  // @@protoc_insertion_point(field_set:sensor_msgs.msgs.Range.signal_quality)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace msgs
}  // namespace sensor_msgs

// @@protoc_insertion_point(global_scope)
