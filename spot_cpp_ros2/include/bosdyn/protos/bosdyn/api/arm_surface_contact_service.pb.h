// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: bosdyn/api/arm_surface_contact_service.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_bosdyn_2fapi_2farm_5fsurface_5fcontact_5fservice_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_bosdyn_2fapi_2farm_5fsurface_5fcontact_5fservice_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3021000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3021012 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
#include "bosdyn/api/header.pb.h"
#include "bosdyn/api/lease.pb.h"
#include "bosdyn/api/arm_surface_contact.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_bosdyn_2fapi_2farm_5fsurface_5fcontact_5fservice_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_bosdyn_2fapi_2farm_5fsurface_5fcontact_5fservice_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_bosdyn_2fapi_2farm_5fsurface_5fcontact_5fservice_2eproto;
namespace bosdyn {
namespace api {
class ArmSurfaceContactCommand;
struct ArmSurfaceContactCommandDefaultTypeInternal;
extern ArmSurfaceContactCommandDefaultTypeInternal _ArmSurfaceContactCommand_default_instance_;
class ArmSurfaceContactResponse;
struct ArmSurfaceContactResponseDefaultTypeInternal;
extern ArmSurfaceContactResponseDefaultTypeInternal _ArmSurfaceContactResponse_default_instance_;
}  // namespace api
}  // namespace bosdyn
PROTOBUF_NAMESPACE_OPEN
template<> ::bosdyn::api::ArmSurfaceContactCommand* Arena::CreateMaybeMessage<::bosdyn::api::ArmSurfaceContactCommand>(Arena*);
template<> ::bosdyn::api::ArmSurfaceContactResponse* Arena::CreateMaybeMessage<::bosdyn::api::ArmSurfaceContactResponse>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace bosdyn {
namespace api {

// ===================================================================

class ArmSurfaceContactCommand final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:bosdyn.api.ArmSurfaceContactCommand) */ {
 public:
  inline ArmSurfaceContactCommand() : ArmSurfaceContactCommand(nullptr) {}
  ~ArmSurfaceContactCommand() override;
  explicit PROTOBUF_CONSTEXPR ArmSurfaceContactCommand(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  ArmSurfaceContactCommand(const ArmSurfaceContactCommand& from);
  ArmSurfaceContactCommand(ArmSurfaceContactCommand&& from) noexcept
    : ArmSurfaceContactCommand() {
    *this = ::std::move(from);
  }

  inline ArmSurfaceContactCommand& operator=(const ArmSurfaceContactCommand& from) {
    CopyFrom(from);
    return *this;
  }
  inline ArmSurfaceContactCommand& operator=(ArmSurfaceContactCommand&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const ArmSurfaceContactCommand& default_instance() {
    return *internal_default_instance();
  }
  static inline const ArmSurfaceContactCommand* internal_default_instance() {
    return reinterpret_cast<const ArmSurfaceContactCommand*>(
               &_ArmSurfaceContactCommand_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ArmSurfaceContactCommand& a, ArmSurfaceContactCommand& b) {
    a.Swap(&b);
  }
  inline void Swap(ArmSurfaceContactCommand* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(ArmSurfaceContactCommand* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  ArmSurfaceContactCommand* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<ArmSurfaceContactCommand>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const ArmSurfaceContactCommand& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const ArmSurfaceContactCommand& from) {
    ArmSurfaceContactCommand::MergeImpl(*this, from);
  }
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _impl_._cached_size_.Get(); }

  private:
  void SharedCtor(::PROTOBUF_NAMESPACE_ID::Arena* arena, bool is_message_owned);
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(ArmSurfaceContactCommand* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "bosdyn.api.ArmSurfaceContactCommand";
  }
  protected:
  explicit ArmSurfaceContactCommand(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kHeaderFieldNumber = 1,
    kLeaseFieldNumber = 2,
    kRequestFieldNumber = 4,
  };
  // .bosdyn.api.RequestHeader header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::bosdyn::api::RequestHeader& header() const;
  PROTOBUF_NODISCARD ::bosdyn::api::RequestHeader* release_header();
  ::bosdyn::api::RequestHeader* mutable_header();
  void set_allocated_header(::bosdyn::api::RequestHeader* header);
  private:
  const ::bosdyn::api::RequestHeader& _internal_header() const;
  ::bosdyn::api::RequestHeader* _internal_mutable_header();
  public:
  void unsafe_arena_set_allocated_header(
      ::bosdyn::api::RequestHeader* header);
  ::bosdyn::api::RequestHeader* unsafe_arena_release_header();

  // .bosdyn.api.Lease lease = 2;
  bool has_lease() const;
  private:
  bool _internal_has_lease() const;
  public:
  void clear_lease();
  const ::bosdyn::api::Lease& lease() const;
  PROTOBUF_NODISCARD ::bosdyn::api::Lease* release_lease();
  ::bosdyn::api::Lease* mutable_lease();
  void set_allocated_lease(::bosdyn::api::Lease* lease);
  private:
  const ::bosdyn::api::Lease& _internal_lease() const;
  ::bosdyn::api::Lease* _internal_mutable_lease();
  public:
  void unsafe_arena_set_allocated_lease(
      ::bosdyn::api::Lease* lease);
  ::bosdyn::api::Lease* unsafe_arena_release_lease();

  // .bosdyn.api.ArmSurfaceContact.Request request = 4;
  bool has_request() const;
  private:
  bool _internal_has_request() const;
  public:
  void clear_request();
  const ::bosdyn::api::ArmSurfaceContact_Request& request() const;
  PROTOBUF_NODISCARD ::bosdyn::api::ArmSurfaceContact_Request* release_request();
  ::bosdyn::api::ArmSurfaceContact_Request* mutable_request();
  void set_allocated_request(::bosdyn::api::ArmSurfaceContact_Request* request);
  private:
  const ::bosdyn::api::ArmSurfaceContact_Request& _internal_request() const;
  ::bosdyn::api::ArmSurfaceContact_Request* _internal_mutable_request();
  public:
  void unsafe_arena_set_allocated_request(
      ::bosdyn::api::ArmSurfaceContact_Request* request);
  ::bosdyn::api::ArmSurfaceContact_Request* unsafe_arena_release_request();

  // @@protoc_insertion_point(class_scope:bosdyn.api.ArmSurfaceContactCommand)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::bosdyn::api::RequestHeader* header_;
    ::bosdyn::api::Lease* lease_;
    ::bosdyn::api::ArmSurfaceContact_Request* request_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_bosdyn_2fapi_2farm_5fsurface_5fcontact_5fservice_2eproto;
};
// -------------------------------------------------------------------

class ArmSurfaceContactResponse final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:bosdyn.api.ArmSurfaceContactResponse) */ {
 public:
  inline ArmSurfaceContactResponse() : ArmSurfaceContactResponse(nullptr) {}
  ~ArmSurfaceContactResponse() override;
  explicit PROTOBUF_CONSTEXPR ArmSurfaceContactResponse(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  ArmSurfaceContactResponse(const ArmSurfaceContactResponse& from);
  ArmSurfaceContactResponse(ArmSurfaceContactResponse&& from) noexcept
    : ArmSurfaceContactResponse() {
    *this = ::std::move(from);
  }

  inline ArmSurfaceContactResponse& operator=(const ArmSurfaceContactResponse& from) {
    CopyFrom(from);
    return *this;
  }
  inline ArmSurfaceContactResponse& operator=(ArmSurfaceContactResponse&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const ArmSurfaceContactResponse& default_instance() {
    return *internal_default_instance();
  }
  static inline const ArmSurfaceContactResponse* internal_default_instance() {
    return reinterpret_cast<const ArmSurfaceContactResponse*>(
               &_ArmSurfaceContactResponse_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(ArmSurfaceContactResponse& a, ArmSurfaceContactResponse& b) {
    a.Swap(&b);
  }
  inline void Swap(ArmSurfaceContactResponse* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(ArmSurfaceContactResponse* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  ArmSurfaceContactResponse* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<ArmSurfaceContactResponse>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const ArmSurfaceContactResponse& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom( const ArmSurfaceContactResponse& from) {
    ArmSurfaceContactResponse::MergeImpl(*this, from);
  }
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _impl_._cached_size_.Get(); }

  private:
  void SharedCtor(::PROTOBUF_NAMESPACE_ID::Arena* arena, bool is_message_owned);
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(ArmSurfaceContactResponse* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "bosdyn.api.ArmSurfaceContactResponse";
  }
  protected:
  explicit ArmSurfaceContactResponse(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kHeaderFieldNumber = 1,
  };
  // .bosdyn.api.ResponseHeader header = 1;
  bool has_header() const;
  private:
  bool _internal_has_header() const;
  public:
  void clear_header();
  const ::bosdyn::api::ResponseHeader& header() const;
  PROTOBUF_NODISCARD ::bosdyn::api::ResponseHeader* release_header();
  ::bosdyn::api::ResponseHeader* mutable_header();
  void set_allocated_header(::bosdyn::api::ResponseHeader* header);
  private:
  const ::bosdyn::api::ResponseHeader& _internal_header() const;
  ::bosdyn::api::ResponseHeader* _internal_mutable_header();
  public:
  void unsafe_arena_set_allocated_header(
      ::bosdyn::api::ResponseHeader* header);
  ::bosdyn::api::ResponseHeader* unsafe_arena_release_header();

  // @@protoc_insertion_point(class_scope:bosdyn.api.ArmSurfaceContactResponse)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  struct Impl_ {
    ::bosdyn::api::ResponseHeader* header_;
    mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  };
  union { Impl_ _impl_; };
  friend struct ::TableStruct_bosdyn_2fapi_2farm_5fsurface_5fcontact_5fservice_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ArmSurfaceContactCommand

// .bosdyn.api.RequestHeader header = 1;
inline bool ArmSurfaceContactCommand::_internal_has_header() const {
  return this != internal_default_instance() && _impl_.header_ != nullptr;
}
inline bool ArmSurfaceContactCommand::has_header() const {
  return _internal_has_header();
}
inline const ::bosdyn::api::RequestHeader& ArmSurfaceContactCommand::_internal_header() const {
  const ::bosdyn::api::RequestHeader* p = _impl_.header_;
  return p != nullptr ? *p : reinterpret_cast<const ::bosdyn::api::RequestHeader&>(
      ::bosdyn::api::_RequestHeader_default_instance_);
}
inline const ::bosdyn::api::RequestHeader& ArmSurfaceContactCommand::header() const {
  // @@protoc_insertion_point(field_get:bosdyn.api.ArmSurfaceContactCommand.header)
  return _internal_header();
}
inline void ArmSurfaceContactCommand::unsafe_arena_set_allocated_header(
    ::bosdyn::api::RequestHeader* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  _impl_.header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:bosdyn.api.ArmSurfaceContactCommand.header)
}
inline ::bosdyn::api::RequestHeader* ArmSurfaceContactCommand::release_header() {
  
  ::bosdyn::api::RequestHeader* temp = _impl_.header_;
  _impl_.header_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::bosdyn::api::RequestHeader* ArmSurfaceContactCommand::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:bosdyn.api.ArmSurfaceContactCommand.header)
  
  ::bosdyn::api::RequestHeader* temp = _impl_.header_;
  _impl_.header_ = nullptr;
  return temp;
}
inline ::bosdyn::api::RequestHeader* ArmSurfaceContactCommand::_internal_mutable_header() {
  
  if (_impl_.header_ == nullptr) {
    auto* p = CreateMaybeMessage<::bosdyn::api::RequestHeader>(GetArenaForAllocation());
    _impl_.header_ = p;
  }
  return _impl_.header_;
}
inline ::bosdyn::api::RequestHeader* ArmSurfaceContactCommand::mutable_header() {
  ::bosdyn::api::RequestHeader* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:bosdyn.api.ArmSurfaceContactCommand.header)
  return _msg;
}
inline void ArmSurfaceContactCommand::set_allocated_header(::bosdyn::api::RequestHeader* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header));
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.header_ = header;
  // @@protoc_insertion_point(field_set_allocated:bosdyn.api.ArmSurfaceContactCommand.header)
}

// .bosdyn.api.Lease lease = 2;
inline bool ArmSurfaceContactCommand::_internal_has_lease() const {
  return this != internal_default_instance() && _impl_.lease_ != nullptr;
}
inline bool ArmSurfaceContactCommand::has_lease() const {
  return _internal_has_lease();
}
inline const ::bosdyn::api::Lease& ArmSurfaceContactCommand::_internal_lease() const {
  const ::bosdyn::api::Lease* p = _impl_.lease_;
  return p != nullptr ? *p : reinterpret_cast<const ::bosdyn::api::Lease&>(
      ::bosdyn::api::_Lease_default_instance_);
}
inline const ::bosdyn::api::Lease& ArmSurfaceContactCommand::lease() const {
  // @@protoc_insertion_point(field_get:bosdyn.api.ArmSurfaceContactCommand.lease)
  return _internal_lease();
}
inline void ArmSurfaceContactCommand::unsafe_arena_set_allocated_lease(
    ::bosdyn::api::Lease* lease) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.lease_);
  }
  _impl_.lease_ = lease;
  if (lease) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:bosdyn.api.ArmSurfaceContactCommand.lease)
}
inline ::bosdyn::api::Lease* ArmSurfaceContactCommand::release_lease() {
  
  ::bosdyn::api::Lease* temp = _impl_.lease_;
  _impl_.lease_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::bosdyn::api::Lease* ArmSurfaceContactCommand::unsafe_arena_release_lease() {
  // @@protoc_insertion_point(field_release:bosdyn.api.ArmSurfaceContactCommand.lease)
  
  ::bosdyn::api::Lease* temp = _impl_.lease_;
  _impl_.lease_ = nullptr;
  return temp;
}
inline ::bosdyn::api::Lease* ArmSurfaceContactCommand::_internal_mutable_lease() {
  
  if (_impl_.lease_ == nullptr) {
    auto* p = CreateMaybeMessage<::bosdyn::api::Lease>(GetArenaForAllocation());
    _impl_.lease_ = p;
  }
  return _impl_.lease_;
}
inline ::bosdyn::api::Lease* ArmSurfaceContactCommand::mutable_lease() {
  ::bosdyn::api::Lease* _msg = _internal_mutable_lease();
  // @@protoc_insertion_point(field_mutable:bosdyn.api.ArmSurfaceContactCommand.lease)
  return _msg;
}
inline void ArmSurfaceContactCommand::set_allocated_lease(::bosdyn::api::Lease* lease) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.lease_);
  }
  if (lease) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(lease));
    if (message_arena != submessage_arena) {
      lease = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, lease, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.lease_ = lease;
  // @@protoc_insertion_point(field_set_allocated:bosdyn.api.ArmSurfaceContactCommand.lease)
}

// .bosdyn.api.ArmSurfaceContact.Request request = 4;
inline bool ArmSurfaceContactCommand::_internal_has_request() const {
  return this != internal_default_instance() && _impl_.request_ != nullptr;
}
inline bool ArmSurfaceContactCommand::has_request() const {
  return _internal_has_request();
}
inline const ::bosdyn::api::ArmSurfaceContact_Request& ArmSurfaceContactCommand::_internal_request() const {
  const ::bosdyn::api::ArmSurfaceContact_Request* p = _impl_.request_;
  return p != nullptr ? *p : reinterpret_cast<const ::bosdyn::api::ArmSurfaceContact_Request&>(
      ::bosdyn::api::_ArmSurfaceContact_Request_default_instance_);
}
inline const ::bosdyn::api::ArmSurfaceContact_Request& ArmSurfaceContactCommand::request() const {
  // @@protoc_insertion_point(field_get:bosdyn.api.ArmSurfaceContactCommand.request)
  return _internal_request();
}
inline void ArmSurfaceContactCommand::unsafe_arena_set_allocated_request(
    ::bosdyn::api::ArmSurfaceContact_Request* request) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.request_);
  }
  _impl_.request_ = request;
  if (request) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:bosdyn.api.ArmSurfaceContactCommand.request)
}
inline ::bosdyn::api::ArmSurfaceContact_Request* ArmSurfaceContactCommand::release_request() {
  
  ::bosdyn::api::ArmSurfaceContact_Request* temp = _impl_.request_;
  _impl_.request_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::bosdyn::api::ArmSurfaceContact_Request* ArmSurfaceContactCommand::unsafe_arena_release_request() {
  // @@protoc_insertion_point(field_release:bosdyn.api.ArmSurfaceContactCommand.request)
  
  ::bosdyn::api::ArmSurfaceContact_Request* temp = _impl_.request_;
  _impl_.request_ = nullptr;
  return temp;
}
inline ::bosdyn::api::ArmSurfaceContact_Request* ArmSurfaceContactCommand::_internal_mutable_request() {
  
  if (_impl_.request_ == nullptr) {
    auto* p = CreateMaybeMessage<::bosdyn::api::ArmSurfaceContact_Request>(GetArenaForAllocation());
    _impl_.request_ = p;
  }
  return _impl_.request_;
}
inline ::bosdyn::api::ArmSurfaceContact_Request* ArmSurfaceContactCommand::mutable_request() {
  ::bosdyn::api::ArmSurfaceContact_Request* _msg = _internal_mutable_request();
  // @@protoc_insertion_point(field_mutable:bosdyn.api.ArmSurfaceContactCommand.request)
  return _msg;
}
inline void ArmSurfaceContactCommand::set_allocated_request(::bosdyn::api::ArmSurfaceContact_Request* request) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.request_);
  }
  if (request) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(request));
    if (message_arena != submessage_arena) {
      request = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, request, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.request_ = request;
  // @@protoc_insertion_point(field_set_allocated:bosdyn.api.ArmSurfaceContactCommand.request)
}

// -------------------------------------------------------------------

// ArmSurfaceContactResponse

// .bosdyn.api.ResponseHeader header = 1;
inline bool ArmSurfaceContactResponse::_internal_has_header() const {
  return this != internal_default_instance() && _impl_.header_ != nullptr;
}
inline bool ArmSurfaceContactResponse::has_header() const {
  return _internal_has_header();
}
inline const ::bosdyn::api::ResponseHeader& ArmSurfaceContactResponse::_internal_header() const {
  const ::bosdyn::api::ResponseHeader* p = _impl_.header_;
  return p != nullptr ? *p : reinterpret_cast<const ::bosdyn::api::ResponseHeader&>(
      ::bosdyn::api::_ResponseHeader_default_instance_);
}
inline const ::bosdyn::api::ResponseHeader& ArmSurfaceContactResponse::header() const {
  // @@protoc_insertion_point(field_get:bosdyn.api.ArmSurfaceContactResponse.header)
  return _internal_header();
}
inline void ArmSurfaceContactResponse::unsafe_arena_set_allocated_header(
    ::bosdyn::api::ResponseHeader* header) {
  if (GetArenaForAllocation() == nullptr) {
    delete reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  _impl_.header_ = header;
  if (header) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_unsafe_arena_set_allocated:bosdyn.api.ArmSurfaceContactResponse.header)
}
inline ::bosdyn::api::ResponseHeader* ArmSurfaceContactResponse::release_header() {
  
  ::bosdyn::api::ResponseHeader* temp = _impl_.header_;
  _impl_.header_ = nullptr;
#ifdef PROTOBUF_FORCE_COPY_IN_RELEASE
  auto* old =  reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(temp);
  temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  if (GetArenaForAllocation() == nullptr) { delete old; }
#else  // PROTOBUF_FORCE_COPY_IN_RELEASE
  if (GetArenaForAllocation() != nullptr) {
    temp = ::PROTOBUF_NAMESPACE_ID::internal::DuplicateIfNonNull(temp);
  }
#endif  // !PROTOBUF_FORCE_COPY_IN_RELEASE
  return temp;
}
inline ::bosdyn::api::ResponseHeader* ArmSurfaceContactResponse::unsafe_arena_release_header() {
  // @@protoc_insertion_point(field_release:bosdyn.api.ArmSurfaceContactResponse.header)
  
  ::bosdyn::api::ResponseHeader* temp = _impl_.header_;
  _impl_.header_ = nullptr;
  return temp;
}
inline ::bosdyn::api::ResponseHeader* ArmSurfaceContactResponse::_internal_mutable_header() {
  
  if (_impl_.header_ == nullptr) {
    auto* p = CreateMaybeMessage<::bosdyn::api::ResponseHeader>(GetArenaForAllocation());
    _impl_.header_ = p;
  }
  return _impl_.header_;
}
inline ::bosdyn::api::ResponseHeader* ArmSurfaceContactResponse::mutable_header() {
  ::bosdyn::api::ResponseHeader* _msg = _internal_mutable_header();
  // @@protoc_insertion_point(field_mutable:bosdyn.api.ArmSurfaceContactResponse.header)
  return _msg;
}
inline void ArmSurfaceContactResponse::set_allocated_header(::bosdyn::api::ResponseHeader* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaForAllocation();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(_impl_.header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena =
        ::PROTOBUF_NAMESPACE_ID::Arena::InternalGetOwningArena(
                reinterpret_cast<::PROTOBUF_NAMESPACE_ID::MessageLite*>(header));
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  _impl_.header_ = header;
  // @@protoc_insertion_point(field_set_allocated:bosdyn.api.ArmSurfaceContactResponse.header)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace api
}  // namespace bosdyn

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_bosdyn_2fapi_2farm_5fsurface_5fcontact_5fservice_2eproto
