// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: bosdyn/api/world_object_service.proto
// Original file comments:
// Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
//
// Downloading, reproducing, distributing or otherwise using the SDK Software
// is subject to the terms and conditions of the Boston Dynamics Software
// Development Kit License (20191101-BDSDK-SL).
//
#ifndef GRPC_bosdyn_2fapi_2fworld_5fobject_5fservice_2eproto__INCLUDED
#define GRPC_bosdyn_2fapi_2fworld_5fobject_5fservice_2eproto__INCLUDED

#include "bosdyn/api/world_object_service.pb.h"

#include <functional>
#include <grpcpp/generic/async_generic_service.h>
#include <grpcpp/support/async_stream.h>
#include <grpcpp/support/async_unary_call.h>
#include <grpcpp/support/client_callback.h>
#include <grpcpp/client_context.h>
#include <grpcpp/completion_queue.h>
#include <grpcpp/support/message_allocator.h>
#include <grpcpp/support/method_handler.h>
#include <grpcpp/impl/codegen/proto_utils.h>
#include <grpcpp/impl/rpc_method.h>
#include <grpcpp/support/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/server_context.h>
#include <grpcpp/impl/service_type.h>
#include <grpcpp/impl/codegen/status.h>
#include <grpcpp/support/stub_options.h>
#include <grpcpp/support/sync_stream.h>

namespace bosdyn {
namespace api {

// The world object service provides a way to track and store objects detected in the world around the robot.
class WorldObjectService final {
 public:
  static constexpr char const* service_full_name() {
    return "bosdyn.api.WorldObjectService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    // Request a list of all the world objects in the robot's perception scene.
    virtual ::grpc::Status ListWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest& request, ::bosdyn::api::ListWorldObjectResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListWorldObjectResponse>> AsyncListWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListWorldObjectResponse>>(AsyncListWorldObjectsRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListWorldObjectResponse>> PrepareAsyncListWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListWorldObjectResponse>>(PrepareAsyncListWorldObjectsRaw(context, request, cq));
    }
    // Mutate (add, change, or delete) the world objects.
    virtual ::grpc::Status MutateWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest& request, ::bosdyn::api::MutateWorldObjectResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::MutateWorldObjectResponse>> AsyncMutateWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::MutateWorldObjectResponse>>(AsyncMutateWorldObjectsRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::MutateWorldObjectResponse>> PrepareAsyncMutateWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::MutateWorldObjectResponse>>(PrepareAsyncMutateWorldObjectsRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      // Request a list of all the world objects in the robot's perception scene.
      virtual void ListWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest* request, ::bosdyn::api::ListWorldObjectResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void ListWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest* request, ::bosdyn::api::ListWorldObjectResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
      // Mutate (add, change, or delete) the world objects.
      virtual void MutateWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest* request, ::bosdyn::api::MutateWorldObjectResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void MutateWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest* request, ::bosdyn::api::MutateWorldObjectResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListWorldObjectResponse>* AsyncListWorldObjectsRaw(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListWorldObjectResponse>* PrepareAsyncListWorldObjectsRaw(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::MutateWorldObjectResponse>* AsyncMutateWorldObjectsRaw(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::MutateWorldObjectResponse>* PrepareAsyncMutateWorldObjectsRaw(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status ListWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest& request, ::bosdyn::api::ListWorldObjectResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListWorldObjectResponse>> AsyncListWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListWorldObjectResponse>>(AsyncListWorldObjectsRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListWorldObjectResponse>> PrepareAsyncListWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListWorldObjectResponse>>(PrepareAsyncListWorldObjectsRaw(context, request, cq));
    }
    ::grpc::Status MutateWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest& request, ::bosdyn::api::MutateWorldObjectResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::MutateWorldObjectResponse>> AsyncMutateWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::MutateWorldObjectResponse>>(AsyncMutateWorldObjectsRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::MutateWorldObjectResponse>> PrepareAsyncMutateWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::MutateWorldObjectResponse>>(PrepareAsyncMutateWorldObjectsRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void ListWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest* request, ::bosdyn::api::ListWorldObjectResponse* response, std::function<void(::grpc::Status)>) override;
      void ListWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest* request, ::bosdyn::api::ListWorldObjectResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
      void MutateWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest* request, ::bosdyn::api::MutateWorldObjectResponse* response, std::function<void(::grpc::Status)>) override;
      void MutateWorldObjects(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest* request, ::bosdyn::api::MutateWorldObjectResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
     private:
      friend class Stub;
      explicit async(Stub* stub): stub_(stub) { }
      Stub* stub() { return stub_; }
      Stub* stub_;
    };
    class async* async() override { return &async_stub_; }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    class async async_stub_{this};
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListWorldObjectResponse>* AsyncListWorldObjectsRaw(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListWorldObjectResponse>* PrepareAsyncListWorldObjectsRaw(::grpc::ClientContext* context, const ::bosdyn::api::ListWorldObjectRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::MutateWorldObjectResponse>* AsyncMutateWorldObjectsRaw(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::MutateWorldObjectResponse>* PrepareAsyncMutateWorldObjectsRaw(::grpc::ClientContext* context, const ::bosdyn::api::MutateWorldObjectRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_ListWorldObjects_;
    const ::grpc::internal::RpcMethod rpcmethod_MutateWorldObjects_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    // Request a list of all the world objects in the robot's perception scene.
    virtual ::grpc::Status ListWorldObjects(::grpc::ServerContext* context, const ::bosdyn::api::ListWorldObjectRequest* request, ::bosdyn::api::ListWorldObjectResponse* response);
    // Mutate (add, change, or delete) the world objects.
    virtual ::grpc::Status MutateWorldObjects(::grpc::ServerContext* context, const ::bosdyn::api::MutateWorldObjectRequest* request, ::bosdyn::api::MutateWorldObjectResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_ListWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_ListWorldObjects() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_ListWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListWorldObjectRequest* /*request*/, ::bosdyn::api::ListWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestListWorldObjects(::grpc::ServerContext* context, ::bosdyn::api::ListWorldObjectRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::ListWorldObjectResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_MutateWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_MutateWorldObjects() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_MutateWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status MutateWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::MutateWorldObjectRequest* /*request*/, ::bosdyn::api::MutateWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestMutateWorldObjects(::grpc::ServerContext* context, ::bosdyn::api::MutateWorldObjectRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::MutateWorldObjectResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_ListWorldObjects<WithAsyncMethod_MutateWorldObjects<Service > > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_ListWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_ListWorldObjects() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::ListWorldObjectRequest, ::bosdyn::api::ListWorldObjectResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::ListWorldObjectRequest* request, ::bosdyn::api::ListWorldObjectResponse* response) { return this->ListWorldObjects(context, request, response); }));}
    void SetMessageAllocatorFor_ListWorldObjects(
        ::grpc::MessageAllocator< ::bosdyn::api::ListWorldObjectRequest, ::bosdyn::api::ListWorldObjectResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::ListWorldObjectRequest, ::bosdyn::api::ListWorldObjectResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_ListWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListWorldObjectRequest* /*request*/, ::bosdyn::api::ListWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* ListWorldObjects(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::ListWorldObjectRequest* /*request*/, ::bosdyn::api::ListWorldObjectResponse* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithCallbackMethod_MutateWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_MutateWorldObjects() {
      ::grpc::Service::MarkMethodCallback(1,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::MutateWorldObjectRequest, ::bosdyn::api::MutateWorldObjectResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::MutateWorldObjectRequest* request, ::bosdyn::api::MutateWorldObjectResponse* response) { return this->MutateWorldObjects(context, request, response); }));}
    void SetMessageAllocatorFor_MutateWorldObjects(
        ::grpc::MessageAllocator< ::bosdyn::api::MutateWorldObjectRequest, ::bosdyn::api::MutateWorldObjectResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(1);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::MutateWorldObjectRequest, ::bosdyn::api::MutateWorldObjectResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_MutateWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status MutateWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::MutateWorldObjectRequest* /*request*/, ::bosdyn::api::MutateWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* MutateWorldObjects(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::MutateWorldObjectRequest* /*request*/, ::bosdyn::api::MutateWorldObjectResponse* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_ListWorldObjects<WithCallbackMethod_MutateWorldObjects<Service > > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_ListWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_ListWorldObjects() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_ListWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListWorldObjectRequest* /*request*/, ::bosdyn::api::ListWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_MutateWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_MutateWorldObjects() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_MutateWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status MutateWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::MutateWorldObjectRequest* /*request*/, ::bosdyn::api::MutateWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_ListWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_ListWorldObjects() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_ListWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListWorldObjectRequest* /*request*/, ::bosdyn::api::ListWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestListWorldObjects(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawMethod_MutateWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_MutateWorldObjects() {
      ::grpc::Service::MarkMethodRaw(1);
    }
    ~WithRawMethod_MutateWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status MutateWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::MutateWorldObjectRequest* /*request*/, ::bosdyn::api::MutateWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestMutateWorldObjects(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_ListWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_ListWorldObjects() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->ListWorldObjects(context, request, response); }));
    }
    ~WithRawCallbackMethod_ListWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListWorldObjectRequest* /*request*/, ::bosdyn::api::ListWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* ListWorldObjects(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_MutateWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_MutateWorldObjects() {
      ::grpc::Service::MarkMethodRawCallback(1,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->MutateWorldObjects(context, request, response); }));
    }
    ~WithRawCallbackMethod_MutateWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status MutateWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::MutateWorldObjectRequest* /*request*/, ::bosdyn::api::MutateWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* MutateWorldObjects(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_ListWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_ListWorldObjects() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::ListWorldObjectRequest, ::bosdyn::api::ListWorldObjectResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::ListWorldObjectRequest, ::bosdyn::api::ListWorldObjectResponse>* streamer) {
                       return this->StreamedListWorldObjects(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_ListWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status ListWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListWorldObjectRequest* /*request*/, ::bosdyn::api::ListWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedListWorldObjects(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::ListWorldObjectRequest,::bosdyn::api::ListWorldObjectResponse>* server_unary_streamer) = 0;
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_MutateWorldObjects : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_MutateWorldObjects() {
      ::grpc::Service::MarkMethodStreamed(1,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::MutateWorldObjectRequest, ::bosdyn::api::MutateWorldObjectResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::MutateWorldObjectRequest, ::bosdyn::api::MutateWorldObjectResponse>* streamer) {
                       return this->StreamedMutateWorldObjects(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_MutateWorldObjects() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status MutateWorldObjects(::grpc::ServerContext* /*context*/, const ::bosdyn::api::MutateWorldObjectRequest* /*request*/, ::bosdyn::api::MutateWorldObjectResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedMutateWorldObjects(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::MutateWorldObjectRequest,::bosdyn::api::MutateWorldObjectResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_ListWorldObjects<WithStreamedUnaryMethod_MutateWorldObjects<Service > > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_ListWorldObjects<WithStreamedUnaryMethod_MutateWorldObjects<Service > > StreamedService;
};

}  // namespace api
}  // namespace bosdyn


#endif  // GRPC_bosdyn_2fapi_2fworld_5fobject_5fservice_2eproto__INCLUDED
