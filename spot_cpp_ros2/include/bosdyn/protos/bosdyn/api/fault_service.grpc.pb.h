// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: bosdyn/api/fault_service.proto
// Original file comments:
// Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
//
// Downloading, reproducing, distributing or otherwise using the SDK Software
// is subject to the terms and conditions of the Boston Dynamics Software
// Development Kit License (20191101-BDSDK-SL).
//
#ifndef GRPC_bosdyn_2fapi_2ffault_5fservice_2eproto__INCLUDED
#define GRPC_bosdyn_2fapi_2ffault_5fservice_2eproto__INCLUDED

#include "bosdyn/api/fault_service.pb.h"

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

// The service fault service enables modification of the robot state ServiceFaultState.
class FaultService final {
 public:
  static constexpr char const* service_full_name() {
    return "bosdyn.api.FaultService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    // Sends a ServiceFault to be reporting in robot state.
    virtual ::grpc::Status TriggerServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest& request, ::bosdyn::api::TriggerServiceFaultResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::TriggerServiceFaultResponse>> AsyncTriggerServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::TriggerServiceFaultResponse>>(AsyncTriggerServiceFaultRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::TriggerServiceFaultResponse>> PrepareAsyncTriggerServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::TriggerServiceFaultResponse>>(PrepareAsyncTriggerServiceFaultRaw(context, request, cq));
    }
    // Clears an active ServiceFault from robot state.
    virtual ::grpc::Status ClearServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest& request, ::bosdyn::api::ClearServiceFaultResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ClearServiceFaultResponse>> AsyncClearServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ClearServiceFaultResponse>>(AsyncClearServiceFaultRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ClearServiceFaultResponse>> PrepareAsyncClearServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ClearServiceFaultResponse>>(PrepareAsyncClearServiceFaultRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      // Sends a ServiceFault to be reporting in robot state.
      virtual void TriggerServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest* request, ::bosdyn::api::TriggerServiceFaultResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void TriggerServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest* request, ::bosdyn::api::TriggerServiceFaultResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
      // Clears an active ServiceFault from robot state.
      virtual void ClearServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest* request, ::bosdyn::api::ClearServiceFaultResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void ClearServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest* request, ::bosdyn::api::ClearServiceFaultResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::TriggerServiceFaultResponse>* AsyncTriggerServiceFaultRaw(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::TriggerServiceFaultResponse>* PrepareAsyncTriggerServiceFaultRaw(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ClearServiceFaultResponse>* AsyncClearServiceFaultRaw(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ClearServiceFaultResponse>* PrepareAsyncClearServiceFaultRaw(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status TriggerServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest& request, ::bosdyn::api::TriggerServiceFaultResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::TriggerServiceFaultResponse>> AsyncTriggerServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::TriggerServiceFaultResponse>>(AsyncTriggerServiceFaultRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::TriggerServiceFaultResponse>> PrepareAsyncTriggerServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::TriggerServiceFaultResponse>>(PrepareAsyncTriggerServiceFaultRaw(context, request, cq));
    }
    ::grpc::Status ClearServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest& request, ::bosdyn::api::ClearServiceFaultResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ClearServiceFaultResponse>> AsyncClearServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ClearServiceFaultResponse>>(AsyncClearServiceFaultRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ClearServiceFaultResponse>> PrepareAsyncClearServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ClearServiceFaultResponse>>(PrepareAsyncClearServiceFaultRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void TriggerServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest* request, ::bosdyn::api::TriggerServiceFaultResponse* response, std::function<void(::grpc::Status)>) override;
      void TriggerServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest* request, ::bosdyn::api::TriggerServiceFaultResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
      void ClearServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest* request, ::bosdyn::api::ClearServiceFaultResponse* response, std::function<void(::grpc::Status)>) override;
      void ClearServiceFault(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest* request, ::bosdyn::api::ClearServiceFaultResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
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
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::TriggerServiceFaultResponse>* AsyncTriggerServiceFaultRaw(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::TriggerServiceFaultResponse>* PrepareAsyncTriggerServiceFaultRaw(::grpc::ClientContext* context, const ::bosdyn::api::TriggerServiceFaultRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ClearServiceFaultResponse>* AsyncClearServiceFaultRaw(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ClearServiceFaultResponse>* PrepareAsyncClearServiceFaultRaw(::grpc::ClientContext* context, const ::bosdyn::api::ClearServiceFaultRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_TriggerServiceFault_;
    const ::grpc::internal::RpcMethod rpcmethod_ClearServiceFault_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    // Sends a ServiceFault to be reporting in robot state.
    virtual ::grpc::Status TriggerServiceFault(::grpc::ServerContext* context, const ::bosdyn::api::TriggerServiceFaultRequest* request, ::bosdyn::api::TriggerServiceFaultResponse* response);
    // Clears an active ServiceFault from robot state.
    virtual ::grpc::Status ClearServiceFault(::grpc::ServerContext* context, const ::bosdyn::api::ClearServiceFaultRequest* request, ::bosdyn::api::ClearServiceFaultResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_TriggerServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_TriggerServiceFault() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_TriggerServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status TriggerServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::TriggerServiceFaultRequest* /*request*/, ::bosdyn::api::TriggerServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestTriggerServiceFault(::grpc::ServerContext* context, ::bosdyn::api::TriggerServiceFaultRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::TriggerServiceFaultResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_ClearServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_ClearServiceFault() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_ClearServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ClearServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ClearServiceFaultRequest* /*request*/, ::bosdyn::api::ClearServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestClearServiceFault(::grpc::ServerContext* context, ::bosdyn::api::ClearServiceFaultRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::ClearServiceFaultResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_TriggerServiceFault<WithAsyncMethod_ClearServiceFault<Service > > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_TriggerServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_TriggerServiceFault() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::TriggerServiceFaultRequest, ::bosdyn::api::TriggerServiceFaultResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::TriggerServiceFaultRequest* request, ::bosdyn::api::TriggerServiceFaultResponse* response) { return this->TriggerServiceFault(context, request, response); }));}
    void SetMessageAllocatorFor_TriggerServiceFault(
        ::grpc::MessageAllocator< ::bosdyn::api::TriggerServiceFaultRequest, ::bosdyn::api::TriggerServiceFaultResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::TriggerServiceFaultRequest, ::bosdyn::api::TriggerServiceFaultResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_TriggerServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status TriggerServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::TriggerServiceFaultRequest* /*request*/, ::bosdyn::api::TriggerServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* TriggerServiceFault(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::TriggerServiceFaultRequest* /*request*/, ::bosdyn::api::TriggerServiceFaultResponse* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithCallbackMethod_ClearServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_ClearServiceFault() {
      ::grpc::Service::MarkMethodCallback(1,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::ClearServiceFaultRequest, ::bosdyn::api::ClearServiceFaultResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::ClearServiceFaultRequest* request, ::bosdyn::api::ClearServiceFaultResponse* response) { return this->ClearServiceFault(context, request, response); }));}
    void SetMessageAllocatorFor_ClearServiceFault(
        ::grpc::MessageAllocator< ::bosdyn::api::ClearServiceFaultRequest, ::bosdyn::api::ClearServiceFaultResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(1);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::ClearServiceFaultRequest, ::bosdyn::api::ClearServiceFaultResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_ClearServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ClearServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ClearServiceFaultRequest* /*request*/, ::bosdyn::api::ClearServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* ClearServiceFault(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::ClearServiceFaultRequest* /*request*/, ::bosdyn::api::ClearServiceFaultResponse* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_TriggerServiceFault<WithCallbackMethod_ClearServiceFault<Service > > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_TriggerServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_TriggerServiceFault() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_TriggerServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status TriggerServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::TriggerServiceFaultRequest* /*request*/, ::bosdyn::api::TriggerServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_ClearServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_ClearServiceFault() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_ClearServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ClearServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ClearServiceFaultRequest* /*request*/, ::bosdyn::api::ClearServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_TriggerServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_TriggerServiceFault() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_TriggerServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status TriggerServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::TriggerServiceFaultRequest* /*request*/, ::bosdyn::api::TriggerServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestTriggerServiceFault(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawMethod_ClearServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_ClearServiceFault() {
      ::grpc::Service::MarkMethodRaw(1);
    }
    ~WithRawMethod_ClearServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ClearServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ClearServiceFaultRequest* /*request*/, ::bosdyn::api::ClearServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestClearServiceFault(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_TriggerServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_TriggerServiceFault() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->TriggerServiceFault(context, request, response); }));
    }
    ~WithRawCallbackMethod_TriggerServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status TriggerServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::TriggerServiceFaultRequest* /*request*/, ::bosdyn::api::TriggerServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* TriggerServiceFault(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_ClearServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_ClearServiceFault() {
      ::grpc::Service::MarkMethodRawCallback(1,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->ClearServiceFault(context, request, response); }));
    }
    ~WithRawCallbackMethod_ClearServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ClearServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ClearServiceFaultRequest* /*request*/, ::bosdyn::api::ClearServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* ClearServiceFault(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_TriggerServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_TriggerServiceFault() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::TriggerServiceFaultRequest, ::bosdyn::api::TriggerServiceFaultResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::TriggerServiceFaultRequest, ::bosdyn::api::TriggerServiceFaultResponse>* streamer) {
                       return this->StreamedTriggerServiceFault(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_TriggerServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status TriggerServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::TriggerServiceFaultRequest* /*request*/, ::bosdyn::api::TriggerServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedTriggerServiceFault(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::TriggerServiceFaultRequest,::bosdyn::api::TriggerServiceFaultResponse>* server_unary_streamer) = 0;
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_ClearServiceFault : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_ClearServiceFault() {
      ::grpc::Service::MarkMethodStreamed(1,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::ClearServiceFaultRequest, ::bosdyn::api::ClearServiceFaultResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::ClearServiceFaultRequest, ::bosdyn::api::ClearServiceFaultResponse>* streamer) {
                       return this->StreamedClearServiceFault(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_ClearServiceFault() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status ClearServiceFault(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ClearServiceFaultRequest* /*request*/, ::bosdyn::api::ClearServiceFaultResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedClearServiceFault(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::ClearServiceFaultRequest,::bosdyn::api::ClearServiceFaultResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_TriggerServiceFault<WithStreamedUnaryMethod_ClearServiceFault<Service > > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_TriggerServiceFault<WithStreamedUnaryMethod_ClearServiceFault<Service > > StreamedService;
};

}  // namespace api
}  // namespace bosdyn


#endif  // GRPC_bosdyn_2fapi_2ffault_5fservice_2eproto__INCLUDED
