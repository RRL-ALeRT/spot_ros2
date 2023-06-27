// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: bosdyn/api/ir_enable_disable_service.proto
// Original file comments:
// Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
//
// Downloading, reproducing, distributing or otherwise using the SDK Software
// is subject to the terms and conditions of the Boston Dynamics Software
// Development Kit License (20191101-BDSDK-SL).
//
#ifndef GRPC_bosdyn_2fapi_2fir_5fenable_5fdisable_5fservice_2eproto__INCLUDED
#define GRPC_bosdyn_2fapi_2fir_5fenable_5fdisable_5fservice_2eproto__INCLUDED

#include "bosdyn/api/ir_enable_disable_service.pb.h"

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

class IREnableDisableService final {
 public:
  static constexpr char const* service_full_name() {
    return "bosdyn.api.IREnableDisableService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    virtual ::grpc::Status IREnableDisable(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest& request, ::bosdyn::api::IREnableDisableResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::IREnableDisableResponse>> AsyncIREnableDisable(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::IREnableDisableResponse>>(AsyncIREnableDisableRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::IREnableDisableResponse>> PrepareAsyncIREnableDisable(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::IREnableDisableResponse>>(PrepareAsyncIREnableDisableRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      virtual void IREnableDisable(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest* request, ::bosdyn::api::IREnableDisableResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void IREnableDisable(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest* request, ::bosdyn::api::IREnableDisableResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::IREnableDisableResponse>* AsyncIREnableDisableRaw(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::IREnableDisableResponse>* PrepareAsyncIREnableDisableRaw(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status IREnableDisable(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest& request, ::bosdyn::api::IREnableDisableResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::IREnableDisableResponse>> AsyncIREnableDisable(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::IREnableDisableResponse>>(AsyncIREnableDisableRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::IREnableDisableResponse>> PrepareAsyncIREnableDisable(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::IREnableDisableResponse>>(PrepareAsyncIREnableDisableRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void IREnableDisable(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest* request, ::bosdyn::api::IREnableDisableResponse* response, std::function<void(::grpc::Status)>) override;
      void IREnableDisable(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest* request, ::bosdyn::api::IREnableDisableResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
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
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::IREnableDisableResponse>* AsyncIREnableDisableRaw(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::IREnableDisableResponse>* PrepareAsyncIREnableDisableRaw(::grpc::ClientContext* context, const ::bosdyn::api::IREnableDisableRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_IREnableDisable_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    virtual ::grpc::Status IREnableDisable(::grpc::ServerContext* context, const ::bosdyn::api::IREnableDisableRequest* request, ::bosdyn::api::IREnableDisableResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_IREnableDisable : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_IREnableDisable() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_IREnableDisable() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status IREnableDisable(::grpc::ServerContext* /*context*/, const ::bosdyn::api::IREnableDisableRequest* /*request*/, ::bosdyn::api::IREnableDisableResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestIREnableDisable(::grpc::ServerContext* context, ::bosdyn::api::IREnableDisableRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::IREnableDisableResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_IREnableDisable<Service > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_IREnableDisable : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_IREnableDisable() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::IREnableDisableRequest, ::bosdyn::api::IREnableDisableResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::IREnableDisableRequest* request, ::bosdyn::api::IREnableDisableResponse* response) { return this->IREnableDisable(context, request, response); }));}
    void SetMessageAllocatorFor_IREnableDisable(
        ::grpc::MessageAllocator< ::bosdyn::api::IREnableDisableRequest, ::bosdyn::api::IREnableDisableResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::IREnableDisableRequest, ::bosdyn::api::IREnableDisableResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_IREnableDisable() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status IREnableDisable(::grpc::ServerContext* /*context*/, const ::bosdyn::api::IREnableDisableRequest* /*request*/, ::bosdyn::api::IREnableDisableResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* IREnableDisable(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::IREnableDisableRequest* /*request*/, ::bosdyn::api::IREnableDisableResponse* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_IREnableDisable<Service > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_IREnableDisable : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_IREnableDisable() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_IREnableDisable() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status IREnableDisable(::grpc::ServerContext* /*context*/, const ::bosdyn::api::IREnableDisableRequest* /*request*/, ::bosdyn::api::IREnableDisableResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_IREnableDisable : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_IREnableDisable() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_IREnableDisable() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status IREnableDisable(::grpc::ServerContext* /*context*/, const ::bosdyn::api::IREnableDisableRequest* /*request*/, ::bosdyn::api::IREnableDisableResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestIREnableDisable(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_IREnableDisable : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_IREnableDisable() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->IREnableDisable(context, request, response); }));
    }
    ~WithRawCallbackMethod_IREnableDisable() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status IREnableDisable(::grpc::ServerContext* /*context*/, const ::bosdyn::api::IREnableDisableRequest* /*request*/, ::bosdyn::api::IREnableDisableResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* IREnableDisable(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_IREnableDisable : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_IREnableDisable() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::IREnableDisableRequest, ::bosdyn::api::IREnableDisableResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::IREnableDisableRequest, ::bosdyn::api::IREnableDisableResponse>* streamer) {
                       return this->StreamedIREnableDisable(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_IREnableDisable() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status IREnableDisable(::grpc::ServerContext* /*context*/, const ::bosdyn::api::IREnableDisableRequest* /*request*/, ::bosdyn::api::IREnableDisableResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedIREnableDisable(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::IREnableDisableRequest,::bosdyn::api::IREnableDisableResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_IREnableDisable<Service > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_IREnableDisable<Service > StreamedService;
};

}  // namespace api
}  // namespace bosdyn


#endif  // GRPC_bosdyn_2fapi_2fir_5fenable_5fdisable_5fservice_2eproto__INCLUDED
