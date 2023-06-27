// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: bosdyn/api/auth_service.proto
// Original file comments:
// Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
//
// Downloading, reproducing, distributing or otherwise using the SDK Software
// is subject to the terms and conditions of the Boston Dynamics Software
// Development Kit License (20191101-BDSDK-SL).
//
#ifndef GRPC_bosdyn_2fapi_2fauth_5fservice_2eproto__INCLUDED
#define GRPC_bosdyn_2fapi_2fauth_5fservice_2eproto__INCLUDED

#include "bosdyn/api/auth_service.pb.h"

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

// The AuthService provides clients the ability to convert a user/password pair into a token. The
// token can then be added to the http2 headers for future requests in order to establish the
// identity of the requester.
class AuthService final {
 public:
  static constexpr char const* service_full_name() {
    return "bosdyn.api.AuthService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    // Request to get the auth token for the robot.
    virtual ::grpc::Status GetAuthToken(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest& request, ::bosdyn::api::GetAuthTokenResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetAuthTokenResponse>> AsyncGetAuthToken(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetAuthTokenResponse>>(AsyncGetAuthTokenRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetAuthTokenResponse>> PrepareAsyncGetAuthToken(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetAuthTokenResponse>>(PrepareAsyncGetAuthTokenRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      // Request to get the auth token for the robot.
      virtual void GetAuthToken(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest* request, ::bosdyn::api::GetAuthTokenResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void GetAuthToken(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest* request, ::bosdyn::api::GetAuthTokenResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetAuthTokenResponse>* AsyncGetAuthTokenRaw(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetAuthTokenResponse>* PrepareAsyncGetAuthTokenRaw(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status GetAuthToken(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest& request, ::bosdyn::api::GetAuthTokenResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetAuthTokenResponse>> AsyncGetAuthToken(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetAuthTokenResponse>>(AsyncGetAuthTokenRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetAuthTokenResponse>> PrepareAsyncGetAuthToken(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetAuthTokenResponse>>(PrepareAsyncGetAuthTokenRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void GetAuthToken(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest* request, ::bosdyn::api::GetAuthTokenResponse* response, std::function<void(::grpc::Status)>) override;
      void GetAuthToken(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest* request, ::bosdyn::api::GetAuthTokenResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
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
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetAuthTokenResponse>* AsyncGetAuthTokenRaw(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetAuthTokenResponse>* PrepareAsyncGetAuthTokenRaw(::grpc::ClientContext* context, const ::bosdyn::api::GetAuthTokenRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_GetAuthToken_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    // Request to get the auth token for the robot.
    virtual ::grpc::Status GetAuthToken(::grpc::ServerContext* context, const ::bosdyn::api::GetAuthTokenRequest* request, ::bosdyn::api::GetAuthTokenResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_GetAuthToken : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_GetAuthToken() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_GetAuthToken() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetAuthToken(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetAuthTokenRequest* /*request*/, ::bosdyn::api::GetAuthTokenResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetAuthToken(::grpc::ServerContext* context, ::bosdyn::api::GetAuthTokenRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::GetAuthTokenResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_GetAuthToken<Service > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_GetAuthToken : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_GetAuthToken() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::GetAuthTokenRequest, ::bosdyn::api::GetAuthTokenResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::GetAuthTokenRequest* request, ::bosdyn::api::GetAuthTokenResponse* response) { return this->GetAuthToken(context, request, response); }));}
    void SetMessageAllocatorFor_GetAuthToken(
        ::grpc::MessageAllocator< ::bosdyn::api::GetAuthTokenRequest, ::bosdyn::api::GetAuthTokenResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::GetAuthTokenRequest, ::bosdyn::api::GetAuthTokenResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_GetAuthToken() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetAuthToken(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetAuthTokenRequest* /*request*/, ::bosdyn::api::GetAuthTokenResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* GetAuthToken(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::GetAuthTokenRequest* /*request*/, ::bosdyn::api::GetAuthTokenResponse* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_GetAuthToken<Service > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_GetAuthToken : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_GetAuthToken() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_GetAuthToken() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetAuthToken(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetAuthTokenRequest* /*request*/, ::bosdyn::api::GetAuthTokenResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_GetAuthToken : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_GetAuthToken() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_GetAuthToken() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetAuthToken(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetAuthTokenRequest* /*request*/, ::bosdyn::api::GetAuthTokenResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetAuthToken(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_GetAuthToken : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_GetAuthToken() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->GetAuthToken(context, request, response); }));
    }
    ~WithRawCallbackMethod_GetAuthToken() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetAuthToken(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetAuthTokenRequest* /*request*/, ::bosdyn::api::GetAuthTokenResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* GetAuthToken(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_GetAuthToken : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_GetAuthToken() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::GetAuthTokenRequest, ::bosdyn::api::GetAuthTokenResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::GetAuthTokenRequest, ::bosdyn::api::GetAuthTokenResponse>* streamer) {
                       return this->StreamedGetAuthToken(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_GetAuthToken() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status GetAuthToken(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetAuthTokenRequest* /*request*/, ::bosdyn::api::GetAuthTokenResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedGetAuthToken(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::GetAuthTokenRequest,::bosdyn::api::GetAuthTokenResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_GetAuthToken<Service > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_GetAuthToken<Service > StreamedService;
};

}  // namespace api
}  // namespace bosdyn


#endif  // GRPC_bosdyn_2fapi_2fauth_5fservice_2eproto__INCLUDED
