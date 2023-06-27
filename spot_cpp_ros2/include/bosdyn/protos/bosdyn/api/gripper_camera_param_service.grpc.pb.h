// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: bosdyn/api/gripper_camera_param_service.proto
// Original file comments:
// Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
//
// Downloading, reproducing, distributing or otherwise using the SDK Software
// is subject to the terms and conditions of the Boston Dynamics Software
// Development Kit License (20191101-BDSDK-SL).
//
#ifndef GRPC_bosdyn_2fapi_2fgripper_5fcamera_5fparam_5fservice_2eproto__INCLUDED
#define GRPC_bosdyn_2fapi_2fgripper_5fcamera_5fparam_5fservice_2eproto__INCLUDED

#include "bosdyn/api/gripper_camera_param_service.pb.h"

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

class GripperCameraParamService final {
 public:
  static constexpr char const* service_full_name() {
    return "bosdyn.api.GripperCameraParamService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    virtual ::grpc::Status SetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest& request, ::bosdyn::api::GripperCameraParamResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraParamResponse>> AsyncSetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraParamResponse>>(AsyncSetParamsRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraParamResponse>> PrepareAsyncSetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraParamResponse>>(PrepareAsyncSetParamsRaw(context, request, cq));
    }
    virtual ::grpc::Status GetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest& request, ::bosdyn::api::GripperCameraGetParamResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraGetParamResponse>> AsyncGetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraGetParamResponse>>(AsyncGetParamsRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraGetParamResponse>> PrepareAsyncGetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraGetParamResponse>>(PrepareAsyncGetParamsRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      virtual void SetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest* request, ::bosdyn::api::GripperCameraParamResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void SetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest* request, ::bosdyn::api::GripperCameraParamResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
      virtual void GetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest* request, ::bosdyn::api::GripperCameraGetParamResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void GetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest* request, ::bosdyn::api::GripperCameraGetParamResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraParamResponse>* AsyncSetParamsRaw(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraParamResponse>* PrepareAsyncSetParamsRaw(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraGetParamResponse>* AsyncGetParamsRaw(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GripperCameraGetParamResponse>* PrepareAsyncGetParamsRaw(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status SetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest& request, ::bosdyn::api::GripperCameraParamResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraParamResponse>> AsyncSetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraParamResponse>>(AsyncSetParamsRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraParamResponse>> PrepareAsyncSetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraParamResponse>>(PrepareAsyncSetParamsRaw(context, request, cq));
    }
    ::grpc::Status GetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest& request, ::bosdyn::api::GripperCameraGetParamResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraGetParamResponse>> AsyncGetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraGetParamResponse>>(AsyncGetParamsRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraGetParamResponse>> PrepareAsyncGetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraGetParamResponse>>(PrepareAsyncGetParamsRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void SetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest* request, ::bosdyn::api::GripperCameraParamResponse* response, std::function<void(::grpc::Status)>) override;
      void SetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest* request, ::bosdyn::api::GripperCameraParamResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
      void GetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest* request, ::bosdyn::api::GripperCameraGetParamResponse* response, std::function<void(::grpc::Status)>) override;
      void GetParams(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest* request, ::bosdyn::api::GripperCameraGetParamResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
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
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraParamResponse>* AsyncSetParamsRaw(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraParamResponse>* PrepareAsyncSetParamsRaw(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraParamRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraGetParamResponse>* AsyncGetParamsRaw(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GripperCameraGetParamResponse>* PrepareAsyncGetParamsRaw(::grpc::ClientContext* context, const ::bosdyn::api::GripperCameraGetParamRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_SetParams_;
    const ::grpc::internal::RpcMethod rpcmethod_GetParams_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    virtual ::grpc::Status SetParams(::grpc::ServerContext* context, const ::bosdyn::api::GripperCameraParamRequest* request, ::bosdyn::api::GripperCameraParamResponse* response);
    virtual ::grpc::Status GetParams(::grpc::ServerContext* context, const ::bosdyn::api::GripperCameraGetParamRequest* request, ::bosdyn::api::GripperCameraGetParamResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_SetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_SetParams() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_SetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraParamRequest* /*request*/, ::bosdyn::api::GripperCameraParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSetParams(::grpc::ServerContext* context, ::bosdyn::api::GripperCameraParamRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::GripperCameraParamResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_GetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_GetParams() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_GetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraGetParamRequest* /*request*/, ::bosdyn::api::GripperCameraGetParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetParams(::grpc::ServerContext* context, ::bosdyn::api::GripperCameraGetParamRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::GripperCameraGetParamResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_SetParams<WithAsyncMethod_GetParams<Service > > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_SetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_SetParams() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::GripperCameraParamRequest, ::bosdyn::api::GripperCameraParamResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::GripperCameraParamRequest* request, ::bosdyn::api::GripperCameraParamResponse* response) { return this->SetParams(context, request, response); }));}
    void SetMessageAllocatorFor_SetParams(
        ::grpc::MessageAllocator< ::bosdyn::api::GripperCameraParamRequest, ::bosdyn::api::GripperCameraParamResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::GripperCameraParamRequest, ::bosdyn::api::GripperCameraParamResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_SetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraParamRequest* /*request*/, ::bosdyn::api::GripperCameraParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* SetParams(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::GripperCameraParamRequest* /*request*/, ::bosdyn::api::GripperCameraParamResponse* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithCallbackMethod_GetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_GetParams() {
      ::grpc::Service::MarkMethodCallback(1,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::GripperCameraGetParamRequest, ::bosdyn::api::GripperCameraGetParamResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::GripperCameraGetParamRequest* request, ::bosdyn::api::GripperCameraGetParamResponse* response) { return this->GetParams(context, request, response); }));}
    void SetMessageAllocatorFor_GetParams(
        ::grpc::MessageAllocator< ::bosdyn::api::GripperCameraGetParamRequest, ::bosdyn::api::GripperCameraGetParamResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(1);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::GripperCameraGetParamRequest, ::bosdyn::api::GripperCameraGetParamResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_GetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraGetParamRequest* /*request*/, ::bosdyn::api::GripperCameraGetParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* GetParams(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::GripperCameraGetParamRequest* /*request*/, ::bosdyn::api::GripperCameraGetParamResponse* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_SetParams<WithCallbackMethod_GetParams<Service > > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_SetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_SetParams() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_SetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraParamRequest* /*request*/, ::bosdyn::api::GripperCameraParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_GetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_GetParams() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_GetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraGetParamRequest* /*request*/, ::bosdyn::api::GripperCameraGetParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_SetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_SetParams() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_SetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraParamRequest* /*request*/, ::bosdyn::api::GripperCameraParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestSetParams(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawMethod_GetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_GetParams() {
      ::grpc::Service::MarkMethodRaw(1);
    }
    ~WithRawMethod_GetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraGetParamRequest* /*request*/, ::bosdyn::api::GripperCameraGetParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetParams(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_SetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_SetParams() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->SetParams(context, request, response); }));
    }
    ~WithRawCallbackMethod_SetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status SetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraParamRequest* /*request*/, ::bosdyn::api::GripperCameraParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* SetParams(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_GetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_GetParams() {
      ::grpc::Service::MarkMethodRawCallback(1,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->GetParams(context, request, response); }));
    }
    ~WithRawCallbackMethod_GetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraGetParamRequest* /*request*/, ::bosdyn::api::GripperCameraGetParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* GetParams(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_SetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_SetParams() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::GripperCameraParamRequest, ::bosdyn::api::GripperCameraParamResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::GripperCameraParamRequest, ::bosdyn::api::GripperCameraParamResponse>* streamer) {
                       return this->StreamedSetParams(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_SetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status SetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraParamRequest* /*request*/, ::bosdyn::api::GripperCameraParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedSetParams(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::GripperCameraParamRequest,::bosdyn::api::GripperCameraParamResponse>* server_unary_streamer) = 0;
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_GetParams : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_GetParams() {
      ::grpc::Service::MarkMethodStreamed(1,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::GripperCameraGetParamRequest, ::bosdyn::api::GripperCameraGetParamResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::GripperCameraGetParamRequest, ::bosdyn::api::GripperCameraGetParamResponse>* streamer) {
                       return this->StreamedGetParams(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_GetParams() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status GetParams(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GripperCameraGetParamRequest* /*request*/, ::bosdyn::api::GripperCameraGetParamResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedGetParams(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::GripperCameraGetParamRequest,::bosdyn::api::GripperCameraGetParamResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_SetParams<WithStreamedUnaryMethod_GetParams<Service > > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_SetParams<WithStreamedUnaryMethod_GetParams<Service > > StreamedService;
};

}  // namespace api
}  // namespace bosdyn


#endif  // GRPC_bosdyn_2fapi_2fgripper_5fcamera_5fparam_5fservice_2eproto__INCLUDED
