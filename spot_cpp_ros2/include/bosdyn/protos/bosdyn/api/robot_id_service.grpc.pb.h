// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: bosdyn/api/robot_id_service.proto
// Original file comments:
// Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
//
// Downloading, reproducing, distributing or otherwise using the SDK Software
// is subject to the terms and conditions of the Boston Dynamics Software
// Development Kit License (20191101-BDSDK-SL).
//
#ifndef GRPC_bosdyn_2fapi_2frobot_5fid_5fservice_2eproto__INCLUDED
#define GRPC_bosdyn_2fapi_2frobot_5fid_5fservice_2eproto__INCLUDED

#include "bosdyn/api/robot_id_service.pb.h"

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

// RobotIdService provides mostly static identifying information about a robot.
// User authentication is not required to access RobotIdService to assist with
// early robot discovery.
class RobotIdService final {
 public:
  static constexpr char const* service_full_name() {
    return "bosdyn.api.RobotIdService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    // Get the robot id information. The ID contains basic information about a robot
    // which is made available over the network as part of robot discovery without
    // requiring user authentication.
    virtual ::grpc::Status GetRobotId(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest& request, ::bosdyn::api::RobotIdResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::RobotIdResponse>> AsyncGetRobotId(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::RobotIdResponse>>(AsyncGetRobotIdRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::RobotIdResponse>> PrepareAsyncGetRobotId(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::RobotIdResponse>>(PrepareAsyncGetRobotIdRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      // Get the robot id information. The ID contains basic information about a robot
      // which is made available over the network as part of robot discovery without
      // requiring user authentication.
      virtual void GetRobotId(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest* request, ::bosdyn::api::RobotIdResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void GetRobotId(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest* request, ::bosdyn::api::RobotIdResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::RobotIdResponse>* AsyncGetRobotIdRaw(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::RobotIdResponse>* PrepareAsyncGetRobotIdRaw(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status GetRobotId(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest& request, ::bosdyn::api::RobotIdResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::RobotIdResponse>> AsyncGetRobotId(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::RobotIdResponse>>(AsyncGetRobotIdRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::RobotIdResponse>> PrepareAsyncGetRobotId(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::RobotIdResponse>>(PrepareAsyncGetRobotIdRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void GetRobotId(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest* request, ::bosdyn::api::RobotIdResponse* response, std::function<void(::grpc::Status)>) override;
      void GetRobotId(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest* request, ::bosdyn::api::RobotIdResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
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
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::RobotIdResponse>* AsyncGetRobotIdRaw(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::RobotIdResponse>* PrepareAsyncGetRobotIdRaw(::grpc::ClientContext* context, const ::bosdyn::api::RobotIdRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_GetRobotId_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    // Get the robot id information. The ID contains basic information about a robot
    // which is made available over the network as part of robot discovery without
    // requiring user authentication.
    virtual ::grpc::Status GetRobotId(::grpc::ServerContext* context, const ::bosdyn::api::RobotIdRequest* request, ::bosdyn::api::RobotIdResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_GetRobotId : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_GetRobotId() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_GetRobotId() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetRobotId(::grpc::ServerContext* /*context*/, const ::bosdyn::api::RobotIdRequest* /*request*/, ::bosdyn::api::RobotIdResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetRobotId(::grpc::ServerContext* context, ::bosdyn::api::RobotIdRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::RobotIdResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_GetRobotId<Service > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_GetRobotId : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_GetRobotId() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::RobotIdRequest, ::bosdyn::api::RobotIdResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::RobotIdRequest* request, ::bosdyn::api::RobotIdResponse* response) { return this->GetRobotId(context, request, response); }));}
    void SetMessageAllocatorFor_GetRobotId(
        ::grpc::MessageAllocator< ::bosdyn::api::RobotIdRequest, ::bosdyn::api::RobotIdResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::RobotIdRequest, ::bosdyn::api::RobotIdResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_GetRobotId() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetRobotId(::grpc::ServerContext* /*context*/, const ::bosdyn::api::RobotIdRequest* /*request*/, ::bosdyn::api::RobotIdResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* GetRobotId(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::RobotIdRequest* /*request*/, ::bosdyn::api::RobotIdResponse* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_GetRobotId<Service > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_GetRobotId : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_GetRobotId() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_GetRobotId() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetRobotId(::grpc::ServerContext* /*context*/, const ::bosdyn::api::RobotIdRequest* /*request*/, ::bosdyn::api::RobotIdResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_GetRobotId : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_GetRobotId() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_GetRobotId() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetRobotId(::grpc::ServerContext* /*context*/, const ::bosdyn::api::RobotIdRequest* /*request*/, ::bosdyn::api::RobotIdResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetRobotId(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_GetRobotId : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_GetRobotId() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->GetRobotId(context, request, response); }));
    }
    ~WithRawCallbackMethod_GetRobotId() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetRobotId(::grpc::ServerContext* /*context*/, const ::bosdyn::api::RobotIdRequest* /*request*/, ::bosdyn::api::RobotIdResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* GetRobotId(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_GetRobotId : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_GetRobotId() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::RobotIdRequest, ::bosdyn::api::RobotIdResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::RobotIdRequest, ::bosdyn::api::RobotIdResponse>* streamer) {
                       return this->StreamedGetRobotId(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_GetRobotId() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status GetRobotId(::grpc::ServerContext* /*context*/, const ::bosdyn::api::RobotIdRequest* /*request*/, ::bosdyn::api::RobotIdResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedGetRobotId(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::RobotIdRequest,::bosdyn::api::RobotIdResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_GetRobotId<Service > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_GetRobotId<Service > StreamedService;
};

}  // namespace api
}  // namespace bosdyn


#endif  // GRPC_bosdyn_2fapi_2frobot_5fid_5fservice_2eproto__INCLUDED
