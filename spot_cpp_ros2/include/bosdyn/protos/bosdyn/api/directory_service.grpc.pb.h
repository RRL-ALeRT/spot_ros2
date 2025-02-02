// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: bosdyn/api/directory_service.proto
// Original file comments:
// Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
//
// Downloading, reproducing, distributing or otherwise using the SDK Software
// is subject to the terms and conditions of the Boston Dynamics Software
// Development Kit License (20191101-BDSDK-SL).
//
#ifndef GRPC_bosdyn_2fapi_2fdirectory_5fservice_2eproto__INCLUDED
#define GRPC_bosdyn_2fapi_2fdirectory_5fservice_2eproto__INCLUDED

#include "bosdyn/api/directory_service.pb.h"

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

// DirectoryService lets clients discover which API services are available on a robot.
class DirectoryService final {
 public:
  static constexpr char const* service_full_name() {
    return "bosdyn.api.DirectoryService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    // Get information about a specific service.
    virtual ::grpc::Status GetServiceEntry(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest& request, ::bosdyn::api::GetServiceEntryResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetServiceEntryResponse>> AsyncGetServiceEntry(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetServiceEntryResponse>>(AsyncGetServiceEntryRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetServiceEntryResponse>> PrepareAsyncGetServiceEntry(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetServiceEntryResponse>>(PrepareAsyncGetServiceEntryRaw(context, request, cq));
    }
    // List all known services at time of call.
    virtual ::grpc::Status ListServiceEntries(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest& request, ::bosdyn::api::ListServiceEntriesResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListServiceEntriesResponse>> AsyncListServiceEntries(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListServiceEntriesResponse>>(AsyncListServiceEntriesRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListServiceEntriesResponse>> PrepareAsyncListServiceEntries(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListServiceEntriesResponse>>(PrepareAsyncListServiceEntriesRaw(context, request, cq));
    }
    class async_interface {
     public:
      virtual ~async_interface() {}
      // Get information about a specific service.
      virtual void GetServiceEntry(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest* request, ::bosdyn::api::GetServiceEntryResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void GetServiceEntry(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest* request, ::bosdyn::api::GetServiceEntryResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
      // List all known services at time of call.
      virtual void ListServiceEntries(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest* request, ::bosdyn::api::ListServiceEntriesResponse* response, std::function<void(::grpc::Status)>) = 0;
      virtual void ListServiceEntries(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest* request, ::bosdyn::api::ListServiceEntriesResponse* response, ::grpc::ClientUnaryReactor* reactor) = 0;
    };
    typedef class async_interface experimental_async_interface;
    virtual class async_interface* async() { return nullptr; }
    class async_interface* experimental_async() { return async(); }
   private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetServiceEntryResponse>* AsyncGetServiceEntryRaw(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::GetServiceEntryResponse>* PrepareAsyncGetServiceEntryRaw(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListServiceEntriesResponse>* AsyncListServiceEntriesRaw(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::bosdyn::api::ListServiceEntriesResponse>* PrepareAsyncListServiceEntriesRaw(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());
    ::grpc::Status GetServiceEntry(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest& request, ::bosdyn::api::GetServiceEntryResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetServiceEntryResponse>> AsyncGetServiceEntry(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetServiceEntryResponse>>(AsyncGetServiceEntryRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetServiceEntryResponse>> PrepareAsyncGetServiceEntry(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetServiceEntryResponse>>(PrepareAsyncGetServiceEntryRaw(context, request, cq));
    }
    ::grpc::Status ListServiceEntries(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest& request, ::bosdyn::api::ListServiceEntriesResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListServiceEntriesResponse>> AsyncListServiceEntries(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListServiceEntriesResponse>>(AsyncListServiceEntriesRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListServiceEntriesResponse>> PrepareAsyncListServiceEntries(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListServiceEntriesResponse>>(PrepareAsyncListServiceEntriesRaw(context, request, cq));
    }
    class async final :
      public StubInterface::async_interface {
     public:
      void GetServiceEntry(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest* request, ::bosdyn::api::GetServiceEntryResponse* response, std::function<void(::grpc::Status)>) override;
      void GetServiceEntry(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest* request, ::bosdyn::api::GetServiceEntryResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
      void ListServiceEntries(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest* request, ::bosdyn::api::ListServiceEntriesResponse* response, std::function<void(::grpc::Status)>) override;
      void ListServiceEntries(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest* request, ::bosdyn::api::ListServiceEntriesResponse* response, ::grpc::ClientUnaryReactor* reactor) override;
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
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetServiceEntryResponse>* AsyncGetServiceEntryRaw(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::GetServiceEntryResponse>* PrepareAsyncGetServiceEntryRaw(::grpc::ClientContext* context, const ::bosdyn::api::GetServiceEntryRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListServiceEntriesResponse>* AsyncListServiceEntriesRaw(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListServiceEntriesResponse>* PrepareAsyncListServiceEntriesRaw(::grpc::ClientContext* context, const ::bosdyn::api::ListServiceEntriesRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_GetServiceEntry_;
    const ::grpc::internal::RpcMethod rpcmethod_ListServiceEntries_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    // Get information about a specific service.
    virtual ::grpc::Status GetServiceEntry(::grpc::ServerContext* context, const ::bosdyn::api::GetServiceEntryRequest* request, ::bosdyn::api::GetServiceEntryResponse* response);
    // List all known services at time of call.
    virtual ::grpc::Status ListServiceEntries(::grpc::ServerContext* context, const ::bosdyn::api::ListServiceEntriesRequest* request, ::bosdyn::api::ListServiceEntriesResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_GetServiceEntry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_GetServiceEntry() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_GetServiceEntry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetServiceEntry(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetServiceEntryRequest* /*request*/, ::bosdyn::api::GetServiceEntryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetServiceEntry(::grpc::ServerContext* context, ::bosdyn::api::GetServiceEntryRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::GetServiceEntryResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithAsyncMethod_ListServiceEntries : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithAsyncMethod_ListServiceEntries() {
      ::grpc::Service::MarkMethodAsync(1);
    }
    ~WithAsyncMethod_ListServiceEntries() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListServiceEntries(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListServiceEntriesRequest* /*request*/, ::bosdyn::api::ListServiceEntriesResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestListServiceEntries(::grpc::ServerContext* context, ::bosdyn::api::ListServiceEntriesRequest* request, ::grpc::ServerAsyncResponseWriter< ::bosdyn::api::ListServiceEntriesResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_GetServiceEntry<WithAsyncMethod_ListServiceEntries<Service > > AsyncService;
  template <class BaseClass>
  class WithCallbackMethod_GetServiceEntry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_GetServiceEntry() {
      ::grpc::Service::MarkMethodCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::GetServiceEntryRequest, ::bosdyn::api::GetServiceEntryResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::GetServiceEntryRequest* request, ::bosdyn::api::GetServiceEntryResponse* response) { return this->GetServiceEntry(context, request, response); }));}
    void SetMessageAllocatorFor_GetServiceEntry(
        ::grpc::MessageAllocator< ::bosdyn::api::GetServiceEntryRequest, ::bosdyn::api::GetServiceEntryResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(0);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::GetServiceEntryRequest, ::bosdyn::api::GetServiceEntryResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_GetServiceEntry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetServiceEntry(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetServiceEntryRequest* /*request*/, ::bosdyn::api::GetServiceEntryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* GetServiceEntry(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::GetServiceEntryRequest* /*request*/, ::bosdyn::api::GetServiceEntryResponse* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithCallbackMethod_ListServiceEntries : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithCallbackMethod_ListServiceEntries() {
      ::grpc::Service::MarkMethodCallback(1,
          new ::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::ListServiceEntriesRequest, ::bosdyn::api::ListServiceEntriesResponse>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::bosdyn::api::ListServiceEntriesRequest* request, ::bosdyn::api::ListServiceEntriesResponse* response) { return this->ListServiceEntries(context, request, response); }));}
    void SetMessageAllocatorFor_ListServiceEntries(
        ::grpc::MessageAllocator< ::bosdyn::api::ListServiceEntriesRequest, ::bosdyn::api::ListServiceEntriesResponse>* allocator) {
      ::grpc::internal::MethodHandler* const handler = ::grpc::Service::GetHandler(1);
      static_cast<::grpc::internal::CallbackUnaryHandler< ::bosdyn::api::ListServiceEntriesRequest, ::bosdyn::api::ListServiceEntriesResponse>*>(handler)
              ->SetMessageAllocator(allocator);
    }
    ~WithCallbackMethod_ListServiceEntries() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListServiceEntries(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListServiceEntriesRequest* /*request*/, ::bosdyn::api::ListServiceEntriesResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* ListServiceEntries(
      ::grpc::CallbackServerContext* /*context*/, const ::bosdyn::api::ListServiceEntriesRequest* /*request*/, ::bosdyn::api::ListServiceEntriesResponse* /*response*/)  { return nullptr; }
  };
  typedef WithCallbackMethod_GetServiceEntry<WithCallbackMethod_ListServiceEntries<Service > > CallbackService;
  typedef CallbackService ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_GetServiceEntry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_GetServiceEntry() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_GetServiceEntry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetServiceEntry(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetServiceEntryRequest* /*request*/, ::bosdyn::api::GetServiceEntryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithGenericMethod_ListServiceEntries : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithGenericMethod_ListServiceEntries() {
      ::grpc::Service::MarkMethodGeneric(1);
    }
    ~WithGenericMethod_ListServiceEntries() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListServiceEntries(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListServiceEntriesRequest* /*request*/, ::bosdyn::api::ListServiceEntriesResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_GetServiceEntry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_GetServiceEntry() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_GetServiceEntry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetServiceEntry(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetServiceEntryRequest* /*request*/, ::bosdyn::api::GetServiceEntryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestGetServiceEntry(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawMethod_ListServiceEntries : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawMethod_ListServiceEntries() {
      ::grpc::Service::MarkMethodRaw(1);
    }
    ~WithRawMethod_ListServiceEntries() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListServiceEntries(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListServiceEntriesRequest* /*request*/, ::bosdyn::api::ListServiceEntriesResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestListServiceEntries(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(1, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_GetServiceEntry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_GetServiceEntry() {
      ::grpc::Service::MarkMethodRawCallback(0,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->GetServiceEntry(context, request, response); }));
    }
    ~WithRawCallbackMethod_GetServiceEntry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status GetServiceEntry(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetServiceEntryRequest* /*request*/, ::bosdyn::api::GetServiceEntryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* GetServiceEntry(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithRawCallbackMethod_ListServiceEntries : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithRawCallbackMethod_ListServiceEntries() {
      ::grpc::Service::MarkMethodRawCallback(1,
          new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
            [this](
                   ::grpc::CallbackServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response) { return this->ListServiceEntries(context, request, response); }));
    }
    ~WithRawCallbackMethod_ListServiceEntries() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ListServiceEntries(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListServiceEntriesRequest* /*request*/, ::bosdyn::api::ListServiceEntriesResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual ::grpc::ServerUnaryReactor* ListServiceEntries(
      ::grpc::CallbackServerContext* /*context*/, const ::grpc::ByteBuffer* /*request*/, ::grpc::ByteBuffer* /*response*/)  { return nullptr; }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_GetServiceEntry : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_GetServiceEntry() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::GetServiceEntryRequest, ::bosdyn::api::GetServiceEntryResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::GetServiceEntryRequest, ::bosdyn::api::GetServiceEntryResponse>* streamer) {
                       return this->StreamedGetServiceEntry(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_GetServiceEntry() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status GetServiceEntry(::grpc::ServerContext* /*context*/, const ::bosdyn::api::GetServiceEntryRequest* /*request*/, ::bosdyn::api::GetServiceEntryResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedGetServiceEntry(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::GetServiceEntryRequest,::bosdyn::api::GetServiceEntryResponse>* server_unary_streamer) = 0;
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_ListServiceEntries : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service* /*service*/) {}
   public:
    WithStreamedUnaryMethod_ListServiceEntries() {
      ::grpc::Service::MarkMethodStreamed(1,
        new ::grpc::internal::StreamedUnaryHandler<
          ::bosdyn::api::ListServiceEntriesRequest, ::bosdyn::api::ListServiceEntriesResponse>(
            [this](::grpc::ServerContext* context,
                   ::grpc::ServerUnaryStreamer<
                     ::bosdyn::api::ListServiceEntriesRequest, ::bosdyn::api::ListServiceEntriesResponse>* streamer) {
                       return this->StreamedListServiceEntries(context,
                         streamer);
                  }));
    }
    ~WithStreamedUnaryMethod_ListServiceEntries() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status ListServiceEntries(::grpc::ServerContext* /*context*/, const ::bosdyn::api::ListServiceEntriesRequest* /*request*/, ::bosdyn::api::ListServiceEntriesResponse* /*response*/) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedListServiceEntries(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::bosdyn::api::ListServiceEntriesRequest,::bosdyn::api::ListServiceEntriesResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_GetServiceEntry<WithStreamedUnaryMethod_ListServiceEntries<Service > > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_GetServiceEntry<WithStreamedUnaryMethod_ListServiceEntries<Service > > StreamedService;
};

}  // namespace api
}  // namespace bosdyn


#endif  // GRPC_bosdyn_2fapi_2fdirectory_5fservice_2eproto__INCLUDED
