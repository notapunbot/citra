// Copyright 2014 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "common/logging/log.h"

#include "core/hle/hle.h"
#include "core/hle/service/srv.h"
#include "core/hle/kernel/event.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
// Namespace SRV

namespace SRV {

static Kernel::SharedPtr<Kernel::Event> event_handle;

static void Initialize(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    cmd_buff[1] = 0; // No error
}

static void GetProcSemaphore(Service::Interface* self) {
    u32* cmd_buff = Kernel::GetCommandBuffer();

    // TODO(bunnei): Change to a semaphore once these have been implemented
    event_handle = Kernel::Event::Create(RESETTYPE_ONESHOT, "SRV:Event");
    event_handle->Clear();

    cmd_buff[1] = 0; // No error
    cmd_buff[3] = Kernel::g_handle_table.Create(event_handle).MoveFrom();
}

static void GetServiceHandle(Service::Interface* self) {
    ResultCode res = RESULT_SUCCESS;
    u32* cmd_buff = Kernel::GetCommandBuffer();

    std::string port_name = std::string((const char*)&cmd_buff[1], 0, Service::kMaxPortSize);
    auto it = Service::g_srv_services.find(port_name);

    if (it != Service::g_srv_services.end()) {
        auto client_session = it->second->CreateSession();
        cmd_buff[3] = Kernel::g_handle_table.Create(client_session).MoveFrom();
        LOG_TRACE(Service_SRV, "called port=%s, handle=0x%08X", port_name.c_str(), cmd_buff[3]);
    } else {
        LOG_ERROR(Service_SRV, "(UNIMPLEMENTED) called port=%s", port_name.c_str());
        res = UnimplementedFunction(ErrorModule::SRV);
    }
    cmd_buff[1] = res.raw;
}

static void RegisterService(Service::Interface* self) {
    ResultCode res = RESULT_SUCCESS;
    u32* cmd_buff = Kernel::GetCommandBuffer();

    u32 max_sessions = cmd_buff[4];
    std::string port_name = std::string((const char*)&cmd_buff[1], 0, std::min<u32>(cmd_buff[3], Service::kMaxPortSize));
    auto it = Service::g_srv_services.find(port_name);

    if (it != Service::g_srv_services.end()) {
        LOG_ERROR(Service_SRV, "(UNIMPLEMENTED) called port=%s already exists", port_name.c_str());
        res = UnimplementedFunction(ErrorModule::SRV);
    } else {
        LOG_TRACE(Service_SRV, "called port=%s, handle=0x%08X", port_name.c_str(), cmd_buff[3]);
        auto server_port = Kernel::ServerPort::Create(port_name, max_sessions);
        Service::g_srv_services.emplace(port_name, server_port);
        cmd_buff[3] = Kernel::g_handle_table.Create(server_port).MoveFrom();
    }

    cmd_buff[1] = res.raw;
}

const Interface::FunctionInfo FunctionTable[] = {
    {0x00010002, Initialize,          "Initialize"},
    {0x00020000, GetProcSemaphore,    "GetProcSemaphore"},
    {0x00030100, RegisterService,     "RegisterService"},
    {0x000400C0, nullptr,             "UnregisterService"},
    {0x00050100, GetServiceHandle,    "GetServiceHandle"},
    {0x000600C2, nullptr,             "RegisterHandle"},
    {0x00090040, nullptr,             "Subscribe"},
    {0x000B0000, nullptr,             "ReceiveNotification"},
    {0x000C0080, nullptr,             "PublishToSubscriber"},
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Interface class

Interface::Interface() {
    Register(FunctionTable);
}

Interface::~Interface() {
    event_handle = nullptr;
}

} // namespace
