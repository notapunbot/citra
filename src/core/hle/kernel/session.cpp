// Copyright 2016 Citra Emulator Project
// Licensed under GPLv2 or any later version
// Refer to the license.txt file included.

#include "core/hle/kernel/session.h"
#include "core/hle/kernel/process.h"
#include "core/hle/kernel/thread.h"

namespace Kernel {

SharedPtr<ServerPort> ServerPort::Create(const std::string& name, u32 max_sessions) {
    SharedPtr<ServerPort> server_port(new ServerPort);
    server_port->name = name;
    server_port->owner_process = g_current_process;
    server_port->max_sessions = max_sessions;

    return server_port;
}

SharedPtr<ClientSession> ServerPort::CreateSession() {
    SharedPtr<ClientSession> client_session(new ClientSession);
    if (IsHLE()) {
        client_session->hle_port = this;
    } else {
        SharedPtr<ServerSession> server_session(new ServerSession);
        client_session->server_session = server_session;
        pending_sessions.push_back(server_session);
        // Set the register to the value of the triggered index from svcReplyAndReceive
        WakeupAllWaitingThreads();
    }
    // TODO(Subv): Schedule to the thread waiting on svcReplyAndReceive
    return client_session;
}

ClientSession::ClientSession() {}
ClientSession::~ClientSession() {}

ResultVal<bool> ClientSession::SyncRequest() {
    if (hle_port) {
        hle_port->SyncRequest();
    } else {
        server_session->data_ready = true;
        server_session->waiting_thread = Kernel::GetCurrentThread();
        // TODO(Subv): Sleep the current thread until the response is ready
        // TODO(Subv): Immediately schedule the handler thread
    }
    return MakeResult<bool>(true);
}

ResultVal<bool> ServerPort::SyncRequest() {

    return MakeResult<bool>(true);
}

}
