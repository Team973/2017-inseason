#include "lib/VisionServer.h"
#include "lib/util/Util.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <vector>
#include <unistd.h>
#include <errno.h>
#include <iostream>

namespace frc973 {
    
VisionServer::VisionServer(RobotStateInterface &stateProvider) :
    m_thread(new SingleThreadTaskMgr(stateProvider, 1/100.0, false)),
    m_updateReceiverMutex(PTHREAD_MUTEX_INITIALIZER),
    m_lastMessageReceivedTime(0.0)
{
    if (StartListenSocket()) {
        fprintf(stderr, "Could not start up vision server");
        return;
    }
    m_thread->Start();
}

VisionServer::~VisionServer() {
    m_thread->Stop();
    delete m_thread;
}

bool VisionServer::IsConnected() {
    return GetSecTime() - m_lastMessageReceivedTime < CONNECTION_TIMEOUT_SEC;
}

void VisionServer::AddUpdateReceiver(VisionUpdateReceiver *receiver) {
    m_updateReceivers.insert(m_updateReceivers.begin(), receiver);
}

void VisionServer::RemoveUpdateReceiver(VisionUpdateReceiver *receiver) {
    for (auto it = m_updateReceivers.begin();
         it != m_updateReceivers.end();
         it++) {
        if ((*it) == receiver) {
            m_updateReceivers.erase(it);
            return;
        }
    }
}

int VisionServer::StartListenSocket() {
    m_serversock = socket(AF_INET, SOCK_STREAM, 0);
    if (m_serversock < 0) {
        perror("socket");
        return -1;
    }

    int reuse = 1;
    if (setsockopt(m_serversock, SOL_SOCKET, SO_REUSEADDR, &reuse,
                   sizeof(reuse))
            != 0) {
        perror("setsockopt");
        return -1;
    }

    struct sockaddr_in my_addr;
    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_addr.s_addr = inet_addr(SERVER_IPV4_ADDR);
    my_addr.sin_port = htons(SERVER_LISTEN_PORT);

    if (bind(m_serversock, (struct sockaddr*) &my_addr,
             sizeof(struct sockaddr))
            != 0) {
        perror("bind");
        return -1;
    }

    if (listen(m_serversock, 10) != 0) {
        perror("listen");
        return -1;
    }

    return 0;
}

int VisionServer::AddClient() {
    struct sockaddr_in client_addr;
    memset(&client_addr, 0, sizeof(client_addr));
    socklen_t client_len = sizeof(client_addr);
    int client_sock = accept(m_serversock, (struct sockaddr *) &client_addr, &client_len);
    if (client_sock < 0) {
        perror("accept");
        return -1;
    }

    char client_ipv4_str[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &client_addr.sin_addr, client_ipv4_str, INET_ADDRSTRLEN);

    Client_t *client = new Client_t(client_sock);

    m_connections.insert(m_connections.begin(), client);

    return 0;
}

int VisionServer::RemoveClient(int target_sock_fd) {
    for (auto it = m_connections.begin(); it != m_connections.end(); it++) {
        if ((*it)->GetSockFd() == target_sock_fd) {
            m_connections.erase(it);
            delete *it;
            close(target_sock_fd);
            return 0;
        }
    }
    fprintf(stderr, "Tried to close a socket that wasn't connected\n");

    return -1;
}

int VisionServer::HandleClientMessage(Client_t *client) {
    int res = client->DoRecv();

    if (res < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            fprintf(stderr, "Connection not ready.  Try again later.\n");
            return 0;
        }
        else {
            perror("recv");
            fprintf(stderr, "client died.\n");
            return -1;
        }
    }
    else if (res == 0) {
        fprintf(stderr, "Client closed connection normally. Removing.\n");
        return -1;
    }
    else  {
        if (client->MessageRecvComplete()) {
            Json::Value value;
            if (client->GetMessage(value)) {
                printf("We received a message but there was an error getting it?\n");
            }
            else {
                printf("*** MESSAGE RECEIVED ***\n");
                std::cout << value << std::endl;
                printf("*** END MESSAGE RECEIVED ***\n");

                std::string msg_type = value.get("type", "none").asString();
                if (msg_type == "heartbeat") {
                    SendHeartbeat(client);
                }
                else {
                    for (auto it = m_updateReceivers.begin();
                         it != m_updateReceivers.end();
                         it++) {
                        (*it)->GotUpdate(value);
                    }
                }
            }
        }
    }

    return 0;
}

int VisionServer::SendHeartbeat(Client_t *client) {
    return client->DoSend("{\"message\": \"{}\", \"type\": \"heartbeat\"}");
}

void VisionServer::TaskPeriodic(RobotMode mode) {
    int high_fd = m_serversock;
    FD_ZERO(&m_fdset);
    FD_SET(m_serversock, &m_fdset);
    for (auto it = m_connections.begin(); it != m_connections.end(); it++) {
        int sock_fd = (*it)->GetSockFd();
        FD_SET(sock_fd, &m_fdset);
        high_fd = std::max(high_fd, sock_fd);
    }

    int res = select(high_fd + 1, &m_fdset, NULL, NULL, NULL);

    if (res < 0) {
        perror("select");
    }
    if (res <= 0) {
        fprintf(stderr, "Error in select.  Return code %d\n", res);
    }

    if (res > 0) {
        if (FD_ISSET(m_serversock, &m_fdset)) {
            AddClient();
        }

        vector<int> clients_to_remove;
        for (auto it = m_connections.begin();
                it != m_connections.end();
                it++) {
            Client_t *client = *it;
            if (FD_ISSET(client->GetSockFd(), &m_fdset)) {
                if (HandleClientMessage(client)) {
                    clients_to_remove.insert(clients_to_remove.begin(),
                                             client->GetSockFd());
                }
            }
        }

        for (auto it = clients_to_remove.begin();
                it != clients_to_remove.end();
                it++) {
            RemoveClient(*it);
        }
    }
}

Client_t::Client_t(int sock_fd) {
    m_sock_fd = sock_fd;
    memset(m_recv_buff, 0, RECV_BUFF_SIZE);
    m_recv_buff_cursor = 0;
}

int Client_t::DoRecv() {
    if (RECV_BUFF_SIZE <= m_recv_buff_cursor + 1) {
        fprintf(stderr, "Message too large to fit in buffer.  Error.\n");
        return 0;
    }

    int res = recv(m_sock_fd, m_recv_buff + m_recv_buff_cursor,
                   RECV_BUFF_SIZE - m_recv_buff_cursor - 1, 0);

    if (res > 0) {
        m_recv_buff_cursor += res;
    }

    return res;
}

int Client_t::DoSend(std::string msg) {
    int sent = 0;
    const char *buff = msg.c_str();

    while (1) {
        int res = send(m_sock_fd, buff + sent, msg.length() - sent, 0); 
        if (res > 0) {
            sent += res;
        }
        else {
            return -1;
        }
    }

    return 0;
}

/**
 * This is not a secure function.  If there is a bracket in a string then
 * this function will return the wrong result.
 * Later this should be JSON::Parse
 **/
bool Client_t::MessageRecvComplete() {
    int openCount = 0;
    for (int i = 0; i < m_recv_buff_cursor; i++) {
        if (m_recv_buff[i] == '{') {
            openCount++;
        }
        else if (m_recv_buff[i] == '}') {
            openCount--;
        }
    }
    return m_recv_buff_cursor > 0 && openCount == 0;
}

int Client_t::GetMessage(Json::Value &value) {
    std::string errs;
    Json::CharReaderBuilder rBuilder;
    std::istringstream buffStream(std::string(m_recv_buff, m_recv_buff_cursor));

    bool success = Json::parseFromStream(rBuilder, buffStream,
                                         &value, &errs);

    if (success == false) {
        fprintf(stderr, "JSON error: %s\n", errs.c_str());
        return 1;
    }

    m_recv_buff_cursor = 0;
    return 0;
}

int Client_t::GetSockFd() {
    return m_sock_fd;
}

}
