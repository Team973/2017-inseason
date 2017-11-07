#pragma once

#include "pthread.h"
#include "lib/TaskMgr.h"
#include "lib/CoopTask.h"
#include "WPILib.h"
#include <vector>
#include <sys/socket.h>
#include "lib/json/json.h"
#include "lib/SingleThreadTaskMgr.h"

using std::vector;

namespace frc973 {

class VisionUpdateReceiver {
public:
    VisionUpdateReceiver() {};
    virtual ~VisionUpdateReceiver() {}

    virtual void GotUpdate(const Json::Value &val) = 0;
};

class Client_t {
public:
    Client_t(int sock_fd);
    virtual ~Client_t() {}
    int DoRecv();
    int DoSend(std::string msg);
    bool MessageRecvComplete();
    int GetMessage(Json::Value &value);
    int GetSockFd();
private:
    static constexpr int RECV_BUFF_SIZE = 1024;
    int m_sock_fd;
    char m_recv_buff[RECV_BUFF_SIZE];
    int m_recv_buff_cursor;
};

class VisionServer : public CoopTask {
public:
    explicit VisionServer(RobotStateInterface &stateProvider);
    virtual ~VisionServer();

    bool IsConnected();
    void RequestAppRestart();
    void RestartAdb();

    void AddUpdateReceiver(VisionUpdateReceiver *receiver);
    void RemoveUpdateReceiver(VisionUpdateReceiver *receiver);

private:
    void TaskPeriodic(RobotMode mode) override;

    int StartListenSocket();
    int AddClient();
    int HandleClientMessage(Client_t *client);
    int SendHeartbeat(Client_t *client);
    int RemoveClient(int target_sock_fd);

    SingleThreadTaskMgr *m_thread;
    vector<VisionUpdateReceiver*> m_updateReceivers;
	pthread_mutex_t	m_updateReceiverMutex;
    double m_lastMessageReceivedTime;

    int m_serversock;
    vector<Client_t*> m_connections;
    fd_set m_fdset;

    static constexpr char const *SERVER_IPV4_ADDR = "127.0.0.1";
    static constexpr int SERVER_LISTEN_PORT = 33235;
    static constexpr double CONNECTION_TIMEOUT_SEC = 0.5;
};

}
