#include <stdio.h>
#include <stdlib.h>
#include <lib/AdbBridge.h>

namespace frc973 {

AdbBridge::AdbBridge() {
}

void AdbBridge::Start() {
    RunCommand("start-server");
}

void AdbBridge::Stop() {
    RunCommand("stop-server");
}

void AdbBridge::RestartAdb() {
    RunCommand("kill-server");
}

void AdbBridge::PortForward(int local_port, int remote_port) {
    char buff[80];
    snprintf(buff, 79, "forward --remove-all");
    RunCommand(buff);
    snprintf(buff, 79, "reverse --remove-all");
    RunCommand(buff);
    snprintf(buff, 79, "forward tcp:%d tcp:%d", local_port, remote_port);
    RunCommand(buff);
}

void AdbBridge::ReversePortForward(int remote_port, int local_port) {
    char buff[80];
    snprintf(buff, 79, "forward --remove-all");
    RunCommand(buff);
    snprintf(buff, 79, "reverse --remove-all");
    RunCommand(buff);
    snprintf(buff, 79, "reverse tcp:%d tcp:%d", remote_port, local_port);
    RunCommand(buff);
}

void AdbBridge::RestartApp() {
    RunCommand("shell am force-stop com.team254.cheezdroid \\; "
               "am start com.team254.cheezdroid/"
               "com.team254.cheezdroid.VisionTrackerActivity");
}

bool AdbBridge::RunCommand(const std::string &command) {
    std::string prefixed_command = ADB_PATH + " " + command;

    printf("Command to be run: \n%s\n", prefixed_command.c_str());

    int res = system(prefixed_command.c_str());

    if (res == 0) {
        return true;
    }
    else if (res == -1) {
        fprintf(stderr, "\nSYSTEM ERROR: Could not create child thread\n");
        perror("SYSTEM ERROR");
        return false;
    }
    else {
        fprintf(stderr, "\nSYSTEM ERROR: non-zero return code: %d\n", res);
        perror("SYSTEM ERROR");
        return false;
    }
}

}
