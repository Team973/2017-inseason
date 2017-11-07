/*
 * AdbBridge.h
 * 
 * Blatantly copied from:
 * https://github.com/Team254/FRC-2017-Public/blob/master/src/com/team254/
 *      frc2017/vision/AdbBridge.java
 */

#pragma once

#include <string>

namespace frc973 {

/*
 * Helper for managing adb link.  This class is not thread safe.
 */
class AdbBridge {
public:
    AdbBridge();
    virtual ~AdbBridge() {}

    void Start();
    void Stop();
    void RestartAdb();
    void PortForward(int local_post, int remote_port);
    void ReversePortForward(int remote_port, int local_post);
    void RestartApp();

private:
    const std::string ADB_PATH = "/usr/bin/adb";
    bool RunCommand(const std::string &command);
};

}
