#ifndef __LINUX_SERIAL_PORT_H__
#define __LINUX_SERIAL_PORT_H__

#include <thread>
#include <inttypes.h>
#include <atomic>
#include <mutex>
#include <vector>
#include <functional>
#include <string>
#include <condition_variable>
#include <string.h>
#include <chrono>

class CmdInterfaceLinux
{
public:
    CmdInterfaceLinux();
    ~CmdInterfaceLinux();

    bool Open(std::string& port_name);
    bool Close();
    bool ReadFromIO(uint8_t *rx_buf, uint32_t rx_buf_len, uint32_t *rx_len);
    bool WriteToIo(const uint8_t *tx_buf, uint32_t tx_buf_len, uint32_t *tx_len);
    bool GetCmdDevices(std::vector<std::pair<std::string, std::string> >& device_list);
	void SetReadCallback(std::function<void(const char *, size_t length)> callback) { mReadCallback = callback; }
    void SetSerialDataTimeoutDuration(int serial_data_timeout_duration) { _serial_data_timeout_duration = std::chrono::seconds(serial_data_timeout_duration); }
	bool IsOpened() { return mIsCmdOpened.load(); };
    bool isTimeout() { return _is_timeout; }

private:
    std::thread *mRxThread;
    void mRxThreadProc();
	long long mRxCount;
    int32_t mComHandle;
    std::atomic<bool> mIsCmdOpened, mRxThreadExitFlag;
	std::function<void(const char *, size_t length)> mReadCallback;
    std::atomic<bool> _is_timeout;
    std::chrono::steady_clock::time_point _last_valid_read;
    std::chrono::seconds _serial_data_timeout_duration;
};

#endif
