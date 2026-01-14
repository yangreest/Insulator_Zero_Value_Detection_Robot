#pragma once
#include <condition_variable>
#include <queue>

#include "IDeviceCom.h"
#include <string>

class CTcpClientCom :public IDeviceCom
{
public:
	CTcpClientCom();
	void SetParam(const char* pComName, int nComPort) override;
	void RegisterReadDataCallBack(const std::function<void(uint8_t*, int, uint64_t)>& f) override;
	void RegisterConnectStatusCallBack(const std::function<void(bool, int)>& f)override;
	bool Write(uint8_t* data, size_t  len) override;
	bool BeginWork() override;
	bool EndWork() override;
private:
	void ConnectionThread();
	void Connect();
	void CloseSocket();
	void SetConnected(bool b);
	void SendThread();
	void ReceiveThread();

	std::string m_strTargetIp;

	uint16_t m_wTargetPort;

	uint64_t m_socket = 0;

	std::atomic<bool> m_running;
	std::atomic<bool> m_connected;

	std::thread m_connectionThread;
	std::thread m_receiveThread;
	std::thread m_sendThread;

	std::queue<std::vector<uint8_t>> m_sendQueue;
	std::mutex m_sendQueueMutex;
	std::condition_variable m_sendQueueCV;

	std::function<void(bool, int)> m_connectCallback;

	std::function<void(uint8_t*, int, uint64_t)> m_function_ReadDataCallBack;
};
