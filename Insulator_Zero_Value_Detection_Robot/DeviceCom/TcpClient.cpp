#include "TcpClient.h"

#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <string>
#include <atomic>
#include <condition_variable>
#include <chrono>

#ifdef _DEBUG
bool bWriteLog = true;
#include <iostream>
#include <sstream>
#include <iomanip>

#endif

#pragma comment(lib, "ws2_32.lib")

CTcpClientCom::CTcpClientCom() : m_wTargetPort(0)
{
	m_function_ReadDataCallBack = nullptr;
	m_connectCallback = nullptr;
}

void CTcpClientCom::SetParam(const char* pComName, int nComPort)
{
	m_strTargetIp = pComName;
	m_wTargetPort = nComPort;
}

void CTcpClientCom::RegisterReadDataCallBack(const std::function<void(uint8_t*, int, uint64_t)>& f)
{
	m_function_ReadDataCallBack = f;
}

void CTcpClientCom::RegisterConnectStatusCallBack(const std::function<void(bool, int)>& f)
{
	m_connectCallback = f;
}

bool CTcpClientCom::Write(uint8_t* data, size_t  len)
{
	if (!m_running || !m_connected)
	{
		return false;
	}

	{
		std::unique_lock<std::mutex> lock(m_sendQueueMutex);
		m_sendQueue.emplace(data, data + len);
		m_sendQueueCV.notify_one();
	}

	return true;
}

bool CTcpClientCom::BeginWork()
{
	if (m_running)
	{
		return true;
	}

	// 初始化Winsock
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		return false;
	}

	m_running = true;

	m_connected = false;

	// 启动连接线程
	m_connectionThread = std::thread(&CTcpClientCom::ConnectionThread, this);

	// 启动接收线程
	m_receiveThread = std::thread(&CTcpClientCom::ReceiveThread, this);

	//启动发送线程
	m_sendThread = std::thread(&CTcpClientCom::SendThread, this);

	return true;
}

bool CTcpClientCom::EndWork()
{
	if (!m_running)
	{
		return true;
	}

	m_running = false;

	// 通知条件变量
	{
		std::unique_lock<std::mutex> lock(m_sendQueueMutex);
		m_sendQueueCV.notify_all();
	}

	// 关闭套接字
	CloseSocket();

	// 等待线程结束
	if (m_connectionThread.joinable())
	{
		m_connectionThread.join();
	}

	if (m_receiveThread.joinable())
	{
		m_receiveThread.join();
	}

	// 清理Winsock
	WSACleanup();
	return true;
}

void CTcpClientCom::ConnectionThread()
{
	while (m_running)
	{
		if (!m_connected)
		{
			Connect();
		}

		// 等待一段时间再尝试重连
		std::this_thread::sleep_for(std::chrono::seconds(5));
	}
}

// 设置连接状态
void CTcpClientCom::SetConnected(bool connected)
{
	if (m_connected != connected)
	{
		m_connected = connected;

		// 调用连接状态回调
		if (m_connectCallback != nullptr)
		{
			m_connectCallback(connected, 0);
		}
	}
}

void CTcpClientCom::CloseSocket()
{
	if (m_socket != INVALID_SOCKET)
	{
		closesocket(m_socket);
		m_socket = INVALID_SOCKET;
	}

	SetConnected(false);
}

void CTcpClientCom::Connect()
{
	CloseSocket();

	// 创建套接字
	m_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (m_socket == INVALID_SOCKET)
	{
		return;
	}

	// 设置为非阻塞模式
	unsigned long nonBlocking = 1;
	if (ioctlsocket(m_socket, FIONBIO, &nonBlocking) != 0)
	{
		CloseSocket();
		return;
	}

	// 设置服务器地址
	sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(m_wTargetPort);

	// 转换IP地址
	inet_pton(AF_INET, m_strTargetIp.c_str(), &serverAddr.sin_addr);

	// 尝试连接
	int result = connect(m_socket, (sockaddr*)&serverAddr, sizeof(serverAddr));
	if (result == SOCKET_ERROR)
	{
		int error = WSAGetLastError();
		if (error == WSAEWOULDBLOCK)
		{
			// 连接正在进行中，使用select检查连接状态
			fd_set writeSet;
			timeval timeout;

			FD_ZERO(&writeSet);
			FD_SET(m_socket, &writeSet);
			timeout.tv_sec = 5;
			timeout.tv_usec = 0;

			result = select(0, nullptr, &writeSet, nullptr, &timeout);
			if (result > 0)
			{
				if (FD_ISSET(m_socket, &writeSet))
				{
					// 检查是否有错误
					int errorCode;
					int errorSize = sizeof(errorCode);
					if (getsockopt(m_socket, SOL_SOCKET, SO_ERROR, (char*)&errorCode, &errorSize) == 0)
					{
						if (errorCode == 0)
						{
							SetConnected(true);
							// 启动发送线程
							//m_sendThread = std::thread(&CTcpClientCom::SendThread, this);
							return;
						}
					}
				}
			}
		}
	}
	else
	{
		SetConnected(true);
		// 启动发送线程
		//m_sendThread = std::thread(&CTcpClientCom::SendThread, this);
	}

	CloseSocket();
}

void CTcpClientCom::SendThread()
{
	while (m_running)
	{
		if (!m_connected)
		{
			Sleep(10);
			continue;
		}
		std::vector<uint8_t> data;

		// 等待数据或连接关闭
		{
			std::unique_lock<std::mutex> lock(m_sendQueueMutex);
			m_sendQueueCV.wait(lock, [this] { return !m_sendQueue.empty() || !m_connected || !m_running; });
			if (!m_connected || !m_running || m_sendQueue.empty())
			{
				Sleep(10);
				continue;
			}

			data = std::move(m_sendQueue.front());
			m_sendQueue.pop();
		}

		// 发送数据
		if (!data.empty() && m_connected)
		{
			auto bytesSent = send(m_socket, (char*)data.data(), (int)data.size(), 0);
#ifdef _DEBUG
			if (bWriteLog)
			{
				auto toHexString = [](const uint8_t* t, size_t  len)
					{
						std::ostringstream oss;
						oss << std::hex << std::setfill('0');
						for (int i = 0; i < len; ++i)
						{
							oss << std::setw(2) << static_cast<int>(t[i]);
							if (i < len - 1) oss << ' ';
						}
						return oss.str();
					};
				std::cout << "send data:" << toHexString(data.data(), data.size()) << std::endl;
			}
#endif
			if (bytesSent == SOCKET_ERROR)
			{
				int error = WSAGetLastError();
				if (error != WSAEWOULDBLOCK && error != WSAEINTR)
				{
					SetConnected(false);
					break;
				}
			}
		}
		else
		{
			Sleep(10);
		}
	}
}

// 接收线程函数
void CTcpClientCom::ReceiveThread()
{
	while (m_running)
	{
		if (m_connected)
		{
			char buffer[1024];
			fd_set readSet;
			timeval timeout;

			FD_ZERO(&readSet);
			FD_SET(m_socket, &readSet);
			timeout.tv_sec = 1; // 1秒超时
			timeout.tv_usec = 0;

			// 使用select检查是否有数据可读
			int result = select(0, &readSet, nullptr, nullptr, &timeout);
			if (result > 0)
			{
				if (FD_ISSET(m_socket, &readSet))
				{
					int bytesReceived = recv(m_socket, buffer, sizeof(buffer) - 1, 0);
					if (bytesReceived > 0)
					{
						if (m_function_ReadDataCallBack != nullptr)
						{
							m_function_ReadDataCallBack((uint8_t*)buffer, bytesReceived, 0);
						}

#ifdef _DEBUG
						if (bWriteLog)
						{
							auto toHexString = [](const uint8_t* t, int len)
								{
									std::ostringstream oss;
									oss << std::hex << std::setfill('0');
									for (int i = 0; i < len; ++i)
									{
										oss << std::setw(2) << static_cast<int>(t[i]);
										if (i < len - 1) oss << ' ';
									}
									return oss.str();
								};
							std::cout << "get data:" << toHexString((uint8_t*)buffer, bytesReceived) << std::endl;
						}
#endif
					}
					else if (bytesReceived == 0)
					{
						// 连接关闭
						SetConnected(false);
					}
					else
					{
						// 接收错误
						int error = WSAGetLastError();
						if (error != WSAEWOULDBLOCK && error != WSAEINTR)
						{
							SetConnected(false);
						}
					}
				}
			}
			else if (result == 0)
			{
				// 超时，继续循环
			}
			else
			{
				// select错误
				int error = WSAGetLastError();
				SetConnected(false);
			}
		}
		else
		{
			// 未连接，等待
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}
}