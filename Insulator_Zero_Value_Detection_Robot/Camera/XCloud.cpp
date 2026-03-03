#include "XCloud.h"

#include "XCloudSDK/XCloudSDK.h"

bool XCloud::m_bInited = false;
int XCloud::m_nUserInfo = 0;

XCloud::XCloud()
{
	m_pViewHandle = nullptr;
}

XCloud::~XCloud()
{
	Deinit();
}

bool XCloud::Init(const std::vector<std::string>& s)
{
	if (m_bInited)
	{
		return false;
	}
	// SDK初始化
	const char* sInit =
		R"({"LogLevel":0,"TempPath":"e:\temp","ConfigPath":"e:\temp","PlatUUID":"","PlatAppKey":"","PlatAppSecret":"","PlatMovedCard":0,"ServerIP":"","ServerPort":34567})";
	auto result = XCloudSDK_Init(sInit);
	if (result < 0)
	{
		return false;
	}
	m_bInited = true;
	m_nUserInfo = XCloudSDK_RegisterCallback((PXSDK_MessageCallBack)pMsgCallbackRealPlay, NULL);
	return m_nUserInfo >= 0;
}

bool XCloud::Deinit()
{
	if (m_bInited)
	{
		if (m_nUserInfo != 0)
		{
			XCloudSDK_UnRegister(m_nUserInfo);
			m_nUserInfo = 0;
		}
		XCloudSDK_UnInit();
		m_bInited = false;
		return true;
	}
	return false;
}

void XCloud::RegisterVideoViewHandle(void* handle)
{
	m_pViewHandle = handle;
}

bool XCloud::Connect(std::string ip, int port, std::string userName, std::string pwd)
{
	auto ptt = XCloudSDK_Device_MediaRealPlay(m_nUserInfo, ip.c_str(), 0, 0, (HWND)m_pViewHandle, 0, "");
	return ptt >= 0;
}

int XCloud::pMsgCallbackRealPlay(int hObject, int nMsgId, int nParam1, int nParam2, int nParam3, const char* szString,
	void* pObject, int64 lParam, int nSeq, void* pUserData, void* pMsg)
{
	//回调暂时先不判断，
	return 0;
}