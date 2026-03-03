#pragma once
#include <XBasic/XBaseTypes.h>

#include "CameraBase.h"

class XCloud final : public ICameraBase
{
public:
	XCloud();
	~XCloud();
	bool Init(const std::vector<std::string>& s) override;
	void RegisterVideoViewHandle(void* handle) override;
	bool Deinit() override;
	bool Connect(std::string ip, int port, std::string userName, std::string pwd) override;

private:
	static bool m_bInited;
	void* m_pViewHandle;
	static int pMsgCallbackRealPlay(XSDK_HANDLE hObject,
		int nMsgId,
		int nParam1,
		int nParam2,
		int nParam3,
		const char* szString,
		void* pObject,
		int64 lParam,
		int nSeq,
		void* pUserData,
		void* pMsg);

private:
	static int m_nUserInfo;
};
