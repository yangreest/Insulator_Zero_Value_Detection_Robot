#pragma once
#include <string>
#include <vector>

class ICameraBase
{
public:
	virtual ~ICameraBase();
	virtual bool Init(const std::vector<std::string>& s) = 0;
	virtual bool Deinit() = 0;
	virtual void RegisterVideoViewHandle(void* handle) = 0;
	virtual bool Connect(std::string ip, int port, std::string userName, std::string pwd) = 0;
	static ICameraBase* GetCameraObj(int type);
};
