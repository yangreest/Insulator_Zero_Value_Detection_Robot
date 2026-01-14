#pragma once

#include <functional>
class IDeviceCom
{
public:
	static IDeviceCom* GetIDeviceCom(int nComType);
	virtual ~IDeviceCom() = default;
	virtual void SetParam(const char* pComName, int nComPort) = 0;
	virtual void RegisterReadDataCallBack(const std::function<void(uint8_t*, int, uint64_t)>& f) = 0;
	virtual void RegisterConnectStatusCallBack(const std::function<void(bool, int)>& f) = 0;
	virtual bool Write(uint8_t* data, size_t  len) = 0;
	virtual bool BeginWork() = 0;
	virtual bool EndWork() = 0;
};