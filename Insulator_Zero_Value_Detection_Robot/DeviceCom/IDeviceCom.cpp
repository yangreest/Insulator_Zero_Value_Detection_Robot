#include "IDeviceCom.h"
#include "TcpClient.h"

IDeviceCom* IDeviceCom::GetIDeviceCom(int nComType)
{
	switch (nComType)
	{
	case 1:
		return new CTcpClientCom();
	default:
		return nullptr;
	}
}