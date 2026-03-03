#include "CameraBase.h"
#include "XCloud.h"

ICameraBase::~ICameraBase()
{
}

ICameraBase* ICameraBase::GetCameraObj(int type)
{
	switch (type)
	{
	case 0:
	{
		return new XCloud();
	}
	default:
		return nullptr;
	}
}