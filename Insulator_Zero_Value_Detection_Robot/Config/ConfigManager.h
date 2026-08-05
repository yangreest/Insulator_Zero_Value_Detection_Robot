#pragma once
#include <string>
#include <vector>


class CControlBoardConfig
{
public:
	CControlBoardConfig();
	std::string m_strIp;
	uint16_t m_wPort;
	uint16_t m_wDeviceHeartBeat;
	bool m_bFactoryMode;
	uint8_t m_cUpAngle;
	uint8_t m_cDownAngle;
	uint8_t m_cUpAngle2;
	uint8_t m_cWalkMotorSpeed;
};

class CCameraConfig
{
public:
	std::string m_strLeftIp;
	std::string m_strMidIp;
	std::string m_strRightIp;

	bool m_bNewCamera; // 使用新款相机
	bool m_bUseMainSp;// 使用主码流
	std::string m_strMainRtsp; // 主码流
	std::string m_strSubRtsp;// 子码流
};


class CConfigManager
{
public:
	CControlBoardConfig m_memControlBoardConfig;
	CCameraConfig m_memCCameraConfig;
	void Read(const std::string& filePath);
	void Write(const std::string& filePath);
};
