#include "ConfigManager.h"
#include "../Tools/tinyxml2.h"

CControlBoardConfig::CControlBoardConfig()
{
	m_strIp = "";
	m_wPort = 0;
	m_wDeviceHeartBeat = 200;
	m_bFactoryMode = false;
}

void CConfigManager::Read(const std::string& filePath)
{
	tinyxml2::XMLDocument doc;
	tinyxml2::XMLError eResult = doc.LoadFile(filePath.c_str());
	int nIntTemp = 0;
	if (eResult == tinyxml2::XML_SUCCESS)
	{
		auto config = doc.RootElement();
		if (config != nullptr)
		{
			{
				auto deviceBoard = config->FirstChildElement("DeviceControlBoard");
				if (deviceBoard != nullptr)
				{
					auto ipElement = deviceBoard->FirstChildElement("Ip");
					if (ipElement && ipElement->GetText())
					{
						m_memControlBoardConfig.m_strIp = ipElement->GetText();
					}
					auto portElement = deviceBoard->FirstChildElement("Port");
					if (portElement != nullptr && portElement->QueryIntText(&nIntTemp) == tinyxml2::XML_SUCCESS)
					{
						m_memControlBoardConfig.m_wPort = nIntTemp;
					}
					auto ht = deviceBoard->FirstChildElement("DeviceHeartBeat");
					if (ht != nullptr && ht->QueryIntText(&nIntTemp) == tinyxml2::XML_SUCCESS)
					{
						m_memControlBoardConfig.m_wDeviceHeartBeat = nIntTemp;
					}

					auto dm = deviceBoard->FirstChildElement("FactoryMode");
					if (dm != nullptr && dm->QueryIntText(&nIntTemp) == tinyxml2::XML_SUCCESS)
					{
						m_memControlBoardConfig.m_bFactoryMode = nIntTemp > 0;
					}


					auto d3m = deviceBoard->FirstChildElement("UpAngle");
					if (d3m != nullptr && d3m->QueryIntText(&nIntTemp) == tinyxml2::XML_SUCCESS)
					{
						m_memControlBoardConfig.m_cUpAngle = nIntTemp;
					}
					auto d4m = deviceBoard->FirstChildElement("DownAngle");
					if (d4m != nullptr && d4m->QueryIntText(&nIntTemp) == tinyxml2::XML_SUCCESS)
					{
						m_memControlBoardConfig.m_cDownAngle = nIntTemp;
					}
					auto d5m = deviceBoard->FirstChildElement("WalkMotorSpeed");
					if (d5m != nullptr && d5m->QueryIntText(&nIntTemp) == tinyxml2::XML_SUCCESS)
					{
						m_memControlBoardConfig.m_cWalkMotorSpeed = nIntTemp;
					}

				}
			}
			{
				auto sb = config->FirstChildElement("Camera");
				if (sb != nullptr)
				{
					auto nLeft = sb->FirstChildElement("Left");
					if (nLeft != nullptr)
					{
						m_memCCameraConfig.m_strLeftIp = nLeft->GetText();
					}



					auto nRight = sb->FirstChildElement("Right");
					if (nRight != nullptr)
					{
						m_memCCameraConfig.m_strRightIp = nRight->GetText();
					}
				}
			}
		}
	}
}