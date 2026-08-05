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
                    auto d6m = deviceBoard->FirstChildElement("UpAngle2");
                    if (d6m != nullptr && d6m->QueryIntText(&nIntTemp) == tinyxml2::XML_SUCCESS)
                    {
                        m_memControlBoardConfig.m_cUpAngle2 = nIntTemp;
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

					auto nMid = sb->FirstChildElement("Mid");
                    if (nMid != nullptr)
                    {
                        m_memCCameraConfig.m_strMidIp = nMid->GetText();
                    }

					auto nRight = sb->FirstChildElement("Right");
					if (nRight != nullptr)
					{
						m_memCCameraConfig.m_strRightIp = nRight->GetText();
					}

					auto bNew = sb->FirstChildElement("NewCamera");
					if (bNew != nullptr && bNew->QueryIntText(&nIntTemp) == tinyxml2::XML_SUCCESS)
					{
						m_memCCameraConfig.m_bNewCamera = nIntTemp > 0;
					}
					
                    auto bUse = doc.FirstChildElement("UseMainSp");
                    if (bUse != nullptr && bUse->QueryIntText(&nIntTemp) == tinyxml2::XML_SUCCESS)
                    {
                        m_memCCameraConfig.m_bUseMainSp = nIntTemp > 0;
                    }

					auto MainRtsp = sb->FirstChildElement("MainRtsp");
					if (MainRtsp != nullptr&& MainRtsp->GetText())
					{
						m_memCCameraConfig.m_strMainRtsp = MainRtsp->GetText();
					}

					auto SubRtsp = sb->FirstChildElement("SubRtsp");
					if (SubRtsp != nullptr&& SubRtsp->GetText())
					{
						m_memCCameraConfig.m_strSubRtsp = SubRtsp->GetText();
					}
				}
			}
		}
	}
}
void CConfigManager::Write(const std::string& filePath)
{
	tinyxml2::XMLDocument doc;
	// 添加 XML 声明
	//doc.InsertEndChild(doc.NewDeclaration());

	// 创建根节点（请根据实际 XML 文件的根节点名称调整，例如 "Config" 或 "Root"）
	tinyxml2::XMLElement* root = doc.NewElement("Config");
	doc.InsertEndChild(root);

	// --- 写入 DeviceControlBoard ---
	{
		tinyxml2::XMLElement* deviceBoard = doc.NewElement("DeviceControlBoard");
		root->InsertEndChild(deviceBoard);

		tinyxml2::XMLElement* ipElem = doc.NewElement("Ip");
		ipElem->SetText(m_memControlBoardConfig.m_strIp.c_str());
		deviceBoard->InsertEndChild(ipElem);

		tinyxml2::XMLElement* portElem = doc.NewElement("Port");
		portElem->SetText(m_memControlBoardConfig.m_wPort);
		deviceBoard->InsertEndChild(portElem);

		tinyxml2::XMLElement* hbElem = doc.NewElement("DeviceHeartBeat");
		hbElem->SetText(m_memControlBoardConfig.m_wDeviceHeartBeat);
		deviceBoard->InsertEndChild(hbElem);

		tinyxml2::XMLElement* fmElem = doc.NewElement("FactoryMode");
		fmElem->SetText(m_memControlBoardConfig.m_bFactoryMode ? 1 : 0);
		deviceBoard->InsertEndChild(fmElem);

		tinyxml2::XMLElement* upElem = doc.NewElement("UpAngle");
		upElem->SetText(m_memControlBoardConfig.m_cUpAngle);
		deviceBoard->InsertEndChild(upElem);

		tinyxml2::XMLElement* downElem = doc.NewElement("DownAngle");
		downElem->SetText(m_memControlBoardConfig.m_cDownAngle);
		deviceBoard->InsertEndChild(downElem);

		tinyxml2::XMLElement* speedElem = doc.NewElement("WalkMotorSpeed");
		speedElem->SetText(m_memControlBoardConfig.m_cWalkMotorSpeed);
		deviceBoard->InsertEndChild(speedElem);

        tinyxml2::XMLElement* up2Elem = doc.NewElement("UpAngle2");
        up2Elem->SetText(m_memControlBoardConfig.m_cUpAngle2);
		deviceBoard->InsertEndChild(up2Elem);
	}

	// --- 写入 Camera ---
	{
		tinyxml2::XMLElement* camera = doc.NewElement("Camera");
		root->InsertEndChild(camera);

		tinyxml2::XMLElement* leftElem = doc.NewElement("Left");
		leftElem->SetText(m_memCCameraConfig.m_strLeftIp.c_str());
		camera->InsertEndChild(leftElem);

		tinyxml2::XMLElement* midElem = doc.NewElement("Mid");
        midElem->SetText(m_memCCameraConfig.m_strMidIp.c_str());
        camera->InsertEndChild(midElem);

		tinyxml2::XMLElement* rightElem = doc.NewElement("Right");
		rightElem->SetText(m_memCCameraConfig.m_strRightIp.c_str());
		camera->InsertEndChild(rightElem);

		tinyxml2::XMLElement* NewCameraElem = doc.NewElement("NewCamera");
        NewCameraElem->SetText(m_memCCameraConfig.m_bNewCamera ? "1":"0");
        camera->InsertEndChild(NewCameraElem);

		tinyxml2::XMLElement* UseMainSpElem = doc.NewElement("UseMainSp");
        UseMainSpElem->SetText(m_memCCameraConfig.m_bUseMainSp ? "1":"0");
        camera->InsertEndChild(UseMainSpElem);

		tinyxml2::XMLElement* MainRtspElem = doc.NewElement("MainRtsp");
        MainRtspElem->SetText(m_memCCameraConfig.m_strMainRtsp.c_str());
        camera->InsertEndChild(MainRtspElem);

        tinyxml2::XMLElement* SubRtspElem = doc.NewElement("SubRtsp");
        SubRtspElem->SetText(m_memCCameraConfig.m_strSubRtsp.c_str());
        camera->InsertEndChild(SubRtspElem);
	}

	// 保存至文件
	tinyxml2::XMLError eResult = doc.SaveFile(filePath.c_str());
	if (eResult != tinyxml2::XML_SUCCESS)
	{
		// 可根据项目规范添加日志记录或异常处理
	}
}