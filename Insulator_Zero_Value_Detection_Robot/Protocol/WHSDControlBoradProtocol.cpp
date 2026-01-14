#include "WHSDControlBoradProtocol.h"

#include <thread>
#include <Windows.h>

#include "Tools/Tools.h"

#define DEVICE_HEART_BEAT_LEN 64
#define MOTOR_COUNT 8
#define OTA_PACK_LEN 128

uint8_t CWHSDControlBoardProtocol::cPackNumber = 0;

CSensorData::CSensorData()
{
	m_cSensorIndex = 0;
	m_cCmd = 0;
	m_wValue = 0;
}

CControlBoardProtocolConfig::CControlBoardProtocolConfig()
{
	m_wLidarMinDis = 0;
	m_wLidarMaxDis = 0;
	m_fSafeAngle = 0;
	m_fSBAngle = 0;
	m_fWalkingV = 0;
	m_fSafeV = 0;
	m_fSBV = 0;
	m_fJYV = 0;
	m_fJY2V = 0;
	m_nSBRun = 0;
}

std::vector<uint8_t> CControlBoardProtocolConfig::GetDataByte()
{
	auto dataLen = sizeof(m_wLidarMinDis) + sizeof(m_wLidarMaxDis) + sizeof(m_fSafeAngle) + sizeof(m_fSBAngle) + sizeof(
		m_fWalkingV) + sizeof(m_fSafeV) + sizeof(m_fSBV) + sizeof(m_fJYV) + sizeof(m_fJY2V) + sizeof(m_nSBRun);
	std::vector<uint8_t> rzt(dataLen);
	int index = 0;
	int si = 0;

	si = sizeof(m_wLidarMinDis);
	memcpy(rzt.data() + index, &m_wLidarMinDis, si);
	index = index + si;

	si = sizeof(m_wLidarMaxDis);
	memcpy(rzt.data() + index, &m_wLidarMaxDis, si);
	index = index + si;

	si = sizeof(m_fSafeAngle);
	memcpy(rzt.data() + index, &m_fSafeAngle, si);
	index = index + si;

	si = sizeof(m_fSBAngle);
	memcpy(rzt.data() + index, &m_fSBAngle, si);
	index = index + si;

	si = sizeof(m_fWalkingV);
	memcpy(rzt.data() + index, &m_fWalkingV, si);
	index = index + si;

	si = sizeof(m_fSafeV);
	memcpy(rzt.data() + index, &m_fSafeV, si);
	index = index + si;

	si = sizeof(m_fSBV);
	memcpy(rzt.data() + index, &m_fSBV, si);
	index = index + si;

	si = sizeof(m_fJYV);
	memcpy(rzt.data() + index, &m_fJYV, si);
	index = index + si;

	si = sizeof(m_fJY2V);
	memcpy(rzt.data() + index, &m_fJY2V, si);
	index = index + si;

	si = sizeof(m_nSBRun);
	memcpy(rzt.data() + index, &m_nSBRun, si);
	index = index + si;

	return rzt;
}

CMotorDeviceStatus::CMotorDeviceStatus()
{
	m_bDeviceEnable = false;
	m_cDeviceStatus = 0;
}

CDeviceHeartBeat::CDeviceHeartBeat()
{
	m_vectorWalkingMotorStatus.resize(MOTOR_COUNT);
	m_vectorWindmillMotorStatus.resize(MOTOR_COUNT);
	m_vectorSafetyMotorStatus.resize(MOTOR_COUNT);
	m_vectorSBMotorStatus.resize(MOTOR_COUNT);
	m_cBattery = 0;
	m_cHardwareYear = 0;
	m_cHardwareMonth = 0;
	m_cHardwareDay = 0;
	m_cHardwareVersionOfDay = 0;
	m_cXRayDeviceStatus = 0;
}

void CDeviceHeartBeat::ExtractMotorStatus(int motorType, const uint8_t* dat)
{
	CMotorDeviceStatus* motorStatus = nullptr;
	switch (motorType)
	{
	case 0:
	{
		motorStatus = m_vectorWalkingMotorStatus.data();
		break;
	}
	case 1:
	{
		motorStatus = m_vectorWindmillMotorStatus.data();
		break;
	}
	case 2:
	{
		motorStatus = m_vectorSafetyMotorStatus.data();
		break;
	}
	case 3:
	{
		motorStatus = m_vectorSBMotorStatus.data();
		break;
	}
	default:
	{
		return;
	}
	}
	for (int i = 0; i < MOTOR_COUNT; ++i)
	{
		motorStatus[i].m_cDeviceStatus = TOOLS_GET_BIT(dat[0], i);
	}
	//后面3个字节处理状态
	auto data = dat + 1;
	for (int i = 0; i < MOTOR_COUNT; ++i)
	{
		int bitPos = i * 3; // 电机状态的起始位
		int byteIdx = bitPos / 8; // 所在字节索引
		int shift = bitPos % 8; // 起始位在字节内的偏移

		// 提取当前电机的3位状态
		// 从当前字节提取部分或全部位
		uint8_t status = data[byteIdx] >> shift & 0x07;

		// 如果有剩余位在下一个字节中
		if (shift > 5 && byteIdx < 2)
		{
			uint8_t remainingBits = (data[byteIdx + 1] & (1 << (shift - 5)) - 1) << (8 - shift);
			status |= remainingBits;
		}

		motorStatus[i].m_cDeviceStatus = status;
	}
}

CWHSDControlBoardProtocol::CWHSDControlBoardProtocol(uint16_t wHeartBeatTime)
{
	m_cPackNumber = 0;
	m_function_Answer = nullptr;
	m_function_DeviceHeartBeat = nullptr;
	m_cCmd = 0;
	m_nIsNeedExit = false;
	m_function_WriteLogCallBack = nullptr;
	m_wHeartBeatTime = wHeartBeatTime;
	m_bPauseHeartBeat = false;
	m_cOTAStatus = 0;
	m_cOTAErrorCount = 0;
	m_nOTAPackIndex = 0;
	m_nOTAAllPacks = 0;
	m_function_OTAStatusCallBack = nullptr;
	m_function_SensorDataCallBack = nullptr;
}

bool CWHSDControlBoardProtocol::BeginWork()
{
	m_nIsNeedExit = false;
	std::thread td(&CWHSDControlBoardProtocol::LoopSendHeartBeat, this);
	td.detach();
	return true;
}

bool CWHSDControlBoardProtocol::EndWork()
{
	m_nIsNeedExit = true;
	return true;
}

void CWHSDControlBoardProtocol::ReceiveNewData(const uint8_t* p, int len)
{
	//1、先将数据添加至缓冲区最尾部
	auto oldSize = m_vectorDataBuffer.size();
	m_vectorDataBuffer.resize(oldSize + len);
	memcpy(m_vectorDataBuffer.data() + oldSize, p, len);
	while (true)
	{
		if (!Parse())
		{
			break;
		}
	}
}

bool CWHSDControlBoardProtocol::Parse()
{
	while (m_vectorDataBuffer.size() > 8)
	{
		if (m_vectorDataBuffer[0] == 0xff && m_vectorDataBuffer[1] == 0xfe)
		{
			//找到了包头
			auto packLen = m_vectorDataBuffer[2];
			if (m_vectorDataBuffer.size() < packLen)
			{
				return false;
			}
			if (m_vectorDataBuffer[packLen - 2] == 0xfd && m_vectorDataBuffer[packLen - 1] == 0xfc)
			{
				//找到了包尾
				auto checkSum = m_vectorDataBuffer[packLen - 3];
				uint8_t checkSum2 = 0;
				for (int i = 0; i < packLen - 3; i++)
				{
					checkSum2 = checkSum2 + m_vectorDataBuffer[i];
				}
				if (checkSum == checkSum2)
				{
					m_cPackNumber = m_vectorDataBuffer[3];
					m_vectorCmdData.resize(packLen - 8);
					auto cmd = m_vectorDataBuffer[4];
					memcpy(m_vectorCmdData.data(), m_vectorDataBuffer.data() + 5, packLen - 8);
					Erase(packLen);
					if (m_function_Answer != nullptr)
					{
						Answer();
					}
					switch (cmd)
					{
					case 0:
					{
						DealHeartBeat();
						break;
					}
					case 0x07:
					{
						if (m_function_WriteLogCallBack != nullptr)
						{
							m_function_WriteLogCallBack(std::string(
								m_vectorCmdData.data(), m_vectorCmdData.data() + m_vectorCmdData.size()));
						}
						break;
					}
					case 0xa:
					{
						//进入OTA模式的反馈
						if (!m_vectorCmdData.empty())
						{
							if (m_vectorCmdData[0] > 0x00)
							{
								m_cOTAErrorCount = 0;
								m_cOTAStatus = 2;
								if (m_vectorCmdData[0] == 0x02)
								{
									m_bPauseHeartBeat = true;
								}
							}
						}
						break;
					}
					case 0x0b:
					{
						if (m_vectorCmdData.size() >= 5)
						{
							if (m_vectorCmdData[0] > 0x00)
							{
								m_cOTAErrorCount = 0;
								memcpy(&m_nOTAPackIndex, m_vectorCmdData.data() + 1, sizeof(uint32_t));
							}
						}
						break;
					}
					case 0x0c:
					{
						//进入OTA模式的反馈
						//if (!m_vectorCmdData.empty())
						//{
						//	if (m_vectorCmdData[0] > 0x00)
						//	{
						//		m_cOTAResult = 255;
						//	}
						//	else
						//	{
						//		m_cOTAResult = 254;
						//	}
						//}
						break;
					}
					case 0x11:
						{
							//传感器指令反馈
						if (m_vectorCmdData.size() >= 3)
						{
							switch (m_vectorCmdData[1])
							{
							case 3:
							case 2:
							case 4:
								{
								m_memCSensorData.m_cSensorIndex = m_vectorCmdData[0];
								m_memCSensorData.m_cCmd = m_vectorCmdData[1];
									break;
								}
								default:
								{
									break;
								}
							}

							memcpy(&(m_memCSensorData.m_wValue), m_vectorCmdData.data() + 2, sizeof(m_memCSensorData.m_wValue));
							if (m_function_SensorDataCallBack != nullptr)
							{
								m_function_SensorDataCallBack(&m_memCSensorData);
							}
						}
							break;
						}
					default:
					{
						break;
					}
					}
					return true;
				}
				Erase(1);
			}
			else
			{
				Erase(1);
			}
		}
		else
		{
			Erase(1);
		}
	}
	return false;
}

void CWHSDControlBoardProtocol::RegisterAnswerFunction(const std::function<bool(uint8_t*, int)>& f)
{
	m_function_Answer = f;
}

void CWHSDControlBoardProtocol::RegisterDeviceHeartBeat(const std::function<void(const CDeviceHeartBeat&)>& f)
{
	m_function_DeviceHeartBeat = f;
}

void CWHSDControlBoardProtocol::RegisterDeviceLog(const std::function<void(const std::string&)>& f)
{
	m_function_WriteLogCallBack = f;
}

void CWHSDControlBoardProtocol::RegisterOTAStatus(const std::function<void(uint8_t, uint32_t, uint32_t)>& f)
{
	m_function_OTAStatusCallBack = f;
}

void CWHSDControlBoardProtocol::BeginOTA(const std::vector<uint8_t>& file)
{
	m_vector_OTAFile = file;
	std::thread td(&CWHSDControlBoardProtocol::OTAThread, this);
	td.detach();
}

void CWHSDControlBoardProtocol::RegisterSensorDataCallBack(const std::function<void(CSensorData*)>& p)
{
	m_function_SensorDataCallBack = p;
}

std::vector<uint8_t> CWHSDControlBoardProtocol::DeviceRun(uint8_t target, uint8_t enable, uint8_t runMode,
                                                          uint8_t speed)
{
	return GetCmdData(0x05, { target, enable, runMode, speed });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::DeviceStop(uint8_t target)
{
	return GetCmdData(0x05, { target, 0x03, 0, 0 });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::DeviceStopAll()
{
	return GetCmdData(0x05, { 0x00, 0x03, 0x00, 0x00 });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::DeviceBreak()
{
	return GetCmdData(0x05, { 0x00, 0x00, 0x02, 0b11 });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::SendNumberOfPulses(uint8_t mc)
{
	return GetCmdData(0x02, { mc });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::SendDelayTime(uint8_t mc)
{
	return GetCmdData(0x01, { mc });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::StartXRay(uint8_t startMode, uint8_t startTTL, uint8_t isDelay)
{
	return GetCmdData(0x03, { startMode, startTTL, isDelay });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::StopXRay()
{
	return GetCmdData(0x04, { 0x00 });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::TurnOnAll()
{
	return GetCmdData(0x08, { 0x01 });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::TurnOffAll()
{
	return GetCmdData(0x08, { 0x00 });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::SetControlBoardConfig(CControlBoardProtocolConfig* config)
{
	return GetCmdData(0x0d, config->GetDataByte());
}

std::vector<uint8_t> CWHSDControlBoardProtocol::SetFactoryMode(bool bFactoryMode)
{
	return GetCmdData(0x09, { (uint8_t)(bFactoryMode ? 0x01 : 0x00) });
}

std::vector<uint8_t> CWHSDControlBoardProtocol::SensorCmd(uint8_t sensorIndex, uint8_t cmd, uint16_t value)
{
	return GetCmdData(0x11, { sensorIndex ,cmd ,(uint8_t)(value>>8),(uint8_t)(value & 0xff) });
}


void CWHSDControlBoardProtocol::Erase(int en)
{
	if (en < m_vectorDataBuffer.size())
	{
		m_vectorDataBuffer.erase(m_vectorDataBuffer.begin(), m_vectorDataBuffer.begin() + en);
	}
	else
	{
		m_vectorDataBuffer.clear();
	}
}

void CWHSDControlBoardProtocol::Answer()
{
	std::vector<uint8_t> needSendData;
	switch (m_cCmd)
	{
	case 0x06:
	{
		needSendData = GetCmdData(0x06, { 0x01 });
		break;
	}
	default:
	{
		return;
	}
	}
	m_function_Answer(needSendData.data(), needSendData.size());
}

std::vector<uint8_t> CWHSDControlBoardProtocol::GetCmdData(uint8_t cmd, const std::vector<uint8_t>& cmdData)
{
	if (++cPackNumber == 0)
	{
		cPackNumber = 0;
	}
	uint8_t panckLen = cmdData.size() + 8;
	std::vector<uint8_t> result(panckLen);
	result[0] = 0xff;
	result[1] = 0xfe;
	result[2] = panckLen;
	result[3] = cPackNumber;
	result[4] = cmd;
	memcpy(result.data() + 5, cmdData.data(), cmdData.size());
	uint8_t checkSum2 = 0;
	for (int i = 0; i < panckLen - 3; i++)
	{
		checkSum2 = checkSum2 + result[i];
	}
	result[panckLen - 3] = checkSum2;
	result[panckLen - 2] = 0xfd;
	result[panckLen - 1] = 0xfc;
	return result;
}

void CWHSDControlBoardProtocol::OTAThread()
{
	//m_bPauseHeartBeat = false;
	//开始进行OTA升级
	m_nOTAPackIndex = 0;
	m_cOTAStatus = 1;
	bool bIsNeedExit = false;
	m_cOTAErrorCount = 0;
	//这是分包的逻辑
	auto otaPack = WHSD_Tools::SplitVectorData(m_vector_OTAFile, OTA_PACK_LEN);
	std::vector<uint8_t> packData(138);
	m_nOTAAllPacks = otaPack.size();
	memcpy(packData.data(), &m_nOTAAllPacks, sizeof(int));
	if (m_function_OTAStatusCallBack != nullptr)
	{
		m_function_OTAStatusCallBack(m_cOTAStatus, m_nOTAAllPacks, m_nOTAPackIndex);
	}
	while (!bIsNeedExit)
	{
		if (m_cOTAStatus == 1)
		{
			//通知设备进入OTA状态
			Sleep(1000);
			auto cmd = GetCmdData(0x0a, { 0x00 });
			SendData(cmd.data(), cmd.size());
			if (m_cOTAErrorCount++ > 5)
			{
				m_cOTAStatus = 255;
				bIsNeedExit = true;
			}
			if (m_function_OTAStatusCallBack != nullptr)
			{
				m_function_OTAStatusCallBack(m_cOTAStatus, m_nOTAAllPacks, m_nOTAPackIndex);
			}
		}
		if (m_cOTAStatus == 2)
		{
			Sleep(500);
			if (m_nOTAPackIndex < m_nOTAAllPacks)
			{
				uint32_t currentPackNumber = m_nOTAPackIndex + 1;
				memcpy(packData.data() + 4, &currentPackNumber, sizeof(uint32_t));
				packData[8] = OTA_PACK_LEN;
				memcpy(packData.data() + 9, otaPack[m_nOTAPackIndex].data(), OTA_PACK_LEN);
				uint8_t checksum = 0;
				for (int i = 0; i < OTA_PACK_LEN + 9; ++i)
				{
					checksum = checksum + packData[i];
				}
				packData[packData.size() - 1] = checksum;
				auto cmd = GetCmdData(0x0b, packData);
				SendData(cmd.data(), cmd.size());
				if (m_cOTAErrorCount++ > 5)
				{
					m_cOTAStatus = 255;
					bIsNeedExit = true;
				}
			}
			else
			{
				m_cOTAStatus = 3;
			}

			if (m_function_OTAStatusCallBack != nullptr)
			{
				m_function_OTAStatusCallBack(m_cOTAStatus, m_nOTAAllPacks, m_nOTAPackIndex);
			}
		}
		if (m_cOTAStatus == 3)
		{
			bIsNeedExit = true;
		}
	}
	auto cmd = GetCmdData(0x0c, { 0x00 });
	SendData(cmd.data(), cmd.size());
	if (m_function_OTAStatusCallBack != nullptr)
	{
		m_function_OTAStatusCallBack(m_cOTAStatus, m_nOTAAllPacks, m_nOTAPackIndex);
	}
	//m_bPauseHeartBeat = false;
}

void CWHSDControlBoardProtocol::SendData(uint8_t* p, int len)
{
	if (m_function_Answer != nullptr)
	{
		m_function_Answer(p, len);
	}
}

void CWHSDControlBoardProtocol::LoopSendHeartBeat()
{
	//return;
	while (!m_nIsNeedExit)
	{
		Sleep(m_wHeartBeatTime);
		if (!m_bPauseHeartBeat)
		{
			auto needSendData = GetCmdData(0x00, { m_cPackNumber });
			SendData(needSendData.data(), needSendData.size());
		}
	}
}

void CWHSDControlBoardProtocol::DealHeartBeat()
{
	if (m_vectorCmdData.size() == DEVICE_HEART_BEAT_LEN && m_function_DeviceHeartBeat != nullptr)
	{
		for (int i = 0; i < 4; ++i)
		{
			m_memDeviceHeartBeat.ExtractMotorStatus(i, m_vectorCmdData.data() + i * 4);
		}
		m_memDeviceHeartBeat.m_cBattery = m_vectorCmdData[16];
		m_memDeviceHeartBeat.m_cHardwareYear = m_vectorCmdData[17];
		m_memDeviceHeartBeat.m_cHardwareMonth = m_vectorCmdData[18];
		m_memDeviceHeartBeat.m_cHardwareDay = m_vectorCmdData[19];
		m_memDeviceHeartBeat.m_cHardwareVersionOfDay = m_vectorCmdData[20];
		m_memDeviceHeartBeat.m_cXRayDeviceStatus = m_vectorCmdData[21];
		m_memDeviceHeartBeat.m_cMainPowerSupply = m_vectorCmdData[22];
		m_memDeviceHeartBeat.m_bFactoryMode = TOOLS_GET_BIT(m_vectorCmdData[23], 0) > 0;
		m_function_DeviceHeartBeat(m_memDeviceHeartBeat);
	}
}