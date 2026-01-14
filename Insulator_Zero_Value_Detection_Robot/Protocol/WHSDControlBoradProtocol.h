#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

class CSensorData
{
public:
	CSensorData();
	uint8_t m_cSensorIndex;
	uint8_t m_cCmd;
	uint16_t m_wValue;

};

class CControlBoardProtocolConfig
{
public:

	CControlBoardProtocolConfig();

	/// <summary>
	/// 激光测距最近距离阈值
	/// </summary>
	uint16_t m_wLidarMinDis;

	/// <summary>
	/// 激光测距最远距离阈值
	/// </summary>
	uint16_t m_wLidarMaxDis;

	/// <summary>
	/// 安全轮角度判断阈值（单位度）
	/// </summary>
	float m_fSafeAngle;

	/// <summary>
	/// 成像板角度判断阈值（单位度）
	/// </summary>
	float m_fSBAngle;

	/// <summary>
	/// 行走电机异常电压阈值
	/// </summary>
	float m_fWalkingV;

	/// <summary>
	/// 安全轮电机异常电压阈值
	/// </summary>
	float m_fSafeV;

	/// <summary>
	/// 成像板电机异常电压阈值
	/// </summary>
	float m_fSBV;

	/// <summary>
	/// 卷扬电机异常电压阈值
	/// </summary>
	float m_fJYV;

	/// <summary>
	/// 备用卷扬电机异常电压阈值
	/// </summary>
	float m_fJY2V;

	/// <summary>
	/// 成像板运行时间阈值，单位毫秒
	/// </summary>
	uint32_t m_nSBRun;

	std::vector<uint8_t> GetDataByte();
};

class CMotorDeviceStatus
{
public:
	CMotorDeviceStatus();
	bool m_bDeviceEnable;
	uint8_t m_cDeviceStatus;
};

class CDeviceHeartBeat
{
public:
	CDeviceHeartBeat();

	/// <summary>
	/// 行走电机状态
	/// </summary>
	std::vector<CMotorDeviceStatus> m_vectorWalkingMotorStatus;

	/// <summary>
	/// 卷扬电机状态
	/// </summary>
	std::vector<CMotorDeviceStatus> m_vectorWindmillMotorStatus;

	/// <summary>
	/// 安全电机状态
	/// </summary>
	std::vector<CMotorDeviceStatus> m_vectorSafetyMotorStatus;

	/// <summary>
	/// 采样板电机状态
	/// </summary>
	std::vector<CMotorDeviceStatus> m_vectorSBMotorStatus;

	/// <summary>
	/// 电池百分比
	/// </summary>
	uint8_t m_cBattery;

	/// <summary>
	/// 固件发布年份，例2025以0x19表示，下同
	/// </summary>
	uint8_t m_cHardwareYear;

	/// <summary>
	/// 固件发布月份
	/// </summary>
	uint8_t m_cHardwareMonth;

	/// <summary>
	/// 固件发布日期
	/// </summary>
	uint8_t m_cHardwareDay;

	/// <summary>
	/// 固件当天编译次数
	/// </summary>
	uint8_t m_cHardwareVersionOfDay;

	/// <summary>
	/// 射线机状态(0x00:空闲    0x01：延时开启中 0x02：工作完成)
	/// </summary>
	uint8_t m_cXRayDeviceStatus;

	/// <summary>
	/// 总电源，0x00-关，0x01-开
	/// </summary>
	uint8_t m_cMainPowerSupply;

	/// <summary>
	/// 是否为
	/// </summary>
	bool m_bFactoryMode;

	void ExtractMotorStatus(int motorType, const uint8_t* data);
};

class CWHSDControlBoardProtocol
{
public:
	CWHSDControlBoardProtocol(uint16_t wHeartBeatTime);

	bool BeginWork();

	bool EndWork();

	void ReceiveNewData(const uint8_t* p, int len);

	bool Parse();

	void RegisterAnswerFunction(const std::function<bool(uint8_t*, int)>& f);

	void RegisterDeviceHeartBeat(const std::function<void(const CDeviceHeartBeat&)>& f);

	void RegisterDeviceLog(const std::function<void(const std::string&)>& f);

	void RegisterOTAStatus(const std::function<void(uint8_t, uint32_t, uint32_t)>& f);

	void BeginOTA(const std::vector<uint8_t>& file);

	void RegisterSensorDataCallBack(const std::function<void(CSensorData*)>& p);



	uint8_t m_cPackNumber;

	/// <summary>
	/// 控制电机运行
	/// </summary>
	/// <param name="target">目标电机</param>
	/// <param name="enable">电机使能</param>
	/// <param name="runMode">运行模式</param>
	/// <param name="speed">运行速度</param>
	/// <returns></returns>
	static std::vector<uint8_t> DeviceRun(uint8_t target, uint8_t enable, uint8_t runMode, uint8_t speed);

	static std::vector<uint8_t> DeviceStop(uint8_t target);

	static std::vector<uint8_t> DeviceStopAll();

	static std::vector<uint8_t> DeviceBreak();

	static std::vector<uint8_t> SendNumberOfPulses(uint8_t mc);

	static std::vector<uint8_t> SendDelayTime(uint8_t mc);

	static std::vector<uint8_t> StartXRay(uint8_t startMode, uint8_t startTTL, uint8_t isDelay);

	static std::vector<uint8_t> StopXRay();

	static std::vector<uint8_t> TurnOnAll();

	static std::vector<uint8_t> TurnOffAll();

	static std::vector<uint8_t> SetControlBoardConfig(CControlBoardProtocolConfig* config);

	static std::vector<uint8_t> SetFactoryMode(bool bFactoryMode);

	static std::vector<uint8_t> SensorCmd(uint8_t sensorIndex, uint8_t cmd, uint16_t value);

private:
	static std::vector<uint8_t> GetCmdData(uint8_t cmd, const std::vector<uint8_t>& cmdData);

	static uint8_t cPackNumber;

	void OTAThread();

	void SendData(uint8_t* p, int len);

	void LoopSendHeartBeat();

	void DealHeartBeat();

	CDeviceHeartBeat m_memDeviceHeartBeat;

	std::vector<uint8_t> m_vectorDataBuffer;

	void Erase(int en);

	std::function<bool(uint8_t*, int)> m_function_Answer;

	std::function<void(CDeviceHeartBeat)> m_function_DeviceHeartBeat;

	std::function<void(const std::string&)> m_function_WriteLogCallBack;

	std::function<void(uint8_t, uint32_t, uint32_t)> m_function_OTAStatusCallBack;

	std::function<void(CSensorData*)> m_function_SensorDataCallBack;

	std::vector<uint8_t> m_vectorCmdData;

	int m_cCmd;

	bool m_nIsNeedExit;

	void Answer();

	uint16_t m_wHeartBeatTime;

	uint8_t m_cOTAStatus;

	bool m_bPauseHeartBeat;

	std::vector<uint8_t> m_vector_OTAFile;

	uint8_t m_cOTAErrorCount;

	uint32_t m_nOTAAllPacks;

	uint32_t m_nOTAPackIndex;

	CSensorData m_memCSensorData;
};
