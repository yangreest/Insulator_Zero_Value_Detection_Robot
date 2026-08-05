#pragma once
#include <mutex>
#include <QtWidgets/QMainWindow>
#include "ui_Insulator_Zero_Value_Detection_Robot.h"
#include "Config/ConfigManager.h"
#include "DeviceCom/IDeviceCom.h"
#include "Log/ScanS_FC.h"
#include "Log/ScanS_WriteLog.h"
#include "Protocol/WHSDControlBoradProtocol.h"
#include "Tools/XInputHelper.h"
#include "Camera/CameraBase.h"
#include "Config/XmlManagerWindow.h"
#include <opencv2/opencv.hpp>

class Insulator_Zero_Value_Detection_Robot : public QMainWindow
{
	Q_OBJECT

public:
	Insulator_Zero_Value_Detection_Robot(QWidget* parent = nullptr);
	~Insulator_Zero_Value_Detection_Robot();
	// 在头文件中声明函数
	static QImage Mat2QImage(const cv::Mat& mat);

private slots:
	void On_timer_timeout();
	void On_timerInput_timeout();
	void On_TurnOnAll_Click();
	void On_TurnOffAll_Click();
	void On_ZeroTest_Click();
	void On_Setting_Click();
	void captureCurrentWindow();
	void On_SetFileName_Click();

private:
	Ui::Insulator_Zero_Value_Detection_RobotClass ui;

	void Callback_DeviceHeartBeat(const CDeviceHeartBeat& b, int nComdeviceIndex);

	void InitUI();

	void InitParam();

	void BindAction();

	void ComDeviceConnectionChanged(const bool connected, int guid, int index);

	void CallBack_ControllerState(int t, const ControllerState* p);

	void RefreshControllerState(const ControllerState* p);

	void CallBack_SensorValue(CSensorData* p);

	void savePixmap(const QPixmap& pixmap);

	ControllerState m_memControllerState;

	CXInputHelper* m_pXInputHelper;

	IDeviceCom* m_pComDevice;

	CConfigManager* m_pConfig;

	CWriteLog* m_pDeviceLog;

	CWHSDControlBoardProtocol* m_pWHSDControlBoardProtocol;

	XmlManagerWindow* xmlManagerWindow;

	std::mutex m_mutexDeviceInfoLock;

	CDeviceHeartBeat m_memDeviceHeartBeat;

	CCFRD_Time m_time_LastHeartBeatTime;

	uint64_t m_nHeartBeatCount;

	bool m_bControlBroadConnected;

	std::mutex m_mutexXInput;

	QTimer* m_pTimer;
	QTimer* m_pTimerInput;

	int m_nLastDir;

	uint16_t m_wSensorStatus;

	uint16_t m_wSensorBat;

	uint16_t m_wSensorResult;

	uint64_t m_nTimeCount;

	bool m_bLastButton;

	QString	m_strFileName;

	bool continueStreaming;

public:
	// 摄像头
	void CameraConnect();
	void NewCameraConnect();
	ICameraBase* m_pC1;
	ICameraBase* m_pC2;

	std::string m_strLeftIp;
	std::string m_strRightIp;
};
