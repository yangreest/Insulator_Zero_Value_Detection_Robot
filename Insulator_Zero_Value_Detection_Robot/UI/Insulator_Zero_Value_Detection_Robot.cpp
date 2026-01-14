#include "Insulator_Zero_Value_Detection_Robot.h"

#include "Config/ConfigManager.h"
#include "Log/ScanS_WriteLog.h"
#include "Protocol/WHSDControlBoradProtocol.h"
#include "Tools/Tools.h"
#include <QTimer>

Insulator_Zero_Value_Detection_Robot::Insulator_Zero_Value_Detection_Robot(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	InitParam();
	InitUI();
	BindAction();
}

Insulator_Zero_Value_Detection_Robot::~Insulator_Zero_Value_Detection_Robot()
{
}

void Insulator_Zero_Value_Detection_Robot::InitUI()
{
	showMaximized();
}

void Insulator_Zero_Value_Detection_Robot::InitParam()
{
	m_wSensorStatus = 0;

	m_wSensorBat = 0;

	m_wSensorResult = 0;
	m_pDeviceLog = new CWriteLog(WHSD_Tools::GetAbsolutePath("Log\\DeviceLog.txt"), 10000, 250);
	m_pDeviceLog->BeginWork();
	m_pConfig = new CConfigManager();
	m_pConfig->Read(WHSD_Tools::GetAbsolutePath("Config.xml"));
	m_pXInputHelper = new CXInputHelper(0);
	m_pXInputHelper->RegisterControllerStateCallBack(std::bind(
		&Insulator_Zero_Value_Detection_Robot::CallBack_ControllerState, this, std::placeholders::_1,
		std::placeholders::_2));
	m_pXInputHelper->BeginWork();

	//鍏堝垵濮嬪寲涓绘帶鍒舵澘

	m_pComDevice = IDeviceCom::GetIDeviceCom(1);
	m_pWHSDControlBoardProtocol = new CWHSDControlBoardProtocol(
		m_pConfig->m_memControlBoardConfig.m_wDeviceHeartBeat);

	auto pWHSDControlBoardProtocol = m_pWHSDControlBoardProtocol;
	auto pDeviceCom = m_pComDevice;
	pDeviceCom->SetParam(m_pConfig->m_memControlBoardConfig.m_strIp.c_str(),
	                     m_pConfig->m_memControlBoardConfig.m_wPort);
	pDeviceCom->RegisterReadDataCallBack(std::bind(&CWHSDControlBoardProtocol::ReceiveNewData,
	                                               pWHSDControlBoardProtocol, std::placeholders::_1,
	                                               std::placeholders::_2));
	pDeviceCom->RegisterConnectStatusCallBack(std::bind(
		&Insulator_Zero_Value_Detection_Robot::ComDeviceConnectionChanged, this,
		std::placeholders::_1, std::placeholders::_2, 0));

	pWHSDControlBoardProtocol->RegisterAnswerFunction(
		std::bind(&IDeviceCom::Write, pDeviceCom, std::placeholders::_1, std::placeholders::_2));
	pWHSDControlBoardProtocol->RegisterDeviceLog(std::bind(&CWriteLog::Write, m_pDeviceLog, std::placeholders::_1));
	pWHSDControlBoardProtocol->RegisterDeviceHeartBeat(
		std::bind(&Insulator_Zero_Value_Detection_Robot::Callback_DeviceHeartBeat, this, std::placeholders::_1, 0));
	pWHSDControlBoardProtocol->RegisterSensorDataCallBack(
		std::bind(&Insulator_Zero_Value_Detection_Robot::CallBack_SensorValue, this, std::placeholders::_1));
	//pWHSDControlBoardProtocol->RegisterOTAStatus(std::bind(&MainForm::Callback_OTAStatus, this,
	//	std::placeholders::_1,
	//	std::placeholders::_2, std::placeholders::_3));
	//pWHSDControlBoardProtocol->RegisterXRaySendResult(xRayResult);
	pDeviceCom->BeginWork();
	pWHSDControlBoardProtocol->BeginWork();
}

void Insulator_Zero_Value_Detection_Robot::BindAction()
{
	m_pTimer = new QTimer(this);
	m_pTimer->setInterval(100);
	connect(m_pTimer, &QTimer::timeout, this, &Insulator_Zero_Value_Detection_Robot::On_timer_timeout);
	m_pTimer->start();

	connect(ui.pushButton, &QPushButton::clicked, this, &Insulator_Zero_Value_Detection_Robot::On_TurnOnAll_Click);
	connect(ui.pushButton_2, &QPushButton::clicked, this, &Insulator_Zero_Value_Detection_Robot::On_TurnOffAll_Click);
}

void Insulator_Zero_Value_Detection_Robot::CallBack_ControllerState(int t, const ControllerState* p)
{
	std::lock_guard<std::mutex> g(m_mutexXInput);
	memcpy(&m_memControllerState, p, sizeof(ControllerState));
}

void Insulator_Zero_Value_Detection_Robot::On_timer_timeout()
{
	if (m_nTimeCount++ % 20 == 0)
	{
		auto cmds = CWHSDControlBoardProtocol::SensorCmd(0, 2, 0);

		m_pComDevice->Write(cmds.data(), cmds.size());


		cmds = CWHSDControlBoardProtocol::SensorCmd(0, 3, 0);

		m_pComDevice->Write(cmds.data(), cmds.size());


		//cmds = CWHSDControlBoardProtocol::SensorCmd(0, 4, 0);

		//m_pComDevice->Write(cmds.data(), cmds.size());
	}
	ControllerState tp;
	{
		std::lock_guard<std::mutex> g(m_mutexXInput);
		memcpy(&tp, &m_memControllerState, sizeof(ControllerState));
	}


	if (!m_bLastButton && tp.buttons[5])
	{
		auto cmds = CWHSDControlBoardProtocol::SensorCmd(0, 1, 0);

		m_pComDevice->Write(cmds.data(), cmds.size());
	}
	m_bLastButton = tp.buttons[5];

	ui.label_20->setText(m_bLastButton ? "ON" : "OFF");
	if (m_nLastDir == 0)
	{
		switch (tp.dpad)
		{
		case 1:
			{
				auto cmds = CWHSDControlBoardProtocol::DeviceRun(0x05, 0b11, 0x01,
				                                                 m_pConfig->m_memControlBoardConfig.m_cUpAngle);
				m_pComDevice->Write(cmds.data(), cmds.size());
				ui.label_2->setText("上");
				break;
			}
		case 2:
			{
				auto cmds = CWHSDControlBoardProtocol::DeviceRun(0x01, 0b11, 0x01,
				                                                 m_pConfig->m_memControlBoardConfig.m_cWalkMotorSpeed);
				m_pComDevice->Write(cmds.data(), cmds.size());
				ui.label_2->setText("右");
				break;
			}
		case 3:
			{
				auto cmds = CWHSDControlBoardProtocol::DeviceRun(0x05, 0b11, 0x01,
				                                                 m_pConfig->m_memControlBoardConfig.m_cDownAngle);
				m_pComDevice->Write(cmds.data(), cmds.size());
				ui.label_2->setText("下");
				break;
			}
		case 4:
			{
				auto cmds = CWHSDControlBoardProtocol::DeviceRun(0x01, 0b11, 0x02,
				                                                 m_pConfig->m_memControlBoardConfig.m_cWalkMotorSpeed);
				m_pComDevice->Write(cmds.data(), cmds.size());
				ui.label_2->setText("左");
				break;
			}
		case 0:
		default:
			{
				ui.label_2->setText("");
				break;
			}
		}
	}
	else if (m_nLastDir == 2 || m_nLastDir == 4)
	{
		if (tp.dpad == 0)
		{
			auto cmds = CWHSDControlBoardProtocol::DeviceStop(0x01);
			m_pComDevice->Write(cmds.data(), cmds.size());
		}
	}
	m_nLastDir = tp.dpad;
	ui.label_34->setText(QString::number(m_nHeartBeatCount));
	ui.label_3->setText(m_bControlBroadConnected ? "已连接" : "未连接");
	const auto memDeviceHeartBeat = m_memDeviceHeartBeat;
	std::string strWalkingMotorStatus("未知");
	switch (memDeviceHeartBeat.m_vectorWalkingMotorStatus.front().m_cDeviceStatus)
	{
	case 0:
	case 3:
		{
			strWalkingMotorStatus = "停止";
			break;
		}
	case 1:
		{
			strWalkingMotorStatus = "右运行";
			break;
		}
	case 2:
		{
			strWalkingMotorStatus = "左运行";
			break;
		}
	case 4:
		{
			strWalkingMotorStatus = "故障";
			break;
		}
	default:
		{
			break;
		}
	}
	ui.label->setText(QString::fromStdString(strWalkingMotorStatus));
	strWalkingMotorStatus = "未知";
	switch (memDeviceHeartBeat.m_vectorSafetyMotorStatus[0].m_cDeviceStatus)
	{
	case 0:
	case 3:
		{
			strWalkingMotorStatus = "停止";
			break;
		}
	case 1:
	case 2:
		{
			strWalkingMotorStatus = "运行中";
			break;
		}
	case 4:
		{
			strWalkingMotorStatus = "故障";
			break;
		}
	default:
		{
			break;
		}
	}

	ui.label_6->setText(QString::fromStdString(strWalkingMotorStatus));
	ui.label_8->setText(memDeviceHeartBeat.m_cMainPowerSupply > 0 ? "开" : "关");
	if (memDeviceHeartBeat.m_cBattery > 100)
	{
		ui.label_13->setText("未知");
	}
	else
	{
		ui.label_13->setText(QString::number(memDeviceHeartBeat.m_cBattery));
	}

	if (m_wSensorBat > 100)
	{
		ui.label_15->setText("未知");
	}
	else
	{
		ui.label_15->setText(QString::number(m_wSensorBat));
	}
	switch (m_wSensorStatus)
	{
	case 0:
		{
			ui.label_17->setText("待机");
			break;
		}
	case 1:
		{
			ui.label_17->setText("触发");
			break;
		}
	case 2:
		{
			ui.label_17->setText("触发完成");
			break;
		}
	case 3:
		{
			ui.label_17->setText("未找到设备");
			break;
		}
	default:
		{
			break;
		}
	}
	switch (m_wSensorResult)
	{
	case 0:
		{
			ui.label_11->setText("零值");
			break;
		}
	case 1:
		{
			ui.label_11->setText("正常");
			break;
		}
	case 2:
		{
			ui.label_11->setText("未知");
			break;
		}
	case 3:
		{
			ui.label_11->setText("未找到设备");
			break;
		}
	default:
		{
			break;
		}
	}
}


void Insulator_Zero_Value_Detection_Robot::ComDeviceConnectionChanged(const bool connected, int guid, int index)
{
	m_bControlBroadConnected = connected;
}

void Insulator_Zero_Value_Detection_Robot::RefreshControllerState(const ControllerState* p)
{
	//鐩存帴灏嗗€兼嫹璐濊繃鏉?
	std::lock_guard<std::mutex> g(m_mutexXInput);
	memcpy(&m_memControllerState, p, sizeof(ControllerState));
}

void Insulator_Zero_Value_Detection_Robot::CallBack_SensorValue(CSensorData* p)
{
	switch (p->m_cCmd)
	{
	case 2:
		{
			m_wSensorStatus = p->m_wValue;
			break;
		}
	case 3:
		{
			m_wSensorBat = p->m_wValue;
			break;
		}
	case 4:
		{
			m_wSensorResult = p->m_wValue;
			break;
		}
	default:
		{
			break;
		}
	}
}

void Insulator_Zero_Value_Detection_Robot::Callback_DeviceHeartBeat(const CDeviceHeartBeat& b, int nComdeviceIndex)
{
	m_mutexDeviceInfoLock.lock();
	m_memDeviceHeartBeat = b;
	m_time_LastHeartBeatTime.GetCurTime();
	m_nHeartBeatCount++;
	m_mutexDeviceInfoLock.unlock();
}


void Insulator_Zero_Value_Detection_Robot::On_TurnOnAll_Click()
{
	auto cmds = CWHSDControlBoardProtocol::TurnOnAll();
	m_pComDevice->Write(cmds.data(), cmds.size());
}

void Insulator_Zero_Value_Detection_Robot::On_TurnOffAll_Click()
{
	auto cmds = CWHSDControlBoardProtocol::TurnOffAll();
	m_pComDevice->Write(cmds.data(), cmds.size());
}
