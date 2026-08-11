#include "Insulator_Zero_Value_Detection_Robot.h"
#include "Config/ConfigManager.h"

#include "Log/ScanS_WriteLog.h"
#include "Protocol/WHSDControlBoradProtocol.h"
#include "Tools/Tools.h"
#include <QTimer>
#include <QScreen>
#include <QApplication>
#include <QMessageBox.h>
#include <QDateTime>
#include <QFileDialog.h>

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
	continueStreaming = false;
}

void Insulator_Zero_Value_Detection_Robot::InitUI()
{
	showMaximized();

	ui.label_22->setVisible(false);
	ui.label_10->setVisible(false);
	ui.label_11->setVisible(false);
	//ui.pushButton_4->setVisible(false);
	ui.pushButton_3->setVisible(false);
}

void Insulator_Zero_Value_Detection_Robot::InitParam()
{
	m_wSensorStatus = 0;

	m_wSensorBat = 0;

	m_strFileName = "C:";

	// 使用标志位控制循环
	continueStreaming = true;

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

	// 获取设备信息

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

	//if (m_pConfig->m_memCCameraConfig.m_bNewCamera)
	//{
	//	std::thread td(&Insulator_Zero_Value_Detection_Robot::NewCameraConnect, this);
	//	td.detach();
	//}
	//else
	{
		m_strLeftIp = m_pConfig->m_memCCameraConfig.m_strLeftIp;
		m_strRightIp = m_pConfig->m_memCCameraConfig.m_strRightIp;

		m_pC1 = ICameraBase::GetCameraObj(0);
		m_pC2 = ICameraBase::GetCameraObj(0);

		auto handleR = reinterpret_cast<HWND>(ui.label_22->winId());// winId的功能是获取窗口句柄
		auto handleL = reinterpret_cast<HWND>(ui.label_9->winId());

		m_pC1->RegisterVideoViewHandle(handleL);
		m_pC2->RegisterVideoViewHandle(handleR);

		std::thread td(&Insulator_Zero_Value_Detection_Robot::CameraConnect, this);
		td.detach();
	}

}

void Insulator_Zero_Value_Detection_Robot::BindAction()
{
	m_pTimer = new QTimer(this);
	m_pTimer->setInterval(100);
	connect(m_pTimer, &QTimer::timeout, this, &Insulator_Zero_Value_Detection_Robot::On_timer_timeout);
	m_pTimer->start();

	m_pTimerInput = new QTimer(this);
	m_pTimerInput->setInterval(10);
	connect(m_pTimerInput, &QTimer::timeout, this, &Insulator_Zero_Value_Detection_Robot::On_timerInput_timeout);
	m_pTimerInput->start();

	connect(ui.pushButton, &QPushButton::clicked, this, &Insulator_Zero_Value_Detection_Robot::On_TurnOnAll_Click);
	connect(ui.pushButton_2, &QPushButton::clicked, this, &Insulator_Zero_Value_Detection_Robot::On_TurnOffAll_Click);
	connect(ui.pushButton_3, &QPushButton::clicked, this, &Insulator_Zero_Value_Detection_Robot::On_ZeroTest_Click);// 零值检测
	connect(ui.pushButton_4, &QPushButton::clicked, this, &Insulator_Zero_Value_Detection_Robot::On_Setting_Click);
	connect(ui.pushButton_PS, &QPushButton::clicked, this, &Insulator_Zero_Value_Detection_Robot::captureCurrentWindow);
	connect(ui.pushButton_SN, &QPushButton::clicked, this, &Insulator_Zero_Value_Detection_Robot::On_SetFileName_Click);
	connect(ui.pushButton_5, &QPushButton::clicked, this, &Insulator_Zero_Value_Detection_Robot::On_ReFreshCamera_Click);
}

void Insulator_Zero_Value_Detection_Robot::CallBack_ControllerState(int t, const ControllerState* p)
{
	std::lock_guard<std::mutex> g(m_mutexXInput);
	memcpy(&m_memControllerState, p, sizeof(ControllerState));
}

void Insulator_Zero_Value_Detection_Robot::On_timer_timeout()
{
	if (m_nTimeCount++ % 10 == 0)
	{
		auto cmds = CWHSDControlBoardProtocol::SensorCmd(0, 2, 0);

		m_pComDevice->Write(cmds.data(), cmds.size());


		cmds = CWHSDControlBoardProtocol::SensorCmd(0, 3, 0);

		m_pComDevice->Write(cmds.data(), cmds.size());


		cmds = CWHSDControlBoardProtocol::SensorCmd(0, 4, 0);

		m_pComDevice->Write(cmds.data(), cmds.size());
	}

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

void Insulator_Zero_Value_Detection_Robot::On_timerInput_timeout()
{
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
		case 1: // 探针向内
		{
			auto cmds = CWHSDControlBoardProtocol::DeviceRun(0x05, 0b11, 0x01,
				m_pConfig->m_memControlBoardConfig.m_cUpAngle);
			m_pComDevice->Write(cmds.data(), cmds.size());
			ui.label_2->setText("内上");
			break;
		}
		case 2:
		{
			auto cmds = CWHSDControlBoardProtocol::DeviceRun(0x01, 0b11, 0x01,
				ui.checkBox->isChecked() ? 0 : m_pConfig->m_memControlBoardConfig.m_cWalkMotorSpeed);
			m_pComDevice->Write(cmds.data(), cmds.size());
			ui.label_2->setText("右");
			break;
		}
		case 3: // 探针方平
		{
			auto cmds = CWHSDControlBoardProtocol::DeviceRun(0x05, 0b11, 0x01,
				m_pConfig->m_memControlBoardConfig.m_cDownAngle);
			m_pComDevice->Write(cmds.data(), cmds.size());
			ui.label_2->setText("外下");
			break;
		}
		case 4:
		{
			auto cmds = CWHSDControlBoardProtocol::DeviceRun(0x01, 0b11, 0x02,
				ui.checkBox->isChecked() ? 0 : m_pConfig->m_memControlBoardConfig.m_cWalkMotorSpeed);
			m_pComDevice->Write(cmds.data(), cmds.size());
			ui.label_2->setText("左");
			break;
		}
		case 5: // 探针向外
		{
            auto cmds = CWHSDControlBoardProtocol::DeviceRun(0x05, 0b11, 0x01,
				m_pConfig->m_memControlBoardConfig.m_cUpAngle2);
            m_pComDevice->Write(cmds.data(), cmds.size());
            ui.label_2->setText("外上");
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
}

void Insulator_Zero_Value_Detection_Robot::ComDeviceConnectionChanged(const bool connected, int guid, int index)
{
	m_bControlBroadConnected = connected;
}

void Insulator_Zero_Value_Detection_Robot::RefreshControllerState(const ControllerState* p)
{
	// 刷新控制器状态
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

void Insulator_Zero_Value_Detection_Robot::CameraConnect()
{
	initStatus = 0;
	bool needBreak = false;
	while (true)
	{
		if (needBreak)
		{
			break;
		}
		switch (initStatus)
		{
		case 0:
		{
			ui.label_9->setText("初始化中..");
			if (m_pC1->Init({}))
			{
				ui.label_9->setText("初始化完成");
				initStatus = 1;
			}
			break;
		}
		case 1:
		{
			ui.label_9->setText("连接中..");
			if (m_pC1->Connect(m_strLeftIp, 0, "", ""))
			{
				ui.label_9->setText("连接成功");
				initStatus = 2;
			}
			break;
		}
		case 3:
		{
			ui.label_9->setText("断开中..");
			if(m_pC1->Deinit())
			{
				ui.label_9->setText("断开成功");
				initStatus = 0;
			}
			break;
		}
		// case 2:
		// {
		// 	if (m_pC2->Connect(m_strRightIp, 0, "", ""))
		// 	{
		// 		initStatus = 3;
		// 	}
		// 	break;
		// }
		//case 3:
		//{
		//	if (m_pC3->Connect(m_strRightIp, 0, "", ""))
		//	{
		//		initStatus = 4;
		//	}
		//	break;
		//}
		default:
		{
			break;
		}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
}

void Insulator_Zero_Value_Detection_Robot::NewCameraConnect()
{
	// 使用项目配置管理器获取RTSP参数

	std::string rtsp_url;
	if (m_pConfig->m_memCCameraConfig.m_bUseMainSp)
	{
		rtsp_url = m_pConfig->m_memCCameraConfig.m_strMainRtsp;
	}
	else
	{
		rtsp_url = m_pConfig->m_memCCameraConfig.m_strSubRtsp;
	}

	cv::VideoCapture cap;

	// 设置低延迟参数
	cap.set(cv::CAP_PROP_BUFFERSIZE, 1);  // 最小化缓冲区

	// 尝试打开RTSP流
	bool isOpen = cap.open(rtsp_url, cv::CAP_FFMPEG);

	if (!isOpen)
	{
		qDebug() << "Failed to open RTSP stream: " << rtsp_url;
		return; // 不应该返回-1，因为这不是main函数
	}

	cv::Mat frame;


	while (continueStreaming)
	{
		// 低延迟处理：获取最新帧
		if (cap.grab())
		{
			cap.retrieve(frame);

			if (!frame.empty())
			{
				//cv::imshow("RTSP Low Delay", frame);
				// 将frame 转成QImage显示在lable上
                QImage qImg = Mat2QImage(frame);
                ui.label->setPixmap(QPixmap::fromImage(qImg));

				//// 检查退出条件
				//int key = cv::waitKey(1) & 0xFF;
				//if (key == 27) // ESC键退出
				//{
				//	break;
				//}
			}
			else
			{
				qDebug() << "Frame is empty";
				break;
			}
		}
		else
		{
			qDebug() << "Failed to grab frame";
			break;
		}
	}
	// 清理资源
	cap.release();
}



// 函数实现
QImage Insulator_Zero_Value_Detection_Robot::Mat2QImage(const cv::Mat& mat)
{
    // 处理空矩阵
    if (mat.empty()) {
        return QImage();
    }

    // 如果是彩色图像（BGR -> RGB）
    if (mat.channels() == 3) {
        cv::Mat rgb;
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
        return QImage((const unsigned char*)rgb.data, 
                     rgb.cols, 
                     rgb.rows, 
                     rgb.step, 
                     QImage::Format_RGB888);
    }
    // 如果是灰度图像
    else if (mat.channels() == 1) {
        return QImage((const unsigned char*)mat.data,
                     mat.cols,
                     mat.rows,
                     mat.step,
                     QImage::Format_Grayscale8); // Qt 5.13+ 支持，更早版本可用 Format_Indexed8
    }
    // 如果是RGBA图像
    else if (mat.channels() == 4) {
        cv::Mat rgba;
        cv::cvtColor(mat, rgba, cv::COLOR_BGRA2RGBA);
        return QImage((const unsigned char*)rgba.data,
                     rgba.cols,
                     rgba.rows,
                     rgba.step,
                     QImage::Format_RGBA8888);
    }
    
    // 对于其他通道数，先转换为RGB
    cv::Mat rgb;
    if (mat.channels() == 3) {
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    } else {
        cv::cvtColor(mat, rgb, cv::COLOR_GRAY2RGB);
    }
    
    return QImage((const unsigned char*)rgb.data,
                 rgb.cols,
                 rgb.rows,
                 rgb.step,
                 QImage::Format_RGB888);
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

void Insulator_Zero_Value_Detection_Robot::On_ZeroTest_Click()
{
	auto cmds = CWHSDControlBoardProtocol::SensorCmd(0, 1, 0);

	m_pComDevice->Write(cmds.data(), cmds.size());
}

void Insulator_Zero_Value_Detection_Robot::On_Setting_Click()
{
	if (xmlManagerWindow == nullptr)
	{
		xmlManagerWindow = new XmlManagerWindow();
	}
	xmlManagerWindow->show();
}

void Insulator_Zero_Value_Detection_Robot::captureCurrentWindow()
{
	// 获取当前窗口的句柄（Windows）/ID（Linux）
	WId windowId = ui.label_9->winId();

	// 获取当前窗口所在的屏幕
	QScreen* screen = QApplication::screenAt(ui.label_9->pos());
	if (!screen) {
		screen = QApplication::primaryScreen();
	}

	// 截取指定窗口
	QPixmap pixmap = screen->grabWindow(windowId);

	// 保存截图
	savePixmap(pixmap);
}

void Insulator_Zero_Value_Detection_Robot::On_SetFileName_Click()
{
	// 此处弹出对话框选择一个文件夹
	QString filePath = QFileDialog::getExistingDirectory(this, "选择文件夹");
	if (!filePath.isEmpty()) {
		m_strFileName = filePath;
		return;
	}
	QMessageBox::information(this, "提示", "文件保存失败");
}

void Insulator_Zero_Value_Detection_Robot::On_ReFreshCamera_Click()
{
	if (initStatus == 2)
	{
		initStatus = 3;
	}
}

// 保存截图到文件
void Insulator_Zero_Value_Detection_Robot::savePixmap(const QPixmap& pixmap)
{
	if (pixmap.isNull()) {
		QMessageBox::warning(this, "错误", "截图失败，像素数据为空");
		return;
	}

	//// 弹出保存文件对话框
	//QString filePath = QFileDialog::getSaveFileName(
	//	this,
	//	"保存截图",
	//	QString("截图_%1.png").arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss")),
	//	"PNG图片 (*.png);;JPG图片 (*.jpg);;BMP图片 (*.bmp)"
	//);

	QString filePath = QString("%1/%2.png").arg(m_strFileName).arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss"));

	if (!filePath.isEmpty()) {
		// 保存图片
		bool success = pixmap.save(filePath);
		if (success) {
			QMessageBox::information(this, "成功", QString("截图已保存到：\n%1").arg(filePath));
		}
		else {
			QMessageBox::warning(this, "错误", "截图保存失败");
		}
	}
}
