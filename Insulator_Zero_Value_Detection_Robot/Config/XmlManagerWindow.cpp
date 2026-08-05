#include "XmlManagerWindow.h"
#include <QSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QRegularExpression>
#include <QCheckBox>
#include <Tools/Tools.h>

XmlManagerWindow::XmlManagerWindow(QWidget* parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	// 初始化UI
	initUI();
	BindAction();
}

XmlManagerWindow::~XmlManagerWindow()
{
	// 清理资源
}

void XmlManagerWindow::initUI()
{
	ui.groupBox->setVisible(true);
	ui.groupBox_4->setVisible(false);
	// 1. 设置输入校验器：仅允许数字和点
	QRegularExpression regExp("^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){0,3}"
		"(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)?$");
	QRegularExpressionValidator* validator = new QRegularExpressionValidator(regExp, this);
	// lineEdit 的格式按照 IP 地址格式
	ui.lineEdit->setValidator(validator);
	ui.lineEdit_5->setValidator(validator);
	ui.lineEdit_6->setValidator(validator);
	ui.lineEdit_7->setValidator(validator);

	CConfigManager* m_pConfig = new CConfigManager();
	m_pConfig->Read(WHSD_Tools::GetAbsolutePath("Config.xml"));

    ui.lineEdit->setText(m_pConfig->m_memControlBoardConfig.m_strIp.c_str());
    ui.lineEdit_2->setText(QString::number(m_pConfig->m_memControlBoardConfig.m_wPort));
	ui.spinBox->setValue(m_pConfig->m_memControlBoardConfig.m_wDeviceHeartBeat);
	ui.checkBox->setChecked(m_pConfig->m_memControlBoardConfig.m_bFactoryMode);
	ui.lineEdit_3->setText(QString::number(m_pConfig->m_memControlBoardConfig.m_cUpAngle));
    ui.lineEdit_4->setText(QString::number(m_pConfig->m_memControlBoardConfig.m_cDownAngle));
	ui.comboBox->setCurrentIndex(m_pConfig->m_memControlBoardConfig.m_cWalkMotorSpeed);
	ui.lineEdit_10->setText(QString::number(m_pConfig->m_memControlBoardConfig.m_cUpAngle2));

	ui.lineEdit_5->setText(m_pConfig->m_memCCameraConfig.m_strLeftIp.c_str());
	ui.lineEdit_6->setText(m_pConfig->m_memCCameraConfig.m_strMidIp.c_str());
	ui.lineEdit_7->setText(m_pConfig->m_memCCameraConfig.m_strRightIp.c_str());

	ui.radioButton_2->setChecked(m_pConfig->m_memCCameraConfig.m_bNewCamera);
	ui.checkBox_2->setChecked(m_pConfig->m_memCCameraConfig.m_bUseMainSp);
	ui.lineEdit_8->setText(m_pConfig->m_memCCameraConfig.m_strMainRtsp.c_str());
    ui.lineEdit_9->setText(m_pConfig->m_memCCameraConfig.m_strSubRtsp.c_str());
}

void XmlManagerWindow::BindAction()
{ 
	connect(ui.listWidget, &QListWidget::currentItemChanged, this, &XmlManagerWindow::onParameterListCurrentItemChanged);
	connect(ui.pushButton_2, &QPushButton::clicked, this, &XmlManagerWindow::onSaveButtonClicked);
	connect(ui.cancelBtn, &QPushButton::clicked, this, &XmlManagerWindow::onCancelButtonClicked);
    connect(ui.pushButton, &QPushButton::clicked, this, &XmlManagerWindow::onApplyButtonClicked);
}

void XmlManagerWindow::onSaveButtonClicked()
{
	// 保存并退出
	CConfigManager* m_pConfig = new CConfigManager();

	m_pConfig->m_memControlBoardConfig.m_strIp = ui.lineEdit->text().toStdString();
	m_pConfig->m_memControlBoardConfig.m_wPort = ui.lineEdit_2->text().toInt();
	m_pConfig->m_memControlBoardConfig.m_wDeviceHeartBeat = ui.spinBox->value();
	m_pConfig->m_memControlBoardConfig.m_bFactoryMode = ui.checkBox->isChecked();
	m_pConfig->m_memControlBoardConfig.m_cUpAngle = ui.lineEdit_3->text().toInt();
    m_pConfig->m_memControlBoardConfig.m_cDownAngle = ui.lineEdit_4->text().toInt();
	m_pConfig->m_memControlBoardConfig.m_cWalkMotorSpeed = ui.comboBox->currentIndex();
	m_pConfig->m_memControlBoardConfig.m_cUpAngle2 = ui.lineEdit_10->text().toInt();

	m_pConfig->m_memCCameraConfig.m_strLeftIp = ui.lineEdit_5->text().toStdString();
    m_pConfig->m_memCCameraConfig.m_strMidIp = ui.lineEdit_6->text().toStdString();
    m_pConfig->m_memCCameraConfig.m_strRightIp = ui.lineEdit_7->text().toStdString();

	m_pConfig->m_memCCameraConfig.m_bNewCamera = ui.radioButton_2->isChecked();
    m_pConfig->m_memCCameraConfig.m_bUseMainSp = ui.checkBox_2->isChecked();
    m_pConfig->m_memCCameraConfig.m_strMainRtsp = ui.lineEdit_8->text().toStdString();
    m_pConfig->m_memCCameraConfig.m_strSubRtsp = ui.lineEdit_9->text().toStdString();
	m_pConfig->Write(WHSD_Tools::GetAbsolutePath("Config.xml"));

	// 退出
    this->close();
}

void XmlManagerWindow::onCancelButtonClicked()
{
	CConfigManager* m_pConfig = new CConfigManager();
	m_pConfig->Read(WHSD_Tools::GetAbsolutePath("Config.xml"));

	ui.lineEdit->setText(m_pConfig->m_memControlBoardConfig.m_strIp.c_str());
	ui.lineEdit_2->setText(QString::number(m_pConfig->m_memControlBoardConfig.m_wPort));
	ui.spinBox->setValue(m_pConfig->m_memControlBoardConfig.m_wDeviceHeartBeat);
	ui.checkBox->setChecked(m_pConfig->m_memControlBoardConfig.m_bFactoryMode);
	ui.lineEdit_3->setText(QString::number(m_pConfig->m_memControlBoardConfig.m_cUpAngle));
	ui.lineEdit_4->setText(QString::number(m_pConfig->m_memControlBoardConfig.m_cDownAngle));
	ui.comboBox->setCurrentIndex(m_pConfig->m_memControlBoardConfig.m_cWalkMotorSpeed);

	ui.lineEdit_5->setText(m_pConfig->m_memCCameraConfig.m_strLeftIp.c_str());
	ui.lineEdit_6->setText(m_pConfig->m_memCCameraConfig.m_strMidIp.c_str());
	ui.lineEdit_7->setText(m_pConfig->m_memCCameraConfig.m_strRightIp.c_str());

	ui.radioButton_2->setChecked(m_pConfig->m_memCCameraConfig.m_bNewCamera);
	ui.checkBox_2->setChecked(m_pConfig->m_memCCameraConfig.m_bUseMainSp);
	ui.lineEdit_8->setText(m_pConfig->m_memCCameraConfig.m_strMainRtsp.c_str());
	ui.lineEdit_9->setText(m_pConfig->m_memCCameraConfig.m_strSubRtsp.c_str());
	// 退出
	this->close();
}

void XmlManagerWindow::onApplyButtonClicked()
{
	// 保存并退出
	CConfigManager* m_pConfig = new CConfigManager();

	m_pConfig->m_memControlBoardConfig.m_strIp = ui.lineEdit->text().toStdString();
	m_pConfig->m_memControlBoardConfig.m_wPort = ui.lineEdit_2->text().toInt();
	m_pConfig->m_memControlBoardConfig.m_wDeviceHeartBeat = ui.spinBox->value();
	m_pConfig->m_memControlBoardConfig.m_bFactoryMode = ui.checkBox->isChecked();
	m_pConfig->m_memControlBoardConfig.m_cUpAngle = ui.lineEdit_3->text().toInt();
	m_pConfig->m_memControlBoardConfig.m_cDownAngle = ui.lineEdit_4->text().toInt();
	m_pConfig->m_memControlBoardConfig.m_cWalkMotorSpeed = ui.comboBox->currentIndex();

	m_pConfig->m_memCCameraConfig.m_strLeftIp = ui.lineEdit_5->text().toStdString();
	m_pConfig->m_memCCameraConfig.m_strMidIp = ui.lineEdit_6->text().toStdString();
	m_pConfig->m_memCCameraConfig.m_strRightIp = ui.lineEdit_7->text().toStdString();

	m_pConfig->m_memCCameraConfig.m_bNewCamera = ui.radioButton_2->isChecked();
	m_pConfig->m_memCCameraConfig.m_bUseMainSp = ui.checkBox_2->isChecked();
	m_pConfig->m_memCCameraConfig.m_strMainRtsp = ui.lineEdit_8->text().toStdString();
	m_pConfig->m_memCCameraConfig.m_strSubRtsp = ui.lineEdit_9->text().toStdString();
	m_pConfig->Write(WHSD_Tools::GetAbsolutePath("Config.xml"));
}

void XmlManagerWindow::onParameterListCurrentItemChanged(QListWidgetItem* current, QListWidgetItem* previous)
{
	int index = ui.listWidget->currentIndex().row();

	switch (index)
	{
	case 0:
		ui.groupBox->setVisible(true);
		ui.groupBox_4->setVisible(false);
		break;
	case 1:
		ui.groupBox->setVisible(false);
		ui.groupBox_4->setVisible(true);
		break;
	default:
		break;
	}
}