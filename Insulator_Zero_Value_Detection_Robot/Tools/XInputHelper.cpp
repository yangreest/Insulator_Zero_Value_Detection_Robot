#include "XInputHelper.h"

#include <cmath>
#include <thread>
#include <windows.h>
#include <XInput.h>
// 链接 XInput 库
#pragma comment(lib, "XInput.lib")

// 死区定义 (根据实际需求调整)
const float LEFT_THUMB_DEADZONE = 0.2f;
const float RIGHT_THUMB_DEADZONE = 0.2f;
const BYTE TRIGGER_DEADZONE = 30;

CXInputHelper::CXInputHelper(int index) : m_memControllerState()
{
	m_nHandleIndex = index;
	m_function_ControllerStateCallBack = nullptr;
	m_bIsNeedExit = true;
}

bool CXInputHelper::BeginWork()
{
	if (!m_bIsNeedExit)
	{
		return false;
	}
	m_bIsNeedExit = false;
	std::thread td(&CXInputHelper::LoopGetHandleValue, this);
	td.detach();
	return true;
}

void CXInputHelper::EndWork()
{
	m_bIsNeedExit = true;
}

void CXInputHelper::RegisterControllerStateCallBack(const std::function<void(int, ControllerState*)>& p)
{
	m_function_ControllerStateCallBack = p;
}

bool CXInputHelper::ReadControllerState()
{
	XINPUT_STATE xState;
	ZeroMemory(&xState, sizeof(XINPUT_STATE));

	DWORD result = XInputGetState(m_nHandleIndex, &xState);
	m_memControllerState.connected = (result == ERROR_SUCCESS);

	if (!m_memControllerState.connected)
	{
		return false;
	}
	m_memControllerState.buttons[0] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_A) != 0;
	m_memControllerState.buttons[1] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_B) != 0;
	m_memControllerState.buttons[2] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_X) != 0;
	m_memControllerState.buttons[3] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_Y) != 0;
	m_memControllerState.buttons[4] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) != 0;
	m_memControllerState.buttons[5] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0;
	m_memControllerState.buttons[6] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_BACK) != 0;
	m_memControllerState.buttons[7] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_START) != 0;
	m_memControllerState.buttons[8] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_LEFT_THUMB) != 0;
	m_memControllerState.buttons[9] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) != 0;
	m_memControllerState.buttons[10] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_UP) != 0;
	m_memControllerState.buttons[11] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT) != 0;
	m_memControllerState.buttons[12] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_DOWN) != 0;
	m_memControllerState.buttons[13] = (xState.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT) != 0;

	m_memControllerState.dpad = 0;
	if (m_memControllerState.buttons[10])
	{
		m_memControllerState.dpad = 1;
	}
	else if (m_memControllerState.buttons[11])
	{
		m_memControllerState.dpad = 2;
	}
	else if (m_memControllerState.buttons[12])
	{
		m_memControllerState.dpad = 3;
	}
	else if (m_memControllerState.buttons[13])
	{
		m_memControllerState.dpad = 4;
	}

	// 处理扳机键 (应用死区)
	m_memControllerState.leftTrigger = (xState.Gamepad.bLeftTrigger > TRIGGER_DEADZONE)
		? NormalizeTrigger(xState.Gamepad.bLeftTrigger)
		: 0.0f;

	m_memControllerState.rightTrigger = (xState.Gamepad.bRightTrigger > TRIGGER_DEADZONE)
		? NormalizeTrigger(xState.Gamepad.bRightTrigger)
		: 0.0f;

	// 处理左摇杆 (应用死区)
	float lx = NormalizeThumbstick(xState.Gamepad.sThumbLX);
	float ly = NormalizeThumbstick(xState.Gamepad.sThumbLY);
	m_memControllerState.leftThumbX = (fabs(lx) > LEFT_THUMB_DEADZONE) ? lx : 0.0f;
	m_memControllerState.leftThumbY = (fabs(ly) > LEFT_THUMB_DEADZONE) ? ly : 0.0f;

	// 处理右摇杆 (应用死区)
	float rx = NormalizeThumbstick(xState.Gamepad.sThumbRX);
	float ry = NormalizeThumbstick(xState.Gamepad.sThumbRY);
	m_memControllerState.rightThumbX = (fabs(rx) > RIGHT_THUMB_DEADZONE) ? rx : 0.0f;
	m_memControllerState.rightThumbY = (fabs(ry) > RIGHT_THUMB_DEADZONE) ? ry : 0.0f;
	return true;
}

void CXInputHelper::LoopGetHandleValue()
{
	while (!m_bIsNeedExit)
	{
		Sleep(100);
		if (ReadControllerState() && m_function_ControllerStateCallBack != nullptr)
		{
			m_function_ControllerStateCallBack(m_nHandleIndex, &m_memControllerState);
		}
	}
}

float CXInputHelper::NormalizeTrigger(unsigned char value)
{
	return static_cast<float>(value) / 255.0f;
}

float CXInputHelper::NormalizeThumbstick(short value)
{
	if (value > 0)
	{
		return static_cast<float>(value) / 32767.0f;
	}
	else if (value < 0)
	{
		return static_cast<float>(value) / 32768.0f;
	}
	return 0.0f;
}