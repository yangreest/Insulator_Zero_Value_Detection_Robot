#pragma once

#include <functional>

struct ControllerState
{
	bool connected; // 控制器是否连接
	bool buttons[14]; // 按钮状态
	float leftTrigger; // 左扳机值 (0.0f - 1.0f)
	float rightTrigger; // 右扳机值 (0.0f - 1.0f)
	float leftThumbX; // 左摇杆X轴 (-1.0f - 1.0f)
	float leftThumbY; // 左摇杆Y轴 (-1.0f - 1.0f)
	float rightThumbX; // 右摇杆X轴 (-1.0f - 1.0f)
	float rightThumbY; // 右摇杆Y轴 (-1.0f - 1.0f)
	int dpad; // 方向键 (0=中, 1=上, 2=右, 3=下, 4=左)
};

class CXInputHelper
{
public:
	CXInputHelper(int index);

	bool BeginWork();

	void EndWork();

	void RegisterControllerStateCallBack(const std::function<void(int, ControllerState*)>& p);

private:
	bool ReadControllerState();

	void LoopGetHandleValue();

	// 将扳机原始值标准化到 [0.0, 1.0] 范围
	static float NormalizeTrigger(unsigned char value);

	// 将摇杆原始值标准化到 [-1.0, 1.0] 范围
	static float NormalizeThumbstick(short value);

	int m_nHandleIndex;

	bool m_bIsNeedExit;

	ControllerState m_memControllerState;

	std::function<void(int, ControllerState*)> m_function_ControllerStateCallBack;
};
