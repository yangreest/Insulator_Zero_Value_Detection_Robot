
#ifdef _DEBUG
#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <iostream>
#endif

#include "UI/Insulator_Zero_Value_Detection_Robot.h"
#include <QtWidgets/QApplication>

int main(int argc, char* argv[])
{

#ifdef _DEBUG

	AllocConsole();
	freopen("CONOUT$", "w", stdout); // 重定向标准输出到控制台
	std::cout << "Console allocated at runtime!" << std::endl;
#endif
	QApplication app(argc, argv);
	Insulator_Zero_Value_Detection_Robot window;
	window.show();
	return app.exec();
}