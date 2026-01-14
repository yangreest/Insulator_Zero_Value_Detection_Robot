#pragma once

#include <string>

class CWriteLog
{
public:
	//创建一个对象后，可以在多个线程中通过此对象写日志
	//多个对象不能写同一个日志文件

	//strPath：路径+文件名
	//dwMaxRowCount：日志文件的最大行数(最大值不能超过999999)，循环覆盖
	//dwMaxColCount：日志文件每行的字节数
	//dwMaxColCount须大于38字节,其中34字节为固定的日期长度,4个字节为连续2个回车换行符)
	//dwMaxColCount须小于等于1024字节
	//nSync  1:同步   0:异步
	CWriteLog(
		const std::string& strPath, int dwMaxRowCount, int dwMaxColCount,
		int nSync = 0);

	~CWriteLog();

	//构造函数之后调用
	void BeginWork();

	//析构函数之前调用
	void EndWork();

	//参数nSync为1时会立即往文件中同步写入（包括之前的待写入的行）
	//参数nSync为0时，根据构造实例时的参数，进行同步或异步写入
	//szMessage必须是'\0'结尾
	void Write(const std::string& strMessage);

	void Write_Sync(const std::string& s);

	void WriteFormat(const char* fmt, ...);

	void WriteFormat_Sync(const char* fmt, ...);

private:
	void* m_pLog;
};
