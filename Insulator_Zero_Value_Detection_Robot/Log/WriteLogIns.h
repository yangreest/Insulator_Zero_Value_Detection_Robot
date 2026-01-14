#pragma once

#include <thread>
#include <vector>
using std::vector;

#include <string>
using std::string;

#include "ScanS_FC.h"

const char g_szMsgTail[] = "\r\n\r\n";
const int g_nMsgTailLen = (int)(strlen(g_szMsgTail));

const char g_szMsgHeadFormat[] = "[%06d][%04d-%02d-%02d %02d:%02d:%02d:%03d] ";
const int g_nMsgHeadLen = 34;

enum LogError
{
	LE_Succeed = 0, //成功
	LE_Init = 1, //初始化时行数或列数错误
	LE_Open = 2, //打开日志文件错误
	LE_Overflow = 3 //写入的内容溢出
};

class CLogHead
{
public:
	const static int LineBitCount = 6;
	const static int HeadLen = LineBitCount * 4 + 3;

public:
	int m_nMaxLineNum;
	int m_nMaxColNum;
	int m_nCurLine;
	int m_nNextLine;

public:
	CLogHead();
	~CLogHead();

public:
	void Initialize(int nMaxLineNum, int nMaxColNum, int nCurLine, int nNextLine);

	void GoToNextLine(int nStep);

	bool InitializeFromFile(const char* szLine);
	void GenerateString(char* szLine, int nSize);
};

class CLogInfo
{
public:
	string m_strMessage;
	SYSTEMTIME m_timeMsg;

public:
	CLogInfo(const char* szMessage, int nMaxMsgLen)
		: m_strMessage(szMessage, min(nMaxMsgLen, strlen(szMessage)))
	{
		GetLocalTime(&m_timeMsg);
	}
};

class CWriteLogIns
{
private:
	static const int MaxColCount = 1024;
	static const int MaxRowCount = 999999;
	static const int MaxRowCount_OneChunk = 256;

private:
	string m_strFileName;
	DWORD m_dwMaxRowCount;
	DWORD m_dwMaxColCount;

	int m_nSync; //1:同步   0:异步

	//以下几个变量可以使外部程序的调用与内部的写操作分离
	//用于锁住m_vecMessages_Chunk
	CCFRD_CriticalSection m_csLogMessages;
	vector<CLogInfo> m_vecMessages_Chunk;
	vector<CLogInfo> m_vecMessages_Write;
	vector<CLogInfo> m_vecMessages_Write_Sync;

	CLogHead m_LogHead;

	char* m_acMsgBuf_OneLine;
	char* m_acMsgBuf_MultiLine;

private:
	//用于锁住文件的指针，这样外面的程序可以多个线程同时往同一个文件中写
	CCFRD_CriticalSection m_CS;

private:
	HANDLE m_hEventQuitWrite;
	HANDLE m_hEventWrite;
	std::thread* m_pThreadWrite;

public:
	//strPath：路径+文件名
	//dwMaxRowCount：日志文件的最大行数(最大值不能超过999999)，循环覆盖
	//dwMaxColCount：日志文件每行的字节数
	//dwMaxColCount的最小值不得少于38字节,其中34字节为固定的日期长度,4个字节为连续2个回车换行符)
	//nSync  1:同步   0:异步
	CWriteLogIns(
		const string& strPath,
		int dwMaxRowCount, int dwMaxColCount,
		int nSync);

	~CWriteLogIns();

	void BeginWork();

	void EndWork();

public:
	//参数nSync为1时会立即往文件中同步写入（包括之前的待写入的行）
	//参数nSync为0时，根据构造实例时的参数，进行同步或异步写入
	void Write(const char* szMessage, int nSync = 0);

private:
	void InitDirectory();

	UINT Write_Multi(const vector<CLogInfo>& vecMessages);

private:
	void FuncWrite();
};
