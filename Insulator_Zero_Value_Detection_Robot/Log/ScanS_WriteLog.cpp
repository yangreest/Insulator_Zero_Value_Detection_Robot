#include "WriteLogIns.h"

#include "ScanS_WriteLog.h"

CWriteLog::CWriteLog(
	const std::string& strPath, int dwMaxRowCount, int dwMaxColCount,
	int nSync/* = 0*/)
{
	m_pLog = new CWriteLogIns(strPath, dwMaxRowCount, dwMaxColCount, nSync);
}

CWriteLog::~CWriteLog()
{
	delete (CWriteLogIns*)m_pLog;
	m_pLog = NULL;
}

void CWriteLog::BeginWork()
{
	((CWriteLogIns*)m_pLog)->BeginWork();
}

void CWriteLog::EndWork()
{
	((CWriteLogIns*)m_pLog)->EndWork();
}

void CWriteLog::Write(const std::string& strMessage)
{
	((CWriteLogIns*)m_pLog)->Write(strMessage.c_str(), 0);
}

void CWriteLog::Write_Sync(const std::string& s)
{
	((CWriteLogIns*)m_pLog)->Write(s.c_str(), 1);
}

void CWriteLog::WriteFormat(const char* fmt, ...)
{
	char acBuf[4096] = "";

	va_list argp;

	va_start(argp, fmt); /* 将可变长参数转换为va_list */
	if (-1 == vsnprintf_s(acBuf, sizeof(acBuf), _TRUNCATE, fmt, argp))
	{
	}
	va_end(argp);

	((CWriteLogIns*)m_pLog)->Write(acBuf, 0);
}

void CWriteLog::WriteFormat_Sync(const char* fmt, ...)
{
	char acBuf[4096] = "";

	va_list argp;

	va_start(argp, fmt); /* 将可变长参数转换为va_list */
	if (-1 == vsnprintf_s(acBuf, sizeof(acBuf), _TRUNCATE, fmt, argp))
	{
	}
	va_end(argp);

	((CWriteLogIns*)m_pLog)->Write(acBuf, 1);
}