// CFRD_FC.cpp : 定义 DLL 应用程序的导出函数。
//

#include<iomanip>
#include <sstream>
#include "ScanS_FC.h"

using std::ios;
using std::stringstream;
using std::setfill;
using std::setw;

//CCFRD_CriticalSection////////////////////////////////////////////////////////////////////////////
CCFRD_CriticalSection::CCFRD_CriticalSection()
{
	InitCS();
}

CCFRD_CriticalSection::~CCFRD_CriticalSection()
{
	DelCS();
}

void CCFRD_CriticalSection::Lock()
{
	EnterCriticalSection(&m_CS);
}

void CCFRD_CriticalSection::UnLock()
{
	LeaveCriticalSection(&m_CS);
}

void CCFRD_CriticalSection::InitCS()
{
	InitializeCriticalSection(&m_CS);
}

void CCFRD_CriticalSection::DelCS()
{
	DeleteCriticalSection(&m_CS);
}

///CCFRD_Convert
///////////////////////////////////////////////////////////////////////////////////////////////////
WORD CCFRD_Convert::StringToWORD(IN const std::string& strNumber)
{
	WORD wResult = 0;
	stringstream oss;
	oss << strNumber;
	oss >> wResult;

	return wResult;
}

std::string CCFRD_Convert::WORDToString(IN WORD wNumber)
{
	/*****************************************
	数组大小必须要大于转成的string类型的长度
	************************************************/
	std::string strResult;
	stringstream oss;
	oss << wNumber;
	oss >> strResult;

	return strResult;
}

bool CCFRD_Convert::WORDToCharArray(IN WORD wNumber, IN OUT char acResult[], IN size_t size)
{
	/*****************************************
	数组大小必须要大于转成的string类型的长度
	************************************************/
	std::string strResult;
	stringstream oss;
	oss << wNumber;
	oss >> strResult;

	if (strResult.length() < size)
	{
		memcpy_s(acResult, size, strResult.c_str(), strResult.length());
		acResult[strResult.length()] = 0x00;
		return true;
	}

	return false;
}

WORD CCFRD_Convert::CharArrayToWORD(IN const char acNumber[])
{
	return StringToWORD(acNumber);
}

int CCFRD_Convert::StringToInt(IN const std::string& strNumber)
{
	int iResult = 0;
	stringstream oss;
	oss << strNumber;
	oss >> iResult;

	return iResult;
}

std::string CCFRD_Convert::IntToString(IN int iNumber)
{
	std::string strResult;
	stringstream oss;
	oss << iNumber;
	oss >> strResult;

	return strResult;
}

bool CCFRD_Convert::IntToCharArray(IN int iNumber, IN OUT char acResult[], IN size_t size, IN int iRadix /* = 10 */)
{
	/*****************************************
	首先看iNumber能不能转换成string类型，如果能，
	在看看数组大小必须要大于转成的string类型的长度
	************************************************/
	char acBuf[32] = "";
	memset(acBuf, 0, sizeof(acBuf));
	_itoa_s(iNumber, acBuf, sizeof(acBuf), iRadix);

	std::string str = acBuf;
	if (str.length() < size)
	{
		memcpy_s(acResult, size, str.c_str(), str.length());
		acResult[str.length()] = 0x00;
		return true;
	}

	return false;
}

int CCFRD_Convert::CharArrayToInt(IN const char acNumber[])
{
	return StringToInt(acNumber);
}

FLOAT CCFRD_Convert::StringToFloat(IN const std::string& strNumber)
{
	float fResult = 0;
	stringstream oss;
	oss << strNumber;
	oss >> fResult;

	return fResult;
}

std::string CCFRD_Convert::FloatToString(IN float fNumber, IN int iDecimalCount/* = 3*/)
{
	/*****************************************
	数组大小必须要大于转成的string类型的长度
	************************************************/
	std::string strResult = "";

	stringstream oss;
	oss << std::setiosflags(ios::fixed) << std::setprecision(iDecimalCount) << fNumber;
	oss >> strResult;

	return strResult;
}

bool CCFRD_Convert::FloatToCharArray(IN float fNumber, IN OUT char acResult[], IN int iSize,
	IN int iDecimalCount /* = 3 */)
{
	/*****************************************
	数组大小必须要大于转成的string类型的长度
	************************************************/
	std::string strResult = "";

	stringstream oss;
	oss << std::setiosflags(ios::fixed) << std::setprecision(iDecimalCount) << fNumber;
	oss >> strResult;

	if (strResult.length() < iSize)
	{
		memcpy_s(acResult, iSize, strResult.c_str(), strResult.length());
		acResult[strResult.length()] = 0x00;
		return true;
	}

	return false;
}

float CCFRD_Convert::CharArrayToFloat(IN const char acNumber[])
{
	return StringToFloat(acNumber);
}

double CCFRD_Convert::StringToDouble(IN const std::string& strNumber)
{
	double dfResult = 0.0;

	stringstream oss;
	oss << strNumber;
	oss >> dfResult;

	return dfResult;
}

std::string CCFRD_Convert::DoubleToString(IN double dfNumber, IN int iDecimalCount/* = 3*/)
{
	/*****************************************
	数组大小必须要大于转成的string类型的长度
	************************************************/
	std::string strResult = "";
	stringstream oss;

	oss << std::setiosflags(ios::fixed) << std::setprecision(iDecimalCount) << dfNumber;
	oss >> strResult;

	return strResult;
}

bool CCFRD_Convert::DoubleToCharArray(IN double dfNumber, IN OUT char acResult[], IN size_t iSize,
	IN int iDecimalCount /* = 3 */)
{
	/*****************************************
	数组大小必须要大于转成的string类型的长度
	************************************************/
	std::string strResult = "";
	stringstream oss;

	oss << std::setiosflags(ios::fixed) << std::setprecision(iDecimalCount) << dfNumber;
	oss >> strResult;

	if (strResult.length() < iSize)
	{
		memcpy_s(acResult, iSize, strResult.c_str(), strResult.length());
		acResult[strResult.length()] = 0x00;
		return true;
	}

	return false;
}

double CCFRD_Convert::CharArrayToDouble(IN const char acNumber[])
{
	return StringToDouble(acNumber);
}

std::string CCFRD_Convert::SysTimeToString_s(IN const SYSTEMTIME& sysTime)
{
	/*****************************************
	数组大小必须要大于转成的string类型的长度
	************************************************/
	std::string strResult = "";
	stringstream oss;
	oss << setfill('0') << setw(4) << sysTime.wYear << "-" << setw(2) << sysTime.wMonth << "-" << setw(2)
		<< sysTime.wDay << " " << setw(2) << sysTime.wHour << ":" << setw(2) << sysTime.wMinute << ":" << setw(2)
		<< sysTime.wSecond; // << ":" << setw(2) << sysTime.wMilliseconds;
	strResult = oss.str();

	return strResult;
}

bool CCFRD_Convert::SysTimeToCharArray_s(IN const SYSTEMTIME& sysTime, IN OUT char acResult[], IN int iSize)
{
	/*****************************************
	数组大小必须要大于转成的string类型的长度
	************************************************/
	std::string strResult = "";
	stringstream oss;
	oss << setfill('0') << setw(4) << sysTime.wYear << "-" << setw(2) << sysTime.wMonth << "-" << setw(2)
		<< sysTime.wDay << " " << setw(2) << sysTime.wHour << ":" << setw(2) << sysTime.wMinute << ":" << setw(2)
		<< sysTime.wSecond; // << ":" << setw(2) << sysTime.wMilliseconds;
	strResult = oss.str();
	if (strResult.length() < iSize)
	{
		memcpy_s(acResult, iSize, strResult.c_str(), strResult.length());
		acResult[strResult.length()] = 0x00;
		return true;
	}

	return false;
}

std::string CCFRD_Convert::SysTimeToString_ms(IN const SYSTEMTIME& sysTime)
{
	/*****************************************
	数组大小必须要大于转成的string类型的长度
	************************************************/
	std::string strResult = "";
	stringstream oss;
	oss << setfill('0') << setw(4) << sysTime.wYear << "-" << setw(2) << sysTime.wMonth << "-" << setw(2)
		<< sysTime.wDay << " " << setw(2) << sysTime.wHour << ":" << setw(2) << sysTime.wMinute << ":" << setw(2)
		<< sysTime.wSecond << ":" << setw(3) << sysTime.wMilliseconds;
	strResult = oss.str();

	return strResult;
}

bool CCFRD_Convert::SysTimeToCharArray_ms(IN const SYSTEMTIME& sysTime, IN OUT char acResult[], IN int iSize)
{
	/*****************************************
	数组大小必须要大于转成的string类型的长度
	************************************************/
	std::string strResult = "";
	stringstream oss;
	oss << setfill('0') << setw(4) << sysTime.wYear << "-" << setw(2) << sysTime.wMonth << "-" << setw(2)
		<< sysTime.wDay << " " << setw(2) << sysTime.wHour << ":" << setw(2) << sysTime.wMinute << ":" << setw(2)
		<< sysTime.wSecond << ":" << setw(3) << sysTime.wMilliseconds;
	strResult = oss.str();
	if (strResult.length() < iSize)
	{
		memcpy_s(acResult, iSize, strResult.c_str(), strResult.length());
		acResult[strResult.length()] = 0x00;
		return true;
	}

	return false;
}

SYSTEMTIME CCFRD_Convert::CharArrayToSysTime_s(IN const char szSysTime[])
{
	return StringToSysTime_s(szSysTime);
}

SYSTEMTIME CCFRD_Convert::CharArrayToSysTime_ms(IN const char szSysTime[])
{
	return StringToSysTime_ms(szSysTime);
}

SYSTEMTIME CCFRD_Convert::StringToSysTime_s(IN const std::string& strSysTime)
{
	SYSTEMTIME sysTime = {};
	char ch;

	stringstream oss(strSysTime);

	oss >> sysTime.wYear >> ch >> sysTime.wMonth >> ch >> sysTime.wDay >>
		sysTime.wHour >> ch >> sysTime.wMinute >> ch >> sysTime.wSecond;
	sysTime.wMilliseconds = 0;

	return sysTime;
}

SYSTEMTIME CCFRD_Convert::StringToSysTime_ms(IN const std::string& strSysTime)
{
	SYSTEMTIME sysTime = {};
	char ch;

	stringstream oss(strSysTime);

	oss >> sysTime.wYear >> ch >> sysTime.wMonth >> ch >> sysTime.wDay >>
		sysTime.wHour >> ch >> sysTime.wMinute >> ch >> sysTime.wSecond >> ch >> sysTime.wMilliseconds;
	sysTime.wMilliseconds = 0;

	return sysTime;
}

void CCFRD_Convert::SplitString(
	const std::string& strOri, const std::vector<char>& vecSplit,
	std::vector<std::string>& vecSplitResult)
{
	vecSplitResult.clear();

	bool ret = false;
	int sizeStrOri = (int)(strOri.size()); // 多次使用该值，先保存结果

	char* pcTmp = new char[sizeStrOri + 1]; // 分配内存，+1是为了保存末尾\0
	memcpy(pcTmp, strOri.c_str(), sizeStrOri);
	pcTmp[sizeStrOri] = 0x00;

	// 将所有分隔符变成'\0'
	for (char* pcIndex = pcTmp; *pcIndex != 0x00; ++pcIndex)
	{
		for (std::vector<char>::const_iterator it = vecSplit.begin(); it != vecSplit.end(); ++it)
		{
			if (*pcIndex == *it)
			{
				*pcIndex = 0x00; // 将分隔符变成'\0'
				break;
			}
		}
	}

	// 将所有数据插入输出向量中
	std::string strTmp = ""; // 用于单个输出的字符串
	for (char* pcIndex = pcTmp; pcIndex - pcTmp <= sizeStrOri;)
	{
		if (0x00 != *pcIndex) // 第一个不为0则直接压入
		{
			strTmp = pcIndex;
			pcIndex += strTmp.size() + 1; // 排除内容加上0x00
		}
		else // 第一个为0 压入""
		{
			strTmp = "";
			++pcIndex; // 只用排除0x00
		}
		vecSplitResult.push_back(strTmp);
	}

	delete[] pcTmp;
	pcTmp = NULL;
}

/* /////////////////////CCFRD_Time /////////////////////// */

CCFRD_Time::CCFRD_Time()
{
	GetLocalTime(&m_SystemTime);
}

CCFRD_Time::CCFRD_Time(IN const SYSTEMTIME& sysTime)
	: m_SystemTime(sysTime)
{
}

CCFRD_Time::CCFRD_Time(IN const std::string& strSysTime)
	: m_SystemTime({ 0 })
{
	SYSTEMTIME sysTime;
	bool bUpdate = StringToSysTime(strSysTime, sysTime);

	/************************************
	只有将strSysTime转换成功才能使得
	*************************************/
	if (bUpdate)
	{
		m_SystemTime = sysTime;
	}
}

CCFRD_Time::CCFRD_Time(IN const CCFRD_Time& time_CFRD)
	: m_SystemTime(time_CFRD.m_SystemTime)
{
}

CCFRD_Time& CCFRD_Time::operator=(IN const CCFRD_Time& time_CFRD)
{
	/* 排除自身赋值 */
	if (this != &time_CFRD)
	{
		m_SystemTime = time_CFRD.m_SystemTime;
	}

	return *this;
}

CCFRD_Time& CCFRD_Time::operator=(IN const SYSTEMTIME& sysTime)
{
	*this = CCFRD_Time(sysTime);

	return *this;
}

CCFRD_Time& CCFRD_Time::operator=(IN const std::string& strSysTime)
{
	*this = CCFRD_Time(strSysTime);

	return *this;
}

void CCFRD_Time::GetCurTime()
{
	GetLocalTime(&m_SystemTime);
}

LONGLONG CCFRD_Time::GetTimeSpan_ms(IN bool bOverride /* = false */)
{
	SYSTEMTIME sysTime;
	GetLocalTime(&sysTime);
	LONGLONG result = GetTimeSpan_ms(sysTime, m_SystemTime);

	/* 如果需要覆盖本类对象则将这次调用保存的时间赋值给成员变量 */
	if (bOverride)
	{
		m_SystemTime = sysTime;
	}

	return result;
}

LONGLONG CCFRD_Time::GetTimeSpan_s(IN bool bOverride /* = false */)
{
	SYSTEMTIME sysTime;
	GetLocalTime(&sysTime);
	LONGLONG result = GetTimeSpan_s(sysTime, m_SystemTime);

	/* 如果需要覆盖本类对象则将这次调用保存的时间赋值给成员变量 */
	if (bOverride)
	{
		m_SystemTime = sysTime;
	}

	return result;
}

LONGLONG CCFRD_Time::GetTimeSpan_ms(IN const SYSTEMTIME& lSysTime, IN const SYSTEMTIME& rSysTime)
{
	ULARGE_INTEGER fTime1; /* FILETIME */
	ULARGE_INTEGER fTime2; /* FILETIME */
	SystemTimeToFileTime(&rSysTime, (FILETIME*)&fTime1);
	SystemTimeToFileTime(&lSysTime, (FILETIME*)&fTime2);
	LONGLONG f = ((LONGLONG)fTime2.QuadPart - (LONGLONG)fTime1.QuadPart) / 10000;
	return f;
}

LONGLONG CCFRD_Time::GetTimeSpan_s(IN const SYSTEMTIME& lSysTime, IN const SYSTEMTIME& rSysTime)
{
	ULARGE_INTEGER fTime1; /* FILETIME */
	ULARGE_INTEGER fTime2; /* FILETIME */
	SystemTimeToFileTime(&rSysTime, (FILETIME*)&fTime1);
	SystemTimeToFileTime(&lSysTime, (FILETIME*)&fTime2);
	LONGLONG f = ((LONGLONG)fTime2.QuadPart - (LONGLONG)fTime1.QuadPart) / 10000000;
	return f;
}

SYSTEMTIME CCFRD_Time::Time() const
{
	return m_SystemTime;
}

bool CCFRD_Time::TimeToCharArray_ms(IN OUT char acResult[], IN size_t size) const
{
	/***********************************
	直接将SYSTEMTIME结构体转成string，
	然后判断转换后的string类型对象的长
	度是不是小于字符数组的缓冲区大小
	**************************************/
	std::string strSysTime = SysTimeToString_ms(m_SystemTime);
	if (strSysTime.length() < size)
	{
		memcpy_s(acResult, size, strSysTime.c_str(), strSysTime.length());
		acResult[strSysTime.length()] = 0x00;
		return true;
	}

	return false;
}

std::string CCFRD_Time::TimeToString_ms() const
{
	return SysTimeToString_ms(m_SystemTime);
}

bool CCFRD_Time::TimeToCharArray_s(IN OUT char acResult[], IN size_t size) const
{
	/******************************
	直接将SYSTEMTIME结构体转成string
	然后判断长度小于字符数组的大小
	********************************/
	std::string strSysTime = SysTimeToString_s(m_SystemTime);
	if (strSysTime.length() < size)
	{
		memcpy_s(acResult, size, strSysTime.c_str(), strSysTime.length());
		acResult[strSysTime.length()] = 0x00;
		return true;
	}

	return false;
}

std::string CCFRD_Time::TimeToString_s() const
{
	return SysTimeToString_s(m_SystemTime);
}

LONGLONG CCFRD_Time::TimingStart()
{
	LARGE_INTEGER litmp;
	LONGLONG QPart1;

	QueryPerformanceCounter(&litmp);
	QPart1 = litmp.QuadPart; //   获得初始值

	return QPart1;
}

double CCFRD_Time::TimingEnd(IN LONGLONG lStartTime)
{
	LARGE_INTEGER litmp;
	LONGLONG QPart2;
	double dfMinus, dfFreq, dfTim;

	QueryPerformanceFrequency(&litmp);
	dfFreq = (double)litmp.QuadPart; //   获得计数器的时钟频率

	QueryPerformanceCounter(&litmp);
	QPart2 = litmp.QuadPart; // 获得终止值
	dfMinus = (double)(QPart2 - lStartTime);
	dfTim = dfMinus / dfFreq; //   获得对应的时间值，单位为秒

	return dfTim;
}

bool CCFRD_Time::StringToSysTime(IN const std::string& strSysTime, OUT SYSTEMTIME& sysTime)
{
	int rYear[13] = { 0, 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	int pYear[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	stringstream oss(strSysTime);
	char ch;
	bool ret = false;
	if (strSysTime.length() == strlen("yyyy-MM-dd hh:mm:ss"))
	{
		oss >> sysTime.wYear >> ch >> sysTime.wMonth >> ch >> sysTime.wDay >>
			sysTime.wHour >> ch >> sysTime.wMinute >> ch >> sysTime.wSecond;
		sysTime.wMilliseconds = 0;
		if (std::ios::eofbit == oss.rdstate() /*&&oss.rdstate() == std::ios::goodbit*/)
		{
			ret = sysTime.wYear >= 0
				&& (sysTime.wMonth > 0 && sysTime.wMonth < 13)
				&& sysTime.wDay > 0
				&& (sysTime.wHour >= 0 && sysTime.wHour < 24)
				&& (sysTime.wMinute >= 0 && sysTime.wMinute < 60)
				&& (sysTime.wSecond >= 0 && sysTime.wSecond < 60);

			if (ret)
			{
				return (((sysTime.wYear % 4 == 0) && (sysTime.wYear % 100 != 0)) || (sysTime.wYear % 400 == 0))
					? sysTime.wDay <= rYear[sysTime.wMonth]
					: sysTime.wDay <= pYear[sysTime.wMonth];
			}
		}
	}

	if (strSysTime.length() == strlen("yyyy-MM-dd hh:mm:ss:mmm"))
	{
		oss >> sysTime.wYear >> ch >> sysTime.wMonth >> ch >> sysTime.wDay >>
			sysTime.wHour >> ch >> sysTime.wMinute >> ch >> sysTime.wSecond >> ch >> sysTime.wMilliseconds;
		if (std::ios::eofbit == oss.rdstate() /*&&oss.rdstate() == std::ios::goodbit*/)
		{
			ret = sysTime.wYear >= 0
				&& (sysTime.wMonth > 0 && sysTime.wMonth < 13)
				&& sysTime.wDay > 0
				&& (sysTime.wHour >= 0 && sysTime.wHour < 24)
				&& (sysTime.wMinute >= 0 && sysTime.wMinute < 60)
				&& (sysTime.wSecond >= 0 && sysTime.wSecond < 60)
				&& (sysTime.wMilliseconds >= 0 && sysTime.wMilliseconds < 1000);

			if (ret)
			{
				return (((sysTime.wYear % 4 == 0) && (sysTime.wYear % 100 != 0)) || (sysTime.wYear % 400 == 0))
					? sysTime.wDay <= rYear[sysTime.wMonth]
					: sysTime.wDay <= pYear[sysTime.wMonth];
			}
		}
	}

	return false;
}

std::string CCFRD_Time::SysTimeToString_s(IN const SYSTEMTIME& sysTime)
{
	stringstream oss;
	oss << setfill('0') << setw(4) << sysTime.wYear << "-" << setw(2) << sysTime.wMonth << "-" << setw(2) << sysTime.
		wDay
		<< " " << setw(2) << sysTime.wHour << ":" << setw(2) << sysTime.wMinute << ":" << setw(2) << sysTime.wSecond;
	// oss >> strSysTime //会移除空格

	return oss.str();
}

std::string CCFRD_Time::SysTimeToString_ms(IN const SYSTEMTIME& sysTime)
{
	stringstream oss;
	oss << setfill('0') << setw(4) << sysTime.wYear << "-" << setw(2) << sysTime.wMonth << "-" << setw(2) << sysTime.
		wDay
		<< " " << setw(2) << sysTime.wHour << ":" << setw(2) << sysTime.wMinute << ":" << setw(2) << sysTime.wSecond
		<< ":" << setw(3) << sysTime.wMilliseconds;
	// oss >> strSysTime //会移除空格

	return oss.str();
}

bool operator <(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs)
{
	if (&lhs != &rhs)
	{
		//在实际应用中，绝大多数情况下，2个时间进行比较，2个时间的年、月、日、时、分都是相等的，只有秒或毫秒是不等的
		//所以采用下面的算法，只需要比较7次，就可以得出结果，提高了时间
		const SYSTEMTIME& lTime = lhs.m_SystemTime;
		const SYSTEMTIME& rTime = rhs.m_SystemTime;

		if (lTime.wMilliseconds < rTime.wMilliseconds &&
			lTime.wSecond == rTime.wSecond &&
			lTime.wMinute == rTime.wMinute &&
			lTime.wHour == rTime.wHour &&
			lTime.wDay == rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wSecond < rTime.wSecond &&
			lTime.wMinute == rTime.wMinute &&
			lTime.wHour == rTime.wHour &&
			lTime.wDay == rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wMinute < rTime.wMinute &&
			lTime.wHour == rTime.wHour &&
			lTime.wDay == rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wHour < rTime.wHour &&
			lTime.wDay == rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wDay < rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wMonth < rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wYear < rTime.wYear)
		{
			return true;
		}

		return false;
	}

	return false;
}

bool operator >(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs)
{
	if (&lhs != &rhs)
	{
		//在实际应用中，绝大多数情况下，2个时间进行比较，2个时间的年、月、日、时、分都是相等的，只有秒或毫秒是不等的
		//所以采用下面的算法，只需要比较7次，就可以得出结果，提高了时间
		const SYSTEMTIME& lTime = lhs.m_SystemTime;
		const SYSTEMTIME& rTime = rhs.m_SystemTime;

		if (lTime.wMilliseconds > rTime.wMilliseconds &&
			lTime.wSecond == rTime.wSecond &&
			lTime.wMinute == rTime.wMinute &&
			lTime.wHour == rTime.wHour &&
			lTime.wDay == rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wSecond > rTime.wSecond &&
			lTime.wMinute == rTime.wMinute &&
			lTime.wHour == rTime.wHour &&
			lTime.wDay == rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wMinute > rTime.wMinute &&
			lTime.wHour == rTime.wHour &&
			lTime.wDay == rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wHour > rTime.wHour &&
			lTime.wDay == rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wDay > rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wMonth > rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		if (lTime.wYear > rTime.wYear)
		{
			return true;
		}

		return false;
	}

	return false;
}

bool operator ==(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs)
{
	if (&lhs != &rhs)
	{
		const SYSTEMTIME& lTime = lhs.m_SystemTime;
		const SYSTEMTIME& rTime = rhs.m_SystemTime;

		if (lTime.wMilliseconds == rTime.wMilliseconds &&
			lTime.wSecond == rTime.wSecond &&
			lTime.wMinute == rTime.wMinute &&
			lTime.wHour == rTime.wHour &&
			lTime.wDay == rTime.wDay &&
			lTime.wMonth == rTime.wMonth &&
			lTime.wYear == rTime.wYear)
		{
			return true;
		}

		return false;
	}

	return true;
}

bool operator <=(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs)
{
	return !(lhs > rhs);
}

bool operator >=(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs)
{
	return !(lhs < rhs);
}

bool operator !=(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs)
{
	return !(lhs == rhs);
}

LONGLONG CCFRD_Time::GetTimeSpan_ms(IN const CCFRD_Time& lSysTime, IN const CCFRD_Time& rSysTime)
{
	ULARGE_INTEGER fTime1; /* FILETIME */
	ULARGE_INTEGER fTime2; /* FILETIME */
	SystemTimeToFileTime(&rSysTime.m_SystemTime, (FILETIME*)&fTime1);
	SystemTimeToFileTime(&lSysTime.m_SystemTime, (FILETIME*)&fTime2);
	LONGLONG f = ((LONGLONG)fTime2.QuadPart - (LONGLONG)fTime1.QuadPart) / 10000;
	return f;
}

LONGLONG CCFRD_Time::GetTimeSpan_s(IN const CCFRD_Time& lSysTime, IN const CCFRD_Time& rSysTime)
{
	ULARGE_INTEGER fTime1; /* FILETIME */
	ULARGE_INTEGER fTime2; /* FILETIME */
	SystemTimeToFileTime(&rSysTime.m_SystemTime, (FILETIME*)&fTime1);
	SystemTimeToFileTime(&lSysTime.m_SystemTime, (FILETIME*)&fTime2);
	LONGLONG f = ((LONGLONG)fTime2.QuadPart - (LONGLONG)fTime1.QuadPart) / 10000000;
	return f;
}