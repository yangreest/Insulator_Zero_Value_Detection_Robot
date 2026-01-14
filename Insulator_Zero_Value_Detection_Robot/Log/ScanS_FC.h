#pragma once

#include <windows.h>
#include <vector>
#include <string>

#define PI 3.141592653589793238462643383279
#define Radian_Degree 0.01745329251994329576923690768488

enum FUNCRETURN
{
	OK = 0,
	ERR = -1,
	NG = 1,
	NG2 = 2,
	ERR2 = -2,
	ERR3 = -3
};

class CCFRD_CriticalSection
{
public:
	/**********************************************************************
	* 功能描述： 创建CCFRD_CriticalSection类的对象，并且初始化临界区
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07	  V3.0.0
	***********************************************************************/
	CCFRD_CriticalSection();

	/**********************************************************************
	* 功能描述： 析构CCFRD_CriticalSection类的对象，并且删除临界区
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07	  V3.0.0
	***********************************************************************/
	~CCFRD_CriticalSection();

	/**********************************************************************
	* 功能描述： 进行加锁
	* 其它说明： 和UnLock()搭配使用
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07	  V3.0.0
	***********************************************************************/
	void Lock();

	/**********************************************************************
	* 功能描述： 解除锁定
	* 其它说明： 和Lock()搭配使用
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07	  V3.0.0
	***********************************************************************/
	void UnLock();

private:
	void InitCS();
	void DelCS();

private:
	CRITICAL_SECTION m_CS;
};

class CCFRD_Convert
{
public:
	static bool WORDToCharArray(IN WORD wNumber, IN OUT char acResult[], IN size_t size);
	static WORD CharArrayToWORD(IN const char acNumber[]);
	static std::string WORDToString(IN WORD wNumber);
	static WORD StringToWORD(IN const std::string& strNumber);

	static bool IntToCharArray(IN int iNumber, IN OUT char acResult[], IN size_t size, IN int iRadix = 10);
	static int CharArrayToInt(IN const char acNumber[]);
	static std::string IntToString(IN int iNumber);
	static int StringToInt(IN const std::string& strNumber);

	/**********************************************************************
	* 功能描述： 将dwNumber转换成字符数组
	* 输入参数： IN DWORD dwNumber									//将要转换的DWORD类型的数据
				 IN char acResult[]									//将整数转换成字符数组的缓冲区首地址
				 IN size_t size										// 缓冲区的大小
	* 输出参数： OUT char acResult[]								//将dNumber转换成字符数组的结果
	* 返 回 值： 非十进制或者acResult缓冲区大小小于fNumber的整数位数+stCount位小数位数+1返回false，否则返回true
	***********************************************************************/
	static bool FloatToCharArray(IN float fNumber, IN OUT char acResult[], IN int iSize, IN int iDecimalCount = 3);
	static float CharArrayToFloat(IN const char acNumber[]);
	static std::string FloatToString(IN float fNumber, IN int iDecimalCount = 3);
	static float StringToFloat(IN const std::string& strNumber);

	static bool DoubleToCharArray(IN double dfNumber, IN OUT char acResult[], IN size_t iSize,
		IN int iDecimalCount = 3);
	static double CharArrayToDouble(IN const char acNumber[]);
	static std::string DoubleToString(IN double fNumber, IN int iDecimalCount = 3);
	static double StringToDouble(IN const std::string& strNumber);

	//yyyy-mm-dd hh:mm:ss
	static bool SysTimeToCharArray_s(IN const SYSTEMTIME& sysTime, IN OUT char acResult[], IN int iSize);
	static SYSTEMTIME CharArrayToSysTime_s(IN const char szSysTime[]);
	static std::string SysTimeToString_s(IN const SYSTEMTIME& sysTime);
	static SYSTEMTIME StringToSysTime_s(IN const std::string& strSysTime);

	//yyyy-mm-dd hh：mm:ss:mmm
	static bool SysTimeToCharArray_ms(IN const SYSTEMTIME& sysTime, IN OUT char acResult[], IN int iSize);
	static SYSTEMTIME CharArrayToSysTime_ms(IN const char szSysTime[]);
	static std::string SysTimeToString_ms(IN const SYSTEMTIME& sysTime);
	static SYSTEMTIME StringToSysTime_ms(IN const std::string& strSysTime);

	/**************************************************************************
	* 函数名称： SplitString()
	* 功能描述：分割字符串
	* 输入参数：strOri：待分割的字符串 vecSplit：分割符，可能有多种 vecSplitResult 分割结果
	* 其它说明：该函数主要用于分割字符串，将strOri字符串按照vecSplit中所有的分割字符进行分割，分割
	结果保存在vecSplitResult中
	比如:strOri = "1,2:3",vecSplit=","和":"则分割成"1"、"2"、"3"保存在vecSplitResult中
	strOri = ",",  vecSplit=","则分割成""和""保存在vecSplitResult中
	strOri = ",," vecSplit = ","则分割成""、 ""和""3个保存在vecSplitResult中
	**************************************************************************/
	static void SplitString(
		const std::string& strOri, const std::vector<char>& vecSplit,
		std::vector<std::string>& vecSplitResult);
};

class CCFRD_Time;
bool operator <(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);
bool operator >(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);
bool operator <=(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);
bool operator >=(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);
bool operator ==(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);
bool operator !=(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);

class CCFRD_Time
{
public:
	/**********************************************************************
	* 功能描述： 构造CCFRD_Time类的对象, 并用当前时间初始化对象中的时间
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2016/03/24	   V1.0	      刘文方
	***********************************************************************/
	CCFRD_Time();

	/**********************************************************************
	* 功能描述： 构造CCFRD_Time类的对象，并且用sysTime初始化对象的时间
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	CCFRD_Time(IN const SYSTEMTIME& sysTime);

	/**********************************************************************
	* 功能描述： 构造CCFRD_Time类的对象，并且用strSysTime初始化对象的时间
	* 其它说明： 字符串格式必须为 yyyy-mm-dd hh:mm:ss 或则yyyy-mm-dd hh:mm:ss:mmm，而且时间必须合法，否则对象初始化失败
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	CCFRD_Time(IN const std::string& strSysTime);

	/**********************************************************************
	* 功能描述： 拷贝构造CCFRD_Time类的对象，把time_CFRD数据拷贝到调用对象中
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	CCFRD_Time(IN const CCFRD_Time& time_CFRD);

	/**********************************************************************
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	CCFRD_Time& operator=(IN const CCFRD_Time& sysTime);

	/**********************************************************************
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	CCFRD_Time& operator=(IN const SYSTEMTIME& sysTime);

	/**********************************************************************
	* 其它说明： 字符串格式必须为 yyyy-mm-dd hh:mm:ss 或则yyyy-mm-dd hh:mm:ss:mmm，而且时间必须合法，否则对象赋值失败
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	CCFRD_Time& operator=(IN const std::string& strSysTime);

	/**********************************************************************
	* 功能描述： 为本对象赋值当前时间
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	void GetCurTime();

	/**********************************************************************
	* 功能描述： 计算当前时间减去本对象的时间值，精确到毫秒（ms）
	* 输入参数： 如果bOverride==true，则计算完时间差后为本对象赋值为当前时间
	* 输出参数： 无
	* 返 回 值：当前时间减去本对象的时间值，精确到毫秒（ms）
	* 其它说明： 返回值精确到毫秒（ms）
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	LONGLONG GetTimeSpan_ms(IN bool bOverride = false);

	/**********************************************************************
	* 功能描述：  计算当前时间减去本对象的时间值，精确到秒（s）
	* 输入参数： 如果bOverride==true，则计算完时间差后为本对象赋值为当前时间
	* 返 回 值： 返回当前时间减去本对象的时间值，精确到秒（s）
	* 其它说明： 返回值精确到秒（s）
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	LONGLONG GetTimeSpan_s(IN bool bOverride = false);

	/**********************************************************************
	* 功能描述： 计算lSysTime减去rSysTime的差值，精确到毫秒（ms）
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	static LONGLONG GetTimeSpan_ms(IN const SYSTEMTIME& lSysTime, IN const SYSTEMTIME& rSysTime);

	/**********************************************************************
	* 功能描述： 计算lSysTime减去rSysTime的差值，精确到秒（s）
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	static LONGLONG GetTimeSpan_s(IN const SYSTEMTIME& lSysTime, IN const SYSTEMTIME& rSysTime);

	/**********************************************************************
	* 功能描述： 获取对象的时间，返回值格式为SYSTEMTIME
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	SYSTEMTIME Time() const;

	/**********************************************************************
	* 功能描述： 获取对象的时间，并转换成字符数组类型输出
	* 输入参数： 外部程序必须创建足够大的内存空间
	* 返 回 值： 如果空间不足，则返回false
	* 其它说明： 字符数组格式为yyyy-mm-dd hh:mm:ss:mmm
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	bool TimeToCharArray_ms(IN OUT char acResult[], IN size_t size) const;

	std::string TimeToString_ms() const;

	/**********************************************************************
	* 功能描述： 获取对象的时间，并转换成字符数组类型输出
	* 输入参数： 外部程序必须创建足够大的内存空间
	* 返 回 值： 如果空间不足，则返回false
	* 其它说明： 字符数组格式为yyyy-mm-dd hh:mm:ss
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	bool TimeToCharArray_s(IN OUT char acResult[], IN size_t size) const;

	std::string TimeToString_s() const;

	/**********************************************************************
	* 功能描述： 获取定时器的计数值
	* 其它说明： 与TimingEnd()搭配使用，可以计算调用此2个函数之间的代码执行的时间
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	static LONGLONG TimingStart();

	/**********************************************************************
	* 功能描述： 获取定时器的计数值，之后减去lStartTime，且转为以秒（s）为单位返回
	* 返 回 值： 单位秒（s）
	* 其它说明： 与TimingStart()搭配使用，可以计算调用此2个函数之间的代码执行的时间
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	static double TimingEnd(IN LONGLONG lStartTime);

	/**********************************************************************
	* 功能描述： 判断左边CCFRD_Time对象是不是小于右边CCFRD_Time对象
	* 返 回 值： 如果左边CCFRD_Time对象小于右边CCFRD_Time对象则返回true，否则返回false
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	friend bool operator <(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);

	/**********************************************************************
	* 功能描述： 判断左边CCFRD_Time对象是不是大于右边CCFRD_Time对象
	* 返 回 值： 如果左边CCFRD_Time对象大于右边CCFRD_Time对象则返回true，否则返回false
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	friend bool operator >(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);

	/**********************************************************************
	* 功能描述： 判断左边CCFRD_Time对象是不是小于等于右边CCFRD_Time对象
	* 返 回 值： 如果左边CCFRD_Time对象小于等于右边CCFRD_Time对象则返回true，否则返回false
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	friend bool operator <=(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);

	/**********************************************************************
	* 功能描述： 判断左边CCFRD_Time对象是不是大于等于右边CCFRD_Time对象
	* 返 回 值： 如果左边CCFRD_Time对象大于等于右边CCFRD_Time对象则返回true，否则返回false
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	friend bool operator >=(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);

	/**********************************************************************
	* 功能描述： 判断左边CCFRD_Time对象是不是等于右边CCFRD_Time对象
	* 返 回 值： 如果左边CCFRD_Time对象等于右边CCFRD_Time对象则返回true，否则返回false
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	friend bool operator ==(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);

	/**********************************************************************
	* 功能描述： 判断左边CCFRD_Time对象是不是不等于右边CCFRD_Time对象
	* 返 回 值： 如果左边CCFRD_Time对象不等于右边CCFRD_Time对象则返回true，否则返回false
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	friend bool operator !=(IN const CCFRD_Time& lhs, IN const CCFRD_Time& rhs);

	static bool StringToSysTime(IN const std::string& strSysTime, OUT SYSTEMTIME& sysTime);
	static std::string SysTimeToString_s(IN const SYSTEMTIME& sysTime);
	static std::string SysTimeToString_ms(IN const SYSTEMTIME& sysTime);

	/**********************************************************************
	* 功能描述： 计算lSysTime减去rSysTime的差值，精确到毫秒（ms）
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	static LONGLONG GetTimeSpan_ms(IN const CCFRD_Time& lSysTime, IN const CCFRD_Time& rSysTime);

	/**********************************************************************
	* 功能描述： 计算lSysTime减去rSysTime的差值，精确到秒（s）
	* 修改日期        版本号     修改人	      修改内容
	* -----------------------------------------------
	* 2018/03/07      V3.0.0
	***********************************************************************/
	static LONGLONG GetTimeSpan_s(IN const CCFRD_Time& lSysTime, IN const CCFRD_Time& rSysTime);

private:
	SYSTEMTIME m_SystemTime;
};
