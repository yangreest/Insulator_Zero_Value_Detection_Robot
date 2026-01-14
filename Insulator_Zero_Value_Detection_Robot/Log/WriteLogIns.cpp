#include <io.h>
#include "WriteLogIns.h"

//CWriteLogIns
///////////////////////////////////////////////////////////////////////////////////////////////////
CWriteLogIns::CWriteLogIns(
	const string& strPath,
	int dwMaxRowCount, int dwMaxColCount,
	int nSync)
	: m_strFileName(strPath),
	m_dwMaxRowCount(dwMaxRowCount), m_dwMaxColCount(dwMaxColCount), m_nSync(nSync)
{
	m_dwMaxRowCount = max(1, m_dwMaxRowCount);
	m_dwMaxRowCount = min(MaxRowCount, m_dwMaxRowCount);

	m_dwMaxColCount = max(CLogHead::HeadLen, m_dwMaxColCount);
	m_dwMaxColCount = max(g_nMsgHeadLen + g_nMsgTailLen, m_dwMaxColCount);
	m_dwMaxColCount = min(m_dwMaxColCount, m_dwMaxColCount);

	m_LogHead.Initialize(m_dwMaxRowCount, m_dwMaxColCount, 0, 1);

	InitDirectory();

	//
	m_pThreadWrite = NULL;
	m_hEventWrite = NULL;
	m_hEventQuitWrite = NULL;

	//
	m_acMsgBuf_OneLine = new char[m_dwMaxColCount];
	m_acMsgBuf_MultiLine = new char[MaxRowCount_OneChunk * m_dwMaxColCount];
}

CWriteLogIns::~CWriteLogIns()
{
	delete[] m_acMsgBuf_OneLine;
	m_acMsgBuf_OneLine = NULL;

	delete[] m_acMsgBuf_MultiLine;
	m_acMsgBuf_MultiLine = NULL;
}

void CWriteLogIns::BeginWork()
{
	//1.
	m_hEventQuitWrite = CreateEvent(NULL, FALSE, FALSE, NULL);
	m_hEventWrite = CreateEvent(NULL, FALSE, FALSE, NULL);
	m_pThreadWrite = new std::thread(&CWriteLogIns::FuncWrite, this);
	//m_hThreadWrite = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)(&FuncWrite), this, 0, 0);
}

void CWriteLogIns::EndWork()
{
	//1.
	SetEvent(m_hEventQuitWrite);
	m_pThreadWrite->join();

	CloseHandle(m_hEventQuitWrite);
	delete m_pThreadWrite;
	CloseHandle(m_hEventWrite);

	m_hEventQuitWrite = NULL;
	m_hEventWrite = NULL;
}

void CWriteLogIns::FuncWrite()
{
	HANDLE hWaitArray[2];

	hWaitArray[0] = m_hEventQuitWrite;
	hWaitArray[1] = m_hEventWrite;

	while (true)
	{
		DWORD rc = WaitForMultipleObjects(2, hWaitArray, FALSE, INFINITE);

		if (rc == WAIT_OBJECT_0)
		{
			break;
		}
		else if (rc == WAIT_OBJECT_0 + 1)
		{
			m_csLogMessages.Lock();
			{
				m_vecMessages_Write = m_vecMessages_Chunk;
				m_vecMessages_Chunk.clear();
			}
			m_csLogMessages.UnLock();

			Write_Multi(m_vecMessages_Write);
		}
	}
}

void CWriteLogIns::InitDirectory()
{
	size_t nFlg = m_strFileName.rfind("\\");
	string strTmp = m_strFileName.substr(0, nFlg + 1);

	if (nFlg != string::npos)
	{
		const char* pcPath = strTmp.c_str();
		if ((_access(pcPath, 0)) == -1)
		{
			char FilePath[MAX_PATH] = "";

			for (int nIndex = 0; nIndex < MAX_PATH; ++nIndex)
			{
				if (pcPath[nIndex] == 0)
				{
					break;
				}

				if (pcPath[nIndex] == '\\' || pcPath[nIndex] == '/')
				{
					DWORD dwAttributesA = GetFileAttributesA(FilePath);
					if (dwAttributesA == 0xFFFFFFFF) //目录不存在则创建
					{
						CreateDirectoryA(FilePath, NULL);
					}
				}
				FilePath[nIndex] = pcPath[nIndex];
			}
		}
	}
}

void CWriteLogIns::Write(const char* szMessage, int nSync/* = 0*/)
{
	CLogInfo logInfo(szMessage, m_dwMaxColCount);

	m_csLogMessages.Lock();
	{
		if (m_vecMessages_Chunk.size() > (size_t)(MaxRowCount_OneChunk))
		{
			m_vecMessages_Chunk.clear();
		}

		m_vecMessages_Chunk.push_back(logInfo);
	}
	m_csLogMessages.UnLock();

	//3.
	if (m_nSync == 1 || nSync == 1)
	{
		m_csLogMessages.Lock();
		{
			m_vecMessages_Write_Sync = m_vecMessages_Chunk;
			m_vecMessages_Chunk.clear();
		}
		m_csLogMessages.UnLock();

		Write_Multi(m_vecMessages_Write_Sync);
	}
	else
	{
		SetEvent(m_hEventWrite);
	}
}

UINT CWriteLogIns::Write_Multi(const vector<CLogInfo>& vecMessages)
{
	if (vecMessages.empty())
	{
		return LE_Succeed;
	}

	UINT uiFlg = LE_Succeed;

	m_CS.Lock();
	do
	{
		//2.打开文件
		FILE* hFile = NULL;
		errno_t err;
		if (::_access(m_strFileName.c_str(), 0))
		{
			err = fopen_s(&hFile, m_strFileName.c_str(), "a+");
			if (err != 0)
			{
				uiFlg = LE_Open;
				break;
			}

			fclose(hFile);
			hFile = NULL;
		}

		err = fopen_s(&hFile, m_strFileName.c_str(), "rb+");
		if (err != 0 ||
			hFile == NULL)
		{
			uiFlg = LE_Open;
			break;
		}

		//3.查找文件头
		bool bEraseFile = true;

		fseek(hFile, 0, SEEK_END);
		LONG lFileSize = ftell(hFile);
		if (lFileSize > 0)
		{
			//3.1 检查文件头信息
			fseek(hFile, 0, SEEK_SET);
			size_t nHeadLen = fread(m_acMsgBuf_OneLine, 1, CLogHead::HeadLen, hFile);

			if (nHeadLen == CLogHead::HeadLen)
			{
				if (m_LogHead.InitializeFromFile(m_acMsgBuf_OneLine))
				{
					bEraseFile = false;
				}
			}
		}
		else
		{
			bEraseFile = false;

			m_LogHead.Initialize(m_dwMaxRowCount, m_dwMaxColCount, 0, 1);
		}

		//3.2
		if (bEraseFile)
		{
			fclose(hFile);
			hFile = NULL;

			err = fopen_s(&hFile, m_strFileName.c_str(), "wb+");
			if (err != 0 ||
				hFile == NULL)
			{
				uiFlg = LE_Open;
				break;
			}

			m_LogHead.Initialize(m_dwMaxRowCount, m_dwMaxColCount, 0, 1);
		}

		//4. 更新头
		m_LogHead.GoToNextLine(vecMessages.size());

		m_LogHead.GenerateString(m_acMsgBuf_OneLine, m_dwMaxColCount);

		fseek(hFile, 0, SEEK_SET);
		fwrite(m_acMsgBuf_OneLine, m_dwMaxColCount, 1, hFile);

		//5.写入内容
		int nPos = 0;

		int nCurLine = m_LogHead.m_nCurLine;
		int nFirstLine = m_LogHead.m_nCurLine;
		int nCount = 0;

		for (int i = 0; i < vecMessages.size(); ++i)
		{
			const CLogInfo& logInfo = vecMessages[i];
			string strTmp = logInfo.m_strMessage;
			SYSTEMTIME systime = logInfo.m_timeMsg;

			int nLen = strTmp.length();
			if (m_dwMaxColCount < nLen + g_nMsgHeadLen + g_nMsgTailLen)
			{
				strTmp = strTmp.substr(0, m_dwMaxColCount - g_nMsgHeadLen - g_nMsgTailLen);
				nLen = m_dwMaxColCount - g_nMsgHeadLen - g_nMsgTailLen;

				uiFlg = LE_Overflow;
			}

			sprintf_s(m_acMsgBuf_OneLine, m_dwMaxColCount, g_szMsgHeadFormat,
				nCurLine,
				systime.wYear, systime.wMonth, systime.wDay,
				systime.wHour, systime.wMinute, systime.wSecond, systime.wMilliseconds);
			memcpy(m_acMsgBuf_OneLine + g_nMsgHeadLen, strTmp.c_str(), nLen);
			memset(m_acMsgBuf_OneLine + g_nMsgHeadLen + nLen, ' ',
				m_dwMaxColCount - g_nMsgHeadLen - nLen - g_nMsgTailLen);
			memcpy(m_acMsgBuf_OneLine + m_dwMaxColCount - g_nMsgTailLen, g_szMsgTail, g_nMsgTailLen);

			memcpy(m_acMsgBuf_MultiLine + nPos, m_acMsgBuf_OneLine, m_dwMaxColCount);
			nPos += m_dwMaxColCount;

			++nCount;

			if ((nCurLine % m_dwMaxRowCount) == 0 ||
				nCount == MaxRowCount_OneChunk)
			{
				fseek(hFile, nFirstLine * m_dwMaxColCount, SEEK_SET);
				fwrite(m_acMsgBuf_MultiLine, 1, nCount * m_dwMaxColCount, hFile);

				nPos = 0;

				nFirstLine = nCurLine + 1;
				nFirstLine %= m_dwMaxRowCount;
				if (nFirstLine == 0)
				{
					nFirstLine = m_dwMaxRowCount;
				}

				nCount = 0;
			}

			++nCurLine;
			nCurLine %= m_dwMaxRowCount;
			if (nCurLine == 0)
			{
				nCurLine = m_dwMaxRowCount;
			}
		}

		if (nCount > 0)
		{
			fseek(hFile, nFirstLine * m_dwMaxColCount, SEEK_SET);
			fwrite(m_acMsgBuf_MultiLine, 1, nCount * m_dwMaxColCount, hFile);
		}

		fclose(hFile);
	} while (0);
	m_CS.UnLock();

	return uiFlg;
}

//CLogHead
///////////////////////////////////////////////////////////////////////////////////////////////////
CLogHead::CLogHead()
{
	m_nMaxLineNum = 100;
	m_nMaxColNum = 256;
	m_nNextLine = 1;
	m_nCurLine = 0;
}

CLogHead::~CLogHead()
{
}

void CLogHead::Initialize(int nMaxLineNum, int nMaxColNum, int nCurLine, int nNextLine)
{
	m_nMaxLineNum = nMaxLineNum;
	m_nMaxColNum = nMaxColNum;
	m_nCurLine = nCurLine;
	m_nNextLine = nNextLine;
}

void CLogHead::GoToNextLine(int nStep)
{
	m_nCurLine = m_nNextLine;

	m_nNextLine += nStep;
	m_nNextLine %= m_nMaxLineNum;
	if (m_nNextLine == 0)
	{
		m_nNextLine = m_nMaxLineNum;
	}
}

bool CLogHead::InitializeFromFile(const char* szLine)
{
	char szTemp[LineBitCount + 1] = "";

	if (m_nCurLine == 0)
	{
		memcpy(szTemp, szLine, LineBitCount);
		if (m_nMaxLineNum != atoi(szTemp))
		{
			return false;
		}

		memcpy(szTemp, szLine + (LineBitCount + 1), LineBitCount);
		m_nCurLine = atoi(szTemp);

		memcpy(szTemp, szLine + (LineBitCount + 1) * 2, LineBitCount);
		m_nNextLine = atoi(szTemp);

		memcpy(szTemp, szLine + (LineBitCount + 1) * 3, LineBitCount);
		if (m_nMaxColNum != atoi(szTemp))
		{
			return false;
		}
	}
	else
	{
		memcpy(szTemp, szLine, LineBitCount);
		if (m_nMaxLineNum != atoi(szTemp))
		{
			return false;
		}

		memcpy(szTemp, szLine + (LineBitCount + 1), LineBitCount);
		if (m_nCurLine != atoi(szTemp))
		{
			return false;
		}

		memcpy(szTemp, szLine + (LineBitCount + 1) * 2, LineBitCount);
		if (m_nNextLine != atoi(szTemp))
		{
			return false;
		}

		memcpy(szTemp, szLine + (LineBitCount + 1) * 3, LineBitCount);
		if (m_nMaxColNum != atoi(szTemp))
		{
			return false;
		}
	}

	return true;
}

void CLogHead::GenerateString(char* szLine, int nSize)
{
	if (nSize != m_nMaxColNum)
	{
		return;
	}

	sprintf_s(szLine, nSize, "%06d %06d %06d %06d", m_nMaxLineNum, m_nCurLine, m_nNextLine, m_nMaxColNum);
	memset(szLine + HeadLen, ' ', nSize - HeadLen);
	memcpy(szLine + (nSize - g_nMsgTailLen), g_szMsgTail, g_nMsgTailLen);
}