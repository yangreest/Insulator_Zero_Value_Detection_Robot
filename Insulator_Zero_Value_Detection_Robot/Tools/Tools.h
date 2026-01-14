#pragma once
#include <string>
#include <vector>

// 读取uint8_t中指定位置的bit值（返回0或1）
#define TOOLS_GET_BIT(value, bit) (((value) & (1U << (bit))) ? 1U : 0U)

// 设置uint8_t中指定位置的bit为1
#define TOOLS_SET_BIT(value, bit) ((value) |= (1U << (bit)))

// 清除uint8_t中指定位置的bit为0
#define TOOLS_CLEAR_BIT(value, bit) ((value) &= ~(1U << (bit)))

// 翻转uint8_t中指定位置的bit值（0变1，1变0）
#define TOOLS_TOGGLE_BIT(value, bit) ((value) ^= (1U << (bit)))

// 设置uint8_t中指定位置的bit为特定值（value为0或1）
#define TOOLS_WRITE_BIT(value, bit, bitvalue) ((bitvalue) ? TOOLS_SET_BIT(value, bit) : TOOLS_CLEAR_BIT(value, bit))

namespace WHSD_Tools
{
	template <typename T>
	void SafeRelease(T*& p)
	{
		if (p != nullptr)
		{
			delete p;
			p = nullptr;
		}
	}

	template <typename T>
	void SafeReleaseWithEndWork(T*& p)
	{
		if (p != nullptr)
		{
			p->EndWork();
			delete p;
			p = nullptr;
		}
	}

	std::string GetCurrentTimeString();

	bool SaveDataToFile(uint8_t* data, int len, const std::string& fileType = ".sdraw");

	bool SaveDataToFile2(uint8_t* data, int len, const std::string& fileType = ".sdraw");

	bool SaveFileByGuid(uint8_t* data, int len, const std::string& guid, int workMode, const std::string& fileType = ".sdraw");

	std::vector<std::string> SplitString(const std::string& str, char delimiter);

	bool ReadFileToVector(const std::string& filename, std::vector<uint8_t>* buffer);

	std::vector<std::vector<uint8_t>> SplitVectorData(const std::vector<uint8_t>& data, size_t n);

	std::string GetExeDirectory();

	std::vector<std::string> GetAllFilesInDirectory(const std::string& directory, const std::string& fileExt);

	std::vector<int> ExtractIntegers(const std::vector<std::string>& strs);

	std::string GetAbsolutePath(const std::string& s);

	void ScaleUInt16Array(uint16_t* data, int len);

	// 将二进制数据编码为Base64字符串
	std::string Base64Encode(const std::vector<uint8_t>& data);

	// 将Base64字符串解码为二进制数据
	std::vector<uint8_t> Base64Decode(const std::string& base64_str);

	bool CreateFolderRecursively(const std::string& folderPath);

	std::string GetGuidPath(const std::string& guid, const std::string& fName);

	bool DeleteDirectoryContents(const std::string& dirPath);
}
