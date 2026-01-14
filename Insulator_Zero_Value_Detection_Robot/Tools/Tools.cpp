#include "Tools.h"
#include <ctime>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <shlwapi.h>
#pragma comment(lib, "shlwapi.lib") // 链接 shlwapi 库

std::string WHSD_Tools::GetCurrentTimeString()
{
	time_t now = time(0);
	tm ltm{};

#ifdef _MSC_VER
	// Windows 平台使用安全版本
	localtime_s(&ltm, &now);
#else
	// 其他平台使用标准版本
	localtime_r(&now, &ltm);
#endif

	char timeBuffer[20]; // 格式: YYYYMMDD_HHMMSS
	strftime(timeBuffer, sizeof(timeBuffer), "%Y%m%d_%H%M%S", &ltm);
	return std::string(timeBuffer);
}

bool WHSD_Tools::SaveDataToFile(uint8_t* data, int len, const std::string& fileType)
{
	try
	{
		// 获取当前日期并格式化为 yyyy_MM_dd
		time_t now = time(0);
		tm ltm{};

#ifdef _MSC_VER
		// Windows 平台使用安全版本
		localtime_s(&ltm, &now);
#else
		// 其他平台使用标准版本
		localtime_r(&now, &ltm);
#endif

		char dateBuffer[11];
		strftime(dateBuffer, sizeof(dateBuffer), "%Y_%m_%d", &ltm);
		std::string dateFolderName(dateBuffer);

		// 构建完整路径
#ifdef _DEBUG
		std::filesystem::path currentPath = std::filesystem::current_path();
#else
		std::filesystem::path currentPath = GetExeDirectory();
#endif
		std::filesystem::path targetDir = (currentPath / "AutoSave") / dateFolderName;

		CreateFolderRecursively(targetDir.string());
		auto fileName = GetCurrentTimeString() + fileType;
		// 构建以当前时间为名称的文件
		std::filesystem::path filePath = targetDir / fileName;
		std::string fullPath = filePath.string();

		// 保存二进制文件
		std::ofstream file(fullPath, std::ios::binary);
		if (!file.is_open())
		{
			return false;
		}

		file.write(reinterpret_cast<const char*>(data), len);
		file.close();

		return true;
	}
	catch (const std::exception& e)
	{
		return false;
	}
}

bool WHSD_Tools::SaveDataToFile2(uint8_t* data, int len, const std::string& fullPath)
{
	try
	{
		// 保存二进制文件
		std::ofstream file(fullPath, std::ios::binary);
		if (!file.is_open())
		{
			return false;
		}

		file.write(reinterpret_cast<const char*>(data), len);
		file.close();

		return true;
	}
	catch (const std::exception& e)
	{
		return false;
	}
}

std::string WHSD_Tools::GetGuidPath(const std::string& guid, const std::string& fName)
{
	// 构建完整路径
#ifdef _DEBUG
	std::filesystem::path currentPath = std::filesystem::current_path();
#else
	std::filesystem::path currentPath = GetExeDirectory();
#endif
	std::filesystem::path targetDir = (currentPath / fName) / guid;
	auto path = targetDir.string();
	if (CreateFolderRecursively(path))
	{
		return path;
	}
	else
	{
		return {};
	}
}

bool WHSD_Tools::SaveFileByGuid(uint8_t* data, int len, const std::string& guid, int workMode,
	const std::string& fileType)
{
	auto pt = GetGuidPath(guid, workMode == 1 ? "pic" : "temp");
	if (pt.empty())
	{
		return false;
	}
	std::string fileName = GetCurrentTimeString();
	fileName.append(fileType);
	std::string fullPath = pt + "//" + fileName;
	// 保存二进制文件
	std::ofstream file(fullPath, std::ios::binary);
	if (!file.is_open())
	{
		return false;
	}

	file.write(reinterpret_cast<const char*>(data), len);
	file.close();

	return true;
}

std::vector<std::string> WHSD_Tools::SplitString(const std::string& str, char delimiter)
{
	std::vector<std::string> tokens;
	std::stringstream ss(str);
	std::string token;

	while (std::getline(ss, token, delimiter))
	{
		tokens.push_back(token);
	}

	return tokens;
}

bool WHSD_Tools::ReadFileToVector(const std::string& filename, std::vector<uint8_t>* buffer)
{
	std::ifstream file(filename, std::ios::binary);
	if (!file.is_open())
	{
		return false;
	}

	// 分块读取（每次4KB）
	const size_t chunkSize = 4096;
	buffer->clear();

	while (true)
	{
		size_t oldSize = buffer->size();
		buffer->resize(oldSize + chunkSize);

		file.read(reinterpret_cast<char*>(buffer->data() + oldSize), chunkSize);
		size_t bytesRead = file.gcount();

		if (bytesRead < chunkSize)
		{
			buffer->resize(oldSize + bytesRead); // 调整为实际读取的大小
			break;
		}
	}

	return true;
}

std::vector<std::vector<uint8_t>> WHSD_Tools::SplitVectorData(const std::vector<uint8_t>& data, size_t n)
{
	if (n == 0) return {}; // 处理无效输入

	const size_t totalSize = data.size();
	const size_t numChunks = (totalSize + n - 1) / n; // 向上取整计算块数

	std::vector<std::vector<uint8_t>> result;
	result.reserve(numChunks); // 预分配空间

	for (size_t i = 0; i < numChunks; ++i)
	{
		const size_t start = i * n;
		const size_t length = (std::min)(n, totalSize - start);

		std::vector<uint8_t> chunk(n, 0); // 初始化为全0
		std::copy_n(data.begin() + start, length, chunk.begin());

		result.emplace_back(std::move(chunk)); // 使用emplace_back避免拷贝
	}

	return result;
}

std::string WHSD_Tools::GetExeDirectory()
{
	char szExePath[MAX_PATH] = { 0 };

	// 获取当前可执行文件的完整路径（ANSI版本）
	if (GetModuleFileNameA(NULL, szExePath, MAX_PATH) == 0)
	{
		return ""; // 获取失败
	}

	// 移除文件名，只保留目录路径（ANSI版本）
	if (PathRemoveFileSpecA(szExePath) == FALSE)
	{
		return ""; // 处理失败
	}

	return std::string(szExePath);
}

std::vector<std::string> WHSD_Tools::GetAllFilesInDirectory(const std::string& directory, const std::string& fileExt)
{
	std::vector<std::string> filenames;
	std::string searchPath = directory + "\\*.*"; // 搜索所有文件

	WIN32_FIND_DATAA findData;
	HANDLE hFind = FindFirstFileA(searchPath.c_str(), &findData);

	// 检查查找是否成功
	if (hFind == INVALID_HANDLE_VALUE)
	{
		return filenames;
	}

	do
	{
		// 跳过当前目录 (.) 和上级目录 (..)
		if (strcmp(findData.cFileName, ".") == 0 || strcmp(findData.cFileName, "..") == 0)
		{
			continue;
		}

		// 判断是否为文件（不是目录）
		if (!(findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
		{
			std::string fileName = findData.cFileName;

			// 如果不需要筛选文件类型，直接添加
			if (fileExt.empty())
			{
				filenames.push_back(fileName);
			}
			// 否则检查文件扩展名是否匹配
			else
			{
				// 使用PathFindExtensionA获取文件扩展名并比较
				const char* ext = PathFindExtensionA(fileName.c_str());
				if (ext != nullptr && _stricmp(ext, fileExt.c_str()) == 0)
				{
					filenames.push_back(fileName);
				}
			}
		}
	} while (FindNextFileA(hFind, &findData) != 0);

	// 关闭查找句柄
	FindClose(hFind);

	// 检查是否因为没有更多文件而结束循环
	DWORD lastError = GetLastError();
	if (lastError != ERROR_NO_MORE_FILES)
	{
	}

	return filenames;
}

std::vector<int> WHSD_Tools::ExtractIntegers(const std::vector<std::string>& strs)
{
	std::vector<int> result;
	result.reserve(strs.size()); // 预分配空间，提高效率

	for (const std::string& s : strs)
	{
		try
		{
			// 尝试转换字符串为 int
			int num = std::stoi(s);
			result.push_back(num);
		}
		catch (const std::invalid_argument&)
		{
			// 无法转换（如非数字字符串），跳过
			continue;
		}
		catch (const std::out_of_range&)
		{
			// 数值超出 int 范围，跳过
			continue;
		}
	}

	return result;
}

std::string WHSD_Tools::GetAbsolutePath(const std::string& s)
{
#ifdef _DEBUG
	return s;
#else
	auto rst = GetExeDirectory();
	rst.append("\\");
	rst.append(s);
	return rst;
#endif
}

void WHSD_Tools::ScaleUInt16Array(uint16_t* data, int len)
{
	// 检查输入有效性
	if (data == nullptr)
	{
		throw std::invalid_argument("数据指针不能为空");
	}
	if (len <= 0)
	{
		throw std::invalid_argument("数组长度必须为正数");
	}

	// 找到数组中的最大值
	// 使用std::max_element，需要指定起始和结束迭代器（指针可作为迭代器）
	uint16_t max_val = *std::max_element(data, data + len);

	// 避免除以零（所有元素都为0时无需处理）
	if (max_val == 0)
	{
		return;
	}

	// 计算缩放因子
	const double scale_factor = 65535.0 / static_cast<double>(max_val);

	// 遍历数组并应用缩放
	for (int i = 0; i < len; ++i)
	{
		data[i] = static_cast<uint16_t>(static_cast<double>(data[i]) * scale_factor);
	}
}

// 将二进制数据编码为Base64字符串
std::string WHSD_Tools::Base64Encode(const std::vector<uint8_t>& data)
{
	static const char* base64_chars =
		"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

	std::string encoded;
	size_t n = data.size();

	for (size_t i = 0; i < n; i += 3) {
		// 取当前组的 3 个字节（不足 3 个时补 0）
		uint8_t b0 = (i < n) ? data[i] : 0;
		uint8_t b1 = (i + 1 < n) ? data[i + 1] : 0;
		uint8_t b2 = (i + 2 < n) ? data[i + 2] : 0;

		// 计算 4 个 6 位子组的值
		uint8_t c0 = (b0 >> 2) & 0x3F;          // 取 b0 的高 6 位
		uint8_t c1 = ((b0 & 0x03) << 4) | ((b1 >> 4) & 0x0F); // b0 低 2 位 + b1 高 4 位
		uint8_t c2 = ((b1 & 0x0F) << 2) | ((b2 >> 6) & 0x03); // b1 低 4 位 + b2 高 2 位
		uint8_t c3 = (b2 & 0x3F);               // b2 的低 6 位

		// 根据实际字节数确定有效字符和填充符
		int num_bytes = (i + 2 < n) ? 3 : (i + 1 < n) ? 2 : 1;
		switch (num_bytes) {
		case 3:
			encoded += base64_chars[c0];
			encoded += base64_chars[c1];
			encoded += base64_chars[c2];
			encoded += base64_chars[c3];
			break;
		case 2:
			encoded += base64_chars[c0];
			encoded += base64_chars[c1];
			encoded += base64_chars[c2];
			encoded += '='; // 最后一个字符填充 =
			break;
		case 1:
			encoded += base64_chars[c0];
			encoded += base64_chars[c1];
			encoded += '='; // 最后两个字符填充 =
			encoded += '=';
			break;
		}
	}

	return encoded;
}

// 将Base64字符串解码为二进制数据
std::vector<uint8_t> WHSD_Tools::Base64Decode(const std::string& encoded)
{
	static const int8_t base64_values[256] = {
		// 标准 Base64 字符映射表（'A'=0, 'B'=1,..., '+'=62, '/'=63，其他为 -1）
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 62, -1, -1, -1, 63,
		52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1, -1, -1, -1, -1, -1,
		-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
		15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, -1, -1, -1, -1, -1,
		-1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
		41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, -1, -1, -1, -1, -1
	};

	std::vector<uint8_t> decoded;
	size_t n = encoded.size();

	// 检查长度是否为 4 的倍数
	if (n % 4 != 0) {
		throw std::invalid_argument("Base64 string length must be a multiple of 4");
	}

	for (size_t i = 0; i < n; i += 4) {
		// 获取 4 个字符
		char c0 = encoded[i];
		char c1 = encoded[i + 1];
		char c2 = encoded[i + 2];
		char c3 = encoded[i + 3];

		// 转换为 6 位值（-1 表示非法字符）
		int v0 = base64_values[static_cast<unsigned char>(c0)];
		int v1 = base64_values[static_cast<unsigned char>(c1)];
		int v2 = base64_values[static_cast<unsigned char>(c2)];
		int v3 = base64_values[static_cast<unsigned char>(c3)];

		// 检查非法字符
		if (v0 == -1 || v1 == -1 || (c2 != '=' && v2 == -1) || (c3 != '=' && v3 == -1)) {
			throw std::invalid_argument("Invalid Base64 character");
		}

		// 根据填充符处理有效位数
		if (c2 == '=' && c3 == '=') {
			// 两个填充符：仅前两个字符有效，生成 1 个字节
			decoded.push_back(static_cast<uint8_t>((v0 << 2) | ((v1 & 0x30) >> 4)));
		}
		else if (c2 == '=') {
			// 一个填充符：前三个字符有效，生成 2 个字节
			decoded.push_back(static_cast<uint8_t>((v0 << 2) | ((v1 & 0x30) >> 4)));
			decoded.push_back(static_cast<uint8_t>(((v1 & 0x0F) << 4) | ((v2 & 0x3C) >> 2)));
		}
		else {
			// 无填充符：四个字符有效，生成 3 个字节
			decoded.push_back(static_cast<uint8_t>((v0 << 2) | ((v1 & 0x30) >> 4)));
			decoded.push_back(static_cast<uint8_t>(((v1 & 0x0F) << 4) | ((v2 & 0x3C) >> 2)));
			decoded.push_back(static_cast<uint8_t>(((v2 & 0x03) << 6) | v3));
		}
	}

	return decoded;
}

bool WHSD_Tools::CreateFolderRecursively(const std::string& folderPath)
{
	// 检查当前路径是否存在且为目录
	DWORD attr = GetFileAttributesA(folderPath.c_str());
	if (attr != INVALID_FILE_ATTRIBUTES)
	{
		// 路径存在，检查是否为目录
		return (attr & FILE_ATTRIBUTE_DIRECTORY) != 0;
	}

	// 路径不存在，查找父目录
	size_t lastSeparator = folderPath.find_last_of("\\/");
	if (lastSeparator == std::string::npos)
	{
		// 没有父目录，直接尝试创建当前目录
		if (!CreateDirectoryA(folderPath.c_str(), nullptr))
		{
			DWORD error = GetLastError();
			if (error != ERROR_ALREADY_EXISTS)
			{
				throw std::runtime_error("创建目录失败，错误代码: " + std::to_string(error));
			}
		}
		return true;
	}

	// 递归创建父目录
	std::string parentPath = folderPath.substr(0, lastSeparator);
	if (!CreateFolderRecursively(parentPath))
	{
		return false;
	}

	// 父目录创建成功后，创建当前目录
	if (!CreateDirectoryA(folderPath.c_str(), nullptr))
	{
		DWORD error = GetLastError();
		// 处理并发创建的情况
		if (error != ERROR_ALREADY_EXISTS)
		{
			throw std::runtime_error("创建目录失败，错误代码: " + std::to_string(error));
		}
	}

	return true;
}

bool WHSD_Tools::DeleteDirectoryContents(const std::string& dirPath)
{
	// 第一步：验证目标目录是否存在且为有效目录（使用 ANSI API）
	DWORD attr = GetFileAttributesA(dirPath.c_str());
	if (attr == INVALID_FILE_ATTRIBUTES || !(attr & FILE_ATTRIBUTE_DIRECTORY))
	{
		return false;
	}

	// 第二步：构造搜索路径（匹配所有文件和子目录，ANSI 版本）
	std::string searchPattern = dirPath + "\\*";
	WIN32_FIND_DATAA findData; // ANSI 版本的结构体
	HANDLE hFind = FindFirstFileA(searchPattern.c_str(), &findData);

	// 处理查找失败的情况（如无权限）
	if (hFind == INVALID_HANDLE_VALUE)
	{
		return false;
	}

	// 第三步：遍历目录下的所有内容（文件和子目录）
	do
	{
		// 跳过当前目录（.）和父目录（..），避免无限递归
		if (strcmp(findData.cFileName, ".") == 0 || strcmp(findData.cFileName, "..") == 0)
		{
			continue;
		}

		// 拼接完整路径（ANSI 版本）
		std::string fullPath = dirPath + "\\" + findData.cFileName;

		// 第四步：根据类型处理（文件或子目录）
		if (findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		{
			// 子目录：递归删除其所有内容（包括自身）
			if (!DeleteDirectoryContents(fullPath))
			{
				// 递归调用自身处理子目录
				FindClose(hFind);
				return false;
			}
		}
		else
		{
			// 文件：移除阻碍删除的属性后删除（ANSI 版本）
			DWORD fileAttr = GetFileAttributesA(fullPath.c_str());
			if (fileAttr == INVALID_FILE_ATTRIBUTES)
			{
				continue; // 跳过已删除的文件
			}

			// 移除只读、隐藏、系统属性（这些属性可能阻止删除）
			fileAttr &= ~(FILE_ATTRIBUTE_READONLY | FILE_ATTRIBUTE_HIDDEN | FILE_ATTRIBUTE_SYSTEM);
			if (!SetFileAttributesA(fullPath.c_str(), fileAttr))
			{
				// 尝试强制删除（即使属性未完全移除）
				if (!DeleteFileA(fullPath.c_str()))
				{
					FindClose(hFind);
					return false;
				}
			}
			else
			{
				// 属性设置成功后删除文件
				if (!DeleteFileA(fullPath.c_str()))
				{
					FindClose(hFind);
					return false;
				}
			}
		}
	} while (FindNextFileA(hFind, &findData) != 0); // 继续遍历下一个文件/目录（ANSI 版本）

	// 关闭查找句柄
	FindClose(hFind);

	// 所有内容已删除，目标目录保留
	return true;
}