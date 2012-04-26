//**********************************************************************
// Logger utility for program
//**********************************************************************

#include "Logger.h"

// OBSOLETE - Not really needed in grand scheme of things

//----------------------------------------------------------------------
// Static variables
//----------------------------------------------------------------------
std::map<std::string, std::ofstream *> Logger::logFiles;
std::ofstream * Logger::currentFile;

//----------------------------------------------------------------------
// Opens log file
//----------------------------------------------------------------------
void Logger::OpenLogFile(std::string filename)
{
	// create and open file
	std::ofstream * file = new std::ofstream(filename);

	// add to map
	logFiles[filename] = file;

	// set as current file
	currentFile = file;
}

//----------------------------------------------------------------------
// Closes log file
//----------------------------------------------------------------------
void Logger::CloseLogFile(std::string filename)
{
	// get file from map
	std::ofstream * file = logFiles[filename];

	// close file
	file->close();

	// remove from map
	logFiles.erase(filename);

	// free memory
	delete file;
}

//----------------------------------------------------------------------
// Print text to log file
//----------------------------------------------------------------------
void Logger::Print(std::string text)
{
	(*currentFile) << text;
}

//----------------------------------------------------------------------
// Print num to log file
//----------------------------------------------------------------------
void Logger::Print(std::size_t num)
{
	(*currentFile) << num;
}

//----------------------------------------------------------------------
// Print num to log file
//----------------------------------------------------------------------
void Logger::Print(int num)
{
	(*currentFile) << num;
}

//----------------------------------------------------------------------
// Print num to log file
//----------------------------------------------------------------------
void Logger::Print(double num)
{
	(*currentFile) << num;
}

//----------------------------------------------------------------------
// Print Vec3d to log file
//----------------------------------------------------------------------
void Logger::Print(Vec3d & vec)
{
	(*currentFile) << vec;
}

//----------------------------------------------------------------------
// Print Vecd to log file
//----------------------------------------------------------------------
void Logger::Print(Vecd & vec)
{
	(*currentFile) << vec;
}

//----------------------------------------------------------------------
// Print Mat4d to log file
//----------------------------------------------------------------------
void Logger::Print(Mat4d & mat)
{
	(*currentFile) << mat;
}

//----------------------------------------------------------------------
// Print Vec4d to log file
//----------------------------------------------------------------------
void Logger::Print(Vec4d & vec)
{
	(*currentFile) << vec;
}

//----------------------------------------------------------------------
// Print text and newline to file
//----------------------------------------------------------------------
void Logger::PrintLine(std::string text)
{
	(*currentFile) << text << std::endl;
}

//----------------------------------------------------------------------
// Print num and newline to file
//----------------------------------------------------------------------
void Logger::PrintLine(std::size_t num)
{
	(*currentFile) << num << std::endl;
}

//----------------------------------------------------------------------
// Print num and newline to file
//----------------------------------------------------------------------
void Logger::PrintLine(int num)
{
	(*currentFile) << num << std::endl;
}

//----------------------------------------------------------------------
// Print Vec3d and newline to log file
//----------------------------------------------------------------------
void Logger::PrintLine(Vec3d & vec)
{
	(*currentFile) << vec << std::endl;
}

//----------------------------------------------------------------------
// Print Vecd and newline to log file
//----------------------------------------------------------------------
void Logger::PrintLine(Vecd & vec)
{
	(*currentFile) << vec << std::endl;
}

//----------------------------------------------------------------------
// Print num and newline to file
//----------------------------------------------------------------------
void Logger::PrintLine(double num)
{
	(*currentFile) << num << std::endl;
}

//----------------------------------------------------------------------
// Print Mat4d and newline to log file
//----------------------------------------------------------------------
void Logger::PrintLine(Mat4d & mat)
{
	(*currentFile) << mat << std::endl;
}

//----------------------------------------------------------------------
// Print Vec4d and newline to log file
//----------------------------------------------------------------------
void Logger::PrintLine(Vec4d & vec)
{
	(*currentFile) << vec << std::endl;
}