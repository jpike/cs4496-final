#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <map>
#include <fstream>
#include "vl/VLd.h"

//**********************************************************************
// Logger utility for program
// OBSOLETE - Not really needed in grand scheme of things
//**********************************************************************
class Logger
{
public:
	static void OpenLogFile(std::string filename);
	static void CloseLogFile(std::string filename);

	static void Print(std::string text);
	static void Print(std::size_t num);
	static void Print(int num);
	static void Print(Vec3d & vec);
	static void Print(Vecd & vec);
	static void Print(double num);
	static void Print(Mat4d & mat);
	static void Print(Vec4d & vec);

	static void PrintLine(std::string text);
	static void PrintLine(std::size_t num);
	static void PrintLine(int num);
	static void PrintLine(Vec3d & vec);
	static void PrintLine(Vecd & vec);
	static void PrintLine(double num);
	static void PrintLine(Mat4d & mat);
	static void PrintLine(Vec4d & vec);

private:
	static std::map<std::string, std::ofstream *> logFiles;
	static std::ofstream * currentFile;
};


#endif