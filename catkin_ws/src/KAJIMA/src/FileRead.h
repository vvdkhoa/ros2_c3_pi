#pragma once

#ifndef __FILE_READ__
#define __FILE_READ__

#include <iostream> 
#include <string> 
#include <vector>
#include <fstream>
#include <ctime>
#include <time.h> 
#include <map>

class FileRead {
public:
	FileRead() {
	}
	~FileRead() {
		if (FilePtr.is_open()) {
			FilePtr.close();
		}
	}
	void FileLoadInit(std::string ImgFileName) {
		FileLoadInit(ImgFileName, ':');
	}

	void FileLoadInit(std::string ImgFileName, char DataCheckMarker ) {
		CmdCutMarker = DataCheckMarker;
		bool ReadCanUse = false;
		std::string dataTmp;
		if (FilePtr.is_open()) {
			FileClose();
		}
		FilePtr.open(ImgFileName);
		if (FilePtr.is_open()) {
			//FileString.clear();
			FileStringMap.clear();
			while (getline(FilePtr, strLine)) {
				dataTmp = "";
				ReadCanUse = CutHeadAndBody(strLine, dataTmp);
				//if (ReadCanUse) {
				//	FileString.push_back(dataTmp);
				//	BeTaken.push_back(false);
				//}
				//strLine = removeSpaces(strLine);
				//if (strLine.size() > 0 && strLine.at(0) != '/') {
				//	FileString.push_back(strLine);
				//	BeTaken.push_back(false);
				//}		
			}
		}
		else {
			//FileString.push_back("  ");
		}
	}
	void FileClose() {
		FilePtr.close();
		//FileString.clear();
		FileStringMap.clear();
	}
	void FileCloseOnly() {
		FilePtr.close();
	}
	void DataClear() {
		//FileString.clear();
		FileStringMap.clear();
	}


	bool FindPara_Int(std::string Tag, int* res) {
		std::string Output = FindPara(Tag);
		if (Output == "ERROR") {
			return false;
		}
		else {
			int restmp = 0;
			try {
				restmp = std::stoi(Output);
			}
			catch (std::exception const& e)
			{
				std::cout << "error : " << e.what() << std::endl;
				return false;
			}
			*res = restmp;
			return true;
		}
	}
	bool FindPara_Double(std::string Tag, double* res) {
		std::string Output = FindPara(Tag);
		if (Output == "ERROR") {
			return false;
		}
		else {
			double restmp = 0;
			try {
				restmp = std::stod(Output);
			}
			catch (std::exception const& e)
			{
				std::cout << "error : " << e.what() << std::endl;
				return false;
			}
			*res = restmp;
			return true;
		}
	}
	bool FindPara_Int(std::string Tag, std::string Cut, int* res) {
		return FindPara_Int(Tag + Cut, res);
	}
	bool FindPara_Double(std::string Tag, std::string Cut, double* res) {
		return FindPara_Double(Tag + Cut, res);
	}
	std::string FindPara(std::string Tag, std::string Cut) {
		return FindPara(Tag + Cut);
	}
	std::string FindPara(std::string Tag) {
		int tmpDir = -1;
		std::string tmp_string  = "ERROR";

		auto res = FileStringMap.find(Tag);
		if (res != FileStringMap.end()){
			tmp_string = FileStringMap[Tag];
		}

		return tmp_string;
	}

	std::string FileLineReturn(int line_dir) {
		std::string strTmp;
		if (line_dir < FileString.size()) {
			strTmp = FileString.at(line_dir);
		}
		else {
			strTmp = "ERROR";
		}
		return strTmp;
	}

	int FileLineMax() {
		return FileString.size();
	}



	bool FileExist(std::string ImgFileName) {
		std::fstream TXT_Ptr_read;
		TXT_Ptr_read.open(ImgFileName.c_str());
		if (!TXT_Ptr_read.is_open())
		{
			return false;
		}
		TXT_Ptr_read.close();
		return true;
	}
	void FileRemove(std::string ImgFileName) {
		if (remove(ImgFileName.c_str()) != 0)
			printf("Error deleting file\n");
		else
			printf("File [ %s ] deleted\n", ImgFileName.c_str());
	}
	void MakeFile(std::string ImgFileName) {
		std::ofstream TXT_tmp;
		TXT_tmp.open(ImgFileName.c_str());
		TXT_tmp << " " << std::endl;
		TXT_tmp.close();
	}
	void MakeFile(std::string ImgFileName, int count) {
		MakeFile(ImgFileName, count, 30);
	}
	void MakeFile(std::string ImgFileName, int count, int fail_count) {
		std::ofstream TXT_tmp;
		TXT_tmp.open(ImgFileName.c_str());
		TXT_tmp << "Detect_Times:" << count << std::endl;
		TXT_tmp << "Fail_Detect_Times:" << fail_count << std::endl;
		TXT_tmp.close();
	}
private:
	bool CutHeadAndBody(std::string Input, std::string& output) {
		bool res = false;
		int tmp = Input.find(CmdCutMarker);
		std::string Tagtmp, Bodytmp;

		if ((Input.size() > 0) && (Input.at(0) != '/') && (tmp != -1)) {
			tmp += 1;
			Tagtmp = Input.substr(0, tmp);
			Tagtmp = removeSpaces(Tagtmp);
			Bodytmp = Input.substr(tmp);
			//Bodytmp = removeSpaces(Bodytmp);
			FileStringMap[Tagtmp] = Bodytmp;

			if (Bodytmp.size() > 0) {
				output = Tagtmp + Bodytmp;
				res = true;
			}
		}
		return res;

	}
	std::string removeSpaces(const std::string s) {
		std::string tmp = s;
		tmp.erase(std::remove_if(tmp.begin(), tmp.end(), isspace), tmp.end());

		//tmp.erase(std::remove(tmp.begin(), tmp.end(), ' '), tmp.end());
		return tmp;
	}
public:
private:
	std::fstream FilePtr;
	std::vector<std::string> FileString;
	std::vector<bool> BeTaken;
	std::string strLine;
	std::map<std::string, std::string> FileStringMap;
	int AllLine = 0;
	char CmdCutMarker = ':';
};

class ParameterStRead :public FileRead {


public:
	struct DataStruct
	{
		int FncType;
		std::vector <std::string> DataString;
		std::vector<int> DataInt;
		std::vector<double> DataDouble;
		std::vector<bool> DataBool;
	};
	ParameterStRead(int bool_size, int int_size, int double_size, int string_size) {
		InitData;
		InitData.DataBool.clear();
		InitData.DataInt.clear();
		InitData.DataDouble.clear();
		InitData.DataString.clear();
		InitData.DataBool.assign(bool_size, false);
		InitData.DataInt.assign(int_size, 0);
		InitData.DataDouble.assign(double_size, 0.0);
		InitData.DataString.assign(string_size, "");

		TempData = InitData;


	}
	~ParameterStRead() {

	}


public:
	void ParaFileRead(std::string File_Name) {
		DataList.clear();
		FileLoadInit(File_Name);


		for (int i = 0; i < 100; i++) {
			if (!CheckListExist_and_Get(i)) {
				break;
			}
			TempData = InitData;

			ParaFileRead_Int(i);
			ParaFileRead_Double(i);
			ParaFileRead_Bool(i);
			ParaFileRead_String(i);
			DataList.push_back(TempData);
			printf("====Data Ptr [%d]====\n",i);
			ShowDataInside(TempData);
			printf("=====================\n");
		}

	}

private:

	int StringToInt(std::string input) {
		int restmp = 0;
		if (input == "") {
			return restmp;
		}
		else {
			try {
				restmp = std::stoi(input);
			}
			catch (std::exception const& e)
			{
				std::cout << "error : " << e.what() << std::endl;
				restmp = 0;
			}
			return restmp;
		}
	}
	double StringToDouble(std::string input) {
		double restmp = 0;
		if (input == "") {
			return 0;
		}
		else {
			try {
				restmp = std::stod(input);
			}
			catch (std::exception const& e)
			{
				std::cout << "error : " << e.what() << std::endl;
				restmp = 0;
			}
			return restmp;
		}
	}
	bool StringToBool(std::string input) {
		int restmp = false;

		if (input.find("true") != -1) {
			restmp = true;
		}
		else if (input.find("false") != -1) {
			restmp = false;
		}
		else {
			restmp = false;
		}
		return restmp;
	}

	bool CheckListExist_and_Get(int LisPtr) {
		std::string Tag = "Data," + std::to_string(LisPtr);
		std::string GetString = FindPara(Tag);

		if (GetString == "ERROR") {
			return false;
		}
		return true;

	}
	void GetBracketsPos(int* Pos, std::string brackets, int count) {
		int startPos = 0;
		int nowPos = 0;
		bool StillWork = true;

		for (int i = 0; i < count; i++) {
			if (StillWork) {
				nowPos = brackets.find(',', startPos);
				if (nowPos != -1) {
					Pos[i] = nowPos;
					startPos = nowPos + 1;
					//if (startPos >= brackets.size()) {
					//	StillWork = false;
					//}
				}
				else {
					Pos[i] = brackets.size();
					StillWork = false;
				}
			}
			else {
				Pos[i] = brackets.size();
			}
		}


	}
	std::string GetBracketsValue(int* data, std::string brackets, int dir) {
		std::string tmp;
		int whereIcut = data[dir] + 1;
		int wherecutend = data[dir + 1];
		int cutsize = wherecutend - data[dir] - 1;

		if (wherecutend < 0) {
			cutsize = brackets.size() - whereIcut;
		}
		//printf("  whereIcut [%d] ,cutsize [%d] , size [%d]\n", whereIcut, cutsize, (int)brackets.size());
		if ((whereIcut >= brackets.size())) {
			//printf("  whereIcut >= brackets.size\n" );
			return "";
		}
		tmp = tmp.assign(brackets, whereIcut, cutsize);
		return tmp;
	}

	void ShowDataInside(DataStruct data) {
		int size = data.DataBool.size();
		printf("DataBool size [%d]\n", size);
		for (int i = 0; i < size; i++) {
			printf("[%d]", (int)data.DataBool[i] );
		}
		printf("\n");

		size = data.DataInt.size();
		printf("DataInt size [%d]\n", size);
		for (int i = 0; i < size; i++) {
			printf("[%d]", data.DataInt[i]);
		}
		printf("\n");

		size = data.DataDouble.size();
		printf("DataDouble size [%d]\n", size);
		for (int i = 0; i < size; i++) {
			printf("[%f]", data.DataDouble[i]);
		}
		printf("\n");

		size = data.DataString.size();
		printf("DataString size [%d]\n", size);
		for (int i = 0; i < size; i++) {
			printf("[%s]", data.DataString[i].c_str());
		}
		printf("\n");
	}

	void ParaFileRead_Int(int Dir) {
		int tmp;
		std::string Tag = "Data," + std::to_string(Dir) + ",int,";
		std::string Output = FindPara(Tag);
		if (Output == "ERROR") {
			for (int i = 0; i < InitData.DataInt.size(); i++) {
				TempData.DataInt[i] = 0;
			}
		}
		else {
			//printf("ParaFileRead_Int_GetData[%d]\n", Dir);
			ParaFileRead_Int_GetData(Output, TempData.DataInt.begin());
		}
	}
	void ParaFileRead_Int_GetData(std::string Input, std::vector<int>::iterator data) {

		PosSave.clear();
		PosSave.assign(InitData.DataInt.size() + 1, -1);

		std::string CutTmpString;
		GetBracketsPos(&PosSave[1], Input, InitData.DataInt.size());

		for (int i = 0; i < InitData.DataInt.size(); i++) {
			CutTmpString = GetBracketsValue(&PosSave[0], Input, i);
			data[i] = StringToInt(CutTmpString);
		}
	}

	void ParaFileRead_Double(int Dir) {
		int tmp;
		std::string Tag = "Data," + std::to_string(Dir) + ",double,";
		std::string Output = FindPara(Tag);
		if (Output == "ERROR") {
			for (int i = 0; i < InitData.DataDouble.size(); i++) {
				TempData.DataDouble[i] = 0;
			}
		}
		else {
			//printf("ParaFileRead_Double_GetData[%d]\n", Dir);
			ParaFileRead_Double_GetData(Output, TempData.DataDouble.begin());
		}
	}
	void ParaFileRead_Double_GetData(std::string Input, std::vector<double>::iterator data) {

		PosSave.clear();
		PosSave.assign(InitData.DataDouble.size() + 1, -1);

		std::string CutTmpString;
		GetBracketsPos(&PosSave[1], Input, InitData.DataDouble.size());

		for (int i = 0; i < InitData.DataDouble.size(); i++) {
			CutTmpString = GetBracketsValue(&PosSave[0], Input, i);
			data[i] = StringToDouble(CutTmpString);
			//printf("[%.2f]\n", data[i]);
		}
	}

	void ParaFileRead_Bool(int Dir) {
		int tmp;
		std::string Tag = "Data," + std::to_string(Dir) + ",bool,";
		std::string Output = FindPara(Tag);
		if (Output == "ERROR") {
			for (int i = 0; i < InitData.DataBool.size(); i++) {
				TempData.DataBool[i] = false;
			}
		}
		else {
			std::vector<bool>::iterator it = TempData.DataBool.begin();
			ParaFileRead_Bool_GetData(Output, it);
		}
	}
	void ParaFileRead_Bool_GetData(std::string Input, std::vector<bool>::iterator data) {
		PosSave.clear();
		PosSave.assign(InitData.DataBool.size() + 1, -1);

		std::string CutTmpString;
		GetBracketsPos(&PosSave[1], Input, InitData.DataBool.size());

		for (int i = 0; i < InitData.DataBool.size(); i++) {
			CutTmpString = GetBracketsValue(&PosSave[0], Input, i);
			data[i] = StringToBool(CutTmpString);
			//printf("[%.2f]\n", data[i]);
		}
	}

	void ParaFileRead_String(int Dir) {
		int tmp;
		std::string Tag = "Data," + std::to_string(Dir) + ",string,";
		std::string Output = FindPara(Tag);
		if (Output == "ERROR") {
			for (int i = 0; i < InitData.DataBool.size(); i++) {
				TempData.DataBool[i] = false;
			}
		}
		else {
			std::vector<std::string>::iterator it = TempData.DataString.begin();
			ParaFileRead_String_GetData(Output, it);
		}
	}
	void ParaFileRead_String_GetData(std::string Input, std::vector<std::string>::iterator data) {
		PosSave.clear();
		PosSave.assign(InitData.DataString.size() + 1, -1);

		std::string CutTmpString;
		GetBracketsPos(&PosSave[1], Input, InitData.DataString.size());

		for (int i = 0; i < InitData.DataString.size(); i++) {
			CutTmpString = GetBracketsValue(&PosSave[0], Input, i);
			//printf("%s\n", CutTmpString.c_str());
			data[i] = CutTmpString;
		}
	}



public:
	std::vector<DataStruct> DataList;
	//FileRead FileName;
	DataStruct TempData;
	DataStruct InitData;
	std::fstream FilePtr;
	std::vector<std::string> FileString;
	std::string strLine;
	std::vector<int> PosSave;

private:

};


#endif