#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>
#include <cstdlib>
#include <ctime>
#include <fcntl.h>
#include <sys/stat.h>

using namespace std;

class DataLog
{
public:
    DataLog(string strDir = "./.log");
    ~DataLog();

    void addLog(string log);	//添加日志记录到日志文件
    string getLogFileName();	//获取日志文件名称
    void setFileName(string); //设置日志文件名
    void setLogDir(string strDir);		//设置日志文件目录
    bool checkFolderExist( const string  & strPath);
private:
    void fileOffset();		//文件名称进行偏移
    string getCurrentTime();


private:
    string m_LogFileName;	//文件名
    fstream *m_outputFile;	//输出文件流
    string m_strDir;		//目录


};

DataLog::DataLog(string strDir)
{
    m_strDir = strDir;
    m_LogFileName = m_strDir+string("/")+getCurrentTime();
    
    if(!checkFolderExist(strDir))
    {
        mkdir(strDir.c_str(),S_IRWXU);
    }

    m_outputFile = new fstream;
    string strname = m_LogFileName+".csv";
    m_outputFile->open(strname,ofstream::out|ofstream::app);	//打开日志文件
    bool b=m_outputFile->is_open();


}
DataLog::~DataLog()
{
    if(m_outputFile)
        delete m_outputFile;
}

//********************************
//函数名：DataLog::addLog
//描  述：向文件中添加日志信息
//参  数 log 为全部信息内容，包括换行
//返回值：void
//*************************************
void DataLog::addLog(string log)
{
    string currentTime = getCurrentTime(); //获取本地时间

    *m_outputFile<<log;

}
//********************************
//函数名：DataLog::checkFolderExist
//描  述：测试目录是否存在
//参  数：strPath 目录名
//返回值：存在返回真
//*************************************
bool DataLog::checkFolderExist( const string  & strPath)
{
    struct stat buffer;   
    return (stat(strPath.c_str(), &buffer) == 0);
}

//获取文件名
string DataLog::getLogFileName()
{
    return m_LogFileName+".csv";
}

//设置文件名
void DataLog::setFileName(string filename)
{
    m_LogFileName = m_strDir+string("/")+filename;
}

//********************************
//函数名：DataLog::getCurrentTime
//描  述：获取本地时间
//返回值：时间字符串
//*************************************
string DataLog::getCurrentTime()
{
    //获取系统时间
    time_t seconds = time(nullptr);
    struct tm *p;
    p = localtime(&seconds);//获取本地时间
    char strTime[100] = {0};
    sprintf(strTime,"%d-%d-%d-%d-%d-%d",1900+p->tm_year,1+p->tm_mon,p->tm_mday,p->tm_hour,p->tm_min,p->tm_sec);
    return string(strTime);
}