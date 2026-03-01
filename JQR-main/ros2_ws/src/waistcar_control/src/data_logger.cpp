#include "data_logger.h"

void DataLogger::init(const std::string &file_name_prefix, const bool &enable_time_suffix,
                const std::string *header, const int &cols)
{
    log_filename = ".txt";
    if (enable_time_suffix)
    {
        char time_str[16];
        getCurrentTimeString(time_str,16);
        log_filename.insert(0,time_str);
        log_filename.insert(0,"_");
    }
    log_filename.insert(0,file_name_prefix);
    fout.open(log_filename);
    std::cout<< "DataLogger:  " << log_filename << std::endl;

    table_cols = cols;

    for (int i = 0; i < table_cols; i++)
    {
        fout << *(header + i);
        if (i == table_cols - 1)
        {
            fout << std::endl;
        }
        else
        {
            fout << ",";
        }
    }
    count = 0;
}

void DataLogger::close()
{
    fout.close();
}

void DataLogger::getCurrentTimeString(char *time_str, int str_len) {
  time_t timep;
  time(&timep);
  strftime(time_str, str_len, "%Y%m%d-%H%M%S", localtime(&timep));
}