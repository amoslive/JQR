#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

class DataLogger{
public:
    void init(const std::string &file_name_prefix, const bool &enable_time_suffix,
                const std::string *header, const int &cols);
    void close();
    static void getCurrentTimeString(char *time_str, int str_len);

    DataLogger & operator<< (const double &data){
        ++count;
        fout << data;
        return *this;
    }

    DataLogger & operator, (const double &data){
        ++count;
        fout << ", " << data;
        return *this;
    }

    DataLogger & operator<< (std::ostream& (*op) (std::ostream&)) {
        (void)op;
        if (count != table_cols) {
        throw std::string("DataLogger:: The cols of data should equal to the cols of header.");
        }
        count = 0;
        fout << std::endl;
        return *this;
    }


private:
    std::string log_filename;
    std::ofstream fout;
    int table_cols;
    int count;

};