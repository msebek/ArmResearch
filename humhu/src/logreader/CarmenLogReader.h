#ifndef _CARMENREADER_H_
#define _CARMENREADER_H_

#include <pcl/point_cloud.h>
#include <iostream>
#include <string>
#include <fstream>
#include <boost/tokenizer.hpp>

#include "ThreadsafeClass.h"

// Default log line container. Should be extended.
struct CarmenLogLine {
    bool valid;
    std::string message_name;
    double ipc_timestamp;
    std::string ipc_hostname;
    double logger_timestamp;

    CarmenLogLine() : valid(false) {}
    
};

// Default log reader. Should be extended.
template <class T>
class CarmenLogReader : private ThreadsafeClass {
public:

    CarmenLogReader(std::string filename);
    ~CarmenLogReader();

    virtual T GetNextLine(std::string tag);
    
protected:

    typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;
    typedef boost::char_separator<char> TokenizerSeparator;
    typedef Tokenizer::iterator TokenizerIterator;

    /*! \brief The log file interface. */
    std::ifstream logFile;

    /*! \brief Return the next line starting with the string tag */
    std::string ReadNextLine(std::string tag);
    
    /*! \brief Parse the common beginning fields from a line. */
    void ParseLineStart(TokenizerIterator &iter, CarmenLogLine &logLine);

    /*! \brief Parse the content fields from a line. */
    virtual void ParseLineMiddle(TokenizerIterator &iter, T &logline) = 0;
    
    /*! \brief Parse the common ending fields from a line. */
    void ParseLineEnd(TokenizerIterator &iter, CarmenLogLine &logLine);
    
};

#include "CarmenLogReader.cc"

#endif 