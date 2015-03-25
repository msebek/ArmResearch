#ifndef _LOG_READER_H_
#define _LOG_READER_H_

#include <boost/tokenizer.hpp>

#include "ThreadsafeClass.h"

// TODO Complete?

class LogReader : private ThreadsafeClass {
public:

    LogReader(std::string filename);
    ~LogReader();

    virtual 
    
protected:

    typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;
    typedef boost::char_separator<char> TokenizerSeparator;
    typedef Tokenizer::iterator TokenizerIterator;
    
    /*! \brief The log file interface. */
    std::ifstream logFile;

    
    
};

#endif