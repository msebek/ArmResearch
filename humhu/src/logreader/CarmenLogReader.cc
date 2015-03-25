#include <boost/lexical_cast.hpp>

template <class T>
CarmenLogReader<T>::CarmenLogReader(std::string filename) :
    logFile(filename) {
    if(!logFile.is_open()) {
        std::cout << "CarmenLogReader: Failed to open log " << filename << std::endl;
    }
    
}

template <class T>
CarmenLogReader<T>::~CarmenLogReader() {}

template <class T>
T CarmenLogReader<T>::GetNextLine(std::string tag) {

    T logLine;
    std::string line = ReadNextLine(tag);
    
    if(line.empty()) { return logLine; }

    TokenizerSeparator sep(" ");
    Tokenizer tokens(line, sep);
    TokenizerIterator iter = tokens.begin();

    ParseLineStart(iter, logLine);
    ParseLineMiddle(iter, logLine);
    ParseLineEnd(iter, logLine);
    logLine.valid = true;
    
    return logLine;
}

template <class T>
std::string CarmenLogReader<T>::ReadNextLine(std::string tag) {

    WriteLock lock(mutex);
    
    std::string line;
    while(!logFile.eof()) {
    
        std::getline(logFile, line);

        if(line.size() == 0) {
            continue;
        }
        
        TokenizerSeparator sep(" ");
        Tokenizer tokens(line, sep);
        TokenizerIterator iter = tokens.begin();
        if(tag.compare(*iter) == 0) { break; }
    }

    return line;
    
}

template <class T>
void CarmenLogReader<T>::ParseLineStart(TokenizerIterator &iter, CarmenLogLine &logLine) {
    
    logLine.message_name = *iter;
    iter++;
    
}

template <class T>
void CarmenLogReader<T>::ParseLineEnd(TokenizerIterator &iter, CarmenLogLine &logLine) {

    logLine.ipc_timestamp = boost::lexical_cast<double>(*iter);
    iter++;
    logLine.ipc_hostname = *iter;
    iter++;
    logLine.logger_timestamp = boost::lexical_cast<double>(*iter);
    iter++;
    
}