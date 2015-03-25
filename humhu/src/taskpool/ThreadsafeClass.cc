#include "ThreadsafeClass.h"

ThreadsafeClass::ThreadsafeClass() {};

ThreadsafeClass::~ThreadsafeClass() {};

void ThreadsafeClass::AssertExternalLock(ReadLock &lock) const {
    if(!lock.owns_lock() || lock.mutex() != &mutex) {
        throw "Lock error: Wrong lock given!";
    }
}

void ThreadsafeClass::AssertExternalLock(WriteLock &lock) const {
    if(!lock.owns_lock() || lock.mutex() != &mutex) {
        throw "Lock error: Wrong lock given!";
    }
}