#ifndef CFAULTALG_H
#define CFAULTALG_H
#include <FailureReporter.h>
#include <boost/shared_ptr.hpp>
#include <iostream>
class CFaultAlg
{
public:
    CFaultAlg();
    ~CFaultAlg();
private:
    FailureReporter *m_pFailureSdk;
};
typedef  boost::shared_ptr<CFaultAlg> CFaultAlgPtr;

#endif // CFAULTALG_H
