#include "IIMUPayload.h"
#include "ISensorDataTransform.h"

using namespace mems;

IIMUPayload::IIMUPayload()
{
    setTransform(NULL);
}

IIMUPayload::IIMUPayload(ISensorDataTransform * transform)
{
    setTransform(transform);
}

IIMUPayload::~IIMUPayload()
{
    m_sdt = NULL;
}

void IIMUPayload::setTransform(mems::ISensorDataTransform * transform)
{
    m_sdt = transform;
}

mems::ISensorDataTransform * IIMUPayload::getTransform()
{
    return(m_sdt);
}
